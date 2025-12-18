// The Controller module is responsible for determining the control signals for the processor that are used to run the datapath.
//It contains the instruction decoder and sign extender, the main decoder, the ALU decoder, and the main finite state machine (FSM).
//Each class of RV32I instructions has a different configuration of instruction fields, so the Instruction decoder takes the instruction
//and separates it into its constituent fields (funct7, funct3, Immediate, rs1, rs2, rd, and opcode). The Instruction decoder then also
//functions as a sign extender and extends the immediate based on the instruction type. The Main decoder takes the opcode from the Instruction 
//decoder and generates the primary control signals (RegWrite, MemWrite, ALUSrc, ResultSrc, Branch, Jump, and ALUOp) that determine how data
//flows through the processor and what operations are performed. The ALU decoder then uses the ALUOp signal along with funct3 and funct7 fields
//to generate the specific 4-bit ALUControl signal that tells the ALU which operation to perform (ADD, SUB, AND, OR, XOR, shifts, etc.). 
//The heart of the controller file is in the main FSM where the processor executes different states and sequences through the fetch-decode-execute
//cycle, managing when instructions are fetched from memory, when they are decoded, when the ALU performs computations, when memory is accessed for
//loads & stores, and when results are written back to registers.

module Controller(
    input logic clk,
    //Instruction relevant variables
    input logic [31:0] instr,
    output logic [31:0] ImmExt, //extended immediate
    output logic [6:0] opcode,
    input logic reset,

    //Main Decoder Relevant Variables
    output logic [1:0] ResultSrc,
    output logic MemWrite,
    output logic ALUsrc,
    output logic RegWrite,
    output logic Branch,
    output logic Jump,

    //ALU Decoder Relevant Variables
    output logic [3:0] ALUControl,

    //Instructions Relevant to FSM
    output logic AdrSrc,
    output logic PCWrite,
    output logic IRWrite,

    output logic [1:0] SrcA,
    output logic [1:0] SrcB,
    output logic [1:0] ResultSrc_fsm,
    output logic RegWrite_fsm,
    output logic MemWrite_fsm,
    output logic Branch_fsm
);

    //Instruction relevant Variables
    logic [6:0] funct7;
    logic [2:0] funct3;
    logic [31:0] Imm; //immediate from instruction
    logic [4:0] rs1;
    logic [4:0] rs2;
    logic [4:0] rd;

    //Main Decoder Relevant Variables
    logic [2:0] ImmSrc; //control signal for sign extender/Instruction Decoder

    //ALU Decoder Relevant Variables
    logic [1:0] ALUOp;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // BEGINNING OF INSTRUCTION DECODER + SIGN EXTENDOR                 BEGINNING OF INSTRUCTION DECODER + SIGN EXTENDOR
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    initial
        begin //maybe add some sort of trigger ontop of clk for when there is a new instruction @(posedge clk) 
        ImmExt = 32'd0;
        Imm = 32'd0;
        funct7 = 7'd0;
        funct3 = 3'd0;
        rs1 = 5'd0;
        rs2 = 5'd0;
        rd = 5'd0;
        //opcode = instr[6:0];
    end

    assign opcode = instr[6:0];

    always_comb
        begin
            //Default assignments to prevent latches but code doesn't like it if I remove this block even though I already state default case idk
            rs1 = 5'd0;
            rs2 = 5'd0;
            rd = 5'd0;
            funct7 = 7'd0;
            funct3 = 3'd0;
            Imm = 32'd0;
            ImmExt = 32'd0;

            case (opcode)
                7'b0110011: begin //R-type
                    rs1 = instr[19:15];
                    rs2 = instr[24:20];
                    rd = instr[11:7];
                    funct7 = instr[31:25];
                    funct3 = instr[14:12];
                    ImmExt = 32'd0; //no ImmExt R-type

                end

                7'b0000011: begin //I-type (Loads)
                    rs1 = instr[19:15];
                    rd = instr[11:7];
                    funct3 = instr[14:12];

                    Imm = instr[31:20];
                    ImmExt = {{20{Imm[11]}}, Imm[11:0]}; //takes the MSB of Imm and makes it the MSBs of ImmExt
                end

                7'b0010011: begin //I-type (ALU) 
                    rs1 = instr[19:15];
                    rd = instr[11:7];
                    funct3 = instr[14:12];

                    case (funct3)
                        3'b001: begin //SLLI
                            funct7 = instr[31:25];
                            ImmExt = {27'd0, instr[24:20]};
                        end
                        
                        3'b101: begin //SRLI or SRAI 
                            funct7 = instr[31:25]; 
                            ImmExt = {27'd0, instr[24:20]};
                        end
                        
                        default: begin //ADDI, SLTI, SLTIU, XORI, ORI, ANDI
                            Imm = instr[31:20];
                            ImmExt = {{20{instr[31]}}, instr[31:20]}; // Sign-extend all 12 bits
                        end
                    endcase
                end

                7'b1100111: begin //I-type (JALR)
                    rs1 = instr[19:15];
                    rd = instr[11:7];
                    funct3 = instr[14:12];

                    Imm = instr[31:20];
                    ImmExt = {{20{Imm[11]}}, Imm[11:0]};
                end

                7'b0100011: begin //S-type (Stores)
                    rs1 = instr[19:15];
                    rs2 = instr[24:20];
                    funct3 = instr[14:12];
                
                    Imm[11:5] = instr[31:25];
                    Imm[4:0] = instr[11:7];
                    ImmExt = {{20{Imm[11]}}, Imm[11:0]};
                end

                7'b1100011: begin //B-type (Branches)
                    rs1 = instr[19:15];
                    rs2 = instr[24:20];
                    funct3 = instr[14:12];

                    Imm[12] = instr[31];
                    Imm[10:5] = instr[30:25];
                    Imm[4:1] = instr[11:8];
                    Imm[11] = instr[7];
                    ImmExt = {{19{Imm[12]}}, Imm[12:1], 1'b0};
                end

                7'b0110111: begin //U-type (LUI)
                    rd = instr[11:7];

                    Imm = instr[31:12];
                    ImmExt = {Imm[19:0], 12'b0}; //ImmExt = {Imm[31:12], 12'b0} no
                end

                7'b0010111: begin //U-type (AUIPC)
                    rd = instr[11:7];

                    Imm = instr[31:12];
                    ImmExt = {Imm[19:0], 12'b0}; //ImmExt = {Imm[31:12], 12'b0} no
                end

                7'b1101111: begin //J-type (JAL)
                    rd = instr[11:7];

                    Imm[20] = instr[31];
                    Imm[10:1] = instr[30:21];
                    Imm[11] = instr[20];
                    Imm[19:12] = instr[19:12];
                    ImmExt = {{11{Imm[20]}}, Imm[20:1], 1'b0};
                end

                default: begin
                    ImmExt = 32'd0;
                    Imm = 32'd0;
                    funct7 = 7'd0;
                    funct3 = 3'd0;
                    rs1 = 5'd0;
                    rs2 = 5'd0;
                    rd = 5'd0;
                end
            endcase
        end

    //////////////////////////////////////////////////////////////////////////
    // BEGINNING OF MAIN DECODER          BEGINNING OF MAIN DECODER
    //////////////////////////////////////////////////////////////////////////

    //should be a giant opcode case statement that does different things based on the opcode
    //is removing this inital block a good idea? prob not
    initial
        begin
            //set everything to 0 by default
            RegWrite = 1'b0;
            MemWrite = 1'b0;
            ALUsrc = 1'b0;
            ResultSrc = 2'b00;
            Branch = 1'b0;
            Jump = 1'b0;
            ALUOp = 2'b00;
        end
    
    always_comb
        begin
            //Default Values here aswell
            RegWrite = 1'b0;
            MemWrite = 1'b0;
            ALUsrc = 1'b0;
            ResultSrc = 2'b00;
            Branch = 1'b0;
            Jump = 1'b0;
            ALUOp = 2'b00;

        case(opcode)
            7'b0110011: begin //R-type Instructions(ADD, SUB, AND, OR, SLL, SLT, SLTU, XOR, SRL, SRA)
                RegWrite = 1'b1;
                ALUsrc = 1'b0; //rs2
                ResultSrc = 2'b00; //Write ALU result
                ALUOp = 2'b10; //R-type ALU operation
            end
            
            7'b0010011: begin //I-type ALU Instructions (ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI)
                RegWrite = 1'b1;
                ALUsrc = 1'b1; //immediate
                ResultSrc = 2'b00; //Write ALU result
                ALUOp = 2'b10;//I-type ALU operation
            end
            
            7'b0000011: begin //I-type Load Instructions(LW, LH, LB, LBU, LHU)
                RegWrite = 1'b1;
                ALUsrc = 1'b1;//immediate
                ResultSrc = 2'b01;//Write memory data
                ALUOp = 2'b00; //Add for address calculation
            end
            
            7'b0100011: begin //S-type Instructions (Store) (SW, SH, SB)
                MemWrite = 1'b1;
                ALUsrc = 1'b1; //immediate
                ALUOp = 2'b00; //Add for address calculation
            end
            
            7'b1100011: begin //B-type Instructions (Branch) (BEQ, BNE, BLT, BGE, BLTU, BGEU)
                Branch = 1'b1;
                ALUsrc = 1'b0; //Compare rs1 and rs2
                ALUOp = 2'b01; //Branch comparison
            end
            
            7'b1101111: begin //JAL
                RegWrite = 1'b1;
                Jump = 1'b1;
                ResultSrc = 2'b10; // Write PC+4
                ALUOp = 2'b00;
            end
            
            7'b1100111: begin //JALR
                RegWrite = 1'b1;
                Jump = 1'b1;
                ALUsrc = 1'b1; //Use immediate
                ResultSrc = 2'b10; //Write PC+4
                ALUOp = 2'b00; //Add for target address
            end
            
            7'b0110111: begin //LUI
                RegWrite = 1'b1;
                ALUsrc = 1'b1;
                ResultSrc = 2'b11; //Write immediate directly
            end
            
            7'b0010111: begin //AUIPC
                RegWrite = 1'b1;
                ALUsrc = 1'b1;
                ResultSrc = 2'b00; // Write ALU result
                ALUOp = 2'b00; // Add
            end
            
            default: begin
                //For some reason the default needs to be set at the order to not get latch error
            end
        endcase
    end

    //////////////////////////////////////////////////////////////////////////
    // BEGINNING OF ALU DECODER      BEGINNING OF ALU DECODER
    //////////////////////////////////////////////////////////////////////////

    // ALU Decoder outputs ALUControl based on ALUOp, funct3, and sometimes funct7
    always_comb 
        begin
        //default value to prevent latch
        ALUControl = 4'b0000;

        case(ALUOp)
            2'b00: begin 
                ALUControl = 4'b0000; // ADD 
            end
            
            2'b01: begin // Branch operations, these cauesd me 5 hours of headache!!
                case(funct3)
                    3'b000: begin
                        ALUControl = 4'b0001; //BEQ (subtract)
                    end

                    3'b001: begin 
                        ALUControl = 4'b0001; //BNE (subtract)
                    end

                    3'b100: begin
                        ALUControl = 4'b1000; //BLT (set less than)
                    end

                    3'b101: begin 
                        ALUControl = 4'b1000; //BGE (set less than)
                    end

                    3'b110: begin 
                        ALUControl = 4'b1001; //BLTU (set less than unsigned)
                    end

                    3'b111: begin ALUControl = 4'b1001; //BGEU (set less than unsigned)
                    end 
                
                    default: begin
                        ALUControl = 4'b0000;
                    end

                endcase
            end
            
            2'b10: begin //R-type and I-type ALU operations
                case(funct3)
                    3'b000: begin

                        if (opcode == 7'b0110011 && funct7[5]) 
                            ALUControl = 4'b0001; //SUB
                        else if (funct7 == 7'b0000001)
                            ALUControl = 4'b1010; //MUL
                        else
                            ALUControl = 4'b0000; // ADD or ADDI
                    end

                    3'b001: begin
                        if (funct7 == 7'b0000000)
                            ALUControl = 4'b0101; //SLL
                        else
                            ALUControl = 4'b1011; //MULH
                    end

                    3'b010: begin
                        if (funct7 == 7'b0000000) 
                            ALUControl = 4'b1000; //SLT
                        else
                            ALUControl = 4'b1100; //MULHSU
                    end

                    3'b011: begin
                        if (funct7 == 7'b0000000)
                            ALUControl = 4'b1001; //SLTU
                        else
                            ALUControl = 4'b1101; //MULHU
                    end

                    3'b100: begin
                        if (funct7 == 7'b000000)
                            ALUControl = 4'b0100; //XOR
                        else 
                            ALUControl = 4'b1110; //DIV
                    end 

                    3'b101: begin
                        if (funct7[5]) //SRA
                            ALUControl = 4'b0111;
                        if (funct7 == 7'b0000001)
                            ALUControl = 4'b1111; //DIVU
                        else //SRL
                            ALUControl = 4'b0110;
                    end

                    3'b110: begin
                        ALUControl = 4'b0011; //OR
                    end

                    3'b111: begin 
                        ALUControl = 4'b0010; //AND
                    end

                    default: begin 
                        ALUControl = 4'b0000;
                    end

                endcase
            end
            
            default: begin 
                ALUControl = 4'b0000;
            end
        endcase
    end

    //////////////////////////////////////////////////////////////////////////
    // BEGINNING OF MAIN FSM          BEGINNING OF MAIN FSM
    //////////////////////////////////////////////////////////////////////////

    // Encode States
    typedef enum logic [3:0] {
        Fetch = 4'b0000, //S0
        Decode = 4'b0001, //S1
        MemAdr = 4'b0010, //S2
        MemRead = 4'b0011, //S3
        MemWB = 4'b0100, //S4
        MemWrite_state = 4'b0101, //S5
        ExecuteR = 4'b0110, //S6
        ALUWb = 4'b0111, //S7
        ExecuteI = 4'b1000, //S8
        jal = 4'b1001, //S9
        beq = 4'b1010 //S10
    } state_t;

    state_t current_state, next_state;

    // State register
    always_ff @(posedge clk or negedge reset) begin
        if (!reset)
            current_state <= Fetch;
        else
            current_state <= next_state;
    end

    // Next state logic
    always_comb begin
        AdrSrc = 0;
        IRWrite = 0;
        RegWrite_fsm = 0;
        SrcA = 2'b00;
        SrcB = 2'b00;
        ResultSrc_fsm = 2'b00;
        MemWrite_fsm = 0;
        PCWrite = 0;
        Branch_fsm = 0;
        ALUOp = 2'b00;
        next_state = Fetch;

        case (current_state)
            Fetch: begin
                //Next State
                next_state = Decode; 
                
                //Action
                AdrSrc = 0;
                IRWrite = 1;
                SrcA = 2'b00; //PC
                SrcB = 2'b10; //4
                ALUOp = 2'b00; //Add
                PCWrite = 1;
                ResultSrc_fsm = 2'b10; //ALUResult to PC
            end 

            Decode: begin
                //Next State
                case (opcode)
                    7'b0000011: begin 
                        next_state = MemAdr; //Load
                    end

                    7'b0100011: begin 
                        next_state = MemAdr; //Store
                    end 

                    7'b0110011: begin
                        next_state = ExecuteR; //R-type 
                    end

                    7'b0010011: begin
                        next_state = ExecuteI; //I-type ALU
                    end

                    7'b1101111: begin
                        next_state = jal; //JAL
                    end

                    7'b1100011: begin
                        next_state = beq; //Branch
                    end

                    7'b0110111: begin
                        next_state = ALUWb; //LUI - write immediate directly?
                    end

                    7'b0010111: begin
                        next_state = ExecuteI; //AUIPC - PC + immediate 
                    end

                    default: begin 
                        next_state = Fetch;
                    end 
                endcase 

                //Action
                SrcA = 2'b10; //OldPC
                SrcB = 2'b01; //ImmExt
                ALUOp = 2'b00; // Add (for branch target calculation) 
            end

            MemAdr: begin
                //Next State
                if (opcode == 7'b0000011)// Load
                    next_state = MemRead;
                else if (opcode == 7'b0100011) // Store
                    next_state = MemWrite_state;
                else
                    next_state = Fetch;
                
                
                //Action
                SrcA = 2'b01; //Register data
                SrcB = 2'b01; //ImmExt
                ALUOp = 2'b00; //Add
            end

            MemRead: begin
                next_state = MemWB;

                AdrSrc = 1; //Use ALUResult as memory address
                ResultSrc_fsm = 2'b01; //Memory data
            end 

            MemWB: begin 
                next_state = Fetch;

                ResultSrc_fsm = 2'b01; //Memory data. 01 is memory data
                RegWrite_fsm = 1; //Write to register
            end 

            MemWrite_state: begin 
                next_state = Fetch;

                AdrSrc = 1;
                MemWrite_fsm = 1; //Enable memory write
            end 

            ExecuteR: begin
                next_state = ALUWb;

                SrcA = 2'b01; //Register rs1
                SrcB = 2'b00; //Register rs2
                ALUOp = 2'b10; //R-type operation (determined by funct3/funct7)
            end 

            ExecuteI: begin 
                next_state = ALUWb;

                if (opcode == 7'b0010111) begin // AUIPC
                    SrcA = 2'b10; // OldPC is 2'b10
                    SrcB = 2'b01; // ImmExt
                    ALUOp = 2'b00; // Simple ADD
                end else begin // I-type ALU (ADDI, SLTI, etc.)
                    SrcA = 2'b01; // Register rs1
                    SrcB = 2'b01; // ImmExt
                    ALUOp = 2'b10; // I-type operation (uses funct3)
                end
            end

            ALUWb: begin
                next_state = Fetch;

                if (opcode == 7'b0110111) begin
                    ResultSrc_fsm = 2'b11; // Same as ImmExt (from Main Decoder)
                end else begin
                    ResultSrc_fsm = 2'b00; // ALUResult
                end
                RegWrite_fsm = 1; //Write to register
            end 

            jal: begin
                next_state = Fetch;

                SrcA = 2'b10; //oldPC (the PC of the JAL instruction) double check MainFSM to see if SrcA =2'bxx is right
                SrcB = 2'b10; //Constant 4
                ALUOp = 2'b00; //ADD
                ResultSrc_fsm = 2'b10; //current ALUresult (OldPC + 4)
                RegWrite_fsm = 1; // Write return address to destination register
                PCWrite = 1; //update PC to jump target (use ALUOut from Decode)
            end

            beq: begin
                next_state = Fetch;

                Branch_fsm = 1; //Enable branch logic
                SrcA = 2'b01; //Register rs1
                SrcB = 2'b00; //Register rs2
                ALUOp = 2'b01; //Subtract (for comparison)
            end

            default: begin
                next_state = Fetch;
            end
        endcase
    end

endmodule