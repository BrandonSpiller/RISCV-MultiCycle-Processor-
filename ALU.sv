module ALU(
        input logic [31:0] SrcA,
        input logic [31:0] SrcB,
        input logic [3:0] ALUcontrol,
        output logic [31:0] ALUresult,
        output logic zero
    );

    logic [4:0] bits; //this will help me prevent bit slicing
    logic [63:0] temp_result //temporary result for certain signed operations
    assign bits = SrcB[4:0];

    always_comb begin
       case(ALUcontrol)
            4'b0000:begin //ADD
                zero = 0;
                ALUresult = SrcA + SrcB; 
            end

            4'b0001:begin //SUB
                zero = 0; 
                ALUresult = SrcA - SrcB;
            end

            4'b0010:begin //AND
                zero = 0;
                ALUresult = SrcA & SrcB;
            end

            4'b0011:begin //OR
                zero = 0;
                ALUresult = SrcA | SrcB;
            end

            4'b0100:begin //XOR
                zero = 0;
                ALUresult = SrcA ^ SrcB;
            end

            4'b0101:begin //SLL
                zero = 0;
                ALUresult = SrcA << bits;
            end

            4'b0110:begin //SRL
                zero = 0;
                ALUresult = SrcA >> bits;
            end

            4'b0111:begin //Shift Right
                zero = 0;
                ALUresult = $signed(SrcA) >>> bits;
            end

            4'b1000:begin //Set Less Than
                zero = 0;
                ALUresult = ($signed(SrcA) < $signed(SrcB)) ? 32'd1 : 32'd0;
            end
            4'b1001:begin //Set Less Than Unsigned
                zero = 0;
                ALUresult = (SrcA < SrcB) ? 32'd1 : 32'd0;
            end

            4'b1010: begin //MUL
                zero = 0;
                temp_result = srcA * SrcB;
                ALUresult = temp_result[31:0];

            end

            4'b1011: begin //MULH
                zero = 0;
                temp_result = (($signed(srcA)) * ($signed(srcB)));
                ALUresult = temp_result[63:32];
            end

            4'b1100: begin //MULHSU
                zero = 0;
                temp_result = ($signed(srcA)) * srcB;
                ALUresult = temp_result[63:32];
            end

            4'b1101: begin //MULHU
                zero = 0;
                temp_result = SrcA * srcB;
                ALUresult = temp_result[63:32];
            end

            //FOR ALL DIVIDE INSTRUCTIONS DOUBLE CHECK IF BRAD IS OKAY WITH US JUST DOING /

            4'b1110: begin //DIV 
                zero = 0;
                ALUresult = ($signed(SrcA) / $signed(srcB));
            end

            4'b1111: begin //DIVU
                zero = 0;
                ALUresult = (SrcA / srcB);
            end
             
            default:begin
                zero = 0;
                ALUresult = 32'd0;
            end
       endcase

       zero = (ALUresult == 32'd0);
    end
endmodule