//The ALU module contains the processor’s Arithmetic Logic Unit. The ALU is able to do functions such as ADD, SUB, AND, OR, XOR, SLL, SRL, SLT, SL, and more based 
//on the current instruction. The ALU knows which operation to perform from the ALUcontrol from the controller module. The controller module knows what to tell the
//ALU to perform due to the ALU decoder in the controller.

module ALU(
        input logic [31:0] SrcA,
        input logic [31:0] SrcB,
        input logic [3:0] ALUcontrol,
        output logic [31:0] ALUresult,
        output logic zero
    );

    logic [4:0] bits; //this will help me prevent bit slicing
    logic [63:0] temp_result; //temporary result for certain signed operations
    logic signed [63:0] temp_result_signed; //signed version of temp_result

    assign bits = SrcB[4:0];

    always_comb begin
        temp_result[63:0] = 64'd0;
        zero = 0;
        ALUresult = 32'd0;
        temp_result_signed[63:0] = 64'sd0;

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
                temp_result = SrcA * SrcB;
                ALUresult = temp_result[31:0];

            end

            4'b1011: begin //MULH
                zero = 0;
                temp_result_signed = $signed(SrcA) * $signed(SrcB);
                ALUresult = temp_result_signed[63:32];
            end

            4'b1100: begin //MULHSU
                zero = 0;
                temp_result_signed = $signed({{32{SrcA[31]}}, SrcA}) * $signed({32'b0, SrcB});
                ALUresult = temp_result_signed[63:32];
            end

            4'b1101: begin //MULHU
                zero = 0;
                temp_result = SrcA * SrcB;
                ALUresult = temp_result[63:32];
            end

            4'b1110: begin //DIV 
                zero = 0;
                ALUresult = ($signed(SrcA) / $signed(SrcB));
            end

            4'b1111: begin //DIVU
                zero = 0;
                ALUresult = (SrcA / SrcB);
            end
             
            default:begin
                zero = 0;
                ALUresult = 32'd0;
            end
       endcase

       zero = (ALUresult == 32'd0);
    end
endmodule