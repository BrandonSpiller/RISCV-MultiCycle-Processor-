//`include "top.sv"

`timescale 1ns/1ps
`include "./top.sv"

module test_bench;

    logic clk = 0;
    logic SW = 0;
    logic BOOT = 0;
    logic led, red, green, blue;

    top u0 (
        .clk    (clk),
        .SW     (SW),
        .BOOT   (BOOT),
        .led    (led),
        .red    (red),
        .green  (green),
        .blue   (blue)
    );

    // Monitor signals
    logic [31:0] prev_pc;
    logic [31:0] prev_instr;
    logic [3:0] prev_state;
    
 initial begin
    $dumpfile("test_bench.vcd");
    $dumpvars(0, test_bench);
    $dumpvars(0, u0.u3.register_array[0]);
    $dumpvars(0, u0.u3.register_array[1]);
    $dumpvars(0, u0.u3.register_array[2]);
    $dumpvars(0, u0.u3.register_array[3]);
    $dumpvars(0, u0.u3.register_array[4]);
    $dumpvars(0, u0.u3.register_array[5]);
    $dumpvars(0, u0.u3.register_array[6]);
    $dumpvars(0, u0.u3.register_array[7]);
    $dumpvars(0, u0.u3.register_array[8]);
    $dumpvars(0, u0.u3.register_array[9]);
    $dumpvars(0, u0.u3.register_array[10]);
    $dumpvars(0, u0.u3.register_array[11]);
    $dumpvars(0, u0.u3.register_array[12]);
    $dumpvars(0, u0.u3.register_array[13]);
    $dumpvars(0, u0.u3.register_array[14]);
    $dumpvars(0, u0.u3.register_array[15]);
    $dumpvars(0, u0.u3.register_array[16]);
    $dumpvars(0, u0.u3.register_array[17]);
    $dumpvars(0, u0.u3.register_array[18]);
    $dumpvars(0, u0.u3.register_array[19]);
    $dumpvars(0, u0.u3.register_array[20]);
    
    // Initialize previous values
    #100;
    
    prev_pc = u0.pc;
    prev_instr = u0.Instr_reg;
    prev_state = u0.u4.current_state;
    
    #10000;

    $finish;
end
    
    // Display state changes
    always @(posedge clk) begin
        if (u0.reset) begin
            // Every time the state changes
            if (u0.u4.current_state !== prev_state) begin
                $display("PC: 0x%08h  State: %1d  Instr: 0x%08h  rd: x%0d  rs1: x%0d  rs2: x%0d", 
                         u0.pc,
                         u0.u4.current_state,
                         u0.Instr_reg,
                         u0.Instr_reg[11:7],
                         u0.Instr_reg[19:15],
                         u0.Instr_reg[24:20]);
                prev_state = u0.u4.current_state;
            end
        end
    end
    
    // Display register writes
    always @(posedge clk) begin
        if (u0.reset && u0.WriteEnable) begin
            if (u0.Address3 != 0) begin
                $display("    >>> REG WRITE: x%0d = 0x%08h", 
                         u0.Address3,
                         u0.WriteData);
            end
        end
    end
    
    // Display memory writes
    always @(posedge clk) begin
        if (u0.reset && u0.dmem_wren) begin
            $display("    >>> MEM WRITE: Addr=0x%08h Data=0x%08h", 
                     u0.dmem_address,
                     u0.dmem_data_in);
        end
    end

    // Clock generation (12MHz = 83.33ns period)
    always begin
        #41.67 clk = ~clk;
    end

endmodule