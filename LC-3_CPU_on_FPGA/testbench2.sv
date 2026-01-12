//`include "types.sv"

//import SLC3_TYPES::*;



module testbench2();





timeunit 10ns;

timeprecision 1ns;





logic clk, reset, run_i, continue_i;

logic [15:0] sw_i, ir, cpu_bus, pc;

logic [15:0] mdr;

logic [15:0] mdr_in;

logic ld_mdr;

logic [15:0] led_o;

logic [7:0]  hex_seg_left;

logic [3:0]  hex_grid_left;

logic [7:0]  hex_seg_right;

logic [3:0]  hex_grid_right;

logic [4:0] state;

logic [3:0] hex_in[4];

logic [15:0] R0;

logic [15:0] R1;

logic [15:0] R2;

logic [15:0] R5;

logic [15:0] R7;

logic [15:0] PC_MUX;

logic [15:0] bus;

logic [15:0] SR1out;

logic [15:0] ALU;

logic [15:0] cpu_addr;

logic cpu_mem_ena;

logic cpu_wr_ena;

logic [3:0] opcode;

processor_top test(.*);

always begin

#1

mdr_in = test.slc3.cpu.miomux_out;

bus = test.slc3.cpu.bus_out;

SR1out = test.slc3.cpu.SR1_OUT;

pc = test.slc3.cpu.pc_reg.data_q;

mdr = test.slc3.cpu.MDR.data_q;

ir = test.slc3.cpu.ir_reg.data_q;

ld_mdr = test.slc3.cpu.cpu_control.ld_mdr;

state = test.slc3.cpu.cpu_control.state;

hex_in = test.slc3.io_bridge.hex_o.in;

cpu_addr = test.slc3.io_bridge.cpu_addr;

cpu_mem_ena = test.slc3.cpu_mem_ena;

cpu_wr_ena = test.slc3.cpu_wr_ena;

R7 = test.slc3.cpu.reg8_out;

R1 = test.slc3.cpu.reg2_out;

R2 = test.slc3.cpu.reg3_out;

ALU = test.slc3.cpu.ALU_OUT;

// opcode = test.slc3.cpu.cpu_control.opcode;

end

always begin : CLOCK_GENERATION

#1 clk = ~clk;

end


initial begin: CLOCK_INITIALIZATION

clk = 0;

end



initial begin: TEST_VECTORS

reset = 0;

continue_i = 1;

run_i = 0;

 #2 sw_i = 16'h002A;

 reset = 1;

 #10 reset = 0;

 #10 run_i = 1;

 #10 run_i = 0;

 #50000

$finish;

end



endmodule