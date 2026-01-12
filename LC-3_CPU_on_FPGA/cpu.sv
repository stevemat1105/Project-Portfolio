//------------------------------------------------------------------------------
// Company: 		 UIUC ECE Dept.
// Engineer:		 Stephen Kempf
//
// Create Date:    
// Design Name:    ECE 385 Given Code - SLC-3 core
// Module Name:    SLC3
//
// Comments:
//    Revised 03-22-2007
//    Spring 2007 Distribution
//    Revised 07-26-2013
//    Spring 2015 Distribution
//    Revised 09-22-2015 
//    Revised 06-09-2020
//	  Revised 03-02-2021
//    Xilinx vivado
//    Revised 07-25-2023 
//    Revised 12-29-2023
//    Revised 09-25-2024
//------------------------------------------------------------------------------

module cpu (
    input   logic        clk,
    input   logic        reset,

    input   logic        run_i,
    input   logic        continue_i,
    output  logic [15:0] hex_display_debug,
    output  logic [15:0] led_o,
   
    input   logic [15:0] mem_rdata,
    output  logic [15:0] mem_wdata,
    output  logic [15:0] mem_addr,
    output  logic        mem_mem_ena,
    output  logic        mem_wr_ena
);


// Internal connections, follow the datapath block diagram and add the additional needed signals
logic ld_mar; 
logic ld_mdr; 
logic ld_ir; 
logic ld_pc; 
logic ld_led;

logic gate_marmux;
logic gate_pc;
logic gate_alu;
logic gate_mdr;

logic [1:0] pcmux;

logic [15:0] mar; 
logic [15:0] mdr;
logic [15:0] ir;
logic [15:0] pc;
logic ben;


assign mem_addr = mar;
assign mem_wdata = mdr;

// State machine, you need to fill in the code here as well
// .* auto-infers module input/output connections which have the same name
// This can help visually condense modules with large instantiations, 
// but can also lead to confusing code if used too commonly
control cpu_control (
    .*
);


assign led_o = pc;
assign hex_display_debug = ir;

logic [15:0] bus_out;
logic [15:0] pcmux_output;
logic [15:0] miomux_out;
logic [15:0] ADDR1MUX_OUT;
logic [15:0] ADDR2MUX_OUT;
logic [15:0] ADDRMUX_ADDER_OUTPUT;
logic ld_reg1, ld_reg2, ld_reg3, ld_reg4, ld_reg5, ld_reg6, ld_reg7, ld_reg8;
logic [15:0] reg1_out, reg2_out, reg3_out, reg4_out, reg5_out, reg6_out, reg7_out, reg8_out;
logic [2:0] drmux_out;
logic [2:0] SR1MUX_EXIT;
logic [15:0] SR1_OUT;
logic [15:0] SR2_OUT;
logic [15:0] SR2MUX_OUT;
logic [15:0] ALU_OUT;
logic [2:0] nzp_next;
logic [2:0] nzp_output;

logic [1:0] addr2mux_select;
logic addr1mux_select;
logic ld_reg;
logic dr_select;
logic sr1mux_select;
logic ld_cc;
logic ld_ben;
logic [1:0] alu_select;
logic sr2mux_select;



load_reg #(.DATA_WIDTH(16)) ir_reg (
    .clk    (clk),
    .reset  (reset),

    .load   (ld_ir),
    .data_i (bus_out),

    .data_q (ir)
);

load_reg #(.DATA_WIDTH(16)) pc_reg (
    .clk(clk),
    .reset(reset),

    .load(ld_pc),
    .data_i(pcmux_output),

    .data_q(pc)
);

bus lc3_bus(
    .MARMUX(ADDRMUX_ADDER_OUTPUT),
    .PC(pc),
    .ALU(ALU_OUT),
    .MDR(mdr),
    .GateMARMUX(gate_marmux),
    .GatePC(gate_pc),
    .GateALU(gate_alu),
    .GateMDR(gate_mdr),
    .bus_out(bus_out)
);

logic [15:0] incremented_pc;
assign incremented_pc = pc + 1;

pcmux lc3_pcmux(
    .pcmux_select(pcmux),
    .bus_out(bus_out),
    .marmux_out(ADDRMUX_ADDER_OUTPUT),
    .pc_incremented(incremented_pc),
    .pcmux_output(pcmux_output)
);

load_reg #(.DATA_WIDTH(16)) MAR (
    .clk(clk),
    .reset(reset),

    .load(ld_mar),
    .data_i(bus_out),

    .data_q(mar)
);

miomux lc3_miomux(
    .mio_en(mem_mem_ena),
    .bus_out(bus_out),
    .rdata(mem_rdata),
    .miomux_out(miomux_out)
    );
    

load_reg #(.DATA_WIDTH(16)) MDR (
    .clk(clk),
    .reset(reset),

    .load(ld_mdr),
    .data_i(miomux_out),

    .data_q(mdr)
);

ADDR1MUX lc3_ADDR1MUX(
    .ADDR1MUX_SELECT(addr1mux_select),
    .pc_input(pc),
    .sr1_input(SR1_OUT),
    .ADDR1MUX_OUT(ADDR1MUX_OUT)
);

ADDR2MUX lc3_ADDR2MUX(
    .ADDR2MUX_SELECT(addr2mux_select),
    .ir(ir),
    .ADDR2MUX_OUT(ADDR2MUX_OUT)
);

ADDRMUX_ADDER lc3_ADDRMUX_ADDER(
    .ADDR1MUX_INPUT(ADDR1MUX_OUT),
    .ADDR2MUX_INPUT(ADDR2MUX_OUT),
    .ADDRMUX_ADDER_OUTPUT(ADDRMUX_ADDER_OUTPUT)
);

demux lc3_demux(
    .ld_signal(ld_reg),
    .drmux_signal(drmux_out),
    .ld_reg1(ld_reg1),
    .ld_reg2(ld_reg2),
    .ld_reg3(ld_reg3),
    .ld_reg4(ld_reg4),
    .ld_reg5(ld_reg5),
    .ld_reg6(ld_reg6),
    .ld_reg7(ld_reg7),
    .ld_reg8(ld_reg8)
);

register_file lc3_register_file(
    .bus_input(bus_out),
    .reset(reset),
    .clk(clk),
    .ld_reg1(ld_reg1),
    .ld_reg2(ld_reg2),
    .ld_reg3(ld_reg3),
    .ld_reg4(ld_reg4),
    .ld_reg5(ld_reg5),
    .ld_reg6(ld_reg6),
    .ld_reg7(ld_reg7),
    .ld_reg8(ld_reg8),
    .reg1_out(reg1_out),
    .reg2_out(reg2_out),
    .reg3_out(reg3_out),
    .reg4_out(reg4_out),
    .reg5_out(reg5_out),
    .reg6_out(reg6_out),
    .reg7_out(reg7_out),
    .reg8_out(reg8_out)
);

drmux lc3_drmux(
    .dr_signal(dr_select),
    .ir(ir),
    .drmux_out(drmux_out)
);

SR1MUX lc3_SR1MUX(
    .ir(ir),
    .SR1MUX_signal(sr1mux_select),
    .SR1MUX_EXIT(SR1MUX_EXIT)
);

SR1FINMUX lc3_SR1FINMUX(
    .SR1MUX_EXIT(SR1MUX_EXIT),
    .reg1_in(reg1_out),
    .reg2_in(reg2_out),
    .reg3_in(reg3_out),
    .reg4_in(reg4_out),
    .reg5_in(reg5_out),
    .reg6_in(reg6_out),
    .reg7_in(reg7_out),
    .reg8_in(reg8_out),
    .SR1_OUT(SR1_OUT)
);

SR2FINMUX lc3_SR2FINMUX(
    .SR2_signal(ir[2:0]),
    .reg1_in(reg1_out),
    .reg2_in(reg2_out),
    .reg3_in(reg3_out),
    .reg4_in(reg4_out),
    .reg5_in(reg5_out),
    .reg6_in(reg6_out),
    .reg7_in(reg7_out),
    .reg8_in(reg8_out),
    .SR2_OUT(SR2_OUT)
);

SR2MUX lc3_SR2MUX(
    .SR2MUX_signal(sr2mux_select),
    .SR2OUT_in(SR2_OUT),
    .ir_extended(ir),
    .SR2MUX_OUT(SR2MUX_OUT)
);

ALU lc3_ALU(
    .ALU_signal(alu_select),
    .SR1OUT_in(SR1_OUT),
    .SR2MUXOUT_in(SR2MUX_OUT),
    .ALU_OUT(ALU_OUT)
);

nzp_setter lc3_nzp_setter(
    .bus_input(bus_out),
    .nzp_next(nzp_next)
);

load_reg #(.DATA_WIDTH(3)) nzp_reg (
    .clk(clk),
    .reset(reset),

    .load(ld_cc),
    .data_i(nzp_next),

    .data_q(nzp_output)
);

load_reg #(.DATA_WIDTH(1)) ben_reg (
    .clk(clk),
    .reset(reset),

    .load(ld_ben),
    .data_i((ir[11] & nzp_output[2]) | (ir[10] & nzp_output[1]) | (ir[9] & nzp_output[0])),

    .data_q(ben)
);

endmodule