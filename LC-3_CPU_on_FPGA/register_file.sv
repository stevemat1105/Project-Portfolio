module register_file(
    input logic [15:0] bus_input,
    input logic reset,
    input logic clk,
    input logic ld_reg1, ld_reg2, ld_reg3, ld_reg4, ld_reg5, ld_reg6, ld_reg7, ld_reg8,
    output logic [15:0] reg1_out, reg2_out, reg3_out, reg4_out, reg5_out, reg6_out, reg7_out, reg8_out
    );
    
    load_reg #(.DATA_WIDTH(16)) REG1 (
        .clk(clk),
        .reset(reset),
    
        .load(ld_reg1),
        .data_i(bus_input),
    
        .data_q(reg1_out)
    );
    
    load_reg #(.DATA_WIDTH(16)) REG2 (
        .clk(clk),
        .reset(reset),
    
        .load(ld_reg2),
        .data_i(bus_input),
    
        .data_q(reg2_out)
    );
    
    load_reg #(.DATA_WIDTH(16)) REG3 (
        .clk(clk),
        .reset(reset),
    
        .load(ld_reg3),
        .data_i(bus_input),
    
        .data_q(reg3_out)
    );
    
    load_reg #(.DATA_WIDTH(16)) REG4 (
        .clk(clk),
        .reset(reset),
    
        .load(ld_reg4),
        .data_i(bus_input),
    
        .data_q(reg4_out)
    );
    
    load_reg #(.DATA_WIDTH(16)) REG5 (
        .clk(clk),
        .reset(reset),
    
        .load(ld_reg5),
        .data_i(bus_input),
    
        .data_q(reg5_out)
    );
    
    load_reg #(.DATA_WIDTH(16)) REG6 (
        .clk(clk),
        .reset(reset),
    
        .load(ld_reg6),
        .data_i(bus_input),
    
        .data_q(reg6_out)
    );
    
    load_reg #(.DATA_WIDTH(16)) REG7 (
        .clk(clk),
        .reset(reset),
    
        .load(ld_reg7),
        .data_i(bus_input),
    
        .data_q(reg7_out)
    );
    
    load_reg #(.DATA_WIDTH(16)) REG8 (
        .clk(clk),
        .reset(reset),
    
        .load(ld_reg8),
        .data_i(bus_input),
    
        .data_q(reg8_out)
    );
    
endmodule
