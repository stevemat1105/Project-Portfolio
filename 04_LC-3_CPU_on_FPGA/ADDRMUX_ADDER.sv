module ADDRMUX_ADDER(
    input logic [15:0] ADDR1MUX_INPUT,
    input logic [15:0] ADDR2MUX_INPUT,
    output logic [15:0] ADDRMUX_ADDER_OUTPUT
    );
    
    assign ADDRMUX_ADDER_OUTPUT = ADDR1MUX_INPUT + ADDR2MUX_INPUT;
    
endmodule
