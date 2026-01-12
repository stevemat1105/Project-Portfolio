module pcmux(
    input logic [1:0] pcmux_select,
    input logic [15:0] bus_out,
    input logic [15:0] marmux_out,
    input logic [15:0] pc_incremented,
    output logic [15:0] pcmux_output
    );

    always_comb begin
        case (pcmux_select)
            2'b00: pcmux_output = pc_incremented;
            2'b01: pcmux_output = bus_out;
            2'b10: pcmux_output = marmux_out;
            default: pcmux_output = 16'b0;
        endcase
    end
    
endmodule
