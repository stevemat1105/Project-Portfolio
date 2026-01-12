module drmux(
    input logic dr_signal,
    input logic [15:0] ir,
    output logic  [2:0] drmux_out
    );
    
    logic [2:0] ir_shortened;
    assign ir_shortened = ir[11:9];
    
    logic [2:0] ones;
    assign ones = 3'b111;
    
    always_comb begin
        case (dr_signal)
            1'b0: drmux_out = ir_shortened;
            1'b1: drmux_out = ones;
            default: drmux_out = 3'b000;
        endcase
    end
    
endmodule
