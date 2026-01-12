module SR2MUX(
    input logic SR2MUX_signal,
    input logic [15:0] SR2OUT_in,
    input logic [15:0] ir_extended,
    output logic [15:0] SR2MUX_OUT 
    );
    
    logic [15:0] ir_four;
    assign ir_four = {{11{ir_extended[4]}}, ir_extended[4:0]};
    
    always_comb begin
        case (SR2MUX_signal)
            1'b0: SR2MUX_OUT = SR2OUT_in;
            1'b1: SR2MUX_OUT = ir_four;
            default: SR2MUX_OUT = 16'b0; 
        endcase
    end
    
endmodule
