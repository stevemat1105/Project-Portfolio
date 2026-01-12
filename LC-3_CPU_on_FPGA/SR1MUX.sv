module SR1MUX(
    input logic [15:0] ir,
    input logic SR1MUX_signal,
    output logic [2:0] SR1MUX_EXIT
    );
    
    logic [2:0] ir_shortened;
    assign ir_shortened = ir[11:9];
    
    logic [2:0] ir_shortened2;
    assign ir_shortened2 = ir[8:6];
    
    always_comb begin
        case (SR1MUX_signal)
            1'b0: SR1MUX_EXIT = ir_shortened;
            1'b1: SR1MUX_EXIT = ir_shortened2;
            default: SR1MUX_EXIT = 3'b0; 
        endcase
    end
    
endmodule
