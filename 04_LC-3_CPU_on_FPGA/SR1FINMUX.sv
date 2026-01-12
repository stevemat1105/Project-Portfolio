module SR1FINMUX(
    input logic [2:0] SR1MUX_EXIT,
    input logic [15:0] reg1_in, reg2_in, reg3_in, reg4_in, reg5_in, reg6_in, reg7_in, reg8_in,
    output logic [15:0] SR1_OUT
    );
    
    always_comb begin
        case(SR1MUX_EXIT)
            3'b000: SR1_OUT = reg1_in; 
            3'b001: SR1_OUT = reg2_in; 
            3'b010: SR1_OUT = reg3_in; 
            3'b011: SR1_OUT = reg4_in; 
            3'b100: SR1_OUT = reg5_in; 
            3'b101: SR1_OUT = reg6_in; 
            3'b110: SR1_OUT = reg7_in; 
            3'b111: SR1_OUT = reg8_in; 
            default: SR1_OUT = 16'b0;   
        endcase
    end
    
endmodule
