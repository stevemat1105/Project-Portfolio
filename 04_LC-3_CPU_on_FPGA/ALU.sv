module ALU(
    input logic [1:0] ALU_signal,
    input logic [15:0] SR1OUT_in,
    input logic [15:0] SR2MUXOUT_in,
    output logic [15:0] ALU_OUT
);

    always_comb begin
        case (ALU_signal)
            2'b00: ALU_OUT = SR1OUT_in + SR2MUXOUT_in;  
            2'b01: ALU_OUT = SR1OUT_in & SR2MUXOUT_in; 
            2'b10: ALU_OUT = ~SR1OUT_in;               
            2'b11: ALU_OUT = SR1OUT_in;               
            default: ALU_OUT = 16'b0; 
        endcase
    end

endmodule
