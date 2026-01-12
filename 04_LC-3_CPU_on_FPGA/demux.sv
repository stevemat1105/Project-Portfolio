module demux(
    input logic ld_signal,
    input logic [2:0] drmux_signal,
    output logic ld_reg1, ld_reg2, ld_reg3, ld_reg4, ld_reg5, ld_reg6, ld_reg7, ld_reg8
    );
    
    always_comb begin
        ld_reg1 = 1'b0;
        ld_reg2 = 1'b0;
        ld_reg3 = 1'b0;
        ld_reg4 = 1'b0;
        ld_reg5 = 1'b0;
        ld_reg6 = 1'b0;
        ld_reg7 = 1'b0;
        ld_reg8 = 1'b0;
        
        case (drmux_signal)
            3'b000: ld_reg1 = ld_signal;
            3'b001: ld_reg2 = ld_signal;
            3'b010: ld_reg3 = ld_signal;
            3'b011: ld_reg4 = ld_signal;
            3'b100: ld_reg5 = ld_signal;
            3'b101: ld_reg6 = ld_signal;
            3'b110: ld_reg7 = ld_signal;
            3'b111: ld_reg8 = ld_signal;
            default: begin
                ld_reg1 = 1'b0;
                ld_reg2 = 1'b0;
                ld_reg3 = 1'b0;
                ld_reg4 = 1'b0;
                ld_reg5 = 1'b0;
                ld_reg6 = 1'b0;
                ld_reg7 = 1'b0;
                ld_reg8 = 1'b0;
            end
        endcase
    end
    
endmodule
