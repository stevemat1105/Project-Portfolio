module ADDR1MUX(
    input logic ADDR1MUX_SELECT,
    input logic [15:0] pc_input,
    input logic [15:0] sr1_input,
    output logic [15:0] ADDR1MUX_OUT
    );
    
    always_comb begin
        case (ADDR1MUX_SELECT)
            1'b0: ADDR1MUX_OUT = pc_input;
            1'b1: ADDR1MUX_OUT = sr1_input;
            default: ADDR1MUX_OUT = 16'b0000000000000000; 
        endcase
    end
    
endmodule

