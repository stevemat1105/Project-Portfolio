module ADDR2MUX(
    input logic [1:0] ADDR2MUX_SELECT,
    input logic [15:0] ir,
    output logic [15:0] ADDR2MUX_OUT
    );
    
    logic [15:0] ten_choice;
    logic [15:0] eight_choice;
    logic [15:0] five_choice;
    logic [15:0] zero_choice;
    
    assign ten_choice = {{5{ir[10]}}, ir[10:0]};
    assign eight_choice = {{7{ir[8]}}, ir[8:0]};
    assign five_choice = {{10{ir[5]}}, ir[5:0]};
    assign zero_choice = 16'b0;
    
    always_comb begin
        case (ADDR2MUX_SELECT)
            2'b00: ADDR2MUX_OUT = zero_choice;
            2'b01: ADDR2MUX_OUT = five_choice;
            2'b10: ADDR2MUX_OUT = eight_choice;
            2'b11: ADDR2MUX_OUT = ten_choice;
            default: ADDR2MUX_OUT = 16'b0; 
        endcase
    end

endmodule
