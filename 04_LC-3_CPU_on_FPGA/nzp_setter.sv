module nzp_setter(
    input logic [15:0] bus_input,
    output logic [2:0] nzp_next
);

always_comb
begin
    nzp_next = 3'b000;
        if (bus_input == 16'h0000) begin
            nzp_next = 3'b010;
        end
        else if (bus_input[15] == 1'b1) begin
            nzp_next = 3'b100;
        end
        else if (bus_input[15] == 1'b0) begin
            nzp_next = 3'b001;
        end

end

endmodule