module bus(
    input logic [15:0] MARMUX,  
    input logic [15:0] PC,  
    input logic [15:0] ALU,  
    input logic [15:0] MDR,
    input logic GateMARMUX,
    input logic GatePC,
    input logic GateALU,
    input logic GateMDR,
    output logic [15:0] bus_out 
);

    logic [3:0] select;
    assign select = {GateMARMUX, GatePC, GateALU, GateMDR};
    
    always_comb begin
        case (select)
            4'b1000: bus_out = MARMUX;
            4'b0100: bus_out = PC;
            4'b0010: bus_out = ALU;
            4'b0001: bus_out = MDR;
            default: bus_out = 16'bx;
        endcase
    end
endmodule
