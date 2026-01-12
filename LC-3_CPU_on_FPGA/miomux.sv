module miomux(
    input logic mio_en,
    input logic [15:0] bus_out,
    input logic [15:0] rdata,
    output logic [15:0] miomux_out
    );
    
    always_comb begin
        case (mio_en)
            1'b0: miomux_out = bus_out;
            1'b1: miomux_out = rdata;
            default: miomux_out = 16'b0;
        endcase
    end
    
endmodule
