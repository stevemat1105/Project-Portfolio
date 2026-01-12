module alu
import types::*;
(
    input  alu_op_t     op,
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] result
);

    always_comb begin
        case (op)
            ALU_ADD:   result = a + b;
            ALU_SUB:   result = a - b;
            ALU_SLL:   result = a << b[4:0];
            ALU_SLT:   result = {31'b0, $signed(a) < $signed(b)};
            ALU_SLTU:  result = {31'b0, a < b};
            ALU_XOR:   result = a ^ b;
            ALU_SRL:   result = a >> b[4:0];
            ALU_SRA:   result = $unsigned($signed(a) >>> b[4:0]);
            ALU_OR:    result = a | b;
            ALU_AND:   result = a & b;
            ALU_LUI:   result = b;
            ALU_AUIPC: result = a + b;  // PC + imm
            default:   result = 32'h0;
        endcase
    end
endmodule : alu