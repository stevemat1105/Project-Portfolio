module branch_unit
import types::*;
(
    // from PRF
    input  logic [31:0] rs1_data,
    input  logic [31:0] rs2_data,
    
    // from decode
    input  branch_op_t  branch_op,
    input  inst_type_t  inst_type,
    
    // Branch target calculation
    input  logic [31:0] pc,
    input  logic [31:0] imm,
    
    // Outputs
    output logic        taken,
    output logic [31:0] target
);
    
    // Signed versions of operands for signed comparisons
    logic signed [31:0] rs1_signed, rs2_signed;
    assign rs1_signed = $signed(rs1_data);
    assign rs2_signed = $signed(rs2_data);
    
    logic branch_condition;
    
    always_comb begin
        branch_condition = 1'b0;
        
        case (branch_op)
            BR_BEQ:  branch_condition = (rs1_data == rs2_data);
            BR_BNE:  branch_condition = (rs1_data != rs2_data);
            BR_BLT:  branch_condition = (rs1_signed < rs2_signed);
            BR_BGE:  branch_condition = (rs1_signed >= rs2_signed);
            BR_BLTU: branch_condition = (rs1_data < rs2_data);
            BR_BGEU: branch_condition = (rs1_data >= rs2_data);
            default: branch_condition = 1'b0;
        endcase
    end
    
    always_comb begin
        case (inst_type)
            ITYPE_BRANCH: taken = branch_condition;
            ITYPE_JAL:    taken = 1'b1;
            ITYPE_JALR:   taken = 1'b1;
            default:      taken = 1'b0;
        endcase
    end
    
    always_comb begin
        case (inst_type)
            ITYPE_BRANCH: target = pc + imm;
            ITYPE_JAL:    target = pc + imm;
            ITYPE_JALR:   target = (rs1_data + imm) & ~32'h1;
            default:      target = pc + 32'd4;
        endcase
    end
    
endmodule : branch_unit