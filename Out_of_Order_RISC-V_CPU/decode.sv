// DECODE STAGE - 2-WAY SUPERSCALAR
//
// Decodes two instructions per cycle (purely combinational)
//
// 1. get inst fields (opcode, funct3, funct7, regs)
// 2. generate immediate values (I, S, B, U, J)
// 3. determine inst type (ALU, MUL, DIV)
// 4. set ctrl signals (uses_rs1, uses_rs2, writes_rd)
// 5. decode ALU/MUL/DIV ops

module decode
import types::*;
(

     
    // INSTRUCTION 0 - Input
     
    input  logic [31:0] pc_in,
    input  logic [31:0] inst_in,
    input  logic        valid_in,
    input  logic [8:0]  prediction_ghr_in,
    input  logic        predicted_taken_in,
    input  logic [31:0] predicted_target_in,

     
    // INSTRUCTION 1 - Input (superscalar)
     
    input  logic [31:0] pc_in_1,
    input  logic [31:0] inst_in_1,
    input  logic        valid_in_1,
    input  logic [8:0]  prediction_ghr_in_1,
    input  logic        predicted_taken_in_1,
    input  logic [31:0] predicted_target_in_1,

     
    // INSTRUCTION 0 - Output
     
    output decoded_inst_t decoded_out,
    output logic          valid_out,

     
    // INSTRUCTION 1 - Output (superscalar)
     
    output decoded_inst_t decoded_out_1,
    output logic          valid_out_1
);

     
    // INSTRUCTION 0 DECODE LOGIC
     

    // inst field breakdown
    logic [6:0] opcode;
    logic [2:0] funct3;
    logic [6:0] funct7;
    logic [4:0] rs1, rs2, rd;

    assign opcode = inst_in[6:0];
    assign funct3 = inst_in[14:12];
    assign funct7 = inst_in[31:25];
    assign rs1    = inst_in[19:15];
    assign rs2    = inst_in[24:20];
    assign rd     = inst_in[11:7];

    // immediate gen
    logic [31:0] imm_i, imm_s, imm_b, imm_u, imm_j;

    assign imm_i = {{20{inst_in[31]}}, inst_in[31:20]};
    assign imm_s = {{20{inst_in[31]}}, inst_in[31:25], inst_in[11:7]};
    assign imm_b = {{19{inst_in[31]}}, inst_in[31], inst_in[7], inst_in[30:25], inst_in[11:8], 1'b0};
    assign imm_u = {inst_in[31:12], 12'h0};
    assign imm_j = {{11{inst_in[31]}}, inst_in[31], inst_in[19:12], inst_in[20], inst_in[30:21], 1'b0};

    logic [31:0] branch_target_calc;
    branch_op_t  branch_op_extracted;

    always_comb begin
        branch_target_calc = 32'h0;
        branch_op_extracted = BR_BEQ;

        case (opcode)
            7'b1100011: begin
                branch_target_calc = pc_in + imm_b;
                branch_op_extracted = branch_op_t'(funct3);
            end
            7'b1101111: begin
                branch_target_calc = pc_in + imm_j;
                branch_op_extracted = BR_BEQ;
            end
            7'b1100111: begin
                branch_target_calc = predicted_target_in;
                branch_op_extracted = BR_BEQ;
            end
            default: begin
                branch_target_calc = 32'h0;
                branch_op_extracted = BR_BEQ;
            end
        endcase
    end

    // Main decode logic for instruction 0
    always_comb begin
        decoded_out = '0;
        decoded_out.pc = pc_in;
        decoded_out.inst = inst_in;
        decoded_out.opcode = opcode;
        decoded_out.funct3 = funct3;
        decoded_out.funct7 = funct7;
        decoded_out.rs1 = rs1;
        decoded_out.rs2 = rs2;
        decoded_out.rd = rd;
        decoded_out.imm = 32'h0;
        decoded_out.uses_rs1 = 1'b0;
        decoded_out.uses_rs2 = 1'b0;
        decoded_out.writes_rd = 1'b0;
        decoded_out.inst_type = ITYPE_ALU;
        decoded_out.alu_op = ALU_ADD;
        decoded_out.mul_op = MUL_MUL;
        decoded_out.div_op = DIV_DIV;

        decoded_out.branch_op = branch_op_extracted;
        decoded_out.branch_target = branch_target_calc;
        decoded_out.predicted_taken = predicted_taken_in;
        decoded_out.prediction_ghr = {1'b0, prediction_ghr_in};

        case (opcode)
            7'b0110011: begin  // R-type
                decoded_out.uses_rs1 = 1'b1;
                decoded_out.uses_rs2 = 1'b1;
                decoded_out.writes_rd = (rd != 5'b0);

                if (funct7 == 7'b0000001) begin
                    case (funct3)
                        3'b000: begin decoded_out.inst_type = ITYPE_MUL; decoded_out.mul_op = MUL_MUL; end
                        3'b001: begin decoded_out.inst_type = ITYPE_MUL; decoded_out.mul_op = MUL_MULH; end
                        3'b010: begin decoded_out.inst_type = ITYPE_MUL; decoded_out.mul_op = MUL_MULHSU; end
                        3'b011: begin decoded_out.inst_type = ITYPE_MUL; decoded_out.mul_op = MUL_MULHU; end
                        3'b100: begin decoded_out.inst_type = ITYPE_DIV; decoded_out.div_op = DIV_DIV; end
                        3'b101: begin decoded_out.inst_type = ITYPE_DIV; decoded_out.div_op = DIV_DIVU; end
                        3'b110: begin decoded_out.inst_type = ITYPE_DIV; decoded_out.div_op = DIV_REM; end
                        3'b111: begin decoded_out.inst_type = ITYPE_DIV; decoded_out.div_op = DIV_REMU; end
                    endcase
                end
                else begin
                    decoded_out.inst_type = ITYPE_ALU;
                    case (funct3)
                        3'b000: decoded_out.alu_op = (funct7[5]) ? ALU_SUB : ALU_ADD;
                        3'b001: decoded_out.alu_op = ALU_SLL;
                        3'b010: decoded_out.alu_op = ALU_SLT;
                        3'b011: decoded_out.alu_op = ALU_SLTU;
                        3'b100: decoded_out.alu_op = ALU_XOR;
                        3'b101: decoded_out.alu_op = (funct7[5]) ? ALU_SRA : ALU_SRL;
                        3'b110: decoded_out.alu_op = ALU_OR;
                        3'b111: decoded_out.alu_op = ALU_AND;
                    endcase
                end
            end

            7'b0010011: begin  // I-type ALU
                decoded_out.uses_rs1 = 1'b1;
                decoded_out.uses_rs2 = 1'b0;
                decoded_out.writes_rd = (rd != 5'b0);
                decoded_out.inst_type = ITYPE_ALU;
                decoded_out.imm = imm_i;

                case (funct3)
                    3'b000: decoded_out.alu_op = ALU_ADD;
                    3'b001: decoded_out.alu_op = ALU_SLL;
                    3'b010: decoded_out.alu_op = ALU_SLT;
                    3'b011: decoded_out.alu_op = ALU_SLTU;
                    3'b100: decoded_out.alu_op = ALU_XOR;
                    3'b101: decoded_out.alu_op = inst_in[30] ? ALU_SRA : ALU_SRL;
                    3'b110: decoded_out.alu_op = ALU_OR;
                    3'b111: decoded_out.alu_op = ALU_AND;
                endcase
            end

            7'b0110111: begin  // LUI
                decoded_out.uses_rs1 = 1'b0;
                decoded_out.uses_rs2 = 1'b0;
                decoded_out.writes_rd = (rd != 5'b0);
                decoded_out.inst_type = ITYPE_ALU;
                decoded_out.alu_op = ALU_LUI;
                decoded_out.imm = imm_u;
            end

            7'b0010111: begin  // AUIPC
                decoded_out.uses_rs1 = 1'b0;
                decoded_out.uses_rs2 = 1'b0;
                decoded_out.writes_rd = (rd != 5'b0);
                decoded_out.inst_type = ITYPE_ALU;
                decoded_out.alu_op = ALU_AUIPC;
                decoded_out.imm = imm_u;
            end

            7'b0000011: begin  // Load
                decoded_out.uses_rs1 = 1'b1;
                decoded_out.uses_rs2 = 1'b0;
                decoded_out.writes_rd = (rd != 5'b0);
                decoded_out.inst_type = ITYPE_LOAD;
                decoded_out.imm = imm_i;
                case (funct3)
                    3'b000: decoded_out.mem_op = MEM_LB;
                    3'b001: decoded_out.mem_op = MEM_LH;
                    3'b010: decoded_out.mem_op = MEM_LW;
                    3'b100: decoded_out.mem_op = MEM_LBU;
                    3'b101: decoded_out.mem_op = MEM_LHU;
                    default: decoded_out.mem_op = MEM_LW;
                endcase
            end

            7'b0100011: begin  // Store
                decoded_out.uses_rs1 = 1'b1;
                decoded_out.uses_rs2 = 1'b1;
                decoded_out.writes_rd = 1'b0;
                decoded_out.inst_type = ITYPE_STORE;
                decoded_out.imm = imm_s;
                case (funct3)
                    3'b000: decoded_out.mem_op = MEM_SB;
                    3'b001: decoded_out.mem_op = MEM_SH;
                    3'b010: decoded_out.mem_op = MEM_SW;
                    default: decoded_out.mem_op = MEM_SW;
                endcase
            end

            7'b1100011: begin  // Branch
                decoded_out.uses_rs1 = 1'b1;
                decoded_out.uses_rs2 = 1'b1;
                decoded_out.writes_rd = 1'b0;
                decoded_out.inst_type = ITYPE_BRANCH;
                decoded_out.imm = imm_b;
            end

            7'b1101111: begin  // JAL
                decoded_out.uses_rs1 = 1'b0;
                decoded_out.uses_rs2 = 1'b0;
                decoded_out.writes_rd = (rd != 5'b0);
                decoded_out.inst_type = ITYPE_JAL;
                decoded_out.imm = imm_j;
            end

            7'b1100111: begin  // JALR
                decoded_out.uses_rs1 = 1'b1;
                decoded_out.uses_rs2 = 1'b0;
                decoded_out.writes_rd = (rd != 5'b0);
                decoded_out.inst_type = ITYPE_JALR;
                decoded_out.imm = imm_i;
            end

            default: begin
                decoded_out.writes_rd = 1'b0;
            end
        endcase
    end

    assign valid_out = valid_in;

     
    // INSTRUCTION 1 DECODE LOGIC (identical combinational logic)
     

    logic [6:0] opcode_1;
    logic [2:0] funct3_1;
    logic [6:0] funct7_1;
    logic [4:0] rs1_1, rs2_1, rd_1;

    assign opcode_1 = inst_in_1[6:0];
    assign funct3_1 = inst_in_1[14:12];
    assign funct7_1 = inst_in_1[31:25];
    assign rs1_1    = inst_in_1[19:15];
    assign rs2_1    = inst_in_1[24:20];
    assign rd_1     = inst_in_1[11:7];

    logic [31:0] imm_i_1, imm_s_1, imm_b_1, imm_u_1, imm_j_1;

    assign imm_i_1 = {{20{inst_in_1[31]}}, inst_in_1[31:20]};
    assign imm_s_1 = {{20{inst_in_1[31]}}, inst_in_1[31:25], inst_in_1[11:7]};
    assign imm_b_1 = {{19{inst_in_1[31]}}, inst_in_1[31], inst_in_1[7], inst_in_1[30:25], inst_in_1[11:8], 1'b0};
    assign imm_u_1 = {inst_in_1[31:12], 12'h0};
    assign imm_j_1 = {{11{inst_in_1[31]}}, inst_in_1[31], inst_in_1[19:12], inst_in_1[20], inst_in_1[30:21], 1'b0};

    logic [31:0] branch_target_calc_1;
    branch_op_t  branch_op_extracted_1;

    always_comb begin
        branch_target_calc_1 = 32'h0;
        branch_op_extracted_1 = BR_BEQ;

        case (opcode_1)
            7'b1100011: begin
                branch_target_calc_1 = pc_in_1 + imm_b_1;
                branch_op_extracted_1 = branch_op_t'(funct3_1);
            end
            7'b1101111: begin
                branch_target_calc_1 = pc_in_1 + imm_j_1;
                branch_op_extracted_1 = BR_BEQ;
            end
            7'b1100111: begin
                branch_target_calc_1 = predicted_target_in_1;
                branch_op_extracted_1 = BR_BEQ;
            end
            default: begin
                branch_target_calc_1 = 32'h0;
                branch_op_extracted_1 = BR_BEQ;
            end
        endcase
    end

    // Main decode logic for instruction 1
    always_comb begin
        decoded_out_1 = '0;
        decoded_out_1.pc = pc_in_1;
        decoded_out_1.inst = inst_in_1;
        decoded_out_1.opcode = opcode_1;
        decoded_out_1.funct3 = funct3_1;
        decoded_out_1.funct7 = funct7_1;
        decoded_out_1.rs1 = rs1_1;
        decoded_out_1.rs2 = rs2_1;
        decoded_out_1.rd = rd_1;
        decoded_out_1.imm = 32'h0;
        decoded_out_1.uses_rs1 = 1'b0;
        decoded_out_1.uses_rs2 = 1'b0;
        decoded_out_1.writes_rd = 1'b0;
        decoded_out_1.inst_type = ITYPE_ALU;
        decoded_out_1.alu_op = ALU_ADD;
        decoded_out_1.mul_op = MUL_MUL;
        decoded_out_1.div_op = DIV_DIV;

        decoded_out_1.branch_op = branch_op_extracted_1;
        decoded_out_1.branch_target = branch_target_calc_1;
        decoded_out_1.predicted_taken = predicted_taken_in_1;
        decoded_out_1.prediction_ghr = {1'b0, prediction_ghr_in_1};

        case (opcode_1)
            7'b0110011: begin  // R-type
                decoded_out_1.uses_rs1 = 1'b1;
                decoded_out_1.uses_rs2 = 1'b1;
                decoded_out_1.writes_rd = (rd_1 != 5'b0);

                if (funct7_1 == 7'b0000001) begin
                    case (funct3_1)
                        3'b000: begin decoded_out_1.inst_type = ITYPE_MUL; decoded_out_1.mul_op = MUL_MUL; end
                        3'b001: begin decoded_out_1.inst_type = ITYPE_MUL; decoded_out_1.mul_op = MUL_MULH; end
                        3'b010: begin decoded_out_1.inst_type = ITYPE_MUL; decoded_out_1.mul_op = MUL_MULHSU; end
                        3'b011: begin decoded_out_1.inst_type = ITYPE_MUL; decoded_out_1.mul_op = MUL_MULHU; end
                        3'b100: begin decoded_out_1.inst_type = ITYPE_DIV; decoded_out_1.div_op = DIV_DIV; end
                        3'b101: begin decoded_out_1.inst_type = ITYPE_DIV; decoded_out_1.div_op = DIV_DIVU; end
                        3'b110: begin decoded_out_1.inst_type = ITYPE_DIV; decoded_out_1.div_op = DIV_REM; end
                        3'b111: begin decoded_out_1.inst_type = ITYPE_DIV; decoded_out_1.div_op = DIV_REMU; end
                    endcase
                end
                else begin
                    decoded_out_1.inst_type = ITYPE_ALU;
                    case (funct3_1)
                        3'b000: decoded_out_1.alu_op = (funct7_1[5]) ? ALU_SUB : ALU_ADD;
                        3'b001: decoded_out_1.alu_op = ALU_SLL;
                        3'b010: decoded_out_1.alu_op = ALU_SLT;
                        3'b011: decoded_out_1.alu_op = ALU_SLTU;
                        3'b100: decoded_out_1.alu_op = ALU_XOR;
                        3'b101: decoded_out_1.alu_op = (funct7_1[5]) ? ALU_SRA : ALU_SRL;
                        3'b110: decoded_out_1.alu_op = ALU_OR;
                        3'b111: decoded_out_1.alu_op = ALU_AND;
                    endcase
                end
            end

            7'b0010011: begin  // I-type ALU
                decoded_out_1.uses_rs1 = 1'b1;
                decoded_out_1.uses_rs2 = 1'b0;
                decoded_out_1.writes_rd = (rd_1 != 5'b0);
                decoded_out_1.inst_type = ITYPE_ALU;
                decoded_out_1.imm = imm_i_1;

                case (funct3_1)
                    3'b000: decoded_out_1.alu_op = ALU_ADD;
                    3'b001: decoded_out_1.alu_op = ALU_SLL;
                    3'b010: decoded_out_1.alu_op = ALU_SLT;
                    3'b011: decoded_out_1.alu_op = ALU_SLTU;
                    3'b100: decoded_out_1.alu_op = ALU_XOR;
                    3'b101: decoded_out_1.alu_op = inst_in_1[30] ? ALU_SRA : ALU_SRL;
                    3'b110: decoded_out_1.alu_op = ALU_OR;
                    3'b111: decoded_out_1.alu_op = ALU_AND;
                endcase
            end

            7'b0110111: begin  // LUI
                decoded_out_1.uses_rs1 = 1'b0;
                decoded_out_1.uses_rs2 = 1'b0;
                decoded_out_1.writes_rd = (rd_1 != 5'b0);
                decoded_out_1.inst_type = ITYPE_ALU;
                decoded_out_1.alu_op = ALU_LUI;
                decoded_out_1.imm = imm_u_1;
            end

            7'b0010111: begin  // AUIPC
                decoded_out_1.uses_rs1 = 1'b0;
                decoded_out_1.uses_rs2 = 1'b0;
                decoded_out_1.writes_rd = (rd_1 != 5'b0);
                decoded_out_1.inst_type = ITYPE_ALU;
                decoded_out_1.alu_op = ALU_AUIPC;
                decoded_out_1.imm = imm_u_1;
            end

            7'b0000011: begin  // Load
                decoded_out_1.uses_rs1 = 1'b1;
                decoded_out_1.uses_rs2 = 1'b0;
                decoded_out_1.writes_rd = (rd_1 != 5'b0);
                decoded_out_1.inst_type = ITYPE_LOAD;
                decoded_out_1.imm = imm_i_1;
                case (funct3_1)
                    3'b000: decoded_out_1.mem_op = MEM_LB;
                    3'b001: decoded_out_1.mem_op = MEM_LH;
                    3'b010: decoded_out_1.mem_op = MEM_LW;
                    3'b100: decoded_out_1.mem_op = MEM_LBU;
                    3'b101: decoded_out_1.mem_op = MEM_LHU;
                    default: decoded_out_1.mem_op = MEM_LW;
                endcase
            end

            7'b0100011: begin  // Store
                decoded_out_1.uses_rs1 = 1'b1;
                decoded_out_1.uses_rs2 = 1'b1;
                decoded_out_1.writes_rd = 1'b0;
                decoded_out_1.inst_type = ITYPE_STORE;
                decoded_out_1.imm = imm_s_1;
                case (funct3_1)
                    3'b000: decoded_out_1.mem_op = MEM_SB;
                    3'b001: decoded_out_1.mem_op = MEM_SH;
                    3'b010: decoded_out_1.mem_op = MEM_SW;
                    default: decoded_out_1.mem_op = MEM_SW;
                endcase
            end

            7'b1100011: begin  // Branch
                decoded_out_1.uses_rs1 = 1'b1;
                decoded_out_1.uses_rs2 = 1'b1;
                decoded_out_1.writes_rd = 1'b0;
                decoded_out_1.inst_type = ITYPE_BRANCH;
                decoded_out_1.imm = imm_b_1;
            end

            7'b1101111: begin  // JAL
                decoded_out_1.uses_rs1 = 1'b0;
                decoded_out_1.uses_rs2 = 1'b0;
                decoded_out_1.writes_rd = (rd_1 != 5'b0);
                decoded_out_1.inst_type = ITYPE_JAL;
                decoded_out_1.imm = imm_j_1;
            end

            7'b1100111: begin  // JALR
                decoded_out_1.uses_rs1 = 1'b1;
                decoded_out_1.uses_rs2 = 1'b0;
                decoded_out_1.writes_rd = (rd_1 != 5'b0);
                decoded_out_1.inst_type = ITYPE_JALR;
                decoded_out_1.imm = imm_i_1;
            end

            default: begin
                decoded_out_1.writes_rd = 1'b0;
            end
        endcase
    end

    assign valid_out_1 = valid_in_1;

endmodule : decode