// COMMIT STAGE
//
// commits insts from ROB head in program order
// - updates architectural state (RRF)
// - frees old physical regs to free list
// - generates RVFI signals
//
// 1. read old preg mapping from RRF
// 2. update RRF with new mapping
// 3. free old preg (ONLY if >= p32, never free p0-p31!)
// 4. generate RVFI signals from ROB data
// 5. dequeue ROB head

module commit
import types::*;
(
    input  logic        clk,
    input  logic        rst,

    // from ROB - port 0 (head)
    input  rob_entry_t  rob_head,
    input  logic        rob_head_valid,
    input  logic        rob_head_ready,
    output logic        rob_commit,

    // from ROB - port 1 (head+1) for dual commit
    input  rob_entry_t  rob_head_1,
    input  logic        rob_head_valid_1,
    input  logic        rob_head_ready_1,
    output logic        rob_commit_1,

    // from ROB - misprediction signal to block dual commit
    input  logic        head_branch_mispredict,

    // to RRF - port 0
    output logic [4:0]  rrf_read_areg,
    input  logic [5:0]  rrf_old_preg,
    output logic        rrf_commit_en,
    output logic [4:0]  rrf_commit_areg,
    output logic [5:0]  rrf_commit_preg,

    // to RRF - port 1 (dual commit)
    output logic [4:0]  rrf_read_areg_1,
    input  logic [5:0]  rrf_old_preg_1,
    output logic        rrf_commit_en_1,
    output logic [4:0]  rrf_commit_areg_1,
    output logic [5:0]  rrf_commit_preg_1,

    // to free list - port 0
    output logic        fl_free,
    output logic [5:0]  fl_free_preg,

    // to free list - port 1 (dual commit)
    output logic        fl_free_1,
    output logic [5:0]  fl_free_preg_1,

    // RVFI intf - array for dual commit: [0] = port 0, [1] = port 1
    output logic        rvfi_valid     [2],
    output logic [63:0] rvfi_order     [2],
    output logic [31:0] rvfi_inst      [2],
    output logic [4:0]  rvfi_rs1_addr  [2],
    output logic [4:0]  rvfi_rs2_addr  [2],
    output logic [31:0] rvfi_rs1_rdata [2],
    output logic [31:0] rvfi_rs2_rdata [2],
    output logic [4:0]  rvfi_rd_addr   [2],
    output logic [31:0] rvfi_rd_wdata  [2],
    output logic [31:0] rvfi_pc_rdata  [2],
    output logic [31:0] rvfi_pc_wdata  [2],
    output logic [31:0] rvfi_mem_addr  [2],
    output logic [3:0]  rvfi_mem_rmask [2],
    output logic [3:0]  rvfi_mem_wmask [2],
    output logic [31:0] rvfi_mem_rdata [2],
    output logic [31:0] rvfi_mem_wdata [2],

    // branch predictor update intf
    output logic        bp_update_en,
    output logic [31:0] bp_update_pc,
    output logic [31:0] bp_update_instr,
    output logic        bp_actual_taken,
    output logic [31:0] bp_actual_target,
    output logic [4:0]  bp_update_rob_idx,
    output logic [8:0]  bp_update_ghr,
    output logic        bp_update_predicted_taken,  // What was predicted (for profiling)

    output logic        bp_update_is_conditional_branch,

    // RAS update intf - port 0
    output logic        ras_update_en,
    output logic        ras_update_is_push,
    output logic        ras_update_is_pop,
    output logic [31:0] ras_update_push_addr,

    // RAS update intf - port 1 (for dual commit)
    output logic        ras_update_en_1,
    output logic        ras_update_is_push_1,
    output logic        ras_update_is_pop_1,
    output logic [31:0] ras_update_push_addr_1
);

    logic [63:0] commit_count;

    // port 0
    logic [6:0] opcode;
    logic uses_rs1, uses_rs2;
    logic [4:0] rs1_addr, rs2_addr, rd_addr;

    // port 1
    logic [6:0] opcode_1;
    logic uses_rs1_1, uses_rs2_1;
    logic [4:0] rs1_addr_1, rs2_addr_1, rd_addr_1;

    logic is_conditional_branch;

    assign is_conditional_branch = (rob_head.inst_type == ITYPE_BRANCH);

    // RAS call/return detection - port 0
    assign rd_addr = rob_head.inst[11:7];
    logic is_call, is_return;

    always_comb begin
        // call detection: JAL/JALR with rd = x1 (ra) or x5 (t0)
        is_call = (rob_head.inst_type == ITYPE_JAL || rob_head.inst_type == ITYPE_JALR) && (rd_addr == 5'd1 || rd_addr == 5'd5);

        // return detection: JALR with rs1 = x1 or x5, and rd = x0
        is_return = (rob_head.inst_type == ITYPE_JALR) && (rs1_addr == 5'd1 || rs1_addr == 5'd5) && (rd_addr == 5'd0);
    end

    // RAS call/return detection - port 1
    logic is_call_1, is_return_1;

    always_comb begin
        // call detection: JAL/JALR with rd = x1 (ra) or x5 (t0)
        is_call_1 = (rob_head_1.inst_type == ITYPE_JAL || rob_head_1.inst_type == ITYPE_JALR) && (rd_addr_1 == 5'd1 || rd_addr_1 == 5'd5);

        // return detection: JALR with rs1 = x1 or x5, and rd = x0
        is_return_1 = (rob_head_1.inst_type == ITYPE_JALR) && (rs1_addr_1 == 5'd1 || rs1_addr_1 == 5'd5) && (rd_addr_1 == 5'd0);
    end
    
    // port 0 decode
    assign opcode = rob_head.inst[6:0];
    assign rs1_addr = rob_head.inst[19:15];
    assign rs2_addr = rob_head.inst[24:20];

    // port 1 decode
    assign opcode_1 = rob_head_1.inst[6:0];
    assign rs1_addr_1 = rob_head_1.inst[19:15];
    assign rs2_addr_1 = rob_head_1.inst[24:20];
    assign rd_addr_1 = rob_head_1.inst[11:7];

    always_comb begin
        // port 0
        case (opcode)
            7'b0110111,     // LUI - no rs1
            7'b0010111,     // AUIPC - no rs1
            7'b1101111:     // JAL - not rs1
                uses_rs1 = 1'b0;
            default:
                uses_rs1 = 1'b1;
        endcase

        case (opcode)
            7'b0110011: uses_rs2 = 1'b1;    // R-type
            7'b0100011: uses_rs2 = 1'b1;    // S-type
            7'b1100011: uses_rs2 = 1'b1;    // B-type
            default:    uses_rs2 = 1'b0;    // I-type, U-type, J-type
        endcase

        // port 1
        case (opcode_1)
            7'b0110111,     // LUI - no rs1
            7'b0010111,     // AUIPC - no rs1
            7'b1101111:     // JAL - not rs1
                uses_rs1_1 = 1'b0;
            default:
                uses_rs1_1 = 1'b1;
        endcase

        case (opcode_1)
            7'b0110011: uses_rs2_1 = 1'b1;    // R-type
            7'b0100011: uses_rs2_1 = 1'b1;    // S-type
            7'b1100011: uses_rs2_1 = 1'b1;    // B-type
            default:    uses_rs2_1 = 1'b0;    // I-type, U-type, J-type
        endcase
    end
    
    // can commit if head is valid and ready
    assign rob_commit = rob_head_valid && rob_head_ready;

    assign rob_commit_1 = rob_commit && !head_branch_mispredict &&
                          rob_head_valid_1 && rob_head_ready_1;

    // read old mapping from RRF - port 0
    assign rrf_read_areg = rob_head.dest_areg;

    // read old mapping from RRF - port 1
    assign rrf_read_areg_1 = rob_head_1.dest_areg;

    // update RRF with new mapping - port 0
    assign rrf_commit_en = rob_commit && (rob_head.dest_areg != 5'b0) && (rob_head.inst_type != ITYPE_STORE) && (rob_head.inst_type != ITYPE_BRANCH);
    assign rrf_commit_areg = rob_head.dest_areg;
    assign rrf_commit_preg = rob_head.dest_preg;

    // update RRF with new mapping - port 1
    assign rrf_commit_en_1 = rob_commit_1 && (rob_head_1.dest_areg != 5'b0) && (rob_head_1.inst_type != ITYPE_STORE) && (rob_head_1.inst_type != ITYPE_BRANCH);
    assign rrf_commit_areg_1 = rob_head_1.dest_areg;
    assign rrf_commit_preg_1 = rob_head_1.dest_preg;

    // free old physical reg - port 0
    // only free speculative regs (p32-p63), never free p0-p31!
    always_comb begin
        if (rob_commit && (rob_head.dest_areg != 5'b0) && (rrf_old_preg != 6'd0) && (rob_head.inst_type != ITYPE_STORE) && (rob_head.inst_type != ITYPE_BRANCH)) begin
            fl_free = 1'b1;
            fl_free_preg = rrf_old_preg;
        end
        else begin
            fl_free = 1'b0;
            fl_free_preg = 6'h0;
        end
    end

    // free old physical reg - port 1
    always_comb begin
        if (rob_commit_1 && (rob_head_1.dest_areg != 5'b0) && (rrf_old_preg_1 != 6'd0) && (rob_head_1.inst_type != ITYPE_STORE) && (rob_head_1.inst_type != ITYPE_BRANCH)) begin
            fl_free_1 = 1'b1;
            fl_free_preg_1 = rrf_old_preg_1;
        end
        else begin
            fl_free_1 = 1'b0;
            fl_free_preg_1 = 6'h0;
        end
    end
    
    // generate RVFI signals
    logic actual_taken;
    integer commit_branch_count;

    always_ff @(posedge clk) begin
        if (rst) begin
            commit_count <= 64'h0;
            rvfi_valid[0] <= 1'b0;
            rvfi_valid[1] <= 1'b0;
            bp_update_en <= 1'b0;
            bp_actual_taken <= 1'b0;
            bp_update_predicted_taken <= 1'b0;
            bp_update_ghr <= 9'h0;
            bp_update_is_conditional_branch <= 1'b0;
            ras_update_en <= 1'b0;
            ras_update_is_push <= 1'b0;
            ras_update_is_pop <= 1'b0;
            ras_update_push_addr <= 32'h0;
            // Port 1 RAS
            ras_update_en_1 <= 1'b0;
            ras_update_is_push_1 <= 1'b0;
            ras_update_is_pop_1 <= 1'b0;
            ras_update_push_addr_1 <= 32'h0;
        end
        else if (rob_commit) begin
            // PORT 0 RVFI
            rvfi_valid[0] <= 1'b1;
            rvfi_order[0] <= commit_count;
            rvfi_inst[0] <= rob_head.inst;

            // src operands - read from RRF+PRF at commit
            rvfi_rs1_addr[0] <= uses_rs1 ? rs1_addr : 5'b0;
            rvfi_rs2_addr[0] <= uses_rs2 ? rs2_addr : 5'b0;
            rvfi_rs1_rdata[0] <= uses_rs1 ? rob_head.rs1_data : 32'h0;
            rvfi_rs2_rdata[0] <= uses_rs2 ? rob_head.rs2_data : 32'h0;

            // dest operand
            rvfi_rd_addr[0] <= rob_head.dest_areg;
            rvfi_rd_wdata[0] <= (rob_head.inst_type == ITYPE_BRANCH) ? 32'h0 : rob_head.result;

            // PC
            rvfi_pc_rdata[0] <= rob_head.pc;
            if (rob_head.inst_type == ITYPE_JAL || rob_head.inst_type == ITYPE_JALR) begin

                if (rob_head.inst_type == ITYPE_JALR) begin

                    logic signed [31:0] jalr_imm;
                    jalr_imm = $signed({{20{rob_head.inst[31]}}, rob_head.inst[31:20]});

                    rvfi_pc_wdata[0] <= (rob_head.rs1_data + 32'($unsigned(jalr_imm))) & ~32'h1;
                end
                else
                    rvfi_pc_wdata[0] <= rob_head.branch_target;
            end
            else if (rob_head.is_branch) begin
                actual_taken = rob_head.result[0];

                if (actual_taken)
                    rvfi_pc_wdata[0] <= rob_head.branch_target;
                else
                    rvfi_pc_wdata[0] <= rob_head.pc + 32'd4;
            end
            else
                rvfi_pc_wdata[0] <= rob_head.pc + 32'd4;

            // memory
            rvfi_mem_addr[0] <= rob_head.mem_addr;
            rvfi_mem_rmask[0] <= rob_head.mem_rmask;
            rvfi_mem_wmask[0] <= rob_head.mem_wmask;
            rvfi_mem_rdata[0] <= (rob_head.mem_rmask != 4'b0) ? rob_head.mem_wdata : 32'h0;
            rvfi_mem_wdata[0] <= (rob_head.mem_wmask != 4'b0) ? rob_head.mem_wdata : 32'h0;

            // PORT 1 RVFI (dual commit)
            if (rob_commit_1) begin
                rvfi_valid[1] <= 1'b1;
                rvfi_order[1] <= commit_count + 64'h1;
                rvfi_inst[1] <= rob_head_1.inst;

                rvfi_rs1_addr[1] <= uses_rs1_1 ? rs1_addr_1 : 5'b0;
                rvfi_rs2_addr[1] <= uses_rs2_1 ? rs2_addr_1 : 5'b0;
                rvfi_rs1_rdata[1] <= uses_rs1_1 ? rob_head_1.rs1_data : 32'h0;
                rvfi_rs2_rdata[1] <= uses_rs2_1 ? rob_head_1.rs2_data : 32'h0;

                rvfi_rd_addr[1] <= rob_head_1.dest_areg;
                rvfi_rd_wdata[1] <= (rob_head_1.inst_type == ITYPE_BRANCH) ? 32'h0 : rob_head_1.result;

                rvfi_pc_rdata[1] <= rob_head_1.pc;

                // Handle PC update for inst_1 - need to handle branches/jumps correctly
                if (rob_head_1.inst_type == ITYPE_JAL || rob_head_1.inst_type == ITYPE_JALR) begin
                    if (rob_head_1.inst_type == ITYPE_JALR) begin
                        logic signed [31:0] jalr_imm_1;
                        jalr_imm_1 = $signed({{20{rob_head_1.inst[31]}}, rob_head_1.inst[31:20]});
                        rvfi_pc_wdata[1] <= (rob_head_1.rs1_data + 32'($unsigned(jalr_imm_1))) & ~32'h1;
                    end
                    else
                        rvfi_pc_wdata[1] <= rob_head_1.branch_target;
                end
                else if (rob_head_1.is_branch) begin
                    logic actual_taken_1_rvfi;
                    actual_taken_1_rvfi = rob_head_1.result[0];
                    if (actual_taken_1_rvfi)
                        rvfi_pc_wdata[1] <= rob_head_1.branch_target;
                    else
                        rvfi_pc_wdata[1] <= rob_head_1.pc + 32'd4;
                end
                else
                    rvfi_pc_wdata[1] <= rob_head_1.pc + 32'd4;

                rvfi_mem_addr[1] <= rob_head_1.mem_addr;
                rvfi_mem_rmask[1] <= rob_head_1.mem_rmask;
                rvfi_mem_wmask[1] <= rob_head_1.mem_wmask;
                rvfi_mem_rdata[1] <= (rob_head_1.mem_rmask != 4'b0) ? rob_head_1.mem_wdata : 32'h0;
                rvfi_mem_wdata[1] <= (rob_head_1.mem_wmask != 4'b0) ? rob_head_1.mem_wdata : 32'h0;

                commit_count <= commit_count + 64'h2;
            end
            else begin
                rvfi_valid[1] <= 1'b0;
                commit_count <= commit_count + 64'h1;
            end

            // Branch predictor update
            if (rob_head.is_branch) begin
                bp_update_en <= 1'b1;
                bp_update_pc <= rob_head.pc;
                bp_update_instr <= rob_head.inst;
                bp_update_rob_idx <= rob_head.rob_id;
                bp_update_ghr <= rob_head.prediction_ghr[8:0];
                bp_update_predicted_taken <= rob_head.predicted_taken;  // For profiling

                // Determine actual_taken based on instruction type
                if (rob_head.inst_type == ITYPE_JAL || rob_head.inst_type == ITYPE_JALR) begin
                    actual_taken = 1'b1;  // Unconditional jumps always taken
                end
                else begin
                    actual_taken = rob_head.result[0];  // Conditional branch result
                end

                bp_actual_taken <= actual_taken;
                bp_actual_target <= rob_head.branch_target;

                bp_update_is_conditional_branch <= is_conditional_branch;

                commit_branch_count = 0;
            end
            else if (rob_commit_1 && rob_head_1.is_branch) begin
                // INST_1 is a branch during dual commit - update BP for inst_1
                // Since head is NOT a branch when dual committing, we can use BP signals for inst_1
                bp_update_en <= 1'b1;
                bp_update_pc <= rob_head_1.pc;
                bp_update_instr <= rob_head_1.inst;
                bp_update_rob_idx <= rob_head_1.rob_id;
                bp_update_ghr <= rob_head_1.prediction_ghr[8:0];
                bp_update_predicted_taken <= rob_head_1.predicted_taken;  // For profiling

                // Determine actual_taken based on instruction type
                if (rob_head_1.inst_type == ITYPE_JAL || rob_head_1.inst_type == ITYPE_JALR) begin
                    actual_taken = 1'b1;  // Unconditional jumps always taken
                end
                else begin
                    actual_taken = rob_head_1.result[0];  // Conditional branch result
                end

                bp_actual_taken <= actual_taken;
                bp_actual_target <= rob_head_1.branch_target;

                // Check if inst_1 is a conditional branch
                bp_update_is_conditional_branch <= (rob_head_1.inst_type == ITYPE_BRANCH);
            end
            else begin
                bp_update_en <= 1'b0;
                bp_update_is_conditional_branch <= 1'b0;
            end

             
            // RAS UPDATE
             

            // Port 0 RAS update (inst_0)
            if (is_call) begin
                ras_update_en <= 1'b1;
                ras_update_is_push <= 1'b1;
                ras_update_is_pop <= 1'b0;
                ras_update_push_addr <= rob_head.pc + 32'd4;
            end
            else if (is_return) begin
                ras_update_en <= 1'b1;
                ras_update_is_push <= 1'b0;
                ras_update_is_pop <= 1'b1;
                ras_update_push_addr <= 32'h0;
            end
            else begin
                ras_update_en <= 1'b0;
                ras_update_is_push <= 1'b0;
                ras_update_is_pop <= 1'b0;
                ras_update_push_addr <= 32'h0;
            end

            // Port 1 RAS update (inst_1) - only during dual commit
            if (rob_commit_1 && is_call_1) begin
                ras_update_en_1 <= 1'b1;
                ras_update_is_push_1 <= 1'b1;
                ras_update_is_pop_1 <= 1'b0;
                ras_update_push_addr_1 <= rob_head_1.pc + 32'd4;
            end
            else if (rob_commit_1 && is_return_1) begin
                ras_update_en_1 <= 1'b1;
                ras_update_is_push_1 <= 1'b0;
                ras_update_is_pop_1 <= 1'b1;
                ras_update_push_addr_1 <= 32'h0;
            end
            else begin
                ras_update_en_1 <= 1'b0;
                ras_update_is_push_1 <= 1'b0;
                ras_update_is_pop_1 <= 1'b0;
                ras_update_push_addr_1 <= 32'h0;
            end
        end
        else begin
            rvfi_valid[0] <= 1'b0;
            rvfi_valid[1] <= 1'b0;
            bp_update_en <= 1'b0;
            ras_update_en <= 1'b0;
            ras_update_en_1 <= 1'b0;
        end
    end

endmodule : commit