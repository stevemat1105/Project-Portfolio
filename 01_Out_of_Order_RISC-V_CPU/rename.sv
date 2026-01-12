// RENAME STAGE
//
// 1. RAT Lookup - map architectural regs to physical regs
// 2. Free List Alloc - get new physical reg for dest
// 3. ROB Alloc - get ROB entry for tracking
// 4. RVFI Data Capture - read PRF to get src operand values and store in ROB
// 5. Dispatch - send renamed instruction to appropriate RS
// 6. Stall Logic - stop when resources unavailable
// 7. x0 Handling - never allocate physical reg for x0
// 8. AUIPC case - pass PC as rs1_data
//
// IN: decoded_inst_t (from decode)
// OUT: renamed_inst_t (to RS dispatch)

module rename
import types::*;
#(
    parameter NUM_PREGS = 64,
    parameter ROB_SIZE = 16
)(
    input  decoded_inst_t   decoded_in,
    input  logic            valid_in,

    input  decoded_inst_t   decoded_in_1,
    input  logic            valid_in_1,

    output logic [4:0]      rat_rs1_areg,
    input  logic [5:0]      rat_rs1_preg,
    input  logic            rat_rs1_ready,
    input  logic [4:0]      rat_rs1_rob_id,

    output logic [4:0]      rat_rs2_areg,
    input  logic [5:0]      rat_rs2_preg,
    input  logic            rat_rs2_ready,
    input  logic [4:0]      rat_rs2_rob_id,

    output logic            rat_alloc_en,
    output logic [4:0]      rat_rd_areg,
    output logic [5:0]      rat_rd_preg,
    output logic [4:0]      rat_rd_rob_id,

    output logic [4:0]      rat_rs1_areg_1,
    input  logic [5:0]      rat_rs1_preg_1,
    input  logic            rat_rs1_ready_1,
    input  logic [4:0]      rat_rs1_rob_id_1,

    output logic [4:0]      rat_rs2_areg_1,
    input  logic [5:0]      rat_rs2_preg_1,
    input  logic            rat_rs2_ready_1,
    input  logic [4:0]      rat_rs2_rob_id_1,

    output logic            rat_alloc_en_1,
    output logic [4:0]      rat_rd_areg_1,
    output logic [5:0]      rat_rd_preg_1,
    output logic [4:0]      rat_rd_rob_id_1,
     
    output logic            fl_alloc,
    input  logic [5:0]      fl_alloc_preg,
    input  logic            fl_empty,
     
    output logic            fl_alloc_1,
    input  logic [5:0]      fl_alloc_preg_1,
    input  logic            fl_almost_empty,

    output logic            rob_enq,
    output rob_entry_t      rob_enq_data,
    input  logic [4:0]      rob_idx,
    input  logic            rob_full,

    output logic            rob_enq_1,
    output rob_entry_t      rob_enq_data_1,
    input  logic [4:0]      rob_idx_1,
    input  logic            rob_almost_full,

    input  logic            alu_rs_full,
    input  logic            alu_rs_almost_full,
    input  logic            mul_rs_full,
    input  logic            div_rs_full,
    input  logic            branch_rs_full,
    input  logic            lq_full,
    input  logic            sq_full,

    input  logic [2:0]      mul_rs_occupancy,
    input  logic [2:0]      div_rs_occupancy,
    input  logic [$clog2(NUM_BRANCH_RS):0] branch_rs_occupancy,
    input  logic [$clog2(NUM_LQ_ENTRIES):0] lq_occupancy,
    input  logic [$clog2(NUM_SQ_ENTRIES):0] sq_occupancy,
     
    output logic [5:0]      prf_rs1_preg,
    input  logic [31:0]     prf_rs1_data,
    
    output logic [5:0]      prf_rs2_preg,
    input  logic [31:0]     prf_rs2_data,

    output logic [5:0]      prf_rs1_preg_1,
    input  logic [31:0]     prf_rs1_data_1,

    output logic [5:0]      prf_rs2_preg_1,
    input  logic [31:0]     prf_rs2_data_1,

    output logic [4:0]      rrf_rs1_areg,
    output logic [4:0]      rrf_rs2_areg,
    output logic [4:0]      rrf_rs1_areg_1,
    output logic [4:0]      rrf_rs2_areg_1,

    input  cdb_t            cdb [NUM_CDB_PORTS],
    
    // ALU forwarding (for RVFI only)
    input  logic            alu_issue_valid,
    input  logic [5:0]      alu_dest_preg,
    input  logic [31:0]     alu_result,
    
    // to RS
    output renamed_inst_t   renamed_out,
    output logic            valid_out,

    output renamed_inst_t   renamed_out_1,
    output logic            valid_out_1,

    // stall to fetch
    output logic            stall_to_fetch,

    input  logic            predicted_taken,
    input  logic            predicted_taken_1,

    input  logic            flush
);

    logic can_dispatch;
    logic need_dest_preg;
    logic is_auipc;
    
    // forwarding match signals
    logic alu_fwd_rs1_match, alu_fwd_rs2_match;
    logic cdb_fwd_rs1_match [NUM_CDB_PORTS];
    logic cdb_fwd_rs2_match [NUM_CDB_PORTS];
    
    logic rs1_ready_for_rs, rs2_ready_for_rs;
    logic [31:0] rs1_data_for_rvfi, rs2_data_for_rvfi;

    logic can_dispatch_1;
    logic need_dest_preg_1;
    logic is_auipc_1;

    // forwarding match signals for inst 1
    logic alu_fwd_rs1_match_1, alu_fwd_rs2_match_1;
    logic cdb_fwd_rs1_match_1 [NUM_CDB_PORTS];
    logic cdb_fwd_rs2_match_1 [NUM_CDB_PORTS];

    logic rs1_ready_for_rs_1, rs2_ready_for_rs_1;
    logic [31:0] rs1_data_for_rvfi_1, rs2_data_for_rvfi_1;

    assign need_dest_preg = decoded_in.writes_rd && (decoded_in.rd != 5'b0) && (decoded_in.inst_type != ITYPE_STORE);
    assign need_dest_preg_1 = decoded_in_1.writes_rd && (decoded_in_1.rd != 5'b0) && (decoded_in_1.inst_type != ITYPE_STORE);

    assign is_auipc = (decoded_in.alu_op == ALU_AUIPC);
    assign is_auipc_1 = (decoded_in_1.alu_op == ALU_AUIPC);

    logic target_rs_full;
    always_comb begin
        case (decoded_in.inst_type)
            ITYPE_ALU: target_rs_full = alu_rs_full;
            ITYPE_MUL: target_rs_full = mul_rs_full;
            ITYPE_DIV: target_rs_full = div_rs_full;
            ITYPE_LOAD:  target_rs_full = lq_full;
            ITYPE_STORE: target_rs_full = sq_full;
            ITYPE_BRANCH, ITYPE_JAL, ITYPE_JALR: target_rs_full = branch_rs_full;
            default:   target_rs_full = 1'b0;
        endcase
    end

    logic target_rs_full_1;
    logic same_rs_type;

    assign same_rs_type = (decoded_in.inst_type == decoded_in_1.inst_type) ||
                          ((decoded_in.inst_type == ITYPE_LOAD || decoded_in.inst_type == ITYPE_STORE) &&
                           (decoded_in_1.inst_type == ITYPE_LOAD || decoded_in_1.inst_type == ITYPE_STORE)) ||
                          ((decoded_in.inst_type == ITYPE_BRANCH || decoded_in.inst_type == ITYPE_JAL || decoded_in.inst_type == ITYPE_JALR) &&
                           (decoded_in_1.inst_type == ITYPE_BRANCH || decoded_in_1.inst_type == ITYPE_JAL || decoded_in_1.inst_type == ITYPE_JALR));

    always_comb begin
        case (decoded_in_1.inst_type)
            ITYPE_ALU: begin
                if (same_rs_type && can_dispatch)
                    target_rs_full_1 = alu_rs_almost_full;
                else
                    target_rs_full_1 = alu_rs_full;
            end
            ITYPE_MUL: begin
                if (same_rs_type && can_dispatch)
                    target_rs_full_1 = (mul_rs_occupancy >= ($clog2(NUM_MUL_RS)+1)'($unsigned(NUM_MUL_RS - 1)));
                else
                    target_rs_full_1 = mul_rs_full;
            end
            ITYPE_DIV: begin
                if (same_rs_type && can_dispatch)
                    target_rs_full_1 = (div_rs_occupancy >= ($clog2(NUM_DIV_RS)+1)'($unsigned(NUM_DIV_RS - 1)));
                else
                    target_rs_full_1 = div_rs_full;
            end
            ITYPE_LOAD: begin
                if (decoded_in.inst_type == ITYPE_LOAD && can_dispatch)
                    target_rs_full_1 = (lq_occupancy >= ($clog2(NUM_LQ_ENTRIES)+1)'($unsigned(NUM_LQ_ENTRIES - 1)));
                else
                    target_rs_full_1 = lq_full;
            end
            ITYPE_STORE: begin
                if (decoded_in.inst_type == ITYPE_STORE && can_dispatch)
                    target_rs_full_1 = (sq_occupancy >= ($clog2(NUM_SQ_ENTRIES)+1)'($unsigned(NUM_SQ_ENTRIES - 1)));
                else
                    target_rs_full_1 = sq_full;
            end
            ITYPE_BRANCH, ITYPE_JAL, ITYPE_JALR: begin
                if (same_rs_type && can_dispatch)
                    target_rs_full_1 = (branch_rs_occupancy >= ($clog2(NUM_BRANCH_RS)+1)'($unsigned(NUM_BRANCH_RS - 1)));
                else
                    target_rs_full_1 = branch_rs_full;
            end
            default: target_rs_full_1 = 1'b0;
        endcase
    end

    assign can_dispatch = valid_in && !rob_full && (!need_dest_preg || !fl_empty) && !target_rs_full;

    logic fl_has_enough;
    assign fl_has_enough = !need_dest_preg_1 || (!need_dest_preg && !fl_empty) || (need_dest_preg && !fl_almost_empty);

    assign can_dispatch_1 = valid_in_1 && can_dispatch && !rob_almost_full && fl_has_enough && !target_rs_full_1;

    assign stall_to_fetch = (valid_in && !can_dispatch) || (valid_in_1 && can_dispatch && !can_dispatch_1);

    assign rat_rs1_areg = decoded_in.rs1;
    assign rat_rs2_areg = decoded_in.rs2;
    assign rat_rs1_areg_1 = decoded_in_1.rs1;
    assign rat_rs2_areg_1 = decoded_in_1.rs2;

    assign rrf_rs1_areg = decoded_in.rs1;
    assign rrf_rs2_areg = decoded_in.rs2;
    assign rrf_rs1_areg_1 = decoded_in_1.rs1;
    assign rrf_rs2_areg_1 = decoded_in_1.rs2;

    logic [5:0] safe_prf_rs1_preg, safe_prf_rs2_preg;
    assign safe_prf_rs1_preg = rat_rs1_ready ? rat_rs1_preg : 6'd0;
    assign safe_prf_rs2_preg = rat_rs2_ready ? rat_rs2_preg : 6'd0;
    assign prf_rs1_preg = safe_prf_rs1_preg;
    assign prf_rs2_preg = safe_prf_rs2_preg;

     
    // PRF READ - Instruction 1
     
    logic [5:0] safe_prf_rs1_preg_1, safe_prf_rs2_preg_1;
    assign safe_prf_rs1_preg_1 = rat_rs1_ready_1 ? rat_rs1_preg_1 : 6'd0;
    assign safe_prf_rs2_preg_1 = rat_rs2_ready_1 ? rat_rs2_preg_1 : 6'd0;
    assign prf_rs1_preg_1 = safe_prf_rs1_preg_1;
    assign prf_rs2_preg_1 = safe_prf_rs2_preg_1;

     
    // FORWARDING - Instruction 0
     
    assign alu_fwd_rs1_match = alu_issue_valid && (rat_rs1_preg == alu_dest_preg) && (rat_rs1_preg != 6'd0);
    assign alu_fwd_rs2_match = alu_issue_valid && (rat_rs2_preg == alu_dest_preg) && (rat_rs2_preg != 6'd0);

    // CDB forwarding with rob_id validation
    logic cdb_fwd_rs1_valid [NUM_CDB_PORTS];
    logic cdb_fwd_rs2_valid [NUM_CDB_PORTS];
    logic any_cdb_fwd_rs1, any_cdb_fwd_rs2;
    logic has_intra_cycle_fwd_rs1, has_intra_cycle_fwd_rs2;

    always_comb begin
        for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
            cdb_fwd_rs1_match[i] = cdb[i].valid && (rat_rs1_preg == cdb[i].preg) && (rat_rs1_preg != 6'd0);
            cdb_fwd_rs2_match[i] = cdb[i].valid && (rat_rs2_preg == cdb[i].preg) && (rat_rs2_preg != 6'd0);
            // Validate CDB broadcast: preg match AND rob_id match
            cdb_fwd_rs1_valid[i] = cdb_fwd_rs1_match[i] && (cdb[i].rob_id == rat_rs1_rob_id);
            cdb_fwd_rs2_valid[i] = cdb_fwd_rs2_match[i] && (cdb[i].rob_id == rat_rs2_rob_id);
        end
    end

    always_comb begin
        any_cdb_fwd_rs1 = 1'b0;
        any_cdb_fwd_rs2 = 1'b0;
        for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
            if (cdb_fwd_rs1_valid[i]) any_cdb_fwd_rs1 = 1'b1;
            if (cdb_fwd_rs2_valid[i]) any_cdb_fwd_rs2 = 1'b1;
        end
    end

    always_comb begin
        if (alu_fwd_rs1_match)
            rs1_data_for_rvfi = alu_result;
        else begin
            rs1_data_for_rvfi = prf_rs1_data;
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb_fwd_rs1_valid[i])  // Use validated match with rob_id check
                    rs1_data_for_rvfi = cdb[i].data;
            end
        end

        if (alu_fwd_rs2_match)
            rs2_data_for_rvfi = alu_result;
        else begin
            rs2_data_for_rvfi = prf_rs2_data;
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb_fwd_rs2_valid[i])  // Use validated match with rob_id check
                    rs2_data_for_rvfi = cdb[i].data;
            end
        end
    end

    // RESTORED CDB forwarding with rob_id validation:
    // This is safe because we now validate that cdb.rob_id matches the expected producer.
    // Stale CDB broadcasts will fail the rob_id check and not incorrectly set ready=1.
    assign rs1_ready_for_rs = rat_rs1_ready || any_cdb_fwd_rs1;
    assign rs2_ready_for_rs = rat_rs2_ready || any_cdb_fwd_rs2;

     
    // FORWARDING - Instruction 1
     
    assign alu_fwd_rs1_match_1 = alu_issue_valid && (rat_rs1_preg_1 == alu_dest_preg) && (rat_rs1_preg_1 != 6'd0);
    assign alu_fwd_rs2_match_1 = alu_issue_valid && (rat_rs2_preg_1 == alu_dest_preg) && (rat_rs2_preg_1 != 6'd0);

    // Detect intra-cycle forwarding: if inst 0 is allocating the same register inst 1 reads
    // This is used to determine if inst 1 needs to wait for inst 0's result
    assign has_intra_cycle_fwd_rs1 = (rat_alloc_en && rat_rd_areg != 5'b0 && rat_rd_areg == rat_rs1_areg_1);
    assign has_intra_cycle_fwd_rs2 = (rat_alloc_en && rat_rd_areg != 5'b0 && rat_rd_areg == rat_rs2_areg_1);

    // CDB forwarding with rob_id validation for instruction 1
    logic cdb_fwd_rs1_valid_1 [NUM_CDB_PORTS];
    logic cdb_fwd_rs2_valid_1 [NUM_CDB_PORTS];
    logic any_cdb_fwd_rs1_1, any_cdb_fwd_rs2_1;

    always_comb begin
        for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
            cdb_fwd_rs1_match_1[i] = cdb[i].valid && (rat_rs1_preg_1 == cdb[i].preg) && (rat_rs1_preg_1 != 6'd0);
            cdb_fwd_rs2_match_1[i] = cdb[i].valid && (rat_rs2_preg_1 == cdb[i].preg) && (rat_rs2_preg_1 != 6'd0);
            // Validate CDB broadcast: preg match AND rob_id match
            cdb_fwd_rs1_valid_1[i] = cdb_fwd_rs1_match_1[i] && (cdb[i].rob_id == rat_rs1_rob_id_1);
            cdb_fwd_rs2_valid_1[i] = cdb_fwd_rs2_match_1[i] && (cdb[i].rob_id == rat_rs2_rob_id_1);
        end
    end

    always_comb begin
        any_cdb_fwd_rs1_1 = 1'b0;
        any_cdb_fwd_rs2_1 = 1'b0;
        for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
            if (cdb_fwd_rs1_valid_1[i]) any_cdb_fwd_rs1_1 = 1'b1;
            if (cdb_fwd_rs2_valid_1[i]) any_cdb_fwd_rs2_1 = 1'b1;
        end
    end

    always_comb begin
        // When intra-cycle RAT forwarding occurs, don't use CDB data (it's stale)
        // Just use PRF data (which will be updated by inst 0's CDB broadcast later)
        if (alu_fwd_rs1_match_1)
            rs1_data_for_rvfi_1 = alu_result;
        else if (has_intra_cycle_fwd_rs1) begin
            // Intra-cycle forwarding: use PRF data only (ignore CDB)
            rs1_data_for_rvfi_1 = prf_rs1_data_1;
        end
        else begin
            rs1_data_for_rvfi_1 = prf_rs1_data_1;
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb_fwd_rs1_valid_1[i])  // Use validated match with rob_id check
                    rs1_data_for_rvfi_1 = cdb[i].data;
            end
        end

        if (alu_fwd_rs2_match_1)
            rs2_data_for_rvfi_1 = alu_result;
        else if (has_intra_cycle_fwd_rs2) begin
            // Intra-cycle forwarding: use PRF data only (ignore CDB)
            rs2_data_for_rvfi_1 = prf_rs2_data_1;
        end
        else begin
            rs2_data_for_rvfi_1 = prf_rs2_data_1;
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb_fwd_rs2_valid_1[i])  // Use validated match with rob_id check
                    rs2_data_for_rvfi_1 = cdb[i].data;
            end
        end
    end

    // RESTORED CDB forwarding with rob_id validation:
    // This is safe because we now validate that cdb.rob_id matches the expected producer.
    // Stale CDB broadcasts will fail the rob_id check and not incorrectly set ready=1.
    assign rs1_ready_for_rs_1 = rat_rs1_ready_1 || any_cdb_fwd_rs1_1;
    assign rs2_ready_for_rs_1 = rat_rs2_ready_1 || any_cdb_fwd_rs2_1;

     
    // FREE LIST ALLOCATION
     
    assign fl_alloc = can_dispatch && need_dest_preg && !flush;
    assign fl_alloc_1 = can_dispatch_1 && need_dest_preg_1 && !flush;

     
    // RAT ALLOCATION
     
    assign rat_alloc_en = can_dispatch && need_dest_preg;
    assign rat_rd_areg = decoded_in.rd;
    assign rat_rd_preg = fl_alloc_preg;
    assign rat_rd_rob_id = rob_idx;  // Pass ROB ID to RAT for CDB validation

    assign rat_alloc_en_1 = can_dispatch_1 && need_dest_preg_1;
    assign rat_rd_areg_1 = decoded_in_1.rd;
    assign rat_rd_preg_1 = fl_alloc_preg_1;
    assign rat_rd_rob_id_1 = rob_idx_1;  // Pass ROB ID to RAT for CDB validation

     
    // ROB ENQUEUE - Instruction 0
     
    assign rob_enq = can_dispatch;

    always_comb begin
        rob_enq_data = '0;

        if (can_dispatch) begin
            rob_enq_data.valid      = 1'b1;
            rob_enq_data.ready      = 1'b0;
            rob_enq_data.dest_preg  = need_dest_preg ? fl_alloc_preg : 6'b0;
            rob_enq_data.dest_areg  = decoded_in.writes_rd ? decoded_in.rd : 5'b0;
            rob_enq_data.pc         = decoded_in.pc;
            rob_enq_data.inst       = decoded_in.inst;
            rob_enq_data.rob_id     = rob_idx;

            rob_enq_data.rs1_data = rs1_data_for_rvfi;
            rob_enq_data.rs2_data = rs2_data_for_rvfi;

            rob_enq_data.rs1_preg = rat_rs1_preg;
            rob_enq_data.rs2_preg = rat_rs2_preg;
            // Use validated CDB forwarding (with rob_id check) for same-cycle wakeup
            rob_enq_data.rs1_valid = rat_rs1_ready || any_cdb_fwd_rs1;
            rob_enq_data.rs2_valid = rat_rs2_ready || any_cdb_fwd_rs2;
            // Store producer rob_ids for CDB validation in ROB
            rob_enq_data.rs1_rob_id = rat_rs1_rob_id;
            rob_enq_data.rs2_rob_id = rat_rs2_rob_id;

            rob_enq_data.result = 32'h0;
            rob_enq_data.prediction_ghr = decoded_in.prediction_ghr;

            rob_enq_data.inst_type = decoded_in.inst_type;
            rob_enq_data.branch_op = decoded_in.branch_op;
            rob_enq_data.is_branch = (decoded_in.inst_type == ITYPE_BRANCH) ||
                                     (decoded_in.inst_type == ITYPE_JAL) ||
                                     (decoded_in.inst_type == ITYPE_JALR);

            if (rob_enq_data.is_branch) begin
                rob_enq_data.predicted_taken = predicted_taken;
                rob_enq_data.branch_target = decoded_in.branch_target;
            end
            else begin
                rob_enq_data.predicted_taken = 1'b0;
                rob_enq_data.branch_target = 32'h0;
            end
        end
    end

     
    // ROB ENQUEUE - Instruction 1
     
    assign rob_enq_1 = can_dispatch_1;

    always_comb begin
        rob_enq_data_1 = '0;

        if (can_dispatch_1) begin
            rob_enq_data_1.valid      = 1'b1;
            rob_enq_data_1.ready      = 1'b0;
            rob_enq_data_1.dest_preg  = need_dest_preg_1 ? fl_alloc_preg_1 : 6'b0;
            rob_enq_data_1.dest_areg  = decoded_in_1.writes_rd ? decoded_in_1.rd : 5'b0;
            rob_enq_data_1.pc         = decoded_in_1.pc;
            rob_enq_data_1.inst       = decoded_in_1.inst;
            rob_enq_data_1.rob_id     = rob_idx_1;

            rob_enq_data_1.rs1_data = rs1_data_for_rvfi_1;
            rob_enq_data_1.rs2_data = rs2_data_for_rvfi_1;

            rob_enq_data_1.rs1_preg = rat_rs1_preg_1;
            rob_enq_data_1.rs2_preg = rat_rs2_preg_1;
            // Use validated CDB forwarding (with rob_id check) for same-cycle wakeup
            rob_enq_data_1.rs1_valid = rat_rs1_ready_1 || any_cdb_fwd_rs1_1;
            rob_enq_data_1.rs2_valid = rat_rs2_ready_1 || any_cdb_fwd_rs2_1;
            // Store producer rob_ids for CDB validation in ROB
            rob_enq_data_1.rs1_rob_id = rat_rs1_rob_id_1;
            rob_enq_data_1.rs2_rob_id = rat_rs2_rob_id_1;

            rob_enq_data_1.result = 32'h0;
            rob_enq_data_1.prediction_ghr = decoded_in_1.prediction_ghr;

            rob_enq_data_1.inst_type = decoded_in_1.inst_type;
            rob_enq_data_1.branch_op = decoded_in_1.branch_op;
            rob_enq_data_1.is_branch = (decoded_in_1.inst_type == ITYPE_BRANCH) ||
                                       (decoded_in_1.inst_type == ITYPE_JAL) ||
                                       (decoded_in_1.inst_type == ITYPE_JALR);

            if (rob_enq_data_1.is_branch) begin
                rob_enq_data_1.predicted_taken = predicted_taken_1;
                rob_enq_data_1.branch_target = decoded_in_1.branch_target;
            end
            else begin
                rob_enq_data_1.predicted_taken = 1'b0;
                rob_enq_data_1.branch_target = 32'h0;
            end
        end
    end

     
    // OUTPUT - Instruction 0
     
    assign valid_out = can_dispatch;

    always_comb begin
        renamed_out = '0;

        if (can_dispatch) begin
            renamed_out.pc          = decoded_in.pc;
            renamed_out.inst        = decoded_in.inst;
            renamed_out.inst_type   = decoded_in.inst_type;
            renamed_out.alu_op      = decoded_in.alu_op;
            renamed_out.mul_op      = decoded_in.mul_op;
            renamed_out.div_op      = decoded_in.div_op;
            renamed_out.mem_op      = decoded_in.mem_op;
            renamed_out.rob_id      = rob_idx;
            renamed_out.rd_areg     = decoded_in.rd;

            renamed_out.branch_op = decoded_in.branch_op;
            renamed_out.branch_target = decoded_in.branch_target;
            renamed_out.predicted_taken = predicted_taken;

            if (is_auipc) begin
                renamed_out.rs1_preg  = 6'b0;
                renamed_out.rs1_ready = 1'b1;
                renamed_out.rs1_data  = decoded_in.pc;

                renamed_out.rs2_imm     = decoded_in.imm;
                renamed_out.rs2_is_imm  = 1'b1;
                renamed_out.rs2_ready   = 1'b1;
                renamed_out.rs2_data    = decoded_in.imm;
            end
            else begin
                if (decoded_in.uses_rs1) begin
                    renamed_out.rs1_preg  = rat_rs1_preg;
                    renamed_out.rs1_ready = rs1_ready_for_rs;
                    renamed_out.rs1_data  = rs1_data_for_rvfi;
                end
                else begin
                    renamed_out.rs1_preg  = 6'b0;
                    renamed_out.rs1_ready = 1'b1;
                    renamed_out.rs1_data  = 32'h0;
                end

                if (decoded_in.uses_rs2) begin
                    renamed_out.rs2_preg    = rat_rs2_preg;
                    renamed_out.rs2_ready   = rs2_ready_for_rs;
                    renamed_out.rs2_data    = rs2_data_for_rvfi;
                    renamed_out.rs2_is_imm  = 1'b0;

                    if (decoded_in.inst_type == ITYPE_STORE)
                        renamed_out.rs2_imm = decoded_in.imm;
                end
                else begin
                    renamed_out.rs2_imm     = decoded_in.imm;
                    renamed_out.rs2_is_imm  = 1'b1;
                    renamed_out.rs2_ready   = 1'b1;
                    renamed_out.rs2_data    = decoded_in.imm;
                end
            end

            if (need_dest_preg)
                renamed_out.rd_preg = fl_alloc_preg;
            else
                renamed_out.rd_preg = 6'b0;
        end
    end

     
    // OUTPUT - Instruction 1
     
    assign valid_out_1 = can_dispatch_1;

    always_comb begin
        renamed_out_1 = '0;

        if (can_dispatch_1) begin
            renamed_out_1.pc          = decoded_in_1.pc;
            renamed_out_1.inst        = decoded_in_1.inst;
            renamed_out_1.inst_type   = decoded_in_1.inst_type;
            renamed_out_1.alu_op      = decoded_in_1.alu_op;
            renamed_out_1.mul_op      = decoded_in_1.mul_op;
            renamed_out_1.div_op      = decoded_in_1.div_op;
            renamed_out_1.mem_op      = decoded_in_1.mem_op;
            renamed_out_1.rob_id      = rob_idx_1;
            renamed_out_1.rd_areg     = decoded_in_1.rd;

            renamed_out_1.branch_op = decoded_in_1.branch_op;
            renamed_out_1.branch_target = decoded_in_1.branch_target;
            renamed_out_1.predicted_taken = predicted_taken_1;

            if (is_auipc_1) begin
                renamed_out_1.rs1_preg  = 6'b0;
                renamed_out_1.rs1_ready = 1'b1;
                renamed_out_1.rs1_data  = decoded_in_1.pc;

                renamed_out_1.rs2_imm     = decoded_in_1.imm;
                renamed_out_1.rs2_is_imm  = 1'b1;
                renamed_out_1.rs2_ready   = 1'b1;
                renamed_out_1.rs2_data    = decoded_in_1.imm;
            end
            else begin
                if (decoded_in_1.uses_rs1) begin
                    renamed_out_1.rs1_preg  = rat_rs1_preg_1;
                    renamed_out_1.rs1_ready = rs1_ready_for_rs_1;
                    renamed_out_1.rs1_data  = rs1_data_for_rvfi_1;
                end
                else begin
                    renamed_out_1.rs1_preg  = 6'b0;
                    renamed_out_1.rs1_ready = 1'b1;
                    renamed_out_1.rs1_data  = 32'h0;
                end

                if (decoded_in_1.uses_rs2) begin
                    renamed_out_1.rs2_preg    = rat_rs2_preg_1;
                    renamed_out_1.rs2_ready   = rs2_ready_for_rs_1;
                    renamed_out_1.rs2_data    = rs2_data_for_rvfi_1;
                    renamed_out_1.rs2_is_imm  = 1'b0;

                    if (decoded_in_1.inst_type == ITYPE_STORE)
                        renamed_out_1.rs2_imm = decoded_in_1.imm;
                end
                else begin
                    renamed_out_1.rs2_imm     = decoded_in_1.imm;
                    renamed_out_1.rs2_is_imm  = 1'b1;
                    renamed_out_1.rs2_ready   = 1'b1;
                    renamed_out_1.rs2_data    = decoded_in_1.imm;
                end
            end

            if (need_dest_preg_1)
                renamed_out_1.rd_preg = fl_alloc_preg_1;
            else
                renamed_out_1.rd_preg = 6'b0;
        end
    end

endmodule : rename