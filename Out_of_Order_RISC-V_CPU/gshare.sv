// 2-WAY SUPERSCALAR GSHARE BRANCH PREDICTOR
//
// Optimized for 2-way superscalar fetch with single-port SRAM:
// - When only inst_0 is branch: PHT prediction for inst_0
// - When only inst_1 is branch: PHT prediction for inst_1 (KEY OPTIMIZATION!)
// - When both are branches: PHT for inst_0, BTFN for inst_1 (no stall)
// - When neither is branch: speculatively read for next cycle
//
// Also extends RAS to handle inst_1 JALR returns when inst_0 is not control flow.

module gshare_simple
import types::*;
(
    input  logic        clk,
    input  logic        rst,

     
    // INSTRUCTION 0 INTERFACE
     
    input  logic [31:0] fetch_pc,
    input  logic        fetch_valid,
    input  logic [31:0] fetch_instr,
    output logic        predict_taken,
    output logic [31:0] predicted_target,
    output logic [8:0]  prediction_ghr_out,

     
    // INSTRUCTION 1 INTERFACE (2-way superscalar)
     
    input  logic [31:0] fetch_pc_1,
    input  logic        fetch_valid_1,
    input  logic [31:0] fetch_instr_1,
    output logic        predict_taken_1,
    output logic [31:0] predicted_target_1,
    output logic [8:0]  prediction_ghr_out_1,
    output logic        inst_1_uses_pht,      // For profiling: did inst_1 get PHT prediction?

     
    // UPDATE INTERFACE (from commit)
     
    input  logic        update_is_conditional_branch,
    input  logic        update_en,
    input  logic [31:0] update_pc,
    input  logic        actual_taken,
    input  logic [8:0]  update_ghr,

    input  logic        flush,

    // RAS update from commit - port 0
    input  logic        ras_update_en,
    input  logic        ras_update_is_push,
    input  logic        ras_update_is_pop,
    input  logic [31:0] ras_update_push_addr,

    // RAS update from commit - port 1 (dual commit)
    input  logic        ras_update_en_1,
    input  logic        ras_update_is_push_1,
    input  logic        ras_update_is_pop_1,
    input  logic [31:0] ras_update_push_addr_1
);

     
    // INSTRUCTION 0 DECODE
     
    logic [6:0] opcode;
    logic is_branch, is_jal, is_jalr;
    logic [31:0] branch_imm, jal_imm;
    logic [4:0] rd, rs1;
    logic is_control_flow;

    assign opcode = fetch_instr[6:0];
    assign is_branch = (opcode == 7'b1100011);  // B-type
    assign is_jal    = (opcode == 7'b1101111);  // JAL
    assign is_jalr   = (opcode == 7'b1100111);  // JALR
    assign is_control_flow = is_branch || is_jal || is_jalr;

    assign rd  = fetch_instr[11:7];
    assign rs1 = fetch_instr[19:15];

    assign branch_imm = {{20{fetch_instr[31]}}, fetch_instr[7], fetch_instr[30:25], fetch_instr[11:8], 1'b0};
    assign jal_imm = {{12{fetch_instr[31]}}, fetch_instr[19:12], fetch_instr[20], fetch_instr[30:21], 1'b0};

    // call detection: JAL/JALR with rd = x1 (ra) or x5 (t0, alt link reg)
    logic is_call;
    assign is_call = (is_jal || is_jalr) && (rd == 5'd1 || rd == 5'd5);

    // return detection: JALR with rs1 = x1 or x5, and rd = x0
    logic is_return;
    assign is_return = is_jalr && (rs1 == 5'd1 || rs1 == 5'd5) && (rd == 5'd0);

     
    // INSTRUCTION 1 DECODE
     
    logic [6:0] opcode_1;
    logic is_branch_1, is_jal_1, is_jalr_1;
    logic [31:0] branch_imm_1, jal_imm_1;
    logic [4:0] rd_1, rs1_1;
    logic is_control_flow_1;

    assign opcode_1 = fetch_instr_1[6:0];
    assign is_branch_1 = (opcode_1 == 7'b1100011);  // B-type
    assign is_jal_1    = (opcode_1 == 7'b1101111);  // JAL
    assign is_jalr_1   = (opcode_1 == 7'b1100111);  // JALR
    assign is_control_flow_1 = is_branch_1 || is_jal_1 || is_jalr_1;

    assign rd_1  = fetch_instr_1[11:7];
    assign rs1_1 = fetch_instr_1[19:15];

    assign branch_imm_1 = {{20{fetch_instr_1[31]}}, fetch_instr_1[7], fetch_instr_1[30:25], fetch_instr_1[11:8], 1'b0};
    assign jal_imm_1 = {{12{fetch_instr_1[31]}}, fetch_instr_1[19:12], fetch_instr_1[20], fetch_instr_1[30:21], 1'b0};

    // call/return detection for inst_1
    logic is_call_1, is_return_1;
    assign is_call_1 = (is_jal_1 || is_jalr_1) && (rd_1 == 5'd1 || rd_1 == 5'd5);
    assign is_return_1 = is_jalr_1 && (rs1_1 == 5'd1 || rs1_1 == 5'd5) && (rd_1 == 5'd0);

     
    // GHR (Global History Register)
     
    logic [8:0] speculative_ghr;
    logic [8:0] speculative_ghr_for_prediction;
    logic [8:0] committed_ghr;

     
    // RAS (Return Address Stack)
     
    parameter RAS_SIZE = 16;

    // speculative RAS (updated at fetch)
    logic [31:0] speculative_ras [RAS_SIZE];
    logic [3:0]  speculative_ras_tos;
    logic        speculative_ras_empty;

    // committed RAS (updated at commit)
    logic [31:0] committed_ras [RAS_SIZE];
    logic [3:0]  committed_ras_tos;
    logic        committed_ras_empty;

     
    // SRAM PHT (512 entries x 2 bits, single RW port)
     
    logic       pht_valid [512];

    logic       sram_csb;
    logic       sram_web;       // 0=write, 1=read
    logic [8:0] sram_addr;
    logic [1:0] sram_din;
    logic [1:0] sram_dout;

     
    // UPDATE FSM (read-modify-write for SRAM)
     
    typedef enum logic [1:0] {
        UPDATE_IDLE  = 2'b00,
        UPDATE_READ  = 2'b01,
        UPDATE_WRITE = 2'b10
    } update_state_t;

    update_state_t update_state, update_state_next;
    logic [8:0]  update_index_q;
    logic [31:0] update_pc_q;
    logic        update_actual_taken_q;
    logic [1:0]  update_new_counter;

    logic        pending_update;
    logic [8:0]  pending_index;
    logic [31:0] pending_pc;
    logic        pending_taken;

     
    // SRAM INDEX SELECTION (KEY OPTIMIZATION)
     
    // Determine which instruction needs PHT prediction this cycle:
    // - If inst_0 is a branch: read for inst_0 (unless backward - BTFN is good for loops)
    // - Else if inst_1 is a branch: read for inst_1 (inst_0 doesn't need PHT!)
    // - Else: speculatively read for next cycle's inst_0
    //
    // NEW OPTIMIZATION: When both are branches and inst_0 is BACKWARD (loop back-edge),
    // BTFN predicts taken with ~90% accuracy. Give PHT to inst_1 instead!

    logic predict_for_inst_1;  // If true, SRAM read is for inst_1
    logic [31:0] sram_lookup_pc;
    logic [8:0]  sram_lookup_index;

    // Detect if inst_0 is a backward branch (negative immediate = MSB set)
    logic inst_0_is_backward_branch;
    assign inst_0_is_backward_branch = is_branch && branch_imm[31];

    always_comb begin
        if (fetch_valid && is_branch) begin
            if (inst_0_is_backward_branch && fetch_valid_1 && is_branch_1) begin
                // OPTIMIZATION: inst_0 is backward branch (BTFN works well)
                // AND inst_1 is also a branch - give PHT to inst_1!
                predict_for_inst_1 = 1'b1;
                sram_lookup_pc = fetch_pc_1;
            end
            else begin
                // inst_0 is forward branch (needs PHT), or inst_1 is not a branch
                predict_for_inst_1 = 1'b0;
                sram_lookup_pc = fetch_pc;
            end
        end
        else if (fetch_valid_1 && is_branch_1) begin
            // Only inst_1 is a branch - use SRAM for inst_1!
            predict_for_inst_1 = 1'b1;
            sram_lookup_pc = fetch_pc_1;
        end
        else begin
            // Neither is a branch - speculatively read for next cycle
            predict_for_inst_1 = 1'b0;
            sram_lookup_pc = fetch_pc + 32'd8;  // 2-way: next bundle is PC+8
        end
    end

    // GHR to use for index calculation
    // For inst_1, if inst_0 was also a branch, we should include inst_0's prediction in GHR
    // Cases where we read SRAM for inst_1:
    // 1. inst_0 is NOT a branch -> use speculative_ghr
    // 2. inst_0 IS a backward branch (using BTFN=taken) -> include inst_0's taken prediction
    logic [8:0] sram_lookup_ghr;
    always_comb begin
        if (predict_for_inst_1 && inst_0_is_backward_branch) begin
            // Backward branch optimization: inst_0 will predict taken (BTFN)
            // Include this in GHR for inst_1's lookup
            sram_lookup_ghr = {speculative_ghr[7:0], 1'b1};  // inst_0 predicts taken
        end
        else begin
            sram_lookup_ghr = speculative_ghr;
        end
    end

    assign sram_lookup_index = sram_lookup_pc[10:2] ^ sram_lookup_ghr;

     
    // SRAM PORT ARBITRATION
     
    always_comb begin
        case (update_state)
            UPDATE_READ: begin
                sram_csb = 1'b0;
                sram_web = 1'b1;
                sram_addr = update_index_q;
                sram_din = 2'b00;
            end

            UPDATE_WRITE: begin
                sram_csb = 1'b0;
                sram_web = 1'b0;
                sram_addr = update_index_q;
                sram_din = update_new_counter;
            end

            default: begin
                sram_csb = ~(fetch_valid || fetch_valid_1);
                sram_web = 1'b1;
                sram_addr = sram_lookup_index;
                sram_din = 2'b00;
            end
        endcase
    end

     
    // PIPELINE REGISTERS (match SRAM 1-cycle latency)
     
    logic [31:0] prev_fetch_pc;
    logic        pc_sequential;
    logic        predict_for_inst_1_pipe;  // Which instruction the SRAM read was for

    logic [31:0] fetch_pc_pipe;
    logic        fetch_valid_pipe;
    logic [31:0] fetch_instr_pipe;
    logic [8:0]  speculative_ghr_pipe;
    logic [8:0]  sram_lookup_index_pipe;
    logic        pht_read_valid_pipe;

    logic [1:0]  pht_dout;
    logic        pht_valid_out;

    assign pht_dout = sram_dout;
    assign pht_valid_out = pht_valid[sram_lookup_index_pipe] && pht_read_valid_pipe && pc_sequential;

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            fetch_pc_pipe <= 32'h0;
            fetch_valid_pipe <= 1'b0;
            fetch_instr_pipe <= 32'h0;
            speculative_ghr_pipe <= 9'h0;
            sram_lookup_index_pipe <= 9'h0;
            pht_read_valid_pipe <= 1'b0;
            prev_fetch_pc <= 32'hAAAAA000;
            pc_sequential <= 1'b0;
            predict_for_inst_1_pipe <= 1'b0;
        end
        else if (fetch_valid || fetch_valid_1) begin
            fetch_pc_pipe <= fetch_pc;
            fetch_valid_pipe <= fetch_valid;
            fetch_instr_pipe <= fetch_instr;
            speculative_ghr_pipe <= sram_lookup_ghr;
            sram_lookup_index_pipe <= sram_lookup_index;
            pht_read_valid_pipe <= (update_state == UPDATE_IDLE);
            prev_fetch_pc <= fetch_pc;
            // Accept +4 (scalar) or +8 (2-way) as sequential
            pc_sequential <= (fetch_pc == prev_fetch_pc + 32'd4) || (fetch_pc == prev_fetch_pc + 32'd8);
            predict_for_inst_1_pipe <= predict_for_inst_1;
        end
        else begin
            fetch_valid_pipe <= 1'b0;
            pht_read_valid_pipe <= 1'b0;
        end
    end

     
    // PHT VALID TRACKING
     
    logic [8:0] write_pending_index;
    logic       write_pending_valid;
    logic [8:0] write_complete_index;
    logic       write_complete_valid;

    always_ff @(posedge clk) begin
        if (rst) begin
            write_pending_valid <= 1'b0;
            write_pending_index <= 9'h0;
            write_complete_valid <= 1'b0;
            write_complete_index <= 9'h0;
        end
        else begin
            write_pending_valid <= (update_state == UPDATE_WRITE);
            write_pending_index <= update_index_q;
            write_complete_valid <= write_pending_valid;
            write_complete_index <= write_pending_index;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            for (integer i = 0; i < 512; i++) begin
                pht_valid[i] <= 1'b0;
            end
        end
        else if (write_complete_valid) begin
            pht_valid[write_complete_index] <= 1'b1;
        end
    end

     
    // UPDATE FSM
     
    always_ff @(posedge clk) begin
        if (rst)
            update_state <= UPDATE_IDLE;
        else
            update_state <= update_state_next;
    end

    always_comb begin
        update_state_next = update_state;
        case (update_state)
            UPDATE_IDLE: begin
                if (update_en && update_is_conditional_branch)
                    update_state_next = UPDATE_READ;
            end
            UPDATE_READ: begin
                update_state_next = UPDATE_WRITE;
            end
            UPDATE_WRITE: begin
                if ((update_en && update_is_conditional_branch) || pending_update)
                    update_state_next = UPDATE_READ;
                else
                    update_state_next = UPDATE_IDLE;
            end
            default: update_state_next = UPDATE_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            pending_update <= 1'b0;
            pending_index <= 9'h0;
            pending_pc <= 32'h0;
            pending_taken <= 1'b0;
        end
        else if (update_en && update_is_conditional_branch && update_state == UPDATE_READ) begin
            pending_update <= 1'b1;
            pending_index <= update_pc[10:2] ^ update_ghr[8:0];
            pending_pc <= update_pc;
            pending_taken <= actual_taken;
        end
        else if (pending_update && update_state_next == UPDATE_READ) begin
            pending_update <= 1'b0;
        end
    end

    always_ff @(posedge clk) begin
        if (update_state_next == UPDATE_READ) begin
            if (pending_update) begin
                update_index_q <= pending_index;
                update_pc_q <= pending_pc;
                update_actual_taken_q <= pending_taken;
            end
            else if (update_en && update_is_conditional_branch) begin
                update_index_q <= update_pc[10:2] ^ update_ghr[8:0];
                update_pc_q <= update_pc;
                update_actual_taken_q <= actual_taken;
            end
        end
    end

    always_comb begin
        logic [1:0] old_counter;
        if (update_state == UPDATE_WRITE) begin
            old_counter = pht_valid[update_index_q] ? sram_dout : 2'b10;
        end
        else begin
            old_counter = 2'b10;
        end

        if (update_actual_taken_q)
            update_new_counter = (old_counter == 2'b11) ? 2'b11 : old_counter + 2'b01;
        else
            update_new_counter = (old_counter == 2'b00) ? 2'b00 : old_counter - 2'b01;
    end

     
    // INSTRUCTION 0 PREDICTION
     
    assign prediction_ghr_out = speculative_ghr_for_prediction;

    // Determine if PHT data is valid for inst_0
    // PHT is valid for inst_0 if we read for inst_0 (not inst_1) and conditions are met
    logic pht_valid_for_inst_0;
    assign pht_valid_for_inst_0 = pht_valid_out && !predict_for_inst_1_pipe;

    always_comb begin
        if (rst) begin
            predict_taken = 1'b0;
        end
        else if (is_jal || is_jalr) begin
            predict_taken = 1'b1;
        end
        else if (is_branch) begin
            if (pht_valid_for_inst_0)
                predict_taken = pht_dout[1];
            else
                predict_taken = branch_imm[31];  // BTFN fallback
        end
        else begin
            predict_taken = 1'b0;
        end
    end

    always_comb begin
        predicted_target = fetch_pc + 32'd4;
        if (fetch_valid) begin
            if (is_branch && predict_taken)
                predicted_target = fetch_pc + branch_imm;
            else if (is_jal)
                predicted_target = fetch_pc + jal_imm;
            else if (is_jalr) begin
                if (is_return && !speculative_ras_empty)
                    predicted_target = speculative_ras[speculative_ras_tos];
                else
                    predicted_target = fetch_pc + 32'd4;
            end
        end
    end

     
    // INSTRUCTION 1 PREDICTION
     
    // inst_1 can use PHT if:
    // 1. We read SRAM for inst_1 (predict_for_inst_1_pipe = 1)
    // 2. PHT entry is valid
    // Otherwise, use BTFN

    logic pht_valid_for_inst_1;
    assign pht_valid_for_inst_1 = pht_valid_out && predict_for_inst_1_pipe;
    assign inst_1_uses_pht = pht_valid_for_inst_1 && is_branch_1;

    // GHR for inst_1: if inst_0 is a branch, include inst_0's prediction
    logic [8:0] ghr_for_inst_1;
    assign ghr_for_inst_1 = is_branch ? {speculative_ghr[7:0], predict_taken} : speculative_ghr;
    assign prediction_ghr_out_1 = ghr_for_inst_1;

    always_comb begin
        if (rst) begin
            predict_taken_1 = 1'b0;
        end
        else if (is_jal_1 || is_jalr_1) begin
            predict_taken_1 = 1'b1;
        end
        else if (is_branch_1) begin
            if (pht_valid_for_inst_1)
                predict_taken_1 = pht_dout[1];  // PHT prediction!
            else
                predict_taken_1 = branch_imm_1[31];  // BTFN fallback
        end
        else begin
            predict_taken_1 = 1'b0;
        end
    end

    // RAS access for inst_1
    // inst_1 can use RAS if inst_0 is NOT a call (which would change TOS before inst_1 needs it)
    // For simplicity, allow RAS access for inst_1 if inst_0 is not any control flow
    logic can_use_ras_for_inst_1;
    assign can_use_ras_for_inst_1 = !is_control_flow;

    // RAS TOS after inst_0's effect (if inst_0 is a call/return)
    logic [3:0] ras_tos_after_inst_0;
    logic       ras_empty_after_inst_0;
    logic [3:0] ras_tos_next, ras_tos_prev;

    assign ras_tos_next = (speculative_ras_tos == 4'd15) ? 4'd0 : speculative_ras_tos + 4'd1;
    assign ras_tos_prev = (speculative_ras_tos == 4'd0) ? 4'd15 : speculative_ras_tos - 4'd1;

    always_comb begin
        ras_tos_after_inst_0 = speculative_ras_tos;
        ras_empty_after_inst_0 = speculative_ras_empty;

        if (fetch_valid && predict_taken) begin
            if (is_call) begin
                ras_tos_after_inst_0 = ras_tos_next;
                ras_empty_after_inst_0 = 1'b0;
            end
            else if (is_return && !speculative_ras_empty) begin
                ras_tos_after_inst_0 = ras_tos_prev;
                ras_empty_after_inst_0 = (speculative_ras_tos == 4'd0);
            end
        end
    end

    always_comb begin
        predicted_target_1 = fetch_pc_1 + 32'd4;
        if (fetch_valid_1) begin
            if (is_branch_1 && predict_taken_1)
                predicted_target_1 = fetch_pc_1 + branch_imm_1;
            else if (is_jal_1)
                predicted_target_1 = fetch_pc_1 + jal_imm_1;
            else if (is_jalr_1) begin
                // Use RAS for returns, accounting for inst_0's effect
                if (is_return_1 && !ras_empty_after_inst_0) begin
                    // If inst_0 was a call, RAS was pushed, so we read from new TOS
                    // If inst_0 was a return, RAS was popped, so we read from new TOS
                    // If inst_0 was neither, TOS unchanged
                    predicted_target_1 = speculative_ras[ras_tos_after_inst_0];
                end
                else begin
                    predicted_target_1 = fetch_pc_1 + 32'd4;
                end
            end
        end
    end

     
    // GHR UPDATE
     
    always_ff @(posedge clk) begin
        if (rst) begin
            speculative_ghr <= 9'h0;
            speculative_ghr_for_prediction <= 9'h0;
            committed_ghr <= 9'h0;
        end
        else begin
            speculative_ghr_for_prediction <= speculative_ghr;

            priority case (1'b1)
                flush: begin
                    if (update_en && update_is_conditional_branch) begin
                        committed_ghr <= {committed_ghr[7:0], actual_taken};
                        speculative_ghr <= {committed_ghr[7:0], actual_taken};
                    end
                    else begin
                        speculative_ghr <= committed_ghr;
                    end
                end

                update_en: begin
                    if (update_is_conditional_branch)
                        committed_ghr <= {committed_ghr[7:0], actual_taken};
                end

                default: begin
                    // Update speculative GHR for both inst_0 and inst_1 branches
                    if (fetch_valid && is_branch && fetch_valid_1 && is_branch_1) begin
                        // Both are branches: shift in both predictions
                        speculative_ghr <= {speculative_ghr[6:0], predict_taken, predict_taken_1};
                    end
                    else if (fetch_valid && is_branch) begin
                        speculative_ghr <= {speculative_ghr[7:0], predict_taken};
                    end
                    else if (fetch_valid_1 && is_branch_1) begin
                        speculative_ghr <= {speculative_ghr[7:0], predict_taken_1};
                    end
                end
            endcase
        end
    end

     
    // RAS UPDATE
     
    logic [3:0] committed_ras_tos_next, committed_ras_tos_prev;
    assign committed_ras_tos_next = (committed_ras_tos == 4'd15) ? 4'd0 : committed_ras_tos + 4'd1;
    assign committed_ras_tos_prev = (committed_ras_tos == 4'd0) ? 4'd15 : committed_ras_tos - 4'd1;

    // Compute TOS after port 0 commit effect (for dual commit)
    logic [3:0] committed_ras_tos_after_port0;
    logic       committed_ras_empty_after_port0;
    logic [3:0] committed_ras_tos_after_port0_next;

    always_comb begin
        if (ras_update_en && ras_update_is_push) begin
            committed_ras_tos_after_port0 = committed_ras_tos_next;
            committed_ras_empty_after_port0 = 1'b0;
        end
        else if (ras_update_en && ras_update_is_pop && !committed_ras_empty) begin
            committed_ras_tos_after_port0 = committed_ras_tos_prev;
            committed_ras_empty_after_port0 = (committed_ras_tos == 4'd0);
        end
        else begin
            committed_ras_tos_after_port0 = committed_ras_tos;
            committed_ras_empty_after_port0 = committed_ras_empty;
        end
        committed_ras_tos_after_port0_next = (committed_ras_tos_after_port0 == 4'd15) ? 4'd0 : committed_ras_tos_after_port0 + 4'd1;
    end

    // Compute TOS after inst_0 speculative effect (for dual fetch)
    logic [3:0] spec_ras_tos_after_inst0;
    logic       spec_ras_empty_after_inst0;
    logic [3:0] spec_ras_tos_after_inst0_next, spec_ras_tos_after_inst0_prev;

    always_comb begin
        if (fetch_valid && predict_taken && is_call) begin
            spec_ras_tos_after_inst0 = ras_tos_next;
            spec_ras_empty_after_inst0 = 1'b0;
        end
        else if (fetch_valid && predict_taken && is_return && !speculative_ras_empty) begin
            spec_ras_tos_after_inst0 = ras_tos_prev;
            spec_ras_empty_after_inst0 = (speculative_ras_tos == 4'd0);
        end
        else begin
            spec_ras_tos_after_inst0 = speculative_ras_tos;
            spec_ras_empty_after_inst0 = speculative_ras_empty;
        end
        spec_ras_tos_after_inst0_next = (spec_ras_tos_after_inst0 == 4'd15) ? 4'd0 : spec_ras_tos_after_inst0 + 4'd1;
        spec_ras_tos_after_inst0_prev = (spec_ras_tos_after_inst0 == 4'd0) ? 4'd15 : spec_ras_tos_after_inst0 - 4'd1;
    end
    
    logic inst_1_is_call, inst_1_is_return;

    always_ff @(posedge clk) begin
        if (rst) begin
            speculative_ras_tos <= 4'd0;
            speculative_ras_empty <= 1'b1;
            committed_ras_tos <= 4'd0;
            committed_ras_empty <= 1'b1;
            for (integer i = 0; i < RAS_SIZE; i++) begin
                speculative_ras[i] <= 32'h0;
                committed_ras[i] <= 32'h0;
            end
        end
        else begin
            priority case (1'b1)
                flush: begin
                    // Restore speculative from committed, then apply any pending commit updates
                    speculative_ras <= committed_ras;
                    speculative_ras_tos <= committed_ras_tos;
                    speculative_ras_empty <= committed_ras_empty;

                    // Apply port 0 committed update
                    if (ras_update_en && ras_update_is_push) begin
                        committed_ras[committed_ras_tos_next] <= ras_update_push_addr;
                        speculative_ras[committed_ras_tos_next] <= ras_update_push_addr;
                    end

                    // Apply port 1 committed update (uses TOS after port 0)
                    if (ras_update_en_1 && ras_update_is_push_1) begin
                        committed_ras[committed_ras_tos_after_port0_next] <= ras_update_push_addr_1;
                        speculative_ras[committed_ras_tos_after_port0_next] <= ras_update_push_addr_1;
                    end

                    // Update committed TOS accounting for both ports
                    if (ras_update_en_1 && ras_update_is_push_1) begin
                        committed_ras_tos <= committed_ras_tos_after_port0_next;
                        committed_ras_empty <= 1'b0;
                        speculative_ras_tos <= committed_ras_tos_after_port0_next;
                        speculative_ras_empty <= 1'b0;
                    end
                    else if (ras_update_en_1 && ras_update_is_pop_1 && !committed_ras_empty_after_port0) begin
                        logic [3:0] tos_after_both;
                        tos_after_both = (committed_ras_tos_after_port0 == 4'd0) ? 4'd15 : committed_ras_tos_after_port0 - 4'd1;
                        committed_ras_tos <= tos_after_both;
                        committed_ras_empty <= (committed_ras_tos_after_port0 == 4'd0);
                        speculative_ras_tos <= tos_after_both;
                        speculative_ras_empty <= (committed_ras_tos_after_port0 == 4'd0);
                    end
                    else if (ras_update_en && ras_update_is_push) begin
                        committed_ras_tos <= committed_ras_tos_next;
                        committed_ras_empty <= 1'b0;
                        speculative_ras_tos <= committed_ras_tos_next;
                        speculative_ras_empty <= 1'b0;
                    end
                    else if (ras_update_en && ras_update_is_pop && !committed_ras_empty) begin
                        committed_ras_tos <= committed_ras_tos_prev;
                        committed_ras_empty <= (committed_ras_tos == 4'd0);
                        speculative_ras_tos <= committed_ras_tos_prev;
                        speculative_ras_empty <= (committed_ras_tos == 4'd0);
                    end
                end

                (ras_update_en || ras_update_en_1): begin
                    // COMMITTED RAS UPDATE - handle both ports in same cycle

                    // Port 0 data write
                    if (ras_update_en && ras_update_is_push)
                        committed_ras[committed_ras_tos_next] <= ras_update_push_addr;

                    // Port 1 data write (uses TOS after port 0 effect)
                    if (ras_update_en_1 && ras_update_is_push_1)
                        committed_ras[committed_ras_tos_after_port0_next] <= ras_update_push_addr_1;

                    // Update committed TOS accounting for both ports
                    if (ras_update_en_1 && ras_update_is_push_1) begin
                        committed_ras_tos <= committed_ras_tos_after_port0_next;
                        committed_ras_empty <= 1'b0;
                    end
                    else if (ras_update_en_1 && ras_update_is_pop_1 && !committed_ras_empty_after_port0) begin
                        logic [3:0] tos_after_both;
                        tos_after_both = (committed_ras_tos_after_port0 == 4'd0) ? 4'd15 : committed_ras_tos_after_port0 - 4'd1;
                        committed_ras_tos <= tos_after_both;
                        committed_ras_empty <= (committed_ras_tos_after_port0 == 4'd0);
                    end
                    else if (ras_update_en && ras_update_is_push) begin
                        committed_ras_tos <= committed_ras_tos_next;
                        committed_ras_empty <= 1'b0;
                    end
                    else if (ras_update_en && ras_update_is_pop && !committed_ras_empty) begin
                        committed_ras_tos <= committed_ras_tos_prev;
                        committed_ras_empty <= (committed_ras_tos == 4'd0);
                    end
                end

                default: begin
                    // SPECULATIVE RAS UPDATE - handle both inst_0 and inst_1 in same cycle

                    // Determine if inst_0 has RAS effect
                    logic inst_0_is_call, inst_0_is_return;
                    inst_0_is_call = fetch_valid && predict_taken && is_call;
                    inst_0_is_return = fetch_valid && predict_taken && is_return && !speculative_ras_empty;

                    // Determine if inst_1 has RAS effect
                    inst_1_is_call = fetch_valid_1 && predict_taken_1 && is_call_1;
                    inst_1_is_return = fetch_valid_1 && predict_taken_1 && is_return_1 && !spec_ras_empty_after_inst0;

                    // Data writes
                    if (inst_0_is_call)
                        speculative_ras[ras_tos_next] <= fetch_pc + 32'd4;
                    if (inst_1_is_call)
                        speculative_ras[spec_ras_tos_after_inst0_next] <= fetch_pc_1 + 32'd4;

                    // TOS update accounting for both instructions
                    if (inst_1_is_call) begin
                        speculative_ras_tos <= spec_ras_tos_after_inst0_next;
                        speculative_ras_empty <= 1'b0;
                    end
                    else if (inst_1_is_return) begin
                        speculative_ras_tos <= spec_ras_tos_after_inst0_prev;
                        speculative_ras_empty <= (spec_ras_tos_after_inst0 == 4'd0);
                    end
                    else if (inst_0_is_call) begin
                        speculative_ras_tos <= ras_tos_next;
                        speculative_ras_empty <= 1'b0;
                    end
                    else if (inst_0_is_return) begin
                        speculative_ras_tos <= ras_tos_prev;
                        speculative_ras_empty <= (speculative_ras_tos == 4'd0);
                    end
                end
            endcase
        end
    end

     
    // SRAM INSTANTIATION
     
    gshare_pht_array pht_sram (
        .clk0(clk),
        .csb0(sram_csb),
        .web0(sram_web),
        .addr0(sram_addr),
        .din0(sram_din),
        .dout0(sram_dout)
    );

endmodule