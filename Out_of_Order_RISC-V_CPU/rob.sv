// REORDER BUFFER (ROB)
//
// Maintains in-flight insts in program order
// - Size: parametrizable
// - Enqueue: add new inst from rename stage
// - Dequeue: commit inst at head when ready
// - Tracks: PC, inst, operands, result, ready status
//
// 1. circular buff with head/tail ptr
// 2. inst enter at tail (dispatch)
// 3. inst leave from head (commit)
// 4. head commits ONLY when ready is set
// 5. stores RVFI data captured at dispatch


// COMBINATIONAL OR SYNCHRONOUS ???????????????????

module rob 
import types::*;
#(
    parameter SIZE = 32
)(
    input  logic        clk,
    input  logic        rst,
    
    // Dispatch - port 0
    input  logic        enq,
    input  rob_entry_t  enq_data,
    output logic [4:0]  rob_idx,
    output logic        full,

    // Dispatch - port 1 (dual enqueue for superscalar)
    input  logic        enq_1,
    input  rob_entry_t  enq_data_1,
    output logic [4:0]  rob_idx_1,
    output logic        almost_full,      // Only 1 slot left (can't dual enqueue)
    
    input  cdb_t        cdb [NUM_CDB_PORTS],
    
    // commit - port 0 (head)
    output rob_entry_t  head_data,
    output logic        head_valid,
    output logic        head_ready,
    input  logic        commit,

    // commit - port 1 (head+1) for dual commit
    output rob_entry_t  head_data_1,
    output logic        head_valid_1,
    output logic        head_ready_1,
    input  logic        commit_1,

    // status
    output logic        empty,

    // For performance monitoring
    output logic [4:0]  head_ptr,
    output logic [4:0]  tail_ptr,

    output logic        branch_mispredict,
    output logic [31:0] branch_correct_target,

    // NEW: Signal to commit.sv to block dual commit when HEAD mispredicts
    output logic        head_branch_mispredict,

    output logic [4:0]  flush_free_count,       // no. regs to return
    output logic [5:0]  flush_free_pregs [SIZE],  // which pregs to return
    output logic        flush_occurred,         // pulse when flush

    output logic        flush_this_cycle,

    // load update port
    input  logic        load_update_mem_info,
    input  logic [4:0]  load_update_id,
    input  logic [31:0] load_update_addr,
    input  logic [3:0]  load_update_rmask,
    input  logic [31:0] load_update_rdata,

    // store update port
    input  logic        store_update_mem_info,
    input  logic [4:0]  store_update_id,
    input  logic [31:0] store_update_addr,
    input  logic [3:0]  store_update_wmask,
    input  logic [31:0] store_update_wdata,
    input  logic        store_ready
);

    rob_entry_t storage [SIZE];
    logic [4:0] head, tail;
    logic [4:0] next_head, next_tail;

    // Use SIZE-1 for wrap-around instead of hardcoded 15
    localparam logic [4:0] MAX_IDX = 5'(SIZE - 1);

    assign next_head = (head == MAX_IDX) ? 5'd0 : head + 5'd1;
    assign next_tail = (tail == MAX_IDX) ? 5'd0 : tail + 5'd1;

    // head+2 for when both commit
    logic [4:0] next_next_head;
    assign next_next_head = (next_head == MAX_IDX) ? 5'd0 : next_head + 5'd1;

    // tail+2 for when both enqueue (dual dispatch)
    logic [4:0] next_next_tail;
    assign next_next_tail = (next_tail == MAX_IDX) ? 5'd0 : next_tail + 5'd1;

    assign empty = (head == tail) && !storage[head].valid;
    assign full  = (head == tail) && storage[head].valid;

    // almost_full: only 1 slot left (can enqueue 1 but not 2)
    assign almost_full = (next_tail == head) && storage[head].valid;

    // port 0: head - with CDB forwarding for RVFI correctness
    always_comb begin
        head_data = storage[head];

        for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
            if (cdb[i].valid) begin
                if (!storage[head].rs1_valid && storage[head].rs1_preg == cdb[i].preg && cdb[i].preg != 6'd0 &&
                    storage[head].rs1_rob_id == cdb[i].rob_id) begin
                    head_data.rs1_data = cdb[i].data;
                    head_data.rs1_valid = 1'b1;
                end
                if (!storage[head].rs2_valid && storage[head].rs2_preg == cdb[i].preg && cdb[i].preg != 6'd0 &&
                    storage[head].rs2_rob_id == cdb[i].rob_id) begin
                    head_data.rs2_data = cdb[i].data;
                    head_data.rs2_valid = 1'b1;
                end
            end
        end
    end
    assign head_valid = storage[head].valid;
    assign head_ready = storage[head].ready;

    // port 1: head+1 (for dual commit) - with CDB forwarding for RVFI correctness
    always_comb begin
        head_data_1 = storage[next_head];

        for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
            if (cdb[i].valid) begin
                if (!storage[next_head].rs1_valid && storage[next_head].rs1_preg == cdb[i].preg && cdb[i].preg != 6'd0 &&
                    storage[next_head].rs1_rob_id == cdb[i].rob_id) begin
                    head_data_1.rs1_data = cdb[i].data;
                    head_data_1.rs1_valid = 1'b1;
                end
                if (!storage[next_head].rs2_valid && storage[next_head].rs2_preg == cdb[i].preg && cdb[i].preg != 6'd0 &&
                    storage[next_head].rs2_rob_id == cdb[i].rob_id) begin
                    head_data_1.rs2_data = cdb[i].data;
                    head_data_1.rs2_valid = 1'b1;
                end
            end
        end
    end
    assign head_valid_1 = storage[next_head].valid;
    assign head_ready_1 = storage[next_head].ready;

    // Return ROB indices for both dispatch ports
    assign rob_idx = tail;
    assign rob_idx_1 = next_tail;

    // Performance monitoring outputs
    assign head_ptr = head;
    assign tail_ptr = tail;

    logic branch_mispredict_detected;
    logic [31:0] branch_correct_target_calc;

    // Misprediction detection for inst_1 (next_head) when dual committing
    logic branch_mispredict_1_detected;
    logic [31:0] branch_correct_target_1_calc;

    logic do_enq, do_enq_1, do_commit, do_commit_1;

    // Flush happens if:
    // 1. Head is a branch and mispredicts (single commit)
    // 2. Head+1 is a branch and mispredicts during dual commit
    assign flush_this_cycle = (do_commit && storage[head].is_branch && branch_mispredict_detected) ||
                              (do_commit_1 && storage[next_head].is_branch && branch_mispredict_1_detected);

    // Port 0 enqueue: can enqueue if not full and no flush
    assign do_enq    = enq && !full && !flush_this_cycle;
    // Port 1 enqueue: can dual enqueue if port 0 enqueues AND not almost_full
    assign do_enq_1  = enq_1 && do_enq && !almost_full;

    assign do_commit = commit && head_valid && head_ready;
    assign do_commit_1 = commit_1 && do_commit &&
                         !(storage[head].is_branch && branch_mispredict_detected) &&
                         head_valid_1 && head_ready_1;

    // NEW: Export misprediction status to commit.sv so it can block RVFI output for inst_1
    assign head_branch_mispredict = storage[head].is_branch && branch_mispredict_detected;

    // branch misprediction detection
    always_comb begin
        branch_mispredict_detected = 1'b0;
        branch_correct_target_calc = 32'h0;
        
        if (do_commit && storage[head].is_branch && storage[head].ready) begin

            logic actual_taken;
            logic [31:0] branch_result;
            logic [31:0] actual_jump_target;
            
            // take result from storage or forward from CDB
            branch_result = storage[head].result;

            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb[i].valid && cdb[i].rob_id == head[4:0])
                    branch_result = (cdb[i].data);
            end
            
            if (storage[head].inst_type == ITYPE_JALR) begin
                logic signed [31:0] jalr_imm;

                jalr_imm = $signed({{20{storage[head].inst[31]}}, storage[head].inst[31:20]});

                actual_jump_target = (head_data.rs1_data + $unsigned(32'(jalr_imm))) & ~32'h1;

                // compare actual target vs predicted target
                if (actual_jump_target != storage[head].branch_target) begin
                    branch_mispredict_detected = 1'b1;
                    branch_correct_target_calc = actual_jump_target;
                end
            end
            else if (storage[head].inst_type == ITYPE_JAL) begin
                if (!storage[head].predicted_taken) begin
                    branch_mispredict_detected = 1'b1;
                    // Use the correct target from decode (already in branch_target)
                    branch_correct_target_calc = storage[head].branch_target;
                end
            end
            else begin
                // Conditional BRANCH: check taken pos and target
                actual_taken = branch_result[0];
                actual_jump_target = {branch_result[31:1], 1'b0};
                
                if (actual_taken != storage[head].predicted_taken) begin
                    branch_mispredict_detected = 1'b1;

                    if (actual_taken)
                        branch_correct_target_calc = actual_jump_target;
                    else
                        branch_correct_target_calc = storage[head].pc + 32'd4;
                end
            end
        end
    end

     
    // INST_1 MISPREDICTION DETECTION (for dual commit with branch in position 1)
     
    // When dual committing and inst_1 (next_head) is a branch, we need to detect
    // its misprediction in the same cycle. If it mispredicts, we commit both
    // head and next_head, then flush from next_next_head onwards.
    always_comb begin
        branch_mispredict_1_detected = 1'b0;
        branch_correct_target_1_calc = 32'h0;

        // Only check when we're dual committing AND inst_1 is a branch
        if (do_commit_1 && storage[next_head].is_branch && storage[next_head].ready) begin

            logic actual_taken_1;
            logic [31:0] branch_result_1;
            logic [31:0] actual_jump_target_1;

            // Get result from storage or forward from CDB
            branch_result_1 = storage[next_head].result;

            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb[i].valid && cdb[i].rob_id == next_head[4:0])
                    branch_result_1 = cdb[i].data;
            end

            if (storage[next_head].inst_type == ITYPE_JALR) begin
                logic signed [31:0] jalr_imm_1;

                jalr_imm_1 = $signed({{20{storage[next_head].inst[31]}}, storage[next_head].inst[31:20]});

                actual_jump_target_1 = (head_data_1.rs1_data + $unsigned(32'(jalr_imm_1))) & ~32'h1;

                // compare actual target vs predicted target
                if (actual_jump_target_1 != storage[next_head].branch_target) begin
                    branch_mispredict_1_detected = 1'b1;
                    branch_correct_target_1_calc = actual_jump_target_1;
                end
            end
            else if (storage[next_head].inst_type == ITYPE_JAL) begin
                // JAL: can mispredict if predictor didn't handle it
                if (!storage[next_head].predicted_taken) begin
                    branch_mispredict_1_detected = 1'b1;
                    branch_correct_target_1_calc = storage[next_head].branch_target;
                end
            end
            else begin
                // Conditional BRANCH: check taken/not-taken and target
                actual_taken_1 = branch_result_1[0];
                actual_jump_target_1 = {branch_result_1[31:1], 1'b0};

                if (actual_taken_1 != storage[next_head].predicted_taken) begin
                    branch_mispredict_1_detected = 1'b1;

                    if (actual_taken_1)
                        branch_correct_target_1_calc = actual_jump_target_1;
                    else
                        branch_correct_target_1_calc = storage[next_head].pc + 32'd4;
                end
            end
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            branch_mispredict <= 1'b0;
            branch_correct_target <= 32'h0;
        end
        else if (do_commit && storage[head].is_branch && branch_mispredict_detected) begin
            // Head mispredicted (single commit)
            branch_mispredict <= 1'b1;
            branch_correct_target <= branch_correct_target_calc;
        end
        else if (do_commit_1 && storage[next_head].is_branch && branch_mispredict_1_detected) begin
            // Inst_1 mispredicted during dual commit
            branch_mispredict <= 1'b1;
            branch_correct_target <= branch_correct_target_1_calc;
        end
        else
            branch_mispredict <= 1'b0;
    end

    logic [4:0] free_count;
    logic [63:0] preg_already_collected;
    
    always_ff @(posedge clk) begin
        if (rst) begin
            head <= '0;
            tail <= '0;
            // POWER OPT: Only clear valid bits, don't toggle all data bits
            for (integer i = 0; i < SIZE; i++) begin
                storage[i].valid <= 1'b0;
            end
            flush_free_count <= 5'd0;
            flush_occurred <= 1'b0;

            for (integer i = 0; i < SIZE; i++) begin
                flush_free_pregs[i] <= 6'd0;
            end
        end
        else begin
            // Dual enqueue logic for superscalar
            if (do_enq && do_enq_1) begin
                // Dual enqueue: write both entries, advance tail by 2
                storage[tail] <= enq_data;
                storage[next_tail] <= enq_data_1;
                tail <= next_next_tail;
            end
            else if (do_enq) begin
                // Single enqueue: write entry, advance tail by 1
                storage[tail] <= enq_data;
                tail <= next_tail;
            end
            
            if (do_commit) begin
                if (storage[head].is_branch && branch_mispredict_detected) begin
                     
                    // CASE 1: HEAD is branch and mispredicts (single commit)
                     
                    free_count = 5'd0;

                    for (integer j = 0; j < SIZE; j++) begin
                        flush_free_pregs[j] <= 6'd0;
                    end

                    // Track which pregs have already been collected to prevent duplicates
                    // (duplicates can cause double-allocation bugs in free list)
                    preg_already_collected = 64'b0;

                    // collect registers from ALL entries that will be invalidated
                    for (integer unsigned i = 0; i < SIZE; i++) begin
                        if (storage[i].valid) begin
                            automatic logic [4:0] i_unsigned;
                            automatic logic [5:0] preg_to_free;
                            i_unsigned = 5'(i);
                            preg_to_free = storage[i].dest_preg;

                            if (i_unsigned != head[4:0] && storage[i].dest_areg != 5'b0 && preg_to_free != 6'd0) begin
                                // Only add if not already collected (prevent duplicates)
                                if (!preg_already_collected[preg_to_free]) begin
                                    flush_free_pregs[free_count] <= preg_to_free;
                                    free_count = free_count + 5'd1;
                                    preg_already_collected[preg_to_free] = 1'b1;
                                end
                            end
                        end
                        // invalidate after collecting
                        storage[i].valid <= 1'b0;
                    end

                    flush_free_count <= free_count;
                    flush_occurred <= 1'b1;
                    tail <= next_head;
                    head <= next_head;
                end
                else if (do_commit_1 && storage[next_head].is_branch && branch_mispredict_1_detected) begin
                     
                    // CASE 2: Dual commit where INST_1 (next_head) is branch and mispredicts
                    // Both head and next_head commit successfully, flush from next_next_head onwards
                     
                    free_count = 5'd0;

                    for (integer j = 0; j < SIZE; j++) begin
                        flush_free_pregs[j] <= 6'd0;
                    end

                    preg_already_collected = 64'b0;

                    // collect registers from ALL entries EXCEPT head AND next_head (both commit)
                    for (integer unsigned i = 0; i < SIZE; i++) begin
                        if (storage[i].valid) begin
                            automatic logic [4:0] i_unsigned;
                            automatic logic [5:0] preg_to_free;
                            i_unsigned = 5'(i);
                            preg_to_free = storage[i].dest_preg;

                            // Skip both head and next_head - they both commit successfully
                            if (i_unsigned != head[4:0] && i_unsigned != next_head[4:0] &&
                                storage[i].dest_areg != 5'b0 && preg_to_free != 6'd0) begin
                                if (!preg_already_collected[preg_to_free]) begin
                                    flush_free_pregs[free_count] <= preg_to_free;
                                    free_count = free_count + 5'd1;
                                    preg_already_collected[preg_to_free] = 1'b1;
                                end
                            end
                        end
                        // invalidate after collecting
                        storage[i].valid <= 1'b0;
                    end

                    flush_free_count <= free_count;
                    flush_occurred <= 1'b1;
                    // Both head and next_head committed, so new head/tail is next_next_head
                    tail <= next_next_head;
                    head <= next_next_head;
                end
                else begin
                     
                    // CASE 3: Normal commit (no misprediction)
                     
                    storage[head].valid <= 1'b0;

                    if (do_commit_1) begin
                        // dual commit: invalidate both, advance head by 2
                        storage[next_head].valid <= 1'b0;
                        head <= next_next_head;
                    end
                    else begin
                        // single commit: advance head by 1
                        head <= next_head;
                    end

                    flush_free_count <= 5'd0;
                    flush_occurred <= 1'b0;
                end
            end
            else begin
                // no commit
                flush_free_count <= 5'd0;
                flush_occurred <= 1'b0;
            end
            
            // CDB updates
            // IMPORTANT: Skip CDB updates for entries being newly enqueued this cycle.
            // When we enqueue a new entry, it has fresh rs1_data/rs2_data from rename.
            // If we don't skip, CDB updates might check OLD entry's rs2_preg (before the
            // enqueue), match it, and overwrite the newly enqueued rs2_data with wrong value.
            // This is because non-blocking assignments all evaluate RHS using OLD values,
            // and if both enqueue and CDB update write to the same field, the later one
            // in the code (CDB update) wins.
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb[i].valid) begin
                    automatic logic cdb_source_valid;
                    cdb_source_valid = storage[cdb[i].rob_id].valid &&
                                       (storage[cdb[i].rob_id].dest_preg == cdb[i].preg);

                    // Skip ready/result update for newly enqueued entries
                    if (!(do_enq && cdb[i].rob_id == tail) && !(do_enq_1 && cdb[i].rob_id == next_tail)) begin
                        if (cdb_source_valid && !storage[cdb[i].rob_id].ready) begin
                            storage[cdb[i].rob_id].ready  <= 1'b1;
                            storage[cdb[i].rob_id].result <= cdb[i].data;

                            if (storage[cdb[i].rob_id].inst_type == ITYPE_BRANCH)
                                storage[cdb[i].rob_id].branch_target <= {cdb[i].data[31:1], 1'b0};
                        end
                    end

                    // Only process rs1/rs2 updates if the CDB source is valid
                    if (cdb_source_valid) begin
                        for (integer j = 0; j < SIZE; j++) begin
                            // Skip entries being newly enqueued this cycle - they have fresh data from rename
                            if (do_enq && 5'($unsigned(j)) == tail)
                                ; // skip - newly enqueued slot
                            else if (do_enq_1 && 5'($unsigned(j)) == next_tail)
                                ; // skip - newly enqueued slot
                            else if (storage[j].valid) begin
                                if (!storage[j].rs1_valid && storage[j].rs1_preg == cdb[i].preg && cdb[i].preg != 6'd0) begin
                                    if (storage[j].rs1_rob_id == cdb[i].rob_id) begin
                                        // Valid CDB broadcast from correct producer
                                        storage[j].rs1_data <= cdb[i].data;
                                        storage[j].rs1_valid <= 1'b1;
                                    end
                                end

                                // RS2 update with rob_id validation
                                if (!storage[j].rs2_valid && storage[j].rs2_preg == cdb[i].preg && cdb[i].preg != 6'd0) begin
                                    if (storage[j].rs2_rob_id == cdb[i].rob_id) begin
                                        // Valid CDB broadcast from correct producer
                                        storage[j].rs2_data <= cdb[i].data;
                                        storage[j].rs2_valid <= 1'b1;
                                    end
                                end
                            end
                        end
                    end
                end
            end
            
            // load update port
            if (load_update_mem_info) begin
                storage[load_update_id].mem_addr   <= load_update_addr;
                storage[load_update_id].mem_rmask  <= load_update_rmask;
                storage[load_update_id].mem_wmask  <= 4'b0;
                storage[load_update_id].mem_wdata  <= load_update_rdata;
            end

            // store update port
            if (store_update_mem_info) begin
                storage[store_update_id].mem_addr   <= store_update_addr;
                storage[store_update_id].mem_rmask  <= 4'b0;
                storage[store_update_id].mem_wmask  <= store_update_wmask;
                storage[store_update_id].mem_wdata  <= store_update_wdata;

                // store ready when LSQ signals ready
                if (store_ready) begin
                    storage[store_update_id].ready <= 1'b1;
                end
            end
        end
    end

endmodule : rob