module load_queue
import types::*;
#(
    parameter NUM_ENTRIES = NUM_LQ_ENTRIES,
    parameter NUM_INFLIGHT = 2
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        flush,

    // dispatch intf (from rename) - port 0
    input  logic        dispatch_valid,
    input  lq_entry_t   dispatch_entry,
    output logic        full,
    output logic        almost_full,

    // dispatch intf - port 1 (superscalar)
    input  logic        dispatch_valid_1,
    input  lq_entry_t   dispatch_entry_1,

    // CDB intf (for wakeup)
    input  cdb_t        cdb [NUM_CDB_PORTS],

    // store queue intf (for memory ordering)
    input  logic [$clog2(NUM_SQ_ENTRIES)-1:0] sq_head,
    input  sq_entry_t   sq_entries [NUM_SQ_ENTRIES],

    // PRF read intf (for address calc)
    output logic [5:0]  prf_src1_preg,
    input  logic [31:0] prf_src1_data,
    output logic        prf_req,
    input  logic        prf_grant,

    // D-Cache intf
    output logic [31:0] dcache_addr,
    output logic [3:0]  dcache_rmask,
    input  logic [31:0] dcache_rdata,
    input  logic        dcache_resp,
    output logic        dcache_req,
    input  logic        dcache_grant,

    // CDB output (load results)
    output cdb_t        load_cdb,
    input  logic        load_cdb_grant,

    // ROB update intf (for RVFI)
    output logic        load_update_mem_info,
    output logic [4:0]  load_update_id,
    output logic [31:0] load_update_addr,
    output logic [3:0]  load_update_rmask,
    output logic [31:0] load_update_rdata,

    output logic [$clog2(NUM_ENTRIES):0] occupancy,

    output logic        dbg_issue_blocked_by_ordering,
    output logic        dbg_issue_blocked_by_unknown_addr,
    output logic        dbg_cache_req_sent,
    output logic        dbg_cache_hit,
    output logic        dbg_fwd_used
);

    // in-flight entry: tracks a load that has been sent to cache
    typedef struct packed {
        logic        valid;
        logic [$clog2(NUM_ENTRIES)-1:0] lq_idx;
        logic [31:0] address;
        mem_op_t     mem_op;
        logic [5:0]  dest_preg;
        logic [4:0]  rob_id;
        logic        waiting;
        logic        complete;
        logic        forwarded;
        logic [31:0] result;
        logic [31:0] raw_data;
    } inflight_entry_t;

    // in-flight FIFO (responses come back in order)
    inflight_entry_t inflight [NUM_INFLIGHT];
    logic [$clog2(NUM_INFLIGHT)-1:0] inflight_head, inflight_tail;
    logic [$clog2(NUM_INFLIGHT)-1:0] inflight_next_head, inflight_next_tail;
    logic inflight_full, inflight_empty;
    logic [$clog2(NUM_INFLIGHT):0] inflight_count;

    // in-flight FIFO ptr stuff
    localparam [$clog2(NUM_INFLIGHT)-1:0] INFLIGHT_MAX = NUM_INFLIGHT - 1;
    assign inflight_next_head = (inflight_head == INFLIGHT_MAX) ? '0 : inflight_head + 1'b1;
    assign inflight_next_tail = (inflight_tail == INFLIGHT_MAX) ? '0 : inflight_tail + 1'b1;

    // count valid in-flight entries
    always_comb begin
        inflight_count = '0;
        for (integer i = 0; i < NUM_INFLIGHT; i++) begin
            if (inflight[i].valid)
                inflight_count = inflight_count + 1'b1;
        end
    end

    assign inflight_full = (inflight_count == NUM_INFLIGHT[$clog2(NUM_INFLIGHT):0]);
    assign inflight_empty = (inflight_count == '0);

    // QUEUE
    lq_entry_t entries [NUM_ENTRIES];

    // queue ptr
    logic [$clog2(NUM_ENTRIES)-1:0] head, tail;
    logic [$clog2(NUM_ENTRIES)-1:0] next_head, next_tail;

    localparam [$clog2(NUM_ENTRIES)-1:0] MAX_INDEX = NUM_ENTRIES - 1;

    assign next_head = (head == MAX_INDEX) ? '0 : head + 1'b1;
    assign next_tail = (tail == MAX_INDEX) ? '0 : tail + 1'b1;

    // Next-next tail for dual dispatch
    logic [$clog2(NUM_ENTRIES)-1:0] next_next_tail;
    assign next_next_tail = (next_tail == MAX_INDEX) ? '0 : next_tail + 1'b1;

    logic empty, queue_valid;
    assign empty = !queue_valid && (head == tail);
    assign full = queue_valid && (head == tail);
    assign almost_full = queue_valid && (next_tail == head);

    function automatic logic [3:0] get_rmask(mem_op_t op, logic [1:0] offset);
        logic [3:0] mask;
        case (op[1:0])
            2'b00: mask = 4'b0001 << offset;    // LB
            2'b01: mask = 4'b0011 << offset;    // LH
            2'b10: mask = 4'b1111;              // LW
            default: mask = 4'b0000;
        endcase
        return mask;
    endfunction

    function automatic logic [31:0] format_load_data(logic [31:0] raw_data, mem_op_t op, logic [1:0] offset);
        logic [31:0] result;
        logic [31:0] shifted_data;

        shifted_data = raw_data >> (offset * 8);

        case (op)
            MEM_LB:  result = {{24{shifted_data[7]}}, shifted_data[7:0]};
            MEM_LBU: result = {24'b0, shifted_data[7:0]};
            MEM_LH:  result = {{16{shifted_data[15]}}, shifted_data[15:0]};
            MEM_LHU: result = {16'b0, shifted_data[15:0]};
            MEM_LW:  result = shifted_data;
            default: result = shifted_data;
        endcase
        return result;
    endfunction

    function automatic logic [31:0] get_raw_ld_data(logic [31:0] raw_data, mem_op_t op, logic [1:0] offset);
        logic [31:0] result;
        logic [3:0] mask;

        mask = get_rmask(op, offset);

        result = 32'h0;
        for (integer i = 0; i < 4; i++) begin
            if (mask[i]) begin
                result[i*8 +: 8] = raw_data[i*8 +: 8];
            end
        end

        return result;
    endfunction

    function automatic logic [3:0] get_wmask(mem_op_t op, logic [1:0] offset);
        logic [3:0] mask;
        case (op[1:0])
            2'b00: mask = 4'b0001 << offset;    // SB
            2'b01: mask = 4'b0011 << offset;    // SH
            2'b10: mask = 4'b1111;              // SW
            default: mask = 4'b0000;
        endcase
        return mask;
    endfunction

    function automatic logic is_rob_id_older(logic [4:0] store_rob_id, logic [4:0] load_rob_id);
        logic [4:0] forward_dist;

        // Calculate forward distance from store to load in circular ROB
        // This is the number of slots from store to load going forward (incrementing IDs)
        if (load_rob_id >= store_rob_id)
            forward_dist = load_rob_id - store_rob_id;
        else
            // Wrap around: distance = (ROB_SIZE - store) + load
            forward_dist = (5'($unsigned(ROB_SIZE)) - store_rob_id) + load_rob_id;

        // If forward distance is 1 to ROB_SIZE/2, store is older (was dispatched before load)
        // If forward distance is ROB_SIZE/2+1 to ROB_SIZE-1 or 0, store is same or newer
        return (forward_dist >= 5'd1) && (forward_dist <= 5'($unsigned(ROB_SIZE/2)));
    endfunction

    function automatic logic is_in_captured_sq_range(logic [$clog2(NUM_SQ_ENTRIES)-1:0] sq_idx, logic [$clog2(NUM_SQ_ENTRIES):0] captured, logic [$clog2(NUM_SQ_ENTRIES)-1:0] current_sq_head
    );
        logic [$clog2(NUM_SQ_ENTRIES)-1:0] captured_tail;
        captured_tail = captured[$clog2(NUM_SQ_ENTRIES)-1:0];

        if (current_sq_head <= captured_tail)
            return (sq_idx >= current_sq_head) && (sq_idx < captured_tail);
        else
            return (sq_idx >= current_sq_head) || (sq_idx < captured_tail);
    endfunction

    // Combined check: store is truly older if it's BOTH in the captured SQ range AND has an older ROB ID
    function automatic logic is_older_store(
        logic [$clog2(NUM_SQ_ENTRIES)-1:0] sq_idx,
        logic [$clog2(NUM_SQ_ENTRIES):0] captured,
        logic [$clog2(NUM_SQ_ENTRIES)-1:0] current_sq_head,
        logic [4:0] store_rob_id,
        logic [4:0] load_rob_id
    );
        // Check both conditions:
        // 1. SQ index is in the captured range (quick filter)
        // 2. Store's ROB ID is actually older than load's ROB ID (prevents stale sq_tail_capture bug)
        return is_in_captured_sq_range(sq_idx, captured, current_sq_head) &&
               is_rob_id_older(store_rob_id, load_rob_id);
    endfunction

    logic issue_pending;
    logic [$clog2(NUM_ENTRIES)-1:0] issue_pending_idx;
    logic issue_pending_forward;
    logic [31:0] issue_pending_fwd_data;
    logic [31:0] issue_pending_address;
    mem_op_t issue_pending_mem_op;
    logic [5:0] issue_pending_dest_preg;
    logic [4:0] issue_pending_rob_id;

    // DISPATCH - port 0
    logic [$clog2(NUM_ENTRIES)-1:0] dispatch_idx;
    logic can_dispatch;

    assign dispatch_idx = tail;
    assign can_dispatch = dispatch_valid && !full;

    // DISPATCH - port 1 (superscalar)
    logic [$clog2(NUM_ENTRIES)-1:0] dispatch_idx_1;
    logic can_dispatch_1;

    assign dispatch_idx_1 = dispatch_valid ? next_tail : tail;

    assign can_dispatch_1 = dispatch_valid_1 && (
        (dispatch_valid && !full && !almost_full) ||  // Both loads: need 2 slots
        (!dispatch_valid && !full)                     // Only inst 1: need 1 slot
    );

    // LOAD ISSUE - find ready load that can be issued
    logic [$clog2(NUM_ENTRIES)-1:0] load_issue_idx;
    logic can_issue_load;
    logic blocked_by_ordering;
    logic blocked_by_unknown_addr;

    logic dcache_pending;
    logic [31:0] dcache_pending_addr;
    logic [3:0] dcache_pending_rmask;
    logic [$clog2(NUM_ENTRIES)-1:0] dcache_pending_lq_idx;
    mem_op_t dcache_pending_mem_op;
    logic [5:0] dcache_pending_dest_preg;
    logic [4:0] dcache_pending_rob_id;

    logic fwd_pending;
    logic [$clog2(NUM_SQ_ENTRIES)-1:0] fwd_sq_idx_r;
    logic [$clog2(NUM_ENTRIES)-1:0] fwd_lq_idx_r;
    logic [31:0] fwd_address_r;
    mem_op_t fwd_mem_op_r;
    logic [5:0] fwd_dest_preg_r;
    logic [4:0] fwd_rob_id_r;

    // which LQ entries are already in-flight
    logic [NUM_ENTRIES-1:0] in_flight_mask;
    always_comb begin
        in_flight_mask = '0;
        for (integer i = 0; i < NUM_INFLIGHT; i++) begin
            if (inflight[i].valid)
                in_flight_mask[inflight[i].lq_idx] = 1'b1;
        end
        if (issue_pending)
            in_flight_mask[issue_pending_idx] = 1'b1;
        if (dcache_pending)
            in_flight_mask[dcache_pending_lq_idx] = 1'b1;
        if (fwd_pending)
            in_flight_mask[fwd_lq_idx_r] = 1'b1;
    end

    always_comb begin
        can_issue_load = 1'b0;
        load_issue_idx = head;
        blocked_by_ordering = 1'b0;
        blocked_by_unknown_addr = 1'b0;

        if (!inflight_full) begin
            for (integer i = 0; i < NUM_ENTRIES; i++) begin
                automatic logic [$clog2(NUM_ENTRIES)-1:0] idx;
                idx = ($clog2(NUM_ENTRIES))'((head + ($clog2(NUM_ENTRIES))'($unsigned(i))) % NUM_ENTRIES);

                if (entries[idx].valid && entries[idx].addr_ready && !entries[idx].executed && !in_flight_mask[idx]) begin

                    automatic logic blocked;
                    automatic logic blocked_unknown;
                    automatic logic blocked_conflict;
                    blocked = 1'b0;
                    blocked_unknown = 1'b0;
                    blocked_conflict = 1'b0;

                    // Iterate from OLDEST to YOUNGEST older store
                    // When we find a forwarding candidate, it "covers" all OLDER unknown stores
                    // (because even if those unknowns match, the forwarding candidate's data is newer)
                    for (integer j = 0; j < NUM_SQ_ENTRIES; j++) begin
                        automatic logic [$clog2(NUM_SQ_ENTRIES)-1:0] sq_idx;
                        sq_idx = ($clog2(NUM_SQ_ENTRIES))'((sq_head + ($clog2(NUM_SQ_ENTRIES))'($unsigned(j))) % NUM_SQ_ENTRIES);

                        if (sq_entries[sq_idx].valid) begin
                            // Pass store and load ROB IDs to verify actual age ordering
                            if (is_older_store(sq_idx, entries[idx].sq_tail_capture, sq_head,
                                               sq_entries[sq_idx].rob_id, entries[idx].rob_id)) begin

                                if (!sq_entries[sq_idx].addr_ready) begin
                                    // Unknown address - potential conflict
                                    blocked_unknown = 1'b1;
                                end
                                else if (!sq_entries[sq_idx].executed && sq_entries[sq_idx].address[31:2] == entries[idx].address[31:2]) begin
                                    // Address matches - check if can forward
                                    if (sq_entries[sq_idx].src2_ready) begin
                                        automatic logic [3:0] ld_mask, st_mask;
                                        ld_mask = get_rmask(entries[idx].mem_op, entries[idx].address[1:0]);
                                        st_mask = get_wmask(sq_entries[sq_idx].mem_op, sq_entries[sq_idx].address[1:0]);

                                        if ((ld_mask & st_mask) == ld_mask) begin
                                            // This store can forward - it covers all OLDER unknown stores
                                            // Reset blocked_unknown since this forwarding candidate is newer
                                            blocked_unknown = 1'b0;
                                        end else begin
                                            // Partial overlap - can't forward, must block
                                            blocked_conflict = 1'b1;
                                        end
                                    end else begin
                                        // Address matches but data not ready - must block
                                        blocked_conflict = 1'b1;
                                    end
                                end
                                // else: different address, no conflict from this store
                            end
                        end
                    end

                    blocked = blocked_unknown || blocked_conflict;

                    if (!blocked) begin
                        can_issue_load = 1'b1;
                        load_issue_idx = idx;
                        break;
                    end
                    else begin
                        if (blocked_unknown)
                            blocked_by_unknown_addr = 1'b1;
                        if (blocked_conflict)
                            blocked_by_ordering = 1'b1;
                    end
                end
            end
        end
    end

    assign dbg_issue_blocked_by_ordering = blocked_by_ordering && !can_issue_load;
    assign dbg_issue_blocked_by_unknown_addr = blocked_by_unknown_addr && !can_issue_load;

    // STORE FORWARDING CHECK
    logic can_forward;
    logic [$clog2(NUM_SQ_ENTRIES)-1:0] forward_sq_idx;
    logic [3:0] issue_load_mask;
    logic forward_from_uncommitted;  // NEW: tracks if forwarding from uncommitted store

    always_comb begin
        can_forward = 1'b0;
        forward_sq_idx = '0;
        issue_load_mask = '0;
        forward_from_uncommitted = 1'b0;

        if (can_issue_load) begin
            issue_load_mask = get_rmask(entries[load_issue_idx].mem_op, entries[load_issue_idx].address[1:0]);

            for (integer j = 0; j < NUM_SQ_ENTRIES; j++) begin

                automatic logic [$clog2(NUM_SQ_ENTRIES)-1:0] sq_idx;

                sq_idx = ($clog2(NUM_SQ_ENTRIES))'((sq_head + ($clog2(NUM_SQ_ENTRIES))'($unsigned(j))) % NUM_SQ_ENTRIES);

                if (sq_entries[sq_idx].valid && sq_entries[sq_idx].addr_ready && sq_entries[sq_idx].src2_ready && !sq_entries[sq_idx].executed &&
                    sq_entries[sq_idx].address[31:2] == entries[load_issue_idx].address[31:2]) begin

                    // Pass store and load ROB IDs to verify actual age ordering
                    if (is_older_store(sq_idx, entries[load_issue_idx].sq_tail_capture, sq_head,
                                       sq_entries[sq_idx].rob_id, entries[load_issue_idx].rob_id)) begin
                        automatic logic [3:0] store_mask;
                        store_mask = 4'b0;
                        case (sq_entries[sq_idx].mem_op[1:0])
                            2'b00: store_mask = 4'b0001 << sq_entries[sq_idx].address[1:0];
                            2'b01: store_mask = 4'b0011 << sq_entries[sq_idx].address[1:0];
                            2'b10: store_mask = 4'b1111;
                            default: store_mask = 4'b0;
                        endcase

                        if ((issue_load_mask & store_mask) == issue_load_mask) begin
                            can_forward = 1'b1;
                            forward_sq_idx = sq_idx;
                            forward_from_uncommitted = !sq_entries[sq_idx].committed;
                        end
                    end
                end
            end
        end
    end

    // OOO DETECTION SIGNALS
    // Count older pending stores when load is about to issue
    logic has_older_pending_stores;
    logic [$clog2(NUM_SQ_ENTRIES):0] num_older_pending_stores;
    logic load_issued_not_at_head;  // Load issued from non-head position (OOO within LQ)

    always_comb begin
        has_older_pending_stores = 1'b0;
        num_older_pending_stores = '0;
        load_issued_not_at_head = 1'b0;

        if (can_issue_load) begin
            // Check if load being issued is NOT at LQ head
            load_issued_not_at_head = (load_issue_idx != head);

            // Count older stores that are still pending (not executed)
            for (integer j = 0; j < NUM_SQ_ENTRIES; j++) begin
                if (sq_entries[j].valid && !sq_entries[j].executed) begin
                    // Pass store and load ROB IDs to verify actual age ordering
                    if (is_older_store(($clog2(NUM_SQ_ENTRIES))'(j), entries[load_issue_idx].sq_tail_capture, sq_head,
                                       sq_entries[j].rob_id, entries[load_issue_idx].rob_id)) begin
                        has_older_pending_stores = 1'b1;
                        num_older_pending_stores = num_older_pending_stores + 1'b1;
                    end
                end
            end
        end
    end

    // PRF ACCESS FOR ADDR CALC
    logic need_prf_for_addr_comb;
    logic [$clog2(NUM_ENTRIES)-1:0] addr_calc_idx_comb;

    logic prf_req_r;
    logic [$clog2(NUM_ENTRIES)-1:0] addr_calc_idx_r;
    logic [31:0] offset_r;
    logic [5:0] src1_preg_r;

    always_comb begin
        need_prf_for_addr_comb = 1'b0;
        addr_calc_idx_comb = head;

        for (integer i = 0; i < NUM_ENTRIES; i++) begin
            automatic logic [$clog2(NUM_ENTRIES)-1:0] idx;
            idx = ($clog2(NUM_ENTRIES))'((head + ($clog2(NUM_ENTRIES))'($unsigned(i))) % NUM_ENTRIES);

            if (entries[idx].valid && !entries[idx].addr_ready && entries[idx].src1_ready) begin
                need_prf_for_addr_comb = 1'b1;
                addr_calc_idx_comb = idx;
                break;
            end
        end
    end

    assign prf_req = prf_req_r;
    assign prf_src1_preg = src1_preg_r;

    // D-CACHE INTF
    logic cache_req_outstanding;
    logic [$clog2(NUM_INFLIGHT)-1:0] cache_req_inflight_idx;

    logic [$clog2(NUM_INFLIGHT):0] waiting_count;
    always_comb begin
        waiting_count = '0;
        for (integer i = 0; i < NUM_INFLIGHT; i++) begin
            if (inflight[i].valid && inflight[i].waiting)
                waiting_count = waiting_count + 1'b1;
        end
    end

    logic should_start_cache_req;
    assign should_start_cache_req = can_issue_load && !can_forward && !cache_req_outstanding && !inflight_full && !issue_pending && !dcache_pending;

    assign dcache_req = dcache_pending;

    always_comb begin
        if (dcache_pending) begin
            dcache_addr = {dcache_pending_addr[31:2], 2'b00};
            dcache_rmask = dcache_pending_rmask;
        end
        else if (cache_req_outstanding) begin
            dcache_addr = {inflight[cache_req_inflight_idx].address[31:2], 2'b00};
            dcache_rmask = get_rmask(inflight[cache_req_inflight_idx].mem_op, inflight[cache_req_inflight_idx].address[1:0]);
        end
        else begin
            dcache_addr = '0;
            dcache_rmask = '0;
        end
    end

    assign dbg_cache_req_sent = dcache_req && dcache_grant;
    assign dbg_cache_hit = dcache_resp && cache_req_outstanding;
    assign dbg_fwd_used = can_issue_load && can_forward && !inflight_full;

    // WRITEBACK SELECT
    logic writeback_valid;
    logic [$clog2(NUM_INFLIGHT)-1:0] writeback_idx;

    always_comb begin
        writeback_valid = 1'b0;
        writeback_idx = '0;

        for (integer i = 0; i < NUM_INFLIGHT; i++) begin
            automatic logic [$clog2(NUM_INFLIGHT)-1:0] idx;

            idx = ($clog2(NUM_INFLIGHT))'((inflight_head + ($clog2(NUM_INFLIGHT))'($unsigned(i))) % NUM_INFLIGHT);

            if (inflight[idx].valid && inflight[idx].complete) begin
                writeback_valid = 1'b1;
                writeback_idx = idx;
                break;
            end
        end
    end

    // CDB OUTPUT
    always_comb begin
        if (writeback_valid) begin
            load_cdb.valid = 1'b1;
            load_cdb.data = inflight[writeback_idx].result;
            load_cdb.preg = inflight[writeback_idx].dest_preg;
            load_cdb.rob_id = inflight[writeback_idx].rob_id;
        end
        else begin
            load_cdb.valid = 1'b0;
            load_cdb.data = 32'h0;
            load_cdb.preg = 6'b0;
            load_cdb.rob_id = 5'b0;
        end
    end

    // ROB UPDATE OUTPUT
    logic rob_update_pending_fwd;
    logic [$clog2(NUM_INFLIGHT)-1:0] rob_update_idx_fwd;
    logic rob_update_pending_cache;
    logic [$clog2(NUM_INFLIGHT)-1:0] rob_update_idx_cache;

    always_comb begin
        load_update_mem_info = 1'b0;
        load_update_id = '0;
        load_update_addr = '0;
        load_update_rmask = '0;
        load_update_rdata = '0;

        if (rob_update_pending_cache) begin
            load_update_mem_info = 1'b1;
            load_update_id = inflight[rob_update_idx_cache].rob_id;
            load_update_addr = {inflight[rob_update_idx_cache].address[31:2], 2'b00};
            load_update_rmask = get_rmask(inflight[rob_update_idx_cache].mem_op, inflight[rob_update_idx_cache].address[1:0]);
            load_update_rdata = inflight[rob_update_idx_cache].raw_data;
        end
        else if (rob_update_pending_fwd) begin
            load_update_mem_info = 1'b1;
            load_update_id = inflight[rob_update_idx_fwd].rob_id;
            load_update_addr = {inflight[rob_update_idx_fwd].address[31:2], 2'b00};
            load_update_rmask = get_rmask(inflight[rob_update_idx_fwd].mem_op, inflight[rob_update_idx_fwd].address[1:0]);
            load_update_rdata = inflight[rob_update_idx_fwd].raw_data;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            head <= '0;
            tail <= '0;
            queue_valid <= 1'b0;
            inflight_head <= '0;
            inflight_tail <= '0;

            prf_req_r <= 1'b0;
            addr_calc_idx_r <= '0;
            offset_r <= '0;
            src1_preg_r <= '0;

            rob_update_pending_fwd <= 1'b0;
            rob_update_idx_fwd <= '0;
            rob_update_pending_cache <= 1'b0;
            rob_update_idx_cache <= '0;

            cache_req_outstanding <= 1'b0;
            cache_req_inflight_idx <= '0;

            issue_pending <= 1'b0;
            issue_pending_idx <= '0;
            issue_pending_forward <= 1'b0;
            issue_pending_fwd_data <= '0;
            issue_pending_address <= '0;
            issue_pending_mem_op <= MEM_LW;
            issue_pending_dest_preg <= '0;
            issue_pending_rob_id <= '0;

            dcache_pending <= 1'b0;
            dcache_pending_addr <= '0;
            dcache_pending_rmask <= '0;
            dcache_pending_lq_idx <= '0;
            dcache_pending_mem_op <= MEM_LW;
            dcache_pending_dest_preg <= '0;
            dcache_pending_rob_id <= '0;

            fwd_pending <= 1'b0;
            fwd_sq_idx_r <= '0;
            fwd_lq_idx_r <= '0;
            fwd_address_r <= '0;
            fwd_mem_op_r <= MEM_LW;
            fwd_dest_preg_r <= '0;
            fwd_rob_id_r <= '0;

            for (integer i = 0; i < NUM_ENTRIES; i++) begin
                entries[i].valid <= 1'b0;
            end
            for (integer i = 0; i < NUM_INFLIGHT; i++) begin
                inflight[i].valid <= 1'b0;
            end

        end else if (flush) begin
            head <= '0;
            tail <= '0;
            queue_valid <= 1'b0;
            inflight_head <= '0;
            inflight_tail <= '0;

            prf_req_r <= 1'b0;
            rob_update_pending_fwd <= 1'b0;
            rob_update_pending_cache <= 1'b0;

            cache_req_outstanding <= 1'b0;
            cache_req_inflight_idx <= '0;
            fwd_pending <= 1'b0;
            issue_pending <= 1'b0;
            dcache_pending <= 1'b0;
            for (integer i = 0; i < NUM_ENTRIES; i++) begin
                entries[i].valid <= 1'b0;
            end
            for (integer i = 0; i < NUM_INFLIGHT; i++) begin
                inflight[i].valid <= 1'b0;
            end

        end else begin
            // DISPATCH: add new ld to queue
            if (can_dispatch && can_dispatch_1) begin
                // Dual dispatch
                entries[dispatch_idx] <= dispatch_entry;
                entries[dispatch_idx_1] <= dispatch_entry_1;
                tail <= next_next_tail;

                if (next_next_tail == head || next_tail == head)
                    queue_valid <= 1'b1;
            end
            else if (can_dispatch) begin
                // Single dispatch - only inst 0
                entries[dispatch_idx] <= dispatch_entry;
                tail <= next_tail;

                if (next_tail == head)
                    queue_valid <= 1'b1;
            end
            else if (can_dispatch_1) begin
                entries[dispatch_idx_1] <= dispatch_entry_1;
                tail <= next_tail;

                if (next_tail == head)
                    queue_valid <= 1'b1;
            end

            // CDB WAKEUP (with rob_id validation to prevent stale wakeups)
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb[i].valid) begin
                    for (integer j = 0; j < NUM_ENTRIES; j++) begin
                        if (entries[j].valid && !entries[j].src1_ready) begin
                            // Must match both preg AND rob_id to prevent stale CDB wakeups
                            if (entries[j].src1_preg == cdb[i].preg &&
                                cdb[i].preg != 6'd0 &&
                                entries[j].src1_rob_id == cdb[i].rob_id)
                                entries[j].src1_ready <= 1'b1;
                        end
                    end
                end
            end

            // PRF REQUEST
            if (prf_grant && prf_req_r)
                prf_req_r <= 1'b0;
            else if (need_prf_for_addr_comb && !prf_req_r) begin
                prf_req_r <= 1'b1;
                addr_calc_idx_r <= addr_calc_idx_comb;
                offset_r <= entries[addr_calc_idx_comb].offset;
                src1_preg_r <= entries[addr_calc_idx_comb].src1_preg;
            end

            // ADDRESS CALC
            if (prf_req_r && prf_grant) begin
                if (entries[addr_calc_idx_r].valid && !entries[addr_calc_idx_r].addr_ready) begin
                    
                    automatic logic [31:0] calc_addr;
                    calc_addr = prf_src1_data + offset_r;
                    entries[addr_calc_idx_r].address <= calc_addr;
                    entries[addr_calc_idx_r].addr_ready <= 1'b1;
                end
            end

            if (can_issue_load && !inflight_full && !issue_pending && !fwd_pending) begin
                if (can_forward) begin
                    fwd_pending <= 1'b1;
                    fwd_sq_idx_r <= forward_sq_idx;
                    fwd_lq_idx_r <= load_issue_idx;
                    fwd_address_r <= entries[load_issue_idx].address;
                    fwd_mem_op_r <= entries[load_issue_idx].mem_op;
                    fwd_dest_preg_r <= entries[load_issue_idx].dest_preg;
                    fwd_rob_id_r <= entries[load_issue_idx].rob_id;

                end else if (should_start_cache_req) begin
                    dcache_pending <= 1'b1;
                    dcache_pending_addr <= entries[load_issue_idx].address;
                    dcache_pending_rmask <= get_rmask(entries[load_issue_idx].mem_op, entries[load_issue_idx].address[1:0]);
                    dcache_pending_lq_idx <= load_issue_idx;
                    dcache_pending_mem_op <= entries[load_issue_idx].mem_op;
                    dcache_pending_dest_preg <= entries[load_issue_idx].dest_preg;
                    dcache_pending_rob_id <= entries[load_issue_idx].rob_id;
                end
            end

            if (fwd_pending && !issue_pending) begin
                fwd_pending <= 1'b0;
                issue_pending <= 1'b1;
                issue_pending_idx <= fwd_lq_idx_r;
                issue_pending_forward <= 1'b1;
                issue_pending_fwd_data <= sq_entries[fwd_sq_idx_r].store_data;
                issue_pending_address <= fwd_address_r;
                issue_pending_mem_op <= fwd_mem_op_r;
                issue_pending_dest_preg <= fwd_dest_preg_r;
                issue_pending_rob_id <= fwd_rob_id_r;
            end

            if (dcache_pending && dcache_grant) begin
                dcache_pending <= 1'b0;

                inflight[inflight_tail].valid <= 1'b1;
                inflight[inflight_tail].lq_idx <= dcache_pending_lq_idx;
                inflight[inflight_tail].address <= dcache_pending_addr;
                inflight[inflight_tail].mem_op <= dcache_pending_mem_op;
                inflight[inflight_tail].dest_preg <= dcache_pending_dest_preg;
                inflight[inflight_tail].rob_id <= dcache_pending_rob_id;
                inflight[inflight_tail].waiting <= 1'b1;
                inflight[inflight_tail].complete <= 1'b0;
                inflight[inflight_tail].forwarded <= 1'b0;
                inflight[inflight_tail].result <= '0;
                inflight[inflight_tail].raw_data <= '0;

                inflight_tail <= inflight_next_tail;

                cache_req_outstanding <= 1'b1;
                cache_req_inflight_idx <= inflight_tail;
            end

            if (issue_pending && issue_pending_forward && !inflight_full) begin

                inflight[inflight_tail].valid <= 1'b1;
                inflight[inflight_tail].lq_idx <= issue_pending_idx;
                inflight[inflight_tail].address <= issue_pending_address;
                inflight[inflight_tail].mem_op <= issue_pending_mem_op;
                inflight[inflight_tail].dest_preg <= issue_pending_dest_preg;
                inflight[inflight_tail].rob_id <= issue_pending_rob_id;
                inflight[inflight_tail].waiting <= 1'b0;
                inflight[inflight_tail].complete <= 1'b1;
                inflight[inflight_tail].forwarded <= 1'b1;
                inflight[inflight_tail].result <= format_load_data(issue_pending_fwd_data, issue_pending_mem_op, issue_pending_address[1:0]);
                inflight[inflight_tail].raw_data <= get_raw_ld_data(issue_pending_fwd_data, issue_pending_mem_op, issue_pending_address[1:0]);

                inflight_tail <= inflight_next_tail;

                rob_update_pending_fwd <= 1'b1;
                rob_update_idx_fwd <= inflight_tail;

                issue_pending <= 1'b0;
            end

            if (dcache_resp && cache_req_outstanding) begin

                cache_req_outstanding <= 1'b0;

                for (integer i = 0; i < NUM_INFLIGHT; i++) begin
                    automatic logic [$clog2(NUM_INFLIGHT)-1:0] idx;

                    idx = ($clog2(NUM_INFLIGHT))'((inflight_head + ($clog2(NUM_INFLIGHT))'($unsigned(i))) % NUM_INFLIGHT);

                    if (inflight[idx].valid && inflight[idx].waiting) begin
                        inflight[idx].waiting <= 1'b0;
                        inflight[idx].complete <= 1'b1;
                        inflight[idx].result <= format_load_data(dcache_rdata, inflight[idx].mem_op, inflight[idx].address[1:0]);
                        inflight[idx].raw_data <= get_raw_ld_data(dcache_rdata, inflight[idx].mem_op, inflight[idx].address[1:0]);

                        rob_update_pending_cache <= 1'b1;
                        rob_update_idx_cache <= idx;
                        break;
                    end
                end
            end

            if (rob_update_pending_cache)
                rob_update_pending_cache <= 1'b0;
            else if (rob_update_pending_fwd)
                rob_update_pending_fwd <= 1'b0;

            if (writeback_valid && load_cdb_grant) begin

                entries[inflight[writeback_idx].lq_idx].executed <= 1'b1;
                inflight[writeback_idx].valid <= 1'b0;

                if (writeback_idx == inflight_head)
                    inflight_head <= inflight_next_head;
            end

            // DEQUEUE: remove executed ld from LQ
            if (!empty && entries[head].valid && entries[head].executed) begin
                entries[head].valid <= 1'b0;
                head <= next_head;

                if (next_head == tail)
                    queue_valid <= 1'b0;
            end

            if (!empty && !entries[head].valid) begin
                head <= next_head;
                if (next_head == tail)
                    queue_valid <= 1'b0;
            end
        end
    end

    always_comb begin
        automatic logic [$clog2(NUM_ENTRIES):0] count;
        count = '0;
        for (integer i = 0; i < NUM_ENTRIES; i++) begin
            if (entries[i].valid)
                count = count + ($clog2(NUM_ENTRIES)+1)'(1);
        end
        occupancy = count;
    end

endmodule : load_queue