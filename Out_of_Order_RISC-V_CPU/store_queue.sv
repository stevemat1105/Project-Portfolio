// STORE QUEUE MODULE (Split LSQ)
// Handles store operations with independent FSM and performance counters

module store_queue
import types::*;
#(
    parameter NUM_ENTRIES = NUM_SQ_ENTRIES
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        flush,

    // dispatch intf (from rename) - port 0
    input  logic        dispatch_valid,
    input  sq_entry_t   dispatch_entry,
    output logic        full,
    output logic        almost_full,

    // dispatch intf - port 1 (superscalar)
    input  logic        dispatch_valid_1,
    input  sq_entry_t   dispatch_entry_1,

    // CDB intf (for wakeup)
    input  cdb_t        cdb [NUM_CDB_PORTS],

    // ROB commit intf - port 0
    input  logic        rob_commit_valid,
    input  logic [4:0]  rob_commit_id,
    input  logic        rob_commit_is_store,

    // ROB commit intf - port 1 (dual commit)
    input  logic        rob_commit_valid_1,
    input  logic [4:0]  rob_commit_id_1,
    input  logic        rob_commit_is_store_1,

    // PRF read intf (for addr calc)
    output logic [5:0]  prf_src1_preg,
    input  logic [31:0] prf_src1_data,
    output logic [5:0]  prf_src2_preg,
    output logic        prf_req,
    input  logic        prf_grant,

    // D-Cache intf
    output logic [31:0] dcache_addr,
    output logic [3:0]  dcache_wmask,
    output logic [31:0] dcache_wdata,
    input  logic        dcache_resp,
    output logic        dcache_req,
    input  logic        dcache_grant,

    // ROB update intf (for RVFI)
    output logic        store_update_mem_info,
    output logic [4:0]  store_update_id,
    output logic [31:0] store_update_addr,
    output logic [3:0]  store_update_wmask,
    output logic [31:0] store_update_wdata,
    output logic        store_ready,

    // queue state outputs (for load queue ordering)
    output logic [$clog2(NUM_ENTRIES)-1:0] sq_head_out,
    output logic [$clog2(NUM_ENTRIES)-1:0] sq_tail_out,
    output logic        sq_empty_out,
    output sq_entry_t   sq_entries_out [NUM_ENTRIES],

    output logic [$clog2(NUM_ENTRIES):0] occupancy,

    output logic        dbg_store_committed,
    output logic        dbg_cache_req_sent,
    output logic        dbg_cache_complete
);

    //FSM
    typedef enum logic [1:0] {
        SQ_IDLE,
        SQ_REQ,
        SQ_WAIT
    } sq_state_t;

    sq_state_t state, next_state;

    sq_entry_t entries [NUM_ENTRIES];

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

    assign sq_head_out = head;
    assign sq_tail_out = tail;
    assign sq_empty_out = empty;

    always_comb begin
        for (integer i = 0; i < NUM_ENTRIES; i++)
            sq_entries_out[i] = entries[i];
    end

    logic [$clog2(NUM_ENTRIES)-1:0] current_idx;

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

    function automatic logic [31:0] format_store_data(logic [31:0] data, mem_op_t op, logic [1:0] offset);
        logic [31:0] result;
        case (op[2:0])
            3'b000:     // SB
                result = offset[1] ? (offset[0] ? {data[7:0], 24'b0} : {8'b0, data[7:0], 16'b0}) : (offset[0] ? {16'b0, data[7:0], 8'b0} : {24'b0, data[7:0]});
            3'b001:     // SH
                result = offset[1] ? {data[15:0], 16'b0} : {16'b0, data[15:0]};
            3'b010:     // SW
                result = data;

            default: result = data;
        endcase
        return result;
    endfunction

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
        (dispatch_valid && !full && !almost_full) ||  // Both stores: need 2 slots
        (!dispatch_valid && !full)                     // Only inst 1: need 1 slot
    );

    // STORE ISSUE
    logic [$clog2(NUM_ENTRIES)-1:0] store_issue_idx;
    logic can_issue_store;

    always_comb begin
        can_issue_store = 1'b0;
        store_issue_idx = head;

        for (integer i = 0; i < NUM_ENTRIES; i++) begin
            automatic logic [$clog2(NUM_ENTRIES)-1:0] idx;
            idx = ($clog2(NUM_ENTRIES))'((head + ($clog2(NUM_ENTRIES))'($unsigned(i))) % NUM_ENTRIES);

            if (entries[idx].valid && entries[idx].committed && entries[idx].addr_ready && entries[idx].src2_ready && !entries[idx].executed) begin
                can_issue_store = 1'b1;
                store_issue_idx = idx;
                break;
            end
        end
    end

    // STORE BROADCAST
    logic need_store_broadcast;
    logic [$clog2(NUM_ENTRIES)-1:0] store_broadcast_idx;

    always_comb begin
        need_store_broadcast = 1'b0;
        store_broadcast_idx = head;

        for (integer i = 0; i < NUM_ENTRIES; i++) begin
            automatic logic [$clog2(NUM_ENTRIES)-1:0] idx;
            idx = ($clog2(NUM_ENTRIES))'((head + ($clog2(NUM_ENTRIES))'($unsigned(i))) % NUM_ENTRIES);

            if (entries[idx].valid && entries[idx].addr_ready && entries[idx].src2_ready && !entries[idx].cdb_broadcast) begin
                need_store_broadcast = 1'b1;
                store_broadcast_idx = idx;
                break;
            end
        end
    end

    assign store_ready = need_store_broadcast;

    // PRF FOR ADDR CALC
    logic need_prf_for_addr_comb;
    logic [$clog2(NUM_ENTRIES)-1:0] addr_calc_idx_comb;

    logic prf_req_r;
    logic [$clog2(NUM_ENTRIES)-1:0] addr_calc_idx_r;
    logic [31:0] offset_r;
    logic [5:0] src1_preg_r, src2_preg_r;

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
    assign prf_src2_preg = src2_preg_r;

    // FSM - STORE EXEC
    always_ff @(posedge clk) begin
        if (rst || flush) begin
            state <= SQ_IDLE;
            current_idx <= '0;
        end
        else begin
            state <= next_state;

            if (state == SQ_IDLE && next_state == SQ_REQ)
                current_idx <= store_issue_idx;
        end
    end

    always_comb begin
        next_state = state;
        dcache_req = 1'b0;

        case (state)
            SQ_IDLE: begin
                if (can_issue_store) begin
                    dcache_req = 1'b1;
                    if (dcache_grant)
                        next_state = SQ_REQ;
                end
            end

            SQ_REQ: begin
                dcache_req = 1'b1;
                if (dcache_resp)
                    next_state = SQ_IDLE;
                else
                    next_state = SQ_WAIT;
            end

            SQ_WAIT: begin
                dcache_req = 1'b1;
                if (dcache_resp)
                    next_state = SQ_IDLE;
            end

            default: next_state = SQ_IDLE;
        endcase
    end

    assign dbg_store_committed = (rob_commit_valid && rob_commit_is_store) || (rob_commit_valid_1 && rob_commit_is_store_1);
    assign dbg_cache_req_sent = (state == SQ_REQ);
    assign dbg_cache_complete = ((state == SQ_WAIT || state == SQ_REQ) && dcache_resp);

    // D-CACHE INTF
    always_comb begin
        dcache_addr = '0;
        dcache_wmask = '0;
        dcache_wdata = '0;

        if (state == SQ_REQ || state == SQ_WAIT) begin
            dcache_addr = {entries[current_idx].address[31:2], 2'b00};
            dcache_wmask = get_wmask(entries[current_idx].mem_op, entries[current_idx].address[1:0]);
            dcache_wdata = format_store_data(entries[current_idx].store_data, entries[current_idx].mem_op, entries[current_idx].address[1:0]);
        end
    end

    // ROB UPDATE OUTPUT
    always_comb begin
        store_update_mem_info = 1'b0;
        store_update_id = '0;
        store_update_addr = '0;
        store_update_wmask = '0;
        store_update_wdata = '0;

        if (need_store_broadcast) begin
            store_update_mem_info = 1'b1;
            store_update_id = entries[store_broadcast_idx].rob_id;
            store_update_addr = {entries[store_broadcast_idx].address[31:2], 2'b00};
            store_update_wmask = get_wmask(entries[store_broadcast_idx].mem_op, entries[store_broadcast_idx].address[1:0]);
            store_update_wdata = format_store_data(entries[store_broadcast_idx].store_data, entries[store_broadcast_idx].mem_op, entries[store_broadcast_idx].address[1:0]);
        end
    end

    // QUEUE STUFF
    logic should_dequeue;
    logic [$clog2(NUM_ENTRIES)-1:0] new_tail;

    always_ff @(posedge clk) begin
        if (rst) begin
            head <= '0;
            tail <= '0;
            queue_valid <= 1'b0;

            prf_req_r <= 1'b0;
            addr_calc_idx_r <= '0;
            offset_r <= '0;
            src1_preg_r <= '0;
            src2_preg_r <= '0;

            for (integer i = 0; i < NUM_ENTRIES; i++) begin
                entries[i].valid <= 1'b0;
            end
        end
        else if (flush) begin
            new_tail = '0;

            for (integer i = 0; i < NUM_ENTRIES; i++) begin
                // Check if this store is already committed OR being committed this cycle
                automatic logic is_committed_now;
                is_committed_now = entries[i].committed ||
                                   (rob_commit_valid && rob_commit_is_store && entries[i].rob_id == rob_commit_id) ||
                                   (rob_commit_valid_1 && rob_commit_is_store_1 && entries[i].rob_id == rob_commit_id_1);

                if (entries[i].valid && is_committed_now) begin
                    if (new_tail != ($clog2(NUM_ENTRIES))'($unsigned(i))) begin
                        entries[new_tail] <= entries[i];
                        entries[new_tail].committed <= 1'b1;
                        entries[i].valid <= 1'b0;
                    end else begin
                        entries[i].committed <= 1'b1;
                    end
                    new_tail = ($clog2(NUM_ENTRIES))'(new_tail + 1);
                end
                else if (entries[i].valid) begin
                    entries[i].valid <= 1'b0;
                end
            end

            head <= '0;
            tail <= new_tail;
            queue_valid <= (new_tail != 0);

            prf_req_r <= 1'b0;
        end
        else begin
            // DISPATCH
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

            // port 0 commit
            if (rob_commit_valid && rob_commit_is_store) begin
                for (integer i = 0; i < NUM_ENTRIES; i++) begin
                    if (entries[i].valid && entries[i].rob_id == rob_commit_id) begin
                        entries[i].committed <= 1'b1;
                    end
                end
            end

            // port 1 commit (dual commit)
            if (rob_commit_valid_1 && rob_commit_is_store_1) begin
                for (integer i = 0; i < NUM_ENTRIES; i++) begin
                    if (entries[i].valid && entries[i].rob_id == rob_commit_id_1) begin
                        entries[i].committed <= 1'b1;
                    end
                end
            end

            // CDB WAKEUP (with rob_id validation to prevent stale wakeups)
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb[i].valid) begin
                    for (integer j = 0; j < NUM_ENTRIES; j++) begin
                        if (entries[j].valid) begin
                            // src1 wakeup: must match both preg AND rob_id
                            if (!entries[j].src1_ready &&
                                entries[j].src1_preg == cdb[i].preg &&
                                cdb[i].preg != 6'd0 &&
                                entries[j].src1_rob_id == cdb[i].rob_id)
                                entries[j].src1_ready <= 1'b1;

                            // src2 wakeup: must match both preg AND rob_id
                            if (!entries[j].src2_ready &&
                                entries[j].src2_preg == cdb[i].preg &&
                                cdb[i].preg != 6'd0 &&
                                entries[j].src2_rob_id == cdb[i].rob_id) begin
                                entries[j].src2_ready <= 1'b1;
                                entries[j].store_data <= cdb[i].data;
                            end
                        end
                    end
                end
            end

            if (prf_grant && prf_req_r)
                prf_req_r <= 1'b0;
            else if (need_prf_for_addr_comb && !prf_req_r) begin
                prf_req_r <= 1'b1;
                addr_calc_idx_r <= addr_calc_idx_comb;
                offset_r <= entries[addr_calc_idx_comb].offset;
                src1_preg_r <= entries[addr_calc_idx_comb].src1_preg;
                src2_preg_r <= entries[addr_calc_idx_comb].src2_preg;
            end

            if (prf_req_r && prf_grant) begin
                if (entries[addr_calc_idx_r].valid && !entries[addr_calc_idx_r].addr_ready) begin
                    automatic logic [31:0] calc_addr;
                    calc_addr = prf_src1_data + offset_r;
                    entries[addr_calc_idx_r].address <= calc_addr;
                    entries[addr_calc_idx_r].addr_ready <= 1'b1;
                end
            end

            if ((state == SQ_WAIT || state == SQ_REQ) && dcache_resp) begin
                entries[current_idx].executed <= 1'b1;
            end

            if (need_store_broadcast)
                entries[store_broadcast_idx].cdb_broadcast <= 1'b1;

            should_dequeue = 1'b0;
            if (!empty && entries[head].valid && entries[head].executed)
                should_dequeue = 1'b1;

            if (should_dequeue) begin
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

endmodule : store_queue