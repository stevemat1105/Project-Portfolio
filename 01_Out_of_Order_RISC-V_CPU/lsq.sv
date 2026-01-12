
module lsq
import types::*;
#(
    parameter NUM_ENTRIES = 8
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        flush,
    
    // (from rename)
    input  logic        dispatch_valid,
    input  lsq_entry_t  dispatch_entry,
    output logic        full,
    
    // (for wakeup)
    input  cdb_t        cdb [NUM_CDB_PORTS],
    
    //  (for stores)
    input  logic        rob_commit_valid,
    input  logic [4:0]  rob_commit_id,
    input  logic        rob_commit_is_store,
    
    // PRF read intf (for addr calc and store data)
    output logic [5:0]  prf_src1_preg,
    input  logic [31:0] prf_src1_data,
    output logic [5:0]  prf_src2_preg,
    input  logic [31:0] prf_src2_data,
    output logic        prf_req,
    input  logic        prf_grant,
    
    // D-Cache intf
    output logic [31:0] dcache_addr,
    output logic [3:0]  dcache_rmask,
    output logic [3:0]  dcache_wmask,
    output logic [31:0] dcache_wdata,
    input  logic [31:0] dcache_rdata,
    input  logic        dcache_resp,
    
    output cdb_t        lsq_load_cdb,
    input  logic        lsq_load_cdb_grant,

    // load update port to ROB
    output logic        load_update_mem_info,
    output logic [4:0]  load_update_id,
    output logic [31:0] load_update_addr,
    output logic [3:0]  load_update_rmask,
    output logic [31:0] load_update_rdata,

    // store update port to ROB
    output logic        store_update_mem_info,
    output logic [4:0]  store_update_id,
    output logic [31:0] store_update_addr,
    output logic [3:0]  store_update_wmask,
    output logic [31:0] store_update_wdata,
    output logic        store_ready,

    // Area profiling
    output logic [$clog2(NUM_ENTRIES):0] occupancy  // Current number of valid entries
);

    typedef enum logic [2:0] {
        IDLE,
        LOAD_REQ,
        LOAD_WAIT,
        STORE_REQ,
        STORE_WAIT,
        LOAD_WRITEBACK
    } mem_state_t;
    
    mem_state_t state, next_state;

    lsq_entry_t entries [NUM_ENTRIES];
    
    logic [$clog2(NUM_ENTRIES)-1:0] head, tail;
    logic [$clog2(NUM_ENTRIES)-1:0] next_head, next_tail;
    
    localparam [$clog2(NUM_ENTRIES)-1:0] MAX_INDEX = NUM_ENTRIES - 1;
    
    assign next_head = (head == MAX_INDEX) ? '0 : head + 1'b1;
    assign next_tail = (tail == MAX_INDEX) ? '0 : tail + 1'b1;
    
    logic empty, queue_valid;
    assign empty = !queue_valid && (head == tail);
    assign full = queue_valid && (head == tail);
    
    logic [$clog2(NUM_ENTRIES)-1:0] current_idx;
    // Load result
    logic        load_result_valid;
    logic [31:0] load_result;
    logic [31:0] load_raw_data;
    logic        load_rob_update_ready;

    
    function automatic logic [3:0] get_rmask(mem_op_t op, logic [1:0] offset);
        logic [3:0] mask;
        case (op[1:0])
            2'b00: mask = 4'b0001 << offset;
            2'b01: mask = 4'b0011 << offset;
            2'b10: mask = 4'b1111;
            default: mask = 4'b0000;
        endcase
        return mask;
    endfunction
    
    function automatic logic [3:0] get_wmask(mem_op_t op, logic [1:0] offset);
        logic [3:0] mask;
        case (op[1:0])
            2'b00: mask = 4'b0001 << offset;
            2'b01: mask = 4'b0011 << offset;
            2'b10: mask = 4'b1111;
            default: mask = 4'b0000;
        endcase
        return mask;
    endfunction

    function automatic logic [31:0] format_load_data(logic [31:0] raw_data, mem_op_t op, logic [1:0] offset);
        logic [31:0] result;
        logic [31:0] shifted_data;

        shifted_data = raw_data >> (offset * 8);

        case (op)
            MEM_LB: result = {{24{shifted_data[7]}}, shifted_data[7:0]};
            MEM_LBU: result = {24'b0, shifted_data[7:0]};
            MEM_LH: result = {{16{shifted_data[15]}}, shifted_data[15:0]};
            MEM_LHU: result = {16'b0, shifted_data[15:0]};
            MEM_LW: result = shifted_data;
            default: result = shifted_data;
        endcase
        return result;
    endfunction


    function automatic logic [31:0] extract_raw_load_data(logic [31:0] raw_data, mem_op_t op, logic [1:0] offset);
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
    
    function automatic logic [31:0] format_store_data(logic [31:0] data, mem_op_t op, logic [1:0] offset);
        logic [31:0] result;
        case (op[2:0])
            3'b000: begin
                result = offset[1] ? (offset[0] ? {data[7:0], 24'b0} : {8'b0, data[7:0], 16'b0}) : (offset[0] ? {16'b0, data[7:0], 8'b0} : {24'b0, data[7:0]});
            end
            3'b001: begin
                result = offset[1] ? {data[15:0], 16'b0} : {16'b0, data[15:0]};
            end
            3'b010: begin
                result = data;
            end
            default: result = data;
        endcase
        return result;
    endfunction
    
    // dispatch code    
    logic [$clog2(NUM_ENTRIES)-1:0] dispatch_idx;
    logic can_dispatch;

    
    logic need_store_broadcast;
    logic [$clog2(NUM_ENTRIES)-1:0] store_broadcast_idx;

    
    assign dispatch_idx = tail;
    assign can_dispatch = dispatch_valid && !full;
    
    logic any_store_addr_unknown;
    always_comb begin
        any_store_addr_unknown = 1'b0;
        for (integer i = 0; i < NUM_ENTRIES; i++) begin
            if (entries[i].valid && entries[i].is_store && !entries[i].addr_ready)
                any_store_addr_unknown = 1'b1;
        end
    end

    // find oldest ready load
    logic [$clog2(NUM_ENTRIES)-1:0] load_issue_idx;
    logic can_issue_load;

    always_comb begin
        can_issue_load = 1'b0;
        load_issue_idx = head;

        for (integer i = 0; i < NUM_ENTRIES; i++) begin
            logic [$clog2(NUM_ENTRIES)-1:0] idx;
            idx = ($clog2(NUM_ENTRIES))'(head + ($clog2(NUM_ENTRIES))'(i));

            if (idx >= NUM_ENTRIES)
                idx = idx - ($clog2(NUM_ENTRIES))'(NUM_ENTRIES);

            if (entries[idx].valid && entries[idx].is_load &&
                entries[idx].addr_ready && !entries[idx].executed) begin

                logic blocked_by_older_store;
                logic blocked_by_addr_match;
                blocked_by_older_store = 1'b0;
                blocked_by_addr_match = 1'b0;

                for (integer j = 0; j < i; j++) begin
                    logic [$clog2(NUM_ENTRIES)-1:0] older_idx;
                    older_idx = ($clog2(NUM_ENTRIES))'(head + ($clog2(NUM_ENTRIES))'(j));

                    if (older_idx >= NUM_ENTRIES)
                        older_idx = older_idx - ($clog2(NUM_ENTRIES))'(NUM_ENTRIES);

                    // Block if older store doesn't have address yet
                    if (entries[older_idx].valid && entries[older_idx].is_store && !entries[older_idx].addr_ready) begin
                        blocked_by_older_store = 1'b1;
                        break;
                    end

                    if (entries[older_idx].valid && entries[older_idx].is_store &&
                        entries[older_idx].addr_ready && !entries[older_idx].executed &&
                        entries[older_idx].address[31:2] == entries[idx].address[31:2]) begin
                        blocked_by_addr_match = 1'b1;
                        break;
                    end
                end

                if (!blocked_by_older_store && !blocked_by_addr_match) begin
                    can_issue_load = 1'b1;
                    load_issue_idx = idx;
                    break;
                end
            end
        end
    end
    
    
    logic [$clog2(NUM_ENTRIES)-1:0] store_issue_idx;
    logic can_issue_store;

    always_comb begin
        can_issue_store = 1'b0;
        store_issue_idx = head;

        for (integer i = 0; i < NUM_ENTRIES; i++) begin
            automatic logic [$clog2(NUM_ENTRIES)-1:0] idx;
            idx = ($clog2(NUM_ENTRIES))'(head + ($clog2(NUM_ENTRIES))'(i));

            if (idx >= NUM_ENTRIES)
                idx = idx - ($clog2(NUM_ENTRIES))'(NUM_ENTRIES);

            if (entries[idx].valid && entries[idx].is_store && entries[idx].committed && entries[idx].addr_ready && entries[idx].src2_ready && !entries[idx].executed) begin

                can_issue_store = 1'b1;
                store_issue_idx = idx;
                break;
            end
        end
    end
    
    // PRF access for addr calc - PIPELINED for timing
    // Stage 1: Combinational selection
    logic need_prf_for_addr_comb;
    logic [$clog2(NUM_ENTRIES)-1:0] addr_calc_idx_comb;

    // Stage 2: Registered request + data snapshot (breaks timing path)
    logic prf_req_r;
    logic [$clog2(NUM_ENTRIES)-1:0] addr_calc_idx_r;
    logic [31:0] offset_r;  // Snapshot the offset we need
    logic [5:0] src1_preg_r, src2_preg_r;

    always_comb begin
        need_prf_for_addr_comb = 1'b0;
        addr_calc_idx_comb = head;

        for (integer i = 0; i < NUM_ENTRIES; i++) begin
            logic [$clog2(NUM_ENTRIES)-1:0] idx;
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
    

    always_ff @(posedge clk) begin

        if (rst || flush) begin
            load_result_valid <= 1'b0;
            load_result <= 32'h0;
            load_raw_data <= 32'h0;
            load_rob_update_ready <= 1'b0;

            prf_req_r <= 1'b0;
            addr_calc_idx_r <= '0;
            offset_r <= '0;
            src1_preg_r <= '0;
            src2_preg_r <= '0;
        end
        else begin
            if (prf_grant && prf_req_r)
                prf_req_r <= 1'b0;
            
            else if (need_prf_for_addr_comb && !prf_req_r) begin
                prf_req_r <= 1'b1;
                addr_calc_idx_r <= addr_calc_idx_comb;
                offset_r <= entries[addr_calc_idx_comb].offset;
                src1_preg_r <= entries[addr_calc_idx_comb].src1_preg;
                src2_preg_r <= entries[addr_calc_idx_comb].src2_preg;
            end

            if ((state == LOAD_REQ || state == LOAD_WAIT) && dcache_resp) begin

                load_result <= format_load_data(dcache_rdata, entries[current_idx].mem_op, entries[current_idx].address[1:0]);

                load_raw_data <= extract_raw_load_data(dcache_rdata, entries[current_idx].mem_op,entries[current_idx].address[1:0]);

                load_result_valid <= 1'b1;
                load_rob_update_ready <= 1'b1;
            end

            else if (load_rob_update_ready)
                load_rob_update_ready <= 1'b0;
            
            if (state == LOAD_WRITEBACK && lsq_load_cdb_grant) begin
                load_result_valid <= 1'b0;
                load_result <= 32'h0;
                load_raw_data <= 32'h0;
            end
        end
    end
    
    // FSM
    always_ff @(posedge clk) begin
        if (rst || flush) begin
            state <= IDLE;
            current_idx <= '0;
        end
        
        else begin
            state <= next_state;
            
            if (state == IDLE && next_state == LOAD_REQ)
                current_idx <= load_issue_idx;
            else if (state == IDLE && next_state == STORE_REQ)
                current_idx <= store_issue_idx;
        end
    end
    
    always_comb begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (can_issue_store)
                    next_state = STORE_REQ;
                else if (can_issue_load)
                    next_state = LOAD_REQ;
            end
            
            LOAD_REQ: begin
                next_state = LOAD_WAIT;
            end
            
            LOAD_WAIT: begin
                if (dcache_resp)
                    next_state = LOAD_WRITEBACK;
            end
            
            LOAD_WRITEBACK: begin
                if (lsq_load_cdb_grant) begin
                    next_state = IDLE;
                end
            end
            
            STORE_REQ: begin
                next_state = STORE_WAIT;
            end
            
            STORE_WAIT: begin
                if (dcache_resp)
                    next_state = IDLE;
            end
            
            default: next_state = IDLE;
        endcase
    end
    
    // D-cache intf.
    always_comb begin
        dcache_addr = '0;
        dcache_rmask = '0;
        dcache_wmask = '0;
        dcache_wdata = '0;

        case (state)
            LOAD_REQ, LOAD_WAIT: begin
                dcache_addr = {entries[current_idx].address[31:2], 2'b00};
                dcache_rmask = get_rmask(entries[current_idx].mem_op, entries[current_idx].address[1:0]);
            end

            STORE_REQ, STORE_WAIT: begin
                dcache_addr = {entries[current_idx].address[31:2], 2'b00};
                dcache_wmask = get_wmask(entries[current_idx].mem_op, entries[current_idx].address[1:0]);
                dcache_wdata = format_store_data(entries[current_idx].store_data, entries[current_idx].mem_op, entries[current_idx].address[1:0]);
            end
        endcase
    end
    
    // find store that needs CDB broadcast to mark ROB ready
    always_comb begin
        need_store_broadcast = 1'b0;
        store_broadcast_idx = head;

        for (integer i = 0; i < NUM_ENTRIES; i++) begin
            logic [$clog2(NUM_ENTRIES)-1:0] idx;
            idx = ($clog2(NUM_ENTRIES))'((head + ($clog2(NUM_ENTRIES))'($unsigned(i))) % NUM_ENTRIES);

            if (entries[idx].valid && entries[idx].is_store && entries[idx].addr_ready && entries[idx].src2_ready && !entries[idx].cdb_broadcast) begin
                need_store_broadcast = 1'b1;
                store_broadcast_idx = idx;
                break;
            end
        end
    end


    // load CDB
    always_comb begin
        if ((state == LOAD_WRITEBACK) && load_result_valid) begin
            lsq_load_cdb.valid = 1'b1;
            lsq_load_cdb.data = load_result;
            lsq_load_cdb.preg = entries[current_idx].dest_preg;
            lsq_load_cdb.rob_id = entries[current_idx].rob_id;
        end
        else begin
            lsq_load_cdb.valid = 1'b0;
            lsq_load_cdb.data = 32'h0;
            lsq_load_cdb.preg = 6'b0;
            lsq_load_cdb.rob_id = 4'b0;
        end
    end

    assign store_ready = need_store_broadcast;
    // load update port
    always_comb begin
        load_update_mem_info = 1'b0;
        load_update_id = '0;
        load_update_addr = '0;
        load_update_rmask = '0;
        load_update_rdata = '0;

        if (load_rob_update_ready) begin
            
            load_update_mem_info = 1'b1;
            load_update_id = entries[current_idx].rob_id;
            load_update_addr = {entries[current_idx].address[31:2], 2'b00};

            load_update_rmask = get_rmask(entries[current_idx].mem_op, entries[current_idx].address[1:0]);

            load_update_rdata = load_raw_data;
        end
    end

    // store update port
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
    
    logic should_dequeue;
    logic cdb_updating_src2;
    logic [$clog2(NUM_ENTRIES)-1:0] new_tail;
    logic was_executing;

    
    always_ff @(posedge clk) begin

        if (rst) begin
            head <= '0;
            tail <= '0;
            queue_valid <= 1'b0;

            for (integer i = 0; i < NUM_ENTRIES; i++) begin
                entries[i] <= '0;
            end
        end
        else if (flush) begin

            new_tail = '0;
            
            for (integer i = 0; i < NUM_ENTRIES; i++) begin

                was_executing = (($clog2(NUM_ENTRIES))'($unsigned(i)) == current_idx) && (state == STORE_REQ || state == STORE_WAIT || state == LOAD_REQ || state == LOAD_WAIT);

                if (entries[i].valid && entries[i].is_store && entries[i].committed) begin

                    if (new_tail != ($clog2(NUM_ENTRIES))'($unsigned(i))) begin
                        entries[new_tail] <= entries[i];
                        entries[i] <= '0;
                    end

                    new_tail = ($clog2(NUM_ENTRIES))'(new_tail + 1);
                end
                else
                    entries[i] <= '0;
            end
            
            head <= '0;
            tail <= new_tail;
            queue_valid <= (new_tail != 0);
        end

        else begin
            if (can_dispatch) begin

                entries[dispatch_idx] <= dispatch_entry;
                tail <= next_tail;

                if (next_tail == head)
                    queue_valid <= 1'b1;
            end
            
            if (rob_commit_valid && rob_commit_is_store) begin

                logic found_match;
                found_match = 1'b0;

                for (integer i = 0; i < NUM_ENTRIES; i++) begin
                    if (entries[i].valid && entries[i].is_store && entries[i].rob_id == rob_commit_id) begin
                        entries[i].committed <= 1'b1;
                        found_match = 1'b1;
                    end
                end
            end
            
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin

                if (cdb[i].valid) begin
                    
                    for (integer j = 0; j < NUM_ENTRIES; j++) begin

                        if (entries[j].valid) begin

                            if (entries[j].src1_preg == cdb[i].preg && cdb[i].preg != 6'd0)
                                entries[j].src1_ready <= 1'b1;
                            
                            if (entries[j].src2_preg == cdb[i].preg && cdb[i].preg != 6'd0 && !entries[j].src2_ready) begin

                                entries[j].src2_ready <= 1'b1;
                                
                                if (entries[j].is_store)
                                    entries[j].store_data <= cdb[i].data;
                            end
                        end
                    end
                end
            end
            
            if (prf_req_r && prf_grant) begin

                if (entries[addr_calc_idx_r].valid && !entries[addr_calc_idx_r].addr_ready) begin

                    logic [31:0] calc_addr;
                    calc_addr = prf_src1_data + offset_r;
                    entries[addr_calc_idx_r].address <= calc_addr;
                    entries[addr_calc_idx_r].addr_ready <= 1'b1;

                    cdb_updating_src2 = 1'b0;

                    for (integer k = 0; k < NUM_CDB_PORTS; k++) begin
                        if (cdb[k].valid && cdb[k].preg == entries[addr_calc_idx_r].src2_preg && cdb[k].preg != 6'd0)
                            cdb_updating_src2 = 1'b1;
                    end
                end
            end
            
            if (state == LOAD_WRITEBACK && lsq_load_cdb_grant)
                entries[current_idx].executed <= 1'b1;

            if (state == STORE_WAIT && dcache_resp)
                entries[current_idx].executed <= 1'b1;

            // store broadcast when ready
            if (need_store_broadcast)
                entries[store_broadcast_idx].cdb_broadcast <= 1'b1;

            should_dequeue = 1'b0;

            if (!empty && entries[head].valid && entries[head].executed)
                should_dequeue = 1'b1;

            if (should_dequeue) begin
                // clear entire entry to prevent stale data
                entries[head] <= '0;
                head <= next_head;

                if (next_head == tail) begin
                    queue_valid <= 1'b0;
                end
            end
            if (!empty && !entries[head].valid) begin
                head <= next_head;
                if (next_head == tail)
                    queue_valid <= 1'b0;
            end
        end
    end

    // Compute occupancy for area profiling
    always_comb begin
        automatic logic [$clog2(NUM_ENTRIES):0] count;
        count = 0;
        for (integer i = 0; i < NUM_ENTRIES; i++) begin
            if (entries[i].valid)
                count = count + 1;
        end
        occupancy = count;
    end

endmodule : lsq