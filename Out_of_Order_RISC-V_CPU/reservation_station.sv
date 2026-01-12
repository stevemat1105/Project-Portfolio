// RESERVATION STATION (2-WAY SUPERSCALAR)
//
// holds insts until operands ready, then issues to exec
// - parameterizable depth (8 for ALU, 4 for MUL/DIV)
// - FIFO ordering: oldest ready inst issued first
// - wakeup: mark sources ready when CDB broadcasts
// - issue: select oldest ready inst for exec
//
// FOR PRF ARBITER:
// - requests PRF access when entry ready (prf_req)
// - only issue when granted PRF access (prf_grant)
//
// 2-WAY SUPERSCALAR SUPPORT:
// - Dual dispatch ports for 2 instructions per cycle
// - full: cannot dispatch even 1 instruction
// - almost_full: can dispatch 1 but not 2 instructions
//
// 1. Dispatch - allocate entry for new inst from rename
// 2. Wakeup - CDB checking to mark src ready
// 3. Request - signal arbiter when ready to issue (prf_req)
// 4. Issue - pick oldest ready inst IF granted (prf_grant)
// 5. handle immediate operands (src2_is_imm)

module reservation_station
import types::*;
#(
    parameter NUM_ENTRIES = 8
)(
    input  logic        clk,
    input  logic        rst,

     
    // DISPATCH INTERFACE - Instruction 0 (from rename)
     
    input  logic        dispatch_valid,
    input  rs_entry_t   dispatch_entry,
    output logic        full,
    output logic        almost_full,

     
    // DISPATCH INTERFACE - Instruction 1 (superscalar)
     
    input  logic        dispatch_valid_1,
    input  rs_entry_t   dispatch_entry_1,

    input  logic        flush,

    // CDB intf (for wakeup)
    input  cdb_t        cdb [NUM_CDB_PORTS],

    // issue intf (to execute)
    output logic        issue_valid,
    output rs_entry_t   issue_entry,
    input  logic        execute_ready,

    // PRF arbiter intf
    output logic        prf_req,    // request PRF when entry ready
    input  logic        prf_grant,  // grant from arbiter

    // Area profiling
    output logic [$clog2(NUM_ENTRIES):0] occupancy  // Current number of valid entries
);

    rs_entry_t entries [NUM_ENTRIES];

     
    // EMPTY SLOT DETECTION (dual dispatch)
     
    logic [$clog2(NUM_ENTRIES)-1:0] empty_idx;
    logic [$clog2(NUM_ENTRIES)-1:0] empty_idx_1;  // Second empty slot for inst 1
    logic [$clog2(NUM_ENTRIES)-1:0] dispatch_idx;
    logic [$clog2(NUM_ENTRIES)-1:0] dispatch_idx_1;

    logic [$clog2(NUM_ENTRIES)-1:0] issue_idx;
    logic can_issue;

    // Count empty slots
    logic [$clog2(NUM_ENTRIES):0] empty_count;

    // signal when freeing a slot this cycle
    logic will_issue;
    assign will_issue = can_issue && prf_grant && execute_ready;
    logic found_first;

    always_comb begin
        empty_idx = '0;
        empty_idx_1 = '0;
        empty_count = '0;
        found_first = 1'b0;

        // Find first and second empty entries
        for (integer unsigned i = 0; i < NUM_ENTRIES; i++) begin
            if (!entries[i].valid) begin
                if (!found_first) begin
                    empty_idx = $clog2(NUM_ENTRIES)'(i);
                    found_first = 1'b1;
                end
                else if (empty_count == 1) begin
                    empty_idx_1 = $clog2(NUM_ENTRIES)'(i);
                end
                empty_count = empty_count + ($clog2(NUM_ENTRIES)+1)'(1);
            end
        end

        // Full/almost_full based on empty count
        full = (empty_count == 0);
        almost_full = (empty_count == 1);

        // Account for slot being freed by issue
        if (will_issue) begin
            if (full) begin
                full = 1'b0;
                almost_full = 1'b1;
            end
            else if (almost_full) begin
                almost_full = 1'b0;
            end
        end

        // Where to dispatch inst 0
        if (will_issue && empty_count == 0) begin
            // If issuing from a full RS, dispatch to the slot being freed
            dispatch_idx = issue_idx;
        end
        else begin
            dispatch_idx = empty_idx;
        end

        // Where to dispatch inst 1
        // CASE A: Both inst_0 and inst_1 dispatch to this RS
        // CASE B: Only inst_1 dispatches here (inst_0 went elsewhere)
        if (!dispatch_valid) begin
            // CASE B: Only inst_1 - use first empty slot (same logic as inst_0)
            if (will_issue && empty_count == 0)
                dispatch_idx_1 = issue_idx;
            else
                dispatch_idx_1 = empty_idx;
        end
        else begin
            // CASE A: Both - inst_1 uses second slot
            if (will_issue && empty_count == 1) begin
                // Only one empty slot but issuing - use issuing slot for inst 1
                dispatch_idx_1 = issue_idx;
            end
            else if (empty_count >= 2) begin
                dispatch_idx_1 = empty_idx_1;
            end
            else begin
                // Fallback (shouldn't dispatch inst 1 in this case)
                dispatch_idx_1 = empty_idx_1;
            end
        end
    end

    
    always_comb begin
        can_issue = 1'b0;
        issue_idx = $clog2(NUM_ENTRIES)'(0);
        
        for (integer unsigned i = 0; i < NUM_ENTRIES; i++) begin
            if (entries[i].valid && 
                entries[i].src1_ready && 
                (entries[i].src2_ready || entries[i].src2_is_imm)) begin
                can_issue = 1'b1;
                issue_idx = $clog2(NUM_ENTRIES)'(i);
                break;
            end
        end
        
        issue_entry = entries[issue_idx];
    end
    
    assign prf_req = can_issue;
    assign issue_valid = can_issue && prf_grant && execute_ready;

    
     
    // DUAL DISPATCH CONTROL
     
    logic do_dispatch, do_dispatch_1;
    logic [1:0] available_slots;

    always_comb begin
        // Calculate how many slots are available (accounting for issuing)
        available_slots = {1'b0, !full} + {1'b0, !almost_full && !full};
        if (will_issue)
            available_slots = available_slots + 2'(1);

        // Can dispatch inst 0 if slot available
        do_dispatch = dispatch_valid && (available_slots >= 1);

        // Can dispatch inst 1 if:
        // - CASE A: Both insts go to same RS - need 2 slots (or 1 + issuing)
        // - CASE B: Only inst 1 goes here (inst 0 went elsewhere) - need 1 slot
        if (dispatch_valid_1) begin
            if (dispatch_valid) begin
                // CASE A: Both insts to same RS - need 2 slots
                do_dispatch_1 = do_dispatch && (available_slots >= 2);
            end else begin
                // CASE B: Only inst 1 to this RS - need 1 slot
                do_dispatch_1 = (available_slots >= 1);
            end
        end else begin
            do_dispatch_1 = 1'b0;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            // reset all entries to invalid
            for (integer i = 0; i < NUM_ENTRIES; i++) begin
                entries[i].valid <= 1'b0;
            end
        end
        else if (flush) begin
            for (integer i = 0; i < NUM_ENTRIES; i++) begin
                entries[i].valid <= 1'b0;
            end
        end
        else begin
             
            // DUAL DISPATCH
             
            // Dispatch inst 0
            if (do_dispatch)
                entries[dispatch_idx] <= dispatch_entry;

            // Dispatch inst 1
            if (do_dispatch_1)
                entries[dispatch_idx_1] <= dispatch_entry_1;

             
            // WAKEUP from all CDB ports (with rob_id validation to prevent stale wakeups)
             
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb[i].valid) begin
                    for (integer j = 0; j < NUM_ENTRIES; j++) begin
                        if (entries[j].valid) begin
                            // src1 wakeup: must match both preg AND rob_id
                            if (!entries[j].src1_ready &&
                                entries[j].src1_preg == cdb[i].preg &&
                                cdb[i].preg != 6'd0 &&
                                entries[j].src1_rob_id == cdb[i].rob_id) begin
                                entries[j].src1_ready <= 1'b1;
                            end
                            // src2 wakeup: must match both preg AND rob_id
                            if (!entries[j].src2_is_imm && !entries[j].src2_ready &&
                                entries[j].src2_preg == cdb[i].preg &&
                                cdb[i].preg != 6'd0 &&
                                entries[j].src2_rob_id == cdb[i].rob_id) begin
                                entries[j].src2_ready <= 1'b1;
                            end
                        end
                    end
                end
            end

             
            // ISSUE - clear issued entry's valid  
             
            // BUT NOT if we're simultaneously dispatching to that same slot
            if (will_issue) begin
                // Check if we're dispatching to the issuing slot
                logic dispatching_to_issue_slot;
                dispatching_to_issue_slot = (do_dispatch && dispatch_idx == issue_idx) ||
                                            (do_dispatch_1 && dispatch_idx_1 == issue_idx);
                if (!dispatching_to_issue_slot)
                    entries[issue_idx].valid <= 1'b0;
            end
        end
    end

    // Compute occupancy for area profiling
    always_comb begin
        automatic logic [$clog2(NUM_ENTRIES):0] count;
        count = '0;
        for (integer i = 0; i < NUM_ENTRIES; i++) begin
            if (entries[i].valid)
                count = count + ($clog2(NUM_ENTRIES)+1)'(1);
        end
        occupancy = count;
    end

endmodule