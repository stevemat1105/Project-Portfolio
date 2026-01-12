module free_list
import types::*;
#(
    parameter NUM_PREGS = 48
)(
    input  logic        clk,
    input  logic        rst,
    
    // allocate intf (dequeue) - used at rename - port 0
    input  logic        alloc,
    output logic [5:0]  alloc_preg,
    output logic        empty,

    // allocate intf (dequeue) - port 1 (dual allocation for superscalar)
    input  logic        alloc_1,
    output logic [5:0]  alloc_preg_1,
    output logic        almost_empty,     // Only 1 register left (can't dual allocate)
    
    // free intf (enqueue) - used at commit - port 0
    input  logic        free,
    input  logic [5:0]  free_preg,
    output logic        full,

    // free intf (enqueue) - used at commit - port 1 (dual commit)
    input  logic        free_1,
    input  logic [5:0]  free_preg_1,

    // free intf - used during flush
    input  logic        flush_free_en,
    input  logic [4:0]  flush_free_count,
    input  logic [5:0]  flush_free_pregs [ROB_SIZE],

    // occupancy output for profiling
    output logic [$clog2(NUM_PREGS-1):0] occupancy_out
);

    localparam DEPTH = NUM_PREGS - 1;
    
    logic [5:0] storage [DEPTH];
    
    logic [$clog2(DEPTH)-1:0] head, tail;
    
    logic valid;
    
    logic [$clog2(DEPTH)-1:0] next_head, next_tail;
    
    localparam [$clog2(DEPTH)-1:0] MAX_INDEX = DEPTH - 1;
    
    assign next_head = (head == MAX_INDEX) ? '0 : head + 1'b1;
    assign next_tail = (tail == MAX_INDEX) ? '0 : tail + 1'b1;

    // head+2 for dual allocation
    logic [$clog2(DEPTH)-1:0] next_next_head;
    assign next_next_head = (next_head == MAX_INDEX) ? '0 : next_head + 1'b1;

    logic do_alloc, do_alloc_1, do_free, do_free_1;
    // Port 0 alloc: can allocate if not empty and no flush
    assign do_alloc   = alloc && !empty && !flush_free_en;
    // Port 1 alloc: can allocate if:
    //   - Both ports allocating: need 2+ available (do_alloc && !almost_empty)
    //   - Only port 1 allocating: need 1+ available (!alloc && !empty)
    assign do_alloc_1 = alloc_1 && !flush_free_en &&
                        ((do_alloc && !almost_empty) ||  // Dual allocation
                         (!alloc && !empty));             // Only inst_1 needs preg
    assign do_free    = free && !full && !flush_free_en;
    assign do_free_1  = free_1 && !flush_free_en;

    assign empty = (head == tail) && !valid;
    assign full  = (head == tail) && valid;

    // almost_empty: only 1 register left (can allocate 1 but not 2)
    assign almost_empty = (next_head == tail) && valid;

    // Return physical registers for both allocation ports
    assign alloc_preg = storage[head];
    // Port 1 gets next_head if port 0 is also allocating, otherwise gets head
    assign alloc_preg_1 = do_alloc ? storage[next_head] : storage[head];

    logic [$clog2(DEPTH):0] occupancy;
    always_comb begin
        if (empty)
            occupancy = '0;
        else if (full)
            occupancy = ($clog2(DEPTH) +1)'($unsigned(DEPTH));
        else if (tail > head)
            occupancy = ($clog2(DEPTH) +1)'(tail - head);
        else
            occupancy = ($clog2(DEPTH) +1)'($unsigned(DEPTH) - head + tail);
    end

    // Output occupancy for profiling
    assign occupancy_out = occupancy;

    logic [$clog2(DEPTH):0] new_tail_simple;

    always_ff @(posedge clk) begin
        if (rst) begin
            // initialize storage with p32-p(NUM_PREGS-1) (speculative regs available at start)
            for (integer unsigned i = 0; i < $unsigned(NUM_PREGS - 32); i++) begin
                storage[i] <= 6'(32 + i);    // p32, p33, ..., p(NUM_PREGS-1)
            end

            head  <= '0;
            tail  <= ($clog2(DEPTH))'($unsigned(NUM_PREGS - 32));  // Number of speculative regs
            valid <= 1'b0;
        end
        else if (flush_free_en && flush_free_count > 0) begin
            for (integer unsigned i = 0; i < ROB_SIZE; i++) begin
                if (($clog2(DEPTH)+1)'(i) < ($clog2(DEPTH)+1)'({1'b0, flush_free_count})) begin
                    automatic logic [$clog2(DEPTH):0] sum;
                    automatic logic [$clog2(DEPTH)-1:0] target_idx;
                    sum = {1'b0, tail} + ($clog2(DEPTH)+1)'(i);  // Zero-extend tail to 7 bits before addition
                    target_idx = (sum >= ($clog2(DEPTH)+1)'(DEPTH)) ? ($clog2(DEPTH))'(sum - DEPTH) : sum[$clog2(DEPTH)-1:0];
                    storage[target_idx] <= flush_free_pregs[i];
                end
            end

            new_tail_simple = {1'b0, tail} + {1'b0, flush_free_count};
            tail <= (new_tail_simple >= ($clog2(DEPTH)+1)'(DEPTH)) ? ($clog2(DEPTH))'(new_tail_simple - DEPTH) : new_tail_simple[$clog2(DEPTH)-1:0];

            // Set valid conservatively
            valid <= 1'b1;
        end
        else begin
             
            // ALLOCATION LOGIC - advance head pointer when allocating
             
            if (do_alloc && do_alloc_1) begin
                // Dual allocation: advance head by 2
                head <= next_next_head;
            end
            else if (do_alloc) begin
                // Single allocation (port 0 only): advance head by 1
                head <= next_head;
            end
            else if (do_alloc_1) begin
                // Single allocation (port 1 only): advance head by 1
                head <= next_head;
            end

             
            // FREE LOGIC - advance tail pointer when freeing
             
            if (do_free && do_free_1) begin
                // both ports freeing
                storage[tail] <= free_preg;
                storage[next_tail] <= free_preg_1;
                tail <= (next_tail == MAX_INDEX) ? '0 : next_tail + 1'b1;
            end
            else if (do_free) begin
                storage[tail] <= free_preg;
                tail <= next_tail;
            end
            else if (do_free_1) begin
                storage[tail] <= free_preg_1;
                tail <= next_tail;
            end

             
            // VALID FLAG UPDATE - track empty/full state
             
            if ((do_free || do_free_1) && empty)
                valid <= 1'b1;
            else if (do_alloc && do_alloc_1 && !do_free && !do_free_1 && next_next_head == tail)
                // Dual allocation empties the list
                valid <= 1'b0;
            else if (do_alloc && !do_alloc_1 && !do_free && !do_free_1 && next_head == tail)
                // Single allocation (port 0 only) empties the list
                valid <= 1'b0;
            else if (!do_alloc && do_alloc_1 && !do_free && !do_free_1 && next_head == tail)
                // Single allocation (port 1 only) empties the list
                valid <= 1'b0;
            else if (do_free || do_free_1)
                valid <= 1'b1;
        end
    end

endmodule : free_list