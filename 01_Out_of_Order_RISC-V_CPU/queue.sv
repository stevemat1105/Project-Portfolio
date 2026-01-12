// 2-WAY SUPERSCALAR QUEUE
//
// Supports dual enqueue and dual dequeue per cycle:
// - enq/enq_data: first enqueue
// - enq_1/enq_data_1: second enqueue (only if first succeeds)
// - deq/deq_data: first dequeue
// - deq_1/deq_data_1: second dequeue (only if first succeeds)
//
// Flow control signals:
// - full: cannot enqueue even 1 entry
// - almost_full: can enqueue 1 but not 2
// - empty: cannot dequeue even 1 entry
// - almost_empty: can dequeue 1 but not 2

module queue #(
    parameter WIDTH = 32,
    parameter DEPTH = 8
)
(
    input  logic        clk,
    input  logic        rst,

     
    // ENQUEUE INTERFACE - Instruction 0
     
    input  logic        enq,
    input  logic [WIDTH-1:0] enq_data,
    output logic        full,
    output logic        almost_full,

     
    // ENQUEUE INTERFACE - Instruction 1 (superscalar)
     
    input  logic        enq_1,
    input  logic [WIDTH-1:0] enq_data_1,

    input  logic        flush,

     
    // DEQUEUE INTERFACE - Instruction 0
     
    input  logic        deq,
    output logic [WIDTH-1:0] deq_data,
    output logic        empty,
    output logic        almost_empty,

     
    // DEQUEUE INTERFACE - Instruction 1 (superscalar)
     
    input  logic        deq_1,
    output logic [WIDTH-1:0] deq_data_1,

    // Area profiling
    output logic [$clog2(DEPTH):0] occupancy  // Current number of entries
);

    logic [WIDTH-1:0] storage [DEPTH];

    // head and tail pointers
    logic [$clog2(DEPTH)-1:0] head, tail;

    logic valid;

    // next pointers (for dual operations)
    logic [$clog2(DEPTH)-1:0] next_head, next_tail;
    logic [$clog2(DEPTH)-1:0] next_next_head, next_next_tail;

    localparam [$clog2(DEPTH)-1:0] MAX_INDEX = DEPTH - 1;

    // Single-step pointer increments
    assign next_head = (head == MAX_INDEX) ? '0 : head + 1'b1;
    assign next_tail = (tail == MAX_INDEX) ? '0 : tail + 1'b1;

    // Double-step pointer increments (for dual operations)
    assign next_next_head = (next_head == MAX_INDEX) ? '0 : next_head + 1'b1;
    assign next_next_tail = (next_tail == MAX_INDEX) ? '0 : next_tail + 1'b1;

     
    // OCCUPANCY AND FLOW CONTROL
    localparam [$clog2(DEPTH):0] ALMOST_FULL_COUNT = DEPTH - 1;
     

    // Count entries in queue
    logic [$clog2(DEPTH):0] count;
    always_comb begin
        if (!valid && head == tail)
            count = '0;
        else if (valid && head == tail)
            count = DEPTH[$clog2(DEPTH):0];
        else if (tail > head)
            count = ($clog2(DEPTH)+1)'(tail - head);
        else
            count = ($clog2(DEPTH)+1)'(DEPTH + tail - head);
    end

    assign empty        = (count == 0);
    assign almost_empty = (count == 1);
    assign full         = (count == DEPTH[$clog2(DEPTH):0]);
    assign almost_full  = (count == ALMOST_FULL_COUNT);
     
    // DEQUEUE DATA OUTPUTS
     
    assign deq_data   = storage[head];
    assign deq_data_1 = storage[next_head];

     
    // OPERATION CONTROL LOGIC
     

    // How many operations can we actually do?
    logic [1:0] can_enq_count;  // 0, 1, or 2 slots available
    logic [1:0] can_deq_count;  // 0, 1, or 2 entries available

    always_comb begin
        // Calculate how many we can enqueue
        if (full)
            can_enq_count = 2'd0;
        else if (almost_full)
            can_enq_count = 2'd1;
        else
            can_enq_count = 2'd2;

        // Calculate how many we can dequeue
        if (empty)
            can_deq_count = 2'd0;
        else if (almost_empty)
            can_deq_count = 2'd1;
        else
            can_deq_count = 2'd2;
    end

    // Actual operations - taking into account simultaneous enq/deq that frees space
    logic do_enq, do_enq_1, do_deq, do_deq_1;
    logic [1:0] actual_deq_count;
    logic [2:0] space_freed;  // 3 bits to avoid overflow when adding to can_enq_count
    logic [2:0] total_space;  // 3 bits: can_enq_count (max 2) + space_freed (max 2) = max 4

    always_comb begin
        // First, figure out how many dequeues
        do_deq   = deq && (can_deq_count >= 2'd1);
        do_deq_1 = deq_1 && do_deq && (can_deq_count >= 2'd2);
        actual_deq_count = {1'b0, do_deq} + {1'b0, do_deq_1};

        // Space freed by dequeues can be used for enqueues
        // Use 3 bits to avoid overflow: max value = 2
        space_freed = {1'b0, actual_deq_count};

        // Total available space = existing space + space freed by dequeues
        // Use 3 bits: max value = 2 + 2 = 4
        total_space = {1'b0, can_enq_count} + space_freed;

        // Now figure out how many enqueues (with freed space)
        if (enq && (total_space >= 3'd1)) begin
            do_enq = 1'b1;
            if (enq_1 && (total_space >= 3'd2))
                do_enq_1 = 1'b1;
            else
                do_enq_1 = 1'b0;
        end
        else begin
            do_enq   = 1'b0;
            do_enq_1 = 1'b0;
        end
    end

     
    // SEQUENTIAL LOGIC
     
    logic [$clog2(DEPTH)-1:0] new_head, new_tail;

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            head  <= '0;
            tail  <= '0;
            valid <= 1'b0;
        end
        else begin
             
            // HEAD POINTER UPDATE (dequeue)
             
            case ({do_deq_1, do_deq})
                2'b01: head <= next_head;           // Single dequeue
                2'b11: head <= next_next_head;      // Dual dequeue
                default: ;                          // No dequeue
            endcase

             
            // TAIL POINTER UPDATE AND WRITE (enqueue)
             
            case ({do_enq_1, do_enq})
                2'b01: begin  // Single enqueue
                    storage[tail] <= enq_data;
                    tail <= next_tail;
                end
                2'b11: begin  // Dual enqueue
                    storage[tail] <= enq_data;
                    storage[next_tail] <= enq_data_1;
                    tail <= next_next_tail;
                end
                default: ;    // No enqueue
            endcase

             
            // VALID FLAG UPDATE (for full/empty detection when head == tail)
             
            // Net change = enqueues - dequeues
            // valid becomes 1 when queue becomes full (tail catches up to head)
            // valid becomes 0 when queue becomes empty (head catches up to tail)

            // Calculate new head and tail for comparison
            case ({do_deq_1, do_deq})
                2'b01: new_head = next_head;
                2'b11: new_head = next_next_head;
                default: new_head = head;
            endcase

            case ({do_enq_1, do_enq})
                2'b01: new_tail = next_tail;
                2'b11: new_tail = next_next_tail;
                default: new_tail = tail;
            endcase

            // Update valid flag based on whether we're becoming full or empty
            if (new_head == new_tail) begin
                // Pointers equal - either full or empty
                // Full if we did more enqueues than dequeues (net positive)
                // Empty if we did more dequeues than enqueues (net negative)
                logic [1:0] enq_count, deq_count;
                enq_count = {1'b0, do_enq} + {1'b0, do_enq_1};
                deq_count = {1'b0, do_deq} + {1'b0, do_deq_1};

                if (enq_count > deq_count)
                    valid <= 1'b1;  // Became full
                else if (deq_count > enq_count)
                    valid <= 1'b0;  // Became empty
                // else: net zero change, keep current valid state
            end
        end
    end

    // Compute occupancy for area profiling (use count already computed)
    assign occupancy = count;

endmodule : queue