// Next-Line Prefetcher for I-Cache

module i_prefetcher
import types::*;
#(
    parameter NUM_ENTRIES = 2
)
(
    input   logic           clk,
    input   logic           rst,
    input   logic           flush,

    // I-Cache side (upstream) - from pp_icache DFP
    input   logic   [31:0]  cache_addr,
    input   logic           cache_read,
    output  logic   [255:0] cache_rdata,
    output  logic           cache_resp,

    // Adapter side (downstream) - to cacheline_adapter
    output  logic   [31:0]  mem_addr,
    output  logic           mem_read,
    input   logic   [255:0] mem_rdata,
    input   logic           mem_resp,
    input   logic   [31:0]  mem_raddr   // Response address for verification
);

    // prefetch buffer entry
    typedef struct packed {
        logic           valid;
        logic   [26:0]  tag;        // addr[31:5] - cacheline tag
        logic   [255:0] data;       // cached data
    } pf_entry_t;

    pf_entry_t pf_buffer [NUM_ENTRIES];

    // FSM
    typedef enum logic [1:0] {
        IDLE,
        DEMAND_WAIT,
        PREFETCH_WAIT
    } state_t;

    state_t state, state_next;

    logic [31:0]  demand_addr_r;
    logic [31:0]  prefetch_addr_r;

    logic [$clog2(NUM_ENTRIES)-1:0] alloc_ptr;
    logic [$clog2(NUM_ENTRIES)-1:0] pf_hit_idx;
    logic         pf_buffer_hit;

    logic         prefetch_in_buffer;

    logic [31:0]  next_line_addr;
    assign next_line_addr = {demand_addr_r[31:5] + 27'd1, 5'b0};

    logic         resp_matches_demand;
    logic         resp_matches_prefetch;
    logic         resp_matches_cache_addr;

    assign resp_matches_demand   = (mem_raddr[31:5] == demand_addr_r[31:5]);
    assign resp_matches_prefetch = (mem_raddr[31:5] == prefetch_addr_r[31:5]);
    assign resp_matches_cache_addr = (mem_raddr[31:5] == cache_addr[31:5]);

    always_comb begin
        pf_buffer_hit = 1'b0;
        pf_hit_idx = '0;

        for (integer i = 0; i < NUM_ENTRIES; i++) begin
            if (pf_buffer[i].valid && pf_buffer[i].tag == cache_addr[31:5]) begin
                pf_buffer_hit = 1'b1;
                pf_hit_idx = ($clog2(NUM_ENTRIES))'($unsigned(i));
            end
        end
    end

    always_comb begin
        prefetch_in_buffer = 1'b0;

        for (integer i = 0; i < NUM_ENTRIES; i++) begin
            if (pf_buffer[i].valid && pf_buffer[i].tag == prefetch_addr_r[31:5]) begin
                prefetch_in_buffer = 1'b1;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (rst || flush)
            state <= IDLE;
        else
            state <= state_next;
    end

    always_comb begin
        state_next = state;

        case (state)
            IDLE: begin
                if (cache_read && !pf_buffer_hit)
                    state_next = DEMAND_WAIT;
            end

            DEMAND_WAIT: begin
                if (mem_resp && resp_matches_demand)
                    state_next = PREFETCH_WAIT;
            end

            PREFETCH_WAIT: begin
                if (cache_read && !pf_buffer_hit)
                    state_next = DEMAND_WAIT;
                else if (mem_resp && resp_matches_prefetch)
                    state_next = IDLE;
                else if (prefetch_in_buffer)
                    state_next = IDLE;
            end

            default: state_next = IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            demand_addr_r <= 32'b0;
            prefetch_addr_r <= 32'b0;
        end
        else if (flush) begin
            demand_addr_r <= 32'b0;
            prefetch_addr_r <= 32'b0;
        end
        else begin
            if (state == IDLE && cache_read && !pf_buffer_hit)
                demand_addr_r <= cache_addr;
            else if (state == PREFETCH_WAIT && cache_read && !pf_buffer_hit)
                demand_addr_r <= cache_addr;

            if (state == DEMAND_WAIT && mem_resp && resp_matches_demand)
                prefetch_addr_r <= next_line_addr;
        end
    end

    always_comb begin
        mem_addr = 32'b0;
        mem_read = 1'b0;

        case (state)
            IDLE: begin
                if (cache_read && !pf_buffer_hit) begin
                    mem_addr = cache_addr;
                    mem_read = 1'b1;
                end
            end

            DEMAND_WAIT: begin
                mem_addr = demand_addr_r;
                mem_read = 1'b1;
            end

            PREFETCH_WAIT: begin
                if (cache_read && !pf_buffer_hit) begin
                    mem_addr = cache_addr;
                    mem_read = 1'b1;
                end
                else if (!prefetch_in_buffer) begin
                    mem_addr = prefetch_addr_r;
                    mem_read = 1'b1;
                end
            end

            default: begin
                mem_addr = 32'b0;
                mem_read = 1'b0;
            end
        endcase
    end

    always_comb begin
        cache_rdata = 256'b0;
        cache_resp = 1'b0;

        if (pf_buffer_hit && cache_read) begin
            cache_rdata = pf_buffer[pf_hit_idx].data;
            cache_resp = 1'b1;
        end
        else if (state == DEMAND_WAIT && mem_resp && resp_matches_demand) begin
            cache_rdata = mem_rdata;
            cache_resp = 1'b1;
        end
        else if (state == PREFETCH_WAIT && mem_resp && cache_read && resp_matches_cache_addr) begin
            cache_rdata = mem_rdata;
            cache_resp = 1'b1;
        end
    end

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            for (integer i = 0; i < NUM_ENTRIES; i++) begin
                pf_buffer[i].valid <= 1'b0;
                pf_buffer[i].tag <= 27'b0;
                pf_buffer[i].data <= 256'b0;
            end
            alloc_ptr <= '0;
        end
        else begin
            if (pf_buffer_hit && cache_read)
                pf_buffer[pf_hit_idx].valid <= 1'b0;

            if (state == PREFETCH_WAIT && mem_resp && resp_matches_prefetch) begin
                if (!(cache_read && resp_matches_cache_addr)) begin
                    pf_buffer[alloc_ptr].valid <= 1'b1;
                    pf_buffer[alloc_ptr].tag <= prefetch_addr_r[31:5];
                    pf_buffer[alloc_ptr].data <= mem_rdata;
                    alloc_ptr <= alloc_ptr + 1'b1;
                end
            end
        end
    end

endmodule : i_prefetcher