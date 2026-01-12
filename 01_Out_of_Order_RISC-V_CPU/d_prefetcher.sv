module d_stride_prefetcher
#(
    parameter integer unsigned NUM_ENTRIES     = 8,
    parameter integer unsigned PREFETCH_DEGREE = 1,
    parameter integer unsigned MAX_STRIDE      = 1024
)
(
    input  logic        clk,
    input  logic        rst,
    input  logic        flush,

    // UPSTREAM: D-CACHE DFP
    input  logic [31:0]  cache_addr,
    input  logic         cache_read,
    input  logic         cache_write,
    output logic [255:0] cache_rdata,
    output logic         cache_resp,

    // DOWNSTREAM: Adapter-D
    output logic [31:0]  mem_addr,
    output logic         mem_read,
    output logic         mem_write,
    input  logic [255:0] mem_rdata,
    input  logic         mem_resp,
    input  logic [31:0]  mem_raddr,

    // Prefetcher statistics
    output logic [31:0]  stat_demand_loads,
    output logic [31:0]  stat_demand_stores,
    output logic [31:0]  stat_prefetch_issued,
    output logic [31:0]  stat_prefetch_filled,
    output logic [31:0]  stat_prefetch_hits
);

    typedef struct packed {
        logic         valid;
        logic [26:0]  tag;
        logic [255:0] data;
    } pf_entry_t;

    pf_entry_t pf_buf [NUM_ENTRIES];

    typedef enum logic [1:0] {
        S_IDLE,
        S_DEMAND,
        S_PREFETCH
    } st_t;

    st_t state, state_n;

    logic [31:0] last_addr;
    logic [31:0] stride;
    logic        stride_valid;

    logic [31:0] last_addr_next;
    logic [31:0] stride_next;
    logic        stride_valid_next;

    logic [1:0]  stride_conf;
    logic [1:0]  stride_conf_next;

    logic [31:0] pf_addr_r;
    logic [31:0] pf_addr_next;

    logic [31:0] demand_addr_r;
    logic        demand_is_read;
    logic        demand_is_write;

    logic                           pf_hit;
    logic [$clog2(NUM_ENTRIES)-1:0] pf_idx;
    logic                           pf_hit_read;

    logic is_demand_read_miss;

    logic [31:0] tmp_i;

    logic        pf_pending, pf_pending_n;
    logic [31:0] pf_issue_addr, pf_issue_addr_n;

    always_comb begin
        pf_hit = 1'b0;
        pf_idx = '0;

        for (integer i = 0; i < NUM_ENTRIES; i++) begin
            tmp_i = i;
            if (pf_buf[i].valid && pf_buf[i].tag == cache_addr[31:5]) begin
                pf_hit = 1'b1;
                pf_idx = tmp_i[$clog2(NUM_ENTRIES)-1:0];
            end
        end
    end

    assign pf_hit_read = pf_hit & cache_read;

    assign is_demand_read_miss =
        (state == S_IDLE) &&
        cache_read && !cache_write &&
        !pf_hit_read;

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            state         <= S_IDLE;
            pf_pending    <= 1'b0;
            pf_issue_addr <= 32'b0;
        end
        else begin
            state         <= state_n;
            pf_pending    <= pf_pending_n;
            pf_issue_addr <= pf_issue_addr_n;
        end
    end

    always_comb begin
        state_n         = state;
        pf_pending_n    = pf_pending;
        pf_issue_addr_n = pf_issue_addr;

        unique case (state)
            S_IDLE: begin
                if ((cache_read || cache_write) && !pf_hit_read) begin
                    state_n = S_DEMAND;
                end
                else if (pf_pending) begin
                    state_n      = S_PREFETCH;
                    pf_pending_n = 1'b0;
                end
            end

            S_DEMAND: begin
                if (mem_resp && (mem_raddr[31:5] == demand_addr_r[31:5])) begin
                    if (demand_is_read && stride_valid) begin
                        pf_pending_n    = 1'b1;
                        pf_issue_addr_n = pf_addr_r; 
                    end
                    state_n = S_IDLE;
                end
            end

            S_PREFETCH: begin
                if (mem_resp && (mem_raddr[31:5] == pf_issue_addr[31:5])) begin
                    state_n = S_IDLE;
                end
            end

            default: state_n = S_IDLE;
        endcase
    end

    always_comb begin
        last_addr_next    = last_addr;
        stride_next       = stride;
        stride_valid_next = stride_valid;
        stride_conf_next  = stride_conf;
        pf_addr_next      = pf_addr_r;

        if (cache_read) begin
            if (last_addr == 32'b0) begin
                last_addr_next    = cache_addr;
                stride_conf_next  = 2'd0;
                stride_valid_next = 1'b0;
            end
            else if (cache_addr != last_addr) begin
                logic signed [31:0] delta_s;
                logic        [31:0] delta_abs;

                delta_s   = $signed(cache_addr) - $signed(last_addr);
                delta_abs = delta_s[31] ? (~delta_s + 32'd1) : delta_s;

                last_addr_next = cache_addr;

                if (delta_abs != 32'd0 && delta_abs <= MAX_STRIDE) begin
                    logic signed [31:0] stride_s;
                    stride_s = $signed(stride);

                    if (stride_valid && (delta_s == stride_s)) begin
                        if (stride_conf_next != 2'd3) begin
                            stride_conf_next = stride_conf_next + 2'd1;
                        end
                    end
                    else begin
                        stride_next      = delta_s;
                        stride_conf_next = 2'd2;
                    end
                    stride_valid_next = 1'b1;
                end
            end

            if (stride_valid_next) begin
                logic [31:0] raw_pf_addr;

                raw_pf_addr = cache_addr + (stride_next * PREFETCH_DEGREE);

                if (raw_pf_addr[31:12] == cache_addr[31:12]) begin
                    pf_addr_next = raw_pf_addr;
                end
                else begin
                    pf_addr_next = pf_addr_r;
                end
            end
            else begin
                logic [26:0] next_line;
                next_line    = cache_addr[31:5] + 27'd1;
                pf_addr_next = {next_line, 5'b0};
            end
        end

        if (rst || flush) begin
            last_addr_next    = 32'b0;
            stride_next       = 32'b0;
            stride_valid_next = 1'b0;
            stride_conf_next  = 2'd0;
            pf_addr_next      = 32'b0;
        end
    end

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            last_addr    <= 32'b0;
            stride       <= 32'b0;
            stride_valid <= 1'b0;
            stride_conf  <= 2'd0;
            pf_addr_r    <= 32'b0;
        end
        else begin
            last_addr    <= last_addr_next;
            stride       <= stride_next;
            stride_valid <= stride_valid_next;
            stride_conf  <= stride_conf_next;
            pf_addr_r    <= pf_addr_next;
        end
    end

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            demand_addr_r   <= 32'b0;
            demand_is_read  <= 1'b0;
            demand_is_write <= 1'b0;
        end
        else begin
            if (state == S_IDLE && (cache_read || cache_write) && !pf_hit_read) begin
                demand_addr_r   <= cache_addr;
                demand_is_read  <= cache_read;
                demand_is_write <= cache_write;
            end
        end
    end

    always_comb begin
        mem_addr  = 32'b0;
        mem_read  = 1'b0;
        mem_write = 1'b0;

        unique case (state)
            S_IDLE: begin
                if ((cache_read || cache_write) && !pf_hit_read) begin
                    mem_addr  = cache_addr;
                    mem_read  = cache_read;
                    mem_write = cache_write;
                end
            end

            S_DEMAND: begin
                mem_addr  = demand_addr_r;
                mem_read  = demand_is_read;
                mem_write = demand_is_write;
            end

            S_PREFETCH: begin
                mem_addr  = pf_issue_addr;
                mem_read  = 1'b1;
                mem_write = 1'b0;
            end

            default: ;
        endcase
    end

    always_comb begin
        cache_rdata = 256'b0;
        cache_resp  = 1'b0;

        if (pf_hit_read) begin
            cache_rdata = pf_buf[pf_idx].data;
            cache_resp  = 1'b1;
        end
        else if (state == S_DEMAND &&
                 mem_resp &&
                 (mem_raddr[31:5] == demand_addr_r[31:5])) begin
            if (demand_is_read) begin
                cache_rdata = mem_rdata;
            end
            cache_resp = 1'b1;
        end
    end

    logic [$clog2(NUM_ENTRIES)-1:0] alloc_ptr;

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            alloc_ptr <= '0;
            for (integer i = 0; i < NUM_ENTRIES; i++) begin
                pf_buf[i].valid <= 1'b0;
                pf_buf[i].tag   <= 27'b0;
                pf_buf[i].data  <= 256'b0;
            end
        end
        else begin
            if (pf_hit_read) begin
                pf_buf[pf_idx].valid <= 1'b0;
            end
            if (state == S_PREFETCH &&
                mem_resp &&
                (mem_raddr[31:5] == pf_issue_addr[31:5]) &&
                !(cache_read && !pf_hit_read)) begin

                pf_buf[alloc_ptr].valid <= 1'b1;
                pf_buf[alloc_ptr].tag   <= pf_issue_addr[31:5];
                pf_buf[alloc_ptr].data  <= mem_rdata;
                alloc_ptr               <= alloc_ptr + 1'b1;
            end
        end
    end

    // Statistics counters
    always_ff @(posedge clk) begin
        if (rst) begin
            stat_demand_loads     <= 32'd0;
            stat_demand_stores    <= 32'd0;
            stat_prefetch_issued  <= 32'd0;
            stat_prefetch_filled  <= 32'd0;
            stat_prefetch_hits    <= 32'd0;
        end
        else begin
            if (state == S_IDLE && cache_read && !pf_hit_read) begin
                stat_demand_loads <= stat_demand_loads + 32'd1;
            end

            if (state == S_IDLE && cache_write) begin
                stat_demand_stores <= stat_demand_stores + 32'd1;
            end

            if (state == S_PREFETCH &&
                mem_read && !mem_write &&
                !(cache_read || cache_write)) begin
                stat_prefetch_issued <= stat_prefetch_issued + 32'd1;
            end

            if (state == S_PREFETCH &&
                mem_resp &&
                (mem_raddr[31:5] == pf_issue_addr[31:5])) begin
                stat_prefetch_filled <= stat_prefetch_filled + 32'd1;
            end

            if (pf_hit_read && cache_resp) begin
                stat_prefetch_hits <= stat_prefetch_hits + 32'd1;
            end
        end
    end

endmodule
