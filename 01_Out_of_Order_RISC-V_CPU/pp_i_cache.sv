module pp_icache
import types::*;
(
    input   logic           clk,
    input   logic           rst,

    // UFP (CPU side) - READ ONLY - Port 0
    input   logic   [31:0]  ufp_addr,
    input   logic   [3:0]   ufp_rmask,
    output  logic   [31:0]  ufp_rdata,
    output  logic           ufp_resp,
    input   logic           ufp_flush,

    // UFP Port 1 (superscalar - same cacheline as port 0)
    input   logic   [31:0]  ufp_addr_1,
    input   logic   [3:0]   ufp_rmask_1,
    output  logic   [31:0]  ufp_rdata_1,
    output  logic           ufp_resp_1,

    // DFP (Memory side) - READ ONLY (no writeback)
    output  logic   [31:0]  dfp_addr,
    output  logic           dfp_read,
    input   logic   [255:0] dfp_rdata,
    input   logic           dfp_resp
);

    // PIPELINE REGS: S1 -> S2
    typedef struct packed {
        logic           valid;
        logic   [31:0]  addr;
        logic   [3:0]   rmask;
    } s2_reg_t;

    s2_reg_t s2_reg, s2_reg_next;

    // MISS HANDLING STATE
    typedef enum logic {
        PP_NORMAL   = 1'b0,     // Normal pipeline operation
        PP_ALLOCATE = 1'b1      // Fetching new line from memory
    } pp_state_t;

    pp_state_t miss_state, miss_state_next;

    // data arrays (256-  cachelines)
    logic [3:0]   data_csb;
    logic [3:0]   data_web;
    logic [3:0]   sram_addr;
    logic [255:0] data_din  [4];
    logic [255:0] data_dout [4];
    logic [31:0]  data_wmask [4];

    // tag arrays (23-  tags: addr[31:9])
    logic [3:0]   tag_csb;
    logic [3:0]   tag_web;
    logic [22:0]  tag_din;
    logic [22:0]  tag_dout [4];

    // valid arrays (FF-based, 1-  each)
    logic [3:0]   valid_csb;
    logic [3:0]   valid_web;
    logic         valid_din;
    logic         valid_dout [4];

    // PLRU array (3-  per set)
    logic         plru_csb;
    logic         plru_web;
    logic [2:0]   plru_din;
    logic [2:0]   plru_dout;

    // HIT/MISS LOGIC
    logic [3:0]   way_hit;
    logic         cache_hit;
    logic [1:0]   hit_way;
    logic [1:0]   lru_way;
    logic [1:0]   evict_way;

    logic         stall;
    logic         internal_stall;
    logic         s1_valid;
    logic         s2_valid;
    logic         miss_handling;
    logic         arrays_en;

    // Serviced tracking - prevents double resp after allocate
    logic         s2_serviced_r;        // request in S2 has been serviced
    logic         allocate_complete;    // allocate finished this cycle
    logic         allocate_recovery_r;  // recovery cycle after allocate (for SRAM write)

    // DFP request tracking - detect stale resp after flush
    logic [31:0]  dfp_req_addr_r;       // address of in-flight DFP request
    logic         dfp_req_valid_r;      // DFP request is in-flight
    logic         dfp_addr_match;       // DFP response matches current request

    // address breakdown for S2
    logic [22:0]  s2_tag;
    logic [3:0]   s2_index;
    logic [4:0]   s2_offset;
    logic [2:0]   s2_word_offset;

    // Port 1 address tracking for same-cacheline access
    logic [31:0]  s2_addr_1_r;
    logic [3:0]   s2_rmask_1_r;
    logic [2:0]   s2_word_offset_1;
    logic         s2_same_cacheline;

    logic [63:0] perf_cycles;
    logic [63:0] perf_requests;
    logic [63:0] perf_hits;
    logic [63:0] perf_misses;
    logic [63:0] perf_responses;

    logic [63:0] perf_stall_cycles;
    logic [63:0] perf_stall_internal;
    logic [63:0] perf_stall_alloc_complete;
    logic [63:0] perf_stall_alloc_recovery;
    logic [63:0] perf_stall_miss_handling;

    logic [63:0] perf_allocate_cycles;

    logic [63:0] perf_s1_active;
    logic [63:0] perf_s2_active;
    logic [63:0] perf_s2_idle;

    logic [63:0] perf_back_to_back_hits;
    logic [63:0] perf_flush_count;
    logic [63:0] perf_flush_during_miss;

    logic [63:0] perf_recovery_avoided;

    logic [63:0] perf_total_hit_latency;
    logic [63:0] perf_total_miss_latency;
    logic [63:0] perf_miss_start_cycle;
    logic        perf_in_miss;

    logic [63:0] perf_way0_hits;
    logic [63:0] perf_way1_hits;
    logic [63:0] perf_way2_hits;
    logic [63:0] perf_way3_hits;

    logic [63:0] perf_evict_valid_clean;
    logic [63:0] perf_evict_invalid;

    generate
        for (genvar i = 0; i < 4; i++) begin : arrays
            mp_cache_data_array data_array (
                .clk0       (clk & !data_csb[i]),
                .csb0       (data_csb[i]),
                .web0       (data_web[i]),
                .wmask0     (data_wmask[i]),
                .addr0      (sram_addr),
                .din0       (data_din[i]),
                .dout0      (data_dout[i])
            );

            mp_cache_tag_array tag_array (
                .clk0       (clk),
                .csb0       (tag_csb[i]),
                .web0       (tag_web[i]),
                .addr0      (sram_addr),
                .din0       (tag_din),
                .dout0      (tag_dout[i])
            );

            sp_ff_array valid_array (
                .clk0       (clk),
                .rst0       (rst),
                .csb0       (valid_csb[i]),
                .web0       (valid_web[i]),
                .addr0      (sram_addr),
                .din0       (valid_din),
                .dout0      (valid_dout[i])
            );
        end
    endgenerate

    sp_ff_array #(
        .WIDTH(3)
    ) plru_array (
        .clk0       (clk),
        .rst0       (rst),
        .csb0       (plru_csb),
        .web0       (plru_web),
        .addr0      (sram_addr),
        .din0       (plru_din),
        .dout0      (plru_dout)
    );

    // S2 ADDRESS BREAKDOWN
    assign s2_tag         = s2_reg.addr[31:9];
    assign s2_index       = s2_reg.addr[8:5];
    assign s2_offset      = s2_reg.addr[4:0];
    assign s2_word_offset = s2_reg.addr[4:2];

    assign s1_valid = (ufp_rmask != 4'b0);
    assign s2_valid = s2_reg.valid;
    assign miss_handling = (miss_state != PP_NORMAL);

    assign dfp_addr_match = dfp_req_valid_r && (dfp_req_addr_r[31:5] == {s2_tag, s2_index});

    assign allocate_complete = (miss_state == PP_ALLOCATE) && dfp_resp && dfp_addr_match;

    // internal stall: S2 has valid unserviced request that can't be served
    assign internal_stall = s2_valid && (!s2_serviced_r || miss_handling) && (!cache_hit || miss_handling) && !allocate_complete;

    assign stall = internal_stall || allocate_complete || (miss_handling && !allocate_complete);

    // SERVICED TRACKING (prevents double resp after allocate)
    always_ff @(posedge clk) begin
        if (rst) begin
            s2_serviced_r <= 1'b0;
            allocate_recovery_r <= 1'b0;
        end
        else begin
            allocate_recovery_r <= allocate_complete;

            if (!stall)
                s2_serviced_r <= 1'b0;
            else if (ufp_flush && (miss_handling || (s2_valid && !cache_hit && !s2_serviced_r)))
                s2_serviced_r <= 1'b1;
            else if (dfp_resp && miss_state == PP_ALLOCATE && !dfp_addr_match)
                s2_serviced_r <= 1'b0;
            else if (ufp_resp)
                s2_serviced_r <= 1'b1;
        end
    end

    // DFP REQUEST TRACKING (detect stale responses after flush)
    always_ff @(posedge clk) begin
        if (rst) begin
            dfp_req_addr_r <= 32'b0;
            dfp_req_valid_r <= 1'b0;
        end
        else begin
            if (dfp_read && !dfp_req_valid_r) begin
                dfp_req_addr_r <= dfp_addr;
                dfp_req_valid_r <= 1'b1;
            end
            else if (dfp_resp)
                dfp_req_valid_r <= 1'b0;
        end
    end

    // HIT/MISS DETECTION
    always_comb begin
        for (integer i = 0; i < 4; i++) begin
            way_hit[i] = (tag_dout[i] == s2_tag) && valid_dout[i];
        end

        cache_hit = |way_hit;

        hit_way = way_hit[3] ? 2'd3 : way_hit[2] ? 2'd2 : way_hit[1] ? 2'd1 : 2'd0;
    end

    // PLRU DECODE
    always_comb begin
        unique casez (plru_dout)
            3'b00? : lru_way = 2'b00;
            3'b01? : lru_way = 2'b01;
            3'b1?0 : lru_way = 2'b10;
            3'b1?1 : lru_way = 2'b11;
            default: lru_way = 2'b00;
        endcase
    end

    function automatic logic [2:0] update_plru(input logic [2:0] curr, input logic [1:0] way_access);
        logic [2:0] updated;
        updated = curr;
        case (way_access)
            2'b00: begin updated[2] = 1'b1; updated[1] = 1'b1; end
            2'b01: begin updated[2] = 1'b1; updated[1] = 1'b0; end
            2'b10: begin updated[2] = 1'b0; updated[0] = 1'b1; end
            2'b11: begin updated[2] = 1'b0; updated[0] = 1'b0; end
        endcase
        return updated;
    endfunction

    always_ff @(posedge clk) begin
        if (rst)
            evict_way <= 2'b00;
        else if (s2_valid && !cache_hit && miss_state == PP_NORMAL && !s2_serviced_r)
            evict_way <= lru_way;
    end

    // MISS STATE MACHINE
    always_ff @(posedge clk) begin
        if (rst)
            miss_state <= PP_NORMAL;
        else
            miss_state <= miss_state_next;
    end

    always_comb begin
        miss_state_next = miss_state;

        case (miss_state)
            PP_NORMAL: begin
                if (s2_valid && !cache_hit && !s2_serviced_r)
                    miss_state_next = PP_ALLOCATE;
            end

            PP_ALLOCATE: begin
                if (allocate_complete)
                    miss_state_next = PP_NORMAL;
            end

            default: miss_state_next = PP_NORMAL;
        endcase
    end

    // SRAM ADDRESS SELECTION
    always_comb begin
        if (miss_handling)
            sram_addr = s2_index;
        else if (stall)
            sram_addr = s2_index;
        else
            sram_addr = ufp_addr[8:5];
    end

    always_comb begin
        data_csb  = 4'b0000;
        tag_csb   = 4'b0000;
        valid_csb = 4'b0000;
        plru_csb  = 1'b0;

        data_web  = 4'b1111;
        tag_web   = 4'b1111;
        valid_web = 4'b1111;
        plru_web  = 1'b1;

        tag_din   = s2_tag;
        valid_din = 1'b1;
        plru_din  = plru_dout;

        for (integer i = 0; i < 4; i++) begin
            data_din[i]   = 256'b0;
            data_wmask[i] = 32'b0;
        end

        arrays_en = (!stall && s1_valid) || miss_handling || (s2_valid && cache_hit);

        if (!arrays_en) begin
            data_csb  = 4'b1111;
            tag_csb   = 4'b1111;
            valid_csb = 4'b1111;
            plru_csb  = 1'b1;
        end

        // S2 HIT
        if (s2_valid && cache_hit && !miss_handling && !s2_serviced_r) begin
            plru_web = 1'b0;
            plru_din = update_plru(plru_dout, hit_way);
        end

        // ALLOCATE (write new cacheline from memory)
        if (miss_state == PP_ALLOCATE && dfp_resp) begin
            data_web[evict_way]  = 1'b0;
            tag_web[evict_way]   = 1'b0;
            valid_web[evict_way] = 1'b0;

            data_din[evict_way] = dfp_rdata;
            data_wmask[evict_way] = 32'hFFFFFFFF;
        end
    end

    // DFP (MEMORY) INTF
    always_comb begin
        dfp_addr = 32'b0;
        dfp_read = 1'b0;

        if (miss_state == PP_ALLOCATE) begin
            dfp_read = 1'b1;
            dfp_addr = {s2_tag, s2_index, 5'b0};
        end
    end

    always_comb begin
        if (stall)
            s2_reg_next = s2_reg;
        else begin
            s2_reg_next.valid = s1_valid;
            s2_reg_next.addr  = ufp_addr;
            s2_reg_next.rmask = ufp_rmask;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            s2_reg.valid <= 1'b0;
            s2_reg.addr  <= 32'b0;
            s2_reg.rmask <= 4'b0;
            s2_addr_1_r  <= 32'b0;
            s2_rmask_1_r <= 4'b0;
        end
        else begin
            s2_reg <= s2_reg_next;
            // Track port 1 address for same-cacheline access
            if (!stall) begin
                s2_addr_1_r  <= ufp_addr_1;
                s2_rmask_1_r <= ufp_rmask_1;
            end
        end
    end

    // Port 1 calculations
    assign s2_word_offset_1 = s2_addr_1_r[4:2];
    assign s2_same_cacheline = (s2_reg.addr[31:5] == s2_addr_1_r[31:5]);

    // UFP OUTPUT (resp to CPU)
    assign ufp_resp = !s2_serviced_r && !allocate_recovery_r && ((s2_valid && cache_hit && !miss_handling) || allocate_complete);

    always_comb begin
        ufp_rdata = 32'b0;

        if (s2_valid && cache_hit && !miss_handling) begin
            logic [31:0] word_data;
            word_data = data_dout[hit_way][s2_word_offset*32 +: 32];

            for (integer i = 0; i < 4; i++) begin
                if (s2_reg.rmask[i])
                    ufp_rdata[i*8 +: 8] = word_data[i*8 +: 8];
                else
                    ufp_rdata[i*8 +: 8] = 8'b0;
            end
        end
        else if (allocate_complete) begin
            logic [31:0] word_data;
            word_data = dfp_rdata[s2_word_offset*32 +: 32];

            for (integer i = 0; i < 4; i++) begin
                if (s2_reg.rmask[i])
                    ufp_rdata[i*8 +: 8] = word_data[i*8 +: 8];
                else
                    ufp_rdata[i*8 +: 8] = 8'b0;
            end
        end
    end

     
    // UFP PORT 1 OUTPUT (superscalar - same cacheline as port 0)
     
    // Port 1 responds only if:
    // - Port 0 responds (hit or allocate complete)
    // - Port 1 is in the same cacheline
    // - Port 1 has a valid rmask
    assign ufp_resp_1 = ufp_resp && s2_same_cacheline && (s2_rmask_1_r != 4'b0);

    always_comb begin
        ufp_rdata_1 = 32'b0;

        if (s2_valid && cache_hit && !miss_handling && s2_same_cacheline) begin
            logic [31:0] word_data_1;
            word_data_1 = data_dout[hit_way][s2_word_offset_1*32 +: 32];

            for (integer i = 0; i < 4; i++) begin
                if (s2_rmask_1_r[i])
                    ufp_rdata_1[i*8 +: 8] = word_data_1[i*8 +: 8];
                else
                    ufp_rdata_1[i*8 +: 8] = 8'b0;
            end
        end
        else if (allocate_complete && s2_same_cacheline) begin
            logic [31:0] word_data_1;
            word_data_1 = dfp_rdata[s2_word_offset_1*32 +: 32];

            for (integer i = 0; i < 4; i++) begin
                if (s2_rmask_1_r[i])
                    ufp_rdata_1[i*8 +: 8] = word_data_1[i*8 +: 8];
                else
                    ufp_rdata_1[i*8 +: 8] = 8'b0;
            end
        end
    end

endmodule : pp_icache