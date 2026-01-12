module pp_dcache
import types::*;
(
    input   logic           clk,
    input   logic           rst,

    // UFP (CPU side)
    input   logic   [31:0]  ufp_addr,
    input   logic   [3:0]   ufp_rmask,
    input   logic   [3:0]   ufp_wmask,
    output  logic   [31:0]  ufp_rdata,
    input   logic   [31:0]  ufp_wdata,
    output  logic           ufp_resp,
    input   logic           ufp_flush,

    // DFP (Memory side)
    output  logic   [31:0]  dfp_addr,
    output  logic           dfp_read,
    output  logic           dfp_write,
    input   logic   [255:0] dfp_rdata,
    output  logic   [255:0] dfp_wdata,
    input   logic           dfp_resp
);

    // PIPELINE REGS: S1 -> S2
    typedef struct packed {
        logic           valid;
        logic   [31:0]  addr;
        logic   [3:0]   rmask;
        logic   [3:0]   wmask;
        logic   [31:0]  wdata;
    } s2_reg_t;

    s2_reg_t s2_reg, s2_reg_next;

    // MISS HANDLING STATE
    typedef enum logic [1:0] {
        PP_NORMAL    = 2'b00,   // Normal pipeline operation
        PP_WRITEBACK = 2'b01,   // Writing back dirty line
        PP_ALLOCATE  = 2'b10    // Fetching new line from memory
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

    // valid/dirty arrays (FF-based, 1-  each)
    logic [3:0]   valid_csb;
    logic [3:0]   valid_web;
    logic         valid_din;
    logic         valid_dout [4];

    logic [3:0]   dirty_csb;
    logic [3:0]   dirty_web;
    logic         dirty_din;
    logic         dirty_dout [4];

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
    logic [1:0]   evict_way;    // saved LRU way for miss handling

    logic         stall;
    logic         internal_stall;
    logic         s1_valid;
    logic         s2_valid;
    logic         s2_is_read;
    logic         s2_is_write;
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

            sp_ff_array dirty_array (
                .clk0       (clk),
                .rst0       (rst),
                .csb0       (dirty_csb[i]),
                .web0       (dirty_web[i]),
                .addr0      (sram_addr),
                .din0       (dirty_din),
                .dout0      (dirty_dout[i])
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

    assign s1_valid = (ufp_rmask != 4'b0) || (ufp_wmask != 4'b0);
    assign s2_valid = s2_reg.valid;
    assign s2_is_read = s2_reg.valid && (s2_reg.rmask != 4'b0);
    assign s2_is_write = s2_reg.valid && (s2_reg.wmask != 4'b0);
    assign miss_handling = (miss_state != PP_NORMAL);

    assign dfp_addr_match = dfp_req_valid_r && (dfp_req_addr_r[31:5] == {s2_tag, s2_index});

    assign allocate_complete = (miss_state == PP_ALLOCATE) && dfp_resp && dfp_addr_match;

    // internal stall: S2 has valid unserviced request that can't be served
    assign internal_stall = s2_valid && (!s2_serviced_r || miss_handling) && (!cache_hit || miss_handling) && !allocate_complete;

    // write hit detection - for stall and SRAM address selection
    logic write_hit;
    assign write_hit = s2_valid && cache_hit && s2_is_write && !miss_handling && !s2_serviced_r;

    logic write_hit_needs_stall;
    assign write_hit_needs_stall = write_hit && s1_valid;

    assign stall = internal_stall || allocate_complete || write_hit_needs_stall || (miss_handling && !allocate_complete);

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
            if ((dfp_read || dfp_write) && !dfp_req_valid_r) begin
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
                if (s2_valid && !cache_hit && !s2_serviced_r) begin
                    if (valid_dout[lru_way] && dirty_dout[lru_way])
                        miss_state_next = PP_WRITEBACK;
                    else
                        miss_state_next = PP_ALLOCATE;
                end
            end

            PP_WRITEBACK: begin
                if (dfp_resp)
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
        else if (write_hit)
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
        dirty_csb = 4'b0000;
        plru_csb  = 1'b0;

        data_web  = 4'b1111;
        tag_web   = 4'b1111;
        valid_web = 4'b1111;
        dirty_web = 4'b1111;
        plru_web  = 1'b1;

        tag_din   = s2_tag;
        valid_din = 1'b1;
        dirty_din = 1'b0;
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
            dirty_csb = 4'b1111;
            plru_csb  = 1'b1;
        end

        // S2 HIT
        if (s2_valid && cache_hit && !miss_handling && !s2_serviced_r) begin
            plru_web = 1'b0;
            plru_din = update_plru(plru_dout, hit_way);

            if (s2_is_write) begin
                data_web[hit_way]  = 1'b0;
                dirty_web[hit_way] = 1'b0;
                dirty_din = 1'b1;

                data_din[hit_way] = data_dout[hit_way];
                for (integer unsigned byte_idx = 0; byte_idx < 4; byte_idx++) begin
                    if (s2_reg.wmask[byte_idx]) begin
                        data_wmask[hit_way][s2_word_offset*4 + byte_idx] = 1'b1;
                        data_din[hit_way][(s2_word_offset*32) + (byte_idx*8) +: 8] = s2_reg.wdata[byte_idx*8 +: 8];
                    end
                end
            end
        end

        // ALLOCATE
        if (miss_state == PP_ALLOCATE && dfp_resp) begin
            data_web[evict_way]  = 1'b0;
            tag_web[evict_way]   = 1'b0;
            valid_web[evict_way] = 1'b0;
            dirty_web[evict_way] = 1'b0;

            dirty_din = (s2_reg.wmask != 4'b0) ? 1'b1 : 1'b0;

            data_din[evict_way] = dfp_rdata;
            data_wmask[evict_way] = 32'hFFFFFFFF;

            if (s2_reg.wmask != 4'b0) begin
                for (integer unsigned byte_idx = 0; byte_idx < 4; byte_idx++) begin
                    if (s2_reg.wmask[byte_idx])
                        data_din[evict_way][(s2_word_offset*32) + (byte_idx*8) +: 8] = s2_reg.wdata[byte_idx*8 +: 8];
                end
            end
        end
    end

    // DFP (MEMORY) INTF
    always_comb begin
        dfp_addr  = 32'b0;
        dfp_read  = 1'b0;
        dfp_write = 1'b0;
        dfp_wdata = 256'b0;

        case (miss_state)
            PP_WRITEBACK: begin
                dfp_write = 1'b1;
                dfp_addr  = {tag_dout[evict_way], s2_index, 5'b0};
                dfp_wdata = data_dout[evict_way];
            end

            PP_ALLOCATE: begin
                dfp_read = 1'b1;
                dfp_addr = {s2_tag, s2_index, 5'b0};
            end

            default: begin
                dfp_addr  = 32'b0;
                dfp_read  = 1'b0;
                dfp_write = 1'b0;
                dfp_wdata = 256'b0;
            end
        endcase
    end

    always_comb begin
        if (stall)
            s2_reg_next = s2_reg;
        else begin
            s2_reg_next.valid = s1_valid;
            s2_reg_next.addr  = ufp_addr;
            s2_reg_next.rmask = ufp_rmask;
            s2_reg_next.wmask = ufp_wmask;
            s2_reg_next.wdata = ufp_wdata;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            s2_reg.valid <= 1'b0;
            s2_reg.addr  <= 32'b0;
            s2_reg.rmask <= 4'b0;
            s2_reg.wmask <= 4'b0;
            s2_reg.wdata <= 32'b0;
        end
        else begin
            s2_reg <= s2_reg_next;
        end
    end

    // UFP OUTPUT (resp to CPU)
    assign ufp_resp = !s2_serviced_r && !allocate_recovery_r && ((s2_valid && cache_hit && !miss_handling) || allocate_complete);

    always_comb begin
        ufp_rdata = 32'b0;

        if (s2_valid && cache_hit && s2_is_read && !miss_handling) begin
            logic [31:0] word_data;
            word_data = data_dout[hit_way][s2_word_offset*32 +: 32];

            for (integer i = 0; i < 4; i++) begin
                if (s2_reg.rmask[i])
                    ufp_rdata[i*8 +: 8] = word_data[i*8 +: 8];
                else
                    ufp_rdata[i*8 +: 8] = 8'b0;
            end
        end
        else if (allocate_complete && s2_is_read) begin
            logic [255:0] alloc_data;
            logic [31:0] word_data;

            alloc_data = dfp_rdata;
            if (s2_reg.wmask != 4'b0) begin
                for (integer unsigned byte_idx = 0; byte_idx < 4; byte_idx++) begin
                    if (s2_reg.wmask[byte_idx])
                        alloc_data[(s2_word_offset*32) + (byte_idx*8) +: 8] = s2_reg.wdata[byte_idx*8 +: 8];
                end
            end

            word_data = alloc_data[s2_word_offset*32 +: 32];

            for (integer i = 0; i < 4; i++) begin
                if (s2_reg.rmask[i])
                    ufp_rdata[i*8 +: 8] = word_data[i*8 +: 8];
                else
                    ufp_rdata[i*8 +: 8] = 8'b0;
            end
        end
    end

endmodule : pp_dcache