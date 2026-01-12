module split_lsq
import types::*;
(
    input  logic        clk,
    input  logic        rst,
    input  logic        flush,

    // load dispatch intf (from rename) - port 0
    input  logic        lq_dispatch_valid,
    input  lq_entry_t   lq_dispatch_entry,
    output logic        lq_full,
    output logic        lq_almost_full,

    // load dispatch intf - port 1 (superscalar)
    input  logic        lq_dispatch_valid_1,
    input  lq_entry_t   lq_dispatch_entry_1,

    // store dispatch intf (from rename) - port 0
    input  logic        sq_dispatch_valid,
    input  sq_entry_t   sq_dispatch_entry,
    output logic        sq_full,
    output logic        sq_almost_full,

    // store dispatch intf - port 1 (superscalar)
    input  logic        sq_dispatch_valid_1,
    input  sq_entry_t   sq_dispatch_entry_1,

    // CDB intf (for wakeup)
    input  cdb_t        cdb [NUM_CDB_PORTS],

    // ROB commit intf (for stores) - port 0
    input  logic        rob_commit_valid,
    input  logic [4:0]  rob_commit_id,
    input  logic        rob_commit_is_store,

    // ROB commit intf - port 1 (dual commit)
    input  logic        rob_commit_valid_1,
    input  logic [4:0]  rob_commit_id_1,
    input  logic        rob_commit_is_store_1,

    // PRF read intf - load queue
    output logic [5:0]  lq_prf_src1_preg,
    input  logic [31:0] lq_prf_src1_data,
    output logic        lq_prf_req,
    input  logic        lq_prf_grant,

    // PRF read intf - store queue
    output logic [5:0]  sq_prf_src1_preg,
    input  logic [31:0] sq_prf_src1_data,
    output logic [5:0]  sq_prf_src2_preg,
    output logic        sq_prf_req,
    input  logic        sq_prf_grant,

    // D-Cache intf
    output logic [31:0] dcache_addr,
    output logic [3:0]  dcache_rmask,
    output logic [3:0]  dcache_wmask,
    output logic [31:0] dcache_wdata,
    input  logic [31:0] dcache_rdata,
    input  logic        dcache_resp,

    // load CDB output
    output cdb_t        lsq_load_cdb,
    input  logic        lsq_load_cdb_grant,

    // ROB update intf - loads
    output logic        load_update_mem_info,
    output logic [4:0]  load_update_id,
    output logic [31:0] load_update_addr,
    output logic [3:0]  load_update_rmask,
    output logic [31:0] load_update_rdata,

    // ROB update intf - stores
    output logic        store_update_mem_info,
    output logic [4:0]  store_update_id,
    output logic [31:0] store_update_addr,
    output logic [3:0]  store_update_wmask,
    output logic [31:0] store_update_wdata,
    output logic        store_ready,

    output logic [$clog2(NUM_LQ_ENTRIES):0] lq_occupancy,
    output logic [$clog2(NUM_SQ_ENTRIES):0] sq_occupancy,

    output logic [$clog2(NUM_SQ_ENTRIES)-1:0] sq_tail_out_cpu
);

    // load queue <-> D-Cache
    logic [31:0] lq_dcache_addr;
    logic [3:0]  lq_dcache_rmask;
    logic        lq_dcache_req;
    logic        lq_dcache_grant;

    // store queue <-> D-Cache
    logic [31:0] sq_dcache_addr;
    logic [3:0]  sq_dcache_wmask;
    logic [31:0] sq_dcache_wdata;
    logic        sq_dcache_req;
    logic        sq_dcache_grant;

    logic [$clog2(NUM_SQ_ENTRIES)-1:0] sq_head;
    logic [$clog2(NUM_SQ_ENTRIES)-1:0] sq_tail;
    logic        sq_empty;
    sq_entry_t   sq_entries [NUM_SQ_ENTRIES];

    assign sq_tail_out_cpu = sq_tail;

    logic        lq_dbg_blocked_ordering;
    logic        lq_dbg_blocked_unknown;
    logic        lq_dbg_cache_req;
    logic        lq_dbg_cache_hit;
    logic        lq_dbg_fwd_used;
    logic        sq_dbg_committed;
    logic        sq_dbg_cache_req;
    logic        sq_dbg_cache_complete;

    logic        cache_serving_load;
    logic        cache_serving_store;

    // D-CACHE ARB
    // loads get priority
    always_comb begin
        lq_dcache_grant = 1'b0;
        sq_dcache_grant = 1'b0;
        dcache_addr = '0;
        dcache_rmask = '0;
        dcache_wmask = '0;
        dcache_wdata = '0;

        if (cache_serving_load) begin
            // Waiting for cache response - maintain signals, but NO GRANT
            // Grant was already given when request started
            dcache_addr = lq_dcache_addr;
            dcache_rmask = lq_dcache_rmask;
            dcache_wmask = '0;
            dcache_wdata = '0;
        end
        else if (cache_serving_store) begin
            // Waiting for cache response - maintain signals, but NO GRANT
            // Grant was already given when request started
            dcache_addr = sq_dcache_addr;
            dcache_rmask = '0;
            dcache_wmask = sq_dcache_wmask;
            dcache_wdata = sq_dcache_wdata;
        end
        else if (lq_dcache_req) begin
            lq_dcache_grant = 1'b1;
            dcache_addr = lq_dcache_addr;
            dcache_rmask = lq_dcache_rmask;
            dcache_wmask = '0;
            dcache_wdata = '0;
        end 
        else if (sq_dcache_req) begin
            sq_dcache_grant = 1'b1;
            dcache_addr = sq_dcache_addr;
            dcache_rmask = '0;
            dcache_wmask = sq_dcache_wmask;
            dcache_wdata = sq_dcache_wdata;
        end
    end

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            cache_serving_load <= 1'b0;
            cache_serving_store <= 1'b0;
        end 
        else begin
            if (!cache_serving_load && !cache_serving_store && lq_dcache_req && lq_dcache_grant)
                cache_serving_load <= 1'b1;
            else if (!cache_serving_load && !cache_serving_store && sq_dcache_req && sq_dcache_grant)
                cache_serving_store <= 1'b1;

            if (cache_serving_load && dcache_resp)
                cache_serving_load <= 1'b0;

            if (cache_serving_store && dcache_resp)
                cache_serving_store <= 1'b0;
        end
    end

    load_queue #(
        .NUM_ENTRIES(NUM_LQ_ENTRIES)
    ) lq_inst (
        .clk(clk),
        .rst(rst),
        .flush(flush),

        // Dispatch - port 0
        .dispatch_valid(lq_dispatch_valid),
        .dispatch_entry(lq_dispatch_entry),
        .full(lq_full),
        .almost_full(lq_almost_full),

        // Dispatch - port 1 (superscalar)
        .dispatch_valid_1(lq_dispatch_valid_1),
        .dispatch_entry_1(lq_dispatch_entry_1),

        .cdb(cdb),

        .sq_head(sq_head),
        .sq_entries(sq_entries),

        .prf_src1_preg(lq_prf_src1_preg),
        .prf_src1_data(lq_prf_src1_data),
        .prf_req(lq_prf_req),
        .prf_grant(lq_prf_grant),

        .dcache_addr(lq_dcache_addr),
        .dcache_rmask(lq_dcache_rmask),
        .dcache_rdata(dcache_rdata),
        .dcache_resp(dcache_resp && cache_serving_load),
        .dcache_req(lq_dcache_req),
        .dcache_grant(lq_dcache_grant),

        .load_cdb(lsq_load_cdb),
        .load_cdb_grant(lsq_load_cdb_grant),

        .load_update_mem_info(load_update_mem_info),
        .load_update_id(load_update_id),
        .load_update_addr(load_update_addr),
        .load_update_rmask(load_update_rmask),
        .load_update_rdata(load_update_rdata),

        .occupancy(lq_occupancy),

        .dbg_issue_blocked_by_ordering(lq_dbg_blocked_ordering),
        .dbg_issue_blocked_by_unknown_addr(lq_dbg_blocked_unknown),
        .dbg_cache_req_sent(lq_dbg_cache_req),
        .dbg_cache_hit(lq_dbg_cache_hit),
        .dbg_fwd_used(lq_dbg_fwd_used)
    );


    store_queue #(
        .NUM_ENTRIES(NUM_SQ_ENTRIES)
    ) sq_inst (
        .clk(clk),
        .rst(rst),
        .flush(flush),

        // Dispatch - port 0
        .dispatch_valid(sq_dispatch_valid),
        .dispatch_entry(sq_dispatch_entry),
        .full(sq_full),
        .almost_full(sq_almost_full),

        // Dispatch - port 1 (superscalar)
        .dispatch_valid_1(sq_dispatch_valid_1),
        .dispatch_entry_1(sq_dispatch_entry_1),

        .cdb(cdb),

        .rob_commit_valid(rob_commit_valid),
        .rob_commit_id(rob_commit_id),
        .rob_commit_is_store(rob_commit_is_store),

        .rob_commit_valid_1(rob_commit_valid_1),
        .rob_commit_id_1(rob_commit_id_1),
        .rob_commit_is_store_1(rob_commit_is_store_1),

        .prf_src1_preg(sq_prf_src1_preg),
        .prf_src1_data(sq_prf_src1_data),
        .prf_src2_preg(sq_prf_src2_preg),
        .prf_req(sq_prf_req),
        .prf_grant(sq_prf_grant),

        .dcache_addr(sq_dcache_addr),
        .dcache_wmask(sq_dcache_wmask),
        .dcache_wdata(sq_dcache_wdata),
        .dcache_resp(dcache_resp && cache_serving_store),
        .dcache_req(sq_dcache_req),
        .dcache_grant(sq_dcache_grant),

        .store_update_mem_info(store_update_mem_info),
        .store_update_id(store_update_id),
        .store_update_addr(store_update_addr),
        .store_update_wmask(store_update_wmask),
        .store_update_wdata(store_update_wdata),
        .store_ready(store_ready),

        .sq_head_out(sq_head),
        .sq_tail_out(sq_tail),
        .sq_empty_out(sq_empty),
        .sq_entries_out(sq_entries),

        .occupancy(sq_occupancy),

        .dbg_store_committed(sq_dbg_committed),
        .dbg_cache_req_sent(sq_dbg_cache_req),
        .dbg_cache_complete(sq_dbg_cache_complete)
    );

endmodule : split_lsq