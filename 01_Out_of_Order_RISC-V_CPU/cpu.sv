module cpu
import types::*;
(
    input   logic               clk,
    input   logic               rst,
    
    // BMEM interface
    output  logic   [31:0]      bmem_addr,
    output  logic               bmem_read,
    output  logic               bmem_write,
    output  logic   [63:0]      bmem_wdata,
    input   logic               bmem_ready,
    input   logic   [31:0]      bmem_raddr,
    input   logic   [63:0]      bmem_rdata,
    input   logic               bmem_rvalid
);

    // RVFI - arrays for dual commit
    logic        rvfi_valid     [2];
    logic [63:0] rvfi_order     [2];
    logic [31:0] rvfi_inst      [2];
    logic [4:0]  rvfi_rs1_addr  [2];
    logic [4:0]  rvfi_rs2_addr  [2];
    logic [31:0] rvfi_rs1_rdata [2];
    logic [31:0] rvfi_rs2_rdata [2];
    logic [4:0]  rvfi_rd_addr   [2];
    logic [31:0] rvfi_rd_wdata  [2];
    logic [31:0] rvfi_pc_rdata  [2];
    logic [31:0] rvfi_pc_wdata  [2];
    logic [31:0] rvfi_mem_addr  [2];
    logic [3:0]  rvfi_mem_rmask [2];
    logic [3:0]  rvfi_mem_wmask [2];
    logic [31:0] rvfi_mem_rdata [2];
    logic [31:0] rvfi_mem_wdata [2];

    // Fetch <-> Pipelined I-Cache (direct connection)
    logic [31:0] fetch_icache_addr;
    logic [3:0]  fetch_icache_rmask;
    logic [31:0] fetch_icache_rdata;
    logic        fetch_icache_resp;
    
    // I-Cache <-> Prefetcher
    logic [31:0]  icache_dfp_addr;
    logic         icache_dfp_read;
    logic [255:0] icache_dfp_rdata;
    logic         icache_dfp_resp;

    // Prefetcher <-> Adapter-I
    logic [31:0]  cache_dfp_addr;
    logic         cache_dfp_read;
    logic         cache_dfp_write;
    logic [255:0] cache_dfp_rdata;
    logic [255:0] cache_dfp_wdata;
    logic         cache_dfp_resp;
    logic [31:0]  cache_dfp_raddr;  // Response address for verification
    
    // NEW: D-Cache <-> LSQ signals
    logic [31:0]  dcache_ufp_addr;
    logic [3:0]   dcache_ufp_rmask;
    logic [3:0]   dcache_ufp_wmask;
    logic [31:0]  dcache_ufp_rdata;
    logic [31:0]  dcache_ufp_wdata;
    logic         dcache_ufp_resp;
    
    // NEW: D-Cache <-> Adapter-D signals
    logic [31:0]  dcache_dfp_addr;
    logic         dcache_dfp_read;
    logic         dcache_dfp_write;
    logic [255:0] dcache_dfp_rdata;
    logic [255:0] dcache_dfp_wdata;
    logic         dcache_dfp_resp;
    
    // Fetch outputs - instruction 0
    logic [31:0] fetch_instr;
    logic [31:0] fetch_pc;
    logic        fetch_valid;
    logic [8:0]  fetch_prediction_ghr;

    // Fetch outputs - instruction 1 (superscalar)
    logic [31:0] fetch_instr_1;
    logic [31:0] fetch_pc_1;
    logic        fetch_valid_1;
    logic        fetch_predicted_taken_1;
    logic [31:0] fetch_predicted_target_1;
    logic [8:0]  fetch_prediction_ghr_1;

    // Linebuffer/cache dual ports
    logic [31:0] icache_addr_1;
    logic [3:0]  icache_rmask_1;
    logic [31:0] icache_rdata_1;
    logic        icache_resp_1;

    // Queue signals - port 0
    logic         queue_full, queue_empty;
    logic         queue_almost_full, queue_almost_empty;
    logic         queue_enq;
    logic [106:0] queue_enq_data;   // {ghr[8:0], predicted_target[31:0], predicted_taken, fetch_pc[31:0], fetch_instr[31:0]}
    logic         queue_deq;
    logic [106:0] queue_deq_data;
    logic [3:0]   queue_occupancy;

    // Queue signals - port 1 (superscalar)
    logic         queue_enq_1;
    logic [106:0] queue_enq_data_1;
    logic         queue_deq_1;
    logic [106:0] queue_deq_data_1;

    // Queue output - instruction 0
    logic [31:0]  queue_pc;
    logic [31:0]  queue_inst;
    logic         queue_predicted_taken;
    logic [31:0]  queue_predicted_target;
    logic [8:0]   queue_prediction_ghr;
    logic         queue_valid;

    // Queue output - instruction 1 (superscalar)
    logic [31:0]  queue_pc_1;
    logic [31:0]  queue_inst_1;
    logic         queue_predicted_taken_1;
    logic [31:0]  queue_predicted_target_1;
    logic [8:0]   queue_prediction_ghr_1;
    logic         queue_valid_1;

    // Decode - instruction 0
    decoded_inst_t decoded_out;
    logic          decode_valid;

    // Decode - instruction 1 (superscalar)
    decoded_inst_t decoded_out_1;
    logic          decode_valid_1;

    // Rename - instruction 0
    renamed_inst_t renamed_out;
    logic          rename_valid;
    logic          stall_to_fetch;

    // Rename - instruction 1 (superscalar)
    renamed_inst_t renamed_out_1;
    logic          rename_valid_1;

    // RAT - instruction 0
    logic [4:0]  rat_rs1_areg, rat_rs2_areg;
    logic [5:0]  rat_rs1_preg, rat_rs2_preg;
    logic        rat_rs1_ready, rat_rs2_ready;
    logic [4:0]  rat_rs1_rob_id, rat_rs2_rob_id;  // Producer ROB ID for CDB validation in rename
    logic        rat_alloc_en;
    logic [4:0]  rat_rd_areg;
    logic [5:0]  rat_rd_preg;
    logic [4:0]  rat_rd_rob_id;  // ROB ID of the producer (for CDB validation)

    // RAT - instruction 1 (superscalar)
    logic [4:0]  rat_rs1_areg_1, rat_rs2_areg_1;
    logic [5:0]  rat_rs1_preg_1, rat_rs2_preg_1;
    logic        rat_rs1_ready_1, rat_rs2_ready_1;
    logic [4:0]  rat_rs1_rob_id_1, rat_rs2_rob_id_1;  // Producer ROB ID for CDB validation in rename
    logic        rat_alloc_en_1;
    logic [4:0]  rat_rd_areg_1;
    logic [5:0]  rat_rd_preg_1;
    logic [4:0]  rat_rd_rob_id_1;  // ROB ID of the producer (for CDB validation)

    logic [5:0] rat_mappings [32];

    // Free list - instruction 0
    logic        fl_alloc, fl_free, fl_free_1;
    logic [5:0]  fl_alloc_preg, fl_free_preg, fl_free_preg_1;
    logic        fl_empty, fl_full;

    // Free list - instruction 1 (superscalar)
    logic        fl_alloc_1;
    logic [5:0]  fl_alloc_preg_1;
    logic        fl_almost_empty;
    logic [$clog2(NUM_PREGS-1):0] fl_occupancy;  // For profiling

    // ROB - instruction 0
    logic        rob_enq, rob_commit, rob_commit_1;
    rob_entry_t  rob_enq_data, rob_head, rob_head_1;
    logic [4:0]  rob_idx;
    logic        rob_full, rob_empty;
    logic        rob_head_valid, rob_head_ready;
    logic        rob_head_valid_1, rob_head_ready_1;

    // ROB - instruction 1 (superscalar dual enqueue)
    logic        rob_enq_1;
    rob_entry_t  rob_enq_data_1;
    logic [4:0]  rob_idx_1;
    logic        rob_almost_full;

    // PRF rename ports - instruction 0
    logic [5:0]  prf_rs1_preg_rename, prf_rs2_preg_rename;
    logic [31:0] prf_rs1_data_rename, prf_rs2_data_rename;

    // PRF rename ports - instruction 1 (superscalar)
    logic [5:0]  prf_rs1_preg_rename_1, prf_rs2_preg_rename_1;
    logic [31:0] prf_rs1_data_rename_1, prf_rs2_data_rename_1;

    // exec read ports
    logic [5:0]  prf_exec_rs_preg [NUM_EXEC_READ_PORTS][2];     // [port][0=rs1, 1=rs2]
    logic [31:0] prf_exec_rs_data [NUM_EXEC_READ_PORTS][2];

    // PRF arbiter array (6 requesters: ALU, MUL, DIV, BRANCH, LQ, SQ)
    logic        arb_exec_req [NUM_EXEC_READ_PORTS];
    logic [5:0]  arb_exec_rs1_preg [NUM_EXEC_READ_PORTS];
    logic [5:0]  arb_exec_rs2_preg [NUM_EXEC_READ_PORTS];
    logic        arb_exec_grant [NUM_EXEC_READ_PORTS];
    logic [31:0] arb_exec_rs1_data [NUM_EXEC_READ_PORTS];
    logic [31:0] arb_exec_rs2_data [NUM_EXEC_READ_PORTS];

    // ALU_0 RS
    logic        alu_0_rs_dispatch_valid, alu_0_rs_full;
    rs_entry_t   alu_0_rs_dispatch_entry;
    logic        alu_0_rs_issue_valid;
    rs_entry_t   alu_0_rs_issue_entry;
    logic [$clog2(NUM_ALU_RS/2):0]  alu_0_rs_occupancy;
    logic        alu_0_execute_ready;

    // ALU_1 RS
    logic        alu_1_rs_dispatch_valid, alu_1_rs_full;
    rs_entry_t   alu_1_rs_dispatch_entry;
    logic        alu_1_rs_issue_valid;
    rs_entry_t   alu_1_rs_issue_entry;
    logic [$clog2(NUM_ALU_RS/2):0]  alu_1_rs_occupancy;
    logic        alu_1_execute_ready;

    // Unified ALU RS full signal
    logic        alu_rs_full;
    assign alu_rs_full = alu_0_rs_full && alu_1_rs_full;

    // For dual ALU dispatch: blocked if EITHER RS is full (since they go to different RS modules)
    logic        alu_rs_almost_full;
    assign alu_rs_almost_full = alu_0_rs_full || alu_1_rs_full;

    // ALU_0
    logic [31:0] alu_0_a, alu_0_b, alu_0_result;
    alu_op_t     alu_0_op;

    // ALU_1
    logic [31:0] alu_1_a, alu_1_b, alu_1_result;
    alu_op_t     alu_1_op;

    // ALU_0 PRF access (via arbiter)
    logic        alu_0_prf_req;
    logic        alu_0_prf_grant;
    logic [5:0]  alu_0_rs1_preg, alu_0_rs2_preg;
    logic [31:0] alu_0_rs1_data, alu_0_rs2_data;

    // ALU_1 PRF access (via arbiter)
    logic        alu_1_prf_req;
    logic        alu_1_prf_grant;
    logic [5:0]  alu_1_rs1_preg, alu_1_rs2_preg;
    logic [31:0] alu_1_rs1_data, alu_1_rs2_data;

    // MUL RS
    logic        mul_rs_dispatch_valid, mul_rs_full;
    rs_entry_t   mul_rs_dispatch_entry;
    logic        mul_rs_issue_valid;
    rs_entry_t   mul_rs_issue_entry;
    logic [$clog2(NUM_MUL_RS):0]  mul_rs_occupancy;
    logic        mul_execute_ready;

    // MUL PRF access (via arbiter)
    logic        mul_prf_req;
    logic        mul_prf_grant;
    logic [5:0]  mul_rs1_preg, mul_rs2_preg;
    logic [31:0] mul_rs1_data, mul_rs2_data;

    logic        mul_busy;

    // DIV RS
    logic        div_rs_dispatch_valid, div_rs_full;
    rs_entry_t   div_rs_dispatch_entry;
    logic        div_rs_issue_valid;
    rs_entry_t   div_rs_issue_entry;
    logic [$clog2(NUM_DIV_RS):0]  div_rs_occupancy;
    logic        div_execute_ready;

    // DIV PRF access (via arbiter)
    logic        div_prf_req;
    logic        div_prf_grant;
    logic [5:0]  div_rs1_preg, div_rs2_preg;
    logic [31:0] div_rs1_data, div_rs2_data;

    logic        div_busy;

     
    // INSTRUCTION 1 DISPATCH SIGNALS (2-way superscalar)
     

    // ALU RS - instruction 1 dispatch (to opposite RS of inst 0)
    logic        alu_0_rs_dispatch_valid_1;
    rs_entry_t   alu_0_rs_dispatch_entry_1;
    logic        alu_0_rs_almost_full;
    logic        alu_1_rs_dispatch_valid_1;
    rs_entry_t   alu_1_rs_dispatch_entry_1;
    logic        alu_1_rs_almost_full;

    // MUL RS - instruction 1 dispatch (dual dispatch port)
    logic        mul_rs_dispatch_valid_1;
    rs_entry_t   mul_rs_dispatch_entry_1;
    logic        mul_rs_almost_full;

    // DIV RS - instruction 1 dispatch (dual dispatch port)
    logic        div_rs_dispatch_valid_1;
    rs_entry_t   div_rs_dispatch_entry_1;
    logic        div_rs_almost_full;

    // BRANCH RS - instruction 1 dispatch (dual dispatch port)
    logic        branch_rs_dispatch_valid_1;
    rs_entry_t   branch_rs_dispatch_entry_1;
    logic        branch_rs_almost_full;

    // CDB
    cdb_t        cdb [NUM_CDB_PORTS];
    cdb_t        exec_unit_cdbs [NUM_EXEC_UNITS];

    cdb_t        alu_0_cdb, alu_1_cdb, mul_cdb, div_cdb;

    // CDB arbiter request/grant signals
    logic        alu_0_cdb_request, alu_1_cdb_request, mul_cdb_request, div_cdb_request;
    logic        alu_0_cdb_grant, alu_1_cdb_grant, mul_cdb_grant, div_cdb_grant;
    logic        exec_cdb_requests [NUM_EXEC_UNITS];
    logic        exec_cdb_grants [NUM_EXEC_UNITS];
    
    // ALU_0 result buffer
    logic        alu_0_result_valid;
    logic [31:0] alu_0_result_buffered;
    logic [5:0]  alu_0_dest_preg_buffered;
    logic [4:0]  alu_0_rob_id_buffered;

    // ALU_1 result buffer
    logic        alu_1_result_valid;
    logic [31:0] alu_1_result_buffered;
    logic [5:0]  alu_1_dest_preg_buffered;
    logic [4:0]  alu_1_rob_id_buffered;

    // ALU dispatch routing (round-robin)
    logic        alu_dispatch_toggle;


    // Commit - port 0
    logic [4:0]  rrf_read_areg;
    logic [5:0]  rrf_old_preg;
    logic        rrf_commit_en;
    logic [4:0]  rrf_commit_areg;
    logic [5:0]  rrf_commit_preg;

    // Commit - port 1 (dual commit)
    logic [4:0]  rrf_read_areg_1;
    logic [5:0]  rrf_old_preg_1;
    logic        rrf_commit_en_1;
    logic [4:0]  rrf_commit_areg_1;
    logic [5:0]  rrf_commit_preg_1;
    
    // RRF RVFI read ports - instruction 0 (for rename stage RVFI capture)
    logic [4:0]  rrf_rvfi_rs1_areg;
    logic [5:0]  rrf_rvfi_rs1_preg;
    logic [4:0]  rrf_rvfi_rs2_areg;
    logic [5:0]  rrf_rvfi_rs2_preg;

    // RRF RVFI read ports - instruction 1 (superscalar)
    logic [4:0]  rrf_rvfi_rs1_areg_1;
    logic [5:0]  rrf_rvfi_rs1_preg_1;
    logic [4:0]  rrf_rvfi_rs2_areg_1;
    logic [5:0]  rrf_rvfi_rs2_preg_1;

    // Adapter-I <-> Arbiter
    logic [31:0]  adapter_i_bmem_addr;
    logic         adapter_i_bmem_read;
    logic         adapter_i_bmem_write;
    logic [63:0]  adapter_i_bmem_rdata;
    logic [63:0]  adapter_i_bmem_wdata;
    logic         adapter_i_bmem_ready;
    logic [31:0]  adapter_i_bmem_raddr;
    logic         adapter_i_bmem_rvalid;

    // Adapter-D <-> Arbiter (for future D-Cache)
    logic [31:0]  adapter_d_bmem_addr;
    logic         adapter_d_bmem_read;
    logic         adapter_d_bmem_write;
    logic [63:0]  adapter_d_bmem_rdata;
    logic [63:0]  adapter_d_bmem_wdata;
    logic         adapter_d_bmem_ready;
    logic [31:0]  adapter_d_bmem_raddr;
    logic         adapter_d_bmem_rvalid;

    logic        rob_branch_mispredict;
    logic [31:0] rob_correct_target;
    logic        head_branch_mispredict;  // From ROB to commit to block dual commit on mispredicting branch
    logic        bp_update_en;
    logic [31:0] bp_update_pc;
    logic [31:0] bp_update_instr;
    logic        bp_actual_taken;
    logic [31:0] bp_actual_target;
    logic [4:0]  bp_update_rob_idx;
    logic [8:0]  bp_update_ghr;
    logic        bp_update_predicted_taken;  // What was predicted (for profiler)

    // RAS signals - port 0
    logic        ras_update_en;
    logic        ras_update_is_push;
    logic        ras_update_is_pop;
    logic [31:0] ras_update_push_addr;

    // RAS signals - port 1 (dual commit)
    logic        ras_update_en_1;
    logic        ras_update_is_push_1;
    logic        ras_update_is_pop_1;
    logic [31:0] ras_update_push_addr_1;

    logic        flush;
    logic        fetch_predicted_taken;
    logic [31:0] fetch_predicted_target;
    
    // Branch execution signals (for executing branches at commit)
    logic        branch_unit_taken;
    logic [31:0] branch_unit_target;

    // BRANCH RS
    logic        branch_rs_dispatch_valid, branch_rs_full;
    rs_entry_t   branch_rs_dispatch_entry;
    logic        branch_rs_issue_valid;
    rs_entry_t   branch_rs_issue_entry;
    logic [$clog2(NUM_BRANCH_RS):0]  branch_rs_occupancy;
    logic        branch_execute_ready;

    // BRANCH PRF access (via arbiter)
    logic        branch_prf_req;
    logic        branch_prf_grant;
    logic [5:0]  branch_rs1_preg, branch_rs2_preg;
    logic [31:0] branch_rs1_data, branch_rs2_data;

    // BRANCH execution
    logic        branch_busy;
    cdb_t        branch_cdb;
    logic        branch_cdb_request, branch_cdb_grant;
    
    // SPLIT LSQ signals
    // Load Queue dispatch - port 0
    logic        lq_dispatch_valid, lq_full;
    lq_entry_t   lq_dispatch_entry;
    logic [$clog2(NUM_LQ_ENTRIES):0]  lq_occupancy;
    logic        lq_almost_full;

    // Load Queue dispatch - port 1 (superscalar)
    logic        lq_dispatch_valid_1;
    lq_entry_t   lq_dispatch_entry_1;

    // Store Queue dispatch - port 0
    logic        sq_dispatch_valid, sq_full;
    sq_entry_t   sq_dispatch_entry;
    logic [$clog2(NUM_SQ_ENTRIES):0]  sq_occupancy;
    logic [$clog2(NUM_SQ_ENTRIES)-1:0] sq_tail_for_capture;     // SQ tail for memory ordering
    logic        sq_almost_full;

    // Store Queue dispatch - port 1 (superscalar)
    logic        sq_dispatch_valid_1;
    sq_entry_t   sq_dispatch_entry_1;

    logic        lsq_full;
    assign lsq_full = lq_full || sq_full;

    logic [$clog2(NUM_LSQ_ENTRIES):0]  lsq_occupancy;
    assign lsq_occupancy = lq_occupancy + sq_occupancy;

    // Load Queue PRF access
    logic        lq_prf_req;
    logic        lq_prf_grant;
    logic [5:0]  lq_src1_preg;
    logic [31:0] lq_src1_data;

    // Store Queue PRF access
    logic        sq_prf_req;
    logic        sq_prf_grant;
    logic [5:0]  sq_src1_preg, sq_src2_preg;
    logic [31:0] sq_src1_data, sq_src2_data;

    // LSQ CDB - load only (store uses direct ready signal)
    cdb_t        lsq_load_cdb;
    logic        lsq_load_cdb_request, lsq_load_cdb_grant;

    // LSQ <-> ROB interface - Separate Load and Store Ports
    // Load update port
    logic        lsq_load_update_mem_info;
    logic [4:0]  lsq_load_update_id;
    logic [31:0] lsq_load_update_addr;
    logic [3:0]  lsq_load_update_rmask;
    logic [31:0] lsq_load_update_rdata;

    // Store update port
    logic        lsq_store_update_mem_info;
    logic [4:0]  lsq_store_update_id;
    logic [31:0] lsq_store_update_addr;
    logic [3:0]  lsq_store_update_wmask;
    logic [31:0] lsq_store_update_wdata;
    logic        lsq_store_ready;           // Store ready signal (replaces store CDB)

    // Flush signal generation
    logic [5:0] rrf_mappings [32];

    logic [4:0]  rob_flush_free_count;
    logic [5:0]  rob_flush_free_pregs [ROB_SIZE];
    logic        rob_flush_occurred;

    logic [4:0]  cpu_flush_free_count;
    logic [5:0]  cpu_flush_free_pregs [ROB_SIZE];
    logic        cpu_flush_occurred;
    logic flush_this_cycle;

    logic bp_update_is_conditional_branch;

    assign flush = rob_branch_mispredict;

      // D-Cache Prefetcher Signals
    logic [31:0]  dpf_addr;
    logic         dpf_read;
    logic         dpf_write;
    logic [255:0] dpf_rdata;
    logic         dpf_resp;
    logic [31:0]  dpf_raddr;

    // Prefetcher <- Adapter-D (response path)
    logic [255:0] ad_mem_rdata;
    logic         ad_mem_resp;
    logic [31:0]  ad_mem_raddr;





/*
CP1 DIAGRAM
                                 +<----> ADAPTER <-------> I-CACHE <------> FETCH STAGE -------> QUEUE     
DRAM <-----------> ARBITER <---->|
                                 +<----> ADAPTER <-------> D-CACHE
*/

/*
CP2 DIAGRAM

        DRAM
          |
        ARBITER
        /     \
    ADAPTER  ADAPTER
      |          |
    I-CACHE   D-CACHE
      |
    FETCH -----> QUEUE -----> DECODE
                                |
                            RENAME (RAT, ROB alloc, Pyhsical reg)
                                |               
                    +-----------+-----------+-----------+
                    |           |           |           |
                ROB_ENQ     RAT_ALLOC    FL_ALLOC    DISPATCH
                   |           |           |           |
                   v           v           v           v
              +--------+   +-------+   +--------+   +--------+
              |  ROB   |   |  RAT  |   | FREE   |   |   RS   |
              | (16)   |   | (32)  |   | LIST   |   | LOGIC  |
              +--------+   +-------+   +--------+   +--------+
                   |           |           |           |
                   |           |           |           v
                   |           |           |   +-----+-----+-----+
                   |           |           |   |     |     |     |
                   |           |           |   v     v     v     v
                   |           |           | +-----+ +-----+ +-----+
                   |           |           | |ALU  | |MUL  | |DIV  |
                   |           |           | |RS(8)| |RS(4)| |RS(2)|
                   |           |           | +-----+ +-----+ +-----+
                   |           |           |     |     |     |
                   |           |           |     v     v     v
                   |           |           |   SELECT READY INST
                   |           |           |     |     |     |
                   |           |           |     v     v     v
                   |           |           |  +-----------------+
                   |           |           |  |  PRF ARBITER    |
                   |           |           |  +-----------------+
                   |           |           |     |     |     |
                   |           |           |     v     v     v
                   |           |           |  +-----------------+
                   |           |           |  |   PRF (64)      |
                   |           |           |  |   READ PORTS    |
                   |           |           |  +-----------------+
                   |           |           |     |     |     |
                   |           |           |     v     v     v
                   |           |           |  +-----+ +-----+ +-----+
                   |           |           |  | ALU | | MUL | | DIV |
                   |           |           |  +-----+ +-----+ +-----+
                   |           |           |     |     |     |
                   |           |           |     v     v     v
                   |           |           |  +-----+ +-----+ +-----+
                   |           |           |  |alu_ | |mul_ | |div_ |
                   |           |           |  |cdb  | |cdb  | |cdb  |
                   |           |           |  +-----+ +-----+ +-----+
                   |           |           |     |     |     |
                   |           |           |     +-----+-----+
                   |           |           |           |
                   |           |           |           v
                   |           |           |    +--------------+
                   |           |           |    | CDB ARBITER  |
                   |           |           |    +--------------+
                   |           |           |           |
                   |           |           |     +-----+-----+
                   |           |           |     |     |     |
                   |           |           |     v     v     v
                   |           |           |  +-----+ +-----+ +-----+
                   |           |           |  |CDB  | |CDB  | |CDB  |
                   |           |           |  | [0] | | [1] | | [2] |
                   |           |           |  +-----+ +-----+ +-----+
                   |           |           |     |     |     |
                   |           |           |     +-----+-----+
                   |           |           |           |
                   |           +--+--------+-----------+
                   |              |        |           |
                   +-------+------+        |           |
                           |               |           |
               +-----------+---------------+-----------+
               |           |               |           |
               v           v               v           v
          +--------+   +-------+       +-------+   +--------+
          |  PRF   |   |  RAT  |       |  RS   |   |  ROB   |
          | WRITE  |   | READY |       | WAKEUP|   | READY  |
          +--------+   +-------+       +-------+   +--------+
               |           |               |           |
               |           |               |           v
               |           |               |      ROB HEAD READY?
               |           |               |           |
               |           |               |           v
               |           |               |    +-------------+
               |           |               |    | COMMIT      |
               |           |               |    +-------------+
               |           |               |           |
               |           +---------------+-----------+
               |                           |           |
               v                           v           v
          +--------+                  +--------+   +--------+
          |  RRF   |                  | FREE   |   | RVFI   |
          | UPDATE |                  | LIST   |   +--------+
          +--------+                  +--------+

*/

    // FETCH STAGE (2-WAY SUPERSCALAR)
    fetch_stage fetch (
        .clk(clk),
        .rst(rst),
        .stall_backend(queue_full || stall_to_fetch),
        .queue_almost_full(queue_almost_full),

        // branch resolution from ROB
        .branch_mispredict(rob_branch_mispredict),
        .correct_target(rob_correct_target),

        // branch predictor update from commit
        .bp_update_en(bp_update_en),
        .bp_update_pc(bp_update_pc),
        .bp_actual_taken(bp_actual_taken),
        .update_ghr(bp_update_ghr),

        .bp_update_is_conditional_branch(bp_update_is_conditional_branch),

        // RAS update from commit - port 0
        .ras_update_en(ras_update_en),
        .ras_update_is_push(ras_update_is_push),
        .ras_update_is_pop(ras_update_is_pop),
        .ras_update_push_addr(ras_update_push_addr),

        // RAS update from commit - port 1 (dual commit)
        .ras_update_en_1(ras_update_en_1),
        .ras_update_is_push_1(ras_update_is_push_1),
        .ras_update_is_pop_1(ras_update_is_pop_1),
        .ras_update_push_addr_1(ras_update_push_addr_1),

        // pipeline flush
        .flush(flush),

        // I-cache port 0 (primary)
        .icache_addr(fetch_icache_addr),
        .icache_rmask(fetch_icache_rmask),
        .icache_rdata(fetch_icache_rdata),
        .icache_resp(fetch_icache_resp),

        // I-cache port 1 (superscalar)
        .icache_addr_1(icache_addr_1),
        .icache_rmask_1(icache_rmask_1),
        .icache_rdata_1(icache_rdata_1),
        .icache_resp_1(icache_resp_1),

        // Output - instruction 0
        .fetch_instr(fetch_instr),
        .fetch_pc(fetch_pc),
        .fetch_valid(fetch_valid),
        .fetch_predicted_taken(fetch_predicted_taken),
        .fetch_predicted_target(fetch_predicted_target),
        .fetch_prediction_ghr(fetch_prediction_ghr),

        // Output - instruction 1 (superscalar)
        .fetch_instr_1(fetch_instr_1),
        .fetch_pc_1(fetch_pc_1),
        .fetch_valid_1(fetch_valid_1),
        .fetch_predicted_taken_1(fetch_predicted_taken_1),
        .fetch_predicted_target_1(fetch_predicted_target_1),
        .fetch_prediction_ghr_1(fetch_prediction_ghr_1)
    );

    // I-CACHE (2-WAY SUPERSCALAR)
    pp_icache icache (
        .clk(clk),
        .rst(rst),

        // UFP Port 0 - directly from fetch stage
        .ufp_addr(fetch_icache_addr),
        .ufp_rmask(fetch_icache_rmask),
        .ufp_rdata(fetch_icache_rdata),
        .ufp_resp(fetch_icache_resp),
        .ufp_flush(flush),

        // UFP Port 1 - superscalar (same cacheline as port 0)
        .ufp_addr_1(icache_addr_1),
        .ufp_rmask_1(icache_rmask_1),
        .ufp_rdata_1(icache_rdata_1),
        .ufp_resp_1(icache_resp_1),

        // DFP - to prefetcher
        .dfp_addr(icache_dfp_addr),
        .dfp_read(icache_dfp_read),
        .dfp_rdata(icache_dfp_rdata),
        .dfp_resp(icache_dfp_resp)
    );

    // I-CACHE NEXT-LINE PREFETCHER
    i_prefetcher #(.NUM_ENTRIES(2)) icache_prefetcher (
        .clk(clk),
        .rst(rst),
        .flush(flush),

        // I-Cache side (upstream)
        .cache_addr(icache_dfp_addr),
        .cache_read(icache_dfp_read),
        .cache_rdata(icache_dfp_rdata),
        .cache_resp(icache_dfp_resp),

        // Adapter side (downstream)
        .mem_addr(cache_dfp_addr),
        .mem_read(cache_dfp_read),
        .mem_rdata(cache_dfp_rdata),
        .mem_resp(cache_dfp_resp),
        .mem_raddr(cache_dfp_raddr)
    );

    assign cache_dfp_write = 1'b0;
    assign cache_dfp_wdata = 256'b0;

    // Adapter-I
    cacheline_adapter adapter_i (
        .clk(clk),
        .rst(rst),

        .cache_addr(cache_dfp_addr),
        .cache_read(cache_dfp_read),
        .cache_write(cache_dfp_write),
        .cache_rdata(cache_dfp_rdata),
        .cache_wdata(cache_dfp_wdata),
        .cache_resp(cache_dfp_resp),
        .cache_raddr(cache_dfp_raddr),

        // Arbiter side
        .bmem_addr(adapter_i_bmem_addr),
        .bmem_read(adapter_i_bmem_read),
        .bmem_write(adapter_i_bmem_write),
        .bmem_rdata(adapter_i_bmem_rdata),
        .bmem_wdata(adapter_i_bmem_wdata),
        .bmem_ready(adapter_i_bmem_ready),
        .bmem_raddr(adapter_i_bmem_raddr),
        .bmem_rvalid(adapter_i_bmem_rvalid)
    );

     // D-CACHE
    pp_dcache dcache (
        .clk(clk),
        .rst(rst),

        // UFP - from LSQ
        .ufp_addr(dcache_ufp_addr),
        .ufp_rmask(dcache_ufp_rmask),
        .ufp_wmask(dcache_ufp_wmask),
        .ufp_rdata(dcache_ufp_rdata),
        .ufp_wdata(dcache_ufp_wdata),
        .ufp_resp(dcache_ufp_resp),
        .ufp_flush(flush),

        // DFP - to adapter-D
        .dfp_addr(dcache_dfp_addr),
        .dfp_read(dcache_dfp_read),
        .dfp_write(dcache_dfp_write),
        .dfp_rdata(dcache_dfp_rdata),
        .dfp_wdata(dcache_dfp_wdata),
        .dfp_resp(dcache_dfp_resp)
    );


d_stride_prefetcher #(
    .NUM_ENTRIES(2)
    // .PREFETCH_DEGREE left as default
) dcache_prefetcher (
    .clk   (clk),
    .rst   (rst),
    .flush (flush),

    // Upstream: from D-cache DFP
    .cache_addr (dcache_dfp_addr),
    .cache_read (dcache_dfp_read),
    .cache_write(dcache_dfp_write),

    // Prefetcher -> D-cache
    .cache_rdata(dcache_dfp_rdata),
    .cache_resp (dcache_dfp_resp),

    // Prefetcher -> Adapter-D (request)
    .mem_addr  (dpf_addr),
    .mem_read  (dpf_read),
    .mem_write (dpf_write),

    // Prefetcher <- Adapter-D (response)
    .mem_rdata (ad_mem_rdata),
    .mem_resp  (ad_mem_resp),
    .mem_raddr (ad_mem_raddr)
);



   cacheline_adapter adapter_d (
    .clk (clk),
    .rst (rst),

    // Prefetcher -> Adapter-D (request)
    .cache_addr (dpf_addr),
    .cache_read (dpf_read),
    .cache_write(dpf_write),
    .cache_wdata(dcache_dfp_wdata),

    // Adapter-D -> Prefetcher (response)
    .cache_rdata(ad_mem_rdata),
    .cache_resp (ad_mem_resp),
    .cache_raddr(ad_mem_raddr),

    // Arbiter / BMEM side
    .bmem_addr  (adapter_d_bmem_addr),
    .bmem_read  (adapter_d_bmem_read),
    .bmem_write (adapter_d_bmem_write),
    .bmem_rdata (adapter_d_bmem_rdata),
    .bmem_wdata (adapter_d_bmem_wdata),
    .bmem_ready (adapter_d_bmem_ready),
    .bmem_raddr (adapter_d_bmem_raddr),
    .bmem_rvalid(adapter_d_bmem_rvalid)
);

    // Arbiter
    arbiter arbiter (
        .clk(clk),
        .rst(rst),
        
        // Adapter-I
        .bmem_addr_i(adapter_i_bmem_addr),
        .bmem_read_i(adapter_i_bmem_read),
        .bmem_write_i(adapter_i_bmem_write),
        .bmem_rdata_i(adapter_i_bmem_rdata),
        .bmem_wdata_i(adapter_i_bmem_wdata),
        .bmem_ready_i(adapter_i_bmem_ready),
        .bmem_raddr_i(adapter_i_bmem_raddr),
        .bmem_rvalid_i(adapter_i_bmem_rvalid),
        
        // Adapter-D
        .bmem_addr_d(adapter_d_bmem_addr),
        .bmem_read_d(adapter_d_bmem_read),
        .bmem_write_d(adapter_d_bmem_write),
        .bmem_rdata_d(adapter_d_bmem_rdata),
        .bmem_wdata_d(adapter_d_bmem_wdata),
        .bmem_ready_d(adapter_d_bmem_ready),
        .bmem_raddr_d(adapter_d_bmem_raddr),
        .bmem_rvalid_d(adapter_d_bmem_rvalid),
        
        // DRAM
        .bmem_addr(bmem_addr),
        .bmem_read(bmem_read),
        .bmem_write(bmem_write),
        .bmem_rdata(bmem_rdata),
        .bmem_wdata(bmem_wdata),
        .bmem_ready(bmem_ready),
        .bmem_raddr(bmem_raddr),
        .bmem_rvalid(bmem_rvalid)
    );

    logic block_queue_enq;
    always_comb begin
        block_queue_enq = 1'b0;
        
        // check if about to commit a mispredicted branch
        if (rob_head_valid && rob_head_ready && rob_head.is_branch) begin

            logic [31:0] branch_result_check;
            logic actual_taken_check;
            logic mispredicted;
            
            branch_result_check = rob_head.result;
            
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb[i].valid && cdb[i].rob_id == rob_idx[4:0])
                    branch_result_check = cdb[i].data;
            end
            
            if (rob_head.inst_type == ITYPE_JAL)
                mispredicted = 1'b0;

            else if (rob_head.inst_type == ITYPE_JALR) begin

                logic signed [31:0] jalr_imm;
                logic [31:0] actual_target, predicted_target;
                
                jalr_imm = $signed({{20{rob_head.inst[31]}}, rob_head.inst[31:20]});

                actual_target = (rob_head.rs1_data + 32'($unsigned(jalr_imm))) & ~32'h1;
                
                predicted_target = rob_head.branch_target;
                mispredicted = (actual_target != predicted_target);
            end
            else begin
                actual_taken_check = branch_result_check[0];
                mispredicted = (actual_taken_check != rob_head.predicted_taken);
            end
            
            if (mispredicted)
                block_queue_enq = 1'b1;     // block enqueue
        end
    end

    // FETCH QUEUE (2-WAY SUPERSCALAR)
    // Instruction 0 enqueue
    assign queue_enq = fetch_valid && !queue_full && !flush_this_cycle && !block_queue_enq;
    assign queue_enq_data = {fetch_prediction_ghr, fetch_predicted_target, fetch_predicted_taken, fetch_pc, fetch_instr};

    // Instruction 1 enqueue - only if inst 0 enqueues AND inst 1 is valid AND not almost full
    assign queue_enq_1 = fetch_valid_1 && queue_enq && !queue_almost_full && !flush_this_cycle && !block_queue_enq;
    assign queue_enq_data_1 = {fetch_prediction_ghr_1, fetch_predicted_target_1, fetch_predicted_taken_1, fetch_pc_1, fetch_instr_1};

    assign queue_deq = !queue_empty && rename_valid && !flush_this_cycle;

    // Instruction 1 dequeue - only if inst 0 dequeues AND inst 1 also dispatches
    assign queue_deq_1 = queue_deq && !queue_almost_empty && rename_valid_1 && !flush_this_cycle;

    queue #(.WIDTH(107), .DEPTH(8)) fetch_queue (
        .clk(clk),
        .rst(rst),
        .flush(flush_this_cycle),
        // Instruction 0
        .enq(queue_enq),
        .enq_data(queue_enq_data),
        .full(queue_full),
        .almost_full(queue_almost_full),
        .deq(queue_deq),
        .deq_data(queue_deq_data),
        .empty(queue_empty),
        .almost_empty(queue_almost_empty),
        // Instruction 1 (superscalar)
        .enq_1(queue_enq_1),
        .enq_data_1(queue_enq_data_1),
        .deq_1(queue_deq_1),
        .deq_data_1(queue_deq_data_1),
        .occupancy(queue_occupancy)
    );

    logic queue_needs_flush;
    assign queue_needs_flush = flush_this_cycle;

    logic queue_valid_internal;
    assign queue_valid_internal = !queue_empty && !queue_needs_flush;
    assign queue_valid = queue_valid_internal;

    logic queue_valid_1_internal;
    assign queue_valid_1_internal = queue_valid_internal && !queue_almost_empty;
    assign queue_valid_1 = queue_valid_1_internal;

    assign queue_prediction_ghr = queue_deq_data[105:97];
    assign queue_predicted_target = queue_deq_data[96:65];
    assign queue_predicted_taken = queue_deq_data[64];
    assign queue_pc = queue_deq_data[63:32];
    assign queue_inst = queue_deq_data[31:0];

    assign queue_prediction_ghr_1 = queue_deq_data_1[105:97];
    assign queue_predicted_target_1 = queue_deq_data_1[96:65];
    assign queue_predicted_taken_1 = queue_deq_data_1[64];
    assign queue_pc_1 = queue_deq_data_1[63:32];
    assign queue_inst_1 = queue_deq_data_1[31:0];

    // DECODE STAGE (2-WAY SUPERSCALAR)
    decode decode_stage (
        // Instruction 0
        .pc_in(queue_pc),
        .predicted_target_in(queue_predicted_target),
        .predicted_taken_in(queue_predicted_taken),
        .prediction_ghr_in(queue_prediction_ghr),
        .inst_in(queue_inst),
        .valid_in(queue_valid),
        .decoded_out(decoded_out),
        .valid_out(decode_valid),

        // Instruction 1 (superscalar)
        .pc_in_1(queue_pc_1),
        .predicted_target_in_1(queue_predicted_target_1),
        .predicted_taken_in_1(queue_predicted_taken_1),
        .prediction_ghr_in_1(queue_prediction_ghr_1),
        .inst_in_1(queue_inst_1),
        .valid_in_1(queue_valid_1),
        .decoded_out_1(decoded_out_1),
        .valid_out_1(decode_valid_1)
    );

    // RENAME STAGE (2-WAY SUPERSCALAR)
    rename #(
        .NUM_PREGS(NUM_PREGS),
        .ROB_SIZE(ROB_SIZE)
    ) rename_stage
    (
        // Instruction 0 - from Decode
         
        .decoded_in(decoded_out),
        .valid_in(decode_valid && !flush_this_cycle),

         
        // Instruction 1 - from Decode (superscalar)
         
        .decoded_in_1(decoded_out_1),
        .valid_in_1(decode_valid_1 && !flush_this_cycle),

         
        // RAT interface - Instruction 0
         
        .rat_rs1_areg(rat_rs1_areg),
        .rat_rs1_preg(rat_rs1_preg),
        .rat_rs1_ready(rat_rs1_ready),
        .rat_rs1_rob_id(rat_rs1_rob_id),  // Producer ROB ID for CDB validation
        .rat_rs2_areg(rat_rs2_areg),
        .rat_rs2_preg(rat_rs2_preg),
        .rat_rs2_ready(rat_rs2_ready),
        .rat_rs2_rob_id(rat_rs2_rob_id),  // Producer ROB ID for CDB validation
        .rat_alloc_en(rat_alloc_en),
        .rat_rd_areg(rat_rd_areg),
        .rat_rd_preg(rat_rd_preg),
        .rat_rd_rob_id(rat_rd_rob_id),  // ROB ID of the producer (for CDB validation)

         
        // RAT interface - Instruction 1 (superscalar)
         
        .rat_rs1_areg_1(rat_rs1_areg_1),
        .rat_rs1_preg_1(rat_rs1_preg_1),
        .rat_rs1_ready_1(rat_rs1_ready_1),
        .rat_rs1_rob_id_1(rat_rs1_rob_id_1),  // Producer ROB ID for CDB validation
        .rat_rs2_areg_1(rat_rs2_areg_1),
        .rat_rs2_preg_1(rat_rs2_preg_1),
        .rat_rs2_ready_1(rat_rs2_ready_1),
        .rat_rs2_rob_id_1(rat_rs2_rob_id_1),  // Producer ROB ID for CDB validation
        .rat_alloc_en_1(rat_alloc_en_1),
        .rat_rd_areg_1(rat_rd_areg_1),
        .rat_rd_preg_1(rat_rd_preg_1),
        .rat_rd_rob_id_1(rat_rd_rob_id_1),  // ROB ID of the producer (for CDB validation)

         
        // Free list interface - Instruction 0
         
        .fl_alloc(fl_alloc),
        .fl_alloc_preg(fl_alloc_preg),
        .fl_empty(fl_empty),

         
        // Free list interface - Instruction 1 (superscalar)
         
        .fl_alloc_1(fl_alloc_1),
        .fl_alloc_preg_1(fl_alloc_preg_1),
        .fl_almost_empty(fl_almost_empty),

         
        // ROB interface - Instruction 0
         
        .rob_enq(rob_enq),
        .rob_enq_data(rob_enq_data),
        .rob_idx(rob_idx),
        .rob_full(rob_full),

         
        // ROB interface - Instruction 1 (superscalar)
         
        .rob_enq_1(rob_enq_1),
        .rob_enq_data_1(rob_enq_data_1),
        .rob_idx_1(rob_idx_1),
        .rob_almost_full(rob_almost_full),

         
        // RS full signals
         
        .alu_rs_full(alu_rs_full),
        .alu_rs_almost_full(alu_rs_almost_full),  // For dual ALU dispatch check
        .mul_rs_full(mul_rs_full),
        .div_rs_full(div_rs_full),
        .branch_rs_full(branch_rs_full),
        .lq_full(lq_full),
        .sq_full(sq_full),

         
        // RS occupancy signals (for dual dispatch structural hazard checking)
         
        .mul_rs_occupancy(mul_rs_occupancy),
        .div_rs_occupancy(div_rs_occupancy),
        .branch_rs_occupancy(branch_rs_occupancy),
        .lq_occupancy(lq_occupancy),
        .sq_occupancy(sq_occupancy),

         
        // PRF interface - Instruction 0
         
        .prf_rs1_preg(prf_rs1_preg_rename),
        .prf_rs1_data(prf_rs1_data_rename),
        .prf_rs2_preg(prf_rs2_preg_rename),
        .prf_rs2_data(prf_rs2_data_rename),

         
        // PRF interface - Instruction 1 (superscalar)
         
        .prf_rs1_preg_1(prf_rs1_preg_rename_1),
        .prf_rs1_data_1(prf_rs1_data_rename_1),
        .prf_rs2_preg_1(prf_rs2_preg_rename_1),
        .prf_rs2_data_1(prf_rs2_data_rename_1),

         
        // RRF interface (for RVFI)
         
        .rrf_rs1_areg(rrf_rvfi_rs1_areg),
        .rrf_rs2_areg(rrf_rvfi_rs2_areg),
        .rrf_rs1_areg_1(rrf_rvfi_rs1_areg_1),
        .rrf_rs2_areg_1(rrf_rvfi_rs2_areg_1),

         
        // CDB and forwarding
         
        .cdb(cdb),
        .alu_issue_valid(alu_0_prf_grant),
        .alu_dest_preg(alu_0_rs_issue_entry.dest_preg),
        .alu_result(alu_0_result),

         
        // Output - Instruction 0
         
        .renamed_out(renamed_out),
        .valid_out(rename_valid),

         
        // Output - Instruction 1 (superscalar)
         
        .renamed_out_1(renamed_out_1),
        .valid_out_1(rename_valid_1),

        .stall_to_fetch(stall_to_fetch),
        .predicted_taken(queue_predicted_taken),
        .predicted_taken_1(queue_predicted_taken_1),
        .flush(flush_this_cycle)
    );

    logic prev_rob_enq;
    logic [5:0] prev_rob_dest_preg;

    always_ff @(posedge clk) begin
        if (!rst) begin
            
            prev_rob_enq <= rob_enq;
            prev_rob_dest_preg <= rob_enq_data.dest_preg;
        end
    end

    // PHYSICAL REGISTER FILE (2-WAY SUPERSCALAR)
    prf #(
        .NUM_PREGS(NUM_PREGS),
        .NUM_EXEC_READ_PORTS(NUM_EXEC_READ_PORTS),
        .NUM_RENAME_READ_PORTS(NUM_RENAME_READ_PORTS)
    ) physical_regfile (
        .clk(clk),
        .rst(rst),
        // Rename stage read ports - instruction 0
        .rename_rs1_preg(prf_rs1_preg_rename),
        .rename_rs1_data(prf_rs1_data_rename),
        .rename_rs2_preg(prf_rs2_preg_rename),
        .rename_rs2_data(prf_rs2_data_rename),
        // Rename stage read ports - instruction 1 (superscalar)
        .rename_rs1_preg_1(prf_rs1_preg_rename_1),
        .rename_rs1_data_1(prf_rs1_data_rename_1),
        .rename_rs2_preg_1(prf_rs2_preg_rename_1),
        .rename_rs2_data_1(prf_rs2_data_rename_1),
        // execution stage read ports
        .exec_rs_preg(prf_exec_rs_preg),
        .exec_rs_data(prf_exec_rs_data),
        // CDB write ports
        .cdb(cdb)
    );

    // Priority: ALU_0 (0) > ALU_1 (1) > MUL (2) > DIV (3) > BRANCH (4) > LQ (5) > SQ (6)
    always_comb begin
        // requests to arbiter
        arb_exec_req[0] = alu_0_prf_req;
        arb_exec_req[1] = alu_1_prf_req;
        arb_exec_req[2] = mul_prf_req;
        arb_exec_req[3] = div_prf_req;
        arb_exec_req[4] = branch_prf_req;
        arb_exec_req[5] = lq_prf_req;
        arb_exec_req[6] = sq_prf_req;

        // register numbers to arbiter
        arb_exec_rs1_preg[0] = alu_0_rs1_preg;
        arb_exec_rs1_preg[1] = alu_1_rs1_preg;
        arb_exec_rs1_preg[2] = mul_rs1_preg;
        arb_exec_rs1_preg[3] = div_rs1_preg;
        arb_exec_rs1_preg[4] = branch_rs1_preg;
        arb_exec_rs1_preg[5] = lq_src1_preg;
        arb_exec_rs1_preg[6] = sq_src1_preg;

        arb_exec_rs2_preg[0] = alu_0_rs2_preg;
        arb_exec_rs2_preg[1] = alu_1_rs2_preg;
        arb_exec_rs2_preg[2] = mul_rs2_preg;
        arb_exec_rs2_preg[3] = div_rs2_preg;
        arb_exec_rs2_preg[4] = branch_rs2_preg;
        arb_exec_rs2_preg[5] = '0;
        arb_exec_rs2_preg[6] = sq_src2_preg;

        // grants from arbiter
        alu_0_prf_grant = arb_exec_grant[0];
        alu_1_prf_grant = arb_exec_grant[1];
        mul_prf_grant = arb_exec_grant[2];
        div_prf_grant = arb_exec_grant[3];
        branch_prf_grant = arb_exec_grant[4];
        lq_prf_grant = arb_exec_grant[5];
        sq_prf_grant = arb_exec_grant[6];

        // data from arbiter to exec units
        alu_0_rs1_data = arb_exec_rs1_data[0];
        alu_0_rs2_data = arb_exec_rs2_data[0];

        alu_1_rs1_data = arb_exec_rs1_data[1];
        alu_1_rs2_data = arb_exec_rs2_data[1];

        mul_rs1_data = arb_exec_rs1_data[2];
        mul_rs2_data = arb_exec_rs2_data[2];

        div_rs1_data = arb_exec_rs1_data[3];
        div_rs2_data = arb_exec_rs2_data[3];

        branch_rs1_data = arb_exec_rs1_data[4];
        branch_rs2_data = arb_exec_rs2_data[4];

        lq_src1_data = arb_exec_rs1_data[5];

        sq_src1_data = arb_exec_rs1_data[6];
        sq_src2_data = arb_exec_rs2_data[6];
    end

    // PRF READ ARBITER (6 requesters: ALU, MUL, DIV, BRANCH, LQ, SQ)
    prf_read_arbiter #(
        .NUM_EXEC_UNITS(NUM_EXEC_READ_PORTS),
        .NUM_EXEC_READ_PORTS(NUM_EXEC_READ_PORTS)
    ) prf_arb (
        // array-based intf
        .exec_req(arb_exec_req),
        .exec_rs1_preg(arb_exec_rs1_preg),
        .exec_rs2_preg(arb_exec_rs2_preg),
        .exec_grant(arb_exec_grant),
        .prf_exec_preg(prf_exec_rs_preg),
        .prf_exec_data(prf_exec_rs_data),
        .exec_rs1_data(arb_exec_rs1_data),
        .exec_rs2_data(arb_exec_rs2_data)
    );

    // RAT (2-WAY SUPERSCALAR)
    rat #(
        .NUM_PREGS(NUM_PREGS)
    ) register_alias_table
    (
        .clk(clk),
        .rst(rst),
        .flush(flush),
        .rat_mappings(rat_mappings),
        .rrf_mapping(rrf_mappings),
        // Instruction 0
        .rs1_areg(rat_rs1_areg),
        .rs1_preg(rat_rs1_preg),
        .rs1_ready(rat_rs1_ready),
        .rs1_rob_id(rat_rs1_rob_id),  // Producer ROB ID for CDB validation
        .rs2_areg(rat_rs2_areg),
        .rs2_preg(rat_rs2_preg),
        .rs2_ready(rat_rs2_ready),
        .rs2_rob_id(rat_rs2_rob_id),  // Producer ROB ID for CDB validation
        .alloc_en(rat_alloc_en),
        .rd_areg(rat_rd_areg),
        .rd_preg(rat_rd_preg),
        .rd_rob_id(rat_rd_rob_id),  // ROB ID of the producer (for CDB validation)
        // Instruction 1 (superscalar)
        .rs1_areg_1(rat_rs1_areg_1),
        .rs1_preg_1(rat_rs1_preg_1),
        .rs1_ready_1(rat_rs1_ready_1),
        .rs1_rob_id_1(rat_rs1_rob_id_1),  // Producer ROB ID for CDB validation
        .rs2_areg_1(rat_rs2_areg_1),
        .rs2_preg_1(rat_rs2_preg_1),
        .rs2_ready_1(rat_rs2_ready_1),
        .rs2_rob_id_1(rat_rs2_rob_id_1),  // Producer ROB ID for CDB validation
        .alloc_en_1(rat_alloc_en_1),
        .rd_areg_1(rat_rd_areg_1),
        .rd_preg_1(rat_rd_preg_1),
        .rd_rob_id_1(rat_rd_rob_id_1),  // ROB ID of the producer (for CDB validation)
        // CDB wakeup
        .cdb(cdb)
    );

    // FREE LIST (2-WAY SUPERSCALAR)
    free_list #(
        .NUM_PREGS(NUM_PREGS)
    ) freelist
    (
        .clk(clk),
        .rst(rst),
        // Instruction 0 allocation
        .alloc(fl_alloc),
        .alloc_preg(fl_alloc_preg),
        .empty(fl_empty),
        // Instruction 1 allocation (superscalar)
        .alloc_1(fl_alloc_1),
        .alloc_preg_1(fl_alloc_preg_1),
        .almost_empty(fl_almost_empty),
        // Free port 0 (commit)
        .free(fl_free),
        .free_preg(fl_free_preg),
        .full(fl_full),
        // Free port 1 (dual commit)
        .free_1(fl_free_1),
        .free_preg_1(fl_free_preg_1),
        // Flush recovery
        .flush_free_en(rob_flush_occurred),
        .flush_free_count(rob_flush_free_count),
        .flush_free_pregs(rob_flush_free_pregs),
        // Profiling
        .occupancy_out(fl_occupancy)
    );


    logic [4:0] rob_head_ptr, rob_tail_ptr;

    // RE-ORDER BUFFER (2-WAY SUPERSCALAR)
    rob #(
        .SIZE(ROB_SIZE)
    ) reorder_buffer (
        .clk(clk),
        .rst(rst),
        // Instruction 0 enqueue
        .enq(rob_enq && !flush),
        .enq_data(rob_enq_data),
        .rob_idx(rob_idx),
        .full(rob_full),
        // Instruction 1 enqueue (superscalar)
        .enq_1(rob_enq_1 && !flush),
        .enq_data_1(rob_enq_data_1),
        .rob_idx_1(rob_idx_1),
        .almost_full(rob_almost_full),
        // CDB wakeup
        .cdb(cdb),
        // Commit port 0
        .head_data(rob_head),
        .head_valid(rob_head_valid),
        .head_ready(rob_head_ready),
        .commit(rob_commit),
        // Commit port 1 (dual commit)
        .head_data_1(rob_head_1),
        .head_valid_1(rob_head_valid_1),
        .head_ready_1(rob_head_ready_1),
        .commit_1(rob_commit_1),
        .empty(rob_empty),

        .head_ptr(rob_head_ptr),
        .tail_ptr(rob_tail_ptr),

        .branch_mispredict(rob_branch_mispredict),
        .branch_correct_target(rob_correct_target),

        .head_branch_mispredict(head_branch_mispredict),

        .flush_free_count(rob_flush_free_count),
        .flush_free_pregs(rob_flush_free_pregs),
        .flush_occurred(rob_flush_occurred),

        .flush_this_cycle(flush_this_cycle),

        // Load update port from LSQ
        .load_update_mem_info(lsq_load_update_mem_info),
        .load_update_id(lsq_load_update_id),
        .load_update_addr(lsq_load_update_addr),
        .load_update_rmask(lsq_load_update_rmask),
        .load_update_rdata(lsq_load_update_rdata),

        // Store update port from LSQ
        .store_update_mem_info(lsq_store_update_mem_info),
        .store_update_id(lsq_store_update_id),
        .store_update_addr(lsq_store_update_addr),
        .store_update_wmask(lsq_store_update_wmask),
        .store_update_wdata(lsq_store_update_wdata),
        .store_ready(lsq_store_ready)
    );

    // RRF
    rrf #(
        .NUM_PREGS(NUM_PREGS)
    ) retirement_regfile
    (
        .clk(clk),
        .rst(rst),
        // port 0
        .read_areg(rrf_read_areg),
        .old_preg(rrf_old_preg),
        // port 1 (dual commit)
        .read_areg_1(rrf_read_areg_1),
        .old_preg_1(rrf_old_preg_1),
        // commit update - port 0
        .commit_en(rrf_commit_en),
        .commit_areg(rrf_commit_areg),
        .commit_preg(rrf_commit_preg),
        // commit update - port 1 (dual commit)
        .commit_en_1(rrf_commit_en_1),
        .commit_areg_1(rrf_commit_areg_1),
        .commit_preg_1(rrf_commit_preg_1),
        .rrf_mappings(rrf_mappings)
    );

    // DISPATCH LOGIC

    // ALU dispatch routing toggle (round-robin between ALU_0 and ALU_1)
    always_ff @(posedge clk) begin
        if (rst || flush_this_cycle) begin
            alu_dispatch_toggle <= 1'b0;
        end else if (rename_valid && (renamed_out.inst_type == ITYPE_ALU) && !flush) begin
            alu_dispatch_toggle <= ~alu_dispatch_toggle;
        end
    end

    // Determine which ALU RS to dispatch to based on toggle and fullness
    logic dispatch_to_alu_0, dispatch_to_alu_1;
    logic is_alu_inst;
    assign is_alu_inst = rename_valid && (renamed_out.inst_type == ITYPE_ALU) && !flush;

    always_comb begin
        dispatch_to_alu_0 = 1'b0;
        dispatch_to_alu_1 = 1'b0;

        if (is_alu_inst) begin
            // Use round-robin, but fall back if target is full
            if (alu_dispatch_toggle == 1'b0) begin
                // Prefer ALU_0
                if (!alu_0_rs_full)
                    dispatch_to_alu_0 = 1'b1;
                else if (!alu_1_rs_full)
                    dispatch_to_alu_1 = 1'b1;
            end else begin
                // Prefer ALU_1
                if (!alu_1_rs_full)
                    dispatch_to_alu_1 = 1'b1;
                else if (!alu_0_rs_full)
                    dispatch_to_alu_0 = 1'b1;
            end
        end
    end

    // ALU_0 RS dispatch entry
    always_comb begin
        alu_0_rs_dispatch_entry = '0;
        alu_0_rs_dispatch_entry.valid = dispatch_to_alu_0;
        alu_0_rs_dispatch_entry.dest_preg = renamed_out.rd_preg;
        alu_0_rs_dispatch_entry.rob_id = renamed_out.rob_id;
        alu_0_rs_dispatch_entry.src1_preg = renamed_out.rs1_preg;
        alu_0_rs_dispatch_entry.src1_rob_id = rat_rs1_rob_id;  // Producer ROB ID for CDB validation
        alu_0_rs_dispatch_entry.src1_ready = renamed_out.rs1_ready;
        alu_0_rs_dispatch_entry.src2_preg = renamed_out.rs2_preg;
        alu_0_rs_dispatch_entry.src2_rob_id = rat_rs2_rob_id;  // Producer ROB ID for CDB validation
        alu_0_rs_dispatch_entry.src2_imm = renamed_out.rs2_imm;
        alu_0_rs_dispatch_entry.src2_is_imm = renamed_out.rs2_is_imm;
        alu_0_rs_dispatch_entry.src2_ready = renamed_out.rs2_ready;
        alu_0_rs_dispatch_entry.alu_op = renamed_out.alu_op;
        alu_0_rs_dispatch_entry.inst_type = renamed_out.inst_type;
        alu_0_rs_dispatch_entry.pc = renamed_out.pc;
    end

    assign alu_0_rs_dispatch_valid = dispatch_to_alu_0;

    // ALU_1 RS dispatch entry
    always_comb begin
        alu_1_rs_dispatch_entry = '0;
        alu_1_rs_dispatch_entry.valid = dispatch_to_alu_1;
        alu_1_rs_dispatch_entry.dest_preg = renamed_out.rd_preg;
        alu_1_rs_dispatch_entry.rob_id = renamed_out.rob_id;
        alu_1_rs_dispatch_entry.src1_preg = renamed_out.rs1_preg;
        alu_1_rs_dispatch_entry.src1_rob_id = rat_rs1_rob_id;  // Producer ROB ID for CDB validation
        alu_1_rs_dispatch_entry.src1_ready = renamed_out.rs1_ready;
        alu_1_rs_dispatch_entry.src2_preg = renamed_out.rs2_preg;
        alu_1_rs_dispatch_entry.src2_rob_id = rat_rs2_rob_id;  // Producer ROB ID for CDB validation
        alu_1_rs_dispatch_entry.src2_imm = renamed_out.rs2_imm;
        alu_1_rs_dispatch_entry.src2_is_imm = renamed_out.rs2_is_imm;
        alu_1_rs_dispatch_entry.src2_ready = renamed_out.rs2_ready;
        alu_1_rs_dispatch_entry.alu_op = renamed_out.alu_op;
        alu_1_rs_dispatch_entry.inst_type = renamed_out.inst_type;
        alu_1_rs_dispatch_entry.pc = renamed_out.pc;
    end

    assign alu_1_rs_dispatch_valid = dispatch_to_alu_1;

    // MUL RS dispatch
    always_comb begin
        mul_rs_dispatch_entry = '0;
        mul_rs_dispatch_entry.valid = rename_valid && (renamed_out.inst_type == ITYPE_MUL) && !flush;
        mul_rs_dispatch_entry.dest_preg = renamed_out.rd_preg;
        mul_rs_dispatch_entry.rob_id = renamed_out.rob_id;
        mul_rs_dispatch_entry.src1_preg = renamed_out.rs1_preg;
        mul_rs_dispatch_entry.src1_rob_id = rat_rs1_rob_id;  // Producer ROB ID for CDB validation
        mul_rs_dispatch_entry.src1_ready = renamed_out.rs1_ready;
        mul_rs_dispatch_entry.src2_preg = renamed_out.rs2_preg;
        mul_rs_dispatch_entry.src2_rob_id = rat_rs2_rob_id;  // Producer ROB ID for CDB validation
        mul_rs_dispatch_entry.src2_ready = renamed_out.rs2_ready;
        mul_rs_dispatch_entry.mul_op = renamed_out.mul_op;
        mul_rs_dispatch_entry.src2_is_imm = 1'b0;
        mul_rs_dispatch_entry.inst_type = renamed_out.inst_type;
        mul_rs_dispatch_entry.pc = renamed_out.pc;
        mul_rs_dispatch_entry.inst = renamed_out.inst;
    end
    assign mul_rs_dispatch_valid = rename_valid && (renamed_out.inst_type == ITYPE_MUL) && !flush;

    // DIV RS dispatch
    always_comb begin
        div_rs_dispatch_entry = '0;
        div_rs_dispatch_entry.valid = rename_valid && (renamed_out.inst_type == ITYPE_DIV);
        div_rs_dispatch_entry.dest_preg = renamed_out.rd_preg;
        div_rs_dispatch_entry.rob_id = renamed_out.rob_id;
        div_rs_dispatch_entry.src1_preg = renamed_out.rs1_preg;
        div_rs_dispatch_entry.src1_rob_id = rat_rs1_rob_id;  // Producer ROB ID for CDB validation
        div_rs_dispatch_entry.src1_ready = renamed_out.rs1_ready;
        div_rs_dispatch_entry.src2_preg = renamed_out.rs2_preg;
        div_rs_dispatch_entry.src2_rob_id = rat_rs2_rob_id;  // Producer ROB ID for CDB validation
        div_rs_dispatch_entry.src2_ready = renamed_out.rs2_ready;
        div_rs_dispatch_entry.div_op = renamed_out.div_op;
        div_rs_dispatch_entry.src2_is_imm = 1'b0;
        div_rs_dispatch_entry.inst_type = renamed_out.inst_type;
        div_rs_dispatch_entry.pc = renamed_out.pc;
    end
    assign div_rs_dispatch_valid = rename_valid && (renamed_out.inst_type == ITYPE_DIV);

    // BRANCH RS dispatch
    logic is_branch_inst, is_jal_inst, is_jalr_inst;

    always_comb begin
        branch_rs_dispatch_entry = '0;
        
        is_branch_inst = (renamed_out.inst_type == ITYPE_BRANCH);
        is_jal_inst = (renamed_out.inst_type == ITYPE_JAL);
        is_jalr_inst = (renamed_out.inst_type == ITYPE_JALR);
        
        branch_rs_dispatch_entry.valid = rename_valid && !flush && (is_branch_inst || is_jal_inst || is_jalr_inst);
        
        branch_rs_dispatch_entry.dest_preg = renamed_out.rd_preg;
        branch_rs_dispatch_entry.rob_id = renamed_out.rob_id;
        branch_rs_dispatch_entry.branch_op = renamed_out.branch_op;
        branch_rs_dispatch_entry.inst_type = renamed_out.inst_type;
        branch_rs_dispatch_entry.pc = renamed_out.pc;
        branch_rs_dispatch_entry.inst = renamed_out.inst;
        
        if (is_branch_inst) begin
            // BRANCH: rs1 and rs2 are register operands
            branch_rs_dispatch_entry.src1_preg = renamed_out.rs1_preg;
            branch_rs_dispatch_entry.src1_rob_id = rat_rs1_rob_id;  // Producer ROB ID for CDB validation
            branch_rs_dispatch_entry.src1_ready = renamed_out.rs1_ready;
            branch_rs_dispatch_entry.src2_preg = renamed_out.rs2_preg;
            branch_rs_dispatch_entry.src2_rob_id = rat_rs2_rob_id;  // Producer ROB ID for CDB validation
            branch_rs_dispatch_entry.src2_ready = renamed_out.rs2_ready;
            branch_rs_dispatch_entry.src2_is_imm = 1'b0;
            branch_rs_dispatch_entry.src2_imm = renamed_out.branch_target - renamed_out.pc;
        end
        else if (is_jal_inst) begin
            branch_rs_dispatch_entry.src1_preg = 6'b0;
            branch_rs_dispatch_entry.src1_rob_id = 5'b0;  // No producer (p0)
            branch_rs_dispatch_entry.src1_ready = 1'b1;
            branch_rs_dispatch_entry.src2_preg = 6'b0;
            branch_rs_dispatch_entry.src2_rob_id = 5'b0;  // No producer (p0)
            branch_rs_dispatch_entry.src2_ready = 1'b1;
            branch_rs_dispatch_entry.src2_is_imm = 1'b1;
            branch_rs_dispatch_entry.src2_imm = renamed_out.branch_target - renamed_out.pc;
        end
        else if (is_jalr_inst) begin
            branch_rs_dispatch_entry.src1_preg = renamed_out.rs1_preg;
            branch_rs_dispatch_entry.src1_rob_id = rat_rs1_rob_id;  // Producer ROB ID for CDB validation
            branch_rs_dispatch_entry.src1_ready = renamed_out.rs1_ready;
            branch_rs_dispatch_entry.src2_preg = 6'b0;
            branch_rs_dispatch_entry.src2_rob_id = 5'b0;  // No producer (immediate)
            branch_rs_dispatch_entry.src2_ready = 1'b1;
            branch_rs_dispatch_entry.src2_is_imm = 1'b1;
            branch_rs_dispatch_entry.src2_imm = renamed_out.rs2_imm;
        end
    end

    assign branch_rs_dispatch_valid = branch_rs_dispatch_entry.valid;

    // SPLIT LSQ DISPATCH
    logic is_load_inst, is_store_inst;

    logic [$clog2(NUM_SQ_ENTRIES):0] sq_tail_capture;

    always_comb begin
        is_load_inst = (renamed_out.inst_type == ITYPE_LOAD);
        is_store_inst = (renamed_out.inst_type == ITYPE_STORE);

        // LOAD QUEUE DISPATCH ENTRY
        lq_dispatch_entry = '0;
        lq_dispatch_entry.valid = rename_valid && !flush && is_load_inst;
        lq_dispatch_entry.rob_id = renamed_out.rob_id;
        lq_dispatch_entry.src1_preg = renamed_out.rs1_preg;
        lq_dispatch_entry.src1_rob_id = rat_rs1_rob_id;  // Producer ROB ID for CDB validation
        lq_dispatch_entry.src1_ready = renamed_out.rs1_ready;
        lq_dispatch_entry.offset = renamed_out.rs2_imm;
        lq_dispatch_entry.dest_preg = renamed_out.rd_preg;
        lq_dispatch_entry.mem_op = renamed_out.mem_op;
        lq_dispatch_entry.pc = renamed_out.pc;
        lq_dispatch_entry.inst = renamed_out.inst;
        lq_dispatch_entry.addr_ready = 1'b0;
        lq_dispatch_entry.address = 32'h0;
        lq_dispatch_entry.executed = 1'b0;
        lq_dispatch_entry.sq_tail_capture = sq_tail_capture;

        // STORE QUEUE DISPATCH ENTRY
        sq_dispatch_entry = '0;
        sq_dispatch_entry.valid = rename_valid && !flush && is_store_inst;
        sq_dispatch_entry.rob_id = renamed_out.rob_id;
        sq_dispatch_entry.src1_preg = renamed_out.rs1_preg;
        sq_dispatch_entry.src1_rob_id = rat_rs1_rob_id;  // Producer ROB ID for CDB validation
        sq_dispatch_entry.src1_ready = renamed_out.rs1_ready;
        sq_dispatch_entry.src2_preg = renamed_out.rs2_preg;
        sq_dispatch_entry.src2_rob_id = rat_rs2_rob_id;  // Producer ROB ID for CDB validation
        sq_dispatch_entry.src2_ready = renamed_out.rs2_ready;
        sq_dispatch_entry.offset = renamed_out.rs2_imm;
        sq_dispatch_entry.mem_op = renamed_out.mem_op;
        sq_dispatch_entry.pc = renamed_out.pc;
        sq_dispatch_entry.inst = renamed_out.inst;
        sq_dispatch_entry.addr_ready = 1'b0;
        sq_dispatch_entry.address = 32'h0;
        sq_dispatch_entry.executed = 1'b0;
        sq_dispatch_entry.cdb_broadcast = 1'b0;
        sq_dispatch_entry.committed = 1'b0;

        if (is_store_inst && renamed_out.rs2_ready)
            sq_dispatch_entry.store_data = renamed_out.rs2_data;
        else
            sq_dispatch_entry.store_data = 32'h0;
    end

    assign lq_dispatch_valid = rename_valid && is_load_inst && !flush;
    assign sq_dispatch_valid = rename_valid && is_store_inst && !flush;

    logic lsq_dispatch_valid;
    assign lsq_dispatch_valid = lq_dispatch_valid || sq_dispatch_valid;

     
    // INSTRUCTION 1 DISPATCH LOGIC (2-way superscalar)
     

    // Determine which ALU RS to dispatch instruction 1 to (opposite of inst 0)
    logic dispatch_1_to_alu_0, dispatch_1_to_alu_1;
    logic is_alu_inst_1;
    assign is_alu_inst_1 = rename_valid_1 && (renamed_out_1.inst_type == ITYPE_ALU) && !flush;

    always_comb begin
        dispatch_1_to_alu_0 = 1'b0;
        dispatch_1_to_alu_1 = 1'b0;

        if (is_alu_inst_1) begin
            // Dispatch instruction 1 to the opposite RS from instruction 0
            // If inst 0 went to ALU_0, inst 1 goes to ALU_1 and vice versa
            if (dispatch_to_alu_0) begin
                // Inst 0 going to ALU_0, so inst 1 goes to ALU_1
                if (!alu_1_rs_full)
                    dispatch_1_to_alu_1 = 1'b1;
            end else if (dispatch_to_alu_1) begin
                // Inst 0 going to ALU_1, so inst 1 goes to ALU_0
                if (!alu_0_rs_full)
                    dispatch_1_to_alu_0 = 1'b1;
            end else begin
                // Inst 0 is not ALU, use round-robin for inst 1
                if (alu_dispatch_toggle == 1'b0) begin
                    if (!alu_0_rs_full)
                        dispatch_1_to_alu_0 = 1'b1;
                    else if (!alu_1_rs_full)
                        dispatch_1_to_alu_1 = 1'b1;
                end else begin
                    if (!alu_1_rs_full)
                        dispatch_1_to_alu_1 = 1'b1;
                    else if (!alu_0_rs_full)
                        dispatch_1_to_alu_0 = 1'b1;
                end
            end
        end
    end

    // ALU_0 RS dispatch entry for instruction 1
    always_comb begin
        alu_0_rs_dispatch_entry_1 = '0;
        alu_0_rs_dispatch_entry_1.valid = dispatch_1_to_alu_0;
        alu_0_rs_dispatch_entry_1.dest_preg = renamed_out_1.rd_preg;
        alu_0_rs_dispatch_entry_1.rob_id = renamed_out_1.rob_id;
        alu_0_rs_dispatch_entry_1.src1_preg = renamed_out_1.rs1_preg;
        alu_0_rs_dispatch_entry_1.src1_rob_id = rat_rs1_rob_id_1;  // Producer ROB ID for CDB validation
        alu_0_rs_dispatch_entry_1.src1_ready = renamed_out_1.rs1_ready;
        alu_0_rs_dispatch_entry_1.src2_preg = renamed_out_1.rs2_preg;
        alu_0_rs_dispatch_entry_1.src2_rob_id = rat_rs2_rob_id_1;  // Producer ROB ID for CDB validation
        alu_0_rs_dispatch_entry_1.src2_imm = renamed_out_1.rs2_imm;
        alu_0_rs_dispatch_entry_1.src2_is_imm = renamed_out_1.rs2_is_imm;
        alu_0_rs_dispatch_entry_1.src2_ready = renamed_out_1.rs2_ready;
        alu_0_rs_dispatch_entry_1.alu_op = renamed_out_1.alu_op;
        alu_0_rs_dispatch_entry_1.inst_type = renamed_out_1.inst_type;
        alu_0_rs_dispatch_entry_1.pc = renamed_out_1.pc;
    end
    assign alu_0_rs_dispatch_valid_1 = dispatch_1_to_alu_0;

    // ALU_1 RS dispatch entry for instruction 1
    always_comb begin
        alu_1_rs_dispatch_entry_1 = '0;
        alu_1_rs_dispatch_entry_1.valid = dispatch_1_to_alu_1;
        alu_1_rs_dispatch_entry_1.dest_preg = renamed_out_1.rd_preg;
        alu_1_rs_dispatch_entry_1.rob_id = renamed_out_1.rob_id;
        alu_1_rs_dispatch_entry_1.src1_preg = renamed_out_1.rs1_preg;
        alu_1_rs_dispatch_entry_1.src1_rob_id = rat_rs1_rob_id_1;  // Producer ROB ID for CDB validation
        alu_1_rs_dispatch_entry_1.src1_ready = renamed_out_1.rs1_ready;
        alu_1_rs_dispatch_entry_1.src2_preg = renamed_out_1.rs2_preg;
        alu_1_rs_dispatch_entry_1.src2_rob_id = rat_rs2_rob_id_1;  // Producer ROB ID for CDB validation
        alu_1_rs_dispatch_entry_1.src2_imm = renamed_out_1.rs2_imm;
        alu_1_rs_dispatch_entry_1.src2_is_imm = renamed_out_1.rs2_is_imm;
        alu_1_rs_dispatch_entry_1.src2_ready = renamed_out_1.rs2_ready;
        alu_1_rs_dispatch_entry_1.alu_op = renamed_out_1.alu_op;
        alu_1_rs_dispatch_entry_1.inst_type = renamed_out_1.inst_type;
        alu_1_rs_dispatch_entry_1.pc = renamed_out_1.pc;
    end
    assign alu_1_rs_dispatch_valid_1 = dispatch_1_to_alu_1;

    // MUL RS dispatch entry for instruction 1
    always_comb begin
        mul_rs_dispatch_entry_1 = '0;
        mul_rs_dispatch_entry_1.valid = rename_valid_1 && (renamed_out_1.inst_type == ITYPE_MUL) && !flush;
        mul_rs_dispatch_entry_1.dest_preg = renamed_out_1.rd_preg;
        mul_rs_dispatch_entry_1.rob_id = renamed_out_1.rob_id;
        mul_rs_dispatch_entry_1.src1_preg = renamed_out_1.rs1_preg;
        mul_rs_dispatch_entry_1.src1_rob_id = rat_rs1_rob_id_1;  // Producer ROB ID for CDB validation
        mul_rs_dispatch_entry_1.src1_ready = renamed_out_1.rs1_ready;
        mul_rs_dispatch_entry_1.src2_preg = renamed_out_1.rs2_preg;
        mul_rs_dispatch_entry_1.src2_rob_id = rat_rs2_rob_id_1;  // Producer ROB ID for CDB validation
        mul_rs_dispatch_entry_1.src2_ready = renamed_out_1.rs2_ready;
        mul_rs_dispatch_entry_1.mul_op = renamed_out_1.mul_op;
        mul_rs_dispatch_entry_1.src2_is_imm = 1'b0;
        mul_rs_dispatch_entry_1.inst_type = renamed_out_1.inst_type;
        mul_rs_dispatch_entry_1.pc = renamed_out_1.pc;
        mul_rs_dispatch_entry_1.inst = renamed_out_1.inst;
    end
    assign mul_rs_dispatch_valid_1 = rename_valid_1 && (renamed_out_1.inst_type == ITYPE_MUL) && !flush;

    // DIV RS dispatch entry for instruction 1
    always_comb begin
        div_rs_dispatch_entry_1 = '0;
        div_rs_dispatch_entry_1.valid = rename_valid_1 && (renamed_out_1.inst_type == ITYPE_DIV);
        div_rs_dispatch_entry_1.dest_preg = renamed_out_1.rd_preg;
        div_rs_dispatch_entry_1.rob_id = renamed_out_1.rob_id;
        div_rs_dispatch_entry_1.src1_preg = renamed_out_1.rs1_preg;
        div_rs_dispatch_entry_1.src1_rob_id = rat_rs1_rob_id_1;  // Producer ROB ID for CDB validation
        div_rs_dispatch_entry_1.src1_ready = renamed_out_1.rs1_ready;
        div_rs_dispatch_entry_1.src2_preg = renamed_out_1.rs2_preg;
        div_rs_dispatch_entry_1.src2_rob_id = rat_rs2_rob_id_1;  // Producer ROB ID for CDB validation
        div_rs_dispatch_entry_1.src2_ready = renamed_out_1.rs2_ready;
        div_rs_dispatch_entry_1.div_op = renamed_out_1.div_op;
        div_rs_dispatch_entry_1.src2_is_imm = 1'b0;
        div_rs_dispatch_entry_1.inst_type = renamed_out_1.inst_type;
        div_rs_dispatch_entry_1.pc = renamed_out_1.pc;
    end
    assign div_rs_dispatch_valid_1 = rename_valid_1 && (renamed_out_1.inst_type == ITYPE_DIV);

    // BRANCH RS dispatch entry for instruction 1
    logic is_branch_inst_1, is_jal_inst_1, is_jalr_inst_1;

    always_comb begin
        branch_rs_dispatch_entry_1 = '0;

        is_branch_inst_1 = (renamed_out_1.inst_type == ITYPE_BRANCH);
        is_jal_inst_1 = (renamed_out_1.inst_type == ITYPE_JAL);
        is_jalr_inst_1 = (renamed_out_1.inst_type == ITYPE_JALR);

        branch_rs_dispatch_entry_1.valid = rename_valid_1 && !flush && (is_branch_inst_1 || is_jal_inst_1 || is_jalr_inst_1);

        branch_rs_dispatch_entry_1.dest_preg = renamed_out_1.rd_preg;
        branch_rs_dispatch_entry_1.rob_id = renamed_out_1.rob_id;
        branch_rs_dispatch_entry_1.branch_op = renamed_out_1.branch_op;
        branch_rs_dispatch_entry_1.inst_type = renamed_out_1.inst_type;
        branch_rs_dispatch_entry_1.pc = renamed_out_1.pc;
        branch_rs_dispatch_entry_1.inst = renamed_out_1.inst;

        if (is_branch_inst_1) begin
            branch_rs_dispatch_entry_1.src1_preg = renamed_out_1.rs1_preg;
            branch_rs_dispatch_entry_1.src1_rob_id = rat_rs1_rob_id_1;  // Producer ROB ID for CDB validation
            branch_rs_dispatch_entry_1.src1_ready = renamed_out_1.rs1_ready;
            branch_rs_dispatch_entry_1.src2_preg = renamed_out_1.rs2_preg;
            branch_rs_dispatch_entry_1.src2_rob_id = rat_rs2_rob_id_1;  // Producer ROB ID for CDB validation
            branch_rs_dispatch_entry_1.src2_ready = renamed_out_1.rs2_ready;
            branch_rs_dispatch_entry_1.src2_is_imm = 1'b0;
            branch_rs_dispatch_entry_1.src2_imm = renamed_out_1.branch_target - renamed_out_1.pc;
        end
        else if (is_jal_inst_1) begin
            branch_rs_dispatch_entry_1.src1_preg = 6'b0;
            branch_rs_dispatch_entry_1.src1_rob_id = 5'b0;  // No producer (p0)
            branch_rs_dispatch_entry_1.src1_ready = 1'b1;
            branch_rs_dispatch_entry_1.src2_preg = 6'b0;
            branch_rs_dispatch_entry_1.src2_rob_id = 5'b0;  // No producer (p0)
            branch_rs_dispatch_entry_1.src2_ready = 1'b1;
            branch_rs_dispatch_entry_1.src2_is_imm = 1'b1;
            branch_rs_dispatch_entry_1.src2_imm = renamed_out_1.branch_target - renamed_out_1.pc;
        end
        else if (is_jalr_inst_1) begin
            branch_rs_dispatch_entry_1.src1_preg = renamed_out_1.rs1_preg;
            branch_rs_dispatch_entry_1.src1_rob_id = rat_rs1_rob_id_1;  // Producer ROB ID for CDB validation
            branch_rs_dispatch_entry_1.src1_ready = renamed_out_1.rs1_ready;
            branch_rs_dispatch_entry_1.src2_preg = 6'b0;
            branch_rs_dispatch_entry_1.src2_rob_id = 5'b0;  // No producer (immediate)
            branch_rs_dispatch_entry_1.src2_ready = 1'b1;
            branch_rs_dispatch_entry_1.src2_is_imm = 1'b1;
            branch_rs_dispatch_entry_1.src2_imm = renamed_out_1.rs2_imm;
        end
    end

    assign branch_rs_dispatch_valid_1 = branch_rs_dispatch_entry_1.valid;

    // LSQ dispatch for instruction 1
    logic is_load_inst_1, is_store_inst_1;

    always_comb begin
        is_load_inst_1 = (renamed_out_1.inst_type == ITYPE_LOAD);
        is_store_inst_1 = (renamed_out_1.inst_type == ITYPE_STORE);

        // LOAD QUEUE DISPATCH ENTRY - instruction 1
        lq_dispatch_entry_1 = '0;
        lq_dispatch_entry_1.valid = rename_valid_1 && !flush && is_load_inst_1;
        lq_dispatch_entry_1.rob_id = renamed_out_1.rob_id;
        lq_dispatch_entry_1.src1_preg = renamed_out_1.rs1_preg;
        lq_dispatch_entry_1.src1_rob_id = rat_rs1_rob_id_1;  // Producer ROB ID for CDB validation
        lq_dispatch_entry_1.src1_ready = renamed_out_1.rs1_ready;
        lq_dispatch_entry_1.offset = renamed_out_1.rs2_imm;
        lq_dispatch_entry_1.dest_preg = renamed_out_1.rd_preg;
        lq_dispatch_entry_1.mem_op = renamed_out_1.mem_op;
        lq_dispatch_entry_1.pc = renamed_out_1.pc;
        lq_dispatch_entry_1.inst = renamed_out_1.inst;
        lq_dispatch_entry_1.addr_ready = 1'b0;
        lq_dispatch_entry_1.address = 32'h0;
        lq_dispatch_entry_1.executed = 1'b0;
        // If inst 0 is also a store, sq_tail moves by 1, so capture sq_tail+1
        if (is_store_inst)
            lq_dispatch_entry_1.sq_tail_capture = ($clog2(NUM_SQ_ENTRIES)+1)'(sq_tail_capture + 1);
        else
            lq_dispatch_entry_1.sq_tail_capture = sq_tail_capture;

        // STORE QUEUE DISPATCH ENTRY - instruction 1
        sq_dispatch_entry_1 = '0;
        sq_dispatch_entry_1.valid = rename_valid_1 && !flush && is_store_inst_1;
        sq_dispatch_entry_1.rob_id = renamed_out_1.rob_id;
        sq_dispatch_entry_1.src1_preg = renamed_out_1.rs1_preg;
        sq_dispatch_entry_1.src1_rob_id = rat_rs1_rob_id_1;  // Producer ROB ID for CDB validation
        sq_dispatch_entry_1.src1_ready = renamed_out_1.rs1_ready;
        sq_dispatch_entry_1.src2_preg = renamed_out_1.rs2_preg;
        sq_dispatch_entry_1.src2_rob_id = rat_rs2_rob_id_1;  // Producer ROB ID for CDB validation
        sq_dispatch_entry_1.src2_ready = renamed_out_1.rs2_ready;
        sq_dispatch_entry_1.offset = renamed_out_1.rs2_imm;
        sq_dispatch_entry_1.mem_op = renamed_out_1.mem_op;
        sq_dispatch_entry_1.pc = renamed_out_1.pc;
        sq_dispatch_entry_1.inst = renamed_out_1.inst;
        sq_dispatch_entry_1.addr_ready = 1'b0;
        sq_dispatch_entry_1.address = 32'h0;
        sq_dispatch_entry_1.executed = 1'b0;
        sq_dispatch_entry_1.cdb_broadcast = 1'b0;
        sq_dispatch_entry_1.committed = 1'b0;

        if (is_store_inst_1 && renamed_out_1.rs2_ready)
            sq_dispatch_entry_1.store_data = renamed_out_1.rs2_data;
        else
            sq_dispatch_entry_1.store_data = 32'h0;
    end

    assign lq_dispatch_valid_1 = rename_valid_1 && is_load_inst_1 && !flush;
    assign sq_dispatch_valid_1 = rename_valid_1 && is_store_inst_1 && !flush;

    // ALU_0 RESERVATION STATION (2-way superscalar)
    reservation_station #(
        .NUM_ENTRIES(NUM_ALU_RS / 2)
    ) alu_0_rs (
        .clk(clk),
        .rst(rst),
        .flush(flush_this_cycle),
        .dispatch_valid(alu_0_rs_dispatch_valid),
        .dispatch_entry(alu_0_rs_dispatch_entry),
        .full(alu_0_rs_full),
        .almost_full(alu_0_rs_almost_full),
        .dispatch_valid_1(alu_0_rs_dispatch_valid_1),
        .dispatch_entry_1(alu_0_rs_dispatch_entry_1),
        .cdb(cdb),
        .issue_valid(alu_0_rs_issue_valid),
        .issue_entry(alu_0_rs_issue_entry),
        .execute_ready(alu_0_execute_ready),
        .prf_req(alu_0_prf_req),
        .prf_grant(alu_0_prf_grant),
        .occupancy(alu_0_rs_occupancy)
    );

    // ALU_1 RESERVATION STATION (2-way superscalar)
    reservation_station #(
        .NUM_ENTRIES(NUM_ALU_RS / 2)
    ) alu_1_rs (
        .clk(clk),
        .rst(rst),
        .flush(flush_this_cycle),
        .dispatch_valid(alu_1_rs_dispatch_valid),
        .dispatch_entry(alu_1_rs_dispatch_entry),
        .full(alu_1_rs_full),
        .almost_full(alu_1_rs_almost_full),
        .dispatch_valid_1(alu_1_rs_dispatch_valid_1),
        .dispatch_entry_1(alu_1_rs_dispatch_entry_1),
        .cdb(cdb),
        .issue_valid(alu_1_rs_issue_valid),
        .issue_entry(alu_1_rs_issue_entry),
        .execute_ready(alu_1_execute_ready),
        .prf_req(alu_1_prf_req),
        .prf_grant(alu_1_prf_grant),
        .occupancy(alu_1_rs_occupancy)
    );

    // MUL RESERVATION STATION (2-way superscalar)
    reservation_station #(
        .NUM_ENTRIES(NUM_MUL_RS)
    ) mul_rs (
        .clk(clk),
        .rst(rst),
        .flush(flush_this_cycle),
        .dispatch_valid(mul_rs_dispatch_valid),
        .dispatch_entry(mul_rs_dispatch_entry),
        .full(mul_rs_full),
        .almost_full(mul_rs_almost_full),
        .dispatch_valid_1(mul_rs_dispatch_valid_1),
        .dispatch_entry_1(mul_rs_dispatch_entry_1),
        .cdb(cdb),
        .issue_valid(mul_rs_issue_valid),
        .issue_entry(mul_rs_issue_entry),
        .execute_ready(mul_execute_ready),
        .prf_req(mul_prf_req),
        .prf_grant(mul_prf_grant),
        .occupancy(mul_rs_occupancy)
    );

    assign mul_execute_ready = !mul_busy;

    // DIV RESERVATION STATION (2-way superscalar)
    reservation_station #(
        .NUM_ENTRIES(NUM_DIV_RS)
    ) div_rs (
        .clk(clk),
        .rst(rst),
        .flush(flush_this_cycle),
        .dispatch_valid(div_rs_dispatch_valid),
        .dispatch_entry(div_rs_dispatch_entry),
        .full(div_rs_full),
        .almost_full(div_rs_almost_full),
        .dispatch_valid_1(div_rs_dispatch_valid_1),
        .dispatch_entry_1(div_rs_dispatch_entry_1),
        .cdb(cdb),
        .issue_valid(div_rs_issue_valid),
        .issue_entry(div_rs_issue_entry),
        .execute_ready(div_execute_ready),
        .prf_req(div_prf_req),
        .prf_grant(div_prf_grant),
        .occupancy(div_rs_occupancy)
    );

    assign div_execute_ready = !div_busy;

    // BRANCH RESERVATION STATION (2-way superscalar)
    reservation_station #(.NUM_ENTRIES(NUM_BRANCH_RS)) branch_rs (
        .clk(clk),
        .rst(rst),
        .cdb(cdb),
        .flush(flush_this_cycle),
        .dispatch_valid(branch_rs_dispatch_valid),
        .dispatch_entry(branch_rs_dispatch_entry),
        .full(branch_rs_full),
        .almost_full(branch_rs_almost_full),
        .dispatch_valid_1(branch_rs_dispatch_valid_1),
        .dispatch_entry_1(branch_rs_dispatch_entry_1),
        .issue_valid(branch_rs_issue_valid),
        .issue_entry(branch_rs_issue_entry),
        .execute_ready(branch_execute_ready),
        .prf_req(branch_prf_req),
        .prf_grant(branch_prf_grant),
        .occupancy(branch_rs_occupancy)
    );

    assign branch_execute_ready = !branch_busy;

    assign branch_rs1_preg = branch_rs_issue_entry.src1_preg;
    assign branch_rs2_preg = branch_rs_issue_entry.src2_preg;

    branch_exec branch_execution_unit (
        .clk(clk),
        .rst(rst),
        // .valid_in(branch_rs_issue_valid),
        .valid_in(branch_rs_issue_valid && branch_prf_grant),
        // .valid_in(branch_prf_grant),
        .rs1_data(branch_rs1_data),
        .rs2_data(branch_rs2_data),
        .branch_op(branch_rs_issue_entry.branch_op),
        .inst_type(branch_rs_issue_entry.inst_type),
        .pc(branch_rs_issue_entry.pc),
        .imm(branch_rs_issue_entry.src2_imm),
        .dest_preg(branch_rs_issue_entry.dest_preg),
        .rob_id(branch_rs_issue_entry.rob_id),
        .cdb_grant(branch_cdb_grant),
        .branch_cdb(branch_cdb),
        .busy(branch_busy)
    );

    // SPLIT LSQ
    split_lsq split_load_store_queue (
        .clk(clk),
        .rst(rst),
        .flush(flush_this_cycle),

        // Load Queue dispatch - port 0
        .lq_dispatch_valid(lq_dispatch_valid),
        .lq_dispatch_entry(lq_dispatch_entry),
        .lq_full(lq_full),
        .lq_almost_full(lq_almost_full),

        // Load Queue dispatch - port 1 (superscalar)
        .lq_dispatch_valid_1(lq_dispatch_valid_1),
        .lq_dispatch_entry_1(lq_dispatch_entry_1),

        // Store Queue dispatch - port 0
        .sq_dispatch_valid(sq_dispatch_valid),
        .sq_dispatch_entry(sq_dispatch_entry),
        .sq_full(sq_full),
        .sq_almost_full(sq_almost_full),

        // Store Queue dispatch - port 1 (superscalar)
        .sq_dispatch_valid_1(sq_dispatch_valid_1),
        .sq_dispatch_entry_1(sq_dispatch_entry_1),

        // CDB intf.
        .cdb(cdb),

        // ROB commit intf - port 0
        .rob_commit_valid(rob_commit),
        .rob_commit_id(rob_head.rob_id),
        .rob_commit_is_store(rob_head.inst_type == ITYPE_STORE),

        // ROB commit intf - port 1 (dual commit)
        .rob_commit_valid_1(rob_commit_1),
        .rob_commit_id_1(rob_head_1.rob_id),
        .rob_commit_is_store_1(rob_head_1.inst_type == ITYPE_STORE),

        // Load Queue PRF read intf.
        .lq_prf_src1_preg(lq_src1_preg),
        .lq_prf_src1_data(lq_src1_data),
        .lq_prf_req(lq_prf_req),
        .lq_prf_grant(lq_prf_grant),

        // Store Queue PRF read intf.
        .sq_prf_src1_preg(sq_src1_preg),
        .sq_prf_src1_data(sq_src1_data),
        .sq_prf_src2_preg(sq_src2_preg),
        .sq_prf_req(sq_prf_req),
        .sq_prf_grant(sq_prf_grant),

        // D-Cache intf.
        .dcache_addr(dcache_ufp_addr),
        .dcache_rmask(dcache_ufp_rmask),
        .dcache_wmask(dcache_ufp_wmask),
        .dcache_wdata(dcache_ufp_wdata),
        .dcache_rdata(dcache_ufp_rdata),
        .dcache_resp(dcache_ufp_resp),
        
        // writeback to CDB (load only)
        .lsq_load_cdb(lsq_load_cdb),
        .lsq_load_cdb_grant(lsq_load_cdb_grant),

        // load update port to ROB
        .load_update_mem_info(lsq_load_update_mem_info),
        .load_update_id(lsq_load_update_id),
        .load_update_addr(lsq_load_update_addr),
        .load_update_rmask(lsq_load_update_rmask),
        .load_update_rdata(lsq_load_update_rdata),

        // store update port to ROB (with ready signal)
        .store_update_mem_info(lsq_store_update_mem_info),
        .store_update_id(lsq_store_update_id),
        .store_update_addr(lsq_store_update_addr),
        .store_update_wmask(lsq_store_update_wmask),
        .store_update_wdata(lsq_store_update_wdata),
        .store_ready(lsq_store_ready),

        .lq_occupancy(lq_occupancy),
        .sq_occupancy(sq_occupancy),

        // SQ tail for memory ordering
        .sq_tail_out_cpu(sq_tail_for_capture)
    );

    assign sq_tail_capture = {1'b0, sq_tail_for_capture};

     
    // ALU_0 EXECUTION PATH
     
    assign alu_0_rs1_preg = alu_0_rs_issue_entry.src1_preg;
    assign alu_0_rs2_preg = alu_0_rs_issue_entry.src2_preg;

    // detect AUIPC for ALU_0
    logic is_auipc_exec_0;
    assign is_auipc_exec_0 = (alu_0_rs_issue_entry.alu_op == ALU_AUIPC);

    assign alu_0_op = alu_0_rs_issue_entry.alu_op;

    logic [31:0] alu_0_a_ungated, alu_0_b_ungated;
    assign alu_0_a_ungated = is_auipc_exec_0 ? alu_0_rs_issue_entry.pc : alu_0_rs1_data;
    assign alu_0_b_ungated = alu_0_rs_issue_entry.src2_is_imm ? alu_0_rs_issue_entry.src2_imm : alu_0_rs2_data;
    assign alu_0_a = alu_0_rs_issue_valid ? alu_0_a_ungated : 32'h0;
    assign alu_0_b = alu_0_rs_issue_valid ? alu_0_b_ungated : 32'h0;

    alu alu_0_unit (
        .op(alu_0_op),
        .a(alu_0_a),
        .b(alu_0_b),
        .result(alu_0_result)
    );

    // ALU_0 result buffer
    always_ff @(posedge clk) begin
        if (rst) begin
            alu_0_result_valid <= 1'b0;
            alu_0_result_buffered <= '0;
            alu_0_dest_preg_buffered <= '0;
            alu_0_rob_id_buffered <= '0;
        end else begin
            case ({alu_0_rs_issue_valid, alu_0_cdb_grant})
                2'b00: begin
                    // Neither: hold state
                end
                2'b01: begin
                    // CDB grant only: clear buffer
                    alu_0_result_valid <= 1'b0;
                end
                2'b10: begin
                    // RS actually issues: capture new result
                    alu_0_result_valid <= 1'b1;
                    alu_0_result_buffered <= alu_0_result;
                    alu_0_dest_preg_buffered <= alu_0_rs_issue_entry.dest_preg;
                    alu_0_rob_id_buffered <= alu_0_rs_issue_entry.rob_id;
                end
                2'b11: begin
                    // BOTH: broadcast old AND capture new
                    alu_0_result_valid <= 1'b1;
                    alu_0_result_buffered <= alu_0_result;
                    alu_0_dest_preg_buffered <= alu_0_rs_issue_entry.dest_preg;
                    alu_0_rob_id_buffered <= alu_0_rs_issue_entry.rob_id;
                end
            endcase
        end
    end

    // ALU_0 -> CDB (using buffered result)
    assign alu_0_cdb_request = alu_0_result_valid;
    assign alu_0_cdb.valid = alu_0_result_valid;
    assign alu_0_cdb.data = alu_0_result_buffered;
    assign alu_0_cdb.preg = alu_0_dest_preg_buffered;
    assign alu_0_cdb.rob_id = alu_0_rob_id_buffered;

    // don't issue new ALU_0 instruction if buffer full
    assign alu_0_execute_ready = !alu_0_result_valid || alu_0_cdb_grant;

     
    // ALU_1 EXECUTION PATH
     
    assign alu_1_rs1_preg = alu_1_rs_issue_entry.src1_preg;
    assign alu_1_rs2_preg = alu_1_rs_issue_entry.src2_preg;

    // detect AUIPC for ALU_1
    logic is_auipc_exec_1;
    assign is_auipc_exec_1 = (alu_1_rs_issue_entry.alu_op == ALU_AUIPC);

    assign alu_1_op = alu_1_rs_issue_entry.alu_op;

    logic [31:0] alu_1_a_ungated, alu_1_b_ungated;
    assign alu_1_a_ungated = is_auipc_exec_1 ? alu_1_rs_issue_entry.pc : alu_1_rs1_data;
    assign alu_1_b_ungated = alu_1_rs_issue_entry.src2_is_imm ? alu_1_rs_issue_entry.src2_imm : alu_1_rs2_data;
    assign alu_1_a = alu_1_rs_issue_valid ? alu_1_a_ungated : 32'h0;
    assign alu_1_b = alu_1_rs_issue_valid ? alu_1_b_ungated : 32'h0;

    alu alu_1_unit (
        .op(alu_1_op),
        .a(alu_1_a),
        .b(alu_1_b),
        .result(alu_1_result)
    );

    // ALU_1 result buffer
    always_ff @(posedge clk) begin
        if (rst) begin
            alu_1_result_valid <= 1'b0;
            alu_1_result_buffered <= '0;
            alu_1_dest_preg_buffered <= '0;
            alu_1_rob_id_buffered <= '0;
        end else begin
            case ({alu_1_rs_issue_valid, alu_1_cdb_grant})
                2'b00: begin
                    // Neither: hold state
                end
                2'b01: begin
                    // CDB grant only: clear buffer
                    alu_1_result_valid <= 1'b0;
                end
                2'b10: begin
                    // RS actually issues: capture new result
                    alu_1_result_valid <= 1'b1;
                    alu_1_result_buffered <= alu_1_result;
                    alu_1_dest_preg_buffered <= alu_1_rs_issue_entry.dest_preg;
                    alu_1_rob_id_buffered <= alu_1_rs_issue_entry.rob_id;
                end
                2'b11: begin
                    // BOTH: broadcast old AND capture new
                    alu_1_result_valid <= 1'b1;
                    alu_1_result_buffered <= alu_1_result;
                    alu_1_dest_preg_buffered <= alu_1_rs_issue_entry.dest_preg;
                    alu_1_rob_id_buffered <= alu_1_rs_issue_entry.rob_id;
                end
            endcase
        end
    end

    // ALU_1 -> CDB (using buffered result)
    assign alu_1_cdb_request = alu_1_result_valid;
    assign alu_1_cdb.valid = alu_1_result_valid;
    assign alu_1_cdb.data = alu_1_result_buffered;
    assign alu_1_cdb.preg = alu_1_dest_preg_buffered;
    assign alu_1_cdb.rob_id = alu_1_rob_id_buffered;

    // don't issue new ALU_1 instruction if buffer full
    assign alu_1_execute_ready = !alu_1_result_valid || alu_1_cdb_grant;


    // MUL PRF ACCESS & EXEC
    assign mul_rs1_preg = mul_rs_issue_entry.src1_preg;
    assign mul_rs2_preg = mul_rs_issue_entry.src2_preg;

    multiplier #(
        .NUM_MUL_STAGES(NUM_MUL_STAGES)
    ) mul_unit (
        .clk(clk),
        .rst(rst),
        .flush(flush),
        .valid_in(mul_prf_grant),
        .src1_data(mul_rs1_data),
        .src2_data(mul_rs2_data),
        .op(mul_rs_issue_entry.mul_op),
        .dest_preg(mul_rs_issue_entry.dest_preg),
        .rob_id(mul_rs_issue_entry.rob_id),
        .cdb_grant(mul_cdb_grant),
        .mul_cdb(mul_cdb),
        .busy(mul_busy)
    );

    // DIV PRF ACCESS & EXEC
    assign div_rs1_preg = div_rs_issue_entry.src1_preg;
    assign div_rs2_preg = div_rs_issue_entry.src2_preg;

    divider div_unit (
        .clk(clk),
        .rst(rst),
        .flush(flush),
        .valid_in(div_prf_grant),
        .dividend(div_rs1_data),
        .divisor(div_rs2_data),
        .op(div_rs_issue_entry.div_op),
        .dest_preg(div_rs_issue_entry.dest_preg),
        .rob_id(div_rs_issue_entry.rob_id),
        .cdb_grant(div_cdb_grant),
        .div_cdb(div_cdb),
        .busy(div_busy)
    );

    // put exec units in array
    // slowest -> fastest (6 units: MUL, DIV, BRANCH, ALU_0, ALU_1, LQ)
    always_comb begin
        exec_unit_cdbs[0] = mul_cdb;      // highest priority (slowest)
        exec_unit_cdbs[1] = div_cdb;
        exec_unit_cdbs[2] = branch_cdb;
        exec_unit_cdbs[3] = alu_0_cdb;
        exec_unit_cdbs[4] = alu_1_cdb;
        exec_unit_cdbs[5] = lsq_load_cdb; // lowest priority
    end

    // CDB request signals (exec units request when they have valid result)
    assign div_cdb_request = div_cdb.valid;
    assign mul_cdb_request = mul_cdb.valid;
    assign branch_cdb_request = branch_busy || branch_cdb.valid;
    assign lsq_load_cdb_request = lsq_load_cdb.valid;
    // lsq_store_cdb_request REMOVED (store uses direct ready signal to ROB)
    logic lsq_store_cdb_request, lsq_store_cdb_grant;
    assign lsq_store_cdb_request = 1'b0;
    assign lsq_store_cdb_grant = 1'b0;
    // alu_0_cdb_request and alu_1_cdb_request assigned in ALU sections

    // pack request/grant arrays for arbiter (6 units: MUL, DIV, BRANCH, ALU_0, ALU_1, LQ)
    always_comb begin
        exec_cdb_requests[0] = mul_cdb_request;
        exec_cdb_requests[1] = div_cdb_request;
        exec_cdb_requests[2] = branch_cdb_request;
        exec_cdb_requests[3] = alu_0_cdb_request;
        exec_cdb_requests[4] = alu_1_cdb_request;
        exec_cdb_requests[5] = lsq_load_cdb_request;
    end

    // grant signals (6 units)
    assign mul_cdb_grant = exec_cdb_grants[0];
    assign div_cdb_grant = exec_cdb_grants[1];
    assign branch_cdb_grant = exec_cdb_grants[2];
    assign alu_0_cdb_grant = exec_cdb_grants[3];
    assign alu_1_cdb_grant = exec_cdb_grants[4];
    assign lsq_load_cdb_grant = exec_cdb_grants[5];

    // CDB ARBITER
    cdb_arbiter #(
        .NUM_CDB_PORTS(NUM_CDB_PORTS),
        .NUM_EXEC_UNITS(NUM_EXEC_UNITS)
    ) cdb_arb (
        .exec_cdb_req(exec_cdb_requests),
        .exec_cdb_data(exec_unit_cdbs),
        .exec_cdb_grant(exec_cdb_grants),
        .cdb_out(cdb)
    );

    // COMMIT STAGE
    commit commit_stage (
        .clk(clk),
        .rst(rst),
        // ROB port 0
        .rob_head_valid(rob_head_valid),
        .rob_head_ready(rob_head_ready),
        .rob_head(rob_head),
        .rob_commit(rob_commit),
        // ROB port 1 (dual commit)
        .rob_head_valid_1(rob_head_valid_1),
        .rob_head_ready_1(rob_head_ready_1),
        .rob_head_1(rob_head_1),
        .rob_commit_1(rob_commit_1),
        // ROB misprediction signal
        .head_branch_mispredict(head_branch_mispredict),
        // RRF port 0
        .rrf_read_areg(rrf_read_areg),
        .rrf_old_preg(rrf_old_preg),
        .rrf_commit_en(rrf_commit_en),
        .rrf_commit_areg(rrf_commit_areg),
        .rrf_commit_preg(rrf_commit_preg),
        // RRF port 1 (dual commit)
        .rrf_read_areg_1(rrf_read_areg_1),
        .rrf_old_preg_1(rrf_old_preg_1),
        .rrf_commit_en_1(rrf_commit_en_1),
        .rrf_commit_areg_1(rrf_commit_areg_1),
        .rrf_commit_preg_1(rrf_commit_preg_1),
        // free list port 0
        .fl_free(fl_free),
        .fl_free_preg(fl_free_preg),
        // free list port 1 (dual commit)
        .fl_free_1(fl_free_1),
        .fl_free_preg_1(fl_free_preg_1),
        // RVFI output (arrays for dual commit)
        .rvfi_valid(rvfi_valid),
        .rvfi_order(rvfi_order),
        .rvfi_inst(rvfi_inst),
        .rvfi_rs1_addr(rvfi_rs1_addr),
        .rvfi_rs2_addr(rvfi_rs2_addr),
        .rvfi_rs1_rdata(rvfi_rs1_rdata),
        .rvfi_rs2_rdata(rvfi_rs2_rdata),
        .rvfi_rd_addr(rvfi_rd_addr),
        .rvfi_rd_wdata(rvfi_rd_wdata),
        .rvfi_pc_rdata(rvfi_pc_rdata),
        .rvfi_pc_wdata(rvfi_pc_wdata),
        .rvfi_mem_addr(rvfi_mem_addr),
        .rvfi_mem_rmask(rvfi_mem_rmask),
        .rvfi_mem_wmask(rvfi_mem_wmask),
        .rvfi_mem_rdata(rvfi_mem_rdata),
        .rvfi_mem_wdata(rvfi_mem_wdata),

        .bp_update_en(bp_update_en),
        .bp_update_pc(bp_update_pc),
        .bp_update_instr(bp_update_instr),
        .bp_actual_taken(bp_actual_taken),
        .bp_actual_target(bp_actual_target),
        .bp_update_rob_idx(bp_update_rob_idx),
        .bp_update_ghr(bp_update_ghr),
        .bp_update_predicted_taken(bp_update_predicted_taken),
        .bp_update_is_conditional_branch(bp_update_is_conditional_branch),

        // RAS update - port 0
        .ras_update_en(ras_update_en),
        .ras_update_is_push(ras_update_is_push),
        .ras_update_is_pop(ras_update_is_pop),
        .ras_update_push_addr(ras_update_push_addr),

        // RAS update - port 1 (dual commit)
        .ras_update_en_1(ras_update_en_1),
        .ras_update_is_push_1(ras_update_is_push_1),
        .ras_update_is_pop_1(ras_update_is_pop_1),
        .ras_update_push_addr_1(ras_update_push_addr_1)
    );

endmodule : cpu