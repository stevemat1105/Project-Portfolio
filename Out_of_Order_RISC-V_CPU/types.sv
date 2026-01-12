package types;

    typedef enum logic [2:0] {
        IDLE        = 3'b000,
        HIT         = 3'b001,
        WRITEBACK   = 3'b010,
        ALLOCATE    = 3'b011
    } cache_state_t;

    typedef struct packed {
        logic [22:0] tag;
        logic [3:0]  index;
        logic [4:0]  offset;
    } cache_addr_t;

    // ROB parameter
    localparam  ROB_SIZE = 24;
    localparam  ROB_ID_WIDTH = $clog2(ROB_SIZE);  // 5 bits for 24-entry ROB

    // Physical Regs parameter
    localparam  NUM_PREGS = 64;           // 32 arch + ROB_SIZE speculative

    // RS parameters - increased for dual dispatch
    localparam  NUM_ALU_RS = 8;           // ALU reservation station entries (was 4)
    localparam  NUM_MUL_RS = 4;           // Multiplier RS entries (keep same - single unit)
    localparam  NUM_DIV_RS = 4;           // Divider RS entries (keep same - single unit)
    localparam  NUM_BRANCH_RS = 6;        // Branch RS entries (was 4)

    // LSQ PARAMETERS (Split LSQ) - increased for more in-flight ops
    localparam  NUM_LSQ_ENTRIES = 12;     // Load/Store Queue entries (was 8)
    localparam  NUM_LQ_ENTRIES = 6;       // Load Queue entries (was 4)
    localparam  NUM_SQ_ENTRIES = 6;       // Store Queue entries (was 4)

    // CDB / PRF Write parameters
    // 6 units: ALU_0, ALU_1, MUL, DIV, BRANCH, LQ
    localparam  NUM_CDB_PORTS = 6;        // number of CDB ports (was 5, now 6 for dual ALU)
    localparam  NUM_EXEC_UNITS = 6;       // for cdb (was 5, now 6 for dual ALU)

    // PRF Read parameters
    localparam  NUM_RENAME_READ_PORTS = 4;    // rename stage read ports (was 2, now 2 insts × 2 operands)
    localparam  NUM_EXEC_READ_PORTS = 7;      // exec stage read ports (was 6, +1 for dual ALU)

    localparam  NUM_MUL_STAGES = 9;

    // ALU OPS
    typedef enum logic [3:0] {
        ALU_ADD  = 4'b0000,
        ALU_SUB  = 4'b0001,
        ALU_SLL  = 4'b0010,
        ALU_SLT  = 4'b0011,
        ALU_SLTU = 4'b0100,
        ALU_XOR  = 4'b0101,
        ALU_SRL  = 4'b0110,
        ALU_SRA  = 4'b0111,
        ALU_OR   = 4'b1000,
        ALU_AND  = 4'b1001,
        ALU_LUI  = 4'b1010,
        ALU_AUIPC = 4'b1011
    } alu_op_t;

    // MULTIPLY OPS
    typedef enum logic [1:0] {
        MUL_MUL    = 2'b00,
        MUL_MULH   = 2'b01,
        MUL_MULHSU = 2'b10,
        MUL_MULHU  = 2'b11
    } mul_op_t;

    // DIVIDE OPS
    typedef enum logic [1:0] {
        DIV_DIV  = 2'b00,
        DIV_DIVU = 2'b01,
        DIV_REM  = 2'b10,
        DIV_REMU = 2'b11
    } div_op_t;

    // MEMORY OPERATION TYPE - {is_store, funct3[2:0]}
    typedef enum logic [3:0] {
        // Loads ([3] = 0)
        MEM_LB   = 4'b0_000,
        MEM_LH   = 4'b0_001,
        MEM_LW   = 4'b0_010,
        MEM_LBU  = 4'b0_100,
        MEM_LHU  = 4'b0_101,
        
        // Stores ([3] = 1)
        MEM_SB   = 4'b1_000,
        MEM_SH   = 4'b1_001,
        MEM_SW   = 4'b1_010
    } mem_op_t;

    // INSTRUCTION TYPE
    typedef enum logic [2:0] {
        ITYPE_ALU    = 3'b000,
        ITYPE_MUL    = 3'b001,
        ITYPE_DIV    = 3'b010,
        ITYPE_LOAD   = 3'b011,      // CP3
        ITYPE_STORE  = 3'b100,      // CP3
        ITYPE_BRANCH = 3'b101,      // CP3
        ITYPE_JAL    = 3'b110,      // CP3
        ITYPE_JALR   = 3'b111       // CP3
    } inst_type_t;

    typedef enum logic [2:0] {
        BR_BEQ  = 3'b000,
        BR_BNE  = 3'b001,
        BR_BLT  = 3'b100,
        BR_BGE  = 3'b101,
        BR_BLTU = 3'b110,
        BR_BGEU = 3'b111
    } branch_op_t;

    // DECODED INSTRUCTION
    typedef struct packed {
        logic [31:0]  pc;
        logic [31:0]  inst;
        logic [6:0]   opcode;
        logic [2:0]   funct3;
        logic [6:0]   funct7;
        logic [4:0]   rs1;
        logic [4:0]   rs2;
        logic [4:0]   rd;
        logic [31:0]  imm;
        logic         uses_rs1;
        logic         uses_rs2;
        logic         writes_rd;
        inst_type_t   inst_type;
        alu_op_t      alu_op;
        mul_op_t      mul_op;
        div_op_t      div_op;
        mem_op_t      mem_op;

        branch_op_t   branch_op;
        logic [31:0]  branch_target;
        logic         predicted_taken;
        logic [9:0]   prediction_ghr;
    } decoded_inst_t;

    // RENAMED INSTRUCTION
    typedef struct packed {
        logic [31:0]  pc;
        logic [31:0]  inst;
        logic [5:0]   rs1_preg;
        logic         rs1_ready;
        logic [5:0]   rs2_preg;
        logic         rs2_ready;
        logic [31:0]  rs2_imm;
        logic         rs2_is_imm;
        logic [5:0]   rd_preg;
        logic [4:0]   rob_id;
        inst_type_t   inst_type;
        alu_op_t      alu_op;
        mul_op_t      mul_op;
        div_op_t      div_op;
        mem_op_t      mem_op;
        logic [4:0]   rd_areg;
        logic [31:0]  rs1_data;
        logic [31:0]  rs2_data;

        branch_op_t   branch_op;
        logic [31:0]  branch_target;
        logic         predicted_taken;
    } renamed_inst_t;

    // RESERVATION STATION ENTRY
    typedef struct packed {
        logic         valid;
        logic [5:0]   dest_preg;
        logic [4:0]   rob_id;
        logic [5:0]   src1_preg;
        logic [4:0]   src1_rob_id;    // Producer ROB ID for src1 (for CDB validation)
        logic         src1_ready;
        logic [5:0]   src2_preg;
        logic [4:0]   src2_rob_id;    // Producer ROB ID for src2 (for CDB validation)
        logic [31:0]  src2_imm;
        logic         src2_is_imm;
        logic         src2_ready;
        alu_op_t      alu_op;
        mul_op_t      mul_op;
        div_op_t      div_op;
        branch_op_t   branch_op;
        inst_type_t   inst_type;
        logic [31:0]  pc;
        logic [31:0]  inst;
    } rs_entry_t;

    // LSQ ENTRY
    typedef struct packed {
        logic         valid;
        logic [4:0]   rob_id;
        logic [5:0]   src1_preg;    // base addr reg
        logic         src1_ready;
        logic [5:0]   src2_preg;    // store data reg
        logic         src2_ready;
        logic [31:0]  offset;       // imm offset
        logic [5:0]   dest_preg;    // dest for loads
        logic         is_load;
        logic         is_store;
        mem_op_t      mem_op;
        logic [2:0]   funct3;
        logic [31:0]  pc;
        logic [31:0]  inst;
        
        // Computed values
        logic         addr_ready;
        logic [31:0]  address;
        logic         executed;     // operation completed (load writeback or store dcache write)
        logic [31:0]  store_data;   // cached store data (stores only)
        logic         cdb_broadcast; // store broadcast on CDB to mark ROB ready
        logic         committed;
    } lsq_entry_t;

    // LOAD QUEUE ENTRY (split LSQ)
    typedef struct packed {
        logic         valid;
        logic [4:0]   rob_id;
        logic [5:0]   src1_preg;    // base addr reg
        logic [4:0]   src1_rob_id;  // Producer ROB ID for src1 (for CDB validation)
        logic         src1_ready;
        logic [31:0]  offset;       // imm offset
        logic [5:0]   dest_preg;    // dest for loads
        mem_op_t      mem_op;
        logic [31:0]  pc;
        logic [31:0]  inst;

        logic         addr_ready;
        logic [31:0]  address;
        logic         executed;     // load done (writeback done)

        // store queue ordering
        logic [$clog2(NUM_SQ_ENTRIES):0] sq_tail_capture;
    } lq_entry_t;

    // STORE QUEUE ENTRY (Split LSQ)
    typedef struct packed {
        logic         valid;
        logic [4:0]   rob_id;
        logic [5:0]   src1_preg;    // base addr reg
        logic [4:0]   src1_rob_id;  // Producer ROB ID for src1 (for CDB validation)
        logic         src1_ready;
        logic [5:0]   src2_preg;    // store data reg
        logic [4:0]   src2_rob_id;  // Producer ROB ID for src2 (for CDB validation)
        logic         src2_ready;
        logic [31:0]  offset;       // imm offset
        mem_op_t      mem_op;
        logic [31:0]  pc;
        logic [31:0]  inst;

        // Computed values
        logic         addr_ready;
        logic [31:0]  address;
        logic         executed;     // store cdone (cache write done)
        logic [31:0]  store_data;   // cached store data
        logic         cdb_broadcast;// broadcast to mark ROB ready
        logic         committed;    // committed from ROB
    } sq_entry_t;

    // REORDER BUFFER ENTRY
    typedef struct packed {
        logic         valid;
        logic         ready;
        logic [5:0]   dest_preg;
        logic [4:0]   dest_areg;
        logic [31:0]  pc;
        logic [31:0]  inst;
        logic [4:0]   rob_id;
        logic [31:0]  rs1_data;
        logic [31:0]  rs2_data;
        logic [31:0]  result;

        logic [5:0]   rs1_preg;
        logic [5:0]   rs2_preg;
        logic         rs1_valid;
        logic         rs2_valid;
        logic [4:0]   rs1_rob_id;   // Producer ROB ID for rs1 (for CDB validation)
        logic [4:0]   rs2_rob_id;   // Producer ROB ID for rs2 (for CDB validation)

        inst_type_t   inst_type;
        branch_op_t   branch_op;
        logic         is_branch;
        logic         predicted_taken;
        logic [31:0]  branch_target;
        logic [9:0]   prediction_ghr;
        
        logic [31:0]  mem_addr;
        logic [3:0]   mem_rmask;
        logic [3:0]   mem_wmask;
        logic [31:0]  mem_wdata;
    } rob_entry_t;

    // COMMON DATA BUS (CDB)
    typedef struct packed {
        logic         valid;
        logic [31:0]  data;
        logic [5:0]   preg;
        logic [4:0]   rob_id;
    } cdb_t;
endpackage