// RAT - Register Alias Table (Speculative Mappings)
//
// The RAT maintains the curr speculative mapping from architectural registers (x0-x31) to physical registers (p0-p63).
//
// 2-WAY SUPERSCALAR SUPPORT:
// - Dual read ports for 2 instructions (4 source operand lookups)
// - Dual allocation ports for 2 instructions
// - INTRA-CYCLE FORWARDING: inst_1's reads must see inst_0's allocation if they conflict


module rat
import types::*;
#(
    parameter NUM_PREGS = 64
)(
    input  logic        clk,
    input  logic        rst,

    input  logic        flush,
    input  logic [5:0]  rrf_mapping [32],

    output logic [5:0]  rat_mappings [32],

     
    // INSTRUCTION 0 - src operand lookup (port 0)
     
    input  logic [4:0]  rs1_areg,
    output logic [5:0]  rs1_preg,
    output logic        rs1_ready,
    output logic [4:0]  rs1_rob_id,      // Producer ROB ID for CDB validation in rename

    input  logic [4:0]  rs2_areg,
    output logic [5:0]  rs2_preg,
    output logic        rs2_ready,
    output logic [4:0]  rs2_rob_id,      // Producer ROB ID for CDB validation in rename

     
    // INSTRUCTION 1 - src operand lookup (port 1) - WITH INTRA-CYCLE FORWARDING
     
    input  logic [4:0]  rs1_areg_1,
    output logic [5:0]  rs1_preg_1,
    output logic        rs1_ready_1,
    output logic [4:0]  rs1_rob_id_1,    // Producer ROB ID for CDB validation in rename

    input  logic [4:0]  rs2_areg_1,
    output logic [5:0]  rs2_preg_1,
    output logic        rs2_ready_1,
    output logic [4:0]  rs2_rob_id_1,    // Producer ROB ID for CDB validation in rename

     
    // INSTRUCTION 0 - dest allocation (port 0)
     
    input  logic        alloc_en,
    input  logic [4:0]  rd_areg,
    input  logic [5:0]  rd_preg,
    input  logic [4:0]  rd_rob_id,       // ROB ID of the producer (for CDB validation)

     
    // INSTRUCTION 1 - dest allocation (port 1)
     
    input  logic        alloc_en_1,
    input  logic [4:0]  rd_areg_1,
    input  logic [5:0]  rd_preg_1,
    input  logic [4:0]  rd_rob_id_1,     // ROB ID of the producer (for CDB validation)

    input  cdb_t        cdb [NUM_CDB_PORTS]
);

    logic [5:0] rat_mapping [32];   // areg -> preg
    logic       rat_ready   [32];   // the preg ready?
    logic [4:0] rat_rob_id  [32];   // rob_id of producer (for CDB validation)

     
    // INSTRUCTION 0 LOOKUPS (no forwarding needed - first in program order)
     

    // Inst 0 - src 1 lookup
    always_comb begin
        if (rs1_areg == 5'b0) begin
            // x0 always maps to p0 and is always ready
            rs1_preg = 6'd0;
            rs1_ready = 1'b1;
            rs1_rob_id = 5'd0;  // No producer for x0
        end
        else begin
            rs1_preg = rat_mapping[rs1_areg];
            rs1_ready = rat_ready[rs1_areg];
            rs1_rob_id = rat_rob_id[rs1_areg];  // Producer ROB ID for CDB validation
        end
    end

    // Inst 0 - src 2 lookup
    always_comb begin
        if (rs2_areg == 5'b0) begin
            // x0 always maps to p0 and is always ready
            rs2_preg = 6'd0;
            rs2_ready = 1'b1;
            rs2_rob_id = 5'd0;  // No producer for x0
        end
        else begin
            rs2_preg = rat_mapping[rs2_areg];
            rs2_ready = rat_ready[rs2_areg];
            rs2_rob_id = rat_rob_id[rs2_areg];  // Producer ROB ID for CDB validation
        end
    end

     
    // INSTRUCTION 1 LOOKUPS - WITH INTRA-CYCLE FORWARDING FROM INST 0
     
    // If inst 0 is allocating a register that inst 1 reads, forward the new mapping
    // Example: inst0: ADD x5, x1, x2  (allocates p40 for x5)
    //          inst1: SUB x6, x5, x3  (reads x5 - must see p40, not old mapping!)

    // Inst 1 - src 1 lookup with forwarding
    always_comb begin
        if (rs1_areg_1 == 5'b0) begin
            // x0 always maps to p0 and is always ready
            rs1_preg_1 = 6'd0;
            rs1_ready_1 = 1'b1;
            rs1_rob_id_1 = 5'd0;  // No producer for x0
        end
        else if (alloc_en && rd_areg != 5'b0 && rd_areg == rs1_areg_1) begin
            // INTRA-CYCLE FORWARD: inst 0 is writing to the same register inst 1 reads
            rs1_preg_1 = rd_preg;   // Use inst 0's newly allocated preg
            rs1_ready_1 = 1'b0;     // Not ready - inst 0 hasn't executed yet
            rs1_rob_id_1 = rd_rob_id;  // Use inst 0's rob_id as the producer
        end
        else begin
            rs1_preg_1 = rat_mapping[rs1_areg_1];
            rs1_ready_1 = rat_ready[rs1_areg_1];
            rs1_rob_id_1 = rat_rob_id[rs1_areg_1];  // Producer ROB ID for CDB validation
        end
    end

    // Inst 1 - src 2 lookup with forwarding
    always_comb begin
        if (rs2_areg_1 == 5'b0) begin
            // x0 always maps to p0 and is always ready
            rs2_preg_1 = 6'd0;
            rs2_ready_1 = 1'b1;
            rs2_rob_id_1 = 5'd0;  // No producer for x0
        end
        else if (alloc_en && rd_areg != 5'b0 && rd_areg == rs2_areg_1) begin
            // INTRA-CYCLE FORWARD: inst 0 is writing to the same register inst 1 reads
            rs2_preg_1 = rd_preg;   // Use inst 0's newly allocated preg
            rs2_ready_1 = 1'b0;     // Not ready - inst 0 hasn't executed yet
            rs2_rob_id_1 = rd_rob_id;  // Use inst 0's rob_id as the producer
        end
        else begin
            rs2_preg_1 = rat_mapping[rs2_areg_1];
            rs2_ready_1 = rat_ready[rs2_areg_1];
            rs2_rob_id_1 = rat_rob_id[rs2_areg_1];  // Producer ROB ID for CDB validation
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            // initialize RAT: x0->p0, x1->p1, ..., x31->p31
            for (integer unsigned i = 0; i < 32; i++) begin
                rat_mapping[i] <= 6'(i);
                rat_ready[i] <= 1'b1;
                rat_rob_id[i] <= 5'd0;  // No pending producer at reset
            end
        end
        else if (flush) begin
            // restore RAT from RRF (committed state)
            for (integer i = 0; i < 32; i++) begin
                rat_mapping[i] <= rrf_mapping[i];
                rat_ready[i] <= 1'b1;
                rat_rob_id[i] <= 5'd0;  // All committed, no pending producers
            end
        end
        else begin
             
            // DUAL ALLOCATION LOGIC
             
            // Both inst 0 and inst 1 can allocate in same cycle
            // If both write to same areg (WAW), inst 1 wins (later in program order)

            // Port 0 allocation (inst 0)
            if (alloc_en && rd_areg != 5'b0) begin
                rat_mapping[rd_areg] <= rd_preg;
                rat_ready[rd_areg] <= 1'b0;
                rat_rob_id[rd_areg] <= rd_rob_id;
            end

            // Port 1 allocation (inst 1) - overwrites port 0 if same areg (WAW)
            if (alloc_en_1 && rd_areg_1 != 5'b0) begin
                rat_mapping[rd_areg_1] <= rd_preg_1;
                rat_ready[rd_areg_1] <= 1'b0;
                rat_rob_id[rd_areg_1] <= rd_rob_id_1;
            end

            // CDB READY UPDATES
            for (integer i = 0; i < NUM_CDB_PORTS; i++) begin
                if (cdb[i].valid) begin
                    for (integer unsigned j = 1; j < 32; j++) begin
                        automatic logic [4:0] j_unsigned;
                        j_unsigned = 5'(j);
                        if (!rat_ready[j] && rat_mapping[j] == cdb[i].preg &&
                            rat_rob_id[j] == cdb[i].rob_id &&  // Validate CDB is from correct producer
                            !(alloc_en && rd_areg == j_unsigned) &&
                            !(alloc_en_1 && rd_areg_1 == j_unsigned))
                            rat_ready[j] <= 1'b1;
                    end
                end
            end
        end
    end

    assign rat_mappings = rat_mapping;

endmodule : rat