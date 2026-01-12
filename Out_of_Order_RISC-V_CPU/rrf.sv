module rrf 
    import types::*;
#(
    parameter NUM_PREGS = 64
)(
    input  logic        clk,
    input  logic        rst,
    
    // read old mapping (during commit) - port 0
    input  logic [4:0]  read_areg,
    output logic [5:0]  old_preg,

    // read old mapping (during commit) - port 1 (dual commit)
    input  logic [4:0]  read_areg_1,
    output logic [5:0]  old_preg_1,

    output logic [5:0]  rrf_mappings [32],
    
    // update mapping on commit - port 0
    input  logic        commit_en,
    input  logic [4:0]  commit_areg,
    input  logic [5:0]  commit_preg,

    // update mapping on commit - port 1 (dual commit)
    input  logic        commit_en_1,
    input  logic [4:0]  commit_areg_1,
    input  logic [5:0]  commit_preg_1
);

    logic [5:0] rrf_table [32];

    assign rrf_mappings = rrf_table;
    
    // mapping p0-p31
    // dual commit: port 1 is later in program order, so it wins WAW conflicts
    always_ff @(posedge clk) begin
        if (rst) begin
            for (integer unsigned i = 0; i < 32; i++) begin
                rrf_table[i] <= 6'(i);
            end
        end
        else begin
            // port 0 writes first
            if (commit_en && commit_areg != 5'b0)
                rrf_table[commit_areg] <= commit_preg;
            // port 1 writes second - overwrites port 0 if same areg
            if (commit_en_1 && commit_areg_1 != 5'b0)
                rrf_table[commit_areg_1] <= commit_preg_1;
        end
    end

    // commit read port 0 (for freeing old physical regs)
    assign old_preg = rrf_table[read_areg];

    // commit read port 1 - needs to see port 0's update for WAW
    // if port 0 writes to same areg, port 1 should free port 0's preg, not the old one
    always_comb begin
        if (commit_en && commit_areg != 5'b0 && commit_areg == read_areg_1)
            // WAW: port 1 reads the preg that port 0 is about to write
            old_preg_1 = commit_preg;
        else
            old_preg_1 = rrf_table[read_areg_1];
    end

endmodule