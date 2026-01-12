module branch_exec
import types::*;
(
    input  logic        clk,
    input  logic        rst,
    
    input  logic        valid_in,
    input  logic [31:0] rs1_data,
    input  logic [31:0] rs2_data,
    input  branch_op_t  branch_op,
    input  inst_type_t  inst_type,
    input  logic [31:0] pc,
    input  logic [31:0] imm,
    input  logic [5:0]  dest_preg,
    input  logic [4:0]  rob_id,
    
    input  logic        cdb_grant,
    output cdb_t        branch_cdb,
    output logic        busy
);

    logic        branch_taken;
    logic [31:0] branch_target;

        
    logic        result_valid;
    logic [31:0] result_buffered;
    logic [5:0]  dest_preg_buffered;
    logic [4:0]  rob_id_buffered;

    logic [31:0] cdb_result;
    
    logic [31:0] rs1_data_gated, rs2_data_gated, pc_gated, imm_gated;
    assign rs1_data_gated = valid_in ? rs1_data : 32'h0;
    assign rs2_data_gated = valid_in ? rs2_data : 32'h0;
    assign pc_gated = valid_in ? pc : 32'h0;
    assign imm_gated = valid_in ? imm : 32'h0;

    branch_unit br_unit (
        .rs1_data(rs1_data_gated),
        .rs2_data(rs2_data_gated),
        .branch_op(branch_op),
        .inst_type(inst_type),
        .pc(pc_gated),
        .imm(imm_gated),
        .taken(branch_taken),
        .target(branch_target)
    );
    
    
    always_comb begin
        case (inst_type)
            ITYPE_BRANCH: cdb_result = {branch_target[31:1], branch_taken};
            ITYPE_JAL, ITYPE_JALR: cdb_result = pc + 32'd4;
            default: cdb_result = 32'h0;
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            result_valid <= 1'b0;
            result_buffered <= 32'h0;
            dest_preg_buffered <= 6'h0;
            rob_id_buffered <= 5'h0;
        end
        else begin
            case ({valid_in, cdb_grant})
                2'b00: begin
                end
                
                2'b01: begin
                    result_valid <= 1'b0;
                end
                
                2'b10: begin
                    result_valid <= 1'b1;
                    result_buffered <= cdb_result;
                    dest_preg_buffered <= dest_preg;
                    rob_id_buffered <= rob_id;
                end
                
                2'b11: begin
                    result_valid <= 1'b1;
                    result_buffered <= cdb_result;
                    dest_preg_buffered <= dest_preg;
                    rob_id_buffered <= rob_id;
                end
            endcase
        end
    end
    
    assign busy = result_valid;
    
    always_ff @(posedge clk) begin
        if (rst) begin
            branch_cdb.valid <= 1'b0;
            branch_cdb.preg <= 6'h0;
            branch_cdb.data <= 32'h0;
            branch_cdb.rob_id <= 5'h0;
        end
        else if (result_valid && cdb_grant) begin
            branch_cdb.valid <= 1'b1;
            branch_cdb.preg <= dest_preg_buffered;
            branch_cdb.data <= result_buffered;
            branch_cdb.rob_id <= rob_id_buffered;
        end
        else
            branch_cdb.valid <= 1'b0;
    end
    
endmodule