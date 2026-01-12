module multiplier
import types::*;
#(
    parameter NUM_MUL_STAGES = 5
)
(
    input  logic        clk,
    input  logic        rst,
    input  logic        flush,

    // From RS
    input  logic        valid_in,
    input  logic [31:0] src1_data,
    input  logic [31:0] src2_data,
    input  mul_op_t     op,
    input  logic [5:0]  dest_preg,
    input  logic [4:0]  rob_id,

    // CDB grant from arbiter
    input  logic        cdb_grant,

    // to CDB
    output cdb_t        mul_cdb,

    // status
    output logic        busy
);

    localparam PIPE_DEPTH = NUM_MUL_STAGES;

    logic [5:0]  pipe_dest_preg [PIPE_DEPTH];
    logic [4:0]  pipe_rob_id    [PIPE_DEPTH];
    mul_op_t     pipe_op_type   [PIPE_DEPTH];
    logic        pipe_src1_neg  [PIPE_DEPTH];
    logic [31:0] pipe_src1      [PIPE_DEPTH];
    logic [31:0] pipe_src2      [PIPE_DEPTH];

    logic        mult_en;
    logic        mult_tc;
    logic [63:0] mult_product;

    logic [31:0] in_a_reg, in_b_reg;
    mul_op_t     in_op_reg;

    logic        result_valid;
    logic [31:0] result_data;
    logic [5:0]  result_preg;
    logic [4:0]  result_rob_id;

    logic [PIPE_DEPTH-1:0] pipe_valid_sr;

    logic pipe_out_valid;
    assign pipe_out_valid = pipe_valid_sr[PIPE_DEPTH-1];

    logic pipe_stall;
    assign pipe_stall = result_valid && !cdb_grant && pipe_out_valid;

    logic accept_new;
    assign accept_new = valid_in && !pipe_stall;

    assign mult_en = accept_new || (|pipe_valid_sr && !pipe_stall);

    assign busy = pipe_stall;

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            in_a_reg  <= '0;
            in_b_reg  <= '0;
            in_op_reg <= MUL_MUL;
        end
        else if (accept_new) begin
            in_a_reg  <= src1_data;
            in_b_reg  <= src2_data;
            in_op_reg <= op;
        end
    end

    assign mult_tc = (in_op_reg == MUL_MULH);

    logic mult_rst_n;
    assign mult_rst_n = ~(rst || flush);

    DW_mult_pipe #(
        .a_width(32),
        .b_width(32),
        .num_stages(NUM_MUL_STAGES),
        .stall_mode(1),
        .rst_mode(1),
        .op_iso_mode(4)
    ) u_mult_pipe (
        .clk(clk),
        .rst_n(mult_rst_n),
        .en(mult_en),
        .tc(mult_tc),
        .a(in_a_reg),
        .b(in_b_reg),
        .product(mult_product)
    );

    always_ff @(posedge clk) begin
        if (rst || flush)
            pipe_valid_sr <= '0;
        else if (mult_en)
            pipe_valid_sr <= {pipe_valid_sr[PIPE_DEPTH-2:0], accept_new};
    end

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            for (integer i = 0; i < PIPE_DEPTH; i++) begin
                pipe_dest_preg[i] <= '0;
                pipe_rob_id[i]    <= '0;
                pipe_op_type[i]   <= MUL_MUL;
                pipe_src1_neg[i]  <= '0;
                pipe_src1[i]      <= '0;
                pipe_src2[i]      <= '0;
            end
        end
        else if (mult_en) begin
            pipe_dest_preg[0] <= dest_preg;
            pipe_rob_id[0]    <= rob_id;
            pipe_op_type[0]   <= op;
            pipe_src1_neg[0]  <= src1_data[31];
            pipe_src1[0]      <= src1_data;
            pipe_src2[0]      <= src2_data;

            for (integer i = 1; i < PIPE_DEPTH; i++) begin
                pipe_dest_preg[i] <= pipe_dest_preg[i-1];
                pipe_rob_id[i]    <= pipe_rob_id[i-1];
                pipe_op_type[i]   <= pipe_op_type[i-1];
                pipe_src1_neg[i]  <= pipe_src1_neg[i-1];
                pipe_src1[i]      <= pipe_src1[i-1];
                pipe_src2[i]      <= pipe_src2[i-1];
            end
        end
    end

    logic [63:0] corrected_product;
    always_comb begin
        corrected_product = mult_product;
        if (pipe_op_type[PIPE_DEPTH-1] == MUL_MULHSU && pipe_src1_neg[PIPE_DEPTH-1]) begin
            corrected_product = mult_product - {pipe_src2[PIPE_DEPTH-1], 32'h0};
        end
    end

    logic [31:0] pipe_out_data;
    always_comb begin
        case (pipe_op_type[PIPE_DEPTH-1])
            MUL_MUL:    pipe_out_data = corrected_product[31:0];
            MUL_MULH:   pipe_out_data = corrected_product[63:32];
            MUL_MULHSU: pipe_out_data = corrected_product[63:32];
            MUL_MULHU:  pipe_out_data = corrected_product[63:32];
            default:    pipe_out_data = corrected_product[31:0];
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            result_valid  <= 1'b0;
            result_data   <= '0;
            result_preg   <= '0;
            result_rob_id <= '0;
        end
        else begin
            if (cdb_grant && result_valid) begin
                if (pipe_out_valid && mult_en) begin
                    result_valid  <= 1'b1;
                    result_data   <= pipe_out_data;
                    result_preg   <= pipe_dest_preg[PIPE_DEPTH-1];
                    result_rob_id <= pipe_rob_id[PIPE_DEPTH-1];
                end
                else
                    result_valid <= 1'b0;
            end
            else if (!result_valid && pipe_out_valid && mult_en) begin
                result_valid  <= 1'b1;
                result_data   <= pipe_out_data;
                result_preg   <= pipe_dest_preg[PIPE_DEPTH-1];
                result_rob_id <= pipe_rob_id[PIPE_DEPTH-1];
            end
        end
    end

    assign mul_cdb.valid  = result_valid;
    assign mul_cdb.data   = result_data;
    assign mul_cdb.preg   = result_preg;
    assign mul_cdb.rob_id = result_rob_id;

endmodule