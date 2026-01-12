module divider
import types::*;
(
    input  logic        clk,
    input  logic        rst,
    input  logic        flush,
    
    // From RS
    input  logic        valid_in,
    input  logic [31:0] dividend,
    input  logic [31:0] divisor,
    input  div_op_t     op,
    input  logic [5:0]  dest_preg,
    input  logic [4:0]  rob_id,
    
    // CDB grant from arbiter
    input  logic        cdb_grant,
    
    // to CDB
    output cdb_t        div_cdb,
    
    
    // status
    output logic        busy
);

    localparam integer A_WIDTH = 32;
    localparam integer B_WIDTH = 32;
    localparam integer NUM_CYC = 12;
    
    typedef enum logic [1:0] {
        IDLE       = 2'd0,
        START      = 2'd1,
        COMPUTING  = 2'd2,
        WAIT       = 2'd3       // wait for CDB grant
    } state_t;
    
    state_t current_state, next_state;
    
    logic        div_start;
    logic        div_hold;
    logic        div_complete;
    logic        div_by_0;
    logic [31:0] div_quotient;
    logic [31:0] div_remainder;
    
    logic        operation_valid;
    div_op_t     operation_type;
    logic [5:0]  operation_dest_preg;
    logic [4:0]  operation_rob_id;
    logic [31:0] dividend_reg, divisor_reg;

    logic dividend_is_neg, divisor_is_neg;
    logic [31:0] abs_dividend, abs_divisor;
    logic [31:0] div_a_input, div_b_input;
    
    // filter first complete after reset
    logic        first_complete_seen;
    logic        valid_complete;
    
    DW_div_seq #(
        .a_width(A_WIDTH),
        .b_width(B_WIDTH),
        .tc_mode(0),
        .num_cyc(NUM_CYC),
        .rst_mode(0),
        .input_mode(1),
        .output_mode(1),
        .early_start(0)
    ) u_div_seq (
        .clk(clk),
        .rst_n(~rst),
        .hold(div_hold),
        .start(div_start),
        .a(div_a_input),
        .b(div_b_input),
        .complete(div_complete),
        .divide_by_0(div_by_0),
        .quotient(div_quotient),
        .remainder(div_remainder)
    );
    
    // ignore first garbage complete
    always_ff @(posedge clk) begin
        if (rst)
            first_complete_seen <= 1'b0;
        else if (div_complete && !first_complete_seen)
            first_complete_seen <= 1'b1;
    end
    
    assign valid_complete = div_complete && first_complete_seen;
    
    assign dividend_is_neg = dividend_reg[31];
    assign divisor_is_neg = divisor_reg[31];

    assign abs_dividend = (current_state != IDLE && dividend_is_neg) ? (~dividend_reg + 32'd1) : dividend_reg;
    assign abs_divisor = (current_state != IDLE && divisor_is_neg) ? (~divisor_reg + 32'd1) : divisor_reg;
    
    always_comb begin
        if (current_state == IDLE) begin
            div_a_input = 32'h0;
            div_b_input = 32'h0;
        end
        else begin
            case (operation_type)
                DIV_DIV, DIV_REM: begin
                    div_a_input = abs_dividend;
                    div_b_input = abs_divisor;
                end
                DIV_DIVU, DIV_REMU: begin
                    div_a_input = dividend_reg;
                    div_b_input = divisor_reg;
                end
                default: begin
                    div_a_input = dividend_reg;
                    div_b_input = divisor_reg;
                end
            endcase
        end
    end
    
    always_ff @(posedge clk) begin
        if (rst)
            current_state <= IDLE;
        else if (flush)
            current_state <= IDLE;
        else
            current_state <= next_state;
    end
    
    always_comb begin
        next_state = current_state;
        
        case (current_state)
            IDLE: begin
                if (valid_in)
                    next_state = START;
            end
            
            START: begin
                next_state = COMPUTING;
            end
            
            COMPUTING: begin
                if (valid_complete) begin
                    // result ready - request CDB
                    // if granted immediately, go to IDLE
                    // else wait for grant
                    if (cdb_grant)
                        next_state = IDLE;
                    else
                        next_state = WAIT;
                end
            end
            
            WAIT: begin
                // hold result until granted CDB access
                if (cdb_grant)
                    next_state = IDLE;
            end
            
            default: next_state = IDLE;
        endcase
    end
    
    assign div_start = (current_state == START);
    assign div_hold = 1'b0;
    
    always_ff @(posedge clk) begin
        if (rst) begin
            operation_valid <= 1'b0;
            operation_type <= DIV_DIV;
            operation_dest_preg <= '0;
            operation_rob_id <= '0;
            dividend_reg <= '0;
            divisor_reg <= '0;
        end 
        else if ((current_state == IDLE) && valid_in) begin
            operation_valid <= 1'b1;
            operation_type <= op;
            operation_dest_preg <= dest_preg;
            operation_rob_id <= rob_id;
            dividend_reg <= dividend;
            divisor_reg <= divisor;
        end
    end
    
    logic [31:0] final_result;
    logic is_div_by_zero;
    logic is_overflow;
    logic quotient_is_neg, remainder_is_neg;
    
    assign is_div_by_zero = (divisor_reg == 32'h0);
    assign is_overflow = (dividend_reg == 32'h80000000 && divisor_reg == 32'hFFFFFFFF);
    
    assign quotient_is_neg = dividend_is_neg ^ divisor_is_neg;
    assign remainder_is_neg = dividend_is_neg;
    
    always_comb begin
        case (operation_type)
            DIV_DIV: begin
                // Signed division
                if (is_div_by_zero)
                    final_result = 32'hFFFFFFFF;
                else if (is_overflow)
                    final_result = 32'h80000000;
                else begin
                    if (quotient_is_neg)
                        final_result = ~div_quotient + 32'd1;
                    else
                        final_result = div_quotient;
                end
            end
            
            DIV_DIVU: begin
                // Unsigned division
                if (is_div_by_zero)
                    final_result = 32'hFFFFFFFF;
                else
                    final_result = div_quotient;
            end
            
            DIV_REM: begin
                if (is_div_by_zero)
                    final_result = dividend_reg;
                else if (is_overflow)
                    final_result = 32'h0;
                else begin
                    if (remainder_is_neg)
                        final_result = ~div_remainder + 32'd1;
                    else
                        final_result = div_remainder;
                end
            end
            
            DIV_REMU: begin
                if (is_div_by_zero)
                    final_result = dividend_reg;
                else
                    final_result = div_remainder;
            end
            
            default: final_result = 32'h0;
        endcase
    end
    
    // output - hold valid until granted CDB access
    always_ff @(posedge clk) begin
        if (rst) begin
            div_cdb.valid <= 1'b0;
            div_cdb.data <= '0;
            div_cdb.preg <= '0;
            div_cdb.rob_id <= '0;
        end 
        else if (flush) begin
            div_cdb.valid <= 1'b0;
        end
        else if (current_state == COMPUTING && valid_complete) begin
            // result ready - set valid and hold
            div_cdb.valid <= 1'b1;
            div_cdb.data <= final_result;
            div_cdb.preg <= operation_dest_preg;
            div_cdb.rob_id <= operation_rob_id;
        end
        else if (current_state == WAIT) begin
            // hold result while waiting for grant
            // keep valid high, data stays same
            div_cdb.valid <= 1'b1;
        end
        else
            div_cdb.valid <= 1'b0;
    end
    
    // busy if in pipeline OR waiting for CDB grant
    assign busy = (current_state != IDLE);

endmodule