// 2-WAY SUPERSCALAR FETCH STAGE
//
// Fetches up to 2 instructions per cycle with Option A branch handling:
// - Fetch inst_1 even when inst_0 is a branch/JAL/JALR
// - Squash inst_1 if inst_0's branch is predicted TAKEN
// - Use BTFN (Backward Taken, Forward Not-taken) for inst_1 branches
// - If PC+4 crosses cacheline boundary, don't fetch inst 1

module fetch_stage
import types::*;
(
    input  logic        clk,
    input  logic        rst,

    // control from backend
    input  logic        stall_backend,
    input  logic        queue_almost_full,

    // branch resolution from commit (for mispredictions)
    input  logic        branch_mispredict,
    input  logic [31:0] correct_target,

    // branch predictor update from commit
    input  logic        bp_update_en,
    input  logic [31:0] bp_update_pc,
    input  logic        bp_actual_taken,
    input  logic [8:0]  update_ghr,

    input  logic        bp_update_is_conditional_branch,

    // RAS update from commit - port 0
    input  logic        ras_update_en,
    input  logic        ras_update_is_push,
    input  logic        ras_update_is_pop,
    input  logic [31:0] ras_update_push_addr,

    // RAS update from commit - port 1 (dual commit)
    input  logic        ras_update_en_1,
    input  logic        ras_update_is_push_1,
    input  logic        ras_update_is_pop_1,
    input  logic [31:0] ras_update_push_addr_1,

    input  logic        flush,

     
    // CACHE/LINEBUFFER INTERFACE - PORT 0 (primary)
     
    output logic [31:0] icache_addr,
    output logic [3:0]  icache_rmask,
    input  logic [31:0] icache_rdata,
    input  logic        icache_resp,

     
    // CACHE/LINEBUFFER INTERFACE - PORT 1 (superscalar)
     
    output logic [31:0] icache_addr_1,
    output logic [3:0]  icache_rmask_1,
    input  logic [31:0] icache_rdata_1,
    input  logic        icache_resp_1,

     
    // OUTPUT - INSTRUCTION 0
     
    output logic [31:0] fetch_instr,
    output logic [31:0] fetch_pc,
    output logic        fetch_valid,
    output logic        fetch_predicted_taken,
    output logic [31:0] fetch_predicted_target,
    output logic [8:0]  fetch_prediction_ghr,

     
    // OUTPUT - INSTRUCTION 1 (superscalar)
     
    output logic [31:0] fetch_instr_1,
    output logic [31:0] fetch_pc_1,
    output logic        fetch_valid_1,
    output logic        fetch_predicted_taken_1,
    output logic [31:0] fetch_predicted_target_1,
    output logic [8:0]  fetch_prediction_ghr_1
);

    // Stage 1 (S1): Send request to cache
    // Stage 2 (S2): Receive resp, decode, predict next PC

    // S1 regs - request tracking
    logic [31:0] s1_pc_r;           // PC being fetched in S1
    logic        s1_valid_r;        // S1 has valid request in flight
    logic        s1_speculative_r;  // S1 request was speculative

    // S2 registers - resp handling (inst 0)
    logic [31:0] s2_pc_r;           // PC whose resp is in S2
    logic        s2_valid_r;        // S2 has valid inst
    logic        s2_speculative_r;  // S2 inst was from speculative fetch
    logic [31:0] s2_instr_r;        // captured inst (when backend stalls)
    logic        s2_captured_r;     // inst is captured

    // S2 registers - inst 1 (superscalar)
    logic [31:0] s2_instr_1_r;      // captured inst 1
    logic        s2_valid_1_r;      // inst 1 is valid (was fetched with inst 0)

     
    // INSTRUCTION 0 DECODE (for control flow detection)
     
    logic [31:0] current_instr;
    assign current_instr = s2_captured_r ? s2_instr_r : icache_rdata;

    logic [6:0]  opcode;
    logic        is_branch, is_jal, is_jalr;
    logic        is_control_flow;

    assign opcode = current_instr[6:0];
    assign is_branch = (opcode == 7'b1100011);  // B-type
    assign is_jal    = (opcode == 7'b1101111);  // JAL
    assign is_jalr   = (opcode == 7'b1100111);  // JALR
    assign is_control_flow = is_branch || is_jal || is_jalr;

     
    // INSTRUCTION 1 DECODE (for Option A - BTFN branch prediction)
     
    logic [31:0] current_instr_1;
    assign current_instr_1 = s2_captured_r ? s2_instr_1_r : icache_rdata_1;

    // Inst 1 control flow detection
    logic [6:0]  opcode_1;
    logic        is_branch_1, is_jal_1, is_jalr_1;
    logic        is_control_flow_1;

    assign opcode_1 = current_instr_1[6:0];
    assign is_branch_1 = (opcode_1 == 7'b1100011);  // B-type
    assign is_jal_1    = (opcode_1 == 7'b1101111);  // JAL
    assign is_jalr_1   = (opcode_1 == 7'b1100111);  // JALR
    assign is_control_flow_1 = is_branch_1 || is_jal_1 || is_jalr_1;

    // Inst 1 prediction signals (now provided by gshare, replaces old BTFN)
    logic        predict_taken_1;
    logic [31:0] predicted_target_1_internal;

     
    // PC+4 CACHELINE CHECK
     
    // For Option 2: only fetch inst 1 if PC+4 is in same cacheline
    logic [31:0] pc_plus_4;
    logic        same_cacheline;
    logic        predict_taken;

    assign pc_plus_4 = (s2_captured_r ? s2_pc_r : s1_pc_r) + 32'd4;
    assign same_cacheline = (s2_captured_r ? s2_pc_r[31:5] : s1_pc_r[31:5]) == pc_plus_4[31:5];

     
    // OPTION A: CAN FETCH INST 1?
     
    // Fetch inst 1 even when inst 0 is control flow (Option A improvement)
    // Don't fetch inst 1 only if:
    // 1. PC+4 crosses cacheline boundary
    // 2. queue_almost_full - queue can only accept 1 instruction
    logic can_fetch_inst_1;
    assign can_fetch_inst_1 = same_cacheline && !queue_almost_full;

     
    // OPTION A: SQUASH INST 1 IF INST 0 IS TAKEN
     
    // If inst_0's branch/jump is predicted TAKEN, we can't use inst_1
    // (it's on the wrong path)
    logic squash_inst_1;
    assign squash_inst_1 = is_control_flow && predict_taken;

     
    // BRANCH PREDICTOR (2-WAY SUPERSCALAR)
     
    // Now handles both inst_0 and inst_1 predictions with single-port SRAM:
    // - When only inst_0 is branch: PHT for inst_0
    // - When only inst_1 is branch: PHT for inst_1 (key optimization!)
    // - When both: PHT for inst_0, BTFN for inst_1
    // - Also provides RAS for inst_1 JALR returns

    logic [8:0]  prediction_ghr_from_bp;
    logic [31:0] predicted_target;

    // inst_1 predictions from gshare
    logic        bp_predict_taken_1;
    logic [31:0] bp_predicted_target_1;
    logic [8:0]  bp_prediction_ghr_1;
    logic        bp_inst_1_uses_pht;

    gshare_simple bp (
        .clk(clk),
        .rst(rst),

        // Instruction 0 interface
        .fetch_pc(fetch_pc),
        .fetch_valid(fetch_valid),
        .fetch_instr(current_instr),
        .predict_taken(predict_taken),
        .predicted_target(predicted_target),
        .prediction_ghr_out(prediction_ghr_from_bp),

        // Instruction 1 interface (2-way superscalar)
        .fetch_pc_1(fetch_pc_1),
        .fetch_valid_1(fetch_valid_1),
        .fetch_instr_1(current_instr_1),
        .predict_taken_1(bp_predict_taken_1),
        .predicted_target_1(bp_predicted_target_1),
        .prediction_ghr_out_1(bp_prediction_ghr_1),
        .inst_1_uses_pht(bp_inst_1_uses_pht),

        .update_is_conditional_branch(bp_update_is_conditional_branch),

        // commit update
        .update_en(bp_update_en),
        .update_pc(bp_update_pc),
        .actual_taken(bp_actual_taken),
        .update_ghr(update_ghr),

        .flush(flush),

        // RAS update from commit - port 0
        .ras_update_en(ras_update_en),
        .ras_update_is_push(ras_update_is_push),
        .ras_update_is_pop(ras_update_is_pop),
        .ras_update_push_addr(ras_update_push_addr),

        // RAS update from commit - port 1 (dual commit)
        .ras_update_en_1(ras_update_en_1),
        .ras_update_is_push_1(ras_update_is_push_1),
        .ras_update_is_pop_1(ras_update_is_pop_1),
        .ras_update_push_addr_1(ras_update_push_addr_1)
    );

    // Use gshare predictions for inst_1 (replaces old BTFN-only logic)
    assign predict_taken_1 = bp_predict_taken_1;
    assign predicted_target_1_internal = bp_predicted_target_1;

     
    // NEXT PC CALCULATION (accounts for 2-way superscalar)
     
    logic [31:0] next_sequential_pc;
    logic [31:0] predicted_next_pc;
    logic        redirect_needed;
    logic [31:0] redirect_pc;

    // Next PC calculation for Option A:
    // 1. If inst_0 is taken branch/jump → use inst_0's predicted target
    // 2. Else if inst_1 is taken branch/jump → use inst_1's predicted target
    // 3. Else if fetched 2 instructions → PC+8
    // 4. Else → PC+4
    always_comb begin
        if (is_control_flow && predict_taken) begin
            // inst_0 control flow taken - use its predicted target
            next_sequential_pc = predicted_target;
        end
        else if (can_fetch_inst_1 && !squash_inst_1 && is_control_flow_1 && predict_taken_1) begin
            // inst_0 not taken, but inst_1 is taken control flow
            next_sequential_pc = predicted_target_1_internal;
        end
        else if (can_fetch_inst_1 && !squash_inst_1) begin
            // Fetched 2 instructions, neither taken - next is PC+8
            next_sequential_pc = fetch_pc + 32'd8;
        end
        else begin
            // Fetched 1 instruction (or inst_1 squashed) - next is PC+4
            next_sequential_pc = fetch_pc + 32'd4;
        end
    end

    assign predicted_next_pc = next_sequential_pc;

    always_comb begin
        redirect_needed = 1'b0;
        redirect_pc = predicted_next_pc;
    end

    logic s1_send_request;
    logic [31:0] s1_request_pc;
    logic s1_request_speculative;

    logic s2_accept_response;
    logic pipeline_stall;

    assign pipeline_stall = stall_backend && (s2_valid_r || (s1_valid_r && icache_resp));

    assign s2_accept_response = s1_valid_r && icache_resp && !stall_backend;

    always_comb begin
        s1_send_request = 1'b0;
        s1_request_pc = s1_pc_r;
        s1_request_speculative = 1'b0;

        if (branch_mispredict || flush) begin
            s1_send_request = 1'b1;
            s1_request_pc = correct_target;
            s1_request_speculative = 1'b0;
        end
        else if (redirect_needed && !stall_backend) begin
            s1_send_request = 1'b1;
            s1_request_pc = redirect_pc;
            s1_request_speculative = 1'b0;
        end
        else if (!s1_valid_r) begin
            if (s2_valid_r && !stall_backend) begin
                s1_send_request = 1'b1;
                s1_request_pc = predicted_next_pc;
                s1_request_speculative = 1'b0;
            end
            else if (!s2_valid_r) begin
                s1_send_request = 1'b1;
                s1_request_pc = s2_pc_r;
                s1_request_speculative = 1'b0;
            end
        end
        else if (s1_valid_r && icache_resp && !stall_backend && !redirect_needed) begin
            s1_send_request = 1'b1;
            s1_request_pc = predicted_next_pc;
            s1_request_speculative = 1'b0;
        end
    end
    
    always_ff @(posedge clk) begin
        if (rst) begin
            s1_pc_r <= 32'hAAAAA000;
            s1_valid_r <= 1'b0;
            s1_speculative_r <= 1'b0;

            s2_pc_r <= 32'hAAAAA000;
            s2_valid_r <= 1'b0;
            s2_speculative_r <= 1'b0;
            s2_instr_r <= 32'b0;
            s2_captured_r <= 1'b0;

            // Superscalar inst 1
            s2_instr_1_r <= 32'b0;
            s2_valid_1_r <= 1'b0;
        end
        else begin
            if (branch_mispredict || flush) begin
                s1_valid_r <= 1'b1;
                s1_pc_r <= correct_target;
                s1_speculative_r <= 1'b0;

                s2_valid_r <= 1'b0;
                s2_captured_r <= 1'b0;
                s2_valid_1_r <= 1'b0;
            end
            else if (redirect_needed && !stall_backend) begin
                // cancel S1 speculative fetch, send correct one
                s1_valid_r <= 1'b1;
                s1_pc_r <= redirect_pc;
                s1_speculative_r <= 1'b0;

                // S2 moves forward (instruction was valid)
                s2_valid_r <= 1'b0;
                s2_captured_r <= 1'b0;
                s2_valid_1_r <= 1'b0;
            end
            else begin
                // S1 -> S2 transfer
                if (s1_valid_r && icache_resp) begin
                    if (stall_backend) begin
                        // Capture both instructions when stalled
                        s2_instr_r <= icache_rdata;
                        s2_captured_r <= 1'b1;
                        s2_pc_r <= s1_pc_r;
                        s2_valid_r <= 1'b1;
                        s2_speculative_r <= s1_speculative_r;

                        // Capture inst 1 if available
                        s2_instr_1_r <= icache_rdata_1;
                        s2_valid_1_r <= icache_resp_1 && can_fetch_inst_1;

                        s1_valid_r <= 1'b0;
                    end
                    else begin
                        s2_pc_r <= s1_pc_r;
                        s2_valid_r <= 1'b1;
                        s2_speculative_r <= s1_speculative_r;
                        s2_captured_r <= 1'b0;

                        // Track if inst 1 is valid (not captured, will be read directly)
                        s2_valid_1_r <= icache_resp_1 && can_fetch_inst_1;

                        // new S1 request
                        if (s1_send_request) begin
                            s1_valid_r <= 1'b1;
                            s1_pc_r <= s1_request_pc;
                            s1_speculative_r <= s1_request_speculative;
                        end
                        else
                            s1_valid_r <= 1'b0;
                    end
                end
                else if (s2_captured_r && !stall_backend) begin
                    s2_valid_r <= 1'b0;
                    s2_captured_r <= 1'b0;
                    s2_valid_1_r <= 1'b0;

                    if (s1_send_request) begin
                        s1_valid_r <= 1'b1;
                        s1_pc_r <= s1_request_pc;
                        s1_speculative_r <= s1_request_speculative;
                    end
                end
                else if (!s1_valid_r && s1_send_request) begin
                    s1_valid_r <= 1'b1;
                    s1_pc_r <= s1_request_pc;
                    s1_speculative_r <= s1_request_speculative;
                end
                else if (s2_valid_r && !s2_captured_r && !stall_backend) begin
                    s2_valid_r <= 1'b0;
                    s2_valid_1_r <= 1'b0;
                end
            end
        end
    end

     
    // ICACHE PORT 0 (primary)
     
    assign icache_addr = s1_send_request ? s1_request_pc : s1_pc_r;
    assign icache_rmask = (s1_send_request || s1_valid_r) ? 4'b1111 : 4'b0000;

     
    // ICACHE PORT 1 (superscalar - PC+4)
     
    logic [31:0] s1_pc_plus_4;
    assign s1_pc_plus_4 = (s1_send_request ? s1_request_pc : s1_pc_r) + 32'd4;

    assign icache_addr_1 = s1_pc_plus_4;
    // Only request port 1 if same cacheline (linebuffer will handle the data)
    assign icache_rmask_1 = ((s1_send_request || s1_valid_r) &&
                             (s1_pc_plus_4[31:5] == (s1_send_request ? s1_request_pc[31:5] : s1_pc_r[31:5])))
                            ? 4'b1111 : 4'b0000;

     
    // OUTPUT - INSTRUCTION 0
     
    assign fetch_instr = current_instr;
    assign fetch_pc = s2_captured_r ? s2_pc_r : s1_pc_r;
    assign fetch_valid = !flush && !branch_mispredict && !stall_backend &&
                         ((s2_valid_r && s2_captured_r) || (s1_valid_r && icache_resp && !redirect_needed));
    assign fetch_predicted_taken = fetch_valid && is_control_flow && predict_taken;
    assign fetch_predicted_target = predicted_target;
    assign fetch_prediction_ghr = prediction_ghr_from_bp;

     
    // OUTPUT - INSTRUCTION 1 (superscalar)
     
    assign fetch_instr_1 = current_instr_1;
    assign fetch_pc_1 = (s2_captured_r ? s2_pc_r : s1_pc_r) + 32'd4;

    // Inst 1 valid only if:
    // - Inst 0 is valid
    // - PC+4 is in same cacheline
    // - Linebuffer/cache provided inst 1
    // - inst_0 is NOT taken (otherwise inst_1 is on wrong path)
    assign fetch_valid_1 = fetch_valid && can_fetch_inst_1 && !squash_inst_1 &&
                           ((s2_captured_r && s2_valid_1_r) || (!s2_captured_r && icache_resp_1));

    // inst_1 prediction outputs (now from 2-way superscalar gshare)
    assign fetch_predicted_taken_1 = fetch_valid_1 && is_control_flow_1 && predict_taken_1;
    assign fetch_predicted_target_1 = predicted_target_1_internal;
    assign fetch_prediction_ghr_1 = bp_prediction_ghr_1;  // Use gshare's GHR for inst_1

endmodule : fetch_stage