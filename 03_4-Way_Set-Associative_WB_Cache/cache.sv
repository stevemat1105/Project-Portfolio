module cache (
    input   logic           clk,
    input   logic           rst,

    // cpu side signals, ufp -> upward facing port
    input   logic   [31:0]  ufp_addr,
    input   logic   [3:0]   ufp_rmask,
    input   logic   [3:0]   ufp_wmask,
    output  logic   [31:0]  ufp_rdata,
    input   logic   [31:0]  ufp_wdata,
    output  logic           ufp_resp,

    // memory side signals, dfp -> downward facing port
    output  logic   [31:0]  dfp_addr,
    output  logic           dfp_read,
    output  logic           dfp_write,
    input   logic   [255:0] dfp_rdata,
    output  logic   [255:0] dfp_wdata,
    input   logic           dfp_resp
);


    logic [31:0] latched_addr;
    logic [3:0]  latched_rmask;
    logic [3:0]  latched_wmask;
    logic [31:0] latched_wdata;

    logic [31:0] eff_addr, eff_wdata;
    logic [3:0]  eff_rmask, eff_wmask;
    logic [22:0] eff_tag;
    logic [3:0]  eff_set;

    logic data_csb0 [3:0], tag_csb0 [3:0], valid_csb0 [3:0], dirty_csb0 [3:0];
    logic lru_csb0; // MEM_EN_ON = 0, MEM_EN_OFF = 1
    logic data_web0 [3:0], tag_web0 [3:0], valid_web0 [3:0], dirty_web0 [3:0];
    logic lru_web0; // web0 = 1 Read, web0 = 0 Write
    logic csb0_universal; // 1 = MEM OFF, 0 = MEM ON

    logic [31:0] data_wmask0 [3:0];
    logic [255:0] data_din0 [3:0], data_dout0 [3:0];
    logic [22:0] tag_din0 [3:0], tag_dout0 [3:0];
    logic valid_din0 [3:0], valid_dout0 [3:0];
    logic dirty_din0 [3:0], dirty_dout0 [3:0];
    logic [2:0] lru_din0, lru_dout0;
    logic [1:0] victim_index;

    logic hit_way [3:0];
    logic hit;
    logic [1:0] hit_index;

    logic [31:0] word; 
    logic [31:0] mask; 


    generate for (genvar i = 0; i < 4; i++) begin : arrays
        mp_cache_data_array data_array (
            .clk0       (clk),
            .csb0       (data_csb0[i] || csb0_universal),
            .web0       (data_web0[i]),
            .wmask0     (data_wmask0[i]),
            .addr0      (eff_set),
            .din0       (data_din0[i]),
            .dout0      (data_dout0[i])
        );
        mp_cache_tag_array tag_array (
            .clk0       (clk),
            .csb0       (tag_csb0[i] || csb0_universal),
            .web0       (tag_web0[i]),
            .addr0      (eff_set),
            .din0       (tag_din0[i]),
            .dout0      (tag_dout0[i])
        );
        sp_ff_array valid_array (
            .clk0       (clk),
            .rst0       (rst),
            .csb0       (valid_csb0[i] || csb0_universal),
            .web0       (valid_web0[i]),
            .addr0      (eff_set),
            .din0       (valid_din0[i]),
            .dout0      (valid_dout0[i]) 
        );
        sp_ff_array dirty_array (
            .clk0   (clk),
            .rst0   (rst),
            .csb0   (dirty_csb0[i] || csb0_universal),
            .web0   (dirty_web0[i]),
            .addr0  (eff_set),
            .din0   (dirty_din0[i]),   // 1 = mark dirty, 0 = mark clean
            .dout0  (dirty_dout0[i])
        );
    end endgenerate

    sp_ff_array #(
        .WIDTH      (3)
    ) lru_array (
        .clk0       (clk),
        .rst0       (rst),
        .csb0       (lru_csb0),
        .web0       (lru_web0),
        .addr0      (eff_set),
        .din0       (lru_din0),
        .dout0      (lru_dout0)
    );


    typedef enum logic [1:0] {
        IDLE,
        HIT,
        WRITEBACK,
        ALLOCATE
    } state_t;

    state_t current_state, next_state;

    logic [1:0] victim_index_next; // LATCHED
    always_ff @(posedge clk) begin
        if (rst) begin
            victim_index <= 2'd0;
        end else begin
            if (current_state == HIT && next_state != HIT) begin
            if (!hit) victim_index <= victim_index_next;
            end
        end
    end

    logic [22:0] req_tag_q;
    always_ff @(posedge clk) begin
        if (rst) begin
            req_tag_q <= '0;
        end else if (current_state == IDLE) begin
            req_tag_q <= ufp_addr[31:9];
        end
    end

    logic [3:0] req_rmask_q;
    always_ff @(posedge clk) begin
        if (rst) begin
            req_rmask_q <= '0;
        end else if (current_state == IDLE) begin
            req_rmask_q <= ufp_rmask;
        end
    end

    always_ff @(posedge clk) begin // Main FSM Drivers
        if (rst) begin
            current_state <= IDLE;
            latched_addr  <= '0;
            latched_rmask <= '0;
            latched_wmask <= '0;
            latched_wdata <= '0;
        end else begin
            current_state <= next_state;

            if (current_state == IDLE) begin
                latched_addr  <= ufp_addr;
                latched_rmask <= ufp_rmask;
                latched_wmask <= ufp_wmask;
                latched_wdata <= ufp_wdata;
            end
        end
    end

    always_comb begin
        if (current_state == IDLE) begin
            eff_addr = ufp_addr;
            eff_rmask = ufp_rmask;
            eff_wmask = ufp_wmask;
            eff_wdata = ufp_wdata;
        end else begin
            eff_addr = latched_addr;
            eff_rmask = latched_rmask;
            eff_wmask = latched_wmask;
            eff_wdata = latched_wdata;
        end
        eff_tag = req_tag_q;
        eff_set = eff_addr[8:5];
    end


    always_comb begin
        next_state = current_state;
        victim_index_next = victim_index;
    
        ufp_rdata = 32'b0;
        ufp_resp = 1'b0;

        dfp_addr  = 32'b0;
        dfp_read = 1'b0;
        dfp_write = 1'b0;
        dfp_wdata = 256'b0;

        hit = 1'b0;
        hit_index = 2'b00;

        csb0_universal = 1'b1; 
        lru_csb0 = 1'b1; lru_web0 = 1'b1; lru_din0  = 3'b0;

        for(integer i = 0; i < 4; i++) begin
            data_csb0[i] = 1'b1; data_web0[i] = 1'b1; data_din0[i] = 256'b0; data_wmask0[i] = 32'b0;
            tag_csb0[i] = 1'b1; tag_web0[i] = 1'b1; tag_din0[i] = 23'b0;
            valid_csb0[i] = 1'b1; valid_web0[i] = 1'b1; valid_din0[i] = 1'b0;
            dirty_csb0[i] = 1'b1; dirty_web0[i] = 1'b1; dirty_din0[i] = 1'b0;
            hit_way[i] = 1'b0;
        end

        case (current_state)
            IDLE: begin
                if (eff_rmask != 4'b0000 || eff_wmask != 4'b0000) begin
                    for(integer i = 0; i < 4; i++) begin
                        data_csb0[i] = 1'b0; data_web0[i] = 1'b1;
                        tag_csb0[i] = 1'b0; tag_web0[i] = 1'b1;
                        valid_csb0[i] = 1'b0; valid_web0[i] = 1'b1;
                        dirty_csb0[i] = 1'b0; dirty_web0[i] = 1'b1;
                    end

                    lru_csb0 = 1'b0;
                    lru_web0 = 1'b1;

                    next_state = HIT;
                    csb0_universal = 1'b0;
                end else begin
                    next_state = IDLE;
                    ufp_resp = 1'b0;
                    csb0_universal = 1'b1;
                end
            end
            HIT: begin
                csb0_universal = 1'b0;

                hit_way[0] = valid_dout0[0] && (tag_dout0[0] == eff_tag);
                hit_way[1] = valid_dout0[1] && (tag_dout0[1] == eff_tag);
                hit_way[2] = valid_dout0[2] && (tag_dout0[2] == eff_tag);
                hit_way[3] = valid_dout0[3] && (tag_dout0[3] == eff_tag);

                hit = hit_way[0] | hit_way[1] | hit_way[2] | hit_way[3];   // True if Any Hit

                if (hit) begin // HIT
                    if (hit_way[0])      hit_index = 2'b00;
                    else if (hit_way[1]) hit_index = 2'b01;
                    else if (hit_way[2]) hit_index = 2'b10;
                    else if (hit_way[3]) hit_index = 2'b11;

                    if(eff_rmask != 4'b0000) begin // READ HIT
                        data_csb0[hit_index] = 1'b0;
                        data_web0[hit_index] = 1'b1;

                        word = data_dout0[hit_index][(eff_addr[4:2]*32) +: 32];
                        mask = { {8{req_rmask_q[3]}}, {8{req_rmask_q[2]}}, {8{req_rmask_q[1]}}, {8{req_rmask_q[0]}} };
                        ufp_rdata = word & mask;
                        ufp_resp = 1'b1;

                        lru_csb0 = 1'b0;
                        lru_web0 = 1'b0;
                        lru_din0 = lru_dout0;  // LRU Update
                        unique case (hit_index)
                            2'd0: begin lru_din0[0] = 1'b0; lru_din0[1] = 1'b0; end
                            2'd1: begin lru_din0[0] = 1'b0; lru_din0[1] = 1'b1; end
                            2'd2: begin lru_din0[0] = 1'b1; lru_din0[2] = 1'b0; end
                            2'd3: begin lru_din0[0] = 1'b1; lru_din0[2] = 1'b1; end
                        endcase

                        next_state = IDLE;
                    end

                    if(eff_wmask != 4'b0000) begin // WRITE HIT
                        data_web0[hit_index] = 1'b0;
                        data_csb0[hit_index] = 1'b0;
                        data_wmask0[hit_index] = {{28{1'b0}}, eff_wmask} << (eff_addr[4:2] * 4);

                        data_din0[hit_index] = data_dout0[hit_index]; 
                        case (eff_addr[4:2])
                            3'd0: data_din0[hit_index][31:0]     = eff_wdata;
                            3'd1: data_din0[hit_index][63:32]    = eff_wdata;
                            3'd2: data_din0[hit_index][95:64]    = eff_wdata;
                            3'd3: data_din0[hit_index][127:96]   = eff_wdata;
                            3'd4: data_din0[hit_index][159:128]  = eff_wdata;
                            3'd5: data_din0[hit_index][191:160]  = eff_wdata;
                            3'd6: data_din0[hit_index][223:192]  = eff_wdata;
                            3'd7: data_din0[hit_index][255:224]  = eff_wdata;
                        endcase

                        dirty_csb0[hit_index] = 1'b0;
                        dirty_web0[hit_index] = 1'b0;
                        dirty_din0[hit_index] = 1'b1;

                        lru_csb0 = 1'b0;
                        lru_web0 = 1'b0;
                        lru_din0 = lru_dout0;  // LRU Update
                        unique case (hit_index)
                            2'd0: begin lru_din0[0] = 1'b0; lru_din0[1] = 1'b0; end
                            2'd1: begin lru_din0[0] = 1'b0; lru_din0[1] = 1'b1; end
                            2'd2: begin lru_din0[0] = 1'b1; lru_din0[2] = 1'b0; end
                            2'd3: begin lru_din0[0] = 1'b1; lru_din0[2] = 1'b1; end
                        endcase

                        ufp_resp = 1'b1;
                        next_state = IDLE;
                    end
                end else begin 
                    lru_csb0 = 1'b0;   
                    lru_web0 = 1'b1;  

                    unique casez (lru_dout0)
                        3'b?11: victim_index_next = 2'd0; 
                        3'b?01: victim_index_next = 2'd1; 
                        3'b1?0: victim_index_next = 2'd2; 
                        3'b0?0: victim_index_next = 2'd3;
                        default: victim_index_next = 2'd0;
                    endcase

                    if (!valid_dout0[victim_index_next]) begin
                        next_state = ALLOCATE;
                    end else if (dirty_dout0[victim_index_next]) begin
                        next_state = WRITEBACK;
                    end else begin
                        next_state = ALLOCATE;
                    end 
                end
            end
            WRITEBACK: begin
                csb0_universal = 1'b0;

                data_csb0[victim_index] = 1'b0; data_web0[victim_index] = 1'b1; 
                tag_csb0 [victim_index] = 1'b0; tag_web0 [victim_index] = 1'b1;

                dfp_addr  = {tag_dout0[victim_index], eff_set, 5'b00000};
                dfp_wdata = data_dout0[victim_index];
                dfp_write = 1'b1;

                if (dfp_resp) begin
                    dirty_csb0[victim_index] = 1'b0;
                    dirty_web0[victim_index] = 1'b0;
                    dirty_din0[victim_index] = 1'b0;
                    next_state = ALLOCATE;
                end else begin
                    next_state = WRITEBACK;
                end
            end

            ALLOCATE: begin
                csb0_universal = 1'b0;

                dfp_addr = {eff_tag, eff_set, 5'b00000};
                dfp_read = 1'b1;

                if (dfp_resp) begin
                    data_csb0[victim_index]  = 1'b0;
                    data_web0[victim_index]  = 1'b0;
                    data_din0[victim_index]  = dfp_rdata;
                    data_wmask0[victim_index] = 32'hFFFF_FFFF;

                    tag_csb0[victim_index]   = 1'b0;
                    tag_web0[victim_index]   = 1'b0;
                    tag_din0[victim_index]   = eff_tag;

                    valid_csb0[victim_index] = 1'b0;
                    valid_web0[victim_index] = 1'b0;
                    valid_din0[victim_index] = 1'b1;

                    dirty_csb0[victim_index] = 1'b0;
                    dirty_web0[victim_index] = 1'b0;
                    dirty_din0[victim_index] = 1'b0; 

                    lru_csb0 = 1'b0; 
                    lru_web0 = 1'b0;
                    lru_din0 = lru_dout0;
                    
                    unique case (victim_index) // LRU Update
                        2'd0: begin lru_din0[0] = 1'b0; lru_din0[1] = 1'b0; end
                        2'd1: begin lru_din0[0] = 1'b0; lru_din0[1] = 1'b1; end
                        2'd2: begin lru_din0[0] = 1'b1; lru_din0[2] = 1'b0; end
                        2'd3: begin lru_din0[0] = 1'b1; lru_din0[2] = 1'b1; end
                    endcase

                    next_state = IDLE;
                end else begin
                    next_state = ALLOCATE;
                end
            end
        endcase
    end 

endmodule
