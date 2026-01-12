module cache 
import types::*;
(
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
    input   logic           dfp_resp,

    output  logic   [255:0] ufp_rdata_line
);

    // addr partition
    // cache_addr_t addr_breakdown;
    // assign addr_breakdown.tag    = ufp_addr[31:9];
    // assign addr_breakdown.index  = ufp_addr[8:5];
    // assign addr_breakdown.offset = ufp_addr[4:0];

    logic [31:0]  saved_addr;
    logic [3:0]   saved_rmask;
    logic [3:0]   saved_wmask;
    logic [31:0]  saved_wdata;

    logic [31:0]  masked_rdata;

    cache_state_t state, nxt_state;

    // 4 ways array outputs
    logic [255:0] data_out [4];
    logic [22:0]  tag_out [4];
    logic         valid_out [4];
    logic         dirty_out [4];
    logic [2:0]   plru_out;

    // hit
    // logic [3:0]   tag_match;
    logic [3:0]   way_hit;
    logic         cache_hit;
    logic [1:0]   hit_way;

    // plru
    logic [1:0]   lru_way;
    logic [2:0]   plru_nxt;

    // array control
    logic         arrays_en;
    logic [3:0]   data_web;
    logic [3:0]   tag_web;
    logic [3:0]   valid_web;
    logic [3:0]   dirty_web;
    logic         plru_web;

    // data write
    logic [255:0] data_din [4];
    logic [22:0]  tag_din;
    logic         dirty_din;
    logic [31:0]  data_wmask [4];

    logic [3:0]   array_addr;
    
    // // Track allocated way for hit after miss
    // logic [1:0] allocated_way;
    // logic allocated_valid;

    logic [1:0] saved_lru_way;

    always_ff @(posedge clk) begin
        if (rst)
            saved_lru_way <= '0;
        else if (state == HIT && !cache_hit)
            saved_lru_way <= lru_way;
    end

    always_ff @(posedge clk) begin

        if(rst) begin
            saved_addr <= '0;
            saved_rmask <= '0;
            saved_wdata <= '0;
            saved_wmask <= '0;
        end
        else if(state == IDLE && (ufp_rmask != '0 || ufp_wmask != '0)) begin
            saved_addr <= ufp_addr;
            saved_rmask <= ufp_rmask;
            saved_wdata <= ufp_wdata;
            saved_wmask <= ufp_wmask;
        end
    end

    
    // addr partition
    cache_addr_t  saved_addr_breakdown;
    
    always_comb begin
        saved_addr_breakdown.tag    = saved_addr[31:9];
        saved_addr_breakdown.index  = saved_addr[8:5];
        saved_addr_breakdown.offset = saved_addr[4:0];
    end

    always_comb begin
        // in IDLE, use curr addr for reading
        // in other states, use saved addr
        if (state == IDLE)
            array_addr = ufp_addr[8:5];
        else
            array_addr = saved_addr[8:5];
    end

    generate for (genvar i = 0; i < 4; i++) begin : arrays
        mp_cache_data_array data_array (
            .clk0       (clk),
            .csb0       (~arrays_en),
            .web0       (data_web[i]),
            .wmask0     (data_wmask[i]),
            .addr0      (array_addr),
            .din0       (data_din[i]),
            .dout0      (data_out[i])
        );
        mp_cache_tag_array tag_array (
            .clk0       (clk),
            .csb0       (~arrays_en),
            .web0       (tag_web[i]),
            .addr0      (array_addr),
            .din0       (tag_din),
            .dout0      (tag_out[i])
        );
        sp_ff_array valid_array (
            .clk0       (clk),
            .rst0       (rst),
            .csb0       (~arrays_en),
            .web0       (valid_web[i]),
            .addr0      (array_addr),
            .din0       ('1),
            .dout0      (valid_out[i])
        );
        sp_ff_array dirty_array (
            .clk0       (clk),
            .rst0       (rst),
            .csb0       (~arrays_en),
            .web0       (dirty_web[i]),
            .addr0      (array_addr),
            .din0       (dirty_din),
            .dout0      (dirty_out[i])
        );
    end endgenerate

    sp_ff_array #(
        .WIDTH      (3)
    ) lru_array (
        .clk0       (clk),
        .rst0       (rst),
        .csb0       (~arrays_en),
        .web0       (plru_web),
        .addr0      (array_addr),
        .din0       (plru_nxt),
        .dout0      (plru_out)
    );

    always_comb begin
        for(integer i = 0 ; i < 4 ; i++) begin
            way_hit[i] = (tag_out[i] == saved_addr_breakdown.tag) & valid_out[i];
        end
        
        cache_hit = |way_hit;

        hit_way = way_hit[3] ? 2'd3 : (way_hit[2] ? 2'd2 : (way_hit[1] ? 2'd1 : 2'd0));
    end

    // plru decode
    always_comb begin
        unique casez (plru_out)

            3'b00? : lru_way = 2'b00;
            3'b01? : lru_way = 2'b01;
            3'b1?0 : lru_way = 2'b10;
            3'b1?1 : lru_way = 2'b11;

            default: lru_way = 2'b00;
        endcase
    end

    function automatic logic [2:0] update_plru(input logic [2:0] curr, input logic [1:0] way_access);

        logic [2:0] updated;

        updated = curr;

        case (way_access)
        
            2'b00 : begin
                updated[2] = '1;
                updated[1] = '1;
            end

            2'b01 : begin
                updated[2] = '1;
                updated[1] = '0;
            end

            2'b10 : begin
                updated[2] = '0;
                updated[0] = '1;
            end

            2'b11 : begin
                updated[2] = '0;
                updated[0] = '0;
            end
        endcase

        return updated;
    endfunction

    always_comb begin
        plru_nxt = plru_out;

        if(state == HIT && cache_hit)
                plru_nxt = update_plru(plru_out, hit_way);
    end

    //FSM
    always_ff @(posedge clk) begin
        if (rst)
            state <= IDLE;
        else
            state <= nxt_state;
    end

    always_comb begin
        nxt_state = state;


        case(state)

            IDLE: begin
                if(ufp_rmask != '0 || ufp_wmask != '0)
                    nxt_state = HIT;
            end

            HIT: begin

                if(cache_hit)
                    nxt_state = IDLE;
                
                else begin
                    // miss, check writeback or no
                    if(valid_out[lru_way] && dirty_out[lru_way])
                        nxt_state = WRITEBACK;
                    else
                        nxt_state = ALLOCATE;
                end
            end

            WRITEBACK: begin
                if(dfp_resp)
                    nxt_state = ALLOCATE;
            end

            ALLOCATE: begin
                if(dfp_resp)
                    nxt_state = IDLE;
            end

            // REREAD: begin
            //     nxt_state = HIT;
            // end

            default: begin
                nxt_state = IDLE;
            end
        endcase
    end

    // array control
    always_comb begin

        // logic [2:0] word_offset;

        arrays_en = '0;
        data_web = 4'b1111;
        tag_web = 4'b1111;
        valid_web = 4'b1111;
        dirty_web = 4'b1111;
        plru_web = '1;

        tag_din = saved_addr_breakdown.tag;
        dirty_din = '0;

        for(integer i = 0 ; i < 4 ; i++) begin
            data_din[i] = '0;
            data_wmask[i] = '0;
        end

        case(state)

            IDLE: begin
                if(ufp_rmask != '0 || ufp_wmask != '0)
                    arrays_en = '1;
            end

            HIT: begin
                arrays_en = '1;

                if(cache_hit) begin
                    plru_web = '0;

                    if(saved_wmask != '0) begin
                        logic [2:0] word_offset;
                        word_offset = saved_addr_breakdown.offset[4:2];
                        
                        data_web[hit_way] = '0;
                        dirty_web[hit_way] = '0;
                        dirty_din = '1;

                        data_din[hit_way] = data_out[hit_way];
                        
                        for (integer unsigned byte_idx = 0; byte_idx < 4; byte_idx++) begin
                            if (saved_wmask[byte_idx]) begin
                                data_wmask[hit_way][word_offset*4 + byte_idx] = 1'b1;
                                data_din[hit_way][(word_offset*32) + (byte_idx*8) +: 8] = saved_wdata[byte_idx*8 +: 8];
                            end
                        end
                    end
                end
            end

            WRITEBACK: begin
                arrays_en = '1;
            end

            ALLOCATE: begin
                arrays_en = '1;
                if(dfp_resp) begin
                    logic [2:0] word_offset;
                    word_offset = saved_addr_breakdown.offset[4:2];
                    
                    data_web[saved_lru_way] = '0;
                    tag_web[saved_lru_way] = '0;
                    valid_web[saved_lru_way] = '0;
                    dirty_web[saved_lru_way] = '0;

                    dirty_din = (saved_wmask != '0) ? '1 : '0;

                    data_din[saved_lru_way] = dfp_rdata;
                    data_wmask[saved_lru_way] = 32'hFFFFFFFF;

                    if(saved_wmask != '0) begin
                        for(integer unsigned byte_idx = 0 ; byte_idx < 4 ; byte_idx++) begin
                            if(saved_wmask[byte_idx]) begin
                                data_din[saved_lru_way][(word_offset*32) + (byte_idx*8) +: 8] = saved_wdata[byte_idx*8 +: 8];
                            end
                        end
                    end
                end
            end

            // REREAD: begin
            //     arrays_en = '1;
            //     plru_web = '0;
            // end

            default: begin
                arrays_en = '0;
            end
        endcase
    end


    // data path
    // always_comb begin
    //     logic [2:0] word_offset;
    //     word_offset = saved_addr_breakdown.offset[4:2];
        
    //     ufp_rdata = (state == HIT && cache_hit) ? data_out[hit_way][word_offset*32 +: 32] : 32'b0;
    // end

    always_comb begin
        logic [2:0] word_offset;
        word_offset = saved_addr_breakdown.offset[4:2];
        
        // return data on READ operations (when rmask != 0)
        if (state == HIT && cache_hit) begin

            if (saved_wmask == '0) begin

                masked_rdata = data_out[hit_way][word_offset*32 +: 32];

                // apply rmask
                for (integer i = 0; i < 4; i++) begin
                    if (!saved_rmask[i])
                        masked_rdata[i*8 +: 8] = 8'h00;
                end

                ufp_rdata = masked_rdata;
            end
            else
                ufp_rdata = 32'b0;
        end
        else
            ufp_rdata = 32'b0;
    end

    assign ufp_resp = (state == HIT && cache_hit);

    // mem interf
    always_comb begin
        dfp_addr = 32'h0;
        dfp_read = 1'b0;
        dfp_write = 1'b0;
        dfp_wdata = 256'h0;
        
        case (state)
            WRITEBACK: begin
                // write dirty line back mem
                dfp_write = 1'b1;
                dfp_addr = {tag_out[saved_lru_way], saved_addr_breakdown.index, 5'b0};
                dfp_wdata = data_out[saved_lru_way];
            end
            
            ALLOCATE: begin
                // read new line from mem
                dfp_read = 1'b1;
                dfp_addr = {saved_addr_breakdown.tag, saved_addr_breakdown.index, 5'b0};
            end
            
            default: begin
                dfp_addr = 32'h0;
                dfp_read = 1'b0;
                dfp_write = 1'b0;
                dfp_wdata = 256'h0;
            end
        endcase
    end

    always_comb begin
        if (state == HIT && cache_hit) begin
            ufp_rdata_line = data_out[hit_way];
        end else begin
            ufp_rdata_line = 256'b0;
        end
    end

endmodule