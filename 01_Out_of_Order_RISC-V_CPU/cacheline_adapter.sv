module cacheline_adapter
(
    input  logic         clk,
    input  logic         rst,
    
    input  logic [31:0]  cache_addr,
    input  logic         cache_read,
    input  logic         cache_write,
    output logic [255:0] cache_rdata,
    input  logic [255:0] cache_wdata,
    output logic         cache_resp,
    output logic [31:0]  cache_raddr,    // Address of the response (for verification)
    
    output logic [31:0]  bmem_addr,
    output logic         bmem_read,
    output logic         bmem_write,
    input  logic [63:0]  bmem_rdata,
    output logic [63:0]  bmem_wdata,
    input  logic         bmem_ready,
    input  logic [31:0]  bmem_raddr,
    input  logic         bmem_rvalid
);

    // FSM
    typedef enum logic [2:0] {
        IDLE        = 3'b000,
        READ_BURST  = 3'b001,
        READ_DONE   = 3'b010,
        WRITE_BURST = 3'b011
    } adapter_state_t;

    adapter_state_t state, nxt_state;

    logic [31:0]  saved_addr;
    logic [255:0] read_buffer;
    logic [255:0] saved_wdata;
    logic [1:0]   burst_counter;
    logic [1:0]   nxt_burst_counter;

    always_ff @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            burst_counter <= 2'b00;
        end else begin
            state <= nxt_state;
            burst_counter <= nxt_burst_counter;
        end
    end

    // Save addr and write data
    always_ff @(posedge clk) begin
        if (rst) begin
            saved_addr <= 32'h0;
            saved_wdata <= 256'h0;
            read_buffer <= 256'h0;
        end 
        
        else begin
            // Save addr on new request
            if (state == IDLE && (cache_read || cache_write))
                saved_addr <= {cache_addr[31:5], 5'b00000};
            
            // Save write data on write request
            if (state == IDLE && cache_write)
                saved_wdata <= cache_wdata;
            
            // Accumulate read data during read burst
            if (state == READ_BURST && bmem_rvalid && bmem_raddr == saved_addr) begin
                case (burst_counter)
                    2'b00: read_buffer[63:0]    <= bmem_rdata;
                    2'b01: read_buffer[127:64]  <= bmem_rdata;
                    2'b10: read_buffer[191:128] <= bmem_rdata;
                    2'b11: read_buffer[255:192] <= bmem_rdata;
                endcase
            end
        end
    end

    always_comb begin
        nxt_state = state;
        nxt_burst_counter = burst_counter;

        case (state)
            IDLE: begin
                if (cache_read && bmem_ready) begin
                    nxt_state = READ_BURST;
                    nxt_burst_counter = 2'b00;
                end 
                
                else if (cache_write && bmem_ready) begin
                    nxt_state = WRITE_BURST;
                    nxt_burst_counter = 2'b01;
                end
            end

            READ_BURST: begin
                if (bmem_rvalid && bmem_raddr == saved_addr) begin
                    if (burst_counter == 2'b11) begin
                        nxt_state = READ_DONE;
                        nxt_burst_counter = 2'b00;
                    end 
                    
                    else
                        nxt_burst_counter = burst_counter + 2'b1;
                end
            end

            WRITE_BURST: begin
                if (bmem_ready) begin
                    if (burst_counter == 2'b11) begin
                        nxt_state = IDLE;
                        nxt_burst_counter = 2'b00;
                    end 
                    else
                        nxt_burst_counter = burst_counter + 2'b1;
                end
                // If not ready, stay in same state with same counter
            end

            READ_DONE: begin
                nxt_state = IDLE;
            end

            default: begin
                nxt_state = IDLE;
                nxt_burst_counter = 2'b00;
            end
        endcase
    end

    // output
    always_comb begin
        bmem_addr = saved_addr;
        bmem_read = 1'b0;
        bmem_write = 1'b0;
        bmem_wdata = 64'h0;
        cache_rdata = read_buffer;
        cache_resp = 1'b0;
        cache_raddr = saved_addr;  // Return address for response verification

        case (state)
            IDLE: begin
                // on new read request, assert bmem_read for ONE cycle
                if (cache_read) begin
                    bmem_addr = {cache_addr[31:5], 5'b00000};
                    bmem_read = 1'b1;
                end
                else if (cache_write) begin
                    bmem_addr = {cache_addr[31:5], 5'b00000};
                    bmem_write = 1'b1;
                    bmem_wdata = cache_wdata[63:0];
                end
            end

            READ_BURST: begin
                // bmem_read stays LOW (only asserted for 1 cycle in IDLE)
                // wait for rvalid and accumulate data
            end

            WRITE_BURST: begin
                // keep bmem_write HIGH for all 4 cycles
                bmem_write = 1'b1;
                
                case (burst_counter)
                    2'b01: bmem_wdata = saved_wdata[127:64];
                    2'b10: bmem_wdata = saved_wdata[191:128];
                    2'b11: bmem_wdata = saved_wdata[255:192];
                    default: bmem_wdata = 64'h0;
                endcase
                
                // assert cache_resp on last cycle
                if (burst_counter == 2'b11 && bmem_ready)
                    cache_resp = 1'b1;
            end

            READ_DONE: begin
                cache_resp = 1'b1;
            end

            default: begin
            end
        endcase
    end

endmodule : cacheline_adapter