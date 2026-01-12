// 2-WAY SUPERSCALAR LINEBUFFER
//
// Supports dual read ports for fetching two instructions from same cacheline.
// Port 0: Primary fetch
// Port 1: Secondary fetch (for PC+4, only valid if in same cacheline as port 0)

module linebuffer
import types::*;
(
    input  logic        clk,
    input  logic        rst,

     
    // FETCH PORT 0 (primary - from fetch stage)
     
    input  logic [31:0] fetch_addr,
    input  logic [3:0]  fetch_rmask,
    output logic [31:0] fetch_rdata,
    output logic        fetch_resp,

     
    // FETCH PORT 1 (secondary - superscalar, same cacheline as port 0)
     
    input  logic [31:0] fetch_addr_1,
    input  logic [3:0]  fetch_rmask_1,
    output logic [31:0] fetch_rdata_1,
    output logic        fetch_resp_1,

    // (from fetch for branches)
    input  logic        invalidate,

    // (to cache)
    output logic [31:0] cache_addr,
    output logic [3:0]  cache_rmask,
    input  logic [31:0] cache_rdata,
    input  logic [255:0] cache_rdata_line,
    input  logic        cache_resp
);

    logic [255:0] linebuffer_data;
    logic [26:0]  linebuffer_tag;    // addr[31:5]
    logic         linebuffer_valid;

    logic [26:0]  pending_tag;
    logic         pending_valid;

     
    // PORT 0 - Primary fetch (same as original)
     
    logic linebuffer_hit;

    assign linebuffer_hit = linebuffer_valid && (fetch_addr[31:5] == linebuffer_tag) && (fetch_rmask != 4'b0000);

    logic [2:0]  word_offset;
    logic [31:0] linebuffer_word;

    assign word_offset = fetch_addr[4:2];
    assign linebuffer_word = linebuffer_data[word_offset*32 +: 32];

    // linebuffer hits ? -> serve directly (1 cycle!)
    // or pass through to cache
    assign fetch_rdata = linebuffer_hit ? linebuffer_word : cache_rdata;
    assign fetch_resp = linebuffer_hit ? 1'b1 : (cache_resp && pending_valid);

     
    // PORT 1 - Secondary fetch (superscalar)
     
    // Port 1 can only hit if:
    // 1. Port 0 hits (both in same linebuffer line), OR
    // 2. Port 0 is getting a cache response AND port 1 is in same line
    // 3. Port 1 address is in same cacheline as port 0

    logic linebuffer_hit_1;
    logic same_cacheline;

    // Check if port 1 is in same cacheline as port 0
    assign same_cacheline = (fetch_addr[31:5] == fetch_addr_1[31:5]);

    // Port 1 hits linebuffer if valid, same tag, and has valid rmask
    assign linebuffer_hit_1 = linebuffer_valid && (fetch_addr_1[31:5] == linebuffer_tag) && (fetch_rmask_1 != 4'b0000);

    logic [2:0]  word_offset_1;
    logic [31:0] linebuffer_word_1;

    assign word_offset_1 = fetch_addr_1[4:2];
    assign linebuffer_word_1 = linebuffer_data[word_offset_1*32 +: 32];

    // Port 1 data: from linebuffer if hit, otherwise from cache response line
    // When cache responds, both port 0 and port 1 can get data from cache_rdata_line
    logic [31:0] cache_word_1;
    assign cache_word_1 = cache_rdata_line[word_offset_1*32 +: 32];

    assign fetch_rdata_1 = linebuffer_hit_1 ? linebuffer_word_1 : cache_word_1;

    // Port 1 responds if:
    // - Linebuffer hit for port 1, OR
    // - Cache response and port 1 is in same cacheline being fetched
    assign fetch_resp_1 = (linebuffer_hit_1) ||
                          (cache_resp && pending_valid && (pending_tag == fetch_addr_1[31:5]) && (fetch_rmask_1 != 4'b0000));

     
    // CACHE REQUEST LOGIC
     
    // pass request to cache only on miss
    assign cache_addr = fetch_addr;
    assign cache_rmask = linebuffer_hit ? 4'b0000 : fetch_rmask;
    // rmask = 0 means no request (linebuffer hit)
    // rmask = fetch_rmask means pass through (linebuffer miss)

    always_ff @(posedge clk) begin
        if (rst || invalidate) begin
            linebuffer_valid <= 1'b0;
            linebuffer_tag <= '0;
            linebuffer_data <= '0;
            pending_valid <= 1'b0;
            pending_tag <= '0;
        end 
        else begin
            // send cache request (miss + valid rmask + no pending request), save the tag
            if (!linebuffer_hit && (fetch_rmask != 4'b0000) && !pending_valid) begin
                pending_valid <= 1'b1;
                pending_tag <= fetch_addr[31:5];
            end
            
            // accept cache resp only if it matches pending request
            if (cache_resp && pending_valid) begin
                linebuffer_data <= cache_rdata_line;
                linebuffer_tag <= pending_tag;
                linebuffer_valid <= 1'b1;
                pending_valid <= 1'b0;
            end
        end
    end

endmodule : linebuffer