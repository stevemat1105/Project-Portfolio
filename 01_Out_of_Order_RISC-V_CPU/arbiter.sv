// 1. READS use entries (need to track 4-beat resp)
// 2. WRITES bypass entries (no resp, just forward 4 beats directly)
// 3. Write burst FSM tracks 4-cycle write operations
//
//   Adapter-I <--v
//                |<--> Arbiter <--> DRAM
//   Adapter-D <--^

module arbiter (
    input  logic        clk,
    input  logic        rst,
    
    // (from I-Cache adapter)
    input  logic [31:0]  bmem_addr_i,
    input  logic         bmem_read_i,
    input  logic         bmem_write_i,
    output logic [63:0]  bmem_rdata_i,
    input  logic [63:0]  bmem_wdata_i,
    output logic         bmem_ready_i,
    output logic [31:0]  bmem_raddr_i,
    output logic         bmem_rvalid_i,
    
    // (from D-Cache adapter)
    input  logic [31:0]  bmem_addr_d,
    input  logic         bmem_read_d,
    input  logic         bmem_write_d,
    output logic [63:0]  bmem_rdata_d,
    input  logic [63:0]  bmem_wdata_d,
    output logic         bmem_ready_d,
    output logic [31:0]  bmem_raddr_d,
    output logic         bmem_rvalid_d,
    
    // DRAM
    output logic [31:0]  bmem_addr,
    output logic         bmem_read,
    output logic         bmem_write,
    input  logic [63:0]  bmem_rdata,
    output logic [63:0]  bmem_wdata,
    input  logic         bmem_ready,
    input  logic [31:0]  bmem_raddr,
    input  logic         bmem_rvalid
);

    // Pending Request for READS
    typedef struct packed {
        logic        valid;     // entry occupied?
        logic [31:0] addr;      // addr of pending request
        logic        source;    // 0 = Adapter-I, 1 = Adapter-D
    } pending_entry_t;
    
    pending_entry_t pending [2];  // up to 2 outstanding requests
    
    // beat counter for each pending entry (0-3 for 4 beats)
    logic [1:0] beat_counter [2];
    
    // WRITE Burst Tracking
    logic        write_in_progress;  // forwarding a write burst?
    logic        write_source;       // 0 = I-Cache, 1 = D-Cache
    logic [31:0] write_addr;         // addr being written (for priority check)
    
    logic [1:0] free_entry;
    logic       found_free;

    logic [1:0] free_count;
    logic       found_free_for_d, found_free_for_i;

    always_comb begin
        free_entry = 2'b00;
        found_free = 1'b0;
        free_count = 2'b00;

        for (integer unsigned i = 0; i < 2; i++) begin
            if (!pending[i].valid) begin
                free_count = free_count + 1'b1;
                if (!found_free) begin
                    free_entry = 2'(i);
                    found_free = 1'b1;
                end
            end
        end
    end

    assign found_free_for_d = (free_count >= 2'd1);
    assign found_free_for_i = (free_count >= 2'd2);
    
    logic grant_read_i, grant_read_d;
    logic grant_write_i, grant_write_d;
    
    always_comb begin
        grant_read_i = 1'b0;
        grant_read_d = 1'b0;
        grant_write_i = 1'b0;
        grant_write_d = 1'b0;
        
        if (!write_in_progress && bmem_ready) begin
            
            // WRITE priority (I > D)
            if (bmem_write_i)
                grant_write_i = 1'b1;
            else if (bmem_write_d)
                grant_write_d = 1'b1;
            
            // READ priority (D > I) - D-cache gets priority to prevent load starvation
            else if (bmem_read_d && found_free_for_d)
                grant_read_d = 1'b1;
            else if (bmem_read_i && found_free_for_i)
                grant_read_i = 1'b1;
        end
    end
    
    always_comb begin
        bmem_ready_i = grant_read_i || grant_write_i;
        bmem_ready_d = grant_read_d || grant_write_d;
        
        // During write bursts, forward DRAM's bmem_ready ONLY if adapter is still writing
        if (write_in_progress) begin
            if (write_source == 1'b0 && bmem_write_i)       // Check if still writing
                bmem_ready_i = bmem_ready;
            else if (write_source == 1'b1 && bmem_write_d)  // Check if still writing
                bmem_ready_d = bmem_ready;
        end
    end
    
    // WRITE Burst FSM
    always_ff @(posedge clk) begin

        if (rst) begin
            write_in_progress <= 1'b0;
            write_source <= 1'b0;
            write_addr <= 32'h0;
        end 
        
        else begin
            // new write burst
            if (grant_write_i) begin
                write_in_progress <= 1'b1;
                write_source <= 1'b0;       // I-Cache
                write_addr <= bmem_addr_i;
            end 
            else if (grant_write_d) begin
                write_in_progress <= 1'b1;
                write_source <= 1'b1;       // D-Cache
                write_addr <= bmem_addr_d;
            end
            
            // end write burst when source deasserts bmem_write
            if (write_in_progress) begin
                if (write_source == 1'b0 && !bmem_write_i)
                    // I write done
                    write_in_progress <= 1'b0;
                else if (write_source == 1'b1 && !bmem_write_d)
                    // D write done
                    write_in_progress <= 1'b0;
            end
        end
    end
    
    always_comb begin
        bmem_addr = 32'h0;
        bmem_read = 1'b0;
        bmem_write = 1'b0;
        bmem_wdata = 64'h0;
        
        // WRITE priority
        if (write_in_progress && ((write_source == 1'b0 && bmem_write_i) || (write_source == 1'b1 && bmem_write_d))) begin
            bmem_addr = write_addr;
            
            if (write_source == 1'b0) begin
                // from I-Cache
                bmem_write = bmem_write_i;
                bmem_wdata = bmem_wdata_i;
            end 
            else begin
                // Forward from D-Cache
                bmem_write = bmem_write_d;
                bmem_wdata = bmem_wdata_d;
            end
        end
        
        // no write in progress? -> handle reads (D-cache has priority)
        else if (grant_read_d) begin
            bmem_addr = bmem_addr_d;
            bmem_read = bmem_read_d;
        end
        else if (grant_read_i) begin
            bmem_addr = bmem_addr_i;
            bmem_read = bmem_read_i;
        end
        
        // new write grants
        else if (grant_write_i) begin
            bmem_addr = bmem_addr_i;
            bmem_write = bmem_write_i;
            bmem_wdata = bmem_wdata_i;
        end 
        else if (grant_write_d) begin
            bmem_addr = bmem_addr_d;
            bmem_write = bmem_write_d;
            bmem_wdata = bmem_wdata_d;
        end
    end
    
    logic [1:0] match_entry;
    logic       found_match;
    
    always_comb begin
        match_entry = 2'b00;
        found_match = 1'b0;
        
        // check both entries for addr match
        for (integer unsigned i = 0; i < 2; i++) begin

            if (pending[i].valid && pending[i].addr == bmem_raddr && !found_match) begin
                match_entry = 2'(i);
                found_match = 1'b1;
            end
        end
    end
    
    // READ Resp routing by addr match
    always_comb begin

        bmem_rvalid_i = 1'b0;
        bmem_rvalid_d = 1'b0;
        
        bmem_rdata_i = bmem_rdata;
        bmem_rdata_d = bmem_rdata;
        bmem_raddr_i = bmem_raddr;
        bmem_raddr_d = bmem_raddr;
        
        if (bmem_rvalid && found_match) begin

            if (pending[match_entry].source == 1'b0)
                // to Adapter-I
                bmem_rvalid_i = 1'b1;
            else
                // to Adapter-D
                bmem_rvalid_d = 1'b1;
        end
    end
    
    // Pending Entry (READS ONLY)
    always_ff @(posedge clk) begin

        if (rst) begin

            for (integer unsigned i = 0; i < 2; i++) begin
                pending[i].valid <= 1'b0;
                pending[i].addr <= 32'h0;
                pending[i].source <= 1'b0;
                beat_counter[i] <= 2'b00;
            end
        end 
        
        else begin
            if (grant_read_i) begin
                pending[free_entry].valid <= 1'b1;
                pending[free_entry].addr <= bmem_addr_i;
                pending[free_entry].source <= 1'b0;     // I-Cache
                beat_counter[free_entry] <= 2'b00;
            end 
            else if (grant_read_d) begin
                pending[free_entry].valid <= 1'b1;
                pending[free_entry].addr <= bmem_addr_d;
                pending[free_entry].source <= 1'b1;     // D-Cache
                beat_counter[free_entry] <= 2'b00;
            end
            
            if (bmem_rvalid && found_match) begin
                // ++ beat counter
                beat_counter[match_entry] <= beat_counter[match_entry] + 2'b01;
                
                if (beat_counter[match_entry] == 2'b11) begin
                    pending[match_entry].valid <= 1'b0;
                    beat_counter[match_entry] <= 2'b00;
                end
            end
        end
    end
endmodule