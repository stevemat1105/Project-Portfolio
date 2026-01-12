module top_tb;
    //---------------------------------------------------------------------------------
    // Waveform generation.
    //---------------------------------------------------------------------------------
    initial begin
        $fsdbDumpfile("dump.fsdb");
        $fsdbDumpvars(0, "+all");
    end

    //---------------------------------------------------------------------------------
    // TODO: Declare cache port signals:
    //---------------------------------------------------------------------------------
        logic           rst;

        // cpu side signals, ufp -> upward facing port
        logic   [31:0]  ufp_addr;
        logic   [3:0]   ufp_rmask;
        logic   [3:0]   ufp_wmask;
        logic   [31:0]  ufp_rdata;
        logic   [31:0]  ufp_wdata;
        logic           ufp_resp;

        // memory side signals, dfp -> downward facing port
        logic   [31:0]  dfp_addr;
        logic           dfp_read;
        logic           dfp_write;
        logic   [255:0] dfp_rdata;
        logic   [255:0] dfp_wdata;
        logic           dfp_resp;


    //---------------------------------------------------------------------------------
    // TODO: Instantiate the DUT:
    //---------------------------------------------------------------------------------
        cache dut(.*);
    //---------------------------------------------------------------------------------
    // TODO: Generate a clock:
    //---------------------------------------------------------------------------------
         bit clk = '0;
        always #2ns clk = ~clk; // Always drive clocks with blocking assignment.

    //---------------------------------------------------------------------------------
    // TODO: Verification constructs (recommended)
    //---------------------------------------------------------------------------------
    // Here's ASCII art of how the recommended testbench works:
    //                                +--------------+                           +-----------+
    //                       +------->| Golden model |---output_transaction_t--->|           |
    //                       |        +--------------+                           |           |
    //  input_transaction ---+                                                   | Check ==  |
    //                       |        +------+                                   |           |
    //                       +------->|  DUT |---output_transaction_t----------->|           |
    //                                +------+                                   +-----------+

    // Struct that defines an "input transaction" -- this is basically one
    // operation that's done on the cache.
    // ... what else defines an input transaction? Think: rmask/wmask, data...

    // Note that it's useful to include the DFP signals here as well for
    // planned misses, like this:

    typedef enum logic [0:0] {
    READ  = 1'b0,
    WRITE = 1'b1
    } transaction_type_e;



    

    typedef struct packed {
        logic [31:0] address; // Address to read from.
        transaction_type_e transaction_type; // Read or write? You could make an enum for this.
        logic [3:0] read_mask;
        logic [3:0] write_mask;
        logic [31:0] wdata;
        bit [255:0] dfp_rdata;
        // bit hit;
        

    } input_transaction_t;

    // The output transaction tells us how the cache is expected to behave due
    // to an input transaction.
    typedef struct packed {
        bit caused_writeback;
        bit caused_allocate;
        bit write_hit;
        bit read_hit;
        bit [31:0] returned_data;
        bit [255:0] dfp_writeback_data;
        bit [2:0] lru_value;
        bit[1:0] evict_way;
        // what else do you need?
    } output_transaction_t;

    logic [255:0] data_golden_arrays[16][4];
    logic [22:0] tag_golden_arrays[16][4];
    logic  dirty_golden_arrays[16][4];
    logic  valid_golden_arrays[16][4];
     logic [2:0] plru_golden_arrays[16];
    // Similarly, make arrays for tags, valid, dirty, plru.

 function input_transaction_t generate_input_transaction();
    input_transaction_t inp;
    
    bit do_hit;
    bit rw;
    bit [3:0] read_mask;
    bit [3:0] write_mask;
    bit [31:0] wdata;
    bit [31:0] addr;
    bit [255:0] dfp_rdata;
    logic [3:0] set_idx;
    logic [22:0] tag;
    bit is_hit;
    bit first_transaction;

    first_transaction = '1;
     for(int i = 0; i < 16; i++) begin
        if(!first_transaction)begin
            break;
        end
        for(int j = 0; j < 4; j++) begin
            if(valid_golden_arrays[i][j]) begin
                first_transaction = '0;
            end
        end
     end

    
    // Randomize control signals
    std::randomize(rw);
    std::randomize(do_hit);
    std::randomize(wdata);
    std::randomize(dfp_rdata);
    if(first_transaction) begin
        do_hit = '0; //force a miss
    end
    // inp.hit = do_hit;
     do begin
        std::randomize(read_mask);
        std::randomize(write_mask);
    end while (read_mask == 4'b0 || write_mask == 4'b0);
    // Set transaction type and data based on read/write
    if(rw) begin
        inp.transaction_type = WRITE;
        inp.read_mask = '0;
        inp.write_mask = write_mask;
        inp.wdata = wdata;
    end
    else begin  
        inp.transaction_type = READ;
        inp.read_mask = read_mask;
        inp.write_mask = '0;
        inp.wdata = '0;
    end
    
    // Set address and dfp signals based on hit/miss
    if (do_hit) begin
        inp.address = get_cached_addresses();  // Changed 'input' to 'inp'
        inp.dfp_rdata = '0;
    end 
    else begin // Miss
        // For a miss, use random address
     bit found_miss_addr;
    found_miss_addr = '0;
    
    while (!found_miss_addr) begin
        std::randomize(addr);
         set_idx = addr[8:5];
         tag = addr[31:9];
         is_hit = '0;
        
        // Check if this address would hit
        for (int i = 0; i < 4; i++) begin
            if (valid_golden_arrays[set_idx][i] && 
                tag_golden_arrays[set_idx][i] == tag) begin
                is_hit = '1;
                break;
            end
        end
        
        if (!is_hit) begin
            found_miss_addr = '1;
        end
    end
        inp.address = addr;
        inp.dfp_rdata = dfp_rdata;
    end
    
    return inp;  
endfunction : generate_input_transaction

function logic[31:0] get_cached_addresses();
    logic [4:0] rand_byte_offset;
    logic [255:0] data;
    logic [22:0] tag;
    logic [3:0] set_idx;
    logic [31:0] new_addr;
    std::randomize(rand_byte_offset);
    
    // First, try to find a valid entry in the cache
    for(int i = 0; i < 16; i++) begin
        for(int j = 0; j < 4; j++) begin
            if(valid_golden_arrays[i][j]) begin
                return {tag_golden_arrays[i][j], i[3:0], rand_byte_offset};
            end
        end
    end

endfunction


  task load_cacheline (input logic [31:0] addr,  input logic [255:0] line, input bit dirty_after);
    input_transaction_t miss;
  output_transaction_t dout_unused;

  miss.address         = addr;
  miss.transaction_type= READ;         // force allocate path
  miss.read_mask       = 4'b1111;
  miss.write_mask      = 4'b0000;
  miss.wdata           = '0;
  miss.dfp_rdata       = line;

  // Update golden as if a miss/allocate happened.
  void'(golden_cache_do(miss));

  // Drive DUT to allocate: it will assert dfp_read, we return dfp_rdata & dfp_resp
  drive_dut(miss, dout_unused);

  if (dirty_after) begin
    // make it dirty in both models by doing a write hit to the same word
    input_transaction_t w;
    output_transaction_t tmp;
    w = '{address: addr, transaction_type: WRITE, read_mask:'0,
         write_mask: 4'b1111, wdata: 32'hFACE_CAFE, dfp_rdata:'0};
    void'(golden_cache_do(w));
    drive_dut(w, tmp);
  end



  endtask
  function output_transaction_t golden_cache_do(input_transaction_t inp);
    output_transaction_t out;
    logic [3:0] set_idx;
    logic [22:0] tag;
    logic [2:0] word_offset;
    logic [4:0] byte_offset;
    logic hit;
    int hit_way, evict_way;
    logic [255:0] shifted_data;
    logic [31:0] shift_mask;
    logic set_full;
    // Find hit
    set_idx = inp.address[8:5];
    tag = inp.address[31:9];
    word_offset = inp.address[4:2];
    byte_offset = inp.address[4:0];
    hit = '0;
    hit_way = -1;
    set_full = '1;

    // Find hit and check if set is full
    for (int i = 0; i < 4; i++) begin
        if (tag_golden_arrays[set_idx][i] == tag && valid_golden_arrays[set_idx][i]) begin
            hit = '1;
            hit_way = i;
        end
        if (!valid_golden_arrays[set_idx][i])
            set_full = '0;
    end

    // Default output
    out = '{default:0};

    if (hit) begin
        out.read_hit = (inp.transaction_type == READ);
        out.write_hit = (inp.transaction_type == WRITE);
        out.caused_allocate = '0;
        out.caused_writeback = '0;
        out.dfp_writeback_data = '0;

        if (inp.transaction_type == READ) begin
            shift_mask = {{8{inp.read_mask[3]}}, {8{inp.read_mask[2]}}, {8{inp.read_mask[1]}}, {8{inp.read_mask[0]}}};
            shifted_data = data_golden_arrays[set_idx][hit_way] >> (word_offset * 32);
            out.returned_data = shifted_data[31:0] & shift_mask;
        end
        else if (inp.transaction_type == WRITE) begin
            // Merge write data into cache line
            for (int b = 0; b < 4; b++) begin
                if (inp.write_mask[b])
                    data_golden_arrays[set_idx][hit_way][(word_offset*32)+(b*8) +: 8] = inp.wdata[b*8 +: 8];
            end
            dirty_golden_arrays[set_idx][hit_way] = 1'b1;
        end
        
                case(hit_way)
                    2'b00: plru_golden_arrays[set_idx] = {plru_golden_arrays[set_idx][2], 1'b0, 1'b0};
                    2'b01: plru_golden_arrays[set_idx] = {plru_golden_arrays[set_idx][2], 1'b1, 1'b0};
                    2'b10: plru_golden_arrays[set_idx] = {1'b0, plru_golden_arrays[set_idx][1], 1'b1};
                    2'b11: plru_golden_arrays[set_idx] = {1'b1, plru_golden_arrays[set_idx][1], 1'b1};
                endcase

                 out.lru_value = plru_golden_arrays[set_idx]; 


    end
    else begin // Miss
        out.read_hit = '0;
        out.write_hit = '0;
        out.returned_data = '0;

   
                casez(plru_golden_arrays[set_idx])
                        3'b?11:begin 
                             evict_way = 0;
                        end
                        3'b?01: begin
                          evict_way = 1;
                        end
                        3'b1?0: begin
                            evict_way = 2;
                        end
                        3'b0?0: begin
                            evict_way = 3;
                        end
                        endcase
           
        

        out.evict_way = unsigned'(evict_way[1:0]);
        // Writeback if dirty
        if (valid_golden_arrays[set_idx][evict_way] && dirty_golden_arrays[set_idx][evict_way]) begin
            out.caused_writeback = '1;
            out.dfp_writeback_data = data_golden_arrays[set_idx][evict_way];
        end else begin
            out.caused_writeback = '0;
            out.dfp_writeback_data = '0;
        end

        // Allocate new line
        out.caused_allocate = '1;
        tag_golden_arrays[set_idx][evict_way] = tag;
        valid_golden_arrays[set_idx][evict_way] = 1'b1;
        dirty_golden_arrays[set_idx][evict_way] = (inp.transaction_type == WRITE);


        case(evict_way)
        2'b00: plru_golden_arrays[set_idx] = {plru_golden_arrays[set_idx][2], 1'b0, 1'b0};
        2'b01: plru_golden_arrays[set_idx] = {plru_golden_arrays[set_idx][2], 1'b1, 1'b0};
        2'b10: plru_golden_arrays[set_idx] = {1'b0, plru_golden_arrays[set_idx][1], 1'b1};
        2'b11: plru_golden_arrays[set_idx] = {1'b1, plru_golden_arrays[set_idx][1], 1'b1};
    endcase

        out.lru_value = plru_golden_arrays[set_idx]; 
        if (inp.transaction_type == READ) begin
            data_golden_arrays[set_idx][evict_way] = inp.dfp_rdata;
            // Return requested word
            shift_mask = {{8{inp.read_mask[3]}}, {8{inp.read_mask[2]}}, {8{inp.read_mask[1]}}, {8{inp.read_mask[0]}}};
            shifted_data = inp.dfp_rdata >> (word_offset * 32);
            out.returned_data = shifted_data[31:0] & shift_mask;
        end
        else if (inp.transaction_type == WRITE) begin
            data_golden_arrays[set_idx][evict_way] = inp.dfp_rdata;
            for (int b = 0; b < 4; b++) begin
                if (inp.write_mask[b])
                    data_golden_arrays[set_idx][evict_way][(word_offset*32)+(b*8) +: 8] = inp.wdata[b*8 +: 8];
            end
        end

    end

    return out;
endfunction : golden_cache_do



// task drive_dut(input input_transaction_t inp, output output_transaction_t out);
//     out = '{default:0};
//     int cycle_counter = 0;
//     ufp_addr <= inp.address;
//     ufp_rmask <= inp.read_mask;
//     ufp_wmask <= inp.write_mask;
//     ufp_wdata <= inp.wdata;

//     @(posedge clk);
  
//   if(inp.hit) begin 
//     fork
//         begin
        
//         end

//         begin
//              if (cycle_counter > 1 ) begin
//                 $error("error timing not met for hit")
                
//              end
//             cycle_counter++;
//         end

//     join

//     disable(fork);
//   end
//   else begin

//     if(inp.writeback) begin
//     fork
//      begin
//     @(posedge clk iff dfp_write);
//          $display("  [DUT] Writeback detected: data=0x%h", dfp_wdata);
//             out.caused_writeback = 1'b1;
//             out.dfp_writeback_data = dfp_wdata;
//             @(posedge clk);
//             dfp_resp <= 1'b1;
//             @(posedge clk);
//             dfp_resp <= 1'b0;
//     end

   
         
//          begin
//              if (cycle_counter > 4 ) begin
//                  $error("timing not met for writeback")
                
//              end
//             cycle_counter++;
//         end

//     join
//     disable(fork);
//     end





//    if(inp.allocate) begin 
//     fork
//         begin
//              @(posedge clk iff dfp_read);
//              $display("  [DUT] Allocate detected: providing data=0x%h", inp.dfp_rdata);
//             out.caused_allocate = 1'b1;
//             dfp_rdata <= inp.dfp_rdata;
//             @(posedge clk);
//             dfp_resp <= 1'b1;
//             @(posedge clk);
//             dfp_resp <= 1'b0;
//         end

//         begin
//              if (cycle_counter > 1 ) begin
//                  $error("timing not met for allocate")
                
//              end
//             cycle_counter++;
//         end

//     join

//     disable(fork);
//   end
//   end

//     out.read_hit = (inp.read_mask != '0 && !out.caused_allocate);
//     out.write_hit = (inp.write_mask != '0 && !out.caused_allocate);
    
//     if (inp.transaction_type_e == READ) begin
//         out.returned_data = ufp_rdata;
//         $display("  [DUT] Read data returned: 0x%h", ufp_rdata);
//     end

//     ufp_rmask <= 4'b0;
//     ufp_wmask <= 4'b0;
//     @(posedge clk);
//     out.lru_value = dut.lru_dout;
//     $display("  [DUT] LRU value: 0b%b", out.lru_value);

   
// endtask





task automatic drive_dut(input input_transaction_t inp, output output_transaction_t out);
   int cycle_count = 0;
  static int max_cycles = 100; // Safety timeout
    out = '{default:0};

    
    // Drive UFP signals
    ufp_addr <= inp.address;
    ufp_rmask <= inp.read_mask;
    ufp_wmask <= inp.write_mask;
    ufp_wdata <= inp.wdata;

    @(posedge clk);
    cycle_count = 0;
  
    
while (!ufp_resp && cycle_count < max_cycles) begin
  @(posedge clk);
  cycle_count++;
  if (dfp_write) begin
    $display("[DUT] Writeback @ c=%0d: addr=0x%h data=0x%h",
             cycle_count, dfp_addr, dfp_wdata);
    out.caused_writeback    = 1'b1;
    out.dfp_writeback_data  = dfp_wdata;

    repeat (1) @(posedge clk);       
    dfp_resp <= 1'b1; @(posedge clk); 
    dfp_resp <= 1'b0;

    cycle_count += 2;  
    continue;          
  end

  if (dfp_read) begin
    $display("[DUT] Allocate @ c=%0d: addr=0x%h", cycle_count, dfp_addr);
    out.caused_allocate = 1'b1;

    repeat (1) @(posedge clk);        
    dfp_rdata <= inp.dfp_rdata;       
    dfp_resp  <= 1'b1; @(posedge clk);
    dfp_resp  <= 1'b0;

    cycle_count += 2;
    continue;
  end

end


    if (cycle_count >= max_cycles) begin
        $error("  [DUT] Timeout waiting for ufp_resp after %0d cycles", cycle_count);
        $finish;
    end
    
    // Capture outputs when ufp_resp is asserted
    $display("  [DUT] Transaction completed at cycle %0d", cycle_count);
    
    // Infer hit/miss from behavior
    out.read_hit = (inp.read_mask != '0 && !out.caused_allocate);
    out.write_hit = (inp.write_mask != '0 && !out.caused_allocate);
    
    // Capture read data if it was a read operation
    if (inp.read_mask != '0) begin
        out.returned_data = ufp_rdata;
        $display("  [DUT] Read data returned: 0x%h", ufp_rdata);
    end
    
    // Check latency requirements
    if (out.read_hit || out.write_hit) begin
        // Hit should complete in 1 cycle
        if (cycle_count > 1) begin
            $error("  [DUT] Hit took %0d cycles, expected 1 cycle", cycle_count);
        end else begin
            $display("  [DUT] ✓ Hit latency: 1 cycle");
        end
    end else begin
        // Miss handling
      int expected_cycles = 0;
        //9 cycles for dirty miss and 6 for clean miss
        if (out.caused_writeback) begin
            expected_cycles += 3; //3 cycles in writeback
        end
        
        if (out.caused_allocate) begin
            expected_cycles += 6; // 3 cycles in allocate + hit + idle + hit
        end
        else begin
            expected_cycles +=1; //latency for a hit 
        end 
        
        if (cycle_count != expected_cycles) begin
            $error("  [DUT] Miss took %0d cycles, expected %0d cycles", 
                     cycle_count, expected_cycles);
        end else begin
            $display("  [DUT] ✓ Miss latency: %0d cycles (writeback=%b, allocate=%b)", 
                     cycle_count, out.caused_writeback, out.caused_allocate);
        end
    end
    
    // Capture evicted way if allocation occurred
    if (out.caused_allocate) begin
        out.evict_way = dut.victim_index;
        $display("  [DUT] Evicted way: %0d", out.evict_way);
    end
    
    // Clear UFP signals
    ufp_rmask <= 4'b0;
    ufp_wmask <= 4'b0;
    @(posedge clk);
    
    // Capture LRU state after transaction
    @(posedge clk);
    out.lru_value = dut.lru_dout0;
    $display("  [DUT] LRU value: 0b%b", out.lru_value);
endtask







function compare_outputs(output_transaction_t golden_out, output_transaction_t dut_out);
    static bit pass = 1'b1;
    
    $display("  [COMPARE] Checking outputs...");
    
    if (golden_out.read_hit !== dut_out.read_hit) begin
        $error("    Read hit mismatch: expected %b, got %b", golden_out.read_hit, dut_out.read_hit);
        pass = 1'b0;
    end else if (golden_out.read_hit) begin
        $display("    ✓ Read hit matches: %b", golden_out.read_hit);
    end
    
    if (golden_out.write_hit !== dut_out.write_hit) begin
        $error("    Write hit mismatch: expected %b, got %b", golden_out.write_hit, dut_out.write_hit);
        pass = 1'b0;
    end else if (golden_out.write_hit) begin
        $display("    ✓ Write hit matches: %b", golden_out.write_hit);
    end
    
    if (golden_out.read_hit && (golden_out.returned_data !== dut_out.returned_data)) begin
        $error("    Returned data mismatch: expected 0x%h, got 0x%h", 
               golden_out.returned_data, dut_out.returned_data);
        pass = 1'b0;
    end else if (golden_out.read_hit) begin
        $display("    ✓ Returned data matches: 0x%h", golden_out.returned_data);
    end
    
    if (golden_out.caused_allocate !== dut_out.caused_allocate) begin
        $error("    Allocate mismatch: expected %b, got %b", golden_out.caused_allocate, dut_out.caused_allocate);
        pass = 1'b0;
    end else if (golden_out.caused_allocate) begin
        $display("    ✓ Allocate matches: %b", golden_out.caused_allocate);
    end
    
    if (golden_out.caused_writeback !== dut_out.caused_writeback) begin
        $error("    Writeback mismatch: expected %b, got %b", golden_out.caused_writeback, dut_out.caused_writeback);
        pass = 1'b0;
    end else if (golden_out.caused_writeback) begin
        $display("    ✓ Writeback matches: %b", golden_out.caused_writeback);
    end
    
    if (golden_out.caused_writeback && (golden_out.dfp_writeback_data !== dut_out.dfp_writeback_data)) begin
        $error("    Writeback data mismatch: expected 0x%h, got 0x%h", 
               golden_out.dfp_writeback_data, dut_out.dfp_writeback_data);
        pass = 1'b0;
    end
    
    if (golden_out.caused_allocate && (golden_out.evict_way !== dut_out.evict_way)) begin
        $error("    Evict way mismatch: expected %d, got %d", golden_out.evict_way, dut_out.evict_way);
        pass = 1'b0;
    end else if (golden_out.caused_allocate) begin
        $display("    ✓ Evict way matches: %0d", golden_out.evict_way);
    end
    
    if (golden_out.lru_value !== dut_out.lru_value) begin
        $error("    LRU value mismatch: expected 0b%b, got 0b%b", golden_out.lru_value, dut_out.lru_value);
        pass = 1'b0;
    end else begin
        $display("    ✓ LRU value matches: 0b%b", golden_out.lru_value);
    end
    
    if (pass) begin
        $display("  [PASS] ✓ Transaction completed successfully\n");
    end else begin
        $display("  [FAIL] ✗ Transaction failed - see errors above\n");
    end
    
    return pass;
endfunction

    // task directed_test();
    //      read_hit();
    //      write_hit();
    //      clean_read_miss();
    //      clean_write_miss();
    //      dirty_read_miss();
    //      dirty_write_miss();

    // endtask

task read_hit();
  input_transaction_t inp;
  output_transaction_t golden_out, dut_out;

  static logic [3:0] set   = 4'h0;
  static logic [22:0] tag  = 23'h12345;
  static logic [31:0] addr = {tag, set, 5'b0};
  static logic [255:0] line= 256'hDEADBEEF_CAFECAFE_12345678_ABCDEF01_0_0_0_0;

  // Prime both models (allocate line into DUT & golden)
  load_cacheline(addr, line,0);

  // Now do an aligned read hit
  inp = '{address: addr, transaction_type: READ,
          read_mask: 4'b1111, write_mask:'0, wdata:'0, dfp_rdata:'0};

  golden_out = golden_cache_do(inp);
  drive_dut(inp, dut_out);
  compare_outputs(golden_out, dut_out);
endtask



  initial begin: main 
    input_transaction_t inp;
    output_transaction_t golden_out, dut_out;
    logic [31:0] addr;
    logic [255:0] data;
    logic [3:0] set;
   logic [22:0] tag;
   logic [255:0] line;


    rst = 1'b1;
    ufp_addr = '0;
    ufp_rmask = '0;
    ufp_wmask = '0;
    ufp_wdata = '0;
    dfp_rdata = '0;
    dfp_resp = '0;

    // Initialize golden arrays
    for (int i = 0; i < 16; i++) begin  
        plru_golden_arrays[i] = '0;
        for (int j = 0; j < 4; j++) begin  
            data_golden_arrays[i][j] = '0;
            tag_golden_arrays[i][j] = '0;
            dirty_golden_arrays[i][j] = '0;
            valid_golden_arrays[i][j] = '0;
        end
    end
    
    repeat (5) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);


  for(int i = 0; i < 50000; i++) begin
        // $display("\n=== Starting transaction %0d ===", i);
        inp = generate_input_transaction();
        
        // $display("Generated transaction: addr=0x%h, type=%s, rmask=%b, wmask=%b", 
        //          inp.address, 
        //          inp.transaction_type == READ ? "READ" : "WRITE",
        //          inp.read_mask, 
        //          inp.write_mask);
        
        golden_out = golden_cache_do(inp);
        // $display("Golden expects: hit=%b, allocate=%b, writeback=%b", 
        //          golden_out.read_hit || golden_out.write_hit,
        //          golden_out.caused_allocate,
        //          golden_out.caused_writeback);
        
        drive_dut(inp, dut_out);
        $display("DUT completed transaction %0d", i);
        
        compare_outputs(golden_out, dut_out);
    end

        $display("All tests finished");
    $finish;
    end

endmodule : top_tb
