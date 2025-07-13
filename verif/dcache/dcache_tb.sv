`timescale 1ns / 1ps // Define timescale for simulation

module tb_dcache;

    // Parameters - Must match the dcache module exactly
    parameter ADDR_WIDTH        = 32;
    parameter CACHE_LINE_BITS   = 128; // 16 bytes per cache line
    parameter WORD_BITS         = 32;  // 4 bytes per CPU word

    // Derived parameters for testbench clarity
    localparam CACHE_LINE_BYTES = CACHE_LINE_BITS / 8; // 16 bytes
    localparam BYTES_PER_WORD   = WORD_BITS / 8;       // 4 bytes

    // CPU Interface Signals - Wires connecting to the D-Cache inputs/outputs
    logic                       cpu_clk;
    logic                       cpu_rst_n;
    logic                       cpu_req;
    logic                       cpu_we;
    logic [ADDR_WIDTH-1:0]      cpu_addr;
    logic [WORD_BITS-1:0]       cpu_wdata;
    logic [WORD_BITS-1:0]       cpu_rdata;
    logic                       cpu_ready;
    logic                       cpu_valid;
    logic                       cpu_hit;
    logic                       cpu_miss;

    // Main Memory Interface Signals - Wires connecting to the D-Cache inputs/outputs
    logic                       mem_clk;
    logic                       mem_rst_n;
    logic                       mem_req;
    logic                       mem_we;
    logic [ADDR_WIDTH-1:0]      mem_addr;
    logic [CACHE_LINE_BITS-1:0] mem_wdata;
    logic [CACHE_LINE_BITS-1:0] mem_rdata;
    logic                       mem_ready;
    logic                       mem_valid;

    // Internal testbench signals for simple Main Memory Model
    localparam MEM_SIZE_LINES = 256; // Defines memory as 256 cache lines (256 * 16 bytes = 4KB)
    localparam ADDR_IDX10_TAG0_WORD0 = {23'h0000000, 5'hA, 4'h0}; // Address 0x0000_00A0
    localparam ADDR_IDX10_TAG1_WORD0 = {23'h0000001, 5'hA, 4'h0}; // Address 0x0000_02A0
    localparam ADDR_IDX10_TAG2_WORD0 = {23'h0000002, 5'hA, 4'h0}; // Address 0x0000_04A0
    localparam ADDR_IDX10_TAG3_WORD0 = {23'h0000003, 5'hA, 4'h0}; // Address 0x0000_06A0
    localparam ADDR_IDX10_TAG4_WORD0 = {23'h0000004, 5'hA, 4'h0}; // Address 0x0000_08A0 (for eviction test)
    localparam WRITE_DATA_TAG0_WORD1 = 32'hFACE_CAFE;
    logic [WORD_BITS-1:0] read_data; // Variable to store data read from cache
    logic is_hit, is_miss;         // Variables to store hit/miss status
    logic [CACHE_LINE_BITS-1:0] line_data_from_mem; // To read entire cache lines from main memory for verification

    localparam ADDR_IDX11_TAG0_WORD0 = {23'h0000000, 5'h11, 4'h0}; // Index 17 (0x11), Tag 0
    localparam ADDR_IDX11_TAG1_WORD0 = {23'h0000001, 5'h11, 4'h0};
    localparam ADDR_IDX11_TAG2_WORD0 = {23'h0000002, 5'h11, 4'h0};
    localparam ADDR_IDX11_TAG3_WORD0 = {23'h0000003, 5'h11, 4'h0};
    localparam ADDR_IDX11_TAG4_WORD0 = {23'h0000004, 5'h11, 4'h0}; // For eviction
    localparam WRITE_DATA_TAG4_WORD1 = 32'hDEAD_BEEF;

    // This array stores the content of our simulated main memory.
    logic [CACHE_LINE_BITS-1:0] main_memory[MEM_SIZE_LINES-1:0];

    // Main Memory Model's internal registers for output signals
    logic mem_ready_reg;
    logic mem_valid_reg;
    logic [CACHE_LINE_BITS-1:0] mem_rdata_reg;

    // Assign internal registers to the actual memory interface outputs
    assign mem_ready = mem_ready_reg;
    assign mem_valid = mem_valid_reg;
    assign mem_rdata = mem_rdata_reg;

    // Memory access latency in CPU clock cycles
    parameter MEM_LATENCY_CPU_CYCLES = 5;

    // Instantiate the D-Cache module under test
    dcache #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .CACHE_LINE_BITS(CACHE_LINE_BITS),
        .WORD_BITS(WORD_BITS)
    ) dut (
        .cpu_clk(cpu_clk),
        .cpu_rst_n(cpu_rst_n),
        .cpu_req(cpu_req),
        .cpu_we(cpu_we),
        .cpu_addr(cpu_addr),
        .cpu_wdata(cpu_wdata),
        .cpu_rdata(cpu_rdata),
        .cpu_ready(cpu_ready),
        .cpu_valid(cpu_valid),
        .cpu_hit(cpu_hit),
        .cpu_miss(cpu_miss),

        .mem_clk(mem_clk),
        .mem_rst_n(mem_rst_n),
        .mem_req(mem_req),
        .mem_we(mem_we),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_rdata(mem_rdata),
        .mem_ready(mem_ready),
        .mem_valid(mem_valid)
    );

    // --- Clock Generation ---
    parameter CLK_PERIOD = 10ns; // 100 MHz clock
    initial begin
        cpu_clk = 0;
        mem_clk = 0; // Assuming mem_clk is synchronous with cpu_clk for simplicity
        forever #(CLK_PERIOD / 2) cpu_clk = ~cpu_clk;
    end
    initial begin
        forever #(CLK_PERIOD / 2) mem_clk = ~mem_clk;
    end

    // --- VCD Waveform Dumping ---
    initial begin
        $dumpfile("dcache.vcd"); // Specify VCD file name
        $dumpvars(0, tb_dcache); // Dump all variables in the testbench
    end

    // --- Simple Main Memory Model ---
    // Internal state and registers for the memory model
    typedef enum {MEM_IDLE, MEM_ACCESS} mem_state_e;
    mem_state_e mem_current_state, mem_next_state;
    int mem_latency_counter;
    logic [ADDR_WIDTH-1:0]      mem_access_addr_reg; // Stores address for ongoing access
    logic                       mem_access_we_reg;   // Stores write enable for ongoing access
    logic [CACHE_LINE_BITS-1:0] mem_access_wdata_reg; // Stores write data for ongoing access

    // Main memory sequential logic (updates registers on clock edge)
    always_ff @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            // Reset state
            mem_current_state <= MEM_IDLE;
            mem_valid_reg <= 1'b0;
            mem_latency_counter <= 0;
            mem_rdata_reg <= '0;
            mem_access_addr_reg <= '0;
            mem_access_we_reg <= 1'b0;
            mem_access_wdata_reg <= '0;
        end else begin
            mem_current_state <= mem_next_state; // State transition

            // Decrement latency counter during access
            if (mem_current_state == MEM_ACCESS) begin
                if (mem_latency_counter > 0) begin
                    mem_latency_counter <= mem_latency_counter - 1;
                end
            end
            mem_valid_reg <= 1'b0; // Default: de-assert valid every cycle

            // When memory access completes (latency_counter becomes 0)
            if (mem_current_state == MEM_ACCESS && mem_latency_counter == 0) begin
                if (!mem_access_we_reg) begin // If it was a read operation
                    mem_rdata_reg <= main_memory[mem_access_addr_reg / CACHE_LINE_BYTES]; // Provide data
                    mem_valid_reg <= 1'b1; // Assert valid for one cycle
                    $display($time, " MEM: Read data 0x%h for addr 0x%h is VALID.", mem_rdata_reg, mem_access_addr_reg);
                end
                // For writes, data was already written in the combinational block when the request was received.
            end
        end
    end

    // Main memory combinational logic (determines next state and immediate outputs)
    always_comb begin
        mem_next_state = mem_current_state; // Default: stay in current state
        mem_ready_reg = 1'b0; // Default: not ready

        case (mem_current_state)
            MEM_IDLE: begin
                mem_ready_reg = 1'b1; // Memory is ready to accept a new request
                if (mem_req) begin // If D-Cache requests memory access
                    mem_next_state = MEM_ACCESS; // Transition to ACCESS state
                    // Initialize latency counter (subtract 1 because it decrements on next cycle)
                    mem_latency_counter = MEM_LATENCY_CPU_CYCLES - 1;
                    // Store request details for processing after latency
                    mem_access_addr_reg = mem_addr;
                    mem_access_we_reg = mem_we;
                    mem_access_wdata_reg = mem_wdata;

                    if (mem_we) begin // If it's a write operation
                        // Data is written immediately on request in this model
                        main_memory[mem_addr / CACHE_LINE_BYTES] = mem_wdata;
                        $display($time, " MEM: Write request received for addr 0x%h, data 0x%h. Latency: %0d cycles.", mem_addr, mem_wdata, MEM_LATENCY_CPU_CYCLES);
                    end else begin // If it's a read operation
                        $display($time, " MEM: Read request received for addr 0x%h. Latency: %0d cycles.", mem_addr, MEM_LATENCY_CPU_CYCLES);
                    end
                end
            end
            MEM_ACCESS: begin
                if (mem_latency_counter == 0) begin
                    mem_next_state = MEM_IDLE; // Go back to idle when latency finishes
                end else begin
                    mem_next_state = MEM_ACCESS; // Stay in ACCESS during latency
                end
            end
        endcase
    end


    // --- Task to send CPU request and wait for cache response ---
    task automatic send_cpu_req(input logic we, input [ADDR_WIDTH-1:0] addr,
                                input [WORD_BITS-1:0] wdata,
                                output logic [WORD_BITS-1:0] rdata,
                                output logic hit, output logic miss);
        @(posedge cpu_clk); // Wait for positive clock edge

        // Wait until the cache is ready to accept a new request
        while (!cpu_ready) begin
            $display($time, " CPU: Waiting for cache to be ready...");
            @(posedge cpu_clk);
        end

        // Assert CPU request signals
        cpu_req   = 1'b1;
        cpu_we    = we;
        cpu_addr  = addr;
        cpu_wdata = wdata;
        $display($time, " CPU: Sending %s request for addr 0x%h, wdata 0x%h", (we ? "WRITE" : "READ"), addr, wdata);

        @(posedge cpu_clk); // Wait one clock cycle
        cpu_req = 1'b0;     // De-assert cpu_req after one cycle

        // Wait for the cache to provide a response (cpu_valid for read data or cpu_hit/miss status)
        while (!(cpu_valid || cpu_hit || cpu_miss)) begin
            @(posedge cpu_clk);
        end

        // Capture output results
        rdata = cpu_rdata;
        hit   = cpu_hit;
        miss  = cpu_miss;
        $display($time, " CPU: Cache response: hit=%b, miss=%b, valid=%b, rdata=0x%h", cpu_hit, cpu_miss, cpu_valid, cpu_rdata);

        @(posedge cpu_clk); // Wait one more cycle for the cache to settle
    endtask

    // --- Main Test Sequence ---
    initial begin
        // Initialize CPU interface signals
        cpu_req     = 1'b0;
        cpu_we      = 1'b0;
        cpu_addr    = '0;
        cpu_wdata   = '0;

        // Initialize Main Memory with known patterns for testing
        // We'll focus tests on a single cache set (e.g., Index 0xA, decimal 10)
        // to thoroughly test associativity and LRU policy.
        // Cache organization: Tag[31:9], Index[8:4], Word_Offset[3:2], Byte_Offset[1:0]
        // Index 0xA means bits [8:4] are 5'b01010.

        // Base addresses for Index 0xA with different Tags (word-aligned, word offset 0)

        // Initialize main_memory lines corresponding to these addresses
        // Each line is 128 bits (4 words). Words are in order: [Word3, Word2, Word1, Word0]
        // Example: {32'hDDDD_DDDD, 32'hCCCC_CCCC, 32'hBBBB_BBBB, 32'hAAAA_AAAA}
        // Word 0 is at offset 0, Word 1 at offset 4, etc.
        main_memory[ADDR_IDX10_TAG0_WORD0 / CACHE_LINE_BYTES] = {32'hDDDD_DDDD, 32'hCCCC_CCCC, 32'hBBBB_BBBB, 32'hAAAA_AAAA};
        main_memory[ADDR_IDX10_TAG1_WORD0 / CACHE_LINE_BYTES] = {32'hEEEE_EEEE, 32'hFFFF_FFFF, 32'h0000_0000, 32'h1111_1111};
        main_memory[ADDR_IDX10_TAG2_WORD0 / CACHE_LINE_BYTES] = {32'h6666_6666, 32'h7777_7777, 32'h8888_8888, 32'h5555_5555};
        main_memory[ADDR_IDX10_TAG3_WORD0 / CACHE_LINE_BYTES] = {32'hAAAA_AAAA, 32'hBBBB_BBBB, 32'hCCCC_CCCC, 32'h9999_9999};
        main_memory[ADDR_IDX10_TAG4_WORD0 / CACHE_LINE_BYTES] = {32'h1010_1010, 32'h2020_2020, 32'h3030_3030, 32'hCAFE_CAFE};

        // Initialize other areas of main memory if needed, or with a default pattern
        for (int i = 0; i < MEM_SIZE_LINES; i++) begin
            if (i != (ADDR_IDX10_TAG0_WORD0 / CACHE_LINE_BYTES) &&
                i != (ADDR_IDX10_TAG1_WORD0 / CACHE_LINE_BYTES) &&
                i != (ADDR_IDX10_TAG2_WORD0 / CACHE_LINE_BYTES) &&
                i != (ADDR_IDX10_TAG3_WORD0 / CACHE_LINE_BYTES) &&
                i != (ADDR_IDX10_TAG4_WORD0 / CACHE_LINE_BYTES)) begin
                main_memory[i] = {128'hDEAD_BEEF_DEAD_BEEF_DEAD_BEEF_DEAD_BEEF + i};
            end
        end


        // --- Perform Reset Sequence ---
        cpu_rst_n = 1'b0;
        mem_rst_n = 1'b0;
        #(CLK_PERIOD * 2); // Hold reset for 2 clock cycles
        cpu_rst_n = 1'b1;
        mem_rst_n = 1'b1;
        $display($time, " Reset released. Starting test...");

        
        // --- Test 1: Initial Read Miss (Tag 0, Index 10, Word 0) ---
        // Expect: Cache miss, fetches line from main memory, provides data.
        $display("\n--- Test 1: Initial Read Miss (Addr 0x%h) ---", ADDR_IDX10_TAG0_WORD0);
        send_cpu_req(0, ADDR_IDX10_TAG0_WORD0, '0, read_data, is_hit, is_miss);
        assert (is_miss) else $error("Test 1 Failed: Expected read miss.");
        assert (read_data == 32'hAAAA_AAAA) else $error("Test 1 Failed: Incorrect data. Got 0x%h, Expected 0xAAAA_AAAA", read_data);
        $display($time, " Test 1: Data matches expected value.");

        // --- Test 2: Read Hit (Tag 0, Index 10, Word 0) ---
        // Expect: Cache hit, provides data immediately.
        $display("\n--- Test 2: Read Hit (Addr 0x%h) ---", ADDR_IDX10_TAG0_WORD0);
        send_cpu_req(0, ADDR_IDX10_TAG0_WORD0, '0, read_data, is_hit, is_miss);
        assert (is_hit) else $error("Test 2 Failed: Expected read hit.");
        assert (read_data == 32'hAAAA_AAAA) else $error("Test 2 Failed: Incorrect data. Got 0x%h, Expected 0xAAAA_AAAA", read_data);
        $display($time, " Test 2: Data matches expected value.");

        // --- Test 3: Read Hit (Tag 0, Index 10, Word 1) ---
        // Expect: Cache hit, reads a different word from the same cached line.
        $display("\n--- Test 3: Read Hit (Addr 0x%h, Word 1) ---", ADDR_IDX10_TAG0_WORD0 + BYTES_PER_WORD);
        send_cpu_req(0, ADDR_IDX10_TAG0_WORD0 + BYTES_PER_WORD, '0, read_data, is_hit, is_miss);
        assert (is_hit) else $error("Test 3 Failed: Expected read hit.");
        assert (read_data == 32'hBBBB_BBBB) else $error("Test 3 Failed: Incorrect data. Got 0x%h, Expected 0xBBBB_BBBB", read_data);
        $display($time, " Test 3: Data matches expected value.");

        // --- Test 4: Write Hit (Tag 0, Index 10, Word 1) ---
        // Expect: Cache hit, updates cache line, sets dirty bit.
        $display("\n--- Test 4: Write Hit (Addr 0x%h, Word 1) ---", ADDR_IDX10_TAG0_WORD0 + BYTES_PER_WORD);
        send_cpu_req(1, ADDR_IDX10_TAG0_WORD0 + BYTES_PER_WORD, WRITE_DATA_TAG0_WORD1, read_data, is_hit, is_miss);
        assert (is_hit) else $error("Test 4 Failed: Expected write hit.");
        $display($time, " Test 4: Write hit successful.");

        // --- Test 5: Read back written data (Tag 0, Index 10, Word 1) ---
        // Expect: Cache hit, reads the recently written data.
        $display("\n--- Test 5: Read back written data (Addr 0x%h, Word 1) ---", ADDR_IDX10_TAG0_WORD0 + BYTES_PER_WORD);
        send_cpu_req(0, ADDR_IDX10_TAG0_WORD0 + BYTES_PER_WORD, '0, read_data, is_hit, is_miss);
        assert (is_hit) else $error("Test 5 Failed: Expected read hit.");
        assert (read_data == WRITE_DATA_TAG0_WORD1) else $error("Test 5 Failed: Incorrect data after write. Got 0x%h, Expected 0x%h", read_data, WRITE_DATA_TAG0_WORD1);
        $display($time, " Test 5: Data matches expected value.");

        // At this point, the cache line for Tag 0, Index 10 is in the cache and is DIRTY.

        // --- Test 6-8: Fill Ways 1-3 of Set 10 with new lines (Read Misses) ---
        // These will be clean fills, forcing LRU updates.
        $display("\n--- Test 6: Read Miss (Tag 1, Index 10, Word 0) ---", ADDR_IDX10_TAG1_WORD0);
        send_cpu_req(0, ADDR_IDX10_TAG1_WORD0, '0, read_data, is_hit, is_miss);
        assert (is_miss) else $error("Test 6 Failed: Expected read miss.");
        assert (read_data == 32'h1111_1111) else $error("Test 6 Failed: Incorrect data. Got 0x%h, Expected 0x1111_1111", read_data);

        $display("\n--- Test 7: Read Miss (Tag 2, Index 10, Word 0) ---", ADDR_IDX10_TAG2_WORD0);
        send_cpu_req(0, ADDR_IDX10_TAG2_WORD0, '0, read_data, is_hit, is_miss);
        assert (is_miss) else $error("Test 7 Failed: Expected read miss.");
        assert (read_data == 32'h5555_5555) else $error("Test 7 Failed: Incorrect data. Got 0x%h, Expected 0x5555_5555", read_data);

        $display("\n--- Test 8: Read Miss (Tag 3, Index 10, Word 0) ---", ADDR_IDX10_TAG3_WORD0);
        send_cpu_req(0, ADDR_IDX10_TAG3_WORD0, '0, read_data, is_hit, is_miss);
        assert (is_miss) else $error("Test 8 Failed: Expected read miss.");
        assert (read_data == 32'h9999_9999) else $error("Test 8 Failed: Incorrect data. Got 0x%h, Expected 0x9999_9999", read_data);

        // At this point, all 4 ways of Set 10 are filled.
        // Based on pseudo-LRU, Tag 0 line (which was accessed first and made dirty) should now be the LRU candidate.

        // --- Test 9: Read Miss forcing Dirty Eviction (Tag 4, Index 10, Word 0) ---
        // Expect: Cache miss. The dirty LRU line (Tag 0) should be written back to main memory,
        // then the new line (Tag 4) should be fetched and placed in cache.
        $display("\n--- Test 9: Read Miss (forcing dirty eviction for Addr 0x%h) ---", ADDR_IDX10_TAG4_WORD0);
        send_cpu_req(0, ADDR_IDX10_TAG4_WORD0, '0, read_data, is_hit, is_miss);
        assert (is_miss) else $error("Test 9 Failed: Expected read miss.");
        assert (read_data == 32'hCAFE_CAFE) else $error("Test 9 Failed: Incorrect data. Got 0x%h, Expected 0xCAFE_CAFE", read_data);

        // Verify that the dirty line (Tag 0, Index 10) was correctly written back to main memory.
        // Original main_memory[ADDR_IDX10_TAG0_WORD0 / CACHE_LINE_BYTES] was {DDDD_DDDD, CCCC_CCCC, BBBB_BBBB, AAAA_AAAA}.
        // After write hit (Test 4), Word 1 (BBBB_BBBB) was changed to FACE_CAFE.
        // So, main memory should now reflect: {DDDD_DDDD, CCCC_CCCC, FACE_CAFE, AAAA_AAAA}.
        line_data_from_mem = main_memory[ADDR_IDX10_TAG0_WORD0 / CACHE_LINE_BYTES];
        $display($time, " Main Memory Content for original Tag 0 line (addr 0x%h): 0x%h", ADDR_IDX10_TAG0_WORD0, line_data_from_mem);
        assert (line_data_from_mem[63:32] == WRITE_DATA_TAG0_WORD1) else
            $error("Test 9 Failed: Dirty evicted data not written back correctly. Expected word 1 0x%h, Got 0x%h", WRITE_DATA_TAG0_WORD1, line_data_from_mem[63:32]);
        $display($time, " Test 9: Dirty eviction successful. Main memory updated correctly.");

        // --- Test 10-14: Write Miss forcing Clean Eviction (using a different index for simplicity) ---
        // This sequence fills another set (Index 0x11, decimal 17) with clean lines.
        // Then, a write miss forces the eviction of a clean line (no write-back),
        // and the new data is merged during refill.


        // Initialize main memory for this new set
        main_memory[ADDR_IDX11_TAG0_WORD0 / CACHE_LINE_BYTES] = {32'h1000_0000, 32'h2000_0000, 32'h3000_0000, 32'h4000_0000};
        main_memory[ADDR_IDX11_TAG1_WORD0 / CACHE_LINE_BYTES] = {32'h5000_0000, 32'h6000_0000, 32'h7000_0000, 32'h8000_0000};
        main_memory[ADDR_IDX11_TAG2_WORD0 / CACHE_LINE_BYTES] = {32'h9000_0000, 32'hA000_0000, 32'hB000_0000, 32'hC000_0000};
        main_memory[ADDR_IDX11_TAG3_WORD0 / CACHE_LINE_BYTES] = {32'hD000_0000, 32'hE000_0000, 32'hF000_0000, 32'h1234_5678};
        main_memory[ADDR_IDX11_TAG4_WORD0 / CACHE_LINE_BYTES] = {32'hFADE_FADE, 32'hDEAD_BEEF, 32'hCAFE_CAFE, 32'hBEEF_CAFE};


        $display("\n--- Test 10: Read Miss (Tag 0, Index 17, Word 0) to fill first way ---");
        send_cpu_req(0, ADDR_IDX11_TAG0_WORD0, '0, read_data, is_hit, is_miss);
        assert (is_miss) else $error("Test 10 Failed: Expected read miss.");
        assert (read_data == 32'h4000_0000) else $error("Test 10 Failed: Incorrect data. Got 0x%h, Expected 0x4000_0000", read_data);

        $display("\n--- Test 11: Read Miss (Tag 1, Index 17, Word 0) ---");
        send_cpu_req(0, ADDR_IDX11_TAG1_WORD0, '0, read_data, is_hit, is_miss);
        assert (is_miss) else $error("Test 11 Failed: Expected read miss.");
        assert (read_data == 32'h8000_0000) else $error("Test 11 Failed: Incorrect data. Got 0x%h, Expected 0x8000_0000", read_data);

        $display("\n--- Test 12: Read Miss (Tag 2, Index 17, Word 0) ---");
        send_cpu_req(0, ADDR_IDX11_TAG2_WORD0, '0, read_data, is_hit, is_miss);
        assert (is_miss) else $error("Test 12 Failed: Expected read miss.");
        assert (read_data == 32'hC000_0000) else $error("Test 12 Failed: Incorrect data. Got 0x%h, Expected 0xC000_0000", read_data);

        $display("\n--- Test 13: Read Miss (Tag 3, Index 17, Word 0) ---");
        send_cpu_req(0, ADDR_IDX11_TAG3_WORD0, '0, read_data, is_hit, is_miss);
        assert (is_miss) else $error("Test 13 Failed: Expected read miss.");
        assert (read_data == 32'h1234_5678) else $error("Test 13 Failed: Incorrect data. Got 0x%h, Expected 0x1234_5678", read_data);

        // At this point, Set 17 is full with clean lines. Tag 0 should be the LRU.
        $display("\n--- Test 14: Write Miss forcing Clean Eviction (Tag 4, Index 17, Word 1) ---");

        send_cpu_req(1, ADDR_IDX11_TAG4_WORD0 + BYTES_PER_WORD, WRITE_DATA_TAG4_WORD1, read_data, is_hit, is_miss);
        assert (is_miss) else $error("Test 14 Failed: Expected write miss.");

        // Verify that the data read back is the merged data from the fetched line and CPU write.
        assert (read_data == WRITE_DATA_TAG4_WORD1) else $error("Test 14 Failed: Incorrect data after write-allocate. Got 0x%h, Expected 0x%h", read_data, WRITE_DATA_TAG4_WORD1);

        // Verify main memory for Tag 0, Index 17 is still original (no write-back for clean eviction).
        line_data_from_mem = main_memory[ADDR_IDX11_TAG0_WORD0 / CACHE_LINE_BYTES];
        $display($time, " Main Memory Content for original Tag 0 line (addr 0x%h): 0x%h", ADDR_IDX11_TAG0_WORD0, line_data_from_mem);
        assert (line_data_from_mem[31:0] == 32'h4000_0000) else // Word 0
            $error("Test 14 Failed: Clean evicted data incorrectly written back. Expected 0x4000_0000, Got 0x%h", line_data_from_mem[31:0]);
        assert (line_data_from_mem[63:32] == 32'h3000_0000) else // Word 1
            $error("Test 14 Failed: Clean evicted data incorrectly written back. Expected 0x3000_0000, Got 0x%h", line_data_from_mem[63:32]);
        $display($time, " Test 14: Clean eviction successful. Main memory unchanged as expected.");

        // --- End of Simulation ---
        $display("\n--- All Tests Completed Successfully ---");
        $finish; // Terminate simulation
    end

endmodule