// dcache.sv
//
// This module implements a 2KB, 4-way set-associative data cache.
// It now supports 32-bit word-aligned CPU read/write access.
//
// Parameters:
//   ADDR_WIDTH         : Width of the address bus (e.g., 32 for 32-bit addresses)
//   CACHE_LINE_BITS    : Width of a cache line in bits (128 bits)
//   WORD_BITS          : Width of a CPU word (32 bits)
//
// Cache Organization:
//   - Cache Size: 2KB
//   - Cache Line Size: 128 bits (16 bytes)
//   - Associativity: 4-way
//   - Write Policy: Write-back
//   - Write Allocate Policy: Write-allocate
//   - Replacement Policy: Pseudo-LRU (using 2-bit counters per way)
//
// CPU Interface (now supports 32-bit word-aligned data transfers):
//   - cpu_clk          : Clock
//   - cpu_rst_n        : Asynchronous reset (active low)
//   - cpu_req          : CPU request (1 for active, 0 for idle)
//   - cpu_we           : CPU write enable (1 for write, 0 for read)
//   - cpu_addr         : CPU address (word-aligned)
//   - cpu_wdata        : CPU write data (32-bit)
//   - cpu_rdata        : CPU read data (32-bit)
//   - cpu_ready        : Cache ready to accept new CPU request
//   - cpu_valid        : Cache has valid data for CPU (for reads)
//   - cpu_hit          : Indicates a cache hit
//   - cpu_miss         : Indicates a cache miss
//
// Main Memory Interface:
//   - mem_clk          : Clock (can be same as cpu_clk)
//   - mem_rst_n        : Asynchronous reset (active low)
//   - mem_req          : Cache request to main memory
//   - mem_we           : Cache write enable to main memory
//   - mem_addr         : Cache address to main memory (line-aligned)
//   - mem_wdata        : Cache write data to main memory (128-bit)
//   - mem_rdata        : Cache read data from main memory (128-bit)
//   - mem_ready        : Main memory ready to accept new request
//   - mem_valid        : Main memory has valid data (for reads)

module dcache #(
    parameter ADDR_WIDTH        = 32,
    parameter CACHE_LINE_BITS   = 128,
    parameter WORD_BITS         = 32
) (
    // CPU Interface
    input  logic                    cpu_clk,
    input  logic                    cpu_rst_n,
    input  logic                    cpu_req,
    input  logic                    cpu_we,
    input  logic [ADDR_WIDTH-1:0]   cpu_addr,
    input  logic [WORD_BITS-1:0]    cpu_wdata, // Changed to 32-bit
    output logic [WORD_BITS-1:0]    cpu_rdata, // Changed to 32-bit
    output logic                    cpu_ready,
    output logic                    cpu_valid,
    output logic                    cpu_hit,
    output logic                    cpu_miss,

    // Main Memory Interface
    input  logic                    mem_clk,
    input  logic                    mem_rst_n,
    output logic                    mem_req,
    output logic                    mem_we,
    output logic [ADDR_WIDTH-1:0]   mem_addr,
    output logic [CACHE_LINE_BITS-1:0] mem_wdata,
    input  logic [CACHE_LINE_BITS-1:0] mem_rdata,
    input  logic                    mem_ready,
    input  logic                    mem_valid
);

    // --- Cache Parameters (Derived) ---
    localparam CACHE_SIZE_BYTES     = 2048; // 2KB
    localparam CACHE_LINE_BYTES     = CACHE_LINE_BITS / 8; // 16 bytes
    localparam NUM_WAYS             = 4;
    localparam NUM_LINES            = CACHE_SIZE_BYTES / CACHE_LINE_BYTES; // 128 lines
    localparam NUM_SETS             = NUM_LINES / NUM_WAYS; // 32 sets (128 lines / 4 ways)

    localparam BYTES_PER_WORD       = WORD_BITS / 8; // 4 bytes/word
    localparam WORDS_PER_LINE       = CACHE_LINE_BYTES / BYTES_PER_WORD; // 16 bytes / 4 bytes = 4 words

    localparam BYTE_OFFSET_BITS     = $clog2(BYTES_PER_WORD); // 2 bits for byte within word
    localparam WORD_OFFSET_BITS     = $clog2(WORDS_PER_LINE); // 2 bits for word within line
    localparam OFFSET_BITS          = WORD_OFFSET_BITS + BYTE_OFFSET_BITS; // Total offset bits (4 bits)

    localparam INDEX_BITS           = $clog2(NUM_SETS);         // 5 bits
    localparam TAG_BITS             = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS; // 23 bits

    localparam LRU_COUNTER_MAX      = NUM_WAYS - 1; // For 4 ways, counters go 0, 1, 2, 3
    localparam LRU_COUNTER_BITS     = $clog2(NUM_WAYS); // 2 bits for 0-3

    // --- FSM States ---
    typedef enum logic [3:0] {
        IDLE,               // Waiting for a CPU request
        LOOKUP,             // Performing tag lookup and hit/miss detection
        WRITE_BACK,         // Writing a dirty evicted block to main memory
        WRITE_BACK_WAIT,    // Waiting for main memory to acknowledge write-back
        FETCH_LINE,         // Requesting a new line from main memory (for read/write miss)
        FETCH_WAIT,         // Waiting for main memory to return the requested line
        REFILL              // Writing the fetched line into the cache
    } cache_state_e;

    logic [3:0] current_state, next_state;

    // --- Cache Memory Structure ---
    // Defines the structure for a single cache way entry
    typedef struct packed {
        logic [CACHE_LINE_BITS-1:0]  data;
        logic [TAG_BITS-1:0]         tag;
        logic                        valid;
        logic                        dirty;
        logic [LRU_COUNTER_BITS-1:0] lru;
    } cache_way_entry_t;

    // Declare the cache memory as an array of cache_way_entry_t
    cache_way_entry_t cache_memory[NUM_SETS-1:0][NUM_WAYS-1:0];

    // --- Internal Registers for FSM State ---
    logic [ADDR_WIDTH-1:0]      req_addr_reg;
    logic                       req_we_reg;
    logic [WORD_BITS-1:0]       req_wdata_reg; // Changed to 32-bit
    logic [TAG_BITS-1:0]        req_tag;
    logic [INDEX_BITS-1:0]      req_idx;
    logic [WORD_OFFSET_BITS-1:0] req_word_offset; // New: for 32-bit word selection
    logic [BYTE_OFFSET_BITS-1:0] req_byte_offset_within_word; // New: for byte within word (not used for word-aligned access)

    logic [NUM_WAYS-1:0]        hit_way;        // One-hot encoding of the hit way
    logic                       is_hit;         // Overall hit signal
    logic [NUM_WAYS-1:0]        invalid_way;    // One-hot encoding of an available invalid way
    logic                       has_invalid_way; // Indicates if an invalid way exists
    logic [NUM_WAYS-1:0]        lru_way;        // One-hot encoding of the LRU way
    logic                       is_lru_dirty;   // Indicates if the LRU way is dirty

    logic [CACHE_LINE_BITS-1:0] evicted_data;   // Data to be written back to memory
    logic [ADDR_WIDTH-1:0]      evicted_addr;   // Address of data to be written back

    // --- Output Registers ---
    logic [WORD_BITS-1:0]       cpu_rdata_reg; // Changed to 32-bit
    logic                       cpu_ready_reg;
    logic                       cpu_valid_reg;
    logic                       cpu_hit_reg;
    logic                       cpu_miss_reg;

    logic                       mem_req_reg;
    logic                       mem_we_reg;
    logic [ADDR_WIDTH-1:0]      mem_addr_reg;
    logic [CACHE_LINE_BITS-1:0] mem_wdata_reg;

    // --- Assign outputs from registers ---
    assign cpu_rdata    = cpu_rdata_reg;
    assign cpu_ready    = cpu_ready_reg;
    assign cpu_valid    = cpu_valid_reg;
    assign cpu_hit      = cpu_hit_reg;
    assign cpu_miss     = cpu_miss_reg;

    assign mem_req      = mem_req_reg;
    assign mem_we       = mem_we_reg;
    assign mem_addr     = mem_addr_reg;
    assign mem_wdata    = mem_wdata_reg;

    // --- Address Decomposition ---
    assign req_tag                     = req_addr_reg[ADDR_WIDTH-1 -: TAG_BITS];
    assign req_idx                     = req_addr_reg[OFFSET_BITS +: INDEX_BITS];
    assign req_word_offset             = req_addr_reg[BYTE_OFFSET_BITS +: WORD_OFFSET_BITS];
    assign req_byte_offset_within_word = req_addr_reg[BYTE_OFFSET_BITS-1:0]; // Not explicitly used for word-aligned access

    // --- FSM State Register ---
    always_ff @(posedge cpu_clk or negedge cpu_rst_n) begin
        if (!cpu_rst_n) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    // --- Combinational Logic for Next State and Output Control ---
    always_comb begin
        // Default assignments (most common case, overridden in specific states)
        next_state          = current_state;
        cpu_ready_reg       = 1'b0; // Default to not ready
        cpu_valid_reg       = 1'b0; // Default to no valid data
        cpu_hit_reg         = 1'b0;
        cpu_miss_reg        = 1'b0;
        cpu_rdata_reg       = '0;

        mem_req_reg         = 1'b0; // Default to no memory request
        mem_we_reg          = 1'b0; // Default to read from memory
        mem_addr_reg        = '0;
        mem_wdata_reg       = '0;

        // Internal signals for next state logic
        logic [NUM_WAYS-1:0]    next_hit_way;
        logic                   next_is_hit;
        logic [NUM_WAYS-1:0]    next_invalid_way;
        logic                   next_has_invalid_way;
        logic [NUM_WAYS-1:0]    next_lru_way;
        logic                   next_is_lru_dirty;

        next_hit_way        = '0;
        next_is_hit         = 1'b0;
        next_invalid_way    = '0;
        next_has_invalid_way = 1'b0;
        next_lru_way        = '0;
        next_is_lru_dirty   = 1'b0;

        // --- Cache Lookup Logic (combinational) ---
        // Determine hit/miss and identify invalid/LRU ways for the current set
        for (int i = 0; i < NUM_WAYS; i++) begin
            if (cache_memory[req_idx][i].valid && (cache_memory[req_idx][i].tag == req_tag)) begin
                next_hit_way[i] = 1'b1;
                next_is_hit     = 1'b1;
            end

            if (!cache_memory[req_idx][i].valid) begin
                next_invalid_way[i] = 1'b1;
                next_has_invalid_way = 1'b1;
            end
        end

        // Find LRU way if no invalid way exists
        if (!next_has_invalid_way) begin
            logic [LRU_COUNTER_BITS-1:0] max_lru_val = 0;
            int lru_idx = 0;
            for (int i = 0; i < NUM_WAYS; i++) begin
                if (cache_memory[req_idx][i].lru > max_lru_val) begin
                    max_lru_val = cache_memory[req_idx][i].lru;
                    lru_idx = i;
                end
            end
            next_lru_way[lru_idx] = 1'b1;
            next_is_lru_dirty = cache_memory[req_idx][lru_idx].dirty;
        end

        hit_way             = next_hit_way;
        is_hit              = next_is_hit;
        invalid_way         = next_invalid_way;
        has_invalid_way     = next_has_invalid_way;
        lru_way             = next_lru_way;
        is_lru_dirty        = next_is_lru_dirty;

        case (current_state)
            IDLE: begin
                cpu_ready_reg = 1'b1; // Ready to accept new request
                if (cpu_req) begin
                    // Store CPU request details
                    req_addr_reg    = cpu_addr;
                    req_we_reg      = cpu_we;
                    req_wdata_reg   = cpu_wdata;
                    next_state      = LOOKUP;
                end else begin
                    next_state      = IDLE;
                end
            end

            LOOKUP: begin
                if (is_hit) begin
                    cpu_hit_reg = 1'b1;
                    cpu_miss_reg = 1'b0;
                    cpu_valid_reg = 1'b1; // Data is available on hit

                    // Read 32-bit word from cache on hit
                    for (int i = 0; i < NUM_WAYS; i++) begin
                        if (hit_way[i]) begin
                            // Select the correct 32-bit word from the 128-bit cache line
                            cpu_rdata_reg = cache_memory[req_idx][i].data[(req_word_offset * WORD_BITS) +: WORD_BITS];
                        end
                    end

                    next_state = IDLE; // Return to IDLE after a hit
                end else begin // Cache Miss
                    cpu_hit_reg = 1'b0;
                    cpu_miss_reg = 1'b1;

                    if (has_invalid_way) begin
                        // No eviction needed, directly fetch
                        // For write-allocate, even a write miss fetches the line
                        next_state = FETCH_LINE;
                    end else begin // All ways are valid, need eviction
                        if (is_lru_dirty) begin
                            // Evict dirty LRU block first
                            for (int i = 0; i < NUM_WAYS; i++) begin
                                if (lru_way[i]) begin
                                    evicted_data = cache_memory[req_idx][i].data;
                                    evicted_addr = {cache_memory[req_idx][i].tag, req_idx, {OFFSET_BITS{1'b0}}};
                                end
                            end
                            next_state = WRITE_BACK;
                        end else begin
                            // LRU block is clean, no write-back needed, directly fetch
                            next_state = FETCH_LINE;
                        end
                    end
                end
            end

            WRITE_BACK: begin
                mem_req_reg     = 1'b1;
                mem_we_reg      = 1'b1; // Write to memory
                mem_addr_reg    = evicted_addr;
                mem_wdata_reg   = evicted_data;
                next_state      = WRITE_BACK_WAIT;
            end

            WRITE_BACK_WAIT: begin
                if (mem_ready) begin
                    mem_req_reg = 1'b0; // De-assert memory request
                    // After write-back, proceed to fetch the new line
                    next_state = FETCH_LINE;
                end else begin
                    mem_req_reg = 1'b1; // Keep requesting until ready
                    mem_we_reg  = 1'b1;
                    mem_addr_reg = evicted_addr;
                    mem_wdata_reg = evicted_data;
                    next_state = WRITE_BACK_WAIT;
                end
            end

            FETCH_LINE: begin
                mem_req_reg     = 1'b1;
                mem_we_reg      = 1'b0; // Read from memory
                // Request full line address (offset bits are 0 for line-aligned)
                mem_addr_reg    = {req_tag, req_idx, {OFFSET_BITS{1'b0}}};
                next_state      = FETCH_WAIT;
            end

            FETCH_WAIT: begin
                if (mem_valid) begin
                    mem_req_reg = 1'b0; // De-assert memory request
                    next_state = REFILL;
                end else begin
                    mem_req_reg = 1'b1; // Keep requesting until valid
                    mem_we_reg  = 1'b0;
                    mem_addr_reg = {req_tag, req_idx, {OFFSET_BITS{1'b0}}};
                    next_state = FETCH_WAIT;
                end
            end

            REFILL: begin
                // Data is now available for CPU (either fetched or modified by write-allocate)
                cpu_valid_reg = 1'b1;
                // Provide the specific 32-bit word to CPU from the fetched line
                cpu_rdata_reg = mem_rdata[(req_word_offset * WORD_BITS) +: WORD_BITS];

                // Find the way to refill (invalid or LRU)
                int fill_way_idx = 0;
                if (has_invalid_way) begin
                    for (int i = 0; i < NUM_WAYS; i++) begin
                        if (invalid_way[i]) begin
                            fill_way_idx = i;
                            break; // Use the first invalid way found
                        end
                    end
                end else begin
                    for (int i = 0; i < NUM_WAYS; i++) begin
                        if (lru_way[i]) begin
                            fill_way_idx = i;
                            break;
                        end
                    end
                end

                // Update cache memory and LRU
                // These updates happen in the always_ff block
                // Mark the selected way for update
                // This logic is handled by the sequential block based on the state.
                next_state = IDLE;
            end

            default: begin
                next_state = IDLE; // Should not happen
            end
        endcase
    end

    // --- Sequential Logic for Cache Memory and LRU Updates ---
    always_ff @(posedge cpu_clk or negedge cpu_rst_n) begin
        if (!cpu_rst_n) begin
            // Initialize all cache entries to invalid on reset
            for (int i = 0; i < NUM_SETS; i++) begin
                for (int j = 0; j < NUM_WAYS; j++) begin
                    cache_memory[i][j].valid <= 1'b0;
                    cache_memory[i][j].dirty <= 1'b0;
                    cache_memory[i][j].lru   <= '0; // Reset LRU counters
                    cache_memory[i][j].data  <= '0; // Initialize data
                    cache_memory[i][j].tag   <= '0; // Initialize tag
                end
            end
        end else begin
            // Update cache contents and LRU based on current state and next state transitions
            if (current_state == LOOKUP && is_hit) begin
                // On a cache hit
                for (int i = 0; i < NUM_WAYS; i++) begin
                    if (hit_way[i]) begin
                        // Update LRU for the hit way
                        cache_memory[req_idx][i].lru <= 0; // Most recently used
                        // If it's a write hit, update data and set dirty bit
                        if (req_we_reg) begin
                            // Update only the specific 32-bit word within the 128-bit line
                            cache_memory[req_idx][i].data[(req_word_offset * WORD_BITS) +: WORD_BITS] <= req_wdata_reg;
                            cache_memory[req_idx][i].dirty <= 1'b1;
                        end
                    end else begin
                        // Increment LRU counters for other valid ways in the set
                        if (cache_memory[req_idx][i].valid && cache_memory[req_idx][i].lru < LRU_COUNTER_MAX) begin
                            cache_memory[req_idx][i].lru <= cache_memory[req_idx][i].lru + 1;
                        end
                    end
                end
            end else if (current_state == REFILL) begin
                // On a cache refill (after a miss)
                int fill_way_idx = 0;
                if (has_invalid_way) begin
                    for (int i = 0; i < NUM_WAYS; i++) begin
                        if (invalid_way[i]) begin
                            fill_way_idx = i;
                            break;
                        end
                    end
                end else begin
                    for (int i = 0; i < NUM_WAYS; i++) begin
                        if (lru_way[i]) begin
                            fill_way_idx = i;
                            break;
                        end
                    end
                end

                // Write new data into the selected way
                cache_memory[req_idx][fill_way_idx].tag   <= req_tag;
                cache_memory[req_idx][fill_way_idx].valid <= 1'b1;

                if (req_we_reg) begin // If the original request was a write (write-allocate)
                    // Merge the fetched line with the CPU's write data for the specific word
                    logic [CACHE_LINE_BITS-1:0] temp_line = mem_rdata;
                    temp_line[(req_word_offset * WORD_BITS) +: WORD_BITS] = req_wdata_reg;
                    cache_memory[req_idx][fill_way_idx].data  <= temp_line;
                    cache_memory[req_idx][fill_way_idx].dirty <= 1'b1; // Newly fetched line is now dirty
                end else begin // If the original request was a read
                    cache_memory[req_idx][fill_way_idx].data  <= mem_rdata;
                    cache_memory[req_idx][fill_way_idx].dirty <= 1'b0; // Newly fetched line is clean
                end

                // Update LRU for the newly filled way
                cache_memory[req_idx][fill_way_idx].lru <= 0; // Most recently used
                for (int i = 0; i < NUM_WAYS; i++) begin
                    if (i != fill_way_idx && cache_memory[req_idx][i].valid && cache_memory[req_idx][i].lru < LRU_COUNTER_MAX) begin
                        cache_memory[req_idx][i].lru <= cache_memory[req_idx][i].lru + 1;
                    end
                end
            end
        end
    end

endmodule