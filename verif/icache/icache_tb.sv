`timescale 1ns / 1ps

module icache_tb;

    // Parameters
    localparam LINE_WIDTH  = 128;
    localparam WORD_WIDTH  = 32;
    localparam ADDR_WIDTH  = 32;
    localparam NUM_WAYS    = 4;
    localparam NUM_SETS    = 64;

    // Clock and reset
    logic clk;
    logic reset;

    // CPU <-> cache
    logic [ADDR_WIDTH-1:0] cpu_addr;
    logic                  cpu_req;
    logic [WORD_WIDTH-1:0] cpu_inst;
    logic                  cpu_valid;

    // Cache <-> memory
    logic                  mem_req;
    logic [ADDR_WIDTH-1:0] mem_addr;
    logic                  mem_valid;
    logic [LINE_WIDTH-1:0] mem_inst;

    // Instantiate cache
    icache #(
        .LINE_WIDTH (LINE_WIDTH),
        .WORD_WIDTH (WORD_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH),
        .NUM_WAYS   (NUM_WAYS),
        .NUM_SETS   (NUM_SETS)
    ) dut (
        .clk         (clk),
        .reset       (reset),
        .cpu_addr_i  (cpu_addr),
        .cpu_req_i   (cpu_req),
        .cpu_inst_o  (cpu_inst),
        .cpu_valid_o (cpu_valid),
        .mem_req_o   (mem_req),
        .mem_addr_o  (mem_addr),
        .mem_valid_i (mem_valid),
        .mem_inst_i  (mem_inst)
    );

    // Clock generation
    always #5 clk = ~clk;

    // Fake memory (simple synchronous model)
    task send_memory_line(input logic [ADDR_WIDTH-1:0] addr);
        // Return a line containing 4 words:
        // Each word will be equal to base + offset
        mem_inst  = {
            addr + 12, addr + 8, addr + 4, addr
        }; // [word3, word2, word1, word0]
        mem_valid = 1;
        @(posedge clk);
        mem_valid = 0;
    endtask

    initial begin
        $display("Starting I-Cache testbench...");

        // Init
        clk   = 0;
        reset = 1;
        cpu_req = 0;
        mem_valid = 0;

        @(posedge clk);
        reset = 0;

        @(posedge clk);

        // 1. Access first address (miss)
        cpu_addr = 32'h0000_0040;
        cpu_req  = 1;
        @(posedge clk);
        cpu_req  = 0;

        wait (mem_req);
        $display("[TB] MISS at 0x%08h → memory request issued", cpu_addr);

        // Send back memory line
        send_memory_line(mem_addr);

        wait (cpu_valid);
        $display("[TB] CPU received: 0x%08h", cpu_inst);

        // 2. Access same line again (hit)
        cpu_addr = 32'h0000_004C; // same line, different word
        cpu_req  = 1;
        @(posedge clk);
        cpu_req  = 0;

        wait (cpu_valid);
        $display("[TB] HIT: CPU received: 0x%08h", cpu_inst);

        // 3. Access different set (another miss)
        cpu_addr = 32'h0000_0240; // different index
        cpu_req  = 1;
        @(posedge clk);
        cpu_req  = 0;

        wait (mem_req);
        $display("[TB] MISS at 0x%08h → memory request issued", cpu_addr);
        send_memory_line(mem_addr);
        wait (cpu_valid);
        $display("[TB] CPU received: 0x%08h", cpu_inst);

        $display("✅ Testbench finished.");
        $stop;
    end

endmodule
