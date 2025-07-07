module icache #(
    parameter int LINE_WIDTH = 128,
    parameter int WORD_WIDTH = 32,
    parameter int NUM_WAYS   = 4,
    parameter int NUM_SETS   = 64,
    parameter int ADDR_WIDTH = 32
) (
    input   logic                   clk,
    input   logic                   rst_n,

    // CPU interface
    input   logic [ADDR_WIDTH-1:0]  cpu_addr_i,
    output  logic [WORD_WIDTH-1:0]  cpu_inst_o,
    output  logic                   cpu_valid_o,
    input   logic                   cpu_req_i,

    // MEM interface
    output  logic                   mem_req_o,
    output  logic [ADDR_WIDTH-1:0]  mem_addr_o,
    input   logic                   mem_valid_i,
    input   logic [WORD_WIDTH-1:0]  mem_inst_i
);

    localparam int WORD_OFFSET_BITS = $clog2(LINE_WIDTH / WORD_WIDTH);
    localparam int INDEX_BITS       = $clog2(NUM_SETS);
    localparam int TAG_BITS         = ADDR_WIDTH - INDEX_BITS - WORD_OFFSET_BITS - 2;

    typedef struct packed {
        logic                   valid;
        logic [TAG_BITS-1:0]    tag;
        logic [LINE_WIDTH-1:0]  inst;
    } cache_line_t;
    
    typedef enum logic [1:0] {
        IDLE,
        MISS,
        REFILL
    } state_t;

    state_t state, next_state;

    logic [INDEX_BITS-1:0]          index;
    logic [TAG_BITS-1:0]            tag;
    logic [WORD_OFFSET_BITS-1:0]    word_offset;
    logic [1:0]                     lru [NUM_SETS];
    cache_line_t                    cache [NUM_SETS][NUM_WAYS];

    // assign index, tag, word_offset
    assign index       = cpu_addr_i[2 + WORD_OFFSET_BITS +: INDEX_BITS];
    assign tag         = cpu_addr_i[ADDR_WIDTH-1 -: TAG_BITS];
    assign word_offset = cpu_addr_i[2 +: WORD_OFFSET_BITS];

    // hit and tag compare
    logic                   hit;
    logic [1:0]             hit_way;
    logic [WORD_WIDTH-1:0]  sel_word;

    always_comb begin
        hit     = '0;
        hit_way = '0;
        for (int way = 0; way < NUM_WAYS; way++) begin
            if(cache[index][way].valid && cache[index][way].tag == tag) begin
                hit     = 1;
                hit_way = way[1:0];
            end
        end
    end

    assign sel_word = cache[index][hit_way].inst[word_offset*WORD_WIDTH +: WORD_WIDTH];

    // refill and lru update
    always_ff @(posedge clk) begin
        if (state == REFILL && mem_valid_i) begin
            int replace_way = lru[index]; // LRU decision
            cache[index][replace_way].valid <= 1;
            cache[index][replace_way].tag   <= tag;
            cache[index][replace_way].data  <= mem_data_i;
            lru[index] <= (lru[index] + 1) % NUM_WAYS; // simple round-robin LRU
        end
    end

    // FSM 
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            state <= IDLE;
        else
            state <= next_state;
    end

    always_comb begin
        next_state = state;
        case (state)
            IDLE: begin
                if (cpu_req_i && !hit)
                    next_state = MISS;
            end
            MISS: begin
                if (mem_valid_i)
                    next_state = REFILL;
            end
            REFILL: begin
                next_state = IDLE;
            end
        endcase
    end

    //output
    assign cpu_inst_o  = (hit) ? sel_word : '0;
    assign cpu_valid_o = (state == IDLE && hit);
    assign mem_req_o   = (state == MISS);
    assign mem_addr_o  = {cpu_addr_i[ADDR_WIDTH-1:WORD_OFFSET_BITS+2], {WORD_OFFSET_BITS+2{1'b0}}};