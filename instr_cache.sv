`timescale 1ns / 1ps

module instr_cache import params_pkg::*; #(
    parameter int ADDR_WIDTH  = params_pkg::ADDR_WIDTH,
    parameter int PADDR_WIDTH = params_pkg::PADDR_WIDTH,
    parameter int LINE_BYTES  = params_pkg::CACHE_LINE_BYTES, // cache line byte
    parameter int N_LINES     = params_pkg::ICACHE_N_LINES    // line number
) (
    input  logic                      clk,
    input  logic                      rstn,

    input  logic                      state_reset,
    input  logic                      cpu_req,       // CPU req came
    input  logic [PADDR_WIDTH-1:0]    cpu_addr,      // Byte adres
    input  logic [1:0]                cpu_size,      // 00=byte, 01=half, 10=word
    output logic                      cpu_ready,     // cache ready for another req
    output logic [ADDR_WIDTH-1:0]     cpu_rdata,     // read data
    output logic                      cpu_rvalid,    // read valid result
    output logic                      curr_cache_hit,

    output logic                      mem_req,       // mem req
    output logic [PADDR_WIDTH-1:0]    mem_addr,      // Line-aligned address
    input  logic                      mem_gnt,       // Request granted (accepted by arbiter)
    input  logic                      mem_rvalid,    // mem read
    input  logic [LINE_BYTES*8-1:0]   mem_rdata      // full line
);

    localparam OFFSET_BITS    = $clog2(LINE_BYTES);
    localparam IDX_BITS       = $clog2(N_LINES);
    localparam TAG_BITS       = PADDR_WIDTH - OFFSET_BITS - IDX_BITS;
    localparam WORDS_PER_LINE = LINE_BYTES / 4;
    localparam WORD_OFF_BITS  = $clog2(WORDS_PER_LINE);

    localparam SIZE_BYTE = 2'b00;
    localparam SIZE_HALF = 2'b01;
    localparam SIZE_WORD = 2'b10;

    typedef enum logic {
        S_IDLE       = 1'b0,
        S_REFILL     = 1'b1    //  fetch new line from memory
    } state_t;

    logic [TAG_BITS-1:0]        tag_array   [N_LINES];
    logic                       valid_array [N_LINES];
    logic [LINE_BYTES*8-1:0]    data_array  [N_LINES];

    typedef struct packed {
        logic                   valid;
        logic [PADDR_WIDTH-1:0] addr;
        logic [1:0]             size;
    } pending_req_t;

    pending_req_t pending;

    logic [IDX_BITS-1:0]      curr_index;
    logic [TAG_BITS-1:0]      curr_tag;
    logic [WORD_OFF_BITS-1:0] curr_word_off;
    logic [1:0]               curr_byte_off;

    logic [IDX_BITS-1:0]      pend_index;
    logic [TAG_BITS-1:0]      pend_tag;
    logic [WORD_OFF_BITS-1:0] pend_word_off;
    logic [1:0]               pend_byte_off;
    logic [PADDR_WIDTH-1:0]   pend_line_addr;

    assign curr_index    = cpu_addr[OFFSET_BITS +: IDX_BITS];
    assign curr_tag      = cpu_addr[PADDR_WIDTH-1 -: TAG_BITS];
    assign curr_word_off = cpu_addr[OFFSET_BITS-1:2];
    assign curr_byte_off = cpu_addr[1:0];

    assign pend_index     = pending.addr[OFFSET_BITS +: IDX_BITS];
    assign pend_tag       = pending.addr[PADDR_WIDTH-1 -: TAG_BITS];
    assign pend_word_off  = pending.addr[OFFSET_BITS-1:2];
    assign pend_byte_off  = pending.addr[1:0];
    assign pend_line_addr = {pending.addr[PADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};

    assign curr_cache_hit = valid_array[curr_index] && (tag_array[curr_index] == curr_tag);

    wire cache_hit = valid_array[pend_index] && (tag_array[pend_index] == pend_tag);

    state_t state;

    logic                      mem_req_r;
    logic [PADDR_WIDTH-1:0]    mem_addr_r;

    assign mem_req   = mem_req_r;
    assign mem_addr  = mem_addr_r;

   function automatic [31:0] load_from_line(
        input [LINE_BYTES*8-1:0]  line_data,
        input [WORD_OFF_BITS-1:0] word_idx,
        input [1:0]               byte_off,
        input [1:0]               size
    );
        logic [31:0] word_data;
        logic [31:0] result;
        begin
            word_data = line_data[(word_idx*32) +: 32];

            case (size)
                SIZE_BYTE: begin
                    case (byte_off)
                        2'b00: result = {24'h0, word_data[7:0]};
                        2'b01: result = {24'h0, word_data[15:8]};
                        2'b10: result = {24'h0, word_data[23:16]};
                        2'b11: result = {24'h0, word_data[31:24]};
                    endcase
                end
                SIZE_HALF: begin
                    case (byte_off[1])
                        1'b0: result = {16'h0, word_data[15:0]};
                        1'b1: result = {16'h0, word_data[31:16]};
                    endcase
                end
                SIZE_WORD: begin
                    result = word_data;
                end
                default: result = word_data;
            endcase

            return result;
        end
    endfunction

    logic [31:0] load_hit_rdata;
    logic        load_hit_valid;

    assign load_hit_valid = cpu_req && curr_cache_hit && (state == S_IDLE);

    assign load_hit_rdata =
        load_from_line(
            data_array[curr_index],
            curr_word_off,
            curr_byte_off,
            cpu_size
        );

    logic                      cpu_ready_r;
    logic [31:0]               cpu_rdata_r;
    logic                      cpu_rvalid_r;

    assign cpu_rvalid = load_hit_valid | cpu_rvalid_r;
    assign cpu_rdata  = load_hit_valid ? load_hit_rdata : cpu_rdata_r;
    assign cpu_ready  = load_hit_valid ? 1'b1 : cpu_ready_r;

    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state <= S_IDLE;

            for (int i = 0; i < N_LINES; i++) begin
                tag_array[i]   <= '0;
                valid_array[i] <= 1'b0;
                data_array[i]  <= '0;
            end

            pending.valid <= 1'b0;
            pending.addr  <= '0;
            pending.size  <= SIZE_WORD;

            cpu_ready_r  <= 1'b1;
            cpu_rdata_r  <= '0;
            cpu_rvalid_r <= 1'b0;

            mem_req_r   <= 1'b0;
            mem_addr_r  <= '0;

        end else begin
            cpu_rvalid_r <= 1'b0;
            case (state)
                S_IDLE: begin
                    mem_req_r <= 1'b0;
                    if (cpu_req && cpu_ready_r) begin
                        if (curr_cache_hit) begin
                            // HIT
                            cpu_ready_r <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            // MISS
                            pending.valid <= 1'b1;
                            pending.addr  <= cpu_addr;
                            pending.size  <= cpu_size;

                            cpu_ready_r <= 1'b0;
                            mem_req_r  <= 1'b1;
                            mem_addr_r <= {cpu_addr[PADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
                            state <= S_REFILL;
                        end
                    end else begin
                        cpu_ready_r <= (state == S_IDLE) ? 1'b1 : cpu_ready_r;
                        state <= S_IDLE;
                    end
                end

                S_REFILL: begin
                    if (state_reset) begin
                      cpu_ready_r <= 1'b1;
                      mem_req_r   <= 1'b0;
                      state       <= S_IDLE;
                    end else begin
                      if (mem_req_r && !mem_gnt) begin
                          // request not grnted, keep trying
                          mem_req_r  <= 1'b1;
                          mem_addr_r <= pend_line_addr;
                      end else if (mem_gnt) begin
                          // request grntd, stop request
                          mem_req_r <= 1'b0;
                      end
                      // wait for mem resp
                      if (mem_rvalid) begin
                          // refill complete
                          tag_array[pend_index]   <= pend_tag;
                          valid_array[pend_index] <= 1'b1;
                          data_array[pend_index]  <= mem_rdata;

                          cpu_rdata_r  <= load_from_line(
                              mem_rdata,
                              pend_word_off,
                              pend_byte_off,
                              pending.size
                          );
                          cpu_rvalid_r <= 1'b1;

                          cpu_ready_r   <= 1'b1;
                          pending.valid <= 1'b0;
                          state <= S_IDLE;
                      end else begin
                          // waiting mem
                          cpu_ready_r <= 1'b0;
                          state <= S_REFILL;
                      end
                    end
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end
endmodule
