`timescale 1ns / 1ps

module data_cache import params_pkg::*; #(
    parameter int ADDR_WIDTH = 32,
    parameter int LINE_BYTES = 16,
    parameter int N_LINES    = 4
) (
    input  logic                      clk,
    input  logic                      rstn,

    // CPU
    input  logic                      cpu_req,    // CPU request
    input  logic                      cpu_wr,     // 1=store, 0=load
    input  logic [ADDR_WIDTH-1:0]     cpu_addr,    // Byte address
    input  logic [31:0]               cpu_wdata, // Write data
    input  logic [3:0]                cpu_wstrb,  // Byte write strobes 
    input  logic [1:0]                cpu_size,   // 00=byte, 01=half, 10=word
    output logic                      cpu_ready,     // Cache ready for new req
    output logic [31:0]               cpu_rdata,  // Read data
    output logic                      cpu_rvalid,   // Read data valid
    output logic                      curr_cache_hit,

    // Memory
    output logic                      mem_req,    // Memory request valid
    output logic                      mem_we,     // 1=write, 0=read
    output logic [ADDR_WIDTH-1:0]     mem_addr,   // Line aligned address
    output logic [LINE_BYTES*8-1:0]   mem_wdata,  // Full line write data
    input  logic                      mem_gnt,    // Request granted (accepted by arbiter)
    input  logic                      mem_rvalid, // Memory read valid
    input  logic [LINE_BYTES*8-1:0]   mem_rdata,   // Full line read data

    input  logic finish,
    output logic done,
    output logic mem_finish,
    input write_done_o
);

    localparam OFFSET_BITS    = $clog2(LINE_BYTES);
    localparam IDX_BITS       = $clog2(N_LINES);
    localparam TAG_BITS       = ADDR_WIDTH - OFFSET_BITS - IDX_BITS;
    localparam WORDS_PER_LINE = LINE_BYTES / 4;
    localparam WORD_OFF_BITS  = $clog2(WORDS_PER_LINE);

    localparam SIZE_BYTE = 2'b00;
    localparam SIZE_HALF = 2'b01;
    localparam SIZE_WORD = 2'b10;

    typedef enum logic [2:0] {
        S_IDLE       = 3'b000,   // Ready for new CPU req
        S_WRITEBACK  = 3'b001,   // Evict dirty line to memory
        S_REFILL     = 3'b010,    // Fetch new line from memory
        S_FLUSH      = 3'b011,  // choose 
        S_FLUSH_WB   = 3'b100,  // send writeback
        S_FLUSH_DONE = 3'b101
    } state_t;

    logic [IDX_BITS:0] flush_idx;


    logic [TAG_BITS-1:0]        tag_array   [N_LINES];
    logic                       valid_array [N_LINES];
    logic                       dirty_array [N_LINES];
    logic [LINE_BYTES*8-1:0]    data_array  [N_LINES];

    typedef struct packed {
        logic                   valid;
        logic                   wr;
        logic [ADDR_WIDTH-1:0]  addr;
        logic [31:0]            wdata;
        logic [3:0]             wstrb;
        logic [1:0]             size;
    } pending_req_t;

    pending_req_t pending;

    wire [IDX_BITS-1:0]      curr_index     = cpu_addr[OFFSET_BITS +: IDX_BITS];
    wire [TAG_BITS-1:0]      curr_tag       = cpu_addr[ADDR_WIDTH-1 -: TAG_BITS];
    wire [WORD_OFF_BITS-1:0] curr_word_off  = cpu_addr[OFFSET_BITS-1:2];
    wire [1:0]               curr_byte_off  = cpu_addr[1:0];

    wire [IDX_BITS-1:0]      pend_index     = pending.addr[OFFSET_BITS +: IDX_BITS];
    wire [TAG_BITS-1:0]      pend_tag       = pending.addr[ADDR_WIDTH-1 -: TAG_BITS];
    wire [WORD_OFF_BITS-1:0] pend_word_off  = pending.addr[OFFSET_BITS-1:2];
    wire [1:0]               pend_byte_off  = pending.addr[1:0];
    wire [ADDR_WIDTH-1:0]    pend_line_addr = {pending.addr[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};

    assign curr_cache_hit      = valid_array[curr_index] && (tag_array[curr_index] == curr_tag);
    wire curr_need_writeback = valid_array[curr_index] && dirty_array[curr_index];

    wire cache_hit       = valid_array[pend_index] && (tag_array[pend_index] == pend_tag);
    wire need_writeback  = valid_array[pend_index] && dirty_array[pend_index];

    state_t state;


    logic                      mem_req_r;
    logic                      mem_we_r;
    logic [ADDR_WIDTH-1:0]     mem_addr_r;
    logic [LINE_BYTES*8-1:0]   mem_wdata_r;


    assign mem_req   = mem_req_r;
    assign mem_we    = mem_we_r;
    assign mem_addr  = mem_addr_r;
    assign mem_wdata = mem_wdata_r;

    function automatic [31:0] load_from_line(
        input [LINE_BYTES*8-1:0] line_data,
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

    function automatic [LINE_BYTES*8-1:0] store_to_line(
        input [LINE_BYTES*8-1:0]  old_line,
        input [WORD_OFF_BITS-1:0] word_idx,
        input [31:0]              wdata,
        input [3:0]               wstrb
    );
        integer byte_idx;
        logic [LINE_BYTES*8-1:0] new_line;
        begin
            new_line = old_line;
            for (byte_idx = 0; byte_idx < 4; byte_idx = byte_idx + 1) begin
                if (wstrb[byte_idx]) begin
                    new_line[(word_idx*32) + (byte_idx*8) +: 8] = wdata[(byte_idx*8) +: 8];
                end
            end
            return new_line;
        end
    endfunction

    function automatic [3:0] gen_wstrb(
        input [1:0] byte_off,
        input [1:0] size
    );
        logic [3:0] strb;
        begin
            case (size)
                SIZE_BYTE: begin
                    case (byte_off)
                        2'b00: strb = 4'b0001;
                        2'b01: strb = 4'b0010;
                        2'b10: strb = 4'b0100;
                        2'b11: strb = 4'b1000;
                    endcase
                end
                SIZE_HALF: begin
                    case (byte_off[1])
                        1'b0: strb = 4'b0011;
                        1'b1: strb = 4'b1100;
                    endcase
                end
                SIZE_WORD: begin
                    strb = 4'b1111;
                end
                default: strb = 4'b1111;
            endcase
            return strb;
        end
    endfunction

    function automatic [ADDR_WIDTH-1:0] evicted_addr(
        input [IDX_BITS-1:0] idx
    );
        return {tag_array[idx], idx, {OFFSET_BITS{1'b0}}};
    endfunction


    logic [31:0] load_hit_rdata;
    logic        load_hit_valid;

    assign load_hit_valid =
    cpu_req && !cpu_wr && curr_cache_hit && (state == S_IDLE);

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

    assign cpu_rvalid   = load_hit_valid | cpu_rvalid_r;

    assign cpu_rdata    = load_hit_valid ? load_hit_rdata : cpu_rdata_r;

    assign cpu_ready     = load_hit_valid ? 1'b1 : cpu_ready_r;
    
    assign mem_finish = (state == S_FLUSH) || (state == S_FLUSH_WB) || (state == S_FLUSH_DONE);

    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state <= S_IDLE;

            for (int i = 0; i < N_LINES; i++) begin
                tag_array[i]   <= '0;
                valid_array[i] <= 1'b0;
                dirty_array[i] <= 1'b0;
                data_array[i]  <= '0;
            end

            pending.valid <= 1'b0;
            pending.wr    <= 1'b0;
            pending.addr  <= '0;
            pending.wdata <= '0;
            pending.wstrb <= '0;
            pending.size  <= SIZE_WORD;

            cpu_ready_r  <= 1'b1;
            cpu_rdata_r  <= '0;
            cpu_rvalid_r <= 1'b0;

            mem_req_r   <= 1'b0;
            mem_we_r    <= 1'b0;
            mem_addr_r  <= '0;
            mem_wdata_r <= '0;

            flush_idx <= '0;
            done      <= 1'b0;
        end else begin
            cpu_rvalid_r <= 1'b0;

            case (state)
                S_IDLE: begin
                    done <= 1'b0;
                    if (finish) begin
                        flush_idx <= '0;
                        cpu_ready_r <= 1'b0;
                        state <= S_FLUSH;
                    end else begin
                        mem_req_r <= 1'b0;

                        if (cpu_req && cpu_ready_r) begin
                            if (curr_cache_hit) begin
                                // HIT
                                if (cpu_wr) begin
                                    // STORE HIT
                                    logic [3:0] wstrb_to_use;
                                    wstrb_to_use = (cpu_wstrb != 4'b0000) ? cpu_wstrb : gen_wstrb(curr_byte_off, cpu_size);
                                    data_array[curr_index] <= store_to_line(
                                        data_array[curr_index],
                                        curr_word_off,
                                        cpu_wdata,
                                        wstrb_to_use
                                    );
                                    dirty_array[curr_index] <= 1'b1;
                                end
                                cpu_ready_r <= 1'b1;
                                state <= S_IDLE;
                            end else begin
                                // MISS
                                pending.valid <= 1'b1;
                                pending.wr    <= cpu_wr;
                                pending.addr  <= cpu_addr;
                                pending.wdata <= cpu_wdata;
                                pending.size  <= cpu_size;

                                if (cpu_wr) begin
                                    pending.wstrb <= (cpu_wstrb != 4'b0000) ? cpu_wstrb : gen_wstrb(curr_byte_off, cpu_size);
                                end else begin
                                    pending.wstrb <= 4'b0000;
                                end
                                cpu_ready_r <= 1'b0;
                                if (curr_need_writeback) begin
                                    mem_req_r   <= 1'b1;
                                    mem_we_r    <= 1'b1;
                                    mem_addr_r  <= evicted_addr(curr_index);
                                    mem_wdata_r <= data_array[curr_index];
                                    state <= S_WRITEBACK;
                                end else begin
                                    mem_req_r  <= 1'b1;
                                    mem_we_r   <= 1'b0;
                                    mem_addr_r <= {cpu_addr[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
                                    state <= S_REFILL;
                                end
                            end
                        end else begin
                            cpu_ready_r <= (state == S_IDLE) ? 1'b1 : cpu_ready_r;
                            state <= S_IDLE;
                        end
                    end
                end

                S_WRITEBACK: begin
                    if (mem_req_r && !mem_gnt) begin
                        mem_req_r   <= 1'b1;
                        mem_we_r    <= 1'b1;
                        mem_addr_r  <= evicted_addr(pend_index);
                        mem_wdata_r <= data_array[pend_index];
                    end else if (mem_gnt && mem_we_r) begin
                        mem_req_r  <= 1'b1;
                        mem_we_r   <= 1'b0;
                        mem_addr_r <= pend_line_addr;
                        state <= S_REFILL;
                    end
                end

                S_REFILL: begin
                    if (mem_req_r && !mem_gnt) begin
                        mem_req_r  <= 1'b1;
                        mem_we_r   <= 1'b0;
                        mem_addr_r <= pend_line_addr;
                    end else if (mem_gnt) begin
                        mem_req_r <= 1'b0;
                    end

                    if (mem_rvalid) begin
                        tag_array[pend_index]   <= pend_tag;
                        valid_array[pend_index] <= 1'b1;

                        if (pending.wr) begin
                            data_array[pend_index] <= store_to_line(
                                mem_rdata,
                                pend_word_off,
                                pending.wdata,
                                pending.wstrb
                            );
                            dirty_array[pend_index] <= 1'b1;
                        end else begin
                            data_array[pend_index]  <= mem_rdata;
                            dirty_array[pend_index] <= 1'b0;

                            cpu_rdata_r  <= load_from_line(
                                mem_rdata,
                                pend_word_off,
                                pend_byte_off,
                                pending.size
                            );
                            cpu_rvalid_r <= 1'b1;
                        end
                        cpu_ready_r   <= 1'b1;
                        pending.valid <= 1'b0;
                        state <= S_IDLE;
                    end else begin
                        cpu_ready_r <= 1'b0;
                        state <= S_REFILL;
                    end
                end

                S_FLUSH: begin
                    if (flush_idx == N_LINES) begin
                        if (write_done_o == 1'b1) begin
                            state <= S_FLUSH_DONE;
                            done <= 1'b1;
                        end
                        flush_idx <= flush_idx;
                    end else if (valid_array[flush_idx] && dirty_array[flush_idx]) begin
                        mem_req_r   <= 1'b1;
                        mem_we_r    <= 1'b1;
                        mem_addr_r  <= evicted_addr(flush_idx);
                        mem_wdata_r <= data_array[flush_idx];
                        state <= S_FLUSH_WB;
                    end else begin
                        flush_idx <= flush_idx + 1'b1;
                        state <= S_FLUSH;
                    end
                end

                S_FLUSH_WB: begin
                    if (mem_req_r && !mem_gnt) begin
                        mem_req_r <= 1'b1;
                    end else if (mem_gnt) begin
                        mem_req_r <= 1'b0;
                        dirty_array[flush_idx] <= 1'b0;
                        valid_array[flush_idx] <= 1'b0;
                        flush_idx <= flush_idx + 1'b1;
                        state <= S_FLUSH;
                    end
                end

                S_FLUSH_DONE: begin
                    done <= 1'b1;
                    cpu_ready_r <= 1'b1;
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
