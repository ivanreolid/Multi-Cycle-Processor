`include "instr_cache.sv"

import params_pkg::*;

module fetch_stage #(
  parameter int ADDR_WIDTH       = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH       = params_pkg::DATA_WIDTH,
  parameter int MEM_SIZE         = params_pkg::MEM_SIZE,
  parameter int CACHE_LINE_BYTES = 16,
  parameter int ICACHE_N_LINES   = 4
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic mem_req_i,
  input  logic alu_branch_taken_i,
  input  logic is_jump_i,
  input  logic dec_stall_i,
  input  logic mem_stall_i,
  input  logic [ADDR_WIDTH-1:0] pc_branch_offset_i,
  input  logic [ADDR_WIDTH-1:0] jump_address_i,

  // Memory interface (Cache to Memory)
  input  logic instr_valid_i,
  input  logic [CACHE_LINE_BYTES*8-1:0] instr_line_i,
  output logic rd_req_valid_o,
  output logic [ADDR_WIDTH-1:0] mem_req_addr_o,
  output access_size_t req_access_size_o,

  // Memory arbiter grant signal
  input  logic mem_gnt_i,

  // Decode stage outputs
  output logic dec_valid_o,
  output logic [ADDR_WIDTH-1:0] dec_pc_o,
  output var instruction_t dec_instr_o
);

  typedef enum logic [1:0] {
    FETCH    = 2'b00,
    WAIT     = 2'b01,
    STALL    = 2'b10
  } state_t;

  state_t state, state_d;
  logic [ADDR_WIDTH-1:0] pc, pc_d;
  logic pc_valid, pc_valid_d;  // Track if PC is valid for fetching

  // Stall Buffer
  logic [ADDR_WIDTH-1:0] pc_buffer;
  instruction_t instr_buffer;
  logic buffer_valid, buffer_valid_d;

  // Cache signals
  logic cache_req;
  logic cache_ready;
  logic cache_rvalid;
  logic [31:0] cache_rdata;
  logic cache_hit;

  // --- Cache Instantiation ---
  instr_cache #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .LINE_BYTES(CACHE_LINE_BYTES),
    .N_LINES(ICACHE_N_LINES)
  ) i_cache (
    .clk(clk_i),
    .rstn(rst_i),
    .cpu_req(cache_req),
    .cpu_addr(pc),
    .cpu_size(WORD),
    .cpu_ready(cache_ready),
    .cpu_rdata(cache_rdata),
    .cpu_rvalid(cache_rvalid),
    .curr_cache_hit(cache_hit),
    .mem_req(rd_req_valid_o),
    .mem_addr(mem_req_addr_o),
    .mem_gnt(mem_gnt_i),
    .mem_rvalid(instr_valid_i),
    .mem_rdata(instr_line_i)
  );

  // --- Next State Logic ---
  always_comb begin : state_update
    // Default values
    state_d        = state;
    pc_d           = pc;
    pc_valid_d     = pc_valid;
    cache_req      = 1'b0;
    buffer_valid_d = buffer_valid;
    dec_valid_o    = 1'b0;
    dec_pc_o       = '0;
    dec_instr_o    = instruction_t'('0);
    req_access_size_o = WORD;

    // Branch/Jump has highest priority
    if (alu_branch_taken_i || is_jump_i) begin
      // Update PC but don't fetch yet - will fetch next cycle
      pc_d = alu_branch_taken_i ? pc_branch_offset_i : jump_address_i;
      pc_valid_d = 1'b1;
      buffer_valid_d = 1'b0;  // Flush buffer
      dec_valid_o = 1'b0;     // Invalidate decode output
      state_d = FETCH;
    end
    else begin
      case (state)
        FETCH: begin
          if (buffer_valid) begin
            // We have buffered instruction from previous stall
            dec_valid_o = 1'b1;
            dec_instr_o = instr_buffer;
            dec_pc_o    = pc_buffer;

            if (!dec_stall_i) begin
              // Decode accepted, clear buffer and advance
              buffer_valid_d = 1'b0;
              pc_d = (pc + 4) % MEM_SIZE;
              pc_valid_d = 1'b1;
              state_d = FETCH;
            end else begin
              // Decode still stalled, keep buffer
              state_d = FETCH;
            end
          end
          else if (pc_valid) begin
            // Try to fetch from cache
            cache_req = 1'b1;

            // Check if cache responds same cycle (hit)
            if (cache_ready && cache_hit) begin
              // Same-cycle cache hit!
              if (dec_stall_i) begin
                // Decode is stalled, buffer the instruction
                buffer_valid_d = 1'b1;
                state_d = FETCH;
              end else begin
                // Send to decode immediately
                dec_valid_o = 1'b1;
                dec_instr_o = instruction_t'(cache_rdata);
                dec_pc_o    = pc;
                
                // Advance PC
                pc_d = (pc + 4) % MEM_SIZE;
                pc_valid_d = 1'b1;
                state_d = FETCH;
              end
            end
            else if (!cache_ready) begin
              // Cache miss, need to wait for refill
              pc_valid_d = 1'b0;  // Mark PC as invalid during wait
              state_d = WAIT;
            end
          end
          else begin
            // PC not valid yet (might be first cycle after reset)
            pc_valid_d = 1'b1;
            state_d = FETCH;
          end
        end

        WAIT: begin
          // Waiting for cache refill
          if (cache_rvalid) begin
            // Data arrived from memory
            if (dec_stall_i) begin
              // Decode stalled, buffer it
              buffer_valid_d = 1'b1;
              pc_valid_d = 1'b1;
              state_d = FETCH;
            end else begin
              // Send to decode
              dec_valid_o = 1'b1;
              dec_instr_o = instruction_t'(cache_rdata);
              dec_pc_o    = pc;
              
              // Advance PC
              pc_d = (pc + 4) % MEM_SIZE;
              pc_valid_d = 1'b1;
              state_d = FETCH;
            end
          end else begin
            // Keep waiting
            state_d = WAIT;
          end
        end

        default: begin
          state_d = FETCH;
          pc_valid_d = 1'b1;
        end
      endcase
    end
  end

  // --- Sequential Logic ---
  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      state        <= FETCH;
      pc           <= '0;
      pc_valid     <= 1'b1;
      instr_buffer <= '0;
      pc_buffer    <= '0;
      buffer_valid <= 1'b0;
    end else begin
      state    <= state_d;
      pc       <= pc_d;
      pc_valid <= pc_valid_d;
      buffer_valid <= buffer_valid_d;

      // Update buffer when transitioning to buffer_valid state
      if (!buffer_valid && buffer_valid_d) begin
        if (state == FETCH && cache_ready && cache_hit) begin
          instr_buffer <= instruction_t'(cache_rdata);
          pc_buffer    <= pc;
        end
        else if (state == WAIT && cache_rvalid) begin
          instr_buffer <= instruction_t'(cache_rdata);
          pc_buffer    <= pc;
        end
      end
    end
  end

endmodule : fetch_stage