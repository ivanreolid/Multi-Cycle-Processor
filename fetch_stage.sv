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

  typedef enum logic [2:0] {
    IDLE     = 3'b000,
    MEM_REQ  = 3'b001,
    MEM_WAIT = 3'b010,
    STALL    = 3'b100
  } state_t;

  state_t state, state_d;
  logic [ADDR_WIDTH-1:0] pc, pc_d;

  // Stall Buffer
  logic [ADDR_WIDTH-1:0] pc_buffer;
  instruction_t instr_buffer;
  logic buffer_wr_en;

  // Cache signals
  logic cache_req;
  logic cache_ready;
  logic cache_rvalid;
  logic [31:0] cache_rdata;

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
    .mem_req(rd_req_valid_o),
    .mem_addr(mem_req_addr_o),
    .mem_gnt(mem_gnt_i),
    .mem_rvalid(instr_valid_i),
    .mem_rdata(instr_line_i)
  );

  // --- Next State Logic ---
  always_comb begin : state_update
    // Default values
    state_d      = state;
    pc_d         = pc;
    cache_req    = 1'b0;
    buffer_wr_en = 1'b0;
    dec_valid_o  = 1'b0;
    dec_pc_o     = pc;
    dec_instr_o  = instruction_t'('0);
    req_access_size_o = WORD;

    if (alu_branch_taken_i || is_jump_i) begin
      pc_d = alu_branch_taken_i ? pc_branch_offset_i : jump_address_i;
      state_d = MEM_REQ;
    end
    else begin
      case (state)
        IDLE: begin
          state_d = MEM_REQ;
        end
        MEM_REQ: begin
          if (cache_ready) begin
            cache_req = 1'b1;
            state_d   = MEM_WAIT;
          end
        end
        MEM_WAIT: begin
          if (cache_rvalid) begin
            if (dec_stall_i) begin
              buffer_wr_en = 1'b1;
              state_d      = STALL;
            end else begin
              dec_valid_o = 1'b1;
              dec_instr_o = instruction_t'(cache_rdata);
              dec_pc_o    = pc;

              pc_d        = (pc + 4) % MEM_SIZE;
              state_d     = MEM_REQ;
            end
          end
        end

        STALL: begin
          dec_valid_o = 1'b1;
          dec_instr_o = instr_buffer;
          dec_pc_o    = pc_buffer;

          if (!dec_stall_i) begin
            pc_d    = (pc + 4) % MEM_SIZE;
            state_d = MEM_REQ;
          end
        end

        default: state_d = IDLE;
      endcase
    end
  end

  // --- Sequential Logic ---
  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      state        <= IDLE;
      pc           <= '0;
      instr_buffer <= '0;
      pc_buffer    <= '0;
    end else begin
      state <= state_d;
      pc    <= pc_d;

      if (buffer_wr_en) begin
        instr_buffer <= instruction_t'(cache_rdata);
        pc_buffer    <= pc;
      end
    end
  end

endmodule : fetch_stage
