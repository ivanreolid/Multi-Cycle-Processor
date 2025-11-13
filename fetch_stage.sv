`include "imem.sv"

import params_pkg::*;

module fetch_stage #(
  parameter int ADDR_WIDTH = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH = params_pkg::DATA_WIDTH,
  parameter int MEM_SIZE   = params_pkg::MEM_SIZE
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
  input  logic instr_valid_i,
  input  logic [DATA_WIDTH-1:0] instr_i,
  output logic rd_req_valid_o,
  output logic dec_valid_o,
  output logic [ADDR_WIDTH-1:0] mem_req_addr_o,
  output logic [ADDR_WIDTH-1:0] dec_pc_o,
  output access_size_t req_access_size_o,
  output var instruction_t instruction_o
);

  typedef enum logic [2:0] {
    IDLE     = 3'b000,
    MEM_REQ  = 3'b001,
    MEM_WAIT = 3'b010,
    FLUSH    = 3'b011,
    STALL    = 3'b100
  } state_t;

  logic [ADDR_WIDTH-1:0] pc, pc_d, dec_pc_d;
  logic [ADDR_WIDTH-1:0] branch_target;

  state_t state, state_d;

  logic dec_valid_d;

  instruction_t dec_instr_d;

  always_comb begin : state_update
    rd_req_valid_o   = 1'b0;
    state_d          = state;

    case (state)
      IDLE: begin
        pc_d        = pc;
        dec_valid_d = 1'b0;
        state_d     = MEM_REQ;
      end
      MEM_REQ: begin
        dec_valid_d         = 1'b0;
        if (!mem_req_i) begin
          rd_req_valid_o    = 1'b1;
          mem_req_addr_o    = pc;
          req_access_size_o = WORD;
          state_d           = MEM_WAIT;
        end
      end
      MEM_WAIT: begin
        if (alu_branch_taken_i | is_jump_i)
          state_d = FLUSH;
        else if (instr_valid_i) begin
          dec_valid_d     = 1'b1;
          pc_d            = (pc + 4) % MEM_SIZE;
          dec_pc_d        = pc;
          dec_instr_d     = instruction_t'(instr_i);
          state_d         = dec_stall_i ? STALL : MEM_REQ;
        end
      end
      STALL: begin
        if (!dec_stall_i)
          state_d     = MEM_REQ;
      end
      FLUSH: begin
        if (instr_valid_i) begin
          pc_d    = branch_target;
          state_d = MEM_REQ;
        end
      end
    endcase
  end

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      state          <= IDLE;
      pc             <= '0;
      branch_target  <= '0;
      dec_valid_o    <= 1'b0;
      dec_pc_o       <= '0;
      instruction_o  <= '0;
    end else if (!dec_stall_i) begin
      state          <= state_d;
      pc             <= pc_d;
      dec_valid_o    <= dec_valid_d;
      dec_pc_o       <= dec_pc_d;
      instruction_o  <= dec_instr_d;

      if (alu_branch_taken_i)
        branch_target <= pc_branch_offset_i;
      else if (is_jump_i)
        branch_target <= jump_address_i;
    end else
      state           <= state_d;
  end

endmodule : fetch_stage
