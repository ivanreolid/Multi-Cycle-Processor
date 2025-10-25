`include "imem.sv"

import params_pkg::*;

module fetch_stage #(
  parameter int ADDR_WIDTH = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH = params_pkg::DATA_WIDTH,
  parameter int MEM_SIZE   = params_pkg::MEM_SIZE
)(
  input  logic valid_i,
  input  logic [ADDR_WIDTH-1:0] pc_i,
  input  logic [ADDR_WIDTH-1:0] jump_address_i,
  input  logic [ADDR_WIDTH-1:0] pc_branch_offset_i,
  input  logic alu_branch_taken_i,
  input  logic is_jump_i,
  output logic valid_o,
  output logic dec_valid_o,
  output logic [ADDR_WIDTH-1:0] next_pc_o,
  output instruction_t instruction_o
);

  logic [ADDR_WIDTH-1:0] pc_added;

  imem #(
    .MEM_SIZE      (MEM_SIZE),
    .ADDR_WIDTH    (ADDR_WIDTH),
    .DATA_WIDTH    (INSTR_WIDTH)
  ) imem (
    .address_i     (pc_i),
    .instruction_o (instruction_o)
  );

  assign pc_added = (alu_branch_taken_i ? pc_branch_offset_i : pc_i + 1) % MEM_SIZE;
  assign next_pc_o = is_jump_i ? jump_address_i : pc_added;

  assign valid_o = 1'b1;
  assign dec_valid_o = valid_i & ~is_jump_i & ~alu_branch_taken_i;

endmodule : fetch_stage
