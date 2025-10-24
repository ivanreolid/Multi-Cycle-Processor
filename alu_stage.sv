`include "alu.sv"

import params_pkg::*;

module alu_stage #(
  parameter int DATA_WIDTH   = params_pkg::DATA_WIDTH,
  parameter int OPCODE_WIDTH = params_pkg::OPCODE_WIDTH
)(
  input  logic [DATA_WIDTH-1:0] data_a_i,
  input  logic [DATA_WIDTH-1:0] data_b_i,
  input  logic [ADDR_WIDTH-1:0] pc_i,
  input  logic [ADDR_WIDTH-1:0] branch_offset_i,
  input  logic [DATA_WIDTH-1:0] offset_sign_extend_i,
  input  logic [OPCODE_WIDTH-1:0] instr_opcode_i,
  input  logic valid_i,
  output logic mem_valid_o,
  output logic [ADDR_WIDTH-1:0] pc_branch_offset_o,
  output logic [ADDR_WIDTH-1:0] jump_address_o,
  output logic [DATA_WIDTH-1:0] alu_result_o,
  output logic is_load_o,
  output logic is_store_o,
  output logic branch_taken_o,
  output logic is_jump_o
);

  logic [DATA_WIDTH-1:0] data_a_to_alu;

  logic is_zero, is_less;

  assign is_load_o  = instr_opcode_i == LW;
  assign is_store_o = instr_opcode_i == SW;
  assign is_jump_o  = valid_i & instr_opcode_i == JMP;

  assign branch_taken_o = valid_i & (((instr_opcode_i == BEQ) & is_zero) | ((instr_opcode_i == BNE) & ~is_zero) | ((instr_opcode_i == BLT) & is_less) | ((instr_opcode_i == BGE) & ~is_less));

  assign pc_branch_offset_o = pc_i + branch_offset_i;
  assign jump_address_o = data_a_i;

  assign data_a_to_alu = (is_load_o | is_store_o) ? offset_sign_extend_i : data_a_i;

  assign mem_valid_o = valid_i;

  alu #(
    .OPCODE_WIDTH (OPCODE_WIDTH),
    .DATA_WIDTH   (DATA_WIDTH  )
  ) alu (
    .op_i      (instr_opcode_i),
    .a_i       (data_a_to_alu),
    .b_i       (data_b_i),
    .is_zero_o (is_zero),
    .is_less_o (is_less),
    .result_o  (alu_result_o)
  );

endmodule : alu_stage
