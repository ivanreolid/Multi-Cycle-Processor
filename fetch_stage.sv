`include "imem.sv"

import params_pkg::*;

module fetch_stage #(
  parameter int ADDR_WIDTH = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH = params_pkg::DATA_WIDTH,
  parameter int MEM_SIZE   = params_pkg::MEM_SIZE
)(
  input  logic valid_i,
  input  logic alu_branch_taken_i,
  input  logic is_jump_i,
  input  logic [ADDR_WIDTH-1:0] pc_i,
  output logic dec_valid_o,
  output instruction_t instruction_o
);

  imem #(
    .MEM_SIZE      (MEM_SIZE),
    .ADDR_WIDTH    (ADDR_WIDTH),
    .DATA_WIDTH    (INSTR_WIDTH)
  ) imem (
    .address_i     (pc_i),
    .instruction_o (instruction_o)
  );

  assign dec_valid_o = valid_i & ~is_jump_i & ~alu_branch_taken_i;

endmodule : fetch_stage
