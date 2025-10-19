`include "alu.sv"

import params_pkg::*;

module alu_stage #(
  parameter int DATA_WIDTH   = params_pkg::DATA_WIDTH,
  parameter int OPCODE_WIDTH = params_pkg::OPCODE_WIDTH
)(
  input  logic [DATA_WIDTH-1:0] data_a_i,
  input  logic [DATA_WIDTH-1:0] data_b_i,
  input  logic [DATA_WIDTH-1:0] offset_sign_extend_i,
  input  logic [OPCODE_WIDTH-1:0] instr_opcode_i,
  output logic [DATA_WIDTH-1:0] alu_result_o,
  output logic is_load_o,
  output logic is_store_o,
  output logic is_zero_o,
  output logic is_less_o
);

  logic [DATA_WIDTH-1:0] data_a_to_alu;

  assign is_load_o = instr_opcode_i == LW;
  assign is_store_o = instr_opcode_i == SW;

  assign data_a_to_alu = (is_load_o | is_store_o) ? offset_sign_extend_i : data_a_i;

  alu #(
    .OPCODE_WIDTH (OPCODE_WIDTH),
    .DATA_WIDTH   (DATA_WIDTH  )
  ) alu (
    .op_i      (instr_opcode_i),
    .a_i       (data_a_to_alu),
    .b_i       (data_b_i),
    .is_zero_o (is_zero_o),
    .is_less_o (is_less_o),
    .result_o  (alu_result_o)
  );

endmodule : alu_stage
