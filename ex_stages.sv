import params_pkg::*;

module ex_stages (
  input  logic ex1_valid_i,
  input  logic ex2_valid_i,
  input  logic ex3_valid_i,
  input  logic ex4_valid_i,
  input  logic ex5_valid_i,
  input  reg_id_t ex1_wr_reg_i,
  input  reg_id_t ex2_wr_reg_i,
  input  reg_id_t ex3_wr_reg_i,
  input  reg_id_t ex4_wr_reg_i,
  input  reg_id_t ex5_wr_reg_i,
  input  data_t ex1_a_i,
  input  data_t ex1_b_i,
  input  data_t ex2_result_i,
  input  data_t ex3_result_i,
  input  data_t ex4_result_i,
  input  data_t ex5_result_i,
`ifndef SYNTHESIS
  input  vaddr_t ex1_debug_pc_i,
  input  vaddr_t ex2_debug_pc_i,
  input  vaddr_t ex3_debug_pc_i,
  input  vaddr_t ex4_debug_pc_i,
  input  vaddr_t ex5_debug_pc_i,
  input  var instruction_t ex1_debug_instr_i,
  input  var instruction_t ex2_debug_instr_i,
  input  var instruction_t ex3_debug_instr_i,
  input  var instruction_t ex4_debug_instr_i,
  input  var instruction_t ex5_debug_instr_i,
`endif
  output logic ex2_valid_o,
  output logic ex3_valid_o,
  output logic ex4_valid_o,
  output logic ex5_valid_o,
  output logic result_ready_o,
  output reg_id_t ex2_wr_reg_o,
  output reg_id_t ex3_wr_reg_o,
  output reg_id_t ex4_wr_reg_o,
  output reg_id_t ex5_wr_reg_o,
  output reg_id_t wr_reg_o,
  output data_t ex2_result_o,
  output data_t ex3_result_o,
  output data_t ex4_result_o,
  output data_t ex5_result_o,
  output data_t result_o,
`ifndef SYNTHESIS
  output vaddr_t ex2_debug_pc_o,
  output vaddr_t ex3_debug_pc_o,
  output vaddr_t ex4_debug_pc_o,
  output vaddr_t ex5_debug_pc_o,
  output vaddr_t debug_pc_o,
  output var instruction_t ex2_debug_instr_o,
  output var instruction_t ex3_debug_instr_o,
  output var instruction_t ex4_debug_instr_o,
  output var instruction_t ex5_debug_instr_o,
  output var instruction_t debug_instr_o
`endif
);

  assign ex2_valid_o = ex1_valid_i;
  assign ex3_valid_o = ex2_valid_i;
  assign ex4_valid_o = ex3_valid_i;
  assign ex5_valid_o = ex4_valid_i;

  assign ex2_wr_reg_o = ex1_wr_reg_i;
  assign ex3_wr_reg_o = ex2_wr_reg_i;
  assign ex4_wr_reg_o = ex3_wr_reg_i;
  assign ex5_wr_reg_o = ex4_wr_reg_i;
  assign wr_reg_o     = ex5_wr_reg_i;

  assign ex2_result_o   = ex1_a_i * ex1_b_i;
  assign ex3_result_o   = ex2_result_i;
  assign ex4_result_o   = ex3_result_i;
  assign ex5_result_o   = ex4_result_i;
  assign result_o       = ex5_result_i;
  assign result_ready_o = ex5_valid_i;

`ifndef SYNTHESIS
  assign ex2_debug_pc_o  = ex1_debug_pc_i;
  assign ex3_debug_pc_o  = ex2_debug_pc_i;
  assign ex4_debug_pc_o  = ex3_debug_pc_i;
  assign ex5_debug_pc_o  = ex4_debug_pc_i;
  assign debug_pc_o      = ex5_debug_pc_i;

  assign ex2_debug_instr_o = ex1_debug_instr_i;
  assign ex3_debug_instr_o = ex2_debug_instr_i;
  assign ex4_debug_instr_o = ex3_debug_instr_i;
  assign ex5_debug_instr_o = ex4_debug_instr_i;
  assign debug_instr_o     = ex5_debug_instr_i;
`endif

endmodule : ex_stages
