import params_pkg::*;

module ex_stages #(
  parameter int REGISTER_WIDTH = params_pkg::REGISTER_WIDTH,
  parameter int ADDR_WIDTH     = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH     = params_pkg::DATA_WIDTH
)(
  input  logic ex1_valid_i,
  input  logic ex2_valid_i,
  input  logic ex3_valid_i,
  input  logic ex4_valid_i,
  input  logic ex5_valid_i,
  input  logic [REGISTER_WIDTH-1:0] ex1_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex2_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex3_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex4_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex5_wr_reg_i,
  input  logic [DATA_WIDTH-1:0] ex1_a_i,
  input  logic [DATA_WIDTH-1:0] ex1_b_i,
  input  logic [DATA_WIDTH-1:0] ex2_result_i,
  input  logic [DATA_WIDTH-1:0] ex3_result_i,
  input  logic [DATA_WIDTH-1:0] ex4_result_i,
  input  logic [DATA_WIDTH-1:0] ex5_result_i,
`ifndef SYNTHESIS
  input  logic [ADDR_WIDTH-1:0] ex1_debug_pc_i,
  input  logic [ADDR_WIDTH-1:0] ex2_debug_pc_i,
  input  logic [ADDR_WIDTH-1:0] ex3_debug_pc_i,
  input  logic [ADDR_WIDTH-1:0] ex4_debug_pc_i,
  input  logic [ADDR_WIDTH-1:0] ex5_debug_pc_i,
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
  output logic wb_is_next_cycle_o,
  output logic result_ready_o,
  output logic [REGISTER_WIDTH-1:0] ex2_wr_reg_o,
  output logic [REGISTER_WIDTH-1:0] ex3_wr_reg_o,
  output logic [REGISTER_WIDTH-1:0] ex4_wr_reg_o,
  output logic [REGISTER_WIDTH-1:0] ex5_wr_reg_o,
  output logic [REGISTER_WIDTH-1:0] wr_reg_o,
  output logic [DATA_WIDTH-1:0] ex2_result_o,
  output logic [DATA_WIDTH-1:0] ex3_result_o,
  output logic [DATA_WIDTH-1:0] ex4_result_o,
  output logic [DATA_WIDTH-1:0] ex5_result_o,
  output logic [DATA_WIDTH-1:0] result_o,
`ifndef SYNTHESIS
  output logic [ADDR_WIDTH-1:0] ex2_debug_pc_o,
  output logic [ADDR_WIDTH-1:0] ex3_debug_pc_o,
  output logic [ADDR_WIDTH-1:0] ex4_debug_pc_o,
  output logic [ADDR_WIDTH-1:0] ex5_debug_pc_o,
  output logic [ADDR_WIDTH-1:0] debug_pc_o,
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

  assign wb_is_next_cycle_o = ex5_valid_o;

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
