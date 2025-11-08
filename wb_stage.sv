import params_pkg::*;

module wb_stage #(
  parameter int REGISTER_WIDTH = params_pkg::REGISTER_WIDTH,
  parameter int DATA_WIDTH     = params_pkg::DATA_WIDTH,
  parameter int ADDR_WIDTH     = params_pkg::ADDR_WIDTH
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic valid_i,
  input  logic reg_wr_en_i,
  input  logic [REGISTER_WIDTH-1:0] wr_reg_i,
  input  logic [DATA_WIDTH-1:0] alu_result_i,
  input  logic [DATA_WIDTH-1:0] data_from_mem_i,
  input  logic is_load_i,
  output logic [DATA_WIDTH-1:0] data_to_reg_o
);

  assign data_to_reg_o = is_load_i ? data_from_mem_i : alu_result_i;

endmodule : wb_stage
