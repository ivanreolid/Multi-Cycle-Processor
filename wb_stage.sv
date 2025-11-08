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
`ifndef SYNTHESIS
  input  logic [ADDR_WIDTH-1:0] debug_pc_i,
  input  instruction_t debug_instr_i,
`endif
  output logic reg_wr_en_o,
  output logic [REGISTER_WIDTH-1:0] wr_reg_o,
  output logic [DATA_WIDTH-1:0] data_to_reg_o,
`ifndef SYNTHESIS
  output logic debug_instr_is_completed_o,
  output logic [ADDR_WIDTH-1:0] debug_pc_o,
  output instruction_t debug_instr_o
`endif
);

  always_ff @(posedge clk_i) begin : flops
`ifndef SYNTHESIS
    debug_instr_is_completed_o <= valid_i;
    debug_pc_o                 <= debug_pc_i;
    debug_instr_o              <= debug_instr_i;
`endif
  end

  assign reg_wr_en_o   = reg_wr_en_i;
  assign wr_reg_o      = wr_reg_i;
  assign data_to_reg_o = is_load_i ? data_from_mem_i : alu_result_i;

endmodule : wb_stage
