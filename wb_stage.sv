import params_pkg::*;

module wb_stage #(
  parameter int DATA_WIDTH = params_pkg::DATA_WIDTH,
  parameter int ADDR_WIDTH = params_pkg::ADDR_WIDTH
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic [DATA_WIDTH-1:0] alu_result_i,
  input  logic [DATA_WIDTH-1:0] data_from_mem_i,
  input  logic is_load_i,
`ifndef SYNTHESIS
  input logic  debug_valid_i,
  input  logic [ADDR_WIDTH-1:0] debug_pc_i,
  input  instruction_t debug_instr_i,
`endif
  output logic [DATA_WIDTH-1:0] data_to_reg_o,
`ifndef SYNTHESIS
  output logic debug_instr_is_completed_o,
  output logic [ADDR_WIDTH-1:0] debug_pc_o,
  output instruction_t debug_instr_o
`endif
);

  always_ff @(posedge clk_i) begin : flops
`ifndef SYNTHESIS
    debug_instr_is_completed_o <= debug_valid_i;
    debug_pc_o                 <= debug_pc_i;
    debug_instr_o              <= debug_instr_i;
`endif
  end

  assign data_to_reg_o = is_load_i ? data_from_mem_i : alu_result_i;

endmodule : wb_stage
