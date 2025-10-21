import params_pkg::*;

module wb_stage #(
  parameter int DATA_WIDTH = params_pkg::DATA_WIDTH
)(
  input  logic [DATA_WIDTH-1:0] alu_result_i,
  input  logic [DATA_WIDTH-1:0] data_from_mem_i, 
  input  logic is_load_i,
  output logic [DATA_WIDTH-1:0] data_to_reg_o,
`ifndef SYNTHESIS
  input logic  debug_valid_i,
  input logic  debug_is_store_i,
  output logic debug_non_store_is_completed_o
`endif
);

  assign data_to_reg_o = is_load_i ? data_from_mem_i : alu_result_i;

`ifndef SYNTHESIS
  assign debug_non_store_is_completed_o = debug_valid_i & ~debug_is_store_i; 
`endif

endmodule : wb_stage
