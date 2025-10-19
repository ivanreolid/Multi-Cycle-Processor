import params_pkg::*;

module wb_stage #(
  parameter int DATA_WIDTH = params_pkg::DATA_WIDTH
)(
  input  logic [DATA_WIDTH-1:0] alu_result_i,
  input  logic [DATA_WIDTH-1:0] data_from_mem_i,
  input  logic is_load_i,
  output logic [DATA_WIDTH-1:0] data_to_reg_o
);

  assign data_to_reg_o = is_load_i ? data_from_mem_i : alu_result_i;

endmodule : wb_stage
