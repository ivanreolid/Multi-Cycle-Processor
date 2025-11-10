import params_pkg::*;

module wb_stage #(
  parameter int REGISTER_WIDTH = params_pkg::REGISTER_WIDTH,
  parameter int DATA_WIDTH     = params_pkg::DATA_WIDTH,
  parameter int ADDR_WIDTH     = params_pkg::ADDR_WIDTH
)(
  input  logic mem_valid_i,
  input  logic ex_valid_i,
  input  logic mem_reg_wr_en_i,
  input  logic [REGISTER_WIDTH-1:0] mem_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex_wr_reg_i,
  input  logic [DATA_WIDTH-1:0] alu_result_i,
  input  logic [DATA_WIDTH-1:0] ex_result_i,
  input  logic [DATA_WIDTH-1:0] data_from_mem_i,
  input  logic is_load_i,
`ifndef SYNTHESIS
  input  logic [ADDR_WIDTH-1:0] debug_mem_pc_i,
  input  logic [ADDR_WIDTH-1:0] debug_ex_pc_i,
  input  instruction_t debug_mem_instr_i,
  input  instruction_t debug_ex_instr_i,
`endif
  output logic reg_wr_en_o,
  output logic [REGISTER_WIDTH-1:0] wr_reg_o,
  output logic [DATA_WIDTH-1:0] data_to_reg_o,
`ifndef SYNTHESIS
  output logic [ADDR_WIDTH-1:0] debug_pc_o,
  output instruction_t debug_instr_o
`endif
);

  always_comb begin : outputs_generations
    if (mem_valid_i) begin
      reg_wr_en_o   = mem_reg_wr_en_i;
      wr_reg_o      = mem_wr_reg_i;
      data_to_reg_o = is_load_i ? data_from_mem_i : alu_result_i;
`ifndef SYNTHESIS
      debug_pc_o    = debug_mem_pc_i;
      debug_instr_o = debug_mem_instr_i;
`endif
    end else if (ex_valid_i) begin
      reg_wr_en_o   = 1'b1;
      wr_reg_o      = ex_wr_reg_i;
      data_to_reg_o = ex_result_i;
`ifndef SYNTHESIS
      debug_pc_o    = debug_ex_pc_i;
      debug_instr_o = debug_ex_instr_i;
`endif
    end else
      reg_wr_en_o   = 1'b0;
  end

endmodule : wb_stage
