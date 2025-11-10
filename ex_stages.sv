import params_pkg::*;

module ex_stages #(
  parameter int REGISTER_WIDTH = params_pkg::REGISTER_WIDTH,
  parameter int ADDR_WIDTH     = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH     = params_pkg::DATA_WIDTH
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic valid_i,
  input  logic [REGISTER_WIDTH-1:0] wr_reg_i,
  input  logic [DATA_WIDTH-1:0] a_i,
  input  logic [DATA_WIDTH-1:0] b_i,
`ifndef SYNTHESIS
  input  logic [ADDR_WIDTH-1:0] debug_pc_i,
  input  instruction_t debug_instr_i,
`endif
  output logic result_ready_o,
  output logic [REGISTER_WIDTH-1:0] wr_reg_o,
  output logic [DATA_WIDTH-1:0] result_o,
`ifndef SYNTHESIS
  output logic [ADDR_WIDTH-1:0] debug_pc_o,
  output instruction_t debug_instr_o
`endif
);

  logic pipe1_valid, pipe2_valid, pipe3_valid, pipe4_valid, pipe5_valid;
  logic [REGISTER_WIDTH-1:0] pipe1_wr_reg, pipe2_wr_reg, pipe3_wr_reg, pipe4_wr_reg, pipe5_wr_reg;
  logic [DATA_WIDTH-1:0] pipe1_result, pipe2_result, pipe3_result, pipe4_result, pipe5_result;

`ifndef SYNTHESIS
  logic [ADDR_WIDTH-1:0] pipe1_debug_pc, pipe2_debug_pc, pipe3_debug_pc, pipe4_debug_pc,
                         pipe5_debug_pc;
  instruction_t pipe1_debug_instr, pipe2_debug_instr, pipe3_debug_instr, pipe4_debug_instr,
                pipe5_debug_instr;
`endif

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      pipe1_valid <= 1'b0;
      pipe2_valid <= 1'b0;
      pipe3_valid <= 1'b0;
      pipe4_valid <= 1'b0;
      pipe5_valid <= 1'b0;
    end else begin
      pipe1_valid  <= valid_i;
      pipe2_valid  <= pipe1_valid;
      pipe3_valid  <= pipe2_valid;
      pipe4_valid  <= pipe3_valid;
      pipe5_valid  <= pipe4_valid;
      pipe1_wr_reg <= wr_reg_i;
      pipe2_wr_reg <= pipe1_wr_reg;
      pipe3_wr_reg <= pipe2_wr_reg;
      pipe4_wr_reg <= pipe3_wr_reg;
      pipe5_wr_reg <= pipe4_wr_reg;
      pipe1_result <= a_i * b_i;
      pipe2_result <= pipe1_result;
      pipe3_result <= pipe2_result;
      pipe4_result <= pipe3_result;
      pipe5_result <= pipe4_result;
`ifndef SYNTHESIS
      pipe1_debug_pc    <= debug_pc_i;
      pipe2_debug_pc    <= pipe1_debug_pc;
      pipe3_debug_pc    <= pipe2_debug_pc;
      pipe4_debug_pc    <= pipe3_debug_pc;
      pipe5_debug_pc    <= pipe4_debug_pc;
      pipe1_debug_instr <= debug_instr_i;
      pipe2_debug_instr <= pipe1_debug_instr;
      pipe3_debug_instr <= pipe2_debug_instr;
      pipe4_debug_instr <= pipe3_debug_instr;
      pipe5_debug_instr <= pipe4_debug_instr;
`endif
    end
  end

  assign result_ready_o = pipe5_valid;
  assign wr_reg_o       = pipe5_wr_reg;
  assign result_o       = pipe5_result;

`ifndef SYNTHESIS
  assign debug_pc_o    = pipe5_debug_pc;
  assign debug_instr_o = pipe5_debug_instr;
`endif

endmodule : ex_stages
