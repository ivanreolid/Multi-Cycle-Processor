`include "dmem.sv"

import params_pkg::*;

module mem_stage #(
  parameter int MEM_SIZE       = params_pkg::MEM_SIZE,
  parameter int ADDR_WIDTH     = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH     = params_pkg::DATA_WIDTH,
  parameter int REGISTER_WIDTH = params_pkg::REGISTER_WIDTH
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic [DATA_WIDTH-1:0] alu_result_i,
  input  logic [DATA_WIDTH-1:0] reg_a_data_i,
  input  logic [REGISTER_WIDTH-1:0] wr_reg_i,
  input  logic valid_i,
  input  logic is_store_i,
  input  logic is_load_i,
  input  logic reg_wr_en_i,
  output logic wb_valid_o,
  output logic wb_reg_wr_en_o,
  output logic wb_is_load_o,
  output logic [REGISTER_WIDTH-1:0] wb_wr_reg_o,
  output logic [DATA_WIDTH-1:0] wb_data_from_mem_o,
  output logic [DATA_WIDTH-1:0] wb_alu_result_o,
`ifndef SYNTHESIS
  output logic debug_store_is_completed_o,
  output logic [DATA_WIDTH-1:0] debug_dmem_o [MEM_SIZE]
`endif
);

  logic wr_en;

  logic [DATA_WIDTH-1:0] wb_data_from_mem_d;

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      wb_valid_o     <= 1'b0;
      wb_reg_wr_en_o <= 1'b0;
      wb_is_load_o   <= 1'b0;
    end else begin
      wb_valid_o         <= valid_i;
      wb_reg_wr_en_o     <= reg_wr_en_i;
      wb_is_load_o       <= is_load_i;
      wb_wr_reg_o        <= wr_reg_i;
      wb_alu_result_o    <= alu_result_i;
      wb_data_from_mem_o <= wb_data_from_mem_d;
    end
  end

  assign wr_en = is_store_i & valid_i;

  dmem #(
    .MEM_SIZE   (MEM_SIZE  ),
    .ADDR_WIDTH (ADDR_WIDTH),
    .DATA_WIDTH (DATA_WIDTH)
  ) dmem (
    .clk_i     (clk_i),
    .rst_i     (rst_i),
    .wr_en_i   (wr_en),
    .addr_i    (alu_result_i),
    .wr_data_i (reg_a_data_i),
    .rd_data_o (wb_data_from_mem_d),
`ifndef SYNTHESIS
    .debug_dmem_o (debug_dmem_o)
`endif
  );

`ifndef SYNTHESIS
  assign debug_store_is_completed_o = wr_en;
`endif

endmodule : mem_stage
