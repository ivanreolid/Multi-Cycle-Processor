`include "dmem.sv"

import params_pkg::*;

module mem_stage #(
  parameter int MEM_SIZE   = params_pkg::MEM_SIZE,
  parameter int ADDR_WIDTH = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH = params_pkg::DATA_WIDTH
)(
  input logic clk_i,
  input logic rst_i,
  input logic [DATA_WIDTH-1:0] alu_result_i,
  input logic [DATA_WIDTH-1:0] reg_a_data_i,
  input logic is_store_i,
  output logic [DATA_WIDTH-1:0] data_from_mem_o
);

  dmem #(
    .MEM_SIZE   (MEM_SIZE  ),
    .ADDR_WIDTH (ADDR_WIDTH),
    .DATA_WIDTH (DATA_WIDTH)
  ) dmem (
    .clk_i     (clk_i),
    .rst_i     (rst_i),
    .wr_en_i   (is_store_i),
    .addr_i    (alu_result_i),
    .wr_data_i (reg_a_data_i),
    .rd_data_o (data_from_mem_o)
/*`ifndef SYNTHESIS
    .debug_dmem_o (debug_dmem_o)
`endif*/
  );

endmodule : mem_stage
