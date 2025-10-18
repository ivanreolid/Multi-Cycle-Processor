`include "imem.sv"

import params_pkg::*;

module fetch_stage #(
  parameter int ADDR_WIDTH = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH = params_pkg::DATA_WIDTH,
  parameter int MEM_SIZE   = params_pkg::MEM_SIZE
)(
  input  logic [ADDR_WIDTH-1:0] pc_i,
  output logic [ADDR_WIDTH-1:0] next_pc_o,
  output instruction_t instruction_o
);

  imem #(
    .MEM_SIZE      (MEM_SIZE),
    .ADDR_WIDTH    (ADDR_WIDTH),
    .DATA_WIDTH    (INSTR_WIDTH)
  ) imem (
    .address_i     (pc_i),
    .instruction_o (instruction_o)
  );

  assign next_pc_o = (pc_i + 1) % MEM_SIZE;

endmodule : fetch_stage
