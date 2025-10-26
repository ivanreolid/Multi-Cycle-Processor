`include "imem.sv"

import params_pkg::*;

module fetch_stage #(
  parameter int ADDR_WIDTH = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH = params_pkg::DATA_WIDTH,
  parameter int MEM_SIZE   = params_pkg::MEM_SIZE
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic alu_branch_taken_i,
  input  logic is_jump_i,
  input  logic [ADDR_WIDTH-1:0] pc_branch_offset_i,
  input  logic [ADDR_WIDTH-1:0] jump_address_i,
  output logic dec_valid_o,
  output logic [ADDR_WIDTH-1:0] pc_o,
  output instruction_t instruction_o
);

  logic [ADDR_WIDTH-1:0] pc, pc_d;
  logic [ADDR_WIDTH-1:0] pc_added;

  imem #(
    .MEM_SIZE      (MEM_SIZE),
    .ADDR_WIDTH    (ADDR_WIDTH),
    .DATA_WIDTH    (INSTR_WIDTH)
  ) imem (
    .address_i     (pc),
    .instruction_o (instruction_o)
  );

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      pc <= 1;
    end else begin
      pc <= pc_d;
    end
  end

  assign pc_added = (alu_branch_taken_i ? pc_branch_offset_i : pc + 1) % MEM_SIZE;
  assign pc_d = is_jump_i ? jump_address_i : pc_added;

  assign dec_valid_o = ~is_jump_i & ~alu_branch_taken_i;
  assign pc_o = pc;

endmodule : fetch_stage
