`include "alu.sv"

import params_pkg::*;

module alu_stage #(
  parameter int DATA_WIDTH     = params_pkg::DATA_WIDTH,
  parameter int OPCODE_WIDTH   = params_pkg::OPCODE_WIDTH,
  parameter int REGISTER_WIDTH = params_pkg::REGISTER_WIDTH
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic valid_i,
  input  logic reg_wr_en_i,
  input  logic mem_stall_i,
  input  logic [DATA_WIDTH-1:0] data_a_i,
  input  logic [DATA_WIDTH-1:0] data_b_i,
  input  logic [ADDR_WIDTH-1:0] pc_i,
  input  logic [ADDR_WIDTH-1:0] branch_offset_i,
  input  logic [DATA_WIDTH-1:0] offset_sign_extend_i,
  input  logic [OPCODE_WIDTH-1:0] instr_opcode_i,
  input  logic [REGISTER_WIDTH-1:0] wr_reg_i,
`ifndef SYNTHESIS
  input  logic [ADDR_WIDTH-1:0] debug_pc_i,
  input  instruction_t debug_instr_i,
`endif
  output logic mem_valid_o,
  output logic mem_reg_wr_en_o,
  output logic [ADDR_WIDTH-1:0] pc_branch_offset_o,
  output logic [ADDR_WIDTH-1:0] jump_address_o,
  output logic [DATA_WIDTH-1:0] mem_alu_result_o,
  output logic [DATA_WIDTH-1:0] mem_reg_a_data_o,
  output logic [REGISTER_WIDTH-1:0] mem_wr_reg_o,
  output logic mem_is_load_o,
  output logic mem_is_store_o,
  output logic branch_taken_o,
  output logic is_jump_o,
  output access_size_t mem_access_size_o,
`ifndef SYNTHESIS
  output logic [ADDR_WIDTH-1:0] debug_mem_pc_o,
  output instruction_t debug_mem_instr_o
`endif
);

  logic [DATA_WIDTH-1:0] data_a_to_alu;
  logic [DATA_WIDTH-1:0] mem_alu_result_d;

  logic is_zero, is_less;
  logic is_load_d, is_store_d;

  access_size_t access_size_d;

  always_comb begin : opcode
    is_load_d  = 1'b0;
    is_store_d = 1'b0;
    is_jump_o  = 1'b0;

    case(instr_opcode_i)
      LW: begin
        is_load_d     = 1'b1;
        access_size_d = WORD;
      end
      LB: begin
        is_load_d     = 1'b1;
        access_size_d = BYTE;
      end
      SW: begin
        is_store_d    = 1'b1;
        access_size_d = WORD;
      end
      SB: begin
        is_store_d    = 1'b1;
        access_size_d = BYTE;
      end
      JMP: begin
        is_jump_o = valid_i;
      end
    endcase
  end

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      mem_valid_o       <= 1'b0;
      mem_reg_wr_en_o   <= 1'b0;
      mem_is_load_o     <= 1'b0;
      mem_is_store_o    <= 1'b0;
    end else if (!mem_stall_i) begin
      mem_valid_o       <= valid_i;
      mem_reg_wr_en_o   <= reg_wr_en_i;
      mem_is_load_o     <= is_load_d;
      mem_is_store_o    <= is_store_d;
      mem_reg_a_data_o  <= data_a_i;
      mem_wr_reg_o      <= wr_reg_i;
      mem_alu_result_o  <= mem_alu_result_d;
      mem_access_size_o <= access_size_d;
`ifndef SYNTHESIS
      debug_mem_pc_o    <= debug_pc_i;
      debug_mem_instr_o <= debug_instr_i;
`endif
    end
  end

  assign branch_taken_o = valid_i & (((instr_opcode_i == BEQ) & is_zero) | ((instr_opcode_i == BNE) & ~is_zero) | ((instr_opcode_i == BLT) & is_less) | ((instr_opcode_i == BGE) & ~is_less));

  assign pc_branch_offset_o = pc_i + branch_offset_i;
  assign jump_address_o = data_a_i;

  assign data_a_to_alu = (is_load_d | is_store_d) ? offset_sign_extend_i : data_a_i;

  alu #(
    .OPCODE_WIDTH (OPCODE_WIDTH),
    .DATA_WIDTH   (DATA_WIDTH  )
  ) alu (
    .op_i      (instr_opcode_i),
    .a_i       (data_a_to_alu),
    .b_i       (data_b_i),
    .is_zero_o (is_zero),
    .is_less_o (is_less),
    .result_o  (mem_alu_result_d)
  );

endmodule : alu_stage
