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
  input  logic mem_stall_i,
  input  logic [DATA_WIDTH-1:0] data_a_i,
  input  logic [DATA_WIDTH-1:0] data_b_i,
  input  logic [ADDR_WIDTH-1:0] pc_i,
  input  logic [DATA_WIDTH-1:0] offset_sign_extend_i,
  input  instruction_t instruction_i,
`ifndef SYNTHESIS
  input  logic [ADDR_WIDTH-1:0] debug_pc_i,
`endif
  output logic mem_valid_o,
  output logic mem_reg_wr_en_o,
  output logic is_instr_wbalu_o,
  output logic instr_finishes_o,
  output logic [REGISTER_WIDTH-1:0] wr_reg_o,
  output logic [ADDR_WIDTH-1:0] pc_branch_offset_o,
  output logic [ADDR_WIDTH-1:0] jump_address_o,
  output logic [DATA_WIDTH-1:0] mem_alu_result_o,
  output logic [DATA_WIDTH-1:0] mem_rs2_data_o,
  output logic [REGISTER_WIDTH-1:0] mem_wr_reg_o,
  output logic [DATA_WIDTH-1:0] data_to_reg_o,
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

  logic [DATA_WIDTH-1:0] alu_data_a, alu_data_b;
  logic [DATA_WIDTH-1:0] mem_alu_result_d;

  logic is_zero, is_less;
  logic is_load_d, is_store_d;
  logic mem_valid_d, mem_reg_wr_en_d;

  access_size_t access_size_d;

  always_comb begin : opcode
    is_load_d        = 1'b0;
    is_store_d       = 1'b0;
    is_jump_o        = 1'b0;
    mem_valid_d      = 1'b0;
    mem_reg_wr_en_d  = 1'b0;
    branch_taken_o   = 1'b0;
    is_instr_wbalu_o = 1'b0;
    instr_finishes_o = 1'b0;
    alu_data_a       = data_a_i;
    alu_data_b       = data_b_i;

    case(instruction_i.opcode)
      R: begin
        is_instr_wbalu_o = valid_i;
        instr_finishes_o = valid_i;
      end
      LOAD: begin
        is_load_d        = 1'b1;
        alu_data_b       = offset_sign_extend_i;
        access_size_d    = instruction_i.funct3 == 3'b000 ? BYTE : WORD;
        mem_valid_d      = valid_i;
        mem_reg_wr_en_d  = 1'b1;
      end
      STORE: begin
        is_store_d       = 1'b1;
        alu_data_b       = offset_sign_extend_i;
        mem_valid_d      = valid_i;
        access_size_d    = instruction_i.funct3 == 3'b000 ? BYTE : WORD;
      end
      BRANCH: begin
        if (valid_i) begin
          instr_finishes_o = 1'b1;
          if (instruction_i.funct3 == 3'b000 & is_zero)       // BEQ
            branch_taken_o = 1'b1;
          else if (instruction_i.funct3 == 3'b001 & ~is_zero) // BNE
            branch_taken_o = 1'b1;
          else if (instruction_i.funct3 == 3'b100 & is_less)  // BLT
            branch_taken_o = 1'b1;
          else if (instruction_i.funct3 == 3'b101 & ~is_less) // BGE
            branch_taken_o = 1'b1;
        end
      end
      JAL: begin
        is_jump_o        = valid_i;
        is_instr_wbalu_o = valid_i;
        instr_finishes_o = valid_i;
        alu_data_a       = pc_i;
        alu_data_b       = 4;
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
      mem_valid_o       <= mem_valid_d;
      mem_reg_wr_en_o   <= mem_reg_wr_en_d;
      mem_is_load_o     <= is_load_d;
      mem_is_store_o    <= is_store_d;
      mem_rs2_data_o    <= data_b_i;
      mem_wr_reg_o      <= instruction_i.rd;
      mem_alu_result_o  <= mem_alu_result_d;
      mem_access_size_o <= access_size_d;
`ifndef SYNTHESIS
      debug_mem_pc_o    <= debug_pc_i;
      debug_mem_instr_o <= instruction_i;
`endif
    end
  end

  assign pc_branch_offset_o = pc_i + offset_sign_extend_i;
  assign jump_address_o     = pc_i + offset_sign_extend_i;

  assign wr_reg_o      = instruction_i.rd;
  assign data_to_reg_o = mem_alu_result_d;  // ALU result

  alu #(
    .OPCODE_WIDTH (OPCODE_WIDTH),
    .DATA_WIDTH   (DATA_WIDTH  )
  ) alu (
    .opcode_i  (instruction_i.opcode),
    .funct3_i  (instruction_i.funct3),
    .funct7_i  (instruction_i.funct7),
    .a_i       (alu_data_a),
    .b_i       (alu_data_b),
    .is_zero_o (is_zero),
    .is_less_o (is_less),
    .result_o  (mem_alu_result_d)
  );

endmodule : alu_stage
