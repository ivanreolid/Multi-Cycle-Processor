`include "alu.sv"

import params_pkg::*;

module alu_stage #(
  parameter int SHAMT_WIDTH    = params_pkg::SHAMT_WIDTH,
  parameter int DATA_WIDTH     = params_pkg::DATA_WIDTH,
  parameter int OPCODE_WIDTH   = params_pkg::OPCODE_WIDTH,
  parameter int REGISTER_WIDTH = params_pkg::REGISTER_WIDTH
)(
  input  logic valid_i,
  input  logic mem_stall_i,
  input  logic [SHAMT_WIDTH-1:0] shamt_i,
  input  logic [DATA_WIDTH-1:0] data_a_i,
  input  logic [DATA_WIDTH-1:0] data_b_i,
  input  logic [ADDR_WIDTH-1:0] pc_i,
  input  logic [DATA_WIDTH-1:0] offset_sign_extend_i,
  input  var instruction_t instruction_i,
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
  output var instruction_t debug_mem_instr_o
`endif
);

  logic [DATA_WIDTH-1:0] alu_data_a, alu_data_b;

  logic is_zero, is_less;

  always_comb begin : opcode
    mem_valid_o       = 1'b0;
    mem_is_load_o     = 1'b0;
    mem_is_store_o    = 1'b0;
    mem_reg_wr_en_o   = 1'b0;
    mem_access_size_o = WORD;
    is_jump_o         = 1'b0;
    branch_taken_o    = 1'b0;
    is_instr_wbalu_o  = 1'b0;
    instr_finishes_o  = 1'b0;
    alu_data_a        = data_a_i;
    alu_data_b        = data_b_i;

    case(instruction_i.opcode)
      R: begin
        is_instr_wbalu_o = valid_i;
        instr_finishes_o = valid_i;
      end
      LOAD: begin
        mem_valid_o       = valid_i;
        mem_is_load_o     = 1'b1;
        mem_reg_wr_en_o   = 1'b1;
        alu_data_b        = offset_sign_extend_i;
        mem_access_size_o = instruction_i.funct3 == 3'b000 ? BYTE : WORD;
      end
      STORE: begin
        mem_valid_o       = valid_i;
        mem_is_store_o    = 1'b1;
        alu_data_b        = offset_sign_extend_i;
        mem_access_size_o = instruction_i.funct3 == 3'b000 ? BYTE : WORD;
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
      IMMEDIATE: begin
        is_instr_wbalu_o = valid_i;
        instr_finishes_o = valid_i;
        case (instruction_i.funct3)
          3'b001  : alu_data_b = shamt_i;  // SLLI
          default : alu_data_b = offset_sign_extend_i;
        endcase
      end
      AUIPC: begin
        is_instr_wbalu_o = valid_i;
        instr_finishes_o = valid_i;
        alu_data_a       = pc_i;
        alu_data_b       = offset_sign_extend_i;
      end
    endcase
  end

  assign mem_rs2_data_o    = data_b_i;
  assign mem_wr_reg_o      = instruction_i.rd;
  assign mem_alu_result_o  = data_to_reg_o;
`ifndef SYNTHESIS
  assign debug_mem_pc_o    = debug_pc_i;
  assign debug_mem_instr_o = instruction_i;
`endif

  assign pc_branch_offset_o = pc_i + offset_sign_extend_i;
  assign jump_address_o     = pc_i + offset_sign_extend_i;

  assign wr_reg_o      = instruction_i.rd;

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
    .result_o  (data_to_reg_o)
  );

endmodule : alu_stage
