`include "fetch_stage.sv"
`include "decode_stage.sv"
`include "alu_stage.sv"
`include "mem_stage.sv"
`include "wb_stage.sv"
`include "rbank.sv"

import params_pkg::*;

module cpu (
  input logic clk_i,
  input logic rst_i,
`ifndef SYNTHESIS
  output logic [DATA_WIDTH-1:0] debug_dmem_o [0:MEM_SIZE-1],
  output logic [DATA_WIDTH-1:0] debug_regs_o [0:31],
  output logic [ADDR_WIDTH-1:0] debug_pc_committed_o,
  output instruction_t debug_instr_committed_o,
  output logic debug_instr_is_committed_o
`endif
);

  // PC wires
  logic [ADDR_WIDTH-1:0] pc_d, pc_q, dec_pc, alu_pc;
  logic alu_branch_taken, mem_branch_taken;

  // Instruction wires
  instruction_t instruction_d, instruction_q;
  logic [DATA_WIDTH-1:0] dec_offset_sign_extend, alu_offset_sign_extend;
  logic is_jump;
  logic alu_is_load, mem_is_load, wb_is_load;
  logic alu_is_store, mem_is_store;

  logic [OPCODE_WIDTH-1:0] dec_instr_opcode, alu_instr_opcode;

  // ALU wires
  logic [DATA_WIDTH-1:0] dec_reg_a_data, alu_reg_a_data, mem_reg_a_data;
  logic [DATA_WIDTH-1:0] dec_reg_b_data, alu_reg_b_data;
  logic [DATA_WIDTH-1:0] data_a_to_alu;
  logic [DATA_WIDTH-1:0] alu_alu_result, mem_alu_result, wb_alu_result;
  logic [ADDR_WIDTH-1:0] dec_branch_offset, alu_branch_offset;
  logic [ADDR_WIDTH-1:0] alu_pc_branch_offset, mem_pc_branch_offset;
  logic [ADDR_WIDTH-1:0] jump_address;

  logic [DATA_WIDTH-1:0] mem_data_from_mem, wb_data_from_mem;
  logic [DATA_WIDTH-1:0] data_to_reg;

  logic [REGISTER_WIDTH-1:0] reg_a;
  
  logic dec_reg_wr_en, alu_reg_wr_en, mem_reg_wr_en, wb_reg_wr_en;
  logic [REGISTER_WIDTH-1:0] dec_wr_reg, alu_wr_reg, mem_wr_reg, wb_wr_reg;

  fetch_stage #(
    .ADDR_WIDTH         (ADDR_WIDTH),
    .DATA_WIDTH         (DATA_WIDTH),
    .MEM_SIZE           (MEM_SIZE)
  ) fetch_stage (
    .pc_i               (pc_q),
    .jump_address_i     (jump_address),
    .pc_branch_offset_i (mem_pc_branch_offset),
    .branch_taken_i     (mem_branch_taken),
    .is_jump_i          (is_jump),
    .next_pc_o          (pc_d),
    .instruction_o      (instruction_d)
  );

  decode_stage #(
    .DATA_WIDTH           (DATA_WIDTH),
    .ADDR_WIDTH           (ADDR_WIDTH),
    .REGISTER_WIDTH       (REGISTER_WIDTH),
    .OPCODE_WIDTH         (OPCODE_WIDTH)
  ) decode_stage (
    .instruction_i        (instruction_q),
    .offset_sign_extend_o (dec_offset_sign_extend),
    .reg_a_o              (reg_a),
    .reg_wr_en_o          (dec_reg_wr_en),
    .wr_reg_o             (dec_wr_reg),
    .instr_opcode_o       (dec_instr_opcode),
    .branch_offset_o      (dec_branch_offset)
  );

  alu_stage #(
    .DATA_WIDTH           (DATA_WIDTH),
    .OPCODE_WIDTH         (OPCODE_WIDTH)
  ) alu_stage (
    .data_a_i             (alu_reg_a_data),
    .data_b_i             (alu_reg_b_data),
    .pc_i                 (alu_pc),
    .branch_offset_i      (alu_branch_offset),
    .offset_sign_extend_i (alu_offset_sign_extend),
    .instr_opcode_i       (alu_instr_opcode),
    .pc_branch_offset_o   (alu_pc_branch_offset),
    .jump_address_o       (jump_address),
    .alu_result_o         (alu_alu_result),
    .is_load_o            (alu_is_load),
    .is_store_o           (alu_is_store),
    .branch_taken_o       (alu_branch_taken),
    .is_jump_o            (is_jump)
  );

  mem_stage #(
    .MEM_SIZE        (MEM_SIZE),
    .ADDR_WIDTH      (ADDR_WIDTH),
    .DATA_WIDTH      (DATA_WIDTH)
  ) mem_stage (
    .clk_i           (clk_i),
    .rst_i           (rst_i),
    .alu_result_i    (mem_alu_result),
    .reg_a_data_i    (mem_reg_a_data),
    .is_store_i      (mem_is_store),
    .data_from_mem_o (mem_data_from_mem)
  );

  wb_stage #(
    .DATA_WIDTH      (DATA_WIDTH)
  ) wb_stage (
    .alu_result_i    (wb_alu_result),
    .data_from_mem_i (wb_data_from_mem),
    .is_load_i       (wb_is_load),
    .data_to_reg_o   (data_to_reg) 
  );

  rbank #(
    .REGISTER_WIDTH (REGISTER_WIDTH),
    .DATA_WIDTH     (DATA_WIDTH)
   ) rbank (
    .clk_i        (clk_i),
    .rst_i        (rst_i),
    .wr_en_i      (wb_reg_wr_en),
    .rd_reg_a_i   (reg_a),
    .rd_reg_b_i   (instruction_q.rb),
    .wr_reg_i     (wb_wr_reg),
    .wr_data_i    (data_to_reg),
    .reg_a_data_o (dec_reg_a_data),
    .reg_b_data_o (dec_reg_b_data),
`ifndef SYNTHESIS
    .debug_regs_o (debug_regs_o)
`endif
  );

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      pc_q              <= 0;
      is_jump           <= 1'b0;
      alu_branch_taken  <= 1'b0;
      alu_branch_offset <= 0;
      alu_is_load       <= 1'b0;
      alu_is_store      <= 1'b0;
      alu_reg_wr_en     <= 1'b0;
      mem_branch_taken  <= 1'b0;
      mem_is_load       <= 1'b0;
      mem_is_store      <= 1'b0;
    end else begin
      pc_q                   <= pc_d;
      instruction_q          <= instruction_d;
      dec_pc                 <= pc_q;
      alu_pc                 <= dec_pc;
      alu_reg_wr_en          <= dec_reg_wr_en;
      alu_wr_reg             <= dec_wr_reg;
      alu_reg_a_data         <= dec_reg_a_data;
      alu_reg_b_data         <= dec_reg_b_data;
      alu_offset_sign_extend <= dec_offset_sign_extend;
      alu_instr_opcode       <= dec_instr_opcode;
      alu_branch_offset      <= dec_branch_offset;
      mem_pc_branch_offset   <= alu_pc_branch_offset;
      mem_branch_taken       <= alu_branch_taken;
      mem_reg_wr_en          <= alu_reg_wr_en;
      mem_wr_reg             <= alu_wr_reg;
      mem_reg_a_data         <= alu_reg_a_data;
      mem_is_load            <= alu_is_load;
      mem_is_store           <= alu_is_store;
      mem_alu_result         <= alu_alu_result;
      wb_reg_wr_en           <= mem_reg_wr_en;
      wb_wr_reg              <= mem_wr_reg;
      wb_is_load             <= mem_is_load;
      wb_alu_result          <= mem_alu_result;
      wb_data_from_mem       <= mem_data_from_mem;
`ifndef SYNTHESIS
      debug_pc_committed_o       <= pc_q;
      debug_instr_is_committed_o <= 1'b1;
      debug_instr_committed_o    <= instruction_q;
`endif
    end
  end

endmodule
