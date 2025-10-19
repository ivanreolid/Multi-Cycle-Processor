`include "fetch_stage.sv"
`include "decode_stage.sv"
`include "alu_stage.sv"
`include "rbank.sv"
`include "dmem.sv"
`include "control.sv"

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
  logic [ADDR_WIDTH-1:0] pc_d, pc_q;
  logic [ADDR_WIDTH-1:0] pc_offset, pc_added;
  logic is_zero, is_less;
  logic branch_taken;

  // Instruction wires
  instruction_t instruction_d, instruction_q;
  logic [DATA_WIDTH-1:0] dec_offset_sign_extend, alu_offset_sign_extend;
  logic is_jump;
  logic alu_is_load, mem_is_load;
  logic alu_is_store, mem_is_store;

  logic [OPCODE_WIDTH-1:0] dec_instr_opcode, alu_instr_opcode;

  // ALU wires
  logic [DATA_WIDTH-1:0] dec_reg_a_data, alu_reg_a_data, mem_reg_a_data;
  logic [DATA_WIDTH-1:0] dec_reg_b_data, alu_reg_b_data;
  logic [DATA_WIDTH-1:0] data_a_to_alu;
  logic [DATA_WIDTH-1:0] alu_alu_result, mem_alu_result;

  logic [DATA_WIDTH-1:0] data_from_mem;
  logic [DATA_WIDTH-1:0] data_to_reg;

  logic [REGISTER_WIDTH-1:0] reg_a;
  
  logic dec_reg_wr_en, alu_reg_wr_en;
  logic [REGISTER_WIDTH-1:0] dec_wr_reg, alu_wr_reg;

  //assign pc_offset = branch_taken ? {instruction.free, instruction.rd} : {{ADDR_WIDTH-1{1'b0}}, 1'b1};
  //assign pc_added = (pc_q + pc_offset) % MEM_SIZE;
  //assign pc_d = is_jump ? reg_a_data : pc_added;

  // Only loads write data from memory into the register bank
  // assign data_to_reg = is_load ? data_from_mem : result_alu;

  fetch_stage #(
    .ADDR_WIDTH (ADDR_WIDTH),
    .DATA_WIDTH (DATA_WIDTH),
    .MEM_SIZE   (MEM_SIZE)
  ) fetch_stage (
    .pc_i           (pc_q),
    .next_pc_o      (pc_d),
    .instruction_o  (instruction_d)
  );

  decode_stage #(
    .DATA_WIDTH           (DATA_WIDTH),
    .REGISTER_WIDTH       (REGISTER_WIDTH),
    .OPCODE_WIDTH         (OPCODE_WIDTH)
  ) decode_stage (
    .instruction_i        (instruction_q),
    .offset_sign_extend_o (dec_offset_sign_extend),
    .reg_a_o              (reg_a),
    .reg_wr_en_o          (dec_reg_wr_en),
    .wr_reg_o             (dec_wr_reg),
    .instr_opcode_o       (dec_instr_opcode)
  );

  alu_stage #(
    .DATA_WIDTH           (DATA_WIDTH),
    .OPCODE_WIDTH         (OPCODE_WIDTH)
  ) alu_stage (
    .data_a_i             (alu_reg_a_data),
    .data_b_i             (alu_reg_b_data),
    .offset_sign_extend_i (alu_offset_sign_extend),
    .instr_opcode_i       (alu_instr_opcode),
    .alu_result_o         (alu_alu_result),
    .is_load_o            (alu_is_load),
    .is_store_o           (alu_is_store),
    .is_zero_o            (is_zero),
    .is_less_o            (is_less)
  );

  control #(
    .OPCODE_WIDTH (OPCODE_WIDTH)
  ) control (
    .opcode_i       (instruction_q.opcode),
    .is_zero_i      (is_zero),
    .is_less_i      (is_less),
    .is_load_o      (is_load),
    .is_store_o     (is_store),
    .is_jump_o      (is_jump),
    .branch_taken_o (branch_taken),
    .reg_wr_en_o    (reg_wr_en)
  );

  rbank #(
    .REGISTER_WIDTH (REGISTER_WIDTH),
    .DATA_WIDTH     (DATA_WIDTH)
   ) rbank (
    .clk_i        (clk_i),
    .rst_i        (rst_i),
    .wr_en_i      (reg_wr_en),
    .rd_reg_a_i   (reg_a),
    .rd_reg_b_i   (instruction_q.rb),
    .wr_reg_i     (instruction_q.rd),
    .wr_data_i    (data_to_reg),
    .reg_a_data_o (dec_reg_a_data),
    .reg_b_data_o (dec_reg_b_data),
`ifndef SYNTHESIS
    .debug_regs_o (debug_regs_o)
`endif
  );

  dmem #(
    .MEM_SIZE   (MEM_SIZE  ),
    .ADDR_WIDTH (ADDR_WIDTH),
    .DATA_WIDTH (DATA_WIDTH)
  ) dmem (
    .clk_i     (clk_i),
    .rst_i     (rst_i),
    .wr_en_i   (is_store),
    .addr_i    (result_alu),
    .wr_data_i (reg_a_data),
    .rd_data_o (data_from_mem),
`ifndef SYNTHESIS
    .debug_dmem_o (debug_dmem_o)
`endif
  );

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      pc_q          <= 0;
      alu_is_load   <= 1'b0;
      alu_is_store  <= 1'b0;
      alu_reg_wr_en <= 1'b0;
      mem_is_load   <= 1'b0;
      mem_is_store  <= 1'b0;
    end else begin
      pc_q                   <= pc_d;
      instruction_q          <= instruction_d;
      alu_reg_wr_en          <= dec_reg_wr_en;
      alu_wr_reg             <= dec_wr_reg;
      alu_reg_a_data         <= dec_reg_a_data;
      alu_reg_b_data         <= dec_reg_b_data;
      alu_offset_sign_extend <= dec_offset_sign_extend;
      alu_instr_opcode       <= dec_instr_opcode;
      mem_reg_a_data         <= alu_reg_a_data;
      mem_is_load            <= alu_is_load;
      mem_is_store           <= alu_is_store;
      mem_alu_result         <= alu_alu_result;
`ifndef SYNTHESIS
      debug_pc_committed_o       <= pc_q;
      debug_instr_is_committed_o <= 1'b1;
      debug_instr_committed_o    <= instruction_q;
`endif
    end
  end

endmodule
