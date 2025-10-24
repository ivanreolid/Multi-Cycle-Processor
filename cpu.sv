`include "fetch_stage.sv"
`include "decode_stage.sv"
`include "alu_stage.sv"
`include "mem_stage.sv"
`include "wb_stage.sv"
`include "rbank.sv"

import params_pkg::*;

module cpu (
  input  logic clk_i,
  input  logic rst_i,
  input  logic imem_instr_valid_i,
  input  instruction_t imem_instr_i,
  output logic imem_req_valid_o,
  output logic [ADDR_WIDTH-1:0] pc_o,
`ifndef SYNTHESIS
  output logic debug_store_is_completed_o,
  output logic debug_non_store_is_completed_o,
  output logic [DATA_WIDTH-1:0] debug_dmem_o [0:MEM_SIZE-1],
  output logic [DATA_WIDTH-1:0] debug_regs_o [0:31],
  output logic [ADDR_WIDTH-1:0] debug_mem_pc_o,
  output logic [ADDR_WIDTH-1:0] debug_wb_pc_o,
  output instruction_t debug_mem_instr_o,
  output instruction_t debug_wb_instr_o
`endif
);

  // Fetch stage wires
  logic dec_valid_d;
  logic [ADDR_WIDTH-1:0] dec_pc_d;
  instruction_t instruction_d;

  // Decode stage wires
  logic [ADDR_WIDTH-1:0] dec_pc;
  logic [DATA_WIDTH-1:0] dec_offset_sign_extend;
  logic [REGISTER_WIDTH-1:0] reg_a;
  logic [REGISTER_WIDTH-1:0] dec_wr_reg;
  logic [OPCODE_WIDTH-1:0] dec_instr_opcode;
  logic [ADDR_WIDTH-1:0] dec_branch_offset;
  logic [DATA_WIDTH-1:0] dec_reg_a_data, dec_reg_b_data;
  logic dec_reg_wr_en;
  logic dec_valid, alu_valid_d;
  instruction_t instruction_q;

  // ALU stage wires
  logic [ADDR_WIDTH-1:0] alu_pc;
  logic [DATA_WIDTH-1:0] alu_reg_a_data, alu_reg_b_data;
  logic [ADDR_WIDTH-1:0] alu_branch_offset;
  logic [DATA_WIDTH-1:0] alu_offset_sign_extend;
  logic [OPCODE_WIDTH-1:0] alu_instr_opcode;
  logic [ADDR_WIDTH-1:0] alu_pc_branch_offset;
  logic [DATA_WIDTH-1:0] alu_alu_result;
  logic [ADDR_WIDTH-1:0] jump_address;
  logic [REGISTER_WIDTH-1:0] alu_wr_reg;
  logic alu_reg_wr_en;
  logic alu_is_load, alu_is_store, is_jump;
  logic alu_branch_taken;
  logic alu_valid, mem_valid_d;
`ifndef SYNTHESIS
  instruction_t debug_alu_instr;
`endif

  // Mem stage wires
  logic [DATA_WIDTH-1:0] mem_alu_result;
  logic [DATA_WIDTH-1:0] mem_reg_a_data;
  logic [DATA_WIDTH-1:0] mem_data_from_mem;
  logic [REGISTER_WIDTH-1:0] mem_wr_reg;
  logic mem_reg_wr_en;
  logic mem_is_load, mem_is_store;
  logic mem_branch_taken;
  logic mem_valid, wb_valid_d;
`ifndef SYNTHESIS
  logic [ADDR_WIDTH-1:0] debug_mem_pc;
  logic debug_store_is_completed;
  instruction_t debug_mem_instr;
`endif

  // WB stage wires
  logic [DATA_WIDTH-1:0] wb_alu_result;
  logic [DATA_WIDTH-1:0] wb_data_from_mem;
  logic [DATA_WIDTH-1:0] data_to_reg;
  logic [REGISTER_WIDTH-1:0] wb_wr_reg;
  logic wb_is_load;
  logic wb_reg_wr_en;
  logic wb_valid;
`ifndef SYNTHESIS
  logic [ADDR_WIDTH-1:0] debug_wb_pc;
  logic debug_non_store_is_completed;
  logic wb_is_store;
  instruction_t debug_wb_instr;
`endif

  fetch_stage #(
    .ADDR_WIDTH         (ADDR_WIDTH),
    .DATA_WIDTH         (DATA_WIDTH),
    .MEM_SIZE           (MEM_SIZE)
  ) fetch_stage (
    .clk_i              (clk_i),
    .rst_i              (rst_i),
    .alu_branch_taken_i (alu_branch_taken),
    .is_jump_i          (is_jump),
    .pc_branch_offset_i (alu_pc_branch_offset),
    .jump_address_i     (jump_address),
    .instr_valid_i      (imem_instr_valid_i),
    .instr_i            (imem_instr_i),
    .imem_req_valid_o   (imem_req_valid_o),
    .dec_valid_o        (dec_valid_d),
    .imem_req_pc_o      (pc_o),
    .dec_pc_o           (dec_pc_d),
    .instruction_o      (instruction_d)
  );

  decode_stage #(
    .DATA_WIDTH           (DATA_WIDTH),
    .ADDR_WIDTH           (ADDR_WIDTH),
    .REGISTER_WIDTH       (REGISTER_WIDTH),
    .OPCODE_WIDTH         (OPCODE_WIDTH)
  ) decode_stage (
    .valid_i              (dec_valid),
    .is_jump_i            (is_jump),
    .branch_taken_i       (alu_branch_taken),
    .instruction_i        (instruction_q),
    .alu_valid_o          (alu_valid_d),
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
    .valid_i              (alu_valid),
    .mem_valid_o          (mem_valid_d),
    .pc_branch_offset_o   (alu_pc_branch_offset),
    .jump_address_o       (jump_address),
    .alu_result_o         (alu_alu_result),
    .is_load_o            (alu_is_load),
    .is_store_o           (alu_is_store),
    .branch_taken_o       (alu_branch_taken),
    .is_jump_o            (is_jump)
  );

  mem_stage #(
    .MEM_SIZE                   (MEM_SIZE),
    .ADDR_WIDTH                 (ADDR_WIDTH),
    .DATA_WIDTH                 (DATA_WIDTH)
  ) mem_stage (
    .clk_i                      (clk_i),
    .rst_i                      (rst_i),
    .alu_result_i               (mem_alu_result),
    .reg_a_data_i               (mem_reg_a_data),
    .valid_i                    (mem_valid),
    .is_store_i                 (mem_is_store),
    .wb_valid_o                 (wb_valid_d),
    .data_from_mem_o            (mem_data_from_mem),
`ifndef SYNTHESIS
    .debug_store_is_completed_o (debug_store_is_completed),
    .debug_dmem_o               (debug_dmem_o)
`endif
  );

  wb_stage #(
    .DATA_WIDTH                     (DATA_WIDTH)
  ) wb_stage (
    .alu_result_i                   (wb_alu_result),
    .data_from_mem_i                (wb_data_from_mem),
    .is_load_i                      (wb_is_load),
    .data_to_reg_o                  (data_to_reg),
`ifndef SYNTHESIS
    .debug_valid_i                  (wb_valid),
    .debug_is_store_i               (wb_is_store),
    .debug_non_store_is_completed_o (debug_non_store_is_completed)
`endif
  );

  rbank #(
    .REGISTER_WIDTH (REGISTER_WIDTH),
    .DATA_WIDTH     (DATA_WIDTH)
   ) rbank (
    .clk_i        (clk_i),
    .rst_i        (rst_i),
    .wr_en_i      (wb_reg_wr_en & wb_valid),
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
      dec_valid         <= 1'b0;
      is_jump           <= 1'b0;
      alu_valid         <= 1'b0;
      alu_branch_taken  <= 1'b0;
      alu_branch_offset <= 0;
      alu_is_load       <= 1'b0;
      alu_is_store      <= 1'b0;
      alu_reg_wr_en     <= 1'b0;
      mem_valid         <= 1'b0;
      mem_is_load       <= 1'b0;
      mem_is_store      <= 1'b0;
      wb_valid          <= 1'b0;
    end else begin
      instruction_q          <= instruction_d;
      dec_valid              <= dec_valid_d;
      dec_pc                 <= dec_pc_d;
      alu_valid              <= alu_valid_d;
      alu_pc                 <= dec_pc;
      alu_reg_wr_en          <= dec_reg_wr_en;
      alu_wr_reg             <= dec_wr_reg;
      alu_reg_a_data         <= dec_reg_a_data;
      alu_reg_b_data         <= dec_reg_b_data;
      alu_offset_sign_extend <= dec_offset_sign_extend;
      alu_instr_opcode       <= dec_instr_opcode;
      alu_branch_offset      <= dec_branch_offset;
      mem_valid              <= mem_valid_d;
      mem_reg_wr_en          <= alu_reg_wr_en;
      mem_wr_reg             <= alu_wr_reg;
      mem_reg_a_data         <= alu_reg_a_data;
      mem_is_load            <= alu_is_load;
      mem_is_store           <= alu_is_store;
      mem_alu_result         <= alu_alu_result;
      wb_valid               <= wb_valid_d;
      wb_reg_wr_en           <= mem_reg_wr_en;
      wb_wr_reg              <= mem_wr_reg;
      wb_is_load             <= mem_is_load;
      wb_alu_result          <= mem_alu_result;
      wb_data_from_mem       <= mem_data_from_mem;
`ifndef SYNTHESIS
      debug_alu_instr        <= instruction_q;
      debug_mem_instr        <= debug_alu_instr;
      debug_mem_pc           <= alu_pc;
      debug_mem_pc_o         <= debug_mem_pc;
      debug_mem_instr_o      <= debug_mem_instr;
      debug_non_store_is_completed_o <= debug_non_store_is_completed;
      debug_store_is_completed_o <= debug_store_is_completed;
      debug_wb_instr         <= debug_mem_instr;
      debug_wb_pc            <= debug_mem_pc;
      debug_wb_pc_o          <= debug_wb_pc;
      debug_wb_instr_o       <= debug_wb_instr;
      wb_is_store            <= mem_is_store;
`endif
    end
  end

endmodule
