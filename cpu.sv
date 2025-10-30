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
  input  logic mem_data_valid_i,
  input  logic mem_data_is_instr_i,
  input  logic [DATA_WIDTH-1:0] mem_data_i,
  output logic rd_req_valid_o,
  output logic wr_req_valid_o,
  output logic req_is_instr_o,
  output logic [ADDR_WIDTH-1:0] req_address_o,
  output logic [DATA_WIDTH-1:0] wr_data_o,
`ifndef SYNTHESIS
  output logic debug_instr_is_completed_o,
  output logic [DATA_WIDTH-1:0] debug_regs_o [32],
  output logic [ADDR_WIDTH-1:0] debug_pc_o,
  output instruction_t debug_instr_o
`endif
);

  // Fetch stage wires
  logic fetch_rd_req_valid;
  logic [ADDR_WIDTH-1:0] fetch_req_address;

  // Decode stage wires
  logic [ADDR_WIDTH-1:0] dec_pc;
  logic [REGISTER_WIDTH-1:0] reg_a;
  logic [DATA_WIDTH-1:0] dec_reg_a_data, dec_reg_b_data;
  logic dec_valid;
  instruction_t instruction_q;

  // ALU stage wires
  logic [ADDR_WIDTH-1:0] alu_pc;
  logic [DATA_WIDTH-1:0] alu_reg_a_data, alu_reg_b_data;
  logic [ADDR_WIDTH-1:0] alu_branch_offset;
  logic [DATA_WIDTH-1:0] alu_offset_sign_extend;
  logic [OPCODE_WIDTH-1:0] alu_instr_opcode;
  logic [ADDR_WIDTH-1:0] alu_pc_branch_offset;
  logic [ADDR_WIDTH-1:0] jump_address;
  logic [REGISTER_WIDTH-1:0] alu_wr_reg;
  logic alu_reg_wr_en;
  logic is_jump;
  logic alu_branch_taken;
  logic alu_valid;
`ifndef SYNTHESIS
  logic [ADDR_WIDTH-1:0] debug_alu_pc;
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
  logic mem_rd_req_valid, mem_wr_req_valid;
  logic mem_stall;
  logic [ADDR_WIDTH-1:0] mem_req_address;
`ifndef SYNTHESIS
  logic [ADDR_WIDTH-1:0] debug_mem_pc;
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
  instruction_t debug_wb_instr;
`endif

  fetch_stage #(
    .ADDR_WIDTH         (ADDR_WIDTH),
    .DATA_WIDTH         (DATA_WIDTH),
    .MEM_SIZE           (MEM_SIZE)
  ) fetch_stage (
    .clk_i               (clk_i),
    .rst_i               (rst_i),
    .mem_req_i           (mem_rd_req_valid | mem_wr_req_valid),
    .alu_branch_taken_i  (alu_branch_taken),
    .is_jump_i           (is_jump),
    .mem_stall_i         (mem_stall),
    .pc_branch_offset_i  (alu_pc_branch_offset),
    .jump_address_i      (jump_address),
    .instr_valid_i       (mem_data_valid_i & mem_data_is_instr_i),
    .instr_i             (mem_data_i),
    .rd_req_valid_o      (fetch_rd_req_valid),
    .dec_valid_o         (dec_valid),
    .mem_req_addr_o      (fetch_req_address),
    .dec_pc_o            (dec_pc),
    .instruction_o       (instruction_q)
  );

  decode_stage #(
    .DATA_WIDTH           (DATA_WIDTH),
    .ADDR_WIDTH           (ADDR_WIDTH),
    .REGISTER_WIDTH       (REGISTER_WIDTH),
    .OPCODE_WIDTH         (OPCODE_WIDTH)
  ) decode_stage (
    .clk_i                (clk_i),
    .rst_i                (rst_i),
    .valid_i              (dec_valid),
    .is_jump_i            (is_jump),
    .branch_taken_i       (alu_branch_taken),
    .mem_stall_i          (mem_stall),
    .pc_i                 (dec_pc),
    .reg_a_data_i         (dec_reg_a_data),
    .reg_b_data_i         (dec_reg_b_data),
    .instruction_i        (instruction_q),
    .alu_valid_o          (alu_valid),
    .offset_sign_extend_o (alu_offset_sign_extend),
    .reg_a_o              (reg_a),
    .reg_wr_en_o          (alu_reg_wr_en),
    .wr_reg_o             (alu_wr_reg),
    .instr_opcode_o       (alu_instr_opcode),
    .branch_offset_o      (alu_branch_offset),
    .alu_pc_o             (alu_pc),
    .alu_reg_a_data_o     (alu_reg_a_data),
    .alu_reg_b_data_o     (alu_reg_b_data),
`ifndef SYNTHESIS
    .debug_alu_pc_o       (debug_alu_pc),
    .debug_alu_instr_o    (debug_alu_instr)
`endif
  );

  alu_stage #(
    .DATA_WIDTH           (DATA_WIDTH),
    .OPCODE_WIDTH         (OPCODE_WIDTH)
  ) alu_stage (
    .clk_i                (clk_i),
    .rst_i                (rst_i),
    .valid_i              (alu_valid),
    .reg_wr_en_i          (alu_reg_wr_en),
    .mem_stall_i          (mem_stall),
    .data_a_i             (alu_reg_a_data),
    .data_b_i             (alu_reg_b_data),
    .pc_i                 (alu_pc),
    .branch_offset_i      (alu_branch_offset),
    .offset_sign_extend_i (alu_offset_sign_extend),
    .instr_opcode_i       (alu_instr_opcode),
    .wr_reg_i             (alu_wr_reg),
`ifndef SYNTHESIS
    .debug_pc_i           (debug_alu_pc),
    .debug_instr_i        (debug_alu_instr),
`endif
    .mem_valid_o          (mem_valid),
    .mem_reg_wr_en_o      (mem_reg_wr_en),
    .pc_branch_offset_o   (alu_pc_branch_offset),
    .jump_address_o       (jump_address),
    .mem_alu_result_o     (mem_alu_result),
    .mem_reg_a_data_o     (mem_reg_a_data),
    .mem_wr_reg_o         (mem_wr_reg),
    .mem_is_load_o        (mem_is_load),
    .mem_is_store_o       (mem_is_store),
    .branch_taken_o       (alu_branch_taken),
    .is_jump_o            (is_jump),
`ifndef SYNTHESIS
    .debug_mem_pc_o       (debug_mem_pc),
    .debug_mem_instr_o    (debug_mem_instr)
`endif
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
    .wr_reg_i                   (mem_wr_reg),
    .mem_data_i                 (mem_data_i),
    .valid_i                    (mem_valid),
    .is_load_i                  (mem_is_load),
    .is_store_i                 (mem_is_store),
    .reg_wr_en_i                (mem_reg_wr_en),
    .mem_data_is_valid_i        (mem_data_valid_i & ~mem_data_is_instr_i),
`ifndef SYNTHESIS
    .debug_pc_i                 (debug_mem_pc),
    .debug_instr_i              (debug_mem_instr),
`endif
    .wb_valid_o                 (wb_valid),
    .wb_reg_wr_en_o             (wb_reg_wr_en),
    .wb_is_load_o               (wb_is_load),
    .rd_req_valid_o             (mem_rd_req_valid),
    .wr_req_valid_o             (mem_wr_req_valid),
    .stall_o                    (mem_stall),
    .wb_wr_reg_o                (wb_wr_reg),
    .wb_data_from_mem_o         (wb_data_from_mem),
    .wb_alu_result_o            (wb_alu_result),
    .mem_req_address_o          (mem_req_address),
    .wr_data_o                  (wr_data_o),
`ifndef SYNTHESIS
    .debug_wb_pc_o              (debug_wb_pc),
    .debug_wb_instr_o           (debug_wb_instr)
`endif
  );

  wb_stage #(
    .DATA_WIDTH                     (DATA_WIDTH)
  ) wb_stage (
    .clk_i                          (clk_i),
    .rst_i                          (rst_i),
    .alu_result_i                   (wb_alu_result),
    .data_from_mem_i                (wb_data_from_mem),
    .is_load_i                      (wb_is_load),
`ifndef SYNTHESIS
    .debug_valid_i                  (wb_valid),
    .debug_pc_i                     (debug_wb_pc),
    .debug_instr_i                  (debug_wb_instr),
`endif
    .data_to_reg_o                  (data_to_reg),
`ifndef SYNTHESIS
    .debug_instr_is_completed_o     (debug_instr_is_completed_o),
    .debug_pc_o                     (debug_pc_o),
    .debug_instr_o                  (debug_instr_o)
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
      is_jump           <= 1'b0;
      alu_branch_taken  <= 1'b0;
      alu_branch_offset <= 0;
    end
  end

  assign rd_req_valid_o = fetch_rd_req_valid | mem_rd_req_valid;
  assign wr_req_valid_o = mem_wr_req_valid;
  assign req_is_instr_o = fetch_rd_req_valid;
  assign req_address_o  = fetch_rd_req_valid ? fetch_req_address : mem_req_address;

endmodule
