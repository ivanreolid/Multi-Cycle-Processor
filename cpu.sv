`include "fetch_stage.sv"
`include "decode_stage.sv"
`include "alu_stage.sv"
`include "mem_stage.sv"
`include "ex_stages.sv"
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
  output access_size_t req_access_size_o,
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
  access_size_t fetch_req_access_size;

  // Decode stage wires
  logic [ADDR_WIDTH-1:0] dec_pc;
  logic [REGISTER_WIDTH-1:0] rs1, rs2;
  logic [DATA_WIDTH-1:0] dec_rs1_data, dec_rs2_data;
  logic dec_valid;
  logic dec_stall;
  instruction_t instruction_q;

  // ALU stage wires
  logic [ADDR_WIDTH-1:0] alu_pc;
  logic [DATA_WIDTH-1:0] alu_rs1_data, alu_rs2_data;
  logic [DATA_WIDTH-1:0] alu_offset_sign_extend;
  logic [ADDR_WIDTH-1:0] alu_pc_branch_offset;
  logic [ADDR_WIDTH-1:0] jump_address;
  logic is_jump;
  logic alu_branch_taken;
  logic alu_valid;
  logic alu_is_instr_wbalu, alu_instr_finishes;
  logic [REGISTER_WIDTH-1:0] alu_wr_reg;
  logic [DATA_WIDTH-1:0] alu_data_to_reg;
  instruction_t alu_instruction;
`ifndef SYNTHESIS
  logic [ADDR_WIDTH-1:0] debug_alu_pc;
`endif

  // Mem stage wires
  logic [DATA_WIDTH-1:0] mem_alu_result;
  logic [DATA_WIDTH-1:0] mem_rs2_data;
  logic [DATA_WIDTH-1:0] mem_data_from_mem;
  logic [REGISTER_WIDTH-1:0] mem_wr_reg;
  logic mem_reg_wr_en;
  logic mem_is_load, mem_is_store;
  logic mem_branch_taken;
  logic mem_valid;
  logic mem_rd_req_valid, mem_wr_req_valid;
  logic mem_stall;
  logic mem_wb_is_next_cycle;
  logic [ADDR_WIDTH-1:0] mem_req_address;
  access_size_t mem_access_size, mem_req_access_size;
`ifndef SYNTHESIS
  logic [ADDR_WIDTH-1:0] debug_mem_pc;
  instruction_t debug_mem_instr;
`endif

  // EX1 stage wires
  logic ex_valid, ex2_valid, ex3_valid, ex4_valid;
  logic ex_wb_is_next_cycle;
  logic [REGISTER_WIDTH-1:0] ex_wr_reg, ex2_wr_reg, ex3_wr_reg, ex4_wr_reg;
  logic [DATA_WIDTH-1:0] ex_a, ex_b;
`ifndef SYNTHESIS
  logic [ADDR_WIDTH-1:0] debug_ex_pc;
  instruction_t debug_ex_instr;
`endif

  // WB stage wires
  logic wb_valid_from_mem, wb_valid_from_ex;
  logic wb_reg_wr_en_from_mem;
  logic wb_is_load_from_mem;
  logic [REGISTER_WIDTH-1:0] wb_wr_reg_from_mem, wb_wr_reg_from_ex;
  logic [REGISTER_WIDTH-1:0] wb_wr_reg;
  logic [DATA_WIDTH-1:0] wb_alu_result;
  logic [DATA_WIDTH-1:0] wb_data_from_mem, wb_data_from_ex;
  logic wb_is_load;
  logic wb_reg_wr_en;
  logic wb_valid;
  logic [DATA_WIDTH-1:0] wb_data_to_reg;
`ifndef SYNTHESIS
  logic [ADDR_WIDTH-1:0] debug_wb_pc_from_mem, debug_wb_pc_from_ex, debug_wb_pc;
  logic debug_non_store_is_completed;
  instruction_t debug_wb_instr_from_mem, debug_wb_instr_from_ex, debug_wb_instr;
`endif

  // Rbank wires coming from WB stage
  logic reg_wr_en;
  logic [REGISTER_WIDTH-1:0] wr_reg;
  logic [DATA_WIDTH-1:0] data_to_reg;

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
    .dec_stall_i         (dec_stall),
    .mem_stall_i         (mem_stall),
    .pc_branch_offset_i  (alu_pc_branch_offset),
    .jump_address_i      (jump_address),
    .instr_valid_i       (mem_data_valid_i & mem_data_is_instr_i),
    .instr_i             (mem_data_i),
    .rd_req_valid_o      (fetch_rd_req_valid),
    .dec_valid_o         (dec_valid),
    .mem_req_addr_o      (fetch_req_address),
    .dec_pc_o            (dec_pc),
    .req_access_size_o   (fetch_req_access_size),
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
    .alu_instr_finishes_i (alu_instr_finishes),
    .mem_stall_i          (mem_stall),
    .wb_is_next_cycle_i   (mem_wb_is_next_cycle | ex_wb_is_next_cycle),
    .wb_reg_wr_en_i       (reg_wr_en),
    .ex1_valid_i          (ex_valid),
    .ex2_valid_i          (ex2_valid),
    .ex3_valid_i          (ex3_valid),
    .ex4_valid_i          (ex4_valid),
    .wb_wr_reg_i          (wr_reg),
    .ex1_wr_reg_i         (ex_wr_reg),
    .ex2_wr_reg_i         (ex2_wr_reg),
    .ex3_wr_reg_i         (ex3_wr_reg),
    .ex4_wr_reg_i         (ex4_wr_reg),
    .pc_i                 (dec_pc),
    .rs1_data_i           (dec_rs1_data),
    .rs2_data_i           (dec_rs2_data),
    .wb_data_to_reg_i     (data_to_reg),
    .instruction_i        (instruction_q),
    .stall_o              (dec_stall),
    .alu_valid_o          (alu_valid),
    .ex_valid_o           (ex_valid),
    .offset_sign_extend_o (alu_offset_sign_extend),
    .rs1_o                (rs1),
    .rs2_o                (rs2),
    .ex_wr_reg_o          (ex_wr_reg),
    .alu_pc_o             (alu_pc),
    .alu_rs1_data_o       (alu_rs1_data),
    .alu_rs2_data_o       (alu_rs2_data),
    .instruction_o        (alu_instruction),
`ifndef SYNTHESIS
    .debug_alu_pc_o       (debug_alu_pc),
    .debug_ex_pc_o        (debug_ex_pc),
    .debug_ex_instr_o     (debug_ex_instr)
`endif
  );

  alu_stage #(
    .DATA_WIDTH           (DATA_WIDTH),
    .OPCODE_WIDTH         (OPCODE_WIDTH)
  ) alu_stage (
    .clk_i                (clk_i),
    .rst_i                (rst_i),
    .valid_i              (alu_valid),
    .mem_stall_i          (mem_stall),
    .data_a_i             (alu_rs1_data),
    .data_b_i             (alu_rs2_data),
    .pc_i                 (alu_pc),
    .offset_sign_extend_i (alu_offset_sign_extend),
    .instruction_i        (alu_instruction),
`ifndef SYNTHESIS
    .debug_pc_i           (debug_alu_pc),
`endif
    .mem_valid_o          (mem_valid),
    .mem_reg_wr_en_o      (mem_reg_wr_en),
    .is_instr_wbalu_o     (alu_is_instr_wbalu),
    .instr_finishes_o     (alu_instr_finishes),
    .wr_reg_o             (alu_wr_reg),
    .pc_branch_offset_o   (alu_pc_branch_offset),
    .jump_address_o       (jump_address),
    .mem_alu_result_o     (mem_alu_result),
    .mem_rs2_data_o       (mem_rs2_data),
    .mem_wr_reg_o         (mem_wr_reg),
    .data_to_reg_o        (alu_data_to_reg),
    .mem_is_load_o        (mem_is_load),
    .mem_is_store_o       (mem_is_store),
    .branch_taken_o       (alu_branch_taken),
    .is_jump_o            (is_jump),
    .mem_access_size_o    (mem_access_size),
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
    .rs2_data_i                 (mem_rs2_data),
    .wr_reg_i                   (mem_wr_reg),
    .mem_data_i                 (mem_data_i),
    .valid_i                    (mem_valid),
    .is_load_i                  (mem_is_load),
    .is_store_i                 (mem_is_store),
    .reg_wr_en_i                (mem_reg_wr_en),
    .mem_data_is_valid_i        (mem_data_valid_i & ~mem_data_is_instr_i),
    .access_size_i              (mem_access_size),
`ifndef SYNTHESIS
    .debug_pc_i                 (debug_mem_pc),
    .debug_instr_i              (debug_mem_instr),
`endif
    .wb_valid_o                 (wb_valid_from_mem),
    .wb_reg_wr_en_o             (wb_reg_wr_en_from_mem),
    .wb_is_load_o               (wb_is_load_from_mem),
    .rd_req_valid_o             (mem_rd_req_valid),
    .wr_req_valid_o             (mem_wr_req_valid),
    .stall_o                    (mem_stall),
    .wb_is_next_cycle_o         (mem_wb_is_next_cycle),
    .wb_wr_reg_o                (wb_wr_reg_from_mem),
    .wb_data_from_mem_o         (wb_data_from_mem),
    .wb_alu_result_o            (wb_alu_result),
    .mem_req_address_o          (mem_req_address),
    .wr_data_o                  (wr_data_o),
    .req_access_size_o          (mem_req_access_size),
`ifndef SYNTHESIS
    .debug_wb_pc_o              (debug_wb_pc_from_mem),
    .debug_wb_instr_o           (debug_wb_instr_from_mem)
`endif
  );

  ex_stages #(
    .REGISTER_WIDTH     (REGISTER_WIDTH),
    .DATA_WIDTH         (DATA_WIDTH)
  ) ex_stages (
    .clk_i              (clk_i),
    .rst_i              (rst_i),
    .valid_i            (ex_valid),
    .wr_reg_i           (ex_wr_reg),
    .a_i                (alu_rs1_data),
    .b_i                (alu_rs2_data),
`ifndef SYNTHESIS
    .debug_pc_i         (debug_ex_pc),
    .debug_instr_i      (debug_ex_instr),
`endif
    .ex2_valid_o        (ex2_valid),
    .ex3_valid_o        (ex3_valid),
    .ex4_valid_o        (ex4_valid),
    .wb_is_next_cycle_o (ex_wb_is_next_cycle),
    .result_ready_o     (wb_valid_from_ex),
    .ex2_wr_reg_o       (ex2_wr_reg),
    .ex3_wr_reg_o       (ex3_wr_reg),
    .ex4_wr_reg_o       (ex4_wr_reg),
    .wr_reg_o           (wb_wr_reg_from_ex),
    .result_o           (wb_data_from_ex),
`ifndef SYNTHESIS
    .debug_pc_o         (debug_wb_pc_from_ex),
    .debug_instr_o      (debug_wb_instr_from_ex)
`endif
  );

  wb_stage #(
    .DATA_WIDTH      (DATA_WIDTH)
  ) wb_stage (
    .mem_valid_i       (wb_valid_from_mem),
    .ex_valid_i        (wb_valid_from_ex),
    .mem_reg_wr_en_i   (wb_reg_wr_en_from_mem),
    .mem_wr_reg_i      (wb_wr_reg_from_mem),
    .ex_wr_reg_i       (wb_wr_reg_from_ex),
    .alu_result_i      (wb_alu_result),
    .ex_result_i       (wb_data_from_ex),
    .data_from_mem_i   (wb_data_from_mem),
    .is_load_i         (wb_is_load_from_mem),
`ifndef SYNTHESIS
    .debug_mem_pc_i    (debug_wb_pc_from_mem),
    .debug_ex_pc_i     (debug_wb_pc_from_ex),
    .debug_mem_instr_i (debug_wb_instr_from_mem),
    .debug_ex_instr_i  (debug_wb_instr_from_ex),
`endif
    .reg_wr_en_o       (wb_reg_wr_en),
    .wr_reg_o          (wb_wr_reg),
    .data_to_reg_o     (wb_data_to_reg),
`ifndef SYNTHESIS
    .debug_pc_o        (debug_wb_pc),
    .debug_instr_o     (debug_wb_instr)
`endif
  );

  rbank #(
    .REGISTER_WIDTH (REGISTER_WIDTH),
    .DATA_WIDTH     (DATA_WIDTH)
   ) rbank (
    .clk_i        (clk_i),
    .rst_i        (rst_i),
    .wr_en_i      (reg_wr_en),
    .rd_reg_a_i   (rs1),
    .rd_reg_b_i   (rs2),
    .wr_reg_i     (wr_reg),
    .wr_data_i    (data_to_reg),
    .reg_a_data_o (dec_rs1_data),
    .reg_b_data_o (dec_rs2_data),
`ifndef SYNTHESIS
    .debug_regs_o (debug_regs_o)
`endif
  );

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      is_jump           <= 1'b0;
      alu_branch_taken  <= 1'b0;
    end else begin
`ifndef SYNTHESIS
      debug_instr_is_completed_o <= alu_instr_finishes | wb_valid_from_mem | wb_valid_from_ex;
      debug_pc_o                 <= alu_instr_finishes ? alu_pc          : debug_wb_pc;
      debug_instr_o              <= alu_instr_finishes ? alu_instruction : debug_wb_instr;
`endif
    end
  end

  assign rd_req_valid_o    = fetch_rd_req_valid | mem_rd_req_valid;
  assign wr_req_valid_o    = mem_wr_req_valid;
  assign req_is_instr_o    = fetch_rd_req_valid;
  assign req_address_o     = fetch_rd_req_valid ? fetch_req_address : mem_req_address;
  assign req_access_size_o = fetch_rd_req_valid ? fetch_req_access_size : mem_req_access_size;

  assign reg_wr_en   = alu_is_instr_wbalu | wb_reg_wr_en;
  assign wr_reg      = alu_is_instr_wbalu ? alu_wr_reg : wb_wr_reg;
  assign data_to_reg = alu_is_instr_wbalu ? alu_data_to_reg : wb_data_to_reg;

endmodule
