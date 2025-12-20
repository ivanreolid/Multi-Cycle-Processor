`include "hazard_unit.sv"
`include "fetch_stage.sv"
`include "decode_stage.sv"
`include "alu_stage.sv"
`include "mem_stage.sv"
`include "ex_stages.sv"
`include "wb_arbiter.sv"
`include "register_file.sv"

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
  logic fetch_stall;
  logic dec_valid_d;
  logic [ADDR_WIDTH-1:0] dec_pc_d;
  logic [ADDR_WIDTH-1:0] fetch_req_address;
  access_size_t fetch_req_access_size;
  instruction_t dec_instruction_d;

  // Decode stage wires
  logic dec_valid_q;
  logic alu_valid_d;
  logic ex1_valid_d;
  logic [REGISTER_WIDTH-1:0] dec_wr_reg;
  logic [SHAMT_WIDTH-1:0] alu_shamt_d;
  logic [ADDR_WIDTH-1:0] dec_pc_q;
  logic [DATA_WIDTH-1:0] dec_rs1_data, dec_rs2_data;
  logic [DATA_WIDTH-1:0] alu_rs1_data_d, alu_rs2_data_d;
  logic [DATA_WIDTH-1:0] alu_offset_sign_extend_d;
  logic dec_stall;
  logic alu_bubble, ex_bubble;
  hazard_ctrl_t hazard_signals;
  instruction_t dec_instruction_q;

  // ALU stage wires
  logic [ADDR_WIDTH-1:0] alu_pc_q;
  logic [DATA_WIDTH-1:0] alu_rs1_data_q, alu_rs2_data_q;
  logic [SHAMT_WIDTH-1:0] alu_shamt_q;
  logic [DATA_WIDTH-1:0] alu_offset_sign_extend_q;
  logic [ADDR_WIDTH-1:0] alu_pc_branch_offset;
  logic [ADDR_WIDTH-1:0] jump_address;
  logic is_jump;
  logic alu_branch_taken;
  logic alu_valid_q;
  logic alu_stall;
  logic mem_valid_d;
  logic mem_is_load_d;
  logic mem_is_store_d;
  logic mem_reg_wr_en_d;
  logic alu_is_instr_wbalu, alu_instr_finishes;
  logic [REGISTER_WIDTH-1:0] alu_wr_reg_q;
  logic [DATA_WIDTH-1:0] alu_data_to_reg;
  logic [DATA_WIDTH-1:0] mem_alu_result_d;
  logic [DATA_WIDTH-1:0] mem_rs2_data_d;
  access_size_t mem_access_size_d;
  instruction_t alu_instruction_q;
`ifndef SYNTHESIS
  instruction_t debug_mem_instr_d;
`endif

  // Mem stage wires
  logic [DATA_WIDTH-1:0] mem_alu_result_q;
  logic [DATA_WIDTH-1:0] mem_rs2_data_q;
  logic [REGISTER_WIDTH-1:0] mem_wr_reg_q;
  logic mem_reg_wr_en_q;
  logic mem_is_load_q, mem_is_store_q;
  logic mem_branch_taken;
  logic mem_valid_q;
  logic mem_rd_req_valid, mem_wr_req_valid;
  logic mem_stall;
  logic mem_wb_is_next_cycle;
  logic [ADDR_WIDTH-1:0] mem_req_address;
  access_size_t mem_access_size_q, mem_req_access_size;
`ifndef SYNTHESIS
  logic [ADDR_WIDTH-1:0] debug_mem_pc_q;
  instruction_t debug_mem_instr_q;
`endif

  // EX1 stage wires
  logic ex2_valid_d, ex3_valid_d, ex4_valid_d, ex5_valid_d;
  logic ex1_valid_q, ex2_valid_q, ex3_valid_q, ex4_valid_q, ex5_valid_q;
  logic ex_stall;
  logic [REGISTER_WIDTH-1:0] ex2_wr_reg_d, ex3_wr_reg_d, ex4_wr_reg_d, ex5_wr_reg_d;
  logic [REGISTER_WIDTH-1:0] ex1_wr_reg_q, ex2_wr_reg_q, ex3_wr_reg_q, ex4_wr_reg_q, ex5_wr_reg_q;
  logic [DATA_WIDTH-1:0] ex_a, ex_b;
  logic [DATA_WIDTH-1:0] ex2_result_d, ex3_result_d, ex4_result_d, ex5_result_d;
  logic [DATA_WIDTH-1:0] ex2_result_q, ex3_result_q, ex4_result_q, ex5_result_q;
`ifndef SYNTHESIS
  logic [ADDR_WIDTH-1:0] ex2_debug_pc_d, ex3_debug_pc_d, ex4_debug_pc_d, ex5_debug_pc_d;
  logic [ADDR_WIDTH-1:0] ex1_debug_pc_q, ex2_debug_pc_q, ex3_debug_pc_q, ex4_debug_pc_q,
                         ex5_debug_pc_q;
  instruction_t ex2_debug_instr_d, ex3_debug_instr_d, ex4_debug_instr_d, ex5_debug_instr_d;
  instruction_t ex1_debug_instr_q, ex2_debug_instr_q, ex3_debug_instr_q, ex4_debug_instr_q,
                ex5_debug_instr_q;
`endif

  // WB stage wires
  logic wb_valid_from_mem, wb_valid_from_ex;
  logic wb_reg_wr_en_from_mem, wb_reg_wr_en_from_ex_q;
  logic [REGISTER_WIDTH-1:0] wb_wr_reg_from_mem, wb_wr_reg_from_ex;
  logic [REGISTER_WIDTH-1:0] wb_wr_reg;
  logic [DATA_WIDTH-1:0] wb_data_from_mem, wb_data_from_ex;
  logic wb_reg_wr_en;
  logic mem_is_completed, ex_is_completed, alu_is_completed;
  logic ex_allowed_wb, alu_allowed_wb;
  logic wb_valid;
  logic [DATA_WIDTH-1:0] wb_data_to_reg;
`ifndef SYNTHESIS
  logic [ADDR_WIDTH-1:0] debug_wb_pc_from_mem, debug_wb_pc_from_ex, debug_wb_pc;
  logic debug_non_store_is_completed;
  instruction_t debug_wb_instr_from_mem, debug_wb_instr_from_ex, debug_wb_instr;
`endif

  // Future file wires
`ifndef SYNTHESIS
  logic [DATA_WIDTH-1:0] debug_future_file [32];
`endif

  hazard_unit #(
    .REGISTER_WIDTH     (REGISTER_WIDTH)
  ) hazard_unit (
    .dec_valid_i        (dec_valid_q),
    .alu_valid_i        (alu_valid_q),
    .alu_instr_finishes_i (alu_instr_finishes),
    .mem_valid_i        (mem_valid_q),
    .mem_busy_i         (mem_stall),
    .mem_reg_wr_en_i    (mem_reg_wr_en_q),
    .ex_allowed_wb_i    (ex_allowed_wb),
    .alu_allowed_wb_i   (alu_allowed_wb),
    .wb_is_next_cycle_i (mem_wb_is_next_cycle | wb_valid_from_ex),
    .ex1_valid_i        (ex1_valid_q),
    .ex2_valid_i        (ex2_valid_q),
    .ex3_valid_i        (ex3_valid_q),
    .ex4_valid_i        (ex4_valid_q),
    .ex5_valid_i        (ex5_valid_q),
    .mem_wr_reg_i       (mem_wr_reg_q),
    .ex1_wr_reg_i       (ex1_wr_reg_q),
    .ex2_wr_reg_i       (ex2_wr_reg_q),
    .ex3_wr_reg_i       (ex3_wr_reg_q),
    .ex4_wr_reg_i       (ex4_wr_reg_q),
    .hazard_signals_i   (hazard_signals),
    .stall_ex_o         (ex_stall),
    .stall_mem_o        (),
    .stall_alu_o        (alu_stall),
    .stall_decode_o     (dec_stall),
    .stall_fetch_o      (fetch_stall),
    .alu_bubble_o       (alu_bubble),
    .ex_bubble_o        (ex_bubble)
  );

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
    .dec_valid_o         (dec_valid_d),
    .mem_req_addr_o      (fetch_req_address),
    .dec_pc_o            (dec_pc_d),
    .req_access_size_o   (fetch_req_access_size),
    .dec_instr_o         (dec_instruction_d)
  );

  decode_stage #(
    .SHAMT_WIDTH           (SHAMT_WIDTH),
    .DATA_WIDTH            (DATA_WIDTH),
    .ADDR_WIDTH            (ADDR_WIDTH),
    .REGISTER_WIDTH        (REGISTER_WIDTH),
    .OPCODE_WIDTH          (OPCODE_WIDTH)
  ) decode_stage (
    .valid_i               (dec_valid_q),
    .is_jump_i             (is_jump),
    .branch_taken_i        (alu_branch_taken),
    .mem_stage_valid_i     (mem_valid_q),
    .wb_reg_wr_en_i        (wb_reg_wr_en),
    .mem_stage_reg_wr_en_i (mem_reg_wr_en_q),
    .ex1_valid_i           (ex1_valid_q),
    .ex2_valid_i           (ex2_valid_q),
    .ex3_valid_i           (ex3_valid_q),
    .ex4_valid_i           (ex4_valid_q),
    .ex5_valid_i           (ex5_valid_q),
    .mem_stage_wr_reg_i    (mem_wr_reg_q),
    .wb_wr_reg_i           (wb_wr_reg),
    .ex1_wr_reg_i          (ex1_wr_reg_q),
    .ex2_wr_reg_i          (ex2_wr_reg_q),
    .ex3_wr_reg_i          (ex3_wr_reg_q),
    .ex4_wr_reg_i          (ex4_wr_reg_q),
    .ex5_wr_reg_i          (ex5_wr_reg_q),
    .rs1_data_i            (dec_rs1_data),
    .rs2_data_i            (dec_rs2_data),
    .mem_stage_result_i    (wb_data_from_mem),
    .ex5_result_i          (ex5_result_q),
    .wb_data_to_reg_i      (wb_data_to_reg),
    .instruction_i         (dec_instruction_q),
    .alu_valid_o           (alu_valid_d),
    .ex_valid_o            (ex1_valid_d),
    .shamt_o               (alu_shamt_d),
    .offset_sign_extend_o  (alu_offset_sign_extend_d),
    .wr_reg_o              (dec_wr_reg),
    .alu_rs1_data_o        (alu_rs1_data_d),
    .alu_rs2_data_o        (alu_rs2_data_d),
    .hazard_signals_o      (hazard_signals)
  );

  alu_stage #(
    .SHAMT_WIDTH          (SHAMT_WIDTH),
    .DATA_WIDTH           (DATA_WIDTH),
    .OPCODE_WIDTH         (OPCODE_WIDTH)
  ) alu_stage (
    .valid_i              (alu_valid_q),
    .mem_stall_i          (mem_stall),
    .shamt_i              (alu_shamt_q),
    .data_a_i             (alu_rs1_data_q),
    .data_b_i             (alu_rs2_data_q),
    .pc_i                 (alu_pc_q),
    .offset_sign_extend_i (alu_offset_sign_extend_q),
    .instruction_i        (alu_instruction_q),
    .mem_valid_o          (mem_valid_d),
    .mem_reg_wr_en_o      (mem_reg_wr_en_d),
    .is_instr_wbalu_o     (alu_is_instr_wbalu),
    .instr_finishes_o     (alu_instr_finishes),
    .pc_branch_offset_o   (alu_pc_branch_offset),
    .jump_address_o       (jump_address),
    .mem_alu_result_o     (mem_alu_result_d),
    .mem_rs2_data_o       (mem_rs2_data_d),
    .data_to_reg_o        (alu_data_to_reg),
    .mem_is_load_o        (mem_is_load_d),
    .mem_is_store_o       (mem_is_store_d),
    .branch_taken_o       (alu_branch_taken),
    .is_jump_o            (is_jump),
    .mem_access_size_o    (mem_access_size_d),
`ifndef SYNTHESIS
    .debug_mem_instr_o    (debug_mem_instr_d)
`endif
  );

  mem_stage #(
    .MEM_SIZE                   (MEM_SIZE),
    .ADDR_WIDTH                 (ADDR_WIDTH),
    .DATA_WIDTH                 (DATA_WIDTH)
  ) mem_stage (
    .clk_i                      (clk_i),
    .rst_i                      (rst_i),
    .alu_result_i               (mem_alu_result_q),
    .rs2_data_i                 (mem_rs2_data_q),
    .wr_reg_i                   (mem_wr_reg_q),
    .mem_data_i                 (mem_data_i),
    .valid_i                    (mem_valid_q),
    .is_load_i                  (mem_is_load_q),
    .is_store_i                 (mem_is_store_q),
    .reg_wr_en_i                (mem_reg_wr_en_q),
    .mem_data_is_valid_i        (mem_data_valid_i & ~mem_data_is_instr_i),
    .access_size_i              (mem_access_size_q),
`ifndef SYNTHESIS
    .debug_pc_i                 (debug_mem_pc_q),
    .debug_instr_i              (debug_mem_instr_q),
`endif
    .wb_valid_o                 (wb_valid_from_mem),
    .wb_reg_wr_en_o             (wb_reg_wr_en_from_mem),
    .rd_req_valid_o             (mem_rd_req_valid),
    .wr_req_valid_o             (mem_wr_req_valid),
    .stall_o                    (mem_stall),
    .wb_is_next_cycle_o         (mem_wb_is_next_cycle),
    .wb_wr_reg_o                (wb_wr_reg_from_mem),
    .wb_data_from_mem_o         (wb_data_from_mem),
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
    .ex1_valid_i        (ex1_valid_q),
    .ex2_valid_i        (ex2_valid_q),
    .ex3_valid_i        (ex3_valid_q),
    .ex4_valid_i        (ex4_valid_q),
    .ex5_valid_i        (ex5_valid_q),
    .ex1_wr_reg_i       (ex1_wr_reg_q),
    .ex2_wr_reg_i       (ex2_wr_reg_q),
    .ex3_wr_reg_i       (ex3_wr_reg_q),
    .ex4_wr_reg_i       (ex4_wr_reg_q),
    .ex5_wr_reg_i       (ex5_wr_reg_q),
    .ex1_a_i            (alu_rs1_data_q),
    .ex1_b_i            (alu_rs2_data_q),
    .ex2_result_i       (ex2_result_q),
    .ex3_result_i       (ex3_result_q),
    .ex4_result_i       (ex4_result_q),
    .ex5_result_i       (ex5_result_q),
`ifndef SYNTHESIS
    .ex1_debug_pc_i     (ex1_debug_pc_q),
    .ex2_debug_pc_i     (ex2_debug_pc_q),
    .ex3_debug_pc_i     (ex3_debug_pc_q),
    .ex4_debug_pc_i     (ex4_debug_pc_q),
    .ex5_debug_pc_i     (ex5_debug_pc_q),
    .ex1_debug_instr_i  (ex1_debug_instr_q),
    .ex2_debug_instr_i  (ex2_debug_instr_q),
    .ex3_debug_instr_i  (ex3_debug_instr_q),
    .ex4_debug_instr_i  (ex4_debug_instr_q),
    .ex5_debug_instr_i  (ex5_debug_instr_q),
`endif
    .ex2_valid_o        (ex2_valid_d),
    .ex3_valid_o        (ex3_valid_d),
    .ex4_valid_o        (ex4_valid_d),
    .ex5_valid_o        (ex5_valid_d),
    .result_ready_o     (wb_valid_from_ex),
    .ex2_wr_reg_o       (ex2_wr_reg_d),
    .ex3_wr_reg_o       (ex3_wr_reg_d),
    .ex4_wr_reg_o       (ex4_wr_reg_d),
    .ex5_wr_reg_o       (ex5_wr_reg_d),
    .wr_reg_o           (wb_wr_reg_from_ex),
    .ex2_result_o       (ex2_result_d),
    .ex3_result_o       (ex3_result_d),
    .ex4_result_o       (ex4_result_d),
    .ex5_result_o       (ex5_result_d),
    .result_o           (wb_data_from_ex),
`ifndef SYNTHESIS
    .ex2_debug_pc_o     (ex2_debug_pc_d),
    .ex3_debug_pc_o     (ex3_debug_pc_d),
    .ex4_debug_pc_o     (ex4_debug_pc_d),
    .ex5_debug_pc_o     (ex5_debug_pc_d),
    .debug_pc_o         (debug_wb_pc_from_ex),
    .ex2_debug_instr_o  (ex2_debug_instr_d),
    .ex3_debug_instr_o  (ex3_debug_instr_d),
    .ex4_debug_instr_o  (ex4_debug_instr_d),
    .ex5_debug_instr_o  (ex5_debug_instr_d),
    .debug_instr_o      (debug_wb_instr_from_ex)
`endif
  );

  wb_arbiter #(
    .DATA_WIDTH          (DATA_WIDTH)
  ) wb_arbiter (
    .alu_ready_i        (alu_instr_finishes),
    .alu_is_instr_wb_i  (alu_is_instr_wbalu),
    .mem_ready_i        (wb_valid_from_mem),
    .ex_ready_i         (wb_valid_from_ex),
    .mem_reg_wr_en_i    (wb_reg_wr_en_from_mem),
    .alu_wr_reg_i       (alu_wr_reg_q),
    .mem_wr_reg_i       (wb_wr_reg_from_mem),
    .ex_wr_reg_i        (wb_wr_reg_from_ex),
    .alu_result_i       (alu_data_to_reg),
    .ex_result_i        (wb_data_from_ex),
    .data_from_mem_i    (wb_data_from_mem),
`ifndef SYNTHESIS
    .debug_alu_pc_i     (alu_pc_q),
    .debug_mem_pc_i     (debug_wb_pc_from_mem),
    .debug_ex_pc_i      (debug_wb_pc_from_ex),
    .debug_alu_instr_i  (debug_mem_instr_d),
    .debug_mem_instr_i  (debug_wb_instr_from_mem),
    .debug_ex_instr_i   (debug_wb_instr_from_ex),
`endif
    .reg_wr_en_o        (wb_reg_wr_en),
    .mem_is_completed_o (mem_is_completed),
    .ex_is_completed_o  (ex_is_completed),
    .alu_is_completed_o (alu_is_completed),
    .ex_allowed_wb_o    (ex_allowed_wb),
    .alu_allowed_wb_o   (alu_allowed_wb),
    .wr_reg_o           (wb_wr_reg),
    .data_to_reg_o      (wb_data_to_reg),
`ifndef SYNTHESIS
    .debug_pc_o         (debug_wb_pc),
    .debug_instr_o      (debug_wb_instr)
`endif
  );

  register_file #(
    .REGISTER_WIDTH (REGISTER_WIDTH),
    .DATA_WIDTH     (DATA_WIDTH)
  ) future_file (
    .clk_i          (clk_i),
    .rst_i          (rst_i),
    .wr_en_i        (wb_reg_wr_en),
    .rd_reg_a_i     (hazard_signals.rs1),
    .rd_reg_b_i     (hazard_signals.rs2),
    .wr_reg_i       (wb_wr_reg),
    .wr_data_i      (wb_data_to_reg),
    .reg_a_data_o   (dec_rs1_data),
    .reg_b_data_o   (dec_rs2_data)
`ifndef SYNTHESIS
    , .debug_regs_o (debug_future_file)
`endif
);

  register_file #(
    .REGISTER_WIDTH (REGISTER_WIDTH),
    .DATA_WIDTH     (DATA_WIDTH)
   ) architectural_file (
    .clk_i        (clk_i),
    .rst_i        (rst_i),
    .wr_en_i      (wb_reg_wr_en),
    .rd_reg_a_i   (),
    .rd_reg_b_i   (),
    .wr_reg_i     (wb_wr_reg),
    .wr_data_i    (wb_data_to_reg),
    .reg_a_data_o (),
    .reg_b_data_o (),
`ifndef SYNTHESIS
    .debug_regs_o (debug_regs_o)
`endif
  );

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      dec_valid_q         <= 1'b0;
      is_jump             <= 1'b0;
      alu_branch_taken    <= 1'b0;
      alu_valid_q         <= 1'b0;
      mem_valid_q         <= 1'b0;
      ex1_valid_q         <= 1'b0;
      ex2_valid_q         <= 1'b0;
      ex3_valid_q         <= 1'b0;
      ex4_valid_q         <= 1'b0;
      ex5_valid_q         <= 1'b0;
    end else begin

      // Fetch -> Decode flops
      if (!dec_stall) begin
        dec_valid_q       <= dec_valid_d;
        dec_pc_q          <= dec_pc_d;
        dec_instruction_q <= dec_instruction_d;
      end

      // Decode -> ALU flops
      if (alu_bubble) begin
        alu_valid_q              <= 1'b0;
      end else if (!alu_stall) begin
        alu_valid_q              <= alu_valid_d;
        alu_pc_q                 <= dec_pc_q;
        alu_wr_reg_q             <= dec_wr_reg;
        alu_rs1_data_q           <= alu_rs1_data_d;
        alu_rs2_data_q           <= alu_rs2_data_d;
        alu_shamt_q              <= alu_shamt_d;
        alu_offset_sign_extend_q <= alu_offset_sign_extend_d;
        alu_instruction_q        <= dec_instruction_q;
      end

      // Decode -> EX flops
      if (ex_bubble) begin
        ex1_valid_q              <= 1'b0;
      end else if (!ex_stall) begin
        ex1_valid_q              <= ex1_valid_d;
        ex1_wr_reg_q             <= dec_wr_reg;
`ifndef SYNTHESIS
        ex1_debug_pc_q           <= dec_pc_q;
        ex1_debug_instr_q        <= dec_instruction_q;
`endif
      end

      // Internal EX flops
      ex2_valid_q       <= ex2_valid_d;
      ex3_valid_q       <= ex3_valid_d;
      ex4_valid_q       <= ex4_valid_d;
      ex5_valid_q       <= ex5_valid_d;
      ex2_wr_reg_q      <= ex2_wr_reg_d;
      ex3_wr_reg_q      <= ex3_wr_reg_d;
      ex4_wr_reg_q      <= ex4_wr_reg_d;
      ex5_wr_reg_q      <= ex5_wr_reg_d;
      ex2_result_q      <= ex2_result_d;
      ex3_result_q      <= ex3_result_d;
      ex4_result_q      <= ex4_result_d;
      ex5_result_q      <= ex5_result_d;
`ifndef SYNTHESIS
      ex2_debug_pc_q    <= ex2_debug_pc_d;
      ex3_debug_pc_q    <= ex3_debug_pc_d;
      ex4_debug_pc_q    <= ex4_debug_pc_d;
      ex5_debug_pc_q    <= ex5_debug_pc_d;
      ex2_debug_instr_q <= ex2_debug_instr_d;
      ex3_debug_instr_q <= ex3_debug_instr_d;
      ex4_debug_instr_q <= ex4_debug_instr_d;
      ex5_debug_instr_q <= ex5_debug_instr_d;
`endif

      // ALU -> MEM flops
      if (!mem_stall) begin
        mem_valid_q       <= mem_valid_d;
        mem_is_load_q     <= mem_is_load_d;
        mem_is_store_q    <= mem_is_store_d;
        mem_reg_wr_en_q   <= mem_reg_wr_en_d;
        mem_wr_reg_q      <= alu_wr_reg_q;
        mem_alu_result_q  <= mem_alu_result_d;
        mem_rs2_data_q    <= mem_rs2_data_d;
        mem_access_size_q <= mem_access_size_d;
`ifndef SYNTHESIS
        debug_mem_pc_q    <= alu_pc_q;
        debug_mem_instr_q <= debug_mem_instr_d;
`endif
      end

`ifndef SYNTHESIS
      debug_instr_is_completed_o <= mem_is_completed | ex_is_completed | alu_is_completed;
      debug_pc_o                 <= debug_wb_pc;
      debug_instr_o              <= debug_wb_instr;
`endif
    end
  end

  assign rd_req_valid_o    = fetch_rd_req_valid | mem_rd_req_valid;
  assign wr_req_valid_o    = mem_wr_req_valid;
  assign req_is_instr_o    = fetch_rd_req_valid;
  assign req_address_o     = fetch_rd_req_valid ? fetch_req_address : mem_req_address;
  assign req_access_size_o = fetch_rd_req_valid ? fetch_req_access_size : mem_req_access_size;

endmodule
