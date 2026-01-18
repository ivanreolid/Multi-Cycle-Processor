`include "hazard_unit.sv"
`include "fetch_stage.sv"
`include "decode_stage.sv"
`include "alu_stage.sv"
`include "mem_stage.sv"
`include "ex_stages.sv"
`include "wb_arbiter.sv"
`include "csr_regfile.sv"
`include "register_file.sv"
`include "mem_arbiter.sv"
`include "reorder_buffer.sv"

module cpu import params_pkg::*; #(
  parameter int CSR_ADDR_WIDTH    = params_pkg::CSR_ADDR_WIDTH,
  parameter int PADDR_WIDTH       = params_pkg::PADDR_WIDTH,
  parameter int CACHE_LINE_BYTES  = params_pkg::CACHE_LINE_BYTES,
  parameter int ICACHE_N_LINES    = params_pkg::ICACHE_N_LINES,
  parameter int DCACHE_N_LINES    = params_pkg::DCACHE_N_LINES
)(
  input  logic clk_i,
  input  logic rst_i,

  input  logic mem_data_valid_i,
  input  logic [CACHE_LINE_BYTES*8-1:0] mem_data_i,
  output logic rd_req_valid_o,
  output logic wr_req_valid_o,
  output logic req_is_instr_o,
  output logic [PADDR_WIDTH-1:0] req_address_o,
  output logic [CACHE_LINE_BYTES*8-1:0] wr_data_o,
  output access_size_t req_access_size_o,

  input  logic finish,   // flush request
  output logic done,     // flush completed
  input write_done_o,   //mem finished writing
`ifndef SYNTHESIS
  output logic debug_vm_en_o,
  output logic debug_trap_bypass_mmu_o,
  output logic debug_instr_is_completed_o,
  output logic [DATA_WIDTH-1:0] debug_satp_o,
  output logic [DATA_WIDTH-1:0] debug_regs_o [32],
  output logic [ADDR_WIDTH-1:0] debug_pc_o,
  output instruction_t debug_instr_o
`endif
);
  logic icache_req, icache_gnt, icache_rvalid;
  logic [PADDR_WIDTH-1:0] icache_addr;
  logic [CACHE_LINE_BYTES*8-1:0] icache_rdata;

  logic dcache_req, dcache_we, dcache_gnt, dcache_rvalid;
  logic [PADDR_WIDTH-1:0] dcache_addr;
  logic [CACHE_LINE_BYTES*8-1:0] dcache_wdata, dcache_rdata;

  // Fetch stage wires
  logic fetch_stall;
  logic fetch_present_table_req;
  logic fetch_ppn_is_present;
  logic dec_valid_d;
  logic dec_excpt_d;
  logic [PPN_WIDTH-1:0] fetch_present_table_ppn;
  logic [DATA_WIDTH-1:0] satp_data;
  logic [DATA_WIDTH-1:0] mtvec_data;
  logic [DATA_WIDTH-1:0] mepc_data;
  logic [ADDR_WIDTH-1:0] next_pc_flush;
  logic [ADDR_WIDTH-1:0] dec_pc_d;
  excpt_cause_t dec_excpt_cause_d;
  access_size_t fetch_req_access_size;
  instruction_t dec_instruction_d;

  // Decode stage wires
  logic dec_valid_q;
  logic dec_excpt_q;
  logic dec_instr_is_wb, dec_instr_is_csr_wb;
  logic dec_instr_is_mret;
  logic alu_valid_d;
  logic ex1_valid_d;
  logic [ROB_ENTRY_WIDTH-1:0] dec_rob_new_instr_idx;
  logic [REGISTER_WIDTH-1:0] dec_wr_reg;
  logic [SHAMT_WIDTH-1:0] alu_shamt_d;
  logic [ADDR_WIDTH-1:0] dec_pc_q;
  logic [CSR_ADDR_WIDTH-1:0] dec_csr_addr;
  logic [DATA_WIDTH-1:0] dec_csr_data;
  logic [DATA_WIDTH-1:0] dec_rs1_data, dec_rs2_data;
  logic [DATA_WIDTH-1:0] alu_rs1_data_d, alu_rs2_data_d;
  logic [DATA_WIDTH-1:0] alu_offset_sign_extend_d;
  logic dec_stall;
  logic alu_bubble, ex_bubble;
  excpt_cause_t dec_excpt_cause_q;
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
  logic [ROB_ENTRY_WIDTH-1:0] alu_rob_instr_idx_q;
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
  logic [ROB_ENTRY_WIDTH-1:0] mem_rob_instr_idx_q;
  logic [REGISTER_WIDTH-1:0] mem_wr_reg_q;
  logic mem_ppn_is_present;
  logic mem_present_table_req;
  logic mem_reg_wr_en_q;
  logic mem_is_load_q, mem_is_store_q;
  logic mem_valid_q;
  logic mem_stall;
  logic mem_wb_is_next_cycle;
  access_size_t mem_access_size_q;
  logic mem_rd_req, mem_wr_req;
  logic [PPN_WIDTH-1:0] mem_present_table_ppn;
  logic [PADDR_WIDTH-1:0] mem_req_addr;
  logic [CACHE_LINE_BYTES*8-1:0] mem_wr_line;
  access_size_t mem_req_access_size;
`ifndef SYNTHESIS
  logic [ADDR_WIDTH-1:0] debug_mem_pc_q;
  instruction_t debug_mem_instr_q;
`endif

  // EX1 stage wires
  logic ex2_valid_d, ex3_valid_d, ex4_valid_d, ex5_valid_d;
  logic ex1_valid_q, ex2_valid_q, ex3_valid_q, ex4_valid_q, ex5_valid_q;
  logic ex_stall;
  logic [ROB_ENTRY_WIDTH-1:0] ex1_rob_instr_idx_q, ex2_rob_instr_idx_q, ex3_rob_instr_idx_q,
                              ex4_rob_instr_idx_q, ex5_rob_instr_idx_q;
  logic [REGISTER_WIDTH-1:0] ex2_wr_reg_d, ex3_wr_reg_d, ex4_wr_reg_d, ex5_wr_reg_d;
  logic [REGISTER_WIDTH-1:0] ex1_wr_reg_q, ex2_wr_reg_q, ex3_wr_reg_q, ex4_wr_reg_q, ex5_wr_reg_q;
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
  logic wb_excpt_from_mem;
  logic wb_reg_wr_en_from_mem, wb_reg_wr_en_from_ex_q;
  logic [REGISTER_WIDTH-1:0] wb_wr_reg_from_mem, wb_wr_reg_from_ex;
  logic [REGISTER_WIDTH-1:0] wb_wr_reg;
  logic [DATA_WIDTH-1:0] wb_data_from_mem, wb_data_from_ex;
  logic wb_reg_wr_en;
  logic instr_with_excpt;
  logic instr_complete;
  logic ex_allowed_wb, alu_allowed_wb;
  logic wb_valid;
  logic [DATA_WIDTH-1:0] wb_data_to_reg;
  logic [ADDR_WIDTH-1:0] wb_excpt_tval_from_mem, instr_complete_excpt_tval;
  excpt_cause_t wb_excpt_cause_from_mem, instr_complete_excp_cause;
`ifndef SYNTHESIS
  logic [ADDR_WIDTH-1:0] debug_wb_pc_from_mem, debug_wb_pc_from_ex, debug_wb_pc;
  instruction_t debug_wb_instr_from_mem, debug_wb_instr_from_ex, debug_wb_instr;
`endif

  // Future file wires
  logic [31:0] ff_valid;
  logic [DATA_WIDTH-1:0] ff_data_a, ff_data_b;
`ifndef SYNTHESIS
  logic [DATA_WIDTH-1:0] debug_future_file [32];
`endif

  // Architectural file wires
  logic [DATA_WIDTH-1:0] af_data_a, af_data_b;

  // Reorder buffer wires
  logic rob_is_full;
  logic rob_excp_we;
  logic flush;
  logic rob_new_instr_valid;
  logic rob_instr_commit_valid;
  logic rob_instr_commit_is_wb, rob_instr_commit_is_csr_wb;
  logic rob_instr_commit_is_mret;
  logic rob_instr_commit_is_csrrw_satp;
  logic [ROB_ENTRY_WIDTH-1:0] rob_instr_complete_idx;
  logic [REGISTER_WIDTH-1:0] rob_instr_commit_reg_id;
  logic [CSR_ADDR_WIDTH-1:0] rob_instr_commit_csr_addr;
  logic [DATA_WIDTH-1:0] rob_instr_commit_data, rob_instr_commit_csr_data;
  logic [ADDR_WIDTH-1:0] rob_excp_pc;
  logic [ADDR_WIDTH-1:0] rob_excp_tval;
  logic [ADDR_WIDTH-1:0] rob_instr_commit_pc;
  excpt_cause_t rob_excp_cause;
`ifndef SYNTHESIS
  instruction_t debug_rob_instr_commit;
`endif

logic mem_req,mem_we;
logic [PADDR_WIDTH-1:0] mem_addr;
logic [CACHE_LINE_BYTES*8-1:0] mem_wdata;

  // Virtual memory wires
  logic vm_en;
  logic trap_bypass_mmu;

  mem_arbiter #(
  .ADDR_WIDTH   (ADDR_WIDTH),
  .PADDR_WIDTH  (PADDR_WIDTH),
  .DATA_WIDTH   (CACHE_LINE_BYTES*8)
) mem_arbiter (
  .clk          (clk_i),
  .rst          (rst_i),

  // ======================
  // I-CACHE
  // ======================
  .icache_req   (icache_req),
  .icache_addr  (icache_addr),
  .icache_gnt   (icache_gnt),
  .icache_rdata (icache_rdata),
  .icache_rvalid(icache_rvalid),

  // ======================
  // D-CACHE
  // ======================
  .dcache_req   (dcache_req),
  .dcache_we    (dcache_we),
  .dcache_addr  (dcache_addr),
  .dcache_wdata (dcache_wdata),
  .dcache_gnt   (dcache_gnt),
  .dcache_rdata (dcache_rdata),
  .dcache_rvalid(dcache_rvalid),

  // ======================
  // MEMORY
  // ======================
  .mem_req      (mem_req),
  .mem_we       (mem_we),
  .mem_addr     (mem_addr),
  .mem_wdata    (mem_wdata),
  .mem_rdata    (mem_data_i),
  .mem_rvalid   (mem_data_valid_i)
  );

  hazard_unit #(
    .REGISTER_WIDTH     (REGISTER_WIDTH)
  ) hazard_unit (
    .rob_is_full_i      (rob_is_full),
    .dec_valid_i        (dec_valid_q),
    .alu_valid_i        (alu_valid_q),
    .alu_instr_finishes_i (alu_instr_finishes),
    .alu_branch_taken_i (alu_branch_taken),
    .alu_is_jump_i      (is_jump),
    .alu_is_load_i      (mem_is_load_d),
    .mem_valid_i        (mem_valid_q),
    .mem_busy_i         (mem_stall),
    .mem_reg_wr_en_i    (mem_reg_wr_en_q),
    .ex_allowed_wb_i    (ex_allowed_wb),
    .alu_allowed_wb_i   (alu_allowed_wb),
    .ex1_valid_i        (ex1_valid_q),
    .ex2_valid_i        (ex2_valid_q),
    .ex3_valid_i        (ex3_valid_q),
    .ex4_valid_i        (ex4_valid_q),
    .ex5_valid_i        (ex5_valid_q),
    .alu_wr_reg_i       (alu_wr_reg_q),
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

  fetch_stage fetch_stage (
    .clk_i               (clk_i),
    .rst_i               (rst_i),
    .vm_en_i             (vm_en),
    .trap_bypass_mmu_i   (trap_bypass_mmu),
    .mem_req_i           (dcache_req),
    .ppn_is_present_i    (fetch_ppn_is_present),
    .flush_i             (flush),
    .alu_branch_taken_i  (alu_branch_taken),
    .is_jump_i           (is_jump),
    .dec_stall_i         (dec_stall),
    .mem_stall_i         (mem_stall),
    .satp_data_i         (satp_data),
    .next_pc_flush_i     (next_pc_flush),
    .pc_branch_offset_i  (alu_pc_branch_offset),
    .jump_address_i      (jump_address),
    .instr_valid_i       (icache_rvalid),
    .instr_line_i        (icache_rdata),
    .present_table_req_o (fetch_present_table_req),
    .rd_req_valid_o      (icache_req),
    .present_table_ppn_o (fetch_present_table_ppn),
    .mem_req_addr_o      (icache_addr),
    .req_access_size_o   (fetch_req_access_size),
    .mem_gnt_i           (icache_gnt),
    .dec_valid_o         (dec_valid_d),
    .dec_excpt_o         (dec_excpt_d),
    .dec_excpt_cause_o   (dec_excpt_cause_d),
    .dec_pc_o            (dec_pc_d),
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
    .alu_stage_is_wb_i     (alu_is_instr_wbalu),
    .mem_stage_valid_i     (mem_valid_q),
    .wb_reg_wr_en_i        (wb_reg_wr_en),
    .mem_stage_reg_wr_en_i (mem_reg_wr_en_q),
    .ex1_valid_i           (ex1_valid_q),
    .ex2_valid_i           (ex2_valid_q),
    .ex3_valid_i           (ex3_valid_q),
    .ex4_valid_i           (ex4_valid_q),
    .ex5_valid_i           (ex5_valid_q),
    .alu_stage_wr_reg_i    (alu_wr_reg_q),
    .mem_stage_wr_reg_i    (mem_wr_reg_q),
    .wb_wr_reg_i           (wb_wr_reg),
    .ex1_wr_reg_i          (ex1_wr_reg_q),
    .ex2_wr_reg_i          (ex2_wr_reg_q),
    .ex3_wr_reg_i          (ex3_wr_reg_q),
    .ex4_wr_reg_i          (ex4_wr_reg_q),
    .ex5_wr_reg_i          (ex5_wr_reg_q),
    .rs1_data_i            (dec_rs1_data),
    .rs2_data_i            (dec_rs2_data),
    .alu_stage_result_i    (alu_data_to_reg),
    .mem_stage_result_i    (wb_data_from_mem),
    .ex5_result_i          (ex5_result_q),
    .wb_data_to_reg_i      (wb_data_to_reg),
    .csr_data_i            (dec_csr_data),
    .instruction_i         (dec_instruction_q),
    .alu_valid_o           (alu_valid_d),
    .ex_valid_o            (ex1_valid_d),
    .instr_is_wb_o         (dec_instr_is_wb),
    .instr_is_csr_wb_o     (dec_instr_is_csr_wb),
    .instr_is_mret_o       (dec_instr_is_mret),
    .shamt_o               (alu_shamt_d),
    .offset_sign_extend_o  (alu_offset_sign_extend_d),
    .wr_reg_o              (dec_wr_reg),
    .csr_addr_o            (dec_csr_addr),
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
    .DATA_WIDTH                 (DATA_WIDTH),
    .REGISTER_WIDTH             (REGISTER_WIDTH),
    .CACHE_LINE_BYTES           (CACHE_LINE_BYTES),
    .DCACHE_N_LINES             (DCACHE_N_LINES)
  ) mem_stage (
    .clk_i                      (clk_i),
    .rst_i                      (rst_i),
    .vm_en_i                    (vm_en),
    .trap_bypass_mmu_i          (trap_bypass_mmu),
    .flush_i                    (flush),
    .ppn_is_present_i           (mem_ppn_is_present),
    .satp_data_i                (satp_data),
    .alu_result_i               (mem_alu_result_q),
    .rs2_data_i                 (mem_rs2_data_q),
    .wr_reg_i                   (mem_wr_reg_q),
    .mem_line_data_i            (dcache_rdata),
    .mem_rvalid_i               (dcache_rvalid),
    .mem_gnt_i                  (dcache_gnt),
    .valid_i                    (mem_valid_q),
    .is_load_i                  (mem_is_load_q),
    .is_store_i                 (mem_is_store_q),
    .reg_wr_en_i                (mem_reg_wr_en_q),
    .access_size_i              (mem_access_size_q),
`ifndef SYNTHESIS
    .debug_pc_i                 (debug_mem_pc_q),
    .debug_instr_i              (debug_mem_instr_q),
`endif
    .present_table_req_o        (mem_present_table_req),
    .wb_valid_o                 (wb_valid_from_mem),
    .wb_excpt_o                (wb_excpt_from_mem),
    .wb_reg_wr_en_o             (wb_reg_wr_en_from_mem),
    .rd_req_valid_o             (mem_rd_req),
    .wr_req_valid_o             (mem_wr_req),
    .stall_o                    (mem_stall),
    .wb_wr_reg_o                (wb_wr_reg_from_mem),
    .present_table_ppn_o        (mem_present_table_ppn),
    .wb_data_from_mem_o         (wb_data_from_mem),
    .wb_excpt_tval_o            (wb_excpt_tval_from_mem),
    .mem_req_address_o          (mem_req_addr),
    .wr_line_data_o             (mem_wr_line),
    .req_access_size_o          (mem_req_access_size),
    .wb_excpt_cause_o           (wb_excpt_cause_from_mem),
    .finish(finish),
    .done(done),
    .write_done_o(write_done_o),
`ifndef SYNTHESIS
    .debug_wb_pc_o              (debug_wb_pc_from_mem),
    .debug_wb_instr_o           (debug_wb_instr_from_mem)
`endif
  );

  assign dcache_req = mem_rd_req | mem_wr_req;
  assign dcache_we = mem_wr_req;
  assign dcache_addr = mem_req_addr;
  assign dcache_wdata = mem_wr_line;

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
    .ROB_ENTRY_WIDTH    (ROB_ENTRY_WIDTH),
    .REGISTER_WIDTH     (REGISTER_WIDTH),
    .DATA_WIDTH         (DATA_WIDTH),
    .ADDR_WIDTH         (ADDR_WIDTH)
  ) wb_arbiter (
    .alu_ready_i        (alu_instr_finishes),
    .alu_is_instr_wb_i  (alu_is_instr_wbalu),
    .mem_ready_i        (wb_valid_from_mem),
    .mem_excpt_i        (wb_excpt_from_mem),
    .ex_ready_i         (wb_valid_from_ex),
    .mem_reg_wr_en_i    (wb_reg_wr_en_from_mem),
    .alu_rob_idx_i      (alu_rob_instr_idx_q),
    .mem_rob_idx_i      (mem_rob_instr_idx_q),
    .ex_rob_idx_i       (ex5_rob_instr_idx_q),
    .alu_wr_reg_i       (alu_wr_reg_q),
    .mem_wr_reg_i       (wb_wr_reg_from_mem),
    .ex_wr_reg_i        (wb_wr_reg_from_ex),
    .alu_result_i       (alu_data_to_reg),
    .ex_result_i        (wb_data_from_ex),
    .data_from_mem_i    (wb_data_from_mem),
    .mem_excpt_tval_i   (wb_excpt_tval_from_mem),
    .mem_excpt_cause_i  (wb_excpt_cause_from_mem)
`ifndef SYNTHESIS
    , .debug_alu_pc_i     (alu_pc_q),
    .debug_mem_pc_i     (debug_wb_pc_from_mem),
    .debug_ex_pc_i      (debug_wb_pc_from_ex),
    .debug_alu_instr_i  (debug_mem_instr_d),
    .debug_mem_instr_i  (debug_wb_instr_from_mem),
    .debug_ex_instr_i   (debug_wb_instr_from_ex),
`endif
    .reg_wr_en_o        (wb_reg_wr_en),
    .instr_with_excpt_o (instr_with_excpt),
    .instr_is_completed_o (instr_complete),
    .ex_allowed_wb_o    (ex_allowed_wb),
    .alu_allowed_wb_o   (alu_allowed_wb),
    .rob_idx_o          (rob_instr_complete_idx),
    .wr_reg_o           (wb_wr_reg),
    .data_to_reg_o      (wb_data_to_reg),
    .instr_excpt_tval_o (instr_complete_excpt_tval),
    .instr_excpt_cause_o (instr_complete_excp_cause)
`ifndef SYNTHESIS
    , .debug_pc_o         (debug_wb_pc),
    .debug_instr_o      (debug_wb_instr)
`endif
  );

  csr_regfile csr_file (
    .clk_i                  (clk_i),
    .rst_i                  (rst_i),
    .excpt_we_i             (rob_excp_we),
    .csr_instr_we_i         (rob_instr_commit_valid & rob_instr_commit_is_csr_wb),
    .fetch_present_req_i    (fetch_present_table_req),
    .mem_present_req_i      (mem_present_table_req),
    .rd_addr_i              (dec_csr_addr),
    .csr_wraddr_i           (rob_instr_commit_csr_addr),
    .fetch_present_ppn_i    (fetch_present_table_ppn),
    .mem_present_ppn_i      (mem_present_table_ppn),
    .csr_wrdata_i           (rob_instr_commit_data),
    .excpt_mepc_i           (rob_excp_pc),
    .excpt_mtval_i          (rob_excp_tval),
    .excpt_mcause_i         (rob_excp_cause),
    .fetch_ppn_is_present_o (fetch_ppn_is_present),
    .mem_ppn_is_present_o   (mem_ppn_is_present),
    .data_o                 (dec_csr_data),
    .satp_o                 (satp_data),
    .mtvec_o                (mtvec_data),
    .mepc_o                 (mepc_data)
`ifndef SYNTHESIS
    , .debug_satp_o         (debug_satp_o)
`endif
  );

  register_file #(
    .REGISTER_WIDTH (REGISTER_WIDTH),
    .DATA_WIDTH     (DATA_WIDTH)
  ) future_file (
    .clk_i          (clk_i),
    .rst_i          (rst_i),
    .wr_en_i        (wb_reg_wr_en & !flush),
    .rd_reg_a_i     (hazard_signals.rs1),
    .rd_reg_b_i     (hazard_signals.rs2),
    .wr_reg_i       (wb_wr_reg),
    .wr_data_i      (wb_data_to_reg),
    .reg_a_data_o   (ff_data_a),
    .reg_b_data_o   (ff_data_b)
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
    .wr_en_i      (rob_instr_commit_valid & rob_instr_commit_is_wb),
    .rd_reg_a_i   (hazard_signals.rs1),
    .rd_reg_b_i   (hazard_signals.rs2),
    .wr_reg_i     (rob_instr_commit_reg_id),
    .wr_data_i    (rob_instr_commit_is_csr_wb ? rob_instr_commit_csr_data : rob_instr_commit_data),
    .reg_a_data_o (af_data_a),
    .reg_b_data_o (af_data_b),
`ifndef SYNTHESIS
    .debug_regs_o (debug_regs_o)
`endif
  );

  reorder_buffer rob (
    .clk_i                    (clk_i),
    .rst_i                    (rst_i),
    .new_instr_valid_i        (rob_new_instr_valid),
    .new_instr_is_wb_i        (dec_instr_is_wb),
    .new_instr_is_csr_wb_i    (dec_instr_is_csr_wb),
    .new_instr_is_mret_i      (dec_instr_is_mret),
    .instr_complete_excpt_i   (instr_with_excpt),
    .instr_complete_valid_i   (instr_complete),
    .instr_excp_valid_i       (dec_excpt_q),
    .new_instr_reg_id_i       (dec_wr_reg),
    .new_instr_csr_addr_i     (dec_csr_addr),
    .instr_complete_idx_i     (rob_instr_complete_idx),
    .new_instr_pc_i           (dec_pc_q),
    .instr_excp_tval_i        (dec_pc_q),
    .instr_complete_excpt_tval_i (instr_complete_excpt_tval),
    .new_instr_csr_data_i     (dec_csr_data),
    .instr_complete_data_i    (wb_data_to_reg),
    .instr_excp_cause_i       (dec_excpt_cause_q),
    .instr_complete_excpt_cause_i (instr_complete_excp_cause)
`ifndef SYNTHESIS
    , .new_instr_i            (dec_instruction_q)
`endif
    , .full_o                 (rob_is_full),
    .excp_we_o                (rob_excp_we),
    .flush_o                  (flush),
    .instr_commit_valid_o     (rob_instr_commit_valid),
    .instr_commit_is_wb_o     (rob_instr_commit_is_wb),
    .instr_commit_is_csr_wb_o (rob_instr_commit_is_csr_wb),
    .instr_commit_is_mret_o   (rob_instr_commit_is_mret),
    .instr_commit_reg_id_o    (rob_instr_commit_reg_id),
    .instr_commit_csr_addr_o  (rob_instr_commit_csr_addr),
    .new_instr_idx_o          (dec_rob_new_instr_idx),
    .excp_pc_o                (rob_excp_pc),
    .excp_tval_o              (rob_excp_tval),
    .instr_commit_data_o      (rob_instr_commit_data),
    .instr_commit_csr_data_o  (rob_instr_commit_csr_data),
    .instr_commit_pc_o        (rob_instr_commit_pc),
    .excp_cause_o             (rob_excp_cause)
`ifndef SYNTHESIS
    , .instr_commit_o         (debug_rob_instr_commit)
`endif
  );

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      vm_en               <= 1'b0;
      trap_bypass_mmu     <= 1'b0;
      dec_valid_q         <= 1'b0;
      dec_excpt_q         <= 1'b0;
      is_jump             <= 1'b0;
      alu_branch_taken    <= 1'b0;
      alu_valid_q         <= 1'b0;
      mem_valid_q         <= 1'b0;
      ex1_valid_q         <= 1'b0;
      ex2_valid_q         <= 1'b0;
      ex3_valid_q         <= 1'b0;
      ex4_valid_q         <= 1'b0;
      ex5_valid_q         <= 1'b0;
      ff_valid            <= '0;
    end else begin
      if (rob_instr_commit_is_csrrw_satp) begin
        vm_en <= 1'b1;
      end

      if (rob_excp_we) begin
        trap_bypass_mmu <= 1'b1;
      end else if (rob_instr_commit_is_mret) begin
        trap_bypass_mmu <= 1'b0;
      end

      if (flush) begin
        ff_valid <= '0;
      end else if (wb_reg_wr_en) begin
        ff_valid[wb_wr_reg] <= 1'b1;
      end

      // Fetch -> Decode flops
      if (flush || alu_branch_taken || is_jump) begin
        dec_valid_q       <= 1'b0;
      end else if (!dec_stall) begin
        dec_valid_q       <= dec_valid_d;
        dec_excpt_q       <= dec_excpt_d;
        dec_excpt_cause_q <= dec_excpt_cause_d;
        dec_pc_q          <= dec_pc_d;
        dec_instruction_q <= dec_instruction_d;
      end

      // Decode -> ALU flops
      if (flush || alu_bubble) begin
        alu_valid_q              <= 1'b0;
      end else if (!alu_stall) begin
        alu_valid_q              <= alu_valid_d;
        alu_pc_q                 <= dec_pc_q;
        alu_rob_instr_idx_q      <= dec_rob_new_instr_idx;
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
        ex1_rob_instr_idx_q      <= dec_rob_new_instr_idx;
        ex1_wr_reg_q             <= dec_wr_reg;
`ifndef SYNTHESIS
        ex1_debug_pc_q           <= dec_pc_q;
        ex1_debug_instr_q        <= dec_instruction_q;
`endif
      end

      // Internal EX flops
      ex2_valid_q         <= ex2_valid_d;
      ex3_valid_q         <= ex3_valid_d;
      ex4_valid_q         <= ex4_valid_d;
      ex5_valid_q         <= ex5_valid_d;
      ex2_rob_instr_idx_q <= ex1_rob_instr_idx_q;
      ex3_rob_instr_idx_q <= ex2_rob_instr_idx_q;
      ex4_rob_instr_idx_q <= ex3_rob_instr_idx_q;
      ex5_rob_instr_idx_q <= ex4_rob_instr_idx_q;
      ex2_wr_reg_q        <= ex2_wr_reg_d;
      ex3_wr_reg_q        <= ex3_wr_reg_d;
      ex4_wr_reg_q        <= ex4_wr_reg_d;
      ex5_wr_reg_q        <= ex5_wr_reg_d;
      ex2_result_q        <= ex2_result_d;
      ex3_result_q        <= ex3_result_d;
      ex4_result_q        <= ex4_result_d;
      ex5_result_q        <= ex5_result_d;
`ifndef SYNTHESIS
      ex2_debug_pc_q      <= ex2_debug_pc_d;
      ex3_debug_pc_q      <= ex3_debug_pc_d;
      ex4_debug_pc_q      <= ex4_debug_pc_d;
      ex5_debug_pc_q      <= ex5_debug_pc_d;
      ex2_debug_instr_q   <= ex2_debug_instr_d;
      ex3_debug_instr_q   <= ex3_debug_instr_d;
      ex4_debug_instr_q   <= ex4_debug_instr_d;
      ex5_debug_instr_q   <= ex5_debug_instr_d;
`endif

      // ALU -> MEM flops
      if (!mem_stall) begin
        mem_valid_q         <= mem_valid_d;
        mem_is_load_q       <= mem_is_load_d;
        mem_is_store_q      <= mem_is_store_d;
        mem_reg_wr_en_q     <= mem_reg_wr_en_d;
        mem_rob_instr_idx_q <= alu_rob_instr_idx_q;
        mem_wr_reg_q        <= alu_wr_reg_q;
        mem_alu_result_q    <= mem_alu_result_d;
        mem_rs2_data_q      <= mem_rs2_data_d;
        mem_access_size_q   <= mem_access_size_d;
`ifndef SYNTHESIS
        debug_mem_pc_q      <= alu_pc_q;
        debug_mem_instr_q   <= debug_mem_instr_d;
`endif
      end

`ifndef SYNTHESIS
      debug_vm_en_o              <= vm_en;
      debug_trap_bypass_mmu_o    <= trap_bypass_mmu;
      debug_instr_is_completed_o <= rob_instr_commit_valid;
      debug_pc_o                 <= rob_instr_commit_pc;
      debug_instr_o              <= debug_rob_instr_commit;
`endif
    end
  end

  always_comb begin : rob_new_instr
    logic alu_can_be_issued;
    logic ex_can_be_issued;

    alu_can_be_issued = alu_valid_d && !alu_bubble && !alu_stall;
    ex_can_be_issued  = ex1_valid_d && !ex_bubble && !ex_stall;

    rob_new_instr_valid = alu_can_be_issued || ex_can_be_issued;
  end

  assign dec_rs1_data = ff_valid[hazard_signals.rs1] ? ff_data_a : af_data_a;
  assign dec_rs2_data = ff_valid[hazard_signals.rs2] ? ff_data_b : af_data_b;

  assign rd_req_valid_o = mem_req & ~mem_we;
  assign wr_req_valid_o = mem_req &  mem_we;
  assign req_is_instr_o = (icache_gnt);
  assign req_address_o  = mem_addr;
  assign wr_data_o      = mem_wdata;

  always_comb begin : rob_instr_commit
    next_pc_flush = '0;

    rob_instr_commit_is_csrrw_satp = (rob_instr_commit_valid && rob_instr_commit_is_csr_wb &&
                                      rob_instr_commit_csr_addr == CSR_SATP);

    if (flush) begin
      if (rob_instr_commit_is_mret) begin
        next_pc_flush = mepc_data;
      end else if (rob_excp_we) begin
        next_pc_flush = mtvec_data;
      end else begin
        next_pc_flush = (rob_instr_commit_pc + 4) % MEM_SIZE;
      end
    end
  end

endmodule
