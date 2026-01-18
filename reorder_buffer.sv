module reorder_buffer import params_pkg::*; #(
  parameter int REGISTER_WIDTH  = params_pkg::REGISTER_WIDTH,
  parameter int ROB_ENTRIES     = params_pkg::ROB_ENTRIES,
  parameter int ROB_ENTRY_WIDTH = params_pkg::ROB_ENTRY_WIDTH,
  parameter int CSR_ADDR_WIDTH  = params_pkg::CSR_ADDR_WIDTH,
  parameter int ADDR_WIDTH      = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH      = params_pkg::DATA_WIDTH
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic new_instr_valid_i,
  input  logic new_instr_is_wb_i,
  input  logic new_instr_is_csr_wb_i,
  input  logic new_instr_is_mret_i,
  input  logic instr_complete_valid_i,
  input  logic instr_excp_valid_i,
  input  logic [REGISTER_WIDTH-1:0] new_instr_reg_id_i,
  input  logic [CSR_ADDR_WIDTH-1:0] new_instr_csr_addr_i,
  input  logic [ROB_ENTRY_WIDTH-1:0] instr_complete_idx_i,
  input  logic [ADDR_WIDTH-1:0] new_instr_pc_i,
  input  logic [ADDR_WIDTH-1:0] instr_excp_tval_i,
  input  logic [DATA_WIDTH-1:0] new_instr_csr_data_i,
  input  logic [DATA_WIDTH-1:0] instr_complete_data_i,
  input  var excpt_cause_t instr_excp_cause_i
`ifndef SYNTHESIS
  , input  var instruction_t new_instr_i
`endif
  , output logic full_o,
  output logic excp_we_o,
  output logic flush_o,
  output logic instr_commit_valid_o,
  output logic instr_commit_is_wb_o,
  output logic instr_commit_is_csr_wb_o,
  output logic instr_commit_is_mret_o,
  output logic [REGISTER_WIDTH-1:0] instr_commit_reg_id_o,
  output logic [CSR_ADDR_WIDTH-1:0] instr_commit_csr_addr_o,
  output logic [ROB_ENTRY_WIDTH-1:0] new_instr_idx_o,
  output logic [DATA_WIDTH-1:0] excp_pc_o,
  output logic [DATA_WIDTH-1:0] excp_tval_o,
  output logic [DATA_WIDTH-1:0] instr_commit_data_o,
  output logic [DATA_WIDTH-1:0] instr_commit_csr_data_o,
  output logic [ADDR_WIDTH-1:0] instr_commit_pc_o,
  output var excpt_cause_t excp_cause_o
`ifndef SYNTHESIS
  , output var instruction_t instr_commit_o
`endif
);

  typedef struct packed {
    logic [DATA_WIDTH-1:0] data;
    logic [DATA_WIDTH-1:0] csr_data;
    logic [REGISTER_WIDTH-1:0] reg_id;
    logic [CSR_ADDR_WIDTH-1:0] csr_addr;
    logic wb;
    logic csr_wb;
    logic mret;
    logic excp;
    logic valid;
    logic [ADDR_WIDTH-1:0] pc;
    logic [ADDR_WIDTH-1:0] excp_tval;
    excpt_cause_t excp_cause;
`ifndef SYNTHESIS
    instruction_t instr;
`endif
  } rob_data_t;

  rob_data_t [ROB_ENTRIES] rob_q, rob_d;

  logic [ADDR_WIDTH-1:0] instr_commit_pc_d;

  logic [ROB_ENTRY_WIDTH-1:0] head_q, head_d;
  logic [ROB_ENTRY_WIDTH-1:0] tail_d, tail_q;

  always_comb begin : rob_update
    rob_d                    = rob_q;
    head_d                   = head_q;
    tail_d                   = tail_q;
    excp_we_o                = 1'b0;
    flush_o                  = 1'b0;
    instr_commit_valid_o     = 1'b0;
    instr_commit_is_wb_o     = 1'b0;
    instr_commit_is_csr_wb_o = 1'b0;
    instr_commit_is_mret_o   = 1'b0;
    excp_cause_o             = '{default: 0};
    excp_pc_o                = '0;
    excp_tval_o              = '0;
    instr_commit_csr_addr_o  = '0;
    instr_commit_csr_data_o  = '0;

    if (new_instr_valid_i) begin
      rob_d[tail_q].valid      = 1'b0;
      rob_d[tail_q].excp       = instr_excp_valid_i;
      rob_d[tail_q].wb         = new_instr_is_wb_i;
      rob_d[tail_q].csr_wb     = new_instr_is_csr_wb_i;
      rob_d[tail_q].mret       = new_instr_is_mret_i;
      rob_d[tail_q].excp_cause = instr_excp_cause_i;
      rob_d[tail_q].excp_tval  = instr_excp_tval_i;
      rob_d[tail_q].pc         = new_instr_pc_i;
      rob_d[tail_q].reg_id     = new_instr_reg_id_i;
      rob_d[tail_q].csr_addr   = new_instr_csr_addr_i;
      rob_d[tail_q].csr_data   = new_instr_csr_data_i;
`ifndef SYNTHESIS
      rob_d[tail_q].instr  = new_instr_i;
`endif
      new_instr_idx_o      = tail_q;
      tail_d               = (tail_q + 1) % ROB_ENTRIES;
    end

    if (instr_complete_valid_i) begin
      rob_d[instr_complete_idx_i].valid = 1'b1;
      rob_d[instr_complete_idx_i].excp  = instr_excp_valid_i;
      rob_d[instr_complete_idx_i].data  = instr_complete_data_i;
    end

    if (rob_q[head_q].valid) begin
      instr_commit_valid_o     = 1'b1;
      flush_o                  = rob_q[head_q].mret ||
                                 (rob_q[head_q].csr_wb && rob_q[head_q].csr_addr == CSR_SATP);
      instr_commit_is_wb_o     = rob_q[head_q].wb;
      instr_commit_is_csr_wb_o = rob_q[head_q].csr_wb;
      instr_commit_is_mret_o   = rob_q[head_q].mret;
      instr_commit_reg_id_o    = rob_q[head_q].reg_id;
      instr_commit_csr_addr_o  = rob_q[head_q].csr_addr;
      instr_commit_data_o      = rob_q[head_q].data;
      instr_commit_csr_data_o  = rob_q[head_q].csr_data;
      instr_commit_pc_o        = rob_q[head_q].pc;
`ifndef SYNTHESIS
      instr_commit_o        = rob_q[head_q].instr;
`endif
      rob_d[head_q].valid   = 1'b0;
      head_d                = (head_q + 1) % ROB_ENTRIES;
    end

    if (rob_q[head_q].excp) begin
      flush_o      = 1'b1;
      excp_we_o    = 1'b1;
      excp_cause_o = rob_q[head_q].excp_cause;
      excp_pc_o    = rob_q[head_q].pc;
      excp_tval_o  = rob_q[head_q].excp_tval;
      head_d        = (head_q + 1) % ROB_ENTRIES;
    end

    if (flush_o) begin
      tail_d = head_d;
    end
  end

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      head_q                <= '0;
      tail_q                <= '0;
      rob_q                 <= '0;
    end else begin
      head_q                <= head_d;
      tail_q                <= tail_d;

      rob_q                 <= rob_d;

      if (flush_o) begin
        for (int i = 0; i < ROB_ENTRIES; i++) begin
          rob_q[i].valid <= 1'b0;
          rob_q[i].excp  <= 1'b0;
        end
      end
    end
  end

  assign full_o = ((tail_q + 1) % ROB_ENTRIES) == head_q;

endmodule : reorder_buffer
