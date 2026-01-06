module reorder_buffer import params_pkg::*; #(
  parameter int REGISTER_WIDTH  = 32,
  parameter int ROB_ENTRIES     = 4,
  parameter int ROB_ENTRY_WIDTH = 2,
  parameter int ADDR_WIDTH      = 32,
  parameter int DATA_WIDTH      = 32
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic new_instr_valid_i,
  input  logic new_instr_is_wb_i,
  input  logic instr_complete_valid_i,
  input  logic instr_excp_valid_i,
  input  logic [REGISTER_WIDTH-1:0] new_instr_reg_id_i,
  input  logic [ROB_ENTRY_WIDTH-1:0] instr_complete_idx_i,
  input  logic [ADDR_WIDTH-1:0] new_instr_pc_i,
  input  logic [DATA_WIDTH-1:0] instr_complete_data_i,
`ifndef SYNTHESIS
  input  var instruction_t new_instr_i,
`endif
  output logic full_o,
  output logic instr_commit_valid_o,
  output logic instr_commit_is_wb_o,
  output logic [REGISTER_WIDTH-1:0] instr_commit_reg_id_o,
  output logic [ROB_ENTRY_WIDTH-1:0] new_instr_idx_o,
  output logic [DATA_WIDTH-1:0] instr_commit_data_o
`ifndef SYNTHESIS
  , output logic [ADDR_WIDTH-1:0] instr_commit_pc_o,
  output var instruction_t instr_commit_o
`endif
);

  typedef struct packed {
    logic [DATA_WIDTH-1:0] data;
    logic [REGISTER_WIDTH-1:0] reg_id;
    logic wb;
    logic excp;
    logic valid;
    logic [ADDR_WIDTH-1:0] pc;
`ifndef SYNTHESIS
    instruction_t instr;
`endif
  } rob_data_t;

  rob_data_t [ROB_ENTRIES] rob_q, rob_d;

  logic [ADDR_WIDTH-1:0] instr_commit_pc_d;

  logic [ROB_ENTRY_WIDTH-1:0] head_q, head_d;
  logic [ROB_ENTRY_WIDTH-1:0] tail_d, tail_q;

  always_comb begin : rob_update
    rob_d                 = rob_q;
    head_d                = head_q;
    tail_d                = tail_q;
    new_instr_idx_o       = '0;
    instr_commit_valid_o  = 1'b0;
    instr_commit_is_wb_o  = 1'b0;
    instr_commit_reg_id_o = '0;
    instr_commit_data_o   = '0;
`ifndef SYNTHESIS
    instr_commit_pc_o     = '0;
    instr_commit_o        = '{default:0};
`endif

    if (new_instr_valid_i) begin
      rob_d[tail_q].valid  = 1'b0;
      rob_d[tail_q].excp   = 1'b0;
      rob_d[tail_q].wb     = new_instr_is_wb_i;
      rob_d[tail_q].pc     = new_instr_pc_i;
      rob_d[tail_q].reg_id = new_instr_reg_id_i;
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
      instr_commit_valid_o  = 1'b1;
      instr_commit_is_wb_o  = rob_q[head_q].wb;
      instr_commit_reg_id_o = rob_q[head_q].reg_id;
      instr_commit_data_o   = rob_q[head_q].data;
`ifndef SYNTHESIS
      instr_commit_pc_o     = rob_q[head_q].pc;
      instr_commit_o        = rob_q[head_q].instr;
`endif
      rob_d[head_q].valid   = 1'b0;
      head_d                = (head_q + 1) % ROB_ENTRIES;
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
    end
  end

  assign full_o = ((tail_q + 1) % ROB_ENTRIES) == head_q;

endmodule : reorder_buffer
