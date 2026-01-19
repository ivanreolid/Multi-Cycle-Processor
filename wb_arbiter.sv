import params_pkg::*;

module wb_arbiter (
  input  logic alu_ready_i,
  input  logic alu_is_instr_wb_i,
  input  logic mem_ready_i,
  input  logic mem_excpt_i,
  input  logic ex_ready_i,
  input  logic mem_reg_wr_en_i,
  input  rob_idx_t alu_rob_idx_i,
  input  rob_idx_t mem_rob_idx_i,
  input  rob_idx_t ex_rob_idx_i,
  input  reg_id_t alu_wr_reg_i,
  input  reg_id_t mem_wr_reg_i,
  input  reg_id_t ex_wr_reg_i,
  input  data_t alu_result_i,
  input  data_t ex_result_i,
  input  data_t data_from_mem_i,
  input  vaddr_t mem_excpt_tval_i,
  input  var excpt_cause_t mem_excpt_cause_i,
`ifndef SYNTHESIS
  input  vaddr_t debug_alu_pc_i,
  input  vaddr_t debug_mem_pc_i,
  input  vaddr_t debug_ex_pc_i,
  input  var instruction_t debug_alu_instr_i,
  input  var instruction_t debug_mem_instr_i,
  input  var instruction_t debug_ex_instr_i,
`endif
  output logic reg_wr_en_o,
  output logic instr_with_excpt_o,
  output logic instr_is_completed_o,
  output logic ex_allowed_wb_o,
  output logic alu_allowed_wb_o,
  output rob_idx_t rob_idx_o,
  output reg_id_t wr_reg_o,
  output data_t data_to_reg_o,
  output vaddr_t instr_excpt_tval_o,
  output var excpt_cause_t instr_excpt_cause_o
`ifndef SYNTHESIS
  , output vaddr_t debug_pc_o,
  output var instruction_t debug_instr_o
`endif
);

  always_comb begin : wb_arbitration
    reg_wr_en_o        = 1'b0;
    wr_reg_o           = '0;
    data_to_reg_o      = '0;

    instr_is_completed_o = 1'b0;
    instr_with_excpt_o   = 1'b0;
    instr_excpt_tval_o   = '0;
    instr_excpt_cause_o  = '{default: 0};
    rob_idx_o            = '0;

    ex_allowed_wb_o    = 1'b1;
    alu_allowed_wb_o   = 1'b1;

    if (mem_ready_i) begin
      wr_reg_o             = mem_wr_reg_i;
      data_to_reg_o        = data_from_mem_i;
      ex_allowed_wb_o      = 1'b0;
      alu_allowed_wb_o     = 1'b0;
      rob_idx_o            = mem_rob_idx_i;
      if (mem_excpt_i) begin
        instr_with_excpt_o  = 1'b1;
        instr_excpt_tval_o  = mem_excpt_tval_i;
        instr_excpt_cause_o = mem_excpt_cause_i;
      end else begin
        instr_is_completed_o = 1'b1;
        reg_wr_en_o          = mem_reg_wr_en_i;
      end
`ifndef SYNTHESIS
      debug_pc_o    = debug_mem_pc_i;
      debug_instr_o = debug_mem_instr_i;
`endif
    end else if (ex_ready_i) begin
      reg_wr_en_o       = 1'b1;
      wr_reg_o          = ex_wr_reg_i;
      data_to_reg_o     = ex_result_i;
      instr_is_completed_o = 1'b1;
      alu_allowed_wb_o  = 1'b0;
      rob_idx_o         = ex_rob_idx_i;
`ifndef SYNTHESIS
      debug_pc_o    = debug_ex_pc_i;
      debug_instr_o = debug_ex_instr_i;
`endif
    end else if (alu_ready_i) begin
      reg_wr_en_o        = alu_is_instr_wb_i;
      wr_reg_o           = alu_wr_reg_i;
      data_to_reg_o      = alu_result_i;
      instr_is_completed_o = 1'b1;
      rob_idx_o          = alu_rob_idx_i;
`ifndef SYNTHESIS
      debug_pc_o    = debug_alu_pc_i;
      debug_instr_o = debug_alu_instr_i;
`endif
    end
  end

endmodule : wb_arbiter
