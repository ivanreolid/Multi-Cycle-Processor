import params_pkg::*;

module hazard_unit #(
  parameter int REGISTER_WIDTH = params_pkg::REGISTER_WIDTH
)(
  input  logic rob_is_full_i,
  input  logic dec_valid_i,
  input  logic alu_valid_i,
  input  logic alu_instr_finishes_i,
  input  logic mem_valid_i,
  input  logic mem_busy_i,
  input  logic mem_reg_wr_en_i,
  input  logic ex_allowed_wb_i,
  input  logic alu_allowed_wb_i,
  input  logic ex1_valid_i,
  input  logic ex2_valid_i,
  input  logic ex3_valid_i,
  input  logic ex4_valid_i,
  input  logic ex5_valid_i,
  input  logic [REGISTER_WIDTH-1:0] mem_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex1_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex2_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex3_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex4_wr_reg_i,
  input  var   hazard_ctrl_t hazard_signals_i,
  output logic stall_ex_o,
  output logic stall_mem_o,
  output logic stall_alu_o,
  output logic stall_decode_o,
  output logic stall_fetch_o,
  output logic alu_bubble_o,
  output logic ex_bubble_o
);

  logic rs1_needed, rs2_needed;
  logic [REGISTER_WIDTH-1:0] rs1, rs2;

  logic ex_raw_hazard_rs1, ex_raw_hazard_rs2, mem_raw_hazard_rs1, mem_raw_hazard_rs2;

  logic ex_stage_is_busy, ex_stage_is_full;

  assign ex_stage_is_busy = ex1_valid_i || ex2_valid_i || ex3_valid_i || ex4_valid_i || ex5_valid_i;
  assign ex_stage_is_full = ex1_valid_i && ex2_valid_i && ex3_valid_i && ex4_valid_i && ex5_valid_i;

  assign stall_mem_o = mem_busy_i;
  assign stall_ex_o  = !ex_allowed_wb_i && ex_stage_is_full;

  always_comb begin : decode_raw_hazard
    ex_raw_hazard_rs1  = 1'b0;
    ex_raw_hazard_rs2  = 1'b0;
    mem_raw_hazard_rs1 = 1'b0;
    mem_raw_hazard_rs2 = 1'b0;

    if (dec_valid_i && hazard_signals_i.rs1_needed && hazard_signals_i.rs1 != '0) begin
      if ( (ex1_valid_i && ex1_wr_reg_i == hazard_signals_i.rs1) ||
           (ex2_valid_i && ex2_wr_reg_i == hazard_signals_i.rs1) ||
           (ex3_valid_i && ex3_wr_reg_i == hazard_signals_i.rs1) ||
           (ex4_valid_i && ex4_wr_reg_i == hazard_signals_i.rs1) ) begin
             ex_raw_hazard_rs1 = 1'b1;
      end
    end

    if (dec_valid_i && hazard_signals_i.rs2_needed && hazard_signals_i.rs2 != '0) begin
      if ( (ex1_valid_i && ex1_wr_reg_i == hazard_signals_i.rs2) ||
           (ex2_valid_i && ex2_wr_reg_i == hazard_signals_i.rs2) ||
           (ex3_valid_i && ex3_wr_reg_i == hazard_signals_i.rs2) ||
           (ex4_valid_i && ex4_wr_reg_i == hazard_signals_i.rs2) ) begin
             ex_raw_hazard_rs2 = 1'b1;
      end
    end

    if (dec_valid_i && mem_valid_i && mem_busy_i && mem_reg_wr_en_i) begin
      mem_raw_hazard_rs1 = hazard_signals_i.rs1_needed & (mem_wr_reg_i == hazard_signals_i.rs1);
      mem_raw_hazard_rs2 = hazard_signals_i.rs2_needed & (mem_wr_reg_i == hazard_signals_i.rs2);
    end
  end

  always_comb begin : alu_stall
    logic stall_reason_wb_contention;
    logic stall_reason_backpressure;

    stall_reason_wb_contention = !alu_allowed_wb_i && alu_instr_finishes_i;
    stall_reason_backpressure  = stall_mem_o && !alu_instr_finishes_i;

    stall_alu_o = alu_valid_i && (stall_reason_backpressure || stall_reason_wb_contention);
  end

  always_comb begin : decode_stall
    logic stall_reason_ex_backpressure;
    logic stall_reason_alu_backpressure;
    logic stall_reason_mem_raw_hazard;
    logic stall_reason_ex_raw_hazard;

    stall_reason_ex_backpressure  = hazard_signals_i.is_mul && stall_ex_o;
    stall_reason_alu_backpressure = !hazard_signals_i.is_mul && stall_alu_o;
    stall_reason_mem_raw_hazard   = mem_raw_hazard_rs1 || mem_raw_hazard_rs2;
    stall_reason_ex_raw_hazard    = ex_raw_hazard_rs1 || ex_raw_hazard_rs2;

    stall_decode_o = rob_is_full_i ||
                     stall_reason_ex_backpressure ||
                     stall_reason_alu_backpressure ||
                     stall_reason_mem_raw_hazard ||
                     stall_reason_ex_raw_hazard;

    alu_bubble_o   = stall_decode_o && !stall_reason_alu_backpressure;
    ex_bubble_o    = stall_decode_o && !stall_reason_ex_backpressure;
  end

  assign stall_fetch_o = stall_decode_o;

endmodule : hazard_unit
