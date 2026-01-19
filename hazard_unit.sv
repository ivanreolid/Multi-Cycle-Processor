import params_pkg::*;

module hazard_unit (
  input  logic rob_is_full_i,
  input  logic dec_valid_i,
  input  logic alu_valid_i,
  input  logic alu_instr_finishes_i,
  input  logic alu_branch_taken_i,
  input  logic alu_is_jump_i,
  input  logic alu_is_load_i,
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
  input  reg_id_t alu_wr_reg_i,
  input  reg_id_t mem_wr_reg_i,
  input  reg_id_t ex1_wr_reg_i,
  input  reg_id_t ex2_wr_reg_i,
  input  reg_id_t ex3_wr_reg_i,
  input  reg_id_t ex4_wr_reg_i,
  input  hazard_ctrl_t hazard_signals_i,
  output logic stall_ex5_o,
  output logic stall_ex4_o,
  output logic stall_ex3_o,
  output logic stall_ex2_o,
  output logic stall_ex1_o,
  output logic stall_alu_o,
  output logic stall_decode_o,
  output logic stall_fetch_o,
  output logic alu_bubble_o,
  output logic ex_bubble_o
);

  logic rs1_needed, rs2_needed;
  reg_id_t rs1, rs2;

  logic alu_raw_hazard_rs1, alu_raw_hazard_rs2;
  logic ex_raw_hazard_rs1, ex_raw_hazard_rs2;
  logic mem_raw_hazard_rs1, mem_raw_hazard_rs2;

  logic ex_stage_is_busy;

  assign ex_stage_is_busy = ex1_valid_i || ex2_valid_i || ex3_valid_i || ex4_valid_i || ex5_valid_i;

  always_comb begin : decode_raw_hazard
    alu_raw_hazard_rs1 = 1'b0;
    alu_raw_hazard_rs2 = 1'b0;
    ex_raw_hazard_rs1  = 1'b0;
    ex_raw_hazard_rs2  = 1'b0;
    mem_raw_hazard_rs1 = 1'b0;
    mem_raw_hazard_rs2 = 1'b0;

    if (dec_valid_i && alu_valid_i && alu_is_load_i) begin
      alu_raw_hazard_rs1 = hazard_signals_i.rs1_needed & (alu_wr_reg_i == hazard_signals_i.rs1);
      alu_raw_hazard_rs2 = hazard_signals_i.rs2_needed & (alu_wr_reg_i == hazard_signals_i.rs2);
    end

    if (dec_valid_i && hazard_signals_i.rs1_needed && hazard_signals_i.rs1 != X0) begin
      if ( (ex1_valid_i && ex1_wr_reg_i == hazard_signals_i.rs1) ||
           (ex2_valid_i && ex2_wr_reg_i == hazard_signals_i.rs1) ||
           (ex3_valid_i && ex3_wr_reg_i == hazard_signals_i.rs1) ||
           (ex4_valid_i && ex4_wr_reg_i == hazard_signals_i.rs1) ) begin
             ex_raw_hazard_rs1 = 1'b1;
      end
    end

    if (dec_valid_i && hazard_signals_i.rs2_needed && hazard_signals_i.rs2 != X0) begin
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

  always_comb begin : ex_stall
    stall_ex5_o = !ex_allowed_wb_i && ex5_valid_i;
    stall_ex4_o = stall_ex5_o && ex4_valid_i;
    stall_ex3_o = stall_ex4_o && ex3_valid_i;
    stall_ex2_o = stall_ex3_o && ex2_valid_i;
    stall_ex1_o = stall_ex2_o && ex1_valid_i;
  end

  always_comb begin : alu_stall
    logic stall_reason_wb_contention;
    logic stall_reason_backpressure;

    stall_reason_wb_contention = !alu_allowed_wb_i && alu_instr_finishes_i;
    stall_reason_backpressure  = mem_busy_i && !alu_instr_finishes_i;

    stall_alu_o = alu_valid_i && (stall_reason_backpressure || stall_reason_wb_contention);
  end

  always_comb begin : decode_stall
    logic stall_reason_ex_backpressure;
    logic stall_reason_alu_backpressure;
    logic stall_reason_alu_raw_hazard;
    logic stall_reason_mem_raw_hazard;
    logic stall_reason_ex_raw_hazard;

    stall_reason_ex_backpressure  = hazard_signals_i.is_mul && stall_ex1_o;
    stall_reason_alu_backpressure = !hazard_signals_i.is_mul && stall_alu_o;
    stall_reason_alu_raw_hazard   = alu_raw_hazard_rs1 || alu_raw_hazard_rs2;
    stall_reason_mem_raw_hazard   = mem_raw_hazard_rs1 || mem_raw_hazard_rs2;
    stall_reason_ex_raw_hazard    = ex_raw_hazard_rs1 || ex_raw_hazard_rs2;

    stall_decode_o = rob_is_full_i ||
                     stall_reason_ex_backpressure ||
                     stall_reason_alu_backpressure ||
                     stall_reason_alu_raw_hazard ||
                     stall_reason_mem_raw_hazard ||
                     stall_reason_ex_raw_hazard;

    ex_bubble_o    = stall_decode_o && !stall_reason_ex_backpressure;
  end

  assign stall_fetch_o = stall_decode_o;

  assign alu_bubble_o = stall_decode_o && !stall_alu_o;

endmodule : hazard_unit
