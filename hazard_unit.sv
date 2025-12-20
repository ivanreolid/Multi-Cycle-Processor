import params_pkg::*;

module hazard_unit #(
  parameter int REGISTER_WIDTH = params_pkg::REGISTER_WIDTH
)(
  input  logic dec_valid_i,
  input  logic alu_valid_i,
  input  logic alu_instr_finishes_i,
  input  logic mem_busy_i,
  input  logic ex_allowed_wb_i,
  input  logic alu_allowed_wb_i,
  input  logic wb_is_next_cycle_i,
  input  logic ex1_valid_i,
  input  logic ex2_valid_i,
  input  logic ex3_valid_i,
  input  logic ex4_valid_i,
  input  logic ex5_valid_i,
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

  logic raw_hazard_rs1, raw_hazard_rs2, any_raw_hazard;

  logic ex_stage_is_busy;

  assign stall_mem_o = mem_busy_i;
  assign stall_ex_o  = !ex_allowed_wb_i;

  always_comb begin : decode_raw_hazard
    raw_hazard_rs1 = 1'b0;
    raw_hazard_rs2 = 1'b0;

    if (dec_valid_i && hazard_signals_i.rs1_needed && hazard_signals_i.rs1 != '0) begin
      if ( (ex1_valid_i && ex1_wr_reg_i == hazard_signals_i.rs1) ||
           (ex2_valid_i && ex2_wr_reg_i == hazard_signals_i.rs1) ||
           (ex3_valid_i && ex3_wr_reg_i == hazard_signals_i.rs1) ||
           (ex4_valid_i && ex4_wr_reg_i == hazard_signals_i.rs1) ) begin
             raw_hazard_rs1 = 1'b1;
      end
    end

    if (dec_valid_i && hazard_signals_i.rs2_needed && hazard_signals_i.rs2 != '0) begin
      if ( (ex1_valid_i && ex1_wr_reg_i == hazard_signals_i.rs2) ||
           (ex2_valid_i && ex2_wr_reg_i == hazard_signals_i.rs2) ||
           (ex3_valid_i && ex3_wr_reg_i == hazard_signals_i.rs2) ||
           (ex4_valid_i && ex4_wr_reg_i == hazard_signals_i.rs2) ) begin
             raw_hazard_rs2 = 1'b1;
      end
    end

    any_raw_hazard = raw_hazard_rs1 | raw_hazard_rs2;
  end

  assign ex_stage_is_busy = ex1_valid_i | ex2_valid_i | ex3_valid_i | ex4_valid_i | ex5_valid_i;

  always_comb begin : alu_stall
    stall_alu_o = 1'b0;

    if (alu_valid_i) begin
      if (stall_mem_o) begin
        stall_alu_o = 1'b1;
      end else if (!alu_allowed_wb_i) begin
        stall_alu_o = alu_instr_finishes_i;
      end
    end
  end

  always_comb begin : decode_stall
    stall_decode_o = 1'b0;
    alu_bubble_o   = 1'b0;
    ex_bubble_o    = 1'b0;

    if (stall_alu_o || stall_ex_o) begin
      stall_decode_o = 1'b1;
      // TODO: remove as soon as we allow instructions to complete out of order
      alu_bubble_o   = !stall_alu_o;
      ex_bubble_o    = 1'b1;
    end else if (any_raw_hazard) begin
      stall_decode_o = 1'b1;
      alu_bubble_o   = 1'b1;
      ex_bubble_o    = 1'b1;
    end
    // TODO: remove as soon as we allow instructions to complete out of order
    else if (hazard_signals_i.is_branch && ex_stage_is_busy) begin
      stall_decode_o = 1'b1;
      alu_bubble_o   = 1'b1;
    end else if (hazard_signals_i.is_instr_wb_alu && wb_is_next_cycle_i) begin
      stall_decode_o = 1'b1;
      alu_bubble_o   = 1'b1;
    end else if (hazard_signals_i.is_instr_mem && (ex1_valid_i || ex2_valid_i)) begin
      stall_decode_o = 1'b1;
    end
  end

  assign stall_fetch_o = stall_decode_o;

endmodule : hazard_unit
