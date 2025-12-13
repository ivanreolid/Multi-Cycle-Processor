import params_pkg::*;

module hazard_unit #(
  parameter int REGISTER_WIDTH = params_pkg::REGISTER_WIDTH
)(
  input  logic dec_valid_i,
  input  logic mem_busy_i,
  input  logic wb_is_next_cycle_i,
  input  logic ex1_valid_i,
  input  logic ex2_valid_i,
  input  logic ex3_valid_i,
  input  logic ex4_valid_i,
  input  logic [REGISTER_WIDTH-1:0] ex1_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex2_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex3_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex4_wr_reg_i,
  input  var   instruction_t dec_instr_i,
  output logic stall_ex_o,
  output logic stall_mem_o,
  output logic stall_alu_o,
  output logic stall_decode_o,
  output logic stall_fetch_o
);

  logic rs1_needed, rs2_needed;
  logic [REGISTER_WIDTH-1:0] rs1, rs2;

  logic raw_hazard_rs1, raw_hazard_rs2, any_raw_hazard;

  assign stall_mem_o = mem_busy_i;
  assign stall_ex_o  = 1'b0;
  assign stall_alu_o = stall_mem_o;

  always_comb begin : decode_dependencies
    rs1 = dec_instr_i.rs1;
    rs2 = dec_instr_i.rs2;

    case (dec_instr_i.opcode)
      AUIPC, JAL: rs1_needed = 1'b0;
      default:    rs1_needed = 1'b1;
    endcase

    case (dec_instr_i.opcode)
      R, STORE, BRANCH: rs2_needed = 1'b1;
      default:          rs2_needed = 1'b0;
    endcase
  end

  always_comb begin : decode_raw_hazard
    raw_hazard_rs1 = 1'b0;
    raw_hazard_rs2 = 1'b0;

    if (dec_valid_i && rs1_needed && rs1 != '0) begin
      if ( (ex1_valid_i && ex1_wr_reg_i == rs1) ||
           (ex2_valid_i && ex2_wr_reg_i == rs1) ||
           (ex3_valid_i && ex3_wr_reg_i == rs1) ||
           (ex4_valid_i && ex4_wr_reg_i == rs1) ) begin
             raw_hazard_rs1 = 1'b1;
      end
    end

    if (dec_valid_i && rs2_needed && rs2 != '0) begin
      if ( (ex1_valid_i && ex1_wr_reg_i == rs2) ||
           (ex2_valid_i && ex2_wr_reg_i == rs2) ||
           (ex3_valid_i && ex3_wr_reg_i == rs2) ||
           (ex4_valid_i && ex4_wr_reg_i == rs2) ) begin
             raw_hazard_rs2 = 1'b1;
      end
    end

    any_raw_hazard = raw_hazard_rs1 | raw_hazard_rs2;
  end

  always_comb begin : decode_stall
    stall_decode_o = 1'b0;

    if (stall_alu_o || stall_ex_o) begin
      stall_decode_o = 1'b1;
    end else if (is_instr_wbalu(dec_instr_i) && wb_is_next_cycle_i) begin
      stall_decode_o = 1'b1;
    end else if (is_mem(dec_instr_i) && (ex1_valid_i || ex2_valid_i)) begin
      stall_decode_o = 1'b1;
    end else if (any_raw_hazard) begin
      stall_decode_o = 1'b1;
    end
  end

  assign stall_fetch_o = stall_decode_o;

  function automatic logic is_instr_wbalu(instruction_t instr);
    return (instr.opcode == IMMEDIATE || instr.opcode == AUIPC) ||
           (instr.opcode == JAL) || (instr.opcode == R && instr.funct7 != 7'b0000001);
  endfunction

  function automatic logic is_mem(instruction_t instr);
    return (instr.opcode == LOAD || instr.opcode == STORE);
  endfunction

endmodule : hazard_unit
