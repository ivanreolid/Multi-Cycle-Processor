import params_pkg::*;

module control #(
  parameter int OPCODE_WIDTH = params_pkg::OPCODE_WIDTH
)(
  input  logic is_zero_i,
  input  logic is_less_i,
  input  logic [OPCODE_WIDTH-1:0] opcode_i,
  output logic is_load_o,
  output logic is_store_o,
  output logic is_jump_o,
  output logic branch_taken_o,
  output logic reg_wr_en_o
);

  logic is_branch;

  assign is_load_o   = opcode_i == LW;
  assign is_store_o  = opcode_i == SW;
  assign is_jump_o   = opcode_i == JMP;
  assign is_branch = (opcode_i == BEQ) | (opcode_i == BNE) | (opcode_i == BLT) | (opcode_i == BGE);

  assign branch_taken_o = ((opcode_i == BEQ) & is_zero_i) | ((opcode_i == BNE) & ~is_zero_i) | ((opcode_i == BLT) & is_less_i) | ((opcode_i == BGE) & ~is_less_i);
  
  assign reg_wr_en_o = ~is_store_o & ~is_branch & ~is_jump_o;

endmodule : control
