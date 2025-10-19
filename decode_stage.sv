import params_pkg::*;

module decode_stage #(
  parameter int DATA_WIDTH     = params_pkg::DATA_WIDTH,
  parameter int REGISTER_WIDTH = params_pkg::REGISTER_WIDTH,
  parameter int OPCODE_WIDTH   = params_pkg::OPCODE_WIDTH
)(
  input instruction_t instruction_i,
  output logic [DATA_WIDTH-1:0] offset_sign_extend_o,
  output logic [REGISTER_WIDTH-1:0] reg_a_o,
  output logic is_load_o,
  output logic is_store_o,
  output logic reg_wr_en_o,
  output logic [REGISTER_WIDTH-1:0] wr_reg_o,
  output logic [OPCODE_WIDTH-1:0] instr_opcode_o
);

  logic is_branch;

  assign offset_sign_extend_o = {{(DATA_WIDTH-(INSTR_WIDTH-14)){instruction_i[INSTR_WIDTH-1]}}, instruction_i[INSTR_WIDTH-1:14]};

  assign reg_a_o = is_store_o ? instruction_i.rd : instruction_i.ra; 

  assign is_load_o  = instruction_i.opcode == LW;
  assign is_store_o = instruction_i.opcode == SW;
  assign is_branch = (instruction_i.opcode == BEQ) | (instruction_i.opcode == BNE) | (instruction_i.opcode == BLT) | (instruction_i.opcode == BGE); 
  
  assign reg_wr_en_o = ~is_store_o & ~is_branch & instruction_i.opcode != JMP; 
  assign wr_reg_o = instruction_i.rd;

  assign instr_opcode_o = instruction_i.opcode;

endmodule : decode_stage
