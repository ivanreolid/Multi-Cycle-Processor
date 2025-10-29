import params_pkg::*;

module decode_stage #(
  parameter int DATA_WIDTH     = params_pkg::DATA_WIDTH,
  parameter int ADDR_WIDTH     = params_pkg::ADDR_WIDTH,
  parameter int REGISTER_WIDTH = params_pkg::REGISTER_WIDTH,
  parameter int OPCODE_WIDTH   = params_pkg::OPCODE_WIDTH
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic valid_i,
  input  logic is_jump_i,
  input  logic branch_taken_i,
  input  logic mem_stall_i,
  input  logic [ADDR_WIDTH-1:0] pc_i,
  input  logic [DATA_WIDTH-1:0] reg_a_data_i,
  input  logic [DATA_WIDTH-1:0] reg_b_data_i,
  input  instruction_t instruction_i,
  output logic alu_valid_o,
  output logic [DATA_WIDTH-1:0] offset_sign_extend_o,
  output logic [REGISTER_WIDTH-1:0] reg_a_o,
  output logic reg_wr_en_o,
  output logic [REGISTER_WIDTH-1:0] wr_reg_o,
  output logic [OPCODE_WIDTH-1:0] instr_opcode_o,
  output logic [ADDR_WIDTH-1:0] branch_offset_o,
  output logic [ADDR_WIDTH-1:0] alu_pc_o,
  output logic [DATA_WIDTH-1:0] alu_reg_a_data_o,
  output logic [DATA_WIDTH-1:0] alu_reg_b_data_o,
`ifndef SYNTHESIS
  output logic [ADDR_WIDTH-1:0] debug_alu_pc_o,
  output instruction_t debug_alu_instr_o
`endif
);

  logic is_branch, is_store;

  logic [DATA_WIDTH-1:0] offset_sign_extend_d;

  logic reg_wr_en_d;

  logic [ADDR_WIDTH-1:0] branch_offset_d;

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      alu_valid_o          <= 1'b0;
      reg_wr_en_o          <= 1'b0;
    end else if (!mem_stall_i) begin
      alu_valid_o          <= valid_i & ~is_jump_i & ~branch_taken_i;
      reg_wr_en_o          <= reg_wr_en_d;
      offset_sign_extend_o <= offset_sign_extend_d;
      wr_reg_o             <= instruction_i.rd;
      instr_opcode_o       <= instruction_i.opcode;
      branch_offset_o      <= branch_offset_d;
      alu_pc_o             <= pc_i;
      alu_reg_a_data_o     <= reg_a_data_i;
      alu_reg_b_data_o     <= reg_b_data_i;
`ifndef SYNTHESIS
      debug_alu_pc_o       <= pc_i;
      debug_alu_instr_o    <= instruction_i;
`endif
    end
  end

  assign offset_sign_extend_d = {{(DATA_WIDTH-(INSTR_WIDTH-14)){instruction_i[INSTR_WIDTH-1]}},
                                instruction_i[INSTR_WIDTH-1:14]};

  assign reg_a_o = is_store ? instruction_i.rd : instruction_i.ra;

  assign is_store = instruction_i.opcode == SW;
  assign is_branch = (instruction_i.opcode == BEQ) | (instruction_i.opcode == BNE) |
                     (instruction_i.opcode == BLT) | (instruction_i.opcode == BGE);

  assign reg_wr_en_d = ~is_store & ~is_branch & instruction_i.opcode != JMP;

  assign branch_offset_d = {{(ADDR_WIDTH-18){instruction_i.free[12]}},
                           instruction_i.free, instruction_i.rd};

endmodule : decode_stage
