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
  input  logic alu_instr_finishes_i,
  input  logic mem_stall_i,
  input  logic wb_reg_wr_en_i,
  input  logic [REGISTER_WIDTH-1:0] wb_wr_reg_i,
  input  logic [ADDR_WIDTH-1:0] pc_i,
  input  logic [DATA_WIDTH-1:0] rs1_data_i,
  input  logic [DATA_WIDTH-1:0] rs2_data_i,
  input  logic [DATA_WIDTH-1:0] wb_data_to_reg_i,
  input  instruction_t instruction_i,
  output logic alu_valid_o,
  output logic [DATA_WIDTH-1:0] offset_sign_extend_o,
  output logic [REGISTER_WIDTH-1:0] rs1_o,
  output logic [REGISTER_WIDTH-1:0] rs2_o,
  output logic [ADDR_WIDTH-1:0] alu_pc_o,
  output logic [DATA_WIDTH-1:0] alu_rs1_data_o,
  output logic [DATA_WIDTH-1:0] alu_rs2_data_o,
  output instruction_t instruction_o,
`ifndef SYNTHESIS
  output logic [ADDR_WIDTH-1:0] debug_alu_pc_o
`endif
);

  logic is_branch, is_store;

  logic [DATA_WIDTH-1:0] alu_rs1_data_d, alu_rs2_data_d;
  logic [DATA_WIDTH-1:0] offset_sign_extend_d;

  logic [ADDR_WIDTH-1:0] branch_offset_d;

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      alu_valid_o          <= 1'b0;
    end else if (!mem_stall_i) begin
      alu_valid_o          <= valid_i & ~is_jump_i & ~branch_taken_i;
      alu_pc_o             <= pc_i;
      alu_rs1_data_o       <= alu_rs1_data_d;
      alu_rs2_data_o       <= alu_rs2_data_d;
      offset_sign_extend_o <= offset_sign_extend_d;
      instruction_o        <= instruction_i;
`ifndef SYNTHESIS
      debug_alu_pc_o       <= pc_i;
`endif
    end
  end

  always_comb begin : offset_computation
    case (instruction_i.opcode)
      LOAD   : offset_sign_extend_d = {{20{instruction_i[31]}}, instruction_i[31:20]};
      STORE  : offset_sign_extend_d = {{20{instruction_i[31]}}, instruction_i[31:25],
                                      instruction_i[11:7]};
      BRANCH : offset_sign_extend_d = {{19{instruction_i[31]}}, instruction_i[31],
                                      instruction_i[7], instruction_i[30:25],
                                      instruction_i[11:8], 1'b0};
      JAL    : offset_sign_extend_d = {{11{instruction_i[31]}}, instruction_i[31],
                                      instruction_i[19:12], instruction_i[20], instruction_i[30:21],
                                      1'b0};
      default: offset_sign_extend_d = '0;
    endcase
  end

  always_comb begin : bypass_computation
    logic is_bypass_wb_rs1;
    logic is_bypass_wb_rs2;

    is_bypass_wb_rs1 = wb_reg_wr_en_i && (instruction_i.rs1 == wb_wr_reg_i);
    is_bypass_wb_rs2 = wb_reg_wr_en_i && (instruction_i.rs2 == wb_wr_reg_i);

    alu_rs1_data_d = is_bypass_wb_rs1 ? wb_data_to_reg_i : rs1_data_i;
    alu_rs2_data_d = is_bypass_wb_rs2 ? wb_data_to_reg_i : rs2_data_i;
  end

  assign rs1_o = instruction_i.rs1;
  assign rs2_o = instruction_i.rs2;

endmodule : decode_stage
