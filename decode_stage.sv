import params_pkg::*;

module decode_stage #(
  parameter int SHAMT_WIDTH    = params_pkg::SHAMT_WIDTH,
  parameter int DATA_WIDTH     = params_pkg::DATA_WIDTH,
  parameter int ADDR_WIDTH     = params_pkg::ADDR_WIDTH,
  parameter int REGISTER_WIDTH = params_pkg::REGISTER_WIDTH,
  parameter int OPCODE_WIDTH   = params_pkg::OPCODE_WIDTH
)(
  input  logic valid_i,
  input  logic is_jump_i,
  input  logic branch_taken_i,
  input  logic mem_stage_valid_i,
  input  logic wb_reg_wr_en_i,
  input  logic mem_stage_reg_wr_en_i,
  input  logic ex1_valid_i,
  input  logic ex2_valid_i,
  input  logic ex3_valid_i,
  input  logic ex4_valid_i,
  input  logic ex5_valid_i,
  input  logic [REGISTER_WIDTH-1:0] mem_stage_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] wb_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex1_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex2_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex3_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex4_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex5_wr_reg_i,
  input  logic [DATA_WIDTH-1:0] rs1_data_i,
  input  logic [DATA_WIDTH-1:0] rs2_data_i,
  input  logic [DATA_WIDTH-1:0] mem_stage_result_i,
  input  logic [DATA_WIDTH-1:0] ex5_result_i,
  input  logic [DATA_WIDTH-1:0] wb_data_to_reg_i,
  input  var   instruction_t instruction_i,
  output logic alu_valid_o,
  output logic ex_valid_o,
  output logic [SHAMT_WIDTH-1:0] shamt_o,
  output logic [DATA_WIDTH-1:0] offset_sign_extend_o,
  output logic [REGISTER_WIDTH-1:0] wr_reg_o,
  output logic [DATA_WIDTH-1:0] alu_rs1_data_o,
  output logic [DATA_WIDTH-1:0] alu_rs2_data_o,
  output var   hazard_ctrl_t hazard_signals_o
);

  logic valid_instruction;

  always_comb begin : decode_instruction
    hazard_signals_o.rs1             = instruction_i.rs1;
    hazard_signals_o.rs2             = instruction_i.rs2;
    hazard_signals_o.rs1_needed      = 1'b0;
    hazard_signals_o.rs2_needed      = 1'b0;
    hazard_signals_o.is_mul          = 1'b0;
    hazard_signals_o.is_instr_wb_alu = 1'b0;
    hazard_signals_o.is_instr_mem    = 1'b0;
    hazard_signals_o.is_branch       = 1'b0;

    case (instruction_i.opcode)
      R: begin
        hazard_signals_o.rs1_needed      = 1'b1;
        hazard_signals_o.rs2_needed      = 1'b1;
        hazard_signals_o.is_mul          = instruction_i.funct3 == 3'b000 &&
                                           instruction_i.funct7 == 7'b0000001;
        hazard_signals_o.is_instr_wb_alu = instruction_i.funct7 != 7'b0000001;
      end
      LOAD: begin
        hazard_signals_o.rs1_needed      = 1'b1;
        hazard_signals_o.is_instr_mem    = 1'b1;
      end
      STORE: begin
        hazard_signals_o.rs1_needed      = 1'b1;
        hazard_signals_o.rs2_needed      = 1'b1;
        hazard_signals_o.is_instr_mem    = 1'b1;
      end
      BRANCH: begin
        hazard_signals_o.rs1_needed      = 1'b1;
        hazard_signals_o.rs2_needed      = 1'b1;
        hazard_signals_o.is_branch       = 1'b1;
      end
      JAL: begin
        hazard_signals_o.is_instr_wb_alu = 1'b1;
      end
      IMMEDIATE: begin
        hazard_signals_o.rs1_needed      = 1'b1;
        hazard_signals_o.is_instr_wb_alu = 1'b1;
      end
      AUIPC: begin
        hazard_signals_o.is_instr_wb_alu = 1'b1;
      end
      default;
    endcase
  end

  always_comb begin : shamt_computation
    case (instruction_i.opcode)
      IMMEDIATE : shamt_o = {instruction_i[25], instruction_i.rs2};
      default   : shamt_o = '0;
    endcase
  end

  always_comb begin : offset_computation
    case (instruction_i.opcode)
      LOAD      : offset_sign_extend_o = {{20{instruction_i[31]}}, instruction_i[31:20]};
      STORE     : offset_sign_extend_o = {{20{instruction_i[31]}}, instruction_i[31:25],
                                         instruction_i[11:7]};
      BRANCH    : offset_sign_extend_o = {{19{instruction_i[31]}}, instruction_i[31],
                                         instruction_i[7], instruction_i[30:25],
                                         instruction_i[11:8], 1'b0};
      JAL       : offset_sign_extend_o = {{11{instruction_i[31]}}, instruction_i[31],
                                         instruction_i[19:12], instruction_i[20],
                                         instruction_i[30:21], 1'b0};
      IMMEDIATE : offset_sign_extend_o = {{20{instruction_i[31]}}, instruction_i[31:20]};
      AUIPC     : offset_sign_extend_o = {{instruction_i[31]}, instruction_i[31:12] << 12};
      default   : offset_sign_extend_o = '0;
    endcase
  end

  always_comb begin : bypass_computation
    logic is_bypass_wb_rs1;
    logic is_bypass_wb_rs2;
    logic is_bypass_mem_rs1;
    logic is_bypass_mem_rs2;
    logic is_bypass_ex5_rs1;
    logic is_bypass_ex5_rs2;

    is_bypass_mem_rs1 = hazard_signals_o.rs1_needed && mem_stage_valid_i &&
                        mem_stage_reg_wr_en_i && (instruction_i.rs1 == mem_stage_wr_reg_i);
    is_bypass_mem_rs2 = hazard_signals_o.rs2_needed && mem_stage_valid_i &&
                        (mem_stage_reg_wr_en_i && instruction_i.rs2 == mem_stage_wr_reg_i);

    is_bypass_ex5_rs1 = hazard_signals_o.rs1_needed && ex5_valid_i    &&
                        (instruction_i.rs1 == ex5_wr_reg_i);
    is_bypass_ex5_rs2 = hazard_signals_o.rs2_needed && ex5_valid_i    &&
                        (instruction_i.rs2 == ex5_wr_reg_i);
    is_bypass_wb_rs1  = hazard_signals_o.rs1_needed && wb_reg_wr_en_i &&
                        (instruction_i.rs1 == wb_wr_reg_i);
    is_bypass_wb_rs2  = hazard_signals_o.rs2_needed && wb_reg_wr_en_i &&
                        (instruction_i.rs2 == wb_wr_reg_i);

    alu_rs1_data_o = rs1_data_i;
    alu_rs2_data_o = rs2_data_i;

    if (is_bypass_mem_rs1) begin
      alu_rs1_data_o = mem_stage_result_i;
    end else if (is_bypass_ex5_rs1) begin
      alu_rs1_data_o = ex5_result_i;
    end else if (is_bypass_wb_rs1) begin
      alu_rs1_data_o = wb_data_to_reg_i;
    end

    if (is_bypass_mem_rs2) begin
      alu_rs2_data_o = mem_stage_result_i;
    end else if (is_bypass_ex5_rs2) begin
      alu_rs2_data_o = ex5_result_i;
    end else if (is_bypass_wb_rs2) begin
      alu_rs2_data_o = wb_data_to_reg_i;
    end
  end

  assign valid_instruction = valid_i & ~is_jump_i & ~branch_taken_i;

  assign alu_valid_o          = ~hazard_signals_o.is_mul & valid_instruction;
  assign ex_valid_o           = hazard_signals_o.is_mul & valid_instruction;
  assign wr_reg_o             = instruction_i.rd;

endmodule : decode_stage
