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
  input  logic wb_is_next_cycle_i,
  input  logic wb_reg_wr_en_i,
  input  logic ex1_valid_i,
  input  logic ex2_valid_i,
  input  logic ex3_valid_i,
  input  logic ex4_valid_i,
  input  logic ex5_valid_i,
  input  logic [REGISTER_WIDTH-1:0] wb_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex1_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex2_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex3_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex4_wr_reg_i,
  input  logic [REGISTER_WIDTH-1:0] ex5_wr_reg_i,
  input  logic [ADDR_WIDTH-1:0] pc_i,
  input  logic [DATA_WIDTH-1:0] rs1_data_i,
  input  logic [DATA_WIDTH-1:0] rs2_data_i,
  input  logic [DATA_WIDTH-1:0] ex5_result_i,
  input  logic [DATA_WIDTH-1:0] wb_data_to_reg_i,
  input  var instruction_t instruction_i,
  output logic stall_o,
  output logic alu_valid_o,
  output logic ex_valid_o,
  output logic [DATA_WIDTH-1:0] offset_sign_extend_o,
  output logic [REGISTER_WIDTH-1:0] rs1_o,
  output logic [REGISTER_WIDTH-1:0] rs2_o,
  output logic [REGISTER_WIDTH-1:0] ex_wr_reg_o,
  output logic [ADDR_WIDTH-1:0] alu_pc_o,
  output logic [DATA_WIDTH-1:0] alu_rs1_data_o,
  output logic [DATA_WIDTH-1:0] alu_rs2_data_o,
  output var instruction_t instruction_o,
`ifndef SYNTHESIS
  output logic [ADDR_WIDTH-1:0] debug_alu_pc_o,
  output logic [ADDR_WIDTH-1:0] debug_ex_pc_o,
  output instruction_t debug_ex_instr_o
`endif
);

  logic valid_instruction;
  logic instr_reads_rs1, instr_reads_rs2;
  logic ex_stage_busy;
  logic is_instr_wbalu;
  logic is_mul;
  logic send_nop;

  logic [DATA_WIDTH-1:0] alu_rs1_data_d, alu_rs2_data_d;
  logic [DATA_WIDTH-1:0] offset_sign_extend_d;

  logic [ADDR_WIDTH-1:0] branch_offset_d;

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      alu_valid_o          <= 1'b0;
      ex_valid_o           <= 1'b0;
    end else if (send_nop) begin
      alu_valid_o          <= 1'b0;
      ex_valid_o           <= 1'b0;
    end else if (!stall_o) begin
      alu_valid_o          <= ~is_mul & valid_instruction;
      ex_valid_o           <= is_mul & valid_instruction;
      ex_wr_reg_o          <= instruction_i.rd;
      alu_pc_o             <= pc_i;
      alu_rs1_data_o       <= alu_rs1_data_d;
      alu_rs2_data_o       <= alu_rs2_data_d;
      offset_sign_extend_o <= offset_sign_extend_d;
      instruction_o        <= instruction_i;
`ifndef SYNTHESIS
      debug_alu_pc_o       <= pc_i;
      debug_ex_pc_o        <= pc_i;
      debug_ex_instr_o     <= instruction_i;
`endif
    end
  end

  always_comb begin : offset_computation
    case (instruction_i.opcode)
      LOAD      : offset_sign_extend_d = {{20{instruction_i[31]}}, instruction_i[31:20]};
      STORE     : offset_sign_extend_d = {{20{instruction_i[31]}}, instruction_i[31:25],
                                         instruction_i[11:7]};
      BRANCH    : offset_sign_extend_d = {{19{instruction_i[31]}}, instruction_i[31],
                                         instruction_i[7], instruction_i[30:25],
                                         instruction_i[11:8], 1'b0};
      JAL       : offset_sign_extend_d = {{11{instruction_i[31]}}, instruction_i[31],
                                         instruction_i[19:12], instruction_i[20],
                                         instruction_i[30:21], 1'b0};
      IMMEDIATE : offset_sign_extend_d = {{20{instruction_i[31]}}, instruction_i[31:20]};
      AUIPC     : offset_sign_extend_d = {{instruction_i[31]}, instruction_i[31:12] << 12};
      default   : offset_sign_extend_d = '0;
    endcase
  end

  always_comb begin : bypass_computation
    logic is_bypass_wb_rs1;
    logic is_bypass_wb_rs2;
    logic is_bypass_ex5_rs1;
    logic is_bypass_ex5_rs2;

    is_bypass_ex5_rs1 = instr_reads_rs1 && ex5_valid_i    && (instruction_i.rs1 == ex5_wr_reg_i);
    is_bypass_ex5_rs2 = instr_reads_rs2 && ex5_valid_i    && (instruction_i.rs2 == ex5_wr_reg_i);
    is_bypass_wb_rs1  = instr_reads_rs1 && wb_reg_wr_en_i && (instruction_i.rs1 == wb_wr_reg_i);
    is_bypass_wb_rs2  = instr_reads_rs2 && wb_reg_wr_en_i && (instruction_i.rs2 == wb_wr_reg_i);

    alu_rs1_data_d = is_bypass_ex5_rs1 ? ex5_result_i : is_bypass_wb_rs1 ?
                                                             wb_data_to_reg_i : rs1_data_i;
    alu_rs2_data_d = is_bypass_ex5_rs2 ? ex5_result_i : is_bypass_wb_rs2 ?
                                                             wb_data_to_reg_i : rs2_data_i;
  end

  assign valid_instruction = valid_i & ~is_jump_i & ~branch_taken_i;

  assign instr_reads_rs1 = instruction_i.opcode != JAL && instruction_i.opcode != AUIPC;
  assign instr_reads_rs2 = instruction_i.opcode != JAL && instruction_i.opcode != AUIPC &&
                           instruction_i.opcode != LOAD;

  assign is_mul = instruction_i.opcode == R && instruction_i.funct3 == 3'b000 &&
                  instruction_i.funct7 == 7'b0000001;
  assign is_instr_wbalu = (instruction_i.opcode == R && ~is_mul) || (instruction_i.opcode == JAL) ||
                          (instruction_i.opcode == IMMEDIATE)    || (instruction_i.opcode == AUIPC);

  assign rs1_o = instruction_i.rs1;
  assign rs2_o = instruction_i.rs2;

  assign ex_stage_busy = ex1_valid_i | ex2_valid_i | ex3_valid_i | ex4_valid_i;

  assign ex_raw_hazard = valid_i &&
                               ( (instr_reads_rs1 && ((ex1_valid_i && (ex1_wr_reg_i == rs1_o))   ||
                                                      (ex2_valid_i && (ex2_wr_reg_i == rs1_o))   ||
                                                      (ex3_valid_i && (ex3_wr_reg_i == rs1_o))   ||
                                                      (ex4_valid_i && (ex4_wr_reg_i == rs1_o)))) ||
                                 (instr_reads_rs2 && ((ex1_valid_i && (ex1_wr_reg_i == rs2_o))   ||
                                                      (ex2_valid_i && (ex2_wr_reg_i == rs2_o))   ||
                                                      (ex3_valid_i && (ex3_wr_reg_i == rs2_o))   ||
                                                      (ex4_valid_i && (ex4_wr_reg_i == rs2_o)))) );

  assign send_nop = is_instr_wbalu && (ex_stage_busy || (alu_valid_o && ~alu_instr_finishes_i));
  assign stall_o  = mem_stall_i || (is_instr_wbalu && wb_is_next_cycle_i) || send_nop ||
                    ex_raw_hazard;

endmodule : decode_stage
