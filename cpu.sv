`include "imem.sv"
`include "rbank.sv"
`include "dmem.sv"
`include "alu.sv"
`include "control.sv"

import params_pkg::*;

module cpu (
  input logic clk_i,
  input logic rst_i,
`ifndef SYNTHESIS
  output logic [DATA_WIDTH-1:0] debug_dmem_o [0:MEM_SIZE-1],
  output logic [DATA_WIDTH-1:0] debug_regs_o [0:31],
  output logic [ADDR_WIDTH-1:0] debug_pc_committed_o,
  output instruction_t debug_instr_committed_o,
  output logic debug_instr_is_committed_o
`endif
);

  // PC wires
  logic [ADDR_WIDTH-1:0] pc_d, pc_q;
  logic [ADDR_WIDTH-1:0] pc_offset, pc_added;
  logic is_zero, is_less;
  logic branch_taken;

  // Instruction wires
  instruction_t instruction;
  logic [DATA_WIDTH-1:0] offset_sign_extend;
  logic is_load, is_store, is_jump;

  // ALU wires
  logic [DATA_WIDTH-1:0] reg_a_data, reg_b_data;
  logic [DATA_WIDTH-1:0] data_a_to_alu;
  logic [DATA_WIDTH-1:0] result_alu;

  logic [DATA_WIDTH-1:0] data_from_mem;
  logic [DATA_WIDTH-1:0] data_to_reg;

  logic [REGISTER_WIDTH-1:0] reg_a;
  
  logic reg_wr_en;

  assign pc_offset = branch_taken ? {instruction.free, instruction.rd} : {{ADDR_WIDTH-1{1'b0}}, 1'b1};
  assign pc_added = (pc_q + pc_offset) % MEM_SIZE;
  assign pc_d = is_jump ? reg_a_data : pc_added;

  // Data a from register bank must be rd data for stores
  assign reg_a = is_store ? instruction.rd : instruction.ra;

  assign offset_sign_extend = {{(DATA_WIDTH-(INSTR_WIDTH-14)){instruction[INSTR_WIDTH-1]}}, instruction[INSTR_WIDTH-1:14]};
 
  // Data a for the ALU is the offset sign extended for both load and store instructions
  assign data_a_to_alu = (is_load | is_store) ? offset_sign_extend : reg_a_data;

  // Only loads write data from memory into the register bank
  assign data_to_reg = is_load ? data_from_mem : result_alu;

  imem #(
    .MEM_SIZE      (MEM_SIZE   ),
    .ADDR_WIDTH    (ADDR_WIDTH ),
    .DATA_WIDTH    (INSTR_WIDTH)
  ) imem (
    .address_i     (pc_q       ),
    .instruction_o (instruction)
  );

  control #(
    .OPCODE_WIDTH (OPCODE_WIDTH)
  ) control (
    .opcode_i       (instruction.opcode),
    .is_zero_i      (is_zero),
    .is_less_i      (is_less),
    .is_load_o      (is_load),
    .is_store_o     (is_store),
    .is_jump_o      (is_jump),
    .branch_taken_o (branch_taken),
    .reg_wr_en_o    (reg_wr_en)
  );

  rbank #(
    .REGISTER_WIDTH (REGISTER_WIDTH),
    .DATA_WIDTH     (DATA_WIDTH)
   ) rbank (
    .clk_i        (clk_i),
    .rst_i        (rst_i),
    .wr_en_i      (reg_wr_en),
    .rd_reg_a_i   (reg_a),
    .rd_reg_b_i   (instruction.rb),
    .wr_reg_i     (instruction.rd),
    .wr_data_i    (data_to_reg),
    .reg_a_data_o (reg_a_data),
    .reg_b_data_o (reg_b_data),
`ifndef SYNTHESIS
    .debug_regs_o (debug_regs_o)
`endif
  );

  dmem #(
    .MEM_SIZE   (MEM_SIZE  ),
    .ADDR_WIDTH (ADDR_WIDTH),
    .DATA_WIDTH (DATA_WIDTH)
  ) dmem (
    .clk_i     (clk_i),
    .rst_i     (rst_i),
    .wr_en_i   (is_store),
    .addr_i    (result_alu),
    .wr_data_i (reg_a_data),
    .rd_data_o (data_from_mem),
`ifndef SYNTHESIS
    .debug_dmem_o (debug_dmem_o)
`endif
  );

  alu #(
    .OPCODE_WIDTH (OPCODE_WIDTH),
    .DATA_WIDTH   (DATA_WIDTH  )
  ) alu (
    .op_i      (instruction.opcode),
    .a_i       (data_a_to_alu),
    .b_i       (reg_b_data),
    .is_zero_o (is_zero),
    .is_less_o (is_less),
    .result_o  (result_alu)
  );

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      pc_q <= 0;
    end else begin
      pc_q <= pc_d;
`ifndef SYNTHESIS
      debug_pc_committed_o       <= pc_q;
      debug_instr_is_committed_o <= 1'b1;
      debug_instr_committed_o    <= instruction;
`endif
    end
  end

endmodule
