import params_pkg::*;

module fetch_ctrl #(
  parameter int ADDR_WIDTH = params_pkg::ADDR_WIDTH
)(
  input  logic alu_branch_taken_i,
  input  logic is_jump_i,
  input  logic [ADDR_WIDTH-1:0] pc_i,
  input  logic [ADDR_WIDTH-1:0] pc_branch_offset_i,
  input  logic [ADDR_WIDTH-1:0] jump_address_i,
  output logic next_valid_o,
  output logic [ADDR_WIDTH-1:0] next_pc_o
);

  logic [ADDR_WIDTH-1:0] pc_added;

  assign pc_added = (alu_branch_taken_i ? pc_branch_offset_i : pc_i + 1) % MEM_SIZE;
  assign next_pc_o = is_jump_i ? jump_address_i : pc_added;

  assign next_valid_o = 1'b1;

endmodule
