import params_pkg::*;

module alu #(
  parameter OPCODE_WIDTH = params_pkg::OPCODE_WIDTH,
  parameter DATA_WIDTH   = params_pkg::DATA_WIDTH
)(
  input  opcode opcode_i,
  input  logic [2:0] funct3_i,
  input  logic [6:0] funct7_i,
  input  logic [DATA_WIDTH-1:0] a_i,
  input  logic [DATA_WIDTH-1:0] b_i,
  output logic is_zero_o,
  output logic is_less_o,
  output logic [DATA_WIDTH-1:0] result_o
);

  always_comb begin : operation
    case (opcode_i)
      R: begin
        if (funct3_i == 3'b000)
          result_o = funct7_i == 0 ? a_i + b_i : a_i - b_i;
      end
      LOAD: begin
        if (funct3_i == 3'b000)         // LB
          result_o = a_i + b_i;
        else if (funct3_i == 3'b010)    // LW
          result_o = a_i + b_i;
      end
      STORE: begin
        if (funct3_i == 3'b000)         // SB
          result_o = a_i + b_i;
        else if (funct3_i == 3'b010)    // SW
          result_o = a_i + b_i;
      end
      JAL: result_o = a_i + b_i;
      IMMEDIATE: begin
        if (funct3_i == 3'b000)
          result_o = a_i + b_i;
      end
      default:     result_o = 0;
    endcase
  end

  assign is_zero_o = a_i == b_i;
  assign is_less_o = a_i < b_i;

endmodule
