import params_pkg::*;

module alu #(
  parameter OPCODE_WIDTH = params_pkg::OPCODE_WIDTH,
  parameter DATA_WIDTH   = params_pkg::DATA_WIDTH
)(
  input  logic [OPCODE_WIDTH-1:0] op_i,
  input  logic [DATA_WIDTH-1:0] a_i,
  input  logic [DATA_WIDTH-1:0] b_i,
  output logic is_zero_o,
  output logic is_less_o,
  output logic [DATA_WIDTH-1:0] result_o
);

  always_comb begin : operation
    case (op_i)
      ADD:    result_o = a_i + b_i;
      LW, SW: result_o = a_i + (b_i << 2);
      SUB:    result_o = a_i - b_i;
      AND:    result_o = a_i & b_i;
      MUL:    result_o = a_i * b_i;
      DIV:    result_o = a_i / b_i;
      OR:     result_o = a_i | b_i;
      XOR:    result_o = a_i ^ b_i;
      default: result_o = 0;
    endcase
  end

  assign is_zero_o = a_i == b_i;
  assign is_less_o = a_i < b_i;

endmodule
