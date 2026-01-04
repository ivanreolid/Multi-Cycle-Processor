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

  logic [DATA_WIDTH-1:0] sum_result;
  logic [DATA_WIDTH-1:0] sub_result;
  logic [DATA_WIDTH-1:0] srl_result, sra_result;

  assign sum_result         = a_i + b_i;
  assign sub_result         = a_i - b_i;
  assign srl_result         = a_i >> b_i[4:0];
  assign sra_result         = $signed(a_i) >>> b_i[4:0];

  logic overflow;
  assign overflow = (a_i[31] != b_i[31]) && (sub_result[31] != a_i[31]);

  assign is_zero_o = (sub_result == 0);
  assign is_less_o = sub_result[31] ^ overflow;

  always_comb begin : operation
    result_o = '0;

    case (opcode_i)
      R: begin
        case (funct3_i)
          3'b000:  result_o = (funct7_i == 0) ? sum_result : sub_result;
          default: result_o = '0;
        endcase
      end
      IMMEDIATE: begin
        case (funct3_i)
          3'b000:  result_o = sum_result;
          3'b001:  result_o = a_i << b_i[4:0];           // SLLI
          3'b101:  result_o = funct7_i[5] ? sra_result : srl_result; // SRLI, SRAI
          default: result_o = sum_result;
        endcase
      end
      LOAD, STORE, JAL, LUI, AUIPC, SYSTEM: result_o = sum_result;
      default;
    endcase
  end

endmodule
