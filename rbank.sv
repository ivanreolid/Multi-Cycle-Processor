import params_pkg::*;

module rbank #(
  parameter int REGISTER_WIDTH = params_pkg::REGISTER_WIDTH,
  parameter int DATA_WIDTH     = params_pkg::DATA_WIDTH
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic wr_en_i,
  input  logic [REGISTER_WIDTH-1:0]  rd_reg_a_i,
  input  logic [REGISTER_WIDTH-1:0]  rd_reg_b_i,
  input  logic [REGISTER_WIDTH-1:0]  wr_reg_i,
  input  logic [DATA_WIDTH-1:0] wr_data_i,
  output logic [DATA_WIDTH-1:0] reg_a_data_o,
  output logic [DATA_WIDTH-1:0] reg_b_data_o,
`ifndef SYNTHESIS
  output logic [DATA_WIDTH-1:0] debug_regs_o [0:31]
`endif
);

  localparam BANK_SIZE = 32;

  logic [DATA_WIDTH-1:0] regs [0:BANK_SIZE-1];

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      for (int i = 0; i < BANK_SIZE; ++i) begin
        regs[i] <= i;
      end
    end else if (wr_en_i) begin
      regs[wr_reg_i] <= wr_data_i;
    end
  end

  assign reg_a_data_o = regs[rd_reg_a_i];
  assign reg_b_data_o = regs[rd_reg_b_i];

`ifndef SYNTHESIS
  assign debug_regs_o = regs;
`endif

  initial begin
    $readmemh("init_regs.hex", regs);
  end

endmodule
