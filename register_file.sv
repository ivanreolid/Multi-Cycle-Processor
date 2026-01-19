import params_pkg::*;

module register_file (
  input  logic clk_i,
  input  logic rst_i,
  input  logic wr_en_i,
  input  reg_id_t  rd_reg_a_i,
  input  reg_id_t  rd_reg_b_i,
  input  reg_id_t  wr_reg_i,
  input  data_t wr_data_i,
  output data_t reg_a_data_o,
  output data_t reg_b_data_o,
`ifndef SYNTHESIS
  output data_t debug_regs_o [32]
`endif
);

  localparam int BANK_SIZE = 32;

  data_t regs [BANK_SIZE];

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      for (int i = 0; i < BANK_SIZE; ++i) begin
        regs[i] <= '0;
      end
    end else if (wr_en_i && (wr_reg_i != 5'd0)) begin
      regs[wr_reg_i] <= wr_data_i;
    end
  end

  assign reg_a_data_o = regs[rd_reg_a_i];
  assign reg_b_data_o = regs[rd_reg_b_i];

`ifndef SYNTHESIS
  assign debug_regs_o = regs;
`endif

endmodule
