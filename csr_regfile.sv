module csr_regfile import params_pkg::*; #(
  parameter int DATA_WIDTH = params_pkg::DATA_WIDTH,
  parameter int ADDR_WIDTH = params_pkg::ADDR_WIDTH
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic wr_en_i,
  input  logic [ADDR_WIDTH-1:0] wr_addr_i,
  input  logic [DATA_WIDTH-1:0] wr_data_i,
  output logic [DATA_WIDTH-1:0] satp_o
);

  logic [DATA_WIDTH-1:0] satp;

  localparam int CSR_SATP = 12'h180;

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      satp <= 32'b0;
    end else if (wr_en_i) begin
      case (wr_addr_i)
        CSR_SATP: satp <= wr_data_i;
        default :;
      endcase
    end
  end

  assign satp_o = satp;

endmodule : csr_regfile
