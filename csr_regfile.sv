module csr_regfile import params_pkg::*; #(
  parameter int DATA_WIDTH     = params_pkg::DATA_WIDTH,
  parameter int CSR_ADDR_WIDTH = params_pkg::CSR_ADDR_WIDTH,
  parameter int CSR_SATP       = params_pkg::CSR_SATP
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic wr_en_i,
  input  logic [CSR_ADDR_WIDTH-1:0] rd_addr_i,
  input  logic [CSR_ADDR_WIDTH-1:0] wr_addr_i,
  input  logic [DATA_WIDTH-1:0] wr_data_i,
  output logic [DATA_WIDTH-1:0] data_o,
  output logic [DATA_WIDTH-1:0] satp_o
`ifndef SYNTHESIS
  , output logic [DATA_WIDTH-1:0] debug_satp_o
`endif
);

  logic [DATA_WIDTH-1:0] satp;

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

  always_comb begin : csr_read
    data_o = '0;

    case (rd_addr_i)
      CSR_SATP: data_o = satp;
      default :;
    endcase
  end

  assign satp_o = satp;

`ifndef SYNTHESIS
  assign debug_satp_o = satp;
`endif

endmodule : csr_regfile
