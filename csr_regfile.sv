module csr_regfile import params_pkg::*; #(
  parameter int PPN_WIDTH      = params_pkg::PPN_WIDTH,
  parameter int CSR_SATP       = params_pkg::CSR_SATP
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic excpt_we_i,
  input  logic csr_instr_we_i,
  input  logic fetch_present_req_i,
  input  logic mem_present_req_i,
  input  csr_addr_t rd_addr_i,
  input  csr_addr_t csr_wraddr_i,
  input  logic [PPN_WIDTH-1:0] fetch_present_ppn_i,
  input  logic [PPN_WIDTH-1:0] mem_present_ppn_i,
  input  data_t csr_wrdata_i,
  input  data_t excpt_mepc_i,
  input  data_t excpt_mtval_i,
  input  var excpt_cause_t excpt_mcause_i,
  output logic fetch_ppn_is_present_o,
  output logic mem_ppn_is_present_o,
  output data_t data_o,
  output data_t satp_o,
  output data_t mtvec_o,
  output data_t mepc_o
`ifndef SYNTHESIS
  , output data_t debug_satp_o
`endif
);

  data_t satp;

  data_t mepc;
  data_t mtval;
  data_t mtvec;
  excpt_cause_t mcause;

  logic present [256];
  logic [PPN_WIDTH-1:0] ppn_sel;

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      satp    <= '0;
      mepc    <= '0;
      mtval   <= '0;
      mtvec   <= '0;
      mcause  <= '{default: 0};
      ppn_sel <= '0;

      for (int i = 0; i < 256; i++) begin
        present[i] <= 1'b0;
      end
    end else begin
      if (excpt_we_i) begin
        mepc   <= excpt_mepc_i;
        mcause <= excpt_mcause_i;
        mtval  <= excpt_mtval_i;
      end else if (csr_instr_we_i) begin
        case (csr_wraddr_i)
          CSR_SATP:  satp  <= csr_wrdata_i;
          CSR_MTVEC: mtvec <= csr_wrdata_i;
          CSR_PPN_SEL: ppn_sel <= csr_wrdata_i;
          CSR_PPN_FLAG: present[ppn_sel] <= csr_wrdata_i;
          default :;
        endcase
      end
    end
  end

  always_comb begin : csr_read
    data_o = '0;

    case (rd_addr_i)
      CSR_SATP:   data_o = satp;
      CSR_MCAUSE: data_o = mcause;
      CSR_MTVAL:  data_o = mtval;
      default :;
    endcase
  end

  always_comb begin : read_presence
    fetch_ppn_is_present_o = 1'b0;
    mem_ppn_is_present_o   = 1'b0;

    if (fetch_present_req_i) begin
      fetch_ppn_is_present_o = present[fetch_present_ppn_i];
    end

    if (mem_present_req_i) begin
      mem_ppn_is_present_o = present[mem_present_ppn_i];
    end
  end

  assign satp_o = satp;

  assign mtvec_o = mtvec;
  assign mepc_o  = mepc;

`ifndef SYNTHESIS
  assign debug_satp_o = satp;
`endif

endmodule : csr_regfile
