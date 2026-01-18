module tlb import params_pkg::*; #(
  parameter int VPN_WIDTH = params_pkg::VPN_WIDTH,
  parameter int PPN_WIDTH = params_pkg::PPN_WIDTH,
  parameter int TLB_DEPTH = params_pkg::ITLB_DEPTH
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic flush_i,
  input  logic wr_en_i,
  input  logic [VPN_WIDTH-1:0] vpn_i,
  input  logic [PPN_WIDTH-1:0] ppn_i,
  output logic hit_o,
  output logic [PPN_WIDTH-1:0] ppn_o
);

  localparam int INDEX_WIDTH = $clog2(TLB_DEPTH);

  typedef struct packed {
    logic             valid;
    logic [VPN_WIDTH-1:0] vpn;
    logic [PPN_WIDTH-1:0] ppn;
  } tlb_entry_t;

  tlb_entry_t [TLB_DEPTH-1:0] tlb;

  logic [INDEX_WIDTH-1:0] replacement_ptr;

  always_ff @(posedge clk_i or negedge rst_i) begin : flops
    if (!rst_i) begin
      tlb             <= '{default: 0};
      replacement_ptr <= '0;
    end else begin
      if (flush_i) begin
        for (int i = 0; i < TLB_DEPTH; i++) begin
          tlb[i].valid <= 1'b0;
        end
      end else if (wr_en_i) begin
        tlb[replacement_ptr].valid <= 1'b1;
        tlb[replacement_ptr].vpn   <= vpn_i;
        tlb[replacement_ptr].ppn   <= ppn_i;

        if (replacement_ptr == TLB_DEPTH - 1) begin
          replacement_ptr <= '0;
        end else begin
          replacement_ptr <= replacement_ptr + 1'b1;
        end
      end
    end
  end

  always_comb begin : tlb_check
    hit_o = 1'b0;
    ppn_o = '0;

    for (int i = 0; i < TLB_DEPTH; i++) begin
      if (tlb[i].valid && (tlb[i].vpn == vpn_i)) begin
        hit_o = 1'b1;
        ppn_o = tlb[i].ppn;
      end
    end
  end

endmodule
