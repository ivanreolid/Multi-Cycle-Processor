module ptw import params_pkg::*; #(
  parameter int PADDR_WIDTH    = params_pkg::PADDR_WIDTH
)(
  input  logic req_i,
  input  logic ppn_is_present_i,
  input  vaddr_t vaddr_i,
  input  data_t satp_data_i,
  output logic valid_o,
  output logic error_o,
  output paddr_t paddr_o
);

  vaddr_t sum_result;

  assign sum_result = vaddr_i + {12'b0, satp_data_i[19:0]};
  assign paddr_o    = sum_result[PADDR_WIDTH-1:0];

  always_comb begin : page_walk
    valid_o       = 1'b0;
    error_o       = 1'b0;

    if (req_i) begin
      if (!ppn_is_present_i) begin
        error_o       = 1'b1;
      end else begin
        valid_o = 1'b1;
      end
    end
  end

endmodule
