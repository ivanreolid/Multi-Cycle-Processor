import params_pkg::*;

module dmem #(
  parameter int MEM_SIZE   = params_pkg::MEM_SIZE,
  parameter int ADDR_WIDTH = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH = params_pkg::DATA_WIDTH
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic wr_en_i,
  input  logic [ADDR_WIDTH-1:0] addr_i,
  input  logic [DATA_WIDTH-1:0] wr_data_i,
  output logic [DATA_WIDTH-1:0] rd_data_o
/*`ifndef SYNTHESIS
  output logic [DATA_WIDTH-1:0] debug_dmem_o [0:MEM_SIZE-1]
`endif*/
);

  logic [DATA_WIDTH-1:0] mem [0:MEM_SIZE-1];

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      for (int i = 0; i < MEM_SIZE; ++i) begin
        mem[i] <= i;
      end
    end else if (wr_en_i) begin
      mem[addr_i] <= wr_data_i;
    end
  end

  assign rd_data_o = mem[addr_i];

/*`ifndef SYNTHESIS
  genvar i;
  generate
    for (i = 0; i < MEM_SIZE; ++i) begin
      assign debug_dmem_o[i] = mem[i];
    end
  endgenerate
`endif*/
  
endmodule
