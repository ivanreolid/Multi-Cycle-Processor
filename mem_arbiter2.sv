
module mem_arbiter #(
  parameter ADDR_WIDTH = 32,
  parameter DATA_WIDTH = 128
)(
  input  logic clk,
  input  logic rst,


  input  logic                  icache_req,
  input  logic [ADDR_WIDTH-1:0]  icache_addr,
  output logic                  icache_gnt,
  output logic [DATA_WIDTH-1:0]  icache_rdata,
  output logic                  icache_rvalid,


  input  logic                  dcache_req,
  input  logic                  dcache_we,
  input  logic [ADDR_WIDTH-1:0]  dcache_addr,
  input  logic [DATA_WIDTH-1:0]  dcache_wdata,
  output logic                  dcache_gnt,
  output logic [DATA_WIDTH-1:0]  dcache_rdata,
  output logic                  dcache_rvalid,

  output logic                  mem_req,
  output logic                  mem_we,
  output logic [ADDR_WIDTH-1:0]  mem_addr,
  output logic [DATA_WIDTH-1:0]  mem_wdata,
  input  logic [DATA_WIDTH-1:0]  mem_rdata,
  input  logic                  mem_rvalid
  );


  typedef enum logic [1:0] {
    IDLE,
    SERVE_DCACHE,
    SERVE_ICACHE
  } arb_state_t;

  arb_state_t state, next_state;

  logic serving_write;

  always_ff @(posedge clk or negedge rst) begin
    if (!rst) begin
      state         <= IDLE;
      serving_write <= 1'b0;
    end else begin
      state <= next_state;
      if (state == IDLE && dcache_req)
        serving_write <= dcache_we;
      else if (state == IDLE)
        serving_write <= 1'b0;
    end
  end


  always_comb begin
    next_state = state;

    case (state)
      IDLE: begin
        if (dcache_req)
          next_state = SERVE_DCACHE;
        else if (icache_req)
          next_state = SERVE_ICACHE;
      end

      SERVE_DCACHE: begin
        if (serving_write)
          next_state = IDLE; 
        else if (mem_rvalid)
          next_state = IDLE;
      end

      SERVE_ICACHE: begin
        if (mem_rvalid)
          next_state = IDLE;
      end
    endcase
  end

  always_comb begin
    mem_req   = 1'b0;
    mem_we    = 1'b0;
    mem_addr  = '0;
    mem_wdata = '0;

    case (state)
      IDLE: begin
        if (dcache_req) begin
          mem_req   = 1'b1;
          mem_we    = dcache_we;
          mem_addr  = dcache_addr;
          mem_wdata = dcache_wdata;
        end else if (icache_req) begin
          mem_req  = 1'b1;
          mem_we   = 1'b0;
          mem_addr  = icache_addr;
        end
      end

      SERVE_DCACHE: begin
        if (!serving_write && !mem_rvalid) begin
          mem_req   = 1'b1;
          mem_we    = 1'b0;
          mem_addr  = dcache_addr;
          mem_wdata = dcache_wdata;
        end
      end

      SERVE_ICACHE: begin
        if (!mem_rvalid) begin
          mem_req  = 1'b1;
          mem_we   = 1'b0;
          mem_addr = icache_addr;
        end
      end
    endcase
  end

  assign dcache_gnt = (state == IDLE && dcache_req) || (state == SERVE_DCACHE);
  assign icache_gnt = (state == IDLE && icache_req && !dcache_req) || (state == SERVE_ICACHE);

  assign dcache_rdata  = mem_rdata;
  assign icache_rdata  = mem_rdata;

  assign dcache_rvalid = (state == SERVE_DCACHE) && mem_rvalid;
  assign icache_rvalid = (state == SERVE_ICACHE) && mem_rvalid;

endmodule
