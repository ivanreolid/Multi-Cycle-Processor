module mem_arbiter import params_pkg::*; (
  input  logic clk,
  input  logic rst,

  // ======================
  // I-CACHE INTERFACE
  // ======================
  input  logic                   icache_req,
  input  paddr_t                 icache_addr,
  output logic                   icache_gnt,
  output cacheline_t             icache_rdata,
  output logic                   icache_rvalid,

  // ======================
  // D-CACHE INTERFACE
  // ======================
  input  logic                  dcache_req,
  input  logic                  dcache_we,
  input  paddr_t                dcache_addr,
  input  cacheline_t            dcache_wdata,
  output logic                  dcache_gnt,
  output cacheline_t dcache_rdata,
  output logic                  dcache_rvalid,

  // ======================
  // MEMORY INTERFACE
  // ======================
  output logic                   mem_req,
  output logic                   mem_we,
  output paddr_t                 mem_addr,
  output cacheline_t             mem_wdata,
  input  cacheline_t             mem_rdata,
  input  logic                   mem_rvalid
  );

  // ============================================================
  // STATE: ownership kimde?
  // ============================================================
  typedef enum logic [1:0] {
    IDLE,
    SERVE_DCACHE,
    SERVE_ICACHE
  } arb_state_t;

  arb_state_t state, next_state;

  // ============================================================
  // STATE REGISTER
  // ============================================================
  always_ff @(posedge clk or negedge rst) begin
    if (!rst)
      state <= IDLE;
    else
      state <= next_state;
  end

  // ============================================================
  // NEXT STATE LOGIC
  // ============================================================
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
        if (mem_rvalid)   // read OR write response
          next_state = IDLE;
      end

      SERVE_ICACHE: begin
        if (mem_rvalid)
          next_state = IDLE;
      end
    endcase
  end

  // ============================================================
  // EMORY OUTPUT MUX
  // ============================================================
  always_comb begin
    mem_req   = 1'b0;
    mem_we    = 1'b0;
    mem_addr  = '0;
    mem_wdata = '0;

    case (state)
      SERVE_DCACHE: begin
        mem_req   = dcache_req;
        mem_we    = dcache_we;
        mem_addr  = dcache_addr;
        mem_wdata = dcache_wdata;
      end

      SERVE_ICACHE: begin
        mem_req   = icache_req;
        mem_we    = 1'b0;
        mem_addr  = icache_addr;
      end
    endcase
  end

  // ============================================================
  // GRANT SIGNALS
  // ============================================================
  assign dcache_gnt = (state == SERVE_DCACHE);
  assign icache_gnt = (state == SERVE_ICACHE);

  // ============================================================
  // READ DATA ROUTING
  // ============================================================
  assign dcache_rdata  = mem_rdata;
  assign icache_rdata  = mem_rdata;

  assign dcache_rvalid = (state == SERVE_DCACHE) & mem_rvalid;
  assign icache_rvalid = (state == SERVE_ICACHE) & mem_rvalid;

endmodule
