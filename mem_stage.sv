
`include "data_cache.sv"

import params_pkg::*;

module mem_stage #(
  parameter int MEM_SIZE         = params_pkg::MEM_SIZE,
  parameter int ADDR_WIDTH       = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH       = params_pkg::DATA_WIDTH,
  parameter int REGISTER_WIDTH   = params_pkg::REGISTER_WIDTH,
  parameter int CACHE_LINE_BYTES = 16,
  parameter int DCACHE_N_LINES   = 4
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic [DATA_WIDTH-1:0] alu_result_i,
  input  logic [DATA_WIDTH-1:0] rs2_data_i,
  input  logic [REGISTER_WIDTH-1:0] wr_reg_i,

  // Memory interface - cache line width
  input  logic [CACHE_LINE_BYTES*8-1:0] mem_line_data_i,
  input  logic mem_rvalid_i,
  input  logic mem_gnt_i,
  input  logic valid_i,
  input  logic is_load_i,
  input  logic is_store_i,
  input  logic reg_wr_en_i,
  input  access_size_t access_size_i,
`ifndef SYNTHESIS
  input  logic [ADDR_WIDTH-1:0] debug_pc_i,
  input  var instruction_t debug_instr_i,
`endif
  output logic wb_valid_o,
  output logic wb_reg_wr_en_o,
  output logic rd_req_valid_o,
  output logic wr_req_valid_o,
  output logic stall_o,
  output logic wb_is_next_cycle_o,
  output logic [REGISTER_WIDTH-1:0] wb_wr_reg_o,
  output logic [DATA_WIDTH-1:0] wb_data_from_mem_o,
  output logic [ADDR_WIDTH-1:0] mem_req_address_o,
  output logic [CACHE_LINE_BYTES*8-1:0] wr_line_data_o,
  output access_size_t req_access_size_o,

    input  logic finish,   // flush request
    output logic done,     // flush completed
    input write_done_o,   //mem finished writing
`ifndef SYNTHESIS
  output logic [ADDR_WIDTH-1:0] debug_wb_pc_o,
  output var instruction_t debug_wb_instr_o
`endif
);

  typedef enum logic [1:0] {
    IDLE     = 2'b00,
    READY    = 2'b01,
    WAITING  = 2'b10
  } state_t;

  state_t state, state_d;

  // Cache signals
  logic cache_req;
  logic cache_hit;
  logic cache_wr;
  logic cache_ready;
  logic cache_rvalid;
  logic [31:0] cache_rdata;
  logic [31:0] cache_wdata;
  logic [3:0] cache_wstrb;
  logic [1:0] cache_size;

  assign cache_size = (access_size_i == BYTE) ? 2'b00 :
                      (access_size_i == HALF) ? 2'b01 : 2'b10;

  // Generate write strobes
  always_comb begin
  case (req_access_size_o)
      BYTE: cache_wstrb = 4'b0001;
      HALF: cache_wstrb = 4'b0011;
      WORD: cache_wstrb = 4'b1111;
      default: cache_wstrb = 4'b1111;
  endcase
  end

  logic mem_req;

  data_cache #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .LINE_BYTES(CACHE_LINE_BYTES),
    .N_LINES(DCACHE_N_LINES)
  ) d_cache (
    .clk(clk_i),
    .rstn(rst_i),
    .cpu_req(cache_req),
    .cpu_wr(cache_wr),
    .cpu_addr(alu_result_i),
    .cpu_wdata(rs2_data_i),
    .cpu_wstrb(cache_wstrb),
    .cpu_size(cache_size),
    .cpu_ready(cache_ready),
    .cpu_rdata(cache_rdata),
    .cpu_rvalid(cache_rvalid),
    .curr_cache_hit(cache_hit),

    .mem_req(mem_req),
    .mem_we(wr_req_valid_o),
    .mem_addr(mem_req_address_o),
    .mem_wdata(wr_line_data_o),
    .mem_gnt(mem_gnt_i),
    .mem_rvalid(mem_rvalid_i),
    .mem_rdata(mem_line_data_i),

    .finish(finish),
    .done(done) ,
    .write_done_o(write_done_o)    
  );

  assign rd_req_valid_o = mem_req && !wr_req_valid_o ;

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      state              <= IDLE;
    end else begin
      state              <= state_d;
    end
  end

  assign req_access_size_o  = access_size_i;
  assign wb_wr_reg_o        = wr_reg_i;
  assign wb_data_from_mem_o = cache_rdata;
`ifndef SYNTHESIS
  assign debug_wb_pc_o      = debug_pc_i;
  assign debug_wb_instr_o   = debug_instr_i;
`endif

  always_comb begin : state_update
    cache_req          = 1'b0;
    cache_wr           = 1'b0;
    wb_valid_o         = 1'b0;
    wb_reg_wr_en_o     = 1'b0;
    stall_o            = 1'b0;
    state_d            = state;

    case(state)
      IDLE: begin
        state_d            = READY;
      end
      READY: begin
        if (valid_i) begin
          cache_req         = is_load_i | is_store_i;
          cache_wr          = is_store_i;

          if (is_load_i) begin
            if (cache_hit && cache_rvalid) begin
              wb_valid_o      = 1'b1;
              wb_reg_wr_en_o  = 1'b1;
              stall_o         = 1'b0;
              state_d         = READY;
            end else begin
              wb_valid_o      = 1'b0;
              wb_reg_wr_en_o  = 1'b0;
              stall_o         = 1'b1;
              state_d         = WAITING;
            end
          end else if (is_store_i) begin
            if (cache_hit) begin
              wb_reg_wr_en_o  = reg_wr_en_i;
              wb_valid_o      = 1'b1;
              stall_o         = 1'b0;
              state_d         = READY;
            end else begin
              wb_reg_wr_en_o  = 1'b0;
              wb_valid_o      = 1'b0;
              stall_o         = 1'b1;
              state_d         = WAITING;
            end
          end else begin
            wb_reg_wr_en_o  = reg_wr_en_i;
            wb_valid_o      = 1'b1;
            stall_o         = 1'b0;
            state_d         = READY;
          end
        end
      end

      WAITING: begin
        stall_o              = 1'b1;
        if (cache_rvalid) begin
          wb_valid_o         = 1'b1;
          wb_reg_wr_en_o     = 1'b1;
          stall_o            = 1'b0;
          state_d            = READY;
        end else if (cache_hit && !is_load_i) begin
          wb_valid_o         = 1'b1;
          wb_reg_wr_en_o     = reg_wr_en_i;
          stall_o            = 1'b0;
          state_d            = READY;
        end
      end
    endcase
  end

  assign wb_is_next_cycle_o = wb_valid_o;

endmodule : mem_stage
