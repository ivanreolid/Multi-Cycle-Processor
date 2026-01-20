`include "data_cache.sv"

import params_pkg::*;

module mem_stage #(
  parameter int MEM_SIZE         = params_pkg::MEM_SIZE,
  parameter int ADDR_WIDTH       = params_pkg::ADDR_WIDTH,
  parameter int PPN_WIDTH        = params_pkg::PPN_WIDTH,
  parameter int CACHE_LINE_BYTES = 16,
  parameter int DCACHE_N_LINES   = 4
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic vm_en_i,
  input  logic trap_bypass_mmu_i,
  input  logic flush_i,
  input  logic ppn_is_present_i,
  input  data_t satp_data_i,
  input  data_t alu_result_i,
  input  data_t rs2_data_i,
  input  reg_id_t wr_reg_i,

  // Memory interface - cache line width
  input  cacheline_t mem_line_data_i,
  input  logic mem_rvalid_i,
  input  logic mem_gnt_i,
  input  logic valid_i,
  input  logic is_load_i,
  input  logic is_store_i,
  input  logic reg_wr_en_i,
  input  access_size_t access_size_i,
`ifndef SYNTHESIS
  input  vaddr_t debug_pc_i,
  input  var instruction_t debug_instr_i,
`endif
  output logic present_table_req_o,
  output logic wb_valid_o,
  output logic wb_excpt_o,
  output logic wb_reg_wr_en_o,
  output logic rd_req_valid_o,
  output logic wr_req_valid_o,
  output logic stall_o,
  output reg_id_t wb_wr_reg_o,
  output logic [PPN_WIDTH-1:0] present_table_ppn_o,
  output data_t wb_data_from_mem_o,
  output vaddr_t wb_excpt_tval_o,
  output paddr_t mem_req_address_o,
  output cacheline_t wr_line_data_o,
  output var access_size_t req_access_size_o,
  output var excpt_cause_t wb_excpt_cause_o,
  input  logic finish,   // flush request
  output logic done,     // flush completed
  input write_done_o,   //mem finished writing
`ifndef SYNTHESIS
  output vaddr_t debug_wb_pc_o,
  output var instruction_t debug_wb_instr_o
`endif
);

  typedef enum logic [1:0] {
    IDLE     = 2'b00,
    READY    = 2'b01,
    WAITING  = 2'b10,
    FLUSH    = 2'b11
  } state_t;

  state_t state, state_d;
  paddr_t paddr;

  // Cache signals
  logic cache_req;
  logic cache_hit;
  logic cache_wr;
  logic cache_ready;
  logic cache_rvalid;
  data_t cache_rdata;
  data_t cache_wdata;
  logic [3:0] cache_wstrb;
  logic [1:0] cache_size;

  logic mmu_enable;

  // DTLB wires
  logic dtlb_hit;
  logic dtlb_wr_en;
  logic [PPN_WIDTH-1:0] dtlb_ppn;

  // DPTW wires
  logic dptw_req;
  logic dptw_valid;
  logic dptw_error;
  paddr_t dptw_paddr;

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

  tlb d_tlb (
    .clk_i     (clk_i),
    .rst_i     (rst_i),
    .flush_i   (flush_i),
    .wr_en_i   (dtlb_wr_en),
    .vpn_i     (alu_result_i[31:12]),
    .ppn_i     (dptw_paddr[19:12]),
    .hit_o     (dtlb_hit),
    .ppn_o     (dtlb_ppn)
  );

  ptw d_ptw (
    .req_i            (dptw_req),
    .ppn_is_present_i (ppn_is_present_i),
    .vaddr_i          (alu_result_i),
    .satp_data_i      (satp_data_i),
    .valid_o          (dptw_valid),
    .error_o          (dptw_error),
    .paddr_o          (dptw_paddr)
  );

  data_cache #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .LINE_BYTES(CACHE_LINE_BYTES),
    .N_LINES(DCACHE_N_LINES)
  ) d_cache (
    .clk(clk_i),
    .rstn(rst_i),
    .cpu_req(cache_req),
    .cpu_wr(cache_wr),
    .cpu_addr(paddr),
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

  assign mmu_enable = vm_en_i && !trap_bypass_mmu_i;

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
    wb_excpt_o         = 1'b0;
    wb_excpt_tval_o    = '0;
    wb_excpt_cause_o   = '{default: 0};
    wb_reg_wr_en_o     = 1'b0;
    stall_o            = 1'b0;
    state_d            = state;

    dtlb_wr_en          = 1'b0;

    dptw_req            = 1'b0;
    present_table_req_o = 1'b0;
    present_table_ppn_o = '0;

    if (flush_i) begin
      state_d = state == WAITING ? FLUSH : READY;
    end else begin
      case(state)
        IDLE: begin
          state_d            = READY;
        end
        READY: begin
          if (valid_i) begin
            if (mmu_enable) begin
              if (dtlb_hit) begin
                paddr = {dtlb_ppn, alu_result_i[11:0]};
              end else begin
                dptw_req = 1'b1;
                present_table_req_o = 1'b1;
                present_table_ppn_o = dptw_paddr[19:12];

                if (dptw_valid) begin
                  paddr      = dptw_paddr;
                  dtlb_wr_en = 1'b1;
                end
              end
            end else begin
              paddr = alu_result_i[19:0];
            end

            if (dptw_error) begin
              wb_valid_o = 1'b1;
              wb_excpt_o       = 1'b1;
              wb_excpt_tval_o  = alu_result_i;
              wb_excpt_cause_o = is_load_i ? LOAD_PAGE_FAULT : STORE_PAGE_FAULT;
              state_d          = READY;
            end else begin
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
        FLUSH: begin
          if (cache_rvalid) begin
            state_d = READY;
          end
        end
      endcase
    end
  end

endmodule : mem_stage
