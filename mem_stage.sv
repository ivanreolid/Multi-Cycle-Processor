`include "data_cache.sv"
`include "store_buffer.sv"

import params_pkg::*;

module mem_stage #(
  parameter int MEM_SIZE         = params_pkg::MEM_SIZE,
  parameter int ADDR_WIDTH       = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH       = params_pkg::DATA_WIDTH,
  parameter int REGISTER_WIDTH   = params_pkg::REGISTER_WIDTH,
  parameter int ROB_ENTRY_WIDTH  = params_pkg::ROB_ENTRY_WIDTH,
  parameter int CACHE_LINE_BYTES = 16,
  parameter int DCACHE_N_LINES   = 4
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic [DATA_WIDTH-1:0] alu_result_i,
  input  logic [DATA_WIDTH-1:0] rs2_data_i,
  input  logic [REGISTER_WIDTH-1:0] wr_reg_i,
  input  logic [ROB_ENTRY_WIDTH-1:0] rob_instr_idx_i,

  input  logic [CACHE_LINE_BYTES*8-1:0] mem_line_data_i,
  input  logic mem_rvalid_i,
  input  logic mem_gnt_i,
  input  logic valid_i,
  input  logic is_load_i,
  input  logic is_store_i,
  input  logic reg_wr_en_i,
  input  access_size_t access_size_i,
  input  logic rob_commit_valid_i,
  input  logic [ROB_ENTRY_WIDTH-1:0] rob_commit_idx_i,

`ifndef SYNTHESIS
  input  logic [ADDR_WIDTH-1:0] debug_pc_i,
  input  var instruction_t debug_instr_i,
`endif
  output logic wb_valid_o,
  output logic wb_reg_wr_en_o,
  output logic rd_req_valid_o,
  output logic wr_req_valid_o,
  output logic stall_o,
  output logic sb_full_o,
  output logic [REGISTER_WIDTH-1:0] wb_wr_reg_o,
  output logic [DATA_WIDTH-1:0] wb_data_from_mem_o,
  output logic [ADDR_WIDTH-1:0] mem_req_address_o,
  output logic [CACHE_LINE_BYTES*8-1:0] wr_line_data_o,
  output access_size_t req_access_size_o,

  input  logic finish,
  output logic done,
  output logic mem_finish,
  input  logic write_done_o,
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

  logic sb_store_req;
  logic [ADDR_WIDTH-1:0] sb_store_addr;
  logic [DATA_WIDTH-1:0] sb_store_data;
  access_size_t sb_store_size;
  logic sb_store_ack;
  logic sb_load_forward_valid;
  logic [DATA_WIDTH-1:0] sb_load_forward_data;
  logic sb_load_must_wait;
  logic sb_done;

  logic cache_req;
  logic cache_hit;
  logic cache_wr;
  logic cache_ready;
  logic cache_rvalid;
  logic [31:0] cache_rdata;

  logic cache_mem_req;
  logic cache_mem_we;
  logic [ADDR_WIDTH-1:0] cache_mem_addr;
  logic [CACHE_LINE_BYTES*8-1:0] cache_mem_wdata;

  logic cache_done;
  
  // Finish control
  logic cache_finish;
  logic sb_finish;
  logic finish_reg;
  logic [3:0] sb_done_delay;
  
  always_ff @(posedge clk_i) begin
    if (!rst_i) begin
      finish_reg <= 1'b0;
      sb_done_delay <= '0;
    end else begin
      if (finish && !finish_reg) begin
        finish_reg <= 1'b1;
      end
      
      // SB done olduktan sonra birkaç cycle bekle
      if (sb_done && finish_reg) begin
        if (sb_done_delay < 4'd10) begin
          sb_done_delay <= sb_done_delay + 1'b1;
        end
      end
    end
  end
  
  // SB gets finish immediately
  assign sb_finish = finish_reg;
  
  // Cache gets finish only after SB is done AND we waited a bit
  assign cache_finish = finish_reg && sb_done && (sb_done_delay >= 4'd3);
  
  // Overall done when cache is also done
  assign done = cache_done && sb_done && (sb_done_delay >= 4'd3);
  
  logic sb_drain_store;
  assign sb_drain_store = sb_store_req && !valid_i;

  logic [1:0] cache_size;
  assign cache_size = (access_size_i == BYTE) ? 2'b00 :
                      (access_size_i == HALF) ? 2'b01 : 2'b10;

  logic [1:0] sb_cache_size;
  assign sb_cache_size = (sb_store_size == BYTE) ? 2'b00 :
                         (sb_store_size == HALF) ? 2'b01 : 2'b10;

  logic [3:0] sb_wstrb;
  always_comb begin
    case (sb_store_size)
      BYTE: sb_wstrb = 4'b0001;
      HALF: sb_wstrb = 4'b0011;
      WORD: sb_wstrb = 4'b1111;
      default: sb_wstrb = 4'b1111;
    endcase
  end

  logic [3:0] cpu_wstrb;
  always_comb begin
    case (access_size_i)
      BYTE: cpu_wstrb = 4'b0001;
      HALF: cpu_wstrb = 4'b0011;
      WORD: cpu_wstrb = 4'b1111;
      default: cpu_wstrb = 4'b1111;
    endcase
  end
  
  logic [DATA_WIDTH-1:0] final_load_data;
  logic load_data_ready;

  always_ff @(posedge clk_i) begin
    if (rst_i && valid_i && is_store_i) begin
      $display("t=%0t: MEM STORE to SB - rob_idx=%0d addr=0x%h data=0x%h", 
              $time, rob_instr_idx_i, alu_result_i, rs2_data_i);
    end
  end

  StoreBuffer #(
    .ROB_ENTRY_WIDTH    (ROB_ENTRY_WIDTH),
    .BUFFER_INDEX_WIDTH (2),
    .DATA_WIDTH         (DATA_WIDTH),
    .ADDR_WIDTH         (ADDR_WIDTH),
    .REGISTER_WIDTH     (REGISTER_WIDTH)
  ) store_buffer (
    .clk_i                  (clk_i),
    .rst_i                  (rst_i),
    
    .finish_i               (sb_finish),
    .done_o                 (sb_done),
    
    .sb_full_o              (sb_full_o),
    
    .new_store_valid_i      (valid_i && is_store_i),
    .new_store_rob_idx_i    (rob_instr_idx_i),
    .new_store_addr_i       (alu_result_i),
    .new_store_data_i       (rs2_data_i),
    .new_store_size_i       (access_size_i),
    
    .rob_commit_valid_i     (rob_commit_valid_i),
    .rob_commit_idx_i       (rob_commit_idx_i),
    
    .load_check_valid_i     (valid_i && is_load_i),
    .load_addr_i            (alu_result_i),
    .load_size_i            (access_size_i),
    .load_forward_valid_o   (sb_load_forward_valid),
    .load_forward_data_o    (sb_load_forward_data),
    .load_must_wait_o       (sb_load_must_wait),
    
    .dcache_store_req_o     (sb_store_req),
    .dcache_store_addr_o    (sb_store_addr),
    .dcache_store_data_o    (sb_store_data),
    .dcache_store_size_o    (sb_store_size),
    .dcache_store_ack_i     (sb_store_ack)
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
    .cpu_addr(sb_drain_store ? sb_store_addr : alu_result_i),
    .cpu_wdata(sb_drain_store ? sb_store_data : rs2_data_i),
    .cpu_wstrb(sb_drain_store ? sb_wstrb : cpu_wstrb),
    .cpu_size(sb_drain_store ? sb_cache_size : cache_size),
    .cpu_ready(cache_ready),
    .cpu_rdata(cache_rdata),
    .cpu_rvalid(cache_rvalid),
    .curr_cache_hit(cache_hit),

    .mem_req(cache_mem_req),
    .mem_we(cache_mem_we),
    .mem_addr(cache_mem_addr),
    .mem_wdata(cache_mem_wdata),
    .mem_gnt(mem_gnt_i),
    .mem_rvalid(mem_rvalid_i),
    .mem_rdata(mem_line_data_i),

    .finish(cache_finish),
    .done(cache_done),
    .write_done_o(write_done_o),
    .mem_finish(mem_finish)
  );

  assign sb_store_ack = sb_drain_store && cache_ready;

  assign rd_req_valid_o = cache_mem_req && !cache_mem_we;
  assign wr_req_valid_o = cache_mem_req && cache_mem_we;
  assign mem_req_address_o = cache_mem_addr;
  assign wr_line_data_o = cache_mem_wdata;
  assign req_access_size_o = access_size_i;

  always_comb begin
    if (sb_load_forward_valid) begin
      final_load_data = sb_load_forward_data;
      load_data_ready = 1'b1;
    end else if (sb_load_must_wait) begin
      final_load_data = '0;
      load_data_ready = 1'b0;
    end else begin
      final_load_data = cache_rdata;
      load_data_ready = cache_rvalid;
    end
  end

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      state <= IDLE;
    end else begin
      state <= state_d;
    end
  end

  assign wb_wr_reg_o        = wr_reg_i;
  assign wb_data_from_mem_o = final_load_data;
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
        state_d = READY;
      end
      
      READY: begin
        // Priority 1: Drain store buffer
        if (!valid_i && sb_store_req) begin
          cache_req = 1'b1;
          cache_wr  = 1'b1;
          stall_o   = 1'b0;
          state_d   = READY;
        end
        // Priority 2: Handle CPU request
        else if (valid_i) begin
          if (is_load_i) begin
            if (sb_load_forward_valid) begin
              wb_valid_o      = 1'b1;
              wb_reg_wr_en_o  = 1'b1;
              stall_o         = 1'b0;
              state_d         = READY;
            end else if (sb_load_must_wait) begin
              wb_valid_o      = 1'b0;
              wb_reg_wr_en_o  = 1'b0;
              stall_o         = 1'b1;
              state_d         = WAITING;
            end else begin
              cache_req = 1'b1;
              cache_wr  = 1'b0;
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
            end
          end else if (is_store_i) begin
            if (sb_full_o) begin
              wb_valid_o     = 1'b0;
              wb_reg_wr_en_o = 1'b0;
              stall_o        = 1'b1;
              state_d        = WAITING;
            end else begin
              wb_valid_o     = 1'b1;
              wb_reg_wr_en_o = reg_wr_en_i;
              stall_o        = 1'b0;
              state_d        = READY;
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
        stall_o = 1'b1;
        if (is_load_i) begin
          if (sb_load_forward_valid) begin
            wb_valid_o     = 1'b1;
            wb_reg_wr_en_o = 1'b1;
            stall_o        = 1'b0;
            state_d        = READY;
          end else if (sb_load_must_wait) begin
            stall_o        = 1'b1;
            state_d        = WAITING;
          end else begin
            
            cache_req = 1'b1; 
            cache_wr  = 1'b0; 
            
            if (load_data_ready) begin
              wb_valid_o     = 1'b1;
              wb_reg_wr_en_o = 1'b1;
              stall_o        = 1'b0;
              state_d        = READY;
            end else begin
              stall_o        = 1'b1;
              state_d        = WAITING;
            end
          end
        end else if (is_store_i) begin
          if (!sb_full_o) begin
            wb_valid_o     = 1'b1;
            wb_reg_wr_en_o = reg_wr_en_i;
            stall_o        = 1'b0;
            state_d        = READY;
          end
        end else begin
          wb_valid_o     = 1'b1;
          wb_reg_wr_en_o = reg_wr_en_i;
          stall_o        = 1'b0;
          state_d        = READY;
        end
      end
    endcase
  end


endmodule : mem_stage