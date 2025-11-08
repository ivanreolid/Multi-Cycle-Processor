`include "dmem.sv"

import params_pkg::*;

module mem_stage #(
  parameter int MEM_SIZE       = params_pkg::MEM_SIZE,
  parameter int ADDR_WIDTH     = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH     = params_pkg::DATA_WIDTH,
  parameter int REGISTER_WIDTH = params_pkg::REGISTER_WIDTH
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic [DATA_WIDTH-1:0] alu_result_i,
  input  logic [DATA_WIDTH-1:0] rs2_data_i,
  input  logic [REGISTER_WIDTH-1:0] wr_reg_i,
  input  logic [DATA_WIDTH-1:0] mem_data_i,
  input  logic valid_i,
  input  logic is_load_i,
  input  logic is_store_i,
  input  logic reg_wr_en_i,
  input  logic mem_data_is_valid_i,
  input  access_size_t access_size_i,
`ifndef SYNTHESIS
  input  logic [ADDR_WIDTH-1:0] debug_pc_i,
  input  instruction_t debug_instr_i,
`endif
  output logic wb_valid_o,
  output logic wb_reg_wr_en_o,
  output logic wb_is_load_o,
  output logic rd_req_valid_o,
  output logic wr_req_valid_o,
  output logic stall_o,
  output logic [REGISTER_WIDTH-1:0] wb_wr_reg_o,
  output logic [DATA_WIDTH-1:0] wb_data_from_mem_o,
  output logic [DATA_WIDTH-1:0] wb_alu_result_o,
  output logic [ADDR_WIDTH-1:0] mem_req_address_o,
  output logic [DATA_WIDTH-1:0] wr_data_o,
  output access_size_t req_access_size_o,
`ifndef SYNTHESIS
  output logic [ADDR_WIDTH-1:0] debug_wb_pc_o,
  output instruction_t debug_wb_instr_o
`endif
);

  typedef enum logic [1:0] {
    IDLE     = 2'b00,
    READY    = 2'b01,
    WAITING  = 2'b10
  } state_t;

  logic wb_valid_d;
  logic wb_reg_wr_en_d;
  logic [DATA_WIDTH-1:0] wb_data_from_mem_d;

  state_t state, state_d;

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      wb_valid_o         <= 1'b0;
      wb_reg_wr_en_o     <= 1'b0;
      wb_is_load_o       <= 1'b0;
      state              <= IDLE;
    end else begin
      wb_valid_o         <= wb_valid_d;
      wb_reg_wr_en_o     <= wb_reg_wr_en_d;
      wb_is_load_o       <= is_load_i;
      wb_wr_reg_o        <= wr_reg_i;
      wb_alu_result_o    <= alu_result_i;
      wb_data_from_mem_o <= wb_data_from_mem_d;
      state              <= state_d;
`ifndef SYNTHESIS
      debug_wb_pc_o      <= debug_pc_i;
      debug_wb_instr_o   <= debug_instr_i;
`endif
    end
  end

  always_comb begin : state_update
    rd_req_valid_o = 1'b0;
    wr_req_valid_o = 1'b0;
    wb_valid_d     = 1'b0;
    wb_reg_wr_en_d = 1'b0;
    stall_o        = 1'b0;
    state_d        = state;

    case(state)
      IDLE: begin
        state_d            = READY;
      end
      READY: begin
        if (valid_i) begin
          mem_req_address_o = alu_result_i;
          req_access_size_o = access_size_i;
          rd_req_valid_o    = is_load_i;
          wr_req_valid_o    = is_store_i;
          wr_data_o         = rs2_data_i;
          wb_reg_wr_en_d    = is_load_i ? 1'b0 : reg_wr_en_i;
          wb_valid_d        = is_load_i ? 1'b0 : 1'b1;
          stall_o           = is_load_i;
          state_d           = is_load_i ? WAITING : READY;
        end
      end
      WAITING: begin
        stall_o              = 1'b1;
        if (mem_data_is_valid_i) begin
          wb_data_from_mem_d = mem_data_i;
          wb_valid_d         = 1'b1;
          wb_reg_wr_en_d     = 1'b1;
          stall_o            = 1'b0;
          state_d            = READY;
        end
      end
    endcase
  end

endmodule : mem_stage
