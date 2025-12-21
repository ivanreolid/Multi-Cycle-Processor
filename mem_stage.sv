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
  output logic [DATA_WIDTH-1:0] wr_data_o,
  output access_size_t req_access_size_o,
  output logic [3:0] wr_strb_o,
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

  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      state              <= IDLE;
    end else begin
      state              <= state_d;
    end
  end

  assign mem_req_address_o  = alu_result_i;
  assign req_access_size_o  = access_size_i;
  assign wr_data_o          = rs2_data_i;

  assign wb_wr_reg_o        = wr_reg_i;
  assign wb_data_from_mem_o = mem_data_i;
`ifndef SYNTHESIS
  assign debug_wb_pc_o      = debug_pc_i;
  assign debug_wb_instr_o   = debug_instr_i;
`endif

  // Write strobe generation for stores
  always_comb begin
    if (is_store_i) begin
      case (access_size_i)
        BYTE: begin
          case (alu_result_i[1:0])
            2'b00: wr_strb_o = 4'b0001;
            2'b01: wr_strb_o = 4'b0010;
            2'b10: wr_strb_o = 4'b0100;
            2'b11: wr_strb_o = 4'b1000;
          endcase
        end
        HALF: begin
          case (alu_result_i[1])
            1'b0: wr_strb_o = 4'b0011;
            1'b1: wr_strb_o = 4'b1100;
          endcase
        end
        WORD: begin
          wr_strb_o = 4'b1111;
        end
        default: wr_strb_o = 4'b0000;
      endcase
    end else begin
      wr_strb_o = 4'b0000;
    end
  end

  always_comb begin : state_update
    rd_req_valid_o     = 1'b0;
    wr_req_valid_o     = 1'b0;
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
          rd_req_valid_o    = is_load_i;
          wr_req_valid_o    = is_store_i;
          wb_reg_wr_en_o    = is_load_i ? 1'b0 : reg_wr_en_i;
          wb_valid_o        = is_load_i ? 1'b0 : 1'b1;
          stall_o           = is_load_i;
          state_d           = is_load_i ? WAITING : READY;
        end
      end
      WAITING: begin
        stall_o              = 1'b1;
        if (mem_data_is_valid_i) begin
          wb_valid_o         = 1'b1;
          wb_reg_wr_en_o     = 1'b1;
          stall_o            = 1'b0;
          state_d            = READY;
        end
      end
    endcase
  end

  assign wb_is_next_cycle_o = wb_valid_o;

endmodule : mem_stage
