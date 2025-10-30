import params_pkg::*;

module imem #(
  parameter int MEM_SIZE   = params_pkg::MEM_SIZE,
  parameter int ADDR_WIDTH = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH = params_pkg::DATA_WIDTH
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic rd_req_valid_i,
  input  logic wr_req_valid_i,
  input  logic req_is_instr_i,
  input  logic [ADDR_WIDTH-1:0] address_i,
  input  logic [DATA_WIDTH-1:0] wr_data_i,
  input  access_size_t access_size_i,
  output logic data_valid_o,
  output logic data_is_instr_o,
  output logic [DATA_WIDTH-1:0] data_o,
`ifndef SYNTHESIS
  output [7:0] debug_mem_o [MEM_SIZE]
`endif
);

  logic [7:0] mem [MEM_SIZE];

  logic pipe1_valid_d, pipe2_valid_d, pipe3_valid_d, pipe4_valid_d, pipe5_valid_d, pipe6_valid_d,
        pipe7_valid_d, pipe8_valid_d, pipe9_valid_d, pipe10_valid_d;
  logic pipe1_valid, pipe2_valid, pipe3_valid, pipe4_valid, pipe5_valid, pipe6_valid, pipe7_valid,
        pipe8_valid, pipe9_valid, pipe10_valid;

  logic pipe1_is_wr_d, pipe2_is_wr_d, pipe3_is_wr_d, pipe4_is_wr_d, pipe5_is_wr_d;
  logic pipe1_is_wr, pipe2_is_wr, pipe3_is_wr, pipe4_is_wr, pipe5_is_wr;

  access_size_t pipe1_access_size_d;
  access_size_t pipe1_access_size, pipe2_access_size, pipe3_access_size, pipe4_access_size,
                pipe5_access_size, pipe6_access_size, pipe7_access_size, pipe8_access_size,
                pipe9_access_size, pipe10_access_size;

  logic pipe1_is_instr_d, pipe2_is_instr_d, pipe3_is_instr_d, pipe4_is_instr_d, pipe5_is_instr_d,
        pipe6_is_instr_d, pipe7_is_instr_d, pipe8_is_instr_d, pipe9_is_instr_d, pipe10_is_instr_d;
  logic pipe1_is_instr, pipe2_is_instr, pipe3_is_instr, pipe4_is_instr, pipe5_is_instr,
        pipe6_is_instr, pipe7_is_instr, pipe8_is_instr, pipe9_is_instr, pipe10_is_instr;

  logic [ADDR_WIDTH-1:0] pipe1_addr_d, pipe2_addr_d, pipe3_addr_d, pipe4_addr_d, pipe5_addr_d;
  logic [ADDR_WIDTH-1:0] pipe1_addr, pipe2_addr, pipe3_addr, pipe4_addr, pipe5_addr;

  logic [ADDR_WIDTH-1:0] pipe6_instr_d, pipe7_instr_d, pipe8_instr_d, pipe9_instr_d, pipe10_instr_d;
  logic [ADDR_WIDTH-1:0] pipe6_instr, pipe7_instr, pipe8_instr, pipe9_instr, pipe10_instr;

  logic [DATA_WIDTH-1:0] pipe1_data_d, pipe2_data_d, pipe3_data_d, pipe4_data_d, pipe5_data_d;
  logic [DATA_WIDTH-1:0] pipe1_data, pipe2_data, pipe3_data, pipe4_data, pipe5_data;

  always_comb begin : memory_operation
    if (pipe5_valid & ~pipe5_is_wr) begin
      case (pipe5_access_size)
        BYTE: pipe6_instr_d = {24'b0, mem[pipe5_addr]};
        WORD: begin
          pipe6_instr_d = {
            mem[pipe5_addr + 3],
            mem[pipe5_addr + 2],
            mem[pipe5_addr + 1],
            mem[pipe5_addr]
          };
        end
      endcase
    end
  end

  always_ff @(posedge clk_i) begin : pipeline
    if (!rst_i) begin
      pipe1_valid     <= 1'b0;
      pipe2_valid     <= 1'b0;
      pipe3_valid     <= 1'b0;
      pipe4_valid     <= 1'b0;
      pipe5_valid     <= 1'b0;
      pipe6_valid     <= 1'b0;
      pipe7_valid     <= 1'b0;
      pipe8_valid     <= 1'b0;
      pipe9_valid     <= 1'b0;
      pipe10_valid    <= 1'b0;
      pipe1_is_instr  <= 1'b0;
      pipe2_is_instr  <= 1'b0;
      pipe3_is_instr  <= 1'b0;
      pipe4_is_instr  <= 1'b0;
      pipe5_is_instr  <= 1'b0;
      pipe6_is_instr  <= 1'b0;
      pipe7_is_instr  <= 1'b0;
      pipe8_is_instr  <= 1'b0;
      pipe9_is_instr  <= 1'b0;
      pipe10_is_instr <= 1'b0;
      pipe1_is_wr     <= 1'b0;
      pipe2_is_wr     <= 1'b0;
      pipe3_is_wr     <= 1'b0;
      pipe4_is_wr     <= 1'b0;
      pipe5_is_wr     <= 1'b0;
    end else begin
      pipe1_valid     <= pipe1_valid_d;
      pipe2_valid     <= pipe1_valid;
      pipe3_valid     <= pipe2_valid;
      pipe4_valid     <= pipe3_valid;
      pipe5_valid     <= pipe4_valid;
      pipe6_valid     <= pipe5_is_wr ? 1'b0 : pipe5_valid;
      pipe7_valid     <= pipe6_valid;
      pipe8_valid     <= pipe7_valid;
      pipe9_valid     <= pipe8_valid;
      pipe10_valid    <= pipe9_valid;
      pipe1_is_instr  <= pipe1_is_instr_d;
      pipe2_is_instr  <= pipe1_is_instr;
      pipe3_is_instr  <= pipe2_is_instr;
      pipe4_is_instr  <= pipe3_is_instr;
      pipe5_is_instr  <= pipe4_is_instr;
      pipe6_is_instr  <= pipe5_is_instr;
      pipe7_is_instr  <= pipe6_is_instr;
      pipe8_is_instr  <= pipe7_is_instr;
      pipe9_is_instr  <= pipe8_is_instr;
      pipe10_is_instr <= pipe9_is_instr;
      pipe1_addr      <= pipe1_addr_d;
      pipe2_addr      <= pipe1_addr;
      pipe3_addr      <= pipe2_addr;
      pipe4_addr      <= pipe3_addr;
      pipe5_addr      <= pipe4_addr;
      pipe6_instr     <= pipe6_instr_d;
      pipe7_instr     <= pipe6_instr;
      pipe8_instr     <= pipe7_instr;
      pipe9_instr     <= pipe8_instr;
      pipe10_instr    <= pipe9_instr;
      pipe1_data      <= pipe1_data_d;
      pipe2_data      <= pipe1_data;
      pipe3_data      <= pipe2_data;
      pipe4_data      <= pipe3_data;
      pipe5_data      <= pipe4_data;
      pipe1_is_wr     <= pipe1_is_wr_d;
      pipe2_is_wr     <= pipe1_is_wr;
      pipe3_is_wr     <= pipe2_is_wr;
      pipe4_is_wr     <= pipe3_is_wr;
      pipe5_is_wr     <= pipe4_is_wr;
      pipe1_access_size <= pipe1_access_size_d;
      pipe2_access_size <= pipe1_access_size;
      pipe3_access_size <= pipe2_access_size;
      pipe4_access_size <= pipe3_access_size;
      pipe5_access_size <= pipe4_access_size;
      pipe6_access_size <= pipe5_access_size;
      pipe7_access_size <= pipe6_access_size;
      pipe8_access_size <= pipe7_access_size;
      pipe9_access_size <= pipe8_access_size;
      pipe10_access_size <= pipe9_access_size;

      if (pipe5_valid & pipe5_is_wr) begin
        case (pipe5_access_size)
          BYTE: mem[pipe5_addr] <= pipe5_data[7:0];
          WORD: begin
            mem[pipe5_addr]     <= pipe5_data[7:0];
            mem[pipe5_addr + 1] <= pipe5_data[15:8];
            mem[pipe5_addr + 2] <= pipe5_data[23:16];
            mem[pipe5_addr + 3] <= pipe5_data[31:24];
          end
        endcase
      end
    end
  end

  assign pipe1_valid_d = rd_req_valid_i | wr_req_valid_i;
  assign pipe1_is_instr_d = req_is_instr_i;
  assign pipe1_is_wr_d = wr_req_valid_i;

  assign pipe1_access_size_d = access_size_i;
  assign pipe1_addr_d = address_i;
  assign pipe1_data_d = wr_data_i;

  assign data_valid_o = pipe10_valid;
  assign data_is_instr_o = pipe10_is_instr;
  assign data_o = pipe10_instr;

`ifndef SYNTHESIS
  assign debug_mem_o = mem;
`endif

  initial begin
    for (int i = 0; i < MEM_SIZE; ++i) begin
      mem[i] = 8'h0;
    end
    /*mem[1] = 32'h4470;      // ADD r1, r2 -> r7
    mem[2] = 32'h40B23;     // SUB r16, r5 -> r18
    mem[3] = 32'h446F1;     // LW @17(r3) -> r15
    mem[4] = 32'hFFFE1981;  // LW @-8(r12) -> r24
    mem[5] = 32'h36212;     // SW r1 -> @13(r17)
    mem[6] = 32'hFFFF50E2;  // SW r14 -> @-3(r8)
    mem[7] = 32'hA0DF9;     // BEQ r16, r6, 63
    mem[8] = 32'h98DF9;     // BEQ r6, r6, 63
    mem[61] = 32'h427A;     // BNE r1, r1, 3
    mem[62] = 32'h4CAA;     // BNE r1, r6, 10
    mem[71] = 32'hFFF98D69; // BEQ r6, r6, -10
    mem[72] = 32'h390AB;    // BLT r14, r8, 10
    mem[73] = 32'h21CAB;    // BLT r8, r14, 10
    mem[83] = 32'h6BEAC;    // BGE r26, r31, 10
    mem[84] = 32'h7F4AC;    // BGE r31, r26, 10
    mem[94] = 32'h400D;     // JMP r1*/
    
    mem[1] = 32'h446F1;
    mem[2] = 32'h4470;
    mem[3] = 32'h4470;
    mem[4] = 32'h4470;
    mem[5] = 32'h4470;
    mem[6] = 32'h4470;
    mem[7] = 32'h4470;
    mem[8] = 32'h4470;
    mem[9] = 32'h4470;
    mem[10] = 32'h4470;
  end

endmodule
