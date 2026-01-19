module mem import params_pkg::*; #(
  parameter int MEM_SIZE   = 4096,
  parameter int DATA_WIDTH = 128
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic rd_req_valid_i,
  input  logic wr_req_valid_i,
  input  logic req_is_instr_i,
  input  paddr_t address_i,
  input  cacheline_t wr_data_i,
  input  access_size_t access_size_i,
  output logic data_valid_o,
  output logic data_is_instr_o,
  output cacheline_t data_o,
  output logic write_done_o,
  input logic finish,
`ifndef SYNTHESIS
  output [7:0] debug_mem_o [MEM_SIZE]
`endif
);

  logic [7:0] mem [MEM_SIZE];

  logic pipe1_valid_d;
  logic pipe1_valid, pipe2_valid, pipe3_valid, pipe4_valid, pipe5_valid, pipe6_valid, pipe7_valid,
        pipe8_valid, pipe9_valid, pipe10_valid;

  logic pipe1_is_wr_d;
  logic pipe1_is_wr, pipe2_is_wr, pipe3_is_wr, pipe4_is_wr, pipe5_is_wr;

  access_size_t pipe1_access_size_d;
  access_size_t pipe1_access_size, pipe2_access_size, pipe3_access_size, pipe4_access_size,
                pipe5_access_size, pipe6_access_size, pipe7_access_size, pipe8_access_size,
                pipe9_access_size, pipe10_access_size;

  logic pipe1_is_instr_d;
  logic pipe1_is_instr, pipe2_is_instr, pipe3_is_instr, pipe4_is_instr, pipe5_is_instr,
        pipe6_is_instr, pipe7_is_instr, pipe8_is_instr, pipe9_is_instr, pipe10_is_instr;

  paddr_t pipe1_addr_d;
  paddr_t pipe1_addr, pipe2_addr, pipe3_addr, pipe4_addr, pipe5_addr;

  cacheline_t pipe6_read_data_d;
  cacheline_t pipe6_read_data, pipe7_read_data, pipe8_read_data, pipe9_read_data,
                         pipe10_read_data;

  cacheline_t pipe1_write_data_d;
  cacheline_t pipe1_write_data, pipe2_write_data, pipe3_write_data, pipe4_write_data,
                         pipe5_write_data;

  // Write done logic
  logic finish_reg;
  logic [4:0] idle_counter;

  always_ff @(posedge clk_i) begin
    if (!rst_i) begin
      finish_reg <= 1'b0;
      idle_counter <= '0;
      write_done_o <= 1'b0;
    end else begin
      if (finish && !finish_reg) begin
        finish_reg <= 1'b1;
        idle_counter <= '0;
        write_done_o <= 1'b0;
      end

      if (finish_reg) begin
        if (wr_req_valid_i || pipe1_is_wr || pipe2_is_wr || pipe3_is_wr || pipe4_is_wr ||
                              pipe5_is_wr) begin
          idle_counter <= '0;
          write_done_o <= 1'b0;
        end else begin
          if (idle_counter < 5'd15) begin
            idle_counter <= idle_counter + 1'b1;
          end

          if (idle_counter >= 5'd10) begin
            write_done_o <= 1'b1;
          end
        end
      end
    end
  end

  always_comb begin : memory_operation
    pipe6_read_data_d = '0;
    if (pipe5_valid && !pipe5_is_wr) begin
      for (int i = 0; i < DATA_WIDTH/8; i++) begin
        pipe6_read_data_d[8*i +: 8] = mem[pipe5_addr+i];
      end
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
      pipe1_valid       <= pipe1_valid_d;
      pipe2_valid       <= pipe1_valid;
      pipe3_valid       <= pipe2_valid;
      pipe4_valid       <= pipe3_valid;
      pipe5_valid       <= pipe4_valid;
      pipe6_valid       <= pipe5_is_wr ? 1'b0 : pipe5_valid;
      pipe7_valid       <= pipe6_valid;
      pipe8_valid       <= pipe7_valid;
      pipe9_valid       <= pipe8_valid;
      pipe10_valid      <= pipe9_valid;
      pipe1_is_instr    <= pipe1_is_instr_d;
      pipe2_is_instr    <= pipe1_is_instr;
      pipe3_is_instr    <= pipe2_is_instr;
      pipe4_is_instr    <= pipe3_is_instr;
      pipe5_is_instr    <= pipe4_is_instr;
      pipe6_is_instr    <= pipe5_is_instr;
      pipe7_is_instr    <= pipe6_is_instr;
      pipe8_is_instr    <= pipe7_is_instr;
      pipe9_is_instr    <= pipe8_is_instr;
      pipe10_is_instr   <= pipe9_is_instr;
      pipe1_addr        <= pipe1_addr_d;
      pipe2_addr        <= pipe1_addr;
      pipe3_addr        <= pipe2_addr;
      pipe4_addr        <= pipe3_addr;
      pipe5_addr        <= pipe4_addr;
      pipe6_read_data   <= pipe6_read_data_d;
      pipe7_read_data   <= pipe6_read_data;
      pipe8_read_data   <= pipe7_read_data;
      pipe9_read_data   <= pipe8_read_data;
      pipe10_read_data  <= pipe9_read_data;
      pipe1_write_data  <= pipe1_write_data_d;
      pipe2_write_data  <= pipe1_write_data;
      pipe3_write_data  <= pipe2_write_data;
      pipe4_write_data  <= pipe3_write_data;
      pipe5_write_data  <= pipe4_write_data;
      pipe1_is_wr        <= pipe1_is_wr_d;
      pipe2_is_wr        <= pipe1_is_wr;
      pipe3_is_wr        <= pipe2_is_wr;
      pipe4_is_wr        <= pipe3_is_wr;
      pipe5_is_wr        <= pipe4_is_wr;
      pipe1_access_size  <= pipe1_access_size_d;
      pipe2_access_size  <= pipe1_access_size;
      pipe3_access_size  <= pipe2_access_size;
      pipe4_access_size  <= pipe3_access_size;
      pipe5_access_size  <= pipe4_access_size;
      pipe6_access_size  <= pipe5_access_size;
      pipe7_access_size  <= pipe6_access_size;
      pipe8_access_size  <= pipe7_access_size;
      pipe9_access_size  <= pipe8_access_size;
      pipe10_access_size <= pipe9_access_size;

      if (pipe5_valid && pipe5_is_wr) begin
        for (int i = 0; i < DATA_WIDTH/8; i++) begin
          mem[pipe5_addr+i] <= pipe5_write_data[i*8 +: 8];
        end
      end
    end
  end

  assign pipe1_valid_d        = rd_req_valid_i | wr_req_valid_i;
  assign pipe1_is_instr_d     = req_is_instr_i;
  assign pipe1_is_wr_d        = wr_req_valid_i;
  assign pipe1_access_size_d  = access_size_i;
  assign pipe1_addr_d         = address_i;
  assign pipe1_write_data_d   = wr_data_i;

  assign data_valid_o     = pipe10_valid;
  assign data_is_instr_o = pipe10_is_instr;
  assign data_o          = pipe10_read_data;

`ifndef SYNTHESIS
  assign debug_mem_o = mem;
`endif

  integer i;
  initial begin

    for (i = 0; i < MEM_SIZE; i = i + 1) begin
      mem[i] = 8'b0;
    end
    //$readmemh("buffer_sum.mem", mem);
    //$readmemh("buffer_sum_no_vm.mem", mem);
    //$readmemh("mem_copy.mem", mem);
    //$readmemh("mem_copy_no_vm.mem", mem);
    $readmemh("matrix_multiply.mem", mem);
    //$readmemh("matrix_multiply_no_vm.mem", mem);
  end

endmodule
