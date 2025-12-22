`timescale 1ns / 1ps

import params_pkg::*;

module mem #(
    parameter int MEM_SIZE   = 65536,      // Total memory size in bytes
    parameter int ADDR_WIDTH = 32,
    parameter int LINE_BYTES = 16          // 128-bit = 16 bytes per cache line
)(
    input  logic clk,
    input  logic rstn,
    
    input  logic                      mem_req,
    input  logic                      mem_we,
    input  logic [ADDR_WIDTH-1:0]     mem_addr,
    input  logic [LINE_BYTES*8-1:0]   mem_wdata,
    output logic                      mem_gnt,
    output logic                      mem_rvalid,
    output logic [LINE_BYTES*8-1:0]   mem_rdata,

`ifndef SYNTHESIS
    output logic [7:0] debug_mem_o [MEM_SIZE]
`endif
);

    logic [7:0] mem [MEM_SIZE];

    
    logic                      pipe1_valid, pipe2_valid, pipe3_valid, pipe4_valid, pipe5_valid;
    logic                      pipe1_is_write, pipe2_is_write, pipe3_is_write, pipe4_is_write, pipe5_is_write;
    logic [ADDR_WIDTH-1:0]     pipe1_addr, pipe2_addr, pipe3_addr, pipe4_addr, pipe5_addr;
    logic [LINE_BYTES*8-1:0]   pipe1_wdata, pipe2_wdata, pipe3_wdata, pipe4_wdata, pipe5_wdata;
    logic                      pipe6_valid, pipe7_valid, pipe8_valid, pipe9_valid, pipe10_valid;
    logic [LINE_BYTES*8-1:0]   pipe6_rdata, pipe7_rdata, pipe8_rdata, pipe9_rdata, pipe10_rdata;

    always_comb begin
        pipe6_rdata = '0;

        if (pipe5_valid && !pipe5_is_write) begin
            for (int i = 0; i < LINE_BYTES; i++) begin
                if ((pipe5_addr + i) < MEM_SIZE) begin
                    pipe6_rdata[(i*8) +: 8] = mem[pipe5_addr + i];
                end
            end
        end
    end

    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
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
            pipe1_is_write  <= 1'b0;
            pipe2_is_write  <= 1'b0;
            pipe3_is_write  <= 1'b0;
            pipe4_is_write  <= 1'b0;
            pipe5_is_write  <= 1'b0;
            pipe1_addr      <= '0;
            pipe1_wdata     <= '0;
        end else begin
            if (mem_req) begin
                pipe1_valid     <= 1'b1;
                pipe1_is_write  <= mem_we;
                pipe1_addr      <= mem_addr;
                pipe1_wdata     <= mem_wdata;
            end else begin
                pipe1_valid     <= 1'b0;
                pipe1_is_write  <= 1'b0;
                pipe1_addr      <= '0;
                pipe1_wdata     <= '0;
            end
            
            pipe2_valid     <= pipe1_valid;
            pipe2_is_write  <= pipe1_is_write;
            pipe2_addr      <= pipe1_addr;
            pipe2_wdata     <= pipe1_wdata;
            
            pipe3_valid     <= pipe2_valid;
            pipe3_is_write  <= pipe2_is_write;
            pipe3_addr      <= pipe2_addr;
            pipe3_wdata     <= pipe2_wdata;
            
            pipe4_valid     <= pipe3_valid;
            pipe4_is_write  <= pipe3_is_write;
            pipe4_addr      <= pipe3_addr;
            pipe4_wdata     <= pipe3_wdata;
            
            pipe5_valid     <= pipe4_valid;
            pipe5_is_write  <= pipe4_is_write;
            pipe5_addr      <= pipe4_addr;
            pipe5_wdata     <= pipe4_wdata;
            
            pipe6_valid     <= pipe5_valid && !pipe5_is_write;
            
            pipe7_valid     <= pipe6_valid;
            pipe7_rdata     <= pipe6_rdata;
            
            pipe8_valid     <= pipe7_valid;
            pipe8_rdata     <= pipe7_rdata;
            
            pipe9_valid     <= pipe8_valid;
            pipe9_rdata     <= pipe8_rdata;
            
            pipe10_valid     <= pipe9_valid;
            pipe10_rdata     <= pipe9_rdata;
            
            if (pipe5_valid && pipe5_is_write) begin
                for (int i = 0; i < LINE_BYTES; i++) begin
                    if ((pipe5_addr + i) < MEM_SIZE) begin
                        mem[pipe5_addr + i] <= pipe5_wdata[(i*8) +: 8];
                    end
                end
            end
        end
    end
    
    assign mem_gnt    = mem_req; 
    assign mem_rvalid = pipe10_valid;
    assign mem_rdata  = pipe10_rdata;

`ifndef SYNTHESIS
    assign debug_mem_o = mem;
`endif

    initial begin
        //$readmemh("buffer_sum.mem", mem);
        //$readmemh("mem_copy.mem", mem);
        $readmemh("matrix_multiply.mem", mem);
    end

endmodule
