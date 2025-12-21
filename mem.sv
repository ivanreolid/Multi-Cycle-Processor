`timescale 1ns / 1ps

import params_pkg::*;

module mem #(
    parameter int MEM_SIZE   = 65536,      // Total memory size in bytes
    parameter int ADDR_WIDTH = 32,
    parameter int LINE_BYTES = 16          // 128-bit = 16 bytes per cache line
)(
    input  logic clk,
    input  logic rstn,
    
    // Instruction Cache
    input  logic                      icache_req,
    input  logic [ADDR_WIDTH-1:0]     icache_addr,    // Line-aligned
    output logic                      icache_gnt,      // Request granted
    output logic                      icache_rvalid,
    output logic [LINE_BYTES*8-1:0]   icache_rdata,   // 128-bit line
    
    // Data Cache
    input  logic                      dcache_req,
    input  logic                      dcache_we,      // 1=write, 0=read
    input  logic [ADDR_WIDTH-1:0]     dcache_addr,    // Line-aligned 
    input  logic [LINE_BYTES*8-1:0]   dcache_wdata,   // 128-bit line
    output logic                      dcache_gnt,      // Request granted
    output logic                      dcache_rvalid,
    `ifndef SYNTHESIS
    output [7:0] debug_mem_o [MEM_SIZE],
    `endif
    output logic [LINE_BYTES*8-1:0]   dcache_rdata    // 128-bit line


);

    logic [7:0] mem [MEM_SIZE];
    
    // ICache has priority over DCache
    
    logic arb_icache_grant, arb_dcache_grant;
    
    always_comb begin
        if (icache_req) begin
            // ICache has priority
            arb_icache_grant = 1'b1;
            arb_dcache_grant = 1'b0;
        end else if (dcache_req) begin
            // DCache gets access if ICache not requesting
            arb_icache_grant = 1'b0;
            arb_dcache_grant = 1'b1;
        end else begin
            // No requests
            arb_icache_grant = 1'b0;
            arb_dcache_grant = 1'b0;
        end
    end
    
    logic                      pipe1_valid;
    logic                      pipe1_is_icache;  // 1=icache, 0=dcache
    logic                      pipe1_is_write;
    logic [ADDR_WIDTH-1:0]     pipe1_addr;
    logic [LINE_BYTES*8-1:0]   pipe1_wdata;
    
    logic pipe2_valid, pipe3_valid, pipe4_valid, pipe5_valid;
    logic pipe2_is_icache, pipe3_is_icache, pipe4_is_icache, pipe5_is_icache;
    logic pipe2_is_write, pipe3_is_write, pipe4_is_write, pipe5_is_write;
    logic [ADDR_WIDTH-1:0] pipe2_addr, pipe3_addr, pipe4_addr, pipe5_addr;
    logic [LINE_BYTES*8-1:0] pipe2_wdata, pipe3_wdata, pipe4_wdata, pipe5_wdata;
    
    logic                      pipe6_valid;
    logic                      pipe6_is_icache;
    logic [LINE_BYTES*8-1:0]   pipe6_rdata;
    
    logic pipe7_valid, pipe8_valid, pipe9_valid, pipe10_valid;
    logic pipe7_is_icache, pipe8_is_icache, pipe9_is_icache, pipe10_is_icache;
    logic [LINE_BYTES*8-1:0] pipe7_rdata, pipe8_rdata, pipe9_rdata, pipe10_rdata;

    
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
            pipe1_is_icache <= 1'b0;
            pipe2_is_icache <= 1'b0;
            pipe3_is_icache <= 1'b0;
            pipe4_is_icache <= 1'b0;
            pipe5_is_icache <= 1'b0;
            pipe6_is_icache <= 1'b0;
            pipe7_is_icache <= 1'b0;
            pipe8_is_icache <= 1'b0;
            pipe9_is_icache <= 1'b0;
            pipe10_is_icache <= 1'b0;
            pipe1_is_write  <= 1'b0;
            pipe2_is_write  <= 1'b0;
            pipe3_is_write  <= 1'b0;
            pipe4_is_write  <= 1'b0;
            pipe5_is_write  <= 1'b0;
            pipe1_addr      <= '0;
            pipe1_wdata     <= '0;
        end else begin
            if (arb_icache_grant) begin
                pipe1_valid     <= 1'b1;
                pipe1_is_icache <= 1'b1;
                pipe1_is_write  <= 1'b0;
                pipe1_addr      <= icache_addr;
                pipe1_wdata     <= '0;
            end else if (arb_dcache_grant) begin
                pipe1_valid     <= 1'b1;
                pipe1_is_icache <= 1'b0;
                pipe1_is_write  <= dcache_we;
                pipe1_addr      <= dcache_addr;
                pipe1_wdata     <= dcache_wdata;
            end else begin
                pipe1_valid     <= 1'b0;
                pipe1_is_icache <= 1'b0;
                pipe1_is_write  <= 1'b0;
                pipe1_addr      <= '0;
                pipe1_wdata     <= '0;
            end
            
            pipe2_valid     <= pipe1_valid;
            pipe2_is_icache <= pipe1_is_icache;
            pipe2_is_write  <= pipe1_is_write;
            pipe2_addr      <= pipe1_addr;
            pipe2_wdata     <= pipe1_wdata;
            
            pipe3_valid     <= pipe2_valid;
            pipe3_is_icache <= pipe2_is_icache;
            pipe3_is_write  <= pipe2_is_write;
            pipe3_addr      <= pipe2_addr;
            pipe3_wdata     <= pipe2_wdata;
            
            pipe4_valid     <= pipe3_valid;
            pipe4_is_icache <= pipe3_is_icache;
            pipe4_is_write  <= pipe3_is_write;
            pipe4_addr      <= pipe3_addr;
            pipe4_wdata     <= pipe3_wdata;
            
            pipe5_valid     <= pipe4_valid;
            pipe5_is_icache <= pipe4_is_icache;
            pipe5_is_write  <= pipe4_is_write;
            pipe5_addr      <= pipe4_addr;
            pipe5_wdata     <= pipe4_wdata;
            
            pipe6_valid     <= pipe5_valid && !pipe5_is_write; 
            pipe6_is_icache <= pipe5_is_icache;
            
            pipe7_valid     <= pipe6_valid;
            pipe7_is_icache <= pipe6_is_icache;
            pipe7_rdata     <= pipe6_rdata;
            
            pipe8_valid     <= pipe7_valid;
            pipe8_is_icache <= pipe7_is_icache;
            pipe8_rdata     <= pipe7_rdata;
            
            pipe9_valid     <= pipe8_valid;
            pipe9_is_icache <= pipe8_is_icache;
            pipe9_rdata     <= pipe8_rdata;
            
            pipe10_valid     <= pipe9_valid;
            pipe10_is_icache <= pipe9_is_icache;
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
    
   
    assign icache_gnt    = arb_icache_grant;  // Immediate grant feedback
    assign icache_rvalid = pipe10_valid && pipe10_is_icache;
    assign icache_rdata  = pipe10_rdata;
    
    assign dcache_gnt    = arb_dcache_grant;  // Immediate grant feedback
    assign dcache_rvalid = pipe10_valid && !pipe10_is_icache;
    assign dcache_rdata  = pipe10_rdata;

`ifndef SYNTHESIS
    assign debug_mem_o = mem;
`endif

    initial begin
        //$readmemh("buffer_sum.mem", mem);
        //$readmemh("mem_copy.mem", mem);
        $readmemh("matrix_multiply.mem", mem);
    end

endmodule
