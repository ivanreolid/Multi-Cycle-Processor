`ifndef CONFIG_SVH
`define CONFIG_SVH

  `ifndef DCACHE_N_LINES_MACRO
    `define DCACHE_N_LINES_MACRO 4
  `endif

  `ifndef ICACHE_N_LINES_MACRO
    `define ICACHE_N_LINES_MACRO 4
  `endif

  `ifndef CACHE_LINE_BYTES_MACRO
    `define CACHE_LINE_BYTES_MACRO 16
  `endif

`endif

