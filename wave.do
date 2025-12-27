onerror {resume}
quietly set dataset_list [list sim vsim1 vsim]
if {[catch {datasetcheck $dataset_list}]} {abort}
quietly WaveActivateNextPane {} 0
add wave -noupdate sim:/tb/error
add wave -noupdate sim:/tb/mem/clk_i
add wave -noupdate -radix hexadecimal sim:/tb/rd_req_valid
add wave -noupdate sim:/tb/wr_req_valid
add wave -noupdate sim:/tb/req_is_instr
add wave -noupdate sim:/tb/req_address
add wave -noupdate sim:/tb/wr_data
add wave -noupdate sim:/tb/i_cpu/rd_req_valid_o
add wave -noupdate sim:/tb/i_cpu/icache_req
add wave -noupdate sim:/tb/i_cpu/icache_gnt
add wave -noupdate sim:/tb/i_cpu/dcache_req
add wave -noupdate sim:/tb/i_cpu/dcache_gnt
add wave -noupdate sim:/tb/i_cpu/dcache_rdata
add wave -noupdate sim:/tb/mem/pipe1_valid
add wave -noupdate sim:/tb/mem/pipe10_valid
add wave -noupdate -childformat {{{/tb/i_cpu/fetch_stage/i_cache/data_array[0]} -radix hexadecimal} {{/tb/i_cpu/fetch_stage/i_cache/data_array[1]} -radix hexadecimal} {{/tb/i_cpu/fetch_stage/i_cache/data_array[2]} -radix hexadecimal} {{/tb/i_cpu/fetch_stage/i_cache/data_array[3]} -radix hexadecimal}} -expand -subitemconfig {{/tb/i_cpu/fetch_stage/i_cache/data_array[0]} {-height 17 -radix hexadecimal} {/tb/i_cpu/fetch_stage/i_cache/data_array[1]} {-height 17 -radix hexadecimal} {/tb/i_cpu/fetch_stage/i_cache/data_array[2]} {-height 17 -radix hexadecimal} {/tb/i_cpu/fetch_stage/i_cache/data_array[3]} {-height 17 -radix hexadecimal}} sim:/tb/i_cpu/fetch_stage/i_cache/data_array
add wave -noupdate sim:/tb/cpu_wb_pc
add wave -noupdate -radix hexadecimal sim:/tb/i_cpu/fetch_stage/i_cache/cpu_addr
add wave -noupdate -radix hexadecimal sim:/tb/i_cpu/fetch_stage/i_cache/mem_req
add wave -noupdate -radix hexadecimal sim:/tb/i_cpu/fetch_stage/i_cache/cpu_rdata
add wave -noupdate sim:/tb/i_cpu/mem_stall
add wave -noupdate sim:/tb/i_cpu/mem_stage/stall_o
add wave -noupdate sim:/tb/i_cpu/mem_stage/stall_o
add wave -noupdate sim:/tb/i_cpu/fetch_stall
add wave -noupdate sim:/tb/i_cpu/dec_stall
add wave -noupdate sim:/tb/i_cpu/alu_stall
add wave -noupdate sim:/tb/i_cpu/wb_valid_from_mem
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {155991166 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 399
configure wave -valuecolwidth 318
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {154830074 ps} {158845786 ps}
