vlog -sv params_pkg.sv
vlog -sv alu.sv alu_stage.sv rbank.sv
vlog -sv fetch_stage.sv decode_stage.sv ex_stages.sv
vlog -sv mem.sv data_cache.sv instr_cache.sv mem_stage.sv
vlog -sv hazard_unit.sv wb_arbiter.sv
vlog -sv cpu.sv
vlog -sv tb.sv

restart -f
transcript file sim_output.log
run -all
transcript file off
