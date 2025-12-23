VERILOG_SOURCES =  params_pkg.sv tb.sv  wb_arbiter.sv alu.sv alu_stage.sv cpu.sv data_cache.sv decode_stage.sv dmem.sv ex_stages.sv fetch_stage.sv hazard_unit.sv imem.sv instr_cache.sv mem.sv mem_stage.sv  rbank.sv
TOP_MODULE      = tb
BUILD_DIR       = obj_dir
SIM_EXE         = $(BUILD_DIR)/V$(TOP_MODULE)

VERILATOR_FLAGS = \
	--cc \
	--sv \
	--exe \
	--build \
	-Wall \
	-Wno-TIMESCALEMOD \
	-Wno-UNUSEDSIGNAL \
	-Wno-IMPORTSTAR \
	-Wno-MODDUP \
	-Wno-VARHIDDEN \
	-Wno-PINCONNECTEMPTY \
	-Wno-WIDTHEXPAND \
	-Wno-UNUSEDPARAM \
	-Wno-CASEINCOMPLETE \
	-Wno-LATCH \
	--timing \
	-CFLAGS "-O3"


.PHONY: all clean run

all: run
	@echo "success at start"


$(SIM_EXE): $(VERILOG_SOURCES)
	@echo ">>> start"
	@verilator --binary $(VERILATOR_FLAGS) $(VERILOG_SOURCES) --top-module $(TOP_MODULE) 

run: $(SIM_EXE)
	@echo ">>> Terminal"
	@$(SIM_EXE)

clean:
	@echo ">>> clearr"
	@rm -rf $(BUILD_DIR)