GHDL      = ghdl
STD       = --std=08
SRC_DIR   = ../src
TB_DIR    = ../tb

# Auto-discover all VHDL files in src/ and tb/
SRC_FILES = $(wildcard $(SRC_DIR)/*.vhd)
TB_FILES  = $(wildcard $(TB_DIR)/*.vhd)

# Auto-detect the testbench entity name from the tb file
# Assumes one tb file per IP, named tb_<something>.vhd with entity tb_<something>
TOP_TB    = $(basename $(notdir $(firstword $(TB_FILES))))

STOP_TIME = 500us

.PHONY: all analyze elaborate run wave clean info

all: run

info:
	@echo "SRC_FILES = $(SRC_FILES)"
	@echo "TB_FILES  = $(TB_FILES)"
	@echo "TOP_TB    = $(TOP_TB)"

analyze:
	$(GHDL) -a $(STD) $(SRC_FILES)
	$(GHDL) -a $(STD) $(TB_FILES)

elaborate: analyze
	$(GHDL) -e $(STD) $(TOP_TB)

run: elaborate
	$(GHDL) -r $(STD) $(TOP_TB) --fst=wave.fst --stop-time=$(STOP_TIME)

wave: run
	@if [ -f wave.gtkw ]; then \
		gtkwave wave.fst wave.gtkw & \
	else \
		gtkwave wave.fst & \
	fi

clean:
	rm -f *.cf *.o $(TOP_TB) wave.fst


## to run : make wave