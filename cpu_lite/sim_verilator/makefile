
# Default binary to load & run
TEST_IMAGE    ?= firmware.bin

# Waveform trace (VCD output)
TRACE         ?= 1

# Top module (without .v extension)
TOP_MODULE     = tb_top

# CPP Source Files
SRC_CPP        = top.cpp

# Source directories
INC_DIRS       = -I. -I../cpu

# Build directory
BUILD_DIR      = build

# Verilator flags
VERILATOR_OPTS = +define+SIMULATION

ifeq ($(TRACE),1)
    VERILATOR_OPTS += --trace
endif

all: run

compile: clean 
	verilator --cc $(TOP_MODULE).v $(SRC_CPP) $(INC_DIRS) --exe -Mdir $(BUILD_DIR) $(VERILATOR_OPTS)
	make -j -f V$(TOP_MODULE).mk -C $(BUILD_DIR)

run: compile
	./$(BUILD_DIR)/V$(TOP_MODULE) $(TEST_IMAGE)
	
ifeq ($(TRACE),1)	
view:
	gtkwave wave_dump.vcd gtksettings.sav
endif
 
clean :
	-rm -rf $(BUILD_DIR) 2> /dev/null
	-rm wave_dump.vcd 2> /dev/null

