RUN_TIME      	?= 1000us
TRACE   		?= 1
VSIM_PATH		?= /opt/modelsim
VERBOSE         ?= 1

all: compile run view                                                                                      
 
# Testbench
SRC+= ./top_tb.sv inst_gen.sv wb_slave.sv
SRC+= ../cpu/riscv.v
SRC+= ../cpu/riscv_soc.v
SRC+= ../cpu/riscv_core.v
SRC+= ../cpu/riscv_alu.v
SRC+= ../cpu/riscv_regfile_sim.v

SRC_FLAGS = +define+RUN_TIME=$(RUN_TIME) +define+SIMULATION

ifeq ($(TRACE),1)
    SRC_FLAGS += +define+TRACE=$(TRACE)
endif

ifeq ($(VERBOSE),1)
    SRC_FLAGS += +define+VERBOSE
endif

INC_DIRS = -I.

compile : 
	$(VSIM_PATH)/bin/vlib work
	$(VSIM_PATH)/bin/vlog $(SRC) $(SRC_FLAGS)
	gcc -c -g -m32 vpi.c -I$(VSIM_PATH)/include -DVERBOSE=$(VERBOSE)
	ld  -m elf_i386 -shared -E -o vpi.sl vpi.o

run : compile
	vsim -c -do "run -all" top_tb -pli vpi.sl
 
view : compile
ifeq ($(TRACE),1)
	gtkwave waveform.vcd gtksettings.sav  
endif

clean :                                                                                                  
	-rm -rf work waveform.vcd transcript *.o *.sl
