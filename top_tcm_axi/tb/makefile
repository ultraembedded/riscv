###############################################################################
## Tool paths
###############################################################################
VERILATOR_SRC ?= /usr/share/verilator/include
SYSTEMC_HOME  ?= /usr/local/systemc-2.3.1

TEST_IMAGE ?= ../../isa_sim/images/basic.elf

CORE       ?= rv32im

export VERILATOR_SRC
export SYSTEMC_HOME

###############################################################################
## Makefile
###############################################################################
all: build

build:
	make -C ../../isa_sim lib
	make -f makefile.generate_verilated CORE=$(CORE)
	make -f makefile.build_verilated
	make -f makefile.build_sysc_tb

clean:
	make -f makefile.generate_verilated  CORE=$(CORE) $@
	make -f makefile.build_verilated $@
	make -f makefile.build_sysc_tb $@
	-rm *.vcd

run: build
	./build/test.x -f $(TEST_IMAGE)