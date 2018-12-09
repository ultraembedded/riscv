###############################################################################
## Simulator Makefile
###############################################################################

# Target
TARGET	   ?= riscv-sim
TARGET_LIB ?= libisa_sim.a

RUN_ELF    ?= images/linux.elf
RUN_OPTS   ?= "-b 0x80000000 -s 33554432"

# Options
MMU        ?= yes

CFLAGS	    = -O2 -fPIC
ifeq ($(MMU), yes)
CFLAGS     += -DCONFIG_MMU
endif

LDFLAGS     = 
LIBS        = -lelf -lbfd

# Source Files
SRC_DIR    = .

###############################################################################
# Variables
###############################################################################
OBJ_DIR      ?= obj/$(TARGET)/

###############################################################################
# Variables: Lists of objects, source and deps
###############################################################################
# SRC / Object list
src2obj       = $(OBJ_DIR)$(patsubst %$(suffix $(1)),%.o,$(notdir $(1)))

SRC          ?= $(foreach src,$(SRC_DIR),$(wildcard $(src)/*.cpp))
OBJ          ?= $(foreach src,$(SRC),$(call src2obj,$(src)))
LIB_OBJ      ?= $(foreach src,$(filter-out main.cpp,$(SRC)),$(call src2obj,$(src)))

###############################################################################
# Rules: Compilation macro
###############################################################################
define template_cpp
$(call src2obj,$(1)): $(1) | $(OBJ_DIR)
	@echo "# Compiling $(notdir $(1))"
	@g++ $(CFLAGS) -c $$< -o $$@
endef

###############################################################################
# Rules
###############################################################################
all: $(TARGET) lib
	
$(OBJ_DIR):
	@mkdir -p $@

$(foreach src,$(SRC),$(eval $(call template_cpp,$(src))))	

$(TARGET): $(OBJ) makefile
	g++ $(LDFLAGS) $(OBJ) $(LIBS) -o $@

lib: $(LIB_OBJ)
	g++ -shared -o $(TARGET_LIB) $(LIB_OBJ)

clean:
	-rm -rf $(OBJ_DIR) $(TARGET) $(TARGET_LIB)

run: $(TARGET)
	./$(TARGET) -f $(RUN_ELF) $(RUN_OPTS)
