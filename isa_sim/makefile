###############################################################################
## Simulator Makefile
###############################################################################

# Target
TARGET		= riscv-sim

# Options
CFLAGS	    = -O2 -fPIC
LDFLAGS     = 
LIBS        = -lelf

# Source Files
SRC     = $(wildcard *.cpp)
SRC_LIB = $(filter-out main.cpp, $(wildcard *.cpp))
LIB_OBJ = $(patsubst %.cpp,%.o,$(SRC_LIB))
OBJ     = $(patsubst %.cpp,%.o,$(SRC))

###############################################################################
# Rules
###############################################################################
all: $(TARGET)
	
clean:
	-rm *.o $(TARGET) libisa_sim.a

%.o : %.cpp
	g++ -c $(CFLAGS) $< -o $@

$(TARGET): $(OBJ) makefile
	g++ $(LDFLAGS) $(OBJ) $(LIBS) -o $@

lib: $(LIB_OBJ)
	g++ -shared -o libisa_sim.a $(LIB_OBJ)

run: $(TARGET)
	./$(TARGET) -f firmware.elf
