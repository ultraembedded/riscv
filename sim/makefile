###############################################################################
## Simulator Makefile
###############################################################################

# Target
TARGET		= riscv-sim

# Options
CFLAGS	    = -O2
LDFLAGS     = 

# Source Files
OBJ = main.o riscv.o 

###############################################################################
# Rules
###############################################################################
all: $(TARGET)
	
clean:
	-rm *.o $(TARGET)

%.o : %.c
	gcc -c $(CFLAGS) $< -o $@

%.o : %.cpp
	g++ -c $(CFLAGS) $< -o $@

$(TARGET): $(OBJ) makefile
	g++ $(LDFLAGS) $(OBJ) $(LIBS) -o $@

test: $(TARGET)
	./$(TARGET) -f test_image.bin