# RISC-V Instruction Set Simulator

A simple RISC-V instruction set simulator for RV32IM.

## Building

Dependencies;
* gcc
* make
* libelf
* libbfd

To install the dependencies on Linux Ubuntu/Mint;
```
sudo apt-get install libelf-dev binutils-dev
```

To build the executable, type:
```
make
````

## Usage

The simulator will load and run a compiled ELF (compiled with RV32I or RV32IM compiler options);
```
# Using a makerule
make run

# Or running directly
./riscv-sim -f images/basic.elf
./riscv-sim -f images/linux.elf -b 0x80000000 -s 33554432
```

There are two example pre-compiled ELFs provided, one which is a basic machine mode only test program, and one
which boots Linux (modified 4.19 compiled for RV32IM).

## Extensions

The following primitives can be used to print to the console or to exit a simulation;
```

#define CSR_SIM_CTRL_EXIT (0 << 24)
#define CSR_SIM_CTRL_PUTC (1 << 24)

static inline void sim_exit(int exitcode)
{
    unsigned int arg = CSR_SIM_CTRL_EXIT | ((unsigned char)exitcode);
    asm volatile ("csrw dscratch,%0": : "r" (arg));
}

static inline void sim_putc(int ch)
{
    unsigned int arg = CSR_SIM_CTRL_PUTC | (ch & 0xFF);
    asm volatile ("csrw dscratch,%0": : "r" (arg));
}
```
