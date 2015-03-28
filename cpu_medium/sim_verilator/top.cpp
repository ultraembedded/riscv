#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <termios.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "Vtb_top__Syms.h"
#include "verilated.h"

#if VM_TRACE
#include <verilated_vcd_c.h>
#endif

//-----------------------------------------------------------------
// Defines
//-----------------------------------------------------------------
#define MAX_MEM             65536

// Serial print memory location
#define IPC_SER_TX          0x00000004

//-----------------------------------------------------------------
// Locals
//-----------------------------------------------------------------
static Vtb_top *top;

#if VM_TRACE
static unsigned int        main_time = 0;
static VerilatedVcdC*      tfp;
#endif

//-----------------------------------------------------------------
// load_image
//-----------------------------------------------------------------
static int load_image(const char *filename)
{
    FILE *f;
    int res = 0;

    f = fopen(filename, "rb");
    if (f)
    {
        long size;
        unsigned char *buf;

        // Get size
        fseek(f, 0, SEEK_END);
        size = ftell(f);
        rewind(f);

        buf = (unsigned char*)malloc(size);
        if (buf)
        {
            unsigned int addr;

            // Read file data in
            int len = fread((char*)buf, 1, size, f);

            printf("Loading %s (%d bytes)\n", filename, len);
            for (addr=0;addr<len;addr++)
            {
                if (addr < MAX_MEM)
                {
                    top->v->u_iram->ram[addr/4] |= (buf[addr] << (8 * (addr & 0x3)));
                    top->v->u_dram->ram[addr/4] |= (buf[addr] << (8 * (addr & 0x3)));
                }
                else
                {
                    printf("Memory overflow!\n");
                    exit(-1);
                }
            }

            free(buf);
            res = 1;
        }

        fclose(f);
    }

    return res;
}
//-----------------------------------------------------------------
// main
//-----------------------------------------------------------------
int main(int argc, char *argv[])
{
    int current_cycle = 0;
    const char *filename = "firmware.bin";

    if (argc > 1)
        filename = (const char*)argv[1];

    top = new Vtb_top();

#if VM_TRACE                  
    // If verilator was invoked with --trace
    Verilated::traceEverOn(true);
    VL_PRINTF("Enabling GTKWave Trace Output...\n");
    tfp = new VerilatedVcdC;
    top->trace (tfp, 99);
    tfp->open ("wave_dump.vcd");
#endif

    // Initial
    top->clk_i = 0;
    top->rst_i = 1;
    top->eval();

#if VM_TRACE
        if (tfp) tfp->dump (main_time++);
#endif

    // Reset
    top->clk_i = 1;
    top->rst_i = 1;
    top->eval();

#if VM_TRACE
        if (tfp) tfp->dump (main_time++);
#endif

    top->clk_i = 0;
    top->rst_i = 0;
    top->eval();

    if (load_image(filename) == 0)
    {
        printf("Couldn't open file %s\n", filename);
        exit(-1);
    }

    // Run until fault
    while (!Verilated::gotFinish() && !(top->fault_o || top->break_o)) 
    {
        // CLK->L
        top->clk_i = 0;
        top->eval();

#if VM_TRACE
        if (tfp) tfp->dump (main_time++);
#endif

        // CLK->H
        top->clk_i = 1;
        top->eval();

#if VM_TRACE
        if (tfp) tfp->dump (main_time++);
#endif

        // Serial print
        if (top->v->u_dram->ram[IPC_SER_TX/4] != 0)
        {
            printf("%c", top->v->u_dram->ram[IPC_SER_TX/4] & 0xFF);
            top->v->u_dram->ram[IPC_SER_TX/4] = 0;
        }

        current_cycle++;
    }

    printf("Cycles = %d\n", current_cycle);

    // Fault
    if (top->fault_o)
        printf("FAULT!\n");
    // Breakpoint hit?
    else if (top->break_o)
        printf("Break!\n");

    // Cleanup
    top->final();
#if VM_TRACE
    if (tfp)
    {
        tfp->close();
        tfp = NULL;
    }
#endif

    return (top->fault_o) ? -1: 0;
}
