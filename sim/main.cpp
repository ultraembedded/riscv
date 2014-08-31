//-----------------------------------------------------------------
//                     RISC-V ISA Simulator 
//                            V0.1
//                     Ultra-Embedded.com
//                       Copyright 2014
//
//                   admin@ultra-embedded.com
//
//                       License: BSD
//-----------------------------------------------------------------
//
// Copyright (c) 2014, Ultra-Embedded.com
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions 
// are met:
//   - Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   - Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer 
//     in the documentation and/or other materials provided with the 
//     distribution.
//   - Neither the name of the author nor the names of its contributors 
//     may be used to endorse or promote products derived from this 
//     software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
// THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
// SUCH DAMAGE.
//-----------------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "riscv.h"

#include <unistd.h>

//-----------------------------------------------------------------
// Defines
//-----------------------------------------------------------------
#define DEFAULT_MEM_BASE            0x10000000
#define DEFAULT_MEM_SIZE            (10 << 20)
#define DEFAULT_LOAD_ADDR           0x10000000
#define DEFAULT_FILENAME            NULL

//-----------------------------------------------------------------
// Locals
//-----------------------------------------------------------------
static unsigned _cycles = 0;

//-----------------------------------------------------------------
// Uart
//-----------------------------------------------------------------
#define UART_BASE                   (0x12000000 + 0x000)
#define UART_UDR                    (UART_BASE + 0x8)

class Uart: public Memory
{
public:

    virtual void Reset(void) {}

    virtual uint32_t Load(uint32_t address, int width, bool signedLoad)
    {
        return 0;
    }

    virtual void Store(uint32_t address, uint32_t data, int width)
    {
        if (address == (UART_UDR - UART_BASE))
        {
            fprintf(stderr, "%c", data & 0xFF);
            return ;
        }
    }
};

//-----------------------------------------------------------------
// Timer
//-----------------------------------------------------------------
#define TIMER_BASE                  (0x12000000 + 0x100)
#define TIMER_VAL                   (TIMER_BASE + 0x0)
#define SYS_CLK_COUNT               (TIMER_BASE + 0x4)

#define CLK_FREQ                    1000000

class Timer: public Memory
{
public:

    virtual void Reset(void) {}

    virtual uint32_t Load(uint32_t address, int width, bool signedLoad)
    {
        if (address == (TIMER_VAL - TIMER_BASE))
            return _cycles / (CLK_FREQ / 1000);
        else if (address == (SYS_CLK_COUNT - TIMER_BASE))
            return _cycles;
        else
            return 0;
    }

    virtual void Store(uint32_t address, uint32_t data, int width)
    {

    }
};

//-----------------------------------------------------------------
// main
//-----------------------------------------------------------------
int main(int argc, char *argv[])
{
    int c;
    unsigned int loadAddr = DEFAULT_LOAD_ADDR;
    unsigned int memBase = DEFAULT_MEM_BASE;
    unsigned int memSize = DEFAULT_MEM_SIZE;
    unsigned int startAddr = DEFAULT_MEM_BASE;
    int max_cycles = -1;
    char *filename = DEFAULT_FILENAME;
    int help = 0;
    int trace = 0;
    unsigned int trace_mask = 1;
    int exitcode = -1;
    unsigned int stop_pc = 0xFFFFFFFF;    
    Riscv *sim = NULL;

    while ((c = getopt (argc, argv, "t:v:l:b:s:f:c:x:nd:z:r:")) != -1)
    {
        switch(c)
        {
            case 't':
                 trace = strtoul(optarg, NULL, 0);
                 break;
            case 'v':
                 trace_mask = strtoul(optarg, NULL, 0);
                 break;
            case 'l':
                 loadAddr = strtoul(optarg, NULL, 0);
                 break;
            case 'b':
                 memBase = strtoul(optarg, NULL, 0);
                 break;
            case 's':
                 memSize = strtoul(optarg, NULL, 0);
                 break;
            case 'x':
                 startAddr = strtoul(optarg, NULL, 0);
                 break;
            case 'r':
                 stop_pc = strtoul(optarg, NULL, 0);
                 break;
            case 'f':
                 filename = optarg;
                 break;               
            case 'c':
                 max_cycles = (int)strtoul(optarg, NULL, 0);
                 break;
            case '?':
            default:
                help = 1;   
                break;
        }
    }

    if (help || filename == NULL)
    {
        fprintf (stderr,"Usage:\n");
        fprintf (stderr,"-f filename.bin = Executable to load (binary)\n");
        fprintf (stderr,"-t              = Enable program trace\n");
        fprintf (stderr,"-v 0xX          = Trace Mask\n");
        fprintf (stderr,"-b 0xnnnn       = Memory base address\n");
        fprintf (stderr,"-s 0xnnnn       = Memory size\n");
        fprintf (stderr,"-l 0xnnnn       = Executable load address\n");     
        fprintf (stderr,"-x 0xnnnn       = Executable boot address\n");     
        fprintf (stderr,"-c nnnn         = Max instructions to execute\n");
        fprintf (stderr,"-r 0xnnnn       = Stop at PC address\n");     
 
        exit(-1);
    }

    sim = new Riscv(memBase, memSize);
    sim->Reset(startAddr);

    // Add simple peripherals
    sim->AttachMemory(new Uart(),  UART_BASE,  0xFF);
    sim->AttachMemory(new Timer(), TIMER_BASE, 0xFF);

    // Enable trace?
    if (trace)
        sim->EnableTrace(trace_mask);

    // Load file
    FILE *f = fopen(filename, "rb");
    if (f)
    {
        long size;
        char *buf;

        // Get size
        fseek(f, 0, SEEK_END);
        size = ftell(f);
        rewind(f);

        buf = (char*)malloc(size+1);
        if (buf)
        {
            // Read file data in
            int len = fread(buf, 1, size, f);
            buf[len] = 0;

            if (sim->LoadMem(loadAddr, (unsigned char *)buf, len))
            {
                _cycles = 0;

                while (!sim->GetBreak() && !sim->GetFault() && sim->GetPC() != stop_pc)
                {
                    sim->Step();
                    _cycles++;

                    if (max_cycles != -1 && max_cycles == _cycles)
                        break;
                }   
            }
            else
                fprintf (stderr,"Error: Could not load image to memory\n");

            free(buf);
            fclose(f);
        }
        // Show execution stats
        sim->DumpStats();

        // Fault occurred?
        if (sim->GetFault())
            exitcode = 1;
        else
            exitcode = 0;
    }
    else
        fprintf (stderr,"Error: Could not open %s\n", filename);

    delete sim;

    return exitcode;
}
