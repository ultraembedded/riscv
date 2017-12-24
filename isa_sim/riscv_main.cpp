//-----------------------------------------------------------------
//                     RISC-V ISA Simulator 
//                            V1.0
//                     Ultra-Embedded.com
//                     Copyright 2014-2017
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
#include <unistd.h>

#include "riscv.h"
#include "elf_load.h"
#include "cosim_api.h"

#include "riscv_main.h"

//-----------------------------------------------------------------
// mem_create: Create memory region
//-----------------------------------------------------------------
static int mem_create(void *arg, uint32_t base, uint32_t size)
{
    return cosim::instance()->create_memory(base, size);
}
//-----------------------------------------------------------------
// mem_load: Load byte into memory
//-----------------------------------------------------------------
static int mem_load(void *arg, uint32_t addr, uint8_t data)
{
    cosim::instance()->write(addr, data);
    return cosim::instance()->valid_addr(addr);
}
//-----------------------------------------------------------------
// riscv_main
//-----------------------------------------------------------------
int riscv_main(cosim_cpu_api *sim, int argc, char *argv[])
{
    unsigned _cycles = 0;
    int max_cycles = -1;
    char *filename = NULL;
    int help = 0;
    int trace = 0;
    unsigned int trace_mask = 1;
    int c;

    while ((c = getopt (argc, argv, "t:v:f:c:")) != -1)
    {
        switch(c)
        {
            case 't':
                trace = strtoul(optarg, NULL, 0);
                break;
            case 'v':
                trace_mask = strtoul(optarg, NULL, 0);
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
        fprintf (stderr,"-f filename.elf = Executable to load (ELF)\n");
        fprintf (stderr,"-t [0/1]        = Enable program trace\n");
        fprintf (stderr,"-v 0xX          = Trace Mask\n");
        fprintf (stderr,"-c nnnn         = Max instructions to execute\n");
        exit(-1);
    }

    uint32_t start_addr = 0;

    // Load ELF file
    if (elf_load(filename, mem_create, mem_load, sim, &start_addr))
    {
        printf("Starting from 0x%08x\n", start_addr);

        // Reset CPU to given start PC
        sim->reset(start_addr);

        // Enable trace?
        if (trace)
            sim->enable_trace(trace_mask);

        _cycles = 0;

        while (!sim->get_fault() && !sim->get_stopped())
        {
            sim->step();
            _cycles++;

            if (max_cycles != -1 && max_cycles == _cycles)
                break;
        }   
    }
    else
        fprintf (stderr,"Error: Could not open %s\n", filename);

    // Fault occurred?
    if (sim->get_fault())
        return 1;
    else
        return 0;
}
