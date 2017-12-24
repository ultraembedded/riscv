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
#ifndef __RISCV_H__
#define __RISCV_H__

#include <stdint.h>
#include "riscv_isa.h"
#include "cosim_api.h"
#include "memory.h"

//--------------------------------------------------------------------
// Defines:
//--------------------------------------------------------------------
#define LOG_INST            (1 << 0)
#define LOG_OPCODES         (1 << 1)
#define LOG_REGISTERS       (1 << 2)
#define LOG_MEM             (1 << 3)
#define LOG_MACH_TRACE      (1 << 4)

#define MAX_MEM_REGIONS     8

//--------------------------------------------------------------------
// Enums:
//--------------------------------------------------------------------
enum eStats
{ 
    STATS_MIN,
    STATS_INSTRUCTIONS = STATS_MIN,
    STATS_LOADS,
    STATS_STORES,
    STATS_BRANCHES,
    STATS_MAX
};

//--------------------------------------------------------------------
// Abstract interface for stats
//--------------------------------------------------------------------
class StatsInterface
{
public:  
    virtual void        Reset(void) = 0;
    virtual void        Execute(uint32_t pc, uint32_t opcode) = 0;
    virtual void        Print(void) = 0;
};

//--------------------------------------------------------------------
// Class
//--------------------------------------------------------------------
class Riscv: public cosim_cpu_api, public cosim_mem_api
{
public:
                        Riscv();
                        Riscv(uint32_t baseAddr, uint32_t len);
    virtual             ~Riscv();

    bool                create_memory(uint32_t addr, uint32_t size, uint8_t *mem = NULL);
    bool                AttachMemory(Memory *memory, uint32_t baseAddr, uint32_t len);

    bool                LoadMem(uint32_t startAddr, uint8_t *data, int len);

    bool                valid_addr(uint32_t address);
    void                write(uint32_t address, uint8_t data);
    uint8_t             read(uint32_t address);

    void                reset(uint32_t start_addr);
    uint32_t            get_opcode(uint32_t pc);
    void                step(void);

    void                set_interrupt(int irq);

    bool                get_fault(void) { return Fault; }
    bool                get_stopped(void) { return Break; }
    bool                get_reg_valid(int r) { return true; }
    uint32_t            get_register(int r) { return (r < REGISTERS) ? r_gpr[r] : 0; }

    uint32_t            get_pc(void) { return r_pc_x; }
    uint32_t            get_opcode(void) { return get_opcode(r_pc_x); }
    int                 get_num_reg(void) { return REGISTERS; }

    void                set_register(int r, uint32_t val) { r_gpr[r] = val; }

    void                enable_trace(uint32_t mask)  { Trace = mask; }
    void                EnableInstStats(void) { StatsInstStatsEnable = true; }

    void                SetStatsInterface(StatsInterface *stats) { StatsIf = stats; }

    void                ResetStats(void);
    void                DumpStats(void);

protected:  
    void                Execute(void);
    uint32_t            Load(uint32_t pc, uint32_t address, int width, bool signedLoad);
    void                Store(uint32_t pc, uint32_t address, uint32_t data, int width);
    uint32_t            AccessCsr(uint32_t address, uint32_t data, bool set, bool clr);
    void                Exception(uint32_t cause, uint32_t pc);
    
private:

    // CPU Registers
    uint32_t            r_gpr[REGISTERS];
    uint32_t            r_pc;
    uint32_t            r_pc_x;
    uint32_t            csr_epc;
    uint32_t            csr_cause;
    uint32_t            csr_sr;
    uint32_t            csr_evec;
    uint32_t            csr_ie;
    uint32_t            csr_ip;
    uint32_t            csr_time;
    uint32_t            csr_timecmp;

    // Memory
    Memory             *Mem[MAX_MEM_REGIONS];
    uint32_t            MemBase[MAX_MEM_REGIONS];
    uint32_t            MemSize[MAX_MEM_REGIONS];
    int                 MemRegions;


    // Status
    bool                Fault;
    bool                Break;
    int                 Trace;

    // Stats
    uint32_t            Stats[STATS_MAX];
    uint32_t            StatsInst[ENUM_INST_MAX];
    bool                StatsInstStatsEnable;
    StatsInterface     *StatsIf;
};

#endif
