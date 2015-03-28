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
#include "memory.h"

//--------------------------------------------------------------------
// Defines:
//--------------------------------------------------------------------
#define LOG_INST            (1 << 0)
#define LOG_OPCODES         (1 << 1)
#define LOG_REGISTERS       (1 << 2)
#define LOG_MEM             (1 << 3)

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
// Class
//--------------------------------------------------------------------
class Riscv
{
public:
                        Riscv(uint32_t baseAddr, uint32_t len);
    virtual             ~Riscv();

    bool                CreateMemory(uint32_t baseAddr, uint32_t len);
    bool                AttachMemory(Memory *memory, uint32_t baseAddr, uint32_t len);

    bool                LoadMem(uint32_t startAddr, uint8_t *data, int len);

    void                Reset(uint32_t start_addr);
    bool                Step(void);

    void                Interrupt(void);

    bool                GetFault(void) { return Fault; }
    bool                GetBreak(void) { return Break; }
    uint32_t            GetRegister(int r) { return (r < REGISTERS) ? r_gpr[r] : 0; }
    uint32_t            GetPC(void) { return r_pc; }
    uint32_t            GetOpcode(uint32_t address);

    void                EnableTrace(uint32_t mask)  { Trace = mask; }

    void                ResetStats(void);
    void                DumpStats(void);

protected:  
    void                Execute(void);
    uint32_t            Load(uint32_t address, int width, bool signedLoad);
    void                Store(uint32_t address, uint32_t data, int width);
    uint32_t            AccessCsr(uint32_t address, uint32_t data, bool set, bool clr);
    void                Exception(uint32_t cause, uint32_t pc);
    
private:

    // CPU Registers
    uint32_t            r_gpr[REGISTERS];
    uint32_t            r_pc;
    uint32_t            csr_epc;
    uint32_t            csr_cause;
    uint32_t            csr_sr;
    uint32_t            csr_evec;

    // Memory
    Memory             *Mem[MAX_MEM_REGIONS];
    uint32_t            MemBase[MAX_MEM_REGIONS];
    uint32_t            MemSize[MAX_MEM_REGIONS];
    int                 MemRegions;

    // Status
    bool                Fault;
    bool                Break;
    int                 Trace;
    bool                InterruptPending;

    // Stats
    uint32_t            Stats[STATS_MAX];
    uint32_t            StatsInst[ENUM_INST_MAX];
};

#endif
