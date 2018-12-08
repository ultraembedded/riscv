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
#include <vector>
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
#define LOG_MMU             (1 << 4)

#define MAX_MEM_REGIONS     16

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
class IStatsInterface
{
public:  
    virtual void reset(void) = 0;
    virtual void execute(uint32_t pc, uint32_t opcode) = 0;
    virtual void print(void) = 0;
};

//--------------------------------------------------------------------
// Abstract interface for conio
//--------------------------------------------------------------------
class IConsoleIO
{
public:  
    virtual int putchar(int ch) = 0;
    virtual int getchar(void) = 0;
};

//--------------------------------------------------------------------
// Riscv: RV32IM model
//--------------------------------------------------------------------
class Riscv: public cosim_cpu_api, public cosim_mem_api
{
public:
                        Riscv(uint32_t baseAddr = 0, uint32_t len = 0);
    virtual             ~Riscv();

    bool                create_memory(uint32_t addr, uint32_t size, uint8_t *mem = NULL);
    bool                attach_memory(Memory *memory, uint32_t baseAddr, uint32_t len);

    bool                valid_addr(uint32_t address);
    void                write(uint32_t address, uint8_t data);
    void                write32(uint32_t address, uint32_t data);
    uint8_t             read(uint32_t address);
    uint32_t            read32(uint32_t address);

    void                reset(uint32_t start_addr);
    uint32_t            get_opcode(uint32_t pc);
    void                step(void);

    void                set_interrupt(int irq);

    bool                get_fault(void)      { return m_fault; }
    bool                get_stopped(void)    { return m_break; }
    bool                get_reg_valid(int r) { return true; }
    uint32_t            get_register(int r);

    uint32_t            get_pc(void)      { return m_pc_x; }
    uint32_t            get_opcode(void)  { return get_opcode(m_pc_x); }
    int                 get_num_reg(void) { return REGISTERS; }

    void                set_register(int r, uint32_t val);
    void                set_pc(uint32_t val);

    // Breakpoints
    bool                get_break(void);
    bool                set_breakpoint(uint32_t pc);
    bool                clr_breakpoint(uint32_t pc);
    bool                check_breakpoint(uint32_t pc);

    void                enable_trace(uint32_t mask)                 { m_trace = mask; }

    void                set_stats_interface(IStatsInterface *stats) { m_stats_if = stats; }
    void                set_console(IConsoleIO *cio)                { m_console = cio; }

    void                stats_reset(void);
    void                stats_dump(void);

    bool                error(bool terminal, const char *fmt, ...);

protected:  
    void                execute(void);
    int                 load(uint32_t pc, uint32_t address, uint32_t *result, int width, bool signedLoad);
    int                 store(uint32_t pc, uint32_t address, uint32_t data, int width);
    uint32_t            access_csr(uint32_t address, uint32_t data, bool set, bool clr);
    void                exception(uint32_t cause, uint32_t pc, uint32_t badaddr = 0);

// MMU
private:
#ifdef CONFIG_MMU
    int                 mmu_read_word(uint32_t address, uint32_t *val);
    uint32_t            mmu_walk(uint32_t addr);
    int                 mmu_i_translate(uint32_t addr, uint32_t *physical);
    int                 mmu_d_translate(uint32_t pc, uint32_t addr, uint32_t *physical, int writeNotRead);
#endif

private:

    // CPU Registers
    uint32_t            m_gpr[REGISTERS];
    uint32_t            m_pc;
    uint32_t            m_pc_x;

    // CSR - Machine
    uint32_t            m_csr_mepc;
    uint32_t            m_csr_mcause;
    uint32_t            m_csr_msr;
    uint32_t            m_csr_mpriv;
    uint32_t            m_csr_mevec;
    uint32_t            m_csr_mie;
    uint32_t            m_csr_mip;
    uint64_t            m_csr_mtime;
    uint64_t            m_csr_mtimecmp;
    uint32_t            m_csr_mscratch;
    uint32_t            m_csr_mideleg;
    uint32_t            m_csr_medeleg;

    // CSR - Supervisor
    uint32_t            m_csr_sepc;
    uint32_t            m_csr_sevec;
    uint32_t            m_csr_scause;
    uint32_t            m_csr_stval;
    uint32_t            m_csr_satp;
    uint32_t            m_csr_sscratch;

    // Memory
    Memory             *m_mem[MAX_MEM_REGIONS];
    uint32_t            m_mem_base[MAX_MEM_REGIONS];
    uint32_t            m_mem_size[MAX_MEM_REGIONS];
    int                 m_mem_regions;

    // Status
    bool                m_fault;
    bool                m_break;
    int                 m_trace;

    // Breakpoints
    bool                m_has_breakpoints;
    std::vector <uint32_t > m_breakpoints;

    // Stats
    uint32_t            m_stats[STATS_MAX];
    IStatsInterface     *m_stats_if;

    // Console
    IConsoleIO         *m_console;
};

#endif
