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
#include <stdarg.h>
#include <assert.h>
#include "riscv.h"

//-----------------------------------------------------------------
// Defines:
//-----------------------------------------------------------------
#define DPRINTF(l,a)        do { if (m_trace & l) printf a; } while (0)
#define TRACE_ENABLED(l)    (m_trace & l)
#define INST_STAT(l)

//-----------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------
Riscv::Riscv(uint32_t baseAddr /*= 0*/, uint32_t len /*= 0*/)
{
    m_mem_regions        = 0;
    m_stats_if           = NULL;
    m_console            = NULL;
    m_has_breakpoints    = false;

    // Some memory defined
    if (len != 0)
        create_memory(baseAddr, len);

    reset(baseAddr);
}
//-----------------------------------------------------------------
// Deconstructor
//-----------------------------------------------------------------
Riscv::~Riscv()
{
    int m;

    for (m=0;m<m_mem_regions;m++)
    {
        if (m_mem[m])
            delete m_mem[m];
        m_mem[m] = NULL;
    }
}
//-----------------------------------------------------------------
// error: Handle an error
//-----------------------------------------------------------------
bool Riscv::error(bool terminal, const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    exit(-1);

    return true;
}
//-----------------------------------------------------------------
// create_memory: Create a memory region
//-----------------------------------------------------------------
bool Riscv::create_memory(uint32_t baseAddr, uint32_t len, uint8_t *buf /*=NULL*/)
{
    if (buf)
        return attach_memory(new SimpleMemory(buf, len), baseAddr, len);
    else
        return attach_memory(new SimpleMemory(len), baseAddr, len);
}
//-----------------------------------------------------------------
// attach_memory: Attach a memory device to a particular region
//-----------------------------------------------------------------
bool Riscv::attach_memory(Memory *memory, uint32_t baseAddr, uint32_t len)
{
    if (m_mem_regions < MAX_MEM_REGIONS)
    {
        m_mem_base[m_mem_regions] = baseAddr;
        m_mem_size[m_mem_regions] = len;

        m_mem[m_mem_regions] = memory;
        m_mem[m_mem_regions]->reset();

        m_mem_regions++;

        return true;
    }

    return false;
}
//-----------------------------------------------------------------
// set_pc: Set PC
//-----------------------------------------------------------------
void Riscv::set_pc(uint32_t pc)
{
    m_pc        = pc;
    m_pc_x      = pc;
}
//-----------------------------------------------------------------
// set_register: Set register value
//-----------------------------------------------------------------
void Riscv::set_register(int r, uint32_t val)
{
    if (r <= RISCV_REGNO_GPR31)   m_gpr[r] = val;
    else if (r == RISCV_REGNO_PC) m_pc     = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MEPC)) m_csr_mepc = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MCAUSE)) m_csr_mcause = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MSTATUS)) m_csr_msr = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MTVEC)) m_csr_mevec = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MIE)) m_csr_mie = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MIP)) m_csr_mip = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MTIME)) m_csr_mtime = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MTIMEH)) m_csr_mtimecmp = val; // Non-std
    else if (r == (RISCV_REGNO_CSR0 + CSR_MSCRATCH)) m_csr_mscratch = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MIDELEG)) m_csr_mideleg = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MEDELEG)) m_csr_medeleg = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_SEPC)) m_csr_sepc = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_STVEC)) m_csr_sevec = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_SCAUSE)) m_csr_scause = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_STVAL)) m_csr_stval = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_SATP)) m_csr_satp = val;
    else if (r == (RISCV_REGNO_CSR0 + CSR_SSCRATCH)) m_csr_sscratch = val;
    else if (r == RISCV_REGNO_PRIV) m_csr_mpriv = val;
    
}
//-----------------------------------------------------------------
// get_register: Get register value
//-----------------------------------------------------------------
uint32_t Riscv::get_register(int r)
{
    if (r <= RISCV_REGNO_GPR31)   return m_gpr[r];
    else if (r == RISCV_REGNO_PC) return m_pc;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MEPC)) return m_csr_mepc;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MCAUSE)) return m_csr_mcause;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MSTATUS)) return m_csr_msr;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MTVEC)) return m_csr_mevec;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MIE)) return m_csr_mie;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MIP)) return m_csr_mip;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MTIME)) return m_csr_mtime;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MTIMEH)) return m_csr_mtimecmp; // Non-std
    else if (r == (RISCV_REGNO_CSR0 + CSR_MSCRATCH)) return m_csr_mscratch;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MIDELEG)) return m_csr_mideleg;
    else if (r == (RISCV_REGNO_CSR0 + CSR_MEDELEG)) return m_csr_medeleg;
    else if (r == (RISCV_REGNO_CSR0 + CSR_SEPC)) return m_csr_sepc;
    else if (r == (RISCV_REGNO_CSR0 + CSR_STVEC)) return m_csr_sevec;
    else if (r == (RISCV_REGNO_CSR0 + CSR_SCAUSE)) return m_csr_scause;
    else if (r == (RISCV_REGNO_CSR0 + CSR_STVAL)) return m_csr_stval;
    else if (r == (RISCV_REGNO_CSR0 + CSR_SATP)) return m_csr_satp;
    else if (r == (RISCV_REGNO_CSR0 + CSR_SSCRATCH)) return m_csr_sscratch;
    else if (r == RISCV_REGNO_PRIV) return m_csr_mpriv;

    return 0;
}
//-----------------------------------------------------------------
// get_break: Get breakpoint status (and clear)
//-----------------------------------------------------------------
bool Riscv::get_break(void)
{
    bool brk = m_break;
    m_break = false;
    return brk;
}
//-----------------------------------------------------------------
// set_breakpoint: Set breakpoint on a given PC
//-----------------------------------------------------------------
bool Riscv::set_breakpoint(uint32_t pc)
{
    m_breakpoints.push_back(pc);
    m_has_breakpoints = true;
    return true;
}
//-----------------------------------------------------------------
// set_breakpoint: Clear breakpoint on a given PC
//-----------------------------------------------------------------
bool Riscv::clr_breakpoint(uint32_t pc)
{
    for (std::vector<uint32_t>::iterator it = m_breakpoints.begin() ; it != m_breakpoints.end(); ++it)
        if ((*it) == pc)
        {
            m_breakpoints.erase(it);
            m_has_breakpoints = !m_breakpoints.empty();
            return true;
        }

    return false;
}
//-----------------------------------------------------------------
// check_breakpoint: Check if breakpoint has been hit
//-----------------------------------------------------------------
bool Riscv::check_breakpoint(uint32_t pc)
{
    for (std::vector<uint32_t>::iterator it = m_breakpoints.begin() ; it != m_breakpoints.end(); ++it)
        if ((*it) == pc)
            return true;

    return false;
}
//-----------------------------------------------------------------
// reset: Reset CPU state
//-----------------------------------------------------------------
void Riscv::reset(uint32_t start_addr)
{
    m_pc        = start_addr;
    m_pc_x      = start_addr;

    for (int i=0;i<REGISTERS;i++)
        m_gpr[i] = 0;

    m_csr_mpriv    = PRIV_MACHINE;
    m_csr_msr      = 0;
    m_csr_mideleg  = 0;
    m_csr_medeleg  = 0;

    m_csr_mepc     = 0;
    m_csr_mie      = 0;
    m_csr_mip      = 0;
    m_csr_mcause   = 0;
    m_csr_mevec    = 0;
    m_csr_mtime    = 0;
    m_csr_mtimecmp = 0;
    m_csr_mscratch = 0;

    m_csr_sepc     = 0;
    m_csr_sevec    = 0;
    m_csr_scause   = 0;
    m_csr_stval    = 0;
    m_csr_satp     = 0;
    m_csr_sscratch = 0;

    m_fault       = false;
    m_break       = false;
    m_trace       = 0;

    stats_reset();
}
//-----------------------------------------------------------------
// valid_addr: Check if the physical memory address is valid
//-----------------------------------------------------------------
bool Riscv::valid_addr(uint32_t address)
{
    for (int j=0;j<m_mem_regions;j++)
        if (address >= m_mem_base[j] && address < (m_mem_base[j] + m_mem_size[j]))
            return true;

    return false;
}
//-----------------------------------------------------------------
// write: Write a byte to memory (physical address)
//-----------------------------------------------------------------
void Riscv::write(uint32_t address, uint8_t data)
{
    for (int j=0;j<m_mem_regions;j++)
        if (address >= m_mem_base[j] && address < (m_mem_base[j] + m_mem_size[j]))
        {
            m_mem[j]->store(address - m_mem_base[j], data, 1);
            return ;
        }

    error(false, "Failed store @ 0x%08x\n", address);
}
//-----------------------------------------------------------------
// write32: Write a word to memory (physical address)
//-----------------------------------------------------------------
void Riscv::write32(uint32_t address, uint32_t data)
{
    for (int j=0;j<m_mem_regions;j++)
        if (address >= m_mem_base[j] && address < (m_mem_base[j] + m_mem_size[j]))
        {
            m_mem[j]->store(address - m_mem_base[j], data, 4);
            return ;
        }

    error(false, "Failed store @ 0x%08x\n", address);
}
//-----------------------------------------------------------------
// read: Read a byte from memory (physical address)
//-----------------------------------------------------------------
uint8_t Riscv::read(uint32_t address)
{
    for (int j=0;j<m_mem_regions;j++)
        if (address >= m_mem_base[j] && address < (m_mem_base[j] + m_mem_size[j]))
            return m_mem[j]->load(address - m_mem_base[j], 1, false);

    return 0;
}
//-----------------------------------------------------------------
// read32: Read a word from memory (physical address)
//-----------------------------------------------------------------
uint32_t Riscv::read32(uint32_t address)
{
    for (int j=0;j<m_mem_regions;j++)
        if (address >= m_mem_base[j] && address < (m_mem_base[j] + m_mem_size[j]))
            return m_mem[j]->load(address - m_mem_base[j], 4, false);

    return 0;
}
//-----------------------------------------------------------------
// get_opcode: Get instruction from address
//-----------------------------------------------------------------
uint32_t Riscv::get_opcode(uint32_t address)
{
    return read32(address);
}
#ifdef CONFIG_MMU
//-----------------------------------------------------------------
// mmu_read_word: Read a word from memory
//-----------------------------------------------------------------
int Riscv::mmu_read_word(uint32_t address, uint32_t *val)
{
    int m;
    *val = 0;

    for (m=0;m<m_mem_regions;m++)
        if (address >= m_mem_base[m] && address < (m_mem_base[m] + m_mem_size[m]))
        {
            *val = m_mem[m]->load(address - m_mem_base[m], 4, false);
            return 1;
        }

    return 0;
}
//-----------------------------------------------------------------
// mmu_walk: Page table walker
//-----------------------------------------------------------------
uint32_t Riscv::mmu_walk(uint32_t addr)
{
    int shift = 32 - MMU_VA_BITS;
    uint32_t pte = 0;

    DPRINTF(LOG_MMU, ("MMU: Walk %x\n", addr));

    // the address must be a canonical sign-extended VA_BITS-bit number    
    if (((int32_t)addr << shift >> shift) != (int32_t)addr)
    {
        pte = 0; // Bad alignment....

        error(false, "%08x: PTE Walk - Bad alignment %x\n", m_pc, addr);
    }
    else if ((m_csr_satp & SATP_MODE) == 0) // Bare mode
    {
        pte = PAGE_PRESENT | PAGE_READ | PAGE_WRITE | PAGE_EXEC | ((addr >> MMU_PGSHIFT) << MMU_PGSHIFT);
        DPRINTF(LOG_MMU, ("MMU: MMU not enabled\n"));
    }
    else
    {
        uint32_t base = ((m_csr_satp >> SATP_PPN_SHIFT) & SATP_PPN_MASK) * PAGE_SIZE;
        uint32_t asid = ((m_csr_satp >> SATP_ASID_SHIFT) & SATP_ASID_MASK);

        DPRINTF(LOG_MMU, ("MMU: MMU enabled - base 0x%08x\n", base));

        uint32_t i;
        for (i=MMU_LEVELS-1; i >= 0; i--)
        {
            int      ptshift  = i * MMU_PTIDXBITS;
            uint32_t idx      = (addr >> (MMU_PGSHIFT+ptshift)) & ((1<<MMU_PTIDXBITS)-1);
            uint32_t pte_addr = base + (idx * MMU_PTESIZE);

            // Read PTE
            if (!mmu_read_word(pte_addr, &pte))
            {
                DPRINTF(LOG_MMU, ("MMU: Cannot read PTE entry %x\n", pte_addr));
                pte = 0;
                break;
            }

            DPRINTF(LOG_MMU, ("MMU: PTE value = 0x%08x @ 0x%08x\n", pte, pte_addr));

            uint32_t ppn = pte >> PAGE_PFN_SHIFT;

            // Invalid mapping
            if (!(pte & PAGE_PRESENT))
            {
                DPRINTF(LOG_MMU, ("MMU: Invalid mapping %x\n", pte_addr));
                pte = 0;
                break;
            }
            // Next level of page table
            else if (!(pte & (PAGE_READ | PAGE_WRITE | PAGE_EXEC)))
            {
                base = ppn << MMU_PGSHIFT;
                DPRINTF(LOG_MMU, ("MMU: Next level of page table %x\n", base));
            }
            // The actual PTE
            else
            {
                // Keep permission bits
                pte &= PAGE_FLAGS;

                // if this PTE is from a larger PT, fake a leaf
                // PTE so the TLB will work right
                uint64_t vpn   = addr >> MMU_PGSHIFT;
                uint64_t value = (ppn | (vpn & ((((uint64_t)1) << ptshift) - 1))) << MMU_PGSHIFT;

                // Add back in permission bits
                value |= pte;

                assert((value >> 32) == 0);
                pte   = value;

                uint32_t ptd_addr = ((pte >> MMU_PGSHIFT) << MMU_PGSHIFT);

                DPRINTF(LOG_MMU, ("MMU: PTE addr %x (%x)\n", ptd_addr, pte));

                // fault if physical addr is out of range
                if (mmu_read_word(ptd_addr, &ptd_addr))
                {
                    DPRINTF(LOG_MMU, ("MMU: PTE entry found %x\n", pte));
                }
                else
                {
                    DPRINTF(LOG_MMU, ("MMU: PTE access out of range %x\n", ((pte >> MMU_PGSHIFT) << MMU_PGSHIFT)));
                    pte = 0;
                    error(false, "%08x: PTE access out of range %x\n", m_pc, addr);
                }

                break;
            }
        }
    }

    return pte;
}
//-----------------------------------------------------------------
// mmu_i_translate: Translate instruction fetch
//-----------------------------------------------------------------
int Riscv::mmu_i_translate(uint32_t addr, uint32_t *physical)
{
    bool page_fault = false;

    // Machine - no MMU
    if (m_csr_mpriv > PRIV_SUPER)
    {
        *physical = addr;
        return 1; 
    }
    
    uint32_t pte = mmu_walk(addr);

    // Reserved configurations
    if (((pte & (PAGE_EXEC | PAGE_READ | PAGE_WRITE)) == PAGE_WRITE) ||
        ((pte & (PAGE_EXEC | PAGE_READ | PAGE_WRITE)) == (PAGE_EXEC | PAGE_WRITE)))
    {
        page_fault = true;
    }
    // Supervisor mode
    else if (m_csr_mpriv == PRIV_SUPER)
    {
        // Supervisor attempts to execute user mode page
        if (pte & PAGE_USER)
        {
            error(false, "IMMU: Attempt to execute user page 0x%08x\n", addr);
            page_fault = true;
        }
        // Page not executable
        else if ((pte & (PAGE_EXEC)) != (PAGE_EXEC))
        {
            page_fault = true;
        }
    }
    // User mode
    else
    {
        // User mode page not executable
        if ((pte & (PAGE_EXEC | PAGE_USER)) != (PAGE_EXEC | PAGE_USER))
        {
            page_fault = true;
        }
    }

    if (page_fault)
    {
        *physical      = 0xFFFFFFFF;
        exception(MCAUSE_PAGE_FAULT_INST, addr, addr);
        return 0;
    }

    uint32_t pgoff  = addr & (MMU_PGSIZE-1);
    uint32_t pgbase = pte >> MMU_PGSHIFT << MMU_PGSHIFT;
    uint32_t paddr  = pgbase + pgoff;

    DPRINTF(LOG_MMU, ("IMMU: Lookup VA %x -> PA %x\n", addr, paddr));

    *physical = paddr;
    return 1; 
}
//-----------------------------------------------------------------
// mmu_d_translate: Translate load store
//-----------------------------------------------------------------
int Riscv::mmu_d_translate(uint32_t pc, uint32_t addr, uint32_t *physical, int writeNotRead)
{
    bool page_fault = false;

    // Machine - no MMU
    if (m_csr_mpriv > PRIV_SUPER)
    {
        *physical = addr;
        return 1; 
    }

    uint32_t pte = mmu_walk(addr);

    // Reserved configurations
    if (((pte & (PAGE_EXEC | PAGE_READ | PAGE_WRITE)) == PAGE_WRITE) ||
        ((pte & (PAGE_EXEC | PAGE_READ | PAGE_WRITE)) == (PAGE_EXEC | PAGE_WRITE)))
    {
        page_fault = true;
    }
    // Supervisor mode
    else if (m_csr_mpriv == PRIV_SUPER)
    {
        // User page access - super mode access not enabled
        if ((pte & PAGE_USER) && !(m_csr_msr & SR_SUM))
        {
            error(false, "MMU_D: PC=%08x Access %08x - User page access by super\n", pc, addr);
        }
        else if ((writeNotRead  && ((pte & (PAGE_WRITE)) != (PAGE_WRITE))) || 
                 (!writeNotRead && ((pte & (PAGE_READ))  != (PAGE_READ))))
        {
            page_fault = true;
        }
    }
    // User mode
    else
    {
        if ((writeNotRead  && ((pte & (PAGE_WRITE | PAGE_USER)) != (PAGE_WRITE | PAGE_USER))) || 
            (!writeNotRead && ((pte & (PAGE_READ | PAGE_USER))  != (PAGE_READ | PAGE_USER))))
        {
            page_fault = true;
        }
    }

    if (page_fault)
    {
        *physical      = 0xFFFFFFFF;
        exception(writeNotRead ? MCAUSE_PAGE_FAULT_STORE : MCAUSE_PAGE_FAULT_LOAD, pc, addr);
        return 0;
    }

    uint32_t pgoff  = addr & (MMU_PGSIZE-1);
    uint32_t pgbase = pte >> MMU_PGSHIFT << MMU_PGSHIFT;
    uint32_t paddr  = pgbase + pgoff;

    DPRINTF(LOG_MMU, ("DMMU: Lookup VA %x -> PA %x\n", addr, paddr));

    *physical = paddr;
    return 1; 
}
#endif
//-----------------------------------------------------------------
// load: Perform a load operation (with optional MMU lookup)
//-----------------------------------------------------------------
int Riscv::load(uint32_t pc, uint32_t address, uint32_t *result, int width, bool signedLoad)
{
    uint32_t physical = address;

#ifdef CONFIG_MMU
    // Translate addresses if required
    if (!mmu_d_translate(pc, address, &physical, 0))
        return 0;
#endif

    DPRINTF(LOG_MEM, ("LOAD: VA 0x%08x PA 0x%08x Width %d\n", address, physical, width));

    event_push(COSIM_EVENT_LOAD, physical & ~3, 0);

    m_stats[STATS_LOADS]++;

    for (int j=0;j<m_mem_regions;j++)
        if (physical >= m_mem_base[j] && physical < (m_mem_base[j] + m_mem_size[j]))
        {
            *result = m_mem[j]->load(physical - m_mem_base[j], width, signedLoad);

            DPRINTF(LOG_MEM, ("LOAD_RESULT: 0x%08x\n",*result));
            event_push(COSIM_EVENT_LOAD_RESULT, *result, 0);
            return 1;
        }

    error(false, "%08x: Bad memory access 0x%x\n", pc, address);
    return 0;
}
//-----------------------------------------------------------------
// store: Perform a store operation (with optional MMU lookup)
//-----------------------------------------------------------------
int Riscv::store(uint32_t pc, uint32_t address, uint32_t data, int width)
{
    uint32_t physical = address;

#ifdef CONFIG_MMU
    // Translate addresses if required
    if (!mmu_d_translate(pc, address, &physical, 1))
        return 0;
#endif

    DPRINTF(LOG_MEM, ("STORE: VA 0x%08x PA 0x%08x Value 0x%08x Width %d\n", address, physical, data, width));

    m_stats[STATS_STORES]++;

    if (width == 1)
        event_push(COSIM_EVENT_STORE, physical & ~3, data & 0xFF);
    else if (width == 2)
        event_push(COSIM_EVENT_STORE, physical & ~3, data & 0xFFFF);
    else
        event_push(COSIM_EVENT_STORE, physical, data);

    for (int j=0;j<m_mem_regions;j++)
        if (physical >= m_mem_base[j] && physical < (m_mem_base[j] + m_mem_size[j]))
        {
            m_mem[j]->store(physical - m_mem_base[j], data, width);
            return 1;
        }

    error(false, "%08x: Bad memory access 0x%x\n", pc, address);
    return 0;
}
//-----------------------------------------------------------------
// access_csr: Perform CSR access
//-----------------------------------------------------------------
uint32_t Riscv::access_csr(uint32_t address, uint32_t data, bool set, bool clr)
{
    uint32_t result = 0;

#define CSR_STD(name, var_name) \
    case CSR_ ##name: \
    { \
        data       &= CSR_ ##name## _MASK; \
        result      = var_name & CSR_ ##name## _MASK; \
        if (set && clr) \
            var_name  = data; \
        else if (set) \
            var_name |= data; \
        else if (clr) \
            var_name &= ~data; \
    } \
    break;

#define CSR_CONST(name, value) \
    case CSR_ ##name: \
    { \
        result      = value; \
    } \
    break;


    switch (address & 0xFFF)
    {
        //--------------------------------------------------------
        // Simulation control
        //--------------------------------------------------------
        case CSR_DSCRATCH:
            switch (data & 0xFF000000)
            {
                case CSR_SIM_CTRL_EXIT:
                    stats_dump();
                    exit(data & 0xFF);
                    break;
                case CSR_SIM_CTRL_PUTC:
                    if (m_console)
                        m_console->putchar(data & 0xFF);
                    else
                        fprintf(stderr, "%c", (data & 0xFF));
                    break;
                case CSR_SIM_CTRL_GETC:
                    if (m_console)
                        result = m_console->getchar();
                    else
                        result = 0;
                    break;
                case CSR_SIM_CTRL_TRACE:
                    enable_trace(data & 0xFF);
                    break;
                case CSR_SIM_PRINTF:
                {
                    uint32_t fmt_addr = m_gpr[10];
                    uint32_t arg1     = m_gpr[11];
                    uint32_t arg2     = m_gpr[12];
                    uint32_t arg3     = m_gpr[13];
                    uint32_t arg4     = m_gpr[14];

#ifdef CONFIG_MMU
                    {
                        uint32_t pte = mmu_walk(fmt_addr);
                        uint32_t pgoff = fmt_addr & (MMU_PGSIZE-1);
                        uint32_t pgbase = pte >> MMU_PGSHIFT << MMU_PGSHIFT;
                        if (pte != 0) fmt_addr = pgbase + pgoff;
                    }
#endif
                    char fmt_str[1024];
                    int idx = 0;
                    while (idx < (sizeof(fmt_str)-1))
                    {
                        fmt_str[idx] = read(fmt_addr++);
                        if (fmt_str[idx] == 0)
                            break;
                        idx += 1;
                    }

                    char out_str[1024];
                    sprintf(out_str, fmt_str, arg1, arg2, arg3, arg4);
                    printf("%s",out_str);
                }
                break;
            }
         break;
        //--------------------------------------------------------
        // Standard - Machine
        //--------------------------------------------------------
        CSR_STD(MEPC,    m_csr_mepc)
        CSR_STD(MTVEC,   m_csr_mevec)
        CSR_STD(MCAUSE,  m_csr_mcause)
        CSR_STD(MSTATUS, m_csr_msr)
        CSR_STD(MIP,     m_csr_mip)
        CSR_STD(MIE,     m_csr_mie)
        CSR_CONST(MISA,  MISA_VALUE)
        CSR_STD(MIDELEG, m_csr_mideleg)
        CSR_STD(MEDELEG, m_csr_medeleg)
        CSR_STD(MSCRATCH,m_csr_mscratch)
        CSR_CONST(MHARTID,  MHARTID_VALUE)
        //--------------------------------------------------------
        // Standard - Supervisor
        //--------------------------------------------------------
        CSR_STD(SEPC,    m_csr_sepc)
        CSR_STD(STVEC,   m_csr_sevec)
        CSR_STD(SCAUSE,  m_csr_scause)
        CSR_STD(SIP,     m_csr_mip)
        CSR_STD(SIE,     m_csr_mie)
        CSR_STD(SATP,    m_csr_satp)
        CSR_STD(STVAL,   m_csr_stval)
        CSR_STD(SSCRATCH,m_csr_sscratch)
        CSR_STD(SSTATUS, m_csr_msr)
        //--------------------------------------------------------
        // Extensions
        //-------------------------------------------------------- 
        CSR_CONST(PMPCFG0, 0)
        CSR_CONST(PMPCFG1, 0)
        CSR_CONST(PMPCFG2, 0)
        CSR_CONST(PMPADDR0, 0)
        case CSR_MTIME:
            data       &= CSR_MTIME_MASK;
            result      = m_csr_mtime;

            // Non-std behaviour - write to CSR_TIME gives next interrupt threshold            
            if (set && data != 0)
            {
                m_csr_mtimecmp = data;

                // Clear interrupt pending
                m_csr_mip &= ~((m_csr_mideleg & SR_IP_STIP) ? SR_IP_STIP : SR_IP_MTIP);
            }
            break;
    
        case CSR_MTIMEH:
            result      = m_csr_mtime >> 32;
            break;
        default:
            error(false, "*** CSR address not supported %08x [PC=%08x]\n", address, m_pc);
            break;
    }
    return result;
}
//-----------------------------------------------------------------
// exception: Handle an exception or interrupt
//-----------------------------------------------------------------
void Riscv::exception(uint32_t cause, uint32_t pc, uint32_t badaddr /*= 0*/)
{
    uint32_t deleg;
    uint32_t bit;

    // Interrupt
    if (cause >= MCAUSE_INTERRUPT)
    {
        deleg = m_csr_mideleg;
        bit   = 1 << (cause - MCAUSE_INTERRUPT);
    }
    // Exception
    else
    {
        deleg = m_csr_medeleg;
        bit   = 1 << cause;
    }

    // Exception delegated to supervisor mode
    if (m_csr_mpriv <= PRIV_SUPER && (deleg & bit))
    {
        uint32_t s = m_csr_msr;

        // Interrupt save and disable
        s &= ~SR_SPIE;
        s |= (s & SR_SIE) ? SR_SPIE : 0;
        s &= ~SR_SIE;

        // Record previous priviledge level
        s &= ~SR_SPP;
        s |= (m_csr_mpriv == PRIV_SUPER) ? SR_SPP : 0;

        // Raise priviledge to supervisor level
        m_csr_mpriv  = PRIV_SUPER;

        m_csr_msr    = s;
        m_csr_sepc   = pc;
        m_csr_scause = cause;
        m_csr_stval  = badaddr;

        // Set new PC
        m_pc         = m_csr_sevec;
    }
    // Machine mode
    else
    {
        uint32_t s = m_csr_msr;

        // Interrupt save and disable
        s &= ~SR_MPIE;
        s |= (s & SR_MIE) ? SR_MPIE : 0;
        s &= ~SR_MIE;

        // Record previous priviledge level
        s &= ~SR_MPP;
        s |= (m_csr_mpriv << SR_MPP_SHIFT);

        // Raise priviledge to machine level
        m_csr_mpriv  = PRIV_MACHINE;

        m_csr_msr    = s;
        m_csr_mepc   = pc;
        m_csr_mcause = cause;

        // Set new PC
        m_pc         = m_csr_mevec; // TODO: This should be a product of the except num
    }
}
//-----------------------------------------------------------------
// execute: Instruction execution stage
//-----------------------------------------------------------------
void Riscv::execute(void)
{
    uint32_t phy_pc = m_pc;

#ifdef CONFIG_MMU
    // Translate PC to physical address
    if (!mmu_i_translate(m_pc, &phy_pc))
        return ;
#endif

    // Get opcode at current PC
    uint32_t opcode = get_opcode(phy_pc);
    m_pc_x = m_pc;

    // Extract registers
    int rd          = (opcode & OPCODE_RD_MASK)  >> OPCODE_RD_SHIFT;
    int rs1         = (opcode & OPCODE_RS1_MASK) >> OPCODE_RS1_SHIFT;
    int rs2         = (opcode & OPCODE_RS2_MASK) >> OPCODE_RS2_SHIFT;

    // Extract immediates
    int typei_imm   = ((signed)(opcode & OPCODE_TYPEI_IMM_MASK)) >> OPCODE_TYPEI_IMM_SHIFT;
    int typeu_imm   = ((signed)(opcode & OPCODE_TYPEU_IMM_MASK)) >> OPCODE_TYPEU_IMM_SHIFT;
    int imm20       = typeu_imm << OPCODE_TYPEU_IMM_SHIFT;
    int imm12       = typei_imm;
    int bimm        = OPCODE_SBTYPE_IMM(opcode);
    int jimm20      = OPCODE_UJTYPE_IMM(opcode);
    int storeimm    = OPCODE_STYPE_IMM(opcode);
    int shamt       = ((signed)(opcode & OPCODE_SHAMT_MASK)) >> OPCODE_SHAMT_SHIFT;

    // Retrieve registers
    uint32_t reg_rd  = 0;
    uint32_t reg_rs1 = m_gpr[rs1];
    uint32_t reg_rs2 = m_gpr[rs2];
    uint32_t pc      = m_pc;

    bool take_exception = false;

    DPRINTF(LOG_OPCODES,( "%08x: %08x\n", pc, opcode));
    DPRINTF(LOG_OPCODES,( "        rd(%d) r%d = %d, r%d = %d\n", rd, rs1, reg_rs1, rs2, reg_rs2));

    // As RVC is not supported, fault on opcode which is all zeros
    if (opcode == 0)
    {
        error(false, "Bad instruction @ %x\n", pc);

        exception(MCAUSE_ILLEGAL_INSTRUCTION, pc);
        m_fault = true;
        take_exception = true;        
    }
    else if ((opcode & INST_ANDI_MASK) == INST_ANDI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: andi r%d, r%d, %d\n", pc, rd, rs1, imm12));
        INST_STAT(ENUM_INST_ANDI);
        reg_rd = reg_rs1 & imm12;
        pc += 4;        
    }
    else if ((opcode & INST_ORI_MASK) == INST_ORI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: ori r%d, r%d, %d\n", pc, rd, rs1, imm12));
        INST_STAT(ENUM_INST_ORI);
        reg_rd = reg_rs1 | imm12;
        pc += 4;        
    }
    else if ((opcode & INST_XORI_MASK) == INST_XORI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: xori r%d, r%d, %d\n", pc, rd, rs1, imm12));
        INST_STAT(ENUM_INST_XORI);
        reg_rd = reg_rs1 ^ imm12;
        pc += 4;        
    }
    else if ((opcode & INST_ADDI_MASK) == INST_ADDI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: addi r%d, r%d, %d\n", pc, rd, rs1, imm12));
        INST_STAT(ENUM_INST_ADDI);
        reg_rd = reg_rs1 + imm12;
        pc += 4;
    }
    else if ((opcode & INST_SLTI_MASK) == INST_SLTI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: slti r%d, r%d, %d\n", pc, rd, rs1, imm12));
        INST_STAT(ENUM_INST_SLTI);
        reg_rd = (signed)reg_rs1 < (signed)imm12;
        pc += 4;        
    }
    else if ((opcode & INST_SLTIU_MASK) == INST_SLTIU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: sltiu r%d, r%d, %d\n", pc, rd, rs1, (unsigned)imm12));
        INST_STAT(ENUM_INST_SLTIU);
        reg_rd = (unsigned)reg_rs1 < (unsigned)imm12;
        pc += 4;        
    }
    else if ((opcode & INST_SLLI_MASK) == INST_SLLI)
    {
        // ['rd', 'rs1']
        DPRINTF(LOG_INST,("%08x: slli r%d, r%d, %d\n", pc, rd, rs1, shamt));
        INST_STAT(ENUM_INST_SLLI);
        reg_rd = reg_rs1 << shamt;
        pc += 4;        
    }
    else if ((opcode & INST_SRLI_MASK) == INST_SRLI)
    {
        // ['rd', 'rs1', 'shamt']
        DPRINTF(LOG_INST,("%08x: srli r%d, r%d, %d\n", pc, rd, rs1, shamt));
        INST_STAT(ENUM_INST_SRLI);
        reg_rd = (unsigned)reg_rs1 >> shamt;
        pc += 4;        
    }
    else if ((opcode & INST_SRAI_MASK) == INST_SRAI)
    {
        // ['rd', 'rs1', 'shamt']
        DPRINTF(LOG_INST,("%08x: srai r%d, r%d, %d\n", pc, rd, rs1, shamt));
        INST_STAT(ENUM_INST_SRAI);
        reg_rd = (signed)reg_rs1 >> shamt;
        pc += 4;        
    }
    else if ((opcode & INST_LUI_MASK) == INST_LUI)
    {
        // ['rd', 'imm20']
        DPRINTF(LOG_INST,("%08x: lui r%d, 0x%x\n", pc, rd, imm20));
        INST_STAT(ENUM_INST_LUI);
        reg_rd = imm20;
        pc += 4;        
    }
    else if ((opcode & INST_AUIPC_MASK) == INST_AUIPC)
    {
        // ['rd', 'imm20']
        DPRINTF(LOG_INST,("%08x: auipc r%d, 0x%x\n", pc, rd, imm20));
        INST_STAT(ENUM_INST_AUIPC);
        reg_rd = imm20 + pc;
        pc += 4;        
    }
    else if ((opcode & INST_ADD_MASK) == INST_ADD)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: add r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        INST_STAT(ENUM_INST_ADD);
        reg_rd = reg_rs1 + reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SUB_MASK) == INST_SUB)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sub r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        INST_STAT(ENUM_INST_SUB);
        reg_rd = reg_rs1 - reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SLT_MASK) == INST_SLT)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: slt r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        INST_STAT(ENUM_INST_SLT);
        reg_rd = (signed)reg_rs1 < (signed)reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SLTU_MASK) == INST_SLTU)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sltu r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        INST_STAT(ENUM_INST_SLTU);
        reg_rd = (unsigned)reg_rs1 < (unsigned)reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_XOR_MASK) == INST_XOR)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: xor r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        INST_STAT(ENUM_INST_XOR);
        reg_rd = reg_rs1 ^ reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_OR_MASK) == INST_OR)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: or r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        INST_STAT(ENUM_INST_OR);
        reg_rd = reg_rs1 | reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_AND_MASK) == INST_AND)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: and r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        INST_STAT(ENUM_INST_AND);
        reg_rd = reg_rs1 & reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SLL_MASK) == INST_SLL)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sll r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        INST_STAT(ENUM_INST_SLL);
        reg_rd = reg_rs1 << reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SRL_MASK) == INST_SRL)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: srl r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        INST_STAT(ENUM_INST_SRL);
        reg_rd = (unsigned)reg_rs1 >> reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SRA_MASK) == INST_SRA)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sra r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        INST_STAT(ENUM_INST_SRA);
        reg_rd = (signed)reg_rs1 >> reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_JAL_MASK) == INST_JAL)
    {
        // ['rd', 'jimm20']
        DPRINTF(LOG_INST,("%08x: jal r%d, %d\n", pc, rd, jimm20));
        INST_STAT(ENUM_INST_JAL);
        reg_rd = pc + 4;
        pc+= jimm20;

        m_stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_JALR_MASK) == INST_JALR)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: jalr r%d, r%d\n", pc, rs1, imm12));
        INST_STAT(ENUM_INST_JALR);
        reg_rd = pc + 4;
        pc = (reg_rs1 + imm12) & ~1;

        m_stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_BEQ_MASK) == INST_BEQ)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: beq r%d, r%d, %d\n", pc, rs1, rs2, bimm));
        INST_STAT(ENUM_INST_BEQ);
        if (reg_rs1 == reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;

        m_stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_BNE_MASK) == INST_BNE)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: bne r%d, r%d, %d\n", pc, rs1, rs2, bimm));
        INST_STAT(ENUM_INST_BNE);
        if (reg_rs1 != reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;

        m_stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_BLT_MASK) == INST_BLT)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: blt r%d, r%d, %d\n", pc, rs1, rs2, bimm));
        INST_STAT(ENUM_INST_BLT);
        if ((signed)reg_rs1 < (signed)reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;

        m_stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_BGE_MASK) == INST_BGE)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: bge r%d, r%d, %d\n", pc, rs1, rs2, bimm));
        INST_STAT(ENUM_INST_BGE);
        if ((signed)reg_rs1 >= (signed)reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;

        m_stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_BLTU_MASK) == INST_BLTU)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: bltu r%d, r%d, %d\n", pc, rs1, rs2, bimm));
        INST_STAT(ENUM_INST_BLTU);
        if ((unsigned)reg_rs1 < (unsigned)reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;

        m_stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_BGEU_MASK) == INST_BGEU)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: bgeu r%d, r%d, %d\n", pc, rs1, rs2, bimm));
        INST_STAT(ENUM_INST_BGEU);
        if ((unsigned)reg_rs1 >= (unsigned)reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;

        m_stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_LB_MASK) == INST_LB)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lb r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        INST_STAT(ENUM_INST_LB);
        if (load(pc, reg_rs1 + imm12, &reg_rd, 1, true))
            pc += 4;
        else
            return;
    }
    else if ((opcode & INST_LH_MASK) == INST_LH)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lh r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        INST_STAT(ENUM_INST_LH);
        if (load(pc, reg_rs1 + imm12, &reg_rd, 2, true))
            pc += 4;
        else
            return;
    }
    else if ((opcode & INST_LW_MASK) == INST_LW)
    {
        // ['rd', 'rs1', 'imm12']        
        INST_STAT(ENUM_INST_LW);
        DPRINTF(LOG_INST,("%08x: lw r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        if (load(pc, reg_rs1 + imm12, &reg_rd, 4, true))
            pc += 4;
        else
            return;
    }
    else if ((opcode & INST_LBU_MASK) == INST_LBU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lbu r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        INST_STAT(ENUM_INST_LBU);
        if (load(pc, reg_rs1 + imm12, &reg_rd, 1, false))
            pc += 4;
        else
            return;
    }
    else if ((opcode & INST_LHU_MASK) == INST_LHU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lhu r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        INST_STAT(ENUM_INST_LHU);
        if (load(pc, reg_rs1 + imm12, &reg_rd, 2, false))
            pc += 4;
        else
            return;
    }
    else if ((opcode & INST_LWU_MASK) == INST_LWU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lwu r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        INST_STAT(ENUM_INST_LWU);
        if (load(pc, reg_rs1 + imm12, &reg_rd, 4, false))
            pc += 4;
        else
            return;
    }
    else if ((opcode & INST_SB_MASK) == INST_SB)
    {
        // ['imm12hi', 'rs1', 'rs2', 'imm12lo']
        DPRINTF(LOG_INST,("%08x: sb %d(r%d), r%d\n", pc, storeimm, rs1, rs2));
        INST_STAT(ENUM_INST_SB);
        if (store(pc, reg_rs1 + storeimm, reg_rs2, 1))
            pc += 4;
        else
            return ;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_SH_MASK) == INST_SH)
    {
        // ['imm12hi', 'rs1', 'rs2', 'imm12lo']
        DPRINTF(LOG_INST,("%08x: sh %d(r%d), r%d\n", pc, storeimm, rs1, rs2));
        INST_STAT(ENUM_INST_SH);
        if (store(pc, reg_rs1 + storeimm, reg_rs2, 2))
            pc += 4;
        else
            return ;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_SW_MASK) == INST_SW)
    {
        // ['imm12hi', 'rs1', 'rs2', 'imm12lo']
        DPRINTF(LOG_INST,("%08x: sw %d(r%d), r%d\n", pc, storeimm, rs1, rs2));
        INST_STAT(ENUM_INST_SW);
        if (store(pc, reg_rs1 + storeimm, reg_rs2, 4))
            pc += 4;
        else
            return ;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_MUL_MASK) == INST_MUL)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: mul r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        INST_STAT(ENUM_INST_MUL);
        reg_rd = (signed)reg_rs1 * (signed)reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_MULH_MASK) == INST_MULH)
    {
        // ['rd', 'rs1', 'rs2']
        long long res = ((long long) (int)reg_rs1) * ((long long)(int)reg_rs2);
        INST_STAT(ENUM_INST_MULH);
        DPRINTF(LOG_INST,("%08x: mulh r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = (int)(res >> 32);
        pc += 4;
    }
    else if ((opcode & INST_MULHSU_MASK) == INST_MULHSU)
    {
        // ['rd', 'rs1', 'rs2']
        long long res = ((long long) (int)reg_rs1) * ((unsigned long long)(unsigned)reg_rs2);
        INST_STAT(ENUM_INST_MULHSU);
        DPRINTF(LOG_INST,("%08x: mulhsu r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = (int)(res >> 32);
        pc += 4;
    }
    else if ((opcode & INST_MULHU_MASK) == INST_MULHU)
    {
        // ['rd', 'rs1', 'rs2']
        unsigned long long res = ((unsigned long long) (unsigned)reg_rs1) * ((unsigned long long)(unsigned)reg_rs2);
        INST_STAT(ENUM_INST_MULHU);
        DPRINTF(LOG_INST,("%08x: mulhu r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = (int)(res >> 32);
        pc += 4;
    }
    else if ((opcode & INST_DIV_MASK) == INST_DIV)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: div r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        INST_STAT(ENUM_INST_DIV);
        if ((signed)reg_rs1 == INT32_MIN && (signed)reg_rs2 == -1)
            reg_rd = reg_rs1;
        else if (reg_rs2 != 0)
            reg_rd = (signed)reg_rs1 / (signed)reg_rs2;
        else
            reg_rd = (unsigned)-1;
        pc += 4;        
    }
    else if ((opcode & INST_DIVU_MASK) == INST_DIVU)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: divu r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        INST_STAT(ENUM_INST_DIVU);
        if (reg_rs2 != 0)
            reg_rd = (unsigned)reg_rs1 / (unsigned)reg_rs2;
        else
            reg_rd = (unsigned)-1;
        pc += 4;        
    }
    else if ((opcode & INST_REM_MASK) == INST_REM)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: rem r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        INST_STAT(ENUM_INST_REM);

        if((signed)reg_rs1 == INT32_MIN && (signed)reg_rs2 == -1)
            reg_rd = 0;
        else if (reg_rs2 != 0)
            reg_rd = (signed)reg_rs1 % (signed)reg_rs2;
        else
            reg_rd = reg_rs1;
        pc += 4;        
    }
    else if ((opcode & INST_REMU_MASK) == INST_REMU)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: remu r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        INST_STAT(ENUM_INST_REMU);
        if (reg_rs2 != 0)
            reg_rd = (unsigned)reg_rs1 % (unsigned)reg_rs2;
        else
            reg_rd = reg_rs1;
        pc += 4;        
    }
    else if ((opcode & INST_ECALL_MASK) == INST_ECALL)
    {
        DPRINTF(LOG_INST,("%08x: ecall\n", pc));
        INST_STAT(ENUM_INST_ECALL);

        exception(MCAUSE_ECALL_U + m_csr_mpriv, pc);
        take_exception   = true;
    }
    else if ((opcode & INST_EBREAK_MASK) == INST_EBREAK)
    {
        DPRINTF(LOG_INST,("%08x: ebreak\n", pc));
        INST_STAT(ENUM_INST_EBREAK);

        exception(MCAUSE_BREAKPOINT, pc);
        take_exception   = true;
        m_break          = true;
    }
    else if ((opcode & INST_MRET_MASK) == INST_MRET)
    {
        DPRINTF(LOG_INST,("%08x: mret\n", pc));
        INST_STAT(ENUM_INST_MRET);

        assert(m_csr_mpriv == PRIV_MACHINE);

        uint32_t s        = m_csr_msr;
        uint32_t prev_prv = SR_GET_MPP(m_csr_msr);

        // Interrupt enable pop
        s &= ~SR_MIE;
        s |= (s & SR_MPIE) ? SR_MIE : 0;
        s |= SR_MPIE;

        // Set next MPP to user mode
        s &= ~SR_MPP;
        s |=  SR_MPP_U;

        // Set privilege level to previous MPP
        m_csr_mpriv   = prev_prv;
        m_csr_msr     = s;

        // Return to EPC
        pc          = m_csr_mepc;
    }
    else if ((opcode & INST_SRET_MASK) == INST_SRET)
    {
        DPRINTF(LOG_INST,("%08x: sret\n", pc));
        INST_STAT(ENUM_INST_SRET);

        assert(m_csr_mpriv == PRIV_SUPER);

        uint32_t s        = m_csr_msr;
        uint32_t prev_prv = (m_csr_msr & SR_SPP) ? PRIV_SUPER : PRIV_USER;

        // Interrupt enable pop
        s &= ~SR_SIE;
        s |= (s & SR_SPIE) ? SR_SIE : 0;
        s |= SR_SPIE;

        // Set next SPP to user mode
        s &= ~SR_SPP;

        // Set privilege level to previous MPP
        m_csr_mpriv   = prev_prv;
        m_csr_msr     = s;

        // Return to EPC
        pc          = m_csr_sepc;
    }
    else if ( ((opcode & INST_SFENCE_MASK) == INST_SFENCE) ||
              ((opcode & INST_FENCE_MASK) == INST_FENCE) ||
              ((opcode & INST_IFENCE_MASK) == INST_IFENCE))
    {
        DPRINTF(LOG_INST,("%08x: fence\n", pc));
        INST_STAT(ENUM_INST_FENCE);
        pc += 4;
    }
    else if ((opcode & INST_CSRRW_MASK) == INST_CSRRW)
    {
        DPRINTF(LOG_INST,("%08x: csrw r%d, r%d, 0x%x\n", pc, rd, rs1, imm12));
        INST_STAT(ENUM_INST_CSRRW);
        reg_rd = access_csr(imm12, reg_rs1, true, true);
        pc += 4;
    }    
    else if ((opcode & INST_CSRRS_MASK) == INST_CSRRS)
    {
        DPRINTF(LOG_INST,("%08x: csrs r%d, r%d, 0x%x\n", pc, rd, rs1, imm12));
        INST_STAT(ENUM_INST_CSRRS);
        reg_rd = access_csr(imm12, reg_rs1, true, false);
        pc += 4;
    }
    else if ((opcode & INST_CSRRC_MASK) == INST_CSRRC)
    {
        DPRINTF(LOG_INST,("%08x: csrc r%d, r%d, 0x%x\n", pc, rd, rs1, imm12));
        INST_STAT(ENUM_INST_CSRRC);
        reg_rd = access_csr(imm12, reg_rs1, false, true);
        pc += 4;
    }
    else if ((opcode & INST_CSRRWI_MASK) == INST_CSRRWI)
    {
        DPRINTF(LOG_INST,("%08x: csrwi r%d, %d, 0x%x\n", pc, rd, rs1, imm12));
        INST_STAT(ENUM_INST_CSRRWI);
        reg_rd = access_csr(imm12, rs1, true, true);
        pc += 4;
    }
    else if ((opcode & INST_CSRRSI_MASK) == INST_CSRRSI)
    {
        DPRINTF(LOG_INST,("%08x: csrsi r%d, %d, 0x%x\n", pc, rd, rs1, imm12));
        INST_STAT(ENUM_INST_CSRRSI);
        reg_rd = access_csr(imm12, rs1, true, false);
        pc += 4;
    }
    else if ((opcode & INST_CSRRCI_MASK) == INST_CSRRCI)
    {
        DPRINTF(LOG_INST,("%08x: csrci r%d, %d, 0x%x\n", pc, rd, rs1, imm12));
        INST_STAT(ENUM_INST_CSRRCI);
        reg_rd = access_csr(imm12, rs1, false, true);
        pc += 4;
    }
    else if ((opcode & INST_WFI_MASK) == INST_WFI)
    {
        DPRINTF(LOG_INST,("%08x: wfi\n", pc));
        INST_STAT(ENUM_INST_WFI);
        pc += 4;
    }
    else
    {
        error(false, "Bad instruction @ %x (opcode %x)\n", pc, opcode);
        exception(MCAUSE_ILLEGAL_INSTRUCTION, pc);
        m_fault        = true;
        take_exception = true;
    }

    if (rd != 0)
        m_gpr[rd] = reg_rd;

    // Pending interrupt
    if (!take_exception && (m_csr_mip & m_csr_mie))
    {
        uint32_t pending_interrupts = (m_csr_mip & m_csr_mie);
        uint32_t m_enabled          = m_csr_mpriv < PRIV_MACHINE || (m_csr_mpriv == PRIV_MACHINE && (m_csr_msr & SR_MIE));
        uint32_t s_enabled          = m_csr_mpriv < PRIV_SUPER   || (m_csr_mpriv == PRIV_SUPER   && (m_csr_msr & SR_SIE));
        uint32_t m_interrupts       = pending_interrupts & ~m_csr_mideleg & -m_enabled;
        uint32_t s_interrupts       = pending_interrupts & m_csr_mideleg & -s_enabled;
        uint32_t interrupts         = m_interrupts ? m_interrupts : s_interrupts;

        // Interrupt pending and mask enabled
        if (interrupts)
        {
            //printf("Take Interrupt...: %08x\n", interrupts);
            int i;

            for (i=IRQ_MIN;i<IRQ_MAX;i++)
            {
                if (interrupts & (1 << i))
                {
                    // Only service one interrupt per cycle
                    DPRINTF(LOG_INST,( "Interrupt%d taken...\n", i));
                    exception(MCAUSE_INTERRUPT + i, pc);
                    take_exception = true;
                    break;
                }
            }
        }
    }

    // Stats interface
    if (m_stats_if)
        m_stats_if->execute(m_pc, opcode);

    if (!take_exception)
        m_pc = pc;
}
//-----------------------------------------------------------------
// step: Step through one instruction
//-----------------------------------------------------------------
void Riscv::step(void)
{
    m_stats[STATS_INSTRUCTIONS]++;

    // Execute instruction at current PC
    execute();

    // Increment timer counter
    m_csr_mtime++;

    // Timer should generate a interrupt?
    // Limited internal timer, truncate to 32-bits
    m_csr_mtime &= 0xFFFFFFFF;
    if (m_csr_mtime == m_csr_mtimecmp)
        m_csr_mip |= (m_csr_mideleg & SR_IP_STIP) ? SR_IP_STIP : SR_IP_MTIP;

    // Dump state
    if (TRACE_ENABLED(LOG_REGISTERS))
    {
        // Register trace
        int i;
        for (i=0;i<REGISTERS;i+=4)
        {
            DPRINTF(LOG_REGISTERS,( " %d: ", i));
            DPRINTF(LOG_REGISTERS,( " %08x %08x %08x %08x\n", m_gpr[i+0], m_gpr[i+1], m_gpr[i+2], m_gpr[i+3]));
        }
    }

    // Breakpoint hit?
    if (m_has_breakpoints && check_breakpoint(m_pc_x))
        m_break = true;
}
//-----------------------------------------------------------------
// set_interrupt: Register pending interrupt
//-----------------------------------------------------------------
void Riscv::set_interrupt(int irq)
{
    assert(irq == 0);
    m_csr_mip |= SR_IP_MEIP;
}
//-----------------------------------------------------------------
// stats_reset: Reset runtime stats
//-----------------------------------------------------------------
void Riscv::stats_reset(void)
{
    // Clear stats
    for (int i=STATS_MIN;i<STATS_MAX;i++)
        m_stats[i] = 0;
}
//-----------------------------------------------------------------
// stats_dump: Show execution stats
//-----------------------------------------------------------------
void Riscv::stats_dump(void)
{  
    // Stats interface
    if (m_stats_if)
    {
        m_stats_if->print();
        m_stats_if->reset();
    }
    else
    {
        printf( "Runtime Stats:\n");
        printf( "- Total Instructions %d\n", m_stats[STATS_INSTRUCTIONS]);
        if (m_stats[STATS_INSTRUCTIONS] > 0)
        {
            printf( "- Loads %d (%d%%)\n",  m_stats[STATS_LOADS],  (m_stats[STATS_LOADS] * 100)  / m_stats[STATS_INSTRUCTIONS]);
            printf( "- Stores %d (%d%%)\n", m_stats[STATS_STORES], (m_stats[STATS_STORES] * 100) / m_stats[STATS_INSTRUCTIONS]);
            printf( "- Branches Operations %d (%d%%)\n", m_stats[STATS_BRANCHES], (m_stats[STATS_BRANCHES] * 100)  / m_stats[STATS_INSTRUCTIONS]);
        }
    }

    stats_reset();
}
