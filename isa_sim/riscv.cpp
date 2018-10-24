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
#include "riscv.h"

//-----------------------------------------------------------------
// Defines:
//-----------------------------------------------------------------
#define DPRINTF(l,a)        do { if (Trace & l) printf a; } while (0)
#define TRACE_ENABLED(l)    (Trace & l)

//-----------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------
Riscv::Riscv()
{
    MemRegions = 0;
    StatsInstStatsEnable = false;
    StatsIf = NULL;
}
//-----------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------
Riscv::Riscv(uint32_t baseAddr, uint32_t len)
{
    MemRegions = 0;
    StatsInstStatsEnable = false;
    StatsIf = NULL;

    create_memory(baseAddr, len);
    reset(baseAddr);
}
//-----------------------------------------------------------------
// Deconstructor
//-----------------------------------------------------------------
Riscv::~Riscv()
{
    int m;

    for (m=0;m<MemRegions;m++)
    {
        if (Mem[m])
            delete Mem[m];
        Mem[m] = NULL;
    }
}
//-----------------------------------------------------------------
// create_memory:
//-----------------------------------------------------------------
bool Riscv::create_memory(uint32_t baseAddr, uint32_t len, uint8_t *buf /*=NULL*/)
{
    if (buf)
        return AttachMemory(new SimpleMemory(buf, len), baseAddr, len);
    else
        return AttachMemory(new SimpleMemory(len), baseAddr, len);
}
//-----------------------------------------------------------------
// AttachMemory:
//-----------------------------------------------------------------
bool Riscv::AttachMemory(Memory *memory, uint32_t baseAddr, uint32_t len)
{
    if (MemRegions < MAX_MEM_REGIONS)
    {
        MemBase[MemRegions] = baseAddr;
        MemSize[MemRegions] = len;

        Mem[MemRegions] = memory;
        Mem[MemRegions]->Reset();

        MemRegions++;

        return true;
    }

    return false;
}
//-----------------------------------------------------------------
// reset: Reset CPU state
//-----------------------------------------------------------------
void Riscv::reset(uint32_t start_addr)
{
    int i;

    r_pc        = start_addr;
    r_pc_x      = start_addr;

    for (i=0;i<REGISTERS;i++)
        r_gpr[i] = 0;

    csr_epc     = 0;
    csr_sr      = 0;
    csr_ie      = 0;
    csr_ip      = 0;
    csr_cause   = 0;
    csr_evec    = 0;
    csr_time    = 0;
    csr_timecmp = 0;

    Fault       = false;
    Break       = false;
    Trace       = 0;

    ResetStats();
}
//-----------------------------------------------------------------
// ResetStats: Reset runtime stats
//-----------------------------------------------------------------
void Riscv::ResetStats(void)
{
    int i;

    // Clear stats
    for (i=STATS_MIN;i<STATS_MAX;i++)
        Stats[i] = 0;

    for (i=0;i<ENUM_INST_MAX;i++)
        StatsInst[i] = 0;
}
//-----------------------------------------------------------------
// LoadMem: Load program code into startAddr offset
//-----------------------------------------------------------------
bool Riscv::LoadMem(uint32_t startAddr, uint8_t *data, int len)
{
    int i;
    int j;
    bool res = false;

    for (i=0; i<len; i+=4)
    {
        for (j=0;j<MemRegions;j++)
        {
            if (((startAddr+i) >= MemBase[j]) && ((startAddr+i) + len) <= (MemBase[j] + MemSize[j]))
            {
                uint32_t data_word;
                data_word = ((unsigned)*data++) << 0;
                data_word|= ((unsigned)*data++) << 8;
                data_word|= ((unsigned)*data++) << 16;
                data_word|= ((unsigned)*data++) << 24;

                Mem[j]->Store(startAddr - MemBase[j] + i, data_word, 4);
                res = true;
            }
        }
    }

    return res;
}
//-----------------------------------------------------------------
// write: 
//-----------------------------------------------------------------
void Riscv::write(uint32_t address, uint8_t data)
{
    for (int j=0;j<MemRegions;j++)
        if (address >= MemBase[j] && address < (MemBase[j] + MemSize[j]))
        {
            Mem[j]->Store(address - MemBase[j], data, 1);
            return ;
        }
    assert(!"Memory write issues...");
}
//-----------------------------------------------------------------
// valid_addr: 
//-----------------------------------------------------------------
bool Riscv::valid_addr(uint32_t address)
{
    for (int j=0;j<MemRegions;j++)
        if (address >= MemBase[j] && address < (MemBase[j] + MemSize[j]))
            return true;

    return false;
}
//-----------------------------------------------------------------
// read: 
//-----------------------------------------------------------------
uint8_t Riscv::read(uint32_t address)
{
    for (int j=0;j<MemRegions;j++)
        if (address >= MemBase[j] && address < (MemBase[j] + MemSize[j]))
            return Mem[j]->Load(address - MemBase[j], 1, false);

    return 0;
}
//-----------------------------------------------------------------
// get_opcode: Get instruction from address
//-----------------------------------------------------------------
uint32_t Riscv::get_opcode(uint32_t address)
{
    int m;

    for (m=0;m<MemRegions;m++)
        if (address >= MemBase[m] && address < (MemBase[m] + MemSize[m]))
            return Mem[m]->Load(address - MemBase[m], 4, false);

    return 0;
}
//-----------------------------------------------------------------
// Load:
//-----------------------------------------------------------------
uint32_t Riscv::Load(uint32_t pc, uint32_t address, int width, bool signedLoad)
{    
    int j;

    DPRINTF(LOG_MEM, ("        LOAD(%x, %d)\n", address, width));

    event_push(COSIM_EVENT_LOAD, address & ~3, 0);

    Stats[STATS_LOADS]++;

    for (j=0;j<MemRegions;j++)
        if (address >= MemBase[j] && address < (MemBase[j] + MemSize[j]))
        {
            uint32_t result = Mem[j]->Load(address - MemBase[j], width, signedLoad); 
            event_push(COSIM_EVENT_LOAD_RESULT, result, 0);
            return result;
        }

    fprintf(stderr, "%08x: Bad memory access 0x%x\n", pc, address);
    return 0;
}
//-----------------------------------------------------------------
// Store:
//-----------------------------------------------------------------
void Riscv::Store(uint32_t pc, uint32_t address, uint32_t data, int width)
{
    int j;

    DPRINTF(LOG_MEM,( "        STORE(%x, %x, %d)\n", address, data, width));
    Stats[STATS_STORES]++;

    if (width == 1)
        event_push(COSIM_EVENT_STORE, address & ~3, data & 0xFF);
    else if (width == 2)
        event_push(COSIM_EVENT_STORE, address & ~3, data & 0xFFFF);
    else
        event_push(COSIM_EVENT_STORE, address, data);

    for (j=0;j<MemRegions;j++)
        if (address >= MemBase[j] && address < (MemBase[j] + MemSize[j]))
        {
            Mem[j]->Store(address - MemBase[j], data, width);
            return ;
        }

    fprintf(stderr, "%08x: Bad memory access 0x%x\n", pc, address);
}
//-----------------------------------------------------------------
// AccessCsr:
//-----------------------------------------------------------------
uint32_t Riscv::AccessCsr(uint32_t address, uint32_t data, bool set, bool clr)
{
    uint32_t result = 0;
    switch (address & 0xFFF)
    {
        //--------------------------------------------------------
        // Standard
        //--------------------------------------------------------
        case CSR_MEPC:
            data       &= CSR_MEPC_MASK;
            result      = csr_epc;
            if (set && clr)
                csr_epc  = data;
            else if (set)
                csr_epc |= data;
            else if (clr)
                csr_epc &= ~data;
            break;
        case CSR_MTVEC:
            data       &= CSR_MTVEC_MASK;
            result      = csr_evec;
            if (set && clr)
                csr_evec  = data;
            else if (set)
                csr_evec |= data;
            else if (clr)
                csr_evec &= ~data;
            break;
        case CSR_MCAUSE:
            data       &= CSR_MCAUSE_MASK;
            result      = csr_cause;
            if (set && clr)
                csr_cause  = data;
            else if (set)
                csr_cause |= data;
            else if (clr)
                csr_cause &= ~data;
            break;
        case CSR_MSTATUS:
            data       &= CSR_MSTATUS_MASK;
            result      = csr_sr;
            if (set && clr)
                csr_sr  = data;
            else if (set)
                csr_sr |= data;
            else if (clr)
                csr_sr &= ~data;
            break;
        case CSR_MIP:
            data       &= CSR_MIP_MASK;
            result      = csr_ip;
            if (set && clr)
                csr_ip = data;
            else if (set)
                csr_ip |= data;
            else if (clr)
                csr_ip &= ~data;
            break;
        case CSR_MIE:
            data       &= CSR_MIE_MASK;
            result      = csr_ie;
            if (set && clr)
                csr_ie  = data;
            else if (set)
                csr_ie |= data;
            else if (clr)
                csr_ie &= ~data;
            break;
        //--------------------------------------------------------
        // Extensions
        //--------------------------------------------------------
        case CSR_DSCRATCH:
            switch (data & 0xFF000000)
            {
                case CSR_SIM_CTRL_EXIT:
                    exit(data & 0xFF);
                    break;
                case CSR_SIM_CTRL_PUTC:
                    fprintf(stderr, "%c", (data & 0xFF));
                    break;
                case CSR_SIM_CTRL_TRACE:
                    enable_trace(data & 0xFF);
                    break;
                case CSR_SIM_PRINTF:
                {
                    uint32_t fmt_addr = r_gpr[10];
                    uint32_t arg1     = r_gpr[11];
                    uint32_t arg2     = r_gpr[12];
                    uint32_t arg3     = r_gpr[13];
                    uint32_t arg4     = r_gpr[14];

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
        case CSR_MTIME:
            data       &= CSR_MTIME_MASK;
            result      = csr_time;

            // Non-std behaviour - write to CSR_TIME gives next interrupt threshold
            if (set)
            {
                csr_timecmp = data;

                // Clear interrupt pending
                csr_ip &= ~SR_IP_MTIP;
            }
            break;
    }
    return result;
}
//-----------------------------------------------------------------
// Exception:
//-----------------------------------------------------------------
void Riscv::Exception(uint32_t cause, uint32_t pc)
{
    // Interrupt save and disable
    csr_sr &= ~SR_MPIE;
    csr_sr |= (csr_sr & SR_MIE) ? SR_MPIE : 0;
    csr_sr &= ~SR_MIE;

    csr_epc = pc;

    csr_cause = cause;
}
//-----------------------------------------------------------------
// Execute: Instruction execution stage
//-----------------------------------------------------------------
void Riscv::Execute(void)
{
    // Get opcode at current PC
    uint32_t opcode = get_opcode(r_pc);
    r_pc_x = r_pc;

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
    uint32_t reg_rs1 = r_gpr[rs1];
    uint32_t reg_rs2 = r_gpr[rs2];
    uint32_t pc      = r_pc;

    bool exception = false;

    //char sym_name[1024];
    //sym_name[0] = '\0';
    //symbol_get_name(pc, sym_name, sizeof(sym_name)-1);

    DPRINTF(LOG_OPCODES,( "%08x: %08x\n", pc, opcode));
    DPRINTF(LOG_OPCODES,( "        rd(%d) r%d = %d, r%d = %d\n", rd, rs1, reg_rs1, rs2, reg_rs2));    

    //printf("%s\n", sym_name);

    // As RVC is not supported, fault on opcode which is all zeros
    if (opcode == 0)
    {
        fprintf(stderr, "Bad instruction @ %x\n", pc);

        Exception(MCAUSE_ILLEGAL_INSTRUCTION, pc);
        Fault = true;
        exception = true;        
    }
    else if ((opcode & INST_ANDI_MASK) == INST_ANDI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: andi r%d, r%d, %d\n", pc, rd, rs1, imm12));
        StatsInst[ENUM_INST_ANDI]++;
        reg_rd = reg_rs1 & imm12;
        pc += 4;        
    }
    else if ((opcode & INST_ORI_MASK) == INST_ORI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: ori r%d, r%d, %d\n", pc, rd, rs1, imm12));
        StatsInst[ENUM_INST_ORI]++;
        reg_rd = reg_rs1 | imm12;
        pc += 4;        
    }
    else if ((opcode & INST_XORI_MASK) == INST_XORI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: xori r%d, r%d, %d\n", pc, rd, rs1, imm12));
        StatsInst[ENUM_INST_XORI]++;
        reg_rd = reg_rs1 ^ imm12;
        pc += 4;        
    }
    else if ((opcode & INST_ADDI_MASK) == INST_ADDI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: addi r%d, r%d, %d\n", pc, rd, rs1, imm12));
        StatsInst[ENUM_INST_ADDI]++;
        reg_rd = reg_rs1 + imm12;
        pc += 4;
    }
    else if ((opcode & INST_SLTI_MASK) == INST_SLTI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: slti r%d, r%d, %d\n", pc, rd, rs1, imm12));
        StatsInst[ENUM_INST_SLTI]++;
        reg_rd = (signed)reg_rs1 < (signed)imm12;
        pc += 4;        
    }
    else if ((opcode & INST_SLTIU_MASK) == INST_SLTIU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: sltiu r%d, r%d, %d\n", pc, rd, rs1, (unsigned)imm12));
        StatsInst[ENUM_INST_SLTIU]++;
        reg_rd = (unsigned)reg_rs1 < (unsigned)imm12;
        pc += 4;        
    }
    else if ((opcode & INST_SLLI_MASK) == INST_SLLI)
    {
        // ['rd', 'rs1']
        DPRINTF(LOG_INST,("%08x: slli r%d, r%d, %d\n", pc, rd, rs1, shamt));
        StatsInst[ENUM_INST_SLLI]++;
        reg_rd = reg_rs1 << shamt;
        pc += 4;        
    }
    else if ((opcode & INST_SRLI_MASK) == INST_SRLI)
    {
        // ['rd', 'rs1', 'shamt']
        DPRINTF(LOG_INST,("%08x: srli r%d, r%d, %d\n", pc, rd, rs1, shamt));
        StatsInst[ENUM_INST_SRLI]++;
        reg_rd = (unsigned)reg_rs1 >> shamt;
        pc += 4;        
    }
    else if ((opcode & INST_SRAI_MASK) == INST_SRAI)
    {
        // ['rd', 'rs1', 'shamt']
        DPRINTF(LOG_INST,("%08x: srai r%d, r%d, %d\n", pc, rd, rs1, shamt));
        StatsInst[ENUM_INST_SRAI]++;
        reg_rd = (signed)reg_rs1 >> shamt;
        pc += 4;        
    }
    else if ((opcode & INST_LUI_MASK) == INST_LUI)
    {
        // ['rd', 'imm20']
        DPRINTF(LOG_INST,("%08x: lui r%d, 0x%x\n", pc, rd, imm20));
        StatsInst[ENUM_INST_LUI]++;
        reg_rd = imm20;
        pc += 4;        
    }
    else if ((opcode & INST_AUIPC_MASK) == INST_AUIPC)
    {
        // ['rd', 'imm20']
        DPRINTF(LOG_INST,("%08x: auipc r%d, 0x%x\n", pc, rd, imm20));
        StatsInst[ENUM_INST_AUIPC]++;
        reg_rd = imm20 + pc;
        pc += 4;        
    }
    else if ((opcode & INST_ADD_MASK) == INST_ADD)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: add r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        StatsInst[ENUM_INST_ADD]++;
        reg_rd = reg_rs1 + reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SUB_MASK) == INST_SUB)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sub r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        StatsInst[ENUM_INST_SUB]++;
        reg_rd = reg_rs1 - reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SLT_MASK) == INST_SLT)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: slt r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        StatsInst[ENUM_INST_SLT]++;
        reg_rd = (signed)reg_rs1 < (signed)reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SLTU_MASK) == INST_SLTU)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sltu r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        StatsInst[ENUM_INST_SLTU]++;
        reg_rd = (unsigned)reg_rs1 < (unsigned)reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_XOR_MASK) == INST_XOR)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: xor r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        StatsInst[ENUM_INST_XOR]++;
        reg_rd = reg_rs1 ^ reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_OR_MASK) == INST_OR)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: or r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        StatsInst[ENUM_INST_OR]++;
        reg_rd = reg_rs1 | reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_AND_MASK) == INST_AND)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: and r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        StatsInst[ENUM_INST_AND]++;
        reg_rd = reg_rs1 & reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SLL_MASK) == INST_SLL)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sll r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        StatsInst[ENUM_INST_SLL]++;
        reg_rd = reg_rs1 << reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SRL_MASK) == INST_SRL)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: srl r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        StatsInst[ENUM_INST_SRL]++;
        reg_rd = (unsigned)reg_rs1 >> reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SRA_MASK) == INST_SRA)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sra r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        StatsInst[ENUM_INST_SRA]++;
        reg_rd = (signed)reg_rs1 >> reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_JAL_MASK) == INST_JAL)
    {
        // ['rd', 'jimm20']
        DPRINTF(LOG_INST,("%08x: jal r%d, %d\n", pc, rd, jimm20));
        StatsInst[ENUM_INST_JAL]++;
        reg_rd = pc + 4;
        pc+= jimm20;

        Stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_JALR_MASK) == INST_JALR)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: jalr r%d, r%d\n", pc, rs1, imm12));
        StatsInst[ENUM_INST_JALR]++;
        reg_rd = pc + 4;
        pc = (reg_rs1 + imm12) & ~1;

        Stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_BEQ_MASK) == INST_BEQ)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: beq r%d, r%d, %d\n", pc, rs1, rs2, bimm));
        StatsInst[ENUM_INST_BEQ]++;
        if (reg_rs1 == reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;

        Stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_BNE_MASK) == INST_BNE)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: bne r%d, r%d, %d\n", pc, rs1, rs2, bimm));
        StatsInst[ENUM_INST_BNE]++;
        if (reg_rs1 != reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;

        Stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_BLT_MASK) == INST_BLT)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: blt r%d, r%d, %d\n", pc, rs1, rs2, bimm));
        StatsInst[ENUM_INST_BLT]++;
        if ((signed)reg_rs1 < (signed)reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;

        Stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_BGE_MASK) == INST_BGE)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: bge r%d, r%d, %d\n", pc, rs1, rs2, bimm));
        StatsInst[ENUM_INST_BGE]++;
        if ((signed)reg_rs1 >= (signed)reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;

        Stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_BLTU_MASK) == INST_BLTU)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: bltu r%d, r%d, %d\n", pc, rs1, rs2, bimm));
        StatsInst[ENUM_INST_BLTU]++;
        if ((unsigned)reg_rs1 < (unsigned)reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;

        Stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_BGEU_MASK) == INST_BGEU)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: bgeu r%d, r%d, %d\n", pc, rs1, rs2, bimm));
        StatsInst[ENUM_INST_BGEU]++;
        if ((unsigned)reg_rs1 >= (unsigned)reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;

        Stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_LB_MASK) == INST_LB)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lb r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        StatsInst[ENUM_INST_LB]++;
        reg_rd = Load(pc, reg_rs1 + imm12, 1, true);
        pc += 4;
    }
    else if ((opcode & INST_LH_MASK) == INST_LH)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lh r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        StatsInst[ENUM_INST_LH]++;
        reg_rd = Load(pc, reg_rs1 + imm12, 2, true);
        pc += 4;
    }
    else if ((opcode & INST_LW_MASK) == INST_LW)
    {
        // ['rd', 'rs1', 'imm12']        
        reg_rd = Load(pc, reg_rs1 + imm12, 4, true);
        DPRINTF(LOG_INST,("%08x: lw r%d, %d(r%d) = 0x%x\n", pc, rd, imm12, rs1, reg_rd));
        StatsInst[ENUM_INST_LW]++;
        pc += 4;        
    }
    else if ((opcode & INST_LBU_MASK) == INST_LBU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lbu r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        StatsInst[ENUM_INST_LBU]++;
        reg_rd = Load(pc, reg_rs1 + imm12, 1, false);
        pc += 4;
    }
    else if ((opcode & INST_LHU_MASK) == INST_LHU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lhu r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        StatsInst[ENUM_INST_LHU]++;
        reg_rd = Load(pc, reg_rs1 + imm12, 2, false);
        pc += 4;
    }
    else if ((opcode & INST_LWU_MASK) == INST_LWU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lwu r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        StatsInst[ENUM_INST_LWU]++;
        reg_rd = Load(pc, reg_rs1 + imm12, 4, false);
        pc += 4;
    }
    else if ((opcode & INST_SB_MASK) == INST_SB)
    {
        // ['imm12hi', 'rs1', 'rs2', 'imm12lo']
        DPRINTF(LOG_INST,("%08x: sb %d(r%d), r%d\n", pc, storeimm, rs1, rs2));
        StatsInst[ENUM_INST_SB]++;
        Store(pc, reg_rs1 + storeimm, reg_rs2, 1);
        pc += 4;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_SH_MASK) == INST_SH)
    {
        // ['imm12hi', 'rs1', 'rs2', 'imm12lo']
        DPRINTF(LOG_INST,("%08x: sh %d(r%d), r%d\n", pc, storeimm, rs1, rs2));
        StatsInst[ENUM_INST_SH]++;
        Store(pc, reg_rs1 + storeimm, reg_rs2, 2);
        pc += 4;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_SW_MASK) == INST_SW)
    {
        // ['imm12hi', 'rs1', 'rs2', 'imm12lo']
        DPRINTF(LOG_INST,("%08x: sw %d(r%d), r%d\n", pc, storeimm, rs1, rs2));
        StatsInst[ENUM_INST_SW]++;
        Store(pc, reg_rs1 + storeimm, reg_rs2, 4);
        pc += 4;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_MUL_MASK) == INST_MUL)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: mul r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        StatsInst[ENUM_INST_MUL]++;
        reg_rd = (signed)reg_rs1 * (signed)reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_MULH_MASK) == INST_MULH)
    {
        // ['rd', 'rs1', 'rs2']
        long long res = ((long long) (int)reg_rs1) * ((long long)(int)reg_rs2);
        StatsInst[ENUM_INST_MULH]++;
        DPRINTF(LOG_INST,("%08x: mulh r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = (int)(res >> 32);
        pc += 4;
    }
    else if ((opcode & INST_MULHSU_MASK) == INST_MULHSU)
    {
        // ['rd', 'rs1', 'rs2']
        long long res = ((long long) (int)reg_rs1) * ((unsigned long long)(unsigned)reg_rs2);
        StatsInst[ENUM_INST_MULHSU]++;
        DPRINTF(LOG_INST,("%08x: mulhsu r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = (int)(res >> 32);
        pc += 4;
    }
    else if ((opcode & INST_MULHU_MASK) == INST_MULHU)
    {
        // ['rd', 'rs1', 'rs2']
        unsigned long long res = ((unsigned long long) (unsigned)reg_rs1) * ((unsigned long long)(unsigned)reg_rs2);
        StatsInst[ENUM_INST_MULHU]++;
        DPRINTF(LOG_INST,("%08x: mulhu r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = (int)(res >> 32);
        pc += 4;
    }
    else if ((opcode & INST_DIV_MASK) == INST_DIV)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: div r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        StatsInst[ENUM_INST_DIV]++;
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
        StatsInst[ENUM_INST_DIVU]++;
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
        StatsInst[ENUM_INST_REM]++;

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
        StatsInst[ENUM_INST_REMU]++;
        if (reg_rs2 != 0)
            reg_rd = (unsigned)reg_rs1 % (unsigned)reg_rs2;
        else
            reg_rd = reg_rs1;
        pc += 4;        
    }
    else if ((opcode & INST_ECALL_MASK) == INST_ECALL)
    {
        DPRINTF(LOG_INST,("%08x: ecall\n", pc));
        StatsInst[ENUM_INST_ECALL]++;
        
        Exception(MCAUSE_ECALL_U, pc);

        pc          = csr_evec;
        exception   = true;
    }
    else if ((opcode & INST_EBREAK_MASK) == INST_EBREAK)
    {
        DPRINTF(LOG_INST,("%08x: ebreak\n", pc));
        StatsInst[ENUM_INST_EBREAK]++;

        Exception(MCAUSE_BREAKPOINT, pc);

        pc          = csr_evec;
        exception   = true;
        Break       = true;
    }
    else if ((opcode & INST_MRET_MASK) == INST_MRET)
    {
        DPRINTF(LOG_INST,("%08x: mret\n", pc));
        StatsInst[ENUM_INST_MRET]++;

        // Interrupt enable pop
        csr_sr &= ~SR_MIE;
        csr_sr |= (csr_sr & SR_MPIE) ? SR_MIE : 0;
        csr_sr |= SR_MPIE;

        // Return to EPC
        pc          = csr_epc;        
    }
    else if ((opcode & INST_CSRRW_MASK) == INST_CSRRW)
    {
        DPRINTF(LOG_INST,("%08x: csrw r%d, r%d, 0x%x\n", pc, rd, rs1, imm12));
        StatsInst[ENUM_INST_CSRRW]++;
        reg_rd = AccessCsr(imm12, reg_rs1, true, true);
        pc += 4;
    }    
    else if ((opcode & INST_CSRRS_MASK) == INST_CSRRS)
    {
        DPRINTF(LOG_INST,("%08x: csrs r%d, r%d, 0x%x\n", pc, rd, rs1, imm12));
        StatsInst[ENUM_INST_CSRRS]++;
        reg_rd = AccessCsr(imm12, reg_rs1, true, false);
        pc += 4;
    }
    else if ((opcode & INST_CSRRC_MASK) == INST_CSRRC)
    {
        DPRINTF(LOG_INST,("%08x: csrc r%d, r%d, 0x%x\n", pc, rd, rs1, imm12));
        StatsInst[ENUM_INST_CSRRC]++;
        reg_rd = AccessCsr(imm12, reg_rs1, false, true);
        pc += 4;
    }
    else if ((opcode & INST_CSRRWI_MASK) == INST_CSRRWI)
    {
        StatsInst[ENUM_INST_CSRRWI]++;
        reg_rd = AccessCsr(imm12, rs1, true, true);
        pc += 4;
    }
    else if ((opcode & INST_CSRRSI_MASK) == INST_CSRRSI)
    {
        StatsInst[ENUM_INST_CSRRSI]++;
        reg_rd = AccessCsr(imm12, rs1, true, false);
        pc += 4;
    }
    else if ((opcode & INST_CSRRCI_MASK) == INST_CSRRCI)
    {
        StatsInst[ENUM_INST_CSRRCI]++;
        reg_rd = AccessCsr(imm12, rs1, false, true);
        pc += 4;
    }    
    else
    {
        fprintf(stderr, "Bad instruction @ %x (opcode %x)\n", pc, opcode);
        Exception(MCAUSE_ILLEGAL_INSTRUCTION, pc);
        Fault = true;
        exception = true;
    }

    if (rd != 0)
        r_gpr[rd] = reg_rd;

    // Interrupts enabled, check for pending interrupt
    if ((csr_sr & SR_MIE) && !exception)
    {
        uint32_t interrupts =  csr_ip & csr_ie;

        // Interrupt pending and mask enabled
        if (interrupts)
        {
            int i;

            for (i=IRQ_MIN;i<IRQ_MAX;i++)
            {
                if (interrupts & (1 << i))
                {
                    // Only service one interrupt per cycle
                    DPRINTF(LOG_INST,( "Interrupt%d taken...\n", i));
                    Exception(MCAUSE_INTERRUPT + i, pc);

                    pc          = csr_evec;                    
                    break;
                }
            }
        }
    }

    // Stats interface
    if (StatsIf)
        StatsIf->Execute(r_pc, opcode);

    r_pc = pc;
}
//-----------------------------------------------------------------
// step: Step through one instruction
//-----------------------------------------------------------------
void Riscv::step(void)
{
    Stats[STATS_INSTRUCTIONS]++;

    // Execute instruction at current PC
    Execute();

    // Increment timer counter
    csr_time++;

    // Timer should generate a interrupt?
    if (csr_time == csr_timecmp)
        csr_ip |= SR_IP_MTIP;

    // Dump state
    if (TRACE_ENABLED(LOG_REGISTERS))
    {
        // Register trace
        int i;
        for (i=0;i<REGISTERS;i+=4)
        {
            DPRINTF(LOG_REGISTERS,( " %d: ", i));
            DPRINTF(LOG_REGISTERS,( " %08x %08x %08x %08x\n", r_gpr[i+0], r_gpr[i+1], r_gpr[i+2], r_gpr[i+3]));
        }
    }

    // Machine register trace
    if (TRACE_ENABLED(LOG_MACH_TRACE))
    {
        int i;
        DPRINTF(LOG_MACH_TRACE, ( "REG: "));
        for (i=0;i<REGISTERS;i++)
        {
            if (i != 0)
                DPRINTF(LOG_MACH_TRACE, ( ", "));
            DPRINTF(LOG_MACH_TRACE, ( "%08x", r_gpr[i]));
        }
        DPRINTF(LOG_MACH_TRACE, ( "\n"));
    }
}
//-----------------------------------------------------------------
// set_interrupt: Register pending interrupt
//-----------------------------------------------------------------
void Riscv::set_interrupt(int irq)
{
    assert(irq == 0);
    csr_ip |= SR_IP_MEIP;
}
//-----------------------------------------------------------------
// DumpStats: Show execution stats
//-----------------------------------------------------------------
void Riscv::DumpStats(void)
{  
    // Stats interface
    if (StatsIf)
    {
        StatsIf->Print();
        StatsIf->Reset();
    }
    else
    {
        printf( "Runtime Stats:\n");
        printf( "- Total Instructions %d\n", Stats[STATS_INSTRUCTIONS]);
        if (Stats[STATS_INSTRUCTIONS] > 0)
        {
            printf( "- Loads %d (%d%%)\n",  Stats[STATS_LOADS],  (Stats[STATS_LOADS] * 100)  / Stats[STATS_INSTRUCTIONS]);
            printf( "- Stores %d (%d%%)\n", Stats[STATS_STORES], (Stats[STATS_STORES] * 100) / Stats[STATS_INSTRUCTIONS]);
            printf( "- Branches Operations %d (%d%%)\n", Stats[STATS_BRANCHES], (Stats[STATS_BRANCHES] * 100)  / Stats[STATS_INSTRUCTIONS]);
        }
    }

    if (StatsInstStatsEnable)
    {
        int i;

        printf("\nInstructions:\n");
        for (i=0;i<ENUM_INST_MAX;i++)
            printf("- %s = %d\n", inst_names[i], StatsInst[i]);
    }

    ResetStats();
}
