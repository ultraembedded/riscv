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

//-----------------------------------------------------------------
// Defines:
//-----------------------------------------------------------------
#define DPRINTF(l,a)        do { if (Trace & l) printf a; } while (0)
#define TRACE_ENABLED(l)    (Trace & l)

//-----------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------
Riscv::Riscv(uint32_t baseAddr, uint32_t len)
{
    MemRegions = 0;

    CreateMemory(baseAddr, len);
    Reset(baseAddr);
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
// CreateMemory:
//-----------------------------------------------------------------
bool Riscv::CreateMemory(uint32_t baseAddr, uint32_t len)
{
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
// Reset: Reset CPU state
//-----------------------------------------------------------------
void Riscv::Reset(uint32_t start_addr)
{
    int i;

    r_pc        = start_addr;

    for (i=0;i<REGISTERS;i++)
        r_gpr[i] = 0;

    csr_epc     = 0;
    csr_sr      = SR_S;
    csr_cause   = 0;
    csr_evec    = 0;

    Fault       = false;
    Break       = false;
    Trace       = 0;
    InterruptPending = false;

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
// GetOpcode: Get instruction from address
//-----------------------------------------------------------------
uint32_t Riscv::GetOpcode(uint32_t address)
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
uint32_t Riscv::Load(uint32_t address, int width, bool signedLoad)
{    
    int j;

    DPRINTF(LOG_MEM, ("        LOAD(%x, %d)\n", address, width));

    Stats[STATS_LOADS]++;

    for (j=0;j<MemRegions;j++)
        if (address >= MemBase[j] && address < (MemBase[j] + MemSize[j]))
            return Mem[j]->Load(address - MemBase[j], width, signedLoad);

    fprintf(stderr, "Bad memory access 0x%x\n", address);
    return 0;
}
//-----------------------------------------------------------------
// Store:
//-----------------------------------------------------------------
void Riscv::Store(uint32_t address, uint32_t data, int width)
{
    int j;

    DPRINTF(LOG_MEM,( "        STORE(%x, %x, %d)\n", address, data, width));
    Stats[STATS_STORES]++;

    for (j=0;j<MemRegions;j++)
        if (address >= MemBase[j] && address < (MemBase[j] + MemSize[j]))
        {
            Mem[j]->Store(address - MemBase[j], data, width);
            return ;
        }

    fprintf(stderr, "Bad memory access 0x%x\n", address);
}
//-----------------------------------------------------------------
// AccessCsr:
//-----------------------------------------------------------------
uint32_t Riscv::AccessCsr(uint32_t address, uint32_t data, bool set, bool clr)
{
    uint32_t result = 0;
    switch (address)
    {
        case CSR_EPC:
            result      = csr_epc;
            if (set && clr)
                csr_epc = data;
            else if (set)
                csr_epc    |= data;
            else if (clr)
                csr_epc    &= ~data;
            break;
        case CSR_EVEC:
            result      = csr_evec;
            if (set && clr)
                csr_evec = data;
            else if (set)
                csr_evec    |= data;
            else if (clr)
                csr_evec    &= ~data;
            break;
        case CSR_CAUSE:
            result      = csr_cause;
            if (set && clr)
                csr_cause = data;
            else if (set)
                csr_cause    |= data;
            else if (clr)
                csr_cause    &= ~data;
            break;
        case CSR_STATUS:
            result      = csr_sr;
            if (set && clr)
                csr_sr = data;
            else if (set)
                csr_sr    |= data;
            else if (clr)
                csr_sr    &= ~data;
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
    csr_sr &= ~SR_PEI;
    csr_sr |= (csr_sr & SR_EI) ? SR_PEI : 0;
    csr_sr &= ~SR_EI;

    // Supervisor mode save and enable
    csr_sr &= ~SR_PS;
    csr_sr |= (csr_sr & SR_S) ? SR_PS : 0;
    csr_sr |= SR_S;
    
    csr_epc = pc;

    csr_cause = cause;
}
//-----------------------------------------------------------------
// Execute: Instruction execution stage
//-----------------------------------------------------------------
void Riscv::Execute(void)
{
    // Get opcode at current PC
    uint32_t opcode = GetOpcode(r_pc);

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

    DPRINTF(LOG_OPCODES,( "%08x: %08x\n", pc, opcode));
    DPRINTF(LOG_OPCODES,( "        rd(%d) r%d = %d, r%d = %d\n", rd, rs1, reg_rs1, rs2, reg_rs2));    

    // As RVC is not supported, fault on opcode which is all zeros
    if (opcode == 0)
    {
        fprintf(stderr, "Bad instruction @ %x\n", pc);

        Exception(CAUSE_ILLEGAL_INSTRUCTION, pc);
        Fault = true;
        exception = true;        
    }
    else if ((opcode & INST_ANDI_MASK) == INST_ANDI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: andi r%d, r%d, %d\n", pc, rd, rs1, imm12));
        reg_rd = reg_rs1 & imm12;
        pc += 4;        
    }
    else if ((opcode & INST_ORI_MASK) == INST_ORI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: ori r%d, r%d, %d\n", pc, rd, rs1, imm12));
        reg_rd = reg_rs1 | imm12;
        pc += 4;        
    }
    else if ((opcode & INST_XORI_MASK) == INST_XORI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: xori r%d, r%d, %d\n", pc, rd, rs1, imm12));
        reg_rd = reg_rs1 ^ imm12;
        pc += 4;        
    }
    else if ((opcode & INST_ADDI_MASK) == INST_ADDI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: addi r%d, r%d, %d\n", pc, rd, rs1, imm12));
        reg_rd = reg_rs1 + imm12;
        pc += 4;
    }
    else if ((opcode & INST_SLTI_MASK) == INST_SLTI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: slti r%d, r%d, %d\n", pc, rd, rs1, imm12));
        reg_rd = (signed)reg_rs1 < (signed)imm12;
        pc += 4;        
    }
    else if ((opcode & INST_SLTIU_MASK) == INST_SLTIU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: sltiu r%d, r%d, %d\n", pc, rd, rs1, (unsigned)imm12));
        reg_rd = (unsigned)reg_rs1 < (unsigned)imm12;
        pc += 4;        
    }
    else if ((opcode & INST_SLLI_MASK) == INST_SLLI)
    {
        // ['rd', 'rs1']
        DPRINTF(LOG_INST,("%08x: slli r%d, r%d, %d\n", pc, rd, rs1, shamt));
        reg_rd = reg_rs1 << shamt;
        pc += 4;        
    }
    else if ((opcode & INST_SRLI_MASK) == INST_SRLI)
    {
        // ['rd', 'rs1', 'shamt']
        DPRINTF(LOG_INST,("%08x: srli r%d, r%d, %d\n", pc, rd, rs1, shamt));
        reg_rd = (unsigned)reg_rs1 >> shamt;
        pc += 4;        
    }
    else if ((opcode & INST_SRAI_MASK) == INST_SRAI)
    {
        // ['rd', 'rs1', 'shamt']
        DPRINTF(LOG_INST,("%08x: srai r%d, r%d, %d\n", pc, rd, rs1, shamt));
        reg_rd = (signed)reg_rs1 >> shamt;
        pc += 4;        
    }
    else if ((opcode & INST_LUI_MASK) == INST_LUI)
    {
        // ['rd', 'imm20']
        DPRINTF(LOG_INST,("%08x: lui r%d, 0x%x\n", pc, rd, imm20));
        reg_rd = imm20;
        pc += 4;        
    }
    else if ((opcode & INST_AUIPC_MASK) == INST_AUIPC)
    {
        // ['rd', 'imm20']
        DPRINTF(LOG_INST,("%08x: auipc r%d, 0x%x\n", pc, rd, imm20));
        reg_rd = imm20 + pc;
        pc += 4;        
    }
    else if ((opcode & INST_ADD_MASK) == INST_ADD)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: add r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = reg_rs1 + reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SUB_MASK) == INST_SUB)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sub r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = reg_rs1 - reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SLT_MASK) == INST_SLT)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: slt r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = (signed)reg_rs1 < (signed)reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SLTU_MASK) == INST_SLTU)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sltu r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = (unsigned)reg_rs1 < (unsigned)reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_XOR_MASK) == INST_XOR)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: xor r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = reg_rs1 ^ reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_OR_MASK) == INST_OR)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: or r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = reg_rs1 | reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_AND_MASK) == INST_AND)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: and r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = reg_rs1 & reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SLL_MASK) == INST_SLL)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sll r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = reg_rs1 << reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SRL_MASK) == INST_SRL)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: srl r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = (unsigned)reg_rs1 >> reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SRA_MASK) == INST_SRA)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sra r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = (signed)reg_rs1 >> reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_JAL_MASK) == INST_JAL)
    {
        // ['rd', 'jimm20']
        DPRINTF(LOG_INST,("%08x: jal r%d, %d\n", pc, rd, jimm20));
        reg_rd = pc + 4;
        pc+= jimm20;

        Stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_JALR_MASK) == INST_JALR)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: jalr r%d, r%d\n", pc, rs1, imm12));

        reg_rd = pc + 4;
        pc = (reg_rs1 + imm12) & ~1;

        Stats[STATS_BRANCHES]++;        
    }
    else if ((opcode & INST_BEQ_MASK) == INST_BEQ)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: beq r%d, r%d, %d\n", pc, rs1, rs2, bimm));
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
        reg_rd = Load(reg_rs1 + imm12, 1, true);
        pc += 4;
    }
    else if ((opcode & INST_LH_MASK) == INST_LH)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lh r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        reg_rd = Load(reg_rs1 + imm12, 2, true);
        pc += 4;
    }
    else if ((opcode & INST_LW_MASK) == INST_LW)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lw r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        reg_rd = Load(reg_rs1 + imm12, 4, true);
        pc += 4;        
    }
    else if ((opcode & INST_LBU_MASK) == INST_LBU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lbu r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        reg_rd = Load(reg_rs1 + imm12, 1, false);
        pc += 4;
    }
    else if ((opcode & INST_LHU_MASK) == INST_LHU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lhu r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        reg_rd = Load(reg_rs1 + imm12, 2, false);
        pc += 4;
    }
    else if ((opcode & INST_LWU_MASK) == INST_LWU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lwu r%d, %d(r%d)\n", pc, rd, imm12, rs1));
        reg_rd = Load(reg_rs1 + imm12, 4, false);
        pc += 4;
    }
    else if ((opcode & INST_SB_MASK) == INST_SB)
    {
        // ['imm12hi', 'rs1', 'rs2', 'imm12lo']
        DPRINTF(LOG_INST,("%08x: sb %d(r%d), r%d\n", pc, storeimm, rs1, rs2));
        Store(reg_rs1 + storeimm, reg_rs2, 1);
        pc += 4;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_SH_MASK) == INST_SH)
    {
        // ['imm12hi', 'rs1', 'rs2', 'imm12lo']
        DPRINTF(LOG_INST,("%08x: sh %d(r%d), r%d\n", pc, storeimm, rs1, rs2));
        Store(reg_rs1 + storeimm, reg_rs2, 2);
        pc += 4;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_SW_MASK) == INST_SW)
    {
        // ['imm12hi', 'rs1', 'rs2', 'imm12lo']
        DPRINTF(LOG_INST,("%08x: sw %d(r%d), r%d\n", pc, storeimm, rs1, rs2));
        Store(reg_rs1 + storeimm, reg_rs2, 4);
        pc += 4;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_MUL_MASK) == INST_MUL)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: mul r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = (signed)reg_rs1 * (signed)reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_MULH_MASK) == INST_MULH)
    {
        // ['rd', 'rs1', 'rs2']
        long long res = ((long long) (int)reg_rs1) * ((long long)(int)reg_rs2);
        DPRINTF(LOG_INST,("%08x: mulh r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = (int)(res >> 32);
        pc += 4;
    }
    else if ((opcode & INST_MULHSU_MASK) == INST_MULHSU)
    {
        // ['rd', 'rs1', 'rs2']
        long long res = ((long long) (int)reg_rs1) * ((unsigned long long)(unsigned)reg_rs2);
        DPRINTF(LOG_INST,("%08x: mulhsu r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = (int)(res >> 32);
        pc += 4;
    }
    else if ((opcode & INST_MULHU_MASK) == INST_MULHU)
    {
        // ['rd', 'rs1', 'rs2']
        unsigned long long res = ((unsigned long long) (unsigned)reg_rs1) * ((unsigned long long)(unsigned)reg_rs2);
        DPRINTF(LOG_INST,("%08x: mulhu r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        reg_rd = (int)(res >> 32);
        pc += 4;
    }
    else if ((opcode & INST_DIV_MASK) == INST_DIV)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: div r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        if (reg_rs2 != 0)
            reg_rd = (signed)reg_rs1 / (signed)reg_rs2;
        else
            reg_rd = (unsigned)-1;
        pc += 4;        
    }
    else if ((opcode & INST_DIVU_MASK) == INST_DIVU)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: divu r%d, r%d, r%d\n", pc, rd, rs1, rs2));
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
        if (reg_rs2 != 0)
            reg_rd = (signed)reg_rs1 % (signed)reg_rs2;
        else
            reg_rd = reg_rs1;
        pc += 4;        
    }
    else if ((opcode & INST_REMU_MASK) == INST_REMU)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: remu r%d, r%d, r%d\n", pc, rd, rs1, rs2));
        if (reg_rs2 != 0)
            reg_rd = (unsigned)reg_rs1 % (unsigned)reg_rs2;
        else
            reg_rd = reg_rs1;
        pc += 4;        
    }
    else if ((opcode & INST_SCALL_MASK) == INST_SCALL)
    {
        DPRINTF(LOG_INST,("%08x: scall\n", pc));
        
        Exception(CAUSE_SYSCALL, pc);

        pc          = csr_evec;
        exception   = true;
    }
    else if ((opcode & INST_SBREAK_MASK) == INST_SBREAK)
    {
        DPRINTF(LOG_INST,("%08x: sbreak\n", pc));

        Exception(CAUSE_BREAKPOINT, pc);

        pc          = csr_evec;
        exception   = true;
        Break       = true;
    }
    else if ((opcode & INST_SRET_MASK) == INST_SRET)
    {
        DPRINTF(LOG_INST,("%08x: sret\n", pc));

        // Interrupt enable pop
        csr_sr &= ~SR_EI;
        csr_sr |= (csr_sr & SR_PEI) ? SR_EI : 0;

        // Supervisor pop
        csr_sr &= ~SR_S;
        csr_sr |= (csr_sr & SR_PS) ? SR_S : 0;

        // Return to EPC
        pc          = csr_epc;        
    }
    else if ((opcode & INST_CSRRW_MASK) == INST_CSRRW)
    {
        DPRINTF(LOG_INST,("%08x: csrw r%d, r%d, 0x%x\n", pc, rd, rs1, imm12));
        reg_rd = AccessCsr(imm12, reg_rs1, true, true);
        pc += 4;
    }    
    else if ((opcode & INST_CSRRS_MASK) == INST_CSRRS)
    {
        DPRINTF(LOG_INST,("%08x: csrs r%d, r%d, 0x%x\n", pc, rd, rs1, imm12));
        reg_rd = AccessCsr(imm12, reg_rs1, true, false);
        pc += 4;
    }
    else if ((opcode & INST_CSRRC_MASK) == INST_CSRRC)
    {
        DPRINTF(LOG_INST,("%08x: csrc r%d, r%d, 0x%x\n", pc, rd, rs1, imm12));
        reg_rd = AccessCsr(imm12, reg_rs1, false, true);
        pc += 4;
    }    
    else if ((opcode & INST_CSRRSI_MASK) == INST_CSRRSI)
    {
        reg_rd = AccessCsr(imm12, rs1, true, false);
        pc += 4;
    }
    else if ((opcode & INST_CSRRCI_MASK) == INST_CSRRCI)
    {
        reg_rd = AccessCsr(imm12, rs1, false, true);
        pc += 4;
    }    
    else
    {
        fprintf(stderr, "Bad instruction @ %x (opcode %x)\n", pc, opcode);
        Exception(CAUSE_ILLEGAL_INSTRUCTION, pc);
        Fault = true;
        exception = true;
    }

    if (rd != 0)
        r_gpr[rd] = reg_rd;

    // Interrupt pending and enabled
    if (InterruptPending && (csr_sr & SR_EI) && !exception)
    {
        DPRINTF(LOG_INST,( "Interrupt taken...\n"));
        InterruptPending = false;

        Exception(CAUSE_INTERRUPT, pc);

        pc          = csr_evec;
    }

    r_pc = pc;
}
//-----------------------------------------------------------------
// Step: Step through one instruction
//-----------------------------------------------------------------
bool Riscv::Step(void)
{
    Stats[STATS_INSTRUCTIONS]++;

    // Execute instruction at current PC
    Execute();

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

    return true;
}
//-----------------------------------------------------------------
// Interrupt: Register pending interrupt
//-----------------------------------------------------------------
void Riscv::Interrupt(void)
{
    InterruptPending = true;
}
//-----------------------------------------------------------------
// DumpStats: Show execution stats
//-----------------------------------------------------------------
void Riscv::DumpStats(void)
{
    printf( "Runtime Stats:\n");
    printf( "- Total Instructions %d\n", Stats[STATS_INSTRUCTIONS]);
    if (Stats[STATS_INSTRUCTIONS] > 0)
    {
        printf( "- Loads %d (%d%%)\n",  Stats[STATS_LOADS],  (Stats[STATS_LOADS] * 100)  / Stats[STATS_INSTRUCTIONS]);
        printf( "- Stores %d (%d%%)\n", Stats[STATS_STORES], (Stats[STATS_STORES] * 100) / Stats[STATS_INSTRUCTIONS]);
        printf( "- Branches Operations %d (%d%%)\n", Stats[STATS_BRANCHES], (Stats[STATS_BRANCHES] * 100)  / Stats[STATS_INSTRUCTIONS]);
    }

    ResetStats();
}
