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
#include <stdint.h>
#include "riscv_isa.h"
#include "riscv_inst_dump.h"

//-----------------------------------------------------------------
// riscv_inst_decode: Instruction decode to string
//-----------------------------------------------------------------
bool riscv_inst_decode(char *str, uint32_t pc, uint32_t opcode)
{
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

    if ((opcode & INST_ANDI_MASK) == INST_ANDI)
    {
        // ['rd', 'rs1', 'imm12']
        sprintf(str, "%08x: andi r%d, r%d, %d", pc, rd, rs1, imm12);     
    }
    else if ((opcode & INST_ORI_MASK) == INST_ORI)
    {
        // ['rd', 'rs1', 'imm12']
        sprintf(str, "%08x: ori r%d, r%d, %d", pc, rd, rs1, imm12);
    }
    else if ((opcode & INST_XORI_MASK) == INST_XORI)
    {
        // ['rd', 'rs1', 'imm12']
        sprintf(str, "%08x: xori r%d, r%d, %d", pc, rd, rs1, imm12);
    }
    else if ((opcode & INST_ADDI_MASK) == INST_ADDI)
    {
        // ['rd', 'rs1', 'imm12']
        sprintf(str, "%08x: addi r%d, r%d, %d", pc, rd, rs1, imm12);
    }
    else if ((opcode & INST_SLTI_MASK) == INST_SLTI)
    {
        // ['rd', 'rs1', 'imm12']
        sprintf(str, "%08x: slti r%d, r%d, %d", pc, rd, rs1, imm12);
    }
    else if ((opcode & INST_SLTIU_MASK) == INST_SLTIU)
    {
        // ['rd', 'rs1', 'imm12']
        sprintf(str, "%08x: sltiu r%d, r%d, %d", pc, rd, rs1, (unsigned)imm12);
    }
    else if ((opcode & INST_SLLI_MASK) == INST_SLLI)
    {
        // ['rd', 'rs1']
        sprintf(str, "%08x: slli r%d, r%d, %d", pc, rd, rs1, shamt);
    }
    else if ((opcode & INST_SRLI_MASK) == INST_SRLI)
    {
        // ['rd', 'rs1', 'shamt']
        sprintf(str, "%08x: srli r%d, r%d, %d", pc, rd, rs1, shamt);
    }
    else if ((opcode & INST_SRAI_MASK) == INST_SRAI)
    {
        // ['rd', 'rs1', 'shamt']
        sprintf(str, "%08x: srai r%d, r%d, %d", pc, rd, rs1, shamt);
    }
    else if ((opcode & INST_LUI_MASK) == INST_LUI)
    {
        // ['rd', 'imm20']
        sprintf(str, "%08x: lui r%d, 0x%x", pc, rd, imm20);
    }
    else if ((opcode & INST_AUIPC_MASK) == INST_AUIPC)
    {
        // ['rd', 'imm20']
        sprintf(str, "%08x: auipc r%d, 0x%x", pc, rd, imm20);
    }
    else if ((opcode & INST_ADD_MASK) == INST_ADD)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: add r%d, r%d, r%d", pc, rd, rs1, rs2);
    }
    else if ((opcode & INST_SUB_MASK) == INST_SUB)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: sub r%d, r%d, r%d", pc, rd, rs1, rs2);
    }
    else if ((opcode & INST_SLT_MASK) == INST_SLT)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: slt r%d, r%d, r%d", pc, rd, rs1, rs2);
    }
    else if ((opcode & INST_SLTU_MASK) == INST_SLTU)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: sltu r%d, r%d, r%d", pc, rd, rs1, rs2);
    }
    else if ((opcode & INST_XOR_MASK) == INST_XOR)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: xor r%d, r%d, r%d", pc, rd, rs1, rs2);
    }
    else if ((opcode & INST_OR_MASK) == INST_OR)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: or r%d, r%d, r%d", pc, rd, rs1, rs2);
    }
    else if ((opcode & INST_AND_MASK) == INST_AND)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: and r%d, r%d, r%d", pc, rd, rs1, rs2);
    }
    else if ((opcode & INST_SLL_MASK) == INST_SLL)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: sll r%d, r%d, r%d", pc, rd, rs1, rs2);
    }
    else if ((opcode & INST_SRL_MASK) == INST_SRL)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: srl r%d, r%d, r%d", pc, rd, rs1, rs2);
    }
    else if ((opcode & INST_SRA_MASK) == INST_SRA)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: sra r%d, r%d, r%d", pc, rd, rs1, rs2);
    }
    else if ((opcode & INST_JAL_MASK) == INST_JAL)
    {
        // ['rd', 'jimm20']
        sprintf(str, "%08x: jal r%d, %d", pc, rd, jimm20);
    }
    else if ((opcode & INST_JALR_MASK) == INST_JALR)
    {
        // ['rd', 'rs1', 'imm12']
        sprintf(str, "%08x: jalr r%d, r%d", pc, rs1, imm12);
    }
    else if ((opcode & INST_BEQ_MASK) == INST_BEQ)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        sprintf(str, "%08x: beq r%d, r%d, %d", pc, rs1, rs2, bimm);
    }
    else if ((opcode & INST_BNE_MASK) == INST_BNE)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        sprintf(str, "%08x: bne r%d, r%d, %d", pc, rs1, rs2, bimm);  
    }
    else if ((opcode & INST_BLT_MASK) == INST_BLT)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        sprintf(str, "%08x: blt r%d, r%d, %d", pc, rs1, rs2, bimm);
    }
    else if ((opcode & INST_BGE_MASK) == INST_BGE)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        sprintf(str, "%08x: bge r%d, r%d, %d", pc, rs1, rs2, bimm);
    }
    else if ((opcode & INST_BLTU_MASK) == INST_BLTU)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        sprintf(str, "%08x: bltu r%d, r%d, %d", pc, rs1, rs2, bimm);
    }
    else if ((opcode & INST_BGEU_MASK) == INST_BGEU)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        sprintf(str, "%08x: bgeu r%d, r%d, %d", pc, rs1, rs2, bimm);
    }
    else if ((opcode & INST_LB_MASK) == INST_LB)
    {
        // ['rd', 'rs1', 'imm12']
        sprintf(str, "%08x: lb r%d, %d(r%d)", pc, rd, imm12, rs1);
    }
    else if ((opcode & INST_LH_MASK) == INST_LH)
    {
        // ['rd', 'rs1', 'imm12']
        sprintf(str, "%08x: lh r%d, %d(r%d)", pc, rd, imm12, rs1);
    }
    else if ((opcode & INST_LW_MASK) == INST_LW)
    {
        // ['rd', 'rs1', 'imm12']        
        sprintf(str, "%08x: lw r%d, %d(r%d)", pc, rd, imm12, rs1);
    }
    else if ((opcode & INST_LBU_MASK) == INST_LBU)
    {
        // ['rd', 'rs1', 'imm12']
        sprintf(str, "%08x: lbu r%d, %d(r%d)", pc, rd, imm12, rs1);
    }
    else if ((opcode & INST_LHU_MASK) == INST_LHU)
    {
        // ['rd', 'rs1', 'imm12']
        sprintf(str, "%08x: lhu r%d, %d(r%d)", pc, rd, imm12, rs1);
    }
    else if ((opcode & INST_LWU_MASK) == INST_LWU)
    {
        // ['rd', 'rs1', 'imm12']
        sprintf(str, "%08x: lwu r%d, %d(r%d)", pc, rd, imm12, rs1);
    }
    else if ((opcode & INST_SB_MASK) == INST_SB)
    {
        // ['imm12hi', 'rs1', 'rs2', 'imm12lo']
        sprintf(str, "%08x: sb %d(r%d), r%d", pc, storeimm, rs1, rs2);
    }
    else if ((opcode & INST_SH_MASK) == INST_SH)
    {
        // ['imm12hi', 'rs1', 'rs2', 'imm12lo']
        sprintf(str, "%08x: sh %d(r%d), r%d", pc, storeimm, rs1, rs2);
    }
    else if ((opcode & INST_SW_MASK) == INST_SW)
    {
        // ['imm12hi', 'rs1', 'rs2', 'imm12lo']
        sprintf(str, "%08x: sw %d(r%d), r%d", pc, storeimm, rs1, rs2);
    }
    else if ((opcode & INST_MUL_MASK) == INST_MUL)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: mul r%d, r%d, r%d", pc, rd, rs1, rs2);   
    }
    else if ((opcode & INST_MULH_MASK) == INST_MULH)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: mulh r%d, r%d, r%d", pc, rd, rs1, rs2);
    }
    else if ((opcode & INST_MULHSU_MASK) == INST_MULHSU)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: mulhsu r%d, r%d, r%d", pc, rd, rs1, rs2);
    }
    else if ((opcode & INST_MULHU_MASK) == INST_MULHU)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: mulhu r%d, r%d, r%d", pc, rd, rs1, rs2);
    }
    else if ((opcode & INST_DIV_MASK) == INST_DIV)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: div r%d, r%d, r%d", pc, rd, rs1, rs2);     
    }
    else if ((opcode & INST_DIVU_MASK) == INST_DIVU)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: divu r%d, r%d, r%d", pc, rd, rs1, rs2); 
    }
    else if ((opcode & INST_REM_MASK) == INST_REM)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: rem r%d, r%d, r%d", pc, rd, rs1, rs2);
    }
    else if ((opcode & INST_REMU_MASK) == INST_REMU)
    {
        // ['rd', 'rs1', 'rs2']
        sprintf(str, "%08x: remu r%d, r%d, r%d", pc, rd, rs1, rs2);    
    }
    else if ((opcode & INST_ECALL_MASK) == INST_ECALL)
    {
        sprintf(str, "%08x: ecall", pc);
    }
    else if ((opcode & INST_EBREAK_MASK) == INST_EBREAK)
    {
        sprintf(str, "%08x: ebreak", pc);
    }
    else if ((opcode & INST_MRET_MASK) == INST_MRET)
    {
        sprintf(str, "%08x: mret", pc);
    }
    else if ((opcode & INST_SRET_MASK) == INST_SRET)
    {
        sprintf(str, "%08x: sret", pc);
    }
    else if ( ((opcode & INST_SFENCE_MASK) == INST_SFENCE) ||
              ((opcode & INST_FENCE_MASK) == INST_FENCE) ||
              ((opcode & INST_IFENCE_MASK) == INST_IFENCE))
    {
        sprintf(str, "%08x: sfence", pc);
    }
    else if ((opcode & INST_CSRRW_MASK) == INST_CSRRW)
    {
        sprintf(str, "%08x: csrw r%d, r%d, 0x%x", pc, rd, rs1, imm12);
    }    
    else if ((opcode & INST_CSRRS_MASK) == INST_CSRRS)
    {
        sprintf(str, "%08x: csrs r%d, r%d, 0x%x", pc, rd, rs1, imm12);
    }
    else if ((opcode & INST_CSRRC_MASK) == INST_CSRRC)
    {
        sprintf(str, "%08x: csrc r%d, r%d, 0x%x", pc, rd, rs1, imm12);
    }
    else if ((opcode & INST_CSRRWI_MASK) == INST_CSRRWI)
    {
        sprintf(str, "%08x: csrwi r%d, %d, 0x%x", pc, rd, rs1, imm12);
    }
    else if ((opcode & INST_CSRRSI_MASK) == INST_CSRRSI)
    {
        sprintf(str, "%08x: csrsi r%d, %d, 0x%x", pc, rd, rs1, imm12);
    }
    else if ((opcode & INST_CSRRCI_MASK) == INST_CSRRCI)
    {
        sprintf(str, "%08x: csrci r%d, %d, 0x%x", pc, rd, rs1, imm12);
    }
    else if ((opcode & INST_WFI_MASK) == INST_WFI)
    {
        sprintf(str, "%08x: wfi", pc);
    }
    else
    {
        sprintf(str, "%08x: invalid!", pc);
        return false;
    }

   return true;
}
//-----------------------------------------------------------------
// riscv_inst_print: Instruction decode to string
//-----------------------------------------------------------------
void riscv_inst_print(uint32_t pc, uint32_t opcode)
{
    char str[1024];
    riscv_inst_decode(str, pc, opcode);
    printf("%s\n", str);
}
