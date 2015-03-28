#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <vpi_user.h>

#include "riscv_isa.h"

//--------------------------------------------------------------------
// Defines:
//--------------------------------------------------------------------
#define LOG_INST            (1 << 0)
#define LOG_OPCODES         (1 << 1)
#define LOG_REGISTERS       (1 << 2)
#define LOG_MEM             (1 << 3)

#if VERBOSE > 0
#define DPRINTF(l,a)        printf a
#else
#define DPRINTF(l,a)
#endif

#define ASSERT(exp)         do { if (!(exp)) { vpi_printf("ASSERT: %s\n\n", #exp ); vpi_control(vpiFinish, 1); } } while (0)

#define PC_MASK             0xFFFFFFFF

//-----------------------------------------------------------------
// Locals
//-----------------------------------------------------------------
static uint32_t r_gpr[REGISTERS];
static uint32_t r_pc;
static uint32_t csr_epc;
static uint32_t csr_cause;
static uint32_t csr_sr;
static uint32_t csr_evec;
static uint32_t csr_count;
static uint32_t csr_compare;
static uint32_t csr_mem[CSR_MAX_ADDR];
static int      Fault;
static int      Break;
static uint32_t load_val = 0;

//-----------------------------------------------------------------
// GetRegName:
//-----------------------------------------------------------------
static const char * GetRegName(int regnum)
{
    switch (regnum & (REGISTERS-1))
    {
        case 0:  return  "zero";
        case 1:  return  "ra";
        case 2:  return  "s0";
        case 3:  return  "s1";
        case 4:  return  "s2";
        case 5:  return  "s3";
        case 6:  return  "s4";
        case 7:  return  "s5";
        case 8:  return  "s6";
        case 9:  return  "s7";
        case 10: return  "s8";
        case 11: return  "s9";
        case 12: return  "s10";
        case 13: return  "s11";
        case 14: return  "sp";
        case 15: return  "tp";
        case 16: return  "v0";
        case 17: return  "v1";
        case 18: return  "a0";
        case 19: return  "a1";
        case 20: return  "a2";
        case 21: return  "a3";
        case 22: return  "a4";
        case 23: return  "a5";
        case 24: return  "a6";
        case 25: return  "a7";
        case 26: return  "t0";
        case 27: return  "t1";
        case 28: return  "t2";
        case 29: return  "t3";
        case 30: return  "t4";
        case 31: return  "gp";
    }

    return "-";
}
//-----------------------------------------------------------------
// Init:
//-----------------------------------------------------------------
static void Init(void)
{
    int i;

    r_pc = 0x100;

    for (i=0;i<REGISTERS;i++)
        r_gpr[i] = 0;

    csr_epc     = 0;
    csr_sr      = SR_S;
    csr_cause   = 0;
    csr_evec    = 0;
    csr_count   = 0;
    csr_compare = 0;

    for (i=0;i<CSR_MAX_ADDR;i++)
        csr_mem[i] = 0;

    Fault       = 0;
    Break       = 0;    
}
//-----------------------------------------------------------------
// Load:
//-----------------------------------------------------------------
static uint32_t Load(uint32_t pc, uint32_t address, int width, int signedLoad)
{
    uint32_t data = 0;

    switch (width)
    {
        case 4:
            data = load_val++;
        break;
        case 2:
            data = load_val++;

            if (address & 2)
                data = (data >> 16)  & 0xFFFF;
            else
                data = (data >> 0) & 0xFFFF;

            if (signedLoad)
                if (data & (1 << 15))
                    data |= 0xFFFF0000;
        break;
        case 1:
            data = load_val++;
            switch (address & 3)
            {
                case 3:
                    data = (data >> 24) & 0xFF;
                break;
                case 2:
                    data = (data >> 16) & 0xFF;
                break;
                case 1:
                    data = (data >> 8) & 0xFF;
                break;
                case 0:
                    data = (data >> 0) & 0xFF;
                break;
            }

            if (signedLoad)
                if (data & (1 << 7))
                    data |= 0xFFFFFF00;
        break;
    }

    return data;    
}
//-----------------------------------------------------------------
// Store:
//-----------------------------------------------------------------
static void Store(uint32_t pc, uint32_t address, uint32_t data, int width)
{

}
//-----------------------------------------------------------------
// AccessCsr:
//-----------------------------------------------------------------
static uint32_t AccessCsr(uint32_t address, uint32_t data, int set, int clr)
{
    uint32_t result = 0;

    address &= (CSR_MAX_ADDR-1);

    switch (address)
    {
        case CSR_EPC:
            data       &= CSR_EPC_MASK;
            result      = csr_epc;
            if (set && clr)
                csr_epc = data;
            else if (set)
                csr_epc    |= data;
            else if (clr)
                csr_epc    &= ~data;
            break;
        case CSR_EVEC:
            result      = 0;
            break;
        case CSR_CAUSE:
            data       &= CSR_CAUSE_MASK;
            result      = csr_cause;
            if (set && clr)
                csr_cause = data;
            else if (set)
                csr_cause    |= data;
            else if (clr)
                csr_cause    &= ~data;
            break;
        case CSR_STATUS:
            data       &= CSR_STATUS_MASK;
            result      = csr_sr;
            if (set && clr)
                csr_sr = (csr_sr & SR_IP) | (data & ~SR_IP);
            else if (set)
                csr_sr |= (data & ~SR_IP);
            else if (clr)
                csr_sr &= ~(data & ~SR_IP);
            break;
        default:
            printf("CSR: Addr %x\n", address);
            assert(address < CSR_MAX_ADDR);

            result                = csr_mem[address];
            if (set && clr)
                csr_mem[address]  = data;
            else if (set)
                csr_mem[address] |= data;
            else if (clr)
                csr_mem[address] &= ~data;
            printf("CSR: Addr %x = %x\n", address, result);
            break;
    }
    return result;
}
//-----------------------------------------------------------------
// Exception:
//-----------------------------------------------------------------
static void Exception(uint32_t cause, uint32_t pc)
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
static void Execute(uint32_t pc, uint32_t opcode, int intr)
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

    // Retrieve registers
    uint32_t reg_rd  = 0;
    uint32_t reg_rs1 = r_gpr[rs1];
    uint32_t reg_rs2 = r_gpr[rs2];

    Break = 0;

    DPRINTF(LOG_OPCODES,( "%08x: %08x\n", pc, opcode));
    DPRINTF(LOG_INST,("%08x: %s=0x%x, %s=0x%x\n", pc, GetRegName(rs1), reg_rs1, GetRegName(rs2), reg_rs2));

    // Interrupts enabled and one requested
    if ((csr_sr & SR_EI) && intr)
    {
        // If not higher priority exception
        if (((opcode & INST_SCALL_MASK) != INST_SCALL) &&
            ((opcode & INST_SBREAK_MASK) != INST_SBREAK))
        {
            DPRINTF(LOG_INST,( "Interrupt taken...\n"));
            Exception(CAUSE_INTERRUPT, pc);

            r_pc          = csr_evec;
            return ;
        }
    }

    if ((opcode & INST_ANDI_MASK) == INST_ANDI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: andi %s, %s, %d\n", pc, GetRegName(rd), GetRegName(rs1), imm12));
        reg_rd = reg_rs1 & imm12;
        pc += 4;        
    }
    else if ((opcode & INST_ORI_MASK) == INST_ORI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: ori %s, %s, %d\n", pc, GetRegName(rd), GetRegName(rs1), imm12));
        reg_rd = reg_rs1 | imm12;
        pc += 4;        
    }
    else if ((opcode & INST_XORI_MASK) == INST_XORI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: xori %s, %s, %d\n", pc, GetRegName(rd), GetRegName(rs1), imm12));
        reg_rd = reg_rs1 ^ imm12;
        pc += 4;        
    }
    else if ((opcode & INST_ADDI_MASK) == INST_ADDI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: addi %s, %s, %d\n", pc, GetRegName(rd), GetRegName(rs1), imm12));
        reg_rd = reg_rs1 + imm12;
        pc += 4;
    }
    else if ((opcode & INST_SLTI_MASK) == INST_SLTI)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: slti %s, %s, %d\n", pc, GetRegName(rd), GetRegName(rs1), imm12));
        reg_rd = (signed)reg_rs1 < (signed)imm12;
        pc += 4;        
    }
    else if ((opcode & INST_SLTIU_MASK) == INST_SLTIU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: sltiu %s, %s, %d\n", pc, GetRegName(rd), GetRegName(rs1), (unsigned)imm12));
        reg_rd = (unsigned)reg_rs1 < (unsigned)imm12;
        pc += 4;        
    }
    else if ((opcode & INST_SLLI_MASK) == INST_SLLI)
    {
        // ['rd', 'rs1']
        DPRINTF(LOG_INST,("%08x: slli %s, %s, %d\n", pc, GetRegName(rd), GetRegName(rs1), shamt));
        reg_rd = reg_rs1 << shamt;
        pc += 4;        
    }
    else if ((opcode & INST_SRLI_MASK) == INST_SRLI)
    {
        // ['rd', 'rs1', 'shamt']
        DPRINTF(LOG_INST,("%08x: srli %s, %s, %d\n", pc, GetRegName(rd), GetRegName(rs1), shamt));
        reg_rd = (unsigned)reg_rs1 >> shamt;
        pc += 4;        
    }
    else if ((opcode & INST_SRAI_MASK) == INST_SRAI)
    {
        // ['rd', 'rs1', 'shamt']
        DPRINTF(LOG_INST,("%08x: srai %s, %s, %d\n", pc, GetRegName(rd), GetRegName(rs1), shamt));
        reg_rd = (signed)reg_rs1 >> shamt;
        pc += 4;        
    }
    else if ((opcode & INST_LUI_MASK) == INST_LUI)
    {
        // ['rd', 'imm20']
        DPRINTF(LOG_INST,("%08x: lui %s, 0x%x\n", pc, GetRegName(rd), imm20));
        reg_rd = imm20;
        pc += 4;        
    }
    else if ((opcode & INST_AUIPC_MASK) == INST_AUIPC)
    {
        // ['rd', 'imm20']
        DPRINTF(LOG_INST,("%08x: auipc %s, 0x%x\n", pc, GetRegName(rd), imm20));
        reg_rd = imm20 + pc;
        pc += 4;        
    }
    else if ((opcode & INST_ADD_MASK) == INST_ADD)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: add %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        reg_rd = reg_rs1 + reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SUB_MASK) == INST_SUB)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sub %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        reg_rd = reg_rs1 - reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SLT_MASK) == INST_SLT)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: slt %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        reg_rd = (signed)reg_rs1 < (signed)reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SLTU_MASK) == INST_SLTU)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sltu %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        reg_rd = (unsigned)reg_rs1 < (unsigned)reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_XOR_MASK) == INST_XOR)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: xor %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        reg_rd = reg_rs1 ^ reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_OR_MASK) == INST_OR)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: or %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        reg_rd = reg_rs1 | reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_AND_MASK) == INST_AND)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: and %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        reg_rd = reg_rs1 & reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SLL_MASK) == INST_SLL)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sll %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        reg_rd = reg_rs1 << reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SRL_MASK) == INST_SRL)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: srl %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        reg_rd = (unsigned)reg_rs1 >> reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_SRA_MASK) == INST_SRA)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: sra %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        reg_rd = (signed)reg_rs1 >> reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_JAL_MASK) == INST_JAL)
    {
        // ['rd', 'jimm20']
        DPRINTF(LOG_INST,("%08x: jal %s, %d\n", pc, GetRegName(rd), jimm20));
        reg_rd = pc + 4;
        pc+= jimm20;
    }
    else if ((opcode & INST_JALR_MASK) == INST_JALR)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: jalr %s, %d\n", pc, GetRegName(rs1), imm12));
        reg_rd = pc + 4;
        pc = (reg_rs1 + imm12) & ~1;
    }
    else if ((opcode & INST_BEQ_MASK) == INST_BEQ)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: beq %s, %s, %d\n", pc, GetRegName(rs1), GetRegName(rs2), bimm));
        if (reg_rs1 == reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_BNE_MASK) == INST_BNE)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: bne %s, %s, %d\n", pc, GetRegName(rs1), GetRegName(rs2), bimm));
        if (reg_rs1 != reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_BLT_MASK) == INST_BLT)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: blt %s, %s, %d\n", pc, GetRegName(rs1), GetRegName(rs2), bimm));
        if ((signed)reg_rs1 < (signed)reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_BGE_MASK) == INST_BGE)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: bge %s, %s, %d\n", pc, GetRegName(rs1), GetRegName(rs2), bimm));
        if ((signed)reg_rs1 >= (signed)reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_BLTU_MASK) == INST_BLTU)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: bltu %s, %s, %d\n", pc, GetRegName(rs1), GetRegName(rs2), bimm));
        if ((unsigned)reg_rs1 < (unsigned)reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_BGEU_MASK) == INST_BGEU)
    {
        // ['bimm12hi', 'rs1', 'rs2', 'bimm12lo']
        DPRINTF(LOG_INST,("%08x: bgeu %s, %s, %d\n", pc, GetRegName(rs1), GetRegName(rs2), bimm));
        if ((unsigned)reg_rs1 >= (unsigned)reg_rs2)
            pc += bimm;
        else
            pc += 4;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_LB_MASK) == INST_LB)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lb %s, %d(%s)\n", pc, GetRegName(rd), imm12, GetRegName(rs1)));
        reg_rd = Load(pc, reg_rs1 + imm12, 1, 1);
        pc += 4;
    }
    else if ((opcode & INST_LH_MASK) == INST_LH)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lh %s, %d(%s)\n", pc, GetRegName(rd), imm12, GetRegName(rs1)));
        reg_rd = Load(pc, reg_rs1 + imm12, 2, 1);
        pc += 4;
    }
    else if ((opcode & INST_LW_MASK) == INST_LW)
    {
        // ['rd', 'rs1', 'imm12']        
        reg_rd = Load(pc, reg_rs1 + imm12, 4, 1);
        DPRINTF(LOG_INST,("%08x: lw %s, %d(%s) = 0x%x\n", pc, GetRegName(rd), imm12, GetRegName(rs1), reg_rd));
        pc += 4;        
    }
    else if ((opcode & INST_LBU_MASK) == INST_LBU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lbu %s, %d(%s)\n", pc, GetRegName(rd), imm12, GetRegName(rs1)));
        reg_rd = Load(pc, reg_rs1 + imm12, 1, 0);
        pc += 4;
    }
    else if ((opcode & INST_LHU_MASK) == INST_LHU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lhu %s, %d(%s)\n", pc, GetRegName(rd), imm12, GetRegName(rs1)));
        reg_rd = Load(pc, reg_rs1 + imm12, 2, 0);
        pc += 4;
    }
    else if ((opcode & INST_LWU_MASK) == INST_LWU)
    {
        // ['rd', 'rs1', 'imm12']
        DPRINTF(LOG_INST,("%08x: lwu %s, %d(%s)\n", pc, GetRegName(rd), imm12, GetRegName(rs1)));
        reg_rd = Load(pc, reg_rs1 + imm12, 4, 0);
        pc += 4;
    }
    else if ((opcode & INST_SB_MASK) == INST_SB)
    {
        // ['imm12hi', 'rs1', 'rs2', 'imm12lo']
        DPRINTF(LOG_INST,("%08x: sb %d(%s), %s\n", pc, storeimm, GetRegName(rs1), GetRegName(rs2)));
        Store(pc, reg_rs1 + storeimm, reg_rs2, 1);
        pc += 4;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_SH_MASK) == INST_SH)
    {
        // ['imm12hi', 'rs1', 'rs2', 'imm12lo']
        DPRINTF(LOG_INST,("%08x: sh %d(%s), %s\n", pc, storeimm, GetRegName(rs1), GetRegName(rs2)));
        Store(pc, reg_rs1 + storeimm, reg_rs2, 2);
        pc += 4;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_SW_MASK) == INST_SW)
    {
        // ['imm12hi', 'rs1', 'rs2', 'imm12lo']
        DPRINTF(LOG_INST,("%08x: sw %d(%s), %s\n", pc, storeimm, GetRegName(rs1), GetRegName(rs2)));
        Store(pc, reg_rs1 + storeimm, reg_rs2, 4);
        pc += 4;

        // No writeback
        rd = 0;
    }
    else if ((opcode & INST_MUL_MASK) == INST_MUL)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: mul %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        reg_rd = (signed)reg_rs1 * (signed)reg_rs2;
        pc += 4;        
    }
    else if ((opcode & INST_MULH_MASK) == INST_MULH)
    {
        // ['rd', 'rs1', 'rs2']
        long long res = ((long long) (int)reg_rs1) * ((long long)(int)reg_rs2);
        DPRINTF(LOG_INST,("%08x: mulh %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        reg_rd = (int)(res >> 32);
        pc += 4;
    }
    else if ((opcode & INST_MULHSU_MASK) == INST_MULHSU)
    {
        // ['rd', 'rs1', 'rs2']
        long long res = ((long long) (int)reg_rs1) * ((unsigned long long)(unsigned)reg_rs2);
        DPRINTF(LOG_INST,("%08x: mulhsu %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        reg_rd = (int)(res >> 32);
        pc += 4;
    }
    else if ((opcode & INST_MULHU_MASK) == INST_MULHU)
    {
        // ['rd', 'rs1', 'rs2']
        unsigned long long res = ((unsigned long long) (unsigned)reg_rs1) * ((unsigned long long)(unsigned)reg_rs2);
        DPRINTF(LOG_INST,("%08x: mulhu %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        reg_rd = (int)(res >> 32);
        pc += 4;
    }
    else if ((opcode & INST_DIV_MASK) == INST_DIV)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: div %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        if (reg_rs2 != 0)
            reg_rd = (signed)reg_rs1 / (signed)reg_rs2;
        else
            reg_rd = (unsigned)-1;
        pc += 4;        
    }
    else if ((opcode & INST_DIVU_MASK) == INST_DIVU)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: divu %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        if (reg_rs2 != 0)
            reg_rd = (unsigned)reg_rs1 / (unsigned)reg_rs2;
        else
            reg_rd = (unsigned)-1;
        pc += 4;        
    }
    else if ((opcode & INST_REM_MASK) == INST_REM)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: rem %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
        if (reg_rs2 != 0)
            reg_rd = (signed)reg_rs1 % (signed)reg_rs2;
        else
            reg_rd = reg_rs1;
        pc += 4;        
    }
    else if ((opcode & INST_REMU_MASK) == INST_REMU)
    {
        // ['rd', 'rs1', 'rs2']
        DPRINTF(LOG_INST,("%08x: remu %s, %s, %s\n", pc, GetRegName(rd), GetRegName(rs1), GetRegName(rs2)));
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
    }
    else if ((opcode & INST_SBREAK_MASK) == INST_SBREAK)
    {
        DPRINTF(LOG_INST,("%08x: sbreak\n", pc));

        Exception(CAUSE_BREAKPOINT, pc);

        pc          = csr_evec;
        Break       = 1;
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
        DPRINTF(LOG_INST,("%08x: csrw %s, %s, 0x%x\n", pc, GetRegName(rd), GetRegName(rs1), imm12));
        reg_rd = AccessCsr(imm12, reg_rs1, 1, 1);
        pc += 4;
    }    
    else if ((opcode & INST_CSRRS_MASK) == INST_CSRRS)
    {
        DPRINTF(LOG_INST,("%08x: csrs %s, %s, 0x%x\n", pc, GetRegName(rd), GetRegName(rs1), imm12));
        reg_rd = AccessCsr(imm12, reg_rs1, 1, 0);
        pc += 4;
    }
    else if ((opcode & INST_CSRRC_MASK) == INST_CSRRC)
    {
        DPRINTF(LOG_INST,("%08x: csrc %s, %s, 0x%x\n", pc, GetRegName(rd), GetRegName(rs1), imm12));
        reg_rd = AccessCsr(imm12, reg_rs1, 0, 1);
        pc += 4;
    }
    else if ((opcode & INST_CSRRWI_MASK) == INST_CSRRWI)
    {
        DPRINTF(LOG_INST,("%08x: csrw %s, %d, 0x%x\n", pc, GetRegName(rd), rs1, imm12));
        reg_rd = AccessCsr(imm12, rs1, 1, 1);
        pc += 4;
    }
    else if ((opcode & INST_CSRRSI_MASK) == INST_CSRRSI)
    {
        DPRINTF(LOG_INST,("%08x: csrs %s, %d, 0x%x\n", pc, GetRegName(rd), rs1, imm12));
        reg_rd = AccessCsr(imm12, rs1, 1, 0);
        pc += 4;
    }
    else if ((opcode & INST_CSRRCI_MASK) == INST_CSRRCI)
    {
        DPRINTF(LOG_INST,("%08x: csrc %s, %d, 0x%x\n", pc, GetRegName(rd), rs1, imm12));
        reg_rd = AccessCsr(imm12, rs1, 0, 1);
        pc += 4;
    }    
    else
    {
        ASSERT(0 && "Bad instruction!");
    }

    if (rd != 0)
    {
        DPRINTF(LOG_INST,(" Write to R%d = %x\n", rd, reg_rd));
        r_gpr[rd] = reg_rd;
    }

    r_pc = pc & PC_MASK;
}
//-----------------------------------------------------------------
// $monitor_fetch(valid, pc, opcode, intr)
//-----------------------------------------------------------------
static int monitor_fetch(char*user_data)
{
    unsigned valid;
    unsigned pc;
    unsigned opcode;
    unsigned intr;

    vpiHandle h1, h2, h3;
    h1 = vpi_handle(vpiSysTfCall, NULL);
    h2 = vpi_iterate(vpiArgument, h1);

    s_vpi_value val;
    val.format = vpiIntVal;

    h3 = vpi_scan(h2);
    vpi_get_value (h3, &val);
    valid = val.value.integer;

    h3 = vpi_scan(h2);
    vpi_get_value (h3, &val);
    pc = val.value.integer;

    h3 = vpi_scan(h2);
    vpi_get_value (h3, &val);
    opcode = val.value.integer;

    h3 = vpi_scan(h2);
    vpi_get_value (h3, &val);
    intr = val.value.integer;

    if (valid)
    {
        if (r_pc != pc)
            vpi_printf("$monitor_fetch(valid=%d, pc=%x, opcode=%x) (expected = %x)\n", valid, pc, opcode, r_pc);

        ASSERT(r_pc == pc);

        load_val++;
        Execute(pc, opcode, intr);        
    }

    vpi_free_object(h2);
    return 0;
}
//-----------------------------------------------------------------
// $monitor_status(status_break, status_fault)
//-----------------------------------------------------------------
static int monitor_status(char*user_data)
{
    unsigned status_break;
    unsigned status_fault;

    vpiHandle h1, h2, h3;
    h1 = vpi_handle(vpiSysTfCall, NULL);
    h2 = vpi_iterate(vpiArgument, h1);

    s_vpi_value val;
    val.format = vpiIntVal;

    h3 = vpi_scan(h2);
    vpi_get_value (h3, &val);
    status_break = val.value.integer;

    h3 = vpi_scan(h2);
    vpi_get_value (h3, &val);
    status_fault = val.value.integer;

    ASSERT(status_break == Break);
    ASSERT(status_fault == Fault);

    Break = 0;

    vpi_free_object(h2);
    return 0;
}
//-----------------------------------------------------------------
// $monitor_csr_internal(csr_epc, csr_evec, csr_cause, csr_sr)
//-----------------------------------------------------------------
static int monitor_csr_internal(char*user_data)
{
    unsigned epc;
    unsigned evec;
    unsigned cause;
    unsigned sr; 

    vpiHandle h1, h2, h3;
    h1 = vpi_handle(vpiSysTfCall, NULL);
    h2 = vpi_iterate(vpiArgument, h1);

    s_vpi_value val;
    val.format = vpiIntVal;

    h3 = vpi_scan(h2);
    vpi_get_value (h3, &val);
    epc = val.value.integer;

    h3 = vpi_scan(h2);
    vpi_get_value (h3, &val);
    evec = val.value.integer;

    h3 = vpi_scan(h2);
    vpi_get_value (h3, &val);
    cause = val.value.integer;

    h3 = vpi_scan(h2);
    vpi_get_value (h3, &val);
    sr = val.value.integer;  

    ASSERT(csr_epc == epc);
    ASSERT(csr_cause == cause);
    ASSERT(csr_sr == sr);
    ASSERT(csr_evec == evec);

    vpi_free_object(h2);
    return 0;
}
//-----------------------------------------------------------------
// $monitor_writeback(write_en, write_reg, write_value)
//-----------------------------------------------------------------
static int monitor_writeback(char*user_data)
{
    unsigned write_en;
    unsigned write_reg;
    unsigned write_value;

    vpiHandle h1, h2, h3;
    h1 = vpi_handle(vpiSysTfCall, NULL);
    h2 = vpi_iterate(vpiArgument, h1);

    s_vpi_value val;
    val.format = vpiIntVal;

    h3 = vpi_scan(h2);
    vpi_get_value (h3, &val);
    write_en = val.value.integer;

    h3 = vpi_scan(h2);
    vpi_get_value (h3, &val);
    write_reg = val.value.integer;

    h3 = vpi_scan(h2);
    vpi_get_value (h3, &val);
    write_value = val.value.integer;
 
    if (write_en)
    {
        if (r_gpr[write_reg] != write_value)
            printf("MISMATCH: Reg R%d (%s) 0x%x != 0x%x\n", write_reg, GetRegName(write_reg), r_gpr[write_reg], write_value);
        ASSERT(r_gpr[write_reg] == write_value);
    }

    vpi_free_object(h2);
    return 0;
}
//-----------------------------------------------------------------
// register_vpi
//-----------------------------------------------------------------
void register_vpi()
{
    s_vpi_systf_data tf_data;

    Init();

    tf_data.type      = vpiSysTask;
    tf_data.tfname    = "$monitor_fetch";
    tf_data.calltf    = monitor_fetch;
    tf_data.compiletf = 0;
    tf_data.sizetf    = 0;
    tf_data.user_data = 0;
    vpi_register_systf(&tf_data);

    tf_data.type      = vpiSysTask;
    tf_data.tfname    = "$monitor_status";
    tf_data.calltf    = monitor_status;
    tf_data.compiletf = 0;
    tf_data.sizetf    = 0;
    tf_data.user_data = 0;
    vpi_register_systf(&tf_data);

    tf_data.type      = vpiSysTask;
    tf_data.tfname    = "$monitor_csr_internal";
    tf_data.calltf    = monitor_csr_internal;
    tf_data.compiletf = 0;
    tf_data.sizetf    = 0;
    tf_data.user_data = 0;
    vpi_register_systf(&tf_data);    

    tf_data.type      = vpiSysTask;
    tf_data.tfname    = "$monitor_writeback";
    tf_data.calltf    = monitor_writeback;
    tf_data.compiletf = 0;
    tf_data.sizetf    = 0;
    tf_data.user_data = 0;
    vpi_register_systf(&tf_data); 
}

void (*vlog_startup_routines[])() = {
    register_vpi,
    0
};
