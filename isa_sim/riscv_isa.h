#ifndef __RISCV_ISA_H__
#define __RISCV_ISA_H__

//-----------------------------------------------------------------
// General:
//-----------------------------------------------------------------
#define REGISTERS               32

//--------------------------------------------------------------------
// Instruction Encoding
//--------------------------------------------------------------------
#define OPCODE_MAKE_MASK(a,b)   (((1ULL << ((a)+1ULL))-1) & ~((1ULL << (b))-1))

#define OPCODE_RD_SHIFT         7
#define OPCODE_RD_MASK          OPCODE_MAKE_MASK(11, 7)
#define OPCODE_RS1_SHIFT        15
#define OPCODE_RS1_MASK         OPCODE_MAKE_MASK(19,15)
#define OPCODE_RS2_SHIFT        20
#define OPCODE_RS2_MASK         OPCODE_MAKE_MASK(24,20)

#define OPCODE_TYPEI_IMM_SHIFT  20
#define OPCODE_TYPEI_IMM_MASK   OPCODE_MAKE_MASK(31,20)

#define OPCODE_TYPEU_IMM_SHIFT  12
#define OPCODE_TYPEU_IMM_MASK   OPCODE_MAKE_MASK(31,12)

#define OPCODE_SHIFT_MASK(x, s, n)  (((x) >> (s)) & ((1<<(n))-1))
#define OPCODE_IMM_SIGN(x)          (-(((x) >> 31) & 1))

#define OPCODE_ITYPE_IMM(x)     (OPCODE_SHIFT_MASK(x, 20, 12) | (OPCODE_IMM_SIGN(x) << 12))
#define OPCODE_STYPE_IMM(x)     (OPCODE_SHIFT_MASK(x, 7, 5) | (OPCODE_SHIFT_MASK(x, 25, 7) << 5) | (OPCODE_IMM_SIGN(x) << 12))
#define OPCODE_SBTYPE_IMM(x)    ((OPCODE_SHIFT_MASK(x, 8, 4) << 1) | (OPCODE_SHIFT_MASK(x, 25, 6) << 5) | (OPCODE_SHIFT_MASK(x, 7, 1) << 11) | (OPCODE_IMM_SIGN(x) << 12))
#define OPCODE_UTYPE_IMM(x)     (OPCODE_SHIFT_MASK(x, 12, 20) | (OPCODE_IMM_SIGN(x) << 20))
#define OPCODE_UJTYPE_IMM(x)    ((OPCODE_SHIFT_MASK(x, 21, 10) << 1) | (OPCODE_SHIFT_MASK(x, 20, 1) << 11) | (OPCODE_SHIFT_MASK(x, 12, 8) << 12) | (OPCODE_IMM_SIGN(x) << 20))

#define OPCODE_SHAMT_SHIFT      20
#define OPCODE_SHAMT_MASK       OPCODE_MAKE_MASK(25,20)

//--------------------------------------------------------------------
// Instruction Decode Groups
//--------------------------------------------------------------------
// Mask 0x707f
//   andi, addi, slti, sltiu, ori, xori, jalr, beq, bne, blt, bge, bltu, bgeu, lb, lh, lw, lbu, lhu, lwu, sb, sh, sw, csrrs, csrrc, csrrsi, csrrci

// Mask 0xffffffff
//   scall, sbreak, sret

// Mask 0xfe00707f
//   add, sub, slt, sltu, xor, or, and, sll, srl, sra, mul, mulh, mulhsu, mulhu, div, divu, rem, remu

// Mask 0xfc00707f
//   slli, srli, srai

// Mask 0x7f
//   lui, auipc, jal

//--------------------------------------------------------------------
// Instructions
//--------------------------------------------------------------------
enum eInstructions
{
    ENUM_INST_ANDI,
    ENUM_INST_ADDI,
    ENUM_INST_SLTI,
    ENUM_INST_SLTIU,
    ENUM_INST_ORI,
    ENUM_INST_XORI,
    ENUM_INST_SLLI,
    ENUM_INST_SRLI,
    ENUM_INST_SRAI,
    ENUM_INST_LUI,
    ENUM_INST_AUIPC,
    ENUM_INST_ADD,
    ENUM_INST_SUB,
    ENUM_INST_SLT,
    ENUM_INST_SLTU,
    ENUM_INST_XOR,
    ENUM_INST_OR,
    ENUM_INST_AND,
    ENUM_INST_SLL,
    ENUM_INST_SRL,
    ENUM_INST_SRA,
    ENUM_INST_JAL,
    ENUM_INST_JALR,
    ENUM_INST_BEQ,
    ENUM_INST_BNE,
    ENUM_INST_BLT,
    ENUM_INST_BGE,
    ENUM_INST_BLTU,
    ENUM_INST_BGEU,
    ENUM_INST_LB,
    ENUM_INST_LH,
    ENUM_INST_LW,
    ENUM_INST_LBU,
    ENUM_INST_LHU,
    ENUM_INST_LWU,
    ENUM_INST_SB,
    ENUM_INST_SH,
    ENUM_INST_SW,
    ENUM_INST_ECALL,
    ENUM_INST_EBREAK,
    ENUM_INST_MRET,
    ENUM_INST_CSRRW,
    ENUM_INST_CSRRS,
    ENUM_INST_CSRRC,
    ENUM_INST_CSRRWI,
    ENUM_INST_CSRRSI,
    ENUM_INST_CSRRCI,
    ENUM_INST_MUL,
    ENUM_INST_MULH,
    ENUM_INST_MULHSU,
    ENUM_INST_MULHU,
    ENUM_INST_DIV,
    ENUM_INST_DIVU,
    ENUM_INST_REM,
    ENUM_INST_REMU,
    ENUM_INST_MAX
};

static const char * inst_names[ENUM_INST_MAX+1] = 
{
    [ENUM_INST_ANDI] = "andi",
    [ENUM_INST_ADDI] = "addi",
    [ENUM_INST_SLTI] = "slti",
    [ENUM_INST_SLTIU] = "sltiu",
    [ENUM_INST_ORI] = "ori",
    [ENUM_INST_XORI] = "xori",
    [ENUM_INST_SLLI] = "slli",
    [ENUM_INST_SRLI] = "srli",
    [ENUM_INST_SRAI] = "srai",
    [ENUM_INST_LUI] = "lui",
    [ENUM_INST_AUIPC] = "auipc",
    [ENUM_INST_ADD] = "add",
    [ENUM_INST_SUB] = "sub",
    [ENUM_INST_SLT] = "slt",
    [ENUM_INST_SLTU] = "sltu",
    [ENUM_INST_XOR] = "xor",
    [ENUM_INST_OR] = "or",
    [ENUM_INST_AND] = "and",
    [ENUM_INST_SLL] = "sll",
    [ENUM_INST_SRL] = "srl",
    [ENUM_INST_SRA] = "sra",
    [ENUM_INST_JAL] = "jal",
    [ENUM_INST_JALR] = "jalr",
    [ENUM_INST_BEQ] = "beq",
    [ENUM_INST_BNE] = "bne",
    [ENUM_INST_BLT] = "blt",
    [ENUM_INST_BGE] = "bge",
    [ENUM_INST_BLTU] = "bltu",
    [ENUM_INST_BGEU] = "bgeu",
    [ENUM_INST_LB] = "lb",
    [ENUM_INST_LH] = "lh",
    [ENUM_INST_LW] = "lw",
    [ENUM_INST_LBU] = "lbu",
    [ENUM_INST_LHU] = "lhu",
    [ENUM_INST_LWU] = "lwu",
    [ENUM_INST_SB] = "sb",
    [ENUM_INST_SH] = "sh",
    [ENUM_INST_SW] = "sw",
    [ENUM_INST_ECALL] = "ecall",
    [ENUM_INST_EBREAK] = "ebreak",
    [ENUM_INST_MRET] = "mret",
    [ENUM_INST_CSRRW] = "csrw",
    [ENUM_INST_CSRRS] = "csrs",
    [ENUM_INST_CSRRC] = "csrc",
    [ENUM_INST_CSRRWI] = "csrwi",
    [ENUM_INST_CSRRSI] = "csrsi",
    [ENUM_INST_CSRRCI] = "csrci",
    [ENUM_INST_MUL] = "mul",
    [ENUM_INST_MULH] = "mulh",
    [ENUM_INST_MULHSU] = "mulhsu",
    [ENUM_INST_MULHU] = "mulhu",
    [ENUM_INST_DIV] = "div",
    [ENUM_INST_DIVU] = "divu",
    [ENUM_INST_REM] = "rem",
    [ENUM_INST_REMU] = "remu",
    [ENUM_INST_MAX] = ""
};

// andi
#define INST_ANDI 0x7013
#define INST_ANDI_MASK 0x707f

// addi
#define INST_ADDI 0x13
#define INST_ADDI_MASK 0x707f

// slti
#define INST_SLTI 0x2013
#define INST_SLTI_MASK 0x707f

// sltiu
#define INST_SLTIU 0x3013
#define INST_SLTIU_MASK 0x707f

// ori
#define INST_ORI 0x6013
#define INST_ORI_MASK 0x707f

// xori
#define INST_XORI 0x4013
#define INST_XORI_MASK 0x707f

// slli
#define INST_SLLI 0x1013
#define INST_SLLI_MASK 0xfc00707f

// srli
#define INST_SRLI 0x5013
#define INST_SRLI_MASK 0xfc00707f

// srai
#define INST_SRAI 0x40005013
#define INST_SRAI_MASK 0xfc00707f

// lui
#define INST_LUI 0x37
#define INST_LUI_MASK 0x7f

// auipc
#define INST_AUIPC 0x17
#define INST_AUIPC_MASK 0x7f

// add
#define INST_ADD 0x33
#define INST_ADD_MASK 0xfe00707f

// sub
#define INST_SUB 0x40000033
#define INST_SUB_MASK 0xfe00707f

// slt
#define INST_SLT 0x2033
#define INST_SLT_MASK 0xfe00707f

// sltu
#define INST_SLTU 0x3033
#define INST_SLTU_MASK 0xfe00707f

// xor
#define INST_XOR 0x4033
#define INST_XOR_MASK 0xfe00707f

// or
#define INST_OR 0x6033
#define INST_OR_MASK 0xfe00707f

// and
#define INST_AND 0x7033
#define INST_AND_MASK 0xfe00707f

// sll
#define INST_SLL 0x1033
#define INST_SLL_MASK 0xfe00707f

// srl
#define INST_SRL 0x5033
#define INST_SRL_MASK 0xfe00707f

// sra
#define INST_SRA 0x40005033
#define INST_SRA_MASK 0xfe00707f

// jal
#define INST_JAL 0x6f
#define INST_JAL_MASK 0x7f

// jalr
#define INST_JALR 0x67
#define INST_JALR_MASK 0x707f

// beq
#define INST_BEQ 0x63
#define INST_BEQ_MASK 0x707f

// bne
#define INST_BNE 0x1063
#define INST_BNE_MASK 0x707f

// blt
#define INST_BLT 0x4063
#define INST_BLT_MASK 0x707f

// bge
#define INST_BGE 0x5063
#define INST_BGE_MASK 0x707f

// bltu
#define INST_BLTU 0x6063
#define INST_BLTU_MASK 0x707f

// bgeu
#define INST_BGEU 0x7063
#define INST_BGEU_MASK 0x707f

// lb
#define INST_LB 0x3
#define INST_LB_MASK 0x707f

// lh
#define INST_LH 0x1003
#define INST_LH_MASK 0x707f

// lw
#define INST_LW 0x2003
#define INST_LW_MASK 0x707f

// lbu
#define INST_LBU 0x4003
#define INST_LBU_MASK 0x707f

// lhu
#define INST_LHU 0x5003
#define INST_LHU_MASK 0x707f

// lwu
#define INST_LWU 0x6003
#define INST_LWU_MASK 0x707f

// sb
#define INST_SB 0x23
#define INST_SB_MASK 0x707f

// sh
#define INST_SH 0x1023
#define INST_SH_MASK 0x707f

// sw
#define INST_SW 0x2023
#define INST_SW_MASK 0x707f

// ecall
#define INST_ECALL 0x73
#define INST_ECALL_MASK 0xffffffff

// ebreak
#define INST_EBREAK 0x100073
#define INST_EBREAK_MASK 0xffffffff

// mret
#define INST_MRET 0x30200073
#define INST_MRET_MASK 0xffffffff

// csrrw
#define INST_CSRRW 0x1073
#define INST_CSRRW_MASK 0x707f

// csrrs
#define INST_CSRRS 0x2073
#define INST_CSRRS_MASK 0x707f

// csrrc
#define INST_CSRRC 0x3073
#define INST_CSRRC_MASK 0x707f

// csrrwi
#define INST_CSRRWI 0x5073
#define INST_CSRRWI_MASK 0x707f

// csrrsi
#define INST_CSRRSI 0x6073
#define INST_CSRRSI_MASK 0x707f

// csrrci
#define INST_CSRRCI 0x7073
#define INST_CSRRCI_MASK 0x707f

// mul
#define INST_MUL 0x2000033
#define INST_MUL_MASK 0xfe00707f

// mulh
#define INST_MULH 0x2001033
#define INST_MULH_MASK 0xfe00707f

// mulhsu
#define INST_MULHSU 0x2002033
#define INST_MULHSU_MASK 0xfe00707f

// mulhu
#define INST_MULHU 0x2003033
#define INST_MULHU_MASK 0xfe00707f

// div
#define INST_DIV 0x2004033
#define INST_DIV_MASK 0xfe00707f

// divu
#define INST_DIVU 0x2005033
#define INST_DIVU_MASK 0xfe00707f

// rem
#define INST_REM 0x2006033
#define INST_REM_MASK 0xfe00707f

// remu
#define INST_REMU 0x2007033
#define INST_REMU_MASK 0xfe00707f

#define IS_LOAD_INST(a)     (((a) & 0x7F) == 0x03)
#define IS_STORE_INST(a)    (((a) & 0x7F) == 0x23)
#define IS_BRANCH_INST(a)   ((((a) & 0x7F) == 0x6f) || \
                            (((a) & 0x7F) == 0x67) || \
                            (((a) & 0x7F) == 0x63) || \
                            (((a) & INST_ECALL_MASK) == INST_ECALL) || \
                            (((a) & INST_EBREAK_MASK) == INST_EBREAK) || \
                            (((a) & INST_MRET_MASK) == INST_MRET))

#define IS_ALU_3R_INST(a)           (((a) & 0x7F) == 0x33)
#define IS_ALU_2RI_INST(a)          ((((a) & 0x7F) == 0x13) || (((a) & 0x7F) == 0x67))
#define IS_COND_BRANCH_2RI_INST(a)  (((a) & 0x7F) == 0x63)
#define IS_RD_I_INST(a)     (((a) & INST_JAL_MASK) == INST_JAL) || \
                            (((a) & INST_LUI_MASK) == INST_LUI) || \
                            (((a) & INST_AUIPC_MASK) == INST_AUIPC))


//--------------------------------------------------------------------
// Privilege levels
//--------------------------------------------------------------------
#define PRIV_USER         0
#define PRIV_SUPER        1
#define PRIV_MACHINE      3

//--------------------------------------------------------------------
// IRQ Numbers
//--------------------------------------------------------------------
#define IRQ_SOFT          3
#define IRQ_TIMER         7
#define IRQ_EXT           11
#define IRQ_MIN           (IRQ_SOFT)
#define IRQ_MAX           (IRQ_EXT + 1)
#define IRQ_MASK          ((1 << IRQ_SOFT) | (1 << IRQ_TIMER) | (1 << IRQ_EXT))

//--------------------------------------------------------------------
// CSR Registers
//--------------------------------------------------------------------
#define CSR_MSTATUS       0x300
#define CSR_MSTATUS_MASK  0xFFFFFFFF
//#define CSR_MISA          0x301
//#define CSR_MISA_MASK     0xFFFFFFFF
//#define CSR_MEDELEG       0x302
//#define CSR_MEDELEG_MASK  0xFFFFFFFF
//#define CSR_MIDELEG       0x303
//#define CSR_MIDELEG_MASK  0xFFFFFFFF
#define CSR_MIE           0x304
#define CSR_MIE_MASK      IRQ_MASK
#define CSR_MTVEC         0x305
#define CSR_MTVEC_MASK    0xFFFFFFFF
#define CSR_MSCRATCH      0x340
#define CSR_MSCRATCH_MASK 0xFFFFFFFF
    #define CSR_SIM_CTRL_EXIT (0 << 24)
    #define CSR_SIM_CTRL_PUTC (1 << 24)
    #define CSR_SIM_CTRL_GETC (2 << 24)
#define CSR_MEPC          0x341
#define CSR_MEPC_MASK     0xFFFFFFFF
#define CSR_MCAUSE        0x342
#define CSR_MCAUSE_MASK   0x8000000F
//#define CSR_MBADADDR      0x343
//#define CSR_MBADADDR_MASK 0xFFFFFFFF
#define CSR_MIP           0x344
#define CSR_MIP_MASK      IRQ_MASK
#define CSR_MTIME         0xc01
#define CSR_MTIME_MASK    0xFFFFFFFF


//--------------------------------------------------------------------
// Status Register
//--------------------------------------------------------------------
//#define SR_UIE        (1 << 0)
//#define SR_SIE        (1 << 1)
#define SR_MIE          (1 << 3)
//#define SR_UPIE       (1 << 4)
//#define SR_SPIE       (1 << 5)
#define SR_MPIE         (1 << 7)

#define SR_MPP_SHIFT    11
#define SR_MPP_MASK     0x3
#define SR_MPP          (SR_MPP_MASK  << SR_MPP_SHIFT)
#define SR_MPP_U        (PRIV_USER    << SR_MPP_SHIFT)
#define SR_MPP_M        (PRIV_MACHINE << SR_MPP_SHIFT)

#define SR_IP_MSIP      (1 << IRQ_SOFT)
#define SR_IP_MTIP      (1 << IRQ_TIMER)
#define SR_IP_MEIP      (1 << IRQ_EXT)

//--------------------------------------------------------------------
// Exception Causes
//--------------------------------------------------------------------
#define MCAUSE_INT                      31
#define MCAUSE_MISALIGNED_FETCH         ((0 << MCAUSE_INT) | 0)
#define MCAUSE_FAULT_FETCH              ((0 << MCAUSE_INT) | 1)
#define MCAUSE_ILLEGAL_INSTRUCTION      ((0 << MCAUSE_INT) | 2)
#define MCAUSE_BREAKPOINT               ((0 << MCAUSE_INT) | 3)
#define MCAUSE_MISALIGNED_LOAD          ((0 << MCAUSE_INT) | 4)
#define MCAUSE_FAULT_LOAD               ((0 << MCAUSE_INT) | 5)
#define MCAUSE_MISALIGNED_STORE         ((0 << MCAUSE_INT) | 6)
#define MCAUSE_FAULT_STORE              ((0 << MCAUSE_INT) | 7)
#define MCAUSE_ECALL_U                  ((0 << MCAUSE_INT) | 8)
#define MCAUSE_PAGE_FAULT_INST          ((0 << MCAUSE_INT) | 12)
#define MCAUSE_PAGE_FAULT_LOAD          ((0 << MCAUSE_INT) | 13)
#define MCAUSE_PAGE_FAULT_STORE         ((0 << MCAUSE_INT) | 15)
#define MCAUSE_INTERRUPT                (1 << MCAUSE_INT)

#endif

