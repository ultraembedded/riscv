//--------------------------------------------------------------------
// ALU Operations
//--------------------------------------------------------------------
`define ALU_NONE                                4'b0000
`define ALU_SHIFTL                              4'b0001
`define ALU_SHIFTR                              4'b0010
`define ALU_SHIFTR_ARITH                        4'b0011
`define ALU_ADD                                 4'b0100
`define ALU_SUB                                 4'b0110
`define ALU_AND                                 4'b0111
`define ALU_OR                                  4'b1000
`define ALU_XOR                                 4'b1001
`define ALU_LESS_THAN                           4'b1010
`define ALU_LESS_THAN_SIGNED                    4'b1011

//--------------------------------------------------------------------
// Instructions Enumerations
//--------------------------------------------------------------------
`define       ENUM_INST_ANDI    0
`define       ENUM_INST_ADDI    1
`define       ENUM_INST_SLTI    2
`define      ENUM_INST_SLTIU    3
`define        ENUM_INST_ORI    4
`define       ENUM_INST_XORI    5
`define       ENUM_INST_SLLI    6
`define       ENUM_INST_SRLI    7
`define       ENUM_INST_SRAI    8
`define        ENUM_INST_LUI    9
`define      ENUM_INST_AUIPC    10
`define        ENUM_INST_ADD    11
`define        ENUM_INST_SUB    12
`define        ENUM_INST_SLT    13
`define       ENUM_INST_SLTU    14
`define        ENUM_INST_XOR    15
`define         ENUM_INST_OR    16
`define        ENUM_INST_AND    17
`define        ENUM_INST_SLL    18
`define        ENUM_INST_SRL    19
`define        ENUM_INST_SRA    20
`define        ENUM_INST_JAL    21
`define       ENUM_INST_JALR    22
`define        ENUM_INST_BEQ    23
`define        ENUM_INST_BNE    24
`define        ENUM_INST_BLT    25
`define        ENUM_INST_BGE    26
`define       ENUM_INST_BLTU    27
`define       ENUM_INST_BGEU    28
`define         ENUM_INST_LB    29
`define         ENUM_INST_LH    30
`define         ENUM_INST_LW    31
`define        ENUM_INST_LBU    32
`define        ENUM_INST_LHU    33
`define        ENUM_INST_LWU    34
`define         ENUM_INST_SB    35
`define         ENUM_INST_SH    36
`define         ENUM_INST_SW    37
`define      ENUM_INST_ECALL    38
`define     ENUM_INST_EBREAK    39
`define       ENUM_INST_MRET    40
`define      ENUM_INST_CSRRW    41
`define      ENUM_INST_CSRRS    42
`define      ENUM_INST_CSRRC    43
`define     ENUM_INST_CSRRWI    44
`define     ENUM_INST_CSRRSI    45
`define     ENUM_INST_CSRRCI    46
`define        ENUM_INST_MUL    47
`define       ENUM_INST_MULH    48
`define     ENUM_INST_MULHSU    49
`define      ENUM_INST_MULHU    50
`define        ENUM_INST_DIV    51
`define       ENUM_INST_DIVU    52
`define        ENUM_INST_REM    53
`define       ENUM_INST_REMU    54
`define       ENUM_INST_INTR    55
`define        ENUM_INST_MAX    56

//--------------------------------------------------------------------
// Instructions Masks
//--------------------------------------------------------------------
// andi
`define INST_ANDI 32'h7013
`define INST_ANDI_MASK 32'h707f

// addi
`define INST_ADDI 32'h13
`define INST_ADDI_MASK 32'h707f

// slti
`define INST_SLTI 32'h2013
`define INST_SLTI_MASK 32'h707f

// sltiu
`define INST_SLTIU 32'h3013
`define INST_SLTIU_MASK 32'h707f

// ori
`define INST_ORI 32'h6013
`define INST_ORI_MASK 32'h707f

// xori
`define INST_XORI 32'h4013
`define INST_XORI_MASK 32'h707f

// slli
`define INST_SLLI 32'h1013
`define INST_SLLI_MASK 32'hfc00707f

// srli
`define INST_SRLI 32'h5013
`define INST_SRLI_MASK 32'hfc00707f

// srai
`define INST_SRAI 32'h40005013
`define INST_SRAI_MASK 32'hfc00707f

// lui
`define INST_LUI 32'h37
`define INST_LUI_MASK 32'h7f

// auipc
`define INST_AUIPC 32'h17
`define INST_AUIPC_MASK 32'h7f

// add
`define INST_ADD 32'h33
`define INST_ADD_MASK 32'hfe00707f

// sub
`define INST_SUB 32'h40000033
`define INST_SUB_MASK 32'hfe00707f

// slt
`define INST_SLT 32'h2033
`define INST_SLT_MASK 32'hfe00707f

// sltu
`define INST_SLTU 32'h3033
`define INST_SLTU_MASK 32'hfe00707f

// xor
`define INST_XOR 32'h4033
`define INST_XOR_MASK 32'hfe00707f

// or
`define INST_OR 32'h6033
`define INST_OR_MASK 32'hfe00707f

// and
`define INST_AND 32'h7033
`define INST_AND_MASK 32'hfe00707f

// sll
`define INST_SLL 32'h1033
`define INST_SLL_MASK 32'hfe00707f

// srl
`define INST_SRL 32'h5033
`define INST_SRL_MASK 32'hfe00707f

// sra
`define INST_SRA 32'h40005033
`define INST_SRA_MASK 32'hfe00707f

// jal
`define INST_JAL 32'h6f
`define INST_JAL_MASK 32'h7f

// jalr
`define INST_JALR 32'h67
`define INST_JALR_MASK 32'h707f

// beq
`define INST_BEQ 32'h63
`define INST_BEQ_MASK 32'h707f

// bne
`define INST_BNE 32'h1063
`define INST_BNE_MASK 32'h707f

// blt
`define INST_BLT 32'h4063
`define INST_BLT_MASK 32'h707f

// bge
`define INST_BGE 32'h5063
`define INST_BGE_MASK 32'h707f

// bltu
`define INST_BLTU 32'h6063
`define INST_BLTU_MASK 32'h707f

// bgeu
`define INST_BGEU 32'h7063
`define INST_BGEU_MASK 32'h707f

// lb
`define INST_LB 32'h3
`define INST_LB_MASK 32'h707f

// lh
`define INST_LH 32'h1003
`define INST_LH_MASK 32'h707f

// lw
`define INST_LW 32'h2003
`define INST_LW_MASK 32'h707f

// lbu
`define INST_LBU 32'h4003
`define INST_LBU_MASK 32'h707f

// lhu
`define INST_LHU 32'h5003
`define INST_LHU_MASK 32'h707f

// lwu
`define INST_LWU 32'h6003
`define INST_LWU_MASK 32'h707f

// sb
`define INST_SB 32'h23
`define INST_SB_MASK 32'h707f

// sh
`define INST_SH 32'h1023
`define INST_SH_MASK 32'h707f

// sw
`define INST_SW 32'h2023
`define INST_SW_MASK 32'h707f

// ecall
`define INST_ECALL 32'h73
`define INST_ECALL_MASK 32'hffffffff

// ebreak
`define INST_EBREAK 32'h100073
`define INST_EBREAK_MASK 32'hffffffff

// mret
`define INST_MRET 32'h30200073
`define INST_MRET_MASK 32'hffffffff

// csrrw
`define INST_CSRRW 32'h1073
`define INST_CSRRW_MASK 32'h707f

// csrrs
`define INST_CSRRS 32'h2073
`define INST_CSRRS_MASK 32'h707f

// csrrc
`define INST_CSRRC 32'h3073
`define INST_CSRRC_MASK 32'h707f

// csrrwi
`define INST_CSRRWI 32'h5073
`define INST_CSRRWI_MASK 32'h707f

// csrrsi
`define INST_CSRRSI 32'h6073
`define INST_CSRRSI_MASK 32'h707f

// csrrci
`define INST_CSRRCI 32'h7073
`define INST_CSRRCI_MASK 32'h707f

// mul
`define INST_MUL 32'h2000033
`define INST_MUL_MASK 32'hfe00707f

// mulh
`define INST_MULH 32'h2001033
`define INST_MULH_MASK 32'hfe00707f

// mulhsu
`define INST_MULHSU 32'h2002033
`define INST_MULHSU_MASK 32'hfe00707f

// mulhu
`define INST_MULHU 32'h2003033
`define INST_MULHU_MASK 32'hfe00707f

// div
`define INST_DIV 32'h2004033
`define INST_DIV_MASK 32'hfe00707f

// divu
`define INST_DIVU 32'h2005033
`define INST_DIVU_MASK 32'hfe00707f

// rem
`define INST_REM 32'h2006033
`define INST_REM_MASK 32'hfe00707f

// remu
`define INST_REMU 32'h2007033
`define INST_REMU_MASK 32'hfe00707f

//--------------------------------------------------------------------
// Privilege levels
//--------------------------------------------------------------------
`define PRIV_USER         0
`define PRIV_SUPER        1
`define PRIV_MACHINE      3

//--------------------------------------------------------------------
// IRQ Numbers
//--------------------------------------------------------------------
`define IRQ_SOFT          3
`define IRQ_TIMER         7
`define IRQ_EXT           11
`define IRQ_MIN           (`IRQ_SOFT)
`define IRQ_MAX           (`IRQ_EXT + 1)
`define IRQ_MASK          ((1 << `IRQ_SOFT) | (1 << `IRQ_TIMER) | (1 << `IRQ_EXT))

//--------------------------------------------------------------------
// CSR Registers
//--------------------------------------------------------------------
`define CSR_MSTATUS       12'h300
`define CSR_MSTATUS_MASK  32'hFFFFFFFF
`define CSR_MIE           12'h304
`define CSR_MIE_MASK      `IRQ_MASK
`define CSR_MTVEC         12'h305
`define CSR_MTVEC_MASK    32'hFFFFFFFF
`define CSR_MSCRATCH      12'h340
`define CSR_MSCRATCH_MASK 32'hFFFFFFFF
    `define CSR_SIM_CTRL_EXIT (0 << 24)
    `define CSR_SIM_CTRL_PUTC (1 << 24)
    `define CSR_SIM_CTRL_GETC (2 << 24)
`define CSR_MEPC          12'h341
`define CSR_MEPC_MASK     32'hFFFFFFFF
`define CSR_MCAUSE        12'h342
`define CSR_MCAUSE_MASK   32'h8000000F
`define CSR_MIP           12'h344
`define CSR_MIP_MASK      `IRQ_MASK
`define CSR_MTIME         12'hc01
`define CSR_MTIME_MASK    32'hFFFFFFFF

//--------------------------------------------------------------------
// Status Register
//--------------------------------------------------------------------
//`define SR_UIE        (1 << 0)
//`define SR_SIE        (1 << 1)
`define SR_MIE_BIT      (3)
`define SR_MIE          (1 << `SR_MIE_BIT)
//`define SR_UPIE       (1 << 4)
//`define SR_SPIE       (1 << 5)
`define SR_MPIE_BIT     (7)
`define SR_MPIE         (1 << `SR_MPIE_BIT)

`define SR_MPP_SHIFT    11
`define SR_MPP_MASK     3
`define SR_MPP          (`SR_MPP_MASK  << `SR_MPP_SHIFT)
`define SR_MPP_U        (`PRIV_USER    << `SR_MPP_SHIFT)
`define SR_MPP_M        (`PRIV_MACHINE << `SR_MPP_SHIFT)

`define SR_IP_MSIP      (1 << `IRQ_SOFT)
`define SR_IP_MTIP      (1 << `IRQ_TIMER)
`define SR_IP_MEIP      (1 << `IRQ_EXT)

//--------------------------------------------------------------------
// Exception Causes
//--------------------------------------------------------------------
`define MCAUSE_INT                      31
`define MCAUSE_MISALIGNED_FETCH         ((0 << `MCAUSE_INT) | 0)
`define MCAUSE_FAULT_FETCH              ((0 << `MCAUSE_INT) | 1)
`define MCAUSE_ILLEGAL_INSTRUCTION      ((0 << `MCAUSE_INT) | 2)
`define MCAUSE_BREAKPOINT               ((0 << `MCAUSE_INT) | 3)
`define MCAUSE_MISALIGNED_LOAD          ((0 << `MCAUSE_INT) | 4)
`define MCAUSE_FAULT_LOAD               ((0 << `MCAUSE_INT) | 5)
`define MCAUSE_MISALIGNED_STORE         ((0 << `MCAUSE_INT) | 6)
`define MCAUSE_FAULT_STORE              ((0 << `MCAUSE_INT) | 7)
`define MCAUSE_ECALL_U                  ((0 << `MCAUSE_INT) | 8)
`define MCAUSE_PAGE_FAULT_INST          ((0 << `MCAUSE_INT) | 12)
`define MCAUSE_PAGE_FAULT_LOAD          ((0 << `MCAUSE_INT) | 13)
`define MCAUSE_PAGE_FAULT_STORE         ((0 << `MCAUSE_INT) | 15)
`define MCAUSE_INTERRUPT                (1 << `MCAUSE_INT)
