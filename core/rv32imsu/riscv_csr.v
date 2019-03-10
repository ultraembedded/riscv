//-----------------------------------------------------------------
//                         RISC-V Core
//                            V0.9.7
//                     Ultra-Embedded.com
//                     Copyright 2014-2019
//
//                   admin@ultra-embedded.com
//
//                       License: BSD
//-----------------------------------------------------------------
//
// Copyright (c) 2014-2019, Ultra-Embedded.com
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

module riscv_csr
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input           intr_i
    ,input           opcode_valid_i
    ,input  [ 57:0]  opcode_instr_i
    ,input  [ 31:0]  opcode_opcode_i
    ,input  [ 31:0]  opcode_pc_i
    ,input  [  4:0]  opcode_rd_idx_i
    ,input  [  4:0]  opcode_ra_idx_i
    ,input  [  4:0]  opcode_rb_idx_i
    ,input  [ 31:0]  opcode_ra_operand_i
    ,input  [ 31:0]  opcode_rb_operand_i
    ,input           branch_exec_request_i
    ,input  [ 31:0]  branch_exec_pc_i
    ,input  [ 31:0]  cpu_id_i
    ,input  [ 31:0]  reset_vector_i
    ,input           fault_store_i
    ,input           fault_load_i
    ,input           fault_misaligned_store_i
    ,input           fault_misaligned_load_i
    ,input           fault_page_store_i
    ,input           fault_page_load_i
    ,input  [ 31:0]  fault_addr_i

    // Outputs
    ,output [  4:0]  writeback_idx_o
    ,output          writeback_squash_o
    ,output [ 31:0]  writeback_value_o
    ,output          stall_o
    ,output          branch_csr_request_o
    ,output [ 31:0]  branch_csr_pc_o
    ,output          mmu_supervisor_o
    ,output          mmu_sum_o
    ,output          mmu_flush_o
    ,output [ 31:0]  mmu_satp_o
);



//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-----------------------------------------------------------------
// Registers / Wires
//-----------------------------------------------------------------
reg [31:0]  csr_mepc_q;
reg [31:0]  csr_mcause_q;
reg [31:0]  csr_sr_q;
reg [31:0]  csr_mtvec_q;
reg [31:0]  csr_mip_q;
reg [31:0]  csr_mie_q;
reg [1:0]   csr_mpriv_q;
reg [31:0]  csr_mcycle_q;
reg [31:0]  csr_mscratch_q;

// CSR - Supervisor
reg [31:0]  csr_sepc_q;
reg [31:0]  csr_stvec_q;
reg [31:0]  csr_scause_q;
reg [31:0]  csr_stval_q;
reg [31:0]  csr_satp_q;
reg [31:0]  csr_sscratch_q;

reg [31:0]  pc_m_q;

//-----------------------------------------------------------------
// Exception source
//-----------------------------------------------------------------
wire [31:0] exc_src_w;

wire        csr_access_fault_w;

assign exc_src_w[`MCAUSE_MISALIGNED_FETCH]    = 1'b0;
assign exc_src_w[`MCAUSE_FAULT_FETCH]         = opcode_valid_i & opcode_instr_i[`ENUM_INST_FAULT];
assign exc_src_w[`MCAUSE_ILLEGAL_INSTRUCTION] = opcode_valid_i & (opcode_instr_i[`ENUM_INST_INVALID] | csr_access_fault_w);
assign exc_src_w[`MCAUSE_BREAKPOINT]          = opcode_valid_i & opcode_instr_i[`ENUM_INST_EBREAK];
assign exc_src_w[`MCAUSE_MISALIGNED_LOAD]     = fault_misaligned_load_i;
assign exc_src_w[`MCAUSE_FAULT_LOAD]          = fault_load_i;
assign exc_src_w[`MCAUSE_MISALIGNED_STORE]    = fault_misaligned_store_i;
assign exc_src_w[`MCAUSE_FAULT_STORE]         = fault_store_i;
assign exc_src_w[`MCAUSE_ECALL_U]             = opcode_valid_i & opcode_instr_i[`ENUM_INST_ECALL] & (csr_mpriv_q == `PRIV_USER);
assign exc_src_w[`MCAUSE_ECALL_S]             = opcode_valid_i & opcode_instr_i[`ENUM_INST_ECALL] & (csr_mpriv_q == `PRIV_SUPER);
assign exc_src_w[`MCAUSE_ECALL_H]             = 1'b0;
assign exc_src_w[`MCAUSE_ECALL_M]             = opcode_valid_i & opcode_instr_i[`ENUM_INST_ECALL] & (csr_mpriv_q == `PRIV_MACHINE);
assign exc_src_w[`MCAUSE_PAGE_FAULT_INST]     = opcode_valid_i & opcode_instr_i[`ENUM_INST_PAGE_FAULT];
assign exc_src_w[`MCAUSE_PAGE_FAULT_LOAD]     = fault_page_load_i;
assign exc_src_w[`MCAUSE_PAGE_FAULT_LOAD + 1] = 1'b0;
assign exc_src_w[`MCAUSE_PAGE_FAULT_STORE]    = fault_page_store_i;
assign exc_src_w[31:16]                       = 16'b0;

`define EXC_SRC_NO_ECALL_EBREAK   (~((1 << `MCAUSE_ECALL_U) | (1 << `MCAUSE_ECALL_S) | (1 << `MCAUSE_ECALL_H) | (1 << `MCAUSE_ECALL_M) | (1 << `MCAUSE_BREAKPOINT)))
`define EXC_SRC_M_STAGE           ( ((1 << `MCAUSE_MISALIGNED_LOAD) | (1 << `MCAUSE_MISALIGNED_STORE) | (1 << `MCAUSE_PAGE_FAULT_LOAD) | (1 << `MCAUSE_PAGE_FAULT_STORE)))

// FAULT_LOAD and FAULT_STORE are imprecise, but would also be M stage or later.
wire exception_m_w   = |(exc_src_w & `EXC_SRC_M_STAGE);

//-----------------------------------------------------------------
// CSR handling
//-----------------------------------------------------------------
reg [31:0]  imm12_r;
reg [1:0]   priv_r;
reg         readonly_r;
reg         write_r;
reg         set_r;
reg         clr_r;

reg [31:0]  csr_mepc_r;
reg [31:0]  csr_mcause_r;
reg [31:0]  csr_sr_r;
reg [31:0]  csr_mtvec_r;
reg [31:0]  csr_mip_r;
reg [31:0]  csr_mie_r;
reg [31:0]  csr_mtime_r;
reg [31:0]  csr_mtimecmp_r;
reg [1:0]   csr_mpriv_r;
reg [31:0]  csr_mcycle_r;
reg [31:0]  csr_mscratch_r;

// CSR - Supervisor
reg [31:0]  csr_sepc_r;
reg [31:0]  csr_stvec_r;
reg [31:0]  csr_scause_r;
reg [31:0]  csr_stval_r;
reg [31:0]  csr_satp_r;
reg [31:0]  csr_sscratch_r;

reg  [31:0] csr_medeleg_q;
reg  [31:0] csr_medeleg_r;

reg  [31:0] csr_mideleg_q;
reg  [31:0] csr_mideleg_r;

reg         take_intr_r;
reg         take_exception_r;

reg         exception_s_r;
reg         interrupt_s_r;

reg [31:0]  data_r;
reg [31:0]  result_r;

wire valid_unit_inst_w = opcode_valid_i &
                         (opcode_instr_i[`ENUM_INST_CSRRW] | 
                         opcode_instr_i[`ENUM_INST_CSRRS]  | 
                         opcode_instr_i[`ENUM_INST_CSRRC]  | 
                         opcode_instr_i[`ENUM_INST_CSRRWI] | 
                         opcode_instr_i[`ENUM_INST_CSRRSI] | 
                         opcode_instr_i[`ENUM_INST_CSRRCI] |
                         opcode_instr_i[`ENUM_INST_ECALL]  |
                         opcode_instr_i[`ENUM_INST_EBREAK] |
                         opcode_instr_i[`ENUM_INST_ERET]   |
                         opcode_instr_i[`ENUM_INST_FAULT]  |
                         opcode_instr_i[`ENUM_INST_PAGE_FAULT]);


wire load_inst_w = (opcode_instr_i[`ENUM_INST_LB]  || 
                    opcode_instr_i[`ENUM_INST_LH]  || 
                    opcode_instr_i[`ENUM_INST_LW]  || 
                    opcode_instr_i[`ENUM_INST_LBU] || 
                    opcode_instr_i[`ENUM_INST_LHU] || 
                    opcode_instr_i[`ENUM_INST_LWU]);

wire store_inst_w = (opcode_instr_i[`ENUM_INST_SB]  || 
                     opcode_instr_i[`ENUM_INST_SH]  || 
                     opcode_instr_i[`ENUM_INST_SW]);


always @ *
begin
    imm12_r         = {{20{opcode_opcode_i[31]}}, opcode_opcode_i[31:20]};

    set_r           = opcode_instr_i[`ENUM_INST_CSRRW] | opcode_instr_i[`ENUM_INST_CSRRS] | 
                      opcode_instr_i[`ENUM_INST_CSRRWI] | opcode_instr_i[`ENUM_INST_CSRRSI];
    clr_r           = opcode_instr_i[`ENUM_INST_CSRRW] | opcode_instr_i[`ENUM_INST_CSRRC] | 
                      opcode_instr_i[`ENUM_INST_CSRRWI] | opcode_instr_i[`ENUM_INST_CSRRCI];

    priv_r          = imm12_r[9:8];
    readonly_r      = (imm12_r[11:10] == 2'd3);
    write_r         = (opcode_ra_idx_i != 5'b0)        |
                      opcode_instr_i[`ENUM_INST_CSRRW] |
                      opcode_instr_i[`ENUM_INST_CSRRWI];
end

// Detect access fault on CSR access
assign csr_access_fault_w = opcode_valid_i && (set_r | clr_r) && ((write_r && readonly_r) || (csr_mpriv_q < priv_r));

//-----------------------------------------------------------------
// IRQ handling
//-----------------------------------------------------------------
reg [31:0] irq_pending_r;
reg        m_enabled_r;
reg [31:0] m_interrupts_r;
reg        s_enabled_r;
reg [31:0] s_interrupts_r;
reg [31:0] irq_masked_r;

always @ *
begin
    irq_pending_r   = (csr_mip_q & csr_mie_q);
    m_enabled_r     = (csr_mpriv_q < `PRIV_MACHINE) || (csr_mpriv_q == `PRIV_MACHINE && csr_sr_q[`SR_MIE_R]);
    s_enabled_r     = (csr_mpriv_q < `PRIV_SUPER)   || (csr_mpriv_q == `PRIV_SUPER   && csr_sr_q[`SR_SIE_R]);
    m_interrupts_r  = m_enabled_r    ? (irq_pending_r & ~csr_mideleg_q) : 32'b0;
    s_interrupts_r  = s_enabled_r    ? (irq_pending_r &  csr_mideleg_q) : 32'b0;
    irq_masked_r    = (|m_interrupts_r) ? m_interrupts_r : s_interrupts_r;
end

//-----------------------------------------------------------------
// CSR register next state
//-----------------------------------------------------------------
always @ *
begin
    take_intr_r     = 1'b0;
    take_exception_r= 1'b0;
    exception_s_r   = 1'b0;
    interrupt_s_r   = 1'b0;
    csr_mepc_r      = csr_mepc_q;
    csr_sr_r        = csr_sr_q;
    csr_mcause_r    = csr_mcause_q;
    csr_mtvec_r     = csr_mtvec_q;
    csr_mip_r       = csr_mip_q;
    csr_mie_r       = csr_mie_q;
    csr_mpriv_r     = csr_mpriv_q;
    csr_mscratch_r  = csr_mscratch_q;
    csr_medeleg_r   = csr_medeleg_q;
    csr_mideleg_r   = csr_mideleg_q;
    csr_sepc_r      = csr_sepc_q;
    csr_stvec_r     = csr_stvec_q;
    csr_scause_r    = csr_scause_q;
    csr_stval_r     = csr_stval_q;
    csr_satp_r      = csr_satp_q;
    csr_sscratch_r  = csr_sscratch_q;

    result_r        = 32'b0;
    data_r          = 32'b0;

    csr_mcycle_r    = csr_mcycle_q + 32'd1;

    take_intr_r     = |irq_masked_r;

    // If an opcode is not being presented, another unit is probably issuing a stall.
    // Do not interrupt during the stall or if executing a load store....
    if (!opcode_valid_i || load_inst_w || store_inst_w)
    begin
        take_intr_r = 1'b0;
    end

    // Exception/interrupt delegated to supervisor mode
    if (exception_m_w)
        exception_s_r = |(exc_src_w & csr_medeleg_q & `EXC_SRC_M_STAGE);
    else
        exception_s_r = |(exc_src_w & csr_medeleg_q & `EXC_SRC_NO_ECALL_EBREAK);
    interrupt_s_r = |(irq_masked_r & csr_mideleg_q);

    // Fetch/Load/Store fault
    if (exc_src_w[`MCAUSE_FAULT_FETCH]     ||
        exc_src_w[`MCAUSE_FAULT_LOAD]      ||
        exc_src_w[`MCAUSE_FAULT_STORE]     ||
        exc_src_w[`MCAUSE_PAGE_FAULT_LOAD] ||
        exc_src_w[`MCAUSE_PAGE_FAULT_STORE]||
        exc_src_w[`MCAUSE_PAGE_FAULT_INST] ||
        exc_src_w[`MCAUSE_MISALIGNED_LOAD] || 
        exc_src_w[`MCAUSE_MISALIGNED_STORE]||
        exc_src_w[`MCAUSE_ILLEGAL_INSTRUCTION])
        take_exception_r = 1'b1;

    // Exceptions take priority over interrupts
    if (take_exception_r)
    begin
        // Not taking interrupt..
        take_intr_r = 1'b0;

        // Exception delegated to supervisor mode
        if (csr_mpriv_q <= `PRIV_SUPER && exception_s_r)
        begin
            // Save interrupt / supervisor state
            csr_sr_r[`SR_SPIE_R] = csr_sr_r[`SR_SIE_R];
            csr_sr_r[`SR_SPP_R]  = (csr_mpriv_q == `PRIV_SUPER);

            // Disable interrupts and enter supervisor mode
            csr_sr_r[`SR_SIE_R]  = 1'b0;

            // Raise priviledge to supervisor level
            csr_mpriv_r          = `PRIV_SUPER;

            // LSU fault - PC has already advanced
            if (exception_m_w)
                csr_sepc_r       = pc_m_q;
            // Instruction page fault
            else if (exc_src_w[`MCAUSE_PAGE_FAULT_INST])
                csr_sepc_r       = opcode_pc_i;
            // Previous instruction was EBREAK, ERET, ECALL
            // Target inst not executed, so return here later
            else if (branch_csr_request_o)
                csr_sepc_r   = branch_csr_pc_o;
            // Taking interrupt instead of executing instruction handled by this functional unit?
            // Target inst not executed, so return here later
            else if (valid_unit_inst_w)
                csr_sepc_r   = opcode_pc_i;
            // Branch request executed in exec (not squashed)
            else if (branch_exec_request_i)
                csr_sepc_r   = branch_exec_pc_i;
            // Valid instruction, executed by another unit
            else if (opcode_valid_i)
                csr_sepc_r   = opcode_pc_i + 32'd4;
            // Valid instruction not ready, re-run it later
            else
                csr_sepc_r   = opcode_pc_i;       

            // M-stage take priority
            if (exc_src_w[`MCAUSE_MISALIGNED_LOAD])
                csr_scause_r = `MCAUSE_MISALIGNED_LOAD;
            else if (exc_src_w[`MCAUSE_MISALIGNED_STORE])
                csr_scause_r = `MCAUSE_MISALIGNED_STORE;
            else if (exc_src_w[`MCAUSE_PAGE_FAULT_LOAD])
                csr_scause_r = `MCAUSE_PAGE_FAULT_LOAD;
            else if (exc_src_w[`MCAUSE_PAGE_FAULT_STORE])
                csr_scause_r = `MCAUSE_PAGE_FAULT_STORE;
            // Ex-Stage
            else if (exc_src_w[`MCAUSE_PAGE_FAULT_INST])
                csr_scause_r = `MCAUSE_PAGE_FAULT_INST;
            else if (exc_src_w[`MCAUSE_ILLEGAL_INSTRUCTION])
                csr_scause_r = `MCAUSE_ILLEGAL_INSTRUCTION;
            else if (exc_src_w[`MCAUSE_FAULT_FETCH])
                csr_scause_r = `MCAUSE_FAULT_FETCH;
            else if (exc_src_w[`MCAUSE_FAULT_LOAD])
                csr_scause_r = `MCAUSE_FAULT_LOAD;
            else if (exc_src_w[`MCAUSE_FAULT_STORE])
                csr_scause_r = `MCAUSE_FAULT_STORE;

            // Memory fault
            if (exception_m_w)
                csr_stval_r = fault_addr_i;
            // Instruction page fault
            else if (exc_src_w[`MCAUSE_PAGE_FAULT_INST])
                csr_stval_r = opcode_pc_i;
            // Not set...
            else
                csr_stval_r = 32'b0;
        end
        else
        begin
            // Save interrupt / supervisor state
            csr_sr_r[`SR_MPIE_R] = csr_sr_r[`SR_MIE_R];
            csr_sr_r[`SR_MPP_R]  = csr_mpriv_q;

            // Disable interrupts and enter supervisor mode
            csr_sr_r[`SR_MIE_R]  = 1'b0;

            // Raise priviledge to machine level
            csr_mpriv_r          = `PRIV_MACHINE;

            // LSU fault - PC has already advanced
            if (exception_m_w)
                csr_mepc_r   = pc_m_q;
            // Instruction page fault
            else if (exc_src_w[`MCAUSE_PAGE_FAULT_INST])
                csr_mepc_r   = opcode_pc_i;
            // Previous instruction was EBREAK, ERET, ECALL
            // Target inst not executed, so return here later
            else if (branch_csr_request_o)
                csr_mepc_r   = branch_csr_pc_o;
            // Taking interrupt instead of executing instruction handled by this functional unit?
            // Target inst not executed, so return here later
            else if (valid_unit_inst_w)
                csr_mepc_r   = opcode_pc_i;
            // Branch request executed in exec (not squashed)
            else if (branch_exec_request_i)
                csr_mepc_r   = branch_exec_pc_i;
            // Valid instruction, executed by another unit
            else if (opcode_valid_i)
                csr_mepc_r   = opcode_pc_i + 32'd4;
            // Valid instruction not ready, re-run it later
            else
                csr_mepc_r   = opcode_pc_i;

            // M-stage take priority
            if (exc_src_w[`MCAUSE_MISALIGNED_LOAD])
                csr_mcause_r = `MCAUSE_MISALIGNED_LOAD;
            else if (exc_src_w[`MCAUSE_MISALIGNED_STORE])
                csr_mcause_r = `MCAUSE_MISALIGNED_STORE;
            else if (exc_src_w[`MCAUSE_PAGE_FAULT_LOAD])
                csr_mcause_r = `MCAUSE_PAGE_FAULT_LOAD;
            else if (exc_src_w[`MCAUSE_PAGE_FAULT_STORE])
                csr_mcause_r = `MCAUSE_PAGE_FAULT_STORE;
            // Ex-Stage
            else if (exc_src_w[`MCAUSE_PAGE_FAULT_INST])
                csr_mcause_r = `MCAUSE_PAGE_FAULT_INST;
            else if (exc_src_w[`MCAUSE_ILLEGAL_INSTRUCTION])
                csr_mcause_r = `MCAUSE_ILLEGAL_INSTRUCTION;
            else if (exc_src_w[`MCAUSE_FAULT_FETCH])
                csr_mcause_r = `MCAUSE_FAULT_FETCH;
            else if (exc_src_w[`MCAUSE_FAULT_LOAD])
                csr_mcause_r = `MCAUSE_FAULT_LOAD;
            else if (exc_src_w[`MCAUSE_FAULT_STORE])
                csr_mcause_r = `MCAUSE_FAULT_STORE;
        end
    end
    // We will be taking an interrupt, record the reason
    else if (take_intr_r)
    begin
        // Interrupt delegated to supervisor mode
        if (csr_mpriv_q <= `PRIV_SUPER && interrupt_s_r)
        begin
            // Save interrupt / supervisor state
            csr_sr_r[`SR_SPIE_R] = csr_sr_r[`SR_SIE_R];
            csr_sr_r[`SR_SPP_R]  = (csr_mpriv_q == `PRIV_SUPER);

            // Disable interrupts and enter supervisor mode
            csr_sr_r[`SR_SIE_R]  = 1'b0;

            // Raise priviledge to supervisor level
            csr_mpriv_r          = `PRIV_SUPER;

            // Previous instruction was EBREAK, ERET, ECALL
            // Target inst not executed, so return here later
            if (branch_csr_request_o)
                csr_sepc_r   = branch_csr_pc_o;
            // Taking interrupt instead of executing instruction handled by this functional unit?
            // Target inst not executed, so return here later
            else if (valid_unit_inst_w)
                csr_sepc_r   = opcode_pc_i;
            // Branch request executed in exec (not squashed)
            else if (branch_exec_request_i)
                csr_sepc_r   = branch_exec_pc_i;
            // Valid instruction, executed by another unit
            else if (opcode_valid_i)
                csr_sepc_r   = opcode_pc_i + 32'd4;
            // Valid instruction not ready, re-run it later
            else
                csr_sepc_r   = opcode_pc_i;

            // NOTE: Lowest interrupt number wins (in this implementation)
            if (irq_masked_r[`IRQ_S_SOFT])
                csr_scause_r = `MCAUSE_INTERRUPT + `IRQ_S_SOFT;
            else if (irq_masked_r[`IRQ_M_SOFT])
                csr_scause_r = `MCAUSE_INTERRUPT + `IRQ_M_SOFT;
            else if (irq_masked_r[`IRQ_S_TIMER])
                csr_scause_r = `MCAUSE_INTERRUPT + `IRQ_S_TIMER;
            else if (irq_masked_r[`IRQ_M_TIMER])
                csr_scause_r = `MCAUSE_INTERRUPT + `IRQ_M_TIMER;
            else if (irq_masked_r[`IRQ_S_EXT])
                csr_scause_r = `MCAUSE_INTERRUPT + `IRQ_S_EXT;
            else if (irq_masked_r[`IRQ_M_EXT])
                csr_scause_r = `MCAUSE_INTERRUPT + `IRQ_M_EXT;

            // Not set...
            csr_stval_r = 32'b0;
        end
        else
        begin
            // Save interrupt / supervisor state
            csr_sr_r[`SR_MPIE_R] = csr_sr_r[`SR_MIE_R];
            csr_sr_r[`SR_MPP_R]  = csr_mpriv_q;

            // Disable interrupts and enter supervisor mode
            csr_sr_r[`SR_MIE_R]  = 1'b0;

            // Raise priviledge to machine level
            csr_mpriv_r          = `PRIV_MACHINE;

            // Previous instruction was EBREAK, ERET, ECALL
            // Target inst not executed, so return here later
            if (branch_csr_request_o)
                csr_mepc_r   = branch_csr_pc_o;
            // Taking interrupt instead of executing instruction handled by this functional unit?
            // Target inst not executed, so return here later
            else if (valid_unit_inst_w)
                csr_mepc_r   = opcode_pc_i;
            // Branch request executed in exec (not squashed)
            else if (branch_exec_request_i)
                csr_mepc_r   = branch_exec_pc_i;
            // Valid instruction, executed by another unit
            else if (opcode_valid_i)
                csr_mepc_r   = opcode_pc_i + 32'd4;
            // Valid instruction not ready, re-run it later
            else
                csr_mepc_r   = opcode_pc_i;

            // NOTE: Lowest interrupt number wins (in this implementation)
            if (irq_masked_r[`IRQ_S_SOFT])
                csr_mcause_r = `MCAUSE_INTERRUPT + `IRQ_S_SOFT;
            else if (irq_masked_r[`IRQ_M_SOFT])
                csr_mcause_r = `MCAUSE_INTERRUPT + `IRQ_M_SOFT;
            else if (irq_masked_r[`IRQ_S_TIMER])
                csr_mcause_r = `MCAUSE_INTERRUPT + `IRQ_S_TIMER;
            else if (irq_masked_r[`IRQ_M_TIMER])
                csr_mcause_r = `MCAUSE_INTERRUPT + `IRQ_M_TIMER;
            else if (irq_masked_r[`IRQ_S_EXT])
                csr_mcause_r = `MCAUSE_INTERRUPT + `IRQ_S_EXT;
            else if (irq_masked_r[`IRQ_M_EXT])
                csr_mcause_r = `MCAUSE_INTERRUPT + `IRQ_M_EXT;
        end
    end
    // CSR modify instruction
    else if (opcode_valid_i && (set_r || clr_r))
    begin
        data_r = (opcode_instr_i[`ENUM_INST_CSRRWI] | opcode_instr_i[`ENUM_INST_CSRRSI] | opcode_instr_i[`ENUM_INST_CSRRCI]) ?
                           {27'b0, opcode_ra_idx_i} : opcode_ra_operand_i;

        case (imm12_r[11:0])
        `CSR_MSCRATCH:
        begin
            data_r   = data_r     & `CSR_MSCRATCH_MASK;
            result_r = csr_mscratch_q & `CSR_MSCRATCH_MASK;

            if (set_r && clr_r)
                csr_mscratch_r = data_r;
            else if (set_r)
                csr_mscratch_r = csr_mscratch_r | data_r;
            else if (clr_r)
                csr_mscratch_r = csr_mscratch_r & ~data_r;
        end
        `CSR_MEPC:
        begin
            data_r   = data_r     & `CSR_MEPC_MASK;
            result_r = csr_mepc_q & `CSR_MEPC_MASK;

            if (set_r && clr_r)
                csr_mepc_r = data_r;
            else if (set_r)
                csr_mepc_r = csr_mepc_r | data_r;
            else if (clr_r)
                csr_mepc_r = csr_mepc_r & ~data_r;
        end
        `CSR_MTVEC:
        begin
            data_r   = data_r     & `CSR_MTVEC_MASK;
            result_r = csr_mtvec_q & `CSR_MTVEC_MASK;

            if (set_r && clr_r)
                csr_mtvec_r = data_r;
            else if (set_r)
                csr_mtvec_r = csr_mtvec_r | data_r;
            else if (clr_r)
                csr_mtvec_r = csr_mtvec_r & ~data_r;
        end
        `CSR_MCAUSE:
        begin
            data_r   = data_r     & `CSR_MCAUSE_MASK;
            result_r = csr_mcause_q & `CSR_MCAUSE_MASK;

            if (set_r && clr_r)
                csr_mcause_r = data_r;
            else if (set_r)
                csr_mcause_r = csr_mcause_r | data_r;
            else if (clr_r)
                csr_mcause_r = csr_mcause_r & ~data_r;
        end
        `CSR_MSTATUS:
        begin
            data_r   = data_r     & `CSR_MSTATUS_MASK;
            result_r = csr_sr_q & `CSR_MSTATUS_MASK;

            if (set_r && clr_r)
                csr_sr_r = data_r;
            else if (set_r)
                csr_sr_r = csr_sr_r | data_r;
            else if (clr_r)
                csr_sr_r = csr_sr_r & ~data_r;
        end
        `CSR_MIP:
        begin        
            data_r   = data_r    & `CSR_MIP_MASK;
            result_r = csr_mip_r & `CSR_MIP_MASK; // Local version
            if (set_r && clr_r)
                csr_mip_r = data_r;
            else if (set_r)
                csr_mip_r = csr_mip_r | data_r;
            else if (clr_r)
                csr_mip_r = csr_mip_r & ~data_r;
        end
        `CSR_MIE:
        begin
            data_r   = data_r     & `CSR_MIE_MASK;
            result_r = csr_mie_q & `CSR_MIE_MASK;

            if (set_r && clr_r)
                csr_mie_r = data_r;
            else if (set_r)
                csr_mie_r = csr_mie_r | data_r;
            else if (clr_r)
                csr_mie_r = csr_mie_r & ~data_r;
        end
        `CSR_MCYCLE:
        begin
            result_r = csr_mcycle_q;
        end
        `CSR_MHARTID:
        begin
            result_r = cpu_id_i;
        end
        `CSR_MISA:
        begin
            result_r = `MISA_RV32 | `MISA_RVI
                       | `MISA_RVM
                       | `MISA_RVS | `MISA_RVU
                       ;
        end
        `CSR_MEDELEG:
        begin
            data_r   = data_r     & `CSR_MEDELEG_MASK;
            result_r = csr_medeleg_q & `CSR_MEDELEG_MASK;

            if (set_r && clr_r)
                csr_medeleg_r = data_r;
            else if (set_r)
                csr_medeleg_r = csr_medeleg_r | data_r;
            else if (clr_r)
                csr_medeleg_r = csr_medeleg_r & ~data_r;
        end
        `CSR_MIDELEG:
        begin
            data_r   = data_r     & `CSR_MIDELEG_MASK;
            result_r = csr_mideleg_q & `CSR_MIDELEG_MASK;

            if (set_r && clr_r)
                csr_mideleg_r = data_r;
            else if (set_r)
                csr_mideleg_r = csr_mideleg_r | data_r;
            else if (clr_r)
                csr_mideleg_r = csr_mideleg_r & ~data_r;
        end
        `CSR_SEPC:
        begin
            data_r   = data_r     & `CSR_SEPC_MASK;
            result_r = csr_sepc_q & `CSR_SEPC_MASK;

            if (set_r && clr_r)
                csr_sepc_r = data_r;
            else if (set_r)
                csr_sepc_r = csr_sepc_r | data_r;
            else if (clr_r)
                csr_sepc_r = csr_sepc_r & ~data_r;
        end
        `CSR_STVEC:
        begin
            data_r   = data_r     & `CSR_STVEC_MASK;
            result_r = csr_stvec_q & `CSR_STVEC_MASK;

            if (set_r && clr_r)
                csr_stvec_r = data_r;
            else if (set_r)
                csr_stvec_r = csr_stvec_r | data_r;
            else if (clr_r)
                csr_stvec_r = csr_stvec_r & ~data_r;
        end
        `CSR_SCAUSE:
        begin
            data_r   = data_r     & `CSR_SCAUSE_MASK;
            result_r = csr_scause_q & `CSR_SCAUSE_MASK;

            if (set_r && clr_r)
                csr_scause_r = data_r;
            else if (set_r)
                csr_scause_r = csr_scause_r | data_r;
            else if (clr_r)
                csr_scause_r = csr_scause_r & ~data_r;
        end
        `CSR_SIP:
        begin
            data_r   = data_r    & `CSR_SIP_MASK;
            result_r = csr_mip_r & `CSR_SIP_MASK;

            if (set_r && clr_r)
                csr_mip_r = data_r;
            else if (set_r)
                csr_mip_r = csr_mip_r | data_r;
            else if (clr_r)
                csr_mip_r = csr_mip_r & ~data_r;
        end
        `CSR_SIE:
        begin
            data_r   = data_r     & `CSR_SIE_MASK;
            result_r = csr_mie_q & `CSR_SIE_MASK;

            if (set_r && clr_r)
                csr_mie_r = data_r;
            else if (set_r)
                csr_mie_r = csr_mie_r | data_r;
            else if (clr_r)
                csr_mie_r = csr_mie_r & ~data_r;
        end
        `CSR_STVAL:
        begin
            data_r   = data_r     & `CSR_STVAL_MASK;
            result_r = csr_stval_q & `CSR_STVAL_MASK;

            if (set_r && clr_r)
                csr_stval_r = data_r;
            else if (set_r)
                csr_stval_r = csr_stval_r | data_r;
            else if (clr_r)
                csr_stval_r = csr_stval_r & ~data_r;
        end
        `CSR_SATP:
        begin
            data_r   = data_r     & `CSR_SATP_MASK;
            result_r = csr_satp_q & `CSR_SATP_MASK;

            if (set_r && clr_r)
                csr_satp_r = data_r;
            else if (set_r)
                csr_satp_r = csr_satp_r | data_r;
            else if (clr_r)
                csr_satp_r = csr_satp_r & ~data_r;
        end
        `CSR_SSCRATCH:
        begin
            data_r   = data_r     & `CSR_SSCRATCH_MASK;
            result_r = csr_sscratch_q & `CSR_SSCRATCH_MASK;

            if (set_r && clr_r)
                csr_sscratch_r = data_r;
            else if (set_r)
                csr_sscratch_r = csr_sscratch_r | data_r;
            else if (clr_r)
                csr_sscratch_r = csr_sscratch_r & ~data_r;
        end
        `CSR_SSTATUS:
        begin
            data_r   = data_r     & `CSR_SSTATUS_MASK;
            result_r = csr_sr_q & `CSR_SSTATUS_MASK;

            if (set_r && clr_r)
                csr_sr_r = data_r;
            else if (set_r)
                csr_sr_r = csr_sr_r | data_r;
            else if (clr_r)
                csr_sr_r = csr_sr_r & ~data_r;
        end
        default:
            ;
        endcase
    end
    // System Call / Breakpoint
    else if (opcode_valid_i && (opcode_instr_i[`ENUM_INST_ECALL] | opcode_instr_i[`ENUM_INST_EBREAK]))
    begin
        // Exception delegated to supervisor mode
        if ((csr_mpriv_q <= `PRIV_SUPER) && (|(exc_src_w & csr_medeleg_q)))
        begin
            // Save interrupt / supervisor state
            csr_sr_r[`SR_SPIE_R] = csr_sr_q[`SR_SIE_R];
            csr_sr_r[`SR_SPP_R]  = (csr_mpriv_q == `PRIV_SUPER);

            // Disable interrupts and enter supervisor mode
            csr_sr_r[`SR_SIE_R]  = 1'b0;

            // Raise priviledge to supervisor level
            csr_mpriv_r          = `PRIV_SUPER;

            // Save PC of next instruction (not yet executed)
            csr_sepc_r           = opcode_pc_i;

            // Exception source
            if (opcode_instr_i[`ENUM_INST_EBREAK])
                csr_scause_r     = `MCAUSE_BREAKPOINT;
            else
                csr_scause_r     = `MCAUSE_ECALL_U + {30'b0, csr_mpriv_q};

            // Supervisor Trap Value is cleared for these exceptions
            csr_stval_r = 32'b0;
        end
        else
        begin
            // Save interrupt / supervisor state
            csr_sr_r[`SR_MPIE_R] = csr_sr_r[`SR_MIE_R];
            csr_sr_r[`SR_MPP_R]  = csr_mpriv_q;

            // Disable interrupts and enter supervisor mode
            csr_sr_r[`SR_MIE_R]  = 1'b0;

            // Raise priviledge to machine level
            csr_mpriv_r          = `PRIV_MACHINE;

            // Save PC of next instruction (not yet executed)
            csr_mepc_r           = opcode_pc_i;

            // Exception source
            if (opcode_instr_i[`ENUM_INST_EBREAK])
                csr_mcause_r     = `MCAUSE_BREAKPOINT;
            else
                csr_mcause_r     = `MCAUSE_ECALL_U + {30'b0, csr_mpriv_q};
        end
    end
    // Return from interrupt
    else if (opcode_valid_i && opcode_instr_i[`ENUM_INST_ERET])
    begin
        // MRET (return from machine)
        if (opcode_opcode_i[`INST_MRET_R])
        begin
            // Set privilege level to previous MPP
            csr_mpriv_r          = csr_sr_r[`SR_MPP_R];

            // Interrupt enable pop
            csr_sr_r[`SR_MIE_R]  = csr_sr_r[`SR_MPIE_R];
            csr_sr_r[`SR_MPIE_R] = 1'b1;

            // Set next MPP to user mode
            csr_sr_r[`SR_MPP_R] = `SR_MPP_U;
        end
        // SRET (return from supervisor)
        else
        begin
            // Set privilege level to previous privilege level
            csr_mpriv_r          = csr_sr_r[`SR_SPP_R] ? `PRIV_SUPER : `PRIV_USER;

            // Interrupt enable pop
            csr_sr_r[`SR_SIE_R]  = csr_sr_r[`SR_SPIE_R];
            csr_sr_r[`SR_SPIE_R] = 1'b1;

            // Set next SPP to user mode
            csr_sr_r[`SR_SPP_R] = 1'b0;
        end
    end

    // External interrupts
    if (intr_i)
    begin
        // External interrupts handled by supervisor
        if (csr_mideleg_q[`SR_IP_SEIP_R])
            csr_mip_r[`SR_IP_SEIP_R] = 1'b1;
        // External interrupts handled by machine
        else
            csr_mip_r[`SR_IP_MEIP_R] = 1'b1;
    end    
end

//-----------------------------------------------------------------
// Sequential
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    csr_mepc_q         <= 32'b0;
    csr_sr_q           <= 32'b0;
    csr_mcause_q       <= 32'b0;
    csr_mtvec_q        <= 32'b0;
    csr_mip_q          <= 32'b0;
    csr_mie_q          <= 32'b0;
    csr_mpriv_q        <= `PRIV_MACHINE;
    csr_mcycle_q       <= 32'b0;
    csr_mscratch_q     <= 32'b0;
    csr_medeleg_q      <= 32'b0;
    csr_mideleg_q      <= 32'b0;
    csr_sepc_q         <= 32'b0;
    csr_stvec_q        <= 32'b0;
    csr_scause_q       <= 32'b0;
    csr_stval_q        <= 32'b0;
    csr_satp_q         <= 32'b0;
    csr_sscratch_q     <= 32'b0;
end
else
begin
    csr_mepc_q         <= csr_mepc_r;
    csr_sr_q           <= csr_sr_r;
    csr_mcause_q       <= csr_mcause_r;
    csr_mtvec_q        <= csr_mtvec_r;
    csr_mip_q          <= csr_mip_r;
    csr_mie_q          <= csr_mie_r;
    csr_mpriv_q        <= csr_mpriv_r;
    csr_mcycle_q       <= csr_mcycle_r;
    csr_mscratch_q     <= csr_mscratch_r;
    csr_medeleg_q      <= csr_medeleg_r;
    csr_mideleg_q      <= csr_mideleg_r;
    csr_sepc_q         <= csr_sepc_r;
    csr_stvec_q        <= csr_stvec_r;
    csr_scause_q       <= csr_scause_r;
    csr_stval_q        <= csr_stval_r;
    csr_satp_q         <= csr_satp_r;
    csr_sscratch_q     <= csr_sscratch_r;

`ifdef verilator
    // CSR SIM_CTRL
    if (opcode_valid_i && (set_r || clr_r) && (imm12_r[11:0] == `CSR_DSCRATCH) && !csr_access_fault_w)
    begin
        case (data_r & 32'hFF000000)
        `CSR_SIM_CTRL_EXIT:
        begin
            //exit(data_r[7:0]);
            $finish;
            $finish;
        end
        `CSR_SIM_CTRL_PUTC:
        begin
            $write("%c", data_r[7:0]);
        end
        endcase
    end
`endif
end

//-----------------------------------------------------------------
// Writeback
//-----------------------------------------------------------------
reg          writeback_en_q;
reg [  4:0]  writeback_idx_q;
reg [ 31:0]  writeback_value_q;
reg          writeback_squash_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    writeback_en_q     <= 1'b0;
    writeback_idx_q    <= 5'b0;
    writeback_value_q  <= 32'b0;
    writeback_squash_q <= 1'b0;
    pc_m_q             <= 32'b0;
end
else
begin
    if (opcode_valid_i && ~stall_o)
    begin
        writeback_en_q    <= (set_r || clr_r);
        writeback_idx_q   <= opcode_rd_idx_i;
        writeback_value_q <= result_r;
        pc_m_q            <= opcode_pc_i;
    end
    else
    begin
        writeback_en_q    <= 1'b0;
    end

    // Scoreboard will have been allocated so de-allocate if an allocated
    // instruction was aborted
    writeback_squash_q <= (valid_unit_inst_w & ((~exception_m_w & take_exception_r) | take_intr_r)) | fault_page_load_i | fault_misaligned_load_i;
end

assign writeback_idx_o    = {5{writeback_en_q | writeback_squash_q}} & writeback_idx_q;
assign writeback_value_o  = writeback_value_q;
assign writeback_squash_o = writeback_squash_q;

// Stall on load/store fault (as the following instruction is not executed)
assign stall_o           = fault_page_load_i | fault_page_store_i | fault_misaligned_load_i | fault_misaligned_store_i;

//-----------------------------------------------------------------
// Execute - Branch operations
//-----------------------------------------------------------------
reg        branch_r;
reg [31:0] branch_target_r;

always @ *
begin
    branch_r        = 1'b0;
    branch_target_r = 32'b0;

    if (take_exception_r)
    begin
        branch_r        = 1'b1;

        if (csr_mpriv_q <= `PRIV_SUPER && exception_s_r)
            branch_target_r = csr_stvec_q;
        else
            branch_target_r = csr_mtvec_q;
    end
    else if (take_intr_r)
    begin
        branch_r        = 1'b1;

        if (csr_mpriv_q <= `PRIV_SUPER && interrupt_s_r)
            branch_target_r = csr_stvec_q;
        else
            branch_target_r = csr_mtvec_q;
    end
    else if (opcode_instr_i[`ENUM_INST_ECALL])
    begin
        branch_r        = opcode_valid_i;

        // Exception delegated to supervisor mode
        if ((csr_mpriv_q <= `PRIV_SUPER) && (|(exc_src_w & csr_medeleg_q)))
            branch_target_r = csr_stvec_q;
        else
            branch_target_r = csr_mtvec_q;
    end
    else if (opcode_instr_i[`ENUM_INST_EBREAK])
    begin
        branch_r        = opcode_valid_i;

        // Exception delegated to supervisor mode
        if ((csr_mpriv_q <= `PRIV_SUPER) && (|(exc_src_w & csr_medeleg_q)))
            branch_target_r = csr_stvec_q;
        else
            branch_target_r = csr_mtvec_q;
    end
    else if (opcode_instr_i[`ENUM_INST_ERET])
    begin
        branch_r        = opcode_valid_i;

        // SRET (return from super)
        if (!opcode_opcode_i[`INST_MRET_R])
            branch_target_r = csr_sepc_q;
        else
            branch_target_r = csr_mepc_q;
    end

end

reg        branch_q;
reg [31:0] branch_target_q;
reg        reset_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    branch_target_q <= 32'b0;
    branch_q        <= 1'b0;
    reset_q         <= 1'b1;
end
else if (reset_q)
begin
    branch_target_q <= reset_vector_i;
    branch_q        <= 1'b1;
    reset_q         <= 1'b0;
end
else
begin
    branch_target_q <= branch_target_r;
    branch_q        <= branch_r;
end

assign branch_csr_request_o = branch_q;
assign branch_csr_pc_o      = branch_target_q;

wire   mmu_enable_w     = (csr_mpriv_q != `PRIV_MACHINE) && csr_satp_q[31];
assign mmu_satp_o       = {mmu_enable_w, csr_satp_q[30:0]};
assign mmu_flush_o      = (opcode_valid_i && (set_r || clr_r) && (imm12_r[11:0] == `CSR_SATP));
assign mmu_supervisor_o = (csr_mpriv_q == `PRIV_SUPER);
assign mmu_sum_o        = csr_sr_q[`SR_SUM_R];




endmodule
