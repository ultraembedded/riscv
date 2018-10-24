//-----------------------------------------------------------------
//                         RISC-V Core
//                            V0.6
//                     Ultra-Embedded.com
//                     Copyright 2014-2018
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
module riscv_csr
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input           intr_i
    ,input           opcode_valid_i
    ,input  [ 55:0]  opcode_instr_i
    ,input  [ 31:0]  opcode_opcode_i
    ,input  [ 31:0]  opcode_pc_i
    ,input  [  4:0]  opcode_rd_idx_i
    ,input  [  4:0]  opcode_ra_idx_i
    ,input  [  4:0]  opcode_rb_idx_i
    ,input  [ 31:0]  opcode_ra_operand_i
    ,input  [ 31:0]  opcode_rb_operand_i
    ,input  [ 31:0]  cpu_id_i
    ,input           fault_fetch_i
    ,input           fault_store_i
    ,input           fault_load_i
    ,input  [ 31:0]  fault_addr_i

    // Outputs
    ,output [  4:0]  writeback_idx_o
    ,output [ 31:0]  writeback_value_o
    ,output          stall_o
    ,output [ 31:0]  csr_epc_o
    ,output [ 31:0]  csr_evec_o
    ,output          take_interrupt_o
);



//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-------------------------------------------------------------
// Registers / Wires
//-------------------------------------------------------------
reg [31:0]  csr_epc_q;
reg [31:0]  csr_cause_q;
reg [31:0]  csr_sr_q;
reg [31:0]  csr_evec_q;
reg [31:0]  csr_ip_q;
reg [31:0]  csr_ie_q;
reg [31:0]  csr_time_q;
reg [31:0]  csr_timecmp_q;

reg         take_interrupt_q;

//-------------------------------------------------------------
// CSR handling
//-------------------------------------------------------------
reg [31:0]  imm12_r;
reg         set_r;
reg         clr_r;

reg [31:0]  csr_epc_r;
reg [31:0]  csr_cause_r;
reg [31:0]  csr_sr_r;
reg [31:0]  csr_evec_r;
reg [31:0]  csr_ip_r;
reg [31:0]  csr_ie_r;
reg [31:0]  csr_time_r;
reg [31:0]  csr_timecmp_r;

reg         take_intr_r;

reg [31:0]  data_r;
reg [31:0]  result_r;

always @ *
begin
    imm12_r     = {{20{opcode_opcode_i[31]}}, opcode_opcode_i[31:20]};

    set_r = opcode_instr_i[`ENUM_INST_CSRRW] | opcode_instr_i[`ENUM_INST_CSRRS] | opcode_instr_i[`ENUM_INST_CSRRWI] | opcode_instr_i[`ENUM_INST_CSRRSI];
    clr_r = opcode_instr_i[`ENUM_INST_CSRRW] | opcode_instr_i[`ENUM_INST_CSRRC] | opcode_instr_i[`ENUM_INST_CSRRWI] | opcode_instr_i[`ENUM_INST_CSRRCI];

    take_intr_r    = 1'b0;
    csr_epc_r      = csr_epc_q;
    csr_sr_r       = csr_sr_q;
    csr_cause_r    = csr_cause_q;
    csr_evec_r     = csr_evec_q;
    csr_ip_r       = csr_ip_q;
    csr_ie_r       = csr_ie_q;
    csr_time_r     = csr_time_q;
    csr_timecmp_r  = csr_timecmp_q;

    result_r       = 32'b0;
    data_r         = 32'b0;

    csr_time_r = csr_time_q + 32'd1;

    // Timer should generate a interrupt?
    if (csr_time_r == csr_timecmp_r)
        csr_ip_r =  csr_ip_r | `SR_IP_MTIP;

    // CSR modify instruction
    if (opcode_valid_i && (set_r || clr_r))
    begin
        data_r = (opcode_instr_i[`ENUM_INST_CSRRWI] | opcode_instr_i[`ENUM_INST_CSRRSI] | opcode_instr_i[`ENUM_INST_CSRRCI]) ?
                           {27'b0, opcode_ra_idx_i} : opcode_ra_operand_i;

        case (imm12_r[11:0])
        `CSR_MSCRATCH:
            ;
        `CSR_MEPC:
        begin
            data_r = data_r & `CSR_MEPC_MASK;
            result_r = csr_epc_q;
            if (set_r && clr_r)
                csr_epc_r = data_r;
            else if (set_r)
                csr_epc_r = csr_epc_r | data_r;
            else if (clr_r)
                csr_epc_r = csr_epc_r & ~data_r;
        end
        `CSR_MTVEC:
        begin
            data_r = data_r & `CSR_MTVEC_MASK;
            result_r = csr_evec_q;
            if (set_r && clr_r)
                csr_evec_r = data_r;
            else if (set_r)
                csr_evec_r = csr_evec_r | data_r;
            else if (clr_r)
                csr_evec_r = csr_evec_r & ~data_r;
        end
        `CSR_MCAUSE:
        begin
            data_r = data_r & `CSR_MCAUSE_MASK;
            result_r = csr_cause_q;
            if (set_r && clr_r)
                csr_cause_r = data_r;
            else if (set_r)
                csr_cause_r = csr_cause_r | data_r;
            else if (clr_r)
                csr_cause_r = csr_cause_r & ~data_r;
        end
        `CSR_MSTATUS:
        begin
            data_r = data_r & `CSR_MSTATUS_MASK;
            result_r = csr_sr_q;
            if (set_r && clr_r)
                csr_sr_r = data_r;
            else if (set_r)
                csr_sr_r = csr_sr_r | data_r;
            else if (clr_r)
                csr_sr_r = csr_sr_r & ~data_r;
        end
        `CSR_MIP:
        begin        
            data_r = data_r & `CSR_MIP_MASK;
            result_r = csr_ip_r; // Local version
            if (set_r && clr_r)
                csr_ip_r = data_r;
            else if (set_r)
                csr_ip_r = csr_ip_r | data_r;
            else if (clr_r)
                csr_ip_r = csr_ip_r & ~data_r;
        end
        `CSR_MIE:
        begin
            data_r = data_r & `CSR_MIE_MASK;
            result_r = csr_ie_q;
            if (set_r && clr_r)
                csr_ie_r = data_r;
            else if (set_r)
                csr_ie_r = csr_ie_r | data_r;
            else if (clr_r)
                csr_ie_r = csr_ie_r & ~data_r;
        end
        `CSR_MTIME:
        begin
            data_r = data_r & `CSR_MTIME_MASK;
            result_r      = csr_time_q; // Return flopped state

            // Non-std behaviour - write to CSR_TIME gives next interrupt threshold
            if (set_r)
            begin
                csr_timecmp_r = data_r;

                // Clear interrupt pending
                csr_ip_r = csr_ip_r & ~`SR_IP_MTIP;
            end
        end
        `CSR_MHARTID:
        begin
            result_r = cpu_id_i;
        end
        default:
            ;
        endcase
    end
    // System Call
    else if (opcode_valid_i && opcode_instr_i[`ENUM_INST_ECALL])
    begin
        csr_cause_r = `MCAUSE_ECALL_U;
        csr_epc_r   = opcode_pc_i;

        // Interrupt save and disable
        csr_sr_r = csr_sr_r & ~`SR_MPIE;
        csr_sr_r = csr_sr_r | (csr_sr_r[`SR_MIE_BIT] ? `SR_MPIE : 0);
        csr_sr_r = csr_sr_r & ~`SR_MIE;
    end
    // Return from interrupt
    else if (opcode_valid_i && opcode_instr_i[`ENUM_INST_MRET])
    begin
        // Interrupt enable pop
        csr_sr_r = csr_sr_r & ~`SR_MIE;
        csr_sr_r = csr_sr_r | (csr_sr_r[`SR_MPIE_BIT] ? `SR_MIE : 0);
        csr_sr_r = csr_sr_r | `SR_MPIE;
    end
    // Take interrupt
    else if (opcode_valid_i && opcode_instr_i[`ENUM_INST_INTR])
    begin
        csr_epc_r   = opcode_pc_i;

        // Interrupt save and disable
        csr_sr_r = csr_sr_r & ~`SR_MPIE;
        csr_sr_r = csr_sr_r | (csr_sr_r[`SR_MIE_BIT] ? `SR_MPIE : 0);
        csr_sr_r = csr_sr_r & ~`SR_MIE;
    end

    // External interrupts
    csr_ip_r = csr_ip_r | (intr_i ? `SR_IP_MEIP : 0);

    take_intr_r = (((csr_ip_r & csr_ie_q) != 32'd0) && csr_sr_r[`SR_MIE_BIT]) ||
                    fault_fetch_i || fault_store_i || fault_load_i;

    // We will be taking an interrupt, record the reason
    if (take_intr_r)
    begin
        if (fault_fetch_i)
            csr_cause_r = `MCAUSE_FAULT_FETCH;
        else if (fault_load_i)
            csr_cause_r = `MCAUSE_FAULT_LOAD;
        else if (fault_store_i)
            csr_cause_r = `MCAUSE_FAULT_STORE;
        // NOTE: Lowest interrupt number wins
        else if (((csr_ip_r & csr_ie_q) & (1 << `IRQ_SOFT)) != 32'd0)
            csr_cause_r = `MCAUSE_INTERRUPT + `IRQ_SOFT;
        else if (((csr_ip_r & csr_ie_q) & (1 << `IRQ_TIMER)) != 32'd0)
            csr_cause_r = `MCAUSE_INTERRUPT + `IRQ_TIMER;
        else if (((csr_ip_r & csr_ie_q) & (1 << `IRQ_EXT)) != 32'd0)
            csr_cause_r = `MCAUSE_INTERRUPT + `IRQ_EXT;
    end
end

//-------------------------------------------------------------
// Sequential
//-------------------------------------------------------------
reg [  4:0]  writeback_idx_q;
reg [ 31:0]  writeback_value_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    csr_epc_q         <= 32'b0;
    csr_sr_q          <= 32'b0;
    csr_cause_q       <= 32'b0;
    csr_evec_q        <= 32'b0;
    csr_ip_q          <= 32'b0;
    csr_ie_q          <= 32'b0;
    csr_time_q        <= 32'b0;
    csr_timecmp_q     <= 32'b0;
    writeback_idx_q   <= 5'b0;
    writeback_value_q <= 32'b0;
    take_interrupt_q  <= 1'b0;
end
else
begin
    csr_epc_q         <= csr_epc_r;
    csr_sr_q          <= csr_sr_r;
    csr_cause_q       <= csr_cause_r;
    csr_evec_q        <= csr_evec_r;
    csr_ip_q          <= csr_ip_r;
    csr_ie_q          <= csr_ie_r;
    csr_time_q        <= csr_time_r;
    csr_timecmp_q     <= csr_timecmp_r;
    take_interrupt_q  <= take_intr_r;

    if (opcode_valid_i && (set_r || clr_r))
    begin
        writeback_idx_q   <= opcode_rd_idx_i;
        writeback_value_q <= result_r;
    end
    else
    begin
        writeback_idx_q   <= 5'b0;
        writeback_value_q <= 32'b0;
    end

`ifdef verilator
    // CSR SIM_CTRL
    if (opcode_valid_i && (set_r || clr_r) && (imm12_r[11:0] == `CSR_DSCRATCH))
    begin
        case (data_r & 32'hFF000000)
        `CSR_SIM_CTRL_EXIT:
        begin
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

assign writeback_idx_o   = writeback_idx_q;
assign writeback_value_o = writeback_value_q;

assign take_interrupt_o  = take_interrupt_q;
assign stall_o           = 1'b0; // Not used

assign csr_epc_o         = csr_epc_q;
assign csr_evec_o        = csr_evec_q;


endmodule
