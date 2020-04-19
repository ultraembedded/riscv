//-----------------------------------------------------------------
//                         RISC-V Core
//                            V1.0
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

module riscv_issue
//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
#(
     parameter SUPPORT_MULDIV   = 1
    ,parameter SUPPORT_DUAL_ISSUE = 1
    ,parameter SUPPORT_LOAD_BYPASS = 1
    ,parameter SUPPORT_MUL_BYPASS = 1
    ,parameter SUPPORT_REGFILE_XILINX = 0
)
//-----------------------------------------------------------------
// Ports
//-----------------------------------------------------------------
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input           fetch_valid_i
    ,input  [ 31:0]  fetch_instr_i
    ,input  [ 31:0]  fetch_pc_i
    ,input           fetch_fault_fetch_i
    ,input           fetch_fault_page_i
    ,input           fetch_instr_exec_i
    ,input           fetch_instr_lsu_i
    ,input           fetch_instr_branch_i
    ,input           fetch_instr_mul_i
    ,input           fetch_instr_div_i
    ,input           fetch_instr_csr_i
    ,input           fetch_instr_rd_valid_i
    ,input           fetch_instr_invalid_i
    ,input           branch_exec_request_i
    ,input           branch_exec_is_taken_i
    ,input           branch_exec_is_not_taken_i
    ,input  [ 31:0]  branch_exec_source_i
    ,input           branch_exec_is_call_i
    ,input           branch_exec_is_ret_i
    ,input           branch_exec_is_jmp_i
    ,input  [ 31:0]  branch_exec_pc_i
    ,input           branch_d_exec_request_i
    ,input  [ 31:0]  branch_d_exec_pc_i
    ,input  [  1:0]  branch_d_exec_priv_i
    ,input           branch_csr_request_i
    ,input  [ 31:0]  branch_csr_pc_i
    ,input  [  1:0]  branch_csr_priv_i
    ,input  [ 31:0]  writeback_exec_value_i
    ,input           writeback_mem_valid_i
    ,input  [ 31:0]  writeback_mem_value_i
    ,input  [  5:0]  writeback_mem_exception_i
    ,input  [ 31:0]  writeback_mul_value_i
    ,input           writeback_div_valid_i
    ,input  [ 31:0]  writeback_div_value_i
    ,input  [ 31:0]  csr_result_e1_value_i
    ,input           csr_result_e1_write_i
    ,input  [ 31:0]  csr_result_e1_wdata_i
    ,input  [  5:0]  csr_result_e1_exception_i
    ,input           lsu_stall_i
    ,input           take_interrupt_i

    // Outputs
    ,output          fetch_accept_o
    ,output          branch_request_o
    ,output [ 31:0]  branch_pc_o
    ,output [  1:0]  branch_priv_o
    ,output          exec_opcode_valid_o
    ,output          lsu_opcode_valid_o
    ,output          csr_opcode_valid_o
    ,output          mul_opcode_valid_o
    ,output          div_opcode_valid_o
    ,output [ 31:0]  opcode_opcode_o
    ,output [ 31:0]  opcode_pc_o
    ,output          opcode_invalid_o
    ,output [  4:0]  opcode_rd_idx_o
    ,output [  4:0]  opcode_ra_idx_o
    ,output [  4:0]  opcode_rb_idx_o
    ,output [ 31:0]  opcode_ra_operand_o
    ,output [ 31:0]  opcode_rb_operand_o
    ,output [ 31:0]  lsu_opcode_opcode_o
    ,output [ 31:0]  lsu_opcode_pc_o
    ,output          lsu_opcode_invalid_o
    ,output [  4:0]  lsu_opcode_rd_idx_o
    ,output [  4:0]  lsu_opcode_ra_idx_o
    ,output [  4:0]  lsu_opcode_rb_idx_o
    ,output [ 31:0]  lsu_opcode_ra_operand_o
    ,output [ 31:0]  lsu_opcode_rb_operand_o
    ,output [ 31:0]  mul_opcode_opcode_o
    ,output [ 31:0]  mul_opcode_pc_o
    ,output          mul_opcode_invalid_o
    ,output [  4:0]  mul_opcode_rd_idx_o
    ,output [  4:0]  mul_opcode_ra_idx_o
    ,output [  4:0]  mul_opcode_rb_idx_o
    ,output [ 31:0]  mul_opcode_ra_operand_o
    ,output [ 31:0]  mul_opcode_rb_operand_o
    ,output [ 31:0]  csr_opcode_opcode_o
    ,output [ 31:0]  csr_opcode_pc_o
    ,output          csr_opcode_invalid_o
    ,output [  4:0]  csr_opcode_rd_idx_o
    ,output [  4:0]  csr_opcode_ra_idx_o
    ,output [  4:0]  csr_opcode_rb_idx_o
    ,output [ 31:0]  csr_opcode_ra_operand_o
    ,output [ 31:0]  csr_opcode_rb_operand_o
    ,output          csr_writeback_write_o
    ,output [ 11:0]  csr_writeback_waddr_o
    ,output [ 31:0]  csr_writeback_wdata_o
    ,output [  5:0]  csr_writeback_exception_o
    ,output [ 31:0]  csr_writeback_exception_pc_o
    ,output [ 31:0]  csr_writeback_exception_addr_o
    ,output          exec_hold_o
    ,output          mul_hold_o
    ,output          interrupt_inhibit_o
);



`include "riscv_defs.v"

wire enable_muldiv_w     = SUPPORT_MULDIV;
wire enable_mul_bypass_w = SUPPORT_MUL_BYPASS;

wire stall_w;
wire squash_w;

//-------------------------------------------------------------
// Priv level
//-------------------------------------------------------------
reg [1:0] priv_x_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    priv_x_q <= `PRIV_MACHINE;
else if (branch_csr_request_i)
    priv_x_q <= branch_csr_priv_i;

//-------------------------------------------------------------
// Issue Select
//-------------------------------------------------------------
wire opcode_valid_w = fetch_valid_i & ~squash_w & ~branch_csr_request_i;

// Branch request (CSR branch - ecall, xret, or branch instruction)
assign branch_request_o     = branch_csr_request_i | branch_d_exec_request_i;
assign branch_pc_o          = branch_csr_request_i ? branch_csr_pc_i   : branch_d_exec_pc_i;
assign branch_priv_o        = branch_csr_request_i ? branch_csr_priv_i : priv_x_q;

//-------------------------------------------------------------
// Instruction Decoder
//-------------------------------------------------------------
wire [4:0] issue_ra_idx_w   = fetch_instr_i[19:15];
wire [4:0] issue_rb_idx_w   = fetch_instr_i[24:20];
wire [4:0] issue_rd_idx_w   = fetch_instr_i[11:7];
wire       issue_sb_alloc_w = fetch_instr_rd_valid_i;
wire       issue_exec_w     = fetch_instr_exec_i;
wire       issue_lsu_w      = fetch_instr_lsu_i;
wire       issue_branch_w   = fetch_instr_branch_i;
wire       issue_mul_w      = fetch_instr_mul_i;
wire       issue_div_w      = fetch_instr_div_i;
wire       issue_csr_w      = fetch_instr_csr_i;
wire       issue_invalid_w  = fetch_instr_invalid_i;

//-------------------------------------------------------------
// Pipeline status tracking
//------------------------------------------------------------- 
wire        pipe_squash_e1_e2_w;

reg         opcode_issue_r;
reg         opcode_accept_r;
wire        pipe_stall_raw_w;

wire        pipe_load_e1_w;
wire        pipe_store_e1_w;
wire        pipe_mul_e1_w;
wire        pipe_branch_e1_w;
wire [4:0]  pipe_rd_e1_w;

wire [31:0] pipe_pc_e1_w;
wire [31:0] pipe_opcode_e1_w;
wire [31:0] pipe_operand_ra_e1_w;
wire [31:0] pipe_operand_rb_e1_w;

wire        pipe_load_e2_w;
wire        pipe_mul_e2_w;
wire [4:0]  pipe_rd_e2_w;
wire [31:0] pipe_result_e2_w;

wire        pipe_valid_wb_w;
wire        pipe_csr_wb_w;
wire [4:0]  pipe_rd_wb_w;
wire [31:0] pipe_result_wb_w;
wire [31:0] pipe_pc_wb_w;
wire [31:0] pipe_opc_wb_w;
wire [31:0] pipe_ra_val_wb_w;
wire [31:0] pipe_rb_val_wb_w;
wire [`EXCEPTION_W-1:0] pipe_exception_wb_w;

wire [`EXCEPTION_W-1:0] issue_fault_w = fetch_fault_fetch_i ? `EXCEPTION_FAULT_FETCH:
                                        fetch_fault_page_i  ? `EXCEPTION_PAGE_FAULT_INST: `EXCEPTION_W'b0;

riscv_pipe_ctrl
#( 
     .SUPPORT_LOAD_BYPASS(SUPPORT_LOAD_BYPASS)
    ,.SUPPORT_MUL_BYPASS(SUPPORT_MUL_BYPASS)
)
u_pipe_ctrl
(
     .clk_i(clk_i)
    ,.rst_i(rst_i)    

    // Issue
    ,.issue_valid_i(opcode_issue_r)
    ,.issue_accept_i(opcode_accept_r)
    ,.issue_stall_i(stall_w)
    ,.issue_lsu_i(issue_lsu_w)
    ,.issue_csr_i(issue_csr_w)
    ,.issue_div_i(issue_div_w)
    ,.issue_mul_i(issue_mul_w)
    ,.issue_branch_i(issue_branch_w)
    ,.issue_rd_valid_i(issue_sb_alloc_w)
    ,.issue_rd_i(issue_rd_idx_w)
    ,.issue_exception_i(issue_fault_w)
    ,.issue_pc_i(opcode_pc_o)
    ,.issue_opcode_i(opcode_opcode_o)
    ,.issue_operand_ra_i(opcode_ra_operand_o)
    ,.issue_operand_rb_i(opcode_rb_operand_o)
    ,.issue_branch_taken_i(branch_d_exec_request_i)
    ,.issue_branch_target_i(branch_d_exec_pc_i)
    ,.take_interrupt_i(take_interrupt_i)

    // Execution stage 1: ALU result
    ,.alu_result_e1_i(writeback_exec_value_i)
    ,.csr_result_value_e1_i(csr_result_e1_value_i)
    ,.csr_result_write_e1_i(csr_result_e1_write_i)
    ,.csr_result_wdata_e1_i(csr_result_e1_wdata_i)
    ,.csr_result_exception_e1_i(csr_result_e1_exception_i)

    // Execution stage 1
    ,.load_e1_o(pipe_load_e1_w)
    ,.store_e1_o(pipe_store_e1_w)
    ,.mul_e1_o(pipe_mul_e1_w)
    ,.branch_e1_o(pipe_branch_e1_w)
    ,.rd_e1_o(pipe_rd_e1_w)
    ,.pc_e1_o(pipe_pc_e1_w)
    ,.opcode_e1_o(pipe_opcode_e1_w)
    ,.operand_ra_e1_o(pipe_operand_ra_e1_w)
    ,.operand_rb_e1_o(pipe_operand_rb_e1_w)

    // Execution stage 2: Other results
    ,.mem_complete_i(writeback_mem_valid_i)
    ,.mem_result_e2_i(writeback_mem_value_i)
    ,.mem_exception_e2_i(writeback_mem_exception_i)
    ,.mul_result_e2_i(writeback_mul_value_i)

    // Execution stage 2
    ,.load_e2_o(pipe_load_e2_w)
    ,.mul_e2_o(pipe_mul_e2_w)
    ,.rd_e2_o(pipe_rd_e2_w)
    ,.result_e2_o(pipe_result_e2_w)

    ,.stall_o(pipe_stall_raw_w)
    ,.squash_e1_e2_o(pipe_squash_e1_e2_w)
    ,.squash_e1_e2_i(1'b0)
    ,.squash_wb_i(1'b0)

    // Out of pipe: Divide Result
    ,.div_complete_i(writeback_div_valid_i)
    ,.div_result_i(writeback_div_value_i)

    // Commit
    ,.valid_wb_o(pipe_valid_wb_w)
    ,.csr_wb_o(pipe_csr_wb_w)
    ,.rd_wb_o(pipe_rd_wb_w)
    ,.result_wb_o(pipe_result_wb_w)
    ,.pc_wb_o(pipe_pc_wb_w)
    ,.opcode_wb_o(pipe_opc_wb_w)
    ,.operand_ra_wb_o(pipe_ra_val_wb_w)
    ,.operand_rb_wb_o(pipe_rb_val_wb_w)
    ,.exception_wb_o(pipe_exception_wb_w)
    ,.csr_write_wb_o(csr_writeback_write_o)
    ,.csr_waddr_wb_o(csr_writeback_waddr_o)
    ,.csr_wdata_wb_o(csr_writeback_wdata_o)   
);

assign exec_hold_o = stall_w;
assign mul_hold_o  = stall_w;

//-------------------------------------------------------------
// Pipe1 - Status tracking
//-------------------------------------------------------------
assign csr_writeback_exception_o      = pipe_exception_wb_w;
assign csr_writeback_exception_pc_o   = pipe_pc_wb_w;
assign csr_writeback_exception_addr_o = pipe_result_wb_w;

//-------------------------------------------------------------
// Blocking events (division, CSR unit access)
//-------------------------------------------------------------
reg div_pending_q;
reg csr_pending_q;

// Division operations take 2 - 34 cycles and stall
// the pipeline (complete out-of-pipe) until completed.
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    div_pending_q <= 1'b0;
else if (pipe_squash_e1_e2_w)
    div_pending_q <= 1'b0;
else if (div_opcode_valid_o && issue_div_w)
    div_pending_q <= 1'b1;
else if (writeback_div_valid_i)
    div_pending_q <= 1'b0;

// CSR operations are infrequent - avoid any complications of pipelining them.
// These only take a 2-3 cycles anyway and may result in a pipe flush (e.g. ecall, ebreak..).
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    csr_pending_q <= 1'b0;
else if (pipe_squash_e1_e2_w)
    csr_pending_q <= 1'b0;
else if (csr_opcode_valid_o && issue_csr_w)
    csr_pending_q <= 1'b1;
else if (pipe_csr_wb_w)
    csr_pending_q <= 1'b0;

assign squash_w = pipe_squash_e1_e2_w;

//-------------------------------------------------------------
// Issue / scheduling logic
//-------------------------------------------------------------
reg [31:0] scoreboard_r;

always @ *
begin
    opcode_issue_r     = 1'b0;
    opcode_accept_r    = 1'b0;
    scoreboard_r       = 32'b0;

    // Execution units with >= 2 cycle latency
    if (SUPPORT_LOAD_BYPASS == 0)
    begin
        if (pipe_load_e2_w)
            scoreboard_r[pipe_rd_e2_w] = 1'b1;
    end
    if (SUPPORT_MUL_BYPASS == 0)
    begin
        if (pipe_mul_e2_w)
            scoreboard_r[pipe_rd_e2_w] = 1'b1;
    end

    // Execution units with >= 1 cycle latency (loads / multiply)
    if (pipe_load_e1_w || pipe_mul_e1_w)
        scoreboard_r[pipe_rd_e1_w] = 1'b1;

    // Do not start multiply, division or CSR operation in the cycle after a load (leaving only ALU operations and branches)
    if ((pipe_load_e1_w || pipe_store_e1_w) && (issue_mul_w || issue_div_w || issue_csr_w))
        scoreboard_r = 32'hFFFFFFFF;

    // Stall - no issues...
    if (lsu_stall_i || stall_w || div_pending_q || csr_pending_q)
        ;
    // Primary slot (lsu, branch, alu, mul, div, csr)
    else if (opcode_valid_w &&
        !(scoreboard_r[issue_ra_idx_w] || 
          scoreboard_r[issue_rb_idx_w] ||
          scoreboard_r[issue_rd_idx_w]))
    begin
        opcode_issue_r  = 1'b1;
        opcode_accept_r = 1'b1;

        if (opcode_accept_r && issue_sb_alloc_w && (|issue_rd_idx_w))
            scoreboard_r[issue_rd_idx_w] = 1'b1;
    end 
end

assign lsu_opcode_valid_o   = opcode_issue_r & ~take_interrupt_i;
assign exec_opcode_valid_o  = opcode_issue_r;
assign mul_opcode_valid_o   = enable_muldiv_w & opcode_issue_r;
assign div_opcode_valid_o   = enable_muldiv_w & opcode_issue_r;
assign interrupt_inhibit_o  = csr_pending_q || issue_csr_w;

assign fetch_accept_o       = opcode_valid_w ? (opcode_accept_r & ~take_interrupt_i) : 1'b1;

assign stall_w              = pipe_stall_raw_w;

//-------------------------------------------------------------
// Register File
//------------------------------------------------------------- 
wire [31:0] issue_ra_value_w;
wire [31:0] issue_rb_value_w;
wire [31:0] issue_b_ra_value_w;
wire [31:0] issue_b_rb_value_w;

// Register file: 1W2R
riscv_regfile
#(
     .SUPPORT_REGFILE_XILINX(SUPPORT_REGFILE_XILINX)
)
u_regfile
(
    .clk_i(clk_i),
    .rst_i(rst_i),

    // Write ports
    .rd0_i(pipe_rd_wb_w),
    .rd0_value_i(pipe_result_wb_w),

    // Read ports
    .ra0_i(issue_ra_idx_w),
    .rb0_i(issue_rb_idx_w),
    .ra0_value_o(issue_ra_value_w),
    .rb0_value_o(issue_rb_value_w)
);

//-------------------------------------------------------------
// Issue Slot 0
//------------------------------------------------------------- 
assign opcode_opcode_o = fetch_instr_i;
assign opcode_pc_o     = fetch_pc_i;
assign opcode_rd_idx_o = issue_rd_idx_w;
assign opcode_ra_idx_o = issue_ra_idx_w;
assign opcode_rb_idx_o = issue_rb_idx_w;
assign opcode_invalid_o= 1'b0; 

reg [31:0] issue_ra_value_r;
reg [31:0] issue_rb_value_r;

always @ *
begin
    // NOTE: Newest version of operand takes priority
    issue_ra_value_r = issue_ra_value_w;
    issue_rb_value_r = issue_rb_value_w;

    // Bypass - WB
    if (pipe_rd_wb_w == issue_ra_idx_w)
        issue_ra_value_r = pipe_result_wb_w;
    if (pipe_rd_wb_w == issue_rb_idx_w)
        issue_rb_value_r = pipe_result_wb_w;

    // Bypass - E2
    if (pipe_rd_e2_w == issue_ra_idx_w)
        issue_ra_value_r = pipe_result_e2_w;
    if (pipe_rd_e2_w == issue_rb_idx_w)
        issue_rb_value_r = pipe_result_e2_w;

    // Bypass - E1
    if (pipe_rd_e1_w == issue_ra_idx_w)
        issue_ra_value_r = writeback_exec_value_i;
    if (pipe_rd_e1_w == issue_rb_idx_w)
        issue_rb_value_r = writeback_exec_value_i;

    // Reg 0 source
    if (issue_ra_idx_w == 5'b0)
        issue_ra_value_r = 32'b0;
    if (issue_rb_idx_w == 5'b0)
        issue_rb_value_r = 32'b0;
end

assign opcode_ra_operand_o = issue_ra_value_r;
assign opcode_rb_operand_o = issue_rb_value_r;

//-------------------------------------------------------------
// Load store unit
//-------------------------------------------------------------
assign lsu_opcode_opcode_o      = opcode_opcode_o;
assign lsu_opcode_pc_o          = opcode_pc_o;
assign lsu_opcode_rd_idx_o      = opcode_rd_idx_o;
assign lsu_opcode_ra_idx_o      = opcode_ra_idx_o;
assign lsu_opcode_rb_idx_o      = opcode_rb_idx_o;
assign lsu_opcode_ra_operand_o  = opcode_ra_operand_o;
assign lsu_opcode_rb_operand_o  = opcode_rb_operand_o;
assign lsu_opcode_invalid_o     = 1'b0;

//-------------------------------------------------------------
// Multiply
//-------------------------------------------------------------
assign mul_opcode_opcode_o      = opcode_opcode_o;
assign mul_opcode_pc_o          = opcode_pc_o;
assign mul_opcode_rd_idx_o      = opcode_rd_idx_o;
assign mul_opcode_ra_idx_o      = opcode_ra_idx_o;
assign mul_opcode_rb_idx_o      = opcode_rb_idx_o;
assign mul_opcode_ra_operand_o  = opcode_ra_operand_o;
assign mul_opcode_rb_operand_o  = opcode_rb_operand_o;
assign mul_opcode_invalid_o     = 1'b0;

//-------------------------------------------------------------
// CSR unit
//-------------------------------------------------------------
assign csr_opcode_valid_o       = opcode_issue_r & ~take_interrupt_i;
assign csr_opcode_opcode_o      = opcode_opcode_o;
assign csr_opcode_pc_o          = opcode_pc_o;
assign csr_opcode_rd_idx_o      = opcode_rd_idx_o;
assign csr_opcode_ra_idx_o      = opcode_ra_idx_o;
assign csr_opcode_rb_idx_o      = opcode_rb_idx_o;
assign csr_opcode_ra_operand_o  = opcode_ra_operand_o;
assign csr_opcode_rb_operand_o  = opcode_rb_operand_o;
assign csr_opcode_invalid_o     = opcode_issue_r && issue_invalid_w;

//-------------------------------------------------------------
// Checker Interface
//-------------------------------------------------------------
`ifdef verilator
riscv_trace_sim
u_pipe_dec0_verif
(
     .valid_i(pipe_valid_wb_w)
    ,.pc_i(pipe_pc_wb_w)
    ,.opcode_i(pipe_opc_wb_w)
);

wire [4:0] v_pipe_rs1_w = pipe_opc_wb_w[19:15];
wire [4:0] v_pipe_rs2_w = pipe_opc_wb_w[24:20];

function [0:0] complete_valid0; /*verilator public*/
begin
    complete_valid0 = pipe_valid_wb_w;
end
endfunction
function [31:0] complete_pc0; /*verilator public*/
begin
    complete_pc0 = pipe_pc_wb_w;
end
endfunction
function [31:0] complete_opcode0; /*verilator public*/
begin
    complete_opcode0 = pipe_opc_wb_w;
end
endfunction
function [4:0] complete_ra0; /*verilator public*/
begin
    complete_ra0 = v_pipe_rs1_w;
end
endfunction
function [4:0] complete_rb0; /*verilator public*/
begin
    complete_rb0 = v_pipe_rs2_w;
end
endfunction
function [4:0] complete_rd0; /*verilator public*/
begin
    complete_rd0 = pipe_rd_wb_w;
end
endfunction
function [31:0] complete_ra_val0; /*verilator public*/
begin
    complete_ra_val0 = pipe_ra_val_wb_w;
end
endfunction
function [31:0] complete_rb_val0; /*verilator public*/
begin
    complete_rb_val0 = pipe_rb_val_wb_w;
end
endfunction
function [31:0] complete_rd_val0; /*verilator public*/
begin
    if (|pipe_rd_wb_w)
        complete_rd_val0 = pipe_result_wb_w;
    else
        complete_rd_val0 = 32'b0;
end
endfunction
function [5:0] complete_exception; /*verilator public*/
begin
    complete_exception = pipe_exception_wb_w;
end
endfunction
`endif


endmodule
