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

module riscv_fetch
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input           fetch_branch_i
    ,input  [ 31:0]  fetch_branch_pc_i
    ,input           fetch_accept_i
    ,input           icache_accept_i
    ,input           icache_valid_i
    ,input           icache_error_i
    ,input  [ 31:0]  icache_inst_i
    ,input           fetch_invalidate_i

    // Outputs
    ,output          fetch_valid_o
    ,output [ 31:0]  fetch_instr_o
    ,output [ 31:0]  fetch_pc_o
    ,output          icache_rd_o
    ,output          icache_flush_o
    ,output          icache_invalidate_o
    ,output [ 31:0]  icache_pc_o
);



//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-------------------------------------------------------------
// Registers / Wires
//-------------------------------------------------------------
reg         active_q;
reg [31:0]  fetch_pc_q;

reg [31:0]  branch_pc_q;
reg         branch_valid_q;

reg         icache_fetch_q;
reg         icache_invalidate_q;

reg [63:0]  skid_buffer_q;
reg         skid_valid_q;

wire        icache_busy_w =  icache_fetch_q && !icache_valid_i;
wire        stall_w       = !fetch_accept_i || icache_busy_w || !icache_accept_i;

wire        branch_w      = branch_valid_q || fetch_branch_i;
wire [31:0] branch_pc_w   = (branch_valid_q & !fetch_branch_i) ? branch_pc_q : fetch_branch_pc_i;

//-------------------------------------------------------------
// Sequential
//-------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    fetch_pc_q     <= 32'b0;

    branch_pc_q    <= 32'b0;
    branch_valid_q <= 1'b0;

    icache_fetch_q <= 1'b0;

    skid_buffer_q  <= 64'b0;
    skid_valid_q   <= 1'b0;
    active_q       <= 1'b0;
end
else
begin
    // Branch request skid buffer
    if (stall_w || !active_q)
    begin
        branch_valid_q <= branch_w;
        branch_pc_q    <= branch_pc_w;
    end
    else
    begin
        branch_valid_q <= 1'b0;
        branch_pc_q    <= 32'b0;
    end

    if (branch_w)
        active_q <= 1'b1;

    // NPC
    if (!stall_w)
        fetch_pc_q <= icache_pc_o + 32'd4;

    // Instruction output back-pressured - hold in skid buffer
    if (fetch_valid_o && !fetch_accept_i)
    begin
        skid_valid_q  <= 1'b1;
        skid_buffer_q <= {fetch_pc_o, fetch_instr_o};
    end
    else
    begin
        skid_valid_q  <= 1'b0;
        skid_buffer_q <= 64'b0;
    end

    // ICACHE fetch tracking
    if (icache_rd_o && icache_accept_i)
        icache_fetch_q <= 1'b1;
    else if (icache_valid_i)
        icache_fetch_q <= 1'b0;
end

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    icache_invalidate_q <= 1'b0;
else if (icache_invalidate_o && !icache_accept_i)
    icache_invalidate_q <= 1'b1;
else
    icache_invalidate_q <= 1'b0;

reg [31:0] pc_d_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    pc_d_q <= 32'b0;
else if (icache_rd_o && icache_accept_i)
    pc_d_q <= icache_pc_o;


//-------------------------------------------------------------
// Outputs
//-------------------------------------------------------------
assign icache_rd_o         = active_q & fetch_accept_i & !icache_busy_w;
assign icache_pc_o         = branch_w ? branch_pc_w : fetch_pc_q;
assign icache_flush_o      = fetch_invalidate_i | icache_invalidate_q;
assign icache_invalidate_o = 1'b0;


// On fault, insert known invalid opcode into the pipeline
wire [31:0] instruction_w  = icache_error_i ? `INST_FAULT : icache_inst_i;

assign fetch_valid_o = (icache_valid_i || skid_valid_q) & !branch_w;
assign fetch_pc_o    = skid_valid_q ? skid_buffer_q[63:32] : pc_d_q;
assign fetch_instr_o = skid_valid_q ? skid_buffer_q[31:0]  : instruction_w;


endmodule
