//-----------------------------------------------------------------
//                         RISC-V Core
//                            V1.0.1
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

module riscv_decode
//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
#(
     parameter SUPPORT_MULDIV   = 1
    ,parameter EXTRA_DECODE_STAGE = 0
)
//-----------------------------------------------------------------
// Ports
//-----------------------------------------------------------------
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input           fetch_in_valid_i
    ,input  [ 31:0]  fetch_in_instr_i
    ,input  [ 31:0]  fetch_in_pc_i
    ,input           fetch_in_fault_fetch_i
    ,input           fetch_in_fault_page_i
    ,input           fetch_out_accept_i
    ,input           squash_decode_i

    // Outputs
    ,output          fetch_in_accept_o
    ,output          fetch_out_valid_o
    ,output [ 31:0]  fetch_out_instr_o
    ,output [ 31:0]  fetch_out_pc_o
    ,output          fetch_out_fault_fetch_o
    ,output          fetch_out_fault_page_o
    ,output          fetch_out_instr_exec_o
    ,output          fetch_out_instr_lsu_o
    ,output          fetch_out_instr_branch_o
    ,output          fetch_out_instr_mul_o
    ,output          fetch_out_instr_div_o
    ,output          fetch_out_instr_csr_o
    ,output          fetch_out_instr_rd_valid_o
    ,output          fetch_out_instr_invalid_o
);



wire        enable_muldiv_w     = SUPPORT_MULDIV;

//-----------------------------------------------------------------
// Extra decode stage (to improve cycle time)
//-----------------------------------------------------------------
generate
if (EXTRA_DECODE_STAGE)
begin
    wire [31:0] fetch_in_instr_w = (fetch_in_fault_page_i | fetch_in_fault_fetch_i) ? 32'b0 : fetch_in_instr_i;
    reg [66:0]  buffer_q;

    always @(posedge clk_i or posedge rst_i)
    if (rst_i)
        buffer_q <= 67'b0;
    else if (squash_decode_i)
        buffer_q <= 67'b0;
    else if (fetch_out_accept_i || !fetch_out_valid_o)
        buffer_q <= {fetch_in_valid_i, fetch_in_fault_page_i, fetch_in_fault_fetch_i, fetch_in_instr_w, fetch_in_pc_i};

    assign {fetch_out_valid_o,
            fetch_out_fault_page_o,
            fetch_out_fault_fetch_o,
            fetch_out_instr_o,
            fetch_out_pc_o} = buffer_q;

    riscv_decoder
    u_dec
    (
         .valid_i(fetch_out_valid_o)
        ,.fetch_fault_i(fetch_out_fault_page_o | fetch_out_fault_fetch_o)
        ,.enable_muldiv_i(enable_muldiv_w)
        ,.opcode_i(fetch_out_instr_o)

        ,.invalid_o(fetch_out_instr_invalid_o)
        ,.exec_o(fetch_out_instr_exec_o)
        ,.lsu_o(fetch_out_instr_lsu_o)
        ,.branch_o(fetch_out_instr_branch_o)
        ,.mul_o(fetch_out_instr_mul_o)
        ,.div_o(fetch_out_instr_div_o)
        ,.csr_o(fetch_out_instr_csr_o)
        ,.rd_valid_o(fetch_out_instr_rd_valid_o)
    );

    assign fetch_in_accept_o        = fetch_out_accept_i;
end
//-----------------------------------------------------------------
// Straight through decode
//-----------------------------------------------------------------
else
begin
    wire [31:0] fetch_in_instr_w = (fetch_in_fault_page_i | fetch_in_fault_fetch_i) ? 32'b0 : fetch_in_instr_i;

    riscv_decoder
    u_dec
    (
         .valid_i(fetch_in_valid_i)
        ,.fetch_fault_i(fetch_in_fault_fetch_i | fetch_in_fault_page_i)
        ,.enable_muldiv_i(enable_muldiv_w)
        ,.opcode_i(fetch_out_instr_o)

        ,.invalid_o(fetch_out_instr_invalid_o)
        ,.exec_o(fetch_out_instr_exec_o)
        ,.lsu_o(fetch_out_instr_lsu_o)
        ,.branch_o(fetch_out_instr_branch_o)
        ,.mul_o(fetch_out_instr_mul_o)
        ,.div_o(fetch_out_instr_div_o)
        ,.csr_o(fetch_out_instr_csr_o)
        ,.rd_valid_o(fetch_out_instr_rd_valid_o)
    );

    // Outputs
    assign fetch_out_valid_o        = fetch_in_valid_i;
    assign fetch_out_pc_o           = fetch_in_pc_i;
    assign fetch_out_instr_o        = fetch_in_instr_w;
    assign fetch_out_fault_page_o   = fetch_in_fault_page_i;
    assign fetch_out_fault_fetch_o  = fetch_in_fault_fetch_i;

    assign fetch_in_accept_o        = fetch_out_accept_i;
end
endgenerate


endmodule
