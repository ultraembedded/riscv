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

module riscv_muldiv
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input           opcode_valid_i
    ,input  [ 57:0]  opcode_instr_i
    ,input  [ 31:0]  opcode_opcode_i
    ,input  [ 31:0]  opcode_pc_i
    ,input  [  4:0]  opcode_rd_idx_i
    ,input  [  4:0]  opcode_ra_idx_i
    ,input  [  4:0]  opcode_rb_idx_i
    ,input  [ 31:0]  opcode_ra_operand_i
    ,input  [ 31:0]  opcode_rb_operand_i

    // Outputs
    ,output [  4:0]  writeback_idx_o
    ,output          writeback_squash_o
    ,output [ 31:0]  writeback_value_o
    ,output          stall_o
);



//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-------------------------------------------------------------
// Registers / Wires
//-------------------------------------------------------------

reg  [4:0]   rd_q;
reg  [31:0]  wb_result_q;
reg  [4:0]   wb_rd_q;

//-------------------------------------------------------------
// Multiplier
//-------------------------------------------------------------
wire [64:0]  mult_result_w;
reg  [32:0]  operand_b_r;
reg  [32:0]  operand_a_r;
reg  [31:0]  result_r;
reg  [31:0]  mult_result_q;
reg          mult_busy_q;


wire mult_inst_w    = opcode_instr_i[`ENUM_INST_MUL]     || 
                      opcode_instr_i[`ENUM_INST_MULH]    ||
                      opcode_instr_i[`ENUM_INST_MULHSU]  ||
                      opcode_instr_i[`ENUM_INST_MULHU];

// Multiplier takes 1 full cycle, with the result appearing on 
// writeback the cycle after...
always @(posedge clk_i or posedge rst_i)
if (rst_i)
    mult_busy_q <= 1'b0;
else if (opcode_valid_i & !stall_o)
    mult_busy_q <= mult_inst_w;
else
    mult_busy_q <= 1'b0;

always @ *
begin
    if (opcode_instr_i[`ENUM_INST_MULHSU])
        operand_a_r = {opcode_ra_operand_i[31], opcode_ra_operand_i[31:0]};
    else if (opcode_instr_i[`ENUM_INST_MULH])
        operand_a_r = {opcode_ra_operand_i[31], opcode_ra_operand_i[31:0]};
    else // ENUM_INST_MULHU || ENUM_INST_MUL
        operand_a_r = {1'b0, opcode_ra_operand_i[31:0]};
end

always @ *
begin
    if (opcode_instr_i[`ENUM_INST_MULHSU])
        operand_b_r = {1'b0, opcode_rb_operand_i[31:0]};
    else if (opcode_instr_i[`ENUM_INST_MULH])
        operand_b_r = {opcode_rb_operand_i[31], opcode_rb_operand_i[31:0]};
    else // ENUM_INST_MULHU || ENUM_INST_MUL
        operand_b_r = {1'b0, opcode_rb_operand_i[31:0]};
end

assign mult_result_w = {{ 32 {operand_a_r[32]}}, operand_a_r}*{{ 32 {operand_b_r[32]}}, operand_b_r};
 
always @ *
begin
    result_r = mult_result_w[31:0];

    case (1'b1)
       opcode_instr_i[`ENUM_INST_MULH], 
       opcode_instr_i[`ENUM_INST_MULHU],
       opcode_instr_i[`ENUM_INST_MULHSU]:
          result_r = mult_result_w[63:32];
       opcode_instr_i[`ENUM_INST_MUL]:
          result_r = mult_result_w[31:0];
    endcase
end

always @(posedge clk_i or posedge rst_i)
if (rst_i)
    mult_result_q <= 32'b0;
else
    mult_result_q <= result_r;

//-------------------------------------------------------------
// Divider
//-------------------------------------------------------------
wire div_rem_inst_w     = opcode_instr_i[`ENUM_INST_DIV]  || 
                          opcode_instr_i[`ENUM_INST_DIVU] ||
                          opcode_instr_i[`ENUM_INST_REM]  ||
                          opcode_instr_i[`ENUM_INST_REMU];

wire signed_operation_w = opcode_instr_i[`ENUM_INST_DIV] || opcode_instr_i[`ENUM_INST_REM];
wire div_operation_w    = opcode_instr_i[`ENUM_INST_DIV] || opcode_instr_i[`ENUM_INST_DIVU];

reg [31:0] dividend_q;
reg [62:0] divisor_q;
reg [31:0] quotient_q;
reg [31:0] q_mask_q;
reg        div_inst_q;
reg        div_busy_q;
reg        invert_res_q;

wire div_start_w    = opcode_valid_i & div_rem_inst_w & !stall_o;
wire div_complete_w = !(|q_mask_q) & div_busy_q;

always @(posedge clk_i or posedge rst_i)
if (rst_i)
begin
    div_busy_q     <= 1'b0;
    dividend_q     <= 32'b0;
    divisor_q      <= 63'b0;
    invert_res_q   <= 1'b0;
    quotient_q     <= 32'b0;
    q_mask_q       <= 32'b0;
    div_inst_q     <= 1'b0;
end 
else if (div_start_w)
begin
    div_busy_q     <= 1'b1;
    div_inst_q     <= div_operation_w;

    if (signed_operation_w && opcode_ra_operand_i[31])
        dividend_q <= -opcode_ra_operand_i;
    else
        dividend_q <= opcode_ra_operand_i;

    if (signed_operation_w && opcode_rb_operand_i[31])
        divisor_q <= {-opcode_rb_operand_i, 31'b0};
    else
        divisor_q <= {opcode_rb_operand_i, 31'b0};

    invert_res_q  <= (opcode_instr_i[`ENUM_INST_DIV] && (opcode_ra_operand_i[31] != opcode_rb_operand_i[31]) && |opcode_rb_operand_i) || 
                     (opcode_instr_i[`ENUM_INST_REM] && opcode_ra_operand_i[31]);

    quotient_q     <= 32'b0;
    q_mask_q       <= 32'h80000000;
end
else if (div_complete_w)
begin
    div_busy_q <= 1'b0;
end
else if (div_busy_q)
begin
    if (divisor_q <= {31'b0, dividend_q})
    begin
        dividend_q <= dividend_q - divisor_q[31:0];
        quotient_q <= quotient_q | q_mask_q;
    end

    divisor_q <= {1'b0, divisor_q[62:1]};
    q_mask_q  <= {1'b0, q_mask_q[31:1]};
end

reg [31:0] div_result_r;
always @ *
begin
    div_result_r = 32'b0;

    if (div_inst_q)
        div_result_r = invert_res_q ? -quotient_q : quotient_q;
    else
        div_result_r = invert_res_q ? -dividend_q : dividend_q;
end

//-------------------------------------------------------------
// Shared logic
//-------------------------------------------------------------

// Stall if divider logic is busy and new multiplier or divider op
assign stall_o = (div_busy_q  & (mult_inst_w | div_rem_inst_w)) ||
                 (mult_busy_q & div_rem_inst_w);

always @(posedge clk_i or posedge rst_i)
if (rst_i)
    rd_q <= 5'b0;
else if (opcode_valid_i && (div_rem_inst_w | mult_inst_w) && !stall_o)
    rd_q <= opcode_rd_idx_i;
else if (!div_busy_q)
    rd_q <= 5'b0;


always @(posedge clk_i or posedge rst_i)
if (rst_i)
    wb_rd_q <= 5'b0;
else if (mult_busy_q)
    wb_rd_q <= rd_q;
else if (div_complete_w)
    wb_rd_q <= rd_q;
else
    wb_rd_q <= 5'b0;

always @(posedge clk_i or posedge rst_i)
if (rst_i)
    wb_result_q <= 32'b0;
else if (div_complete_w)
    wb_result_q <= div_result_r;
else
    wb_result_q <= mult_result_q;

assign writeback_value_o  = wb_result_q;
assign writeback_idx_o    = wb_rd_q;
assign writeback_squash_o = 1'b0;



endmodule
