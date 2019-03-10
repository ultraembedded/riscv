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

module riscv_regfile
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input  [  4:0]  rd0_i
    ,input  [  4:0]  rd1_i
    ,input  [  4:0]  rd2_i
    ,input  [  4:0]  rd3_i
    ,input  [ 31:0]  rd0_value_i
    ,input  [ 31:0]  rd1_value_i
    ,input  [ 31:0]  rd2_value_i
    ,input  [ 31:0]  rd3_value_i
    ,input  [  4:0]  ra_i
    ,input  [  4:0]  rb_i

    // Outputs
    ,output [ 31:0]  ra_value_o
    ,output [ 31:0]  rb_value_o
);



//-----------------------------------------------------------------
// Register file
//-----------------------------------------------------------------
reg [31:0] reg_r1_q;
reg [31:0] reg_r2_q;
reg [31:0] reg_r3_q;
reg [31:0] reg_r4_q;
reg [31:0] reg_r5_q;
reg [31:0] reg_r6_q;
reg [31:0] reg_r7_q;
reg [31:0] reg_r8_q;
reg [31:0] reg_r9_q;
reg [31:0] reg_r10_q;
reg [31:0] reg_r11_q;
reg [31:0] reg_r12_q;
reg [31:0] reg_r13_q;
reg [31:0] reg_r14_q;
reg [31:0] reg_r15_q;
reg [31:0] reg_r16_q;
reg [31:0] reg_r17_q;
reg [31:0] reg_r18_q;
reg [31:0] reg_r19_q;
reg [31:0] reg_r20_q;
reg [31:0] reg_r21_q;
reg [31:0] reg_r22_q;
reg [31:0] reg_r23_q;
reg [31:0] reg_r24_q;
reg [31:0] reg_r25_q;
reg [31:0] reg_r26_q;
reg [31:0] reg_r27_q;
reg [31:0] reg_r28_q;
reg [31:0] reg_r29_q;
reg [31:0] reg_r30_q;
reg [31:0] reg_r31_q;

//-----------------------------------------------------------------
// Flop based register File (for simulation)
//-----------------------------------------------------------------

// Synchronous register write back
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    reg_r1_q       <= 32'h00000000;
    reg_r2_q       <= 32'h00000000;
    reg_r3_q       <= 32'h00000000;
    reg_r4_q       <= 32'h00000000;
    reg_r5_q       <= 32'h00000000;
    reg_r6_q       <= 32'h00000000;
    reg_r7_q       <= 32'h00000000;
    reg_r8_q       <= 32'h00000000;
    reg_r9_q       <= 32'h00000000;
    reg_r10_q      <= 32'h00000000;
    reg_r11_q      <= 32'h00000000;
    reg_r12_q      <= 32'h00000000;
    reg_r13_q      <= 32'h00000000;
    reg_r14_q      <= 32'h00000000;
    reg_r15_q      <= 32'h00000000;
    reg_r16_q      <= 32'h00000000;
    reg_r17_q      <= 32'h00000000;
    reg_r18_q      <= 32'h00000000;
    reg_r19_q      <= 32'h00000000;
    reg_r20_q      <= 32'h00000000;
    reg_r21_q      <= 32'h00000000;
    reg_r22_q      <= 32'h00000000;
    reg_r23_q      <= 32'h00000000;
    reg_r24_q      <= 32'h00000000;
    reg_r25_q      <= 32'h00000000;
    reg_r26_q      <= 32'h00000000;
    reg_r27_q      <= 32'h00000000;
    reg_r28_q      <= 32'h00000000;
    reg_r29_q      <= 32'h00000000;
    reg_r30_q      <= 32'h00000000;
    reg_r31_q      <= 32'h00000000;
end
else
begin
    if      (rd0_i == 5'd1) reg_r1_q <= rd0_value_i;
    else if (rd1_i == 5'd1) reg_r1_q <= rd1_value_i;
    else if (rd2_i == 5'd1) reg_r1_q <= rd2_value_i;
    else if (rd3_i == 5'd1) reg_r1_q <= rd3_value_i;
    if      (rd0_i == 5'd2) reg_r2_q <= rd0_value_i;
    else if (rd1_i == 5'd2) reg_r2_q <= rd1_value_i;
    else if (rd2_i == 5'd2) reg_r2_q <= rd2_value_i;
    else if (rd3_i == 5'd2) reg_r2_q <= rd3_value_i;
    if      (rd0_i == 5'd3) reg_r3_q <= rd0_value_i;
    else if (rd1_i == 5'd3) reg_r3_q <= rd1_value_i;
    else if (rd2_i == 5'd3) reg_r3_q <= rd2_value_i;
    else if (rd3_i == 5'd3) reg_r3_q <= rd3_value_i;
    if      (rd0_i == 5'd4) reg_r4_q <= rd0_value_i;
    else if (rd1_i == 5'd4) reg_r4_q <= rd1_value_i;
    else if (rd2_i == 5'd4) reg_r4_q <= rd2_value_i;
    else if (rd3_i == 5'd4) reg_r4_q <= rd3_value_i;
    if      (rd0_i == 5'd5) reg_r5_q <= rd0_value_i;
    else if (rd1_i == 5'd5) reg_r5_q <= rd1_value_i;
    else if (rd2_i == 5'd5) reg_r5_q <= rd2_value_i;
    else if (rd3_i == 5'd5) reg_r5_q <= rd3_value_i;
    if      (rd0_i == 5'd6) reg_r6_q <= rd0_value_i;
    else if (rd1_i == 5'd6) reg_r6_q <= rd1_value_i;
    else if (rd2_i == 5'd6) reg_r6_q <= rd2_value_i;
    else if (rd3_i == 5'd6) reg_r6_q <= rd3_value_i;
    if      (rd0_i == 5'd7) reg_r7_q <= rd0_value_i;
    else if (rd1_i == 5'd7) reg_r7_q <= rd1_value_i;
    else if (rd2_i == 5'd7) reg_r7_q <= rd2_value_i;
    else if (rd3_i == 5'd7) reg_r7_q <= rd3_value_i;
    if      (rd0_i == 5'd8) reg_r8_q <= rd0_value_i;
    else if (rd1_i == 5'd8) reg_r8_q <= rd1_value_i;
    else if (rd2_i == 5'd8) reg_r8_q <= rd2_value_i;
    else if (rd3_i == 5'd8) reg_r8_q <= rd3_value_i;
    if      (rd0_i == 5'd9) reg_r9_q <= rd0_value_i;
    else if (rd1_i == 5'd9) reg_r9_q <= rd1_value_i;
    else if (rd2_i == 5'd9) reg_r9_q <= rd2_value_i;
    else if (rd3_i == 5'd9) reg_r9_q <= rd3_value_i;
    if      (rd0_i == 5'd10) reg_r10_q <= rd0_value_i;
    else if (rd1_i == 5'd10) reg_r10_q <= rd1_value_i;
    else if (rd2_i == 5'd10) reg_r10_q <= rd2_value_i;
    else if (rd3_i == 5'd10) reg_r10_q <= rd3_value_i;
    if      (rd0_i == 5'd11) reg_r11_q <= rd0_value_i;
    else if (rd1_i == 5'd11) reg_r11_q <= rd1_value_i;
    else if (rd2_i == 5'd11) reg_r11_q <= rd2_value_i;
    else if (rd3_i == 5'd11) reg_r11_q <= rd3_value_i;
    if      (rd0_i == 5'd12) reg_r12_q <= rd0_value_i;
    else if (rd1_i == 5'd12) reg_r12_q <= rd1_value_i;
    else if (rd2_i == 5'd12) reg_r12_q <= rd2_value_i;
    else if (rd3_i == 5'd12) reg_r12_q <= rd3_value_i;
    if      (rd0_i == 5'd13) reg_r13_q <= rd0_value_i;
    else if (rd1_i == 5'd13) reg_r13_q <= rd1_value_i;
    else if (rd2_i == 5'd13) reg_r13_q <= rd2_value_i;
    else if (rd3_i == 5'd13) reg_r13_q <= rd3_value_i;
    if      (rd0_i == 5'd14) reg_r14_q <= rd0_value_i;
    else if (rd1_i == 5'd14) reg_r14_q <= rd1_value_i;
    else if (rd2_i == 5'd14) reg_r14_q <= rd2_value_i;
    else if (rd3_i == 5'd14) reg_r14_q <= rd3_value_i;
    if      (rd0_i == 5'd15) reg_r15_q <= rd0_value_i;
    else if (rd1_i == 5'd15) reg_r15_q <= rd1_value_i;
    else if (rd2_i == 5'd15) reg_r15_q <= rd2_value_i;
    else if (rd3_i == 5'd15) reg_r15_q <= rd3_value_i;
    if      (rd0_i == 5'd16) reg_r16_q <= rd0_value_i;
    else if (rd1_i == 5'd16) reg_r16_q <= rd1_value_i;
    else if (rd2_i == 5'd16) reg_r16_q <= rd2_value_i;
    else if (rd3_i == 5'd16) reg_r16_q <= rd3_value_i;
    if      (rd0_i == 5'd17) reg_r17_q <= rd0_value_i;
    else if (rd1_i == 5'd17) reg_r17_q <= rd1_value_i;
    else if (rd2_i == 5'd17) reg_r17_q <= rd2_value_i;
    else if (rd3_i == 5'd17) reg_r17_q <= rd3_value_i;
    if      (rd0_i == 5'd18) reg_r18_q <= rd0_value_i;
    else if (rd1_i == 5'd18) reg_r18_q <= rd1_value_i;
    else if (rd2_i == 5'd18) reg_r18_q <= rd2_value_i;
    else if (rd3_i == 5'd18) reg_r18_q <= rd3_value_i;
    if      (rd0_i == 5'd19) reg_r19_q <= rd0_value_i;
    else if (rd1_i == 5'd19) reg_r19_q <= rd1_value_i;
    else if (rd2_i == 5'd19) reg_r19_q <= rd2_value_i;
    else if (rd3_i == 5'd19) reg_r19_q <= rd3_value_i;
    if      (rd0_i == 5'd20) reg_r20_q <= rd0_value_i;
    else if (rd1_i == 5'd20) reg_r20_q <= rd1_value_i;
    else if (rd2_i == 5'd20) reg_r20_q <= rd2_value_i;
    else if (rd3_i == 5'd20) reg_r20_q <= rd3_value_i;
    if      (rd0_i == 5'd21) reg_r21_q <= rd0_value_i;
    else if (rd1_i == 5'd21) reg_r21_q <= rd1_value_i;
    else if (rd2_i == 5'd21) reg_r21_q <= rd2_value_i;
    else if (rd3_i == 5'd21) reg_r21_q <= rd3_value_i;
    if      (rd0_i == 5'd22) reg_r22_q <= rd0_value_i;
    else if (rd1_i == 5'd22) reg_r22_q <= rd1_value_i;
    else if (rd2_i == 5'd22) reg_r22_q <= rd2_value_i;
    else if (rd3_i == 5'd22) reg_r22_q <= rd3_value_i;
    if      (rd0_i == 5'd23) reg_r23_q <= rd0_value_i;
    else if (rd1_i == 5'd23) reg_r23_q <= rd1_value_i;
    else if (rd2_i == 5'd23) reg_r23_q <= rd2_value_i;
    else if (rd3_i == 5'd23) reg_r23_q <= rd3_value_i;
    if      (rd0_i == 5'd24) reg_r24_q <= rd0_value_i;
    else if (rd1_i == 5'd24) reg_r24_q <= rd1_value_i;
    else if (rd2_i == 5'd24) reg_r24_q <= rd2_value_i;
    else if (rd3_i == 5'd24) reg_r24_q <= rd3_value_i;
    if      (rd0_i == 5'd25) reg_r25_q <= rd0_value_i;
    else if (rd1_i == 5'd25) reg_r25_q <= rd1_value_i;
    else if (rd2_i == 5'd25) reg_r25_q <= rd2_value_i;
    else if (rd3_i == 5'd25) reg_r25_q <= rd3_value_i;
    if      (rd0_i == 5'd26) reg_r26_q <= rd0_value_i;
    else if (rd1_i == 5'd26) reg_r26_q <= rd1_value_i;
    else if (rd2_i == 5'd26) reg_r26_q <= rd2_value_i;
    else if (rd3_i == 5'd26) reg_r26_q <= rd3_value_i;
    if      (rd0_i == 5'd27) reg_r27_q <= rd0_value_i;
    else if (rd1_i == 5'd27) reg_r27_q <= rd1_value_i;
    else if (rd2_i == 5'd27) reg_r27_q <= rd2_value_i;
    else if (rd3_i == 5'd27) reg_r27_q <= rd3_value_i;
    if      (rd0_i == 5'd28) reg_r28_q <= rd0_value_i;
    else if (rd1_i == 5'd28) reg_r28_q <= rd1_value_i;
    else if (rd2_i == 5'd28) reg_r28_q <= rd2_value_i;
    else if (rd3_i == 5'd28) reg_r28_q <= rd3_value_i;
    if      (rd0_i == 5'd29) reg_r29_q <= rd0_value_i;
    else if (rd1_i == 5'd29) reg_r29_q <= rd1_value_i;
    else if (rd2_i == 5'd29) reg_r29_q <= rd2_value_i;
    else if (rd3_i == 5'd29) reg_r29_q <= rd3_value_i;
    if      (rd0_i == 5'd30) reg_r30_q <= rd0_value_i;
    else if (rd1_i == 5'd30) reg_r30_q <= rd1_value_i;
    else if (rd2_i == 5'd30) reg_r30_q <= rd2_value_i;
    else if (rd3_i == 5'd30) reg_r30_q <= rd3_value_i;
    if      (rd0_i == 5'd31) reg_r31_q <= rd0_value_i;
    else if (rd1_i == 5'd31) reg_r31_q <= rd1_value_i;
    else if (rd2_i == 5'd31) reg_r31_q <= rd2_value_i;
    else if (rd3_i == 5'd31) reg_r31_q <= rd3_value_i;
end

//-----------------------------------------------------------------
// Asynchronous read
//-----------------------------------------------------------------
reg [31:0] ra_value_r;
reg [31:0] rb_value_r;
always @ *
begin
    case (ra_i)
    5'd1: ra_value_r = reg_r1_q;
    5'd2: ra_value_r = reg_r2_q;
    5'd3: ra_value_r = reg_r3_q;
    5'd4: ra_value_r = reg_r4_q;
    5'd5: ra_value_r = reg_r5_q;
    5'd6: ra_value_r = reg_r6_q;
    5'd7: ra_value_r = reg_r7_q;
    5'd8: ra_value_r = reg_r8_q;
    5'd9: ra_value_r = reg_r9_q;
    5'd10: ra_value_r = reg_r10_q;
    5'd11: ra_value_r = reg_r11_q;
    5'd12: ra_value_r = reg_r12_q;
    5'd13: ra_value_r = reg_r13_q;
    5'd14: ra_value_r = reg_r14_q;
    5'd15: ra_value_r = reg_r15_q;
    5'd16: ra_value_r = reg_r16_q;
    5'd17: ra_value_r = reg_r17_q;
    5'd18: ra_value_r = reg_r18_q;
    5'd19: ra_value_r = reg_r19_q;
    5'd20: ra_value_r = reg_r20_q;
    5'd21: ra_value_r = reg_r21_q;
    5'd22: ra_value_r = reg_r22_q;
    5'd23: ra_value_r = reg_r23_q;
    5'd24: ra_value_r = reg_r24_q;
    5'd25: ra_value_r = reg_r25_q;
    5'd26: ra_value_r = reg_r26_q;
    5'd27: ra_value_r = reg_r27_q;
    5'd28: ra_value_r = reg_r28_q;
    5'd29: ra_value_r = reg_r29_q;
    5'd30: ra_value_r = reg_r30_q;
    5'd31: ra_value_r = reg_r31_q;
    default : ra_value_r = 32'h00000000;
    endcase

    case (rb_i)
    5'd1: rb_value_r = reg_r1_q;
    5'd2: rb_value_r = reg_r2_q;
    5'd3: rb_value_r = reg_r3_q;
    5'd4: rb_value_r = reg_r4_q;
    5'd5: rb_value_r = reg_r5_q;
    5'd6: rb_value_r = reg_r6_q;
    5'd7: rb_value_r = reg_r7_q;
    5'd8: rb_value_r = reg_r8_q;
    5'd9: rb_value_r = reg_r9_q;
    5'd10: rb_value_r = reg_r10_q;
    5'd11: rb_value_r = reg_r11_q;
    5'd12: rb_value_r = reg_r12_q;
    5'd13: rb_value_r = reg_r13_q;
    5'd14: rb_value_r = reg_r14_q;
    5'd15: rb_value_r = reg_r15_q;
    5'd16: rb_value_r = reg_r16_q;
    5'd17: rb_value_r = reg_r17_q;
    5'd18: rb_value_r = reg_r18_q;
    5'd19: rb_value_r = reg_r19_q;
    5'd20: rb_value_r = reg_r20_q;
    5'd21: rb_value_r = reg_r21_q;
    5'd22: rb_value_r = reg_r22_q;
    5'd23: rb_value_r = reg_r23_q;
    5'd24: rb_value_r = reg_r24_q;
    5'd25: rb_value_r = reg_r25_q;
    5'd26: rb_value_r = reg_r26_q;
    5'd27: rb_value_r = reg_r27_q;
    5'd28: rb_value_r = reg_r28_q;
    5'd29: rb_value_r = reg_r29_q;
    5'd30: rb_value_r = reg_r30_q;
    5'd31: rb_value_r = reg_r31_q;
    default : rb_value_r = 32'h00000000;
    endcase
end

assign ra_value_o = ra_value_r;
assign rb_value_o = rb_value_r;



//-------------------------------------------------------------
// get_register: Read register file
//-------------------------------------------------------------
`ifdef verilator
function [31:0] get_register; /*verilator public*/
    input [4:0] r;
begin
    case (r)
    5'd1: get_register = reg_r1_q;
    5'd2: get_register = reg_r2_q;
    5'd3: get_register = reg_r3_q;
    5'd4: get_register = reg_r4_q;
    5'd5: get_register = reg_r5_q;
    5'd6: get_register = reg_r6_q;
    5'd7: get_register = reg_r7_q;
    5'd8: get_register = reg_r8_q;
    5'd9: get_register = reg_r9_q;
    5'd10: get_register = reg_r10_q;
    5'd11: get_register = reg_r11_q;
    5'd12: get_register = reg_r12_q;
    5'd13: get_register = reg_r13_q;
    5'd14: get_register = reg_r14_q;
    5'd15: get_register = reg_r15_q;
    5'd16: get_register = reg_r16_q;
    5'd17: get_register = reg_r17_q;
    5'd18: get_register = reg_r18_q;
    5'd19: get_register = reg_r19_q;
    5'd20: get_register = reg_r20_q;
    5'd21: get_register = reg_r21_q;
    5'd22: get_register = reg_r22_q;
    5'd23: get_register = reg_r23_q;
    5'd24: get_register = reg_r24_q;
    5'd25: get_register = reg_r25_q;
    5'd26: get_register = reg_r26_q;
    5'd27: get_register = reg_r27_q;
    5'd28: get_register = reg_r28_q;
    5'd29: get_register = reg_r29_q;
    5'd30: get_register = reg_r30_q;
    5'd31: get_register = reg_r31_q;
    default : get_register = 32'h00000000;
    endcase
end
endfunction
//-------------------------------------------------------------
// set_register: Write register file
//-------------------------------------------------------------
function set_register; /*verilator public*/
    input [4:0] r;
    input [31:0] value;
begin
    //case (r)
    //5'd1:  reg_r1_q  <= value;
    //5'd2:  reg_r2_q  <= value;
    //5'd3:  reg_r3_q  <= value;
    //5'd4:  reg_r4_q  <= value;
    //5'd5:  reg_r5_q  <= value;
    //5'd6:  reg_r6_q  <= value;
    //5'd7:  reg_r7_q  <= value;
    //5'd8:  reg_r8_q  <= value;
    //5'd9:  reg_r9_q  <= value;
    //5'd10: reg_r10_q <= value;
    //5'd11: reg_r11_q <= value;
    //5'd12: reg_r12_q <= value;
    //5'd13: reg_r13_q <= value;
    //5'd14: reg_r14_q <= value;
    //5'd15: reg_r15_q <= value;
    //5'd16: reg_r16_q <= value;
    //5'd17: reg_r17_q <= value;
    //5'd18: reg_r18_q <= value;
    //5'd19: reg_r19_q <= value;
    //5'd20: reg_r20_q <= value;
    //5'd21: reg_r21_q <= value;
    //5'd22: reg_r22_q <= value;
    //5'd23: reg_r23_q <= value;
    //5'd24: reg_r24_q <= value;
    //5'd25: reg_r25_q <= value;
    //5'd26: reg_r26_q <= value;
    //5'd27: reg_r27_q <= value;
    //5'd28: reg_r28_q <= value;
    //5'd29: reg_r29_q <= value;
    //5'd30: reg_r30_q <= value;
    //5'd31: reg_r31_q <= value;
    //default :
    //    ;
    //endcase
end
endfunction
`endif


endmodule
