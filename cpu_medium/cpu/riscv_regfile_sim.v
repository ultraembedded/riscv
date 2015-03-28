//-----------------------------------------------------------------
//                       RISC-V IP Core
//                            V0.1
//                     Ultra-Embedded.com
//                       Copyright 2015
//
//               Email: admin@ultra-embedded.com
//
//                       License: LGPL
//-----------------------------------------------------------------
// Description:
//   Pipelined 32-bit RISC-V CPU implementation with separate
//   Wishbone interfaces for instruction & data access.
//   Contains configurable instruction cache.
//   Runs SW built with '-m32 -mno-rvm'.
//-----------------------------------------------------------------
//
// Copyright (C) 2015 Ultra-Embedded.com
//
// This source file may be used and distributed without         
// restriction provided that this copyright statement is not    
// removed from the file and that any derivative work contains  
// the original copyright notice and the associated disclaimer. 
//
// This source file is free software; you can redistribute it   
// and/or modify it under the terms of the GNU Lesser General   
// Public License as published by the Free Software Foundation; 
// either version 2.1 of the License, or (at your option) any   
// later version.
//
// This source is distributed in the hope that it will be       
// useful, but WITHOUT ANY WARRANTY; without even the implied   
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      
// PURPOSE.  See the GNU Lesser General Public License for more 
// details.
//
// You should have received a copy of the GNU Lesser General    
// Public License along with this source; if not, write to the 
// Free Software Foundation, Inc., 59 Temple Place, Suite 330, 
// Boston, MA  02111-1307  USA
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-----------------------------------------------------------------
// Module - Simulation register file
//-----------------------------------------------------------------
module riscv_regfile_sim
(
    input             clk_i,
    input             rst_i,
    input             wr_i ,
    input [4:0]       rs1_i,
    input [4:0]       rs2_i,
    input [4:0]       rd_i ,
    output reg [31:0] reg_rs1_o,
    output reg [31:0] reg_rs2_o,
    input [31:0]      reg_rd_i
);

//-----------------------------------------------------------------
// Registers
//-----------------------------------------------------------------

// Register file
reg [31:0] reg_r1_ra;
reg [31:0] reg_r2_s0;
reg [31:0] reg_r3_s1;
reg [31:0] reg_r4_s2;
reg [31:0] reg_r5_s3;
reg [31:0] reg_r6_s4;
reg [31:0] reg_r7_s5;
reg [31:0] reg_r8_s6;
reg [31:0] reg_r9_s7;
reg [31:0] reg_r10_s8;
reg [31:0] reg_r11_s9;
reg [31:0] reg_r12_s10;
reg [31:0] reg_r13_s11;
reg [31:0] reg_r14_sp;
reg [31:0] reg_r15_tp;
reg [31:0] reg_r16_v0;
reg [31:0] reg_r17_v1;
reg [31:0] reg_r18_a0;
reg [31:0] reg_r19_a1;
reg [31:0] reg_r20_a2;
reg [31:0] reg_r21_a3;
reg [31:0] reg_r22_a4;
reg [31:0] reg_r23_a5;
reg [31:0] reg_r24_a6;
reg [31:0] reg_r25_a7;
reg [31:0] reg_r26_t0;
reg [31:0] reg_r27_t1;
reg [31:0] reg_r28_t2;
reg [31:0] reg_r29_t3;
reg [31:0] reg_r30_t4;
reg [31:0] reg_r31_gp;

//-----------------------------------------------------------------
// Register File (for simulation)
//-----------------------------------------------------------------

// Synchronous register write back
always @ (posedge clk_i or posedge rst_i)
begin
   if (rst_i)
   begin
        reg_r1_ra      <= 32'h00000000;
        reg_r2_s0      <= 32'h00000000;
        reg_r3_s1      <= 32'h00000000;
        reg_r4_s2      <= 32'h00000000;
        reg_r5_s3      <= 32'h00000000;
        reg_r6_s4      <= 32'h00000000;
        reg_r7_s5      <= 32'h00000000;
        reg_r8_s6      <= 32'h00000000;
        reg_r9_s7      <= 32'h00000000;
        reg_r10_s8     <= 32'h00000000;
        reg_r11_s9     <= 32'h00000000;
        reg_r12_s10    <= 32'h00000000;
        reg_r13_s11    <= 32'h00000000;
        reg_r14_sp     <= 32'h00000000;
        reg_r15_tp     <= 32'h00000000;
        reg_r16_v0     <= 32'h00000000;
        reg_r17_v1     <= 32'h00000000;
        reg_r18_a0     <= 32'h00000000;
        reg_r19_a1     <= 32'h00000000;
        reg_r20_a2     <= 32'h00000000;
        reg_r21_a3     <= 32'h00000000;
        reg_r22_a4     <= 32'h00000000;
        reg_r23_a5     <= 32'h00000000;
        reg_r24_a6     <= 32'h00000000;
        reg_r25_a7     <= 32'h00000000;
        reg_r26_t0     <= 32'h00000000;
        reg_r27_t1     <= 32'h00000000;
        reg_r28_t2     <= 32'h00000000;
        reg_r29_t3     <= 32'h00000000;
        reg_r30_t4     <= 32'h00000000;
        reg_r31_gp     <= 32'h00000000;
   end
   else
   begin
       if (wr_i == 1'b1)
           case (rd_i[4:0])
               5'b00001 :
                       reg_r1_ra <= reg_rd_i;
               5'b00010 :
                       reg_r2_s0 <= reg_rd_i;
               5'b00011 :
                       reg_r3_s1 <= reg_rd_i;
               5'b00100 :
                       reg_r4_s2 <= reg_rd_i;
               5'b00101 :
                       reg_r5_s3 <= reg_rd_i;
               5'b00110 :
                       reg_r6_s4 <= reg_rd_i;
               5'b00111 :
                       reg_r7_s5 <= reg_rd_i;
               5'b01000 :
                       reg_r8_s6 <= reg_rd_i;
               5'b01001 :
                       reg_r9_s7 <= reg_rd_i;
               5'b01010 :
                       reg_r10_s8 <= reg_rd_i;
               5'b01011 :
                       reg_r11_s9 <= reg_rd_i;
               5'b01100 :
                       reg_r12_s10 <= reg_rd_i;
               5'b01101 :
                       reg_r13_s11 <= reg_rd_i;
               5'b01110 :
                       reg_r14_sp <= reg_rd_i;
               5'b01111 :
                       reg_r15_tp <= reg_rd_i;
               5'b10000 :
                       reg_r16_v0 <= reg_rd_i;
               5'b10001 :
                       reg_r17_v1 <= reg_rd_i;
               5'b10010 :
                       reg_r18_a0 <= reg_rd_i;
               5'b10011 :
                       reg_r19_a1 <= reg_rd_i;
               5'b10100 :
                       reg_r20_a2 <= reg_rd_i;
               5'b10101 :
                       reg_r21_a3 <= reg_rd_i;
               5'b10110 :
                       reg_r22_a4 <= reg_rd_i;
               5'b10111 :
                       reg_r23_a5 <= reg_rd_i;
               5'b11000 :
                       reg_r24_a6 <= reg_rd_i;
               5'b11001 :
                       reg_r25_a7 <= reg_rd_i;
               5'b11010 :
                       reg_r26_t0 <= reg_rd_i;
               5'b11011 :
                       reg_r27_t1 <= reg_rd_i;
               5'b11100 :
                       reg_r28_t2 <= reg_rd_i;
               5'b11101 :
                       reg_r29_t3 <= reg_rd_i;
               5'b11110 :
                       reg_r30_t4 <= reg_rd_i;
               5'b11111 :
                       reg_r31_gp <= reg_rd_i;
               default :
                   ;
           endcase
   end
end


// Asynchronous Register read (Rs1 & Rs2)
always @ *
begin
   case (rs1_i)
       5'b00000 :
               reg_rs1_o = 32'h00000000;
       5'b00001 :
               reg_rs1_o = reg_r1_ra;
       5'b00010 :
               reg_rs1_o = reg_r2_s0;
       5'b00011 :
               reg_rs1_o = reg_r3_s1;
       5'b00100 :
               reg_rs1_o = reg_r4_s2;
       5'b00101 :
               reg_rs1_o = reg_r5_s3;
       5'b00110 :
               reg_rs1_o = reg_r6_s4;
       5'b00111 :
               reg_rs1_o = reg_r7_s5;
       5'b01000 :
               reg_rs1_o = reg_r8_s6;
       5'b01001 :
               reg_rs1_o = reg_r9_s7;
       5'b01010 :
               reg_rs1_o = reg_r10_s8;
       5'b01011 :
               reg_rs1_o = reg_r11_s9;
       5'b01100 :
               reg_rs1_o = reg_r12_s10;
       5'b01101 :
               reg_rs1_o = reg_r13_s11;
       5'b01110 :
               reg_rs1_o = reg_r14_sp;
       5'b01111 :
               reg_rs1_o = reg_r15_tp;
       5'b10000 :
               reg_rs1_o = reg_r16_v0;
       5'b10001 :
               reg_rs1_o = reg_r17_v1;
       5'b10010 :
               reg_rs1_o = reg_r18_a0;
       5'b10011 :
               reg_rs1_o = reg_r19_a1;
       5'b10100 :
               reg_rs1_o = reg_r20_a2;
       5'b10101 :
               reg_rs1_o = reg_r21_a3;
       5'b10110 :
               reg_rs1_o = reg_r22_a4;
       5'b10111 :
               reg_rs1_o = reg_r23_a5;
       5'b11000 :
               reg_rs1_o = reg_r24_a6;
       5'b11001 :
               reg_rs1_o = reg_r25_a7;
       5'b11010 :
               reg_rs1_o = reg_r26_t0;
       5'b11011 :
               reg_rs1_o = reg_r27_t1;
       5'b11100 :
               reg_rs1_o = reg_r28_t2;
       5'b11101 :
               reg_rs1_o = reg_r29_t3;
       5'b11110 :
               reg_rs1_o = reg_r30_t4;
       5'b11111 :
               reg_rs1_o = reg_r31_gp;
       default :
               reg_rs1_o = 32'h00000000;
   endcase

   case (rs2_i)
       5'b00000 :
               reg_rs2_o = 32'h00000000;
       5'b00001 :
               reg_rs2_o = reg_r1_ra;
       5'b00010 :
               reg_rs2_o = reg_r2_s0;
       5'b00011 :
               reg_rs2_o = reg_r3_s1;
       5'b00100 :
               reg_rs2_o = reg_r4_s2;
       5'b00101 :
               reg_rs2_o = reg_r5_s3;
       5'b00110 :
               reg_rs2_o = reg_r6_s4;
       5'b00111 :
               reg_rs2_o = reg_r7_s5;
       5'b01000 :
               reg_rs2_o = reg_r8_s6;
       5'b01001 :
               reg_rs2_o = reg_r9_s7;
       5'b01010 :
               reg_rs2_o = reg_r10_s8;
       5'b01011 :
               reg_rs2_o = reg_r11_s9;
       5'b01100 :
               reg_rs2_o = reg_r12_s10;
       5'b01101 :
               reg_rs2_o = reg_r13_s11;
       5'b01110 :
               reg_rs2_o = reg_r14_sp;
       5'b01111 :
               reg_rs2_o = reg_r15_tp;
       5'b10000 :
               reg_rs2_o = reg_r16_v0;
       5'b10001 :
               reg_rs2_o = reg_r17_v1;
       5'b10010 :
               reg_rs2_o = reg_r18_a0;
       5'b10011 :
               reg_rs2_o = reg_r19_a1;
       5'b10100 :
               reg_rs2_o = reg_r20_a2;
       5'b10101 :
               reg_rs2_o = reg_r21_a3;
       5'b10110 :
               reg_rs2_o = reg_r22_a4;
       5'b10111 :
               reg_rs2_o = reg_r23_a5;
       5'b11000 :
               reg_rs2_o = reg_r24_a6;
       5'b11001 :
               reg_rs2_o = reg_r25_a7;
       5'b11010 :
               reg_rs2_o = reg_r26_t0;
       5'b11011 :
               reg_rs2_o = reg_r27_t1;
       5'b11100 :
               reg_rs2_o = reg_r28_t2;
       5'b11101 :
               reg_rs2_o = reg_r29_t3;
       5'b11110 :
               reg_rs2_o = reg_r30_t4;
       5'b11111 :
               reg_rs2_o = reg_r31_gp;
       default :
               reg_rs2_o = 32'h00000000;
   endcase
end

endmodule
