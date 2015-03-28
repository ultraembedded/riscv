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
// Module - Instruction Fetch
//-----------------------------------------------------------------
module riscv_fetch
(
    // General
    input               clk_i,
    input               rst_i,

    // Instruction Fetch
    output              fetch_o,
    output reg [31:0]   pc_o,
    input [31:0]        data_i,
    input               data_valid_i,

    // Branch target
    input               branch_i,
    input [31:0]        branch_pc_i,
    input               stall_i,

    // Decoded opcode
    output [31:0]       opcode_o,
    output [31:0]       opcode_pc_o,
    output              opcode_valid_o,

    // Decoded register details
    output [4:0]        rs1_o,
    output [4:0]        rs2_o,
    output [4:0]        rd_o
);

//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
parameter   BOOT_VECTOR             = 32'h00000000;

//-----------------------------------------------------------------
// Registers
//-----------------------------------------------------------------
reg [31:0]  pc_next_q;
reg [31:0]  pc_q;
reg         branch_q;

wire        outstanding_req_w;

//-------------------------------------------------------------------
// Next PC
//-------------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
   if (rst_i)
        pc_next_q        <= BOOT_VECTOR + `VECTOR_RESET;
   else if (fetch_o)
        pc_next_q        <= pc_o + 32'd4;

//-------------------------------------------------------------------
// Last PC
//-------------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
   if (rst_i)
        pc_q  <= BOOT_VECTOR + `VECTOR_RESET;
   else if (!outstanding_req_w)
        pc_q  <= pc_o;

//-------------------------------------------------------------------
// Instruction Fetch
//-------------------------------------------------------------------
always @ *
begin
    // Request oustanding, hold PC
    if (outstanding_req_w)
        pc_o    = pc_q;
    // Branch
    else if (branch_i || branch_q)
        pc_o    = branch_pc_i;
    // Stalled, hold current PC
    else if (stall_i)
        pc_o    = pc_q;
    // Default - next PC
    else
        pc_o    = pc_next_q;
end

reg fetch_q;
always @ (posedge clk_i or posedge rst_i)
   if (rst_i)
        fetch_q <= 1'b0;
   else if (fetch_o)
        fetch_q <= 1'b1;
   else if (data_valid_i && !fetch_o)
        fetch_q <= 1'b0;        

assign outstanding_req_w = fetch_q & ~data_valid_i;

always @ (posedge clk_i or posedge rst_i)
   if (rst_i)
        branch_q <= 1'b0;
   // Branch request delayed
   else if (branch_i && (outstanding_req_w || stall_i))
        branch_q <= 1'b1;
   // Branch request serviced
   else if (data_valid_i)
        branch_q <= 1'b0;

assign fetch_o  = ~outstanding_req_w;

//-------------------------------------------------------------------
// Opcode output
//-------------------------------------------------------------------
assign opcode_pc_o  = pc_q;
assign opcode_o     = data_i;

assign opcode_valid_o  = (data_valid_i & !branch_i && !branch_q);

//-------------------------------------------------------------------
// Opcode output
//-------------------------------------------------------------------
assign rs1_o           = opcode_o[19:15];
assign rs2_o           = opcode_o[24:20];
assign rd_o            = opcode_o[11:7];

endmodule
