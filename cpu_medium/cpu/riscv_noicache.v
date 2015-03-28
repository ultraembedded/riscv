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
// Module - Wishbone I-port (no instruction cache)
//-----------------------------------------------------------------
module riscv_noicache
(
    // General
    input               clk_i,
    input               rst_i,

    // CPU Interface
    input [31:0]        cpu_addr_i,
    output [31:0]       cpu_data_o,
    input               cpu_stb_i,
    output              cpu_ack_o,

    // Memory Interface
    output [31:0]       imem_addr_o,
    input [31:0]        imem_data_i,
    output              imem_stb_o,
    output              imem_cyc_o,
    output [2:0]        imem_cti_o,
    input               imem_stall_i,
    input               imem_ack_i
);

//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-----------------------------------------------------------------
// Wires / Registers
//-----------------------------------------------------------------
reg        stb_q;

//-----------------------------------------------------------------
// CYC_O
//-----------------------------------------------------------------
reg cyc_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i == 1'b1)
    cyc_q <= 1'b0;
else if (imem_stb_o)
    cyc_q <= 1'b1;
else if (imem_ack_i)
    cyc_q <= 1'b0;

always @ (posedge clk_i or posedge rst_i)
if (rst_i == 1'b1)
    stb_q <= 1'b0;
else if (imem_stall_i && cpu_stb_i)
    stb_q <= 1'b1;
else if (!imem_stall_i)
    stb_q <= 1'b0;

//-----------------------------------------------------------------
// Hookup
//-----------------------------------------------------------------
assign imem_addr_o  = cpu_addr_i;
assign cpu_data_o   = imem_data_i;
assign imem_stb_o   = stb_q | cpu_stb_i;
assign imem_cyc_o   = cpu_stb_i | cyc_q;
assign imem_cti_o   = 3'b111;
assign cpu_ack_o    = imem_ack_i;

endmodule
