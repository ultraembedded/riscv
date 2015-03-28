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
// Module - Wishbone D-port (no data cache)
//-----------------------------------------------------------------
module riscv_dmem_nocache
(
    // General
    input               clk_i,
    input               rst_i,

    // CPU Interface
    input [31:0]        cpu_addr_i,
    input [31:0]        cpu_data_out_i,
    output [31:0]       cpu_data_in_o,
    input [3:0]         cpu_sel_i,
    input               cpu_we_i,
    input               cpu_stb_i,
    output              cpu_stall_o,
    output              cpu_ack_o,

    // Memory Interface
    output [31:0]       dmem_addr_o,
    output [31:0]       dmem_data_out_o,
    input [31:0]        dmem_data_in_i,
    output [3:0]        dmem_sel_o,
    output              dmem_we_o,
    output              dmem_stb_o,
    output              dmem_cyc_o,
    output [2:0]        dmem_cti_o,
    input               dmem_stall_i,
    input               dmem_ack_i
);

//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-----------------------------------------------------------------
// CYC_O
//-----------------------------------------------------------------
reg cyc_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i == 1'b1)
    cyc_q <= 1'b0;
else if (dmem_stb_o & !dmem_stall_i)
    cyc_q <= 1'b1;
else if (dmem_ack_i)
    cyc_q <= 1'b0;

//-----------------------------------------------------------------
// Hookup
//-----------------------------------------------------------------
assign dmem_addr_o      = {cpu_addr_i[31:2], 2'b0};
assign dmem_data_out_o  = cpu_data_out_i;
assign dmem_stb_o       = cpu_stb_i & ~cyc_q;
assign dmem_cyc_o       = dmem_stb_o | cyc_q;
assign dmem_sel_o       = cpu_sel_i;
assign dmem_cti_o       = 3'b111;
assign dmem_we_o        = cpu_we_i;

assign cpu_data_in_o    = dmem_data_in_i;
assign cpu_ack_o        = dmem_ack_i;
assign cpu_stall_o      = cyc_q | dmem_stall_i;

endmodule
