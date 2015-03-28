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
// Module - Writeback
//-----------------------------------------------------------------
module riscv_writeback
(
    // General
    input               clk_i,
    input               rst_i,

    // Stall from mem stage
    input               stall_mem_i,

    // Opcode
    input [31:0]        opcode_i,

    // Register target
    input [4:0]         rd_i,

    // ALU result
    input [31:0]        alu_result_i,

    // Memory load result
    input [31:0]        load_result_i,
    input [4:0]         load_dest_i,
    output              load_accept_o,

    // Outputs
    output reg          write_enable_o,
    output reg [4:0]    write_addr_o,
    output reg [31:0]   write_data_o
);

//-----------------------------------------------------------------
// Registers / Wires
//-----------------------------------------------------------------

// Register address
reg [4:0]   rd_q;

// Register writeback value
reg [31:0]  result_q;

// Register writeback enable
reg         write_rd_q;

//-------------------------------------------------------------------
// Pipeline Registers
//-------------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
begin
   if (rst_i == 1'b1)
   begin
       write_rd_q   <= 1'b1;
       result_q     <= 32'h00000000;
       rd_q         <= 5'b0;
   end
   else if (!stall_mem_i)
   begin
        // Register writeback required?
        if (rd_i != 5'b0)
        begin
            rd_q        <= rd_i;
            result_q    <= alu_result_i;
            write_rd_q  <= 1'b1;
        end
        else
        begin
            rd_q        <= 5'b0;
            result_q    <= 32'b0;
            write_rd_q  <= 1'b0;
        end
   end
end

//-------------------------------------------------------------------
// Writeback
//-------------------------------------------------------------------
always @ *
begin
    // Normal ALU instruction
    if (write_rd_q)
    begin
        write_addr_o   = rd_q;
        write_enable_o = 1'b1;
        write_data_o   = result_q;
    end
    // Load result
    else
    begin
        write_addr_o   = load_dest_i;
        write_enable_o = (load_dest_i != 5'b0);
        write_data_o   = load_result_i;
    end
end

assign load_accept_o   = ~write_rd_q;

endmodule
