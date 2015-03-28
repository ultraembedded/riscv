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
// Module: riscv_ram_sp - Single port RAM (used in cache)
//-----------------------------------------------------------------
module riscv_ram_sp
#(
    parameter  [31:0]       WIDTH = 8,
    parameter  [31:0]       SIZE = 14
)
( 
    input                   clk_i,
    output [(WIDTH - 1):0]  dat_o,
    input [(WIDTH - 1):0]   dat_i,
    input [(SIZE - 1):0]    adr_i,
    input                   wr_i
);

//-----------------------------------------------------------------
// Registers
//-----------------------------------------------------------------
reg [(WIDTH - 1):0]     ram [((2<< (SIZE-1)) - 1):0];
reg [(SIZE - 1):0]      rd_addr_q;

//-----------------------------------------------------------------
// Processes
//-----------------------------------------------------------------
always @ (posedge clk_i)
begin 
    if (wr_i == 1'b1)
        ram[adr_i] <= dat_i;
    rd_addr_q <= adr_i;
end

//-------------------------------------------------------------------
// Combinatorial
//-------------------------------------------------------------------
assign dat_o = ram[rd_addr_q];

//-----------------------------------------------------------------
// Init Memory
//-----------------------------------------------------------------
`ifdef RISCV_CLEAR_RAM
    integer i;
    initial 
    begin     
        for (i=0;i<((2<< (SIZE-1)) - 1);i=i+1)
        begin
            ram[i] = 0;
        end
    end
`endif

endmodule
