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
// Module: riscv_ram_dp - Dual port RAM (used in cache)
//-----------------------------------------------------------------
module riscv_ram_dp
#(
    parameter               WIDTH = 8,
    parameter               SIZE = 14
)
(
    input                   aclk_i,
    output [(WIDTH - 1):0]  adat_o,
    input [(WIDTH - 1):0]   adat_i,
    input [(SIZE - 1):0]    aadr_i,
    input                   awr_i,

    input                   bclk_i,
    output [(WIDTH - 1):0]  bdat_o,
    input [(WIDTH - 1):0]   bdat_i,
    input [(SIZE - 1):0]    badr_i,
    input                   bwr_i
);

//-----------------------------------------------------------------
// Registers
//-----------------------------------------------------------------
/* verilator lint_off MULTIDRIVEN */
reg [(WIDTH - 1):0]     ram [((2<< (SIZE-1)) - 1):0];
/* verilator lint_on MULTIDRIVEN */

reg [(SIZE - 1):0]      rd_addr_a_q;
reg [(SIZE - 1):0]      rd_addr_b_q;

//-----------------------------------------------------------------
// Processes
//-----------------------------------------------------------------
always @ (posedge aclk_i)
begin
    if (awr_i == 1'b1)
        ram[aadr_i] <= adat_i;
    rd_addr_a_q <= aadr_i;
end
always @ (posedge bclk_i)
begin
    if (bwr_i == 1'b1)
        ram[badr_i] <= bdat_i;
    rd_addr_b_q <= badr_i;
end

// Read / Write collision detection
`ifdef SIM_DPRAM_COLLISIONS
    reg collision_a_q;
    reg collision_b_q;

    always @ (posedge aclk_i)
    begin
        if (aadr_i == badr_i && bwr_i)
            collision_a_q <= 1;
        else
            collision_a_q <= 0;

        if (badr_i == aadr_i && awr_i)
            collision_b_q <= 1;
        else
            collision_b_q <= 0;
    end
`endif

//-------------------------------------------------------------------
// Combinatorial
//-------------------------------------------------------------------
`ifdef SIM_DPRAM_COLLISIONS
    assign adat_o = collision_a_q ? {WIDTH{1'bx}}: ram[rd_addr_a_q];
    assign bdat_o = collision_b_q ? {WIDTH{1'bx}}: ram[rd_addr_b_q];
`else
    assign adat_o = ram[rd_addr_a_q];
    assign bdat_o = ram[rd_addr_b_q];
`endif

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
