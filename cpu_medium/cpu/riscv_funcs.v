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
// less_than_signed: Less than operator (signed)
// Inputs: x = left operand, y = right operand
// Return: (int)x < (int)y
//-----------------------------------------------------------------
function [0:0] less_than_signed;
    input  [31:0] x;
    input  [31:0] y;
    reg [31:0] v;
begin
    v = (x - y);
    if (x[31] != y[31])
        less_than_signed = x[31];
    else
        less_than_signed = v[31];
end
endfunction

//-----------------------------------------------------------------
// greater_than_signed: Greater than operator (signed)
// Inputs: x = left operand, y = right operand
// Return: (int)x > (int)y
//-----------------------------------------------------------------
function [0:0] greater_than_signed;
    input  [31:0] x;
    input  [31:0] y;
    reg [31:0] v;
begin
    v = (y - x);
    if (x[31] != y[31])
        greater_than_signed = y[31];
    else
        greater_than_signed = v[31];
end
endfunction

//-----------------------------------------------------------------
// get_regname_str: Convert register number to string
//-----------------------------------------------------------------
`ifdef SIMULATION
function [79:0] get_regname_str;
    input  [4:0] regnum;
begin
    case (regnum)
        5'd0:  get_regname_str = "zero";
        5'd1:  get_regname_str = "ra";
        5'd2:  get_regname_str = "s0";
        5'd3:  get_regname_str = "s1";
        5'd4:  get_regname_str = "s2";
        5'd5:  get_regname_str = "s3";
        5'd6:  get_regname_str = "s4";
        5'd7:  get_regname_str = "s5";
        5'd8:  get_regname_str = "s6";
        5'd9:  get_regname_str = "s7";
        5'd10: get_regname_str = "s8";
        5'd11: get_regname_str = "s9";
        5'd12: get_regname_str = "s10";
        5'd13: get_regname_str = "s11";
        5'd14: get_regname_str = "sp";
        5'd15: get_regname_str = "tp";
        5'd16: get_regname_str = "v0";
        5'd17: get_regname_str = "v1";
        5'd18: get_regname_str = "a0";
        5'd19: get_regname_str = "a1";
        5'd20: get_regname_str = "a2";
        5'd21: get_regname_str = "a3";
        5'd22: get_regname_str = "a4";
        5'd23: get_regname_str = "a5";
        5'd24: get_regname_str = "a6";
        5'd25: get_regname_str = "a7";
        5'd26: get_regname_str = "t0";
        5'd27: get_regname_str = "t1";
        5'd28: get_regname_str = "t2";
        5'd29: get_regname_str = "t3";
        5'd30: get_regname_str = "t4";
        5'd31: get_regname_str = "gp";
    endcase
end
endfunction
`endif
