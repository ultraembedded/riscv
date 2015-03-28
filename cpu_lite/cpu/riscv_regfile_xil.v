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
//   Simple, small, multi-cycle 32-bit RISC-V CPU implementation.
//   Runs SW built with '-m32 -mno-rvm'.
//   Most instructions take 3 cycles, apart from load and stores
//   which take 4+ cycles (depending on memory latency).
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
// Module - Xilinx register file (async read)
//-----------------------------------------------------------------
module riscv_regfile_xil
(
    input               clk_i,
    input               rst_i,
    input               wr_i ,
    input [4:0]         rs1_i,
    input [4:0]         rs2_i,
    input [4:0]         rd_i ,
    output reg [31:0]   reg_rs1_o,
    output reg [31:0]   reg_rs2_o,
    input [31:0]        reg_rd_i
);

//-----------------------------------------------------------------
// Registers / Wires
//-----------------------------------------------------------------
wire [31:0]     reg_rs1_w;
wire [31:0]     reg_rs2_w;
wire [31:0]     rs1_0_15_w;
wire [31:0]     rs1_16_31_w;
wire [31:0]     rs2_0_15_w;
wire [31:0]     rs2_16_31_w;
wire            write_enable_w;
wire            write_banka_w;
wire            write_bankb_w;

//-----------------------------------------------------------------
// Register File (using RAM16X1D )
//-----------------------------------------------------------------

// Registers 0 - 15
generate
begin
   genvar i;
   for (i=0;i<32;i=i+1)
   begin : reg_loop1
       RAM16X1D reg_bit1a(.WCLK(clk_i), .WE(write_banka_w), .A0(rd_i[0]), .A1(rd_i[1]), .A2(rd_i[2]), .A3(rd_i[3]), .D(reg_rd_i[i]), .DPRA0(rs1_i[0]), .DPRA1(rs1_i[1]), .DPRA2(rs1_i[2]), .DPRA3(rs1_i[3]), .DPO(rs1_0_15_w[i]), .SPO(/* open */));
       RAM16X1D reg_bit2a(.WCLK(clk_i), .WE(write_banka_w), .A0(rd_i[0]), .A1(rd_i[1]), .A2(rd_i[2]), .A3(rd_i[3]), .D(reg_rd_i[i]), .DPRA0(rs2_i[0]), .DPRA1(rs2_i[1]), .DPRA2(rs2_i[2]), .DPRA3(rs2_i[3]), .DPO(rs2_0_15_w[i]), .SPO(/* open */));
   end
end
endgenerate

// Registers 16 - 31
generate
begin
   genvar i;
   for (i=0;i<32;i=i+1)
   begin : reg_loop2
       RAM16X1D reg_bit1b(.WCLK(clk_i), .WE(write_bankb_w), .A0(rd_i[0]), .A1(rd_i[1]), .A2(rd_i[2]), .A3(rd_i[3]), .D(reg_rd_i[i]), .DPRA0(rs1_i[0]), .DPRA1(rs1_i[1]), .DPRA2(rs1_i[2]), .DPRA3(rs1_i[3]), .DPO(rs1_16_31_w[i]), .SPO(/* open */));
       RAM16X1D reg_bit2b(.WCLK(clk_i), .WE(write_bankb_w), .A0(rd_i[0]), .A1(rd_i[1]), .A2(rd_i[2]), .A3(rd_i[3]), .D(reg_rd_i[i]), .DPRA0(rs2_i[0]), .DPRA1(rs2_i[1]), .DPRA2(rs2_i[2]), .DPRA3(rs2_i[3]), .DPO(rs2_16_31_w[i]), .SPO(/* open */));
   end
end
endgenerate

//-----------------------------------------------------------------
// Combinatorial Assignments
//-----------------------------------------------------------------
assign reg_rs1_w       = (rs1_i[4] == 1'b0) ? rs1_0_15_w : rs1_16_31_w;
assign reg_rs2_w       = (rs2_i[4] == 1'b0) ? rs2_0_15_w : rs2_16_31_w;

assign write_enable_w = (rd_i != 5'b00000) & wr_i;

assign write_banka_w  = (write_enable_w & (~rd_i[4]));
assign write_bankb_w  = (write_enable_w & rd_i[4]);

// Register read ports
always @ *
begin
    if (rs1_i == 5'b00000)
        reg_rs1_o = 32'h00000000;
    else
        reg_rs1_o = reg_rs1_w;

    if (rs2_i == 5'b00000)
        reg_rs2_o = 32'h00000000;
    else
        reg_rs2_o = reg_rs2_w;
end

endmodule
