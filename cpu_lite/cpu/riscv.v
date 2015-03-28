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
// Module - RISCV Wrapper
//-----------------------------------------------------------------
module riscv
(
    // General
    input                   clk_i,
    input                   rst_i,

    // Interrupts
    input [7:0]             intr_i,

    // Status
    output                  fault_o,
    output                  break_o,

    // Wishbone Memory
    output [31:0]           mem_addr_o,
    output [31:0]           mem_dat_o,
    input [31:0]            mem_dat_i,
    output [3:0]            mem_sel_o,
    output [2:0]            mem_cti_o,
    output                  mem_cyc_o,
    output                  mem_we_o,
    output                  mem_stb_o,
    input                   mem_stall_i,
    input                   mem_ack_i
);

//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
parameter           BOOT_VECTOR         = 32'h00000000;
parameter           ISR_VECTOR          = 32'h00000000;
parameter           REGISTER_FILE_TYPE  = "SIMULATION";

//-----------------------------------------------------------------
// CPU SOC
//-----------------------------------------------------------------
riscv_soc
#(
    .BOOT_VECTOR(BOOT_VECTOR),
    .REGISTER_FILE_TYPE(REGISTER_FILE_TYPE),
    .ISR_VECTOR(ISR_VECTOR)
)
u_core
(
    .clk_i(clk_i),
    .rst_i(rst_i),

    .enable_i(1'b1),

    .intr_i(intr_i),

    // Status
    .fault_o(fault_o),
    .break_o(break_o),

    // Memory
    .mem_addr_o(mem_addr_o),
    .mem_dat_o(mem_dat_o),
    .mem_dat_i(mem_dat_i),
    .mem_sel_o(mem_sel_o),
    .mem_cti_o(mem_cti_o),
    .mem_cyc_o(mem_cyc_o),
    .mem_we_o(mem_we_o),
    .mem_stb_o(mem_stb_o),
    .mem_stall_i(mem_stall_i),
    .mem_ack_i(mem_ack_i),

    // CSR access
    .csr_addr_o(),
    .csr_data_o(),
    .csr_data_i(32'b0),
    .csr_set_o(),
    .csr_clr_o()
);

endmodule
