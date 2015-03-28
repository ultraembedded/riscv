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
// Module - RISCV SOC
//-----------------------------------------------------------------
module riscv_soc
(
    // General
    input               clk_i,
    input               rst_i,

    input               enable_i,

    input [7:0]         intr_i,
    output              fault_o,
    output              break_o,

    // Data memory
    output [31:0]       mem_addr_o,
    output [31:0]       mem_dat_o,
    input [31:0]        mem_dat_i,
    output [3:0]        mem_sel_o,
    output [2:0]        mem_cti_o,
    output              mem_cyc_o,
    output              mem_we_o,
    output              mem_stb_o,
    input               mem_stall_i,
    input               mem_ack_i,

    // CSR access
    output [11:0]       csr_addr_o,
    output [31:0]       csr_data_o,
    input [31:0]        csr_data_i,
    output              csr_set_o,
    output              csr_clr_o    
);

//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"
`include "riscv_funcs.v"

//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
parameter           BOOT_VECTOR         = 32'h00000000;
parameter           ISR_VECTOR          = 32'h00000000;
parameter           REGISTER_FILE_TYPE  = "SIMULATION";

//-----------------------------------------------------------------
// Wires / Registers
//-----------------------------------------------------------------
wire [11:0] csr_addr_w;
wire [31:0] csr_data_w;
reg  [31:0] csr_data_r;
wire        csr_set_w;
wire        csr_clr_w;

wire        core_int_w;

//-----------------------------------------------------------------
// CPU Core
//-----------------------------------------------------------------
riscv_core
#(
    .BOOT_VECTOR(BOOT_VECTOR),
    .REGISTER_FILE_TYPE(REGISTER_FILE_TYPE),
    .ISR_VECTOR(ISR_VECTOR)
)
u_core
(
    .clk_i(clk_i),
    .rst_i(rst_i),

    .enable_i(enable_i),

    .intr_i(core_int_w),
    .nmi_i(1'b0),

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
    .csr_addr_o(csr_addr_w),
    .csr_data_o(csr_data_w),
    .csr_data_i(csr_data_r),
    .csr_set_o(csr_set_w),
    .csr_clr_o(csr_clr_w)
);

//-----------------------------------------------------------------
// Timer
//-----------------------------------------------------------------
reg [31:0] csr_count_q;
reg [31:0] csr_count_r;

reg [31:0] csr_compare_q;
reg [31:0] csr_compare_r;

always @ *
begin
    // Only support (csrrw/csrrwi)
    if (csr_addr_w == `CSR_COUNT && (csr_set_w && csr_clr_w))
        csr_count_r = csr_data_w;
    else
        csr_count_r = csr_count_q + 32'd1;

    // Only support (csrrw/csrrwi)
    if (csr_addr_w == `CSR_COMPARE && (csr_set_w && csr_clr_w))
        csr_compare_r = csr_data_w;
    else
        csr_compare_r = csr_compare_q;
end

always @ (posedge clk_i or posedge rst_i)
if (rst_i == 1'b1)
begin
    csr_count_q   <= 32'b0;
    csr_compare_q <= 32'b0;
end
else
begin
    csr_count_q   <= csr_count_r;
    csr_compare_q <= csr_compare_r;
end

//-----------------------------------------------------------------
// Interrupts
//-----------------------------------------------------------------
reg [7:0]  int_mask_q;
reg [7:0]  int_mask_r;
reg [7:0]  int_status_q;
reg [7:0]  int_status_r;

always @ *
begin
    int_mask_r   = int_mask_q;
    int_status_r = int_status_q;

    if (csr_addr_w == `CSR_STATUS)
    begin
        if (csr_set_w && csr_clr_w)
            int_mask_r = csr_data_w[`SR_IM_R];
        else if (csr_set_w)
            int_mask_r = int_mask_q | csr_data_w[`SR_IM_R];
        else if (csr_clr_w)
            int_mask_r = int_mask_q & ~csr_data_w[`SR_IM_R];

        if (csr_set_w && csr_clr_w)
            int_status_r = csr_data_w[`SR_IP_R];
        else if (csr_set_w)
            int_status_r = int_status_q | csr_data_w[`SR_IP_R];
        else if (csr_clr_w)
            int_status_r = int_status_q & ~csr_data_w[`SR_IP_R];
    end

    // Timer should generate a interrupt?
    int_status_r[`IRQ_TIMER] = int_status_r[`IRQ_TIMER] | (csr_count_q == csr_compare_q);

    // Record new external interrupts
    int_status_r[`IRQ_EXT_R] = int_status_r[`IRQ_EXT_R] | intr_i[`IRQ_EXT_R];
end

always @ (posedge clk_i or posedge rst_i)
if (rst_i == 1'b1)
begin
    int_status_q   <= 8'b0;
    int_mask_q     <= 8'b0;
end
else
begin
    int_status_q   <= int_status_r;
    int_mask_q     <= int_mask_r;
end

assign core_int_w =  |(int_status_q & int_mask_q);

//-----------------------------------------------------------------
// CSR read mux
//-----------------------------------------------------------------
always @ *
begin
    case (csr_addr_w)
    `CSR_COUNT:     csr_data_r = csr_count_q;
    `CSR_COMPARE:   csr_data_r = csr_compare_q;
    `CSR_STATUS:    csr_data_r = {int_status_q, int_mask_q, 16'b0};
    default:        csr_data_r = csr_data_i;
    endcase
end

// CSR access
assign csr_addr_o = csr_addr_w;
assign csr_data_o = csr_data_w;
assign csr_set_o  = csr_set_w;
assign csr_clr_o  = csr_clr_w;

endmodule
