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
// Module - RISCV CPU
//-----------------------------------------------------------------
module riscv
(
    // General
    input               clk_i,
    input               rst_i,

    input  [7:0]        intr_i,
    output              fault_o,
    output              break_o,

    // Instruction memory
    output [31:0]       imem_addr_o,
    input [31:0]        imem_dat_i,
    output [2:0]        imem_cti_o,
    output              imem_cyc_o,
    output              imem_stb_o,
    input               imem_stall_i,
    input               imem_ack_i,  

    // Data memory
    output [31:0]       dmem_addr_o,
    output [31:0]       dmem_dat_o,
    input [31:0]        dmem_dat_i,
    output [3:0]        dmem_sel_o,
    output [2:0]        dmem_cti_o,
    output              dmem_cyc_o,
    output              dmem_we_o,
    output              dmem_stb_o,
    input               dmem_stall_i,
    input               dmem_ack_i
);

//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
parameter           BOOT_VECTOR         = 32'h00000000;
parameter           REGISTER_FILE_TYPE  = "SIMULATION";
parameter           ENABLE_ICACHE       = "ENABLED";

//-----------------------------------------------------------------
// Registers / Wires
//-----------------------------------------------------------------

// Instruction fetch
wire        fetch_rd_w;
wire [31:0] fetch_pc_w;
wire [31:0] fetch_opcode_w;
wire        fetch_valid_w;

// Decode opcode / PC / state
wire [31:0] dec_opcode_w;
wire [31:0] dec_opcode_pc_w;
wire        dec_opcode_valid_w;

// Register numbers
wire [4:0]  dec_rs1_w;
wire [4:0]  dec_rs2_w;

// Destination register number (pre execute stage)
wire [4:0]  dec_rd_w;

// Register value (rs1)
wire [31:0] dec_rs1_val_w;
wire [31:0] resolved_rs1_val_w;

// Register value (rs2)
wire [31:0] dec_rs2_val_w;
wire [31:0] resolved_rs2_val_w;

// Destination register number (post execute stage)
wire [4:0]  ex_rd_w;

// Current executing instruction
wire [31:0] ex_opcode_w;

// Result from execute
wire [31:0] ex_result_w;

// Branch request
wire        ex_branch_w;
wire [31:0] ex_branch_pc_w;
wire        ex_stall_w;

// Load Store stall
wire        lsu_stall_exec_w;
wire        lsu_stall_mem_w;

wire        lsu_squash_w;

// Load result steering
wire [31:0] load_result_w;
wire [4:0]  load_dest_w;
wire        load_accept_w;

// Register writeback value
wire [4:0]  wb_rd_w;
wire [31:0] wb_rd_val_w;

// Register writeback enable
wire        wb_rd_write_w;

wire [31:0] dcache_addr_w;
wire [31:0] dcache_data_out_w;
wire [31:0] dcache_data_in_w;
wire [3:0]  dcache_sel_w;
wire        dcache_we_w;
wire        dcache_stb_w;
wire        dcache_stall_w;
wire        dcache_ack_w;

wire [11:0] csr_addr_w;
wire [31:0] csr_data_w;
reg  [31:0] csr_data_r;
wire        csr_set_w;
wire        csr_clr_w;

wire        core_int_w;

//-----------------------------------------------------------------
// Instruction Cache
//-----------------------------------------------------------------
generate
if (ENABLE_ICACHE == "ENABLED")
begin : ICACHE
    riscv_icache
    u_icache
    (
        .clk_i(clk_i),
        .rst_i(rst_i),
        
        // CPU Interface
        .pc_i(fetch_pc_w),
        .instruction_o(fetch_opcode_w),
        .rd_i(fetch_rd_w),
        .valid_o(fetch_valid_w),
        .invalidate_i(1'b0),

        // Memory Interface
        .wbm_addr_o(imem_addr_o),
        .wbm_dat_i(imem_dat_i),
        .wbm_stb_o(imem_stb_o),
        .wbm_cyc_o(imem_cyc_o),
        .wbm_cti_o(imem_cti_o),
        .wbm_stall_i(imem_stall_i),
        .wbm_ack_i(imem_ack_i)
    );
end
//-----------------------------------------------------------------
// No instruction cache
//-----------------------------------------------------------------
else
begin : NO_ICACHE
    riscv_noicache
    u_noicache
    (
        .clk_i(clk_i),
        .rst_i(rst_i),
        
        // CPU Interface
        .cpu_addr_i(fetch_pc_w),
        .cpu_data_o(fetch_opcode_w),
        .cpu_stb_i(fetch_rd_w),
        .cpu_ack_o(fetch_valid_w),

        // Memory Interface
        .imem_addr_o(imem_addr_o),
        .imem_data_i(imem_dat_i),
        .imem_stb_o(imem_stb_o),
        .imem_cyc_o(imem_cyc_o),
        .imem_cti_o(imem_cti_o),
        .imem_stall_i(imem_stall_i),
        .imem_ack_i(imem_ack_i)
    );
end
endgenerate

//-----------------------------------------------------------------
// Instruction Fetch
//-----------------------------------------------------------------
riscv_fetch 
#(
    .BOOT_VECTOR(BOOT_VECTOR)
)
u_fetch
(
    // General
    .clk_i(clk_i),
    .rst_i(rst_i),
    
    // Instruction memory
    .pc_o(fetch_pc_w),
    .data_i(fetch_opcode_w),
    .fetch_o(fetch_rd_w),
    .data_valid_i(fetch_valid_w),
    
    // Fetched opcode
    .opcode_o(dec_opcode_w),
    .opcode_pc_o(dec_opcode_pc_w),
    .opcode_valid_o(dec_opcode_valid_w),
    
    // Branch target
    .branch_i(ex_branch_w),
    .branch_pc_i(ex_branch_pc_w),    
    .stall_i(ex_stall_w),

    // Decoded register details
    .rs1_o(dec_rs1_w),
    .rs2_o(dec_rs2_w),
    .rd_o(dec_rd_w)
);

//-----------------------------------------------------------------
// [Xilinx] Register file
//-----------------------------------------------------------------
generate
if (REGISTER_FILE_TYPE == "XILINX")
begin : REGFILE_XIL
    riscv_regfile_xil
    u_regfile
    (
        // Clocking
        .clk_i(clk_i),
        .rst_i(rst_i),
        .wr_i(wb_rd_write_w),

        // Tri-port
        .rs1_i(dec_rs1_w),
        .rs2_i(dec_rs2_w),
        .rd_i(wb_rd_w),
        .reg_rs1_o(dec_rs1_val_w),
        .reg_rs2_o(dec_rs2_val_w),
        .reg_rd_i(wb_rd_val_w)
    );
end
//-----------------------------------------------------------------
// [Simulation] Register file
//-----------------------------------------------------------------
else
begin : REGFILE_SIM
    riscv_regfile_sim
    u_regfile
    (
        // Clocking
        .clk_i(clk_i),
        .rst_i(rst_i),
        .wr_i(wb_rd_write_w),

        // Tri-port
        .rs1_i(dec_rs1_w),
        .rs2_i(dec_rs2_w),
        .rd_i(wb_rd_w),
        .reg_rs1_o(dec_rs1_val_w),
        .reg_rs2_o(dec_rs2_val_w),
        .reg_rd_i(wb_rd_val_w)
    );
end
endgenerate

//-----------------------------------------------------------------
// Execution unit
//-----------------------------------------------------------------
riscv_exec
#(
    .BOOT_VECTOR(BOOT_VECTOR)
)
u_exec
(
    // General
    .clk_i(clk_i),
    .rst_i(rst_i),

    .intr_i(core_int_w),
    .break_i(1'b0),
    
    // Status
    .fault_o(fault_o),
    .break_o(break_o),

    // Branch target
    .branch_o(ex_branch_w),
    .branch_pc_o(ex_branch_pc_w),
    .stall_o(ex_stall_w),

    // Opcode & arguments
    .opcode_i(dec_opcode_w),
    .opcode_pc_i(dec_opcode_pc_w),
    .opcode_valid_i(dec_opcode_valid_w),

    // Operands
    .reg_rs1_i(dec_rs1_w),
    .reg_rs1_value_i(dec_rs1_val_w),
    .reg_rs2_i(dec_rs2_w),
    .reg_rs2_value_i(dec_rs2_val_w),
    .reg_rd_i(dec_rd_w),

    // Resolved operands
    .reg_rs1_value_o(resolved_rs1_val_w),
    .reg_rs2_value_o(resolved_rs2_val_w),

    // Load store stall
    .stall_exec_i(lsu_stall_exec_w),

    // Stall from mem stage
    .stall_mem_i(lsu_stall_mem_w),

    // Opcode not being executed (to other stages)
    .squash_opcode_o(lsu_squash_w),

    // CSR access
    .csr_addr_o(csr_addr_w),
    .csr_data_o(csr_data_w),
    .csr_data_i(csr_data_r),
    .csr_set_o(csr_set_w),
    .csr_clr_o(csr_clr_w),

    // Output
    .opcode_o(ex_opcode_w),
    .opcode_pc_o(/* not used */),
    .reg_rd_o(ex_rd_w),
    .reg_rd_value_o(ex_result_w),

    // Register write back bypass
    .wb_rd_i(wb_rd_w),
    .wb_rd_value_i(wb_rd_val_w)
);

//-----------------------------------------------------------------
// Memory Access
//-----------------------------------------------------------------
riscv_mem
u_mem
(
    .clk_i(clk_i),
    .rst_i(rst_i),

    // Opcode & arguments
    .opcode_i(dec_opcode_w),
    .opcode_valid_i(dec_opcode_valid_w),

    // Opcode not being executed (to other stages)
    .squash_opcode_i(lsu_squash_w),

    // Operands
    .reg_rs1_i(dec_rs1_w),
    .reg_rs1_value_i(resolved_rs1_val_w),
    .reg_rs2_i(dec_rs2_w),
    .reg_rs2_value_i(resolved_rs2_val_w),
    .reg_rd_i(dec_rd_w),

    // Load result to writeback
    .load_result_o(load_result_w),
    .load_dest_o(load_dest_w),
    .load_accept_i(load_accept_w),

    // Load stall
    .stall_exec_o(lsu_stall_exec_w),
    .stall_mem_o(lsu_stall_mem_w),

    // Memory Interface
    .dmem_addr_o(dcache_addr_w),
    .dmem_data_out_o(dcache_data_out_w),
    .dmem_data_in_i(dcache_data_in_w),
    .dmem_sel_o(dcache_sel_w),
    .dmem_we_o(dcache_we_w),
    .dmem_stb_o(dcache_stb_w),
    .dmem_stall_i(dcache_stall_w),
    .dmem_ack_i(dcache_ack_w)    
);

//-----------------------------------------------------------------
// Data cache (not present)
//-----------------------------------------------------------------
riscv_dmem_nocache
u_nodcache
(
    .clk_i(clk_i),
    .rst_i(rst_i),

    // CPU Interface
    .cpu_addr_i(dcache_addr_w),
    .cpu_data_out_i(dcache_data_out_w),
    .cpu_data_in_o(dcache_data_in_w),
    .cpu_sel_i(dcache_sel_w),
    .cpu_we_i(dcache_we_w),
    .cpu_stb_i(dcache_stb_w),
    .cpu_stall_o(dcache_stall_w),
    .cpu_ack_o(dcache_ack_w),

    // Memory Interface
    .dmem_addr_o(dmem_addr_o),
    .dmem_data_out_o(dmem_dat_o),
    .dmem_data_in_i(dmem_dat_i),
    .dmem_sel_o(dmem_sel_o),
    .dmem_we_o(dmem_we_o),
    .dmem_stb_o(dmem_stb_o),
    .dmem_cti_o(dmem_cti_o),
    .dmem_cyc_o(dmem_cyc_o),
    .dmem_stall_i(dmem_stall_i),
    .dmem_ack_i(dmem_ack_i)    
);

//-----------------------------------------------------------------
// Register file writeback
//-----------------------------------------------------------------
riscv_writeback 
u_wb
(
    // General
    .clk_i(clk_i),
    .rst_i(rst_i),

    // Stall from mem stage
    .stall_mem_i(lsu_stall_mem_w),

    // Opcode
    .opcode_i(ex_opcode_w),

    // Register target
    .rd_i(ex_rd_w),
    
    // ALU result
    .alu_result_i(ex_result_w),

    // Memory load result
    .load_result_i(load_result_w),
    .load_dest_i(load_dest_w),
    .load_accept_o(load_accept_w),

    // Outputs
    .write_enable_o(wb_rd_write_w),
    .write_addr_o(wb_rd_w),
    .write_data_o(wb_rd_val_w)
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
    default:        csr_data_r = 32'b0;
    endcase
end

//-------------------------------------------------------------------
// Hooks for debug
//-------------------------------------------------------------------
`ifdef verilator
   function [31:0] get_pc;
      // verilator public
      get_pc = dec_opcode_pc_w;
   endfunction
   function get_fault;
      // verilator public
      get_fault = fault_o;
   endfunction  
   function get_break;
      // verilator public
      get_break = break_o;
   endfunction   
`endif

endmodule
