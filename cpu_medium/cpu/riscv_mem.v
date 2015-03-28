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
// Module - Memory Access
//-----------------------------------------------------------------
module riscv_mem
(
    // General
    input               clk_i,
    input               rst_i,

    // Opcode & arguments
    input [31:0]        opcode_i,
    input               opcode_valid_i,

    // From exec: Do not execute opcode
    input               squash_opcode_i,

    // Reg A
    input [4:0]         reg_rs1_i,
    input [31:0]        reg_rs1_value_i,

    // Reg B
    input [4:0]         reg_rs2_i,
    input [31:0]        reg_rs2_value_i,

    // Reg D
    input [4:0]         reg_rd_i,

    // Load result to writeback
    output [31:0]       load_result_o,
    output [4:0]        load_dest_o,
    input               load_accept_i,

    output              stall_exec_o,
    output              stall_mem_o,

    // Memory Interface
    output reg [31:0]   dmem_addr_o,
    output reg [31:0]   dmem_data_out_o,
    input [31:0]        dmem_data_in_i,
    output reg [3:0]    dmem_sel_o,
    output reg          dmem_we_o,
    output reg          dmem_stb_o,
    input               dmem_stall_i,
    input               dmem_ack_i
);

//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-----------------------------------------------------------------
// Defines
//-----------------------------------------------------------------
`define LSU_CTRL_WIDTH  11
`define LSU_CTRL_REG    4:0
`define LSU_CTRL_SIGNED 5
`define LSU_CTRL_OFFSET 7:6
`define LSU_CTRL_LD_OP  10:8

`define LSU_LD_OP_NONE  3'h0
`define LSU_LD_OP_BYTE  3'h1
`define LSU_LD_OP_HALF  3'h2
`define LSU_LD_OP_WORD  3'h3
`define LSU_LD_OP_STORE 3'h4

//-----------------------------------------------------------------
// Opcode decode
//-----------------------------------------------------------------
reg [31:0]  imm12_r;
reg [31:0]  storeimm_r;

always @ *
begin
    imm12_r     = {{20{opcode_i[31]}}, opcode_i[31:20]};
    storeimm_r  = {{20{opcode_i[31]}}, opcode_i[31:25], opcode_i[11:7]};
end

//-----------------------------------------------------------------
// Instruction Decode
//-----------------------------------------------------------------
wire inst_lb_w       = ((opcode_i & `INST_LB_MASK) == `INST_LB);       // lb
wire inst_lh_w       = ((opcode_i & `INST_LH_MASK) == `INST_LH);       // lh
wire inst_lw_w       = ((opcode_i & `INST_LW_MASK) == `INST_LW);       // lw
wire inst_lbu_w      = ((opcode_i & `INST_LBU_MASK) == `INST_LBU);     // lbu
wire inst_lhu_w      = ((opcode_i & `INST_LHU_MASK) == `INST_LHU);     // lhu
wire inst_lwu_w      = ((opcode_i & `INST_LWU_MASK) == `INST_LWU);     // lwu
wire inst_sb_w       = ((opcode_i & `INST_SB_MASK) == `INST_SB);       // sb
wire inst_sh_w       = ((opcode_i & `INST_SH_MASK) == `INST_SH);       // sh
wire inst_sw_w       = ((opcode_i & `INST_SW_MASK) == `INST_SW);       // sw

//-----------------------------------------------------------------
// Load/Store operation?
//-----------------------------------------------------------------
reg         load_inst_r;
reg         store_inst_r;
reg [31:0]  mem_addr_r;
always @ *
begin
    load_inst_r  = inst_lb_w | inst_lh_w | inst_lw_w |
                   inst_lbu_w | inst_lhu_w | inst_lwu_w;
    store_inst_r = inst_sb_w  | inst_sh_w  | inst_sw_w;

    // Memory address is relative to RA
    mem_addr_r = reg_rs1_value_i + (store_inst_r ? storeimm_r : imm12_r);
end

//-----------------------------------------------------------------
// Execute: Memory operations
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
begin
   if (rst_i == 1'b1)
   begin
       // Data memory
       dmem_addr_o          <= 32'h00000000;
       dmem_data_out_o      <= 32'h00000000;
       dmem_we_o            <= 1'b0;
       dmem_sel_o           <= 4'b0000;
       dmem_stb_o           <= 1'b0;
   end
   // Valid instruction to execute
   else if (opcode_valid_i && !squash_opcode_i && !stall_exec_o)
   begin
        case (1'b1)

        // l.lbs l.lhs l.lws l.lbz l.lhz l.lwz
        load_inst_r:
        begin
             dmem_addr_o      <= mem_addr_r;
             dmem_data_out_o  <= 32'h00000000;
             dmem_sel_o       <= 4'b1111;
             dmem_we_o        <= 1'b0;
             dmem_stb_o       <= 1'b1;
        end

        inst_sb_w: // l.sb
        begin
             dmem_addr_o <= mem_addr_r;

             case (mem_addr_r[1:0])
                 2'h3 :
                 begin
                     dmem_data_out_o  <= {reg_rs2_value_i[7:0],24'h000000};
                     dmem_sel_o       <= 4'b1000;
                     dmem_we_o        <= 1'b1;
                     dmem_stb_o       <= 1'b1;
                 end
                 2'h2 :
                 begin
                     dmem_data_out_o  <= {{8'h00,reg_rs2_value_i[7:0]},16'h0000};
                     dmem_sel_o       <= 4'b0100;
                     dmem_we_o        <= 1'b1;
                     dmem_stb_o       <= 1'b1;
                 end
                 2'h1 :
                 begin
                     dmem_data_out_o  <= {{16'h0000,reg_rs2_value_i[7:0]},8'h00};
                     dmem_sel_o       <= 4'b0010;
                     dmem_we_o        <= 1'b1;
                     dmem_stb_o       <= 1'b1;
                 end
                 2'h0 :
                 begin
                     dmem_data_out_o  <= {24'h000000,reg_rs2_value_i[7:0]};
                     dmem_sel_o       <= 4'b0001;
                     dmem_we_o        <= 1'b1;
                     dmem_stb_o       <= 1'b1;
                 end
                 default :
                    ;
             endcase
        end

        inst_sh_w: // l.sh
        begin
             dmem_addr_o <= mem_addr_r;

             case (mem_addr_r[1:0])
                 2'h2 :
                 begin
                     dmem_data_out_o  <= {reg_rs2_value_i[15:0],16'h0000};
                     dmem_sel_o       <= 4'b1100;
                     dmem_we_o        <= 1'b1;
                     dmem_stb_o       <= 1'b1;
                 end
                 default :
                 begin
                     dmem_data_out_o  <= {16'h0000,reg_rs2_value_i[15:0]};
                     dmem_sel_o       <= 4'b0011;
                     dmem_we_o        <= 1'b1;
                     dmem_stb_o       <= 1'b1;
                 end
             endcase
        end

        inst_sw_w: // l.sw
        begin
             dmem_addr_o      <= mem_addr_r;
             dmem_data_out_o  <= reg_rs2_value_i;
             dmem_sel_o       <= 4'b1111;
             dmem_we_o        <= 1'b1;
             dmem_stb_o       <= 1'b1;
        end

        // Non load / store
        default:
        begin
            dmem_addr_o          <= 32'h00000000;
            dmem_data_out_o      <= 32'h00000000;
            dmem_we_o            <= 1'b0;
            dmem_sel_o           <= 4'b0000;
            dmem_stb_o           <= 1'b0;
        end
        endcase
    end
    // No instruction, clear memory request
    else if (!dmem_stall_i)
    begin
        dmem_addr_o          <= 32'h00000000;
        dmem_data_out_o      <= 32'h00000000;
        dmem_we_o            <= 1'b0;
        dmem_sel_o           <= 4'b0000;
        dmem_stb_o           <= 1'b0;
    end
end

//-------------------------------------------------------------------
// Load control pipeline
//-------------------------------------------------------------------
reg [`LSU_CTRL_WIDTH-1:0]   lsu_ctrl_q;
always @ (posedge clk_i or posedge rst_i)
begin
    if (rst_i == 1'b1)
        lsu_ctrl_q <= `LSU_CTRL_WIDTH'b0;
    // Valid instruction
    else if (opcode_valid_i && !squash_opcode_i && !stall_exec_o)
    begin
        case (1'b1)

        inst_lb_w:
        begin
            lsu_ctrl_q[`LSU_CTRL_REG]       <= reg_rd_i;
            lsu_ctrl_q[`LSU_CTRL_SIGNED]    <= 1'b1;
            lsu_ctrl_q[`LSU_CTRL_LD_OP]     <= `LSU_LD_OP_BYTE;
            lsu_ctrl_q[`LSU_CTRL_OFFSET]    <= mem_addr_r[1:0];
        end

        inst_lh_w:
        begin
            lsu_ctrl_q[`LSU_CTRL_REG]       <= reg_rd_i;
            lsu_ctrl_q[`LSU_CTRL_SIGNED]    <= 1'b1;
            lsu_ctrl_q[`LSU_CTRL_LD_OP]     <= `LSU_LD_OP_HALF;
            lsu_ctrl_q[`LSU_CTRL_OFFSET]    <= mem_addr_r[1:0];
        end

        inst_lw_w, inst_lwu_w:
        begin
            lsu_ctrl_q[`LSU_CTRL_REG]       <= reg_rd_i;
            lsu_ctrl_q[`LSU_CTRL_SIGNED]    <= 1'b0; // Don't care
            lsu_ctrl_q[`LSU_CTRL_LD_OP]     <= `LSU_LD_OP_WORD;
            lsu_ctrl_q[`LSU_CTRL_OFFSET]    <= mem_addr_r[1:0];
        end

        inst_lbu_w:
        begin
            lsu_ctrl_q[`LSU_CTRL_REG]       <= reg_rd_i;
            lsu_ctrl_q[`LSU_CTRL_SIGNED]    <= 1'b0;
            lsu_ctrl_q[`LSU_CTRL_LD_OP]     <= `LSU_LD_OP_BYTE;
            lsu_ctrl_q[`LSU_CTRL_OFFSET]    <= mem_addr_r[1:0];
        end

        inst_lhu_w:
        begin
            lsu_ctrl_q[`LSU_CTRL_REG]       <= reg_rd_i;
            lsu_ctrl_q[`LSU_CTRL_SIGNED]    <= 1'b0;
            lsu_ctrl_q[`LSU_CTRL_LD_OP]     <= `LSU_LD_OP_HALF;
            lsu_ctrl_q[`LSU_CTRL_OFFSET]    <= mem_addr_r[1:0];
        end

        inst_sb_w, inst_sh_w, inst_sw_w:
        begin
            lsu_ctrl_q[`LSU_CTRL_REG]       <= 5'b0;
            lsu_ctrl_q[`LSU_CTRL_SIGNED]    <= 1'b0;
            lsu_ctrl_q[`LSU_CTRL_LD_OP]     <= `LSU_LD_OP_STORE;
            lsu_ctrl_q[`LSU_CTRL_OFFSET]    <= mem_addr_r[1:0];
        end

        default:
        begin
            lsu_ctrl_q[`LSU_CTRL_REG]       <= 5'b0;
            lsu_ctrl_q[`LSU_CTRL_SIGNED]    <= 1'b0;
            lsu_ctrl_q[`LSU_CTRL_LD_OP]     <= `LSU_LD_OP_NONE;
            lsu_ctrl_q[`LSU_CTRL_OFFSET]    <= mem_addr_r[1:0];
        end
        endcase
    end
    else if (!stall_mem_o)
        lsu_ctrl_q <= `LSU_CTRL_WIDTH'b0;
end

// Delayed version of LSU control signals
reg [`LSU_CTRL_WIDTH-1:0]   wb_lsu_ctrl_q;
always @ (posedge clk_i or posedge rst_i)
    if (rst_i == 1'b1)
        wb_lsu_ctrl_q <= `LSU_CTRL_WIDTH'b0;
    else if (!stall_mem_o)
        wb_lsu_ctrl_q <= lsu_ctrl_q;

//-------------------------------------------------------------------
// Load result resolve
//-------------------------------------------------------------------
reg [31:0] load_result_r;
wire [1:0] load_offset_w = wb_lsu_ctrl_q[`LSU_CTRL_OFFSET];

always @ *
begin
    if (wb_lsu_ctrl_q[`LSU_CTRL_LD_OP] == `LSU_LD_OP_BYTE)
    begin
        case (load_offset_w[1:0])
            2'h3:
                load_result_r = {24'b0, dmem_data_in_i[31:24]};
            2'h2:
                load_result_r = {24'b0, dmem_data_in_i[23:16]};
            2'h1:
                load_result_r = {24'b0, dmem_data_in_i[15:8]};
            2'h0:
                load_result_r = {24'b0, dmem_data_in_i[7:0]};
        endcase

        if (wb_lsu_ctrl_q[`LSU_CTRL_SIGNED] && load_result_r[7])
            load_result_r = {24'hFFFFFF, load_result_r[7:0]};
    end
    else if (wb_lsu_ctrl_q[`LSU_CTRL_LD_OP] == `LSU_LD_OP_HALF)
    begin
        if (load_offset_w[1])
            load_result_r = {16'b0, dmem_data_in_i[31:16]};
        else
            load_result_r = {16'b0, dmem_data_in_i[15:0]};

        if (wb_lsu_ctrl_q[`LSU_CTRL_SIGNED] && load_result_r[15])
            load_result_r = {16'hFFFF, load_result_r[15:0]};
    end
    else
    begin
        load_result_r = dmem_data_in_i;
    end
end

assign load_dest_o      = dmem_ack_i ? wb_lsu_ctrl_q[`LSU_CTRL_REG] : 5'b0;
assign load_result_o    = load_result_r;

//-------------------------------------------------------------------
// Load dependancy checker
//-------------------------------------------------------------------

// Mem ack not ready in writeback stage
assign stall_mem_o = (wb_lsu_ctrl_q[`LSU_CTRL_LD_OP] !=`LSU_LD_OP_NONE) & ~dmem_ack_i;

// Outstanding load dependancy (i.e. load delay slot)
assign stall_exec_o = ((lsu_ctrl_q[`LSU_CTRL_REG] == reg_rs1_i) & (reg_rs1_i != 5'b0)) |
                      ((lsu_ctrl_q[`LSU_CTRL_REG] == reg_rs2_i) & (reg_rs2_i != 5'b0)) | 
                      stall_mem_o | dmem_stall_i;

endmodule
