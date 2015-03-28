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
// Module - Instruction Execute
//-----------------------------------------------------------------
module riscv_exec
(
    // General
    input               clk_i,
    input               rst_i,

    // Maskable interrupt    
    input               intr_i,

    // Break interrupt
    input               break_i,

    // Fault
    output reg          fault_o,

    // Breakpoint / Trap
    output reg          break_o,

    // Branch
    output              branch_o,
    output [31:0]       branch_pc_o,
    output              stall_o,

    // Opcode & arguments
    input [31:0]        opcode_i,
    input [31:0]        opcode_pc_i,
    input               opcode_valid_i,

    // Reg A
    input [4:0]         reg_rs1_i,
    input [31:0]        reg_rs1_value_i,
    output [31:0]       reg_rs1_value_o,

    // Reg B
    input [4:0]         reg_rs2_i,
    input [31:0]        reg_rs2_value_i,
    output [31:0]       reg_rs2_value_o,

    // Reg D
    input [4:0]         reg_rd_i,

    // Stall stages before execute
    input               stall_exec_i,

    // Stall stage before mem
    input               stall_mem_i,

    // Opcode not being executed (to other stages)
    output              squash_opcode_o,

    // CSR access
    output [11:0]       csr_addr_o,
    output [31:0]       csr_data_o,
    input [31:0]        csr_data_i,
    output              csr_set_o,
    output              csr_clr_o,

    // Output
    output [31:0]       opcode_o,
    output [31:0]       opcode_pc_o,
    output [4:0]        reg_rd_o,
    output [31:0]       reg_rd_value_o,

    // Register write back bypass
    input [4:0]         wb_rd_i,
    input [31:0]        wb_rd_value_i
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

//-----------------------------------------------------------------
// Registers
//-----------------------------------------------------------------

// Branch PC
reg [31:0]  pc_branch_q;
reg         pc_fetch_q;

// Destination register number (post execute stage)
reg [4:0]   ex_rd_q;

// Current opcode (PC for debug)
reg [31:0]  ex_opcode_q;
reg [31:0]  ex_opcode_pc_q;

// ALU input A
reg [31:0]  ex_alu_a_q;

// ALU input B
reg [31:0]  ex_alu_b_q;

// ALU output
wire [31:0] ex_result_w;

// ALU operation selection
reg [3:0]   ex_alu_func_q;

// CSR read data
reg [31:0]  csr_data_r;

// CSR Registers
reg [31:0]  csr_epc_q;
reg [31:0]  csr_epc_r;
reg [31:0]  csr_evec_q;
reg [31:0]  csr_evec_r;
reg [4:0]   csr_cause_q;
reg [4:0]   csr_cause_r;
reg [3:0]   csr_sr_q;
reg [3:0]   csr_sr_r;

reg         invalid_inst_r;

//-----------------------------------------------------------------
// ALU
//-----------------------------------------------------------------
riscv_alu alu
(
    // ALU operation select
    .op_i(ex_alu_func_q),

    // Operands
    .a_i(ex_alu_a_q),
    .b_i(ex_alu_b_q),

    // Result
    .p_o(ex_result_w)
);

//-----------------------------------------------------------------
// Opcode decode
//-----------------------------------------------------------------
reg [31:0]  imm20_r;
reg [31:0]  imm12_r;
reg [31:0]  bimm_r;
reg [31:0]  jimm20_r;
reg [4:0]   shamt_r;

always @ *
begin
    imm20_r     = {opcode_i[31:12], 12'b0};
    imm12_r     = {{20{opcode_i[31]}}, opcode_i[31:20]};
    bimm_r      = {{19{opcode_i[31]}}, opcode_i[31], opcode_i[7], opcode_i[30:25], opcode_i[11:8], 1'b0};
    jimm20_r    = {{12{opcode_i[31]}}, opcode_i[19:12], opcode_i[20], opcode_i[30:25], opcode_i[24:21], 1'b0};
    shamt_r     = opcode_i[24:20];
end

//-----------------------------------------------------------------
// Register source selection
//-----------------------------------------------------------------
reg [31:0] reg_rs1_r;
reg [31:0] reg_rs2_r;

always @ *
begin
    // Bypass: Exec
    if (ex_rd_q != 5'b0 && ex_rd_q == reg_rs1_i)
        reg_rs1_r   = ex_result_w;
    // Bypass: Writeback
    else if (wb_rd_i != 5'b0 && wb_rd_i == reg_rs1_i)
        reg_rs1_r   = wb_rd_value_i;
    // Normal: Regfile
    else    
        reg_rs1_r   = reg_rs1_value_i;

    // Bypass: Exec
    if (ex_rd_q != 5'b0 && ex_rd_q == reg_rs2_i)
        reg_rs2_r   = ex_result_w;
    // Bypass: Writeback
    else if (wb_rd_i != 5'b0 && wb_rd_i == reg_rs2_i)
        reg_rs2_r   = wb_rd_value_i;
    // Normal: Regfile
    else    
        reg_rs2_r   = reg_rs2_value_i;
end

// Resolved values
assign reg_rs1_value_o = reg_rs1_r;
assign reg_rs2_value_o = reg_rs2_r;

//-----------------------------------------------------------------
// Instruction Decode
//-----------------------------------------------------------------
wire inst_andi_w     = ((opcode_i & `INST_ANDI_MASK) == `INST_ANDI);   // andi
wire inst_addi_w     = ((opcode_i & `INST_ADDI_MASK) == `INST_ADDI);   // addi
wire inst_slti_w     = ((opcode_i & `INST_SLTI_MASK) == `INST_SLTI);   // slti
wire inst_sltiu_w    = ((opcode_i & `INST_SLTIU_MASK) == `INST_SLTIU); // sltiu
wire inst_ori_w      = ((opcode_i & `INST_ORI_MASK) == `INST_ORI);     // ori
wire inst_xori_w     = ((opcode_i & `INST_XORI_MASK) == `INST_XORI);   // xori
wire inst_slli_w     = ((opcode_i & `INST_SLLI_MASK) == `INST_SLLI);   // slli
wire inst_srli_w     = ((opcode_i & `INST_SRLI_MASK) == `INST_SRLI);   // srli
wire inst_srai_w     = ((opcode_i & `INST_SRAI_MASK) == `INST_SRAI);   // srai
wire inst_lui_w      = ((opcode_i & `INST_LUI_MASK) == `INST_LUI);     // lui
wire inst_auipc_w    = ((opcode_i & `INST_AUIPC_MASK) == `INST_AUIPC); // auipc
wire inst_add_w      = ((opcode_i & `INST_ADD_MASK) == `INST_ADD);     // add
wire inst_sub_w      = ((opcode_i & `INST_SUB_MASK) == `INST_SUB);     // sub
wire inst_slt_w      = ((opcode_i & `INST_SLT_MASK) == `INST_SLT);     // slt
wire inst_sltu_w     = ((opcode_i & `INST_SLTU_MASK) == `INST_SLTU);   // sltu
wire inst_xor_w      = ((opcode_i & `INST_XOR_MASK) == `INST_XOR);     // xor
wire inst_or_w       = ((opcode_i & `INST_OR_MASK) == `INST_OR);       // or
wire inst_and_w      = ((opcode_i & `INST_AND_MASK) == `INST_AND);     // and
wire inst_sll_w      = ((opcode_i & `INST_SLL_MASK) == `INST_SLL);     // sll
wire inst_srl_w      = ((opcode_i & `INST_SRL_MASK) == `INST_SRL);     // srl
wire inst_sra_w      = ((opcode_i & `INST_SRA_MASK) == `INST_SRA);     // sra
wire inst_jal_w      = ((opcode_i & `INST_JAL_MASK) == `INST_JAL);     // jal
wire inst_jalr_w     = ((opcode_i & `INST_JALR_MASK) == `INST_JALR);   // jalr
wire inst_beq_w      = ((opcode_i & `INST_BEQ_MASK) == `INST_BEQ);     // beq
wire inst_bne_w      = ((opcode_i & `INST_BNE_MASK) == `INST_BNE);     // bne
wire inst_blt_w      = ((opcode_i & `INST_BLT_MASK) == `INST_BLT);     // blt
wire inst_bge_w      = ((opcode_i & `INST_BGE_MASK) == `INST_BGE);     // bge
wire inst_bltu_w     = ((opcode_i & `INST_BLTU_MASK) == `INST_BLTU);   // bltu
wire inst_bgeu_w     = ((opcode_i & `INST_BGEU_MASK) == `INST_BGEU);   // bgeu
wire inst_lb_w       = ((opcode_i & `INST_LB_MASK) == `INST_LB);       // lb
wire inst_lh_w       = ((opcode_i & `INST_LH_MASK) == `INST_LH);       // lh
wire inst_lw_w       = ((opcode_i & `INST_LW_MASK) == `INST_LW);       // lw
wire inst_lbu_w      = ((opcode_i & `INST_LBU_MASK) == `INST_LBU);     // lbu
wire inst_lhu_w      = ((opcode_i & `INST_LHU_MASK) == `INST_LHU);     // lhu
wire inst_lwu_w      = ((opcode_i & `INST_LWU_MASK) == `INST_LWU);     // lwu
wire inst_sb_w       = ((opcode_i & `INST_SB_MASK) == `INST_SB);       // sb
wire inst_sh_w       = ((opcode_i & `INST_SH_MASK) == `INST_SH);       // sh
wire inst_sw_w       = ((opcode_i & `INST_SW_MASK) == `INST_SW);       // sw
wire inst_scall_w    = ((opcode_i & `INST_SCALL_MASK) == `INST_SCALL); // scall
wire inst_sbreak_w   = ((opcode_i & `INST_SBREAK_MASK) == `INST_SBREAK); // sbreak
wire inst_sret_w     = ((opcode_i & `INST_SRET_MASK) == `INST_SRET);   // sret
wire inst_csrrw_w    = ((opcode_i & `INST_CSRRW_MASK) == `INST_CSRRW); // csrrw
wire inst_csrrs_w    = ((opcode_i & `INST_CSRRS_MASK) == `INST_CSRRS); // csrrs
wire inst_csrrc_w    = ((opcode_i & `INST_CSRRC_MASK) == `INST_CSRRC); // csrrc
wire inst_csrrwi_w   = ((opcode_i & `INST_CSRRWI_MASK) == `INST_CSRRWI); // csrrwi
wire inst_csrrsi_w   = ((opcode_i & `INST_CSRRSI_MASK) == `INST_CSRRSI); // csrrsi
wire inst_csrrci_w   = ((opcode_i & `INST_CSRRCI_MASK) == `INST_CSRRCI); // csrrci

//-----------------------------------------------------------------
// Stall / Execute
//-----------------------------------------------------------------
reg execute_inst_r;

always @ *
begin
    // No opcode ready or load store unit stall
    if (~opcode_valid_i || stall_exec_i)
        execute_inst_r  = 1'b0;
    else
        execute_inst_r  = 1'b1;
end

// Interrupt request and interrupt enabled
wire take_interrupt_w = intr_i & csr_sr_q[`SR_EI];
wire exception_w      = take_interrupt_w || (execute_inst_r && invalid_inst_r);

assign squash_opcode_o = exception_w;

//-----------------------------------------------------------------
// ALU inputs
//-----------------------------------------------------------------

// ALU operation selection
reg [3:0]  alu_func_r;

// ALU operands
reg [31:0] alu_input_a_r;
reg [31:0] alu_input_b_r;
reg        write_rd_r;

always @ *
begin
   alu_func_r     = `ALU_NONE;
   alu_input_a_r  = 32'b0;
   alu_input_b_r  = 32'b0;
   write_rd_r     = 1'b0;

   case (1'b1)

     inst_add_w: // add
     begin
       alu_func_r     = `ALU_ADD;
       alu_input_a_r  = reg_rs1_r;
       alu_input_b_r  = reg_rs2_r;
       write_rd_r     = 1'b1;
     end

     inst_and_w: // and
     begin
         alu_func_r     = `ALU_AND;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = reg_rs2_r;
         write_rd_r     = 1'b1;
     end

     inst_or_w: // or
     begin
         alu_func_r     = `ALU_OR;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = reg_rs2_r;
         write_rd_r     = 1'b1;
     end

     inst_sll_w: // sll
     begin
         alu_func_r     = `ALU_SHIFTL;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = reg_rs2_r;
         write_rd_r     = 1'b1;
     end

     inst_sra_w: // sra
     begin
         alu_func_r     = `ALU_SHIFTR_ARITH;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = reg_rs2_r;
         write_rd_r     = 1'b1;
     end

     inst_srl_w: // srl
     begin
         alu_func_r     = `ALU_SHIFTR;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = reg_rs2_r;
         write_rd_r     = 1'b1;
     end

     inst_sub_w: // sub
     begin
         alu_func_r     = `ALU_SUB;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = reg_rs2_r;
         write_rd_r     = 1'b1;
     end

     inst_xor_w: // xor
     begin
         alu_func_r     = `ALU_XOR;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = reg_rs2_r;
         write_rd_r     = 1'b1;
     end

     inst_slt_w: // slt
     begin
         alu_func_r     = `ALU_LESS_THAN_SIGNED;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = reg_rs2_r;
         write_rd_r     = 1'b1;
     end

     inst_sltu_w: // sltu
     begin
         alu_func_r     = `ALU_LESS_THAN;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = reg_rs2_r;
         write_rd_r     = 1'b1;
     end

     inst_addi_w: // addi
     begin
         alu_func_r     = `ALU_ADD;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = imm12_r;
         write_rd_r     = 1'b1;
     end

     inst_andi_w: // andi
     begin
         alu_func_r     = `ALU_AND;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = imm12_r;
         write_rd_r     = 1'b1;
     end

     inst_slti_w: // slti
     begin
         alu_func_r     = `ALU_LESS_THAN_SIGNED;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = imm12_r;
         write_rd_r     = 1'b1;
     end

     inst_sltiu_w: // sltiu
     begin
         alu_func_r     = `ALU_LESS_THAN;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = imm12_r;
         write_rd_r     = 1'b1;
     end

     inst_ori_w: // ori
     begin
         alu_func_r     = `ALU_OR;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = imm12_r;
         write_rd_r     = 1'b1;
     end

     inst_xori_w: // xori
     begin
         alu_func_r     = `ALU_XOR;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = imm12_r;
         write_rd_r     = 1'b1;
     end

     inst_slli_w: // slli
     begin
         alu_func_r     = `ALU_SHIFTL;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = {27'b0, shamt_r};
         write_rd_r     = 1'b1;
     end

     inst_srli_w: // srli
     begin
         alu_func_r     = `ALU_SHIFTR;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = {27'b0, shamt_r};
         write_rd_r     = 1'b1;
     end

     inst_srai_w: // srai
     begin
         alu_func_r     = `ALU_SHIFTR_ARITH;
         alu_input_a_r  = reg_rs1_r;
         alu_input_b_r  = {27'b0, shamt_r};
         write_rd_r     = 1'b1;
     end

     inst_lui_w: // lui
     begin
         alu_input_a_r  = imm20_r;
         write_rd_r     = 1'b1;
     end

     inst_auipc_w: // auipc
     begin
         alu_func_r     = `ALU_ADD;
         alu_input_a_r  = opcode_pc_i;
         alu_input_b_r  = imm20_r;
         write_rd_r     = 1'b1;
     end     

     inst_jal_w,  // jal
     inst_jalr_w: // jalr
     begin
         alu_func_r     = `ALU_ADD;
         alu_input_a_r  = opcode_pc_i;
         alu_input_b_r  = 4;
         write_rd_r     = 1'b1;
     end

     // lb lh lw lbu lhu lwu
     inst_lb_w,
     inst_lh_w,
     inst_lw_w,
     inst_lbu_w,
     inst_lhu_w,
     inst_lwu_w:
          write_rd_r    = 1'b0;

     inst_csrrw_w,  // csrrw
     inst_csrrs_w,  // csrrs
     inst_csrrc_w,  // csrrc
     inst_csrrwi_w, // csrrwi
     inst_csrrsi_w, // csrrsi
     inst_csrrci_w: // csrrci    
     begin
         alu_func_r     = `ALU_NONE;
         alu_input_a_r  = csr_data_r;
         write_rd_r     = 1'b1;
     end

     default:
        ;     
   endcase
end

//-----------------------------------------------------------------
// Branches
//-----------------------------------------------------------------
reg         branch_r;
reg [31:0]  branch_target_r;
reg         branch_except_r;

always @ *
begin

    branch_r        = 1'b0;
    branch_except_r = 1'b0; 

    // Default branch target is relative to current PC
    branch_target_r = (opcode_pc_i + bimm_r);    

    case (1'b1)
    inst_jal_w: // jal
    begin
        branch_r        = 1'b1;
        branch_target_r = opcode_pc_i + jimm20_r;
    end

    inst_jalr_w: // jalr
    begin
        branch_r            = 1'b1;
        branch_target_r     = reg_rs1_r + imm12_r;
        branch_target_r[0]  = 1'b0;
    end

    inst_beq_w: // beq
        branch_r      = (reg_rs1_r == reg_rs2_r);

    inst_bne_w: // bne
        branch_r      = (reg_rs1_r != reg_rs2_r);

    inst_blt_w: // blt
        branch_r      = less_than_signed(reg_rs1_r, reg_rs2_r);

    inst_bge_w: // bge
        branch_r      = greater_than_signed(reg_rs1_r, reg_rs2_r) | (reg_rs1_r == reg_rs2_r);

    inst_bltu_w: // bltu
        branch_r      = (reg_rs1_r < reg_rs2_r);

    inst_bgeu_w: // bgeu
        branch_r      = (reg_rs1_r >= reg_rs2_r);

    default:
        ;
    endcase
end

//-----------------------------------------------------------------
// Invalid instruction
//-----------------------------------------------------------------
always @ *
begin
    case (1'b1)
       inst_andi_w,
       inst_addi_w,
       inst_slti_w,
       inst_sltiu_w,
       inst_ori_w,
       inst_xori_w,
       inst_slli_w,
       inst_srli_w,
       inst_srai_w,
       inst_lui_w,
       inst_auipc_w,
       inst_add_w,
       inst_sub_w,
       inst_slt_w,
       inst_sltu_w,
       inst_xor_w,
       inst_or_w,
       inst_and_w,
       inst_sll_w,
       inst_srl_w,
       inst_sra_w,
       inst_jal_w,
       inst_jalr_w,
       inst_beq_w,
       inst_bne_w,
       inst_blt_w,
       inst_bge_w,
       inst_bltu_w,
       inst_bgeu_w,
       inst_lb_w,
       inst_lh_w,
       inst_lw_w,
       inst_lbu_w,
       inst_lhu_w,
       inst_lwu_w,
       inst_sb_w,
       inst_sh_w,
       inst_sw_w,
       inst_scall_w,
       inst_sbreak_w,
       inst_sret_w,
       inst_csrrw_w,
       inst_csrrs_w,
       inst_csrrc_w,
       inst_csrrwi_w,
       inst_csrrsi_w,
       inst_csrrci_w:
          invalid_inst_r = 1'b0;
       default:
          invalid_inst_r = 1'b1;
    endcase
end

//-----------------------------------------------------------------
// Execute: ALU control
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
begin
   if (rst_i == 1'b1)
   begin      
       ex_alu_func_q         <= `ALU_NONE;
       ex_alu_a_q            <= 32'h00000000;
       ex_alu_b_q            <= 32'h00000000;
       ex_rd_q               <= 5'b00000;
   end
   else if (!stall_mem_i)
   begin           
       //---------------------------------------------------------------
       // Instruction not ready / take exception
       //---------------------------------------------------------------
       if (~execute_inst_r || exception_w)
       begin
           // No ALU operation (output == input_a)
           ex_alu_func_q   <= `ALU_NONE;
           ex_alu_a_q      <= 32'b0;
           ex_alu_b_q      <= 32'b0;
           ex_rd_q         <= 5'b0;
       end   
       //---------------------------------------------------------------
       // Valid instruction
       //---------------------------------------------------------------
       else
       begin
           // Update ALU input flops
           ex_alu_func_q         <= alu_func_r;
           ex_alu_a_q            <= alu_input_a_r;
           ex_alu_b_q            <= alu_input_b_r;
   
           // Instruction with register writeback
           if (write_rd_r)
              ex_rd_q            <= reg_rd_i;
           else
              ex_rd_q            <= 5'b0;
       end
   end
end

//-----------------------------------------------------------------
// Execute: Update executed PC / opcode
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
begin
   if (rst_i == 1'b1)
   begin      
       ex_opcode_q           <= 32'h00000000;
       ex_opcode_pc_q        <= 32'h00000000;
   end
   else if (!stall_mem_i)
   begin
       // Instruction not ready / take exception
       if (~execute_inst_r || exception_w)
       begin
           // Store bubble opcode
           ex_opcode_q            <= 32'h00000000;
           ex_opcode_pc_q         <= 32'h00000000;
       end   
       // Valid instruction
       else
       begin
           // Store opcode
           ex_opcode_q            <= opcode_i;
           ex_opcode_pc_q         <= opcode_pc_i;
       end
   end
end

//-----------------------------------------------------------------
// Execute: Branch / exceptions
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
begin
   if (rst_i == 1'b1)
   begin
       pc_branch_q          <= 32'h00000000;
       pc_fetch_q           <= 1'b0;
   end
   else if (!stall_mem_i)
   begin
       // Reset branch request
       pc_fetch_q           <= 1'b0;

       // Instruction ready
       if (execute_inst_r)
       begin
           // Exception / Break / Scall
           if (exception_w || inst_sbreak_w || inst_scall_w) 
           begin
                // Perform branch to EVEC
                pc_branch_q     <= csr_evec_r;
                pc_fetch_q      <= 1'b1;
           end
           // SRET
           else if (inst_sret_w) 
           begin
                // Perform branch to EPC
                pc_branch_q    <= csr_epc_r;
                pc_fetch_q     <= 1'b1;
           end           
           // Branch
           else if (branch_r)
           begin
                // Perform branch
                pc_branch_q    <= branch_target_r;
                pc_fetch_q     <= 1'b1;
           end
      end
   end
end

//-----------------------------------------------------------------
// Execute: Break / Fault Status
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
begin
   if (rst_i == 1'b1)
   begin
       fault_o              <= 1'b0;
       break_o              <= 1'b0;
   end
   else
   begin
       // Instruction ready
       if (execute_inst_r && !stall_mem_i)
       begin
            // Invalid instruction
            if (invalid_inst_r)
                fault_o     <= 1'b1;

           // Break Instruction
           if (inst_sbreak_w) 
                break_o     <= 1'b1;
           else
                break_o     <= 1'b0;
      end
      else
          break_o     <= 1'b0;
   end
end

//-----------------------------------------------------------------
// Execute: CSR Access
//-----------------------------------------------------------------
always @ *
begin
    csr_epc_r    = csr_epc_q;
    csr_evec_r   = csr_evec_q;
    csr_cause_r  = csr_cause_q;
    csr_sr_r     = csr_sr_q;

    // Execute instruction / exception
    if (execute_inst_r && !stall_mem_i)
    begin
        // Exception / break / scall
        if (exception_w || inst_sbreak_w || inst_scall_w) 
        begin
            // Save interrupt / supervisor state
            csr_sr_r[`SR_PEI] = csr_sr_q[`SR_EI];
            csr_sr_r[`SR_PS]  = csr_sr_q[`SR_S];

            // Disable interrupts and enter supervisor mode
            csr_sr_r[`SR_EI]  = 1'b0;
            csr_sr_r[`SR_S]   = 1'b1;

            // Save PC of next instruction (not yet executed)
            csr_epc_r         = opcode_pc_i;

            // Exception source
            if (invalid_inst_r)
                csr_cause_r   = `CAUSE_ILLEGAL_INSTRUCTION;
            else if (inst_sbreak_w)
                csr_cause_r   = `CAUSE_BREAKPOINT;
            else if (inst_scall_w)
                csr_cause_r   = `CAUSE_SYSCALL;
            else
                csr_cause_r   = `CAUSE_INTERRUPT;
        end
       // SRET
       else if (inst_sret_w) 
       begin
            // Restore interrupt / supervisor state
            csr_sr_r[`SR_EI] = csr_sr_q[`SR_PEI];
            csr_sr_r[`SR_S]  = csr_sr_q[`SR_PS];
       end  
        else
        begin
            case (csr_addr_o)
            `CSR_EPC:
            begin
                if (csr_set_o && csr_clr_o)
                    csr_epc_r = csr_data_o;
                else if (csr_set_o)
                    csr_epc_r = csr_epc_r | csr_data_o;
                else if (csr_clr_o)
                    csr_epc_r = csr_epc_r & ~csr_data_o;
            end
            `CSR_EVEC:
            begin
                if (csr_set_o && csr_clr_o)
                    csr_evec_r = csr_data_o;
                else if (csr_set_o)
                    csr_evec_r = csr_evec_r | csr_data_o;
                else if (csr_clr_o)
                    csr_evec_r = csr_evec_r & ~csr_data_o;
            end
            `CSR_CAUSE:
            begin
                if (csr_set_o && csr_clr_o)
                    csr_cause_r = csr_data_o[4:0];
                else if (csr_set_o)
                    csr_cause_r = csr_cause_r | csr_data_o[4:0];
                else if (csr_clr_o)
                    csr_cause_r = csr_cause_r & ~csr_data_o[4:0];
            end
            `CSR_STATUS:
            begin
                if (csr_set_o && csr_clr_o)
                    csr_sr_r = csr_data_o[3:0];
                else if (csr_set_o)
                    csr_sr_r = csr_sr_r | csr_data_o[3:0];
                else if (csr_clr_o)
                    csr_sr_r = csr_sr_r & ~csr_data_o[3:0];
            end
            default:
              ;
            endcase
        end
    end
end

always @ (posedge clk_i or posedge rst_i)
begin
   if (rst_i == 1'b1)
   begin
       csr_epc_q        <= 32'b0;
       csr_evec_q       <= 32'b0;
       csr_cause_q      <= 5'b0;
       csr_sr_q         <= 4'b0;
       csr_sr_q[`SR_S]  <= 1'b1;
   end
   else
   begin

       csr_epc_q        <= csr_epc_r;
       csr_evec_q       <= csr_evec_r;
       csr_cause_q      <= csr_cause_r;
       csr_sr_q         <= csr_sr_r;
   end
end

// CSR Read Data MUX
always @ *
begin
    csr_data_r = 32'b0;

    case (csr_addr_o)
    `CSR_EPC:
        csr_data_r[31:0] = csr_epc_q;
    `CSR_EVEC:
        csr_data_r[31:0] = csr_evec_q;
    `CSR_CAUSE:
        csr_data_r[4:0]  = csr_cause_q;
    `CSR_STATUS:
        csr_data_r       = {csr_data_i[31:4], csr_sr_q};
    default:
        csr_data_r       = csr_data_i;
    endcase
end

assign csr_addr_o = imm12_r[11:0];
assign csr_data_o = (inst_csrrwi_w || inst_csrrsi_w || inst_csrrci_w) ? {27'b0, reg_rs1_i} : reg_rs1_r;
assign csr_set_o  = (execute_inst_r && !exception_w) ? (inst_csrrw_w || inst_csrrs_w || inst_csrrwi_w || inst_csrrsi_w): 1'b0;
assign csr_clr_o  = (execute_inst_r && !exception_w) ? (inst_csrrw_w || inst_csrrc_w || inst_csrrwi_w || inst_csrrci_w): 1'b0;

//-------------------------------------------------------------------
// Assignments
//-------------------------------------------------------------------

assign branch_pc_o          = pc_branch_q;
assign branch_o             = pc_fetch_q;
assign stall_o              = stall_exec_i | stall_mem_i;

assign opcode_o             = ex_opcode_q;
assign opcode_pc_o          = ex_opcode_pc_q;

assign reg_rd_o             = ex_rd_q;
assign reg_rd_value_o       = ex_result_w;

//-------------------------------------------------------------------
// Debug
//-------------------------------------------------------------------
`ifdef SIMULATION
reg [79:0] dbg_inst_str;
reg [79:0] dbg_inst_rs1;
reg [79:0] dbg_inst_rs2;
reg [79:0] dbg_inst_rd;
reg [31:0] dbg_inst_imm;

always @ *
begin
    dbg_inst_str = "-";
    dbg_inst_rs1 = "-";
    dbg_inst_rs2 = "-";
    dbg_inst_rd  = "-";
    dbg_inst_imm = 32'b0;

    if (opcode_valid_i)
    begin
        dbg_inst_rs1 = get_regname_str(reg_rs1_i);
        dbg_inst_rs2 = get_regname_str(reg_rs2_i);
        dbg_inst_rd  = get_regname_str(reg_rd_i);
        dbg_inst_imm = 32'b0;

        case (1'b1)
            inst_andi_w:    dbg_inst_str = "andi";
            inst_addi_w:    dbg_inst_str = "addi";
            inst_slti_w:    dbg_inst_str = "slti";
            inst_sltiu_w:   dbg_inst_str = "sltiu";
            inst_ori_w:     dbg_inst_str = "ori";
            inst_xori_w:    dbg_inst_str = "xori";
            inst_slli_w:    dbg_inst_str = "slli";
            inst_srli_w:    dbg_inst_str = "srli";
            inst_srai_w:    dbg_inst_str = "srai";
            inst_lui_w:     dbg_inst_str = "lui";
            inst_auipc_w:   dbg_inst_str = "auipc";
            inst_add_w:     dbg_inst_str = "add";
            inst_sub_w:     dbg_inst_str = "sub";
            inst_slt_w:     dbg_inst_str = "slt";
            inst_sltu_w:    dbg_inst_str = "sltu";
            inst_xor_w:     dbg_inst_str = "xor";
            inst_or_w:      dbg_inst_str = "or";
            inst_and_w:     dbg_inst_str = "and";
            inst_sll_w:     dbg_inst_str = "sll";
            inst_srl_w:     dbg_inst_str = "srl";
            inst_sra_w:     dbg_inst_str = "sra";
            inst_jal_w:     dbg_inst_str = "jal";
            inst_jalr_w:    dbg_inst_str = "jalr";
            inst_beq_w:     dbg_inst_str = "beq";
            inst_bne_w:     dbg_inst_str = "bne";
            inst_blt_w:     dbg_inst_str = "blt";
            inst_bge_w:     dbg_inst_str = "bge";
            inst_bltu_w:    dbg_inst_str = "bltu";
            inst_bgeu_w:    dbg_inst_str = "bgeu";
            inst_lb_w:      dbg_inst_str = "lb";
            inst_lh_w:      dbg_inst_str = "lh";
            inst_lw_w:      dbg_inst_str = "lw";
            inst_lbu_w:     dbg_inst_str = "lbu";
            inst_lhu_w:     dbg_inst_str = "lhu";
            inst_lwu_w:     dbg_inst_str = "lwu";
            inst_sb_w:      dbg_inst_str = "sb";
            inst_sh_w:      dbg_inst_str = "sh";
            inst_sw_w:      dbg_inst_str = "sw";
            inst_scall_w:   dbg_inst_str = "scall";
            inst_sbreak_w:  dbg_inst_str = "sbreak";
            inst_sret_w:    dbg_inst_str = "sret";
            inst_csrrw_w:   dbg_inst_str = "csrw";
            inst_csrrs_w:   dbg_inst_str = "csrs";
            inst_csrrc_w:   dbg_inst_str = "csrc";
            inst_csrrwi_w:   dbg_inst_str = "csrwi";
            inst_csrrsi_w:   dbg_inst_str = "csrsi";
            inst_csrrci_w:   dbg_inst_str = "csrci";            
        endcase

        case (1'b1)

            inst_addi_w,  // addi
            inst_andi_w,  // andi
            inst_slti_w,  // slti
            inst_sltiu_w, // sltiu
            inst_ori_w,   // ori
            inst_xori_w,  // xori
            inst_csrrw_w, // csrrw
            inst_csrrs_w, // csrrs
            inst_csrrc_w, // csrrc
            inst_csrrwi_w,// csrrwi
            inst_csrrsi_w,// csrrsi
            inst_csrrci_w:// csrrci            
            begin
                dbg_inst_rs2 = "-";
                dbg_inst_imm = imm12_r;
            end

            inst_slli_w, // slli
            inst_srli_w, // srli
            inst_srai_w: // srai
            begin
                dbg_inst_rs2 = "-";
                dbg_inst_imm = {27'b0, shamt_r};
            end

            inst_lui_w: // lui
            begin
                dbg_inst_rs1 = "-";
                dbg_inst_rs2 = "-";
                dbg_inst_imm = imm20_r;
            end
 
            inst_auipc_w: // auipc
            begin
                dbg_inst_rs1 = "pc";
                dbg_inst_rs2 = "-";
                dbg_inst_imm = imm20_r;
            end   

            inst_jal_w:  // jal
            begin
                dbg_inst_rs1 = "-";
                dbg_inst_rs2 = "-";
                dbg_inst_imm = opcode_pc_i + jimm20_r;
            end

            inst_jalr_w: // jalr
            begin
                dbg_inst_rs2 = "-";
                dbg_inst_imm = reg_rs1_r + imm12_r;
            end

            // lb lh lw lbu lhu lwu
            inst_lb_w,
            inst_lh_w,
            inst_lw_w,
            inst_lbu_w,
            inst_lhu_w,
            inst_lwu_w:
            begin
                dbg_inst_rs2 = "-";
                dbg_inst_imm = reg_rs1_r + imm12_r;
            end 

            // sb sh sw
            inst_sb_w,
            inst_sh_w,
            inst_sw_w:
            begin
                dbg_inst_rd = "-";
                dbg_inst_imm = reg_rs1_r + {{20{opcode_i[31]}}, opcode_i[31:25], opcode_i[11:7]};
            end
        endcase
    end
end
`endif

//-------------------------------------------------------------------
// Hooks for debug
//-------------------------------------------------------------------
`ifdef verilator
   function [31:0] get_opcode_ex;
      // verilator public
      get_opcode_ex = stall_mem_i ? 32'b0 : ex_opcode_q;
   endfunction
   function [31:0] get_pc_ex;
      // verilator public
      get_pc_ex = ex_opcode_pc_q;
   endfunction   
   function [7:0] get_putc;
      // verilator public
      get_putc = 8'b0;
   endfunction
   function [0:0] get_reg_valid;
      // verilator public
      get_reg_valid = opcode_valid_i & ~stall_o;
   endfunction
   function [4:0] get_reg_ra;
      // verilator public
      get_reg_ra = reg_rs1_i;
   endfunction
   function [31:0] get_reg_ra_value;
      // verilator public
      get_reg_ra_value = reg_rs1_r;
   endfunction
   function [4:0] get_reg_rb;
      // verilator public
      get_reg_rb = reg_rs2_i;
   endfunction   
   function [31:0] get_reg_rb_value;
      // verilator public
      get_reg_rb_value = reg_rs2_r;
   endfunction
`endif

endmodule
