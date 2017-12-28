//-----------------------------------------------------------------
//                         RISC-V Core
//                            V0.5
//                     Ultra-Embedded.com
//                     Copyright 2014-2017
//
//                   admin@ultra-embedded.com
//
//                       License: BSD
//-----------------------------------------------------------------
//
// Copyright (c) 2014, Ultra-Embedded.com
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions 
// are met:
//   - Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   - Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer 
//     in the documentation and/or other materials provided with the 
//     distribution.
//   - Neither the name of the author nor the names of its contributors 
//     may be used to endorse or promote products derived from this 
//     software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
// THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
// SUCH DAMAGE.
//-----------------------------------------------------------------
module riscv_decode
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input           fetch_valid_i
    ,input  [ 31:0]  fetch_instr_i
    ,input  [ 31:0]  fetch_pc_i
    ,input           branch_request_i
    ,input  [ 31:0]  branch_pc_i
    ,input  [  4:0]  writeback_exec_idx_i
    ,input  [ 31:0]  writeback_exec_value_i
    ,input  [  4:0]  writeback_mem_idx_i
    ,input  [ 31:0]  writeback_mem_value_i
    ,input  [  4:0]  writeback_csr_idx_i
    ,input  [ 31:0]  writeback_csr_value_i
    ,input           exec_stall_i
    ,input           lsu_stall_i
    ,input           csr_stall_i
    ,input           take_interrupt_i

    // Outputs
    ,output          fetch_branch_o
    ,output [ 31:0]  fetch_branch_pc_o
    ,output          fetch_accept_o
    ,output          exec_opcode_valid_o
    ,output          lsu_opcode_valid_o
    ,output          csr_opcode_valid_o
    ,output [ 55:0]  opcode_instr_o
    ,output [ 31:0]  opcode_opcode_o
    ,output [ 31:0]  opcode_pc_o
    ,output [  4:0]  opcode_rd_idx_o
    ,output [  4:0]  opcode_ra_idx_o
    ,output [  4:0]  opcode_rb_idx_o
    ,output [ 31:0]  opcode_ra_operand_o
    ,output [ 31:0]  opcode_rb_operand_o
);

`include "riscv_defs.v"

//-------------------------------------------------------------
// Registers / Wires
//-------------------------------------------------------------
reg         valid_q;
reg [31:0]  pc_q;
reg [31:0]  inst_q;

reg [31:0]  scoreboard_q;
reg         stall_scoreboard_r;
reg         intr_taken_q;

wire [31:0] ra_value_w;
wire [31:0] rb_value_w;

wire        stall_input_w = stall_scoreboard_r || exec_stall_i || lsu_stall_i || csr_stall_i;

//-------------------------------------------------------------
// Instances
//-------------------------------------------------------------
riscv_regfile
u_regfile
(
    .clk_i(clk_i),
    .rst_i(rst_i),
    .rd0_i(writeback_exec_idx_i),
    .rd0_value_i(writeback_exec_value_i),
    .rd1_i(writeback_mem_idx_i),
    .rd1_value_i(writeback_mem_value_i),
    .rd2_i(writeback_csr_idx_i),
    .rd2_value_i(writeback_csr_value_i),
    .ra_i(opcode_ra_idx_o),
    .rb_i(opcode_rb_idx_o),
    .ra_value_o(ra_value_w),
    .rb_value_o(rb_value_w)
);

//-------------------------------------------------------------
// Instruction Register
//-------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    valid_q       <= 1'b0;
    pc_q          <= 32'b0;
    inst_q        <= 32'b0;
    intr_taken_q  <= 1'b0;
end
else
begin
    // Take interrupt
    if (!stall_input_w && fetch_valid_i && take_interrupt_i && !intr_taken_q)
    begin
        if (branch_request_i)
            pc_q <= branch_pc_i;
        else
            pc_q <= fetch_pc_i;

        valid_q      <= 1'b1;
        inst_q       <= 32'b0;
        intr_taken_q <= 1'b1;
    end
    // Normal operation
    else if (!stall_input_w || branch_request_i)
    begin
        valid_q      <= fetch_valid_i && !branch_request_i;
        pc_q         <= fetch_pc_i;
        inst_q       <= fetch_instr_i;
        intr_taken_q <= 1'b0;
    end
end
    
//-------------------------------------------------------------
// Scoreboard Register
//-------------------------------------------------------------
reg [31:0] scoreboard_r;

wire sb_alloc_w = (opcode_instr_o[`ENUM_INST_LB]     ||
                   opcode_instr_o[`ENUM_INST_LH]     ||
                   opcode_instr_o[`ENUM_INST_LW]     ||
                   opcode_instr_o[`ENUM_INST_LBU]    ||
                   opcode_instr_o[`ENUM_INST_LHU]    ||
                   opcode_instr_o[`ENUM_INST_LWU]    ||
                   opcode_instr_o[`ENUM_INST_CSRRW]  ||
                   opcode_instr_o[`ENUM_INST_CSRRS]  ||
                   opcode_instr_o[`ENUM_INST_CSRRC]  ||
                   opcode_instr_o[`ENUM_INST_CSRRWI] ||
                   opcode_instr_o[`ENUM_INST_CSRRSI] ||
                   opcode_instr_o[`ENUM_INST_CSRRCI] );

always @ *
begin
    scoreboard_r = scoreboard_q;

    // Allocate register in scoreboard
    if (sb_alloc_w && exec_opcode_valid_o && lsu_opcode_valid_o && csr_opcode_valid_o)
    begin
        scoreboard_r[opcode_rd_idx_o] = 1'b1;
    end

    // Release register on Load / CSR completion
    scoreboard_r[writeback_mem_idx_i] = 1'b0;
    scoreboard_r[writeback_csr_idx_i] = 1'b0;
end

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    scoreboard_q <= 32'b0;
else
    scoreboard_q <= {scoreboard_r[31:1], 1'b0};

//-------------------------------------------------------------
// Instruction Decode
//-------------------------------------------------------------
assign opcode_instr_o[`ENUM_INST_ANDI]   = ((inst_q & `INST_ANDI_MASK) == `INST_ANDI);   // andi
assign opcode_instr_o[`ENUM_INST_ADDI]   = ((inst_q & `INST_ADDI_MASK) == `INST_ADDI);   // addi
assign opcode_instr_o[`ENUM_INST_SLTI]   = ((inst_q & `INST_SLTI_MASK) == `INST_SLTI);   // slti
assign opcode_instr_o[`ENUM_INST_SLTIU]  = ((inst_q & `INST_SLTIU_MASK) == `INST_SLTIU); // sltiu
assign opcode_instr_o[`ENUM_INST_ORI]    = ((inst_q & `INST_ORI_MASK) == `INST_ORI);     // ori
assign opcode_instr_o[`ENUM_INST_XORI]   = ((inst_q & `INST_XORI_MASK) == `INST_XORI);   // xori
assign opcode_instr_o[`ENUM_INST_SLLI]   = ((inst_q & `INST_SLLI_MASK) == `INST_SLLI);   // slli
assign opcode_instr_o[`ENUM_INST_SRLI]   = ((inst_q & `INST_SRLI_MASK) == `INST_SRLI);   // srli
assign opcode_instr_o[`ENUM_INST_SRAI]   = ((inst_q & `INST_SRAI_MASK) == `INST_SRAI);   // srai
assign opcode_instr_o[`ENUM_INST_LUI]    = ((inst_q & `INST_LUI_MASK) == `INST_LUI);     // lui
assign opcode_instr_o[`ENUM_INST_AUIPC]  = ((inst_q & `INST_AUIPC_MASK) == `INST_AUIPC); // auipc
assign opcode_instr_o[`ENUM_INST_ADD]    = ((inst_q & `INST_ADD_MASK) == `INST_ADD);     // add
assign opcode_instr_o[`ENUM_INST_SUB]    = ((inst_q & `INST_SUB_MASK) == `INST_SUB);     // sub
assign opcode_instr_o[`ENUM_INST_SLT]    = ((inst_q & `INST_SLT_MASK) == `INST_SLT);     // slt
assign opcode_instr_o[`ENUM_INST_SLTU]   = ((inst_q & `INST_SLTU_MASK) == `INST_SLTU);   // sltu
assign opcode_instr_o[`ENUM_INST_XOR]    = ((inst_q & `INST_XOR_MASK) == `INST_XOR);     // xor
assign opcode_instr_o[`ENUM_INST_OR]     = ((inst_q & `INST_OR_MASK) == `INST_OR);       // or
assign opcode_instr_o[`ENUM_INST_AND]    = ((inst_q & `INST_AND_MASK) == `INST_AND);     // and
assign opcode_instr_o[`ENUM_INST_SLL]    = ((inst_q & `INST_SLL_MASK) == `INST_SLL);     // sll
assign opcode_instr_o[`ENUM_INST_SRL]    = ((inst_q & `INST_SRL_MASK) == `INST_SRL);     // srl
assign opcode_instr_o[`ENUM_INST_SRA]    = ((inst_q & `INST_SRA_MASK) == `INST_SRA);     // sra
assign opcode_instr_o[`ENUM_INST_JAL]    = ((inst_q & `INST_JAL_MASK) == `INST_JAL);     // jal
assign opcode_instr_o[`ENUM_INST_JALR]   = ((inst_q & `INST_JALR_MASK) == `INST_JALR);   // jalr
assign opcode_instr_o[`ENUM_INST_BEQ]    = ((inst_q & `INST_BEQ_MASK) == `INST_BEQ);     // beq
assign opcode_instr_o[`ENUM_INST_BNE]    = ((inst_q & `INST_BNE_MASK) == `INST_BNE);     // bne
assign opcode_instr_o[`ENUM_INST_BLT]    = ((inst_q & `INST_BLT_MASK) == `INST_BLT);     // blt
assign opcode_instr_o[`ENUM_INST_BGE]    = ((inst_q & `INST_BGE_MASK) == `INST_BGE);     // bge
assign opcode_instr_o[`ENUM_INST_BLTU]   = ((inst_q & `INST_BLTU_MASK) == `INST_BLTU);   // bltu
assign opcode_instr_o[`ENUM_INST_BGEU]   = ((inst_q & `INST_BGEU_MASK) == `INST_BGEU);   // bgeu
assign opcode_instr_o[`ENUM_INST_LB]     = ((inst_q & `INST_LB_MASK) == `INST_LB);       // lb
assign opcode_instr_o[`ENUM_INST_LH]     = ((inst_q & `INST_LH_MASK) == `INST_LH);       // lh
assign opcode_instr_o[`ENUM_INST_LW]     = ((inst_q & `INST_LW_MASK) == `INST_LW);       // lw
assign opcode_instr_o[`ENUM_INST_LBU]    = ((inst_q & `INST_LBU_MASK) == `INST_LBU);     // lbu
assign opcode_instr_o[`ENUM_INST_LHU]    = ((inst_q & `INST_LHU_MASK) == `INST_LHU);     // lhu
assign opcode_instr_o[`ENUM_INST_LWU]    = ((inst_q & `INST_LWU_MASK) == `INST_LWU);     // lwu
assign opcode_instr_o[`ENUM_INST_SB]     = ((inst_q & `INST_SB_MASK) == `INST_SB);       // sb
assign opcode_instr_o[`ENUM_INST_SH]     = ((inst_q & `INST_SH_MASK) == `INST_SH);       // sh
assign opcode_instr_o[`ENUM_INST_SW]     = ((inst_q & `INST_SW_MASK) == `INST_SW);       // sw
assign opcode_instr_o[`ENUM_INST_ECALL]  = ((inst_q & `INST_ECALL_MASK) == `INST_ECALL); // ecall
assign opcode_instr_o[`ENUM_INST_EBREAK] = ((inst_q & `INST_EBREAK_MASK) == `INST_EBREAK); // ebreak
assign opcode_instr_o[`ENUM_INST_MRET]   = ((inst_q & `INST_MRET_MASK) == `INST_MRET);   // mret
assign opcode_instr_o[`ENUM_INST_CSRRW]  = ((inst_q & `INST_CSRRW_MASK) == `INST_CSRRW); // csrrw
assign opcode_instr_o[`ENUM_INST_CSRRS]  = ((inst_q & `INST_CSRRS_MASK) == `INST_CSRRS); // csrrs
assign opcode_instr_o[`ENUM_INST_CSRRC]  = ((inst_q & `INST_CSRRC_MASK) == `INST_CSRRC); // csrrc
assign opcode_instr_o[`ENUM_INST_CSRRWI] = ((inst_q & `INST_CSRRWI_MASK) == `INST_CSRRWI); // csrrwi
assign opcode_instr_o[`ENUM_INST_CSRRSI] = ((inst_q & `INST_CSRRSI_MASK) == `INST_CSRRSI); // csrrsi
assign opcode_instr_o[`ENUM_INST_CSRRCI] = ((inst_q & `INST_CSRRCI_MASK) == `INST_CSRRCI); // csrrci
assign opcode_instr_o[`ENUM_INST_INTR]   = intr_taken_q;

// Decode operands
assign opcode_pc_o     = pc_q;
assign opcode_opcode_o = inst_q;
assign opcode_ra_idx_o = inst_q[19:15];
assign opcode_rb_idx_o = inst_q[24:20];
assign opcode_rd_idx_o = inst_q[11:7];

//-------------------------------------------------------------
// Bypass / Forwarding
//-------------------------------------------------------------
reg [31:0] opcode_ra_operand_r;
reg [31:0] opcode_rb_operand_r;

always @ *
begin
    // Bypass: Exec
    if (writeback_exec_idx_i != 5'd0 && writeback_exec_idx_i == opcode_ra_idx_o)
        opcode_ra_operand_r = writeback_exec_value_i;
    else
        opcode_ra_operand_r = ra_value_w;

    // Bypass: Exec
    if (writeback_exec_idx_i != 5'd0 && writeback_exec_idx_i == opcode_rb_idx_o)
        opcode_rb_operand_r = writeback_exec_value_i;
    else
        opcode_rb_operand_r = rb_value_w;
end

assign opcode_ra_operand_o = opcode_ra_operand_r;
assign opcode_rb_operand_o = opcode_rb_operand_r;

//-------------------------------------------------------------
// Stall logic
//-------------------------------------------------------------
reg opcode_valid;
always @ *
begin
    opcode_valid       = valid_q;
    stall_scoreboard_r = 1'b0;

    // Detect dependancy on the LSU/CSR scoreboard
    if (scoreboard_q[opcode_ra_idx_o] || 
        scoreboard_q[opcode_rb_idx_o] ||
        scoreboard_q[opcode_rd_idx_o])
    begin
        stall_scoreboard_r = 1'b1;
        opcode_valid       = 1'b0;
    end
end

// Opcode valid flags to the various execution units
assign exec_opcode_valid_o = opcode_valid && !lsu_stall_i  && !csr_stall_i;
assign lsu_opcode_valid_o  = opcode_valid && !exec_stall_i && !csr_stall_i;
assign csr_opcode_valid_o  = opcode_valid && !exec_stall_i && !lsu_stall_i;

//-------------------------------------------------------------
// Fetch output
//-------------------------------------------------------------
assign fetch_branch_o    = branch_request_i;
assign fetch_branch_pc_o = branch_pc_i;
assign fetch_accept_o    = !exec_stall_i && !stall_scoreboard_r && !lsu_stall_i && !csr_stall_i;

//-------------------------------------------------------------
// get_reg_valid: Register contents valid
//-------------------------------------------------------------
`ifdef verilator
function [0:0] get_reg_valid; /*verilator public*/
    input [4:0] r;
begin
    get_reg_valid = !scoreboard_q[r];
end
endfunction
//-------------------------------------------------------------
// get_register: Read register file
//-------------------------------------------------------------
function [31:0] get_register; /*verilator public*/
    input [4:0] r;
begin
    get_register = u_regfile.get_register(r);

    if (r != 5'b0 && writeback_exec_idx_i == r)
        get_register = writeback_exec_value_i;

    if (r != 5'b0 && writeback_mem_idx_i == r)
        get_register = writeback_mem_value_i;
end
endfunction
//-------------------------------------------------------------
// set_register: Write register file
//-------------------------------------------------------------
function set_register; /*verilator public*/
    input [4:0] r;
    input [31:0] value;
begin
    u_regfile.set_register(r,value);
end
endfunction
`endif

endmodule
