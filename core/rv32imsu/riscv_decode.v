//-----------------------------------------------------------------
//                         RISC-V Core
//                            V0.9.7
//                     Ultra-Embedded.com
//                     Copyright 2014-2019
//
//                   admin@ultra-embedded.com
//
//                       License: BSD
//-----------------------------------------------------------------
//
// Copyright (c) 2014-2019, Ultra-Embedded.com
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
    ,input           branch_csr_request_i
    ,input  [ 31:0]  branch_csr_pc_i
    ,input  [  4:0]  writeback_exec_idx_i
    ,input           writeback_exec_squash_i
    ,input  [ 31:0]  writeback_exec_value_i
    ,input  [  4:0]  writeback_mem_idx_i
    ,input           writeback_mem_squash_i
    ,input  [ 31:0]  writeback_mem_value_i
    ,input  [  4:0]  writeback_csr_idx_i
    ,input           writeback_csr_squash_i
    ,input  [ 31:0]  writeback_csr_value_i
    ,input  [  4:0]  writeback_muldiv_idx_i
    ,input           writeback_muldiv_squash_i
    ,input  [ 31:0]  writeback_muldiv_value_i
    ,input           exec_stall_i
    ,input           lsu_stall_i
    ,input           csr_stall_i
    ,input           muldiv_stall_i

    // Outputs
    ,output          fetch_branch_o
    ,output [ 31:0]  fetch_branch_pc_o
    ,output          fetch_accept_o
    ,output          exec_opcode_valid_o
    ,output          lsu_opcode_valid_o
    ,output          csr_opcode_valid_o
    ,output          muldiv_opcode_valid_o
    ,output [ 57:0]  opcode_instr_o
    ,output [ 31:0]  opcode_opcode_o
    ,output [ 31:0]  opcode_pc_o
    ,output [  4:0]  opcode_rd_idx_o
    ,output [  4:0]  opcode_ra_idx_o
    ,output [  4:0]  opcode_rb_idx_o
    ,output [ 31:0]  opcode_ra_operand_o
    ,output [ 31:0]  opcode_rb_operand_o
    ,output          fetch_invalidate_o
);



//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-------------------------------------------------------------
// Registers / Wires
//-------------------------------------------------------------
reg         valid_q;
reg [31:0]  pc_q;
reg [31:0]  inst_q;
reg         fault_fetch_q;

reg [31:0]  scoreboard_q;
reg         stall_scoreboard_r;

wire [31:0] ra_value_w;
wire [31:0] rb_value_w;

wire        stall_input_w = stall_scoreboard_r || exec_stall_i || lsu_stall_i || csr_stall_i || muldiv_stall_i;

//-------------------------------------------------------------
// Instances
//-------------------------------------------------------------
wire [4:0] wb_exec_rd_w   = writeback_exec_idx_i   & {5{~writeback_exec_squash_i}};
wire [4:0] wb_mem_rd_w    = writeback_mem_idx_i    & {5{~writeback_mem_squash_i}};
wire [4:0] wb_csr_rd_w    = writeback_csr_idx_i    & {5{~writeback_csr_squash_i}};
wire [4:0] wb_muldiv_rd_w = writeback_muldiv_idx_i & {5{~writeback_muldiv_squash_i}};


reg [4:0]  wb_rd_r;
reg [31:0] wb_res_r;

always @ *
begin
    wb_rd_r  = wb_exec_rd_w;
    wb_res_r = writeback_exec_value_i;

end

riscv_regfile
u_regfile
(
    .clk_i(clk_i),
    .rst_i(rst_i),

    // Write ports
    .rd0_i(wb_rd_r),
    .rd0_value_i(wb_res_r),
    .rd1_i(wb_mem_rd_w),
    .rd1_value_i(writeback_mem_value_i),
    .rd2_i(wb_csr_rd_w),
    .rd2_value_i(writeback_csr_value_i),
    .rd3_i(wb_muldiv_rd_w),
    .rd3_value_i(writeback_muldiv_value_i),

    // Read ports
    .ra_i(opcode_ra_idx_o),
    .rb_i(opcode_rb_idx_o),
    .ra_value_o(ra_value_w),
    .rb_value_o(rb_value_w)
);

//-------------------------------------------------------------
// fence.i
//-------------------------------------------------------------
reg ifence_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    ifence_q <= 1'b0;
else 
    ifence_q <= fetch_valid_i && fetch_accept_o && 
                ((fetch_instr_i & `INST_IFENCE_MASK) == `INST_IFENCE);

assign fetch_invalidate_o = ifence_q;

//-------------------------------------------------------------
// Instruction Register
//-------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    valid_q       <= 1'b0;
    pc_q          <= 32'b0;
    inst_q        <= 32'b0;
end
// Branch request
else if (branch_request_i || branch_csr_request_i)
begin
    valid_q       <= 1'b0;

    if (branch_csr_request_i)
        pc_q      <= branch_csr_pc_i;
    else /*if (branch_request_i)*/
        pc_q      <= branch_pc_i;

    inst_q        <= 32'b0;
end
// Normal operation - decode not stalled
else if (!stall_input_w)
begin
    valid_q       <= fetch_valid_i;

    if (fetch_valid_i)
        pc_q      <= fetch_pc_i;
    // Current instruction accepted, increment PC to unexecuted instruction
    else if (valid_q)
        pc_q      <= pc_q + 32'd4;

    inst_q        <= fetch_instr_i;
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
                   opcode_instr_o[`ENUM_INST_CSRRCI] ||
                   opcode_instr_o[`ENUM_INST_MUL]    || 
                   opcode_instr_o[`ENUM_INST_MULH]   ||
                   opcode_instr_o[`ENUM_INST_MULHSU] ||
                   opcode_instr_o[`ENUM_INST_MULHU]  ||
                   opcode_instr_o[`ENUM_INST_DIV]    ||
                   opcode_instr_o[`ENUM_INST_DIVU]   ||
                   opcode_instr_o[`ENUM_INST_REM]    ||
                   opcode_instr_o[`ENUM_INST_REMU] );


always @ *
begin
    scoreboard_r = scoreboard_q;

    scoreboard_r[writeback_mem_idx_i]    = 1'b0;

    // Allocate register in scoreboard
    if (sb_alloc_w && exec_opcode_valid_o && lsu_opcode_valid_o && csr_opcode_valid_o && muldiv_opcode_valid_o)
    begin
        scoreboard_r[opcode_rd_idx_o] = 1'b1;
    end

    // Release register on Load / CSR completion    
    scoreboard_r[writeback_csr_idx_i]    = 1'b0;
    scoreboard_r[writeback_muldiv_idx_i] = 1'b0;
end

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    scoreboard_q <= 32'b0;
else
    scoreboard_q <= {scoreboard_r[31:1], 1'b0};

//-------------------------------------------------------------
// Instruction Decode
//-------------------------------------------------------------
wire [`ENUM_INST_MAX-1:0] opcode_instr_w;

assign opcode_instr_w[`ENUM_INST_ANDI]   = ((fetch_instr_i & `INST_ANDI_MASK) == `INST_ANDI);   // andi
assign opcode_instr_w[`ENUM_INST_ADDI]   = ((fetch_instr_i & `INST_ADDI_MASK) == `INST_ADDI);   // addi
assign opcode_instr_w[`ENUM_INST_SLTI]   = ((fetch_instr_i & `INST_SLTI_MASK) == `INST_SLTI);   // slti
assign opcode_instr_w[`ENUM_INST_SLTIU]  = ((fetch_instr_i & `INST_SLTIU_MASK) == `INST_SLTIU); // sltiu
assign opcode_instr_w[`ENUM_INST_ORI]    = ((fetch_instr_i & `INST_ORI_MASK) == `INST_ORI);     // ori
assign opcode_instr_w[`ENUM_INST_XORI]   = ((fetch_instr_i & `INST_XORI_MASK) == `INST_XORI);   // xori
assign opcode_instr_w[`ENUM_INST_SLLI]   = ((fetch_instr_i & `INST_SLLI_MASK) == `INST_SLLI);   // slli
assign opcode_instr_w[`ENUM_INST_SRLI]   = ((fetch_instr_i & `INST_SRLI_MASK) == `INST_SRLI);   // srli
assign opcode_instr_w[`ENUM_INST_SRAI]   = ((fetch_instr_i & `INST_SRAI_MASK) == `INST_SRAI);   // srai
assign opcode_instr_w[`ENUM_INST_LUI]    = ((fetch_instr_i & `INST_LUI_MASK) == `INST_LUI);     // lui
assign opcode_instr_w[`ENUM_INST_AUIPC]  = ((fetch_instr_i & `INST_AUIPC_MASK) == `INST_AUIPC); // auipc
assign opcode_instr_w[`ENUM_INST_ADD]    = ((fetch_instr_i & `INST_ADD_MASK) == `INST_ADD);     // add
assign opcode_instr_w[`ENUM_INST_SUB]    = ((fetch_instr_i & `INST_SUB_MASK) == `INST_SUB);     // sub
assign opcode_instr_w[`ENUM_INST_SLT]    = ((fetch_instr_i & `INST_SLT_MASK) == `INST_SLT);     // slt
assign opcode_instr_w[`ENUM_INST_SLTU]   = ((fetch_instr_i & `INST_SLTU_MASK) == `INST_SLTU);   // sltu
assign opcode_instr_w[`ENUM_INST_XOR]    = ((fetch_instr_i & `INST_XOR_MASK) == `INST_XOR);     // xor
assign opcode_instr_w[`ENUM_INST_OR]     = ((fetch_instr_i & `INST_OR_MASK) == `INST_OR);       // or
assign opcode_instr_w[`ENUM_INST_AND]    = ((fetch_instr_i & `INST_AND_MASK) == `INST_AND);     // and
assign opcode_instr_w[`ENUM_INST_SLL]    = ((fetch_instr_i & `INST_SLL_MASK) == `INST_SLL);     // sll
assign opcode_instr_w[`ENUM_INST_SRL]    = ((fetch_instr_i & `INST_SRL_MASK) == `INST_SRL);     // srl
assign opcode_instr_w[`ENUM_INST_SRA]    = ((fetch_instr_i & `INST_SRA_MASK) == `INST_SRA);     // sra
assign opcode_instr_w[`ENUM_INST_JAL]    = ((fetch_instr_i & `INST_JAL_MASK) == `INST_JAL);     // jal
assign opcode_instr_w[`ENUM_INST_JALR]   = ((fetch_instr_i & `INST_JALR_MASK) == `INST_JALR);   // jalr
assign opcode_instr_w[`ENUM_INST_BEQ]    = ((fetch_instr_i & `INST_BEQ_MASK) == `INST_BEQ);     // beq
assign opcode_instr_w[`ENUM_INST_BNE]    = ((fetch_instr_i & `INST_BNE_MASK) == `INST_BNE);     // bne
assign opcode_instr_w[`ENUM_INST_BLT]    = ((fetch_instr_i & `INST_BLT_MASK) == `INST_BLT);     // blt
assign opcode_instr_w[`ENUM_INST_BGE]    = ((fetch_instr_i & `INST_BGE_MASK) == `INST_BGE);     // bge
assign opcode_instr_w[`ENUM_INST_BLTU]   = ((fetch_instr_i & `INST_BLTU_MASK) == `INST_BLTU);   // bltu
assign opcode_instr_w[`ENUM_INST_BGEU]   = ((fetch_instr_i & `INST_BGEU_MASK) == `INST_BGEU);   // bgeu
assign opcode_instr_w[`ENUM_INST_LB]     = ((fetch_instr_i & `INST_LB_MASK) == `INST_LB);       // lb
assign opcode_instr_w[`ENUM_INST_LH]     = ((fetch_instr_i & `INST_LH_MASK) == `INST_LH);       // lh
assign opcode_instr_w[`ENUM_INST_LW]     = ((fetch_instr_i & `INST_LW_MASK) == `INST_LW);       // lw
assign opcode_instr_w[`ENUM_INST_LBU]    = ((fetch_instr_i & `INST_LBU_MASK) == `INST_LBU);     // lbu
assign opcode_instr_w[`ENUM_INST_LHU]    = ((fetch_instr_i & `INST_LHU_MASK) == `INST_LHU);     // lhu
assign opcode_instr_w[`ENUM_INST_LWU]    = ((fetch_instr_i & `INST_LWU_MASK) == `INST_LWU);     // lwu
assign opcode_instr_w[`ENUM_INST_SB]     = ((fetch_instr_i & `INST_SB_MASK) == `INST_SB);       // sb
assign opcode_instr_w[`ENUM_INST_SH]     = ((fetch_instr_i & `INST_SH_MASK) == `INST_SH);       // sh
assign opcode_instr_w[`ENUM_INST_SW]     = ((fetch_instr_i & `INST_SW_MASK) == `INST_SW);       // sw
assign opcode_instr_w[`ENUM_INST_ECALL]  = ((fetch_instr_i & `INST_ECALL_MASK) == `INST_ECALL); // ecall
assign opcode_instr_w[`ENUM_INST_EBREAK] = ((fetch_instr_i & `INST_EBREAK_MASK) == `INST_EBREAK); // ebreak
assign opcode_instr_w[`ENUM_INST_ERET]   = ((fetch_instr_i & `INST_MRET_MASK) == `INST_MRET);   // mret / sret
assign opcode_instr_w[`ENUM_INST_CSRRW]  = ((fetch_instr_i & `INST_CSRRW_MASK) == `INST_CSRRW); // csrrw
assign opcode_instr_w[`ENUM_INST_CSRRS]  = ((fetch_instr_i & `INST_CSRRS_MASK) == `INST_CSRRS); // csrrs
assign opcode_instr_w[`ENUM_INST_CSRRC]  = ((fetch_instr_i & `INST_CSRRC_MASK) == `INST_CSRRC); // csrrc
assign opcode_instr_w[`ENUM_INST_CSRRWI] = ((fetch_instr_i & `INST_CSRRWI_MASK) == `INST_CSRRWI); // csrrwi
assign opcode_instr_w[`ENUM_INST_CSRRSI] = ((fetch_instr_i & `INST_CSRRSI_MASK) == `INST_CSRRSI); // csrrsi
assign opcode_instr_w[`ENUM_INST_CSRRCI] = ((fetch_instr_i & `INST_CSRRCI_MASK) == `INST_CSRRCI); // csrrci
assign opcode_instr_w[`ENUM_INST_MUL]    = ((fetch_instr_i & `INST_MUL_MASK) == `INST_MUL);       // mul
assign opcode_instr_w[`ENUM_INST_MULH]   = ((fetch_instr_i & `INST_MULH_MASK) == `INST_MULH);     // mulh
assign opcode_instr_w[`ENUM_INST_MULHSU] = ((fetch_instr_i & `INST_MULHSU_MASK) == `INST_MULHSU); // mulhsu
assign opcode_instr_w[`ENUM_INST_MULHU]  = ((fetch_instr_i & `INST_MULHU_MASK) == `INST_MULHU);   // mulhu
assign opcode_instr_w[`ENUM_INST_DIV]    = ((fetch_instr_i & `INST_DIV_MASK) == `INST_DIV);       // div
assign opcode_instr_w[`ENUM_INST_DIVU]   = ((fetch_instr_i & `INST_DIVU_MASK) == `INST_DIVU);     // divu
assign opcode_instr_w[`ENUM_INST_REM]    = ((fetch_instr_i & `INST_REM_MASK) == `INST_REM);       // rem
assign opcode_instr_w[`ENUM_INST_REMU]   = ((fetch_instr_i & `INST_REMU_MASK) == `INST_REMU);     // remu
assign opcode_instr_w[`ENUM_INST_FAULT]  = ((fetch_instr_i & `INST_FAULT_MASK) == `INST_FAULT);
assign opcode_instr_w[`ENUM_INST_PAGE_FAULT] = ((fetch_instr_i & `INST_PAGE_FAULT_MASK) == `INST_PAGE_FAULT);
assign opcode_instr_w[`ENUM_INST_INVALID]= 1'b0;

wire nop_instr_w                         = ((fetch_instr_i & `INST_WFI_MASK) == `INST_WFI) |
                                           ((fetch_instr_i & `INST_FENCE_MASK) == `INST_FENCE) |
                                           ((fetch_instr_i & `INST_SFENCE_MASK) == `INST_SFENCE) |
                                           ((fetch_instr_i & `INST_IFENCE_MASK) == `INST_IFENCE);

wire invalid_inst_w = ~(|opcode_instr_w[`ENUM_INST_PAGE_FAULT:0]) & ~nop_instr_w;

reg [`ENUM_INST_MAX-1:0] opcode_instr_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    opcode_instr_q <= `ENUM_INST_MAX'b0;
else if (branch_request_i || branch_csr_request_i)
    opcode_instr_q <= `ENUM_INST_MAX'b0;
else if (!stall_input_w)
    opcode_instr_q <= {invalid_inst_w, opcode_instr_w[`ENUM_INST_PAGE_FAULT:0]};

assign opcode_instr_o = opcode_instr_q;

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
    if (!writeback_exec_squash_i && writeback_exec_idx_i != 5'd0 && writeback_exec_idx_i == opcode_ra_idx_o)
        opcode_ra_operand_r = writeback_exec_value_i;
    // Bypass: Mem
    else if (!writeback_mem_squash_i && writeback_mem_idx_i != 5'd0 && writeback_mem_idx_i == opcode_ra_idx_o)
        opcode_ra_operand_r = writeback_mem_value_i;
    else
        opcode_ra_operand_r = ra_value_w;

    // Bypass: Exec
    if (!writeback_exec_squash_i && writeback_exec_idx_i != 5'd0 && writeback_exec_idx_i == opcode_rb_idx_o)
        opcode_rb_operand_r = writeback_exec_value_i;
    // Bypass: Mem
    else if (!writeback_mem_squash_i && writeback_mem_idx_i != 5'd0 && writeback_mem_idx_i == opcode_rb_idx_o)
        opcode_rb_operand_r = writeback_mem_value_i;
    else
        opcode_rb_operand_r = rb_value_w;
end

assign opcode_ra_operand_o = opcode_ra_operand_r;
assign opcode_rb_operand_o = opcode_rb_operand_r;

//-------------------------------------------------------------
// Stall logic
//-------------------------------------------------------------
reg        opcode_valid_r;
reg [31:0] current_scoreboard_r;

always @ *
begin
    opcode_valid_r       = valid_q & ~branch_csr_request_i;
    stall_scoreboard_r   = 1'b0;
    current_scoreboard_r = scoreboard_q;

    // Mem writeback bypass
    current_scoreboard_r[writeback_mem_idx_i] = 1'b0;

    // Detect dependancy on the LSU/CSR scoreboard
    if (current_scoreboard_r[opcode_ra_idx_o] || 
        current_scoreboard_r[opcode_rb_idx_o] ||
        current_scoreboard_r[opcode_rd_idx_o])
    begin
        stall_scoreboard_r = 1'b1;
        opcode_valid_r     = 1'b0;
    end

end

// Opcode valid flags to the various execution units
assign exec_opcode_valid_o    = opcode_valid_r && !lsu_stall_i  && !csr_stall_i && !muldiv_stall_i;
assign lsu_opcode_valid_o     = opcode_valid_r && !exec_stall_i && !csr_stall_i && !muldiv_stall_i;
assign csr_opcode_valid_o     = opcode_valid_r && !exec_stall_i && !lsu_stall_i && !muldiv_stall_i;
assign muldiv_opcode_valid_o  = opcode_valid_r && !exec_stall_i && !lsu_stall_i && !csr_stall_i;

//-------------------------------------------------------------
// Fetch output
//-------------------------------------------------------------
assign fetch_branch_o    = branch_request_i | branch_csr_request_i;
assign fetch_branch_pc_o = branch_csr_request_i ? branch_csr_pc_i : branch_pc_i;
assign fetch_accept_o    = branch_csr_request_i || (!exec_stall_i && !stall_scoreboard_r && !lsu_stall_i && !csr_stall_i && !muldiv_stall_i);


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


//-----------------------------------------------------------------
// get_regname_str: Convert register number to string
//-----------------------------------------------------------------
`ifdef verilator
function [79:0] get_regname_str;
    input  [4:0] regnum;
begin
    case (regnum)
        5'd0:  get_regname_str = "zero";
        5'd1:  get_regname_str = "ra";
        5'd2:  get_regname_str = "sp";
        5'd3:  get_regname_str = "gp";
        5'd4:  get_regname_str = "tp";
        5'd5:  get_regname_str = "t0";
        5'd6:  get_regname_str = "t1";
        5'd7:  get_regname_str = "t2";
        5'd8:  get_regname_str = "s0";
        5'd9:  get_regname_str = "s1";
        5'd10: get_regname_str = "a0";
        5'd11: get_regname_str = "a1";
        5'd12: get_regname_str = "a2";
        5'd13: get_regname_str = "a3";
        5'd14: get_regname_str = "a4";
        5'd15: get_regname_str = "a5";
        5'd16: get_regname_str = "a6";
        5'd17: get_regname_str = "a7";
        5'd18: get_regname_str = "s2";
        5'd19: get_regname_str = "s3";
        5'd20: get_regname_str = "s4";
        5'd21: get_regname_str = "s5";
        5'd22: get_regname_str = "s6";
        5'd23: get_regname_str = "s7";
        5'd24: get_regname_str = "s8";
        5'd25: get_regname_str = "s9";
        5'd26: get_regname_str = "s10";
        5'd27: get_regname_str = "s11";
        5'd28: get_regname_str = "t3";
        5'd29: get_regname_str = "t4";
        5'd30: get_regname_str = "t5";
        5'd31: get_regname_str = "t6";
    endcase
end
endfunction

//-------------------------------------------------------------------
// Debug strings
//-------------------------------------------------------------------
reg [79:0] dbg_inst_str;
reg [79:0] dbg_inst_ra;
reg [79:0] dbg_inst_rb;
reg [79:0] dbg_inst_rd;
reg [31:0] dbg_inst_imm;
reg [31:0] dbg_inst_pc;

`define DBG_IMM_IMM20     {opcode_opcode_o[31:12], 12'b0}
`define DBG_IMM_IMM12     {{20{opcode_opcode_o[31]}}, opcode_opcode_o[31:20]}
`define DBG_IMM_BIMM      {{19{opcode_opcode_o[31]}}, opcode_opcode_o[31], opcode_opcode_o[7], opcode_opcode_o[30:25], opcode_opcode_o[11:8], 1'b0}
`define DBG_IMM_JIMM20    {{12{opcode_opcode_o[31]}}, opcode_opcode_o[19:12], opcode_opcode_o[20], opcode_opcode_o[30:25], opcode_opcode_o[24:21], 1'b0}
`define DBG_IMM_STOREIMM  {{20{opcode_opcode_o[31]}}, opcode_opcode_o[31:25], opcode_opcode_o[11:7]}
`define DBG_IMM_SHAMT     opcode_opcode_o[24:20]

always @ *
begin
    dbg_inst_str = "-";
    dbg_inst_ra  = "-";
    dbg_inst_rb  = "-";
    dbg_inst_rd  = "-";
    dbg_inst_pc  = 32'bx;

    if (opcode_valid_r)
    begin
        dbg_inst_pc  = opcode_pc_o;
        dbg_inst_ra  = get_regname_str(opcode_ra_idx_o);
        dbg_inst_rb  = get_regname_str(opcode_rb_idx_o);
        dbg_inst_rd  = get_regname_str(opcode_rd_idx_o);

        case (1'b1)
            opcode_instr_o[`ENUM_INST_ANDI]   : dbg_inst_str = "andi";
            opcode_instr_o[`ENUM_INST_ADDI]   : dbg_inst_str = "addi";
            opcode_instr_o[`ENUM_INST_SLTI]   : dbg_inst_str = "slti";
            opcode_instr_o[`ENUM_INST_SLTIU]  : dbg_inst_str = "sltiu";
            opcode_instr_o[`ENUM_INST_ORI]    : dbg_inst_str = "ori";
            opcode_instr_o[`ENUM_INST_XORI]   : dbg_inst_str = "xori";
            opcode_instr_o[`ENUM_INST_SLLI]   : dbg_inst_str = "slli";
            opcode_instr_o[`ENUM_INST_SRLI]   : dbg_inst_str = "srli";
            opcode_instr_o[`ENUM_INST_SRAI]   : dbg_inst_str = "srai";
            opcode_instr_o[`ENUM_INST_LUI]    : dbg_inst_str = "lui";
            opcode_instr_o[`ENUM_INST_AUIPC]  : dbg_inst_str = "auipc";
            opcode_instr_o[`ENUM_INST_ADD]    : dbg_inst_str = "add";
            opcode_instr_o[`ENUM_INST_SUB]    : dbg_inst_str = "sub";
            opcode_instr_o[`ENUM_INST_SLT]    : dbg_inst_str = "slt";
            opcode_instr_o[`ENUM_INST_SLTU]   : dbg_inst_str = "sltu";
            opcode_instr_o[`ENUM_INST_XOR]    : dbg_inst_str = "xor";
            opcode_instr_o[`ENUM_INST_OR]     : dbg_inst_str = "or";
            opcode_instr_o[`ENUM_INST_AND]    : dbg_inst_str = "and";
            opcode_instr_o[`ENUM_INST_SLL]    : dbg_inst_str = "sll";
            opcode_instr_o[`ENUM_INST_SRL]    : dbg_inst_str = "srl";
            opcode_instr_o[`ENUM_INST_SRA]    : dbg_inst_str = "sra";
            opcode_instr_o[`ENUM_INST_JAL]    : dbg_inst_str = "jal";
            opcode_instr_o[`ENUM_INST_JALR]   : dbg_inst_str = "jalr";
            opcode_instr_o[`ENUM_INST_BEQ]    : dbg_inst_str = "beq";
            opcode_instr_o[`ENUM_INST_BNE]    : dbg_inst_str = "bne";
            opcode_instr_o[`ENUM_INST_BLT]    : dbg_inst_str = "blt";
            opcode_instr_o[`ENUM_INST_BGE]    : dbg_inst_str = "bge";
            opcode_instr_o[`ENUM_INST_BLTU]   : dbg_inst_str = "bltu";
            opcode_instr_o[`ENUM_INST_BGEU]   : dbg_inst_str = "bgeu";
            opcode_instr_o[`ENUM_INST_LB]     : dbg_inst_str = "lb";
            opcode_instr_o[`ENUM_INST_LH]     : dbg_inst_str = "lh";
            opcode_instr_o[`ENUM_INST_LW]     : dbg_inst_str = "lw";
            opcode_instr_o[`ENUM_INST_LBU]    : dbg_inst_str = "lbu";
            opcode_instr_o[`ENUM_INST_LHU]    : dbg_inst_str = "lhu";
            opcode_instr_o[`ENUM_INST_LWU]    : dbg_inst_str = "lwu";
            opcode_instr_o[`ENUM_INST_SB]     : dbg_inst_str = "sb";
            opcode_instr_o[`ENUM_INST_SH]     : dbg_inst_str = "sh";
            opcode_instr_o[`ENUM_INST_SW]     : dbg_inst_str = "sw";
            opcode_instr_o[`ENUM_INST_ECALL]  : dbg_inst_str = "ecall";
            opcode_instr_o[`ENUM_INST_EBREAK] : dbg_inst_str = "ebreak";
            opcode_instr_o[`ENUM_INST_ERET]   : dbg_inst_str = "eret";
            opcode_instr_o[`ENUM_INST_CSRRW]  : dbg_inst_str = "csrrw";
            opcode_instr_o[`ENUM_INST_CSRRS]  : dbg_inst_str = "csrrs";
            opcode_instr_o[`ENUM_INST_CSRRC]  : dbg_inst_str = "csrrc";
            opcode_instr_o[`ENUM_INST_CSRRWI] : dbg_inst_str = "csrrwi";
            opcode_instr_o[`ENUM_INST_CSRRSI] : dbg_inst_str = "csrrsi";
            opcode_instr_o[`ENUM_INST_CSRRCI] : dbg_inst_str = "csrrci";
            opcode_instr_o[`ENUM_INST_MUL]    : dbg_inst_str = "mul";
            opcode_instr_o[`ENUM_INST_MULH]   : dbg_inst_str = "mulh";
            opcode_instr_o[`ENUM_INST_MULHSU] : dbg_inst_str = "mulhsu";
            opcode_instr_o[`ENUM_INST_MULHU]  : dbg_inst_str = "mulhu";
            opcode_instr_o[`ENUM_INST_DIV]    : dbg_inst_str = "div";
            opcode_instr_o[`ENUM_INST_DIVU]   : dbg_inst_str = "divu";
            opcode_instr_o[`ENUM_INST_REM]    : dbg_inst_str = "rem";
            opcode_instr_o[`ENUM_INST_REMU]   : dbg_inst_str = "remu";
            opcode_instr_o[`ENUM_INST_FAULT]  : dbg_inst_str = "FAULT";
            opcode_instr_o[`ENUM_INST_PAGE_FAULT]  : dbg_inst_str = "PAGE_FAULT";
        endcase

        case (1'b1)

            opcode_instr_o[`ENUM_INST_ADDI],  // addi
            opcode_instr_o[`ENUM_INST_ANDI],  // andi
            opcode_instr_o[`ENUM_INST_SLTI],  // slti
            opcode_instr_o[`ENUM_INST_SLTIU], // sltiu
            opcode_instr_o[`ENUM_INST_ORI],   // ori
            opcode_instr_o[`ENUM_INST_XORI],  // xori
            opcode_instr_o[`ENUM_INST_CSRRW], // csrrw
            opcode_instr_o[`ENUM_INST_CSRRS], // csrrs
            opcode_instr_o[`ENUM_INST_CSRRC], // csrrc
            opcode_instr_o[`ENUM_INST_CSRRWI],// csrrwi
            opcode_instr_o[`ENUM_INST_CSRRSI],// csrrsi
            opcode_instr_o[`ENUM_INST_CSRRCI]:// csrrci
            begin
                dbg_inst_rb  = "-";
                dbg_inst_imm = `DBG_IMM_IMM12;
            end

            opcode_instr_o[`ENUM_INST_SLLI], // slli
            opcode_instr_o[`ENUM_INST_SRLI], // srli
            opcode_instr_o[`ENUM_INST_SRAI]: // srai
            begin
                dbg_inst_rb  = "-";
                dbg_inst_imm = {27'b0, `DBG_IMM_SHAMT};
            end

            opcode_instr_o[`ENUM_INST_LUI]: // lui
            begin
                dbg_inst_ra  = "-";
                dbg_inst_rb  = "-";
                dbg_inst_imm = `DBG_IMM_IMM20;
            end

            opcode_instr_o[`ENUM_INST_AUIPC]: // auipc
            begin
                dbg_inst_ra  = "pc";
                dbg_inst_rb  = "-";
                dbg_inst_imm = `DBG_IMM_IMM20;
            end   

            opcode_instr_o[`ENUM_INST_JAL]:  // jal
            begin
                dbg_inst_ra  = "-";
                dbg_inst_rb  = "-";
                dbg_inst_imm = opcode_pc_o + `DBG_IMM_JIMM20;
            end

            opcode_instr_o[`ENUM_INST_JALR]: // jalr
            begin
                dbg_inst_rb  = "-";
                dbg_inst_imm = opcode_ra_operand_o + `DBG_IMM_IMM12;
            end

            // lb lh lw lbu lhu lwu
            opcode_instr_o[`ENUM_INST_LB],
            opcode_instr_o[`ENUM_INST_LH],
            opcode_instr_o[`ENUM_INST_LW],
            opcode_instr_o[`ENUM_INST_LBU],
            opcode_instr_o[`ENUM_INST_LHU],
            opcode_instr_o[`ENUM_INST_LWU]:
            begin
                dbg_inst_rb  = "-";
                dbg_inst_imm = opcode_ra_operand_o + `DBG_IMM_IMM12;
            end 

            // sb sh sw
            opcode_instr_o[`ENUM_INST_SB],
            opcode_instr_o[`ENUM_INST_SH],
            opcode_instr_o[`ENUM_INST_SW]:
            begin
                dbg_inst_rd  = "-";
                dbg_inst_imm = opcode_ra_operand_o + `DBG_IMM_STOREIMM;
            end
        endcase        
    end
end
`endif



endmodule
