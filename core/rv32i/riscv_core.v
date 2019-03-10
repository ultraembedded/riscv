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

module riscv_core
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input  [ 31:0]  mem_d_data_rd_i
    ,input           mem_d_accept_i
    ,input           mem_d_ack_i
    ,input           mem_d_error_i
    ,input  [ 10:0]  mem_d_resp_tag_i
    ,input           mem_i_accept_i
    ,input           mem_i_valid_i
    ,input           mem_i_error_i
    ,input  [ 31:0]  mem_i_inst_i
    ,input           intr_i
    ,input  [ 31:0]  reset_vector_i
    ,input  [ 31:0]  cpu_id_i

    // Outputs
    ,output [ 31:0]  mem_d_addr_o
    ,output [ 31:0]  mem_d_data_wr_o
    ,output          mem_d_rd_o
    ,output [  3:0]  mem_d_wr_o
    ,output          mem_d_cacheable_o
    ,output [ 10:0]  mem_d_req_tag_o
    ,output          mem_d_invalidate_o
    ,output          mem_d_flush_o
    ,output          mem_i_rd_o
    ,output          mem_i_flush_o
    ,output          mem_i_invalidate_o
    ,output [ 31:0]  mem_i_pc_o
);

wire           fetch_invalidate_w;
wire  [ 31:0]  fetch_branch_pc_w;
wire           fault_page_load_w;
wire  [  4:0]  opcode_rd_idx_w;
wire           fetch_accept_w;
wire  [ 31:0]  fault_addr_w;
wire  [ 57:0]  opcode_instr_w;
wire  [ 31:0]  fetch_pc_w;
wire           fault_load_w;
wire           exec_stall_w;
wire           fetch_valid_w;
wire           csr_opcode_valid_w;
wire  [  4:0]  opcode_rb_idx_w;
wire           branch_csr_request_w;
wire  [  4:0]  writeback_exec_idx_w;
wire  [  4:0]  writeback_mem_idx_w;
wire  [ 31:0]  opcode_pc_w;
wire           fault_page_store_w;
wire           lsu_opcode_valid_w;
wire           writeback_csr_squash_w;
wire  [ 31:0]  branch_pc_w;
wire  [ 31:0]  fetch_instr_w;
wire           lsu_stall_w;
wire           fault_store_w;
wire  [  4:0]  opcode_ra_idx_w;
wire           exec_opcode_valid_w;
wire  [  4:0]  writeback_csr_idx_w;
wire  [ 31:0]  writeback_mem_value_w;
wire           csr_stall_w;
wire  [ 31:0]  opcode_opcode_w;
wire           fault_misaligned_store_w;
wire           fetch_branch_w;
wire           fault_misaligned_load_w;
wire  [ 31:0]  writeback_exec_value_w;
wire  [ 31:0]  branch_csr_pc_w;
wire  [ 31:0]  writeback_csr_value_w;
wire           branch_request_w;
wire           writeback_mem_squash_w;
wire  [ 31:0]  opcode_rb_operand_w;
wire           writeback_exec_squash_w;
wire  [ 31:0]  opcode_ra_operand_w;


riscv_csr u_csr
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.intr_i(intr_i)
    ,.opcode_valid_i(csr_opcode_valid_w)
    ,.opcode_instr_i(opcode_instr_w)
    ,.opcode_opcode_i(opcode_opcode_w)
    ,.opcode_pc_i(opcode_pc_w)
    ,.opcode_rd_idx_i(opcode_rd_idx_w)
    ,.opcode_ra_idx_i(opcode_ra_idx_w)
    ,.opcode_rb_idx_i(opcode_rb_idx_w)
    ,.opcode_ra_operand_i(opcode_ra_operand_w)
    ,.opcode_rb_operand_i(opcode_rb_operand_w)
    ,.branch_exec_request_i(branch_request_w)
    ,.branch_exec_pc_i(branch_pc_w)
    ,.cpu_id_i(cpu_id_i)
    ,.reset_vector_i(reset_vector_i)
    ,.fault_store_i(fault_store_w)
    ,.fault_load_i(fault_load_w)
    ,.fault_misaligned_store_i(fault_misaligned_store_w)
    ,.fault_misaligned_load_i(fault_misaligned_load_w)
    ,.fault_page_store_i(fault_page_store_w)
    ,.fault_page_load_i(fault_page_load_w)
    ,.fault_addr_i(fault_addr_w)

    // Outputs
    ,.writeback_idx_o(writeback_csr_idx_w)
    ,.writeback_squash_o(writeback_csr_squash_w)
    ,.writeback_value_o(writeback_csr_value_w)
    ,.stall_o(csr_stall_w)
    ,.branch_csr_request_o(branch_csr_request_w)
    ,.branch_csr_pc_o(branch_csr_pc_w)
);


riscv_lsu u_lsu
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.opcode_valid_i(lsu_opcode_valid_w)
    ,.opcode_instr_i(opcode_instr_w)
    ,.opcode_opcode_i(opcode_opcode_w)
    ,.opcode_pc_i(opcode_pc_w)
    ,.opcode_rd_idx_i(opcode_rd_idx_w)
    ,.opcode_ra_idx_i(opcode_ra_idx_w)
    ,.opcode_rb_idx_i(opcode_rb_idx_w)
    ,.opcode_ra_operand_i(opcode_ra_operand_w)
    ,.opcode_rb_operand_i(opcode_rb_operand_w)
    ,.mem_data_rd_i(mem_d_data_rd_i)
    ,.mem_accept_i(mem_d_accept_i)
    ,.mem_ack_i(mem_d_ack_i)
    ,.mem_error_i(mem_d_error_i)
    ,.mem_resp_tag_i(mem_d_resp_tag_i)

    // Outputs
    ,.mem_addr_o(mem_d_addr_o)
    ,.mem_data_wr_o(mem_d_data_wr_o)
    ,.mem_rd_o(mem_d_rd_o)
    ,.mem_wr_o(mem_d_wr_o)
    ,.mem_cacheable_o(mem_d_cacheable_o)
    ,.mem_req_tag_o(mem_d_req_tag_o)
    ,.mem_invalidate_o(mem_d_invalidate_o)
    ,.mem_flush_o(mem_d_flush_o)
    ,.writeback_idx_o(writeback_mem_idx_w)
    ,.writeback_squash_o(writeback_mem_squash_w)
    ,.writeback_value_o(writeback_mem_value_w)
    ,.fault_store_o(fault_store_w)
    ,.fault_load_o(fault_load_w)
    ,.fault_misaligned_store_o(fault_misaligned_store_w)
    ,.fault_misaligned_load_o(fault_misaligned_load_w)
    ,.fault_page_store_o(fault_page_store_w)
    ,.fault_page_load_o(fault_page_load_w)
    ,.fault_addr_o(fault_addr_w)
    ,.stall_o(lsu_stall_w)
);


riscv_exec u_exec
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.opcode_valid_i(exec_opcode_valid_w)
    ,.opcode_instr_i(opcode_instr_w)
    ,.opcode_opcode_i(opcode_opcode_w)
    ,.opcode_pc_i(opcode_pc_w)
    ,.opcode_rd_idx_i(opcode_rd_idx_w)
    ,.opcode_ra_idx_i(opcode_ra_idx_w)
    ,.opcode_rb_idx_i(opcode_rb_idx_w)
    ,.opcode_ra_operand_i(opcode_ra_operand_w)
    ,.opcode_rb_operand_i(opcode_rb_operand_w)

    // Outputs
    ,.branch_request_o(branch_request_w)
    ,.branch_pc_o(branch_pc_w)
    ,.writeback_idx_o(writeback_exec_idx_w)
    ,.writeback_squash_o(writeback_exec_squash_w)
    ,.writeback_value_o(writeback_exec_value_w)
    ,.stall_o(exec_stall_w)
);


riscv_decode u_decode
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.fetch_valid_i(fetch_valid_w)
    ,.fetch_instr_i(fetch_instr_w)
    ,.fetch_pc_i(fetch_pc_w)
    ,.branch_request_i(branch_request_w)
    ,.branch_pc_i(branch_pc_w)
    ,.branch_csr_request_i(branch_csr_request_w)
    ,.branch_csr_pc_i(branch_csr_pc_w)
    ,.writeback_exec_idx_i(writeback_exec_idx_w)
    ,.writeback_exec_squash_i(writeback_exec_squash_w)
    ,.writeback_exec_value_i(writeback_exec_value_w)
    ,.writeback_mem_idx_i(writeback_mem_idx_w)
    ,.writeback_mem_squash_i(writeback_mem_squash_w)
    ,.writeback_mem_value_i(writeback_mem_value_w)
    ,.writeback_csr_idx_i(writeback_csr_idx_w)
    ,.writeback_csr_squash_i(writeback_csr_squash_w)
    ,.writeback_csr_value_i(writeback_csr_value_w)
    ,.writeback_muldiv_idx_i(5'b0)
    ,.writeback_muldiv_squash_i(1'b0)
    ,.writeback_muldiv_value_i(32'b0)
    ,.exec_stall_i(exec_stall_w)
    ,.lsu_stall_i(lsu_stall_w)
    ,.csr_stall_i(csr_stall_w)
    ,.muldiv_stall_i(1'b0)

    // Outputs
    ,.fetch_branch_o(fetch_branch_w)
    ,.fetch_branch_pc_o(fetch_branch_pc_w)
    ,.fetch_accept_o(fetch_accept_w)
    ,.exec_opcode_valid_o(exec_opcode_valid_w)
    ,.lsu_opcode_valid_o(lsu_opcode_valid_w)
    ,.csr_opcode_valid_o(csr_opcode_valid_w)
    ,.muldiv_opcode_valid_o()
    ,.opcode_instr_o(opcode_instr_w)
    ,.opcode_opcode_o(opcode_opcode_w)
    ,.opcode_pc_o(opcode_pc_w)
    ,.opcode_rd_idx_o(opcode_rd_idx_w)
    ,.opcode_ra_idx_o(opcode_ra_idx_w)
    ,.opcode_rb_idx_o(opcode_rb_idx_w)
    ,.opcode_ra_operand_o(opcode_ra_operand_w)
    ,.opcode_rb_operand_o(opcode_rb_operand_w)
    ,.fetch_invalidate_o(fetch_invalidate_w)
);


riscv_fetch u_fetch
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.fetch_branch_i(fetch_branch_w)
    ,.fetch_branch_pc_i(fetch_branch_pc_w)
    ,.fetch_accept_i(fetch_accept_w)
    ,.icache_accept_i(mem_i_accept_i)
    ,.icache_valid_i(mem_i_valid_i)
    ,.icache_error_i(mem_i_error_i)
    ,.icache_inst_i(mem_i_inst_i)
    ,.fetch_invalidate_i(fetch_invalidate_w)

    // Outputs
    ,.fetch_valid_o(fetch_valid_w)
    ,.fetch_instr_o(fetch_instr_w)
    ,.fetch_pc_o(fetch_pc_w)
    ,.icache_rd_o(mem_i_rd_o)
    ,.icache_flush_o(mem_i_flush_o)
    ,.icache_invalidate_o(mem_i_invalidate_o)
    ,.icache_pc_o(mem_i_pc_o)
);



endmodule
