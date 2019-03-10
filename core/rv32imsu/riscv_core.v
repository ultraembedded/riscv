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

wire           mmu_flush_w;
wire           csr_stall_w;
wire           fetch_accept_w;
wire  [ 57:0]  opcode_instr_w;
wire           arb_mmu_error_w;
wire           csr_opcode_valid_w;
wire           arb_mmu_accept_w;
wire           branch_csr_request_w;
wire  [ 31:0]  mmu_ifetch_inst_w;
wire  [ 31:0]  opcode_pc_w;
wire           mmu_lsu_error_w;
wire           writeback_csr_squash_w;
wire           arb_cpu_cacheable_w;
wire  [ 31:0]  mmu_fault_addr_w;
wire           arb_cpu_error_w;
wire           fault_store_w;
wire           mmu_fetch_fault_w;
wire           mmu_ifetch_valid_w;
wire           arb_cpu_rd_w;
wire  [ 31:0]  arb_mmu_data_wr_w;
wire           writeback_exec_squash_w;
wire  [ 10:0]  arb_mmu_req_tag_w;
wire           fault_misaligned_load_w;
wire  [ 31:0]  writeback_muldiv_value_w;
wire  [  3:0]  mmu_lsu_wr_w;
wire           branch_request_w;
wire           mmu_ifetch_flush_w;
wire           fetch_invalidate_w;
wire           exec_stall_w;
wire  [ 31:0]  mmu_lsu_data_wr_w;
wire  [ 10:0]  mmu_lsu_resp_tag_w;
wire  [ 10:0]  mmu_lsu_req_tag_w;
wire  [ 31:0]  opcode_ra_operand_w;
wire           muldiv_opcode_valid_w;
wire  [  4:0]  writeback_exec_idx_w;
wire           arb_mmu_cacheable_w;
wire           fault_page_store_w;
wire  [ 31:0]  branch_pc_w;
wire           muldiv_stall_w;
wire           lsu_stall_w;
wire  [  4:0]  writeback_csr_idx_w;
wire  [ 31:0]  arb_cpu_data_rd_w;
wire  [ 31:0]  opcode_opcode_w;
wire  [  4:0]  opcode_rb_idx_w;
wire           fetch_branch_w;
wire  [ 31:0]  mmu_ifetch_pc_w;
wire           arb_cpu_flush_w;
wire  [  4:0]  writeback_mem_idx_w;
wire           arb_mmu_rd_w;
wire  [ 10:0]  arb_cpu_req_tag_w;
wire  [ 31:0]  arb_mmu_data_rd_w;
wire           arb_mmu_ack_w;
wire           mmu_lsu_accept_w;
wire  [ 31:0]  mmu_lsu_addr_w;
wire  [ 31:0]  fault_addr_w;
wire           mmu_ifetch_accept_w;
wire           mmu_lsu_ack_w;
wire  [ 31:0]  fetch_pc_w;
wire           fault_load_w;
wire           mmu_ifetch_invalidate_w;
wire  [  4:0]  writeback_muldiv_idx_w;
wire           fault_misaligned_store_w;
wire  [ 31:0]  arb_cpu_addr_w;
wire           mmu_lsu_rd_w;
wire           mmu_ifetch_error_w;
wire           mmu_supervisor_w;
wire           arb_cpu_ack_w;
wire           arb_cpu_invalidate_w;
wire  [ 31:0]  arb_mmu_addr_w;
wire  [  4:0]  opcode_ra_idx_w;
wire  [ 31:0]  writeback_mem_value_w;
wire  [  3:0]  arb_cpu_wr_w;
wire           mmu_ifetch_rd_w;
wire  [ 31:0]  branch_csr_pc_w;
wire  [ 10:0]  arb_cpu_resp_tag_w;
wire           arb_cpu_accept_w;
wire           mmu_load_fault_w;
wire  [ 31:0]  mmu_satp_w;
wire  [ 31:0]  opcode_rb_operand_w;
wire           mmu_lsu_invalidate_w;
wire           fault_page_load_w;
wire  [ 31:0]  arb_cpu_data_wr_w;
wire  [  4:0]  opcode_rd_idx_w;
wire           fetch_valid_w;
wire           mmu_lsu_cacheable_w;
wire  [  3:0]  arb_mmu_wr_w;
wire           lsu_opcode_valid_w;
wire  [ 31:0]  fetch_instr_w;
wire           arb_mmu_invalidate_w;
wire  [ 31:0]  mmu_lsu_data_rd_w;
wire           exec_opcode_valid_w;
wire           mmu_lsu_flush_w;
wire  [ 31:0]  fetch_branch_pc_w;
wire           writeback_muldiv_squash_w;
wire           mmu_sum_w;
wire  [ 31:0]  writeback_exec_value_w;
wire  [ 10:0]  arb_mmu_resp_tag_w;
wire  [ 31:0]  writeback_csr_value_w;
wire           writeback_mem_squash_w;
wire           mmu_store_fault_w;
wire           arb_mmu_flush_w;


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


riscv_mmu u_mmu
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.outport_data_rd_i(arb_mmu_data_rd_w)
    ,.outport_accept_i(arb_mmu_accept_w)
    ,.outport_ack_i(arb_mmu_ack_w)
    ,.outport_error_i(arb_mmu_error_w)
    ,.outport_resp_tag_i(arb_mmu_resp_tag_w)
    ,.supervisor_i(mmu_supervisor_w)
    ,.sum_i(mmu_sum_w)
    ,.flush_i(mmu_flush_w)
    ,.satp_i(mmu_satp_w)
    ,.fetch_in_rd_i(mmu_ifetch_rd_w)
    ,.fetch_in_flush_i(mmu_ifetch_flush_w)
    ,.fetch_in_invalidate_i(mmu_ifetch_invalidate_w)
    ,.fetch_in_pc_i(mmu_ifetch_pc_w)
    ,.fetch_out_accept_i(mem_i_accept_i)
    ,.fetch_out_valid_i(mem_i_valid_i)
    ,.fetch_out_error_i(mem_i_error_i)
    ,.fetch_out_inst_i(mem_i_inst_i)
    ,.lsu_in_addr_i(mmu_lsu_addr_w)
    ,.lsu_in_data_wr_i(mmu_lsu_data_wr_w)
    ,.lsu_in_rd_i(mmu_lsu_rd_w)
    ,.lsu_in_wr_i(mmu_lsu_wr_w)
    ,.lsu_in_cacheable_i(mmu_lsu_cacheable_w)
    ,.lsu_in_req_tag_i(mmu_lsu_req_tag_w)
    ,.lsu_in_invalidate_i(mmu_lsu_invalidate_w)
    ,.lsu_in_flush_i(mmu_lsu_flush_w)
    ,.lsu_out_data_rd_i(arb_cpu_data_rd_w)
    ,.lsu_out_accept_i(arb_cpu_accept_w)
    ,.lsu_out_ack_i(arb_cpu_ack_w)
    ,.lsu_out_error_i(arb_cpu_error_w)
    ,.lsu_out_resp_tag_i(arb_cpu_resp_tag_w)

    // Outputs
    ,.outport_addr_o(arb_mmu_addr_w)
    ,.outport_data_wr_o(arb_mmu_data_wr_w)
    ,.outport_rd_o(arb_mmu_rd_w)
    ,.outport_wr_o(arb_mmu_wr_w)
    ,.outport_cacheable_o(arb_mmu_cacheable_w)
    ,.outport_req_tag_o(arb_mmu_req_tag_w)
    ,.outport_invalidate_o(arb_mmu_invalidate_w)
    ,.outport_flush_o(arb_mmu_flush_w)
    ,.fetch_in_accept_o(mmu_ifetch_accept_w)
    ,.fetch_in_valid_o(mmu_ifetch_valid_w)
    ,.fetch_in_error_o(mmu_ifetch_error_w)
    ,.fetch_in_inst_o(mmu_ifetch_inst_w)
    ,.fetch_out_rd_o(mem_i_rd_o)
    ,.fetch_out_flush_o(mem_i_flush_o)
    ,.fetch_out_invalidate_o(mem_i_invalidate_o)
    ,.fetch_out_pc_o(mem_i_pc_o)
    ,.fetch_fault_o(mmu_fetch_fault_w)
    ,.fetch_fault_addr_o()
    ,.lsu_in_data_rd_o(mmu_lsu_data_rd_w)
    ,.lsu_in_accept_o(mmu_lsu_accept_w)
    ,.lsu_in_ack_o(mmu_lsu_ack_w)
    ,.lsu_in_error_o(mmu_lsu_error_w)
    ,.lsu_in_resp_tag_o(mmu_lsu_resp_tag_w)
    ,.lsu_out_addr_o(arb_cpu_addr_w)
    ,.lsu_out_data_wr_o(arb_cpu_data_wr_w)
    ,.lsu_out_rd_o(arb_cpu_rd_w)
    ,.lsu_out_wr_o(arb_cpu_wr_w)
    ,.lsu_out_cacheable_o(arb_cpu_cacheable_w)
    ,.lsu_out_req_tag_o(arb_cpu_req_tag_w)
    ,.lsu_out_invalidate_o(arb_cpu_invalidate_w)
    ,.lsu_out_flush_o(arb_cpu_flush_w)
    ,.load_fault_o(mmu_load_fault_w)
    ,.store_fault_o(mmu_store_fault_w)
    ,.fault_addr_o(mmu_fault_addr_w)
);


riscv_mmu_arb u_arb
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.inport_cpu_addr_i(arb_cpu_addr_w)
    ,.inport_cpu_data_wr_i(arb_cpu_data_wr_w)
    ,.inport_cpu_rd_i(arb_cpu_rd_w)
    ,.inport_cpu_wr_i(arb_cpu_wr_w)
    ,.inport_cpu_cacheable_i(arb_cpu_cacheable_w)
    ,.inport_cpu_req_tag_i(arb_cpu_req_tag_w)
    ,.inport_cpu_invalidate_i(arb_cpu_invalidate_w)
    ,.inport_cpu_flush_i(arb_cpu_flush_w)
    ,.inport_mmu_addr_i(arb_mmu_addr_w)
    ,.inport_mmu_data_wr_i(arb_mmu_data_wr_w)
    ,.inport_mmu_rd_i(arb_mmu_rd_w)
    ,.inport_mmu_wr_i(arb_mmu_wr_w)
    ,.inport_mmu_cacheable_i(arb_mmu_cacheable_w)
    ,.inport_mmu_req_tag_i(arb_mmu_req_tag_w)
    ,.inport_mmu_invalidate_i(arb_mmu_invalidate_w)
    ,.inport_mmu_flush_i(arb_mmu_flush_w)
    ,.outport_data_rd_i(mem_d_data_rd_i)
    ,.outport_accept_i(mem_d_accept_i)
    ,.outport_ack_i(mem_d_ack_i)
    ,.outport_error_i(mem_d_error_i)
    ,.outport_resp_tag_i(mem_d_resp_tag_i)

    // Outputs
    ,.inport_cpu_data_rd_o(arb_cpu_data_rd_w)
    ,.inport_cpu_accept_o(arb_cpu_accept_w)
    ,.inport_cpu_ack_o(arb_cpu_ack_w)
    ,.inport_cpu_error_o(arb_cpu_error_w)
    ,.inport_cpu_resp_tag_o(arb_cpu_resp_tag_w)
    ,.inport_mmu_data_rd_o(arb_mmu_data_rd_w)
    ,.inport_mmu_accept_o(arb_mmu_accept_w)
    ,.inport_mmu_ack_o(arb_mmu_ack_w)
    ,.inport_mmu_error_o(arb_mmu_error_w)
    ,.inport_mmu_resp_tag_o(arb_mmu_resp_tag_w)
    ,.outport_addr_o(mem_d_addr_o)
    ,.outport_data_wr_o(mem_d_data_wr_o)
    ,.outport_rd_o(mem_d_rd_o)
    ,.outport_wr_o(mem_d_wr_o)
    ,.outport_cacheable_o(mem_d_cacheable_o)
    ,.outport_req_tag_o(mem_d_req_tag_o)
    ,.outport_invalidate_o(mem_d_invalidate_o)
    ,.outport_flush_o(mem_d_flush_o)
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
    ,.mem_data_rd_i(mmu_lsu_data_rd_w)
    ,.mem_accept_i(mmu_lsu_accept_w)
    ,.mem_ack_i(mmu_lsu_ack_w)
    ,.mem_error_i(mmu_lsu_error_w)
    ,.mem_resp_tag_i(mmu_lsu_resp_tag_w)
    ,.mmu_load_fault_i(mmu_load_fault_w)
    ,.mmu_store_fault_i(mmu_store_fault_w)
    ,.mmu_fault_addr_i(mmu_fault_addr_w)

    // Outputs
    ,.mem_addr_o(mmu_lsu_addr_w)
    ,.mem_data_wr_o(mmu_lsu_data_wr_w)
    ,.mem_rd_o(mmu_lsu_rd_w)
    ,.mem_wr_o(mmu_lsu_wr_w)
    ,.mem_cacheable_o(mmu_lsu_cacheable_w)
    ,.mem_req_tag_o(mmu_lsu_req_tag_w)
    ,.mem_invalidate_o(mmu_lsu_invalidate_w)
    ,.mem_flush_o(mmu_lsu_flush_w)
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
    ,.mmu_supervisor_o(mmu_supervisor_w)
    ,.mmu_sum_o(mmu_sum_w)
    ,.mmu_flush_o(mmu_flush_w)
    ,.mmu_satp_o(mmu_satp_w)
);


riscv_muldiv u_muldiv
(
    // Inputs
     .clk_i(clk_i)
    ,.rst_i(rst_i)
    ,.opcode_valid_i(muldiv_opcode_valid_w)
    ,.opcode_instr_i(opcode_instr_w)
    ,.opcode_opcode_i(opcode_opcode_w)
    ,.opcode_pc_i(opcode_pc_w)
    ,.opcode_rd_idx_i(opcode_rd_idx_w)
    ,.opcode_ra_idx_i(opcode_ra_idx_w)
    ,.opcode_rb_idx_i(opcode_rb_idx_w)
    ,.opcode_ra_operand_i(opcode_ra_operand_w)
    ,.opcode_rb_operand_i(opcode_rb_operand_w)

    // Outputs
    ,.writeback_idx_o(writeback_muldiv_idx_w)
    ,.writeback_squash_o(writeback_muldiv_squash_w)
    ,.writeback_value_o(writeback_muldiv_value_w)
    ,.stall_o(muldiv_stall_w)
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
    ,.writeback_muldiv_idx_i(writeback_muldiv_idx_w)
    ,.writeback_muldiv_squash_i(writeback_muldiv_squash_w)
    ,.writeback_muldiv_value_i(writeback_muldiv_value_w)
    ,.exec_stall_i(exec_stall_w)
    ,.lsu_stall_i(lsu_stall_w)
    ,.csr_stall_i(csr_stall_w)
    ,.muldiv_stall_i(muldiv_stall_w)

    // Outputs
    ,.fetch_branch_o(fetch_branch_w)
    ,.fetch_branch_pc_o(fetch_branch_pc_w)
    ,.fetch_accept_o(fetch_accept_w)
    ,.exec_opcode_valid_o(exec_opcode_valid_w)
    ,.lsu_opcode_valid_o(lsu_opcode_valid_w)
    ,.csr_opcode_valid_o(csr_opcode_valid_w)
    ,.muldiv_opcode_valid_o(muldiv_opcode_valid_w)
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
    ,.icache_accept_i(mmu_ifetch_accept_w)
    ,.icache_valid_i(mmu_ifetch_valid_w)
    ,.icache_error_i(mmu_ifetch_error_w)
    ,.icache_inst_i(mmu_ifetch_inst_w)
    ,.mmu_fetch_fault_i(mmu_fetch_fault_w)
    ,.fetch_invalidate_i(fetch_invalidate_w)

    // Outputs
    ,.fetch_valid_o(fetch_valid_w)
    ,.fetch_instr_o(fetch_instr_w)
    ,.fetch_pc_o(fetch_pc_w)
    ,.icache_rd_o(mmu_ifetch_rd_w)
    ,.icache_flush_o(mmu_ifetch_flush_w)
    ,.icache_invalidate_o(mmu_ifetch_invalidate_w)
    ,.icache_pc_o(mmu_ifetch_pc_w)
);



endmodule
