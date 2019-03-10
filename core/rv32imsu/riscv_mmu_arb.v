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

module riscv_mmu_arb
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input  [ 31:0]  inport_cpu_addr_i
    ,input  [ 31:0]  inport_cpu_data_wr_i
    ,input           inport_cpu_rd_i
    ,input  [  3:0]  inport_cpu_wr_i
    ,input           inport_cpu_cacheable_i
    ,input  [ 10:0]  inport_cpu_req_tag_i
    ,input           inport_cpu_invalidate_i
    ,input           inport_cpu_flush_i
    ,input  [ 31:0]  inport_mmu_addr_i
    ,input  [ 31:0]  inport_mmu_data_wr_i
    ,input           inport_mmu_rd_i
    ,input  [  3:0]  inport_mmu_wr_i
    ,input           inport_mmu_cacheable_i
    ,input  [ 10:0]  inport_mmu_req_tag_i
    ,input           inport_mmu_invalidate_i
    ,input           inport_mmu_flush_i
    ,input  [ 31:0]  outport_data_rd_i
    ,input           outport_accept_i
    ,input           outport_ack_i
    ,input           outport_error_i
    ,input  [ 10:0]  outport_resp_tag_i

    // Outputs
    ,output [ 31:0]  inport_cpu_data_rd_o
    ,output          inport_cpu_accept_o
    ,output          inport_cpu_ack_o
    ,output          inport_cpu_error_o
    ,output [ 10:0]  inport_cpu_resp_tag_o
    ,output [ 31:0]  inport_mmu_data_rd_o
    ,output          inport_mmu_accept_o
    ,output          inport_mmu_ack_o
    ,output          inport_mmu_error_o
    ,output [ 10:0]  inport_mmu_resp_tag_o
    ,output [ 31:0]  outport_addr_o
    ,output [ 31:0]  outport_data_wr_o
    ,output          outport_rd_o
    ,output [  3:0]  outport_wr_o
    ,output          outport_cacheable_o
    ,output [ 10:0]  outport_req_tag_o
    ,output          outport_invalidate_o
    ,output          outport_flush_o
);



//-----------------------------------------------------------------
// Request Muxing
//-----------------------------------------------------------------
reg  read_hold_q;
reg  src_mmu_q;
wire src_mmu_w = read_hold_q ? src_mmu_q : inport_mmu_rd_i;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    read_hold_q  <= 1'b0;
    src_mmu_q    <= 1'b0;
end
else if ((outport_rd_o || (|outport_wr_o)) && !outport_accept_i)
begin
    read_hold_q  <= 1'b1;
    src_mmu_q    <= src_mmu_w;
end
else if (outport_accept_i)
    read_hold_q  <= 1'b0;

/* verilator lint_off UNSIGNED */
/* verilator lint_off CMPCONST */
wire cacheable_w  = outport_addr_o >= 32'h80000000 && outport_addr_o <= 32'h8fffffff;
/* verilator lint_on CMPCONST */
/* verilator lint_on UNSIGNED */

assign outport_addr_o       = src_mmu_w ? inport_mmu_addr_i       : inport_cpu_addr_i;
assign outport_data_wr_o    = src_mmu_w ? inport_mmu_data_wr_i    : inport_cpu_data_wr_i;
assign outport_rd_o         = src_mmu_w ? inport_mmu_rd_i         : inport_cpu_rd_i;
assign outport_wr_o         = src_mmu_w ? inport_mmu_wr_i         : inport_cpu_wr_i;
assign outport_cacheable_o  = cacheable_w;
assign outport_req_tag_o    = src_mmu_w ? {1'b0, 3'b111, 7'b0}    : inport_cpu_req_tag_i;
assign outport_invalidate_o = src_mmu_w ? inport_mmu_invalidate_i : inport_cpu_invalidate_i;
assign outport_flush_o      = src_mmu_w ? inport_mmu_flush_i      : inport_cpu_flush_i;

assign inport_mmu_accept_o  = src_mmu_w  & outport_accept_i;
assign inport_cpu_accept_o  = ~src_mmu_w & outport_accept_i;

//-----------------------------------------------------------------
// Response Routing
//-----------------------------------------------------------------
// Magic combo used only by MMU
wire resp_mmu_w = (outport_resp_tag_i[9:7] == 3'b111);

assign inport_cpu_data_rd_o  =  outport_data_rd_i;
assign inport_cpu_ack_o      = ~resp_mmu_w & outport_ack_i;
assign inport_cpu_error_o    = ~resp_mmu_w & outport_error_i;
assign inport_cpu_resp_tag_o =  outport_resp_tag_i;

assign inport_mmu_data_rd_o  =  outport_data_rd_i;
assign inport_mmu_ack_o      =  resp_mmu_w & outport_ack_i;
assign inport_mmu_error_o    =  resp_mmu_w &outport_error_i;
assign inport_mmu_resp_tag_o =  outport_resp_tag_i;




endmodule
