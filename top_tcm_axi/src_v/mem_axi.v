//-----------------------------------------------------------------
//                         RISC-V Top
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
module mem_axi
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input  [ 31:0]  mem_addr_i
    ,input  [ 31:0]  mem_data_wr_i
    ,input           mem_rd_i
    ,input  [  3:0]  mem_wr_i
    ,input           mem_cacheable_i
    ,input  [ 10:0]  mem_req_tag_i
    ,input           mem_invalidate_i
    ,input           mem_flush_i
    ,input           mem_resp_accept_i
    ,input           axi_awready_i
    ,input           axi_wready_i
    ,input           axi_bvalid_i
    ,input  [  1:0]  axi_bresp_i
    ,input           axi_arready_i
    ,input           axi_rvalid_i
    ,input  [ 31:0]  axi_rdata_i
    ,input  [  1:0]  axi_rresp_i

    // Outputs
    ,output [ 31:0]  mem_data_rd_o
    ,output          mem_accept_o
    ,output          mem_ack_o
    ,output [ 10:0]  mem_resp_tag_o
    ,output          axi_awvalid_o
    ,output [ 31:0]  axi_awaddr_o
    ,output          axi_wvalid_o
    ,output [ 31:0]  axi_wdata_o
    ,output [  3:0]  axi_wstrb_o
    ,output          axi_bready_o
    ,output          axi_arvalid_o
    ,output [ 31:0]  axi_araddr_o
    ,output          axi_rready_o
);



//-------------------------------------------------------------
// Request FIFO
//-------------------------------------------------------------

// Accepts from both FIFOs
wire          res_accept_w;
wire          req_accept_w;

// Output accept
wire          write_complete_w;
wire          read_complete_w;

wire          req_pop_w   = read_complete_w | write_complete_w;
wire          req_valid_w;
wire [69-1:0] req_w;

// Push on transaction and other FIFO not full
wire          req_push_w   = (mem_rd_i || mem_wr_i != 4'b0) && res_accept_w;

mem_fifo_axi_2
#( .WIDTH(32+32+4+1) )
u_req
(
    .clk_i(clk_i),
    .rst_i(rst_i),

    // Input side
    .data_in_i({mem_rd_i, mem_wr_i, mem_data_wr_i, mem_addr_i}),
    .push_i(req_push_w),
    .accept_o(req_accept_w),

    // Outputs
    .valid_o(req_valid_w),
    .data_out_o(req_w),
    .pop_i(req_pop_w)
);

assign mem_accept_o = req_accept_w & res_accept_w;

//-------------------------------------------------------------
// Response Tracking FIFO
//-------------------------------------------------------------
// Push on transaction and other FIFO not full
wire res_push_w = (mem_rd_i || mem_wr_i != 4'b0) && req_accept_w;

mem_fifo_axi_2
#( .WIDTH(11) )
u_resp
(
    .clk_i(clk_i),
    .rst_i(rst_i),

    // Input side
    .data_in_i(mem_req_tag_i),
    .push_i(res_push_w),
    .accept_o(res_accept_w),

    // Outputs
    .valid_o(), // UNUSED
    .data_out_o(mem_resp_tag_o),
    .pop_i(mem_ack_o && mem_resp_accept_i)
);

assign mem_ack_o   = axi_bvalid_i || axi_rvalid_i;

//-------------------------------------------------------------
// Write Request
//-------------------------------------------------------------
wire req_is_read_w  = (req_valid_w ? req_w[68] : 1'b0);
wire req_is_write_w = (req_valid_w ? ~req_w[68] : 1'b0);

reg awvalid_inhibit_q;
reg wvalid_inhibit_q;

always @ (posedge rst_i or posedge clk_i)
if (rst_i)
    awvalid_inhibit_q <= 1'b0;
else if (axi_awvalid_o && axi_awready_i && axi_wvalid_o && !axi_wready_i)
    awvalid_inhibit_q <= 1'b1;
else if (axi_wvalid_o && axi_wready_i)
    awvalid_inhibit_q <= 1'b0;

always @ (posedge rst_i or posedge clk_i)
if (rst_i)
    wvalid_inhibit_q <= 1'b0;
else if (axi_wvalid_o && axi_wready_i && axi_awvalid_o && !axi_awready_i)
    wvalid_inhibit_q <= 1'b1;
else if (axi_awvalid_o && axi_awready_i)
    wvalid_inhibit_q <= 1'b0;

assign axi_awvalid_o = req_is_write_w && !awvalid_inhibit_q;
assign axi_awaddr_o  = {req_w[31:2], 2'b0};
assign axi_wvalid_o  = req_is_write_w && !wvalid_inhibit_q;
assign axi_wdata_o   = req_w[63:32];
assign axi_wstrb_o   = req_w[67:64];

assign axi_bready_o  = mem_resp_accept_i;

assign write_complete_w = (awvalid_inhibit_q || axi_awready_i) &&
                          (wvalid_inhibit_q || axi_wready_i) && req_is_write_w;

//-------------------------------------------------------------
// Read Request
//-------------------------------------------------------------
assign axi_arvalid_o = req_is_read_w;
assign axi_araddr_o  = {req_w[31:2], 2'b0};

assign axi_rready_o  = mem_resp_accept_i;

assign mem_data_rd_o = axi_rdata_i;

assign read_complete_w = axi_arvalid_o && axi_arready_i;

endmodule
