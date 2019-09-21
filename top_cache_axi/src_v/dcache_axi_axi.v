//-----------------------------------------------------------------
//                         RISC-V Top
//                            V0.6
//                     Ultra-Embedded.com
//                     Copyright 2014-2019
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

//-----------------------------------------------------------------
//                          Generated File
//-----------------------------------------------------------------
module dcache_axi_axi
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input           inport_valid_i
    ,input           inport_write_i
    ,input  [ 31:0]  inport_addr_i
    ,input  [  3:0]  inport_id_i
    ,input  [  7:0]  inport_len_i
    ,input  [  1:0]  inport_burst_i
    ,input  [ 31:0]  inport_wdata_i
    ,input  [  3:0]  inport_wstrb_i
    ,input           inport_bready_i
    ,input           inport_rready_i
    ,input           outport_awready_i
    ,input           outport_wready_i
    ,input           outport_bvalid_i
    ,input  [  1:0]  outport_bresp_i
    ,input  [  3:0]  outport_bid_i
    ,input           outport_arready_i
    ,input           outport_rvalid_i
    ,input  [ 31:0]  outport_rdata_i
    ,input  [  1:0]  outport_rresp_i
    ,input  [  3:0]  outport_rid_i
    ,input           outport_rlast_i

    // Outputs
    ,output          inport_accept_o
    ,output          inport_bvalid_o
    ,output [  1:0]  inport_bresp_o
    ,output [  3:0]  inport_bid_o
    ,output          inport_rvalid_o
    ,output [ 31:0]  inport_rdata_o
    ,output [  1:0]  inport_rresp_o
    ,output [  3:0]  inport_rid_o
    ,output          inport_rlast_o
    ,output          outport_awvalid_o
    ,output [ 31:0]  outport_awaddr_o
    ,output [  3:0]  outport_awid_o
    ,output [  7:0]  outport_awlen_o
    ,output [  1:0]  outport_awburst_o
    ,output          outport_wvalid_o
    ,output [ 31:0]  outport_wdata_o
    ,output [  3:0]  outport_wstrb_o
    ,output          outport_wlast_o
    ,output          outport_bready_o
    ,output          outport_arvalid_o
    ,output [ 31:0]  outport_araddr_o
    ,output [  3:0]  outport_arid_o
    ,output [  7:0]  outport_arlen_o
    ,output [  1:0]  outport_arburst_o
    ,output          outport_rready_o
);



//-------------------------------------------------------------
// Write Request
//-------------------------------------------------------------
reg awvalid_inhibit_q;
reg wvalid_inhibit_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    awvalid_inhibit_q <= 1'b0;
else if (outport_awvalid_o && outport_awready_i && outport_wvalid_o && !outport_wready_i)
    awvalid_inhibit_q <= 1'b1;
else if (outport_awvalid_o && outport_awready_i && outport_awlen_o != 8'b0)
    awvalid_inhibit_q <= 1'b1;
else if (outport_wvalid_o && outport_wready_i && outport_wlast_o)
    awvalid_inhibit_q <= 1'b0;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    wvalid_inhibit_q <= 1'b0;
else if (outport_wvalid_o && outport_wready_i && outport_awvalid_o && !outport_awready_i)
    wvalid_inhibit_q <= 1'b1;
else if (outport_awvalid_o && outport_awready_i)
    wvalid_inhibit_q <= 1'b0;

assign outport_awvalid_o = (inport_valid_i & inport_write_i & ~awvalid_inhibit_q);
assign outport_awaddr_o  = inport_addr_i;
assign outport_awid_o    = inport_id_i;
assign outport_awlen_o   = inport_len_i;
assign outport_awburst_o = inport_burst_i;

//-------------------------------------------------------------
// Write burst tracking
//-------------------------------------------------------------
reg  [7:0] req_cnt_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    req_cnt_q <= 8'b0;
else if (outport_awvalid_o && outport_awready_i)
begin
    // First data not accepted yet
    if (!outport_wready_i && !wvalid_inhibit_q)
        req_cnt_q <= (outport_awlen_o + 8'd1);
    // First data already accepted
    else
        req_cnt_q <= outport_awlen_o;
end
else if (req_cnt_q != 8'd0 && outport_wvalid_o && outport_wready_i)
    req_cnt_q <= req_cnt_q - 8'd1;

wire wlast_w = (outport_awvalid_o && outport_awlen_o == 8'b0) || (req_cnt_q == 8'd1);

//-------------------------------------------------------------
// Write data skid buffer
//-------------------------------------------------------------
reg buf_valid_q;
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    buf_valid_q <= 1'b0;
else if (outport_wvalid_o && !outport_wready_i && outport_awvalid_o && outport_awready_i)
    buf_valid_q <= 1'b1;
else if (outport_wready_i)
    buf_valid_q <= 1'b0;

reg [36:0] buf_q;
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    buf_q <= 37'b0;
else
    buf_q <= {outport_wlast_o, outport_wstrb_o, outport_wdata_o};

assign outport_wvalid_o  = buf_valid_q ? 1'b1 : (inport_valid_i & inport_write_i & ~wvalid_inhibit_q);
assign outport_wdata_o   = buf_valid_q ? buf_q[31:0]  : inport_wdata_i;
assign outport_wstrb_o   = buf_valid_q ? buf_q[35:32] : inport_wstrb_i;
assign outport_wlast_o   = buf_valid_q ? buf_q[36:36] : wlast_w;

assign inport_bvalid_o   = outport_bvalid_i;
assign inport_bresp_o    = outport_bresp_i;
assign inport_bid_o      = outport_bid_i;
assign outport_bready_o  = inport_bready_i;

//-------------------------------------------------------------
// Read Request
//-------------------------------------------------------------
assign outport_arvalid_o = inport_valid_i & ~inport_write_i;
assign outport_araddr_o  = inport_addr_i;
assign outport_arid_o    = inport_id_i;
assign outport_arlen_o   = inport_len_i;
assign outport_arburst_o = inport_burst_i;
assign outport_rready_o  = inport_rready_i;

assign inport_rvalid_o   = outport_rvalid_i;
assign inport_rdata_o    = outport_rdata_i;
assign inport_rresp_o    = outport_rresp_i;
assign inport_rid_o      = outport_rid_i;
assign inport_rlast_o    = outport_rlast_i;

//-------------------------------------------------------------
// Accept logic
//-------------------------------------------------------------
assign inport_accept_o   = (outport_awvalid_o && outport_awready_i) || 
                           (outport_wvalid_o  && outport_wready_i && !buf_valid_q)  ||
                           (outport_arvalid_o && outport_arready_i);


endmodule
