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
module dcache_mux
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
    ,input           mem_writeback_i
    ,input           mem_flush_i
    ,input  [ 31:0]  mem_cached_data_rd_i
    ,input           mem_cached_accept_i
    ,input           mem_cached_ack_i
    ,input           mem_cached_error_i
    ,input  [ 10:0]  mem_cached_resp_tag_i
    ,input  [ 31:0]  mem_uncached_data_rd_i
    ,input           mem_uncached_accept_i
    ,input           mem_uncached_ack_i
    ,input           mem_uncached_error_i
    ,input  [ 10:0]  mem_uncached_resp_tag_i

    // Outputs
    ,output [ 31:0]  mem_data_rd_o
    ,output          mem_accept_o
    ,output          mem_ack_o
    ,output          mem_error_o
    ,output [ 10:0]  mem_resp_tag_o
    ,output [ 31:0]  mem_cached_addr_o
    ,output [ 31:0]  mem_cached_data_wr_o
    ,output          mem_cached_rd_o
    ,output [  3:0]  mem_cached_wr_o
    ,output          mem_cached_cacheable_o
    ,output [ 10:0]  mem_cached_req_tag_o
    ,output          mem_cached_invalidate_o
    ,output          mem_cached_writeback_o
    ,output          mem_cached_flush_o
    ,output [ 31:0]  mem_uncached_addr_o
    ,output [ 31:0]  mem_uncached_data_wr_o
    ,output          mem_uncached_rd_o
    ,output [  3:0]  mem_uncached_wr_o
    ,output          mem_uncached_cacheable_o
    ,output [ 10:0]  mem_uncached_req_tag_o
    ,output          mem_uncached_invalidate_o
    ,output          mem_uncached_writeback_o
    ,output          mem_uncached_flush_o
    ,output          cache_active_o
);



wire hold_w;
reg  cache_access_q;

assign mem_cached_addr_o         = mem_addr_i;
assign mem_cached_data_wr_o      = mem_data_wr_i;
assign mem_cached_rd_o           = (mem_cacheable_i & ~hold_w) ? mem_rd_i : 1'b0;
assign mem_cached_wr_o           = (mem_cacheable_i & ~hold_w) ? mem_wr_i : 4'b0;
assign mem_cached_cacheable_o    = mem_cacheable_i;
assign mem_cached_req_tag_o      = mem_req_tag_i;
assign mem_cached_invalidate_o   = (mem_cacheable_i & ~hold_w) ? mem_invalidate_i : 1'b0;
assign mem_cached_writeback_o    = (mem_cacheable_i & ~hold_w) ? mem_writeback_i : 1'b0;
assign mem_cached_flush_o        = (mem_cacheable_i & ~hold_w) ? mem_flush_i : 1'b0;

assign mem_uncached_addr_o       = mem_addr_i;
assign mem_uncached_data_wr_o    = mem_data_wr_i;
assign mem_uncached_rd_o         = (~mem_cacheable_i & ~hold_w) ? mem_rd_i : 1'b0;
assign mem_uncached_wr_o         = (~mem_cacheable_i & ~hold_w) ? mem_wr_i : 4'b0;
assign mem_uncached_cacheable_o  = mem_cacheable_i;
assign mem_uncached_req_tag_o    = mem_req_tag_i;
assign mem_uncached_invalidate_o = (~mem_cacheable_i & ~hold_w) ? mem_invalidate_i : 1'b0;
assign mem_uncached_writeback_o  = (~mem_cacheable_i & ~hold_w) ? mem_writeback_i : 1'b0;
assign mem_uncached_flush_o      = (~mem_cacheable_i & ~hold_w) ? mem_flush_i : 1'b0;

assign mem_accept_o              =(mem_cacheable_i ? mem_cached_accept_i  : mem_uncached_accept_i) & !hold_w;
assign mem_data_rd_o             = cache_access_q ? mem_cached_data_rd_i  : mem_uncached_data_rd_i;
assign mem_ack_o                 = cache_access_q ? mem_cached_ack_i      : mem_uncached_ack_i;
assign mem_error_o               = cache_access_q ? mem_cached_error_i    : mem_uncached_error_i;
assign mem_resp_tag_o            = cache_access_q ? mem_cached_resp_tag_i : mem_uncached_resp_tag_i;

wire      request_w              = mem_rd_i | mem_wr_i != 4'b0 | mem_flush_i | mem_invalidate_i | mem_writeback_i;

reg [4:0] pending_r;
reg [4:0] pending_q;
always @ *
begin
    pending_r = pending_q;

    if ((request_w && mem_accept_o) && !mem_ack_o)
        pending_r = pending_r + 5'd1;
    else if (!(request_w && mem_accept_o) && mem_ack_o)
        pending_r = pending_r - 5'd1;
end

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    pending_q <= 5'b0;
else
    pending_q <= pending_r;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    cache_access_q <= 1'b0;
else if (request_w && mem_accept_o)
    cache_access_q <= mem_cacheable_i;

assign hold_w = (|pending_q) && (cache_access_q != mem_cacheable_i);

assign cache_active_o = (|pending_q) ? cache_access_q : mem_cacheable_i;


endmodule
