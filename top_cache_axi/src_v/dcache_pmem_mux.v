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
module dcache_pmem_mux
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input           outport_accept_i
    ,input           outport_ack_i
    ,input           outport_error_i
    ,input  [ 31:0]  outport_read_data_i
    ,input           select_i
    ,input  [  3:0]  inport0_wr_i
    ,input           inport0_rd_i
    ,input  [  7:0]  inport0_len_i
    ,input  [ 31:0]  inport0_addr_i
    ,input  [ 31:0]  inport0_write_data_i
    ,input  [  3:0]  inport1_wr_i
    ,input           inport1_rd_i
    ,input  [  7:0]  inport1_len_i
    ,input  [ 31:0]  inport1_addr_i
    ,input  [ 31:0]  inport1_write_data_i

    // Outputs
    ,output [  3:0]  outport_wr_o
    ,output          outport_rd_o
    ,output [  7:0]  outport_len_o
    ,output [ 31:0]  outport_addr_o
    ,output [ 31:0]  outport_write_data_o
    ,output          inport0_accept_o
    ,output          inport0_ack_o
    ,output          inport0_error_o
    ,output [ 31:0]  inport0_read_data_o
    ,output          inport1_accept_o
    ,output          inport1_ack_o
    ,output          inport1_error_o
    ,output [ 31:0]  inport1_read_data_o
);




//-----------------------------------------------------------------
// Output Mux
//-----------------------------------------------------------------
reg [  3:0]  outport_wr_r;
reg          outport_rd_r;
reg [  7:0]  outport_len_r;
reg [ 31:0]  outport_addr_r;
reg [ 31:0]  outport_write_data_r;
reg          select_q;

always @ *
begin
    case (select_i)
    1'd1:
    begin
        outport_wr_r          = inport1_wr_i;
        outport_rd_r          = inport1_rd_i;
        outport_len_r         = inport1_len_i;
        outport_addr_r        = inport1_addr_i;
        outport_write_data_r  = inport1_write_data_i;
    end
    default:
    begin
        outport_wr_r          = inport0_wr_i;
        outport_rd_r          = inport0_rd_i;
        outport_len_r         = inport0_len_i;
        outport_addr_r        = inport0_addr_i;
        outport_write_data_r  = inport0_write_data_i;
    end
    endcase
end

assign outport_wr_o         = outport_wr_r;
assign outport_rd_o         = outport_rd_r;
assign outport_len_o        = outport_len_r;
assign outport_addr_o       = outport_addr_r;
assign outport_write_data_o = outport_write_data_r;

// Delayed version of selector to match phase of response signals
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    select_q <= 1'b0;
else
    select_q <= select_i;

assign inport0_ack_o       = (select_q == 1'd0) && outport_ack_i;
assign inport0_error_o     = (select_q == 1'd0) && outport_error_i;
assign inport0_read_data_o = outport_read_data_i;
assign inport0_accept_o    = (select_i == 1'd0) && outport_accept_i;
assign inport1_ack_o       = (select_q == 1'd1) && outport_ack_i;
assign inport1_error_o     = (select_q == 1'd1) && outport_error_i;
assign inport1_read_data_o = outport_read_data_i;
assign inport1_accept_o    = (select_i == 1'd1) && outport_accept_i;


endmodule
