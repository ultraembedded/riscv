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
module ram_dp_64k
(
    // Inputs
     input           clk0_i
    ,input           rst0_i
    ,input  [ 13:0]  addr0_i
    ,input  [ 31:0]  data0_i
    ,input  [  3:0]  wr0_i
    ,input           clk1_i
    ,input           rst1_i
    ,input  [ 13:0]  addr1_i
    ,input  [ 31:0]  data1_i
    ,input  [  3:0]  wr1_i

    // Outputs
    ,output [ 31:0]  data0_o
    ,output [ 31:0]  data1_o
);

/* verilator lint_off MULTIDRIVEN */
reg [31:0]   ram [16383:0] /*verilator public*/;
/* verilator lint_on MULTIDRIVEN */

reg [31:0] ram_read0_q;
reg [31:0] ram_read1_q;


// Synchronous write
always @ (posedge clk0_i)
begin
    if (wr0_i[0])
        ram[addr0_i][7:0] <= data0_i[7:0];
    if (wr0_i[1])
        ram[addr0_i][15:8] <= data0_i[15:8];
    if (wr0_i[2])
        ram[addr0_i][23:16] <= data0_i[23:16];
    if (wr0_i[3])
        ram[addr0_i][31:24] <= data0_i[31:24];

    ram_read0_q <= ram[addr0_i];
end

always @ (posedge clk1_i)
begin
    if (wr1_i[0])
        ram[addr1_i][7:0] <= data1_i[7:0];
    if (wr1_i[1])
        ram[addr1_i][15:8] <= data1_i[15:8];
    if (wr1_i[2])
        ram[addr1_i][23:16] <= data1_i[23:16];
    if (wr1_i[3])
        ram[addr1_i][31:24] <= data1_i[31:24];

    ram_read1_q <= ram[addr1_i];
end

assign data0_o = ram_read0_q;
assign data1_o = ram_read1_q;


endmodule
