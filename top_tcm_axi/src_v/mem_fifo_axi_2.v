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
module mem_fifo_axi_2
#(
    parameter WIDTH = 1
)
(
    // Inputs
     input               clk_i
    ,input               rst_i
    ,input  [WIDTH-1:0]  data_in_i
    ,input               push_i
    ,input               pop_i

    // Outputs
    ,output [WIDTH-1:0]  data_out_o
    ,output              accept_o
    ,output              valid_o
);

//-----------------------------------------------------------------
// Registers
//-----------------------------------------------------------------
reg [WIDTH-1:0]  ram_q[1:0];
reg [0:0]        rd_ptr_q;
reg [0:0]        wr_ptr_q;
reg [1:0]        count_q;

//-----------------------------------------------------------------
// Sequential
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
begin
    if (rst_i == 1'b1)
    begin
        count_q   <= 2'b0;
        rd_ptr_q  <= 1'b0;
        wr_ptr_q  <= 1'b0;
    end
    else
    begin
        // Push
        if (push_i & accept_o)
        begin
            ram_q[wr_ptr_q] <= data_in_i;
            wr_ptr_q        <= wr_ptr_q + 1'd1;
        end

        // Pop
        if (pop_i & valid_o)
        begin
            rd_ptr_q      <= rd_ptr_q + 1'd1;
        end

        // Count up
        if ((push_i & accept_o) & ~(pop_i & valid_o))
        begin
            count_q <= count_q + 2'd1;
        end
        // Count down
        else if (~(push_i & accept_o) & (pop_i & valid_o))
        begin
            count_q <= count_q - 2'd1;
        end
    end
end

//-------------------------------------------------------------------
// Combinatorial
//-------------------------------------------------------------------
assign valid_o       = (count_q != 2'd0);
assign accept_o      = (count_q != 2'd2);
assign data_out_o    = ram_q[rd_ptr_q];

endmodule
