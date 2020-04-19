//-----------------------------------------------------------------
//                         RISC-V Core
//                            V1.0
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
//-----------------------------------------------------------------
// Module - Xilinx register file (2 async read, 1 write port)
//-----------------------------------------------------------------
module riscv_xilinx_2r1w
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input  [  4:0]  rd0_i
    ,input  [ 31:0]  rd0_value_i
    ,input  [  4:0]  ra_i
    ,input  [  4:0]  rb_i

    // Outputs
    ,output [ 31:0]  ra_value_o
    ,output [ 31:0]  rb_value_o
);


//-----------------------------------------------------------------
// Registers / Wires
//-----------------------------------------------------------------
wire [31:0]     reg_rs1_w;
wire [31:0]     reg_rs2_w;
wire [31:0]     rs1_0_15_w;
wire [31:0]     rs1_16_31_w;
wire [31:0]     rs2_0_15_w;
wire [31:0]     rs2_16_31_w;
wire            write_enable_w;
wire            write_banka_w;
wire            write_bankb_w;

//-----------------------------------------------------------------
// Register File (using RAM16X1D )
//-----------------------------------------------------------------
genvar i;

// Registers 0 - 15
generate
for (i=0;i<32;i=i+1)
begin : reg_loop1
    RAM16X1D reg_bit1a(.WCLK(clk_i), .WE(write_banka_w), .A0(rd0_i[0]), .A1(rd0_i[1]), .A2(rd0_i[2]), .A3(rd0_i[3]), .D(rd0_value_i[i]), .DPRA0(ra_i[0]), .DPRA1(ra_i[1]), .DPRA2(ra_i[2]), .DPRA3(ra_i[3]), .DPO(rs1_0_15_w[i]), .SPO(/* open */));
    RAM16X1D reg_bit2a(.WCLK(clk_i), .WE(write_banka_w), .A0(rd0_i[0]), .A1(rd0_i[1]), .A2(rd0_i[2]), .A3(rd0_i[3]), .D(rd0_value_i[i]), .DPRA0(rb_i[0]), .DPRA1(rb_i[1]), .DPRA2(rb_i[2]), .DPRA3(rb_i[3]), .DPO(rs2_0_15_w[i]), .SPO(/* open */));
end
endgenerate

// Registers 16 - 31
generate
for (i=0;i<32;i=i+1)
begin : reg_loop2
    RAM16X1D reg_bit1b(.WCLK(clk_i), .WE(write_bankb_w), .A0(rd0_i[0]), .A1(rd0_i[1]), .A2(rd0_i[2]), .A3(rd0_i[3]), .D(rd0_value_i[i]), .DPRA0(ra_i[0]), .DPRA1(ra_i[1]), .DPRA2(ra_i[2]), .DPRA3(ra_i[3]), .DPO(rs1_16_31_w[i]), .SPO(/* open */));
    RAM16X1D reg_bit2b(.WCLK(clk_i), .WE(write_bankb_w), .A0(rd0_i[0]), .A1(rd0_i[1]), .A2(rd0_i[2]), .A3(rd0_i[3]), .D(rd0_value_i[i]), .DPRA0(rb_i[0]), .DPRA1(rb_i[1]), .DPRA2(rb_i[2]), .DPRA3(rb_i[3]), .DPO(rs2_16_31_w[i]), .SPO(/* open */));
end
endgenerate

//-----------------------------------------------------------------
// Combinatorial Assignments
//-----------------------------------------------------------------
assign reg_rs1_w       = (ra_i[4] == 1'b0) ? rs1_0_15_w : rs1_16_31_w;
assign reg_rs2_w       = (rb_i[4] == 1'b0) ? rs2_0_15_w : rs2_16_31_w;

assign write_enable_w = (rd0_i != 5'b00000);

assign write_banka_w  = (write_enable_w & (~rd0_i[4]));
assign write_bankb_w  = (write_enable_w & rd0_i[4]);

reg [31:0] ra_value_r;
reg [31:0] rb_value_r;

// Register read ports
always @ *
begin
    if (ra_i == 5'b00000)
        ra_value_r = 32'h00000000;
    else
        ra_value_r = reg_rs1_w;

    if (rb_i == 5'b00000)
        rb_value_r = 32'h00000000;
    else
        rb_value_r = reg_rs2_w;
end

assign ra_value_o = ra_value_r;
assign rb_value_o = rb_value_r;

endmodule

//-------------------------------------------------------------
// RAM16X1D: Verilator target RAM16X1D model
//-------------------------------------------------------------
`ifdef verilator
module RAM16X1D (DPO, SPO, A0, A1, A2, A3, D, DPRA0, DPRA1, DPRA2, DPRA3, WCLK, WE);

    parameter INIT = 16'h0000;

    output DPO, SPO;

    input  A0, A1, A2, A3, D, DPRA0, DPRA1, DPRA2, DPRA3, WCLK, WE;

    reg  [15:0] mem;
    wire [3:0] adr;

    assign adr = {A3, A2, A1, A0};
    assign SPO = mem[adr];
    assign DPO = mem[{DPRA3, DPRA2, DPRA1, DPRA0}];

    initial 
        mem = INIT;

    always @(posedge WCLK) 
        if (WE == 1'b1)
            mem[adr] <= D;

endmodule
`endif
