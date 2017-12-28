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
module axi4_ram_bridge
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input           axi_awvalid_i
    ,input  [ 31:0]  axi_awaddr_i
    ,input  [  3:0]  axi_awid_i
    ,input  [  7:0]  axi_awlen_i
    ,input  [  1:0]  axi_awburst_i
    ,input           axi_wvalid_i
    ,input  [ 31:0]  axi_wdata_i
    ,input  [  3:0]  axi_wstrb_i
    ,input           axi_wlast_i
    ,input           axi_bready_i
    ,input           axi_arvalid_i
    ,input  [ 31:0]  axi_araddr_i
    ,input  [  3:0]  axi_arid_i
    ,input  [  7:0]  axi_arlen_i
    ,input  [  1:0]  axi_arburst_i
    ,input           axi_rready_i
    ,input  [ 31:0]  ram_read_data_i
    ,input           ram_accept_i

    // Outputs
    ,output          axi_awready_o
    ,output          axi_wready_o
    ,output          axi_bvalid_o
    ,output [  1:0]  axi_bresp_o
    ,output [  3:0]  axi_bid_o
    ,output          axi_arready_o
    ,output          axi_rvalid_o
    ,output [ 31:0]  axi_rdata_o
    ,output [  1:0]  axi_rresp_o
    ,output [  3:0]  axi_rid_o
    ,output          axi_rlast_o
    ,output [  3:0]  ram_wr_o
    ,output          ram_rd_o
    ,output [ 31:0]  ram_addr_o
    ,output [ 31:0]  ram_write_data_o
);

//-------------------------------------------------------------
// calculate_addr_next
//-------------------------------------------------------------
function [31:0] calculate_addr_next;
    input [31:0] addr;
    input [1:0]  axtype;
    input [7:0]  axlen;

    reg [31:0]   mask;
begin
    mask = 0;

    case (axtype)
`ifdef SUPPORT_FIXED_BURST
    2'd0: // AXI4_BURST_FIXED
        calculate_addr_next = addr;
`endif
`ifdef SUPPORT_WRAP_BURST
    2'd2: // AXI4_BURST_WRAP
        case (axlen)
        8'd0:      mask = 8'h03;
        8'd1:      mask = 8'h07;
        8'd3:      mask = 8'h0F;
        8'd7:      mask = 8'h1F;
        8'd15:     mask = 8'h3F;
        default:   mask = 8'h3F;
        endcase

        calculate_addr_next = (addr & ~mask) | ((addr + 4) & mask);
`endif
    default: // AXI4_BURST_INCR
        calculate_addr_next = addr + 4;
    endcase
end
endfunction

//-----------------------------------------------------------------
// Registers / Wires
//-----------------------------------------------------------------
reg [7:0]   req_len_q;
reg [31:0]  req_addr_q;
reg         req_rd_q;
reg         req_wr_q;
reg [3:0]   req_id_q;
reg [1:0]   req_axburst_q;
reg [7:0]   req_axlen_q;

reg         bvalid_q;
reg         rvalid_q;
reg         rlast_q;
reg         rbuf_valid_q;
reg [31:0]  rbuf_data_q;
reg         rbuf_last_q;

//-----------------------------------------------------------------
// Sequential
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    req_len_q     <= 8'b0;
    req_addr_q    <= 32'b0;
    req_wr_q      <= 1'b0;
    req_rd_q      <= 1'b0;
    req_id_q      <= 4'b0;
    req_axburst_q <= 2'b0;
    req_axlen_q   <= 8'b0;
    bvalid_q      <= 1'b0;
    rvalid_q      <= 1'b0;
    rlast_q       <= 1'b0;
    rbuf_valid_q  <= 1'b0;
    rbuf_data_q   <= 32'b0;
    rbuf_last_q   <= 1'b0;
end
else
begin
    rvalid_q <= 1'b0;
    rlast_q  <= 1'b0;

    if (axi_bready_i)
        bvalid_q <= 1'b0;

    // Burst continuation
    if ((req_rd_q && ram_accept_i && axi_rready_i) ||  // Read accepted
        (req_wr_q && axi_wvalid_i && axi_wready_o))    // Write data presented
    begin
        rvalid_q <= req_rd_q;

        if (req_len_q == 0)
        begin
            bvalid_q <= req_wr_q;
            rlast_q  <= req_rd_q;
            req_rd_q <= 1'b0;
            req_wr_q <= 1'b0;
        end
        else
        begin
            req_addr_q <= calculate_addr_next(req_addr_q, req_axburst_q, req_axlen_q);
            req_len_q  <= req_len_q - 8'd1;
        end
    end

    // Write command accepted
    if (axi_awvalid_i && axi_awready_o)
    begin
        // Data ready?
        if (axi_wvalid_i && axi_wready_o)
        begin
            req_wr_q      <= !axi_wlast_i;
            req_len_q     <= axi_awlen_i - 8'd1;
            req_id_q      <= axi_awid_i;
            req_axburst_q <= axi_awburst_i;
            req_axlen_q   <= axi_awlen_i;

            req_addr_q    <= calculate_addr_next(axi_awaddr_i, axi_awburst_i, axi_awlen_i);

            bvalid_q      <= axi_wlast_i;
        end
        // Data not ready
        else
        begin
            req_wr_q      <= 1'b1;
            req_len_q     <= axi_awlen_i;
            req_id_q      <= axi_awid_i;
            req_axburst_q <= axi_awburst_i;
            req_axlen_q   <= axi_awlen_i;
            req_addr_q    <= axi_awaddr_i;
        end
    end
    // Read command accepted
    else if (axi_arvalid_i && axi_arready_o)
    begin
        req_rd_q      <= (axi_arlen_i != 0);
        req_len_q     <= axi_arlen_i - 8'd1;
        req_addr_q    <= calculate_addr_next(axi_araddr_i, axi_arburst_i, axi_arlen_i);
        req_id_q      <= axi_arid_i;
        req_axburst_q <= axi_arburst_i;
        req_axlen_q   <= axi_arlen_i;

        rvalid_q      <= 1'b1;
        rlast_q       <= (axi_arlen_i == 0);
    end

    // Response skid buffer
    if (axi_rvalid_o && !axi_rready_i)
    begin
        rbuf_valid_q <= 1'b1;
        rbuf_data_q  <= axi_rdata_o;
        rbuf_last_q  <= axi_rlast_o;
    end
    else
        rbuf_valid_q <= 1'b0;
end

//-----------------------------------------------------------------
// Assignments
//-----------------------------------------------------------------
assign axi_bvalid_o  = bvalid_q;
assign axi_bresp_o   = 2'b0;
assign axi_bid_o     = req_id_q;

assign axi_rvalid_o  = rvalid_q | rbuf_valid_q;
assign axi_rresp_o   = 2'b0;
assign axi_rdata_o   = rbuf_valid_q ? rbuf_data_q : ram_read_data_i;
assign axi_rid_o     = req_id_q;
assign axi_rlast_o   = rbuf_valid_q ? rbuf_last_q : rlast_q;

// Write takes priority
wire write_active    = (axi_awvalid_i || req_wr_q) && !req_rd_q;
wire read_active     = (axi_arvalid_i || req_rd_q) && !write_active;

assign axi_awready_o = write_active && !req_wr_q && (!axi_bvalid_o || axi_bready_i) && ram_accept_i;
assign axi_wready_o  = write_active && (!axi_bvalid_o || axi_bready_i) && ram_accept_i;
assign axi_arready_o = read_active  && !req_rd_q && ram_accept_i && (!axi_rvalid_o || axi_rready_i);

wire [31:0] addr_w   = ((req_wr_q || req_rd_q) ? req_addr_q:
                        write_active ? axi_awaddr_i : axi_araddr_i);

wire wr_w    = write_active && axi_wvalid_i;
wire rd_w    = read_active;

// RAM if
assign ram_addr_o       = addr_w;
assign ram_write_data_o = axi_wdata_i;
assign ram_rd_o         = rd_w;
assign ram_wr_o         = wr_w ? axi_wstrb_i : 4'b0;


endmodule
