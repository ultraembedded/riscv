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

module riscv_lsu
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input           opcode_valid_i
    ,input  [ 57:0]  opcode_instr_i
    ,input  [ 31:0]  opcode_opcode_i
    ,input  [ 31:0]  opcode_pc_i
    ,input  [  4:0]  opcode_rd_idx_i
    ,input  [  4:0]  opcode_ra_idx_i
    ,input  [  4:0]  opcode_rb_idx_i
    ,input  [ 31:0]  opcode_ra_operand_i
    ,input  [ 31:0]  opcode_rb_operand_i
    ,input  [ 31:0]  mem_data_rd_i
    ,input           mem_accept_i
    ,input           mem_ack_i
    ,input           mem_error_i
    ,input  [ 10:0]  mem_resp_tag_i

    // Outputs
    ,output [ 31:0]  mem_addr_o
    ,output [ 31:0]  mem_data_wr_o
    ,output          mem_rd_o
    ,output [  3:0]  mem_wr_o
    ,output          mem_cacheable_o
    ,output [ 10:0]  mem_req_tag_o
    ,output          mem_invalidate_o
    ,output          mem_flush_o
    ,output [  4:0]  writeback_idx_o
    ,output          writeback_squash_o
    ,output [ 31:0]  writeback_value_o
    ,output          fault_store_o
    ,output          fault_load_o
    ,output          fault_misaligned_store_o
    ,output          fault_misaligned_load_o
    ,output          fault_page_store_o
    ,output          fault_page_load_o
    ,output [ 31:0]  fault_addr_o
    ,output          stall_o
);



//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-----------------------------------------------------------------
// Registers / Wires
//-----------------------------------------------------------------
reg [ 31:0]  mem_addr_q;
reg [ 31:0]  mem_data_wr_q;
reg          mem_rd_q;
reg [  3:0]  mem_wr_q;
reg          mem_cacheable_q;
reg [ 10:0]  mem_req_tag_q;
reg          mem_invalidate_q;
reg          mem_flush_q;
reg          mem_unaligned_ld_q;
reg          mem_unaligned_st_q;

//-----------------------------------------------------------------
// Opcode decode
//-----------------------------------------------------------------

wire load_inst_w = (opcode_instr_i[`ENUM_INST_LB]  || 
                    opcode_instr_i[`ENUM_INST_LH]  || 
                    opcode_instr_i[`ENUM_INST_LW]  || 
                    opcode_instr_i[`ENUM_INST_LBU] || 
                    opcode_instr_i[`ENUM_INST_LHU] || 
                    opcode_instr_i[`ENUM_INST_LWU]);

wire load_signed_inst_w = (opcode_instr_i[`ENUM_INST_LB]  || 
                           opcode_instr_i[`ENUM_INST_LH]  || 
                           opcode_instr_i[`ENUM_INST_LW]);

wire store_inst_w = (opcode_instr_i[`ENUM_INST_SB]  || 
                     opcode_instr_i[`ENUM_INST_SH]  || 
                     opcode_instr_i[`ENUM_INST_SW]);

reg [31:0]  mem_addr_r;
reg         mem_unaligned_r;
always @ *
begin
    mem_unaligned_r = 1'b0;

    if (opcode_valid_i && opcode_instr_i[`ENUM_INST_CSRRW])
        mem_addr_r = opcode_ra_operand_i;
    else if (opcode_valid_i && load_inst_w)
        mem_addr_r = opcode_ra_operand_i + {{20{opcode_opcode_i[31]}}, opcode_opcode_i[31:20]};
    else
        mem_addr_r = opcode_ra_operand_i + {{20{opcode_opcode_i[31]}}, opcode_opcode_i[31:25], opcode_opcode_i[11:7]};

    if (opcode_valid_i && (opcode_instr_i[`ENUM_INST_SW] || opcode_instr_i[`ENUM_INST_LW] || opcode_instr_i[`ENUM_INST_LWU]))
        mem_unaligned_r = (mem_addr_r[1:0] != 2'b0);
    else if (opcode_valid_i && (opcode_instr_i[`ENUM_INST_SH] || opcode_instr_i[`ENUM_INST_LH] || opcode_instr_i[`ENUM_INST_LHU]))
        mem_unaligned_r = mem_addr_r[0];
end

wire dcache_flush_w      = opcode_instr_i[`ENUM_INST_CSRRW] && (opcode_opcode_i[31:20] == `CSR_DFLUSH);
wire dcache_writeback_w  = opcode_instr_i[`ENUM_INST_CSRRW] && (opcode_opcode_i[31:20] == `CSR_DWRITEBACK);
wire dcache_invalidate_w = opcode_instr_i[`ENUM_INST_CSRRW] && (opcode_opcode_i[31:20] == `CSR_DINVALIDATE);

//-----------------------------------------------------------------
// Sequential
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    mem_addr_q         <= 32'b0;
    mem_data_wr_q      <= 32'b0;
    mem_rd_q           <= 1'b0;
    mem_wr_q           <= 4'b0;
    mem_req_tag_q      <= 11'b0;
    mem_cacheable_q    <= 1'b0;
    mem_invalidate_q   <= 1'b0;
    mem_flush_q        <= 1'b0;
    mem_unaligned_ld_q <= 1'b0;
    mem_unaligned_st_q <= 1'b0;
end
else if (!((mem_invalidate_o || mem_flush_o || mem_rd_o || mem_wr_o != 4'b0) && !mem_accept_i))
begin
    mem_addr_q         <= 32'b0;
    mem_data_wr_q      <= 32'b0;
    mem_rd_q           <= 1'b0;
    mem_wr_q           <= 4'b0;
    mem_cacheable_q    <= 1'b0;
    mem_req_tag_q      <= 11'b0;
    mem_invalidate_q   <= 1'b0;
    mem_flush_q        <= 1'b0;
    mem_unaligned_ld_q <=  load_inst_w & mem_unaligned_r;
    mem_unaligned_st_q <= ~load_inst_w & mem_unaligned_r;

    // Tag associated with load
    mem_req_tag_q[4:0] <= opcode_rd_idx_i;
    mem_req_tag_q[6:5] <= mem_addr_r[1:0];
    mem_req_tag_q[7]   <= opcode_instr_i[`ENUM_INST_LB] || opcode_instr_i[`ENUM_INST_LBU];
    mem_req_tag_q[8]   <= opcode_instr_i[`ENUM_INST_LH] || opcode_instr_i[`ENUM_INST_LHU];
    mem_req_tag_q[9]   <= opcode_instr_i[`ENUM_INST_LW] || opcode_instr_i[`ENUM_INST_LWU];
    mem_req_tag_q[10]  <= load_signed_inst_w;        

    mem_rd_q <= (opcode_valid_i && load_inst_w && !mem_unaligned_r);

    if (opcode_valid_i && opcode_instr_i[`ENUM_INST_SW] && !mem_unaligned_r)
    begin
        mem_data_wr_q <= opcode_rb_operand_i;
        mem_wr_q      <= 4'hF;
    end
    else if (opcode_valid_i && opcode_instr_i[`ENUM_INST_SH] && !mem_unaligned_r)
    begin
        case (mem_addr_r[1:0])
        2'h2 :
        begin
            mem_data_wr_q  <= {opcode_rb_operand_i[15:0],16'h0000};
            mem_wr_q       <= 4'b1100;
        end
        default :
        begin
            mem_data_wr_q  <= {16'h0000,opcode_rb_operand_i[15:0]};
            mem_wr_q       <= 4'b0011;
        end
        endcase
    end
    else if (opcode_valid_i && opcode_instr_i[`ENUM_INST_SB])
    begin
        case (mem_addr_r[1:0])
        2'h3 :
        begin
            mem_data_wr_q  <= {opcode_rb_operand_i[7:0],24'h000000};
            mem_wr_q       <= 4'b1000;
        end
        2'h2 :
        begin
            mem_data_wr_q  <= {{8'h00,opcode_rb_operand_i[7:0]},16'h0000};
            mem_wr_q       <= 4'b0100;
        end
        2'h1 :
        begin
            mem_data_wr_q  <= {{16'h0000,opcode_rb_operand_i[7:0]},8'h00};
            mem_wr_q       <= 4'b0010;
        end
        2'h0 :
        begin
            mem_data_wr_q  <= {24'h000000,opcode_rb_operand_i[7:0]};
            mem_wr_q       <= 4'b0001;
        end
        default :
            ;
        endcase        
    end
    else
        mem_wr_q         <= 4'b0;

/* verilator lint_off UNSIGNED */
/* verilator lint_off CMPCONST */
    mem_cacheable_q  <= mem_addr_r >= 32'h0 && mem_addr_r <= 32'h7fffffff;
/* verilator lint_on CMPCONST */
/* verilator lint_on UNSIGNED */

    mem_invalidate_q <= opcode_valid_i & dcache_invalidate_w;
    mem_flush_q      <= opcode_valid_i & dcache_flush_w;
    mem_addr_q       <= mem_addr_r;
end

assign mem_addr_o       = mem_addr_q;
assign mem_data_wr_o    = mem_data_wr_q;
assign mem_rd_o         = mem_rd_q;
assign mem_wr_o         = mem_wr_q;
assign mem_cacheable_o  = mem_cacheable_q;
assign mem_req_tag_o    = mem_req_tag_q;
assign mem_invalidate_o = mem_invalidate_q;
assign mem_flush_o      = mem_flush_q;

// Stall upstream if cache is busy
assign stall_o          = ((mem_invalidate_o || mem_flush_o || mem_rd_o || mem_wr_o != 4'b0) && !mem_accept_i);

//-----------------------------------------------------------------
// Error handling / faults
//-----------------------------------------------------------------
// NOTE: Current implementation does not track addresses for multiple outstanding accesses...
assign fault_addr_o             = 32'b0;

assign fault_load_o             = mem_ack_i ? (mem_resp_tag_i[9:7] != 3'b0 && mem_error_i) : 1'b0;
assign fault_store_o            = mem_ack_i ? (mem_resp_tag_i[9:7] == 3'b0 && mem_error_i) : 1'b0;

assign fault_misaligned_store_o = mem_unaligned_st_q;
assign fault_misaligned_load_o  = mem_unaligned_ld_q;
assign fault_page_store_o       = 1'b0;
assign fault_page_load_o        = 1'b0;

//-----------------------------------------------------------------
// Load response
//-----------------------------------------------------------------
reg [1:0]  addr_lsb_r;
reg        load_byte_r;
reg        load_half_r;
reg        load_word_r;
reg        load_signed_r;
reg [4:0]  wb_idx_r;
reg [31:0] wb_result_r;

always @ *
begin
    wb_result_r   = 32'b0;

    // Tag associated with load
    wb_idx_r      = mem_resp_tag_i[4:0];
    addr_lsb_r    = mem_resp_tag_i[6:5];
    load_byte_r   = mem_resp_tag_i[7];
    load_half_r   = mem_resp_tag_i[8];
    load_word_r   = mem_resp_tag_i[9];
    load_signed_r = mem_resp_tag_i[10];

    // Handle responses
    if (mem_ack_i && (load_byte_r || load_half_r || load_word_r))
    begin
        if (load_byte_r)
        begin
            case (addr_lsb_r[1:0])
            2'h3: wb_result_r = {24'b0, mem_data_rd_i[31:24]};
            2'h2: wb_result_r = {24'b0, mem_data_rd_i[23:16]};
            2'h1: wb_result_r = {24'b0, mem_data_rd_i[15:8]};
            2'h0: wb_result_r = {24'b0, mem_data_rd_i[7:0]};
            endcase

            if (load_signed_r && wb_result_r[7])
                wb_result_r = {24'hFFFFFF, wb_result_r[7:0]};
        end
        else if (load_half_r)
        begin
            if (addr_lsb_r[1])
                wb_result_r = {16'b0, mem_data_rd_i[31:16]};
            else
                wb_result_r = {16'b0, mem_data_rd_i[15:0]};

            if (load_signed_r && wb_result_r[15])
                wb_result_r = {16'hFFFF, wb_result_r[15:0]};
        end
        else
            wb_result_r = mem_data_rd_i;
    end
    else
        wb_idx_r    = 5'b0;
end

assign writeback_idx_o   = wb_idx_r;
assign writeback_value_o = wb_result_r;
assign writeback_squash_o= 1'b0;

endmodule
