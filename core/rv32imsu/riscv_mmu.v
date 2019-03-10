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

module riscv_mmu
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input  [ 31:0]  outport_data_rd_i
    ,input           outport_accept_i
    ,input           outport_ack_i
    ,input           outport_error_i
    ,input  [ 10:0]  outport_resp_tag_i
    ,input           supervisor_i
    ,input           sum_i
    ,input           flush_i
    ,input  [ 31:0]  satp_i
    ,input           fetch_in_rd_i
    ,input           fetch_in_flush_i
    ,input           fetch_in_invalidate_i
    ,input  [ 31:0]  fetch_in_pc_i
    ,input           fetch_out_accept_i
    ,input           fetch_out_valid_i
    ,input           fetch_out_error_i
    ,input  [ 31:0]  fetch_out_inst_i
    ,input  [ 31:0]  lsu_in_addr_i
    ,input  [ 31:0]  lsu_in_data_wr_i
    ,input           lsu_in_rd_i
    ,input  [  3:0]  lsu_in_wr_i
    ,input           lsu_in_cacheable_i
    ,input  [ 10:0]  lsu_in_req_tag_i
    ,input           lsu_in_invalidate_i
    ,input           lsu_in_flush_i
    ,input  [ 31:0]  lsu_out_data_rd_i
    ,input           lsu_out_accept_i
    ,input           lsu_out_ack_i
    ,input           lsu_out_error_i
    ,input  [ 10:0]  lsu_out_resp_tag_i

    // Outputs
    ,output [ 31:0]  outport_addr_o
    ,output [ 31:0]  outport_data_wr_o
    ,output          outport_rd_o
    ,output [  3:0]  outport_wr_o
    ,output          outport_cacheable_o
    ,output [ 10:0]  outport_req_tag_o
    ,output          outport_invalidate_o
    ,output          outport_flush_o
    ,output          fetch_in_accept_o
    ,output          fetch_in_valid_o
    ,output          fetch_in_error_o
    ,output [ 31:0]  fetch_in_inst_o
    ,output          fetch_out_rd_o
    ,output          fetch_out_flush_o
    ,output          fetch_out_invalidate_o
    ,output [ 31:0]  fetch_out_pc_o
    ,output          fetch_fault_o
    ,output [ 31:0]  fetch_fault_addr_o
    ,output [ 31:0]  lsu_in_data_rd_o
    ,output          lsu_in_accept_o
    ,output          lsu_in_ack_o
    ,output          lsu_in_error_o
    ,output [ 10:0]  lsu_in_resp_tag_o
    ,output [ 31:0]  lsu_out_addr_o
    ,output [ 31:0]  lsu_out_data_wr_o
    ,output          lsu_out_rd_o
    ,output [  3:0]  lsu_out_wr_o
    ,output          lsu_out_cacheable_o
    ,output [ 10:0]  lsu_out_req_tag_o
    ,output          lsu_out_invalidate_o
    ,output          lsu_out_flush_o
    ,output          load_fault_o
    ,output          store_fault_o
    ,output [ 31:0]  fault_addr_o
);



//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------

// Current state
localparam  STATE_IDLE         = 0;
localparam  STATE_LEVEL_FIRST  = 1;
localparam  STATE_LEVEL_SECOND = 2;
localparam  STATE_UPDATE       = 3;

//-----------------------------------------------------------------
// SATP CSR bits
//-----------------------------------------------------------------
`define SATP_PPN_R        19:0 // TODO: Should be 21??
`define SATP_ASID_R       30:22
`define SATP_MODE_R       31

//--------------------------------------------------------------------
// MMU Defs
//--------------------------------------------------------------------
`define MMU_LEVELS          2
`define MMU_PTIDXBITS       10
`define MMU_PTESIZE         4
`define MMU_PGSHIFT         (`MMU_PTIDXBITS + 2)
`define MMU_PGSIZE          (1 << `MMU_PGSHIFT)
`define MMU_VPN_BITS        (`MMU_PTIDXBITS * `MMU_LEVELS)
`define MMU_PPN_BITS        (32 - `MMU_PGSHIFT)
`define MMU_VA_BITS         (`MMU_VPN_BITS + `MMU_PGSHIFT)

`define PAGE_PRESENT   0
`define PAGE_READ      1
`define PAGE_WRITE     2
`define PAGE_EXEC      3
`define PAGE_USER      4
`define PAGE_GLOBAL    5
`define PAGE_ACCESSED  6
`define PAGE_DIRTY     7
`define PAGE_SOFT      9:8

`define PAGE_FLAGS     10'h3FF

`define PAGE_PFN_SHIFT 10
`define PAGE_SIZE      4096

//-----------------------------------------------------------------
// Registers
//-----------------------------------------------------------------
reg [1:0]   state_q;
wire        idle_w      = (state_q == STATE_IDLE);

wire     resp_valid_w   = outport_ack_i;
wire     resp_error_w   = outport_error_i;

//-----------------------------------------------------------------
// Load / Store
//-----------------------------------------------------------------
reg       load_q;
reg [3:0] store_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    load_q <= 1'b0;
else if (lsu_in_rd_i)
    load_q <= ~lsu_in_accept_o;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    store_q <= 4'b0;
else if (|lsu_in_wr_i)
    store_q <= lsu_in_accept_o ? 4'b0 : lsu_in_wr_i;

wire       load_w  = lsu_in_rd_i | load_q;
wire [3:0] store_w = lsu_in_wr_i | store_q;

reg [31:0] lsu_in_addr_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    lsu_in_addr_q <= 32'b0;
else if (load_w || (|store_w))
    lsu_in_addr_q <= lsu_in_addr_i;

wire [31:0] lsu_addr_w = (load_w || (|store_w)) ? lsu_in_addr_i : lsu_in_addr_q;

//-----------------------------------------------------------------
// Misc
//-----------------------------------------------------------------
wire        itlb_hit_w;
wire        dtlb_hit_w;

reg         dtlb_req_q;

wire        vm_enable_w = satp_i[`SATP_MODE_R];
wire [31:0] ptbr_w      = {satp_i[`SATP_PPN_R], 12'b0};

// TLB entry does not match request address
wire        itlb_miss_w = fetch_in_rd_i & vm_enable_w & ~itlb_hit_w;
wire        dtlb_miss_w = (load_w || (|store_w)) & vm_enable_w & ~dtlb_hit_w;

// Data miss is higher priority than instruction...
wire [31:0] request_addr_w = idle_w ? 
                            (dtlb_miss_w ? lsu_addr_w : fetch_in_pc_i) :
                             dtlb_req_q ? lsu_addr_w : fetch_in_pc_i;

reg [31:0]  pte_addr_q;
reg [31:0]  pte_entry_q;
reg [31:0]  virt_addr_q;

wire [31:0] pte_ppn_w   = {`PAGE_PFN_SHIFT'b0, outport_data_rd_i[31:`PAGE_PFN_SHIFT]};
wire [9:0]  pte_flags_w = outport_data_rd_i[9:0];

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    pte_addr_q  <= 32'b0;
    pte_entry_q <= 32'b0;
    virt_addr_q <= 32'b0;
    dtlb_req_q  <= 1'b0;
    state_q     <= STATE_IDLE;
end
else
begin
    // TLB miss, walk page table
    if (state_q == STATE_IDLE && (itlb_miss_w || dtlb_miss_w))
    begin
        pte_addr_q  <= ptbr_w + {20'b0, request_addr_w[31:22], 2'b0};
        virt_addr_q <= request_addr_w;
        dtlb_req_q  <= dtlb_miss_w;

        state_q     <= STATE_LEVEL_FIRST;
    end
    // First level (4MB superpage)
    else if (state_q == STATE_LEVEL_FIRST && resp_valid_w)
    begin
        // Error or page not present
        if (resp_error_w || !outport_data_rd_i[`PAGE_PRESENT])
        begin
            pte_entry_q <= 32'b0;
            state_q     <= STATE_UPDATE;
        end
        // Valid entry, but another level to fetch
        else if (!(outport_data_rd_i[`PAGE_READ] || outport_data_rd_i[`PAGE_WRITE] || outport_data_rd_i[`PAGE_EXEC]))
        begin
            pte_addr_q  <= {outport_data_rd_i[29:10], 12'b0} + {20'b0, request_addr_w[21:12], 2'b0};
            state_q     <= STATE_LEVEL_SECOND;
        end
        // Valid entry, actual valid PTE
        else
        begin
            pte_entry_q <= ((pte_ppn_w | {22'b0, request_addr_w[21:12]}) << `MMU_PGSHIFT) | {22'b0, pte_flags_w};
            state_q     <= STATE_UPDATE;
        end
    end
    // Second level (4KB page)
    else if (state_q == STATE_LEVEL_SECOND && resp_valid_w)
    begin
        // Valid entry, final level
        if (outport_data_rd_i[`PAGE_PRESENT])
        begin
            pte_entry_q <= (pte_ppn_w << `MMU_PGSHIFT) | {22'b0, pte_flags_w};
            state_q     <= STATE_UPDATE;
        end
        // Page fault
        else
        begin
            pte_entry_q <= 32'b0;
            state_q     <= STATE_UPDATE;
        end
    end
    else if (state_q == STATE_UPDATE)
    begin
        state_q    <= STATE_IDLE;
    end
end

//-----------------------------------------------------------------
// IMMU TLB
//-----------------------------------------------------------------
reg         itlb_valid_q;
reg [31:12] itlb_va_addr_q;
reg [31:0]  itlb_entry_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    itlb_valid_q <= 1'b0;
else if (flush_i)
    itlb_valid_q <= 1'b0;
else if (state_q == STATE_UPDATE && !dtlb_req_q)
    itlb_valid_q <= 1'b1;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    itlb_va_addr_q <= 20'b0;
    itlb_entry_q   <= 32'b0;
end
else if (state_q == STATE_UPDATE && !dtlb_req_q)
begin
    itlb_va_addr_q <= virt_addr_q[31:12];
    itlb_entry_q   <= pte_entry_q;
end

// TLB address matched (even on page fault)
assign itlb_hit_w   = fetch_in_rd_i & itlb_valid_q & (itlb_va_addr_q == fetch_in_pc_i[31:12]);

reg pc_fault_r;
always @ *
begin
    pc_fault_r = 1'b0;

    if (vm_enable_w && itlb_hit_w)
    begin
        // Supervisor mode
        if (supervisor_i)
        begin
            // User page, supervisor user mode not enabled
            if (itlb_entry_q[`PAGE_USER] && !sum_i)
                pc_fault_r = 1'b1;
            // Check exec permissions
            else
                pc_fault_r = ~itlb_entry_q[`PAGE_EXEC];
        end
        // User mode
        else
            pc_fault_r = (~itlb_entry_q[`PAGE_EXEC]) | (~itlb_entry_q[`PAGE_USER]);
    end
end

assign fetch_fault_o          = pc_fault_r;
assign fetch_fault_addr_o     = fetch_in_pc_i;

assign fetch_out_rd_o         = (~vm_enable_w & fetch_in_rd_i) || (itlb_hit_w & ~fetch_fault_o);
assign fetch_out_pc_o         = vm_enable_w ? {itlb_entry_q[31:12], fetch_in_pc_i[11:0]} : fetch_in_pc_i;
assign fetch_out_flush_o      = fetch_in_flush_i;
assign fetch_out_invalidate_o = fetch_in_invalidate_i; // TODO: ...

assign fetch_in_accept_o      = (~vm_enable_w & fetch_out_accept_i) | (vm_enable_w & itlb_hit_w & fetch_out_accept_i) | fetch_fault_o;
assign fetch_in_valid_o       = fetch_out_valid_i;
assign fetch_in_error_o       = fetch_out_error_i;
assign fetch_in_inst_o        = fetch_out_inst_i;

//-----------------------------------------------------------------
// DMMU TLB
//-----------------------------------------------------------------
reg         dtlb_valid_q;
reg [31:12] dtlb_va_addr_q;
reg [31:0]  dtlb_entry_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    dtlb_valid_q <= 1'b0;
else if (flush_i)
    dtlb_valid_q <= 1'b0;
else if (state_q == STATE_UPDATE && dtlb_req_q)
    dtlb_valid_q <= 1'b1;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    dtlb_va_addr_q <= 20'b0;
    dtlb_entry_q   <= 32'b0;
end
else if (state_q == STATE_UPDATE && dtlb_req_q)
begin
    dtlb_va_addr_q <= virt_addr_q[31:12];
    dtlb_entry_q   <= pte_entry_q;
end

// TLB address matched (even on page fault)
assign dtlb_hit_w   = dtlb_valid_q & (dtlb_va_addr_q == lsu_addr_w[31:12]);

reg load_fault_r;
always @ *
begin
    load_fault_r = 1'b0;

    if (vm_enable_w && load_w && dtlb_hit_w)
    begin
        // Supervisor mode
        if (supervisor_i)
        begin
            // User page, supervisor user mode not enabled
            if (dtlb_entry_q[`PAGE_USER] && !sum_i)
                load_fault_r = 1'b1;
            // Check exec permissions
            else
                load_fault_r = ~dtlb_entry_q[`PAGE_READ];
        end
        // User mode
        else
            load_fault_r = (~dtlb_entry_q[`PAGE_READ]) | (~dtlb_entry_q[`PAGE_USER]);
    end
end

reg store_fault_r;
always @ *
begin
    store_fault_r = 1'b0;

    if (vm_enable_w && (|store_w) && dtlb_hit_w)
    begin
        // Supervisor mode
        if (supervisor_i)
        begin
            // User page, supervisor user mode not enabled
            if (dtlb_entry_q[`PAGE_USER] && !sum_i)
                store_fault_r = 1'b1;
            // Check exec permissions
            else
                store_fault_r = (~dtlb_entry_q[`PAGE_READ]) | (~dtlb_entry_q[`PAGE_WRITE]);
        end
        // User mode
        else
            store_fault_r = (~dtlb_entry_q[`PAGE_READ]) | (~dtlb_entry_q[`PAGE_WRITE]) | (~dtlb_entry_q[`PAGE_USER]);
    end
end

assign store_fault_o        = store_fault_r;
assign load_fault_o         = load_fault_r;
assign fault_addr_o         = lsu_in_addr_q;

assign lsu_out_rd_o         = vm_enable_w ? (load_w  & dtlb_hit_w & ~load_fault_o)       : lsu_in_rd_i;
assign lsu_out_wr_o         = vm_enable_w ? (store_w & {4{dtlb_hit_w & ~store_fault_o}}) : lsu_in_wr_i;
assign lsu_out_addr_o       = vm_enable_w ? {dtlb_entry_q[31:12], lsu_addr_w[11:0]}      : lsu_addr_w;
assign lsu_out_data_wr_o    = lsu_in_data_wr_i;

assign lsu_out_invalidate_o = lsu_in_invalidate_i;
assign lsu_out_cacheable_o  = lsu_in_cacheable_i;
assign lsu_out_req_tag_o    = lsu_in_req_tag_i;
assign lsu_out_flush_o      = lsu_in_flush_i;

assign lsu_in_ack_o         = lsu_out_ack_i;
assign lsu_in_resp_tag_o    = lsu_out_resp_tag_i;
assign lsu_in_error_o       = lsu_out_error_i;
assign lsu_in_data_rd_o     = lsu_out_data_rd_i;

assign lsu_in_accept_o      = (~vm_enable_w & lsu_out_accept_i) | (vm_enable_w & dtlb_hit_w & lsu_out_accept_i) | store_fault_o | load_fault_o;

//-----------------------------------------------------------------
// PTE Fetch Port
//-----------------------------------------------------------------
reg mem_req_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    mem_req_q <= 1'b0;
else if (state_q == STATE_IDLE && (itlb_miss_w || dtlb_miss_w))
    mem_req_q <= 1'b1;
else if (state_q == STATE_LEVEL_FIRST && resp_valid_w && !resp_error_w && outport_data_rd_i[`PAGE_PRESENT] && (!(outport_data_rd_i[`PAGE_READ] || outport_data_rd_i[`PAGE_WRITE] || outport_data_rd_i[`PAGE_EXEC])))
    mem_req_q <= 1'b1;    
else if (outport_accept_i)
    mem_req_q <= 1'b0;

assign outport_rd_o         = mem_req_q;
assign outport_addr_o       = pte_addr_q;
assign outport_data_wr_o    = 32'b0;
assign outport_wr_o         = 4'b0;
assign outport_cacheable_o  = 1'b1; // TODO: Really??
assign outport_req_tag_o    = 11'b0;
assign outport_invalidate_o = 1'b0;
assign outport_flush_o      = 1'b0;


endmodule
