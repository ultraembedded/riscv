//-----------------------------------------------------------------
//                       RISC-V IP Core
//                            V0.1
//                     Ultra-Embedded.com
//                       Copyright 2015
//
//               Email: admin@ultra-embedded.com
//
//                       License: LGPL
//-----------------------------------------------------------------
// Description:
//   Pipelined 32-bit RISC-V CPU implementation with separate
//   Wishbone interfaces for instruction & data access.
//   Contains configurable instruction cache.
//   Runs SW built with '-m32 -mno-rvm'.
//-----------------------------------------------------------------
//
// Copyright (C) 2015 Ultra-Embedded.com
//
// This source file may be used and distributed without         
// restriction provided that this copyright statement is not    
// removed from the file and that any derivative work contains  
// the original copyright notice and the associated disclaimer. 
//
// This source file is free software; you can redistribute it   
// and/or modify it under the terms of the GNU Lesser General   
// Public License as published by the Free Software Foundation; 
// either version 2.1 of the License, or (at your option) any   
// later version.
//
// This source is distributed in the hope that it will be       
// useful, but WITHOUT ANY WARRANTY; without even the implied   
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      
// PURPOSE.  See the GNU Lesser General Public License for more 
// details.
//
// You should have received a copy of the GNU Lesser General    
// Public License along with this source; if not, write to the 
// Free Software Foundation, Inc., 59 Temple Place, Suite 330, 
// Boston, MA  02111-1307  USA
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"

//-----------------------------------------------------------------
// Module - Wishbone fetch unit
//-----------------------------------------------------------------
module riscv_icache_mem_if
( 
    input                       clk_i,
    input                       rst_i,

    // Fetch
    input                       fetch_i,
    input                       burst_i,
    input [31:0]                address_i,

    // Response
    output [31:0]               resp_addr_o,
    output [31:0]               data_o,
    output                      valid_o,
    output                      final_o,

    // Memory interface
    output reg [31:0]           wbm_addr_o,
    input [31:0]                wbm_dat_i,
    output reg [2:0]            wbm_cti_o,
    output reg                  wbm_cyc_o,
    output reg                  wbm_stb_o,
    input                       wbm_stall_i,
    input                       wbm_ack_i
);

//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
parameter FETCH_WORDS_W             = 3; /* 2 ^ 3 * 4 = 32 */
parameter FETCH_BYTES_W             = FETCH_WORDS_W + 2;

parameter WB_CTI_BURST              = 3'b010;
parameter WB_CTI_FINAL              = 3'b111;

//-----------------------------------------------------------------
// Registers / Wires
//-----------------------------------------------------------------

// Word currently being fetched within a line
reg [FETCH_WORDS_W-1:0]  fetch_word_q;
reg [FETCH_WORDS_W-1:0]  resp_word_q;

wire [FETCH_WORDS_W-1:0] next_word_w  = fetch_word_q + 1;

wire penultimate_word_w  = (fetch_word_q == ({FETCH_WORDS_W{1'b1}}-1));

wire final_resp_w        = ((resp_word_q  == {FETCH_WORDS_W{1'b1}}) | ~burst_i);

//-----------------------------------------------------------------
// Pipelined Wishbone Master
//-----------------------------------------------------------------
always @ (posedge rst_i or posedge clk_i )
begin
   if (rst_i == 1'b1)
   begin
        wbm_addr_o      <= 32'h00000000;
        wbm_cti_o       <= 3'b0;
        wbm_stb_o       <= 1'b0;
        wbm_cyc_o       <= 1'b0;

        fetch_word_q    <= {FETCH_WORDS_W{1'b0}};
        resp_word_q     <= {FETCH_WORDS_W{1'b0}};
   end
   else
   begin
        // Idle
        if (!wbm_cyc_o)
        begin
            if (fetch_i)
            begin
                if (burst_i)
                begin
                    wbm_addr_o      <= {address_i[31:FETCH_BYTES_W], {FETCH_BYTES_W{1'b0}}};
                    fetch_word_q    <= {FETCH_WORDS_W{1'b0}};
                    resp_word_q     <= {FETCH_WORDS_W{1'b0}};
                    
                    // Incrementing linear burst
                    wbm_cti_o       <= WB_CTI_BURST;
                end
                else
                begin                    
                    wbm_addr_o      <= address_i;
                    resp_word_q     <= address_i[FETCH_BYTES_W-1:2];

                    // Single fetch
                    wbm_cti_o       <= WB_CTI_FINAL;                    
                end

                // Start fetch from memory
                wbm_stb_o           <= 1'b1;
                wbm_cyc_o           <= 1'b1;                        
            end
        end
        // Access in-progress
        else
        begin
            // Command accepted
            if (~wbm_stall_i)
            begin
                // Fetch next word for line
                if (wbm_cti_o != WB_CTI_FINAL)
                begin
                    wbm_addr_o      <= {wbm_addr_o[31:FETCH_BYTES_W], next_word_w, 2'b0};
                    fetch_word_q    <= next_word_w;
                    
                    // Final word to read?
                    if (penultimate_word_w)
                        wbm_cti_o   <= WB_CTI_FINAL;
                end
                // Fetch complete
                else
                    wbm_stb_o       <= 1'b0;
            end

            // Response
            if (wbm_ack_i)
                resp_word_q         <= resp_word_q + 1;

            // Last response?
            if (final_o)
                wbm_cyc_o           <= 1'b0;
        end
   end
end

// Response
assign data_o       = wbm_dat_i;
assign valid_o      = wbm_ack_i;
assign final_o      = final_resp_w & wbm_ack_i;
assign resp_addr_o  = {address_i[31:FETCH_BYTES_W], resp_word_q, 2'b0};

endmodule
