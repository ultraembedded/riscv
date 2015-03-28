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
//   Simple, small, multi-cycle 32-bit RISC-V CPU implementation.
//   Runs SW built with '-m32 -mno-rvm'.
//   Most instructions take 3 cycles, apart from load and stores
//   which take 4+ cycles (depending on memory latency).
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
// Module - Core
//-----------------------------------------------------------------
module riscv_core
(
    // General
    input               clk_i,
    input               rst_i,

    input               enable_i,
    input               intr_i,
    input               nmi_i,
    output reg          fault_o,
    output reg          break_o,

    // Memory
    output [31:0]       mem_addr_o,
    output [31:0]       mem_dat_o,
    input [31:0]        mem_dat_i,
    output [3:0]        mem_sel_o,
    output [2:0]        mem_cti_o,
    output              mem_cyc_o,
    output              mem_we_o,
    output              mem_stb_o,
    input               mem_stall_i,
    input               mem_ack_i,

    // CSR access
    output [11:0]       csr_addr_o,
    output [31:0]       csr_data_o,
    input [31:0]        csr_data_i,
    output              csr_set_o,
    output              csr_clr_o
);

//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"
`include "riscv_funcs.v"

//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
parameter           BOOT_VECTOR         = 32'h00000000;
parameter           ISR_VECTOR          = 32'h00000000;
parameter           REGISTER_FILE_TYPE  = "SIMULATION";

`define PC_W        32
`define ADDR_W      32

parameter           PC_W                = `PC_W;
parameter           PC_PAD_W            = 0;
parameter           PC_EXT_W            = 0;

parameter           ADDR_W              = `ADDR_W;
parameter           ADDR_PAD_W          = 0;

// Current state
parameter           STATE_FETCH       = 0;
parameter           STATE_EXEC        = 1;
parameter           STATE_MEM         = 2;
parameter           STATE_WRITE_BACK  = 3;

//-----------------------------------------------------------------
// Registers
//-----------------------------------------------------------------

// Current state
reg [1:0]       state_q;

// Branch PC
reg [PC_W-1:0]  pc_q;

// Destination register
reg [4:0]       rd_q;

// Destination writeback enable
reg             rd_wr_en_q;

// ALU input A
reg [31:0]      alu_a_q;

// ALU input B
reg [31:0]      alu_b_q;

// ALU operation selection
reg [3:0]       alu_func_q;

// CSR read data
reg [31:0]      csr_data_r;

// CSR Registers
reg [PC_W-1:0]  csr_epc_q;
reg [PC_W-1:0]  csr_epc_r;
reg [4:0]       csr_cause_q;
reg [4:0]       csr_cause_r;
reg [3:0]       csr_sr_q;
reg [3:0]       csr_sr_r;

reg             invalid_inst_r;

// Register operands
wire [4:0]      rd_w;
wire [4:0]      rs1_w;
wire [4:0]      rs2_w;

// Operand values
wire [31:0]     rs1_val_w;
wire [31:0]     rs2_val_w;

// Opcode (memory bus)
wire [31:0]     opcode_w       = mem_dat_i;

wire            opcode_valid_w = (state_q == STATE_EXEC) & mem_ack_i;
wire            opcode_fetch_w = (state_q == STATE_FETCH) & !mem_stall_i;

// Instruction types
reg             load_inst_r;
reg             store_inst_r;

// Interrupt taken (valid in exec state)
wire            take_interrupt_w;

// Execute exception (or interrupt)
wire            exception_w;

// Load result (formatted based on load type)
reg [31:0]      load_result_r;

// Writeback enable / value
wire            rd_writeen_w;
wire [31:0]     rd_val_w;

// Memory interface
reg [31:0]       mem_addr_r;
reg [ADDR_W-1:0] mem_addr_q;
reg [31:0]       mem_data_q;
reg [3:0]        mem_sel_q;
reg              mem_we_q;
reg              mem_stb_q;

// Load type / byte / half index
reg [1:0]       load_offset_q;
reg             load_signed_q;
reg             load_byte_q;
reg             load_half_q;

//-----------------------------------------------------------------
// ALU
//-----------------------------------------------------------------
riscv_alu alu
(
    // ALU operation select
    .op_i(alu_func_q),

    // Operands
    .a_i(alu_a_q),
    .b_i(alu_b_q),

    // Result
    .p_o(rd_val_w)
);

//-----------------------------------------------------------------
// [Xilinx] Register file
//-----------------------------------------------------------------
generate
if (REGISTER_FILE_TYPE == "XILINX")
begin : REGFILE_XIL
    riscv_regfile_xil
    u_regfile
    (
        .clk_i(clk_i),
        .rst_i(rst_i),

        // Writeback
        .wr_i(rd_writeen_w),
        .rd_i(rd_q),
        .reg_rd_i(rd_val_w),

        // Read port 1
        .rs1_i(rs1_w),
        .reg_rs1_o(rs1_val_w),

        // Read port 2
        .rs2_i(rs2_w),
        .reg_rs2_o(rs2_val_w)
    );
end
//-----------------------------------------------------------------
// [Simulation] Register file
//-----------------------------------------------------------------
else
begin : REGFILE_SIM
    riscv_regfile_sim
    u_regfile
    (
        .clk_i(clk_i),
        .rst_i(rst_i),

        // Writeback
        .wr_i(rd_writeen_w),
        .rd_i(rd_q),
        .reg_rd_i(rd_val_w),

        // Read port 1
        .rs1_i(rs1_w),
        .reg_rs1_o(rs1_val_w),

        // Read port 2
        .rs2_i(rs2_w),
        .reg_rs2_o(rs2_val_w)
    );
end
endgenerate

// Writeback enable
assign rd_writeen_w  = rd_wr_en_q & (state_q == STATE_WRITE_BACK);

//-----------------------------------------------------------------
// Next State Logic
//-----------------------------------------------------------------
reg [1:0] next_state_r;
always @ *
begin
    next_state_r = state_q;

    case (state_q)
    //-----------------------------------------
    // FETCH - Fetch opcode from memory
    //-----------------------------------------
    STATE_FETCH :
    begin
        if (!mem_stall_i)
            next_state_r    = STATE_EXEC;
    end
    //-----------------------------------------
    // EXEC - Execute instruction (when ready)
    //-----------------------------------------
    STATE_EXEC :
    begin
        // Instruction ready
        if (opcode_valid_w)
        begin
            if (!take_interrupt_w && (load_inst_r || store_inst_r))
                next_state_r    = STATE_MEM;
            else
                next_state_r    = STATE_WRITE_BACK;
        end
    end
    //-----------------------------------------
    // MEM - Perform load or store
    //-----------------------------------------
    STATE_MEM :
    begin
        // Memory access complete
        if (mem_ack_i)
            next_state_r = STATE_WRITE_BACK;
    end    
    //-----------------------------------------
    // WRITE_BACK - Writeback to register file
    //-----------------------------------------
    STATE_WRITE_BACK :
    begin
        if (enable_i)
            next_state_r    = STATE_FETCH;
    end    
    default:
        ;
   endcase
end

// Update state
always @ (posedge rst_i or posedge clk_i )
begin
   if (rst_i == 1'b1)
        state_q   <= STATE_WRITE_BACK;
   else
        state_q   <= next_state_r;
end

//-----------------------------------------------------------------
// Opcode decode
//-----------------------------------------------------------------
reg [31:0]  imm20_r;
reg [31:0]  imm12_r;
reg [31:0]  bimm_r;
reg [31:0]  jimm20_r;
reg [4:0]   shamt_r;
reg [31:0]  storeimm_r;

always @ *
begin
    imm20_r     = {opcode_w[31:12], 12'b0};
    imm12_r     = {{20{opcode_w[31]}}, opcode_w[31:20]};
    bimm_r      = {{19{opcode_w[31]}}, opcode_w[31], opcode_w[7], opcode_w[30:25], opcode_w[11:8], 1'b0};
    jimm20_r    = {{12{opcode_w[31]}}, opcode_w[19:12], opcode_w[20], opcode_w[30:25], opcode_w[24:21], 1'b0};
    storeimm_r  = {{20{opcode_w[31]}}, opcode_w[31:25], opcode_w[11:7]};
    shamt_r     = opcode_w[24:20];
end

assign rs1_w = opcode_w[19:15];
assign rs2_w = opcode_w[24:20];
assign rd_w  = opcode_w[11:7];

//-----------------------------------------------------------------
// Instruction Decode
//-----------------------------------------------------------------
wire inst_andi_w     = ((opcode_w & `INST_ANDI_MASK) == `INST_ANDI);   // andi
wire inst_addi_w     = ((opcode_w & `INST_ADDI_MASK) == `INST_ADDI);   // addi
wire inst_slti_w     = ((opcode_w & `INST_SLTI_MASK) == `INST_SLTI);   // slti
wire inst_sltiu_w    = ((opcode_w & `INST_SLTIU_MASK) == `INST_SLTIU); // sltiu
wire inst_ori_w      = ((opcode_w & `INST_ORI_MASK) == `INST_ORI);     // ori
wire inst_xori_w     = ((opcode_w & `INST_XORI_MASK) == `INST_XORI);   // xori
wire inst_slli_w     = ((opcode_w & `INST_SLLI_MASK) == `INST_SLLI);   // slli
wire inst_srli_w     = ((opcode_w & `INST_SRLI_MASK) == `INST_SRLI);   // srli
wire inst_srai_w     = ((opcode_w & `INST_SRAI_MASK) == `INST_SRAI);   // srai
wire inst_lui_w      = ((opcode_w & `INST_LUI_MASK) == `INST_LUI);     // lui
wire inst_auipc_w    = ((opcode_w & `INST_AUIPC_MASK) == `INST_AUIPC); // auipc
wire inst_add_w      = ((opcode_w & `INST_ADD_MASK) == `INST_ADD);     // add
wire inst_sub_w      = ((opcode_w & `INST_SUB_MASK) == `INST_SUB);     // sub
wire inst_slt_w      = ((opcode_w & `INST_SLT_MASK) == `INST_SLT);     // slt
wire inst_sltu_w     = ((opcode_w & `INST_SLTU_MASK) == `INST_SLTU);   // sltu
wire inst_xor_w      = ((opcode_w & `INST_XOR_MASK) == `INST_XOR);     // xor
wire inst_or_w       = ((opcode_w & `INST_OR_MASK) == `INST_OR);       // or
wire inst_and_w      = ((opcode_w & `INST_AND_MASK) == `INST_AND);     // and
wire inst_sll_w      = ((opcode_w & `INST_SLL_MASK) == `INST_SLL);     // sll
wire inst_srl_w      = ((opcode_w & `INST_SRL_MASK) == `INST_SRL);     // srl
wire inst_sra_w      = ((opcode_w & `INST_SRA_MASK) == `INST_SRA);     // sra
wire inst_jal_w      = ((opcode_w & `INST_JAL_MASK) == `INST_JAL);     // jal
wire inst_jalr_w     = ((opcode_w & `INST_JALR_MASK) == `INST_JALR);   // jalr
wire inst_beq_w      = ((opcode_w & `INST_BEQ_MASK) == `INST_BEQ);     // beq
wire inst_bne_w      = ((opcode_w & `INST_BNE_MASK) == `INST_BNE);     // bne
wire inst_blt_w      = ((opcode_w & `INST_BLT_MASK) == `INST_BLT);     // blt
wire inst_bge_w      = ((opcode_w & `INST_BGE_MASK) == `INST_BGE);     // bge
wire inst_bltu_w     = ((opcode_w & `INST_BLTU_MASK) == `INST_BLTU);   // bltu
wire inst_bgeu_w     = ((opcode_w & `INST_BGEU_MASK) == `INST_BGEU);   // bgeu
wire inst_lb_w       = ((opcode_w & `INST_LB_MASK) == `INST_LB);       // lb
wire inst_lh_w       = ((opcode_w & `INST_LH_MASK) == `INST_LH);       // lh
wire inst_lw_w       = ((opcode_w & `INST_LW_MASK) == `INST_LW);       // lw
wire inst_lbu_w      = ((opcode_w & `INST_LBU_MASK) == `INST_LBU);     // lbu
wire inst_lhu_w      = ((opcode_w & `INST_LHU_MASK) == `INST_LHU);     // lhu
wire inst_lwu_w      = ((opcode_w & `INST_LWU_MASK) == `INST_LWU);     // lwu
wire inst_sb_w       = ((opcode_w & `INST_SB_MASK) == `INST_SB);       // sb
wire inst_sh_w       = ((opcode_w & `INST_SH_MASK) == `INST_SH);       // sh
wire inst_sw_w       = ((opcode_w & `INST_SW_MASK) == `INST_SW);       // sw
wire inst_scall_w    = ((opcode_w & `INST_SCALL_MASK) == `INST_SCALL); // scall
wire inst_sbreak_w   = ((opcode_w & `INST_SBREAK_MASK) == `INST_SBREAK); // sbreak
wire inst_sret_w     = ((opcode_w & `INST_SRET_MASK) == `INST_SRET);   // sret
wire inst_csrrw_w    = ((opcode_w & `INST_CSRRW_MASK) == `INST_CSRRW); // csrrw
wire inst_csrrs_w    = ((opcode_w & `INST_CSRRS_MASK) == `INST_CSRRS); // csrrs
wire inst_csrrc_w    = ((opcode_w & `INST_CSRRC_MASK) == `INST_CSRRC); // csrrc
wire inst_csrrwi_w   = ((opcode_w & `INST_CSRRWI_MASK) == `INST_CSRRWI); // csrrwi
wire inst_csrrsi_w   = ((opcode_w & `INST_CSRRSI_MASK) == `INST_CSRRSI); // csrrsi
wire inst_csrrci_w   = ((opcode_w & `INST_CSRRCI_MASK) == `INST_CSRRCI); // csrrci

// Interrupt request and interrupt enabled
assign take_interrupt_w = intr_i & csr_sr_q[`SR_EI];
assign exception_w      = take_interrupt_w || (opcode_valid_w && invalid_inst_r);

//-----------------------------------------------------------------
// ALU inputs
//-----------------------------------------------------------------

// ALU operation selection
reg [3:0]  alu_func_r;

// ALU operands
reg [31:0] alu_input_a_r;
reg [31:0] alu_input_b_r;
reg        write_rd_r;

always @ *
begin
   alu_func_r     = `ALU_NONE;
   alu_input_a_r  = 32'b0;
   alu_input_b_r  = 32'b0;
   write_rd_r     = 1'b0;

   case (1'b1)

     inst_add_w: // add
     begin
       alu_func_r     = `ALU_ADD;
       alu_input_a_r  = rs1_val_w;
       alu_input_b_r  = rs2_val_w;
       write_rd_r     = 1'b1;
     end

     inst_and_w: // and
     begin
         alu_func_r     = `ALU_AND;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = rs2_val_w;
         write_rd_r     = 1'b1;
     end

     inst_or_w: // or
     begin
         alu_func_r     = `ALU_OR;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = rs2_val_w;
         write_rd_r     = 1'b1;
     end

     inst_sll_w: // sll
     begin
         alu_func_r     = `ALU_SHIFTL;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = rs2_val_w;
         write_rd_r     = 1'b1;
     end

     inst_sra_w: // sra
     begin
         alu_func_r     = `ALU_SHIFTR_ARITH;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = rs2_val_w;
         write_rd_r     = 1'b1;
     end

     inst_srl_w: // srl
     begin
         alu_func_r     = `ALU_SHIFTR;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = rs2_val_w;
         write_rd_r     = 1'b1;
     end

     inst_sub_w: // sub
     begin
         alu_func_r     = `ALU_SUB;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = rs2_val_w;
         write_rd_r     = 1'b1;
     end

     inst_xor_w: // xor
     begin
         alu_func_r     = `ALU_XOR;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = rs2_val_w;
         write_rd_r     = 1'b1;
     end

     inst_slt_w: // slt
     begin
         alu_func_r     = `ALU_LESS_THAN_SIGNED;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = rs2_val_w;
         write_rd_r     = 1'b1;
     end

     inst_sltu_w: // sltu
     begin
         alu_func_r     = `ALU_LESS_THAN;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = rs2_val_w;
         write_rd_r     = 1'b1;
     end

     inst_addi_w: // addi
     begin
         alu_func_r     = `ALU_ADD;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = imm12_r;
         write_rd_r     = 1'b1;
     end

     inst_andi_w: // andi
     begin
         alu_func_r     = `ALU_AND;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = imm12_r;
         write_rd_r     = 1'b1;
     end

     inst_slti_w: // slti
     begin
         alu_func_r     = `ALU_LESS_THAN_SIGNED;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = imm12_r;
         write_rd_r     = 1'b1;
     end

     inst_sltiu_w: // sltiu
     begin
         alu_func_r     = `ALU_LESS_THAN;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = imm12_r;
         write_rd_r     = 1'b1;
     end

     inst_ori_w: // ori
     begin
         alu_func_r     = `ALU_OR;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = imm12_r;
         write_rd_r     = 1'b1;
     end

     inst_xori_w: // xori
     begin
         alu_func_r     = `ALU_XOR;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = imm12_r;
         write_rd_r     = 1'b1;
     end

     inst_slli_w: // slli
     begin
         alu_func_r     = `ALU_SHIFTL;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = {27'b0, shamt_r};
         write_rd_r     = 1'b1;
     end

     inst_srli_w: // srli
     begin
         alu_func_r     = `ALU_SHIFTR;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = {27'b0, shamt_r};
         write_rd_r     = 1'b1;
     end

     inst_srai_w: // srai
     begin
         alu_func_r     = `ALU_SHIFTR_ARITH;
         alu_input_a_r  = rs1_val_w;
         alu_input_b_r  = {27'b0, shamt_r};
         write_rd_r     = 1'b1;
     end

     inst_lui_w: // lui
     begin
         alu_input_a_r  = imm20_r;
         write_rd_r     = 1'b1;
     end

     inst_auipc_w: // auipc
     begin
         alu_func_r     = `ALU_ADD;
         alu_input_a_r[PC_W-1:0]  = pc_q;
         alu_input_b_r  = imm20_r;
         write_rd_r     = 1'b1;
     end     

     inst_jal_w,  // jal
     inst_jalr_w: // jalr
     begin
         alu_func_r     = `ALU_ADD;
         alu_input_a_r[PC_W-1:0]  = pc_q;
         alu_input_b_r  = 4;
         write_rd_r     = 1'b1;
     end

     // lb lh lw lbu lhu lwu
     inst_lb_w,
     inst_lh_w,
     inst_lw_w,
     inst_lbu_w,
     inst_lhu_w,
     inst_lwu_w:
          write_rd_r    = 1'b1;

     inst_csrrw_w,  // csrrw
     inst_csrrs_w,  // csrrs
     inst_csrrc_w,  // csrrc
     inst_csrrwi_w, // csrrwi
     inst_csrrsi_w, // csrrsi
     inst_csrrci_w: // csrrci    
     begin
         alu_func_r     = `ALU_NONE;
         alu_input_a_r  = csr_data_r;
         write_rd_r     = 1'b1;
     end

     default:
        ;     
   endcase
end

//-------------------------------------------------------------------
// Load result resolve
//-------------------------------------------------------------------
always @ *
begin
    if (load_byte_q)
    begin
        case (load_offset_q[1:0])
            2'h3:
                load_result_r = {24'b0, mem_dat_i[31:24]};
            2'h2:
                load_result_r = {24'b0, mem_dat_i[23:16]};
            2'h1:
                load_result_r = {24'b0, mem_dat_i[15:8]};
            2'h0:
                load_result_r = {24'b0, mem_dat_i[7:0]};
        endcase

        if (load_signed_q && load_result_r[7])
            load_result_r = {24'hFFFFFF, load_result_r[7:0]};
    end
    else if (load_half_q)
    begin
        if (load_offset_q[1])
            load_result_r = {16'b0, mem_dat_i[31:16]};
        else
            load_result_r = {16'b0, mem_dat_i[15:0]};

        if (load_signed_q && load_result_r[15])
            load_result_r = {16'hFFFF, load_result_r[15:0]};
    end
    else
    begin
        load_result_r = mem_dat_i;
    end
end

//-----------------------------------------------------------------
// Branches
//-----------------------------------------------------------------
reg         branch_r;
reg [31:0]  branch_target_r;
reg         branch_except_r;

wire [31:0] pc_ext_w = {{PC_EXT_W{1'b0}}, pc_q};

always @ *
begin

    branch_r        = 1'b0;
    branch_except_r = 1'b0; 

    // Default branch target is relative to current PC
    branch_target_r = (pc_ext_w + bimm_r);    

    case (1'b1)
    inst_jal_w: // jal
    begin
        branch_r        = 1'b1;
        branch_target_r = pc_ext_w + jimm20_r;
    end

    inst_jalr_w: // jalr
    begin
        branch_r            = 1'b1;
        branch_target_r     = rs1_val_w + imm12_r;
        branch_target_r[0]  = 1'b0;
    end

    inst_beq_w: // beq
        branch_r      = (rs1_val_w == rs2_val_w);

    inst_bne_w: // bne
        branch_r      = (rs1_val_w != rs2_val_w);

    inst_blt_w: // blt
        branch_r      = less_than_signed(rs1_val_w, rs2_val_w);

    inst_bge_w: // bge
        branch_r      = greater_than_signed(rs1_val_w, rs2_val_w) | (rs1_val_w == rs2_val_w);

    inst_bltu_w: // bltu
        branch_r      = (rs1_val_w < rs2_val_w);

    inst_bgeu_w: // bgeu
        branch_r      = (rs1_val_w >= rs2_val_w);

    default:
        ;
    endcase
end

//-----------------------------------------------------------------
// Invalid instruction
//-----------------------------------------------------------------
always @ *
begin
    case (1'b1)
       inst_andi_w,
       inst_addi_w,
       inst_slti_w,
       inst_sltiu_w,
       inst_ori_w,
       inst_xori_w,
       inst_slli_w,
       inst_srli_w,
       inst_srai_w,
       inst_lui_w,
       inst_auipc_w,
       inst_add_w,
       inst_sub_w,
       inst_slt_w,
       inst_sltu_w,
       inst_xor_w,
       inst_or_w,
       inst_and_w,
       inst_sll_w,
       inst_srl_w,
       inst_sra_w,
       inst_jal_w,
       inst_jalr_w,
       inst_beq_w,
       inst_bne_w,
       inst_blt_w,
       inst_bge_w,
       inst_bltu_w,
       inst_bgeu_w,
       inst_lb_w,
       inst_lh_w,
       inst_lw_w,
       inst_lbu_w,
       inst_lhu_w,
       inst_lwu_w,
       inst_sb_w,
       inst_sh_w,
       inst_sw_w,
       inst_scall_w,
       inst_sbreak_w,
       inst_sret_w,
       inst_csrrw_w,
       inst_csrrs_w,
       inst_csrrc_w,
       inst_csrrwi_w,
       inst_csrrsi_w,
       inst_csrrci_w:
          invalid_inst_r = 1'b0;
       default:
          invalid_inst_r = 1'b1;
    endcase
end

//-----------------------------------------------------------------
// Execute: ALU control
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
begin
   if (rst_i == 1'b1)
   begin      
       alu_func_q   <= `ALU_NONE;
       alu_a_q      <= 32'h00000000;
       alu_b_q      <= 32'h00000000;
       rd_q         <= 5'b00000;
       rd_wr_en_q   <= 1'b0;
   end
   // Load result ready
   else if ((state_q == STATE_MEM) && mem_ack_i)
   begin
       // Update ALU input with load result
       alu_func_q   <= `ALU_NONE;
       alu_a_q      <= load_result_r;
       alu_b_q      <= 32'b0;
   end
   // Execute instruction
   else if (opcode_valid_w)
   begin
       // Update ALU input flops
       alu_func_q   <= alu_func_r;
       alu_a_q      <= alu_input_a_r;
       alu_b_q      <= alu_input_b_r;

       // Take exception
       if (exception_w)
       begin
           // Do not writeback, no operation
           rd_q         <= 5'b0;
           rd_wr_en_q   <= 1'b0;
       end   
       // Valid instruction
       else
       begin
           // Instruction with register writeback
           rd_q         <= rd_w;
           rd_wr_en_q   <= write_rd_r & (rd_w != 5'b0);
       end
   end
end

//-----------------------------------------------------------------
// Execute: Branch / exceptions
//-----------------------------------------------------------------
wire [31:0] boot_vector_w = BOOT_VECTOR;
wire [31:0] isr_vector_w  = ISR_VECTOR;

always @ (posedge clk_i or posedge rst_i)
begin
   if (rst_i == 1'b1)
   begin
       pc_q         <= boot_vector_w[PC_W-1:0];
   end
   // Writeback but CPU disabled
   else if ((state_q == STATE_WRITE_BACK) && !enable_i)
   begin
       pc_q         <= boot_vector_w[PC_W-1:0];
   end
   else if (opcode_valid_w)
   begin
       // Exception / Break / Scall
       if (exception_w || inst_sbreak_w || inst_scall_w) 
       begin
            // Perform branch to ISR vector
            pc_q    <= isr_vector_w[PC_W-1:0];
       end
       // SRET
       else if (inst_sret_w) 
       begin
            // Perform branch to EPC
            pc_q    <= csr_epc_r;
       end           
       // Branch
       else if (branch_r)
       begin
            // Perform branch
            pc_q    <= branch_target_r[PC_W-1:0];
       end
       else
            pc_q    <= pc_q + `PC_W'd4;
   end
end

//-----------------------------------------------------------------
// Execute: Break / Fault Status
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
begin
   if (rst_i == 1'b1)
   begin
       fault_o              <= 1'b0;
       break_o              <= 1'b0;
   end
   else
   begin
       // Instruction ready
       if (opcode_valid_w)
       begin
            // Invalid instruction
            if (invalid_inst_r)
                fault_o     <= 1'b1;

           // Break Instruction
           if (inst_sbreak_w) 
                break_o     <= 1'b1;
           else
                break_o     <= 1'b0;
      end
      else
          break_o     <= 1'b0;
   end
end

//-----------------------------------------------------------------
// Load/Store operation?
//-----------------------------------------------------------------
always @ *
begin
    load_inst_r  = inst_lb_w | inst_lh_w | inst_lw_w |
                   inst_lbu_w | inst_lhu_w | inst_lwu_w;
    store_inst_r = inst_sb_w  | inst_sh_w  | inst_sw_w;

    // Memory address is relative to RA
    mem_addr_r = rs1_val_w + (store_inst_r ? storeimm_r : imm12_r);
end

//-----------------------------------------------------------------
// Execute: Memory operations
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
begin
   if (rst_i == 1'b1)
   begin
       // Data memory
       mem_addr_q          <= {ADDR_W{1'b0}};
       mem_data_q          <= 32'h00000000;
       mem_we_q            <= 1'b0;
       mem_sel_q           <= 4'b0000;
       mem_stb_q           <= 1'b0;
   end
   // Perform instruction fetch
   else if (state_q == STATE_WRITE_BACK)
   begin
       mem_addr_q          <= {{PC_PAD_W{1'b0}}, pc_q};
       mem_sel_q           <= 4'b1111;
       mem_we_q            <= 1'b0;

       if (enable_i)
            mem_stb_q      <= 1'b1;
       else
            mem_stb_q      <= 1'b0;
   end
   // Valid instruction to execute
   else if (opcode_valid_w && !take_interrupt_w)
   begin
        case (1'b1)

        load_inst_r:
        begin
             mem_addr_q      <= {mem_addr_r[ADDR_W-1:2], 2'b0};
             mem_data_q      <= 32'h00000000;
             mem_sel_q       <= 4'b1111;
             mem_we_q        <= 1'b0;
             mem_stb_q       <= 1'b1;
        end

        inst_sb_w:
        begin
             mem_addr_q <= {mem_addr_r[ADDR_W-1:2], 2'b0};

             case (mem_addr_r[1:0])
                 2'h3 :
                 begin
                     mem_data_q      <= {rs2_val_w[7:0],24'h000000};
                     mem_sel_q       <= 4'b1000;
                     mem_we_q        <= 1'b1;
                     mem_stb_q       <= 1'b1;
                 end
                 2'h2 :
                 begin
                     mem_data_q      <= {{8'h00,rs2_val_w[7:0]},16'h0000};
                     mem_sel_q       <= 4'b0100;
                     mem_we_q        <= 1'b1;
                     mem_stb_q       <= 1'b1;
                 end
                 2'h1 :
                 begin
                     mem_data_q      <= {{16'h0000,rs2_val_w[7:0]},8'h00};
                     mem_sel_q       <= 4'b0010;
                     mem_we_q        <= 1'b1;
                     mem_stb_q       <= 1'b1;
                 end
                 2'h0 :
                 begin
                     mem_data_q      <= {24'h000000,rs2_val_w[7:0]};
                     mem_sel_q       <= 4'b0001;
                     mem_we_q        <= 1'b1;
                     mem_stb_q       <= 1'b1;
                 end
                 default :
                    ;
             endcase
        end

        inst_sh_w:
        begin
             mem_addr_q <= {mem_addr_r[ADDR_W-1:2], 2'b0};

             case (mem_addr_r[1:0])
                 2'h2 :
                 begin
                     mem_data_q      <= {rs2_val_w[15:0],16'h0000};
                     mem_sel_q       <= 4'b1100;
                     mem_we_q        <= 1'b1;
                     mem_stb_q       <= 1'b1;
                 end
                 default :
                 begin
                     mem_data_q      <= {16'h0000,rs2_val_w[15:0]};
                     mem_sel_q       <= 4'b0011;
                     mem_we_q        <= 1'b1;
                     mem_stb_q       <= 1'b1;
                 end
             endcase
        end

        inst_sw_w:
        begin
             mem_addr_q      <= {mem_addr_r[ADDR_W-1:2], 2'b0};
             mem_data_q      <= rs2_val_w;
             mem_sel_q       <= 4'b1111;
             mem_we_q        <= 1'b1;
             mem_stb_q       <= 1'b1;
        end

        // Non load / store
        default:
            mem_stb_q        <= 1'b0;

        endcase
    end
    // No instruction, clear memory request
    else if (!mem_stall_i)
        mem_stb_q           <= 1'b0;
end

always @ (posedge clk_i or posedge rst_i)
begin
   if (rst_i == 1'b1)
   begin
       load_signed_q  <= 1'b0;
       load_byte_q    <= 1'b0;
       load_half_q    <= 1'b0;
       load_offset_q  <= 2'b0;
   end
   // Valid instruction to execute
   else if (opcode_valid_w)
   begin
       load_signed_q  <= inst_lh_w | inst_lb_w;
       load_byte_q    <= inst_lb_w | inst_lbu_w;
       load_half_q    <= inst_lh_w | inst_lhu_w;
       load_offset_q  <= mem_addr_r[1:0];
   end
end

assign mem_addr_o = {{ADDR_PAD_W{1'b0}}, mem_addr_q};
assign mem_dat_o  = mem_data_q;
assign mem_sel_o  = mem_sel_q;
assign mem_cti_o  = 3'b111;
assign mem_we_o   = mem_we_q;
assign mem_stb_o  = mem_stb_q;

// CYC_O
reg mem_cyc_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i == 1'b1)
    mem_cyc_q <= 1'b0;
else if (mem_stb_o)
    mem_cyc_q <= 1'b1;
else if (mem_ack_i)
    mem_cyc_q <= 1'b0;

assign mem_cyc_o  = mem_stb_o | mem_cyc_q;

//-----------------------------------------------------------------
// Execute: CSR Access
//-----------------------------------------------------------------
always @ *
begin
    csr_epc_r    = csr_epc_q;
    csr_cause_r  = csr_cause_q;
    csr_sr_r     = csr_sr_q;

    // Execute instruction / exception
    if (opcode_valid_w)
    begin
        // Exception / break / scall
        if (exception_w || inst_sbreak_w || inst_scall_w) 
        begin
            // Save interrupt / supervisor state
            csr_sr_r[`SR_PEI] = csr_sr_q[`SR_EI];
            csr_sr_r[`SR_PS]  = csr_sr_q[`SR_S];

            // Disable interrupts and enter supervisor mode
            csr_sr_r[`SR_EI]  = 1'b0;
            csr_sr_r[`SR_S]   = 1'b1;

            // Save PC of next instruction (not yet executed)
            csr_epc_r         = pc_q;

            // Exception source
            if (invalid_inst_r)
                csr_cause_r   = `CAUSE_ILLEGAL_INSTRUCTION;
            else if (inst_sbreak_w)
                csr_cause_r   = `CAUSE_BREAKPOINT;
            else if (inst_scall_w)
                csr_cause_r   = `CAUSE_SYSCALL;
            else
                csr_cause_r   = `CAUSE_INTERRUPT;
        end
       // SRET
       else if (inst_sret_w) 
       begin
            // Restore interrupt / supervisor state
            csr_sr_r[`SR_EI] = csr_sr_q[`SR_PEI];
            csr_sr_r[`SR_S]  = csr_sr_q[`SR_PS];
       end  
        else
        begin
            case (csr_addr_o)
            `CSR_EPC:
            begin
                if (csr_set_o && csr_clr_o)
                    csr_epc_r = csr_data_o[PC_W-1:0];
                else if (csr_set_o)
                    csr_epc_r = csr_epc_r[PC_W-1:0] | csr_data_o[PC_W-1:0];
                else if (csr_clr_o)
                    csr_epc_r = csr_epc_r[PC_W-1:0] & ~csr_data_o[PC_W-1:0];
            end
            `CSR_CAUSE:
            begin
                if (csr_set_o && csr_clr_o)
                    csr_cause_r = csr_data_o[4:0];
                else if (csr_set_o)
                    csr_cause_r = csr_cause_r | csr_data_o[4:0];
                else if (csr_clr_o)
                    csr_cause_r = csr_cause_r & ~csr_data_o[4:0];
            end
            `CSR_STATUS:
            begin
                if (csr_set_o && csr_clr_o)
                    csr_sr_r = csr_data_o[3:0];
                else if (csr_set_o)
                    csr_sr_r = csr_sr_r | csr_data_o[3:0];
                else if (csr_clr_o)
                    csr_sr_r = csr_sr_r & ~csr_data_o[3:0];
            end
            default:
              ;
            endcase
        end
    end
end

always @ (posedge clk_i or posedge rst_i)
begin
   if (rst_i == 1'b1)
   begin
       csr_epc_q        <= `PC_W'b0;
       csr_cause_q      <= 5'b0;
       csr_sr_q         <= 4'b0;
       csr_sr_q[`SR_S]  <= 1'b1;
   end
   else
   begin

       csr_epc_q        <= csr_epc_r;
       csr_cause_q      <= csr_cause_r;
       csr_sr_q         <= csr_sr_r;
   end
end

// CSR Read Data MUX
always @ *
begin
    csr_data_r               = 32'b0;

    case (csr_addr_o)
    `CSR_EPC:
        csr_data_r[PC_W-1:0] = csr_epc_q;
    `CSR_CAUSE:
        csr_data_r[4:0]      = csr_cause_q;
    `CSR_STATUS:
        csr_data_r           = {csr_data_i[31:4], csr_sr_q};
    default:
        csr_data_r           = csr_data_i;
    endcase
end

assign csr_addr_o = opcode_valid_w ? imm12_r[11:0] : 12'b0;
assign csr_data_o = (inst_csrrwi_w || inst_csrrsi_w || inst_csrrci_w) ? {27'b0, rs1_w} : rs1_val_w;
assign csr_set_o  = (opcode_valid_w && !exception_w) ? (inst_csrrw_w || inst_csrrs_w || inst_csrrwi_w || inst_csrrsi_w): 1'b0;
assign csr_clr_o  = (opcode_valid_w && !exception_w) ? (inst_csrrw_w || inst_csrrc_w || inst_csrrwi_w || inst_csrrci_w): 1'b0;


//-------------------------------------------------------------------
// Debug
//-------------------------------------------------------------------
`ifdef SIMULATION
reg [79:0] dbg_inst_str;
reg [79:0] dbg_inst_rs1;
reg [79:0] dbg_inst_rs2;
reg [79:0] dbg_inst_rd;
reg [31:0] dbg_inst_imm;

reg [31:0] dbg_opcode_q;
reg [31:0] dbg_opcode_pc_q;

always @ (posedge clk_i or posedge rst_i)
begin
   if (rst_i == 1'b1)
   begin      
       dbg_opcode_q     <= 32'h00000000;
       dbg_opcode_pc_q  <= 32'h00000000;
   end
   else if (opcode_valid_w)
   begin
       // Take exception
       if (exception_w)
       begin
           // Store bubble opcode
           dbg_opcode_q    <= 32'h00000000;
           dbg_opcode_pc_q <= 32'h00000000;
       end   
       // Valid instruction
       else
       begin
           dbg_opcode_q    <= opcode_w;
           dbg_opcode_pc_q <= pc_ext_w;
       end
   end
end

always @ *
begin
    dbg_inst_str = "-";
    dbg_inst_rs1 = "-";
    dbg_inst_rs2 = "-";
    dbg_inst_rd  = "-";
    dbg_inst_imm = 32'b0;

    if (opcode_valid_w)
    begin
        dbg_inst_rs1 = get_regname_str(rs1_w);
        dbg_inst_rs2 = get_regname_str(rs2_w);
        dbg_inst_rd  = get_regname_str(rd_w);
        dbg_inst_imm = 32'b0;

        case (1'b1)
            inst_andi_w:    dbg_inst_str = "andi";
            inst_addi_w:    dbg_inst_str = "addi";
            inst_slti_w:    dbg_inst_str = "slti";
            inst_sltiu_w:   dbg_inst_str = "sltiu";
            inst_ori_w:     dbg_inst_str = "ori";
            inst_xori_w:    dbg_inst_str = "xori";
            inst_slli_w:    dbg_inst_str = "slli";
            inst_srli_w:    dbg_inst_str = "srli";
            inst_srai_w:    dbg_inst_str = "srai";
            inst_lui_w:     dbg_inst_str = "lui";
            inst_auipc_w:   dbg_inst_str = "auipc";
            inst_add_w:     dbg_inst_str = "add";
            inst_sub_w:     dbg_inst_str = "sub";
            inst_slt_w:     dbg_inst_str = "slt";
            inst_sltu_w:    dbg_inst_str = "sltu";
            inst_xor_w:     dbg_inst_str = "xor";
            inst_or_w:      dbg_inst_str = "or";
            inst_and_w:     dbg_inst_str = "and";
            inst_sll_w:     dbg_inst_str = "sll";
            inst_srl_w:     dbg_inst_str = "srl";
            inst_sra_w:     dbg_inst_str = "sra";
            inst_jal_w:     dbg_inst_str = "jal";
            inst_jalr_w:    dbg_inst_str = "jalr";
            inst_beq_w:     dbg_inst_str = "beq";
            inst_bne_w:     dbg_inst_str = "bne";
            inst_blt_w:     dbg_inst_str = "blt";
            inst_bge_w:     dbg_inst_str = "bge";
            inst_bltu_w:    dbg_inst_str = "bltu";
            inst_bgeu_w:    dbg_inst_str = "bgeu";
            inst_lb_w:      dbg_inst_str = "lb";
            inst_lh_w:      dbg_inst_str = "lh";
            inst_lw_w:      dbg_inst_str = "lw";
            inst_lbu_w:     dbg_inst_str = "lbu";
            inst_lhu_w:     dbg_inst_str = "lhu";
            inst_lwu_w:     dbg_inst_str = "lwu";
            inst_sb_w:      dbg_inst_str = "sb";
            inst_sh_w:      dbg_inst_str = "sh";
            inst_sw_w:      dbg_inst_str = "sw";
            inst_scall_w:   dbg_inst_str = "scall";
            inst_sbreak_w:  dbg_inst_str = "sbreak";
            inst_sret_w:    dbg_inst_str = "sret";
            inst_csrrw_w:   dbg_inst_str = "csrw";
            inst_csrrs_w:   dbg_inst_str = "csrs";
            inst_csrrc_w:   dbg_inst_str = "csrc";
            inst_csrrwi_w:  dbg_inst_str = "csrwi";
            inst_csrrsi_w:  dbg_inst_str = "csrsi";
            inst_csrrci_w:  dbg_inst_str = "csrci";            
        endcase

        case (1'b1)

            inst_addi_w,  // addi
            inst_andi_w,  // andi
            inst_slti_w,  // slti
            inst_sltiu_w, // sltiu
            inst_ori_w,   // ori
            inst_xori_w,  // xori
            inst_csrrw_w, // csrrw
            inst_csrrs_w, // csrrs
            inst_csrrc_w, // csrrc
            inst_csrrwi_w,// csrrwi
            inst_csrrsi_w,// csrrsi
            inst_csrrci_w:// csrrci            
            begin
                dbg_inst_rs2 = "-";
                dbg_inst_imm = imm12_r;
            end

            inst_slli_w, // slli
            inst_srli_w, // srli
            inst_srai_w: // srai
            begin
                dbg_inst_rs2 = "-";
                dbg_inst_imm = {27'b0, shamt_r};
            end

            inst_lui_w: // lui
            begin
                dbg_inst_rs1 = "-";
                dbg_inst_rs2 = "-";
                dbg_inst_imm = imm20_r;
            end
 
            inst_auipc_w: // auipc
            begin
                dbg_inst_rs1 = "pc";
                dbg_inst_rs2 = "-";
                dbg_inst_imm = imm20_r;
            end   

            inst_jal_w:  // jal
            begin
                dbg_inst_rs1 = "-";
                dbg_inst_rs2 = "-";
                dbg_inst_imm = pc_ext_w + jimm20_r;
            end

            inst_jalr_w: // jalr
            begin
                dbg_inst_rs2 = "-";
                dbg_inst_imm = rs1_val_w + imm12_r;
            end

            // lb lh lw lbu lhu lwu
            inst_lb_w,
            inst_lh_w,
            inst_lw_w,
            inst_lbu_w,
            inst_lhu_w,
            inst_lwu_w:
            begin
                dbg_inst_rs2 = "-";
                dbg_inst_imm = rs1_val_w + imm12_r;
            end 

            // sb sh sw
            inst_sb_w,
            inst_sh_w,
            inst_sw_w:
            begin
                dbg_inst_rd  = "-";
                dbg_inst_imm = rs1_val_w + {{20{opcode_w[31]}}, opcode_w[31:25], opcode_w[11:7]};
            end
        endcase
    end
end
`endif

//-------------------------------------------------------------------
// Hooks for debug
//-------------------------------------------------------------------
`ifdef verilator
   function [31:0] get_opcode_ex;
      // verilator public
      get_opcode_ex = opcode_valid_w ? dbg_opcode_q : 32'b0;
   endfunction
   function [31:0] get_pc_ex;
      // verilator public
      get_pc_ex = dbg_opcode_pc_q;
   endfunction   
   function [7:0] get_putc;
      // verilator public
      get_putc = 8'b0;
   endfunction
   function [0:0] get_reg_valid;
      // verilator public
      get_reg_valid = opcode_valid_w;
   endfunction
   function [4:0] get_reg_ra;
      // verilator public
      get_reg_ra = rs1_w;
   endfunction
   function [31:0] get_reg_ra_value;
      // verilator public
      get_reg_ra_value = rs1_val_w;
   endfunction
   function [4:0] get_reg_rb;
      // verilator public
      get_reg_rb = rs2_w;
   endfunction   
   function [31:0] get_reg_rb_value;
      // verilator public
      get_reg_rb_value = rs2_val_w;
   endfunction
`endif

endmodule
