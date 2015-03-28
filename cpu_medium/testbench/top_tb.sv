`timescale 100ps/100ps

//-----------------------------------------------------------------
// Module
//-----------------------------------------------------------------
module top_tb ;

//-----------------------------------------------------------------
// Simulation
//-----------------------------------------------------------------
`include "simulation.svh"

`CLOCK_GEN(clk, 100)
`RESET_GEN(rst, 100)

`ifdef TRACE
    `TB_VCD(top_tb, "waveform.vcd")
`endif

`TB_RUN_FOR(`RUN_TIME)

//-----------------------------------------------------------------
// Registers / Wires
//-----------------------------------------------------------------
// Decode opcode / PC / state
wire [31:0] dec_opcode_w;
wire [31:0] dec_opcode_pc_w;
wire        dec_opcode_valid_w;

// Register numbers
wire [4:0]  dec_rs1_w;
wire [4:0]  dec_rs2_w;

// Destination register number (pre execute stage)
wire [4:0]  dec_rd_w;

// Register value (rA)
wire [31:0] dec_rs1_val_w;
wire [31:0] resolved_rs1_val_w;

// Register value (rB)
wire [31:0] dec_rs2_val_w;
wire [31:0] resolved_rs2_val_w;

// Destination register number (post execute stage)
wire [4:0]  ex_rd_w;

// Current executing instruction
wire [31:0] ex_opcode_w;
wire [31:0] ex_opcode_pc_w;

// Result from execute
wire [31:0] ex_result_w;

// Branch request
wire        ex_branch_w;
wire [31:0] ex_branch_pc_w;
wire        ex_stall_w;

// Load Store stall
wire        lsu_stall_exec_w;
wire        lsu_stall_mem_w;

wire        lsu_squash_w;

// Load result steering
wire [31:0] load_result_w;
wire [4:0]  load_dest_w;
wire        load_accept_w;

// Register writeback value
wire [4:0]  wb_rd_w;
wire [31:0] wb_rd_val_w;

// Register writeback enable
wire        wb_rd_write_w;

wire [31:0] mem_addr;
wire [31:0] mem_data_w;
wire [31:0] mem_data_r;
wire [3:0]  mem_sel;
wire        mem_stb;
wire        mem_we;
wire        mem_cyc;
wire [2:0]  mem_cti;
wire        mem_stall;
wire        mem_ack;

wire [31:0] dcache_addr_w;
wire [31:0] dcache_data_out_w;
wire [31:0] dcache_data_in_w;
wire [3:0]  dcache_sel_w;
wire        dcache_we_w;
wire        dcache_stb_w;
wire        dcache_stall_w;
wire        dcache_ack_w;

wire        status_fault;
wire        status_break;

reg         intr;

//-----------------------------------------------------------------
// Instantiation
//-----------------------------------------------------------------

// Instruction Generator
inst_gen 
u_gen
(
    // General
    .clk_i(clk),
    .rst_i(rst),
    
    // Fetched opcode
    .opcode_o(dec_opcode_w),
    .opcode_pc_o(dec_opcode_pc_w),
    .opcode_valid_o(dec_opcode_valid_w),
    
    // Branch target
    .branch_i(ex_branch_w),
    .branch_pc_i(ex_branch_pc_w),    
    .stall_i(ex_stall_w),

    // Decoded register details
    .rs1_o(dec_rs1_w),
    .rs2_o(dec_rs2_w),
    .rd_o(dec_rd_w)
);

riscv_regfile_sim
u_regfile
(
    // Clocking
    .clk_i(clk),
    .rst_i(rst),
    .wr_i(wb_rd_write_w),

    // Tri-port
    .rs1_i(dec_rs1_w),
    .rs2_i(dec_rs2_w),
    .rd_i(wb_rd_w),
    .reg_rs1_o(dec_rs1_val_w),
    .reg_rs2_o(dec_rs2_val_w),
    .reg_rd_i(wb_rd_val_w)
);

riscv_exec
#(
    .BOOT_VECTOR('h100)
)
u_exec
(
    // General
    .clk_i(clk),
    .rst_i(rst),

    .intr_i(intr),
    .break_i(1'b0),
    
    // Status
    .fault_o(status_fault),
    .break_o(status_break),

    // Branch target
    .branch_o(ex_branch_w),
    .branch_pc_o(ex_branch_pc_w),
    .stall_o(ex_stall_w),

    // Opcode & arguments
    .opcode_i(dec_opcode_w),
    .opcode_pc_i(dec_opcode_pc_w),
    .opcode_valid_i(dec_opcode_valid_w),

    // Operands
    .reg_rs1_i(dec_rs1_w),
    .reg_rs1_value_i(dec_rs1_val_w),
    .reg_rs2_i(dec_rs2_w),
    .reg_rs2_value_i(dec_rs2_val_w),
    .reg_rd_i(dec_rd_w),

    // Resolved operands
    .reg_rs1_value_o(resolved_rs1_val_w),
    .reg_rs2_value_o(resolved_rs2_val_w),

    // Load store stall
    .stall_exec_i(lsu_stall_exec_w),

    // Stall from mem stage
    .stall_mem_i(lsu_stall_mem_w),

    // Opcode not being executed (to other stages)
    .squash_opcode_o(lsu_squash_w),

    // CSR access
    .csr_addr_o(),
    .csr_data_o(),
    .csr_data_i(32'b0),
    .csr_set_o(),
    .csr_clr_o(),

    // Output
    .opcode_o(ex_opcode_w),
    .opcode_pc_o(ex_opcode_pc_w),
    .reg_rd_o(ex_rd_w),
    .reg_rd_value_o(ex_result_w),

    // Register write back bypass
    .wb_rd_i(wb_rd_w),
    .wb_rd_value_i(wb_rd_val_w)
);

riscv_mem
u_mem
(
    .clk_i(clk),
    .rst_i(rst),

    // Opcode & arguments
    .opcode_i(dec_opcode_w),
    .opcode_valid_i(dec_opcode_valid_w),

    // Opcode not being executed (to other stages)
    .squash_opcode_i(lsu_squash_w),

    // Operands
    .reg_rs1_i(dec_rs1_w),
    .reg_rs1_value_i(resolved_rs1_val_w),
    .reg_rs2_i(dec_rs2_w),
    .reg_rs2_value_i(resolved_rs2_val_w),
    .reg_rd_i(dec_rd_w),

    // Load result to writeback
    .load_result_o(load_result_w),
    .load_dest_o(load_dest_w),
    .load_accept_i(load_accept_w),

    // Load stall
    .stall_exec_o(lsu_stall_exec_w),
    .stall_mem_o(lsu_stall_mem_w),

    // Memory Interface
    .dmem_addr_o(dcache_addr_w),
    .dmem_data_out_o(dcache_data_out_w),
    .dmem_data_in_i(dcache_data_in_w),
    .dmem_sel_o(dcache_sel_w),
    .dmem_we_o(dcache_we_w),
    .dmem_stb_o(dcache_stb_w),
    .dmem_stall_i(dcache_stall_w),
    .dmem_ack_i(dcache_ack_w) 
);

riscv_dmem_nocache
u_nodcache
(
    .clk_i(clk),
    .rst_i(rst),

    // CPU Interface
    .cpu_addr_i(dcache_addr_w),
    .cpu_data_out_i(dcache_data_out_w),
    .cpu_data_in_o(dcache_data_in_w),
    .cpu_sel_i(dcache_sel_w),
    .cpu_we_i(dcache_we_w),
    .cpu_stb_i(dcache_stb_w),
    .cpu_stall_o(dcache_stall_w),
    .cpu_ack_o(dcache_ack_w),

    // Memory Interface
    .dmem_addr_o(mem_addr),
    .dmem_data_out_o(mem_data_w),
    .dmem_data_in_i(mem_data_r),
    .dmem_sel_o(mem_sel),
    .dmem_we_o(mem_we),
    .dmem_stb_o(mem_stb),
    .dmem_cti_o(mem_cti),
    .dmem_cyc_o(mem_cyc),
    .dmem_stall_i(mem_stall),
    .dmem_ack_i(mem_ack)
);

riscv_writeback 
u_wb
(
    // General
    .clk_i(clk),
    .rst_i(rst),

    // Stall from mem stage
    .stall_mem_i(lsu_stall_mem_w),

    // Opcode
    .opcode_i(ex_opcode_w),

    // Register target
    .rd_i(ex_rd_w),
    
    // ALU result
    .alu_result_i(ex_result_w),

    // Memory load result
    .load_result_i(load_result_w),
    .load_dest_i(load_dest_w),
    .load_accept_o(load_accept_w),

    // Outputs
    .write_enable_o(wb_rd_write_w),
    .write_addr_o(wb_rd_w),
    .write_data_o(wb_rd_val_w)
);

wb_slave
#(
    .RANDOM_STALLS(1),
    .MAX_RESP_RATE(0)
)
u_wbs
(
    .clk_i(clk),
    .rst_i(rst),
    
    .addr_i(mem_addr), 
    .data_i(mem_data_w), 
    .data_o(mem_data_r), 
    .sel_i(mem_sel), 
    .cyc_i(mem_cyc), 
    .stb_i(mem_stb), 
    .cti_i(mem_cti),
    .we_i(mem_we), 
    .stall_o(mem_stall),
    .ack_o(mem_ack)         
);

//-----------------------------------------------------------------
// Simulation Displays
//-----------------------------------------------------------------
`include "../cpu/riscv_defs.v"
`include "../cpu/riscv_funcs.v"

reg [79:0] dbg_load_reg;
reg [79:0] dbg_wb_reg;

always @ *
begin
    dbg_load_reg = load_dest_w != 0 ? get_regname_str(load_dest_w) : "-";
    dbg_wb_reg   = wb_rd_w != 0 ? get_regname_str(wb_rd_w) : "-";
end

//-----------------------------------------------------------------
// Random Interrupts
//-----------------------------------------------------------------
always @(posedge clk or posedge rst)
if (rst)
    intr  <= 1'b0;
else
    intr <= ($urandom_range(1000,0) == 0);

//-----------------------------------------------------------------
// VPI Monitor
//-----------------------------------------------------------------
always @(posedge clk)
begin
    $monitor_csr_internal(u_exec.csr_epc_q, u_exec.csr_evec_q, u_exec.csr_cause_q, u_exec.csr_sr_q);
    $monitor_status(status_break, status_fault);
    $monitor_writeback(ex_rd_w != 0, ex_rd_w, ex_result_w);

    $monitor_fetch(dec_opcode_valid_w & !ex_stall_w, dec_opcode_pc_w, dec_opcode_w, intr);    
end

endmodule
