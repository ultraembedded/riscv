//-----------------------------------------------------------------
// Module
//-----------------------------------------------------------------
module tb_top
( 
    input               clk_i /*verilator public*/,
    input               rst_i /*verilator public*/,
    output              fault_o /*verilator public*/,
    output              break_o /*verilator public*/
);

//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
parameter   BOOT_VECTOR         = 32'h00000000;
parameter   REGISTER_FILE_TYPE  = "SIMULATION";

//-----------------------------------------------------------------
// Wires
//-----------------------------------------------------------------
wire [31:0] imem_addr_w;
wire [31:0] imem_data_w;
wire        imem_stb_w;
wire        imem_cyc_w;
wire [2:0]  imem_cti_w;
wire        imem_stall_w;
wire        imem_ack_w;

wire [31:0] dmem_addr_w;
wire [31:0] dmem_data_w_w;
wire [31:0] dmem_addr_r_w;
wire [3:0]  dmem_sel_w;
wire        dmem_stb_w;
wire        dmem_cyc_w;
wire [2:0]  dmem_cti_w;
wire        dmem_we_w;
wire        dmem_stall_w;
wire        dmem_ack_w;

//-----------------------------------------------------------------
// DUT
//-----------------------------------------------------------------  
riscv
#(
    .BOOT_VECTOR(BOOT_VECTOR),
    .REGISTER_FILE_TYPE(REGISTER_FILE_TYPE)
)
u_cpu
(
    .clk_i(clk_i),
    .rst_i(rst_i),

    .intr_i(8'b0),

    // Status
    .fault_o(fault_o),
    .break_o(break_o),

    // Instruction memory 
    .imem_addr_o(imem_addr_w),
    .imem_dat_i(imem_data_w),
    .imem_cti_o(imem_cti_w),
    .imem_cyc_o(imem_cyc_w),
    .imem_stb_o(imem_stb_w),
    .imem_stall_i(imem_stall_w),
    .imem_ack_i(imem_ack_w),

    // Data Memory
    .dmem_addr_o(dmem_addr_w),
    .dmem_dat_o(dmem_data_w_w),
    .dmem_dat_i(dmem_addr_r_w),
    .dmem_sel_o(dmem_sel_w),
    .dmem_cti_o(dmem_cti_w),
    .dmem_cyc_o(dmem_cyc_w),
    .dmem_we_o(dmem_we_w),
    .dmem_stb_o(dmem_stb_w),
    .dmem_stall_i(dmem_stall_w),
    .dmem_ack_i(dmem_ack_w)
);

//-----------------------------------------------------------------
// I-RAM
//-----------------------------------------------------------------
wb_ram
u_iram
(
    .clk_i(clk_i),
    .rst_i(rst_i),

    // Wishbone I/F
    .addr_i(imem_addr_w),
    .data_i(32'b0),
    .data_o(imem_data_w),
    .sel_i(4'b1111),
    .cti_i(imem_cti_w),
    .cyc_i(imem_cyc_w),
    .we_i(1'b0),
    .stb_i(imem_stb_w),
    .stall_o(imem_stall_w),
    .ack_o(imem_ack_w) 
);

//-----------------------------------------------------------------
// D-RAM
//-----------------------------------------------------------------
wb_ram
u_dram
(
    .clk_i(clk_i),
    .rst_i(rst_i),

    // Wishbone I/F
    .addr_i(dmem_addr_w),
    .data_i(dmem_data_w_w),
    .data_o(dmem_addr_r_w),
    .sel_i(dmem_sel_w),
    .cti_i(dmem_cti_w),
    .cyc_i(dmem_cyc_w),
    .we_i(dmem_we_w),
    .stb_i(dmem_stb_w),
    .stall_o(dmem_stall_w),
    .ack_o(dmem_ack_w) 
);

endmodule
