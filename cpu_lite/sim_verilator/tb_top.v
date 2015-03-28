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
parameter   ISR_VECTOR          = 32'h00000000;
parameter   REGISTER_FILE_TYPE  = "SIMULATION";

//-----------------------------------------------------------------
// Wires
//-----------------------------------------------------------------  
wire [31:0] cpu_addr_w;
wire [31:0] cpu_data_w_w;
wire [31:0] cpu_addr_r_w;
wire [3:0]  cpu_sel_w;
wire        cpu_stb_w;
wire        cpu_cyc_w;
wire [2:0]  cpu_cti_w;
wire        cpu_we_w;
wire        cpu_stall_w;
wire        cpu_ack_w;

//-----------------------------------------------------------------
// DUT
//-----------------------------------------------------------------  
riscv
#(
    .BOOT_VECTOR(BOOT_VECTOR),
    .REGISTER_FILE_TYPE(REGISTER_FILE_TYPE),
    .ISR_VECTOR(ISR_VECTOR)
)
u_cpu
(
    .clk_i(clk_i),
    .rst_i(rst_i),

    .intr_i(8'b0),

    // Status
    .fault_o(fault_o),
    .break_o(break_o),

    // Memory
    .mem_addr_o(cpu_addr_w),
    .mem_dat_o(cpu_data_w_w),
    .mem_dat_i(cpu_addr_r_w),
    .mem_sel_o(cpu_sel_w),
    .mem_cti_o(cpu_cti_w),
    .mem_cyc_o(cpu_cyc_w),
    .mem_we_o(cpu_we_w),
    .mem_stb_o(cpu_stb_w),
    .mem_stall_i(cpu_stall_w),
    .mem_ack_i(cpu_ack_w)
);

//-----------------------------------------------------------------
// RAM
//-----------------------------------------------------------------
wb_ram
u_ram
(
    .clk_i(clk_i),
    .rst_i(rst_i),

    // Wishbone I/F
    .addr_i(cpu_addr_w),
    .data_i(cpu_data_w_w),
    .data_o(cpu_addr_r_w),
    .sel_i(cpu_sel_w),
    .cti_i(cpu_cti_w),
    .cyc_i(cpu_cyc_w),
    .we_i(cpu_we_w),
    .stb_i(cpu_stb_w),
    .stall_o(cpu_stall_w),
    .ack_o(cpu_ack_w) 
);

endmodule
