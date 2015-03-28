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
wire [31:0] mem_addr;
wire [31:0] mem_data_w;
wire [31:0] mem_data_r;
wire [3:0]  mem_sel;
wire        mem_stb;
wire        mem_we;
wire        mem_stall;
wire        mem_ack;

wire        status_fault;
wire        status_break;

reg         intr;

wire        inst_fetch;

wire [31:0] imem_data;

reg [31:0]  csr_mem[4095:0];

wire [11:0] csr_addr_w;
wire [31:0] csr_data_w;
reg [31:0]  csr_data_r;
wire        csr_set_w;
wire        csr_clr_w;

// Instruction fetch detection
reg inst_resp_q;
always @(posedge clk or posedge rst)
if (rst)
    inst_resp_q <= 1'b0;
else if (u_exec.opcode_fetch_w)
    inst_resp_q <= 1'b1;
else if (mem_ack)
    inst_resp_q <= 1'b0;

// Instruction fetch detection
reg [31:0] inst_addr_q;
always @(posedge clk or posedge rst)
if (rst)
    inst_addr_q <= 32'b0;
else if (u_exec.opcode_fetch_w)
    inst_addr_q <= mem_addr;


//-----------------------------------------------------------------
// Instantiation
//-----------------------------------------------------------------

// Instruction Generator
inst_gen 
u_gen
(
    .clk_i(clk),
    .rst_i(rst),
    
    .data_o(imem_data)
);

riscv_core
#(
    .BOOT_VECTOR('h100)
)
u_exec
(
    // General
    .clk_i(clk),
    .rst_i(rst),

    .enable_i(1'b1),

    .intr_i(intr),
    .nmi_i(1'b0),
    
    // Status
    .fault_o(status_fault),
    .break_o(status_break),

    // Memory Interface
    .mem_addr_o(mem_addr),
    .mem_dat_o(mem_data_w),
    .mem_dat_i(inst_resp_q ? imem_data : mem_data_r),
    .mem_sel_o(mem_sel),
    .mem_we_o(mem_we),
    .mem_stb_o(mem_stb),
    .mem_cti_o(),
    .mem_cyc_o(),
    .mem_stall_i(mem_stall),
    .mem_ack_i(mem_ack),

    // CSR access
    .csr_addr_o(csr_addr_w),
    .csr_data_o(csr_data_w),
    .csr_data_i(csr_data_r),
    .csr_set_o(csr_set_w),
    .csr_clr_o(csr_clr_w)
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
    
    .addr_i({mem_addr[31:2], 2'b0}), 
    .data_i(mem_data_w), 
    .data_o(mem_data_r), 
    .sel_i(mem_sel), 
    .cyc_i(1'b1), 
    .stb_i(mem_stb), 
    .cti_i(3'b111),
    .we_i(mem_we), 
    .stall_o(mem_stall),
    .ack_o(mem_ack)         
);

//-----------------------------------------------------------------
// CSR Memory Stub
//-----------------------------------------------------------------
initial
begin
    integer j;
    for (j=0;j<4096;j=j+1)
        csr_mem[j] = 0;
end

always @ *
begin
    if (csr_addr_w == 12'h50a)
        csr_data_r = 32'b0;
    else
        csr_data_r = csr_mem[csr_addr_w];
end    

always @(posedge clk)
begin
    if (csr_set_w && csr_clr_w)
        csr_mem[csr_addr_w] <= csr_data_w;
    else if (csr_set_w)
        csr_mem[csr_addr_w] <= csr_mem[csr_addr_w] |  csr_data_w;
    else if (csr_clr_w)
        csr_mem[csr_addr_w] <= csr_mem[csr_addr_w] & ~csr_data_w;
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
    $monitor_csr_internal(u_exec.csr_epc_q, 32'b0, u_exec.csr_cause_q, u_exec.csr_sr_q);
    $monitor_status(status_break, status_fault);
    $monitor_writeback(u_exec.rd_writeen_w, u_exec.rd_q, u_exec.rd_val_w);

    $monitor_fetch(inst_resp_q & mem_ack, inst_addr_q, imem_data, intr);
end

endmodule
