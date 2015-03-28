//-----------------------------------------------------------------
// Module: wb_ram
//-----------------------------------------------------------------
module wb_ram
(
    input               clk_i, 
    input               rst_i, 

    // Wishbone I/F
    input [31:0]        addr_i,
    input [31:0]        data_i,
    output [31:0]       data_o,
    input [3:0]         sel_i,
    input               cyc_i,
    input               stb_i,
    input [2:0]         cti_i,
    input               we_i,
    output reg          ack_o,
    output reg          stall_o
);

//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
parameter  [31:0]       SIZE = 14;

//-----------------------------------------------------------------
// Registers
//-----------------------------------------------------------------
reg [31:0]              ram [((2<< (SIZE-1)) - 1):0] /*verilator public*/;
wire [(SIZE - 1):0]     addr_w;
reg [(SIZE - 1):0]      addr_q;
reg                     ack_q;

//-----------------------------------------------------------------
// Processes
//-----------------------------------------------------------------
assign addr_w   = addr_i[SIZE+1:2];

always @ (posedge clk_i or posedge rst_i)
if (rst_i == 1'b1)
    addr_q   <= {SIZE{1'b0}};
else if (cyc_i & stb_i)
    addr_q   <= addr_w;

always @ (posedge clk_i)
begin
    if (cyc_i & stb_i & we_i)
    begin
        if (sel_i[3])
            ram[addr_w][31:24] <= data_i[31:24];
        if (sel_i[2])
            ram[addr_w][23:16] <= data_i[23:16];
        if (sel_i[1])
            ram[addr_w][15:8] <= data_i[15:8];
        if (sel_i[0])
            ram[addr_w][7:0] <= data_i[7:0];
    end
end

// ACK
always @ (posedge clk_i or posedge rst_i)
if (rst_i == 1'b1)
    ack_q <= 1'b0;
else
    ack_q <= cyc_i & stb_i;

//-------------------------------------------------------------------
// Combinatorial
//-------------------------------------------------------------------
assign data_o  = ram[addr_q];
assign stall_o = 1'b0;
assign ack_o   = ack_q;

endmodule
