//-----------------------------------------------------------------
// Module
//-----------------------------------------------------------------
module wb_slave
(
    input               clk_i, 
    input               rst_i, 

    // Wishbone I/F
    input [31:0]        addr_i,
    input [31:0]        data_i,
    output reg [31:0]   data_o,
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
parameter MAX_RESP_RATE     = 0;
parameter RESP_DELAY_RATE   = 3;

parameter RANDOM_STALLS     = 1;
parameter STALL_RATE        = 3;

parameter INC_READ_VAL      = 1;

//-----------------------------------------------------------------
// Request Queue
//-----------------------------------------------------------------
typedef struct
{
    reg [31:0] address;
    reg [31:0] data;
    reg [3:0]  sel;
    reg        we;
} request_t;

request_t requests[$];

//-----------------------------------------------------------------
// Memory
//-----------------------------------------------------------------
reg [31:0] mem[*];
reg [31:0] load_val;

//-----------------------------------------------------------------
// write
//-----------------------------------------------------------------
task automatic write(input [31:0] addr, input [31:0] data);
begin
    mem[addr[31:2]] = data;
end
endtask

//-----------------------------------------------------------------
// read
//-----------------------------------------------------------------
task automatic read(input [31:0] addr, output [31:0] data);
begin
    if (mem.exists(addr[31:2]))
        data = mem[addr[31:2]];
    else
        data = 32'bx;
end
endtask

//-----------------------------------------------------------------
// write_bytes
//-----------------------------------------------------------------
task automatic write_bytes(input [31:0] addr, input [31:0] data, input [31:0] sel);
begin
    reg [31:0] new_data;

    read(addr, new_data);

    if (sel[3])
        new_data[31:24] = data[31:24];
    if (sel[2])
        new_data[23:16] = data[23:16];
    if (sel[1])
        new_data[15:8]  = data[15:8];
    if (sel[0])
        new_data[7:0]   = data[7:0];

    write(addr, new_data);
end
endtask

//-----------------------------------------------------------------
// response
//-----------------------------------------------------------------
initial
begin
    request_t  req;

    ack_o  = 1'b0;
    data_o = 32'bz;

    load_val = 0;

    forever
    begin
        @(posedge clk_i);

        ack_o  = 1'b0;
        data_o = 32'bz;

        if (!MAX_RESP_RATE)
        begin
            repeat ($urandom_range(RESP_DELAY_RATE,0)) @(posedge clk_i);
        end

        if (requests.size() > 0)
        begin
            req = requests.pop_front();

            // Write
            if (req.we)
            begin
                ack_o  = 1'b1;
                data_o = 32'bz;
        
                write_bytes(req.address, req.data, req.sel);
            end
            // Read - incrementing value
            else if (INC_READ_VAL)
            begin
                ack_o  = 1'b1;
                
                data_o = 32'bx;

                if (req.sel[3])
                    data_o[31:24] = load_val[31:24];
                if (req.sel[2])
                    data_o[23:16] = load_val[23:16];
                if (req.sel[1])
                    data_o[15:8]  = load_val[15:8];
                if (req.sel[0])
                    data_o[7:0]   = load_val[7:0];

                load_val = load_val + 1;            
            end
            // Read
            else
            begin
                ack_o  = 1'b1;
                read(req.address, data_o);
            end
        end
    end
end

//-----------------------------------------------------------------
// request
//-----------------------------------------------------------------
always @(posedge clk_i)
begin
    // Request presented and accepted
    if (stb_i && cyc_i && !stall_o)
    begin
        request_t req;

        req.address = addr_i;
        req.data    = data_i;
        req.sel     = sel_i;
        req.we      = we_i;

        requests.push_back(req);
    end
end

always @(posedge rst_i or posedge clk_i)
if (rst_i)
    stall_o <= 1'b0;
else
begin
    if (RANDOM_STALLS)
        stall_o <= ($urandom_range(STALL_RATE,0) == 0);
end

//-----------------------------------------------------------------
// CTI
//-----------------------------------------------------------------
initial
begin
    reg        burst;
    reg [2:0]  burst_cnt;

    burst  = 1'b0;
    burst_cnt = 0;

    forever
    begin
        @(posedge clk_i);

        // Start of burst
        if (stb_i && cyc_i && !stall_o && cti_i == 3'b010)
        begin
            burst_cnt = 7;
            burst = 1;

            while (burst_cnt != 0)
            begin
                @(posedge clk_i);

                if (stb_i && cyc_i && !stall_o)
                    burst_cnt = burst_cnt - 1;
            end

            burst = 0;
        end
    end
end

endmodule
