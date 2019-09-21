#include "tb_axi4_mem.h"
#include <queue>

//-----------------------------------------------------------------
// process: Handle AXI requests
//-----------------------------------------------------------------
void tb_axi4_mem::process(void)
{
    std::queue <axi4_master> axi_rd_q;
    std::queue <axi4_master> axi_wr_q;

    axi4_master axi_wr_req;

    while (1)
    {
        axi4_master axi_i = axi_in.read();
        axi4_slave  axi_o = axi_out.read();

        // Read command
        if (axi_i.ARVALID && axi_o.ARREADY)
        {
            sc_uint <AXI4_ADDR_W> next_addr = axi_i.ARADDR & ~calc_wrap_mask(0);
            axi4_master           axi_first = axi_i;

            // Unroll burst
            for (int i=0;i<((int)(axi_first.ARLEN) + 1);i++)
            {
                axi4_master item = axi_first;

                item.ARVALID  = true;
                item.ARADDR   = next_addr;
                item.WLAST    = (i == axi_first.ARLEN);

                axi_rd_q.push(item);

                // Generate next address
                next_addr = calc_next_addr(next_addr, axi_first.ARBURST, axi_first.ARLEN);
            }
        }

        // Write command
        if (axi_i.AWVALID && axi_o.AWREADY)
        {
            // Record command
            axi_wr_req = axi_i;
        }

        // Write data
        if (axi_i.WVALID && axi_o.WREADY)
        {
            sc_assert(axi_wr_req.AWVALID);

            axi4_master item = axi_wr_req;

            item.AWVALID  = true;
            item.AWADDR   = axi_wr_req.AWADDR;

            item.WVALID  = true;
            item.WDATA   = axi_i.WDATA;
            item.WSTRB   = axi_i.WSTRB;
            item.WLAST   = axi_i.WLAST;

            axi_wr_q.push(item);

            // Generate next address
            axi_wr_req.AWADDR = calc_next_addr(axi_wr_req.AWADDR, axi_wr_req.AWBURST, axi_wr_req.AWLEN);

            // Last item
            if (item.WLAST)
                axi_wr_req.AWVALID = false;
        }

        if (axi_o.RVALID && axi_i.RREADY)
        {
            axi_o.RVALID = false;
            axi_o.RDATA  = 0;
            axi_o.RID    = 0;
            axi_o.RRESP  = 0;
            axi_o.RLAST  = false;
        }

        if (!axi_o.RVALID && axi_rd_q.size() > 0 && !delay_cycle())
        {
            axi4_master item = axi_rd_q.front();
            axi_rd_q.pop();

            axi_o.RVALID = true;
            axi_o.RDATA  = read32((uint32_t)item.ARADDR);
            axi_o.RID    = item.ARID;
            axi_o.RLAST  = item.WLAST;
            axi_o.RRESP  = AXI4_RESP_OKAY;
        }

        if (axi_o.BVALID && axi_i.BREADY)
        {
            axi_o.BVALID = false;
            axi_o.BID    = 0;
            axi_o.BRESP  = 0;
        }

        if (!axi_o.BVALID && axi_wr_q.size() > 0 && !delay_cycle())
        {
            axi4_master item = axi_wr_q.front();
            axi_wr_q.pop();

            write32((uint32_t)item.AWADDR, (uint32_t)item.WDATA, (uint8_t)item.WSTRB);

            axi_o.BVALID = item.WLAST;
            axi_o.BID    = item.AWID;
            axi_o.BRESP  = AXI4_RESP_OKAY;
        }        

        // Randomize handshaking
        axi_o.ARREADY = !delay_cycle() && (axi_rd_q.size() < 128);
        axi_o.AWREADY = !delay_cycle() && (axi_wr_q.size() < 128);
        axi_o.WREADY  = axi_o.AWREADY && !delay_cycle();
        axi_o.AWREADY&= !axi_wr_req.AWVALID;

        axi_out.write(axi_o);

        wait();
    }
}
//-----------------------------------------------------------------
// calc_next_addr: Calculate next addr based on burst type
//-----------------------------------------------------------------
sc_uint <AXI4_ADDR_W> tb_axi4_mem::calc_next_addr(sc_uint <AXI4_ADDR_W> addr, sc_uint <AXI4_AXBURST_W> type, sc_uint <AXI4_AXLEN_W> len)
{
    sc_uint <AXI4_AXLEN_W> mask = calc_wrap_mask(len);

    switch (type)
    {
      case AXI4_BURST_WRAP:
          return (addr & ~mask) | ((addr + (AXI4_DATA_W/8)) & mask);
      case AXI4_BURST_INCR:
          return addr + (AXI4_DATA_W/8);
      case AXI4_BURST_FIXED:
      default:
          return addr;
    }

    return 0; // Invalid
}
//-----------------------------------------------------------------
// calc_wrap_mask: Calculate wrap mask for wrapping bursts
//-----------------------------------------------------------------
sc_uint <AXI4_AXLEN_W> tb_axi4_mem::calc_wrap_mask(sc_uint <AXI4_AXLEN_W> len)
{
    switch (len)
    {
      case (1 - 1):
          return 0x03;
      case (2 - 1):
          return 0x07;
      case (4 - 1):
          return 0x0F;
      case (8 - 1):
          return 0x1F;
      case (16 - 1):
      default:
          return 0x3F;
    }

    return 0; // Invalid
}
//-----------------------------------------------------------------
// write32: Write a 32-bit word to memory
//-----------------------------------------------------------------
void tb_axi4_mem::write32(uint32_t addr, uint32_t data, uint8_t strb)
{
    for (int i=0;i<4;i++)
        if (strb & (1 << i))
            tb_memory::write(addr + i,data >> (i*8));
}
//-----------------------------------------------------------------
// read32: Read a 32-bit word from memory
//-----------------------------------------------------------------
uint32_t tb_axi4_mem::read32(uint32_t addr)
{
    uint32_t data = 0;
    for (int i=0;i<4;i++)
        data |= ((uint32_t)tb_memory::read(addr + i)) << (i*8);
    return data;
}
//-----------------------------------------------------------------
// write: Byte write
//-----------------------------------------------------------------
void tb_axi4_mem::write(uint32_t addr, uint8_t data)
{
    tb_memory::write(addr, data);
}
//-----------------------------------------------------------------
// read: Byte read
//-----------------------------------------------------------------
uint8_t tb_axi4_mem::read(uint32_t addr)
{
    return tb_memory::read(addr);
}
