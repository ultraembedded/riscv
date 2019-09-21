
#include "riscv_top.h"
#include "Vriscv_top.h"

#if VM_TRACE
#include "verilated.h"
#include "verilated_vcd_c.h"
#endif

//-------------------------------------------------------------
// Constructor
//-------------------------------------------------------------
riscv_top::riscv_top(sc_module_name name): sc_module(name)
{
    m_rtl = new Vriscv_top("Vriscv_top");
    m_rtl->clk_i(m_clk_in);
    m_rtl->rst_i(m_rst_in);
    m_rtl->axi_i_awready_i(m_axi_i_awready_in);
    m_rtl->axi_i_wready_i(m_axi_i_wready_in);
    m_rtl->axi_i_bvalid_i(m_axi_i_bvalid_in);
    m_rtl->axi_i_bresp_i(m_axi_i_bresp_in);
    m_rtl->axi_i_bid_i(m_axi_i_bid_in);
    m_rtl->axi_i_arready_i(m_axi_i_arready_in);
    m_rtl->axi_i_rvalid_i(m_axi_i_rvalid_in);
    m_rtl->axi_i_rdata_i(m_axi_i_rdata_in);
    m_rtl->axi_i_rresp_i(m_axi_i_rresp_in);
    m_rtl->axi_i_rid_i(m_axi_i_rid_in);
    m_rtl->axi_i_rlast_i(m_axi_i_rlast_in);
    m_rtl->axi_d_awready_i(m_axi_d_awready_in);
    m_rtl->axi_d_wready_i(m_axi_d_wready_in);
    m_rtl->axi_d_bvalid_i(m_axi_d_bvalid_in);
    m_rtl->axi_d_bresp_i(m_axi_d_bresp_in);
    m_rtl->axi_d_bid_i(m_axi_d_bid_in);
    m_rtl->axi_d_arready_i(m_axi_d_arready_in);
    m_rtl->axi_d_rvalid_i(m_axi_d_rvalid_in);
    m_rtl->axi_d_rdata_i(m_axi_d_rdata_in);
    m_rtl->axi_d_rresp_i(m_axi_d_rresp_in);
    m_rtl->axi_d_rid_i(m_axi_d_rid_in);
    m_rtl->axi_d_rlast_i(m_axi_d_rlast_in);
    m_rtl->intr_i(m_intr_in);
    m_rtl->reset_vector_i(m_reset_vector_in);
    m_rtl->axi_i_awvalid_o(m_axi_i_awvalid_out);
    m_rtl->axi_i_awaddr_o(m_axi_i_awaddr_out);
    m_rtl->axi_i_awid_o(m_axi_i_awid_out);
    m_rtl->axi_i_awlen_o(m_axi_i_awlen_out);
    m_rtl->axi_i_awburst_o(m_axi_i_awburst_out);
    m_rtl->axi_i_wvalid_o(m_axi_i_wvalid_out);
    m_rtl->axi_i_wdata_o(m_axi_i_wdata_out);
    m_rtl->axi_i_wstrb_o(m_axi_i_wstrb_out);
    m_rtl->axi_i_wlast_o(m_axi_i_wlast_out);
    m_rtl->axi_i_bready_o(m_axi_i_bready_out);
    m_rtl->axi_i_arvalid_o(m_axi_i_arvalid_out);
    m_rtl->axi_i_araddr_o(m_axi_i_araddr_out);
    m_rtl->axi_i_arid_o(m_axi_i_arid_out);
    m_rtl->axi_i_arlen_o(m_axi_i_arlen_out);
    m_rtl->axi_i_arburst_o(m_axi_i_arburst_out);
    m_rtl->axi_i_rready_o(m_axi_i_rready_out);
    m_rtl->axi_d_awvalid_o(m_axi_d_awvalid_out);
    m_rtl->axi_d_awaddr_o(m_axi_d_awaddr_out);
    m_rtl->axi_d_awid_o(m_axi_d_awid_out);
    m_rtl->axi_d_awlen_o(m_axi_d_awlen_out);
    m_rtl->axi_d_awburst_o(m_axi_d_awburst_out);
    m_rtl->axi_d_wvalid_o(m_axi_d_wvalid_out);
    m_rtl->axi_d_wdata_o(m_axi_d_wdata_out);
    m_rtl->axi_d_wstrb_o(m_axi_d_wstrb_out);
    m_rtl->axi_d_wlast_o(m_axi_d_wlast_out);
    m_rtl->axi_d_bready_o(m_axi_d_bready_out);
    m_rtl->axi_d_arvalid_o(m_axi_d_arvalid_out);
    m_rtl->axi_d_araddr_o(m_axi_d_araddr_out);
    m_rtl->axi_d_arid_o(m_axi_d_arid_out);
    m_rtl->axi_d_arlen_o(m_axi_d_arlen_out);
    m_rtl->axi_d_arburst_o(m_axi_d_arburst_out);
    m_rtl->axi_d_rready_o(m_axi_d_rready_out);

    SC_METHOD(async_outputs);
    sensitive << clk_in;
    sensitive << rst_in;
    sensitive << intr_in;
    sensitive << reset_vector_in;
    sensitive << axi_i_in;
    sensitive << axi_d_in;
    sensitive << m_axi_i_awvalid_out;
    sensitive << m_axi_i_awaddr_out;
    sensitive << m_axi_i_awid_out;
    sensitive << m_axi_i_awlen_out;
    sensitive << m_axi_i_awburst_out;
    sensitive << m_axi_i_wvalid_out;
    sensitive << m_axi_i_wdata_out;
    sensitive << m_axi_i_wstrb_out;
    sensitive << m_axi_i_wlast_out;
    sensitive << m_axi_i_bready_out;
    sensitive << m_axi_i_arvalid_out;
    sensitive << m_axi_i_araddr_out;
    sensitive << m_axi_i_arid_out;
    sensitive << m_axi_i_arlen_out;
    sensitive << m_axi_i_arburst_out;
    sensitive << m_axi_i_rready_out;
    sensitive << m_axi_d_awvalid_out;
    sensitive << m_axi_d_awaddr_out;
    sensitive << m_axi_d_awid_out;
    sensitive << m_axi_d_awlen_out;
    sensitive << m_axi_d_awburst_out;
    sensitive << m_axi_d_wvalid_out;
    sensitive << m_axi_d_wdata_out;
    sensitive << m_axi_d_wstrb_out;
    sensitive << m_axi_d_wlast_out;
    sensitive << m_axi_d_bready_out;
    sensitive << m_axi_d_arvalid_out;
    sensitive << m_axi_d_araddr_out;
    sensitive << m_axi_d_arid_out;
    sensitive << m_axi_d_arlen_out;
    sensitive << m_axi_d_arburst_out;
    sensitive << m_axi_d_rready_out;

#if VM_TRACE
    m_vcd         = NULL;
    m_delay_waves = false;
#endif
}
//-------------------------------------------------------------
// trace_enable
//-------------------------------------------------------------
void riscv_top::trace_enable(VerilatedVcdC * p)
{
#if VM_TRACE
    m_vcd = p;
    m_rtl->trace (m_vcd, 99);
#endif
}
void riscv_top::trace_enable(VerilatedVcdC *p, sc_core::sc_time start_time)
{
#if VM_TRACE
    m_vcd = p;
    m_delay_waves = true;
    m_waves_start = start_time;
    //m_rtl->trace (m_vcd, 99);
#endif
}
//-------------------------------------------------------------
// async_outputs
//-------------------------------------------------------------
void riscv_top::async_outputs(void)
{
    m_clk_in.write(clk_in.read());
    m_rst_in.write(rst_in.read());
    m_intr_in.write(intr_in.read());
    m_reset_vector_in.write(reset_vector_in.read());

    axi4_slave axi_i_i = axi_i_in.read();
    m_axi_i_awready_in.write(axi_i_i.AWREADY); 
    m_axi_i_wready_in.write(axi_i_i.WREADY); 
    m_axi_i_bvalid_in.write(axi_i_i.BVALID); 
    m_axi_i_bresp_in.write(axi_i_i.BRESP); 
    m_axi_i_bid_in.write(axi_i_i.BID); 
    m_axi_i_arready_in.write(axi_i_i.ARREADY); 
    m_axi_i_rvalid_in.write(axi_i_i.RVALID); 
    m_axi_i_rdata_in.write(axi_i_i.RDATA); 
    m_axi_i_rresp_in.write(axi_i_i.RRESP); 
    m_axi_i_rid_in.write(axi_i_i.RID); 
    m_axi_i_rlast_in.write(axi_i_i.RLAST); 


    axi4_master axi_i_o;
    axi_i_o.AWVALID = m_axi_i_awvalid_out.read(); 
    axi_i_o.AWADDR = m_axi_i_awaddr_out.read(); 
    axi_i_o.AWID = m_axi_i_awid_out.read(); 
    axi_i_o.AWLEN = m_axi_i_awlen_out.read(); 
    axi_i_o.AWBURST = m_axi_i_awburst_out.read(); 
    axi_i_o.WVALID = m_axi_i_wvalid_out.read(); 
    axi_i_o.WDATA = m_axi_i_wdata_out.read(); 
    axi_i_o.WSTRB = m_axi_i_wstrb_out.read(); 
    axi_i_o.WLAST = m_axi_i_wlast_out.read(); 
    axi_i_o.BREADY = m_axi_i_bready_out.read(); 
    axi_i_o.ARVALID = m_axi_i_arvalid_out.read(); 
    axi_i_o.ARADDR = m_axi_i_araddr_out.read(); 
    axi_i_o.ARID = m_axi_i_arid_out.read(); 
    axi_i_o.ARLEN = m_axi_i_arlen_out.read(); 
    axi_i_o.ARBURST = m_axi_i_arburst_out.read(); 
    axi_i_o.RREADY = m_axi_i_rready_out.read(); 
    axi_i_out.write(axi_i_o);
    axi4_slave axi_d_i = axi_d_in.read();
    m_axi_d_awready_in.write(axi_d_i.AWREADY); 
    m_axi_d_wready_in.write(axi_d_i.WREADY); 
    m_axi_d_bvalid_in.write(axi_d_i.BVALID); 
    m_axi_d_bresp_in.write(axi_d_i.BRESP); 
    m_axi_d_bid_in.write(axi_d_i.BID); 
    m_axi_d_arready_in.write(axi_d_i.ARREADY); 
    m_axi_d_rvalid_in.write(axi_d_i.RVALID); 
    m_axi_d_rdata_in.write(axi_d_i.RDATA); 
    m_axi_d_rresp_in.write(axi_d_i.RRESP); 
    m_axi_d_rid_in.write(axi_d_i.RID); 
    m_axi_d_rlast_in.write(axi_d_i.RLAST); 


    axi4_master axi_d_o;
    axi_d_o.AWVALID = m_axi_d_awvalid_out.read(); 
    axi_d_o.AWADDR = m_axi_d_awaddr_out.read(); 
    axi_d_o.AWID = m_axi_d_awid_out.read(); 
    axi_d_o.AWLEN = m_axi_d_awlen_out.read(); 
    axi_d_o.AWBURST = m_axi_d_awburst_out.read(); 
    axi_d_o.WVALID = m_axi_d_wvalid_out.read(); 
    axi_d_o.WDATA = m_axi_d_wdata_out.read(); 
    axi_d_o.WSTRB = m_axi_d_wstrb_out.read(); 
    axi_d_o.WLAST = m_axi_d_wlast_out.read(); 
    axi_d_o.BREADY = m_axi_d_bready_out.read(); 
    axi_d_o.ARVALID = m_axi_d_arvalid_out.read(); 
    axi_d_o.ARADDR = m_axi_d_araddr_out.read(); 
    axi_d_o.ARID = m_axi_d_arid_out.read(); 
    axi_d_o.ARLEN = m_axi_d_arlen_out.read(); 
    axi_d_o.ARBURST = m_axi_d_arburst_out.read(); 
    axi_d_o.RREADY = m_axi_d_rready_out.read(); 
    axi_d_out.write(axi_d_o);

}
