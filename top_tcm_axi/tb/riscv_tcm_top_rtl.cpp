#include "riscv_tcm_top_rtl.h"
#include "Vriscv_tcm_top.h"

#if VM_TRACE
#include "verilated.h"
#include "verilated_vcd_c.h"
#endif

//-------------------------------------------------------------
// Constructor
//-------------------------------------------------------------
riscv_tcm_top_rtl::riscv_tcm_top_rtl(sc_module_name name): sc_module(name)
{
    m_rtl = new Vriscv_tcm_top("Vriscv_tcm_top");
    m_rtl->clk_i(m_clk_in);
    m_rtl->rst_i(m_rst_in);
    m_rtl->rst_cpu_i(m_rst_cpu_in);
    m_rtl->axi_i_awready_i(m_axi_i_awready_in);
    m_rtl->axi_i_wready_i(m_axi_i_wready_in);
    m_rtl->axi_i_bvalid_i(m_axi_i_bvalid_in);
    m_rtl->axi_i_bresp_i(m_axi_i_bresp_in);
    m_rtl->axi_i_arready_i(m_axi_i_arready_in);
    m_rtl->axi_i_rvalid_i(m_axi_i_rvalid_in);
    m_rtl->axi_i_rdata_i(m_axi_i_rdata_in);
    m_rtl->axi_i_rresp_i(m_axi_i_rresp_in);
    m_rtl->axi_t_awvalid_i(m_axi_t_awvalid_in);
    m_rtl->axi_t_awaddr_i(m_axi_t_awaddr_in);
    m_rtl->axi_t_awid_i(m_axi_t_awid_in);
    m_rtl->axi_t_awlen_i(m_axi_t_awlen_in);
    m_rtl->axi_t_awburst_i(m_axi_t_awburst_in);
    m_rtl->axi_t_wvalid_i(m_axi_t_wvalid_in);
    m_rtl->axi_t_wdata_i(m_axi_t_wdata_in);
    m_rtl->axi_t_wstrb_i(m_axi_t_wstrb_in);
    m_rtl->axi_t_wlast_i(m_axi_t_wlast_in);
    m_rtl->axi_t_bready_i(m_axi_t_bready_in);
    m_rtl->axi_t_arvalid_i(m_axi_t_arvalid_in);
    m_rtl->axi_t_araddr_i(m_axi_t_araddr_in);
    m_rtl->axi_t_arid_i(m_axi_t_arid_in);
    m_rtl->axi_t_arlen_i(m_axi_t_arlen_in);
    m_rtl->axi_t_arburst_i(m_axi_t_arburst_in);
    m_rtl->axi_t_rready_i(m_axi_t_rready_in);
    m_rtl->intr_i(m_intr_in);
    m_rtl->axi_i_awvalid_o(m_axi_i_awvalid_out);
    m_rtl->axi_i_awaddr_o(m_axi_i_awaddr_out);
    m_rtl->axi_i_wvalid_o(m_axi_i_wvalid_out);
    m_rtl->axi_i_wdata_o(m_axi_i_wdata_out);
    m_rtl->axi_i_wstrb_o(m_axi_i_wstrb_out);
    m_rtl->axi_i_bready_o(m_axi_i_bready_out);
    m_rtl->axi_i_arvalid_o(m_axi_i_arvalid_out);
    m_rtl->axi_i_araddr_o(m_axi_i_araddr_out);
    m_rtl->axi_i_rready_o(m_axi_i_rready_out);
    m_rtl->axi_t_awready_o(m_axi_t_awready_out);
    m_rtl->axi_t_wready_o(m_axi_t_wready_out);
    m_rtl->axi_t_bvalid_o(m_axi_t_bvalid_out);
    m_rtl->axi_t_bresp_o(m_axi_t_bresp_out);
    m_rtl->axi_t_bid_o(m_axi_t_bid_out);
    m_rtl->axi_t_arready_o(m_axi_t_arready_out);
    m_rtl->axi_t_rvalid_o(m_axi_t_rvalid_out);
    m_rtl->axi_t_rdata_o(m_axi_t_rdata_out);
    m_rtl->axi_t_rresp_o(m_axi_t_rresp_out);
    m_rtl->axi_t_rid_o(m_axi_t_rid_out);
    m_rtl->axi_t_rlast_o(m_axi_t_rlast_out);

    SC_METHOD(async_outputs);
    sensitive << clk_in;
    sensitive << rst_in;
    sensitive << rst_cpu_in;
    sensitive << intr_in;
    sensitive << axi_i_in;
    sensitive << axi_t_in;
    sensitive << m_axi_i_awvalid_out;
    sensitive << m_axi_i_awaddr_out;
    sensitive << m_axi_i_wvalid_out;
    sensitive << m_axi_i_wdata_out;
    sensitive << m_axi_i_wstrb_out;
    sensitive << m_axi_i_bready_out;
    sensitive << m_axi_i_arvalid_out;
    sensitive << m_axi_i_araddr_out;
    sensitive << m_axi_i_rready_out;
    sensitive << m_axi_t_awready_out;
    sensitive << m_axi_t_wready_out;
    sensitive << m_axi_t_bvalid_out;
    sensitive << m_axi_t_bresp_out;
    sensitive << m_axi_t_bid_out;
    sensitive << m_axi_t_arready_out;
    sensitive << m_axi_t_rvalid_out;
    sensitive << m_axi_t_rdata_out;
    sensitive << m_axi_t_rresp_out;
    sensitive << m_axi_t_rid_out;
    sensitive << m_axi_t_rlast_out;

#if VM_TRACE
    m_vcd         = NULL;
    m_delay_waves = false;
    SC_METHOD(trace_rtl);
    sensitive << clk_in;
#endif
}
//-------------------------------------------------------------
// trace_rtl
//-------------------------------------------------------------
void riscv_tcm_top_rtl::trace_rtl(void)
{
#if VM_TRACE
    if (m_delay_waves)
    {
        if (sc_time_stamp() > m_waves_start)
        {
            cout << "WAVES: Delayed start reached - " << sc_time_stamp() << endl;
            m_delay_waves = false;
        }
    }
    else if (m_vcd)
        m_vcd->dump((int)(sc_time_stamp().to_double()));
#endif
}
//-------------------------------------------------------------
// trace_enable
//-------------------------------------------------------------
void riscv_tcm_top_rtl::trace_enable(VerilatedVcdC * p)
{
#if VM_TRACE
    m_vcd = p;
    m_rtl->trace (m_vcd, 99);
#endif
}
void riscv_tcm_top_rtl::trace_enable(VerilatedVcdC *p, sc_core::sc_time start_time)
{
#if VM_TRACE
    m_vcd = p;
    m_delay_waves = true;
    m_waves_start = start_time;
    m_rtl->trace (m_vcd, 99);
#endif
}
//-------------------------------------------------------------
// async_outputs
//-------------------------------------------------------------
void riscv_tcm_top_rtl::async_outputs(void)
{
    m_clk_in.write(clk_in.read());
    m_rst_in.write(rst_in.read());
    m_rst_cpu_in.write(rst_cpu_in.read());
    m_intr_in.write(intr_in.read());

    axi4_lite_slave axi_i_i = axi_i_in.read();
    m_axi_i_awready_in.write(axi_i_i.AWREADY); 
    m_axi_i_wready_in.write(axi_i_i.WREADY); 
    m_axi_i_bvalid_in.write(axi_i_i.BVALID); 
    m_axi_i_bresp_in.write(axi_i_i.BRESP); 
    m_axi_i_arready_in.write(axi_i_i.ARREADY); 
    m_axi_i_rvalid_in.write(axi_i_i.RVALID); 
    m_axi_i_rdata_in.write(axi_i_i.RDATA); 
    m_axi_i_rresp_in.write(axi_i_i.RRESP); 


    axi4_lite_master axi_i_o;
    axi_i_o.AWVALID = m_axi_i_awvalid_out.read(); 
    axi_i_o.AWADDR = m_axi_i_awaddr_out.read(); 
    axi_i_o.WVALID = m_axi_i_wvalid_out.read(); 
    axi_i_o.WDATA = m_axi_i_wdata_out.read(); 
    axi_i_o.WSTRB = m_axi_i_wstrb_out.read(); 
    axi_i_o.BREADY = m_axi_i_bready_out.read(); 
    axi_i_o.ARVALID = m_axi_i_arvalid_out.read(); 
    axi_i_o.ARADDR = m_axi_i_araddr_out.read(); 
    axi_i_o.RREADY = m_axi_i_rready_out.read(); 
    axi_i_out.write(axi_i_o);
    axi4_master axi_t_i = axi_t_in.read();
    m_axi_t_awvalid_in.write(axi_t_i.AWVALID); 
    m_axi_t_awaddr_in.write(axi_t_i.AWADDR); 
    m_axi_t_awid_in.write(axi_t_i.AWID); 
    m_axi_t_awlen_in.write(axi_t_i.AWLEN); 
    m_axi_t_awburst_in.write(axi_t_i.AWBURST); 
    m_axi_t_wvalid_in.write(axi_t_i.WVALID); 
    m_axi_t_wdata_in.write(axi_t_i.WDATA); 
    m_axi_t_wstrb_in.write(axi_t_i.WSTRB); 
    m_axi_t_wlast_in.write(axi_t_i.WLAST); 
    m_axi_t_bready_in.write(axi_t_i.BREADY); 
    m_axi_t_arvalid_in.write(axi_t_i.ARVALID); 
    m_axi_t_araddr_in.write(axi_t_i.ARADDR); 
    m_axi_t_arid_in.write(axi_t_i.ARID); 
    m_axi_t_arlen_in.write(axi_t_i.ARLEN); 
    m_axi_t_arburst_in.write(axi_t_i.ARBURST); 
    m_axi_t_rready_in.write(axi_t_i.RREADY); 


    axi4_slave axi_t_o;
    axi_t_o.AWREADY = m_axi_t_awready_out.read(); 
    axi_t_o.WREADY = m_axi_t_wready_out.read(); 
    axi_t_o.BVALID = m_axi_t_bvalid_out.read(); 
    axi_t_o.BRESP = m_axi_t_bresp_out.read(); 
    axi_t_o.BID = m_axi_t_bid_out.read(); 
    axi_t_o.ARREADY = m_axi_t_arready_out.read(); 
    axi_t_o.RVALID = m_axi_t_rvalid_out.read(); 
    axi_t_o.RDATA = m_axi_t_rdata_out.read(); 
    axi_t_o.RRESP = m_axi_t_rresp_out.read(); 
    axi_t_o.RID = m_axi_t_rid_out.read(); 
    axi_t_o.RLAST = m_axi_t_rlast_out.read(); 
    axi_t_out.write(axi_t_o);

}
