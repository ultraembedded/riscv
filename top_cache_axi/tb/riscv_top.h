
#ifndef RISCV_TOP_H
#define RISCV_TOP_H
#include <systemc.h>

#include "axi4.h"
#include "axi4.h"

class Vriscv_top;
class VerilatedVcdC;

//-------------------------------------------------------------
// riscv_top: RTL wrapper class
//-------------------------------------------------------------
class riscv_top: public sc_module
{
public:
    sc_in <bool> clk_in;
    sc_in <bool> rst_in;
    sc_in <bool> intr_in;
    sc_in <sc_uint<32> > reset_vector_in;

    sc_in  <axi4_slave>  axi_i_in;
    sc_out <axi4_master> axi_i_out;
    sc_in  <axi4_slave>  axi_d_in;
    sc_out <axi4_master> axi_d_out;

    //-------------------------------------------------------------
    // Constructor
    //-------------------------------------------------------------
    SC_HAS_PROCESS(riscv_top);
    riscv_top(sc_module_name name);

    //-------------------------------------------------------------
    // Trace
    //-------------------------------------------------------------
    virtual void add_trace(sc_trace_file *vcd, std::string prefix)
    {
        #undef  TRACE_SIGNAL
        #define TRACE_SIGNAL(s) sc_trace(vcd,s,prefix + #s)

        TRACE_SIGNAL(clk_in);
        TRACE_SIGNAL(rst_in);
        TRACE_SIGNAL(intr_in);
        TRACE_SIGNAL(reset_vector_in);
        TRACE_SIGNAL(axi_i_in);
        TRACE_SIGNAL(axi_i_out);
        TRACE_SIGNAL(axi_d_in);
        TRACE_SIGNAL(axi_d_out);

        #undef  TRACE_SIGNAL
    }

    void async_outputs(void);
    void trace_rtl(void);
    void trace_enable(VerilatedVcdC *p);
    void trace_enable(VerilatedVcdC *p, sc_core::sc_time start_time);

    //-------------------------------------------------------------
    // Signals
    //-------------------------------------------------------------
private:
    sc_signal <bool> m_clk_in;
    sc_signal <bool> m_rst_in;
    sc_signal <bool> m_axi_i_awready_in;
    sc_signal <bool> m_axi_i_wready_in;
    sc_signal <bool> m_axi_i_bvalid_in;
    sc_signal <sc_uint<2> > m_axi_i_bresp_in;
    sc_signal <sc_uint<4> > m_axi_i_bid_in;
    sc_signal <bool> m_axi_i_arready_in;
    sc_signal <bool> m_axi_i_rvalid_in;
    sc_signal <sc_uint<32> > m_axi_i_rdata_in;
    sc_signal <sc_uint<2> > m_axi_i_rresp_in;
    sc_signal <sc_uint<4> > m_axi_i_rid_in;
    sc_signal <bool> m_axi_i_rlast_in;
    sc_signal <bool> m_axi_d_awready_in;
    sc_signal <bool> m_axi_d_wready_in;
    sc_signal <bool> m_axi_d_bvalid_in;
    sc_signal <sc_uint<2> > m_axi_d_bresp_in;
    sc_signal <sc_uint<4> > m_axi_d_bid_in;
    sc_signal <bool> m_axi_d_arready_in;
    sc_signal <bool> m_axi_d_rvalid_in;
    sc_signal <sc_uint<32> > m_axi_d_rdata_in;
    sc_signal <sc_uint<2> > m_axi_d_rresp_in;
    sc_signal <sc_uint<4> > m_axi_d_rid_in;
    sc_signal <bool> m_axi_d_rlast_in;
    sc_signal <bool> m_intr_in;
    sc_signal <sc_uint<32> > m_reset_vector_in;

    sc_signal <bool> m_axi_i_awvalid_out;
    sc_signal <sc_uint<32> > m_axi_i_awaddr_out;
    sc_signal <sc_uint<4> > m_axi_i_awid_out;
    sc_signal <sc_uint<8> > m_axi_i_awlen_out;
    sc_signal <sc_uint<2> > m_axi_i_awburst_out;
    sc_signal <bool> m_axi_i_wvalid_out;
    sc_signal <sc_uint<32> > m_axi_i_wdata_out;
    sc_signal <sc_uint<4> > m_axi_i_wstrb_out;
    sc_signal <bool> m_axi_i_wlast_out;
    sc_signal <bool> m_axi_i_bready_out;
    sc_signal <bool> m_axi_i_arvalid_out;
    sc_signal <sc_uint<32> > m_axi_i_araddr_out;
    sc_signal <sc_uint<4> > m_axi_i_arid_out;
    sc_signal <sc_uint<8> > m_axi_i_arlen_out;
    sc_signal <sc_uint<2> > m_axi_i_arburst_out;
    sc_signal <bool> m_axi_i_rready_out;
    sc_signal <bool> m_axi_d_awvalid_out;
    sc_signal <sc_uint<32> > m_axi_d_awaddr_out;
    sc_signal <sc_uint<4> > m_axi_d_awid_out;
    sc_signal <sc_uint<8> > m_axi_d_awlen_out;
    sc_signal <sc_uint<2> > m_axi_d_awburst_out;
    sc_signal <bool> m_axi_d_wvalid_out;
    sc_signal <sc_uint<32> > m_axi_d_wdata_out;
    sc_signal <sc_uint<4> > m_axi_d_wstrb_out;
    sc_signal <bool> m_axi_d_wlast_out;
    sc_signal <bool> m_axi_d_bready_out;
    sc_signal <bool> m_axi_d_arvalid_out;
    sc_signal <sc_uint<32> > m_axi_d_araddr_out;
    sc_signal <sc_uint<4> > m_axi_d_arid_out;
    sc_signal <sc_uint<8> > m_axi_d_arlen_out;
    sc_signal <sc_uint<2> > m_axi_d_arburst_out;
    sc_signal <bool> m_axi_d_rready_out;

public:
    Vriscv_top *m_rtl;
#if VM_TRACE
    VerilatedVcdC  * m_vcd;
    bool             m_delay_waves;
    sc_core::sc_time m_waves_start;
#endif 
};

#endif