#ifndef TB_AXI4_MEM_H
#define TB_AXI4_MEM_H

#include "axi4.h"
#include "axi4_defines.h"
#include "tb_memory.h"

//-------------------------------------------------------------
// tb_axi4_mem: AXI4 testbench memory
//-------------------------------------------------------------
class tb_axi4_mem: public sc_module, public tb_memory
{
public:
    //-------------------------------------------------------------
    // Interface I/O
    //-------------------------------------------------------------
    sc_in <bool>             clk_in;
    sc_in <bool>             rst_in;

    sc_in <axi4_master>      axi_in;
    sc_out <axi4_slave>      axi_out;

    //-------------------------------------------------------------
    // Constructor
    //-------------------------------------------------------------
    SC_HAS_PROCESS(tb_axi4_mem);
    tb_axi4_mem(sc_module_name name): sc_module(name) 
    { 
        SC_CTHREAD(process, clk_in.pos());
        m_enable_delays = true;
    }

    //-------------------------------------------------------------
    // Trace
    //-------------------------------------------------------------
    void add_trace(sc_trace_file *vcd, std::string prefix)
    {
        #undef  TRACE_SIGNAL
        #define TRACE_SIGNAL(s) sc_trace(vcd,s,prefix + #s)

        TRACE_SIGNAL(axi_out);
        TRACE_SIGNAL(axi_in);

        #undef  TRACE_SIGNAL
    }

    //-------------------------------------------------------------
    // API
    //-------------------------------------------------------------
    void         enable_delays(bool enable) { m_enable_delays = enable; }
    void         write(uint32_t addr, uint8_t data);
    uint8_t      read(uint32_t addr);
    void         write32(uint32_t addr, uint32_t data, uint8_t strb = 0xF);
    uint32_t     read32(uint32_t addr);

    void         process(void);
    bool         delay_cycle(void) { return m_enable_delays ? rand() & 1 : 0; }

    sc_uint <AXI4_AXLEN_W> calc_wrap_mask(sc_uint <AXI4_AXLEN_W> len);
    sc_uint <AXI4_ADDR_W>  calc_next_addr(sc_uint <AXI4_ADDR_W> addr, sc_uint <AXI4_AXBURST_W> type, sc_uint <AXI4_AXLEN_W> len);

protected:
    bool m_enable_delays;
};

#endif