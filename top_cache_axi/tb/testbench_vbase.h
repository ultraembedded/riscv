#ifndef TESTBENCH_VBASE_H
#define TESTBENCH_VBASE_H

#include <systemc.h>
#include "verilated.h"
#include "verilated_vcd_sc.h"

#define verilator_trace_enable(vcd_filename, dut) \
        if (waves_enabled()) \
        { \
            Verilated::traceEverOn(true); \
            VerilatedVcdC *v_vcd = new VerilatedVcdC; \
            sc_core::sc_time delay_us; \
            if (waves_delayed(delay_us)) \
                dut->trace_enable (v_vcd, delay_us); \
            else \
                dut->trace_enable (v_vcd); \
            v_vcd->open (vcd_filename); \
            this->m_verilate_vcd = v_vcd; \
        }

//-----------------------------------------------------------------
// Module
//-----------------------------------------------------------------
class testbench_vbase: public sc_module
{
public:
    sc_in <bool>    clk;
    sc_in <bool>    rst;

    virtual void set_testcase(int tc) { }
    virtual void set_delays(bool en) { }
    virtual void set_iterations(int iterations) { }
    virtual void set_argcv(int argc, char* argv[]) { }

    virtual void process(void) { while (1) wait(); }
    virtual void monitor(void) { while (1) wait(); }

    SC_HAS_PROCESS(testbench_vbase);
    testbench_vbase(sc_module_name name): sc_module(name)
    {    
        SC_CTHREAD(process, clk);
        SC_CTHREAD(monitor, clk);
    }

    virtual void add_trace(sc_trace_file * fp, std::string prefix) { }

    virtual void abort(void)
    {
        cout << "TB: Aborted at " << sc_time_stamp() << endl;
        if (m_verilate_vcd)
        {
            m_verilate_vcd->flush();
            m_verilate_vcd->close();
            m_verilate_vcd = NULL;
        }
    }

    bool waves_enabled(void)
    {
        char *s = getenv("ENABLE_WAVES");
        if (s && !strcmp(s, "no"))
            return false;
        else
            return true;
    }

    bool waves_delayed(sc_core::sc_time &delay)
    {
        char *s = getenv("WAVES_DELAY_US");
        if (s != NULL)
        {
            uint32_t us = strtoul(s, NULL, 0);
            printf("WAVES: Delay start until %duS\n", us);
            delay = sc_core::sc_time(us, SC_US);
            return true;
        }
        else
            return false;
    }    

    std::string getenv_str(std::string name, std::string defval)
    {
        char *s = getenv(name.c_str());
        if (!s || (s && !strcmp(s, "")))
            return defval;
        else
            return std::string(s);
    }

protected:
    VerilatedVcdC   *m_verilate_vcd;
};

#endif