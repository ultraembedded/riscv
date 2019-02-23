#include "testbench_vbase.h"

#include "riscv_main.h"
#include "riscv.h"
#include "elf_load.h"

#include <unistd.h>

#include "cosim_api.h"

#include "riscv_tcm_top_rtl.h"
#include "Vriscv_tcm_top.h"
#include "Vriscv_tcm_top__Syms.h"

#include "verilated.h"
#include "verilated_vcd_sc.h"

//-----------------------------------------------------------------
// Module
//-----------------------------------------------------------------
class testbench: public testbench_vbase, public cosim_cpu_api, public cosim_mem_api
{
public:
    //-----------------------------------------------------------------
    // Instances / Members
    //-----------------------------------------------------------------      
    riscv_tcm_top_rtl           *m_dut;

    int                          m_argc;
    char**                       m_argv;
    //-----------------------------------------------------------------
    // Signals
    //-----------------------------------------------------------------    
    sc_signal <bool>            rst_cpu_in;

    sc_signal <axi4_master>      axi_t_in;
    sc_signal <axi4_slave>       axi_t_out;

    sc_signal <axi4_lite_master> axi_i_out;
    sc_signal <axi4_lite_slave>  axi_i_in;

    sc_signal < bool >           intr_in;


    //-----------------------------------------------------------------
    // process: Main loop for CPU execution
    //-----------------------------------------------------------------
    void process(void) 
    {
        cosim::instance()->attach_cpu("rtl", this);
        cosim::instance()->attach_mem("rtl", this, 0, 0xFFFFFFFF);
        wait();
        exit(riscv_main(cosim::instance(), m_argc, m_argv));
    }

    void set_argcv(int argc, char* argv[]) { m_argc = argc; m_argv = argv; }

    //-----------------------------------------------------------------
    // Construction
    //-----------------------------------------------------------------
    SC_HAS_PROCESS(testbench);
    testbench(sc_module_name name): testbench_vbase(name)
    {
        m_dut = new riscv_tcm_top_rtl("DUT");
        m_dut->clk_in(clk);
        m_dut->rst_in(rst);
        m_dut->rst_cpu_in(rst_cpu_in);
        m_dut->axi_t_out(axi_t_out);
        m_dut->axi_t_in(axi_t_in);
        m_dut->axi_i_out(axi_i_out);
        m_dut->axi_i_in(axi_i_in);
        m_dut->intr_in(intr_in);
		
		verilator_trace_enable("verilator.vcd", m_dut);
    }
    //-----------------------------------------------------------------
    // Trace
    //-----------------------------------------------------------------
    void add_trace(sc_trace_file * fp, std::string prefix)
    {
        if (!waves_enabled())
            return;

        // Add signals to trace file
        #define TRACE_SIGNAL(a) sc_trace(fp,a,#a);
        TRACE_SIGNAL(clk);
        TRACE_SIGNAL(rst);

        m_dut->add_trace(fp, "");
    }

    //-----------------------------------------------------------------
    // create_memory: Create memory region
    //-----------------------------------------------------------------
    bool create_memory(uint32_t base, uint32_t size, uint8_t *mem = NULL)
    {
        sc_assert(base >= 0x00000000 && ((base + size) < (0x00000000 + (64 * 1024))));
        return true;
    }
    //-----------------------------------------------------------------
    // valid_addr: Check address range
    //-----------------------------------------------------------------
    bool valid_addr(uint32_t addr) { return true; } 
    //-----------------------------------------------------------------
    // write: Write byte into memory
    //-----------------------------------------------------------------
    void write(uint32_t addr, uint8_t data)
    {
        m_dut->m_rtl->v->u_mem_tcm->write(addr, data);
    }
    //-----------------------------------------------------------------
    // write: Read byte from memory
    //-----------------------------------------------------------------
    uint8_t read(uint32_t addr)
    {
        return m_dut->m_rtl->v->u_mem_tcm->read(addr);
    }
    //-----------------------------------------------------------------
    // step: Execute 1 clock cycle
    //-----------------------------------------------------------------
    void step(void)
    {
        wait();
    }
    //-----------------------------------------------------------------
    // reset: Release core from reset
    //-----------------------------------------------------------------
    void reset(uint32_t addr)
    {
        rst_cpu_in.write(true);
        wait();
        rst_cpu_in.write(false);
    }

    // Not supported
    bool      get_stopped(void) { return false; } 
    bool      get_fault(void)  { return false; }
    void      set_interrupt(int irq)   { }
    void      enable_trace(uint32_t mask) { }
    uint32_t  get_opcode(void)    { }
    uint32_t  get_pc(void)        { return 0; }
    bool      get_reg_valid(int r){ return 0; }
    uint32_t  get_register(int r) { return 0; }
    int       get_num_reg(void)   { return 32; }
    void      set_register(int r, uint32_t val) { }    
};
