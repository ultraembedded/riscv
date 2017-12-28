#include <systemc.h>

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
class testbench: public sc_module, public cosim_cpu_api, public cosim_mem_api
{
public:
    //-----------------------------------------------------------------
    // Signals
    //-----------------------------------------------------------------    
    sc_in <bool>                clk;
    sc_in <bool>                rst;
    sc_signal <bool>            rst_cpu_in;

    //-----------------------------------------------------------------
    // Instances / Members
    //-----------------------------------------------------------------      
    riscv_tcm_top_rtl           *m_dut;

    int                          m_argc;
    char**                       m_argv;

    sc_signal <axi4_master>      axi_t_in;
    sc_signal <axi4_slave>       axi_t_out;

    sc_signal <axi4_lite_master> axi_i_out;
    sc_signal <axi4_lite_slave>  axi_i_in;

    sc_signal < bool >           intr_in;

    VerilatedVcdC               *m_verilate_vcd;

    bool                         m_stopped;

    //-----------------------------------------------------------------
    // thread: Main loop for CPU execution
    //-----------------------------------------------------------------
    void thread(void) 
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
    testbench(sc_module_name name): sc_module(name)
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

        SC_CTHREAD(thread, clk);

        m_stopped = false;
    }

    //-----------------------------------------------------------------
    // Trace
    //-----------------------------------------------------------------
    void add_trace(sc_trace_file *vcd, std::string prefix)
    {
        m_dut->add_trace(vcd, prefix + "DUT/");

        Verilated::traceEverOn(true);
        VerilatedVcdC *v_vcd = new VerilatedVcdC;
        m_dut->trace_enable (v_vcd);
        v_vcd->open ("verilator.vcd");
        m_verilate_vcd = v_vcd;        
    }
    //-----------------------------------------------------------------
    // abort: Flush VCD trace
    //-----------------------------------------------------------------
    void abort(void)
    {
        if (m_verilate_vcd)
        {
            m_verilate_vcd->flush();
            m_verilate_vcd->close();
            m_verilate_vcd = NULL;
        }

        m_stopped = true;
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
    bool      get_stopped(void) { return m_stopped; } 
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
