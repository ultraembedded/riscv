#include "systemc.h"

#include "riscv_main.h"
#include "testbench_vbase.h"
#include "tb_axi4_mem.h"

#include "cosim_api.h"

#include "riscv_top.h"
#include "Vriscv_top.h"
#include "Vriscv_top__Syms.h"

#include "riscv.h"
#include "elf_load.h"

#include <unistd.h>

//-----------------------------------------------------------------
// Module
//-----------------------------------------------------------------
class testbench: public testbench_vbase, public cosim_cpu_api, public cosim_mem_api
{
public:

    //-----------------------------------------------------------------
    // Instances / Members
    //-----------------------------------------------------------------      
    riscv_top                   *m_dut;
    tb_axi4_mem                 *m_icache_mem;
    tb_axi4_mem                 *m_dcache_mem;

    sc_signal <axi4_slave>      mem_i_in;
    sc_signal <axi4_master>     mem_i_out;

    sc_signal <axi4_slave>      mem_d_in;
    sc_signal <axi4_master>     mem_d_out;

    sc_signal < bool >          intr_in;

    sc_signal < sc_uint <32> >  reset_vector_in;

    int                         m_argc;
    char**                      m_argv;

    //-----------------------------------------------------------------
    // process
    //-----------------------------------------------------------------
    void process(void) 
    {
        int exitcode;

        cosim::instance()->attach_cpu("rtl", this);
        cosim::instance()->attach_mem("rtl", this, 0, 0xFFFFFFFF);

        wait();

        exit(riscv_main(cosim::instance(), m_argc, m_argv));
    }

    //-----------------------------------------------------------------
    // Trace
    //-----------------------------------------------------------------
    void add_trace(sc_trace_file * fp, std::string prefix)
    {
        // Add signals to trace file
        #define TRACE_SIGNAL(a) sc_trace(fp,a,#a);
        TRACE_SIGNAL(clk);
        TRACE_SIGNAL(rst);

        m_dut->add_trace(fp, "");
    }


    void set_testcase(int tc) { }
    void set_delays(bool en) { }
    void set_iterations(int iterations) { }
    void set_argcv(int argc, char* argv[]) { m_argc = argc; m_argv = argv; }

    SC_HAS_PROCESS(testbench);
    testbench(sc_module_name name): testbench_vbase(name)
    {
        m_dut = new riscv_top("DUT");
        m_dut->clk_in(clk);
        m_dut->rst_in(rst);
        m_dut->axi_i_out(mem_i_out);
        m_dut->axi_i_in(mem_i_in);
        m_dut->axi_d_out(mem_d_out);
        m_dut->axi_d_in(mem_d_in);
        m_dut->intr_in(intr_in);
        m_dut->reset_vector_in(reset_vector_in);

        // Instruction Cache Memory
        m_icache_mem = new tb_axi4_mem("ICACHE_MEM");
        m_icache_mem->clk_in(clk);
        m_icache_mem->rst_in(rst);
        m_icache_mem->axi_in(mem_i_out);
        m_icache_mem->axi_out(mem_i_in);

        // Data Cache Memory
        m_dcache_mem = new tb_axi4_mem("DCACHE_MEM");
        m_dcache_mem->clk_in(clk);
        m_dcache_mem->rst_in(rst);
        m_dcache_mem->axi_in(mem_d_out);
        m_dcache_mem->axi_out(mem_d_in);

        m_cycles = 0;

        verilator_trace_enable("verilator.vcd", m_dut);
    }

    //-----------------------------------------------------------------
    // create_memory: Create memory region
    //-----------------------------------------------------------------
    bool create_memory(uint32_t base, uint32_t size, uint8_t *mem = NULL)
    {
        base = base & ~(32-1);
        size = (size + 31) & ~(32-1);

        while (m_icache_mem->valid_addr(base))
            base += 1;

        while (m_icache_mem->valid_addr(base + size - 1))
            size -= 1;

        m_icache_mem->add_region(base, size);
        m_dcache_mem->add_region(m_icache_mem->get_array(base), base, size);

        memset(m_icache_mem->get_array(base), 0, size);

        return true;
    }

    bool valid_addr(uint32_t addr) { return true; } 
    //-----------------------------------------------------------------
    // write: Write byte into memory
    //-----------------------------------------------------------------
    void write(uint32_t addr, uint8_t data)
    {
        m_dcache_mem->write(addr, data);
    }
    //-----------------------------------------------------------------
    // write: Read byte from memory
    //-----------------------------------------------------------------
    uint8_t read(uint32_t addr)
    {
        return m_dcache_mem->read(addr);
    }

    // Status    
    bool get_fault(void) { return false; }
    bool get_stopped(void) { return false; } 

    void step(void)
    {
        wait();
        m_cycles++;
    }

    // Reset core to execute from specified PC
    void reset(uint32_t addr)
    {
        reset_vector_in.write(addr);
    }

    // Trigger interrupt
    void set_interrupt(int irq)   { }

    void enable_trace(uint32_t mask) { }

    // State after execution
    uint32_t  get_opcode(void)    { }
    uint32_t  get_pc(void)
    {
        static uint32_t last_pc = 0;

        if (m_dut->m_rtl->__VlSymsp->TOP__v__u_core__u_issue.complete_valid0())
            last_pc = m_dut->m_rtl->__VlSymsp->TOP__v__u_core__u_issue.complete_pc0();


        return last_pc;
    }    
    bool      get_reg_valid(int r){ return 0; }//m_dut->u_core.u_decode.get_reg_valid(r); }
    uint32_t  get_register(int r) { return 0; }//m_dut->u_core.u_decode.get_register(r); }
    int       get_num_reg(void)   { return 32; }

    void      set_register(int r, uint32_t val)
    {
        //m_dut->u_core.u_decode.set_register(r, val);
    }

    uint32_t m_cycles;
};
