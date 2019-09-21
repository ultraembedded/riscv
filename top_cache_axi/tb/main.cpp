#include "sc_reset_gen.h"
#include "testbench.h"
#include <stdlib.h>
#include <math.h>
#include <signal.h>

//--------------------------------------------------------------------
// Defines
//--------------------------------------------------------------------
#ifndef SIM_TIME_RESOLUTION
    #define SIM_TIME_RESOLUTION 1
#endif
#ifndef SIM_TIME_SCALE
    #define SIM_TIME_SCALE SC_NS
#endif

#ifndef CLK0_PERIOD
    #define CLK0_PERIOD  10
#endif

#ifndef CLK0_NAME
    #define CLK0_NAME  clk
#endif

#ifndef RST0_NAME
    #define RST0_NAME  rst
#endif

#define xstr(a) str(a)
#define str(a) #a

//--------------------------------------------------------------------
// Locals
//--------------------------------------------------------------------
static testbench *tb = NULL;

//--------------------------------------------------------------------
// assert_handler: Handling of sc_assert
//--------------------------------------------------------------------
static void assert_handler(const sc_report& rep, const sc_actions& actions)
{
    sc_report_handler::default_handler(rep, actions & ~SC_ABORT);

    if ( actions & SC_ABORT )
    {
        cout << "TEST FAILED" << endl;
        if (tb)
            tb->abort();
        abort();
    }
}
//--------------------------------------------------------------------
// exit_override
//--------------------------------------------------------------------
static void exit_override(void)
{
    if (tb)
        tb->abort();
}
//--------------------------------------------------------------------
// vl_finish: Handling of verilog $finish
//--------------------------------------------------------------------
void vl_finish (const char* filename, int linenum, const char* hier)
{ 
    // Jump to exit handler!
    exit(0);    
}
//-----------------------------------------------------------------
// sigint_handler
//-----------------------------------------------------------------
static void sigint_handler(int s)
{
    exit_override();

    // Jump to exit handler!
    exit(1);
}
//--------------------------------------------------------------------
// sc_main
//--------------------------------------------------------------------
int sc_main(int argc, char* argv[])
{
    bool trace            = true;
    int seed              = 1;
    int last_argc         = 0;
    const char * vcd_name = "sysc_wave";

    // Env variable seed override
    char *s = getenv("SEED");
    if (s && strcmp(s, ""))
        seed = strtol(s, NULL, 0);

    for (int i=1;i<argc;i++)
    {
        if (!strcmp(argv[i], "--trace"))
        {
            trace = strtol(argv[i+1], NULL, 0);
            i++;
        }
        else if (!strcmp(argv[i], "--seed"))
        {
            seed = strtol(argv[i+1], NULL, 0);
            i++;
        }
        else if (!strcmp(argv[i], "--vcd_name"))
        {
            vcd_name = (const char*)argv[i+1];
            i++;
        }
        else
        {
            last_argc = i-1;
            break;
        }
    }

    // Enable waves override
    s = getenv("ENABLE_WAVES");
    if (s && !strcmp(s, "no"))
        trace = 0;    

    sc_report_handler::set_actions("/IEEE_Std_1666/deprecated", SC_DO_NOTHING);
    sc_set_time_resolution(SIM_TIME_RESOLUTION,SIM_TIME_SCALE);

    // Register custom assert handler
    sc_report_handler::set_handler(assert_handler);

    // Capture exit
    atexit(exit_override);

    // Catch SIGINT to restore terminal settings on exit
    signal(SIGINT, sigint_handler);

    // Seed
    srand(seed);

    // Clocks
    sc_clock CLK0_NAME (xstr(CLK0_NAME), CLK0_PERIOD, SIM_TIME_SCALE);
    sc_reset_gen clk0_rst(xstr(RST0_NAME));
                 clk0_rst.clk(CLK0_NAME);

    // Testbench
    tb = new testbench("tb");
    tb->CLK0_NAME(CLK0_NAME);
    tb->RST0_NAME(clk0_rst.rst);

    // Waves
    if (trace)
        tb->add_trace(sc_create_vcd_trace_file(vcd_name), "");

    tb->set_argcv(argc - last_argc, &argv[last_argc]);

    // Go!
    sc_start();

    return 0;
}
