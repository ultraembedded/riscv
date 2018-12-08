#ifndef __CPU_API_H__
#define __CPU_API_H__

#include <stdint.h>
#include <vector>
#include <queue>
#include <string>

//--------------------------------------------------------------------
// Cosimulation events
//--------------------------------------------------------------------
typedef enum e_cosim_event
{
    COSIM_EVENT_LOAD,
    COSIM_EVENT_LOAD_RESULT,
    COSIM_EVENT_STORE,
    COSIM_EVENT_MAX
} t_cosim_event;

class cosim_event
{
public:
    t_cosim_event type;
    uint32_t arg1;
    uint32_t arg2;
};

//--------------------------------------------------------------------
// Abstract interface for CPU simulation API
//--------------------------------------------------------------------
class cosim_cpu_api
{
public:
    // Reset core to execute from specified PC
    virtual void      reset(uint32_t pc) = 0;

    // Status    
    virtual bool      get_fault(void)   = 0;
    virtual bool      get_stopped(void) = 0;
    virtual bool      get_break(void)  { return false; }

    // Execute one instruction
    virtual void      step(void) = 0;

    // Breakpoints
    virtual bool      set_breakpoint(uint32_t pc)   { return false; }
    virtual bool      clr_breakpoint(uint32_t pc) { return false; }

    // State after execution
    virtual uint32_t  get_opcode(void) = 0;
    virtual uint32_t  get_pc(void) = 0;
    virtual bool      get_reg_valid(int r) = 0;
    virtual uint32_t  get_register(int r) = 0;
    virtual int       get_num_reg(void) = 0;

    virtual void      set_register(int r, uint32_t val) = 0;
    virtual void      set_pc(uint32_t val) { }

    // Trigger interrupt
    virtual void      set_interrupt(int irq) = 0;

    // Instruction trace
    virtual void      enable_trace(uint32_t mask) = 0;

    // Event Queue
    std::queue <cosim_event > event_q[COSIM_EVENT_MAX];
    void event_push(t_cosim_event ev, uint32_t arg1, uint32_t arg2)
    {
        cosim_event item;
        
        item.type  = ev;
        item.arg1  = arg1;
        item.arg2  = arg2;

        event_q[ev].push(item);
    }
    bool event_ready(t_cosim_event ev) { return !event_q[ev].empty(); }
    cosim_event event_pop(t_cosim_event ev) 
    { 
        cosim_event item = event_q[ev].front();
        event_q[ev].pop();
        return item;
    }
};

//--------------------------------------------------------------------
// Abstract interface for memory access
//--------------------------------------------------------------------
class cosim_mem_api
{
public:
    virtual bool    create_memory(uint32_t addr, uint32_t size, uint8_t *mem = NULL) = 0;
    virtual bool    valid_addr(uint32_t addr) = 0;
    virtual void    write(uint32_t addr, uint8_t data) = 0;
    virtual uint8_t read(uint32_t addr) = 0;
};

//--------------------------------------------------------------------
// Structures
//--------------------------------------------------------------------
class cosim_cpu_item
{
public:
    std::string    name;
    cosim_cpu_api *cpu;
};

class cosim_mem_item
{
public:
    std::string    name;
    cosim_mem_api *mem;
    uint32_t       base;
    uint32_t       size;
};

//--------------------------------------------------------------------
// Class: Cosimulation framework
//--------------------------------------------------------------------
class cosim: public cosim_cpu_api, cosim_mem_api
{
private:
    static cosim * s_instance;
    cosim() { m_dump_file = NULL; }

public:
    static cosim *instance(void)
    {
        if (!s_instance)
            s_instance = new cosim();
        return s_instance;
    }

    void attach_cpu(std::string name, cosim_cpu_api *p);
    void attach_mem(std::string name, cosim_mem_api *p, uint32_t base, uint32_t size);

    // cosim_cpu_api

    // Reset core to execute from specified PC
    void reset(uint32_t pc);

    // Status    
    bool get_fault(void);
    bool get_stopped(void);

    // Execute one instruction
    void step(void);

    // State after execution
    uint32_t  get_opcode(void);
    uint32_t  get_pc(void);
    bool      get_reg_valid(int r);
    uint32_t  get_register(int r);
    int       get_num_reg(void);

    void      set_register(int r, uint32_t val);

    // Trigger interrupt
    void      set_interrupt(int irq);

    void      enable_trace(uint32_t mask);

    // cosim_mem_api
    bool    create_memory(uint32_t addr, uint32_t size, uint8_t *mem = NULL);
    bool    valid_addr(uint32_t addr);
    void    write(uint32_t addr, uint8_t data);
    uint8_t read(uint32_t addr);

    void     write_word(uint32_t addr, uint32_t data);
    uint32_t read_word(uint32_t addr);

    void    at_exit(uint32_t exitcode);

    // Set memory dump on exit
    void    dump_on_exit(const char *filename, uint32_t dump_start, uint32_t dump_end)
    {
        m_dump_file  = filename;
        m_dump_start = dump_start;
        m_dump_end   = dump_end;
    }

private:
    std::vector <cosim_cpu_item > m_cpu;
    std::vector <cosim_mem_item > m_mem;

    const char * m_dump_file;
    uint32_t     m_dump_start;
    uint32_t     m_dump_end;
};

#endif
