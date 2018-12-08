//-----------------------------------------------------------------
//                     RISC-V ISA Simulator 
//                            V1.0
//                     Ultra-Embedded.com
//                     Copyright 2014-2017
//
//                   admin@ultra-embedded.com
//
//                       License: BSD
//-----------------------------------------------------------------
//
// Copyright (c) 2014, Ultra-Embedded.com
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions 
// are met:
//   - Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   - Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer 
//     in the documentation and/or other materials provided with the 
//     distribution.
//   - Neither the name of the author nor the names of its contributors 
//     may be used to endorse or promote products derived from this 
//     software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
// THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
// SUCH DAMAGE.
//-----------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cosim_api.h"

cosim * cosim::s_instance = NULL;

//--------------------------------------------------------------------
// attach_cpu
//--------------------------------------------------------------------
void cosim::attach_cpu(std::string name, cosim_cpu_api *p)
{
    cosim_cpu_item item;
    
    item.name = name;
    item.cpu  = p;

    m_cpu.push_back(item);
}
//--------------------------------------------------------------------
// attach_mem
//--------------------------------------------------------------------
void cosim::attach_mem(std::string name, cosim_mem_api *p, uint32_t base, uint32_t size)
{
    cosim_mem_item item;
    
    item.name = name;
    item.mem  = p;
    item.base = base;
    item.size = size;

    m_mem.push_back(item);
}
//--------------------------------------------------------------------
// reset: Reset core to execute from specified PC
//--------------------------------------------------------------------
void cosim::reset(uint32_t pc)
{
    for (std::vector<cosim_cpu_item>::iterator it = m_cpu.begin() ; it != m_cpu.end(); ++it)
        it->cpu->reset(pc);
}
//--------------------------------------------------------------------
// get_fault:
//--------------------------------------------------------------------
bool cosim::get_fault(void)
{
    bool fault = false;

    for (std::vector<cosim_cpu_item>::iterator it = m_cpu.begin() ; it != m_cpu.end(); ++it)
        fault |= it->cpu->get_fault();

    return fault;
}
//--------------------------------------------------------------------
// get_stopped:
//--------------------------------------------------------------------
bool cosim::get_stopped(void)
{
    bool stopped = false;

    for (std::vector<cosim_cpu_item>::iterator it = m_cpu.begin() ; it != m_cpu.end(); ++it)
        stopped |= it->cpu->get_stopped();

    return stopped;
}
//--------------------------------------------------------------------
// step:
//--------------------------------------------------------------------
void cosim::step(void)
{
    for (int ev = COSIM_EVENT_LOAD; ev < COSIM_EVENT_MAX; ev++)
    {
        bool all_ready = true;
        for (std::vector<cosim_cpu_item>::iterator it = m_cpu.begin() ; it != m_cpu.end(); ++it)
            if (!it->cpu->event_ready((t_cosim_event)ev))
                all_ready = false;

        if (all_ready)
        {
            bool first = true;
            cosim_event last;
            for (std::vector<cosim_cpu_item>::iterator it = m_cpu.begin() ; it != m_cpu.end(); ++it)
            {
                cosim_event item = it->cpu->event_pop((t_cosim_event)ev);
                        
                if (!first)
                {
                    if (item.arg1 != last.arg1 || item.arg2 != last.arg2)
                    {
                        fprintf(stderr, "ERROR: Event %d mismatch %08x == %08x && %08x == %08x\n", ev, item.arg1, last.arg1, item.arg2, last.arg2);
                        exit(-1);
                    }
                }
                first = false;
                last = item;
            }
            
        }
    }

    for (std::vector<cosim_cpu_item>::iterator it = m_cpu.begin() ; it != m_cpu.end(); ++it)
        it->cpu->step();

    for (std::vector<cosim_cpu_item>::iterator it = m_cpu.begin() ; it != m_cpu.end(); ++it)
        if (m_cpu.front().cpu->get_pc() != it->cpu->get_pc())
        {
            fprintf(stderr, "ERROR: PC mismatch %08x (%s) != %08x (%s)\n", it->cpu->get_pc(), it->name.c_str(), m_cpu.front().cpu->get_pc(), m_cpu.front().name.c_str());
            exit(-1);
        }
        
    int num_reg = get_num_reg();

    for (int i=0;i<num_reg;i++)
        for (std::vector<cosim_cpu_item>::iterator it = m_cpu.begin() ; it != m_cpu.end(); ++it)
            if (m_cpu.front().cpu->get_reg_valid(i) && it->cpu->get_reg_valid(i))
                if (m_cpu.front().cpu->get_register(i) != it->cpu->get_register(i))
                {
                    fprintf(stderr, "ERROR: PC=%08x REG%d mismatch %08x (%s) != %08x (%s)\n", m_cpu.front().cpu->get_pc(), i, it->cpu->get_register(i), it->name.c_str(), m_cpu.front().cpu->get_register(i), m_cpu.front().name.c_str());
                    exit(-1);
                }
}
//--------------------------------------------------------------------
// get_opcode:
//--------------------------------------------------------------------
uint32_t cosim::get_opcode(void)
{
    return m_cpu.front().cpu->get_opcode();
}
//--------------------------------------------------------------------
// get_pc:
//--------------------------------------------------------------------
uint32_t cosim::get_pc(void)
{
    return m_cpu.front().cpu->get_pc();
}
//--------------------------------------------------------------------
// get_reg_valid:
//--------------------------------------------------------------------
bool cosim::get_reg_valid(int r)
{
    return m_cpu.front().cpu->get_reg_valid(r);
}
//--------------------------------------------------------------------
// get_register:
//--------------------------------------------------------------------
uint32_t cosim::get_register(int r)
{
    return m_cpu.front().cpu->get_register(r);
}
//--------------------------------------------------------------------
// get_num_reg:
//--------------------------------------------------------------------
int cosim::get_num_reg(void)
{
    return m_cpu.front().cpu->get_num_reg();
}
//--------------------------------------------------------------------
// set_register:
//--------------------------------------------------------------------
void cosim::set_register(int r, uint32_t val)
{
    for (std::vector<cosim_cpu_item>::iterator it = m_cpu.begin() ; it != m_cpu.end(); ++it)
        it->cpu->set_register(r, val);
}
//--------------------------------------------------------------------
// set_interrupt:
//--------------------------------------------------------------------
void cosim::set_interrupt(int irq)
{
    for (std::vector<cosim_cpu_item>::iterator it = m_cpu.begin() ; it != m_cpu.end(); ++it)
        it->cpu->set_interrupt(irq);
}
//--------------------------------------------------------------------
// enable_trace:
//--------------------------------------------------------------------
void cosim::enable_trace(uint32_t mask)
{
    for (std::vector<cosim_cpu_item>::iterator it = m_cpu.begin() ; it != m_cpu.end(); ++it)
        it->cpu->enable_trace(mask);
}
//--------------------------------------------------------------------
// create_memory:
//--------------------------------------------------------------------
bool cosim::create_memory(uint32_t addr, uint32_t size, uint8_t *mem /*= NULL*/)
{
    bool ok = true;

    for (std::vector<cosim_mem_item>::iterator it = m_mem.begin() ; it != m_mem.end(); ++it)
        ok &= it->mem->create_memory(addr, size, mem);

    return ok;
}
//--------------------------------------------------------------------
// valid_addr:
//--------------------------------------------------------------------
bool cosim::valid_addr(uint32_t addr)
{
    for (std::vector<cosim_mem_item>::iterator it = m_mem.begin() ; it != m_mem.end(); ++it)
        if (addr >= it->base && addr < (it->base + it->size))
            return true;
    return false;
}
//--------------------------------------------------------------------
// write: Byte write
//--------------------------------------------------------------------
void cosim::write(uint32_t addr, uint8_t data)
{
    for (std::vector<cosim_mem_item>::iterator it = m_mem.begin() ; it != m_mem.end(); ++it)
        if (addr >= it->base && addr < (it->base + it->size))
            it->mem->write(addr, data);
}
//--------------------------------------------------------------------
// read: Read write
//--------------------------------------------------------------------
uint8_t cosim::read(uint32_t addr)
{
    for (std::vector<cosim_mem_item>::iterator it = m_mem.begin() ; it != m_mem.end(); ++it)
         if (addr >= it->base && addr < (it->base + it->size))
            return it->mem->read(addr);
    return 0;
}
//--------------------------------------------------------------------
// write_word: Write word
//--------------------------------------------------------------------
void cosim::write_word(uint32_t addr, uint32_t data)
{
    for (int i=0;i<4;i++)
        write(addr + i,data >> (i*8));
}
//--------------------------------------------------------------------
// read_word: Read word
//--------------------------------------------------------------------
uint32_t cosim::read_word(uint32_t addr)
{
    uint32_t data = 0;
    for (int i=0;i<4;i++)
        data |= ((uint32_t)read(addr + i)) << (i*8);
    return data;
}
//--------------------------------------------------------------------
// at_exit: On simulation exit
//--------------------------------------------------------------------
void cosim::at_exit(uint32_t exit_code)
{
    if (m_dump_file)
    {
        bool sig_txt_file = false;

        const char *ext = strrchr(m_dump_file, '.');
        sig_txt_file = ext && !strcmp(ext, ".output");

        int  dump_size  = m_dump_end - m_dump_start;

        printf("Dumping post simulation memory: 0x%08x-0x%08x (%d bytes) [%s]\n", m_dump_start, m_dump_end, dump_size, m_dump_file);

        uint8_t *buffer = new uint8_t[dump_size];
        for (int i=0;i<dump_size;i++)
            buffer[i] = cosim::instance()->read((uint32_t)m_dump_start + i);

        // Binary block
        if (!sig_txt_file)
        {
            // Write file data
            FILE *f = fopen(m_dump_file, "wb");
            if (f)
            {
                fwrite(buffer, 1, dump_size, f);
                fclose(f);
            }
        }
        // Signature text file
        else
        {
            // Write file data
            FILE *f = fopen(m_dump_file, "w");
            if (f)
            {
                for (int r=0;r<(dump_size/16);r++)
                {
                    for (int c=0;c<16;c+=4)
                    {
                        uint32_t w = *(uint32_t*)&buffer[(r*16) + (12-c)];
                        fprintf(f, "%08x", w);
                    }
                    fprintf(f, "\n");
                }
                fclose(f);
            }
        }

        delete buffer;
        buffer = NULL;
    }

    exit(exit_code);
}
