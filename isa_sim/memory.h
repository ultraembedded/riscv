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
#ifndef __MEMORY_H__
#define __MEMORY_H__

#include <stdint.h>

//--------------------------------------------------------------------
// Abstract interface for memories
//--------------------------------------------------------------------
class Memory
{
public:  
    virtual void        reset(void) = 0;
    virtual uint32_t    load(uint32_t address, int width, bool signedLoad) = 0;
    virtual void        store(uint32_t address, uint32_t data, int width) = 0;
};

//-----------------------------------------------------------------
// Simple little endian memory
//-----------------------------------------------------------------
class SimpleMemory: public Memory
{
public:
    SimpleMemory(int size)
    {
        Mem = new uint32_t[(size + 3)/4];
        Size = size;
    }
    SimpleMemory(uint8_t * buf, int size)
    {
        Mem = (uint32_t*)buf;
        Size = size;
    }    

    virtual void reset(void)
    {
        memset(Mem, 0, Size);
    }

    virtual uint32_t load(uint32_t address, int width, bool signedLoad)
    {
        uint32_t data = 0;

        switch (width)
        {
            case 4:
                assert(!(address & 3));
                data = Mem[address / 4];
            break;
            case 2:
                assert(!(address & 1));

                if (address & 2)
                    data = (Mem[address / 4] >> 16)  & 0xFFFF;
                else
                    data = (Mem[address / 4] >> 0) & 0xFFFF;

                if (signedLoad)
                    if (data & (1 << 15))
                        data |= 0xFFFF0000;
            break;
            case 1:
                switch (address & 3)
                {
                    case 3:
                        data = (Mem[address / 4] >> 24) & 0xFF;
                    break;
                    case 2:
                        data = (Mem[address / 4] >> 16) & 0xFF;
                    break;
                    case 1:
                        data = (Mem[address / 4] >> 8) & 0xFF;
                    break;
                    case 0:
                        data = (Mem[address / 4] >> 0) & 0xFF;
                    break;
                }

                if (signedLoad)
                    if (data & (1 << 7))
                        data |= 0xFFFFFF00;
            break;
        }

        return data;
    }

    virtual void store(uint32_t address, uint32_t data, int width)
    {
        switch (width)
        {
            case 4:
                assert(!(address & 3));
                Mem[address / 4] = data;
            break;
            case 2:
                assert(!(address & 1));
                if (address & 2)
                    Mem[address / 4] = (Mem[address / 4] & 0x0000FFFF) | ((data << 16) & 0xFFFF0000);
                else
                    Mem[address / 4] = (Mem[address / 4] & 0xFFFF0000) | ((data << 0)  & 0x0000FFFF);                
            break;
            case 1:
                switch (address & 3)
                {
                    case 3:
                        Mem[address / 4] = (Mem[address / 4] & 0x00FFFFFF) | ((data << 24) & 0xFF000000);
                    break;
                    case 2:
                        Mem[address / 4] = (Mem[address / 4] & 0xFF00FFFF) | ((data << 16) & 0x00FF0000);
                    break;
                    case 1:
                        Mem[address / 4] = (Mem[address / 4] & 0xFFFF00FF) | ((data << 8) & 0x0000FF00);                    
                    break;
                    case 0:
                        Mem[address / 4] = (Mem[address / 4] & 0xFFFFFF00) | ((data << 0) & 0x000000FF);
                    break;
                }
            break;
        }
    }

private:
    uint32_t *Mem;
    int      Size;
};

#endif
