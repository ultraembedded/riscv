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
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <libelf.h>
#include <fcntl.h>
#include <gelf.h>
#include <bfd.h>

#include "elf_load.h"

//-----------------------------------------------------------------
// elf_load
//-----------------------------------------------------------------
int elf_load(const char *filename, cb_mem_create fn_create, cb_mem_load fn_load, void *arg, uint32_t *start_addr)
{
    int fd;
    Elf * e;
    Elf_Kind ek;
    Elf_Scn *scn;
    Elf_Data *data;
    Elf32_Shdr *shdr;
    size_t shstrndx;

    if (elf_version ( EV_CURRENT ) == EV_NONE)
        return 0;

    if ((fd = open ( filename , O_RDONLY , 0)) < 0)
        return 0;

    if ((e = elf_begin ( fd , ELF_C_READ, NULL )) == NULL)
        return 0;
    
    ek = elf_kind ( e );
    if (ek != ELF_K_ELF)
        return 0;

    // Get section name header index
    if (elf_getshdrstrndx(e, &shstrndx)!=0)
        return 0;

    // Get entry point
    if (start_addr)
    {
        GElf_Ehdr _ehdr;
        GElf_Ehdr *ehdr = gelf_getehdr(e, &_ehdr);
        *start_addr = (uint32_t)ehdr->e_entry;
    }

    int section_idx = 0;
    while ((scn = elf_getscn(e, section_idx)) != NULL)
    {
        shdr = elf32_getshdr(scn);

        // Section which need allocating
        if ((shdr->sh_flags & SHF_ALLOC) && (shdr->sh_size > 0))
        {
            data = elf_getdata(scn, NULL);

            printf("Memory: 0x%x - 0x%x (Size=%dKB) [%s]\n", shdr->sh_addr, shdr->sh_addr + shdr->sh_size - 1, shdr->sh_size / 1024, elf_strptr(e, shstrndx, shdr->sh_name));

            if (!fn_create(arg, shdr->sh_addr, shdr->sh_size))
            {
                fprintf(stderr, "ERROR: Cannot allocate memory region\n");
                close (fd);
                return 0;
            }

            if (shdr->sh_type == SHT_PROGBITS)
            {                
                int i;
                for (i=0;i<shdr->sh_size;i++)
                {                    
                    if (!fn_load(arg, shdr->sh_addr + i, ((uint8_t*)data->d_buf)[i]))
                    {
                        fprintf(stderr, "ERROR: Cannot write byte to 0x%08x\n", shdr->sh_addr + i);
                        close (fd);
                        return 0;
                    }
                }
            }
        }

        section_idx++;
    }    

    elf_end ( e );
    close ( fd );
    
    return 1;
}
//-----------------------------------------------------------------
// elf_get_symbol
//-----------------------------------------------------------------
long elf_get_symbol(const char *filename, const char *symname)
{
    bfd *ibfd;
    asymbol **symtab;
    long nsize, nsyms, i;
    symbol_info syminfo;
    char **matching;

    bfd_init();
    ibfd = bfd_openr(filename, NULL);

    if (ibfd == NULL) 
    {
        printf("bfd_openr error\n");
        return -1;
    }

    if (!bfd_check_format_matches(ibfd, bfd_object, &matching)) 
    {
        printf("format_matches\n");
        return -1;
    }

    nsize = bfd_get_symtab_upper_bound (ibfd);
    symtab = (asymbol **)malloc(nsize);
    nsyms = bfd_canonicalize_symtab(ibfd, symtab);

    for (i = 0; i < nsyms; i++) {
        if (strcmp(symtab[i]->name, symname) == 0) {
            bfd_symbol_info(symtab[i], &syminfo);
            return (long) syminfo.value;
        }
    }

    bfd_close(ibfd);
    return -1;
}
