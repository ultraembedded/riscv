#ifndef __ELF_LOAD_H__
#define __ELF_LOAD_H__

#include <stdint.h>

//-------------------------------------------------------------
// Types
//-------------------------------------------------------------
typedef int (*cb_mem_create)(void *arg, uint32_t base, uint32_t size);
typedef int (*cb_mem_load)(void *arg, uint32_t addr, uint8_t data);

//-------------------------------------------------------------
// Functions
//-------------------------------------------------------------
int  elf_load(const char *filename, cb_mem_create fn_create, cb_mem_load fn_load, void *arg, uint32_t *start_addr);
long elf_get_symbol(const char *filename, const char *symname);

#endif