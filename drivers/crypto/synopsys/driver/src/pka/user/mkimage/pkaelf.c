/*
 * This Synopsys software and associated documentation (hereinafter the
 * "Software") is an unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you. The
 * Software IS NOT an item of Licensed Software or a Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Products
 * with Synopsys or any supplement thereto. Synopsys is a registered trademark
 * of Synopsys, Inc. Other names included in the SOFTWARE may be the
 * trademarks of their respective owners.
 *
 * The contents of this file are dual-licensed; you may select either version
 * 2 of the GNU General Public License ("GPL") or the BSD-3-Clause license
 * ("BSD-3-Clause"). The GPL is included in the COPYING file accompanying the
 * SOFTWARE. The BSD License is copied below.
 *
 * BSD-3-Clause License:
 * Copyright (c) 2011-2015 Synopsys, Inc. and/or its affiliates.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions, and the following disclaimer, without
 *    modification.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The names of the above-listed copyright holders may not be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// ------------------------------------------------------------------------
//
//  Project:
//
//   Driver SDK
//
//  Description:
//
//   Helper functions for generating ELF binaries with libelf.
//
// ------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "pkaelf.h"
#include "common.h"

/* Machine type for PKA */
#define EM_ELPPKA 0xe117

struct pka_strtab {
   char *tab;
   size_t alloc, bytes;
};

struct pka_elf {
   struct pka_strtab shstrtab, strtab;
   Elf *e;
};

/*
 * Find the index of an entry in an ELF string table.
 *
 * Returns (size_t)-1 if there is no matching entry.
 */
static size_t
strtab_get_string(const struct pka_strtab *strtab, const char *str)
{
   size_t i;

   /* XXX: is linear search too slow here? */
   for (i = 0; i < strtab->bytes;) {
      if (!strcmp(strtab->tab+i, str))
         return i;
      i += strlen(strtab->tab+i) + 1;
   }

   return -1;
}

size_t pka_elf_get_string(struct pka_elf *elf, const char *str)
{
   return strtab_get_string(&elf->strtab, str);
}

const char *pka_elf_read_strtab(struct pka_elf *elf, size_t stridx)
{
   if (stridx >= elf->strtab.bytes)
      return NULL;
   return elf->strtab.tab + stridx;
}

/*
 * Add a string to the ELF string table.
 *
 * Returns the index of the new entry, or (size_t)-1 on memory allocation
 * failure.
 */
static size_t
strtab_add_string(struct pka_strtab *strtab, const char *str)
{
   size_t len = strlen(str) + 1;

   while (len >= strtab->alloc - strtab->bytes) {
      char *tmp;
      size_t n;

      n = strtab->alloc ? strtab->alloc * 2 : 64;
      tmp = realloc(strtab->tab, n);
      if (!tmp) {
         pka_tool_err(0, NULL);
         return -1;
      }
      strtab->alloc = n;
      strtab->tab = tmp;
   }

   memcpy(strtab->tab + strtab->bytes, str, len);
   strtab->bytes += len;
   return strtab->bytes - len;
}

size_t pka_elf_add_string(struct pka_elf *elf, const char *str)
{
   return strtab_add_string(&elf->strtab, str);
}

static size_t
strtab_add_string_checked(struct pka_strtab *strtab, const char *str)
{
   size_t ret;

   ret = strtab_get_string(strtab, str);
   if (ret == -1)
      ret = strtab_add_string(strtab, str);
   return ret;
}

size_t pka_elf_add_string_checked(struct pka_elf *elf, const char *str)
{
   return strtab_add_string_checked(&elf->strtab, str);
}

static int pka_elf_set_header(Elf *e)
{
   Elf32_Ehdr *ehdr;

   ehdr = elf32_newehdr(e);
   if (ehdr == NULL) {
      pka_tool_err(-1, "failed to create ELF Header: %s", elf_errmsg(-1));
      return -1;
   }

   ehdr->e_ident[EI_DATA] = ELFDATA2LSB;
   ehdr->e_ident[EI_OSABI] = ELFOSABI_STANDALONE;
   ehdr->e_machine = EM_ELPPKA;
   ehdr->e_type = ET_EXEC;
   ehdr->e_version = EV_CURRENT;

   return 0;
}


struct pka_elf *pka_elf_new(int fd)
{
   struct pka_elf *elf;

   if (elf_version(EV_CURRENT) == EV_NONE) {
      pka_tool_err(-1, "failed to initialize libelf: %s", elf_errmsg(-1));
      return NULL;
   }

   elf = malloc(sizeof *elf);
   if (!elf) {
      return NULL;
   }
   *elf = (struct pka_elf) { 0 };

   /* Setup the initial string tables. */
   if (strtab_add_string(&elf->shstrtab, "") == -1
       || strtab_add_string(&elf->shstrtab, ".shstrtab") == -1
       || strtab_add_string(&elf->shstrtab, ".strtab") == -1
       || strtab_add_string(&elf->strtab, "") == -1) {
      goto err_alloc;
   }

   elf->e = elf_begin(fd, ELF_C_WRITE, NULL);
   if (!elf->e) {
      pka_tool_err(-1, "failed to create ELF file: %s", elf_errmsg(-1));
      goto err_alloc;
   }

   if (pka_elf_set_header(elf->e) != 0) {
      goto err_elf;
   }

   return elf;

err_elf:
   elf_end(elf->e);
err_alloc:
   free(elf->shstrtab.tab);
   free(elf->strtab.tab);
   free(elf);
   return NULL;
}

size_t pka_elf_add_section(struct pka_elf *elf,   const char *name,
                           void *data_buf,        size_t data_len,
                           unsigned data_type,    unsigned data_align,
                           unsigned section_type, unsigned section_flags)
{
   Elf_Scn    *scn;
   Elf_Data   *data;
   Elf32_Shdr *shdr;
   size_t name_idx;

   name_idx = strtab_add_string_checked(&elf->shstrtab, name);
   if (name_idx == -1)
      return -1;

   scn = elf_newscn(elf->e);
   if (!scn) {
      pka_tool_err(-1, "failed to create section: %s", elf_errmsg(-1));
      return -1;
   }

   data = elf_newdata(scn);
   if (!data) {
      pka_tool_err(-1, "failed to create section data: %s", elf_errmsg(-1));
      return -1;
   }

   data->d_align   = data_align;
   data->d_buf     = data_buf;
   data->d_off     = 0;
   data->d_size    = data_len;
   data->d_type    = data_type;
   data->d_version = EV_CURRENT;

   shdr = elf32_getshdr(scn);
   if (!shdr) {
      pka_tool_err(-1, "failed to get section header: %s", elf_errmsg(-1));
      return -1;
   }

   shdr->sh_name    = name_idx;
   shdr->sh_type    = section_type;
   shdr->sh_flags   = section_flags;
   shdr->sh_entsize = 0;

   return elf_ndxscn(scn);
}

static Elf32_Shdr *get_shdr(struct pka_elf *elf, size_t index)
{
   Elf_Scn *scn;

   scn = elf_getscn(elf->e, index);
   if (scn)
      return elf32_getshdr(scn);
   return NULL;
}


int
pka_elf_set_section_addr(struct pka_elf *elf, size_t index, unsigned long addr)
{
   Elf32_Shdr *shdr;

   shdr = get_shdr(elf, index);
   if (!shdr) {
      pka_tool_err(-1, "failed to get section header: %s", elf_errmsg(-1));
      return -1;
   }

   shdr->sh_addr = addr;
   return 0;
}

/* Links the ELF symbol table with the string table, if present. */
static void pka_elf_assign_symtab(struct pka_elf *elf, size_t strtab)
{
   Elf_Scn *scn = NULL;

   while ((scn = elf_nextscn(elf->e, scn))) {
      Elf32_Shdr *shdr = elf32_getshdr(scn);

      if (!strcmp(elf->shstrtab.tab+shdr->sh_name, ".symtab")) {
         shdr->sh_link = strtab;
         break;
      }
   }
}

int pka_elf_write(struct pka_elf *elf)
{
   Elf32_Ehdr *ehdr;
   size_t rc;

   /*
    * Note that .strtab and .shstrtab must already exist in the string table.
    * Otherwise, it will be added by add_section but the data length passed
    * here will not account for it.
    */
   rc = pka_elf_add_section(elf, ".shstrtab",
                            elf->shstrtab.tab, elf->shstrtab.bytes,
                            ELF_T_BYTE, 1, SHT_STRTAB,
                            SHF_STRINGS | SHF_ALLOC);
   if (rc == -1) {
      return -1;
   }

   ehdr = elf32_getehdr(elf->e);
   ehdr->e_shstrndx = rc;

   rc = pka_elf_add_section(elf, ".strtab",
                            elf->strtab.tab, elf->strtab.bytes,
                            ELF_T_BYTE, 1, SHT_STRTAB,
                            SHF_STRINGS | SHF_ALLOC);
   if (rc == -1) {
      return -1;
   }

   pka_elf_assign_symtab(elf, rc);

   if (elf_update(elf->e, ELF_C_WRITE) == -1) {
      pka_tool_err(-1, "failed to write ELF file: %s", elf_errmsg(-1));
      return -1;
   }

   return 0;
}

void pka_elf_free(struct pka_elf *elf)
{
   elf_end(elf->e);
   free(elf->strtab.tab);
   free(elf);
}
