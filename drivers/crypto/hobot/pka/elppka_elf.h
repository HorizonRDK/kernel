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
 * Copyright (c) 2013-2015 Synopsys, Inc. and/or its affiliates.
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
//   Subset of the ELF types, constants and structures used by the PKA
//   firmware.
//
// ------------------------------------------------------------------------

/* Use Linux-provided definitions when available. */
#ifdef __KERNEL__
#include <linux/elf.h>
#define ELPPKA_ELF_H_
#endif

#ifndef ELPPKA_ELF_H_
#define ELPPKA_ELF_H_

#include "elppdu.h"

typedef uint32_t Elf32_Word, Elf32_Off, Elf32_Addr;
typedef uint16_t Elf32_Half;

#define EI_MAG0    0
#define EI_MAG1    1
#define EI_MAG2    2
#define EI_MAG3    3
#define EI_CLASS   4
#define EI_DATA    5
#define EI_VERSION 6
#define EI_OSABI   7
#define EI_NIDENT 16

#define ET_NONE 0
#define ET_REL  1
#define ET_EXEC 2
#define ET_DYN  3
#define ET_CORE 4

#define ELFMAG0 0x7f
#define ELFMAG1 0x45
#define ELFMAG2 0x4C
#define ELFMAG3 0x46

#define ELFCLASSNONE 0
#define ELFCLASS32   1
#define ELFCLASS64   2

#define ELFDATANONE 0
#define ELFDATA2LSB 1
#define ELFDATA2MSB 2

#define EV_NONE    0
#define EV_CURRENT 1

#define SHT_NULL     0
#define SHT_PROGBITS 1
#define SHT_SYMTAB   2
#define SHT_STRTAB   3
#define SHT_RELA     4
#define SHT_HASH     5
#define SHT_DYNAMIC  6
#define SHT_NOTE     7
#define SHT_NOBITS   8
#define SHT_REL      9
#define SHT_SHLIB   10
#define SHT_DYNSYM  11
#define SHT_LOPROC  0x70000000
#define SHT_HIPROC  0x7fffffff
#define SHT_LOUSER  0x80000000
#define SHT_HIUSER  0xffffffff

typedef struct {
   unsigned char e_ident[EI_NIDENT];
   Elf32_Half    e_type;
   Elf32_Half    e_machine;
   Elf32_Word    e_version;
   Elf32_Addr    e_entry;
   Elf32_Off     e_phoff;
   Elf32_Off     e_shoff;
   Elf32_Word    e_flags;
   Elf32_Half    e_ehsize;
   Elf32_Half    e_phentsize;
   Elf32_Half    e_phnum;
   Elf32_Half    e_shentsize;
   Elf32_Half    e_shnum;
   Elf32_Half    e_shstrndx;
} Elf32_Ehdr;

typedef struct {
   Elf32_Word sh_name;
   Elf32_Word sh_type;
   Elf32_Word sh_flags;
   Elf32_Addr sh_addr;
   Elf32_Off  sh_offset;
   Elf32_Word sh_size;
   Elf32_Word sh_link;
   Elf32_Word sh_info;
   Elf32_Word sh_addralign;
   Elf32_Word sh_entsize;
} Elf32_Shdr;

typedef struct {
   Elf32_Word    st_name;
   Elf32_Addr    st_value;
   Elf32_Word    st_size;
   unsigned char st_info;
   unsigned char st_other;
   Elf32_Half    st_shndx;
} Elf32_Sym;

#endif
