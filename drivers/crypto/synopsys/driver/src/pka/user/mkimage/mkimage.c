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

#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <stdbool.h>
#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>

#include "readmemh.h"
#include "symbol.h"
#include "common.h"
#include "pkaelf.h"

#define MKIMAGE_VERSION "1.0"

enum {
   SOPT_END = UCHAR_MAX,
   LOPT_RAM_ONLY,
   LOPT_ROM_ONLY,
};

static const char sopts[] = "r::o:e:S:VH";
static const struct option lopts[] = {
   { "output",   1, NULL, 'o' },
   { "entries",  1, NULL, 'e' },
   { "symbols",  1, NULL, 'S' },
   { "rom-base", 1, NULL, 'r' },
   { "rom-only", 0, NULL, LOPT_ROM_ONLY },
   { "ram-only", 0, NULL, LOPT_RAM_ONLY },
   { "version",  0, NULL, 'V' },
   { "help",     0, NULL, 'H' },
   { 0 }
};

struct mkimage_opts {
   char *outfile, *symfile;
   bool sym_is_ent;
   struct symbol *symbols;
   unsigned rom_base;

   struct mkimage_code {
      struct mkimage_code *next;
      struct symbol *symbols;

      unsigned size, origin, taglen;
      size_t section;
      bool rom;

      unsigned long insns[];
   } *code;
};

static void print_usage(FILE *f)
{
   fprintf(f, "usage: %s [options] [hexfile ...]\n", pka_tool_name());
}

static void print_block(int indent, int initial, const char *str)
{
   const char *l = str;

   while (*l) {
      const char *nl;

      nl = strchr(l, '\n');
      if (!nl) {
         printf("%*s%s\n", indent-initial, "", l);
         return;
      }

      printf("%*s%.*s\n", indent-initial, "", (int)(nl-l), l);
      initial = 0;
      l = nl+1;
   }
}

static void print_help(void)
{
   print_usage(stdout);
   puts(
"This is pka-mkimage: a tool to generate ELF binary firmware images for the\n"
"PKA Linux driver.\n");

   puts("Options:");
   for (const struct option *opt = lopts; opt->val; opt++) {
      const char *argjoin = "";
      int w = 0;

      if (opt->has_arg == 2)
         argjoin = "[=";
      else if (opt->has_arg == 1)
         argjoin = "=";

      if (opt->name && opt->val && opt->val <= SOPT_END) {
         w += printf("  -%c, --%s%s", opt->val, opt->name, argjoin);
      } else if (opt->name) {
         w += printf("  --%s%s", opt->name, argjoin);
      } else {
         w += printf("  -%c", opt->val);
      }

      if (opt->has_arg) {
         if (opt->val == 'o' || opt->val == 'S' || opt->val == 'e') {
            w += printf("FILE");
         } else if (opt->val == 'r') {
            w += printf("ADDR");
         } else {
            w += printf("ARG");
         }

         if (opt->has_arg == 2)
            w += printf("]");
      }

      if (w > 22) {
         putchar('\n');
         w = 0;
      }

      switch (opt->val) {
      case 'r':
         print_block(24, w, "Set offset of RAM/ROM split to ADDR, as a count\nof instruction words.");
         break;
      case LOPT_ROM_ONLY:
         print_block(24, w, "Generate a ROM-only image; equivalent to --rom-base=0.");
         break;
      case LOPT_RAM_ONLY:
         print_block(24, w, "Generate a RAM-only image.");
         break;
      case 'e':
         print_block(24, w, "Read entry points from FILE.");
         break;
      case 'S':
         print_block(24, w, "Read symbol table from FILE.");
         break;
      case 'o':
         print_block(24, w, "Write output to FILE instead of standard output.");
         break;
      case 'V':
         print_block(24, w, "Print a version message and then exit.");
         break;
      case 'H':
         print_block(24, w, "Print this message and then exit.");
         break;
      default:
         if (w > 0)
            putchar('\n');
      }
   }

   puts("\nFor more information, see the man page.");
}

/*
 * Check the tag block for sanity, and set code->origin / code->taglen
 * based on the tag value.  Returns 0 on success, -1 on failure (invalid tag).
 */
static int check_tag(struct mkimage_code *code)
{
   unsigned long origin;
   unsigned i, taglen;

   if (code->size == 0 || (code->insns[0] >> 24 != 0xf8)) {
      pka_tool_err(-1, "F/W tag block is missing");
      return -1;
   }

   taglen = code->insns[0] & 0xff;
   if (taglen < 1 || taglen > code->size) {
      pka_tool_err(-1, "F/W tag length is invalid: %u words", taglen);
      return -1;
   }

   origin = code->insns[0] & 0xffff00;
   if (origin > 4*PKA_MAX_FW_WORDS) {
      pka_tool_err(-1, "F/W origin is bogus: %u bytes", origin);
      return -1;
   }

   for (i = 1; i < taglen; i++) {
      if ((code->insns[i] >> 24) != 0xf8) {
         pka_tool_err(-1, "invalid instruction in F/W tag block");
         return -1;
      }
   }

   code->taglen = taglen;
   code->origin = origin;
   return 0;
}

/*
 * Format the PKA instruction sequence into the binary format.  Returns the
 * number of bytes on success (4*nwords), or (size_t)-1 on failure.
 */
static size_t
serialize_code(unsigned char (**buf)[4], const unsigned long *code,
                                         unsigned nwords)
{
   size_t bytes;
   unsigned i;

   if (nwords >= SIZE_MAX/4)
      return -1;
   bytes = (size_t)nwords * 4;

   *buf = malloc(bytes);
   if (!*buf)
      return -1;

   for (i = 0; i < nwords; i++) {
      unsigned char *e = (*buf)[i];

      /* Data is little-endian */
      e[0] = code[i] & 0xff;
      e[1] = (code[i] >> 8) & 0xff;
      e[2] = (code[i] >> 16) & 0xff;
      e[3] = (code[i] >> 24) & 0xff;
   }

   return bytes;
}

/*
 * Create a RAM code section, and add the instructions to it.  If code is
 * NULL, create an empty section.
 *
 * Returns 0 on success, or -1 on failure.
 */
static int set_ram_text(struct pka_elf *elf, struct mkimage_code *code)
{
   unsigned char (*rawcode)[4];
   size_t rawsize, index;

   if (!code) {
      index = pka_elf_add_section(elf, ".ram.null", NULL, 0,
                                  ELF_T_WORD, 0, SHT_NOBITS, 0);
      if (index == (size_t)-1)
         return -1;
      return 0;
   }

   rawsize = serialize_code(&rawcode, code->insns, code->size);
   if (rawsize == -1)
      return -1;

   index = pka_elf_add_section(elf, ".ram.text", rawcode, rawsize,
                               ELF_T_WORD, 4, SHT_PROGBITS, SHF_ALLOC);
   if (index == (size_t)-1)
      return -1;
   code->section = index;

   return pka_elf_set_section_addr(elf, index, code->origin);
}

/*
 * Create a ROM code section, and add the instructions to it.  If code is
 * NULL, create an empty section.
 *
 * Returns 0 on success, or -1 on failure.
 */
static int set_rom_text(struct pka_elf *elf, struct mkimage_code *code)
{
   unsigned char (*rawcode)[4];
   size_t rawsize, index;

   if (!code) {
      index = pka_elf_add_section(elf, ".rom.null", NULL, 0,
                                  ELF_T_WORD, 0, SHT_NOBITS, 0);
      if (index == (size_t)-1)
         return -1;
      return 0;
   }

   /* We only store the tag block for ROM images. */
   rawsize = serialize_code(&rawcode, code->insns, code->taglen);
   if (rawsize == -1)
      return -1;

   index = pka_elf_add_section(elf, ".rom.text", rawcode, rawsize,
                               ELF_T_WORD, 4, SHT_PROGBITS, SHF_ALLOC);
   if (index == (size_t)-1)
      return -1;
   code->section = index;

   return pka_elf_set_section_addr(elf, index, code->origin);
}

/*
 * Format an ELF symbol table for the specified code region.  The symbols
 * are written to buf, but not more than nsyms symbols are written.
 *
 * On success, returns the actual number of symbols in the list (which may
 * be greater than nsyms).  On failure, returns (size_t)-1.
 */
static size_t
serialize_symbols(struct pka_elf *elf, Elf32_Sym *buf, size_t nsyms,
                                       struct mkimage_code *code)
{
   struct symbol *s;
   size_t i;

   for (i = 0, s = code->symbols; s; s = s->next, i++) {
      size_t name_id;

      if (i >= nsyms)
         continue;

      name_id = pka_elf_add_string_checked(elf, s->name);
      if (name_id == -1) {
         return -1;
      }

      buf[i] = (Elf32_Sym) {
         .st_name  = name_id,
         .st_value = s->address * 4,
         .st_info  = ELF32_ST_INFO(1, STT_FUNC),
         .st_shndx = code->section,
      };
   }

   return i;
}

static int sym_compare(const void *a_, const void *b_)
{
   const Elf32_Sym *a = a_, *b = b_;

   if (ELF32_ST_BIND(a->st_info) < ELF32_ST_BIND(b->st_info))
      return -1;
   if (ELF32_ST_BIND(a->st_info) > ELF32_ST_BIND(b->st_info))
      return 1;
   if (a->st_name < b->st_name)
      return -1;
   if (a->st_name > b->st_name)
      return 1;
   if (a->st_value < b->st_value)
      return -1;
   if (a->st_value > b->st_value)
      return 1;

   return 0;
}

/*
 * Remove duplicates from a sorted list of symbols.  Returns the number of
 * symbols remaining.
 */
static size_t
filter_duplicate_symbols(struct pka_elf *elf, Elf32_Sym *syms, size_t nsyms)
{
   size_t lastbind = 0, lastname = -1, lastval = -1;
   size_t i, j;

   for (i = j = 0; i + j < nsyms; i++) {
      if (ELF32_ST_BIND(syms[i+j].st_info) != lastbind
          || syms[i+j].st_name != lastname)
      {
         syms[i] = syms[i+j];
         lastbind = ELF32_ST_BIND(syms[i].st_info);
         lastname = syms[i].st_name;
         lastval  = syms[i].st_value;
      } else if (syms[i+j].st_value != lastval) {
         syms[i] = syms[i+j];
         lastval = syms[i].st_value;

         if (lastbind == STB_GLOBAL) {
            pka_tool_err(-1, "warning: duplicate global symbol: %s",
                             pka_elf_read_strtab(elf, syms[i].st_name));
         }
      } else {
         j++;
         i--;
      }
   }

   return i;
}

static int set_symtab(struct pka_elf *elf, struct mkimage_opts *opts)
{
   size_t nsyms = 0, pos = 0, index;
   Elf32_Sym *syms;

   /* Count all "active" symbols */
   for (struct mkimage_code *c = opts->code; c; c = c->next) {
      for (struct symbol *s = c->symbols; s; s = s->next)
         nsyms++;
   }

   if (!nsyms)
      return 0;

   if (nsyms >= SIZE_MAX / sizeof *syms) {
      pka_tool_err(-1, "symbol table too large");
      return -1;
   }

   syms = malloc(nsyms * sizeof *syms);
   if (!syms) {
      pka_tool_err(0, "failed to allocate memory");
      return -1;
   }

   for (struct mkimage_code *c = opts->code; c; c = c->next) {
      size_t symcount;

      assert(pos <= nsyms);
      symcount = serialize_symbols(elf, syms+pos, nsyms-pos, c);
      if (symcount == (size_t)-1) {
         free(syms);
         return -1;
      }

      assert(symcount <= nsyms-pos);
      pos += symcount;
   }

   qsort(syms, nsyms, sizeof *syms, sym_compare);
   nsyms = filter_duplicate_symbols(elf, syms, nsyms);

   index = pka_elf_add_section(elf, ".symtab", syms, nsyms * sizeof *syms,
                               ELF_T_SYM, 4, SHT_SYMTAB, SHF_ALLOC);
   if (index == (size_t)-1) {
      free(syms);
      return -1;
   }

   return 0;
}

/*
 * Basic insertion sort for simple linked lists made of structs.  Sort order
 * is least to highest.
 *
 *   type:     the element type of the list (a struct type), which must be
 *             such that
 *
 *               type *x;
 *
 *             is a valid declaration of x as a pointer to type.
 *
 *   headp:    pointer to the list 'head'; which is a pointer object referring
 *             to the first list element.  This pointer will be updated to
 *             point to the beginning of the sorted list.
 *
 *   nextelem: the name of the struct member pointing to the next element.
 *
 *   compar:   comparison function to call.  Takes two arguments (a, b) which
 *             are pointers to list elements, should return a value less than,
 *             equal to, or greater than 0 if a is found to be less than, equal
 *             to, or greater than b, respectively.
 */
#define SORT_LIST(type, headp, nextelem, compar) do { \
   type *p__, *c__, **ip__, *i__; \
   for (p__ = *(headp); p__ && (c__ = p__->nextelem);) { \
      if ((compar)(p__, c__) < 0) { \
         p__ = p__->nextelem; \
         continue; \
      } \
      p__->nextelem = c__->nextelem; \
      for (ip__ = (headp); (i__ = *ip__); ip__ = &i__->nextelem) { \
         if ((compar)(i__, c__) > 0) { \
            c__->nextelem = i__; \
            *ip__ = c__; \
            break; \
         } \
      } \
   } \
} while (0)

static int compar_code(struct mkimage_code *a, struct mkimage_code *b)
{
   return a->origin < b->origin ? -1 :
          a->origin > b->origin ?  1 : 0;
}

static int compar_symbol(struct symbol *a, struct symbol *b)
{
   return a->address < b->address ? -1 :
          a->address > b->address ?  1 : 0;
}

static void assign_symbols_to_code(struct mkimage_opts *opts)
{
   struct symbol **start = &opts->symbols, **end, *s;
   struct mkimage_code *c;

   SORT_LIST(struct mkimage_code, &opts->code, next, compar_code);
   SORT_LIST(struct symbol, &opts->symbols, next, compar_symbol);

   for (c = opts->code; c; c = c->next) {
      /* Find starting element for this code section */
      for (; (s = *start); start = &s->next) {
         if (s->address >= c->origin)
            break;
      }

      /* Find final element for this code section */
      for (end = start; (s = *end); end = &s->next) {
         if (s->address - c->origin > c->size)
            break;
      }

      /* Attach symbols to code section */
      c->symbols = *start;
      *start = *end;
      *end = NULL;
   }
}

static int finalize_code_regions(struct mkimage_opts *opts)
{
   unsigned count = 0;

   for (struct mkimage_code *c = opts->code; c; c = c->next) {
      if (c->origin >= opts->rom_base)
         c->rom = true;

      if (c->next && c->next->origin - c->origin < c->size) {
         pka_tool_err(-1, "error: overlapping firmware regions");
         return -1;
      }

      count++;
   }

   return 0;
}

static int generate_image(int outfd, struct mkimage_opts *opts)
{
   struct pka_elf *elf;
   int state = 0;
   int ret = -1;

   assign_symbols_to_code(opts);

   /*
    * Any remaining symbols in the global list are not part of any code
    * region
    */
   for (struct symbol *s = opts->symbols; s; s = s->next) {
      pka_tool_err(-1, "warning: symbol %s address %#jx is invalid, ignoring",
                       s->name, s->address);
   }

   if (finalize_code_regions(opts) < 0)
      return -1;

   elf = pka_elf_new(outfd);
   if (!elf)
      return -1;

   /*
    * Create code sections in output file.  This is complicated somewhat
    * as we currently need to create dummy RAM/ROM sections for the RAM-only
    * and ROM-only cases.
    */
   for (struct mkimage_code *c = opts->code; c; c = c->next) {
      if (state == 2) {
         pka_tool_err(-1, "too many code regions specified, cannot proceed");
         goto out;
      }

      if (state == 1 && !c->rom) {
         pka_tool_err(-1, "multiple RAM regions; incorrect --rom-base?");
         goto out;
      }

      if (state == 0) {
         if (set_ram_text(elf, c->rom ? 0 : c) < 0)
            goto out;
         state = 1;
      }

      if (state == 1 && c->rom) {
         if (set_rom_text(elf, c) < 0)
            goto out;
         state = 2;
      }
   }

   /* Create dummy ROM section if we didn't already. */
   if (state == 1 && set_rom_text(elf, NULL) < 0)
      goto out;

   if (set_symtab(elf, opts) == -1)
      goto out;

   ret = pka_elf_write(elf);
out:
   pka_elf_free(elf);
   return ret;
}

/*
 * Parse a symbol input file.  If entfile is true, the file is interpreted
 * as a .ent file which requires some mangling of the symbol names.
 */
struct symbol *read_symbols(FILE *f, bool entfile)
{
   struct symbol *symbols;

   symbols = pka_parse_fw_sym(f);
   if (!symbols)
      return NULL;

   if (entfile) {
      struct symbol **p = &symbols;

      /* Rewrite all the symbols in uppercase with an ENTRY_ prefix. */
      for (struct symbol *s = *p; s; p = &s->next, s = *p) {
         size_t n = strlen(s->name);
         struct symbol *tmp;

         tmp = malloc(sizeof *tmp + sizeof "ENTRY_" + n);
         if (!tmp) {
            pka_free_symbols(symbols);
            return NULL;
         }

         for (size_t i = 0; i < n; i++) {
            s->name[i] = toupper(s->name[i]);
         }

         memcpy(tmp, s, sizeof *tmp);
         sprintf(tmp->name, "ENTRY_%s", s->name);

         *p = tmp;
         free(s);
         s = tmp;
      }
   }

   return symbols;
}

/*
 * Parse a .hex file.  If f is NULL, then the specified filename is opened
 * for reading.  On success, returns a pointer to the parsed-out code
 * structure, which must be freed by the caller.  Otherwise, an error
 * message is printed and NULL is returned.
 */
static struct mkimage_code *parse_hex(FILE *f, const char *filename)
{
   struct mkimage_code *code;
   int rc, size;

   if (!f) {
      f = fopen(filename, "r");
      if (!f) {
         pka_tool_err(0, "failed to open %s", filename);
         return NULL;
      }
   }

   code = malloc(sizeof *code + PKA_MAX_FW_WORDS * sizeof code->insns[0]);
   if (!code) {
      pka_tool_err(0, "failed to allocate memory");
      goto err_close;
   }
   *code = (struct mkimage_code){0};

   size = pka_parse_fw_hex(f, code->insns);
   if (size < 0)
      goto err_free;

   code->size = size;
   rc = check_tag(code);
   if (rc < 0)
      goto err_free;

   fclose(f);
   return code;

err_free:
   free(code);
err_close:
   fclose(f);
   return NULL;
}

/*
 * Read a list of .hex input files, returning a linked list of mkimage_code
 * structures.  The filename list must be terminated with a null pointer.  If
 * the list is empty, read from standard input.
 *
 * On failure, returns NULL.
 */
static struct mkimage_code *read_inputs(char **filenames)
{
   struct mkimage_code *c, *list = NULL;
   int i;

   for (i = 0; filenames[i]; i++) {
      struct mkimage_code *c;

      c = parse_hex(NULL, filenames[i]);
      if (!c)
         goto err_unwind;

      c->next = list;
      list = c;
   }

   if (!list)
      list = parse_hex(stdin, "stdin");

   return list;
err_unwind:
   while (list) {
      c = list->next;
      free(list);
      list = c;
   }

   return NULL;
}

static int
do_strtoul(const char *s, int base, unsigned long maxval, unsigned long *out)
{
   unsigned long val;
   char *end;
   int errcode;

   errno = 0;
   val = strtoul(s, &end, base);
   if (*end) {
      errcode = -1;
      goto err;
   } else if (errno != 0) {
      errcode = 0;
      goto err;
   } else if (val > maxval) {
      errcode = ERANGE;
      goto err;
   }

   *out = val;
   return 0;
err:
   pka_tool_err(errcode, "invalid argument: %s", s);
   return -1;
}

/*
 * Process command-line options, filling out options structure.  Returns 0 on
 * success, a negative value if processing was halted due to an error, or a
 * positive value if processing was halted due to --help or --version options.
 */
static int do_cmdline_opts(int argc, char **argv, struct mkimage_opts *opts)
{
   int opt;

   optind = 1;
   while ((opt = getopt_long(argc, argv, sopts, lopts, NULL)) != -1) {
      switch (opt) {
      case 'o':
         opts->outfile = optarg;
         break;
      case 'r':
         if (optarg) {
            unsigned long tmp;
            if (do_strtoul(optarg, 0, PKA_MAX_FW_WORDS, &tmp) < 0)
               return -1;
            opts->rom_base = tmp;
         } else {
      case LOPT_ROM_ONLY:
            opts->rom_base = 0;
         }
         break;
      case LOPT_RAM_ONLY:
         opts->rom_base = UINT_MAX;
         break;
      case 'e':
         opts->sym_is_ent = true;
         opts->symfile = optarg;
         break;
      case 'S':
         opts->sym_is_ent = false;
         opts->symfile = optarg;
         break;
      case 'V':
         pka_tool_version(MKIMAGE_VERSION);
         return 1;
      case 'H':
         print_help();
         return 1;
      default:
         print_usage(stderr);
         return -1;
      }
   }

   return 0;
}

int main(int argc, char **argv)
{
   struct mkimage_opts opts = { .rom_base = UINT_MAX };
   int outfd = STDOUT_FILENO;
   FILE *fsym = NULL;
   int rc;

   pka_tool_init("mkimage", argc, argv);

   rc = do_cmdline_opts(argc, argv, &opts);
   if (rc != 0) {
      return rc < 0 ? EXIT_FAILURE : EXIT_SUCCESS;
   }

   if (opts.symfile && (fsym = fopen(opts.symfile, "r")) == NULL) {
      pka_tool_err(0, "failed to open %s", opts.symfile);
      return EXIT_FAILURE;
   } else if (!opts.symfile) {
      pka_tool_err(-1, "no symbol or entry file specified");
      pka_tool_err(-1, "generated image will not have symbols");
      /* non-fatal */
   } else {
      opts.symbols = read_symbols(fsym, opts.sym_is_ent);
      if (!opts.symbols)
         return EXIT_FAILURE;
   }

   /* All remaining arguments are filenames. */
   opts.code = read_inputs(argv+optind);
   if (!opts.code)
      return EXIT_FAILURE;

   /* Write output file */
   if (opts.outfile && (outfd = creat(opts.outfile, 0666)) == -1) {
      pka_tool_err(0, "failed to open %s", opts.outfile);
      return EXIT_FAILURE;
   }

   rc = generate_image(outfd, &opts);
   if (rc != 0)
      return EXIT_FAILURE;

   return 0;
}
