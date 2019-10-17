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
 * Copyright (c) 2016 Synopsys, Inc. and/or its affiliates.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <unistd.h>
#include <assert.h>

#include "pkadev.h"
#include "common.h"

struct lfg_state {
   unsigned char s[17];
   unsigned k, j;
};

/*
 * ALFG(5, 17) algorithm implementation
 */
static void lfg(struct lfg_state *state, void *out_, size_t n)
{
   unsigned char *out = out_;
   size_t i;

   for (i = 0; i < n; i++) {
      out[i] = (state->s[state->k] += state->s[state->j]);
      if (!state->k--)
         state->k = 16;
      if (!state->j--)
         state->j = 16;
   }
}

static const char sopts[] = "d:s:VH";
static const struct option lopts[] = {
   { "device",  1, NULL, 'd' },
   { "version", 0, NULL, 'V' },
   { "help",    0, NULL, 'H' },
   { 0 }
};

static void print_usage(FILE *f)
{
   fprintf(f, "usage: %s [-d device]\n", pka_tool_name());
}

/*
 * Banks A–C: The first of the following for which the configuration
 * supports all operands will cover the entire memory:
 *
 *   - 2× 4K operands (1024 bytes)
 *   - 8× 1K operands (1024 bytes)
 *   - 2× 2K operands (512 bytes)
 *   - 8× 512b operands (512 bytes)
 *   - 2× 1K operands (256 bytes)
 *   - 8× 256b operands (256 bytes)
 *
 * Bank D:
 *
 *   - 4× 4K operands (2048 bytes)
 *   - 8× 1K operands (1024 bytes)
 *   - 4× 2K operands (1024 bytes)
 *   - 8x 512b operands (512 bytes)
 *   - 4× 1K operands (512 bytes)
 *   - 8x 256b operands (256 bytes)
 */

static int rw_all_mem_specmode(int fd, int mode, int read, void *buf)
{
   const char banks[4] = "ABCD";
   unsigned size, i, j;
   char *pos = buf;

   size = (mode % 2 ? 128 : 512) >> mode/2;
   for (i = 0; i < 4; i++) {
      unsigned count = mode % 2 ? 8 : 2 << (banks[i] == 'D');

      for (j = 0; j < count; j++) {
         char name[3] = { banks[i], '0'+j };

         if (read && elppka_get_operand(fd, NULL, name, size, pos) == -1)
            return -1;
         if (!read && elppka_set_operand(fd, NULL, name, size, pos) == -1)
            return -1;
         pos += size;
      }
   }

   return pos-(char *)buf;
}

/* Write the entire operand memory as a flat region. */
static int write_all_mem(int fd, const void *buf)
{
   int mode, ret;

   for (mode = 0; mode < 6; mode++) {
      ret = rw_all_mem_specmode(fd, mode, 0, (void *)buf);
      if (ret >= 0)
         return ret;
   }

   return -1;
}

/* Read the entire operand memory as a flat region. */
static int read_all_mem(int fd, void *buf)
{
   int mode, ret;

   for (mode = 0; mode < 6; mode++) {
      ret = rw_all_mem_specmode(fd, mode, 1, buf);
      if (ret >= 0)
         return ret;
   }

   return -1;
}

static int check_mem(int fd, const void *pattern, size_t n)
{
   unsigned char result[n];
   int wsize, rsize;

   memset(result, 0, sizeof result);
   fflush(stdout);

   wsize = write_all_mem(fd, pattern);
   if (wsize < 0) {
      int saved_errno = errno;
      printf(" FAILED\n");
      pka_tool_err(saved_errno, "error writing operand memory");
      return -1;
   }
   putchar('.');
   fflush(stdout);

   rsize = read_all_mem(fd, result);
   if (rsize < 0) {
      int saved_errno = errno;
      printf(" FAILED\n");
      pka_tool_err(saved_errno, "error reading operand memory");
      return -1;
   }
   putchar('.');
   fflush(stdout);

   if (rsize != wsize || memcmp(pattern, result, wsize) != 0) {
      printf(" FAILED, data mismatch\n");
      return 1;
   }

   printf(" OK (%d bytes)\n", wsize);
   return 0;
}

static int do_memtest(int fd)
{
   unsigned char pattern[1024*3+2048];
   unsigned char fixedpat[] = { 0xAA, 0x55, 0xff, 0x5a, 0x6b, 0x00 };
   int failed = 0;
   unsigned i;

   struct lfg_state rng = {
      .s = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
             0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
             0xf0 },
      .k = 16, .j = 4,
   };

   for (i = 0; i < sizeof fixedpat; i++) {
      memset(pattern, fixedpat[i], sizeof pattern);

      printf("Fixed pattern 0x%.8lx", 0x1010101ul * fixedpat[i]);
      switch (check_mem(fd, pattern, sizeof pattern)) {
      case -1: return -1;
      case 0:  break;
      default: failed = 1;
      }
   }

   lfg(&rng, pattern, sizeof pattern);
   printf("Random pattern");
   switch (check_mem(fd, pattern, sizeof pattern)) {
   case -1: return -1;
   case 0:  break;
   default: failed = 1;
   }

   return failed ? -1 : 0;
}

int main(int argc, char **argv)
{
   const char *dev = NULL;
   int ret = EXIT_SUCCESS;
   int opt, fd, rc;

   pka_tool_init("opdump", argc, argv);

   while ((opt = getopt_long(argc, argv, sopts, lopts, NULL)) != -1) {
      switch (opt) {
      case 'd':
         dev = optarg;
         break;
      case 'H':
         print_usage(stdout);
         return EXIT_SUCCESS;
      case 'V':
         pka_tool_version(NULL);
         return EXIT_SUCCESS;
      default:
         print_usage(stderr);
         return EXIT_FAILURE;
      }
   }

   fd = elppka_device_open(dev);
   if (fd == -1) {
      if (dev)
         pka_tool_err(0, "%s", dev);
      else
         pka_tool_err(0, NULL);
      return EXIT_FAILURE;
   }

   rc = do_memtest(fd);
   if (rc < 0)
      ret = EXIT_FAILURE;

   elppka_device_close(fd);
   return ret;
}
