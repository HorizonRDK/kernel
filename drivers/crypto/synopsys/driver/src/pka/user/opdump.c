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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <unistd.h>
#include <assert.h>

#include "pkadev.h"
#include "common.h"

static const char sopts[] = "d:s:VH";
static const struct option lopts[] = {
   { "device",  1, NULL, 'd' },
   { "size",    1, NULL, 's' },
   { "version", 0, NULL, 'V' },
   { "help",    0, NULL, 'H' },
   { 0 }
};

static void print_usage(FILE *f)
{
   fprintf(f, "usage: %s [-d device] [-s size] [operand ...]\n", pka_tool_name());
}

static void dump_param(unsigned size, const unsigned char *buf, int indent)
{
   unsigned i;

   for (i = 0; i < size; i++) {
      if (i % 16 == 0)
         printf("%*s%.4x:", indent, "", i);
      if (i % 2 == 0)
         putchar(' ');

      printf("%.2x", (unsigned)buf[i]);

      if ((i+1) % 16 == 0)
         putchar('\n');
   }

   if (i % 16)
      putchar('\n');
}

static void dump_all_operands(int fd, unsigned size)
{
   char name[3] = "A0", banks[] = "ABCD";
   unsigned char buf[512];
   unsigned i, j;
   int rc;

   assert(size <= sizeof buf);

   for (i = 0; i < sizeof banks - 1; i++) {
      for (j = 0; j < 8; j++) {
         name[0] = banks[i];
         name[1] = '0' + j;

         rc = elppka_get_operand(fd, NULL, name, size, buf);
         if (rc == -1) {
            if (errno != ENOENT)
               pka_tool_err(0, "elppka_get_operand: %s", name);
            break;
         }

         printf("%s(%u):\n", name, size*8);
         dump_param(size, buf, 3);
      }
   }
}

static void dump_operands(int fd, unsigned size, char **names)
{
   unsigned char buf[512];
   unsigned i;
   int rc;

   for (i = 0; names[i]; i++) {
      rc = elppka_get_operand(fd, NULL, names[i], size, buf);
      if (rc == -1) {
         pka_tool_err(0, "elppka_get_operand: %s", names[i]);
         continue;
      }

      printf("%s(%u):\n", names[i], size*8);
      dump_param(size, buf, 3);
   }
}

int main(int argc, char **argv)
{
   const char *dev = NULL, *size_str = "128";
   unsigned long size;
   int opt, fd;

   pka_tool_init("opdump", argc, argv);

   while ((opt = getopt_long(argc, argv, sopts, lopts, NULL)) != -1) {
      switch (opt) {
      case 'd':
         dev = optarg;
         break;
      case 's':
         size_str = optarg;
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

   size = strtoul(size_str, NULL, 0);
   if (!size || size > 512) {
      pka_tool_err(-1, "invalid operand size: %s", size_str);
      return EXIT_FAILURE;
   }

   fd = elppka_device_open(dev);
   if (fd == -1) {
      if (dev)
         pka_tool_err(0, "%s", dev);
      else
         pka_tool_err(0, NULL);
      return EXIT_FAILURE;
   }

   if (argv[optind]) {
      dump_operands(fd, size, &argv[optind]);
   } else {
      dump_all_operands(fd, size);
   }

   elppka_device_close(fd);
   return 0;
}
