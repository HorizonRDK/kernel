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
 * Copyright (c) 2012-2015 Synopsys, Inc. and/or its affiliates.
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
#include <unistd.h>
#include <assert.h>
#include <getopt.h>

#include "pkadev.h"
#include "common.h"
#include "diag/elpcurve.h"
#include "diag/SECP_160.inc"
#include "diag/SECP_521.inc"

static const char sopts[] = "d:VH";
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

static int dumb_pmult(int fd)
{
   static const unsigned char key[] = {
      0x49, 0x96, 0xd2, 0x7a, 0xd5, 0x6f, 0x2a, 0x89,
      0x4a, 0x46, 0xa9, 0x1e, 0x88, 0xcf, 0x06, 0x05,
      0x76, 0x6f, 0x7b, 0x2a
   }, expect_x[] = {
      0x73, 0x8d, 0x53, 0xb9, 0x27, 0x6c, 0x87, 0x75,
      0x9c, 0xb6, 0x09, 0x40, 0xc1, 0x94, 0xbc, 0xac,
      0xb4, 0x2b, 0xf3, 0xaf,
   }, expect_y[] = {
      0xa7, 0x01, 0x30, 0x95, 0x01, 0x20, 0x0d, 0x3a,
      0xaa, 0x2e, 0x88, 0xcc, 0x4e, 0xbb, 0xad, 0x13,
      0x89, 0x96, 0xb7, 0x4e,
   }, zero[sizeof key];

   unsigned char result_x[sizeof expect_x], result_y[sizeof expect_y];
   const EC_CURVE_DATA *curve = &SEC160R1;
   int rc;

   rc = elppka_run(fd, "pmult", curve->size,
                        "%A2",  curve->x,
                        "%B2",  curve->y,
                        "%A6",  curve->a,
                        "%A7",  zero,
                        "%D0",  curve->m,
                        "%D1",  curve->mp,
                        "%D3",  curve->r,
                        "%D7",  key,
                        "=%A2", result_x,
                        "=%B2", result_y,
                        (char *)NULL);
   if (rc == -1) {
      pka_tool_err(0, "pmult");
      return -1;
   } else if (rc > 0) {
      pka_tool_err(-1, "pmult: device returned %d\n", rc);
      return -1;
   }

   if (memcmp(expect_x, result_x, curve->size)) {
      pka_tool_err(-1, "pmult: X mismatch");
      return -1;
   } else if (memcmp(expect_y, result_y, curve->size)) {
      pka_tool_err(-1, "pmult: Y mismatch");
      return -1;
   }

   return 0;
}

static int dumb_eccblind(int fd)
{
   int rc;

   if (dumb_pmult(fd) != 0) {
      pka_tool_err(-1, "non-blinded pmult failed");
      return EXIT_FAILURE;
   }

   rc = elppka_set_flag(fd, NULL, "F0");
   if (rc == -1) {
      pka_tool_err(0, "SETF");
   }

   if (dumb_pmult(fd) == 0) {
      pka_tool_err(-1, "zero-blinded pmult failed to fail");
      return EXIT_FAILURE;
   }

   return 0;
}

int main(int argc, char **argv)
{
   const char *dev = NULL;
   int opt, fd;

   pka_tool_init("eccblind", argc, argv);

   while ((opt = getopt_long(argc, argv, sopts, lopts, NULL)) != -1) {
      switch (opt) {
      case 'd':
         dev = optarg;
         break;
      case 'V':
         pka_tool_version(NULL);
         return EXIT_SUCCESS;
      case 'H':
         print_usage(stdout);
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

   if (dumb_eccblind(fd) != 0)
      return EXIT_FAILURE;

   elppka_device_close(fd);
   return 0;
}
