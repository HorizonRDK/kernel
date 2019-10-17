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
//   Toplevel parser for PKA F/W .hex files (IEEE-1364 $readmemh-style input)
//
// ------------------------------------------------------------------------

#include <stdio.h>

#include "readmemh-parse.h"
#include "readmemh-scan.h"
#include "readmemh.h"

int pka_parse_fw_hex(FILE *f, unsigned long *fw_out)
{
   uintmax_t fw_data[PKA_MAX_FW_WORDS], fw_xmask[PKA_MAX_FW_WORDS];
   struct readmemh_addrinfo addr = { .max = PKA_MAX_FW_WORDS };
   int rc, top_word = 0;
   yyscan_t scanner;

   for (size_t i = 0; i < sizeof fw_xmask / sizeof fw_xmask[0]; i++) {
      fw_xmask[i] = -1;
   }

   rc = readmemh_yylex_init(&scanner);
   if (rc != 0)
      return -1;

   readmemh_yyset_in(f, scanner);
   rc = readmemh_yyparse(scanner, &addr, fw_data, fw_xmask, NULL);
   readmemh_yylex_destroy(scanner);

   if (rc != 0)
      return -1;

   for (size_t i = 0; i < sizeof fw_data / sizeof fw_data[0]; i++) {
      if ((fw_data[i] & ~fw_xmask[i]) > 0xffffffff) {
         return -1;
      }

      fw_out[i] = fw_data[i] & ~fw_xmask[i];
      if (fw_xmask[i] != -1)
         top_word = i;
   }

   return top_word+1;
}
