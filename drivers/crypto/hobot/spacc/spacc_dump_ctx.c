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
 * Copyright (c) 2015 Synopsys, Inc. and/or its affiliates.
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

#include "elpspacc.h"

void spacc_dump_ctx(spacc_device *spacc, int ctx)
{
   uint32_t buf[256], x;

   if (ctx < 0 || ctx > SPACC_MAX_JOBS) {
      return;
   }

   ELPHW_PRINT("Dumping Cipher Context %d\n00000: ", ctx);
   pdu_from_dev32(buf, spacc->regmap + SPACC_CTX_CIPH_KEY + (spacc->config.ciph_page_size>>2) * ctx, spacc->config.ciph_page_size>>2);

   for (x = 0; x < spacc->config.ciph_page_size>>2;) {
       ELPHW_PRINT("%08lx ", (unsigned long)buf[x]);
       if (!(++x & 3) && x != (spacc->config.ciph_page_size>>2)) {
          ELPHW_PRINT("\n%05d: ", x);
       }
   }
   ELPHW_PRINT("\n");

   ELPHW_PRINT("Dumping Hash Context %d\n00000: ", ctx);
   pdu_from_dev32(buf, spacc->regmap + SPACC_CTX_HASH_KEY + (spacc->config.hash_page_size>>2) * ctx, spacc->config.hash_page_size>>2);

   for (x = 0; x < spacc->config.hash_page_size>>2;) {
       ELPHW_PRINT("%08lx ", (unsigned long)buf[x]);
       if (!(++x & 3) && x != (spacc->config.hash_page_size>>2)) {
          ELPHW_PRINT("\n%05d: ", x);
       }
   }
   ELPHW_PRINT("\n");
}
