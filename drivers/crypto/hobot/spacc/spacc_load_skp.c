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

int spacc_load_skp(spacc_device *spacc, uint32_t *key, int keysz, int idx, int alg, int mode, int size, int enc, int dec)
{
   uint32_t t;
   unsigned long lock_flags;

   // only secure mode can use it
   if (spacc->config.is_secure_port == 0) {
      ELPHW_PRINT("spacc_load_skp:: Only the secure port can use the SKP\n");
      return CRYPTO_FAILED;
   }

   if (keysz*4 > spacc->config.sec_ctx_page_size) {
      ELPHW_PRINT("spacc_load_skp:: Invalid key size for secure key\n");
      return CRYPTO_FAILED;
   }

   if (idx < 0 || idx > spacc->config.num_sec_ctx) {
      ELPHW_PRINT("spacc_load_skp:: Invalid CTX id specified (out of range)\n");
      return CRYPTO_FAILED;
   }

   PDU_LOCK(&spacc->lock, lock_flags);

   // wait for busy to clear
   t = 100000UL;
   while(--t && (pdu_io_read32(spacc->regmap + SPACC_REG_SK_STAT) & SPACC_SK_STAT_BUSY));
   if (!t) {
      ELPHW_PRINT("spacc_load_skp:: SK_STAT never cleared\n");
      PDU_UNLOCK(&spacc->lock, lock_flags);
      return CRYPTO_FAILED;
   }

   // copy key
   pdu_to_dev32(spacc->regmap + SPACC_REG_SK_KEY, key, keysz);

   // set reg
   t = (idx << _SPACC_SK_LOAD_CTX_IDX) |
       (alg << _SPACC_SK_LOAD_ALG)     |
       (mode << _SPACC_SK_LOAD_MODE)   |
       (size << _SPACC_SK_LOAD_SIZE)   |
       (enc << _SPACC_SK_LOAD_ENC_EN)  |
       (dec << _SPACC_SK_LOAD_DEC_EN);

   pdu_io_write32(spacc->regmap + SPACC_REG_SK_LOAD, t);

   PDU_UNLOCK(&spacc->lock, lock_flags);

   return CRYPTO_OK;
}


