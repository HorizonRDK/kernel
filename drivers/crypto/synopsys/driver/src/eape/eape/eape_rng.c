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

#include "eape.h"

// very minimal CLP-27 driver...
#define TRNG_CTRL (EAPE_REG_RNG + 0x00)
#define TRNG_DATA (EAPE_REG_RNG + 0x10)

#define NONCE_RESEED       0x40000000
#define NONCE_RESEED_LD    0x20000000
#define NONCE_SEED_SELECT  0x10000000

int eape_rng_init(eape_device *eape, uint32_t *rng_seed)
{
   uint32_t x;

   // init nonce_reseed
   pdu_io_write32(eape->regmap + TRNG_CTRL, NONCE_RESEED);
   for (x = 0; x < 4; x++) {
      pdu_io_write32(eape->regmap + TRNG_DATA + (x*4), rng_seed[x+0]);
   }
   pdu_io_write32(eape->regmap + TRNG_CTRL, NONCE_RESEED|NONCE_RESEED_LD);
   for (x = 0; x < 4; x++) {
      pdu_io_write32(eape->regmap + TRNG_DATA + (x*4), rng_seed[x+4]);
   }
   pdu_io_write32(eape->regmap + TRNG_CTRL, NONCE_RESEED|NONCE_RESEED_LD|NONCE_SEED_SELECT);
   pdu_io_write32(eape->regmap + TRNG_CTRL, 0);

   return 0;
}
