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

/**************************************************************************
 * Function - eape_init
 *
 * Description - Used to initialize the eape device
 *
 * Inputs - Pointer to eape device
 *          Device address
 *          Number of available contexts
 *
 * Outputs - Error status
 *
 **************************************************************************/
int eape_init(eape_device * eape, void *regmap, int no_ctx)
{
   int x;
   uint32_t tmp;

   eape->regmap = regmap;

   eape->status.num_ctx    = no_ctx;
   eape->sa.sa_ptr_mem_req = EAPE_SA_SIZE * no_ctx;

   //Allocate memory for each context
   eape->ctx = pdu_malloc(sizeof(*(eape->ctx)) * no_ctx);
   if (!eape->ctx) {
      ELPHW_PRINT("Out of memory\n");
      return -1;
   }
   //Initialize memory
   memset(eape->ctx, 0, sizeof(*(eape->ctx)) * no_ctx);
   memset(&(eape->cache), 0, sizeof(eape->cache));

   pdu_io_write32(regmap + EAPE_REG_OUT_SRC_PTR, 0);
   pdu_io_write32(regmap + EAPE_REG_OUT_DST_PTR, 0);
   pdu_io_write32(regmap + EAPE_REG_OUT_OFFSET, 0);
   pdu_io_write32(regmap + EAPE_REG_IN_SRC_PTR, 0);
   pdu_io_write32(regmap + EAPE_REG_IN_DST_PTR, 0);
   pdu_io_write32(regmap + EAPE_REG_IN_OFFSET, 0);

   for (x = 0; x < 512; x++) {
      eape->swid[x] = 0xFFFFFFFF;
   }

   // read version registers ...
   tmp = pdu_io_read32(regmap + EAPE_REG_VERSION0);
   ELPHW_PRINT("eape: VERSION0=%08zx\n", tmp);
   eape->config.version_0.minor    = EAPE_GET_REG_BITS(tmp, EAPE_VERSION0_VER_MINOR_MASK, EAPE_VERSION0_VER_MINOR_OFFSET);
   eape->config.version_0.major    = EAPE_GET_REG_BITS(tmp, EAPE_VERSION0_VER_MAJOR_MASK, EAPE_VERSION0_VER_MAJOR_OFFSET);
   eape->config.version_0.dma_type = EAPE_GET_REG_BITS(tmp, EAPE_VERSION0_DMA_TYPE_MASK,  EAPE_VERSION0_DMA_TYPE_OFFSET);
   eape->config.version_0.ipv6     = EAPE_GET_REG_BITS(tmp, EAPE_VERSION0_IPV6_MASK,      EAPE_VERSION0_IPV6_OFFSET);
   eape->config.version_0.tx_cache = EAPE_GET_REG_BITS(tmp, EAPE_VERSION0_TX_CACHE_MASK,  EAPE_VERSION0_TX_CACHE_OFFSET);
   eape->config.version_0.rx_cache = EAPE_GET_REG_BITS(tmp, EAPE_VERSION0_RX_CACHE_MASK,  EAPE_VERSION0_RX_CACHE_OFFSET);
   eape->config.version_0.sad_64   = EAPE_GET_REG_BITS(tmp, EAPE_VERSION0_SAD_64_MASK,    EAPE_VERSION0_SAD_64_OFFSET);
   eape->config.version_0.pbm_64   = EAPE_GET_REG_BITS(tmp, EAPE_VERSION0_PBM_64_MASK,    EAPE_VERSION0_PBM_64_OFFSET);
   eape->config.version_0.project  = EAPE_GET_REG_BITS(tmp, EAPE_VERSION0_PROJECT_MASK,   EAPE_VERSION0_PROJECT_OFFSET);

   tmp = pdu_io_read32(regmap + EAPE_REG_VERSION1);
   ELPHW_PRINT("eape: VERSION1=%08zx\n", tmp);
   eape->config.version_1.fifo_depth = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_FIFO_DEPTH_MASK,      EAPE_VERSION1_FIFO_DEPTH_OFFSET);
   eape->config.version_1.rng        = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_RNG_PRESENT_MASK,     EAPE_VERSION1_RNG_PRESENT_OFFSET);
   eape->config.version_1.md5        = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_MD5_ENABLED_MASK,     EAPE_VERSION1_MD5_ENABLED_OFFSET);
   eape->config.version_1.sha1       = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_SHA1_ENABLED_MASK,    EAPE_VERSION1_SHA1_ENABLED_OFFSET);
   eape->config.version_1.sha256     = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_SHA256_ENABLED_MASK,  EAPE_VERSION1_SHA256_ENABLED_OFFSET);
   eape->config.version_1.sha384     = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_SHA384_ENABLED_MASK,  EAPE_VERSION1_SHA384_ENABLED_OFFSET);
   eape->config.version_1.sha512     = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_SHA512_ENABLED_MASK,  EAPE_VERSION1_SHA512_ENABLED_OFFSET);
   eape->config.version_1.tx_pipes   = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_TX_PIPES_MASK,        EAPE_VERSION1_TX_PIPES_OFFSET);
   eape->config.version_1.rx_pipes   = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_RX_PIPES_MASK,        EAPE_VERSION1_RX_PIPES_OFFSET);
   eape->config.version_1.des_cbc    = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_DESCBC_ENABLED_MASK,  EAPE_VERSION1_DESCBC_ENABLED_OFFSET);
   eape->config.version_1.aes_cbc    = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_AESCBC_ENABLED_MASK,  EAPE_VERSION1_AESCBC_ENABLED_OFFSET);
   eape->config.version_1.aes_ctr    = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_AESCTR_ENABLED_MASK,  EAPE_VERSION1_AESCTR_ENABLED_OFFSET);
   eape->config.version_1.aes_gcm    = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_AESGCM_ENABLED_MASK,  EAPE_VERSION1_AESGCM_ENABLED_OFFSET);
   eape->config.version_1.aes_128    = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_AES128_ENABLED_MASK,  EAPE_VERSION1_AES128_ENABLED_OFFSET);
   eape->config.version_1.aes_192    = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_AES192_ENABLED_MASK,  EAPE_VERSION1_AES192_ENABLED_OFFSET);
   eape->config.version_1.aes_256    = EAPE_GET_REG_BITS(tmp, EAPE_VERSION1_AES256_ENABLED_MASK,  EAPE_VERSION1_AES256_ENABLED_OFFSET);

   //Initialize our IRQ triggers
   eape->status.stat_cnt = eape->config.version_1.fifo_depth;

   //Initialize the locking function
   PDU_INIT_LOCK(&eape->lock);

   return 0;
}
