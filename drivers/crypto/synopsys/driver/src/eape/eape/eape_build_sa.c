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

static void store32(uint32_t v, unsigned char *dst)
{
#if 0
   *dst++ = (v >> 24) & 0xFF;
   *dst++ = (v >> 16) & 0xFF;
   *dst++ = (v >> 8) & 0xFF;
   *dst++ = (v >> 0) & 0xFF;
#else
   *dst++ = (v >> 0) & 0xFF;
   *dst++ = (v >> 8) & 0xFF;
   *dst++ = (v >> 16) & 0xFF;
   *dst++ = (v >> 24) & 0xFF;
#endif
}

int eape_build_sa(eape_device * eape, int handle, elpxfrm_sa * sa)
{
   unsigned char *buf;
   unsigned long flags;
   uint32_t      tmp, x, *p;

   if (handle < 0 || handle > eape->status.num_ctx || eape->ctx[handle].allocated == 0) {
      ELPHW_PRINT("eape_build_sa::Invalid handle\n");
      return -1;
   }

   buf = eape->sa.sa_ptr_virt + handle * EAPE_SA_SIZE;

   memset(buf, 0, EAPE_SA_SIZE);
   store32(sa->ctrl, buf + EAPE_SA_CTRL);
   store32(htonl(sa->spi), buf + EAPE_SA_SPI);
   store32(sa->seqnum[1], buf + EAPE_SA_SEQ_NUM);
   store32(sa->seqnum[0], buf + EAPE_SA_SEQ_NUM + 4);
   store32(sa->hard_ttl[1], buf + EAPE_SA_HARD_TTL);
   store32(sa->hard_ttl[0], buf + EAPE_SA_HARD_TTL + 4);
   store32(sa->soft_ttl[1], buf + EAPE_SA_SOFT_TTL);
   store32(sa->soft_ttl[0], buf + EAPE_SA_SOFT_TTL + 4);
   memcpy(buf + EAPE_SA_AR_MASK,     sa->armask, 32);
   memcpy(buf + EAPE_SA_CIPHER_KEY,  sa->ckey,   32);
   memcpy(buf + EAPE_SA_CIPHER_SALT, sa->csalt,   4);
   memcpy(buf + EAPE_SA_MAC_KEY,     sa->mackey, 64);

   // swap key/salt/mac
   p = (uint32_t*)&buf[EAPE_SA_CIPHER_KEY];  for (x = 0; x < 8; x++)  { p[x] = htonl(p[x]); }
   p = (uint32_t*)&buf[EAPE_SA_CIPHER_SALT]; for (x = 0; x < 1; x++)  { p[x] = htonl(p[x]); }
   p = (uint32_t*)&buf[EAPE_SA_MAC_KEY];     for (x = 0; x < 16; x++) { p[x] = htonl(p[x]); }

   // swap 32-bit pairs in 64-bit words if we're on a SAD_64 host
   if (eape->config.version_0.sad_64) {
      p = (uint32_t*)buf;
      for (x = 0; x < (256/4); x += 2) {
         tmp = p[x+0]; p[x+0] = p[x+1]; p[x+1] = tmp;
      }
   }

   // flush the SA cache, since we don't know if this is inbound or outbound we flush both.
   PDU_LOCK(&eape->lock, flags);
   if (eape->config.version_0.rx_cache) {
      pdu_io_write32(eape->regmap + EAPE_REG_CACHE_FLUSH, 1 | (eape->sa.sa_ptr_phys + handle * EAPE_SA_SIZE));
      do { tmp = pdu_io_read32(eape->regmap + EAPE_REG_CACHE_RDY); } while (!tmp);
   }

   if (eape->config.version_0.tx_cache) {
      pdu_io_write32(eape->regmap + EAPE_REG_CACHE_FLUSH, 0 | (eape->sa.sa_ptr_phys + handle * EAPE_SA_SIZE));
      do { tmp = pdu_io_read32(eape->regmap + EAPE_REG_CACHE_RDY); } while (!tmp);
   }
   PDU_UNLOCK(&eape->lock, flags);

#ifdef EAPE_DEBUG
   printk("Buffered SA Control = %08zx\n", *(uint32_t *) (buf + EAPE_SA_CTRL));
   printk("Buffered SA SPI = %08zx \n", *(uint32_t *) (buf + EAPE_SA_SPI));
   printk("Buffered SA ARMask = %08zx%08zx%08zx%08zx%08zx%08zx%08zx%08zx \n", *(uint32_t *) (buf + EAPE_SA_AR_MASK), *(uint32_t *) (buf + EAPE_SA_AR_MASK + 4),
          *(uint32_t *) (buf + EAPE_SA_AR_MASK + 8), *(uint32_t *) (buf + EAPE_SA_AR_MASK + 12), *(uint32_t *) (buf + EAPE_SA_AR_MASK + 16),
          *(uint32_t *) (buf + EAPE_SA_AR_MASK + 20), *(uint32_t *) (buf + EAPE_SA_AR_MASK + 24), *(uint32_t *) (buf + EAPE_SA_AR_MASK + 28));
   printk("Buffered SA CIPHER KEY = %08zx%08zx%08zx%08zx%08zx%08zx%08zx%08zx \n", *(uint32_t *) (buf + EAPE_SA_CIPHER_KEY), *(uint32_t *) (buf + EAPE_SA_CIPHER_KEY + 4),
          *(uint32_t *) (buf + EAPE_SA_CIPHER_KEY + 8), *(uint32_t *) (buf + EAPE_SA_CIPHER_KEY + 12), *(uint32_t *) (buf + EAPE_SA_CIPHER_KEY + 16),
          *(uint32_t *) (buf + EAPE_SA_CIPHER_KEY + 20), *(uint32_t *) (buf + EAPE_SA_CIPHER_KEY + 24), *(uint32_t *) (buf + EAPE_SA_CIPHER_KEY + 28));
   printk("Buffered SA MAC KEY = %08zx%08zx%08zx%08zx%08zx%08zx%08zx%08zx%08zx%08zx%08zx%08zx%08zx%08zx%08zx%08zx \n", *(uint32_t *) (buf + EAPE_SA_MAC_KEY),
          *(uint32_t *) (buf + EAPE_SA_MAC_KEY + 4), *(uint32_t *) (buf + EAPE_SA_MAC_KEY + 8), *(uint32_t *) (buf + EAPE_SA_MAC_KEY + 12),
          *(uint32_t *) (buf + EAPE_SA_MAC_KEY + 16), *(uint32_t *) (buf + EAPE_SA_MAC_KEY + 20), *(uint32_t *) (buf + EAPE_SA_MAC_KEY + 24),
          *(uint32_t *) (buf + EAPE_SA_MAC_KEY + 28), *(uint32_t *) (buf + EAPE_SA_MAC_KEY + 32), *(uint32_t *) (buf + EAPE_SA_MAC_KEY + 36),
          *(uint32_t *) (buf + EAPE_SA_MAC_KEY + 40), *(uint32_t *) (buf + EAPE_SA_MAC_KEY + 44), *(uint32_t *) (buf + EAPE_SA_MAC_KEY + 48),
          *(uint32_t *) (buf + EAPE_SA_MAC_KEY + 52), *(uint32_t *) (buf + EAPE_SA_MAC_KEY + 56), *(uint32_t *) (buf + EAPE_SA_MAC_KEY + 60));
#endif

   return 0;
}
