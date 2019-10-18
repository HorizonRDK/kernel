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

#include "elpspacc.h"

#define MIN(x, y) ( ((x)<(y)) ? (x) : (y) )

// prevent reading passed the end of the buffer
static void read_from(unsigned char *dst, unsigned char *src, int off, int n, int max)
{
   if (!dst)
      return;
   while (off < max && n) {
      *dst++ = src[off++];
      --n;
   }
}

int spacc_read_context (spacc_device * spacc, int job_idx, int op, unsigned char * key, int ksz, unsigned char * iv, int ivsz)
{
   int ret = CRYPTO_OK;
   spacc_ctx *ctx = NULL;
   spacc_job *job = NULL;
   unsigned char buf[300];
   int buflen;

   if (job_idx < 0 || job_idx > SPACC_MAX_JOBS) {
      return CRYPTO_INVALID_HANDLE;
   }

   job = &spacc->job[job_idx];
   ctx = context_lookup_by_job(spacc, job_idx);

   if (NULL == ctx) {
      ret = CRYPTO_FAILED;
   } else {
       switch (op) {
         case SPACC_CRYPTO_OPERATION:
            buflen = MIN(sizeof(buf),(unsigned)spacc->config.ciph_page_size);
            pdu_from_dev32_s(buf,  ctx->ciph_key, buflen>>2, spacc_endian);
            switch (job->enc_mode) {
               case CRYPTO_MODE_RC4_40:
                  read_from(key, buf, 0, 8, buflen);
                  break;
               case CRYPTO_MODE_RC4_128:
                  read_from(key, buf, 0, 16, buflen);
                  break;
               case CRYPTO_MODE_RC4_KS:
                  if (key && ksz == 258) {
                     ret = spacc_read_rc4_context (spacc, job_idx, &key[0], &key[1], key + 2);
                  }
                  break;
               case CRYPTO_MODE_AES_ECB:
               case CRYPTO_MODE_AES_CBC:
               case CRYPTO_MODE_AES_CS1:
               case CRYPTO_MODE_AES_CS2:
               case CRYPTO_MODE_AES_CS3:
               case CRYPTO_MODE_AES_CFB:
               case CRYPTO_MODE_AES_OFB:
               case CRYPTO_MODE_AES_CTR:
               case CRYPTO_MODE_AES_CCM:
               case CRYPTO_MODE_AES_GCM:
                  read_from(key, buf, 0, ksz, buflen);
                  read_from(iv, buf,  32, 16, buflen);
                  break;
               case CRYPTO_MODE_AES_F8:
                  if (key) {
                     read_from(key+ksz, buf, 0,  ksz, buflen);
                     read_from(key,     buf, 48, ksz, buflen);
                  }
                  read_from(iv, buf, 32, 16, buflen);
                  break;
               case CRYPTO_MODE_AES_XTS:
                  if (key) {
                     read_from(key,            buf,  0, ksz>>1, buflen);
                     read_from(key + (ksz>>1), buf, 48, ksz>>1, buflen);
                  }
                  read_from(iv, buf, 32, 16, buflen);
                  break;
               case CRYPTO_MODE_MULTI2_ECB:
               case CRYPTO_MODE_MULTI2_CBC:
               case CRYPTO_MODE_MULTI2_OFB:
               case CRYPTO_MODE_MULTI2_CFB:
                  read_from(key, buf, 0, ksz, buflen);
                  // Number of rounds at the end of the IV
                  read_from(iv, buf, 0x28, ivsz, buflen);
                  break;
               case CRYPTO_MODE_3DES_CBC:
               case CRYPTO_MODE_3DES_ECB:
                  read_from(iv,  buf, 0,  8, buflen);
                  read_from(key, buf, 8, 24, buflen);
                  break;
               case CRYPTO_MODE_DES_CBC:
               case CRYPTO_MODE_DES_ECB:
                  read_from(iv,  buf, 0, 8, buflen);
                  read_from(key, buf, 8, 8, buflen);
                  break;

               case CRYPTO_MODE_KASUMI_ECB:
               case CRYPTO_MODE_KASUMI_F8:
                  read_from(iv,  buf, 16,  8, buflen);
                  read_from(key, buf, 0,  16, buflen);
                  break;

               case CRYPTO_MODE_SNOW3G_UEA2:
               case CRYPTO_MODE_ZUC_UEA3:
                  read_from(key, buf, 0, 32, buflen);
                  break;

               case CRYPTO_MODE_NULL:
               default:
                  break;
            }

            break;
         case SPACC_HASH_OPERATION:
            buflen = MIN(sizeof(buf),(unsigned)spacc->config.hash_page_size);
            pdu_from_dev32_s(buf, ctx->hash_key, buflen>>2, spacc_endian);
            switch (job->hash_mode) {
               case CRYPTO_MODE_MAC_XCBC:
                  if (key && ksz <= 64) {
                     read_from(key + (ksz - 32), buf, 32, 32,       buflen);
                     read_from(key,              buf, 0,  ksz - 32, buflen);
                  }
                  break;
               case CRYPTO_MODE_HASH_CRC32:
                  read_from(iv, buf, 0, ivsz, buflen);
                  break;
               case CRYPTO_MODE_MAC_SNOW3G_UIA2:
               case CRYPTO_MODE_MAC_ZUC_UIA3:
                  read_from(key, buf, 0,  32, buflen);
                  break;
               default:
                  read_from(key, buf, 0, ksz, buflen);
            }
            break;
         default:
            ret = CRYPTO_INVALID_MODE;
            break;
      }
   }
   return ret;
}
