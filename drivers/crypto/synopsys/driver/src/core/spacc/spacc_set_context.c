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
 * Copyright (c) 2011-2018 Synopsys, Inc. and/or its affiliates.
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

static void put_buf(unsigned char *dst, const unsigned char *src, int off, int n, int len)
{
   if (!src)
      return;
   while (n && (off < len)) {
      dst[off++] = *src++;
      --n;
   }
}

/* Write appropriate context data which depends on operation and mode */
int spacc_write_context (spacc_device * spacc, int job_idx, int op, const unsigned char * key, int ksz, const unsigned char * iv, int ivsz)
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
  
   if ((NULL == job) || (NULL == ctx)) {
      ret = CRYPTO_FAILED;
   } else {
      switch (op) {
         case SPACC_CRYPTO_OPERATION:
            // get page size and then read so we can do a read-modify-write cycle
            buflen = MIN(sizeof(buf),(unsigned)spacc->config.ciph_page_size);
            pdu_from_dev32_s(buf,  ctx->ciph_key, buflen>>2, spacc_endian);
            switch (job->enc_mode) {
               case CRYPTO_MODE_RC4_40:
                  put_buf(buf, key, 0, 8, buflen);
                  break;
               case CRYPTO_MODE_RC4_128:
                  put_buf(buf, key, 0, 16, buflen);
                  break;
               case CRYPTO_MODE_RC4_KS:
                  if (key) {
                     ret = spacc_write_rc4_context (spacc, job_idx, key[0], key[1], key + 2);
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
                  put_buf(buf, key, 0, ksz, buflen);
                  if (iv) {
                     unsigned char one[4] = { 0, 0, 0, 1 };
                     put_buf(buf, iv, 32, ivsz, buflen);
                     if (ivsz == 12 && job->enc_mode == CRYPTO_MODE_AES_GCM) {
                        put_buf(buf, one, 11*4, 4, buflen);
                     }
                  }
                  break;
               case CRYPTO_MODE_AES_F8:
                  if (key) {
                     put_buf(buf, key + ksz, 0,  ksz, buflen);
                     put_buf(buf, key,       48, ksz, buflen);
                  }
                  put_buf(buf, iv,        32,  16, buflen);
                  break;
               case CRYPTO_MODE_AES_XTS:
                  if (key) {
                     put_buf(buf, key,           0, ksz>>1, buflen);
                     put_buf(buf, key+(ksz>>1), 48, ksz>>1, buflen);
                     ksz = ksz >> 1;   // divide by two since that's what we program the hardware with
                  }
                  put_buf(buf, iv, 32, 16, buflen);
                  break;
               case CRYPTO_MODE_MULTI2_ECB:
               case CRYPTO_MODE_MULTI2_CBC:
               case CRYPTO_MODE_MULTI2_OFB:
               case CRYPTO_MODE_MULTI2_CFB:
                  put_buf(buf, key,   0, ksz,  buflen);
                  put_buf(buf, iv, 0x28, ivsz, buflen);

                  if (ivsz <= 8) {
                     unsigned char rounds[4] = {0, 0, 0, 128}; // default to 128 rounds
                     put_buf(buf, rounds, 0x30, 4, buflen);
                  }
                  break;
               case CRYPTO_MODE_3DES_CBC:
               case CRYPTO_MODE_3DES_ECB:
               case CRYPTO_MODE_DES_CBC:
               case CRYPTO_MODE_DES_ECB:
                  put_buf(buf, iv, 0, 8, buflen);
                  put_buf(buf, key, 8, ksz, buflen);
                  break;

               case CRYPTO_MODE_KASUMI_ECB:
               case CRYPTO_MODE_KASUMI_F8:
                  put_buf(buf, iv, 16, 8, buflen);
                  put_buf(buf, key, 0, 16, buflen);
                  break;

               case CRYPTO_MODE_SNOW3G_UEA2:
               case CRYPTO_MODE_ZUC_UEA3:
                  put_buf(buf, key, 0, 32, buflen);
                  break;

               case CRYPTO_MODE_CHACHA20_STREAM:
               case CRYPTO_MODE_CHACHA20_POLY1305:
                  put_buf(buf, key, 0, ksz, buflen);
                  put_buf(buf, iv, 32, ivsz, buflen);
                  break;

               case CRYPTO_MODE_NULL:
               default:
                  break;
            }
            if (key) {
               job->ckey_sz = SPACC_SET_CIPHER_KEY_SZ (ksz);
               job->first_use = 1;
            }
            pdu_to_dev32_s (ctx->ciph_key, buf, buflen >> 2, spacc_endian);
            break;
         case SPACC_HASH_OPERATION:
            // get page size and then read so we can do a read-modify-write cycle
            buflen = MIN(sizeof(buf),(unsigned)spacc->config.hash_page_size);
            pdu_from_dev32_s(buf,  ctx->hash_key, buflen>>2, spacc_endian);
            switch (job->hash_mode) {
               case CRYPTO_MODE_MAC_XCBC:
                  if (key) {
                     put_buf(buf, key + (ksz - 32), 32,         32, buflen);
                     put_buf(buf, key,               0, (ksz - 32), buflen);
                     job->hkey_sz = SPACC_SET_HASH_KEY_SZ (ksz - 32);
                  }
                  break;
               case CRYPTO_MODE_HASH_CRC32:
               case CRYPTO_MODE_MAC_SNOW3G_UIA2:
               case CRYPTO_MODE_MAC_ZUC_UIA3:
                  if (key) {
                     put_buf(buf, key, 0, ksz, buflen);
                     job->hkey_sz = SPACC_SET_HASH_KEY_SZ (ksz);
                  }
                  break;
               case CRYPTO_MODE_MAC_POLY1305:
                  put_buf(buf, key, 0, ksz, buflen);
                  put_buf(buf, iv, 32, ivsz, buflen);
                  break;
               case CRYPTO_MODE_HASH_CSHAKE128:
               case CRYPTO_MODE_HASH_CSHAKE256:
                  // use "iv" and "key" to pass s-string and n-string
                  put_buf(buf, iv, 0, ivsz, buflen);
                  put_buf(buf, key, spacc->config.string_size, ksz, buflen);
                  break;
               case CRYPTO_MODE_MAC_KMAC128:
               case CRYPTO_MODE_MAC_KMAC256:
               case CRYPTO_MODE_MAC_KMACXOF128:
               case CRYPTO_MODE_MAC_KMACXOF256:
                  // use "iv" and "key" to pass s-string and key
                  put_buf(buf, iv, 0, ivsz, buflen);
                  put_buf(buf, key, spacc->config.string_size, ksz, buflen);
                  job->hkey_sz = SPACC_SET_HASH_KEY_SZ (ksz);
                  break;

               default:
                  if (key) {
                     job->hkey_sz = SPACC_SET_HASH_KEY_SZ (ksz);
                     put_buf(buf, key, 0, ksz, buflen);
                  }
            }
            pdu_to_dev32_s (ctx->hash_key, buf, buflen >> 2, spacc_endian);
            break;
         default:
            ret = CRYPTO_INVALID_MODE;
            break;
      }
   }
   return ret;
}
