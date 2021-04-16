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

/* Context manager */
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

/* ctx_id is requested */
/* This function is not meant to be called directly, it should be called from the job manager */
int spacc_ctx_request(spacc_device *spacc, int ctx_id, int ncontig)
{
   int ret;
   int x, y, count;
   unsigned long lock_flag, timeout;

   if (spacc == NULL) {
      ELPHW_PRINT ("spacc_ctx_request::spacc cannot be NULL\n");
      return -1;
   }
   if (ctx_id > spacc->config.num_ctx) {
      ELPHW_PRINT ("spacc_ctx_request::ctx_id cannot be larger than availble\n");
      return -1;
   }
   if (ncontig < 1 || ncontig > spacc->config.num_ctx) {
      ELPHW_PRINT ("spacc_ctx_request::Invalid number of contiguous contexts requested\n");
      return -1;
   }
   /* test if spacc_hsm_shared, then max is 1 */
   if ((ncontig > 1) && spacc->config.is_hsm_shared) {
      ELPHW_PRINT ("spacc_ctx_request::HSM shared mode can only have 1 context requested\n");
      return -1;
   }

   ret = CRYPTO_OK;

   PDU_LOCK(&spacc->ctx_lock, lock_flag);
   /* allocating scheme, look for contiguous contexts. Free contexts have a ref_cnt of 0.  */

   if (spacc->config.is_hsm_shared) {
      /* request HSM shared CTX */
      if (ctx_id == -1) {
         /* request next available */
         ctx_id = 0;
      } else {
         /* request a specific ctx */
         ctx_id |= SPACC_CTX_CMD_IDX_REQ;
      }
//rintk("Port: %d is attempting to allocate...\n", spacc->is_secure_port);
      pdu_io_write32 (spacc->regmap + SPACC_REG_HSM_CTX_CMD, ctx_id | SPACC_CTX_CMD_REQ);

      /* wait for ready flag */
      timeout = 100000UL;
      while (--timeout && (((ctx_id = pdu_io_read32(spacc->regmap + SPACC_REG_HSM_CTX_STAT)) & SPACC_CTX_STAT_RDY) != SPACC_CTX_STAT_RDY)){};
      if (!timeout) {
         ELPHW_PRINT("spacc_ctx_request::Timeout requesting handle from HSM CTX manager\n");
         ret = CRYPTO_FAILED;
      } else {
         if (ctx_id & SPACC_CTX_STAT_SUCCESS) {
            ctx_id &= spacc->config.ctx_mask;
//printk("Port: %d got CTX %u\n", spacc->is_secure_port, ctx_id);
         } else {
            ELPHW_PRINT("spacc_ctx_request::HSM CTX manager returned an error allocating a handle\n");
            ret = CRYPTO_FAILED;
         }
      }
   } else {

      /* if specific ctx_id is requested, test the ncontig and then bump the ref_cnt */
      if (ctx_id != -1) {
         if ((&spacc->ctx[ctx_id])->ncontig != ncontig - 1) {
            ELPHW_PRINT ("spacc_ctx_request::ncontig mismatch [%d] requested but [%d] already set\n", ncontig, (&spacc->ctx[ctx_id])->ncontig +1);
            ret = -1;
         }

      } else {

         /* check to see if ncontig are free */

         /* loop over all available contexts to find the first ncontig empty ones */
         for (x = 0; x <= (spacc->config.num_ctx - ncontig); ) {
            count = ncontig;
            while (count) {
               if ((&spacc->ctx[x + count -1])->ref_cnt != 0) {
                  /* incr x to past failed count location */
                  x = x + count;
                  break;
               } else {
                  count--;
               }
            }
            if (count != 0) {
               ret = -1;
               /* test next x */
            } else {
               ctx_id = x;
               ret = CRYPTO_OK;
               break;
            }
         }
      }
   }

   if (ret == CRYPTO_OK) {
      /* ctx_id is good so mark used */
      for (y = 0; y < ncontig; y++) {
         (&spacc->ctx[ctx_id + y])->ref_cnt++;
      }
      (&spacc->ctx[ctx_id])->ncontig = ncontig - 1;
   } else {
      ctx_id = -1;
   }
   //ELPHW_PRINT ("ctx_request:: ctx_id= [%d] ncontig[%d]\n", ctx_id, ncontig);

   PDU_UNLOCK(&spacc->ctx_lock, lock_flag);
   return ctx_id;
}

int spacc_ctx_release(spacc_device *spacc, int ctx_id)
{
   int ret = CRYPTO_OK;
   unsigned long lock_flag, timeout;
   int ncontig;
   int y;

   if (ctx_id < 0 || ctx_id > spacc->config.num_ctx) {
      return -1;
   }

   PDU_LOCK(&spacc->ctx_lock, lock_flag);
   /* release the base context and contiguous block */
   ncontig = (&spacc->ctx[ctx_id])->ncontig;
   for (y = 0; y <= ncontig; y++) {
      if ((&spacc->ctx[ctx_id + y])->ref_cnt > 0) {
         (&spacc->ctx[ctx_id + y])->ref_cnt--;
      }
      //ELPHW_PRINT ("spacc_ctx_release:: ctx:[%d] refcnt[%d]\n", ctx_id, (&spacc->ctx[ctx_id + y])->ref_cnt);
   }

   if ((&spacc->ctx[ctx_id])->ref_cnt == 0) {
      (&spacc->ctx[ctx_id])->ncontig = 0;
#ifdef SECURE_MODE
      // TODO:  This driver works in harmony with "normal" kernel processes so we release the context all the time
      // normally this would be done from a "secure" kernel process (trustzone/etc).  This hack is so that SPACC.0 and
      // RE/KEP/MPM cores can both use the same context space.
      pdu_io_write32(spacc->regmap + SPACC_REG_SECURE_RELEASE, ctx_id);
#endif
   }

   if (spacc->config.is_hsm_shared) {
//printk("Port: %d freed CTX %u\n", spacc->is_secure_port, ctx_id);
      /* release HSM shared CTX */
      pdu_io_write32 (spacc->regmap + SPACC_REG_HSM_CTX_CMD, ctx_id);
      /* wait for ready flag */
      timeout = 100000UL;
      while (--timeout && ((pdu_io_read32(spacc->regmap + SPACC_REG_HSM_CTX_STAT) & SPACC_CTX_STAT_RDY) != SPACC_CTX_STAT_RDY)){};
      if (!timeout) {
         ELPHW_PRINT("Failed to release context to HSM\n");
         return CRYPTO_FAILED;
      }
   }

   PDU_UNLOCK(&spacc->ctx_lock, lock_flag);
   return ret;
}

// Function to unload data from an RC4 context.
int spacc_read_rc4_context (spacc_device * spacc, int job_idx, unsigned char * i, unsigned char * j, unsigned char * ctxdata)
{
   int ret = CRYPTO_OK;
   spacc_ctx *ctx = NULL;
   spacc_job *job = NULL;

   if (job_idx < 0 || job_idx > SPACC_MAX_JOBS) {
      return CRYPTO_INVALID_HANDLE;
   }

   job = &spacc->job[job_idx];
   ctx = context_lookup_by_job (spacc, job_idx);
   if (NULL == ctx) {
      ret = CRYPTO_FAILED;
   } else {
      unsigned char ijbuf[4];

      switch (job->enc_mode) {
         case CRYPTO_MODE_RC4_40:
         case CRYPTO_MODE_RC4_128:
         case CRYPTO_MODE_RC4_KS: break;
         default: return CRYPTO_FAILED;
      }

      pdu_from_dev32_s (ctxdata, ctx->rc4_key, SPACC_CTX_RC4_IJ >> 2, spacc_endian);
      pdu_from_dev32_s (ijbuf, ctx->rc4_key + (SPACC_CTX_RC4_IJ >> 2), 1, spacc_endian);

      *i = ijbuf[0];
      *j = ijbuf[1];
   }
   return ret;
}

// Function to load data into an RC4 context.
int spacc_write_rc4_context (spacc_device * spacc, int job_idx, unsigned char i, unsigned char j, const unsigned char * ctxdata)
{
   int ret = CRYPTO_OK;
   spacc_ctx *ctx = NULL;


   if (job_idx < 0 || job_idx > SPACC_MAX_JOBS) {
      return CRYPTO_INVALID_HANDLE;
   }

   ctx = context_lookup_by_job (spacc, job_idx);

   if (NULL == ctx) {
      ret = CRYPTO_FAILED;
   } else {
      unsigned char ij[4] = { i, j, 0, 0 };

      pdu_to_dev32_s (ctx->rc4_key, ctxdata, SPACC_CTX_RC4_IJ >> 2, spacc_endian);
      pdu_to_dev32_s (ctx->rc4_key + (SPACC_CTX_RC4_IJ >> 2), ij, 1, spacc_endian);
   }
   return ret;
}

/* This will reset all reference counts, pointers, etc */
void spacc_ctx_init_all (spacc_device *spacc)
{
   int x;
   spacc_ctx * ctx;
   unsigned long lock_flag;

   PDU_LOCK(&spacc->ctx_lock, lock_flag);
   /* initialize contexts */
   for (x = 0; x < spacc->config.num_ctx; x++) {
      ctx = &spacc->ctx[x];

      /* sets everything including ref_cnt and ncontig to 0 */
      memset (ctx, 0, sizeof (*ctx));

      ctx->ciph_key = spacc->regmap + SPACC_CTX_CIPH_KEY + (x * spacc->config.ciph_page_size);
      ctx->hash_key = spacc->regmap + SPACC_CTX_HASH_KEY + (x * spacc->config.hash_page_size);
      if (x < spacc->config.num_rc4_ctx) {
         ctx->rc4_key = spacc->regmap + SPACC_CTX_RC4_CTX + (x * 512);
      }
   }
   PDU_UNLOCK(&spacc->ctx_lock, lock_flag);
}
