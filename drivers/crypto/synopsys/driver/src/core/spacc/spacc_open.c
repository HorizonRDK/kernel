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
 * Copyright (c) 2015-2018 Synopsys, Inc. and/or its affiliates.
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


int spacc_clone_handle(spacc_device *spacc, int old_handle, void *cbdata)
{
   int new_handle;

   new_handle = spacc_job_request(spacc, spacc->job[old_handle].ctx_idx);
   if (new_handle < 0) {
      return new_handle;
   }
   spacc->job[new_handle]          = spacc->job[old_handle];
   spacc->job[new_handle].job_used = new_handle;
   spacc->job[new_handle].cbdata   = cbdata;
   return new_handle;
}

/*
 Allocates a job for spacc module context and initialize it with an
 appropriate type.
 */
int spacc_open (spacc_device *spacc, int enc, int hash, int ctxid, int secure_mode, spacc_callback cb, void *cbdata)
{
   int ret = CRYPTO_OK;
   int job_idx = 0;
   spacc_job *job = NULL;
   uint32_t ctrl = 0;

   if ((job_idx = spacc_job_request (spacc, ctxid)) < 0) {
      ret = CRYPTO_FAILED;
   } else {
      job = &spacc->job[job_idx];

      if (secure_mode && job->ctx_idx > spacc->config.num_sec_ctx) {
         ELPHW_PRINT("Job context ID is outside allowed range for secure contexts\n");
         spacc_job_release (spacc, job_idx);
         return CRYPTO_FAILED;
      }

      job->auxinfo_cs_mode = 0;
      job->auxinfo_bit_align = 0;
      job->auxinfo_dir = 0;
      job->icv_len = 0;

      switch (enc) {
         case CRYPTO_MODE_RC4_40:
         case CRYPTO_MODE_RC4_128:
         case CRYPTO_MODE_RC4_KS:
            //if (ctx->rc4_key == NULL) {
            if (job->ctx_idx >= spacc->config.num_rc4_ctx) {
               spacc_job_release (spacc, job_idx);
               ELPHW_PRINT ("Out of RC4 contexts (try freeing up some job_idxs first)\n");
               return CRYPTO_FAILED;
            }
      }

      switch (enc) {
         case CRYPTO_MODE_NULL:
            break;
         case CRYPTO_MODE_RC4_40:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_RC4);
            break;
         case CRYPTO_MODE_RC4_128:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_RC4);
            break;
         case CRYPTO_MODE_RC4_KS:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_RC4);
            break;
         case CRYPTO_MODE_AES_ECB:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_AES);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_ECB);
            break;
         case CRYPTO_MODE_AES_CBC:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_AES);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_CBC);
            break;
         case CRYPTO_MODE_AES_CS1:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_AES);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_CBC);
            job->auxinfo_cs_mode = 1;
            break;
         case CRYPTO_MODE_AES_CS2:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_AES);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_CBC);
            job->auxinfo_cs_mode = 2;
            break;
         case CRYPTO_MODE_AES_CS3:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_AES);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_CBC);
            job->auxinfo_cs_mode = 3;
            break;
         case CRYPTO_MODE_AES_CFB:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_AES);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_CFB);
            break;
         case CRYPTO_MODE_AES_OFB:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_AES);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_OFB);
            break;
         case CRYPTO_MODE_AES_CTR:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_AES);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_CTR);
            break;
         case CRYPTO_MODE_AES_CCM:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_AES);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_CCM);
            break;
         case CRYPTO_MODE_AES_GCM:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_AES);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_GCM);
            break;
         case CRYPTO_MODE_AES_F8:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_AES);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_F8);
            break;
         case CRYPTO_MODE_AES_XTS:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_AES);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_XTS);
            break;
         case CRYPTO_MODE_MULTI2_ECB:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_MULTI2);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_ECB);
            break;
         case CRYPTO_MODE_MULTI2_CBC:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_MULTI2);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_CBC);
            break;
         case CRYPTO_MODE_MULTI2_OFB:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_MULTI2);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_OFB);
            break;
         case CRYPTO_MODE_MULTI2_CFB:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_MULTI2);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_CFB);
            break;
         case CRYPTO_MODE_3DES_CBC:
         case CRYPTO_MODE_DES_CBC:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_DES);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_CBC);
            break;
         case CRYPTO_MODE_3DES_ECB:
         case CRYPTO_MODE_DES_ECB:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_DES);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_ECB);
            break;
         case CRYPTO_MODE_KASUMI_ECB:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_KASUMI);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_ECB);
            break;
         case CRYPTO_MODE_KASUMI_F8:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_KASUMI);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_F8);
            break;

         case CRYPTO_MODE_SNOW3G_UEA2:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_SNOW3G_UEA2);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_ECB);
            break;

         case CRYPTO_MODE_ZUC_UEA3:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_ZUC_UEA3);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_ECB);
            break;

         case CRYPTO_MODE_CHACHA20_STREAM:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_CHACHA20);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_CHACHA_STREAM);
            break;

         case CRYPTO_MODE_CHACHA20_POLY1305:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_ALG, C_CHACHA20);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_CIPH_MODE, CM_CHACHA_AEAD);
            break;

         default:
            ret = CRYPTO_INVALID_ALG;
            break;
      }
      switch (hash) {
         case CRYPTO_MODE_NULL:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_NULL);
            break;
         case CRYPTO_MODE_HMAC_SHA1:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA1);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_HMAC);
            break;
         case CRYPTO_MODE_HMAC_MD5:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_MD5);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_HMAC);
            break;
         case CRYPTO_MODE_HMAC_SHA224:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA224);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_HMAC);
            break;

         case CRYPTO_MODE_HMAC_SHA256:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA256);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_HMAC);
            break;

         case CRYPTO_MODE_HMAC_SHA384:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA384);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_HMAC);
            break;

         case CRYPTO_MODE_HMAC_SHA512:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA512);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_HMAC);
            break;

         case CRYPTO_MODE_HMAC_SHA512_224:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA512_224);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_HMAC);
            break;

         case CRYPTO_MODE_HMAC_SHA512_256:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA512_256);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_HMAC);
            break;

         case CRYPTO_MODE_SSLMAC_MD5:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_MD5);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_SSLMAC);
            break;

         case CRYPTO_MODE_SSLMAC_SHA1:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA1);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_SSLMAC);
            break;


         case CRYPTO_MODE_HASH_SHA1:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA1);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;
         case CRYPTO_MODE_HASH_MD5:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_MD5);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;

         case CRYPTO_MODE_HASH_SHA224:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA224);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;

         case CRYPTO_MODE_HASH_SHA256:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA256);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;

         case CRYPTO_MODE_HASH_SHA384:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA384);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;

         case CRYPTO_MODE_HASH_SHA512:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA512);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;

         case CRYPTO_MODE_HASH_SHA512_224:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA512_224);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;

         case CRYPTO_MODE_HASH_SHA512_256:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA512_256);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;

         case CRYPTO_MODE_HASH_SHA3_224:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA3_224);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;
         case CRYPTO_MODE_HASH_SHA3_256:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA3_256);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;
         case CRYPTO_MODE_HASH_SHA3_384:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA3_384);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;
         case CRYPTO_MODE_HASH_SHA3_512:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHA3_512);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;

         case CRYPTO_MODE_HASH_SHAKE128:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHAKE128);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_SHAKE_SHAKE);
            break;
         case CRYPTO_MODE_HASH_SHAKE256:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHAKE256);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_SHAKE_SHAKE);
            break;
         case CRYPTO_MODE_HASH_CSHAKE128:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHAKE128);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_SHAKE_CSHAKE);
            break;
         case CRYPTO_MODE_HASH_CSHAKE256:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHAKE256);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_SHAKE_CSHAKE);
            break;
         case CRYPTO_MODE_MAC_KMAC128:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHAKE128);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_SHAKE_KMAC);
            break;
         case CRYPTO_MODE_MAC_KMAC256:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHAKE256);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_SHAKE_KMAC);
            break;
         case CRYPTO_MODE_MAC_KMACXOF128:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHAKE128);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_SHAKE_KMAC);
            job->auxinfo_dir = 1; /* auxinfo_dir reused to indicate XOF */
            break;
         case CRYPTO_MODE_MAC_KMACXOF256:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SHAKE256);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_SHAKE_KMAC);
            job->auxinfo_dir = 1; /* auxinfo_dir reused to indicate XOF */
            break;

         case CRYPTO_MODE_MAC_XCBC:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_XCBC);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;

         case CRYPTO_MODE_MAC_CMAC:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_CMAC);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;

         case CRYPTO_MODE_MAC_KASUMI_F9:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_KF9);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;

         case CRYPTO_MODE_MAC_SNOW3G_UIA2:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_SNOW3G_UIA2);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;

         case CRYPTO_MODE_MAC_ZUC_UIA3:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_ZUC_UIA3);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;

         case CRYPTO_MODE_MAC_POLY1305:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_POLY1305);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;

         case CRYPTO_MODE_HASH_CRC32:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_CRC32_I3E802_3);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE,  HM_RAW);
            break;

         case CRYPTO_MODE_MAC_MICHAEL:
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_ALG, H_MICHAEL);
            ctrl |= SPACC_CTRL_SET(SPACC_CTRL_HASH_MODE, HM_RAW);
            break;

         default:
            ret = CRYPTO_INVALID_ALG;
            break;
      }
   }

   ctrl |= SPACC_CTRL_MASK(SPACC_CTRL_MSG_BEGIN) | SPACC_CTRL_MASK(SPACC_CTRL_MSG_END);

   if (ret != CRYPTO_OK) {
      spacc_job_release (spacc, job_idx);
   } else {
      ret = job_idx;
      job->first_use   = 1;
      job->enc_mode    = enc;
      job->hash_mode   = hash;
      job->ckey_sz     = 0;
      job->hkey_sz     = 0;
      job->job_done    = 0;
      job->job_swid    = 0;
      job->job_secure  = ! !secure_mode;
      job->auxinfo_bit_align = 0;
      job->job_err     = CRYPTO_INPROGRESS;
      job->ctrl        = ctrl | SPACC_CTRL_SET(SPACC_CTRL_CTX_IDX, job->ctx_idx);
      job->cb          = cb;
      job->cbdata      = cbdata;
   }
   return ret;
}
