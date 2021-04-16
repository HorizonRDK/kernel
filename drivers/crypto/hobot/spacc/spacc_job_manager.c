
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

/* Job manager */

/* This will reset all job data, pointers, etc */
void spacc_job_init_all (spacc_device *spacc)
{
   int x;
   spacc_job * job;

   for (x = 0; x < (SPACC_MAX_JOBS); x++) {
      job = &spacc->job[x];
      memset (job, 0, sizeof (spacc_job));
      job->job_swid = SPACC_JOB_IDX_UNUSED;
      job->job_used = SPACC_JOB_IDX_UNUSED;
      spacc->job_lookup[x] = SPACC_JOB_IDX_UNUSED;
   }

   /* if this is a secure mode HSM we have to release all of the contexts first (only issue this for the zeroth) */
   if (spacc->config.is_hsm_shared && spacc->config.is_secure_port == 1) {
      unsigned long x, timeout;
      for (x = 0; x < spacc->config.num_ctx; x++) {
         pdu_io_write32 (spacc->regmap + SPACC_REG_HSM_CTX_CMD, x);

         /* wait for ready flag */
         timeout = 1000000UL;
         while (--timeout && ((pdu_io_read32(spacc->regmap + SPACC_REG_HSM_CTX_STAT) & SPACC_CTX_STAT_RDY) != SPACC_CTX_STAT_RDY)){};
         if (!timeout) {
            ELPHW_PRINT("WARNING:  Failed to release context %lu upon init\n", x);
         }
      }
   }


}

/* get a new job id and use a specific ctx_idx or -1 for a new one */
int spacc_job_request (spacc_device * spacc, int ctx_idx)
{
   int x, ret;
   unsigned long lock_flag;
   spacc_job *job;

   if (spacc == NULL) {
      ELPHW_PRINT ("spacc_job_request::spacc cannot be NULL\n");
      return -1;
   }

   PDU_LOCK(&spacc->lock, lock_flag);
   /* find the first availble job id */
   for (x = 0; x < SPACC_MAX_JOBS; x++) {
      job = &spacc->job[x];
      if (job->job_used == SPACC_JOB_IDX_UNUSED) {
         job->job_used = x;
         break;
      }
   }

   if (x == SPACC_MAX_JOBS) {
      ELPHW_PRINT ("spacc_job_request::max number of jobs reached\n");
      ret = -1;
   } else {
      /* associate a single context to go with job */
      ret = spacc_ctx_request(spacc, ctx_idx, 1);
      if (ret != -1) {
         job->ctx_idx = ret;
         ret = x;
      }
      //ELPHW_PRINT ("spacc_job_request::ctx request [%d]\n", ret);
   }

   PDU_UNLOCK(&spacc->lock, lock_flag);
   return ret;
}

int spacc_job_release (spacc_device * spacc, int job_idx)
{
   int ret;
   unsigned long lock_flag;
   spacc_job *job;

   if (spacc == NULL) {
      return -1;
   }
   if (job_idx >= SPACC_MAX_JOBS) {
      return -1;
   }

   PDU_LOCK(&spacc->lock, lock_flag);

   job = &spacc->job[job_idx];
   /* release context that goes with job */
   ret = spacc_ctx_release(spacc, job->ctx_idx);
   job->ctx_idx  = SPACC_CTX_IDX_UNUSED;
   job->job_used = SPACC_JOB_IDX_UNUSED;
   job->cb       = NULL; // disable any callback

   /* NOTE: this leaves ctrl data in memory */

   PDU_UNLOCK(&spacc->lock, lock_flag);
   return ret;
}

/* Return a context structure for a job idx or null if invalid */
spacc_ctx * context_lookup_by_job (spacc_device * spacc, int job_idx)
{
   if ((job_idx < 0) || (job_idx >= SPACC_MAX_JOBS)) {
      ELPHW_PRINT ("context_lookup::Invalid job number\n");
      return NULL;
   }
   return &spacc->ctx[(&spacc->job[job_idx])->ctx_idx];
}

int spacc_set_auxinfo (spacc_device * spacc, int jobid, uint32_t direction, uint32_t bitsize)
{
   int ret = CRYPTO_OK;
   spacc_job *job;// = job_lookup (spacc, jobid);

   if (jobid < 0 || jobid > SPACC_MAX_JOBS) {
      return CRYPTO_INVALID_HANDLE;
   }

   job = &spacc->job[jobid];
   if (NULL == job) {
      ret = CRYPTO_FAILED;
   } else {
      job->auxinfo_dir = direction;
      job->auxinfo_bit_align = bitsize;
   }
   return ret;
}

int spacc_set_operation (spacc_device * spacc, int handle, int op, uint32_t prot, uint32_t icvcmd, uint32_t icvoff, uint32_t icvsz, uint32_t sec_key)
{
   int ret = CRYPTO_OK;
   spacc_job *job = NULL;

   if (handle < 0 || handle > SPACC_MAX_JOBS) {
      return CRYPTO_INVALID_HANDLE;
   }

   //job = job_lookup (spacc, handle);
   job = &spacc->job[handle];
   if (NULL == job) {
      ret = CRYPTO_FAILED;
   } else {
      if (op == OP_ENCRYPT) {
         job->op = OP_ENCRYPT;
         job->ctrl |= SPACC_CTRL_MASK(SPACC_CTRL_ENCRYPT);
      } else {
         job->op = OP_DECRYPT;
         job->ctrl &= ~SPACC_CTRL_MASK(SPACC_CTRL_ENCRYPT);
      }
      switch (prot) {
         case ICV_HASH:        /* HASH of plaintext */
            job->ctrl |= SPACC_CTRL_MASK(SPACC_CTRL_ICV_PT);
            break;
         case ICV_HASH_ENCRYPT: /* HASH the plaintext and encrypt the lot */
            job->ctrl |= SPACC_CTRL_MASK(SPACC_CTRL_ICV_ENC);   /* ICV_PT and ICV_APPEND must be set too */
            job->ctrl |= SPACC_CTRL_MASK(SPACC_CTRL_ICV_PT);
            job->ctrl |= SPACC_CTRL_MASK(SPACC_CTRL_ICV_APPEND);   /* This mode is not valid when BIT_ALIGN != 0 */
            break;
         case ICV_ENCRYPT_HASH: /* HASH the ciphertext */
            job->ctrl &= ~SPACC_CTRL_MASK(SPACC_CTRL_ICV_PT);    // ICV_PT=0
            job->ctrl &= ~SPACC_CTRL_MASK(SPACC_CTRL_ICV_ENC);   // ICV_ENC=0
            break;
         case ICV_IGNORE:
            break;
         default:
            ret = CRYPTO_INVALID_MODE;
            break;
      }

      job->icv_len = icvsz;

      switch (icvcmd) {
         case IP_ICV_OFFSET:
            job->icv_offset = icvoff;
            job->ctrl &= ~SPACC_CTRL_MASK(SPACC_CTRL_ICV_APPEND);
            break;
         case IP_ICV_APPEND:
            job->ctrl |= SPACC_CTRL_MASK(SPACC_CTRL_ICV_APPEND);
            break;
         case IP_ICV_IGNORE:
            break;
         default:
            ret = CRYPTO_INVALID_MODE;
            break;
      }

      if (sec_key) {
         job->ctrl |= SPACC_CTRL_MASK(SPACC_CTRL_SEC_KEY);
      }
   }
   return ret;
}


/* Return a job structure for a swid or null if invalid */
spacc_job * job_lookup_by_swid (spacc_device * spacc, int swid)
{
   if ((swid < 0) || (swid >= SPACC_MAX_JOBS)) {
      ELPHW_PRINT ("job_lookup::Invalid swid number\n");
      return NULL;
   }
   return &spacc->job[spacc->job_lookup[swid]];
}
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

int spacc_process_jb(spacc_device *spacc)
{
   // are there jobs in the buffer?
   while (spacc->jb_head != spacc->jb_tail) {
      int x, y;

      x = spacc->jb_tail;
      if (spacc->job_buffer[x].active) {
         y = spacc_packet_enqueue_ddt_ex(spacc, 0,
               spacc->job_buffer[x].job_idx,
               spacc->job_buffer[x].src,
               spacc->job_buffer[x].dst,
               spacc->job_buffer[x].proc_sz,
               spacc->job_buffer[x].aad_offset,
               spacc->job_buffer[x].pre_aad_sz,
               spacc->job_buffer[x].post_aad_sz,
               spacc->job_buffer[x].iv_offset,
               spacc->job_buffer[x].prio);

         if (y != CRYPTO_FIFO_FULL) {
            spacc->job_buffer[x].active = 0;
         } else {
            return -1;
         }
      }

      x = (x + 1);
      if (x == SPACC_MAX_JOB_BUFFERS) { x = 0; }
      spacc->jb_tail = x;
   }
   return 0;
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


// Releases a crypto context back into appropriate module's pool
int spacc_close (spacc_device * dev, int handle)
{
   int ret = CRYPTO_OK;


   if (handle < 0 || handle > SPACC_MAX_JOBS) {
      return CRYPTO_INVALID_HANDLE;
   }

   if (spacc_job_release (dev, handle)) {
      ret = CRYPTO_INVALID_HANDLE;
   }

   return ret;
}
