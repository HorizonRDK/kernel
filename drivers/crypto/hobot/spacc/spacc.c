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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/io.h>

#include <linux/fs.h>
#include <linux/uaccess.h>
#include <asm/param.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/miscdevice.h>

#include "elppdu.h"
#include "elpspacc.h"
#include "elpspaccdrv.h"
#include "elpspacc_irq.h"

int spacc_endian;
module_param(spacc_endian, int, 0);
MODULE_PARM_DESC(spacc_endian, "Endianess of data transfers (0==little)");
EXPORT_SYMBOL(spacc_endian);

static unsigned long oldtimer = 100000, timer = 100000; // 1ms @45MHz
module_param(timer, ulong, 0600);
MODULE_PARM_DESC(timer, "Watchdog timer value (default==0xAFC8 which is 1ms@45MHz)");

static bool no_latency;
module_param(no_latency, bool, 0600);
MODULE_PARM_DESC(no_latency, "Set to 1 to have a low latency IRQ mechanism");


static inline uint32_t _spacc_get_stat_cnt (spacc_device * spacc)
{
   uint32_t fifo;

   if (spacc->config.is_qos) {
      fifo = SPACC_FIFO_STAT_STAT_CNT_GET_QOS (pdu_io_read32 (spacc->regmap + SPACC_REG_FIFO_STAT));
   } else {
      fifo = SPACC_FIFO_STAT_STAT_CNT_GET (pdu_io_read32 (spacc->regmap + SPACC_REG_FIFO_STAT));
   }
   return fifo;
}

static int _spacc_fifo_full(spacc_device *spacc, uint32_t prio)
{
   if (spacc->config.is_qos) {
      return pdu_io_read32(spacc->regmap + SPACC_REG_FIFO_STAT) & SPACC_FIFO_STAT_CMDX_FULL(prio);
   } else {
      return pdu_io_read32(spacc->regmap + SPACC_REG_FIFO_STAT) & SPACC_FIFO_STAT_CMD0_FULL;
   }
}


int spacc_request_hsm_semaphore(spacc_device *spacc)
{
   unsigned long timeout, v;
   timeout = 100000UL;
   pdu_io_write32(spacc->regmap + SPACC_REG_HSM_CMD_REQ, 1); // request SP
   while (--timeout && !((v = pdu_io_read32(spacc->regmap + SPACC_REG_HSM_CMD_GNT)) & 1));
   if (!timeout) {
      ELPHW_PRINT("spacc_packet_enqueue_ddt::Timeout on requesting CTRL register from HSM shared\n");
      return CRYPTO_FAILED;
   }
   return CRYPTO_OK;
}

void spacc_free_hsm_semaphore(spacc_device *spacc)
{
   pdu_io_write32(spacc->regmap + SPACC_REG_HSM_CMD_REQ, 0);
}

// When proc_sz != 0 it overrides the ddt_len value defined in the context referenced by 'job_idx'
int spacc_packet_enqueue_ddt_ex (spacc_device * spacc, int use_jb, int job_idx, pdu_ddt * src_ddt, pdu_ddt * dst_ddt, uint32_t proc_sz, uint32_t aad_offset, uint32_t pre_aad_sz,
                              uint32_t post_aad_sz, uint32_t iv_offset, uint32_t prio)
{
   int ret = CRYPTO_OK, proc_len;
   spacc_job *job;
   spacc_ctx *ctx;

   if (job_idx < 0 || job_idx > SPACC_MAX_JOBS) {
      return CRYPTO_INVALID_HANDLE;
   }

   switch (prio)  {
      case SPACC_SW_CTRL_PRIO_MED: if (spacc->config.cmd1_fifo_depth == 0) { return CRYPTO_CMD_FIFO_INACTIVE; }; break;
      case SPACC_SW_CTRL_PRIO_LOW: if (spacc->config.cmd2_fifo_depth == 0) { return CRYPTO_CMD_FIFO_INACTIVE; }; break;
   }

   job = &spacc->job[job_idx];
   ctx = context_lookup_by_job(spacc, job_idx);
   if (NULL == job) {
      ret = CRYPTO_FAILED;
   } else {

      // process any jobs in the jb
      if (use_jb && spacc_process_jb(spacc) != 0) {
         goto fifo_full;
      }

      // If HSM shared mode
      if (spacc->config.is_hsm_shared) {
         ret = spacc_request_hsm_semaphore(spacc);
         if (ret != CRYPTO_OK) {
             return ret;
         }
      }

      if (_spacc_fifo_full(spacc, prio)) {
         if (spacc->config.is_hsm_shared) {
            spacc_free_hsm_semaphore(spacc);
         }
         if (use_jb) {
            goto fifo_full;
         } else {
            return CRYPTO_FIFO_FULL;
         }

      }


      /* compute the length we must process, in decrypt mode with an ICV (hash, hmac or CCM modes)
       * we must subtract the icv length from the buffer size
       */
      if (proc_sz == SPACC_AUTO_SIZE) {
         if ((job->op == OP_DECRYPT) &&
             ((job->hash_mode > 0) || (job->enc_mode == CRYPTO_MODE_AES_CCM || job->enc_mode == CRYPTO_MODE_AES_GCM)) &&
             !(job->ctrl & SPACC_CTRL_MASK(SPACC_CTRL_ICV_ENC))) {
            proc_len = src_ddt->len - job->icv_len;
         } else {
            proc_len = src_ddt->len;
         }
      } else {
         proc_len = proc_sz;
      }

      if (pre_aad_sz & SPACC_AADCOPY_FLAG) {
         job->ctrl |= SPACC_CTRL_MASK(SPACC_CTRL_AAD_COPY);
         pre_aad_sz &= ~(SPACC_AADCOPY_FLAG);
      } else {
         job->ctrl &= ~SPACC_CTRL_MASK(SPACC_CTRL_AAD_COPY);
      }

      job->pre_aad_sz  = pre_aad_sz;
      job->post_aad_sz = post_aad_sz;

      if (spacc->config.dma_type == SPACC_DMA_DDT) {
         pdu_io_cached_write32 (spacc->regmap + SPACC_REG_SRC_PTR, (uint32_t) src_ddt->phys, &spacc->cache.src_ptr);
         pdu_io_cached_write32 (spacc->regmap + SPACC_REG_DST_PTR, (uint32_t) dst_ddt->phys, &spacc->cache.dst_ptr);
      } else if (spacc->config.dma_type == SPACC_DMA_LINEAR) {
         pdu_io_cached_write32 (spacc->regmap + SPACC_REG_SRC_PTR, (uint32_t) src_ddt->virt[0], &spacc->cache.src_ptr);
         pdu_io_cached_write32 (spacc->regmap + SPACC_REG_DST_PTR, (uint32_t) dst_ddt->virt[0], &spacc->cache.dst_ptr);
      } else {
         ELPHW_PRINT("Invalid spacc->dma_type variable: %d\n", (int)spacc->config.dma_type);
         return CRYPTO_FAILED;
      }



      if (spacc->config.is_hsm_shared) {
         pdu_io_write32 (spacc->regmap + SPACC_REG_PROC_LEN,     proc_len - job->post_aad_sz);
         pdu_io_write32 (spacc->regmap + SPACC_REG_ICV_LEN,      job->icv_len);
         pdu_io_write32 (spacc->regmap + SPACC_REG_ICV_OFFSET,   job->icv_offset);
         pdu_io_write32 (spacc->regmap + SPACC_REG_PRE_AAD_LEN,  job->pre_aad_sz);
         pdu_io_write32 (spacc->regmap + SPACC_REG_POST_AAD_LEN, job->post_aad_sz);
         pdu_io_write32 (spacc->regmap + SPACC_REG_IV_OFFSET,    iv_offset);
         pdu_io_write32 (spacc->regmap + SPACC_REG_OFFSET,       aad_offset);
         pdu_io_write32 (spacc->regmap + SPACC_REG_AUX_INFO,
                         AUX_INFO_SET_DIR(job->auxinfo_dir)
                         | AUX_INFO_SET_BIT_ALIGN(job->auxinfo_bit_align)
                         | AUX_INFO_SET_CBC_CS(job->auxinfo_cs_mode));
      } else {
         pdu_io_cached_write32 (spacc->regmap + SPACC_REG_PROC_LEN,     proc_len - job->post_aad_sz, &spacc->cache.proc_len);
         pdu_io_cached_write32 (spacc->regmap + SPACC_REG_ICV_LEN,      job->icv_len               , &spacc->cache.icv_len);
         pdu_io_cached_write32 (spacc->regmap + SPACC_REG_ICV_OFFSET,   job->icv_offset            , &spacc->cache.icv_offset);
         pdu_io_cached_write32 (spacc->regmap + SPACC_REG_PRE_AAD_LEN,  job->pre_aad_sz            , &spacc->cache.pre_aad);
         pdu_io_cached_write32 (spacc->regmap + SPACC_REG_POST_AAD_LEN, job->post_aad_sz           , &spacc->cache.post_aad);
         pdu_io_cached_write32 (spacc->regmap + SPACC_REG_IV_OFFSET,    iv_offset                  , &spacc->cache.iv_offset);
         pdu_io_cached_write32 (spacc->regmap + SPACC_REG_OFFSET,       aad_offset                 , &spacc->cache.offset);
         pdu_io_cached_write32 (spacc->regmap + SPACC_REG_AUX_INFO,
                         AUX_INFO_SET_DIR(job->auxinfo_dir)
                         | AUX_INFO_SET_BIT_ALIGN(job->auxinfo_bit_align)
                         | AUX_INFO_SET_CBC_CS(job->auxinfo_cs_mode), &spacc->cache.aux);
      }

      if (job->first_use == 1) {
         pdu_io_write32 (spacc->regmap + SPACC_REG_KEY_SZ, job->ckey_sz | SPACC_SET_KEY_CTX(job->ctx_idx));
         pdu_io_write32 (spacc->regmap + SPACC_REG_KEY_SZ, job->hkey_sz | SPACC_SET_KEY_CTX(job->ctx_idx));
      }

      /* write the job ID to the core, we keep track of it in software now to avoid excessive port I/O */
      pdu_io_write32 (spacc->regmap + SPACC_REG_SW_CTRL, SPACC_SW_CTRL_ID_SET(spacc->job_tally) | SPACC_SW_CTRL_PRIO_SET(prio));
      job->job_swid = (spacc->job_tally)++ & 0xFF;
      spacc->job_tally &= 0xFF;
      spacc->job_lookup[job->job_swid] = job_idx;

      pr_debug("Crypto Engine is working ...");


#ifdef MAKEAVECTOR
{ int x;
  printk("VEC: Initial cipher context\n");
  for (x = 0; x < spacc->config.ciph_page_size; x += 4) {
     printk("VEC: %08lx\n", htonl(pdu_io_read32(spacc->regmap + SPACC_CTX_CIPH_KEY + (spacc->config.ciph_page_size>>2) * job->ctx_idx + x)));
  }
  printk("VEC: END\n");
  printk("VEC: Initial hash context\n");
  for (x = 0; x < spacc->config.hash_page_size; x += 4) {
     printk("VEC: %08lx\n", htonl(pdu_io_read32(spacc->regmap + SPACC_CTX_HASH_KEY + (spacc->config.hash_page_size>>2) * job->ctx_idx + x)));
  }
  printk("VEC: END\n");
}
#endif

      pdu_io_write32 (spacc->regmap + SPACC_REG_CTRL, job->ctrl);

#ifdef MAKEAVECTOR
   printk("VEC: START\n");
   printk("VEC: 00000012    //Opcode\n");
   printk("VEC: %08d     //vSPAcc Indentifier\n", spacc->config.idx);
   printk("VEC: %08lx    //CTRL register\n", (pdu_io_read32(spacc->regmap + SPACC_REG_CTRL)));
   printk("VEC: %08zx    //source packet length\n", (proc_sz + pre_aad_sz + post_aad_sz));
   printk("VEC: %08lx    //Destination packet length\n", 0L);
   printk("VEC: %08lx    //ICV length\n", (job->icv_len));
   printk("VEC: %08lx    //ICV offset\n", (job->icv_offset));
   printk("VEC: %08lx    //dst/src offset\n", 0L);
   printk("VEC: %08zx    //pre_aad\n", (pre_aad_sz));
   printk("VEC: %08lx    //PROC len\n", (pdu_io_read32(spacc->regmap + SPACC_REG_PROC_LEN)));
   printk("VEC: %08zx    //post_aad\n", (post_aad_sz));
   printk("VEC: %08zx    //IV_OFFSET\n", (iv_offset));
   printk("VEC: %08lx    //AUX register\n", htonl(pdu_io_read32(spacc->regmap + SPACC_REG_AUX_INFO)));
   printk("VEC: %08lx    //secure processing cmd\n", 0L);

   printk("VEC: %08lx    //cipher key sz\n", htonl(job->ckey_sz | SPACC_SET_KEY_CTX(job->ctx_idx)));
   printk("VEC: %08lx    //hash key sz\n",  htonl(job->hkey_sz | SPACC_SET_KEY_CTX(job->ctx_idx)));
   printk("VEC: END\n");
#endif

// If HSM shared mode
// clear the RQST bit
      if (spacc->config.is_hsm_shared) {
         spacc_free_hsm_semaphore(spacc);
      }

      // Clear an expansion key after the first call
      if (job->first_use == 1) {
         job->first_use = 0;
         job->ctrl &= ~SPACC_CTRL_MASK(SPACC_CTRL_KEY_EXP);
      }

   }

   return ret;
fifo_full:
   // try to add a job to the job buffers
{ int i;
   i = spacc->jb_head + 1;
   if (i == SPACC_MAX_JOB_BUFFERS) { i = 0; }
   if (i == spacc->jb_tail)        { return CRYPTO_FIFO_FULL; }
   spacc->job_buffer[spacc->jb_head] = (struct spacc_job_buffer) {
            .active      = 1,
            .job_idx     = job_idx,
            .src         = src_ddt,
            .dst         = dst_ddt,
            .proc_sz     = proc_sz,
            .aad_offset  = aad_offset,
            .pre_aad_sz  = pre_aad_sz,
            .post_aad_sz = post_aad_sz,
            .iv_offset   = iv_offset,
            .prio        = prio
         };
   spacc->jb_head = i;
   return CRYPTO_USED_JB;
}

}

int spacc_packet_enqueue_ddt (spacc_device * spacc, int job_idx, pdu_ddt * src_ddt, pdu_ddt * dst_ddt,
                              uint32_t proc_sz, uint32_t aad_offset, uint32_t pre_aad_sz, uint32_t post_aad_sz, uint32_t iv_offset, uint32_t prio)
{
   unsigned long lock_flags;
   int ret;

   PDU_LOCK(&spacc->lock, lock_flags);
   ret = spacc_packet_enqueue_ddt_ex(spacc, 1, job_idx, src_ddt, dst_ddt, proc_sz, aad_offset, pre_aad_sz, post_aad_sz, iv_offset, prio);
   PDU_UNLOCK(&spacc->lock, lock_flags);
   return ret;
}

int spacc_pop_packets_ex (spacc_device * spacc, int *num_popped, unsigned long *lock_flag)
{
   int ret = CRYPTO_INPROGRESS;
  // spacc_ctx *ctx = NULL;
   spacc_job *job = NULL;
   uint32_t cmdstat, swid;
   int jobs;

   *num_popped = 0;

   while ((jobs = _spacc_get_stat_cnt(spacc))) {
      while (jobs-- > 0) {
         /* write the pop register to get the next job */
         pdu_io_write32 (spacc->regmap + SPACC_REG_STAT_POP, 1);
         cmdstat = pdu_io_read32 (spacc->regmap + SPACC_REG_STATUS);

         swid = SPACC_STATUS_SW_ID_GET(cmdstat);

         if (spacc->job_lookup[swid] == SPACC_JOB_IDX_UNUSED) {
            ELPHW_PRINT ("Invalid sw id (%d) popped off the stack", swid);
            ret = CRYPTO_FAILED;
            goto ERR;
         }

         /* find the associated job with popped swid */
         job = job_lookup_by_swid (spacc, swid);
         if (NULL == job) {
            ret = CRYPTO_FAILED;
            ELPHW_PRINT ("Failed to find job for ID %d\n", swid);
            goto ERR;
         }

         /* mark job as done */
         job->job_done = 1;
         spacc->job_lookup[swid] = SPACC_JOB_IDX_UNUSED;
         switch (SPACC_GET_STATUS_RET_CODE (cmdstat)) {
            case SPACC_ICVFAIL:
               ret = CRYPTO_AUTHENTICATION_FAILED;
               break;
            case SPACC_MEMERR:
               ret = CRYPTO_MEMORY_ERROR;
               break;
            case SPACC_BLOCKERR:
               ret = CRYPTO_INVALID_BLOCK_ALIGNMENT;
               break;
            case SPACC_SECERR:
               ret = CRYPTO_FAILED;
               break;

            case SPACC_OK:
   #ifdef SECURE_MODE
               if (job->job_secure && !(cmdstat & (1 << _SPACC_STATUS_SEC_CMD))) {
                  ret = CRYPTO_INPROGRESS;
                  break;
               }
   #endif
               ret = CRYPTO_OK;
               break;
         }

         job->job_err = ret;

         /*
          * We're done touching the SPAcc hw, so release the lock across the
          * job callback.  It must be reacquired before continuing to the next
          * iteration.
          */

         if (job->cb) {
            PDU_UNLOCK(&spacc->lock, *lock_flag);
            job->cb(spacc, job->cbdata);
            PDU_LOCK(&spacc->lock, *lock_flag);
         }

         (*num_popped)++;

      }
   }
   //if (!*num_popped) { ELPHW_PRINT("ERROR: Failed to pop a single job\n"); }
ERR:
   spacc_process_jb(spacc);

   if (spacc->op_mode == SPACC_OP_MODE_WD) {
      spacc_set_wd_count(spacc, spacc->config.wd_timer); // reset the WD timer to the original value
   }

   if (*num_popped && spacc->spacc_notify_jobs != NULL) {
      spacc->spacc_notify_jobs(spacc);
   }

   return ret;
}

int spacc_pop_packets (spacc_device * spacc, int *num_popped)
{
   unsigned long lock_flag;
   int err;
   PDU_LOCK(&spacc->lock, lock_flag);
   err = spacc_pop_packets_ex(spacc, num_popped, &lock_flag);
   PDU_UNLOCK(&spacc->lock, lock_flag);
   return err;
}


/* test if done */
int spacc_packet_dequeue (spacc_device * spacc, int job_idx)
{
   int ret = CRYPTO_OK;
   spacc_job *job = &spacc->job[job_idx];
   unsigned long lock_flag;

   PDU_LOCK(&spacc->lock, lock_flag);

   if (job == NULL && !(job_idx == SPACC_JOB_IDX_UNUSED)) {
      ret = CRYPTO_FAILED;
   } else {
      if (job->job_done) {
         job->job_done  = 0;
         ret = job->job_err;
      } else {
         ret = CRYPTO_INPROGRESS;
      }
   }

   PDU_UNLOCK(&spacc->lock, lock_flag);
   return ret;
}


int spacc_isenabled(spacc_device *spacc, int mode, int keysize)
{
   int x;
   static const int keysizes[2][7] = {
      { 5,  8, 16, 24, 32,  0,   0 },     // cipher key sizes
      { 8, 16, 20, 24, 32, 64, 128 },  // hash key sizes
   };

   if (mode < 0 || mode > CRYPTO_MODE_LAST) {
      return 0;
   }


   // always return true for NULL
   if (mode == CRYPTO_MODE_NULL) {
      return 1;
   }
 
   if (spacc->config.modes[mode] & 128) {
      for (x = 0; x < 6; x++) {
         if (keysizes[1][x] >= keysize && ((1<<x)&spacc->config.modes[mode])) {
            return 1;
         }
      }
      return 0;
   } else {
      for (x = 0; x < 6; x++) {
         if (keysizes[0][x] == keysize) {
            if (spacc->config.modes[mode] & (1<<x)) {
               return 1;
            } else {
               return 0;
            }
         }
      }
   }

   return 0;
}
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

#ifdef SECURE_MODE
void spacc_set_secure_mode (spacc_device * spacc, int src, int dst, int ddt, int global_lock)
{
   pdu_io_write32 (spacc->regmap + SPACC_REG_SECURE_CTRL,
                   (src ? SPACC_SECURE_CTRL_MS_SRC : 0) |
                   (dst ? SPACC_SECURE_CTRL_MS_DST : 0) | (ddt ? SPACC_SECURE_CTRL_MS_DDT : 0) | (global_lock ? SPACC_SECURE_CTRL_LOCK : 0));
}
#endif

int spacc_set_key_exp(spacc_device *spacc, int job_idx)
{
   spacc_ctx *ctx = NULL;
   spacc_job *job = NULL;

   if (job_idx < 0 || job_idx > SPACC_MAX_JOBS) {
      return CRYPTO_INVALID_HANDLE;
   }

   job = &spacc->job[job_idx];
   ctx = context_lookup_by_job(spacc, job_idx);

   if (!ctx) {
      return CRYPTO_FAILED;
   }

   job->ctrl |= SPACC_CTRL_MASK(SPACC_CTRL_KEY_EXP);
   return CRYPTO_OK;
}
int spacc_virtual_request_rc4 (spacc_device * spacc)
{
   volatile int timeout = 1000000;
   pdu_io_write32 (spacc->regmap + SPACC_REG_VIRTUAL_RC4_KEY_RQST, 1);
   while (pdu_io_read32 (spacc->regmap + SPACC_REG_VIRTUAL_RC4_KEY_GNT) != 1) {
      if (--timeout == 0) {
         ELPHW_PRINT ("Failed to request RC4 key context, timeout\n");
         return CRYPTO_FAILED;
      }
   }
   return CRYPTO_OK;
}

void spacc_virtual_set_weight (spacc_device * spacc, int weight)
{
   if (weight) {
      pdu_io_write32 (spacc->regmap + SPACC_REG_VIRTUAL_PRIO, VPRIO_SET (1, weight));
   } else {
      pdu_io_write32 (spacc->regmap + SPACC_REG_VIRTUAL_PRIO, 0);
   }
}

void spacc_set_wd_count(spacc_device *spacc, uint32_t val)
{
   pdu_io_write32(spacc->regmap + SPACC_REG_STAT_WD_CTRL, val);
}

void spacc_disable_int (spacc_device * spacc)
{
   pdu_io_write32 (spacc->regmap + SPACC_REG_IRQ_EN, 0);
}

/* Read the IRQ status register and process as needed */

uint32_t spacc_process_irq(spacc_device *spacc)
{
   uint32_t temp;
   int x, cmd_max;
   unsigned long lock_flag;

   PDU_LOCK(&spacc->lock, lock_flag);

   temp = pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_STAT);

   /* clear interrupt pin and run registered callback */
   if (temp & SPACC_IRQ_STAT_STAT) {
      SPACC_IRQ_STAT_CLEAR_STAT(spacc);
      if (spacc->op_mode == SPACC_OP_MODE_IRQ) {
         spacc->config.fifo_cnt <<= 2;
         if (spacc->config.fifo_cnt >= spacc->config.stat_fifo_depth) {
            spacc->config.fifo_cnt = spacc->config.stat_fifo_depth;
         } 
         spacc_irq_stat_enable(spacc, spacc->config.fifo_cnt); // update fifo count to allow more stati to pile up
         spacc_irq_cmdx_enable(spacc, 0, 0);            // reenable CMD0 empty interrupt
      } else if (spacc->op_mode == SPACC_OP_MODE_WD) {
      }
      if (spacc->irq_cb_stat != NULL){
         spacc->irq_cb_stat(spacc);
      }
   }

   /* Watchdog IRQ */
   if (spacc->op_mode == SPACC_OP_MODE_WD) {
      if (temp & SPACC_IRQ_STAT_STAT_WD) {
         if (++(spacc->wdcnt) == SPACC_WD_LIMIT) {
            ELPHW_PRINT("spacc_process_irq::Hit SPACC WD LIMIT aborting WD IRQs (%08lx) (this happens when you get too many IRQs that go unanswered)\n",
                        (unsigned long)temp);
            ELPHW_PRINT("spacc_process_irq::Current IRQ_EN settings 0x%08lx\n",
                        (unsigned long)pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN));
            spacc_irq_stat_wd_disable(spacc);
            spacc_irq_stat_enable(spacc, 1); // we set the STAT CNT to 1 so that every job generates an IRQ now
            spacc->op_mode = SPACC_OP_MODE_IRQ;
            ELPHW_PRINT("spacc_process_irq::New IRQ_EN settings 0x%08lx\n",
                        (unsigned long)pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN));
         } else {
            // if the timer isn't too high lets bump it up a bit so as to give the IRQ a chance to reply
            if (spacc->config.wd_timer < (0xFFFFFFUL >> 4)) {
               spacc_set_wd_count(spacc, spacc->config.wd_timer << 4);
            }
         }

         SPACC_IRQ_STAT_CLEAR_STAT_WD(spacc);
         if (spacc->irq_cb_stat_wd != NULL) {
            spacc->irq_cb_stat_wd(spacc);
         }
      }
   }


   if (temp & SPACC_IRQ_STAT_RC4_DMA) {
      SPACC_IRQ_STAT_CLEAR_RC4_DMA(spacc);
      if (spacc->irq_cb_rc4_dma != NULL){
         spacc->irq_cb_rc4_dma(spacc);
      }
   }


   if (spacc->op_mode == SPACC_OP_MODE_IRQ && !spacc->config.is_hsm_shared) {
      cmd_max = (spacc->config.is_qos ? SPACC_CMDX_MAX_QOS : SPACC_CMDX_MAX);
      for (x = 0; x < cmd_max; x++){
         if (temp & SPACC_IRQ_STAT_CMDX(x)) {
             spacc->config.fifo_cnt = 1;
             spacc_irq_cmdx_disable(spacc, x); // disable CMD0 interrupt since STAT=1
             spacc_irq_stat_enable (spacc, spacc->config.fifo_cnt); // reset STAT count to 1

            SPACC_IRQ_STAT_CLEAR_CMDX(spacc, x);
            /* run registered callback */
            if (spacc->irq_cb_cmdx != NULL){
               spacc->irq_cb_cmdx(spacc, x);
            }
         }
      }
   }


   PDU_UNLOCK(&spacc->lock, lock_flag);

   return temp;
}


static int spacc_xof_stringsize_autodetect(spacc_device *spacc)
{
   pdu_ddt     ddt;
   dma_addr_t  dma;
   void        *virt;
   int         ss, alg, i;
   unsigned long spacc_ctrl[2] = {0xF400B400, 0xF400D400};
   unsigned char buf[256];
   unsigned long buflen;
   unsigned char test_str[6] = {0x01, 0x20, 0x54, 0x45, 0x53, 0x54};
   unsigned char md[2][16] = {{0xc3, 0x6d, 0x0a, 0x88, 0xfa, 0x37, 0x4c, 0x9b, 0x44, 0x74, 0xeb, 0x00, 0x5f, 0xe8, 0xca, 0x25},
                              {0x68, 0x77, 0x04, 0x11, 0xf8, 0xe3, 0xb0, 0x1e, 0x0d, 0xbf, 0x71, 0x6a, 0xe9, 0x87, 0x1a, 0x0d}};

   // get memory
   virt = pdu_dma_alloc(256, &dma);
   if (!virt) {
      return CRYPTO_FAILED;
   }
   if (pdu_ddt_init(&ddt, 1)) {
      pdu_dma_free(256, virt, dma);
      return CRYPTO_FAILED;
   }
   pdu_ddt_add(&ddt, dma, 256);

   // populate registers for jobs
   pdu_io_write32(spacc->regmap + SPACC_REG_SRC_PTR, (uint32_t)ddt.phys);
   pdu_io_write32(spacc->regmap + SPACC_REG_DST_PTR, (uint32_t)ddt.phys);
   pdu_io_write32(spacc->regmap + SPACC_REG_PROC_LEN,     16);
   pdu_io_write32(spacc->regmap + SPACC_REG_PRE_AAD_LEN,  16);
   pdu_io_write32(spacc->regmap + SPACC_REG_ICV_LEN,      16);
   pdu_io_write32(spacc->regmap + SPACC_REG_KEY_SZ,       6);
   pdu_io_write32(spacc->regmap + SPACC_REG_SW_CTRL,      0);

   // repeat for 2 algorithms, CSHAKE128 and KMAC128
   for (alg = 0; (alg < 2) && (spacc->config.string_size == 0); alg++) {
      // repeat for 4 string_size sizes
      for (ss = 0; ss < 4; ss++) {
         buflen = (32UL << ss);
         if (buflen > spacc->config.hash_page_size) break;

         // clear I/O memory
         memset(virt, 0, 256);

         // clear buf and then insert test string
         memset(buf, 0, sizeof(buf));
         memcpy(buf, test_str, sizeof(test_str));
         memcpy(buf + (buflen >> 1), test_str, sizeof(test_str));

         //write key context
         pdu_to_dev32_s(spacc->regmap + SPACC_CTX_HASH_KEY, buf, spacc->config.hash_page_size >> 2, spacc_endian);

         //write ctrl
         pdu_io_write32(spacc->regmap + SPACC_REG_CTRL, spacc_ctrl[alg]);

         //wait for job complete
         for (i = 0; i < 20; i++) {
            if (!(pdu_io_read32(spacc->regmap + SPACC_REG_FIFO_STAT) & SPACC_FIFO_STAT_STAT_EMPTY)) {
               //check result, if it matches, we have string_size
               pdu_io_write32 (spacc->regmap + SPACC_REG_STAT_POP, 1);
               if ((SPACC_GET_STATUS_RET_CODE(pdu_io_read32 (spacc->regmap + SPACC_REG_STATUS)) == SPACC_OK)
                   && (!memcmp(virt, md[alg], 16))) {
                  spacc->config.string_size = (16 << ss);
               }
               break;
            }
         }
      }
   }

   // reset registers
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_CTRL,     0);
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_EN,       0);
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_STAT,     0xFFFFFFFF);

   pdu_io_write32(spacc->regmap + SPACC_REG_SRC_PTR,      0);
   pdu_io_write32(spacc->regmap + SPACC_REG_DST_PTR,      0);
   pdu_io_write32(spacc->regmap + SPACC_REG_PROC_LEN,     0);
   pdu_io_write32(spacc->regmap + SPACC_REG_ICV_LEN,      0);
   pdu_io_write32(spacc->regmap + SPACC_REG_PRE_AAD_LEN,  0);

   pdu_ddt_free(&ddt);
   pdu_dma_free(256, virt, dma);

   return CRYPTO_OK;
}

static const uint8_t spacc_ctrl_map[SPACC_CTRL_VER_SIZE][SPACC_CTRL_MAPSIZE] =
                       {{ 0, 8, 4, 12, 24, 16, 31, 25, 26, 27, 28, 29, 14, 15 },
                        { 0, 8, 3, 12, 24, 16, 31, 25, 26, 27, 28, 29, 14, 15 },
                        { 0, 4, 8, 13, 15, 16, 24, 25, 26, 27, 28, 29, 30, 31 }};

int spacc_init (void *baseaddr, spacc_device * spacc, pdu_info * info)
{
   unsigned long id;

   if (baseaddr == NULL) {
      ELPHW_PRINT("spacc_init:: baseaddr is NULL\n");
      return -1;
   }
   if (spacc == NULL) {
      ELPHW_PRINT("spacc_init:: spacc is NULL\n");
      return -1;
   }

   memset (spacc, 0, sizeof *spacc);
   PDU_INIT_LOCK(&spacc->lock);
   PDU_INIT_LOCK(&spacc->ctx_lock);

   // assign the baseaddr
   spacc->regmap = baseaddr;

   // version info
   spacc->config.version     = info->spacc_version.version;
   spacc->config.pdu_version = (info->pdu_config.major << 4) | info->pdu_config.minor;
   spacc->config.project     = info->spacc_version.project;
   spacc->config.is_pdu      = info->spacc_version.is_pdu;
   spacc->config.is_qos      = info->spacc_version.qos;

   // hsm specific
   spacc->config.is_hsm_virtual    = info->spacc_version.is_hsm & info->hsm_config.paradigm;
   spacc->config.is_hsm_shared     = info->spacc_version.is_hsm & !(info->hsm_config.paradigm);
   spacc->config.num_sec_ctx       = info->hsm_config.num_ctx;
   spacc->config.sec_ctx_page_size = 4*(1U<<(info->hsm_config.ctx_page_size+2));

   // misc
   spacc->config.is_partial        = info->spacc_version.partial;
   spacc->config.num_ctx           = info->spacc_config.num_ctx;
   spacc->config.num_rc4_ctx       = info->spacc_config.num_rc4_ctx;
   spacc->config.ciph_page_size    = 1U << info->spacc_config.ciph_ctx_page_size;
   spacc->config.hash_page_size    = 1U << info->spacc_config.hash_ctx_page_size;
   spacc->config.dma_type          = info->spacc_config.dma_type;
   spacc->config.idx               = info->spacc_version.vspacc_idx;
   spacc->config.cmd0_fifo_depth   = info->spacc_config.cmd0_fifo_depth;
   spacc->config.cmd1_fifo_depth   = info->spacc_config.cmd1_fifo_depth;
   spacc->config.cmd2_fifo_depth   = info->spacc_config.cmd2_fifo_depth;
   spacc->config.stat_fifo_depth   = info->spacc_config.stat_fifo_depth;
   spacc->config.fifo_cnt          = 1;

   spacc->config.is_ivimport = info->spacc_version.ivimport;

   // ctrl register map
   if (spacc->config.version <= 0x4E) {
      spacc->config.ctrl_map = spacc_ctrl_map[SPACC_CTRL_VER_0];
   } else if (spacc->config.version <= 0x60){
      spacc->config.ctrl_map = spacc_ctrl_map[SPACC_CTRL_VER_1];
   } else {
      spacc->config.ctrl_map = spacc_ctrl_map[SPACC_CTRL_VER_2];
   }

   spacc->job_tally                = 0;
   spacc->wdcnt                    = 0;
   spacc->config.wd_timer          = SPACC_WD_TIMER_INIT;

   // version 4.10 uses IRQ, above uses WD and we don't support below 4.00
   if (spacc->config.version < 0x40) {
      ELPHW_PRINT("spacc_init::Unsupported SPAcc version\n");
      return CRYPTO_FAILED;
   } else if (spacc->config.version < 0x4B) {
      spacc->op_mode                  = SPACC_OP_MODE_IRQ;
   } else {
      spacc->op_mode                  = SPACC_OP_MODE_WD;
   }

   {
     int x;
     spacc->config.ctx_mask = 0;
     for (x = 1; x < spacc->config.num_ctx; x <<= 1) {
       spacc->config.ctx_mask |= x;
     }
   }

   if (spacc->config.is_hsm_virtual || spacc->config.is_hsm_shared) {
      spacc->config.is_secure      = 1;
      spacc->config.is_secure_port = spacc->config.idx;
   } else {
      spacc->config.is_secure      = 0;
      spacc->config.is_secure_port = 0;
   }

/* set threshold and enable irq */
   // on 4.11 and newer cores we can derive this from the HW reported depths.
   if (spacc->config.stat_fifo_depth == 1) {
      spacc->config.ideal_stat_level = 1;
   } else if (spacc->config.stat_fifo_depth <= 4) {
      spacc->config.ideal_stat_level = spacc->config.stat_fifo_depth - 1;
   } else if (spacc->config.stat_fifo_depth <= 8) {
      spacc->config.ideal_stat_level = spacc->config.stat_fifo_depth - 2;
   } else {
      spacc->config.ideal_stat_level = spacc->config.stat_fifo_depth - 4;
   }

/* determine max PROClen value */
   pdu_io_write32(spacc->regmap + SPACC_REG_PROC_LEN, 0xFFFFFFFF);
   spacc->config.max_msg_size = pdu_io_read32(spacc->regmap + SPACC_REG_PROC_LEN);

   // read config info
   if (spacc->config.is_pdu) {
      ELPHW_PRINT("PDU:\n");
      ELPHW_PRINT("   MAJOR      : %u\n", info->pdu_config.major);
      ELPHW_PRINT("   MINOR      : %u\n", info->pdu_config.minor);
   }
   id = pdu_io_read32 (spacc->regmap + SPACC_REG_ID);
   ELPHW_PRINT ("SPACC ID: (%08lx)\n   MAJOR      : %x\n", (unsigned long) id, info->spacc_version.major);
   ELPHW_PRINT ("   MINOR      : %x\n", info->spacc_version.minor);
   ELPHW_PRINT ("   QOS        : %x\n", info->spacc_version.qos);
   ELPHW_PRINT ("   IVIMPORT   : %x\n", spacc->config.is_ivimport);
   if (spacc->config.version >= 0x48) {
      ELPHW_PRINT ("   TYPE       : %lx (%s)\n", SPACC_ID_TYPE (id), (char *[]){"SPACC", "SPACC-PDU", "SPACC-HSM", "TROOT"}[SPACC_ID_TYPE (id)&3]);
   }
   ELPHW_PRINT ("   AUX        : %x\n", info->spacc_version.qos);
   ELPHW_PRINT ("   IDX        : %lx %s\n", SPACC_ID_VIDX (id), spacc->config.is_secure ? ((char *[]){"(Normal Port)", "(Secure Port)"}[spacc->config.is_secure_port&1]) : "");
   ELPHW_PRINT ("   PARTIAL    : %x\n", info->spacc_version.partial);
   ELPHW_PRINT ("   PROJECT    : %x\n", info->spacc_version.project);
   if (spacc->config.version >= 0x48) {
      id = pdu_io_read32 (spacc->regmap + SPACC_REG_CONFIG);
   } else {
      id = 0xFFFFFFFF;
   }
   ELPHW_PRINT ("SPACC CFG: (%08lx)\n", id);
   ELPHW_PRINT ("   CTX CNT    : %u\n", info->spacc_config.num_ctx);
   ELPHW_PRINT ("   RC4 CNT    : %u\n", info->spacc_config.num_rc4_ctx);
   ELPHW_PRINT ("   VSPACC CNT : %u\n", info->spacc_config.num_vspacc);
   ELPHW_PRINT ("   CIPH SZ    : %-3lu bytes\n", 1UL<<info->spacc_config.ciph_ctx_page_size);
   ELPHW_PRINT ("   HASH SZ    : %-3lu bytes\n", 1UL<<info->spacc_config.hash_ctx_page_size);
   ELPHW_PRINT ("   DMA TYPE   : %u (%s)\n", info->spacc_config.dma_type, (char *[]){"Unknown", "Scattergather", "Linear", "Unknown"}[info->spacc_config.dma_type&3]);
   ELPHW_PRINT ("   MAX PROCLEN: %lu bytes\n", (unsigned long)spacc->config.max_msg_size);
   ELPHW_PRINT ("   FIFO CONFIG :\n");
   ELPHW_PRINT ("      CMD0 DEPTH: %d\n", spacc->config.cmd0_fifo_depth);
   if (spacc->config.is_qos) {
      ELPHW_PRINT ("      CMD1 DEPTH: %d\n", spacc->config.cmd1_fifo_depth);
      ELPHW_PRINT ("      CMD2 DEPTH: %d\n", spacc->config.cmd2_fifo_depth);
   }
   ELPHW_PRINT ("      STAT DEPTH: %d\n", spacc->config.stat_fifo_depth);
   if (spacc->config.is_secure) {
      id = pdu_io_read32(spacc->regmap + SPACC_REG_HSM_VERSION);
      ELPHW_PRINT("HSM CFG: (%08lx)\n", id);
      ELPHW_PRINT("   MAJOR     : %x\n", info->hsm_config.major);
      ELPHW_PRINT("   MINOR     : %x\n", info->hsm_config.minor);
      ELPHW_PRINT("   PARADIGM  : %x (%s)\n", info->hsm_config.paradigm, (char *[]){"Shared", "Virtual"}[info->hsm_config.paradigm&1]);
      ELPHW_PRINT("   CTX CNT   : %u (secure key)\n", info->hsm_config.num_ctx);
      ELPHW_PRINT("   CTX SZ    : %-3lu bytes\n", 4*(1UL<<(info->hsm_config.ctx_page_size+2)));
   }

   // Quick sanity check for ptr registers (mask unused bits)
   if (spacc->config.is_hsm_shared && spacc->config.is_secure_port == 0) {
      // request the semaphore
      if (spacc_request_hsm_semaphore(spacc) != CRYPTO_OK) {
         goto ERR;
      }
   }

   if (spacc->config.dma_type == SPACC_DMA_DDT) {
      pdu_io_write32 (baseaddr + SPACC_REG_DST_PTR, 0x1234567F);
      pdu_io_write32 (baseaddr + SPACC_REG_SRC_PTR, 0xDEADBEEF);
      if (((pdu_io_read32 (baseaddr + SPACC_REG_DST_PTR)) != (0x1234567F & SPACC_DST_PTR_PTR)) ||
          ((pdu_io_read32 (baseaddr + SPACC_REG_SRC_PTR)) != (0xDEADBEEF & SPACC_SRC_PTR_PTR))) {
         ELPHW_PRINT("spacc_init::Failed to set pointers\n");
         goto ERR;
      }
   }

   // zero the IRQ CTRL/EN register (to make sure we're in a sane state)
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_CTRL,     0);
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_EN,       0);
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_STAT,     0xFFFFFFFF);

   // init cache
   memset(&spacc->cache, 0, sizeof(spacc->cache));
   pdu_io_write32(spacc->regmap + SPACC_REG_SRC_PTR,      0);
   pdu_io_write32(spacc->regmap + SPACC_REG_DST_PTR,      0);
   pdu_io_write32(spacc->regmap + SPACC_REG_PROC_LEN,     0);
   pdu_io_write32(spacc->regmap + SPACC_REG_ICV_LEN,      0);
   pdu_io_write32(spacc->regmap + SPACC_REG_ICV_OFFSET,   0);
   pdu_io_write32(spacc->regmap + SPACC_REG_PRE_AAD_LEN,  0);
   pdu_io_write32(spacc->regmap + SPACC_REG_POST_AAD_LEN, 0);
   pdu_io_write32(spacc->regmap + SPACC_REG_IV_OFFSET,    0);
   pdu_io_write32(spacc->regmap + SPACC_REG_OFFSET,       0);
   pdu_io_write32(spacc->regmap + SPACC_REG_AUX_INFO,     0);

   // free the HSM
   if (spacc->config.is_hsm_shared && spacc->config.is_secure_port == 0) {
      spacc_free_hsm_semaphore(spacc);
   }

   spacc->ctx = pdu_malloc (sizeof (spacc_ctx) * spacc->config.num_ctx);
   if (spacc->ctx == NULL) {
      ELPHW_PRINT ("spacc_init::Out of memory for ctx\n");
      goto ERR;
   }
   spacc->job = pdu_malloc (sizeof (spacc_job) * SPACC_MAX_JOBS);
   if (spacc->job == NULL) {
      ELPHW_PRINT ("spacc_init::Out of memory for job\n");
      goto ERR;
   }

   /* initialize job_idx and lookup table */
   spacc_job_init_all(spacc);

   /* initialize contexts */
   spacc_ctx_init_all (spacc);

   // autodetect and set string size setting
   if (spacc->config.version == 0x61) {
      spacc_xof_stringsize_autodetect(spacc);
   }

   return CRYPTO_OK;
 ERR:
   spacc_fini (spacc);
   return CRYPTO_FAILED;
}

/* free up the memory */
void spacc_fini (spacc_device * spacc)
{
   pdu_free (spacc->ctx);
   pdu_free (spacc->job);
}

struct platform_device * get_spacc_platdev_by_epn(uint32_t epn, uint32_t virt)
{
   char name[256];
   struct device *dev;

   snprintf(name, sizeof(name), "spacc.%d", (epn << 16) | virt);
   dev = bus_find_device_by_name(&platform_bus_type, NULL, name);
   if (!dev) {
      printk(KERN_ERR "failed to find device for %s\n", name);
      return NULL;
   }
   return to_platform_device(dev);
}
EXPORT_SYMBOL(get_spacc_platdev_by_epn);

/* This is used by RE and KEP to get the spacc device */
spacc_device * get_spacc_device_by_epn(uint32_t epn, uint32_t virt)
{
   struct platform_device *plat;
   struct spacc_priv *priv;

   plat = get_spacc_platdev_by_epn(epn, virt);
   if (!plat)
      return NULL;

   priv = platform_get_drvdata(plat);
   return &priv->spacc;
}
EXPORT_SYMBOL(get_spacc_device_by_epn);
int spacc_compute_xcbc_key(spacc_device *spacc, int job_idx, const unsigned char *key, int keylen, unsigned char *xcbc_out)
{
         unsigned char *buf;
         dma_addr_t     bufphys;
         pdu_ddt        ddt;
         int err, i, handle, usecbc, ctx_idx;
         unsigned char iv[16];

         if (job_idx >= 0 && job_idx < SPACC_MAX_JOBS) { ctx_idx = spacc->job[job_idx].ctx_idx; } else { ctx_idx = -1; }

         // figure out if we can schedule the key ...
         if (spacc_isenabled(spacc, CRYPTO_MODE_AES_ECB, 16)) {
            usecbc = 0;
         } else if (spacc_isenabled(spacc, CRYPTO_MODE_AES_CBC, 16)) {
            usecbc = 1;
         } else {
            return -1;
         }

         memset(iv, 0, sizeof iv);
         memset(&ddt, 0, sizeof ddt);

         buf = pdu_dma_alloc(64, &bufphys);
         if (!buf) {
            return -EINVAL;
         }
         handle = -1;

         // set to 1111...., 2222...., 333...
         for (i = 0; i < 48; i++) {
            buf[i] = (i >> 4) + 1;
         }

         // build DDT ...
         err = pdu_ddt_init(&ddt, 1);
         if (err) {
            goto xcbc_err;
         }
         pdu_ddt_add(&ddt, bufphys, 48);

         // open a handle in either CBC or ECB mode
         handle = spacc_open(spacc, usecbc ? CRYPTO_MODE_AES_CBC : CRYPTO_MODE_AES_ECB, CRYPTO_MODE_NULL, ctx_idx, 0, NULL, NULL);
         if (handle < 0) {
            err = handle;
            goto xcbc_err;
         }
         spacc_set_operation(spacc, handle, OP_ENCRYPT, 0, 0, 0, 0, 0);

         if (usecbc) {
            // we can do the ECB work in CBC using three jobs with the IV reset to zero each time
            for (i = 0; i < 3; i++) {
               spacc_write_context(spacc, handle, SPACC_CRYPTO_OPERATION, key, keylen, iv, 16);
               err = spacc_packet_enqueue_ddt(spacc, handle, &ddt, &ddt, 16, (i*16)|((i*16)<<16), 0, 0, 0, 0);
               if (err != CRYPTO_OK) {
                  goto xcbc_err;
               }
               do {
                  err = spacc_packet_dequeue(spacc, handle);
               } while (err == CRYPTO_INPROGRESS);
               if (err != CRYPTO_OK) {
                  goto xcbc_err;
               }
            }
         } else {
            // do the 48 bytes as a single SPAcc job this is the ideal case but only possible
            // if ECB was enabled in the core
            spacc_write_context(spacc, handle, SPACC_CRYPTO_OPERATION, key, keylen, iv, 16);
            err = spacc_packet_enqueue_ddt(spacc, handle, &ddt, &ddt, 48, 0, 0, 0, 0, 0);
            if (err != CRYPTO_OK) {
               goto xcbc_err;
            }
            do {
               err = spacc_packet_dequeue(spacc, handle);
            } while (err == CRYPTO_INPROGRESS);
            if (err != CRYPTO_OK) {
               goto xcbc_err;
            }
         }

         // now we can copy the key
         memcpy(xcbc_out, buf, 48);
         memset(buf, 0, 64);
xcbc_err:
         pdu_dma_free(64, buf, bufphys);
         pdu_ddt_free(&ddt);
         if (handle >= 0) {
            spacc_close(spacc, handle);
         }
         if (err) {
            return -EINVAL;
         }
         return 0;
}

/* cmdx and cmdx_cnt depend on HW config */
/* cmdx can be 0, 1 or 2 */
/* cmdx_cnt must be 2^6 or less */
void spacc_irq_cmdx_enable (spacc_device *spacc, int cmdx, int cmdx_cnt)
{
   uint32_t temp;

   //printk("CMDX enable\n");
   /* read the reg, clear the bit range and set the new value */
   temp = pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_CTRL) & (~SPACC_IRQ_CTRL_CMDX_CNT_MASK(cmdx));
   temp |= SPACC_IRQ_CTRL_CMDX_CNT_SET(cmdx, cmdx_cnt);
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_CTRL, temp | SPACC_IRQ_CTRL_CMDX_CNT_SET(cmdx, cmdx_cnt));

   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_EN,  pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN) | SPACC_IRQ_EN_CMD(cmdx));
}

void spacc_irq_cmdx_disable (spacc_device *spacc, int cmdx)
{
   //printk("CMDX disable\n");
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_EN,  pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN) & (~SPACC_IRQ_EN_CMD(cmdx)));
   //printk("IRQ_EN == 0x%08zx\n", pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN));
}


void spacc_irq_stat_enable (spacc_device *spacc, int stat_cnt)
{
   uint32_t temp;

   temp = pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_CTRL);
   if (spacc->config.is_qos) {
      temp &= (~SPACC_IRQ_CTRL_STAT_CNT_MASK_QOS);
      temp |= SPACC_IRQ_CTRL_STAT_CNT_SET_QOS(stat_cnt);
   } else {
      temp &= (~SPACC_IRQ_CTRL_STAT_CNT_MASK);
      temp |= SPACC_IRQ_CTRL_STAT_CNT_SET(stat_cnt);
   }
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_CTRL, temp);
   //printk("stat_enable\nIRQ_CTRL == 0x%08zx\n", temp);
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_EN, pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN) | SPACC_IRQ_EN_STAT);
   //printk("IRQ_EN   == 0x%08zx\n", pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN));
}

void spacc_irq_stat_disable (spacc_device *spacc)
{
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_EN, pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN) & (~SPACC_IRQ_EN_STAT));
}

void spacc_irq_stat_wd_enable (spacc_device *spacc)
{
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_EN, pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN) | SPACC_IRQ_EN_STAT_WD);
}

void spacc_irq_stat_wd_disable (spacc_device *spacc)
{
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_EN, pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN) & (~SPACC_IRQ_EN_STAT_WD));
}


void spacc_irq_rc4_dma_enable (spacc_device *spacc)
{
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_EN, pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN) | SPACC_IRQ_EN_RC4_DMA);
}

void spacc_irq_rc4_dma_disable (spacc_device *spacc)
{
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_EN, pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN) & (~SPACC_IRQ_EN_RC4_DMA));
}

void spacc_irq_glbl_enable (spacc_device *spacc)
{
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_EN, pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN) | SPACC_IRQ_EN_GLBL);
}

void spacc_irq_glbl_disable (spacc_device *spacc)
{
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_EN, pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN) & (~SPACC_IRQ_EN_GLBL));
}


/* a function to run callbacks in the IRQ handler */
static irqreturn_t spacc_irq_handler(int irq, void *dev)
{
   struct spacc_priv *priv = platform_get_drvdata(to_platform_device(dev));
   spacc_device *spacc = &priv->spacc;

   if (oldtimer != timer) {
      spacc_set_wd_count(&priv->spacc, priv->spacc.config.wd_timer = timer);
      printk("spacc::Changed timer from %lu to %lu\n", oldtimer, timer);
      oldtimer = timer;
   }


/* check irq flags and process as required */
   if (!spacc_process_irq(spacc)) {
      return IRQ_NONE;
   }
   return IRQ_HANDLED;
}

/* callback function to initialize tasklet running */
static void spacc_stat_process(spacc_device *spacc)
{
   struct spacc_priv *priv = container_of(spacc, struct spacc_priv, spacc);

   /* run tasklet to pop jobs off fifo */
   tasklet_schedule(&priv->pop_jobs);
}

static void spacc_cmd_process(spacc_device *spacc, int x)
{
   struct spacc_priv *priv = container_of(spacc, struct spacc_priv, spacc);

   /* run tasklet to pop jobs off fifo */
   tasklet_schedule(&priv->pop_jobs);

}

static void spacc_pop_jobs (unsigned long data)
{
   struct spacc_priv * priv =  (struct spacc_priv *)data;
   spacc_device *spacc = &priv->spacc;
   int num;

   // decrement the WD CNT here since now we're actually going to respond to the IRQ completely
   if (spacc->wdcnt) {
      --(spacc->wdcnt);
   }

   spacc_pop_packets(spacc, &num);
}

#define HW_ENTRY(x) { x, #x }
static const struct { unsigned addr; char *name; } reg_names[] = {
   HW_ENTRY(SPACC_REG_IRQ_EN),
   HW_ENTRY(SPACC_REG_IRQ_STAT),
   HW_ENTRY(SPACC_REG_IRQ_CTRL),
   HW_ENTRY(SPACC_REG_FIFO_STAT),
   HW_ENTRY(SPACC_REG_SDMA_BRST_SZ),
   HW_ENTRY(SPACC_REG_HSM_CMD_REQ),
   HW_ENTRY(SPACC_REG_HSM_CMD_GNT),
   HW_ENTRY(SPACC_REG_SRC_PTR),
   HW_ENTRY(SPACC_REG_DST_PTR),
   HW_ENTRY(SPACC_REG_OFFSET),
   HW_ENTRY(SPACC_REG_PRE_AAD_LEN),
   HW_ENTRY(SPACC_REG_POST_AAD_LEN),
   HW_ENTRY(SPACC_REG_PROC_LEN),
   HW_ENTRY(SPACC_REG_ICV_LEN),
   HW_ENTRY(SPACC_REG_ICV_OFFSET),
   HW_ENTRY(SPACC_REG_IV_OFFSET),
   HW_ENTRY(SPACC_REG_SW_CTRL),
   HW_ENTRY(SPACC_REG_AUX_INFO),
   HW_ENTRY(SPACC_REG_CTRL),
   HW_ENTRY(SPACC_REG_STAT_POP),
   HW_ENTRY(SPACC_REG_STATUS),
   HW_ENTRY(SPACC_REG_STAT_WD_CTRL),
   HW_ENTRY(SPACC_REG_KEY_SZ),
   HW_ENTRY(SPACC_REG_VIRTUAL_RQST),
   HW_ENTRY(SPACC_REG_VIRTUAL_ALLOC),
   HW_ENTRY(SPACC_REG_VIRTUAL_PRIO),
   HW_ENTRY(SPACC_REG_VIRTUAL_RC4_KEY_RQST),
   HW_ENTRY(SPACC_REG_VIRTUAL_RC4_KEY_GNT),
   HW_ENTRY(SPACC_REG_ID),
   HW_ENTRY(SPACC_REG_CONFIG),
   HW_ENTRY(SPACC_REG_CONFIG2),
   HW_ENTRY(SPACC_REG_HSM_VERSION),
   HW_ENTRY(SPACC_REG_SECURE_CTRL),
   HW_ENTRY(SPACC_REG_SECURE_RELEASE),
   HW_ENTRY(SPACC_REG_SK_LOAD),
   HW_ENTRY(SPACC_REG_SK_STAT),
   HW_ENTRY(SPACC_REG_SK_KEY),
   HW_ENTRY(SPACC_REG_HSM_CTX_CMD),
   HW_ENTRY(SPACC_REG_HSM_CTX_STAT),

   { 0, NULL },
};
#undef HW_ENTRY

static uint32_t reg_epn = 0x0, reg_virt = 0;

static ssize_t show_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   char *name, out[128];
   unsigned x, reg_addr;
   spacc_device *spacc = get_spacc_device_by_epn(reg_epn, reg_virt);

   if (!spacc) {
      return sprintf(buf, "Could not find SPAcc device (EPN=%lx,%lu), please write to this file first in the form <epn,virt>\n",
                     (unsigned long)reg_epn, (unsigned long)reg_virt);
   }

   buf[0] = out[0] = 0;
   for (reg_addr = 0; reg_addr < 0x300; reg_addr += 4) {
      name = NULL;
      for (x = 0; reg_names[x].name != NULL; x++) {
         if (reg_names[x].addr == reg_addr) {
            name = reg_names[x].name;
            break;
         }
      }
      if (name == NULL) { continue; }
      sprintf(out, "%-35s = %08lx\n", name, (unsigned long)pdu_io_read32(spacc->regmap + reg_addr));
      strcat(buf, out);
   }
   return strlen(buf);
}

static ssize_t store_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   unsigned long x, y;

   if (sscanf(buf, "%lx,%lu", &x, &y) != 2)
      return -EINVAL;

   reg_epn = x;
   reg_virt = y;
   return count;
}

static DEVICE_ATTR(reg,                0600, show_reg, store_reg);
static const struct attribute_group spacc_attr_group = {
   .attrs = (struct attribute *[]) {
      &dev_attr_reg.attr,
      NULL
   }
};


static int __devinit spacc_probe(struct platform_device *pdev)
{
   void *baseaddr;
   struct resource *mem, *irq;
   int x, err, oldmode;
   struct spacc_priv   *priv;
   pdu_info     info;

   dev_dbg(&pdev->dev, "probe called!\n");

   /* Initialize DDT DMA pools based on this device's resources */
   if (pdu_mem_init(&pdev->dev)) {
      dev_err(&pdev->dev, "Could not initialize DMA pools\n");
      return -ENOMEM;
   }

   mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
   if (!mem || !irq) {
      dev_err(&pdev->dev, "no memory/irq resource for spacc\n");
      return -ENXIO;
   }

   priv = devm_kzalloc(&pdev->dev, sizeof *priv, GFP_KERNEL);
   if (!priv) {
      dev_err(&pdev->dev, "no memory for spacc private data\n");
      return -ENOMEM;
   }

   dev_dbg(&pdev->dev, "spacc_probe: Device at %pR\n", mem);
   pr_debug("spacc_probe: Device at %pR\n", irq);
   baseaddr = pdu_linux_map_regs(&pdev->dev, mem);
   if (IS_ERR(baseaddr)) {
      dev_err(&pdev->dev, "unable to map iomem\n");
      return PTR_ERR(baseaddr);
   }

   x = pdev->id;
   dev_dbg(&pdev->dev, "EPN %04X : virt [%d] \n", (x >> 16) & 0xFFFF, x & 0xF);

   pdu_get_version(baseaddr, &info);
   if (pdev->dev.platform_data) {
      pdu_info *parent_info = pdev->dev.platform_data;
      memcpy(&info.pdu_config, &parent_info->pdu_config, sizeof info.pdu_config);
   }

   err = spacc_init (baseaddr, &priv->spacc, &info);
   if (err != CRYPTO_OK) {
      dev_err(&pdev->dev, "spacc_probe::Failed to initialize device %d...\n", x);
      return -ENXIO;
   }

   err = sysfs_create_group(&pdev->dev.kobj, &spacc_attr_group);
   if (err) {
      spacc_fini(&priv->spacc);
      return -1;
   }

   spin_lock_init(&priv->hw_lock);
   spacc_irq_glbl_disable (&priv->spacc);
   tasklet_init(&priv->pop_jobs, spacc_pop_jobs, (unsigned long)priv);
   platform_set_drvdata(pdev, priv);

   /* Determine configured maximum message length. */
   priv->max_msg_len = priv->spacc.config.max_msg_size;

   if (devm_request_irq(&pdev->dev, irq->start, spacc_irq_handler, IRQF_SHARED, dev_name(&pdev->dev), &pdev->dev)) {
      dev_err(&pdev->dev, "failed to request IRQ\n");
      return -EBUSY;
   }

   /* Perform autodetect in no_latency=1 mode */
      priv->spacc.irq_cb_stat = spacc_stat_process;
      priv->spacc.irq_cb_cmdx = spacc_cmd_process;
      oldmode = priv->spacc.op_mode;
      priv->spacc.op_mode     = SPACC_OP_MODE_IRQ;

      spacc_irq_stat_enable (&priv->spacc, 1);
      spacc_irq_cmdx_enable(&priv->spacc, 0, 1);
      spacc_irq_stat_wd_disable (&priv->spacc);
      spacc_irq_glbl_enable (&priv->spacc);

#ifndef MAKEAVECTOR
      spacc_autodetect(&priv->spacc);
#endif
      priv->spacc.op_mode = oldmode;

   /* register irq callback function */
   if (no_latency) {
      // used to set lower latency mode on newer SPAcc device v4.11 and up
      // set above during autodetect
      priv->spacc.op_mode     = SPACC_OP_MODE_IRQ;
      pr_debug("spacc:: Using low latency IRQ mode\n");
   } else {
      if (priv->spacc.op_mode == SPACC_OP_MODE_IRQ) {
         priv->spacc.irq_cb_stat = spacc_stat_process;
         priv->spacc.irq_cb_cmdx = spacc_cmd_process;

         spacc_irq_stat_enable (&priv->spacc, 1);
         spacc_irq_cmdx_enable(&priv->spacc, 0, 1);
         spacc_irq_glbl_enable (&priv->spacc);
      } else {
         priv->spacc.irq_cb_stat    = spacc_stat_process;
         priv->spacc.irq_cb_stat_wd = spacc_stat_process;

         spacc_irq_stat_enable (&priv->spacc, priv->spacc.config.ideal_stat_level);
         spacc_irq_cmdx_disable(&priv->spacc, 0);
         spacc_irq_stat_wd_enable (&priv->spacc);
         spacc_irq_glbl_enable (&priv->spacc);

         /* enable the wd */
         spacc_set_wd_count(&priv->spacc, priv->spacc.config.wd_timer = timer);
      }
   }

   // unlock normal
   if (priv->spacc.config.is_hsm_shared && priv->spacc.config.is_secure_port) {
      uint32_t t;
      t = pdu_io_read32(baseaddr + SPACC_REG_SECURE_CTRL);
      t &= ~(1UL<<31);
      pdu_io_write32(baseaddr + SPACC_REG_SECURE_CTRL, t);
   }

   // unlock device by default
   pdu_io_write32(baseaddr + SPACC_REG_SECURE_CTRL, 0);

   return err;
}

static int __devexit spacc_remove(struct platform_device *pdev)
{
   spacc_device *spacc;

   // free test vector memory
   spacc = &((struct spacc_priv *)platform_get_drvdata(pdev))->spacc;
   spacc_fini(spacc);
   sysfs_remove_group(&pdev->dev.kobj, &spacc_attr_group);

   pdu_mem_deinit(&pdev->dev);

   /* devm functions do proper cleanup */
   dev_info(&pdev->dev, "removed!\n");

   return 0;
}


static long spacc_kernel_irq_ioctl (struct file *fp, unsigned int cmd, unsigned long arg_)
{
   elpspacc_irq_ioctl io;
   spacc_device *spacc;
   void __user *arg = (void __user *) arg_;
   unsigned long flags;

   if (unlikely (copy_from_user (&io, arg, sizeof (io)))) {
      printk ("spacc_irq_ioctl::Cannot copy ioctl buffer from user\n");
      return -EFAULT;
   }

   spacc = get_spacc_device_by_epn (io.spacc_epn, io.spacc_virt);
   if (!spacc) {
      printk("spacc_irq_ioctl::Cannot find SPAcc %lx/%lx\n",
             (unsigned long)io.spacc_epn, (unsigned long)io.spacc_virt);
      return -EIO;
   }

   if (io.command == SPACC_IRQ_CMD_SET) {
      // lock spacc
      PDU_LOCK(&spacc->lock, flags);

      // first disable everything
      spacc_irq_stat_disable(spacc);
      spacc_irq_cmdx_disable(spacc, 0);
      spacc_irq_stat_wd_disable (spacc);
      spacc_irq_glbl_disable (spacc);

      if (io.irq_mode == SPACC_IRQ_MODE_WD) {
         // set WD mode
         spacc->irq_cb_stat    = spacc_stat_process;
         spacc->irq_cb_stat_wd = spacc_stat_process;
         spacc->irq_cb_cmdx    = NULL;

         spacc_irq_stat_enable (spacc, io.stat_value ? io.stat_value : spacc->config.ideal_stat_level);
         spacc_irq_stat_wd_enable (spacc);
         spacc_set_wd_count(spacc, io.wd_value ? io.wd_value : spacc->config.wd_timer);
         spacc->op_mode = SPACC_OP_MODE_WD;
      } else {
         // set STEP mode
         spacc->irq_cb_stat    = spacc_stat_process;
         spacc->irq_cb_cmdx    = spacc_cmd_process;
         spacc->irq_cb_stat_wd = NULL;

         spacc_irq_stat_enable(spacc, io.stat_value ? io.stat_value : 1);
         spacc_irq_cmdx_enable(spacc, 0, io.cmd_value ? io.cmd_value : 1);
         spacc->op_mode = SPACC_OP_MODE_IRQ;
      }
      spacc_irq_glbl_enable (spacc);
      PDU_UNLOCK(&spacc->lock, flags);
   }

   return 0;
}

static struct file_operations spacc_kernel_irq_fops = {
   .owner = THIS_MODULE,
   .unlocked_ioctl = spacc_kernel_irq_ioctl,
};

static struct miscdevice spaccirq_device = {
   .minor = MISC_DYNAMIC_MINOR,
   .name = "spaccirq",
   .fops = &spacc_kernel_irq_fops,
};

static struct platform_driver spacc_driver = {
   .probe  = spacc_probe,
   .remove = __devexit_p(spacc_remove),
   .driver = {
      .name  = "spacc",
      .owner = THIS_MODULE
   },
};

static int __init spacc_mod_init (void)
{
   int err;

   err = misc_register (&spaccirq_device);
   if (err) {
      return err;
   }

   err = platform_driver_register(&spacc_driver);
   if (err) {
      misc_deregister (&spaccirq_device);
   }
   return err;
}

static void __exit spacc_mod_exit (void)
{
   misc_deregister (&spaccirq_device);
   platform_driver_unregister(&spacc_driver);
}

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init (spacc_mod_init);
module_exit (spacc_mod_exit);

// export wrapped library functions
EXPORT_SYMBOL (spacc_open);
EXPORT_SYMBOL (spacc_clone_handle);
EXPORT_SYMBOL (spacc_close);
//EXPORT_SYMBOL (spacc_status);
EXPORT_SYMBOL (spacc_write_context);
EXPORT_SYMBOL (spacc_read_context);
EXPORT_SYMBOL (spacc_write_rc4_context);
EXPORT_SYMBOL (spacc_read_rc4_context);
EXPORT_SYMBOL (spacc_error_msg);
EXPORT_SYMBOL (spacc_set_operation);
EXPORT_SYMBOL (spacc_set_key_exp);
EXPORT_SYMBOL (spacc_set_auxinfo);
EXPORT_SYMBOL (spacc_packet_enqueue_ddt);
EXPORT_SYMBOL (spacc_packet_dequeue);
EXPORT_SYMBOL (spacc_virtual_set_weight);
EXPORT_SYMBOL (spacc_virtual_request_rc4);
EXPORT_SYMBOL (spacc_pop_packets);
EXPORT_SYMBOL (spacc_load_skp);
EXPORT_SYMBOL (spacc_dump_ctx);
EXPORT_SYMBOL (spacc_set_wd_count);

EXPORT_SYMBOL(spacc_irq_cmdx_enable);
EXPORT_SYMBOL(spacc_irq_cmdx_disable);
EXPORT_SYMBOL(spacc_irq_stat_enable);
EXPORT_SYMBOL(spacc_irq_stat_disable);
EXPORT_SYMBOL(spacc_irq_stat_wd_enable);
EXPORT_SYMBOL(spacc_irq_stat_wd_disable);
EXPORT_SYMBOL(spacc_irq_rc4_dma_enable);
EXPORT_SYMBOL(spacc_irq_rc4_dma_disable);
EXPORT_SYMBOL(spacc_irq_glbl_enable);
EXPORT_SYMBOL(spacc_irq_glbl_disable);
EXPORT_SYMBOL(spacc_process_irq);

EXPORT_SYMBOL(spacc_compute_xcbc_key);
EXPORT_SYMBOL(spacc_isenabled);

// used by RE/KEP
EXPORT_SYMBOL (spacc_ctx_request);
EXPORT_SYMBOL (spacc_ctx_release);

