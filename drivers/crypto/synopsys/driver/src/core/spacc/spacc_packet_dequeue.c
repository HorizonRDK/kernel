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
