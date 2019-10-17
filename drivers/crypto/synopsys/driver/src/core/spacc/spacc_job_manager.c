
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

/* Return a job structure for a swid or null if invalid */
spacc_job * job_lookup_by_swid (spacc_device * spacc, int swid)
{
   if ((swid < 0) || (swid >= SPACC_MAX_JOBS)) {
      ELPHW_PRINT ("job_lookup::Invalid swid number\n");
      return NULL;
   }
   return &spacc->job[spacc->job_lookup[swid]];
}

