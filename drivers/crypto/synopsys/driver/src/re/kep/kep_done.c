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
#include "elpkep.h"

/* the basic idea here is this function "finishes" a job, but not necessarily the job
the caller was interested in.  So we read the STAT fifo search the ctx table for the job
id and mark the job_done flag.  This function returns CRYPTO_OK if the job_done flag
for the callers handle is marked true.  Otherwise it returns INPROGRESS */

int kep_done (kep_device * kep, int handle)
{
   unsigned long lock_flag;
   int ret;

   if (handle != -1 && kep_is_valid (kep, handle)) {
      ELPHW_PRINT ("kep_done::Invalid handle specified\n");
      return CRYPTO_FAILED;
   }
   if (handle != -1 && kep->ctx[handle].job_done == 1) {
      return CRYPTO_OK;
   }

   if (handle != -1 && kep->ctx[handle].job_id == -1) {
      ELPHW_PRINT ("kep_done::Invalid handle as job is marked as not active\n");
      return CRYPTO_FAILED;
   }

   PDU_LOCK(&kep->lock, lock_flag);

   // is job already done?
   if (handle != -1 && kep->ctx[handle].job_done == 1) {
      ret = CRYPTO_OK;
      goto ERR;
   }

   if (!KEP_FIFO_STAT_EMPTY (pdu_io_read32 (kep->regmap + KEP_REG_FIFO_STAT))) {
      uint32_t status, swid, retcode, x, y;

      pdu_io_write32 (kep->regmap + KEP_REG_POP, 1);

      status = pdu_io_read32 (kep->regmap + KEP_REG_STATUS);
      swid = KEP_STATUS_SWID (status);
      retcode = KEP_STATUS_RETCODE (status);

      switch (retcode) {
         case 0:               // OK
            y = kep->spacc->config.num_ctx;
            for (x = 0; x < y; x++) {
               if (swid == kep->ctx[x].job_id) {
                  kep->ctx[x].job_done = 1;
                  if (kep->ctx[x].cb) {
                     kep->ctx[x].cb(kep, kep->ctx[x].cbdata);
                  }
                  kep->ctx[x].job_id = -1;
                  goto end;
               }
            }
            ELPHW_PRINT ("kep_done::Job ID not found!\n");
            ret = CRYPTO_FAILED;
            goto ERR;
         default:
            ELPHW_PRINT ("kep_done::KEP returned error %d\n", retcode);
            ret = CRYPTO_FAILED;
            goto ERR;
      }
   } else {
     ELPHW_PRINT("KEP: done called but no job posted\n");
   }
 end:
   if (handle != -1) {
      if (kep->ctx[handle].job_done == 1) {
         ret = CRYPTO_OK;
      } else {
         ret = CRYPTO_INPROGRESS;
      }
   } else {
      ret = CRYPTO_OK;
   }
ERR:
   PDU_UNLOCK(&kep->lock, lock_flag);
   return ret;
}
