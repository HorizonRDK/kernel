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
 * Copyright (c) 2015-2017 Synopsys, Inc. and/or its affiliates.
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
