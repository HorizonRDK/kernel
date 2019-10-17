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

#include "elpre.h"
#include "elprehw.h"

int re_get_spacc_context(re_device *re, int re_ctx)
{
    if (re_ctx < 0 || re_ctx >= re->config.num_ctx) {
       ELPHW_PRINT ("re_get_spacc_context: Invalid Context");
       return CRYPTO_INVALID_CONTEXT;
    }

    return re->re_contexts[re_ctx].spacc_handle;
 }

int re_get_context_ex (re_device * re, int ctxid, re_callback cb, void *cbdata)
{
   int i, j;
   unsigned long flag;

   PDU_LOCK(&re->spacc->lock, flag);

   // find a RE context to use
   for (j = 0; j < SPACC_MAX_JOBS; j++) {
      if (re->re_contexts[j].spacc_handle == -1) { break; }
   }
   if (j == SPACC_MAX_JOBS) {
      PDU_UNLOCK(&re->spacc->lock, flag);
      return -1;
   }

   // allocate a spacc context
   i = spacc_ctx_request (re->spacc, ctxid, 1);
   if (i < 0) {
      PDU_UNLOCK(&re->spacc->lock, flag);
      return -1;
   }


   // TODO: if we are not finding a new one we need a new RE context but we bind it to this handle
   if (re_init_context_ex (re, j, i) != CRYPTO_OK) {
      PDU_UNLOCK(&re->spacc->lock, flag);
      return CRYPTO_FAILED;
   }

   re->re_contexts[j].cb     = cb;
   re->re_contexts[j].cbdata = cbdata;

   PDU_UNLOCK(&re->spacc->lock, flag);
   return j;
}


/*
 * Get an initialized context.. Will return CRYPTO_FAILED
 * if there are no available contexts... or if the context
 * is invalid,or it will return the context number if
 * it succeeded.
 */
int re_get_context (re_device * re, re_callback cb, void *cbdata)
{
   return re_get_context_ex(re, -1, cb, cbdata);
}
