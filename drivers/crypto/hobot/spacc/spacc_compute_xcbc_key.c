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
 * Copyright (c) 2012-2015 Synopsys, Inc. and/or its affiliates.
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
