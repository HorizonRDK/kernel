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

#include "eape.h"

int eape_pop_packet(eape_device * eape)
{
   uint32_t offset, len, retcode, sw_id, status, sttl, x, y, jobs;
   unsigned long flags;
   elpxfrm_callback cb;
   void *cb_data;

   PDU_LOCK(&eape->lock, flags);

   for (offset = 0; offset < 0x40; offset += 0x20) {
      // pop [if any] data off the STAT FIFO, we can't read the STAT register until we pop so we can't even tell if there are jobs to pop until then
      //Check the outbound FIFO first
      for (;;) {
         jobs = pdu_io_read32(eape->regmap + EAPE_REG_FIFO_STAT);
         if (offset) {
            jobs = EAPE_GET_REG_BITS(jobs, EAPE_FIFO_STAT_IN_STAT_CNT_MASK, EAPE_FIFO_STAT_IN_STAT_CNT_OFFSET);
         } else {
            jobs = EAPE_GET_REG_BITS(jobs, EAPE_FIFO_STAT_OUT_STAT_CNT_MASK, EAPE_FIFO_STAT_OUT_STAT_CNT_OFFSET);
         }
         if (!jobs) {
            break;
         }
         while (jobs--) {
            pdu_io_write32(eape->regmap + offset + EAPE_REG_OUT_POP, 1);

            status  = pdu_io_read32(eape->regmap + offset + EAPE_REG_OUT_STAT);
            len     = EAPE_GET_REG_BITS(status, EAPE_OUT_STAT_LENGTH_MASK,   EAPE_OUT_STAT_LENGTH_OFFSET);
            sw_id   = EAPE_GET_REG_BITS(status, EAPE_OUT_STAT_ID_MASK,       EAPE_OUT_STAT_ID_OFFSET);
            retcode = EAPE_GET_REG_BITS(status, EAPE_OUT_STAT_RET_CODE_MASK, EAPE_OUT_STAT_RET_CODE_OFFSET);
            sttl    = EAPE_GET_REG_BITS(status, EAPE_OUT_STAT_RET_CODE_MASK, EAPE_OUT_STAT_STTL_OFFSET);
      #ifdef EAPE_DEBUG
            printk(KERN_DEBUG "SWID popped off FIFO(%02zx) swid==%zu, status==%zx, len==%zu retcode==%zu sttl==%zu\n", offset, sw_id, status, len, retcode, sttl);
      #endif
            // look up the sw_id
            sw_id |= (offset ? 0x100 : 0);   // add 256 if it's inbound
            x = eape->swid[sw_id] >> 16;     // handle
            y = eape->swid[sw_id] & 0xFFFF;  // job slot inside context
            if (!((x < eape->status.num_ctx) && (y < MAXJOBS) && (eape->ctx[x].swid[y] == sw_id))) {
               ELPHW_PRINT("eape_pop_packets::Invalid SW_ID(%zx) returned by EAPE\n", sw_id);
               PDU_UNLOCK(&eape->lock, flags);
               return -1;
            }
            eape->swid[sw_id] = 0xFFFFFFFF;  // reset lookup so it won't match anything again

            /* Store callback information and mark job as done. */
            cb      = eape->ctx[x].cb[y];
            cb_data = eape->ctx[x].cbdata[y];

            eape->ctx[x].cb[y]   = NULL;
            eape->ctx[x].done[y] = 1;
            --(eape->ctx[x].job_cnt);

            if (eape->ctx[x].dismiss && eape->ctx[x].job_cnt == 0) {
               if (eape->ctx[x].dismiss) {
                  eape->ctx[x].allocated = 0;
               }
            }

            if (cb) {
               PDU_UNLOCK(&eape->lock, flags);
               cb(eape, cb_data, len, retcode, sw_id);
               PDU_LOCK(&eape->lock, flags);
            }
         }
      }
   }
   PDU_UNLOCK(&eape->lock, flags);
   return 0;
}
