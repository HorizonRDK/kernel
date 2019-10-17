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
 * Copyright (c) 2013 Synopsys, Inc. and/or its affiliates.
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

#include "clp30.h"

int clp30_pop_packet(clp30_device *clp30)
{
  uint32_t  len, retcode, sw_id, status, x, y, offset;
  unsigned long flags;
  clp30_callback cb;
  void *cb_data;

  PDU_LOCK(&clp30->lock, flags);

  for (offset = 0; offset < 0x40; offset += 0x20) {
     // pop [if any] data off the STAT FIFO, we can't read the STAT register until we pop so we can't even tell if there are jobs to pop until then
#ifndef CLP36_ENABLED
     while (!CLP30_STAT_STAT_FIFO_EMPTY(pdu_io_read32(clp30->regmap + offset + CLP30_REG_OUT_STAT))) {
#else
     while (!CLP36_FIFO_STAT_STAT_EMPTY(pdu_io_read32(clp30->regmap + (offset >> 3) + CLP36_REG_OUT_FIFO_STAT))) {
#endif
        pdu_io_write32(clp30->regmap + offset + CLP30_REG_OUT_POP, 1);
        status = pdu_io_read32(clp30->regmap + offset + CLP30_REG_OUT_STAT);
        len     = CLP30_STAT_LENGTH(status);
        sw_id   = CLP30_STAT_ID(status) | ((!offset)<<8); // add a 9th bit to indicate direction (0x100 == outbound, 0x000 == inbound)
        retcode = CLP30_STAT_RET_CODE(status);

        //     printk(KERN_DEBUG "SWID popped off FIFO swid==%zu, status==%zx, len==%zu retcode==%zu sttl==%zu\n", sw_id, status, len, retcode, sttl);

        // look up the sw_id
        x = clp30->swid[sw_id] >> 16;    // handle
        y = clp30->swid[sw_id] & 0xFFFF; // job slot inside context
        if (!((x < clp30->num_ctx) && (y < MAXJOBS) && (clp30->ctx[x].swid[y] == sw_id))) {
           ELPHW_PRINT("clp30_pop_packets::Invalid SW_ID(%zx) returned by CLP-30\n", sw_id);
           PDU_UNLOCK(&clp30->lock, flags);
           return -1;
        }
        clp30->swid[sw_id] = 0xFFFFFFFF;  // reset lookup so it won't match anything again

        /* Store callback information and mark job as done. */
        cb = clp30->ctx[x].cb[y];
        cb_data = clp30->ctx[x].cbdata[y];

        clp30->ctx[x].cb[y]   = NULL;
        clp30->ctx[x].done[y] = 1;
        --(clp30->ctx[x].job_cnt);

        if (clp30->ctx[x].dismiss && clp30->ctx[x].job_cnt == 0) {
           if (clp30->ctx[x].dismiss) {
              clp30->ctx[x].allocated = 0;
           }
        }

        if (cb) {
           PDU_UNLOCK(&clp30->lock, flags);
           cb(clp30, cb_data, len, retcode, sw_id);
           PDU_LOCK(&clp30->lock, flags);
        }
     }
  }
  PDU_UNLOCK(&clp30->lock, flags);
  return 0;
}
