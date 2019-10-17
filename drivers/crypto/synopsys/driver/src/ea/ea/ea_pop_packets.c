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

#include "elpea.h"

int ea_pop_packet(ea_device *ea)
{
  uint32_t len, retcode, sw_id, sttl, status, x, y;
  unsigned long flags;
  int      jobs;
  elpxfrm_callback cb;
  void *cb_data;

  PDU_LOCK(&ea->spacc->lock, flags);

   for (;;) {
      if (ea->spacc->config.pdu_version >= 0x25) {
         jobs = EA_FIFO_STAT_STAT_CNT(pdu_io_read32(ea->regmap + EA_FIFO_STAT));
      } else {
         jobs = EA_FIFO_STAT_STAT_CNT_V24(pdu_io_read32(ea->regmap + EA_FIFO_STAT));
      }
      if (!jobs) {
         break;
      }

     while (jobs-- > 0) {
        pdu_io_write32(ea->regmap + EA_POP, 1);
        status  = pdu_io_read32(ea->regmap + EA_STATUS);
        len     = EA_STATUS_LEN(status);
        sw_id   = EA_STATUS_SW_ID(status);
        retcode = EA_STATUS_RET_CODE(status);
        sttl    = EA_STATUS_STTL(status);
   //     printk(KERN_DEBUG "SWID popped off FIFO swid==%zu, status==%zx, len==%zu retcode==%zu sttl==%zu\n", sw_id, status, len, retcode, sttl);

        // look up the sw_id
        x = ea->swid[sw_id] >> 16;    // handle
        y = ea->swid[sw_id] & 0xFFFF; // job slot inside context
        if (!((x < ea->num_ctx) && (y < MAXJOBS) && (ea->ctx[x].swid[y] == sw_id))) {
           ELPHW_PRINT("ea_pop_packets::Invalid SW_ID returned by SPAcc-EA %08zx\n", sw_id);
           PDU_UNLOCK(&ea->spacc->lock, flags);
           return -1;
        }
        ea->swid[sw_id] = 0xFFFFFFFF;  // reset lookup so it won't match anything again

        /* Store callback information and mark job as done. */
        cb      = ea->ctx[x].cb[y];
        cb_data = ea->ctx[x].cbdata[y];

        ea->ctx[x].cb[y]   = NULL;
        ea->ctx[x].done[y] = 1;
        --(ea->ctx[x].job_cnt);

        if ((ea->ctx[x].dismiss || ea->ctx[x].lock_ctx == 0) && ea->ctx[x].job_cnt == 0) {
           // free spacc handle
           spacc_ctx_release (ea->spacc, ea->ctx[x].spacc_handle);
           ea->ctx[x].spacc_handle = -1;

           if (ea->ctx[x].dismiss) {
              ea->ctx[x].allocated = 0;
           }
        }

        if (cb) {
           PDU_UNLOCK(&ea->spacc->lock, flags);
           cb(ea, cb_data, len, retcode, sw_id);
           PDU_LOCK(&ea->spacc->lock, flags);
        }
     }
  }
  // are there any jobs pending in the job buffer...
  if (ea->ea_job_buf_use) {
    ea->ea_job_buf_use = 0;
    for (x = 0; x < EA_MAX_BACKLOG; x++) {
      if (ea->ea_job_buf[x].active) {
        y = ea_go_ex(ea, 0, ea->ea_job_buf[x].handle, ea->ea_job_buf[x].direction, ea->ea_job_buf[x].src, ea->ea_job_buf[x].dst, ea->ea_job_buf[x].cb, ea->ea_job_buf[x].cb_data);
        if (y != EA_FIFO_FULL) {
           ea->ea_job_buf[x].active = 0;
        } else {
           ea->ea_job_buf_use |= 1;
        }
      }
    }
  }
  PDU_UNLOCK(&ea->spacc->lock, flags);
  return 0;
}

