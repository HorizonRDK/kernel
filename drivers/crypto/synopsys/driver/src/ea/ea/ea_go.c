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

static int cmd_full(ea_device *ea, int fifo)
{
   if (!fifo) {
      return EA_FIFO_STAT_CMD_FULL(pdu_io_read32(ea->regmap + EA_FIFO_STAT));
   } else {
      return EA_FIFO_STAT_CMD1_FULL(pdu_io_read32(ea->regmap + EA_FIFO_STAT));
   }
}

int ea_go_ex(ea_device *ea, int use_job_buf, int handle, int direction, pdu_ddt *src, pdu_ddt *dst, elpxfrm_callback cb, void *cb_data)
{
   int err, j, fifo;
   unsigned first;

   if (handle < 0 || handle > ea->num_ctx || ea->ctx[handle].allocated == 0) {
      return -1;
   }

   fifo = direction ? ea->config.tx_fifo : ea->config.rx_fifo;

   if (cmd_full(ea, fifo)) {
      if (ea->config.tx_fifo != ea->config.rx_fifo) {
         fifo ^= 1;
         if (cmd_full(ea, fifo)) {
            goto add_to_buffer;
         }
      } else {
         goto add_to_buffer;
      }
   }

   /* we're running the job now */

   // find a slot for the job
   for (j = 0; j < MAXJOBS; j++) {
      if (ea->ctx[handle].done[j] == 1) {
         break;
      }
   }
   if (j == MAXJOBS) {
      err = EA_JOBS_FULL;
      goto ERR;
   }

   // have slot, allocate handle if needed
   first = 0;
   if (ea->ctx[handle].spacc_handle == -1) {
      first = 1;
      ea->ctx[handle].spacc_handle = spacc_ctx_request(ea->spacc, -1, 1);
      if (ea->ctx[handle].spacc_handle < 0) {
         return EA_NO_CONTEXT;
      }
   }

   ea->ctx[handle].job_cnt++;
   ea->ctx[handle].done[j]   = 0;
   ea->ctx[handle].cb[j]     = cb;
   ea->ctx[handle].cbdata[j] = cb_data;

   // write out engine
   if (ea->spacc->config.dma_type == SPACC_DMA_DDT) {
      pdu_io_cached_write32 (ea->regmap + EA_SRC_PTR, (uint32_t) src->phys, &(ea->cache.src_ptr));
      pdu_io_cached_write32 (ea->regmap + EA_DST_PTR, (uint32_t) dst->phys, &(ea->cache.dst_ptr));
   } else if (ea->spacc->config.dma_type == SPACC_DMA_LINEAR) {
      pdu_io_cached_write32 (ea->regmap + EA_SRC_PTR, (uint32_t) src->virt[0], &(ea->cache.src_ptr));
      pdu_io_cached_write32 (ea->regmap + EA_DST_PTR, (uint32_t) dst->virt[0], &(ea->cache.dst_ptr));
   } else {
      err = -1;
      goto ERR;
   }

   pdu_io_cached_write32(ea->regmap + EA_OFFSET, 0, &(ea->cache.offset));
   pdu_io_cached_write32(ea->regmap + EA_SA_PTR, ea->sa_ptr_phys + SA_SIZE * handle, &(ea->cache.sa_ptr));

   err = ea->ctx[handle].swid[j] = pdu_io_read32(ea->regmap + EA_SW_CTRL);
   ea->swid[ea->ctx[handle].swid[j]] = (handle<<16)|(j);

   pdu_io_write32(ea->regmap + EA_CTRL, EA_CTRL_FIFO_SEL(fifo)|EA_CTRL_CMD(direction)|EA_CTRL_CTX_ID(ea->ctx[handle].spacc_handle)|EA_CTRL_LD_CTX(first));

ERR:
   return err;

add_to_buffer:
   if (use_job_buf) {
      // no more room in FIFO so add to job queue
      // find room in job buff
      for (j = 0; j < EA_MAX_BACKLOG; j++) {
         if (ea->ea_job_buf[j].active == 0) {
            // add it here
            ea->ea_job_buf[j] = (struct ea_job_buf) {.handle = handle, .direction = direction, .src = src, .dst = dst, .cb = cb, .cb_data = cb_data, .active = 1 };
            ea->ea_job_buf_use |= 1;
            return 256;
         }
      }
   }
   return EA_FIFO_FULL;
}


int ea_go(ea_device *ea, int handle, int direction, pdu_ddt *src, pdu_ddt *dst, elpxfrm_callback cb, void *cb_data)
{
   int err;
   unsigned long flag;

   PDU_LOCK(&ea->spacc->lock, flag);
   err = ea_go_ex(ea, 1, handle, direction, src, dst, cb, cb_data);
   PDU_UNLOCK(&ea->spacc->lock, flag);
   return err;
}

