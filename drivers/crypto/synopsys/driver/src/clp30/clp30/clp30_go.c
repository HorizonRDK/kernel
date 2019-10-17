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

static int clp30_go_ex(clp30_device *clp30, int handle, int direction, pdu_ddt *src, pdu_ddt *dst, clp30_callback cb, void *cb_data)
{
   int err, j;
   uint32_t offset;

   if (handle < 0 || handle > clp30->num_ctx || clp30->ctx[handle].allocated == 0) {
      return -1;
   }

   if (!direction) {
      offset = 0x20;
   } else {
      offset = 0;
   }

#ifndef CLP36_ENABLED
   /* is fifo is full */
   if (CLP30_STAT_CMD_FIFO_FULL(pdu_io_read32(clp30->regmap + offset + CLP30_REG_OUT_STAT))) {
      return CLP30_FIFO_FULL;
   }
#else
   if (CLP36_FIFO_STAT_CMD_FULL(pdu_io_read32(clp30->regmap + CLP36_REG_OUT_FIFO_STAT + (!direction ? 4 : 0)))) {
      return CLP30_FIFO_FULL;
   }
#endif

   err = 0;

   /* we're running the job now */

   // find a slot for the job
   for (j = 0; j < MAXJOBS; j++) {
      if (clp30->ctx[handle].done[j] == 1) {
         break;
      }
   }
   if (j == MAXJOBS) {
      err = CLP30_JOBS_FULL;
      goto ERR;
   }

   // have slot, allocate handle if needed
   clp30->ctx[handle].job_cnt++;
   clp30->ctx[handle].done[j]   = 0;
   clp30->ctx[handle].cb[j]     = cb;
   clp30->ctx[handle].cbdata[j] = cb_data;

#if 0
{
   int x;
   printk("SRC DDT == %08lx\n", src->phys);
   for (x = 0; x <= src->idx; x++) {
      printk("SRC DDT[%2d] == %08lx\n", 2*x+0, src->virt[2*x+0]);
      printk("SRC DDT[%2d] == %08lx\n", 2*x+1, src->virt[2*x+1]);
   }
   printk("DST DDT == %08lx\n", dst->phys);
   for (x = 0; x <= dst->idx; x++) {
      printk("DST DDT[%2d] == %08lx\n", 2*x+0, dst->virt[2*x+0]);
      printk("DST DDT[%2d] == %08lx\n", 2*x+1, dst->virt[2*x+1]);
   }
   printk("SAI == 0x%08zx\n", clp30->sa_ptr_phys + SA_SIZE*handle);
}
#endif


   // write out engine
   pdu_io_cached_write32(clp30->regmap + offset + CLP30_REG_OUT_SRC_PTR, (uint32_t) src->phys, &(clp30->cache[direction].src_ptr));
   pdu_io_cached_write32(clp30->regmap + offset + CLP30_REG_OUT_DST_PTR, (uint32_t) dst->phys, &(clp30->cache[direction].dst_ptr));
   pdu_io_cached_write32(clp30->regmap + offset + CLP30_REG_OUT_OFFSET, 0, &(clp30->cache[direction].offset));

   err = pdu_io_read32(clp30->regmap + offset + CLP30_REG_OUT_ID);
   err = ((err - 0) & 0xFF) | ((!!direction)<<8);
   clp30->ctx[handle].swid[j] = err;
   clp30->swid[clp30->ctx[handle].swid[j]] = (handle<<16)|(j);

   pdu_io_write32(clp30->regmap + offset + CLP30_REG_OUT_SAI, clp30->sa_ptr_phys + SA_SIZE * handle);
ERR:
   return err;
}


int clp30_go(clp30_device *clp30, int handle, int direction, pdu_ddt *src, pdu_ddt *dst, clp30_callback cb, void *cb_data)
{
   int err;
   unsigned long flag;

   PDU_LOCK(&clp30->lock, flag);
   err = clp30_go_ex(clp30, handle, direction, src, dst, cb, cb_data);
   PDU_UNLOCK(&clp30->lock, flag);
   return err;
}
