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

static int eape_go_ex(eape_device * eape, int handle, int direction, pdu_ddt * src, pdu_ddt * dst, elpxfrm_callback cb, void *cb_data)
{
   int err = EAPE_OK, j;
   uint32_t offset, swid;

   //Check for valid input
   if (handle < 0 || handle > eape->status.num_ctx || eape->ctx[handle].allocated == 0) {
      return -1;
   }
   //Determine register offset between input and output register set
   if (direction == EAPE_OUTBOUND) {
      offset = 0x00;
      swid   = 0;

      //Check if FIFO full
      if (EAPE_CHECK_REG_BIT(pdu_io_read32(eape->regmap + EAPE_REG_FIFO_STAT), EAPE_FIFO_STAT_OUT_CMD_FULL_OFFSET)) {
         EAPE_STATS_INC(eape->stats.dropped_out);
         return EAPE_FIFO_FULL;
      }
      EAPE_STATS_INC(eape->stats.pass_out);
   } else {
      offset = 0x20;
      swid   = 0x100;

      //Check if FIFO full
      if (EAPE_CHECK_REG_BIT(pdu_io_read32(eape->regmap + EAPE_REG_FIFO_STAT), EAPE_FIFO_STAT_IN_CMD_FULL_OFFSET)) {
         EAPE_STATS_INC(eape->stats.dropped_in);
         return EAPE_FIFO_FULL;
      }
      EAPE_STATS_INC(eape->stats.pass_in);
   }

   /* we're running the job now */
   // find a slot for the job
   for (j = 0; j < MAXJOBS; j++) {
      if (eape->ctx[handle].done[j] == EAPE_JOB_DONE) {
         break;
      }
   }
   if (j == MAXJOBS) {
      err = EAPE_JOBS_FULL;
      goto ERR;
   }
   // have slot, allocate handle if needed
   eape->ctx[handle].job_cnt++;
   eape->ctx[handle].done[j]   = EAPE_JOB_IN_PROGRESS;
   eape->ctx[handle].cb[j]     = cb;
   eape->ctx[handle].cbdata[j] = cb_data;

   // write out engine
   pdu_io_cached_write32(eape->regmap + offset + EAPE_REG_OUT_SRC_PTR, (uint32_t) src->phys, &(eape->cache[direction].src_ptr));
   pdu_io_cached_write32(eape->regmap + offset + EAPE_REG_OUT_DST_PTR, (uint32_t) dst->phys, &(eape->cache[direction].dst_ptr));
   pdu_io_cached_write32(eape->regmap + offset + EAPE_REG_OUT_OFFSET, 0, &(eape->cache[direction].offset));

   //Set the software tag packet ID
   swid |= pdu_io_read32(eape->regmap + offset + EAPE_REG_OUT_ID);
   err   = eape->ctx[handle].swid[j] = swid;
   eape->swid[swid] = (handle << 16) | (j);

   //Push the command onto the fifo
   pdu_io_write32(eape->regmap + offset + EAPE_REG_OUT_SAI, eape->sa.sa_ptr_phys + EAPE_SA_SIZE * handle);

#ifdef EAPE_DEBUG
{
   unsigned long testval;
   testval = pdu_io_read32(eape->regmap + EAPE_REG_IRQ_EN);
   printk("EAPE_REG_IRQ_EN = %08lx\n", testval);
   testval = pdu_io_read32(eape->regmap + EAPE_REG_IRQ_STAT);
   printk("EAPE_REG_IRQ_STAT = %08lx\n", testval);
   testval = pdu_io_read32(eape->regmap + EAPE_REG_IRQ_CTRL);
   printk("EAPE_REG_IRQ_CTRL = %08lx\n", testval);
   testval = pdu_io_read32(eape->regmap + EAPE_REG_FIFO_STAT);
   printk("EAPE_REG_FIFO_STAT = %08lx\n", testval);
   testval = pdu_io_read32(eape->regmap + EAPE_REG_STAT_WD_CTL);
   printk("EAPE_REG_STAT_WD_CTL = %08lx\n", testval);
   testval = pdu_io_read32(eape->regmap + EAPE_REG_OUT_SRC_PTR);
   printk("EAPE_REG_OUT_SRC_PTR = %08lx\n", testval);
   testval = pdu_io_read32(eape->regmap + EAPE_REG_OUT_DST_PTR);
   printk("EAPE_REG_OUT_DST_PTR = %08lx\n", testval);
   testval = pdu_io_read32(eape->regmap + EAPE_REG_OUT_STAT);
   printk("EAPE_REG_OUT_STAT = %08lx\n", testval);
   testval = pdu_io_read32(eape->regmap + EAPE_REG_OUT_SAI);
   printk("EAPE_REG_OUT_SAI = %08lx\n", testval);
   testval = pdu_io_read32(eape->regmap + EAPE_REG_IN_SRC_PTR);
   printk("EAPE_REG_IN_SRC_PTR = %08lx\n", testval);
   testval = pdu_io_read32(eape->regmap + EAPE_REG_IN_DST_PTR);
   printk("EAPE_REG_IN_DST_PTR = %08lx\n", testval);
   testval = pdu_io_read32(eape->regmap + EAPE_REG_IN_STAT);
   printk("EAPE_REG_IN_STAT = %08lx\n", testval);
   testval = pdu_io_read32(eape->regmap + EAPE_REG_IN_SAI);
   printk("EAPE_REG_IN_SAI = %08lx\n", testval);
   testp = eape->sa.sa_ptr_virt + handle * EAPE_SA_SIZE;
   printk("The SA is %08lx \n", (unsigned long)*testp);
}
#endif
 ERR:
   return err;
}


int eape_go(eape_device * eape, int handle, int direction, pdu_ddt * src, pdu_ddt * dst, elpxfrm_callback cb, void *cb_data)
{
   int err;
   unsigned long flag;

   PDU_LOCK(&eape->lock, flag);
   err = eape_go_ex(eape, handle, direction, src, dst, cb, cb_data);
   PDU_UNLOCK(&eape->lock, flag);
   return err;
}
