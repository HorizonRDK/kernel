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
 * Copyright (c) 2011-2015 Synopsys, Inc. and/or its affiliates.
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

/* Read the IRQ status register and process as needed */

uint32_t spacc_process_irq(spacc_device *spacc)
{
   uint32_t temp;
   int x, cmd_max;
   unsigned long lock_flag;

   PDU_LOCK(&spacc->lock, lock_flag);

   temp = pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_STAT);

   /* clear interrupt pin and run registered callback */
   if (temp & SPACC_IRQ_STAT_STAT) {
      SPACC_IRQ_STAT_CLEAR_STAT(spacc);
      if (spacc->op_mode == SPACC_OP_MODE_IRQ) {
         spacc->config.fifo_cnt <<= 2;
         if (spacc->config.fifo_cnt >= spacc->config.stat_fifo_depth) {
            spacc->config.fifo_cnt = spacc->config.stat_fifo_depth;
         } 
         spacc_irq_stat_enable(spacc, spacc->config.fifo_cnt); // update fifo count to allow more stati to pile up
         spacc_irq_cmdx_enable(spacc, 0, 0);            // reenable CMD0 empty interrupt
      } else if (spacc->op_mode == SPACC_OP_MODE_WD) {
      }
      if (spacc->irq_cb_stat != NULL){
         spacc->irq_cb_stat(spacc);
      }
   }

   /* Watchdog IRQ */
   if (spacc->op_mode == SPACC_OP_MODE_WD) {
      if (temp & SPACC_IRQ_STAT_STAT_WD) {
         if (++(spacc->wdcnt) == SPACC_WD_LIMIT) {
            ELPHW_PRINT("spacc_process_irq::Hit SPACC WD LIMIT aborting WD IRQs (%08lx) (this happens when you get too many IRQs that go unanswered)\n",
                        (unsigned long)temp);
            ELPHW_PRINT("spacc_process_irq::Current IRQ_EN settings 0x%08lx\n",
                        (unsigned long)pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN));
            spacc_irq_stat_wd_disable(spacc);
            spacc_irq_stat_enable(spacc, 1); // we set the STAT CNT to 1 so that every job generates an IRQ now
            spacc->op_mode = SPACC_OP_MODE_IRQ;
            ELPHW_PRINT("spacc_process_irq::New IRQ_EN settings 0x%08lx\n",
                        (unsigned long)pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_EN));
         } else {
            // if the timer isn't too high lets bump it up a bit so as to give the IRQ a chance to reply
            if (spacc->config.wd_timer < (0xFFFFFFUL >> 4)) {
               spacc_set_wd_count(spacc, spacc->config.wd_timer << 4);
            }
         }

         SPACC_IRQ_STAT_CLEAR_STAT_WD(spacc);
         if (spacc->irq_cb_stat_wd != NULL) {
            spacc->irq_cb_stat_wd(spacc);
         }
      }
   }


   if (temp & SPACC_IRQ_STAT_RC4_DMA) {
      SPACC_IRQ_STAT_CLEAR_RC4_DMA(spacc);
      if (spacc->irq_cb_rc4_dma != NULL){
         spacc->irq_cb_rc4_dma(spacc);
      }
   }


   if (spacc->op_mode == SPACC_OP_MODE_IRQ && !spacc->config.is_hsm_shared) {
      cmd_max = (spacc->config.is_qos ? SPACC_CMDX_MAX_QOS : SPACC_CMDX_MAX);
      for (x = 0; x < cmd_max; x++){
         if (temp & SPACC_IRQ_STAT_CMDX(x)) {
             spacc->config.fifo_cnt = 1;
             spacc_irq_cmdx_disable(spacc, x); // disable CMD0 interrupt since STAT=1
             spacc_irq_stat_enable (spacc, spacc->config.fifo_cnt); // reset STAT count to 1

            SPACC_IRQ_STAT_CLEAR_CMDX(spacc, x);
            /* run registered callback */
            if (spacc->irq_cb_cmdx != NULL){
               spacc->irq_cb_cmdx(spacc, x);
            }
         }
      }
   }


   PDU_UNLOCK(&spacc->lock, lock_flag);

   return temp;
}

