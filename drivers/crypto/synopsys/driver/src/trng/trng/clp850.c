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

#include "elpclp850.h"

int elpclp850_init(elpclp850_state *clp850, uint32_t *base)
{
   uint32_t tmp;
   
   memset(clp850, 0, sizeof *clp850);
   
   clp850->base = base;
   
   /* read features */
   tmp = pdu_io_read32(clp850->base + CLP850_REG_FEATURES);
   clp850->config.features.diag_level_trng3  = CLP850_REG_FEATURES_DIAG_LEVEL_TRNG3(tmp);
   clp850->config.features.diag_level_st_hlt = CLP850_REG_FEATURES_DIAG_LEVEL_ST_HLT(tmp);
   clp850->config.features.secure_rst_state  = CLP850_REG_FEATURES_SECURE_RST_STATE(tmp);
   
   /* read build ID */
   tmp = pdu_io_read32(clp850->base + CLP850_REG_BUILD_ID);
   clp850->config.build_id.stepping = CLP850_REG_BUILD_ID_STEPPING(tmp);
   clp850->config.build_id.epn      = CLP850_REG_BUILD_ID_EPN(tmp);
   
   // display status
   ELPHW_PRINT("CLP850 (epn.%04x v.%02x)\n", clp850->config.build_id.epn, clp850->config.build_id.stepping);
   
   // default mode
   tmp = 0;
   tmp = CLP850_REG_MODE_SET_MAX_REJECTS(tmp, 10);
   tmp = CLP850_REG_MODE_SET_SECURE_EN(tmp, 1);
   pdu_io_write32(clp850->base + CLP850_REG_MODE, tmp);
   
   // TODO: enable IRQs
   
   /* init lock */
   PDU_INIT_LOCK(&clp850->lock);
   
   return 0;
}

int elpclp850_set_keylen(elpclp850_state *clp850, int aes256, int lock)
{
   unsigned long flags;
   uint32_t tmp;

   if (lock) { PDU_LOCK(&clp850->lock, flags); }

   /* enable NONCE mode */
   tmp = pdu_io_read32(clp850->base + CLP850_REG_MODE);
   tmp = CLP850_REG_MODE_SET_SEC_ALG(tmp, aes256);
   pdu_io_write32(clp850->base + CLP850_REG_MODE, tmp);

   if (lock) { PDU_UNLOCK(&clp850->lock, flags); }
   
   return 0;
}

int elpclp850_set_nonce(elpclp850_state *clp850, int nonce, int lock)
{
   unsigned long flags;
   uint32_t tmp;

   if (lock) { PDU_LOCK(&clp850->lock, flags); }

   /* enable NONCE mode */
   tmp = pdu_io_read32(clp850->base + CLP850_REG_MODE);
   tmp = CLP850_REG_MODE_SET_NONCE(tmp, nonce);
   pdu_io_write32(clp850->base + CLP850_REG_MODE, tmp);

   if (lock) { PDU_UNLOCK(&clp850->lock, flags); }
   
   return 0;
}

int elpclp850_set_secure(elpclp850_state *clp850, int secure, int lock)
{
   unsigned long flags;
   uint32_t tmp;

   if (lock) { PDU_LOCK(&clp850->lock, flags); }

   /* enable NONCE mode */
   tmp = pdu_io_read32(clp850->base + CLP850_REG_MODE);
   tmp = CLP850_REG_MODE_SET_SECURE_EN(tmp, secure);
   pdu_io_write32(clp850->base + CLP850_REG_MODE, tmp);

   if (lock) { PDU_UNLOCK(&clp850->lock, flags); }
   
   return 0;
}

int elpclp850_set_kat(elpclp850_state *clp850, int kat, int lock)
{
   unsigned long flags;
   uint32_t tmp;

   if (lock) { PDU_LOCK(&clp850->lock, flags); }

   /* enable NONCE mode */
   tmp = pdu_io_read32(clp850->base + CLP850_REG_MODE);
   tmp = CLP850_REG_MODE_SET_KAT(tmp, kat);
   pdu_io_write32(clp850->base + CLP850_REG_MODE, tmp);

   if (lock) { PDU_UNLOCK(&clp850->lock, flags); }
   
   return 0;
}

int elpclp850_set_wait_for_ht(elpclp850_state *clp850, int ht, int lock)
{
   unsigned long flags;
   uint32_t tmp;

   if (lock) { PDU_LOCK(&clp850->lock, flags); }

   /* enable NONCE mode */
   tmp = pdu_io_read32(clp850->base + CLP850_REG_MODE);
   tmp = CLP850_REG_MODE_SET_WAIT_FOR_HT(tmp, ht);
   pdu_io_write32(clp850->base + CLP850_REG_MODE, tmp);

   if (lock) { PDU_UNLOCK(&clp850->lock, flags); }
   
   return 0;
}

int elpclp850_hw_read(elpclp850_state *clp850, uint32_t *out)
{
   unsigned long flags;
   uint32_t x, tmp, retries;
   
   PDU_LOCK(&clp850->lock, flags);
   
   clp850_zero_status(clp850);
   
   /* wait for non-busy */
   retries = 0; do { tmp = pdu_io_read32(clp850->base + CLP850_REG_STAT); } while (++retries < CLP850_RETRY_MAX && CLP850_REG_STAT_BUSY(tmp));
   if (retries == CLP850_RETRY_MAX) {
      printk("hw_read: ERROR Timed out on BUSY flag\n");
      PDU_UNLOCK(&clp850->lock, flags);
      return -1;
   }
   
   /* disable NONCE mode */
   elpclp850_set_nonce(clp850, 0, 0);

   /* enable RNG mode */
   pdu_io_write32(clp850->base + CLP850_REG_CTRL, CLP850_REG_CTRL_SEED_NOISE);
   
   /* wait for rdy */
   retries = 0; do { elpclp850_handle_irq(clp850); } while (++retries < CLP850_RETRY_MAX && !clp850->status.rnd_rdy && clp850->status.last_alarm == 0);
   if (retries == CLP850_RETRY_MAX || clp850->status.last_alarm) {
      // TODO: HANDLE ALARMS
      printk("hw_read: ERROR Timed out on rnd_rdy or alarms flag (%u) (%u)\n", clp850->status.rnd_rdy, clp850->status.last_alarm);
      PDU_UNLOCK(&clp850->lock, flags);
      return -1;
   }
  
   /* read output */
   for (x = 0; x < 4; x++) {
      out[x] = pdu_io_read32(clp850->base + CLP850_REG_RAND0 + x);
   }
   PDU_UNLOCK(&clp850->lock, flags);
   return 0;
}

int elpclp850_nonce_read(elpclp850_state *clp850, uint32_t *seed, uint32_t *out)
{
   unsigned long flags;
   uint32_t tmp, retries, x;
   
   PDU_LOCK(&clp850->lock, flags);
   
   clp850_zero_status(clp850);

   /* wait for non-busy */
   retries = 0; do { tmp = pdu_io_read32(clp850->base + CLP850_REG_STAT); } while (++retries < CLP850_RETRY_MAX && CLP850_REG_STAT_BUSY(tmp));
   if (retries == CLP850_RETRY_MAX) {
      PDU_UNLOCK(&clp850->lock, flags);
      return -1;
   }
   
   /* enable NONCE mode */
   elpclp850_set_nonce(clp850, 1, 0);
   
   /* wait for NONCE to be enabled in STAT */
   retries = 0; do { tmp = pdu_io_read32(clp850->base + CLP850_REG_STAT); } while (++retries < CLP850_RETRY_MAX && !CLP850_REG_STAT_NONCE_MODE(tmp));
   if (retries == CLP850_RETRY_MAX) {
      PDU_UNLOCK(&clp850->lock, flags);
      return -1;
   }
   
   /* write seed */
   for (x = 0; x < 16; x++) {
      pdu_io_write32(clp850->base + CLP850_REG_SEED0 + x, seed[x]);
   }
   
   /* set NONCE reseed bit */
   pdu_io_write32(clp850->base + CLP850_REG_CTRL, CLP850_REG_CTRL_SEED_NONCE);
   
   /* wait for rdy */
   retries = 0; do { elpclp850_handle_irq(clp850); } while (++retries < CLP850_RETRY_MAX && !clp850->status.rnd_rdy);
   if (retries == CLP850_RETRY_MAX || clp850->status.last_alarm) {
      // TODO: HANDLE ALARMS
      PDU_UNLOCK(&clp850->lock, flags);
      return -1;
   }

   /* read output */
   for (x = 0; x < 4; x++) {
      out[x] = pdu_io_read32(clp850->base + CLP850_REG_RAND0 + x);
   }

   PDU_UNLOCK(&clp850->lock, flags);
   return 0;
}   

int elpclp850_ia_write(elpclp850_state *clp850, uint32_t addr, uint32_t val)
{
   unsigned long flags;
   uint32_t tmp, retries;
   
   PDU_LOCK(&clp850->lock, flags);
   
   pdu_io_write32(clp850->base + CLP850_REG_IA_ADDR, addr);
   pdu_io_write32(clp850->base + CLP850_REG_IA_WDATA, val);
   pdu_io_write32(clp850->base + CLP850_REG_IA_CMD, CLP850_REG_IA_CMD_GO|CLP850_REG_IA_CMD_W_nR);
   
   /* wait for GO to be cleared in IA_CMD */
   retries = 0; do { tmp = pdu_io_read32(clp850->base + CLP850_REG_IA_CMD); } while (++retries < CLP850_RETRY_MAX && (tmp & CLP850_REG_IA_CMD_GO));
   if (retries == CLP850_RETRY_MAX) {
      PDU_UNLOCK(&clp850->lock, flags);
      return -1;
   }
   
   PDU_UNLOCK(&clp850->lock, flags);
   return 0;
}

int elpclp850_ia_read(elpclp850_state *clp850, uint32_t addr, uint32_t *val)
{
   unsigned long flags;
   uint32_t tmp, retries;
   
   PDU_LOCK(&clp850->lock, flags);
   
   pdu_io_write32(clp850->base + CLP850_REG_IA_ADDR, addr);
   pdu_io_write32(clp850->base + CLP850_REG_IA_CMD, CLP850_REG_IA_CMD_GO);
   
   /* wait for GO to be cleared in IA_CMD */
   retries = 0; do { tmp = pdu_io_read32(clp850->base + CLP850_REG_IA_CMD); } while (++retries < CLP850_RETRY_MAX && (tmp & CLP850_REG_IA_CMD_GO));
   if (retries == CLP850_RETRY_MAX) {
      PDU_UNLOCK(&clp850->lock, flags);
      return -1;
   }
   
   *val = pdu_io_read32(clp850->base + CLP850_REG_IA_RDATA);
   PDU_UNLOCK(&clp850->lock, flags);
   return 0;
}

static int elpclp850_handle_alarms(elpclp850_state *clp850)
{
   return 0;
}

/* read istat register and respond accordingly */
int elpclp850_handle_irq(elpclp850_state *clp850)
{
   uint32_t tmp;
   
   tmp = pdu_io_read32(clp850->base + CLP850_REG_ISTAT);
   
   if (tmp & CLP850_REG_ISTAT_ALARMS) {
      clp850->status.last_alarm = pdu_io_read32(clp850->base + CLP850_REG_ALARM);
      elpclp850_handle_alarms(clp850);
   }
   if (tmp & CLP850_REG_ISTAT_SEEDED) {
      clp850->status.seeded = 1;
   }
   if (tmp & CLP850_REG_ISTAT_RND_RDY) {
      clp850->status.rnd_rdy = 1;
   }
   if (tmp & CLP850_REG_ISTAT_KAT_DONE) {
      clp850->status.kat_done = 1;
   }
   if (tmp & CLP850_REG_ISTAT_ZEROIZED) {
      clp850->status.zeroized = 1;
   }
   
   if (tmp) {
      pdu_io_write32(clp850->base + CLP850_REG_ISTAT, tmp);
   }

   return 0;
}
