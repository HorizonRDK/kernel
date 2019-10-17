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

#include "elpmpm.h"

#undef ELPHW_PRINT
#define ELPHW_PRINT printk

// handy 32x32 scaled division without dividing... 
static uint32_t scale(uint32_t nom, uint32_t denom)
{
   uint64_t a, b, c, res;
  
   if (!denom) return 0;
      
   // 64-bit multiply by 10000
#if 1
   a   = (uint64_t)nom * 10000;
#else
   // with just shifts in case your platform doesn't support 64x32 mults 
   a   = nom;
   a   = (a<<13) + (a<<10) + (a<<9) + (a<<8) + (a<<4);
#endif   
   b   = denom;
   c   = 1;
   res = 0;
   
   while (b < a) { b <<= 1; c <<= 1; }
   
   while (a >= denom) {
      while (a >= b) { a -= b; res += c; }
      b >>= 1;
      c >>= 1;
   }
   return res;
}

char *mpm_stats(mpm_device *mpm)
{
   uint32_t tally, x, y;
   char *buf, *p;
   
   mpm_cleanup(mpm);
   
   buf = pdu_malloc(65536);
   if (!buf) {
      return NULL;
   }
   memset(buf, 0, 65536);
   p = buf;
   
   p += sprintf(p, "MPM: Performance Monitoring Stats\n");
   p += sprintf(p, "MPM: EOL Interrupts        : %10lu\n", (unsigned long)mpm->perf.eol_irqs);
   p += sprintf(p, "MPM: DEMAND Interrupts     : %10lu\n", (unsigned long)mpm->perf.demand_irqs);
   p += sprintf(p, "MPM: Jobs Inserted         : %10lu\n", (unsigned long)mpm->perf.job_inserted);
   p += sprintf(p, "MPM: Jobs Cleared          : %10lu\n", (unsigned long)mpm->perf.job_cleared);
   p += sprintf(p, "\nMPM: KEY Full Cache Hit    : %10lu\n", (unsigned long)mpm->perf.key_full_hit);
   p += sprintf(p, "MPM: KEY Part Cache Hit    : %10lu\n", (unsigned long)mpm->perf.key_partial_hit);
   p += sprintf(p, "MPM: KEY Miss              : %10lu\n", (unsigned long)mpm->perf.key_miss);
   p += sprintf(p, "\nMPM: DEMAND jobs hit       : %10lu\n", (unsigned long)mpm->perf.demand_caught);
   p += sprintf(p, "MPM: DEMAND downgrade      : %10lu (job posted as non-demand)\n", (unsigned long)mpm->perf.demand_downgrade);
   p += sprintf(p, "MPM: DEMAND missed         : %10lu (normal EOL IRQ picked it up)\n", (unsigned long)mpm->perf.demand_missed);
   tally = mpm->perf.demand_downgrade + mpm->perf.demand_missed + mpm->perf.demand_caught;
   if (tally) {
      tally = scale(mpm->perf.demand_caught, tally);
      p += sprintf(p, "MPM: DEMAND hitrate        : %7lu.%02lu %%\n", (unsigned long)tally/100, (unsigned long)tally%100);
   }
   p += sprintf(p, "\nMPM: active_total          : %10llu\n", (unsigned long long)mpm->perf.active_counter);
   p += sprintf(p, "MPM: spacc_total           : %10llu\n", (unsigned long long)mpm->perf.spacc_counter);
   p += sprintf(p, "MPM: idle_total            : %10llu\n", (unsigned long long)mpm->perf.idle_counter);
   p += sprintf(p, "MPM: IRQ latency           : %10llu\n", (unsigned long long)mpm->perf.irq_counter);
   tally = mpm->perf.active_counter + mpm->perf.idle_counter;
   if (tally) {
      tally = scale(mpm->perf.active_counter, tally);
      p += sprintf(p, "MPM: active rate           : %5lu.%02lu %%\n", (unsigned long)tally/100, (unsigned long)tally%100);
   }
   tally = mpm->perf.active_counter;
   if (tally) {
      tally = scale(mpm->perf.spacc_counter, tally);
      p += sprintf(p, "MPM: spacc rate            : %5lu.%02lu %%\n", (unsigned long)tally/100, (unsigned long)tally%100);
   }
   tally = mpm->perf.idle_counter;
   if (tally) {
      tally = scale(mpm->perf.irq_counter, tally);
      p += sprintf(p, "MPM: IRQ rate              : %5lu.%02lu %%\n", (unsigned long)tally/100, (unsigned long)tally%100);
   }
   if (mpm->perf.eol_irqs) {
      p += sprintf(p, "MPM: Active Cycles per EOL : %10lu (average)\n", (unsigned long)scale(mpm->perf.active_counter, mpm->perf.eol_irqs)/10000);
   }
   
#if 1
   for (x = 0; x < mpm->config.no_chains; x++) {
      if (mpm->chains[x].curidx) {
         p += sprintf(p, "MPM: chain[%d].state  == %d\n", x, mpm->chains[x].state);
         p += sprintf(p, "MPM: chain[%d].curidx == %d\n", x, mpm->chains[x].curidx);
         for (y = 0; y < mpm->chains[x].curidx; y++) {
            pdubuf *pdu = &mpm->pdu.pdus[mpm->chains[x].pdus[y]];
            p += sprintf(p, "MPM: chain[%d].job[%d].status == %08lx, (this@ %08lx => next@ %08lx, key@ %08lx) \n", x, y, (unsigned long)pdu[0][PDU_DESC_STATUS], (unsigned long)mpm->pdu.pdus_phys + mpm->chains[x].pdus[y] * 0x80, (unsigned long)pdu[0][PDU_DESC_NEXT_PDU], (unsigned long)pdu[0][PDU_DESC_KEY_PTR]);
         }
      }
   }
   p += sprintf(p, "MPM: chain_idx        == %d\n", mpm->chain_idx);
   p += sprintf(p, "MPM: active_chain_idx == %d\n", mpm->active_chain_idx);
   p += sprintf(p, "MPM: busy             == %d\n", mpm->busy);
   p += sprintf(p, "MPM: work_to_clear    == %d\n", mpm->work_to_clear);
   p += sprintf(p, "MPM: pdu/key stack pointers %d/%d\n", mpm->pdu.freepdu_idx, mpm->key.freekey_idx);
#endif  
   return buf;
}
