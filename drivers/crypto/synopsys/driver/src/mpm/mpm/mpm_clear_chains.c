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

void mpm_sync_chain_for_cpu(mpm_device *mpm, int idx)
{
   int x;
   for (x = 0; x < mpm->chains[idx].curidx; x++) {
      pdu_sync_single_for_cpu(((uint32_t)mpm->pdu.pdus_phys) + mpm->chains[idx].pdus[x] * sizeof(pdubuf), sizeof(pdubuf));
   }
}
   
void mpm_sync_chain_for_device(mpm_device *mpm, int idx)
{
   int x;
   for (x = 0; x < mpm->chains[idx].curidx; x++) {
      pdu_sync_single_for_device(((uint32_t)mpm->pdu.pdus_phys) + mpm->chains[idx].pdus[x] * sizeof(pdubuf), sizeof(pdubuf));
   }
}


int mpm_clear_chains(mpm_device *mpm)
{
   int x, y, z;
  // try to run more jobs, avoid locking if the engine is already busy
  if (mpm->busy == 0) {
     /* hunt for the next active */
     // start next active job (could be active_chain_idx if started by enqueue or +1 if from IRQ)
     for (z = 0, x = mpm->active_chain_idx; z != mpm->config.no_chains; x++, z++) {
        if (x >= mpm->config.no_chains) { x = 0; }
        if (mpm->chains[x].state == CHAIN_BUILT) {
           // enqueue this chain 
           mpm->busy             = 1;
           mpm->chains[x].state  = CHAIN_RUNNING;
           mpm->active_chain_idx = x;
#ifdef MPM_TIMING_X86
           asm __volatile__ ("rdtsc\nsubl (%0),%%eax\nsbbl 4(%0),%%edx\nmovl %%eax,(%0)\nmovl %%edx,4(%0)\n"::"r" (&mpm->chains[x].slottime):"%eax", "%edx", "%cc");
           ELPHW_PRINT(ELPHW_PRINT_DEBUG"MPM:Chain took %08zx%08zx cycles from enqueue to running\n", mpm->chains[x].slottime[1], mpm->chains[x].slottime[0]);
#endif
#ifdef MPM_PERF_MON
           mpm->perf.idle_counter = pdu_io_read32(mpm->regmap + MPM_STAT_CNT_WAIT);
#endif  
//           mpm_sync_chain_for_device(mpm, x);
           pdu_io_write32(mpm->regmap + MPM_START, ((uint32_t)mpm->pdu.pdus_phys) + mpm->chains[x].pdus[0] * sizeof(pdubuf));
           break;
        }
     }
  }

   z = 0;
   for (x = 0; x < mpm->config.no_chains; x++) {
      if (mpm->chains[x].state == CHAIN_DONE) {
        mpm->chains[x].state = CHAIN_PROCESSING;
// subtract time spent per job from start time on x86 
#ifdef MPM_TIMING_X86
         for (y = 0; y < mpm->chains[x].curidx; y++) {
            pdubuf *pdu = &mpm->pdu.pdus[mpm->chains[x].pdus[y]];
            asm __volatile__ ("rdtsc\nsubl (%0),%%eax\nsbbl 4(%0),%%edx\nmovl %%eax,(%0)\nmovl %%edx,4(%0)\n"::"r" (&pdu[0][0x78/4]):"%eax", "%edx", "%cc");
         }
#endif
         for (y = 0; y < mpm->chains[x].curidx; y++) {
            int w;
            // did we miss an ON DEMAND job?
            w = mpm->pdu.ondemand_mask[mpm->chains[x].pdus[y]];
            if (w != -1) {
               pdubuf *pdu = &mpm->pdu.pdus[mpm->chains[x].pdus[y]];
               // if we get here it's because an ON DEMAND IRQ was missed so we have to clear the job
               mpm->pdu.ondemand_mask[mpm->chains[x].pdus[y]] = -1; 
               MPM_PERF_INCR(mpm->perf.demand_missed, 1);
               if (mpm->pdu.ondemand_cb[w] != NULL) {
                  mpm->pdu.ondemand_cb[w](mpm, mpm->pdu.ondemand_cb_data[w], mpm->pdu.ondemand[w], mpm->chains[x].keys[y], pdu[0][PDU_DESC_STATUS]);
               }
               mpm->pdu.ondemand_mask[mpm->pdu.ondemand[w]] = -1;
               mpm->pdu.ondemand_cb[w]                      = NULL;
               mpm->pdu.ondemand[w]                         = -1;
           }
         }

         if (mpm->chains[x].per_chain_callback == NULL) {
            for (y = 0; y < mpm->chains[x].curidx; y++) {
               pdubuf *pdu = &mpm->pdu.pdus[mpm->chains[x].pdus[y]];
               ++z;

#ifdef MPM_PERF_MON
               switch (PDU_DESC_STATUS_CACHE_CODE(pdu[0][PDU_DESC_STATUS])) {
                  case 3: MPM_PERF_INCR(mpm->perf.key_full_hit, 1);    break;
                  case 2:
                  case 1: MPM_PERF_INCR(mpm->perf.key_partial_hit, 1); break;
                  case 0: MPM_PERF_INCR(mpm->perf.key_miss, 1);        break;
               }
#endif
 
               if (mpm->chains[x].callbacks[y]) {
                  mpm->chains[x].callbacks[y](mpm, mpm->chains[x].callback_data[y], mpm->chains[x].pdus[y], mpm->chains[x].keys[y], pdu[0][PDU_DESC_STATUS]);
               }
            }
         } else {
            z += mpm->chains[x].curidx;
            mpm->chains[x].per_chain_callback(mpm, mpm->chains[x].per_chain_id);
         }
        mpm->work_to_clear   += 1;
     }
  }
  MPM_PERF_INCR(mpm->perf.job_cleared, z);

  return z;
}
