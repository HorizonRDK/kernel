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

#include "elppdu.h"
#include "elpclp850_hw.h"

#define CLP850_RETRY_MAX 5000000UL

typedef struct {
   uint32_t *base;   
   
   struct {
      struct { 
         unsigned diag_level_trng3,
                  diag_level_st_hlt,
                  secure_rst_state;
      } features;

      struct {
         unsigned stepping,
                  epn;
      } build_id;
   } config;
   
   struct {
      volatile unsigned 
         last_alarm,
         seeded,
         rnd_rdy,
         kat_done,
         zeroized;
   } status;

   PDU_LOCK_TYPE lock;
} elpclp850_state;

#define clp850_zero_status(x) memset(&(x->status), 0, sizeof (x->status))

/* functions users will call */
int elpclp850_init(elpclp850_state *clp850, uint32_t *base);

int elpclp850_hw_read(elpclp850_state *clp850, uint32_t *out);
int elpclp850_nonce_read(elpclp850_state *clp850, uint32_t *seed, uint32_t *out);

int elpclp850_set_keylen(elpclp850_state *clp850, int aes256, int lock);
int elpclp850_set_secure(elpclp850_state *clp850, int secure, int lock);
int elpclp850_set_kat(elpclp850_state *clp850, int kat, int lock);
int elpclp850_set_wait_for_ht(elpclp850_state *clp850, int ht, int lock);
int elpclp850_set_nonce(elpclp850_state *clp850, int nonce, int lock);

int elpclp850_ia_write(elpclp850_state *clp850, uint32_t addr, uint32_t val);
int elpclp850_ia_read(elpclp850_state *clp850, uint32_t addr, uint32_t *val);

/* Functions we use to manage the device */
int elpclp850_handle_irq(elpclp850_state *clp850);
