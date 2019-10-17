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

static void store_word(unsigned char *dst, uint32_t v)
{
#if 0
   *dst++ = (v >> 24) & 0xFF;
   *dst++ = (v >> 16) & 0xFF;
   *dst++ = (v >>  8) & 0xFF;
   *dst++ = (v >>  0) & 0xFF;
#else
   *dst++ = (v >>  0) & 0xFF;
   *dst++ = (v >>  8) & 0xFF;
   *dst++ = (v >> 16) & 0xFF;
   *dst++ = (v >> 24) & 0xFF;
#endif
}

int ea_build_sa(ea_device *ea, int handle, elpxfrm_sa *sa)
{
   unsigned char *sp;

   if (handle < 0 || handle > ea->num_ctx || ea->ctx[handle].allocated == 0) {
      ELPHW_PRINT("ea_build_sa::Invalid handle\n");
      return -1;
   }

   sp = ea->sa_ptr_virt + handle * SA_SIZE;

   store_word(sp+SA_CTRL, sa->ctrl);
   store_word(sp+SA_SPI,  htonl(sa->spi));
   store_word(sp+SA_SEQ_NUM+0,  sa->seqnum[1]);
   store_word(sp+SA_SEQ_NUM+4,  sa->seqnum[0]);
   store_word(sp+SA_HARD_TTL+0, sa->hard_ttl[0]);
   store_word(sp+SA_HARD_TTL+4, sa->hard_ttl[1]);
   store_word(sp+SA_SOFT_TTL+0, sa->soft_ttl[0]);
   store_word(sp+SA_SOFT_TTL+4, sa->soft_ttl[1]);
   memcpy(sp+SA_AR_MASK,     sa->armask, 32);
   memcpy(sp+SA_CIPHER_KEY,  sa->ckey,   32);
   memcpy(sp+SA_CIPHER_SALT, sa->csalt,   4);
   memcpy(sp+SA_MAC_KEY,     sa->mackey, 64);

#if 0
   int x, y;
   for (x = 0; x < SA_SIZE; ) {
      printk("SA %04x: ", x);
      for (y = 0; y < 4; y++) {
         printk("%02X", sp[x+y]);
      }
      printk("\n");
      x += 4;
   }
#endif


   return 0;
}
