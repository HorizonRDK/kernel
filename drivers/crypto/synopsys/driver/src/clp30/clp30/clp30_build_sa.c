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

static void store32(uint32_t v, unsigned char *dst)
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


int clp30_build_sa(clp30_device *clp30, int handle, elpxfrm_sa *sa)
{
   unsigned char *buf;
   uint32_t x, *p;

   if (handle < 0 || handle > clp30->num_ctx || clp30->ctx[handle].allocated == 0) {
      ELPHW_PRINT("clp30_build_sa::Invalid handle\n");
      return -1;
   }

   buf = clp30->sa_ptr_virt + handle * SA_SIZE;

   memset(buf, 0, SA_SIZE);
   store32(sa->seqnum[1], buf+CLP30_SA_SEQNUM);
   store32(sa->seqnum[0], buf+CLP30_SA_SEQNUM+4);
   memcpy(buf+CLP30_SA_CIPH_KEY,  sa->ckey,      32);
   memcpy(buf+CLP30_SA_SALT,      sa->csalt,      4);
   memcpy(buf+CLP30_SA_AUTH_KEY1, sa->mackey,    20);
#ifndef CLP36_ENABLED
   memcpy(buf+CLP30_SA_AUTH_KEY2, sa->mackey+20, 12);
#else
   memcpy(buf+CLP36_SA_AUTH_KEY2, sa->mackey+20, 12);
#endif
   store32(sa->spi, buf+CLP30_SA_REMOTE_SPI);
   store32(sa->hard_ttl[1], buf+CLP30_SA_HARD_TTL_HI);
   store32(sa->hard_ttl[0], buf+CLP30_SA_SOFT_TTL_LO);
   store32(sa->soft_ttl[1], buf+CLP30_SA_HARD_TTL_HI);
   store32(sa->soft_ttl[0], buf+CLP30_SA_SOFT_TTL_LO);
   buf[CLP30_SA_ALG]     = sa->alg;
   sa->ctrl             |= SA_CTRL_ACTIVE;
   buf[CLP30_SA_FLAGS]   = (sa->ctrl>>8)&0xFF;
   buf[CLP30_SA_FLAGS+1] = sa->ctrl&0xFF;

   p = (uint32_t*)buf;
   for (x = 0; x < (SA_SIZE/4); x++) {
      p[x] = htonl(p[x]);
   }

   return 0;
}

