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


#include "elpre.h"
#include "elprehw.h"


/*
 * Start the record engine, must give the type of operation(0-4) and the length of the data
 */
int re_start_operation_ex (re_device * re, int handle, pdu_ddt * src_ddt, pdu_ddt * dst_ddt, uint32_t srcoff, uint32_t dstoff, uint32_t paylen, uint32_t type)
{
   //Make sure the the context is valid, and if it is have ctx point to it
   uint32_t temp, swid;
   elp_re_ctx *ctx;
   unsigned long lock_flag;

   if (handle < 0 || handle >= re->config.num_ctx) {
      return CRYPTO_INVALID_CONTEXT;
   }
   ctx = &re->re_contexts[handle];

   PDU_LOCK(&re->lock, lock_flag);
   if (RE_FIFO_CMD_FULL (pdu_io_read32 (re->regmap + RE_FIFO_STAT))) {
      PDU_UNLOCK(&re->lock, lock_flag);
      return CRYPTO_FIFO_FULL;
   }

   temp = 0x00;
   temp |= type;
   temp |= (ctx->spacc_handle << 16) & 0xFF0000;
   temp |= RE_LOAD_STORE_SA;

   pdu_io_cached_write32 (re->regmap + RE_SA_PTR,  (uint32_t)ctx->samap,    &re->cache.sa_ptr);
   pdu_io_cached_write32 (re->regmap + RE_SRC_PTR, (uint32_t)src_ddt->phys, &re->cache.src_ptr);
   pdu_io_cached_write32 (re->regmap + RE_DST_PTR, (uint32_t)dst_ddt->phys, &re->cache.dst_ptr);
   swid = pdu_io_read32 (re->regmap + RE_SW_CTRL) & 0xFF;
   re->jobid_to_ctx[swid] = handle;
   ctx->curjob_swid = swid;
   pdu_io_cached_write32 (re->regmap + RE_OFFSET, (srcoff<<0)|(dstoff<<16), &re->cache.offset);
   pdu_io_cached_write32 (re->regmap + RE_LEN, paylen & 0x0000FFFF,         &re->cache.len);
   pdu_io_write32 (re->regmap + RE_CTRL, temp);
   PDU_UNLOCK(&re->lock, lock_flag);

   return swid;
}

int re_start_operation (re_device * re, int handle, pdu_ddt * src_ddt, pdu_ddt * dst_ddt, uint32_t type)
{
   return re_start_operation_ex(re, handle, src_ddt, dst_ddt, 0, 0, src_ddt->len, type);
}
