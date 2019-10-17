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

#ifndef _ELPCLP30_H_
#define _ELPCLP30_H_

#include "elppdu.h"
#include "clp30hw.h"
#include "xfrm.h"

// max jobs per context
#define MAXJOBS 64

typedef void (*clp30_callback)(void *clp30_dev, void *data, uint32_t payload_len, uint32_t retcode, uint32_t swid);


typedef struct {
   int dismiss,
       job_cnt,
       allocated,
       swid[MAXJOBS],
       done[MAXJOBS];

   clp30_callback cb[MAXJOBS];
   void       *cbdata[MAXJOBS];
} clp30_ctx;

typedef struct {
  void *regmap;

  clp30_ctx *ctx;
  int     num_ctx,
          fifo_depth,
          stat_cnt;

  void           *sa_ptr_virt;
  PDU_DMA_ADDR_T  sa_ptr_phys;

  PDU_LOCK_TYPE   lock;

  uint32_t           swid[512];

  size_t sa_ptr_mem_req;

  struct {
     uint32_t src_ptr,
              dst_ptr,
              offset;
  } cache[2];

} clp30_device;

int clp30_init(clp30_device *clp30, void *regmap, int no_ctx);
int clp30_deinit(clp30_device *clp30);

int clp30_open(clp30_device *clp30);
int clp30_build_sa(clp30_device *clp30, int handle, elpxfrm_sa *sa);
int clp30_go(clp30_device *clp30, int handle, int direction, pdu_ddt *src, pdu_ddt *dst, clp30_callback cb, void *cb_data);
int clp30_close(clp30_device *clp30, int handle);
int clp30_done(clp30_device *clp30, int handle, int swid);

int clp30_pop_packet(clp30_device *clp30);

// kernel
clp30_device *clp30_get_device(void);

#endif
