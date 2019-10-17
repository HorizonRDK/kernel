
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


#ifndef ELP_KEP_H
#define ELP_KEP_H

#include "elppdu.h"
#include "elpspacc.h"
#include "elpkephw.h"

#define KEP_MAX_DDT 17

enum {
   KEP_SSL3_KEYGEN=0,
   KEP_SSL3_SIGN,
   KEP_TLS_PRF,
   KEP_TLS_SIGN,
   KEP_TLS2_PRF,
   KEP_TLS2_SIGN
};   

// options for commands
#define KEP_OPT_SSL3_SIGN_MD5   0
#define KEP_OPT_SSL3_SIGN_SHA1  1

#define KEP_OPT_TLS_SIGN_CLIENT 0
#define KEP_OPT_TLS_SIGN_SERVER 1

#define KEP_OPT_TLS2_PRF_MD5    (1UL<<1)
#define KEP_OPT_TLS2_PRF_SHA1   (2UL<<1)
#define KEP_OPT_TLS2_PRF_SHA224 (3UL<<1)
#define KEP_OPT_TLS2_PRF_SHA256 (4UL<<1)
#define KEP_OPT_TLS2_PRF_SHA384 (5UL<<1)
#define KEP_OPT_TLS2_PRF_SHA512 (6UL<<1)

#define KEP_OPT_TLS2_SIGN_CLIENT 0
#define KEP_OPT_TLS2_SIGN_SERVER 1
#define KEP_OPT_TLS2_SIGN_MD5    (1UL<<1)
#define KEP_OPT_TLS2_SIGN_SHA1   (2UL<<1)
#define KEP_OPT_TLS2_SIGN_SHA224 (3UL<<1)
#define KEP_OPT_TLS2_SIGN_SHA256 (4UL<<1)
#define KEP_OPT_TLS2_SIGN_SHA384 (5UL<<1)
#define KEP_OPT_TLS2_SIGN_SHA512 (6UL<<1)

typedef void (*kep_callback)(void *kep_dev, void *data);

typedef struct {
   int op,
       option,
       job_id,
       job_done;
       
   kep_callback cb;
   void        *cbdata;       
} elp_kep_ctx;

typedef struct {
   void        *regmap;
   elp_kep_ctx *ctx;   
   PDU_LOCK_TYPE lock;
   spacc_device *spacc;
} kep_device;

int kep_init(void *baseaddr, spacc_device *spacc, kep_device *kep);
void kep_fini(kep_device *kep);

int kep_open(kep_device *kep, int op, int option, kep_callback cb, void *cbdata);
int kep_load_keys(kep_device *kep, int handle, void *s1, uint32_t s1len, void *s2, uint32_t s2len);
int kep_go(kep_device *kep, pdu_ddt *src_ddt, pdu_ddt *dst_ddt, int handle);
int kep_done(kep_device *kep, int handle);
int kep_close(kep_device *kep, int handle);

int kep_is_valid(kep_device *kep, int handle);

#ifdef PDU_USE_KERNEL
kep_device *kep_get_device(void);
#endif

#endif
