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



#ifndef ELP_RE_H
#define ELP_RE_H

#include "elppdu.h"
#include "elpspacc.h"

#define MAX_RE_DDT_SIZE 18432
#define RE_SA_SIZE 1024
#define CRYPTO_MODULE_RE  0x0100

typedef void (*re_callback)(void *re_dev, void *data, uint32_t retcode, uint32_t datalen);

typedef struct elp_re_ctx_{

   unsigned char * sa;   //Pointer to the SA

   PDU_DMA_ADDR_T samap; //Physical address of the SA
   int spacc_handle; //The handle from the spacc that this context has

   int curjob_swid;
   int jobdone;
   int ret_stat;
   int ret_prot;
   int ret_size;
   int ret_err;

   int datalength;

   re_callback cb;
   void       *cbdata;

} elp_re_ctx;

typedef struct {
   void *regmap;

   int module_initialized;

   int jobid_to_ctx[SPACC_MAX_JOBS];

   struct {
      int num_ctx;
      unsigned
         fifo_depth,
         ssl3_en,
         tls10_en,
         tls11_en,
         tls12_en,
         dtls10_en,
         dtls12_en;
   } config;
   unsigned fifo_cnt;
   elp_re_ctx *re_contexts;

   void *sa_pool_virt;
   PDU_DMA_ADDR_T sa_pool_phys;
   unsigned long sa_pool_size;

   struct {
      uint32_t
         sa_ptr,
         src_ptr,
         dst_ptr,
         offset,
         len;
   } cache;

   PDU_LOCK_TYPE lock;
   spacc_device *spacc;
} re_device;

int re_init(void *baseaddr, spacc_device *spacc, re_device *re);
void re_fini(re_device *re);

int re_reset_sa(re_device *re, int handle, uint32_t version);

int re_set_next_read(re_device *re,
                     int handle,
                     unsigned char *iv,              uint32_t ivlen,
                     unsigned char *key,             uint32_t keylen,
                     unsigned char *mackey,          uint32_t mackeylen,
                     unsigned char *params,          uint32_t paramlength,
                     unsigned char *sequence_number, uint32_t seqlength);

int re_set_next_write(re_device *re,
                      int handle,
                      unsigned char *iv,              uint32_t ivlen,
                      unsigned char *key,             uint32_t keylen,
                      unsigned char *mackey,          uint32_t mackeylen,
                      unsigned char *params,          uint32_t paramlength,
                      unsigned char *sequence_number, uint32_t seqlength);

int re_start_operation_ex (re_device * re, int handle, pdu_ddt * src_ddt, pdu_ddt * dst_ddt, uint32_t srcoff, uint32_t dstoff, uint32_t paylen, uint32_t type);
int re_start_operation(re_device *re, int handle, pdu_ddt *src_ddt, pdu_ddt *dst_ddt, uint32_t type);
int re_finish_operation(re_device *re, int handle, uint32_t * length, int * id);
int re_error_msg (int err, unsigned char * buff, uint32_t length);

int re_retrieve_sa(re_device *re, int handle, unsigned char * sabuff, uint32_t bufflength);
int re_write_sa(re_device *re, int handle, unsigned char * sa, uint32_t bufflength);

int re_init_context_ex(re_device * re, int handle, int ctxid);
int re_init_context(re_device *re, int handle);
int re_get_context_ex (re_device * re, int ctxid, re_callback cb, void *cbdata);
int re_get_context(re_device *re, re_callback cb, void *cbdata);
int re_get_spacc_context(re_device *re, int re_ctx);
int re_release_context(re_device *re, int handle);

int re_set_next_read_iv (re_device *re, int handle, unsigned char * iv, uint32_t length);
int re_set_next_read_key (re_device *re, int handle, unsigned char * key, uint32_t length);
int re_set_next_read_mackey (re_device *re, int handle, unsigned char * mackey, uint32_t length);
int re_set_next_read_params (re_device *re, int handle, unsigned char * params, uint32_t length);
int re_set_next_read_sequence_number (re_device *re, int handle, unsigned char * sequence_number, uint32_t length);

int re_set_next_write_iv (re_device *re, int handle, unsigned char * iv, uint32_t length);
int re_set_next_write_key (re_device *re, int handle, unsigned char * key, uint32_t length);
int re_set_next_write_mackey (re_device *re, int handle, unsigned char * mackey, uint32_t length);
int re_set_next_write_params (re_device *re, int handle, unsigned char * params, uint32_t length);
int re_set_next_write_sequence_number (re_device *re, int handle, unsigned char * sequence_number, uint32_t length);

int re_packet_dequeue (re_device *re, int context);

void re_print_diag(re_device *re, int handle);

#ifdef PDU_USE_KERNEL
re_device *re_get_device(void);
#endif

#endif //ELP_RE_H
