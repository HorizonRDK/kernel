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

#define KEP_MAX_SIZE  (1024UL)

#ifndef KERNEL

#include <stdint.h>

// opcodes
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

int kep_ssl3_keygen(const unsigned char *master_secret,
                    const unsigned char *server_secret,
                    const unsigned char *client_secret,
                          unsigned char *key, uint32_t keylen);
                          
int kep_ssl3_sign(const unsigned char *sign_data, uint32_t sign_len, uint32_t options,
                  const unsigned char *master_secret,
                        unsigned char *dgst, uint32_t dgst_len);
                        
int kep_tls_prf(const unsigned char *label, uint32_t label_len, uint32_t options,
                const unsigned char *master_secret,
                const unsigned char *server_secret, uint32_t server_len,
                      unsigned char *prf, uint32_t prf_len);

int kep_tls_sign(const unsigned char *sign_data, uint32_t sign_len, uint32_t options,
                 const unsigned char *master_secret,
                       unsigned char *dgst, uint32_t dgst_len);


int kep_tls2_prf(const unsigned char *label, uint32_t label_len, uint32_t options,
                 const unsigned char *master_secret, uint32_t master_len,
                 const unsigned char *server_secret, uint32_t server_len,
                       unsigned char *prf, uint32_t prf_len);

int kep_tls2_sign(const unsigned char *sign_data, uint32_t sign_len, uint32_t options,
                  const unsigned char *master_secret, uint32_t master_len,
                        unsigned char *dgst, uint32_t dgst_len);

#endif
