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
 * Copyright (c) 2011-2015 Synopsys, Inc. and/or its affiliates.
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

#ifndef _ELPCLUE_H_
#define _ELPCLUE_H_

#define ENCRYPT          0
#define DECRYPT          1

#define CRYPTO_MODULE_CLUE_BN           0x008


//RSA maximum 4096 bit= 512 bytes
#define BN_RSA_MAX_RADIX_SIZE  512
#define BN_RSA_BASE_RADIX_SIZE BN_RSA_MAX_RADIX_SIZE /* XXX: Probably should kill one of these defines */

short clue_bn_modexp (int fd, unsigned char * a, unsigned char * e, unsigned char * m, unsigned char * c, unsigned short size, short precomp);
short clue_bn_modmult (int fd, unsigned char * a, unsigned char * b, unsigned char * m, unsigned char * c, unsigned short size, short precomp);
short clue_bn_moddiv (int fd, unsigned char * a, unsigned char * b, unsigned char * m, unsigned char * c, unsigned short size);
short clue_bn_modinv (int fd, unsigned char * b, unsigned char * m, unsigned char * c, unsigned short size);
short clue_bn_modadd (int fd, unsigned char * a, unsigned char * b, unsigned char * m, unsigned char * c, unsigned short size);
short clue_bn_modsub (int fd, unsigned char * a, unsigned char * b, unsigned char * m, unsigned char * c, unsigned short size);
short clue_bn_modred (int fd, unsigned char * a, unsigned char * m, unsigned char * c, unsigned short size);

int clue_crt_modexp(int fd, const unsigned char *p, const unsigned char *q, const unsigned char *d, const unsigned char *m, unsigned char *c, unsigned size);

void clue_ec_init (void);
int clue_page_size (int size);

short clue_ec_point_mult (int fd, unsigned char * k, unsigned char * x, unsigned char * y, unsigned char * rx, unsigned char * ry, unsigned short size, unsigned short ksize);
short clue_ec_point_mult_base (int fd, unsigned char * k, unsigned char * rx, unsigned char * ry, unsigned short size, unsigned short ksize);
short clue_ec_point_double (int fd, unsigned char * x, unsigned char * y, unsigned char * rx, unsigned char * ry, unsigned short size);
short clue_ec_point_add (int fd, unsigned char * x1, unsigned char * y1, unsigned char * x2, unsigned char * y2, unsigned char * rx, unsigned char * ry, unsigned short size);
short clue_ec_point_verify (int fd, unsigned char * x, unsigned char * y, unsigned short size);

short clue_ec_load_curve_data (int fd, unsigned short curve);
short clue_ec_load_curve_pmult (int fd, unsigned short curve, short loadbase);
short clue_ec_point_mult_shamir (int fd, unsigned char *k1, unsigned char *k2, unsigned char * rx, unsigned char * ry, unsigned char *sx, unsigned char *sy, unsigned short size, unsigned short ksize);
short clue_ec_load_curve_pdbl (int fd, unsigned short curve);
short clue_ec_load_curve_padd (int fd, unsigned short curve);
short clue_ec_load_curve_pver (int fd, unsigned short curve);

#endif
