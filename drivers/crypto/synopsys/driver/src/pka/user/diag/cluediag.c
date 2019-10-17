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

#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>

#include <pkadev.h>
#include "common.h"
#include "elpcrypto.h"

enum {
   PKA_VECTOR_MODEXP_CRT,
   PKA_VECTOR_MODEXP,
   PKA_VECTOR_MODMUL,
   PKA_VECTOR_MODDIV,
   PKA_VECTOR_MODADD,
   PKA_VECTOR_MODSUB,
   PKA_VECTOR_MODINV,
   PKA_VECTOR_MODRED,
   PKA_VECTOR_PMULT,
   PKA_VECTOR_PADD,
   PKA_VECTOR_PVER,
   PKA_VECTOR_PDBL,
   PKA_VECTOR_PSIGN,
   PKA_VECTOR_PMULTBLIND,
   PKA_VECTOR_SHAMIR,
   PKA_VECTOR_PMULT_C25519,
   PKA_VECTOR_PMULTBLIND_C25519,
   PKA_VECTOR_PADD_ED25519,
   PKA_VECTOR_PADDBLIND_ED25519,
   PKA_VECTOR_PDBL_ED25519,
   PKA_VECTOR_PDBLBLIND_ED25519,
   PKA_VECTOR_PMULT_ED25519,
   PKA_VECTOR_PMULTBLIND_ED25519,
   PKA_VECTOR_SHAMIR_ED25519,
   PKA_VECTOR_SHAMIRBLIND_ED25519,
   PKA_VECTOR_PVER_ED25519,
   PKA_VECTOR_MODEXP_C25519
};

#define CURVE_MAX_DATA_SIZE 256

#define htonl(x) \
({ \
   uint32_t __x = (x); \
   ((uint32_t)( \
      (((uint32_t)(__x) & (uint32_t)0x000000ffUL) << 24) | \
      (((uint32_t)(__x) & (uint32_t)0x0000ff00UL) <<  8) | \
      (((uint32_t)(__x) & (uint32_t)0x00ff0000UL) >>  8) | \
      (((uint32_t)(__x) & (uint32_t)0xff000000UL) >> 24) )); \
})

int _test_invalid_case = 0;
int ecc_diag_verbose = 0;
int ecc_diag_silent = 0;
int fd = 0;

char op_name[][30] = {
   "MODEXP_CRT",
   "MODEXP",
   "MODMUL",
   "MODDIV",
   "MODADD",
   "MODSUB",
   "MODINV",
   "MODRED",
   "PMULT",
   "PADD",
   "PVER",
   "PDBL",
   "PSIGN",
   "PMULTBLIND",
   "SHAMIR",
   "PMULT_C25519",
   "PMULTBLIND_C25519",
   "PADD_ED25519",
   "PADDBLIND_ED25519",
   "PDBL_ED25519",
   "PDBLBLIND_ED25519",
   "PMULT_ED25519",
   "PMULTBLIND_ED25519",
   "SHAMIR_ED25519",
   "SHAMIRBLIND_ED25519",
   "PVER_ED25519",
   "MODEXP_C25519"
};

static const unsigned char m25519[32] = {
   0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xed
};

static const unsigned char d25519[32] = {
   0x52, 0x03, 0x6c, 0xee, 0x2b, 0x6f, 0xfe, 0x73, 0x8c, 0xc7, 0x40, 0x79,
   0x77, 0x79, 0xe8, 0x98, 0x00, 0x70, 0x0a, 0x4d, 0x41, 0x41, 0xd8, 0xab,
   0x75, 0xeb, 0x4d, 0xca, 0x13, 0x59, 0x78, 0xa3
};

/* Load C25519 constant data into PKA */
static int load_c25519(int fd)
{
   unsigned char tmp[32] = {0};
   int rc;

   /* Xdiff value */
   tmp[31] = 9;
   rc = elppka_set_operand(fd, NULL, "A4", sizeof tmp, tmp);
   if (rc < 0)
      goto err;

   /* Zdiff value */
   tmp[31] = 1;
   rc = elppka_set_operand(fd, NULL, "B4", sizeof tmp, tmp);
   if (rc < 0)
      goto err;

   /* k value */
   tmp[29] = 1; tmp[30] = 0xdb; tmp[31] = 0x42;
   rc = elppka_set_operand(fd, NULL, "D2", sizeof tmp, tmp);
   if (rc < 0)
      goto err;

   /* modulus */
   rc = elppka_set_operand(fd, NULL, "D0", sizeof m25519, m25519);
   if (rc < 0)
      goto err;

   return 0;
err:
   pka_tool_err(0, "%s failed", __func__);
   return rc;
}

/* Load ED25519 constant data into PKA */
static int load_ed25519(int fd)
{
   int rc;

   /* d value */
   rc = elppka_set_operand(fd, NULL, "C5", sizeof d25519, d25519);
   if (rc < 0)
      goto err;

   /* modulus */
   rc = elppka_set_operand(fd, NULL, "D0", sizeof m25519, m25519);
   if (rc < 0)
      goto err;

   return 0;
err:
   pka_tool_err(0, "%s failed", __func__);
   return rc;
}

static int setup_ecc_blinding(int fd, size_t size, void *blind)
{
   int rc;

   rc = elppka_set_operand(fd, NULL, "A7", size, blind);
   if (rc < 0)
      goto err;


   rc = elppka_set_flag(fd, NULL, "F0");
   if (rc < 0)
      goto err;

   return 0;
err:
   pka_tool_err(0, "%s failed", __func__);
   return rc;
}

/* get a line and skip blanks and comment lines */
static char *get_line(FILE * in)
{
   static char buf[BN_RSA_BASE_RADIX_SIZE];
   char *p;
   unsigned long L;
   do {
      p = fgets(buf, sizeof(buf) - 1, in);
   } while (p && sscanf(buf, "%lx", &L) != 1);
   return p;
}

static void read_words(FILE * in, unsigned char *dst, int nwords)
{
   char *p;
   unsigned long L;
   unsigned long *pdst = (unsigned long *)dst;
   while (nwords-- && (p = get_line(in))) {
      sscanf(p, "%08lx", &L);
      *pdst++ = htonl(L);
//      printf("%08lx\n", pdst[-1]);
   }
}

static int read_params(FILE * in, int *op, int *curve, int *size, int *k_size)
{
   char *p;
   if ((p = get_line(in)) != NULL) {
      sscanf(p, "%d, %d, %d, %d", op, curve, size, k_size);
      return 0;
   }

   return 1;
}

static void print_row_2cols(unsigned offset, const unsigned char *a,
                            const unsigned char *b, size_t n)
{
   size_t i;

   printf("%8u", offset);

   printf("  ");
   for (i = 0; i < 4; i++) {
      if (i < n) {
         printf("%.2x", (unsigned)a[i]);
      } else {
         printf("  ");
      }
   }

   printf("  ");
   for (i = 0; i < 4; i++) {
      if (i < n) {
         printf("%.2x", (unsigned)b[i]);
      } else {
         printf("  ");
      }
   }

   printf("\n");
}

static void outdiff(char *s, unsigned char *buf1, unsigned char *buf2, size_t n)
{
   size_t i;

   printf("Comparing [%s]\n", s);
   printf("  Offset  Received  Expected\n");
   printf("  --------------------------\n");
   for (i = 0; i < n; i += 4) {
      print_row_2cols(i, buf1+i, buf2+i, MIN(4, n-i));
   }
}



static int parse_test_script(char *filename)
{
   int failed;
   FILE *in;

   int op, curve, size, k_size, skip = 0;
   int op_size;
   int rc;

   struct tm *tm;
   time_t start_time;
   char tbuf[BN_RSA_BASE_RADIX_SIZE];

   unsigned char m[BN_RSA_BASE_RADIX_SIZE];
   unsigned char mp[BN_RSA_BASE_RADIX_SIZE];
   unsigned char a[BN_RSA_MAX_RADIX_SIZE];
   unsigned char b[BN_RSA_MAX_RADIX_SIZE];
   unsigned char x[BN_RSA_MAX_RADIX_SIZE];
   unsigned char y[BN_RSA_MAX_RADIX_SIZE];
   unsigned char x2[BN_RSA_MAX_RADIX_SIZE];
   unsigned char y2[BN_RSA_MAX_RADIX_SIZE];
   unsigned char e[BN_RSA_MAX_RADIX_SIZE];
   unsigned char d[BN_RSA_MAX_RADIX_SIZE];
   unsigned char k[BN_RSA_MAX_RADIX_SIZE];
   unsigned char k2[BN_RSA_BASE_RADIX_SIZE];
   unsigned char rx[BN_RSA_BASE_RADIX_SIZE];
   unsigned char ry[BN_RSA_MAX_RADIX_SIZE];

   if (filename) {
      if (!(in = fopen(filename, "r")))
         return 1;
   } else {
      filename = "<stdin>";
      in = stdin;
   }
#define RI read_int(in)
#define RW(b, l) read_words(in, b, l)

   if (read_params(in, &op, &curve, &size, &k_size) != 0)
      return 2;
   if (ecc_diag_verbose == 1) {
      start_time = time(0);
      tm = localtime(&start_time);
      strftime(tbuf, sizeof(tbuf), "%d-%m-%Y %T", tm);
   }

   memset(a, 0, sizeof a);
   memset(b, 0, sizeof b);
   memset(m, 0, sizeof m);
   memset(e, 0, sizeof e);
   memset(d, 0, sizeof d);
   memset(k, 0, sizeof k);
   memset(mp, 0, sizeof mp);
   memset(y, 0, sizeof y);
   memset(rx, 0, sizeof rx);
   memset(ry, 0, sizeof ry);

   switch (op) {
      case PKA_VECTOR_PMULTBLIND:            // PMULT w/ blinding
         // read blind value
         RW(d, (size + 31) / 32);
         /* Fall through to regular PMULT */
      case PKA_VECTOR_PMULT:                 // PMULT use internal curve parameters --- G(x,y)
         RW(k, (k_size+3) / 4);
         RW(rx, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;
      case PKA_VECTOR_PADD:                  // PADD
         RW(a, (size + 31) / 32);   //x1
         RW(b, (size + 31) / 32);   //y1
         RW(m, (size + 31) / 32);   //x2
         RW(mp, (size + 31) / 32);  //y2
         RW(rx, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;
      case PKA_VECTOR_PVER:                  // PVER
         RW(rx, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;
      case PKA_VECTOR_PDBL:                  // PDOUBLE
         RW(a, (size + 31) / 32);   //x
         RW(b, (size + 31) / 32);   //y
         RW(rx, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;
      case PKA_VECTOR_SHAMIR:
         RW(k, (k_size+3) / 4);
         RW(k2, (k_size+3) / 4);
         RW(x, (size + 31) / 32);
         RW(y, (size + 31) / 32);
         RW(x2, (size + 31) / 32);
         RW(y2, (size + 31) / 32);
         RW(rx, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;

      case PKA_VECTOR_MODEXP_C25519:
         RW(x,  (size + 31) / 32);
         RW(y,  (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;

      case PKA_VECTOR_PMULTBLIND_C25519:
         /* Blind value */
         RW(d, (size + 31) / 32);
         /* Fall through to regular operation */
      case PKA_VECTOR_PMULT_C25519:
         RW(k, (k_size + 3) / 4);
         RW(x, (size + 31) / 32);
         RW(rx, (size + 31) / 32);
         break;

      case PKA_VECTOR_PADDBLIND_ED25519:
         /* Blind value */
         RW(d, (size + 31) / 32);
         /* Fall through to regular operation */
      case PKA_VECTOR_PADD_ED25519:
         RW(x, (size + 31) / 32);
         RW(y, (size + 31) / 32);
         RW(x2, (size + 31) / 32);
         RW(y2, (size + 31) / 32);
         RW(rx, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;

      case PKA_VECTOR_PDBLBLIND_ED25519:
         /* Blind value */
         RW(d, (size + 31) / 32);
         /* Fall through to regular operation */
      case PKA_VECTOR_PDBL_ED25519:
         RW(x, (size + 31) / 32);
         RW(y, (size + 31) / 32);
         RW(rx, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;

      case PKA_VECTOR_PMULTBLIND_ED25519:
         /* Blind value */
         RW(d, (size + 31) / 32);
         /* Fall through to regular operation */
      case PKA_VECTOR_PMULT_ED25519:
         RW(k, (size + 31) / 32);
         RW(x, (size + 31) / 32);
         RW(y, (size + 31) / 32);
         RW(rx, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;

      case PKA_VECTOR_SHAMIRBLIND_ED25519:
         /* Blind value */
         RW(d, (size + 31) / 32);
         /* Fall through to regular operation */
      case PKA_VECTOR_SHAMIR_ED25519:
         RW(k, (k_size + 3) / 4);
         RW(k2, (k_size + 3) / 4);
         RW(x, (size + 31) / 32);
         RW(y, (size + 31) / 32);
         RW(x2, (size + 31) / 32);
         RW(y2, (size + 31) / 32);
         RW(rx, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;

      case PKA_VECTOR_PVER_ED25519:
         RW(x, (size + 31) / 32);
         RW(y, (size + 31) / 32);
         break;

      case PKA_VECTOR_MODEXP:                  // mod_exp
         //software do the precompute, instead of RW (rr, size/32); RW (mp, size/32);
         RW(m, (size + 31) / 32);
         RW(a, (size + 31) / 32);
         RW(b, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;
      case PKA_VECTOR_MODMUL:                  // mod_mult
         //software do the precompute, instead of RW (rr, size/32); RW (mp, size/32);
         RW(m, (size + 31) / 32);
         RW(a, (size + 31) / 32);
         RW(b, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;
      case PKA_VECTOR_MODDIV:                 // mod_div
         RW(m, (size + 31) / 32);
         RW(a, (size + 31) / 32);
         RW(b, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;
      case PKA_VECTOR_MODADD:                 // mod_add
         RW(m, (size + 31) / 32);
         RW(a, (size + 31) / 32);
         RW(b, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;
      case PKA_VECTOR_MODSUB:                 // mod_sub
         RW(m, (size + 31) / 32);
         RW(a, (size + 31) / 32);
         RW(b, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;
      case PKA_VECTOR_MODEXP_CRT:                 // CRT
         RW(a, (size + 63) / 64);
         RW(b, (size + 63) / 64);
         RW(d, (size + 31) / 32);
         RW(m, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
      case PKA_VECTOR_MODINV:                 // mod_inv
         RW(m, (size + 31) / 32);
         RW(b, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;
      case PKA_VECTOR_MODRED:                 // mod_reduction
         RW(m, (size + 31) / 32);
         RW(a, (size + 31) / 32);
         RW(ry, (size + 31) / 32);
         break;
      default:
         break;
   }

   /*
    * Clear any values out of PKA operand memory.  We write garbage to the
    * full width of all operands (least power of two >= the vector size).
    */
   op_size = 1;
   while (op_size < size)
      op_size <<= 1;

   if (pka_tool_poison_ops(fd, (op_size+7)/8) != 0) {
      pka_tool_err(0, "failed to poison PKA operands (%d)", (op_size+7)/8);
      return 1;
   }

   failed = 0;
   switch (op) {
      case PKA_VECTOR_PMULTBLIND:
         if (setup_ecc_blinding(fd, (size + 7) / 8, d) < 0) {
            failed = 1;
            break;
         }
         /* fall through to ordinary pmult */
      case PKA_VECTOR_PMULT:
         if ((clue_ec_load_curve_data(fd, curve)) != CRYPTO_OK) {
            failed = 1;
            printf("Failed to load curve data\n");
         }
         if ((clue_ec_load_curve_pmult(fd, curve, 1)) != CRYPTO_OK) {
            failed = 1;
            printf("Failed to load mult data\n");
         }
         if ((clue_ec_point_mult_base(fd, k, x, y, (size + 7) / 8, k_size)) != CRYPTO_OK)
            failed = 1;
         break;
      case PKA_VECTOR_PADD:
         if ((clue_ec_load_curve_data(fd, curve)) != CRYPTO_OK)
            failed = 1;
         if ((clue_ec_load_curve_padd(fd, curve)) != CRYPTO_OK)
            failed = 1;
         if ((clue_ec_point_add(fd, a, b, m, mp, x, y, (size + 7) / 8)) != CRYPTO_OK)
            failed = 1;
         break;
      case PKA_VECTOR_PVER:
         if ((clue_ec_load_curve_data(fd, curve)) != CRYPTO_OK)
            failed = 1;
         if ((clue_ec_load_curve_pver(fd, curve)) != CRYPTO_OK)
            failed = 1;
         if ((clue_ec_point_verify(fd, rx, ry, (size + 7) / 8)) != CRYPTO_OK)
            failed = 1;
         break;
      case PKA_VECTOR_PDBL:
         if ((clue_ec_load_curve_data(fd, curve)) != CRYPTO_OK)
            failed = 1;
         if ((clue_ec_load_curve_pdbl(fd, curve)) != CRYPTO_OK)
            failed = 1;
         if ((clue_ec_point_double(fd, a, b, x, y, (size + 7) / 8)) != CRYPTO_OK)
            failed = 1;
         break;
      case PKA_VECTOR_SHAMIR:
         if ((clue_ec_load_curve_data(fd, curve)) != CRYPTO_OK) {
            failed = 1;
            printf("Failed to load curve data\n");
         }
         if ((clue_ec_load_curve_pmult(fd, curve, 1)) != CRYPTO_OK) {
            failed = 1;
            printf("Failed to load mult data\n");
         }
         if ((clue_ec_point_mult_shamir(fd, k, k2, x, y, x2, y2, (size + 7) / 8, k_size)) != CRYPTO_OK)
            failed = 1;
         break;

      case PKA_VECTOR_MODEXP_C25519:
         rc = pka_tool_run(fd, "c25519_modexp", (size + 7) / 8,
                               "%A0", x, "%D2", y, "%D0", m25519,
                               "=%A0", y,
                               (char *)NULL);
         failed = (rc != 0);
         break;

      case PKA_VECTOR_PMULTBLIND_C25519:
         if (setup_ecc_blinding(fd, (size + 7) / 8, d) < 0) {
            failed = 1;
            break;
         }
         /* fall through to ordinary operation */
      case PKA_VECTOR_PMULT_C25519:
         if (load_c25519(fd) < 0) {
            failed = 1;
            break;
         }

         rc = pka_tool_run(fd, "c25519_pmult", (size + 7) / 8,
                               "%A2", x, "%D7", k,
                               "=%A2", x,
                               (char *)NULL);
         failed = (rc != 0);
         break;

      case PKA_VECTOR_PADDBLIND_ED25519:
         if (setup_ecc_blinding(fd, (size + 7) / 8, d) < 0) {
            failed = 1;
            break;
         }
         /* fall through to ordinary operation */
      case PKA_VECTOR_PADD_ED25519:
         if (load_ed25519(fd) < 0) {
            failed = 1;
            break;
         }

         rc = pka_tool_run(fd, "ed25519_padd", (size + 7) / 8,
                               "%A2", x, "%B2", y, "%A3", x2, "%B3", y2,
                               "=%A2", x, "=%B2", y,
                               (char *)NULL);
         failed = (rc != 0);
         break;

      case PKA_VECTOR_PDBLBLIND_ED25519:
         if (setup_ecc_blinding(fd, (size + 7) / 8, d) < 0) {
            failed = 1;
            break;
         }
         /* fall through to ordinary operation */
      case PKA_VECTOR_PDBL_ED25519:
         if (load_ed25519(fd) < 0) {
            failed = 1;
            break;
         }

         rc = pka_tool_run(fd, "ed25519_pdbl", (size + 7) / 8,
                               "%A2", x, "%B2", y,
                               "=%A2", x, "=%B2", y,
                               (char *)NULL);
         failed = (rc != 0);
         break;

      case PKA_VECTOR_PMULTBLIND_ED25519:
         if (setup_ecc_blinding(fd, (size + 7) / 8, d) < 0) {
            failed = 1;
            break;
         }
         /* fall through to ordinary operation */
      case PKA_VECTOR_PMULT_ED25519:
         if (load_ed25519(fd) < 0) {
            failed = 1;
            break;
         }

         rc = pka_tool_run(fd, "ed25519_pmult", (size + 7) / 8,
                               "%A2", x, "%B2", y, "%D7", k,
                               "=%A2", x, "=%B2", y,
                               (char *)NULL);
         failed = (rc != 0);
         break;

      case PKA_VECTOR_SHAMIRBLIND_ED25519:
         if (setup_ecc_blinding(fd, (size + 7) / 8, d) < 0) {
            failed = 1;
            break;
         }
         /* fall through to ordinary operation */
      case PKA_VECTOR_SHAMIR_ED25519:
         if (load_ed25519(fd) < 0) {
            failed = 1;
            break;
         }

         rc = pka_tool_run(fd, "ed25519_shamir", (size + 7) / 8,
                               "%A2", x, "%B2", y, "%A5", k,
                               "%A3", x2, "%B3", y2, "%B5", k2,
                               "=%A2", x, "=%B2", y,
                               (char *)NULL);
         failed = (rc != 0);
         break;

      case PKA_VECTOR_PVER_ED25519:
         if (load_ed25519(fd) < 0) {
            failed = 1;
            break;
         }

         rc = pka_tool_run(fd, "ed25519_pver", (size + 7) / 8,
                               "%A2", x, "%B2", y,
                               (char *)NULL);
         if (rc != 0) {
            failed = 1;
            break;
         }

         rc = elppka_test_flag(fd, NULL, "Z");
         if (rc < 0)
            pka_tool_err(0, "error reading status");

         failed = !(rc > 0);
         break;

      case PKA_VECTOR_MODEXP:
         if (clue_bn_modexp(fd, a, b, m, y, (size + 7) / 8, 1) != CRYPTO_OK)
            failed = 1;
         break;
      case PKA_VECTOR_MODMUL:
         if (clue_bn_modmult(fd, a, b, m, y, (size + 7) / 8, 1) != CRYPTO_OK)
            failed = 1;
         break;
      case PKA_VECTOR_MODDIV:
         if (clue_bn_moddiv(fd, a, b, m, y, (size + 7) / 8) != CRYPTO_OK)
            failed = 1;
         break;
      case PKA_VECTOR_MODADD:
         if (clue_bn_modadd(fd, a, b, m, y, (size + 7) / 8) != CRYPTO_OK)
            failed = 1;
         break;
      case PKA_VECTOR_MODSUB:
         if (clue_bn_modsub(fd, a, b, m, y, (size + 7) / 8) != CRYPTO_OK)
            failed = 1;
         break;
      case PKA_VECTOR_MODEXP_CRT:
         if (clue_crt_modexp(fd, a, b, d, m, y, (size + 7) / 8) != CRYPTO_OK)
            failed = 1;
         break;
      case PKA_VECTOR_MODINV:
         if (clue_bn_modinv(fd, b, m, y, (size + 7) / 8) != CRYPTO_OK)
            failed = 1;
         break;
      case PKA_VECTOR_MODRED:
         if (clue_bn_modred(fd, a, m, y, (size + 7) / 8) != CRYPTO_OK)
            failed = 1;
         break;
      default:
         fprintf(stderr, "FAILED: Vector not parsed\n");
         failed = skip = 1;
         break;
   }


   if (!skip) {
      // If failed already do not check the result
      if (!failed) {
         failed = 0;
         switch (op) {
         case PKA_VECTOR_PVER:
         case PKA_VECTOR_PVER_ED25519:
            /* PVER has already passed by this point */
            failed = 0;
            break;
         case PKA_VECTOR_PMULTBLIND:
         case PKA_VECTOR_PMULT:
         case PKA_VECTOR_PADD:
         case PKA_VECTOR_PDBL:
         case PKA_VECTOR_SHAMIR:
         case PKA_VECTOR_PMULT_C25519:
         case PKA_VECTOR_PADD_ED25519:
         case PKA_VECTOR_PDBL_ED25519:
         case PKA_VECTOR_PMULT_ED25519:
         case PKA_VECTOR_SHAMIR_ED25519:
         case PKA_VECTOR_PMULTBLIND_C25519:
         case PKA_VECTOR_PADDBLIND_ED25519:
         case PKA_VECTOR_PDBLBLIND_ED25519:
         case PKA_VECTOR_PMULTBLIND_ED25519:
         case PKA_VECTOR_SHAMIRBLIND_ED25519:
            /* Check X for ECC point ops */
            if (memcmp(x, rx, (size + 7) / 8) != 0) {
               if (!ecc_diag_silent)
                  outdiff("ciphertext X", x, rx, (size + 7) / 8);
               failed = 1;
            }
            /* fall through */
         case PKA_VECTOR_MODEXP_C25519:
         case PKA_VECTOR_MODEXP_CRT:
         case PKA_VECTOR_MODEXP:
         case PKA_VECTOR_MODMUL:
         case PKA_VECTOR_MODDIV:
         case PKA_VECTOR_MODADD:
         case PKA_VECTOR_MODSUB:
         case PKA_VECTOR_MODINV:
         case PKA_VECTOR_MODRED:
            /* RSA ops put result in Y */
            if (memcmp(y, ry, (size + 7) / 8) != 0) {
               if (!ecc_diag_silent)
                  outdiff("ciphertext Y", y, ry, (size + 7) / 8);
               failed = 1;
            }
            break;
         default:
            fprintf(stderr, "Cannot check vector: not implemented\n");
            failed = 1;
         }
      }

      if (!ecc_diag_silent)
         fprintf(stderr, "%s SIZE-%4d %s %s\n", op_name[op], size, filename, failed ? "[FAILED]" : "[PASSED]");
   }

   return failed;
}

int parse_cmd_vectors(int argc, char **argv)
{
   int ret = 0;

   while ((ret = parse_test_script(0)) == 0);
   return (ret == 1) ? 1 : 0;

   if (argc == 1) {
      return parse_test_script(0);
   } else {
      return parse_test_script(argv[0]);
   }
}


int main(int argc, char **argv)
{
   int ret = 0, i;
   int nargc = argc;
   char **nargv = argv;
   char *dev = NULL;

   pka_tool_init("pka-diag", argc, argv);

   // skip the program name
   nargc--;
   nargv = (char **)argv[1];

   // added some options and reading stdin stream for redirecting and piping
   // check only two first arguments if any
   // [-v] verbose
   // [-s] silent
   for (i = 1; i <= 10; i++) {
      if (nargc >= 1) {
         if (argv[i][0] == '-') {
            nargc--;
            nargv = (char **)argv[i + 1];
            switch (argv[i][1]) {
               case 'v':
               case 'V':
                  ecc_diag_verbose = 1;
               case 's':
               case 'S':
                  ecc_diag_silent = 1;
                  break;
               case 'I':
               case 'i':
                  _test_invalid_case = 1;
                  break;
               case 'P':
               case 'p':
                  if (argv[i][2] != '=') {
                     printf("SHOULD BE: -p=pka_name\n");
                     return -1;
                  }
                  strcpy(dev, &argv[i][3]);
                  break;
            }
         }
      }
   }

   fd = elppka_device_open(dev);
   if (fd == -1) {
      if (dev)
         fprintf(stderr, "%s: %s\n", dev, strerror(errno));
      else
         fprintf(stderr, "%s\n", strerror(errno));
      return EXIT_FAILURE;
   }
#if DO_ECC
   clue_ec_init();
#endif





   ret = parse_cmd_vectors(nargc, nargv);

   elppka_device_close(fd);

   return ret;
}
