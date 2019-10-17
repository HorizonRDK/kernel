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
 * Copyright (c) 2012-2015 Synopsys, Inc. and/or its affiliates.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <assert.h>
#include <getopt.h>

#include "pkadev.h"
#include "common.h"
#include "diag/elpcurve.h"
#include "diag/SECP_160.inc"
#include "diag/SECP_521.inc"

/* Exit codes */
#define RET_PASS   0 /* Test passed */
#define RET_FAIL   1 /* Test ran to completion, but wrong answer */
#define RET_SKIP  77 /* Cannot complete test since feature is not supported */
#define RET_FATAL 99 /* Cannot complete test due to errors */

static const char sopts[] = "d:VH";
static const struct option lopts[] = {
   { "device",  1, NULL, 'd' },
   { "version", 0, NULL, 'V' },
   { "help",    0, NULL, 'H' },
   { 0 }
};

static void print_usage(FILE *f)
{
   fprintf(f, "usage: %s [-d device]\n", pka_tool_name());
}

static const unsigned char Ed25519_prime64[] = {0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xed, };
static const unsigned char Ed25519_curve_a[] = {0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xec, };
static const unsigned char Ed25519_curve_d[] = {0x52, 0x03, 0x6c, 0xee, 0x2b, 0x6f, 0xfe, 0x73, 0x8c, 0xc7, 0x40, 0x79, 0x77, 0x79, 0xe8, 0x98,
                                        0x00, 0x70, 0x0a, 0x4d, 0x41, 0x41, 0xd8, 0xab, 0x75, 0xeb, 0x4d, 0xca, 0x13, 0x59, 0x78, 0xa3, };
static const unsigned char Ed25519_order64[] = {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x14, 0xde, 0xf9, 0xde, 0xa2, 0xf7, 0x9c, 0xd6, 0x58, 0x12, 0x63, 0x1a, 0x5c, 0xf5, 0xd3, 0xed, };
static const unsigned char Ed25519_Gx64[]    = {0x21, 0x69, 0x36, 0xd3, 0xcd, 0x6e, 0x53, 0xfe, 0xc0, 0xa4, 0xe2, 0x31, 0xfd, 0xd6, 0xdc, 0x5c,
                                        0x69, 0x2c, 0xc7, 0x60, 0x95, 0x25, 0xa7, 0xb2, 0xc9, 0x56, 0x2d, 0x60, 0x8f, 0x25, 0xd5, 0x1a, };
static const unsigned char Ed25519_Gy64[]    = {0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66,
                                        0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x58, };





/* Basic test of SECP160r1 pver */
//  - Base modular op (modadd)
//  - RSA (modexp)
//  * ECC (pver)
//  * ECC Shamir's Trick (shamir)
//  * ECC-521 (pver_521)
//  * ECC-521 Shamir's Trick (shamir_521)
//  * ECC 25519 (ed25519_pver) 
//  * ECC 25519 Shamir's Trick (ed25519_shamir)




//  - Base modular op (modadd)
static int test_modadd(int fd)
{
   /* let's use the parameters from a curve and calculate x^y mod (order of base point) ... */
   const EC_CURVE_DATA *curve = &SEC160R1;
   unsigned char r[66]; 
   int rc;


   rc = elppka_run(fd, "modadd", curve->size,
                        "%A0", curve->x,
                        "%B0", curve->y,
                        "%D0", curve->m,
                        "=%A0", r,  
                        (char *)NULL);
   if (rc == -1) {
      if (errno == ENOENT) {
         printf("SKIPPED modadd (not supported by firmware)\n");
         return RET_SKIP;
      }

      pka_tool_err(0, "ERROR running pver");
      return RET_FATAL;
   } else if (rc > 0) {
      pka_tool_err(-1, "ERROR: device returned %d", rc);
      return RET_FATAL;
   }


   printf("PASSED modadd \n");
   return RET_PASS;
}


//  - RSA (modexp)
static int test_modexp(int fd)
{
   /* let's use the parameters from a curve and calculate x^y mod (order of base point) ... */
   const EC_CURVE_DATA *curve = &SEC160R1;
   unsigned char r[66]; 
   int rc;


   rc = elppka_run(fd, "modexp", curve->size,
                        "%A0", curve->x,
                        "%D2", curve->y,
                        "%D0", curve->m,
                        "%D1", curve->mp,
                        "%D3", curve->r,
                        "=%A0", r,  
                        (char *)NULL);
   if (rc == -1) {
      if (errno == ENOENT) {
         printf("SKIPPED modexp (not supported by firmware)\n");
         return RET_SKIP;
      }

      pka_tool_err(0, "ERROR running pver");
      return RET_FATAL;
   } else if (rc > 0) {
      pka_tool_err(-1, "ERROR: device returned %d", rc);
      return RET_FATAL;
   }


   printf("PASSED modexp \n");
   return RET_PASS;
}

//  - ECC (pver)
static int test_pver(int fd)
{
   const EC_CURVE_DATA *curve = &SEC160R1;
   int rc;

   rc = elppka_run(fd, "pver", curve->size,
                        "%A2", curve->x,
                        "%B2", curve->y,
                        "%A6", curve->a,
                        "%A7", curve->b,
                        "%D0", curve->m,
                        "%D1", curve->mp,
                        "%D3", curve->r,
                        (char *)NULL);
   if (rc == -1) {
      if (errno == ENOENT) {
         printf("SKIPPED pver (not supported by firmware)\n");
         return RET_SKIP;
      }

      pka_tool_err(0, "ERROR running pver");
      return RET_FATAL;
   } else if (rc > 0) {
      pka_tool_err(-1, "ERROR: device returned %d", rc);
      return RET_FATAL;
   }

   rc = elppka_test_flag(fd, NULL, "Z");
   if (rc == -1) {
      pka_tool_err(0, "ERROR testing flag");
      return RET_FATAL;
   } else if (rc == 0) {
      printf("FAILED pver: base point not on curve\n");
      return RET_FAIL;
   } else if (rc ==1) {
      printf("PASSED pver: base point is on curve\n");
   	return RET_PASS;
   }
	return 0; 

}

//  - ECC Shamir's Trick (shamir)
static int test_shamir(int fd)
{
   const EC_CURVE_DATA *curve = &SEC160R1;
   int rc;
   unsigned char R_x[32], R_y[32]; 
   unsigned char k[32] = { 0x2, 0x3, 0x0, }; 
   unsigned char l[32] = { 0x3, 0x2, 0x0, }; 

   /* let's generate another point  .... */
   rc = elppka_run(fd, "pdbl",  curve->size,
                        "%A3",  curve->x,
                        "%B3",  curve->y,
                        "%A6",  curve->a,
                        "%D0",  curve->m,
                        "%D1",  curve->mp,
                        "%D3",  curve->r,
                        "=%A2", R_x,
                        "=%B2", R_y,
                        (char *)NULL);
   if (rc == -1) {
      if (errno == ENOENT) {
         printf("SKIPPED secp160r1 shamir (pdbl not supported by firmware)\n");
         return RET_SKIP;
      }

      pka_tool_err(0, "ERROR running pdbl");
      return RET_FATAL;
   } else if (rc > 0) {
      pka_tool_err(-1, "ERROR: device returned %d", rc);
      return RET_FATAL;
   }

	

   rc = elppka_run(fd, "shamir", curve->size,
                        "%A2", curve->x,
                        "%B2", curve->y,
                        "%A3", R_x,
                        "%B3", R_y,
                        "%A6", curve->a,
                        "%A7", k,
                        "%D7", l,
                        "%D0", curve->m,
                        "%D1", curve->mp,
                        "%D3", curve->r,
                        "=%A3", R_x,
                        "=%B3", R_y,
                        (char *)NULL);
   if (rc == -1) {
      if (errno == ENOENT) {
         printf("SKIPPED secp160r1 shamir (not supported by firmware)\n");
         return RET_SKIP;
      }

      pka_tool_err(0, "ERROR running shamir");
      return RET_FATAL;
   } else if (rc > 0) {
      pka_tool_err(-1, "ERROR: device returned %d", rc);
      return RET_FATAL;
   }

   /* One should calculate now the other way: kG, lR, kG+lR -> R, and compare the results  ... */

   printf("PASSED secp160r1 shamir\n");
   return RET_PASS;
}

//  - ECC-521 (pver_521)
static int test_pver_521(int fd)
{
   const EC_CURVE_DATA *curve = &SEC521R1;
   int rc;

   rc = elppka_run(fd, "pver_521", curve->size,
                        "%A2", curve->x,
                        "%B2", curve->y,
                        "%A6", curve->a,
                        "%A7", curve->b,
                        "%D0", curve->m,
                        (char *)NULL);
   if (rc == -1) {
      if (errno == ENOENT) {
         printf("SKIPPED pver_521 (not supported by firmware)\n");
         return RET_SKIP;
      }

      pka_tool_err(0, "ERROR running pver_521");
      return RET_FATAL;
   } else if (rc > 0) {
      pka_tool_err(-1, "ERROR: device returned %d", rc);
      return RET_FATAL;
   }

   rc = elppka_test_flag(fd, NULL, "Z");
   if (rc == -1) {
      pka_tool_err(0, "ERROR testing flag");
      return RET_FATAL;
   } else if (rc == 0) {
      printf("FAILED pver_521: base point not on curve\n");
      return RET_FAIL;
   } else if (rc ==1) {
      printf("PASSED pver_521: base point is on curve\n");
   	return RET_PASS;
   }
	return 0; 
}


//  - ECC-521 Shamir's Trick (shamir_521)
static int test_shamir_521(int fd)
{
   const EC_CURVE_DATA *curve = &SEC521R1;
   int rc;
   unsigned char R_x[66], R_y[66]; 
   unsigned char k[66] = { 0x2, 0x3, 0x0, }; 
   unsigned char l[66] = { 0x3, 0x2, 0x0, }; 

   /* let's generate another point  .... */
   rc = elppka_run(fd, "pdbl_521", curve->size,
                        "%A3",  curve->x,
                        "%B3",  curve->y,
                        "%A6",  curve->a,
                        "%D0",  curve->m,
                        "%D1",  curve->mp,
                        "%D3",  curve->r,
                        "=%A2", R_x,
                        "=%B2", R_y,
                        (char *)NULL);
   if (rc == -1) {
      if (errno == ENOENT) {
         printf("SKIPPED secp521r1 shamir (pdbl_521 not supported by firmware)\n");
         return RET_SKIP;
      }

      pka_tool_err(0, "ERROR running pdbl_521");
      return RET_FATAL;
   } else if (rc > 0) {
      pka_tool_err(-1, "ERROR: device returned %d", rc);
      return RET_FATAL;
   }

	

   rc = elppka_run(fd, "shamir_521", curve->size,
                        "%A2", curve->x,
                        "%B2", curve->y,
                        "%A3", R_x,
                        "%B3", R_y,
                        "%A6", curve->a,
                        "%A7", k,
                        "%D7", l,
                        "%D0", curve->m,
                        "%D1", curve->mp,
                        "%D3", curve->r,
                        "=%A3", R_x,
                        "=%B3", R_y,
                        (char *)NULL);
   if (rc == -1) {
      if (errno == ENOENT) {
         printf("SKIPPED secp521r1 shamir (not supported by firmware)\n");
         return RET_SKIP;
      }

      pka_tool_err(0, "ERROR running shamir_521");
      return RET_FATAL;
   } else if (rc > 0) {
      pka_tool_err(-1, "ERROR: device returned %d", rc);
      return RET_FATAL;
   }

   /* One should calculate now the other way: kG, lR, kG+lR -> R, and compare the results  ... */


   printf("PASSED secp521r1 shamir\n");
   return RET_PASS;
}


//  - ECC 25519 (ed25519_pver)
static int test_ed25519_pver(int fd)
{
   int rc;

   rc = elppka_run(fd, "ed25519_pver", 32,
                        "%A2", Ed25519_Gx64, 
                        "%B2", Ed25519_Gy64,
                        "%C5", Ed25519_curve_d,
                        "%D0", Ed25519_prime64,
                        (char *)NULL);
   if (rc == -1) {
      if (errno == ENOENT) {
         printf("SKIPPED ed25519_pver (not supported by firmware)\n");
         return RET_SKIP;
      }

      pka_tool_err(0, "ERROR running pver");
      return RET_FATAL;
   } else if (rc > 0) {
      pka_tool_err(-1, "ERROR: device returned %d", rc);
      return RET_FATAL;
   }

   rc = elppka_test_flag(fd, NULL, "Z");
   if (rc == -1) {
      pka_tool_err(0, "ERROR testing flag");
      return RET_FATAL;
   } else if (rc == 0) {
      printf("FAILED ed25519_pver: base point not on curve\n");
      return RET_FAIL;
   } else if (rc ==1) {
      printf("PASSED ed25519_pver: base point is on curve\n");
   	return RET_PASS;
   }
	return 0; 

}


//  - ECC 25519 Shamir's Trick (ed25519_shamir)
static int test_ed25519_shamir(int fd)
{
   int rc;
   unsigned char R_x[32], R_y[32]; 
   unsigned char k[32] = { 0x2, 0x3, 0x0, }; 
   unsigned char l[32] = { 0x3, 0x2, 0x0, }; 

   /* let's generate another point  .... */
   rc = elppka_run(fd, "ed25519_pdbl",  32,
                        "%A2", Ed25519_Gx64, 
                        "%B2", Ed25519_Gy64,
                        "%C5", Ed25519_curve_d,
                        "%D0", Ed25519_prime64,
                        "=%A2", R_x,
                        "=%B2", R_y,
                        (char *)NULL);
   if (rc == -1) {
      if (errno == ENOENT) {
         printf("SKIPPED ed15519_shamir (pdbl not supported by firmware)\n");
         return RET_SKIP;
      }

      pka_tool_err(0, "ERROR running ed25519_pdbl");
      return RET_FATAL;
   } else if (rc > 0) {
      pka_tool_err(-1, "ERROR: device returned %d", rc);
      return RET_FATAL;
   }

	

   rc = elppka_run(fd, "ed25519_shamir", 32,
                        "%A2", Ed25519_Gx64, 
                        "%B2", Ed25519_Gy64,
                        "%A3", R_x,
                        "%B3", R_y,
                        "%A5", k,
                        "%B5", l,
                        "%C5", Ed25519_curve_d,
                        "%D0", Ed25519_prime64,
                        "=%A2", R_x,
                        "=%B2", R_y,
                        (char *)NULL);
   if (rc == -1) {
      if (errno == ENOENT) {
         printf("SKIPPED ed15519_shamir (not supported by firmware)\n");
         return RET_SKIP;
      }

      pka_tool_err(0, "ERROR running shamir");
      return RET_FATAL;
   } else if (rc > 0) {
      pka_tool_err(-1, "ERROR: device returned %d", rc);
      return RET_FATAL;
   }


   /* One should calculate now the other way: kG, lR, kG+lR -> R, and compare the results  ... */


   printf("PASSED ed25519_shamir\n");
   return RET_PASS;
}




int main(int argc, char **argv)
{
   const char *dev = NULL;
   int ret, opt, fd;

   pka_tool_init("simpletest", argc, argv);

   while ((opt = getopt_long(argc, argv, sopts, lopts, NULL)) != -1) {
      switch (opt) {
      case 'd':
         dev = optarg;
         break;
      case 'V':
         pka_tool_version(NULL);
         return EXIT_SUCCESS;
      case 'H':
         print_usage(stdout);
         return EXIT_SUCCESS;
      default:
         print_usage(stderr);
         return RET_FATAL;
      }
   }

   fd = elppka_device_open(dev);
   if (fd == -1) {
      if (dev)
         pka_tool_err(0, "%s", dev);
      else
         pka_tool_err(0, NULL);
      return RET_FATAL;
   }

   ret = test_modadd(fd);
   ret = test_modexp(fd);
   ret = test_pver(fd);
   ret = test_shamir(fd);
   ret = test_pver_521(fd);
   ret = test_shamir_521(fd);
   ret = test_ed25519_pver(fd);
   ret = test_ed25519_shamir(fd);




   elppka_device_close(fd);

   return ret;
}
