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
#include "diag/SECP_192.inc"
#include "diag/SECP_256.inc"
#include "diag/SECP_384.inc"

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

static void dump_param(unsigned size, const unsigned char *buf, int indent)
{
   unsigned i;

   for (i = 0; i < size; i++) {
      if (i % 16 == 0)
         printf("%*s%.4x:", indent, "", i);
      if (i % 2 == 0)
         putchar(' ');

      printf("%.2x", (unsigned)buf[i]);

      if ((i+1) % 16 == 0)
         putchar('\n');
   }

   if (i % 16)
      putchar('\n');


      putchar('\n');
}

/* This is a demo for ECDSA. It does not use an rng, or a hash function, 
 * and it doesn't check for rejection case. 
 * A signature is generated following ECDSA specification and verified. 
 * The message is used as it is, no hash function is involved. 
 * The user has to select the curve (SEC192R1, SEC256R1, SEC384R1) and 
 * to provide the private key d, the message e and the elphemeral key k. 
 */

static int dumb_eccsignature(int fd)
{

   const EC_CURVE_DATA *curve = &SEC384R1;
   puts(curve->comment); 

   /* d is the private key ... */
   static const unsigned char d[CURVE_MAX_DATA_SIZE] = {
      0x49, 0x96, 0xd2, 0x7a, 0xd5, 0x6f, 0x2a, 0x89,
      0x4a, 0x46, 0xa9, 0x1e, 0x88, 0xcf, 0x06, 0x05,
      0x76, 0x6f, 0x7b, 0x2a, 0x00
   }; 
   /* print the private key ... */
	puts("your private key"); 
	puts("d:"); 
	dump_param(curve->size, d, 3); 


   unsigned char pk_x[sizeof d], pk_y[sizeof d], zero[sizeof d]; 


   char  e[sizeof d] = "THIS IS A MESSAGE";
   unsigned char result_x[sizeof d], result_y[sizeof d], r[sizeof d], s[sizeof d]; 

   int rc;


   /* generate the public key ... */ 
   rc = elppka_run(fd, "pmult", curve->size,
                        "%A2",  curve->x,
                        "%B2",  curve->y,
                        "%A6",  curve->a,
                        "%A7",  zero,
                        "%D0",  curve->m,
                        "%D1",  curve->mp,
                        "%D3",  curve->r,
                        "%D7",  d,
                        "=%A2", pk_x,
                        "=%B2", pk_y,
                        (char *)NULL);
   if (rc == -1) {
      pka_tool_err(0, "pmult");
      return -1;
   } else if (rc > 0) {
      pka_tool_err(-1, "pmult: device returned %d\n", rc);
      return -1;
   }
   /* print the public key: : keep it ! you'll need it to verify the signature ... */
	puts("your public key"); 
	puts("x:"); 
	dump_param(curve->size, pk_x, 3); 
	puts("y:"); 
	dump_param(curve->size, pk_y, 3); 


   /* print the message ... */
	puts("the message to be signed"); 
	puts(e); 
      	putchar('\n');
	

   /* generate the ephemeral key pair ... */
   static const unsigned char k[CURVE_MAX_DATA_SIZE] = {
      0x48, 0x96, 0xd2, 0x7a, 0xd5, 0x6f, 0x2a, 0x89,
      0x4a, 0x46, 0xa9, 0x1e, 0x88, 0xcf, 0x06, 0x05,
      0x76, 0x6f, 0x7b, 0x2a, 0x00
   }; 

   rc = elppka_run(fd, "pmult", curve->size,
                        "%A2",  curve->x,
                        "%B2",  curve->y,
                        "%A6",  curve->a,
                        "%A7",  zero,
                        "%D0",  curve->m,
                        "%D1",  curve->mp,
                        "%D3",  curve->r,
                        "%D7",  k,
                        "=%A2", result_x,
                        "=%B2", result_y,
                        (char *)NULL);
   if (rc == -1) {
      pka_tool_err(0, "pmult");
      return -1;
   } else if (rc > 0) {
      pka_tool_err(-1, "pmult: device returned %d\n", rc);
      return -1;
   }

   memcpy(r, result_x, sizeof result_x); 




   rc = elppka_run(fd, "modmult", curve->size, 
                        "%A0",  result_x,
                        "%B0",  d,
                        "%D0",  curve->n,
                        "%D1",  curve->np, 	
                        "%D3",  curve->nr,
                        "%F1",  NULL,		/* only for 521 ... */
                        "=%A0", s,
                        (char *)NULL);
   if (rc == -1) {
      pka_tool_err(0, "modmult");
      return -1;
   } else if (rc > 0) {
      pka_tool_err(-1, "modmult: device returned %d\n", rc);
      return -1;
   }

   rc = elppka_run(fd, "modadd", curve->size,	
                       "%A0",  e,
                       "%B0",  s,
                       "%D0",  curve->n,
                       "=%A0", s,		/* e + dr mod n */
                       (char *)NULL);
   if (rc == -1) {
      pka_tool_err(0, "modadd");
      return -1;
   } else if (rc > 0) {
      pka_tool_err(-1, "modadd: device returned %d\n", rc);
      return -1;
   }

   rc = elppka_run(fd, "moddiv", curve->size,	
                       "%C0",  s,
                       "%A0",  k,
                       "%D0",  curve->n,
                       "=%C0", s,		/* (e + dr)/k  mod n */
                       (char *)NULL);
   if (rc == -1) {
      pka_tool_err(0, "moddiv");
      return -1;
   } else if (rc > 0) {
      pka_tool_err(-1, "moddiv: device returned %d\n", rc);
      return -1;
   }




	/* print the signature ... */
	puts("signature"); 
	puts("r:"); 
	dump_param(curve->size, r, 3); 
	puts("s:"); 
	dump_param(curve->size, s, 3); 



	/* now, verify ... */

	unsigned char r2[sizeof d]; 
	memcpy(r2, r, sizeof r); 

   rc = elppka_run(fd, "moddiv", curve->size,	
                       "%C0",  r,
                       "%A0",  s,
                       "%D0",  curve->n,
                       "=%C0", r2,		/* r/s  mod m */
                       (char *)NULL);
   if (rc == -1) {
      pka_tool_err(0, "moddiv");
      return -1;
   } else if (rc > 0) {
      pka_tool_err(-1, "moddiv: device returned %d\n", rc);
      return -1;
   }

   rc = elppka_run(fd, "pmult", curve->size,
                        "%A2",  pk_x,
                        "%B2",  pk_y,
                        "%A6",  curve->a,
                        "%A7",  zero,
                        "%D0",  curve->m,
                        "%D1",  curve->mp,
                        "%D3",  curve->r,
                        "%D7",  r2,
                        "=%A2", pk_x,		/* r/s * P */
                        "=%B2", pk_y,
                        (char *)NULL);
   if (rc == -1) {
      pka_tool_err(0, "pmult");
      return -1;
   } else if (rc > 0) {
      pka_tool_err(-1, "pmult: device returned %d\n", rc);
      return -1;
   }




   rc = elppka_run(fd, "moddiv", curve->size,	
                       "%C0",  e,
                       "%A0",  s,
                       "%D0",  curve->n,
                       "=%C0", e,		/* e/s  mod m */
                       (char *)NULL);
   if (rc == -1) {
      pka_tool_err(0, "moddiv");
      return -1;
   } else if (rc > 0) {
      pka_tool_err(-1, "moddiv: device returned %d\n", rc);
      return -1;
   }

   rc = elppka_run(fd, "pmult", curve->size,
                        "%A2",  curve->x,
                        "%B2",  curve->y,
                        "%A6",  curve->a,
                        "%A7",  zero,
                        "%D0",  curve->m,
                        "%D1",  curve->mp,
                        "%D3",  curve->r,
                        "%D7",  e,
                        "=%A2", result_x,			/* e/s * G */
                        "=%B2", result_y,
                        (char *)NULL);
   if (rc == -1) {
      pka_tool_err(0, "pmult");
      return -1;
   } else if (rc > 0) {
      pka_tool_err(-1, "pmult: device returned %d\n", rc);
      return -1;
   }



   rc = elppka_run(fd, "padd", curve->size,
                        "%A2",  result_x,
                        "%B2",  result_y,
                        "%A3",  pk_x,
                        "%B3",  pk_y,     
                        "%A6",  curve->a,
                        "%D0",  curve->m,
                        "%D1",  curve->mp,
                        "%D3",  curve->r,
                        "=%A2", result_x,			/* e/s * G  +  r/s* P */
                        "=%B2", result_y,
                        (char *)NULL);
   if (rc == -1) {
      pka_tool_err(0, "pmult");
      return -1;
   } else if (rc > 0) {
      pka_tool_err(-1, "pmult: device returned %d\n", rc);
      return -1;
   }


	puts("verify: "); 


   if (memcmp(r, result_x, curve->size)) {
      pka_tool_err(-1, "signature verified: NO");
      return -1;
   } else {
      pka_tool_err(0, "signature verified: YES");
      return 0;
   }

}


int main(int argc, char **argv)
{
   const char *dev = NULL;
   int opt, fd;

   pka_tool_init("eccsignature", argc, argv);

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
         return EXIT_FAILURE;
      }
   }

   fd = elppka_device_open(dev);
   if (fd == -1) {
      if (dev)
         pka_tool_err(0, "%s", dev);
      else
         pka_tool_err(0, NULL);
      return EXIT_FAILURE;
   }

   if (dumb_eccsignature(fd) != 0)
      return EXIT_FAILURE;

   elppka_device_close(fd);
   return 0;
}
