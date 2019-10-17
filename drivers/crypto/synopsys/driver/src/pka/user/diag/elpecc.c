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

#include <errno.h>

#include <pkadev.h>
#include "common.h"

#include "elpcrypto.h"
#include "elpcurve.h"

#define CLUE_MAX_CURVES 16

#include "SECP_160.inc"
#include "SECP_192.inc"
#include "SECP_224.inc"
#include "SECP_256.inc"
#include "SECP_384.inc"
#include "SECP_512.inc"
#include "SECP_521.inc"

static EC_CURVE_DATA *curve_data[CLUE_MAX_CURVES];

/*
 * ECC-521 has separate entry points; dispatch the operation to
 * the correct one with fallback for older cores.
 */
static int run_ecc(int fd, const char *func, unsigned size, ...)
{
   va_list ap;
   int rc;

   if (size == 66) {
      /* ECC-521 */
      char func521[33];

      snprintf(func521, sizeof func521, "%s_521", func);
      va_start(ap, size);
      rc = elppka_vrun(fd, func521, size, ap);
      va_end(ap);

      if (rc < 0) {
         if (errno == ENOENT)
            goto fallback;
         pka_tool_err(0, "%s", func521);
      } else if (rc > 0) {
         pka_tool_err(-1, "%s: failed (%d)", func521, rc);
      }

      return rc;
   }

fallback:
   /* Normal case */
   va_start(ap, size);
   rc = pka_tool_vrun(fd, func, size, ap);
   va_end(ap);

   return rc;
}

void clue_ec_init (void)
{
  int i;

  for (i = 0; i < CLUE_MAX_CURVES; i++)
    curve_data[i] = 0;

  curve_data[0] = (EC_CURVE_DATA *) &SEC160R1;
  curve_data[1] = (EC_CURVE_DATA *) &SEC160R2;
  curve_data[2] = (EC_CURVE_DATA *) &SEC160K1;
  curve_data[3] = (EC_CURVE_DATA *) &SEC160WTLS9;
  curve_data[4] = (EC_CURVE_DATA *) &SEC192R1;
  curve_data[5] = (EC_CURVE_DATA *) &SEC192K1;
  curve_data[6] = (EC_CURVE_DATA *) &SEC224R1;
  curve_data[7] = (EC_CURVE_DATA *) &SEC224K1;
  curve_data[8] = (EC_CURVE_DATA *) &SEC256R1;
  curve_data[9] = (EC_CURVE_DATA *) &SEC256K1;
  curve_data[10] = (EC_CURVE_DATA *) &SEC384R1;
  curve_data[11] = (EC_CURVE_DATA *) &SEC512R1;
  curve_data[12] = (EC_CURVE_DATA *) &SEC521R1;
}

short clue_ec_load_curve_pmult (int fd, unsigned short curve, short loadbase)
{
  short size;
  unsigned char *x, *y, *a;
  EC_CURVE_DATA *pc = (EC_CURVE_DATA *) curve_data[curve];

  if ((curve > CLUE_MAX_CURVES) || (pc == 0))
    return CRYPTO_INVALID_ARGUMENT;

  size = pc->size;
  x = (unsigned char *) pc->x;
  y = (unsigned char *) pc->y;
  a = (unsigned char *) pc->a;

  if (loadbase == 1) {
    //A2 <- x
    if (elppka_set_operand(fd, "", "A2", size, x) != CRYPTO_OK)
      return CRYPTO_FAILED;
    //B2 <- y
    if (elppka_set_operand(fd, "", "B2", size, y) != CRYPTO_OK)
      return CRYPTO_FAILED;
  }

  //A6  <- a, (curve)
  if (elppka_set_operand(fd, "", "A6", size, a) != CRYPTO_OK)
    return CRYPTO_FAILED;

  return CRYPTO_OK;
}

short clue_ec_load_curve_pdbl (int fd, unsigned short curve)
{
  short size;
  unsigned char *a;
  EC_CURVE_DATA *pc = (EC_CURVE_DATA *) curve_data[curve];
  unsigned char k[CURVE_MAX_DATA_SIZE];
  unsigned short nextpwr;

  if ((curve > CLUE_MAX_CURVES) || (pc == 0))
    return CRYPTO_INVALID_ARGUMENT;

  size = pc->size;
  a = (unsigned char *) pc->a;

  memset (k, 0, sizeof (k));
  k[size - 1] = 1;              // z=1
  //C3  <- z
  if (elppka_set_operand(fd, "", "C3", size, k) != CRYPTO_OK)
     return CRYPTO_FAILED;

  for (nextpwr = 1; nextpwr < size; nextpwr *= 2);

  k[size - 1] = 0;
  k[nextpwr - 1] = 2; // K=2
  //D7  <- k, key (2<key<order of base point)
  if (elppka_set_operand(fd, "", "D7", size, k) != CRYPTO_OK)
     return CRYPTO_FAILED;

  //A6  <- a, (curve)
  if (elppka_set_operand(fd, "", "A6", size, a) != CRYPTO_OK)
     return CRYPTO_FAILED;
  //PDUMPWORD (EDDUMP, p, size, "//a", 1);

  return CRYPTO_OK;
}

short clue_ec_load_curve_padd (int fd, unsigned short curve)
{
  short size;
  unsigned char *a;
  EC_CURVE_DATA *pc = (EC_CURVE_DATA *) curve_data[curve];

  if ((curve > CLUE_MAX_CURVES) || (pc == 0))
    return CRYPTO_INVALID_ARGUMENT;

  size = pc->size;
  a = (unsigned char *) pc->a;

  //A6  <- a, (curve)
  if (elppka_set_operand(fd, "", "A6", size, a) != CRYPTO_OK)
     return CRYPTO_FAILED;

  return CRYPTO_OK;
}

short clue_ec_load_curve_pver (int fd, unsigned short curve)
{
  short size;
  unsigned char *a, *b;
  EC_CURVE_DATA *pc = (EC_CURVE_DATA *) curve_data[curve];

  if ((curve > CLUE_MAX_CURVES) || (pc == 0))
     return CRYPTO_INVALID_ARGUMENT;

  size = pc->size;
  a = (unsigned char *) pc->a;
  b = (unsigned char *) pc->b;


  // A6 <- a,(curve)
  if (elppka_set_operand(fd, "", "A6", size, a) != CRYPTO_OK)
     return CRYPTO_FAILED;
  // A7 <- b,(curve)
  if (elppka_set_operand(fd, "", "A7", size, b) != CRYPTO_OK)
     return CRYPTO_FAILED;

  return CRYPTO_OK;
}


short clue_ec_load_curve_data (int fd, unsigned short curve)
{
  short size;
  unsigned char *m, *mp, *r;
  EC_CURVE_DATA *pc = (EC_CURVE_DATA *) curve_data[curve];

  if ((curve > CLUE_MAX_CURVES) || (pc == 0))
    return CRYPTO_INVALID_ARGUMENT;
  size = pc->size;
  m = (unsigned char *) pc->m;
  mp = (unsigned char *) pc->mp;
  r = (unsigned char *) pc->r;

  //D0  <- modulus m (p)
  if (elppka_set_operand(fd, "", "D0", size, m) != CRYPTO_OK)
     return CRYPTO_FAILED;
  //D1  <- m' ,modular inverse of m (mod R)
  if (elppka_set_operand(fd, "", "D1", size, mp) != CRYPTO_OK)
     return CRYPTO_FAILED;
  //D3  <- r_sqr_mod_m
  if (elppka_set_operand(fd, "", "D3", size, r) != CRYPTO_OK)
     return CRYPTO_FAILED;


  return CRYPTO_OK;
}

/////////////////////////////////////////////// clue ecc
//point multiplication: k*G(x,y)
//intputs:      k, key
//            x, x of G(x,y)  base point
//    y, y of G(x,y)
//output:   rx
//        ry
short clue_ec_point_mult_base (int fd, unsigned char * k, unsigned char * rx, unsigned char * ry, unsigned short size, unsigned short ksize)
{
   int rc;

   if (!rx || !ry || ksize < 2)
      return CRYPTO_INVALID_ARGUMENT;

   /* We must load key separately to support ksize != size */
   if (k) {
      if (elppka_set_operand(fd, "", "D7", ksize, k) != 0)
         return CRYPTO_FAILED;
   }

   rc = run_ecc(fd, "pmult", size, "=%A2", rx,
                                   "=%B2", ry,
                                   (char *)NULL);
   if (rc != 0)
      return CRYPTO_FAILED;

   return CRYPTO_OK;
}

// shamir's trick add rx/ry to sx/sy and store in rx/ry
short clue_ec_point_mult_shamir (int fd, unsigned char *k1, unsigned char *k2, unsigned char * rx, unsigned char * ry, unsigned char *sx, unsigned char *sy, unsigned short size, unsigned short ksize)
{
   int rc;

   if (!rx || !ry || ksize < 2)
      return CRYPTO_INVALID_ARGUMENT;

   /* We must load keys separately to support ksize != size */
   if (k1) {
      if (elppka_set_operand(fd, "", "A7", ksize, k1) != 0)
         return CRYPTO_FAILED;
   }

   if (k2) {
      if (elppka_set_operand(fd, "", "D7", ksize, k2) != 0)
         return CRYPTO_FAILED;
   }

   rc = run_ecc(fd, "shamir", size, "%A2", rx,
                                    "%B2", ry,
                                    "%A3", sx,
                                    "%B3", sy,
                                    "=%A3", rx,
                                    "=%B3", ry,
                                    (char *)NULL);
   if (rc != 0)
      return CRYPTO_FAILED;

  return CRYPTO_OK;
}

//point double: k*P(x,y), where k=2
//intputs:
//          x, x of P(x,y)
//    y, y of P(x,y)
//output: rx
//    ry
short clue_ec_point_double (int fd, unsigned char * x, unsigned char * y, unsigned char * rx, unsigned char * ry, unsigned short size)
{
   int rc;

   if (!x || !y || !rx || !ry)
      return CRYPTO_INVALID_ARGUMENT;

   rc = run_ecc(fd, "pdbl", size, "%A3", x,
                                  "%B3", y,
                                  "=%A2", rx,
                                  "=%B2", ry,
                                  (char *)NULL);
   if (rc != 0)
      return CRYPTO_FAILED;

   return CRYPTO_OK;
}

//point addition: // P(x1,y1)+Q(x2,y2), where k=1
//intputs:
//          x1, x1 of P(x1,y1)
//    y1, y1 of P(x1,y1)
//          x2, x2 of Q(x2,y2)
//    y2, y2 of Q(x2,y2)
//output: rx
//    ry
short clue_ec_point_add (int fd, unsigned char * x1, unsigned char * y1, unsigned char * x2, unsigned char * y2, unsigned char * rx, unsigned char * ry,
                       unsigned short size)
{
  int rc;

   if (!x1 || !y1 || !x2 || !y2 || !rx || !ry)
      return CRYPTO_INVALID_ARGUMENT;

   rc = run_ecc(fd, "padd", size, "%A2", x1,
                                  "%B2", y1,
                                  "%A3", x2,
                                  "%B3", y2,
                                  "=%A2", rx,
                                  "=%B2", ry,
                                  (char *)NULL);
   if (rc != 0)
      return CRYPTO_FAILED;

   return CRYPTO_OK;
}

//point verify: verify if P(x,y) is on EC y^2=x^2 + ax + b
//intputs:
//        x, x of P(x,y)
//        y, y of P(x,y)
//output: zero flag set if lhs=rhs
short clue_ec_point_verify (int fd, unsigned char * x, unsigned char * y, unsigned short size)
{
   int rc;

   if (!x || !y)
      return CRYPTO_INVALID_ARGUMENT;

   rc = run_ecc(fd, "pver", size, "%A2", x,
                                  "%B2", y,
                                  (char *)NULL);
   if (rc != 0)
      return CRYPTO_FAILED;

   rc = elppka_test_flag(fd, "", "Z");
   if (rc < 0) {
      pka_tool_err(0, "pver: error reading result");
      return CRYPTO_FAILED;
   }

   return rc ? CRYPTO_OK : CRYPTO_FAILED;
}
