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

#include <stdint.h>
#include <pkadev.h>
#include "common.h"
#include "elpcrypto.h"
#ifndef CLUE_PRECOMPUTE
#include <math.h>
#endif
#ifdef DO_RSA

#ifndef CLUE_PRECOMPUTE
void dump_mp_int_to_char (mp_int * b, int max, char *c);
#endif



//init memory map


//software gcd check 
static short
clue_bn_denominator_gcd_check (unsigned char * denominator, unsigned char * modulus, unsigned short size)
{
#ifdef CLUE_PRECOMPUTE
#warning CLUE_PRECOMPUTE disables gcd check (removes mp lib)
  return CRYPTO_OK;
#else
  unsigned char val[512];
  short err;
  mp_int b, m, g;

  if ((denominator == 0) || (modulus == 0)) {
    err = CRYPTO_INVALID_ARGUMENT;
  } else if ((size == 0) || (size > BN_RSA_BASE_RADIX_SIZE)) {
    err = CRYPTO_INVALID_ARGUMENT;
  } else {
    if (mp_init_multi (&b, &m, &g, NULL) != CRYPTO_OK) {
      err = CRYPTO_FAILED;
    }
    MEMCPY32EX ((uint32_t *) val, (uint32_t *) denominator, size >> 2);
    if ((err = mp_read_unsigned_bin (&b, (unsigned char *) val, size)) != CRYPTO_OK) {
      err = CRYPTO_FAILED;
    }
    MEMCPY32EX ((uint32_t *) val, (uint32_t *) modulus, size >> 2);
    if ((err = mp_read_unsigned_bin (&m, (unsigned char *) val, size)) != CRYPTO_OK) {
      err = CRYPTO_FAILED;
    }
    if ((err = mp_gcd (&b, &m, &g)) != CRYPTO_OK) {
      err = CRYPTO_FAILED;
    }
    if (mp_cmp_d (&g, 1) == MP_EQ) {
      err = CRYPTO_OK;
    } else {
      err = CRYPTO_FAILED;
    }
    mp_clear_multi (&b, &m, &g, NULL);
    err = CRYPTO_OK;
  }
  return err;
#endif
}

//get len in bits
//reminder input v will be destoryed.
#ifndef CLUE_PRECOMPUTE
static short ff1 (mp_int * v)
{
  int k;
  //if( v == 1 )
  if (mp_cmp_d (v, 1) == LTC_MP_EQ) {
    return (0);
  } else {
    k = 1;
    //while( v > 1 )
    while (mp_cmp_d (v, 1) == LTC_MP_GT) {
      //v /= 2;
      if (mp_div_2 (v, v) != CRYPTO_OK) {
        return (0);
      }
      k++;
    }
    return k;
  }
}
#endif

//precompute
// input: n, size (modulus and size)
// output:
// load mp, rr to hardware

//      mp (m_prime = (r*r^-1)/m )
//      rr (r_sqr_mod_m = r*r mod m)

static short clue_bn_precompute(int fd, const unsigned char * n, unsigned short size)
{
   short err = CRYPTO_OK;
   char val[512];
   int rc;

#ifdef CLUE_PRECOMPUTE
   /*
    * puts 1/r mod m into C0, which is where it should be for
    * the mp and r_sqr operations, but is overwritten in the mp
    * one, so we have to save it so we can use it again in the r_sqr
    */
   rc = pka_tool_run(fd, "calc_r_inv", size, "%D0", n,
                                             "=%C0", val,
                                             (char *)NULL);
   if (rc != 0)
      return CRYPTO_FAILED;

   /* modulus is already in D0, 1/r mod m is already in C0 */
   rc = pka_tool_run(fd, "calc_mp", size, (char *)NULL);
   if (rc != 0)
      return CRYPTO_FAILED;

   /*
    * modulus is already in D0, 1/r mod m was overwritten in C0,
    * mp is already in D1.
    */
   rc = pka_tool_run(fd, "calc_r_sqr", size, "%C0", val,
                                             (char *)NULL);
   if (rc != 0)
      return CRYPTO_FAILED;

   return CRYPTO_OK;
#else
  mp_int m, mp, r, rp, rr;
  int i;

  if (n == 0) {
    err = CRYPTO_INVALID_ARGUMENT;
  } else if ((size == 0) || (size > BN_RSA_BASE_RADIX_SIZE)) {
    err = CRYPTO_INVALID_ARGUMENT;
  } else {
    //init bignum
    if (mp_init_multi (&m, &mp, &r, &rp, &rr, NULL) != CRYPTO_OK) {
      err = CRYPTO_FAILED;
    }

    // m comes in big endian already so pass it as is
    MEMCPY32 ((uint32_t *) val, (uint32_t *) n, size >> 2);
    PDUMPWORD (EDDUMP, val, size, "//m (precompute)", 1);
    if (mp_read_unsigned_bin (&m, (unsigned char *) val, size) != CRYPTO_OK) {
      err = CRYPTO_FAILED;
    }


    if (mp_copy (&m, &r) != CRYPTO_OK) {
      err = CRYPTO_FAILED;
    }

    i = (ff1 (&r));
    //(r = 1^1024; 1024=0x400), fox emample sz=128bytes=1024bit
    if (mp_2expt (&r, i) != CRYPTO_OK) {
      err = CRYPTO_FAILED;
    }

    //rr----r_sqr_mod_m     r_sqr_mod_m = mod( r^2, m );
    //calculate: r = 2^(ff1( m ) + 1);  r_sqr_mod_m = mod( r^2, m );
    if (mp_sqrmod (&r, &m, &rr) != CRYPTO_OK) {
      err = CRYPTO_FAILED;
    }
    //rr
    //mp_int_word_dump (&rr,size);
    //mp_to_unsigned_bin(&rr, (unsigned char *)val);

    dump_mp_int_to_char (&rr, size, val);
    if (elppka_set_operand(fd, "", "D3", size, rr) != CRYPTO_OK)
      return CRYPTO_FAILED;

    //r_inv
    if (mp_invmod (&r, &m, &rp) != CRYPTO_OK) {
      err = CRYPTO_FAILED;
    }
    //mp----m'   m_prime = ((r * r_inv) - 1)/m;
    //calculate: r = 2^(ff1( m ) + 1);  r_inv = inv( r, m );  m_prime = ((r * r_inv) - 1)/m;
    if (mp_mul (&r, &rp, &rp) != CRYPTO_OK) {
      err = CRYPTO_FAILED;
    }
    if (mp_sub_d (&rp, 1, &rp) != CRYPTO_OK) {
      err = CRYPTO_FAILED;
    }
    //mp
    if (mp_div (&rp, &m, &mp, NULL) != CRYPTO_OK) {
      err = CRYPTO_FAILED;
    }
    //mp_int_word_dump (&mp,size);
    //mp_to_unsigned_bin(&mp, (unsigned char *)val);

    dump_mp_int_to_char (&mp, size, val);
    if (elppka_set_operand(fd, "", "D1", size, val) != CRYPTO_OK)
      return CRYPTO_FAILED;

    mp_clear_multi (&m, &mp, &r, &rp, &rr, NULL);
  }
#endif
  return err;
}


//mod exp:  c=a^e mod m
//intputs:  a (base)
//          e (exponent applied to base)
//          m (modulus)
//output: c

//precompute
//          mp (m_prime = (r*r^-1)/m )
//          rr (r_sqr_mod_m = r*r mod m)
//algorithm:
//      mp (m_prime = (r*r^-1)/m )
//      rr (r_sqr_mod_m = r*r mod m)
//  mp----m'   m_prime = ((r * r_inv) - 1)/m;
//  calculate: r_inv = inv( r, m ); m_prime = ((r * r_inv) - 1)/m;
//  rr----r_sqr_mod_m     r_sqr_mod_m = mod( r^2, m );
//  calculate: r = 2^(ff1( m ) + 1); r_sqr_mod_m = mod( r^2, m );
//  mod_exp  = mpower( base, exp, m );
// The precomp parameter forces precompute values to be loaded in the
// CLUE hardware. This whould be done only one time when a new key is used.
       
// a -base
// b - m'
// d - r^2 mod m
// e - exponent
short clue_bn_modexp (int fd, unsigned char * a, unsigned char * e, unsigned char * m, unsigned char * c, unsigned short size, short precomp)
{
   int rc;

   if ((a == 0) || (e == 0) || (c == 0) || (m == 0))
      return CRYPTO_INVALID_ARGUMENT;

   if ((size == 0) || (size > BN_RSA_BASE_RADIX_SIZE))
      return CRYPTO_INVALID_ARGUMENT;

   if (precomp == 1) {
      if (clue_bn_precompute (fd, m, size) != CRYPTO_OK)
         return CRYPTO_FAILED;
   }

   rc = pka_tool_run(fd, "modexp", size, "%A0",  a,
                                         "%D2",  e,
                                         "%D0",  m,
                                         "=%A0", c,
                                         (char *)NULL);
   if (rc != 0)
      return CRYPTO_FAILED;

   return CRYPTO_OK;
}

//modular exponentiation using CRT
// p, q - prime factors of modulus
// d    - exponent (private key)
// m    - message
// c    - ciphertext
int clue_crt_modexp(int fd, const unsigned char *p, const unsigned char *q,
                            const unsigned char *d, const unsigned char *m,
                            unsigned char *c, unsigned size)
{
   unsigned char p_mont[2][256], q_mont[2][256];
   unsigned opsize = size >> 1;
   int rc;

   /* Precompute! */
   rc = clue_bn_precompute(fd, p, opsize);
   if (rc != CRYPTO_OK)
      return rc;

   if (elppka_get_operand(fd, NULL, "D1", opsize, p_mont[0]) != 0)
      return CRYPTO_FAILED;
   if (elppka_get_operand(fd, NULL, "D3", opsize, p_mont[1]) != 0)
      return CRYPTO_FAILED;

   rc = clue_bn_precompute(fd, q, opsize);
   if (rc != CRYPTO_OK)
      return rc;

   if (elppka_get_operand(fd, NULL, "D1", opsize, q_mont[0]) != 0)
      return CRYPTO_FAILED;
   if (elppka_get_operand(fd, NULL, "D3", opsize, q_mont[1]) != 0)
      return CRYPTO_FAILED;

   rc = elppka_run(fd, "crt_key_setup", opsize,
                   "%B1", q, "%D0", p,
                   "%D3", d, "%D1", d+opsize,
                   (char *)NULL);
   if (rc != 0)
      return CRYPTO_FAILED;

   /* Shuffle CRT parameters to the appropriate locations. */
   if (elppka_copy_operand(fd, NULL, "C3", NULL, "B1", opsize) != 0)
      return CRYPTO_FAILED;
   if (elppka_copy_operand(fd, NULL, "D2", NULL, "C0", opsize) != 0)
      return CRYPTO_FAILED;
   if (elppka_copy_operand(fd, NULL, "C2", NULL, "A3", opsize) != 0)
      return CRYPTO_FAILED;

   rc = elppka_run(fd, "crt", opsize,
                       "%A3", m, "%A2", m+opsize,
                       "%B2", p, "%D4", p_mont[0], "%D5", p_mont[1],
                       "%B3", q, "%D6", q_mont[0], "%D3", q_mont[1],
                       "=%A1", c, "=%A0", c+opsize,
                       (char *)NULL);
   if (rc != 0)
      return CRYPTO_FAILED;

   return CRYPTO_OK;
}

//modular multiplication: c=a*b mod m
//intputs:      a
//          b
//    m (modulus)
//output: c

//precompute
//          mp (m_prime = (r*r^-1)/m )
//          rr (r_sqr_mod_m = r*r mod m)
// The precomp parameter forces precompute values to be loaded in the
// CLUE hardware. This whould be done only one time when a new key is used.
short clue_bn_modmult (int fd, unsigned char * a, unsigned char * b, unsigned char * m, unsigned char * c, unsigned short size, short precomp)
{
   int rc;

   if ((a == 0) || (b == 0) || (c == 0) || (m == 0))
      return CRYPTO_INVALID_ARGUMENT;

   if ((size == 0) || (size > BN_RSA_BASE_RADIX_SIZE))
      return CRYPTO_INVALID_ARGUMENT;

   if (precomp == 1) {
      if (clue_bn_precompute (fd, m, size) != CRYPTO_OK)
         return CRYPTO_FAILED;
   }

   rc = pka_tool_run(fd, "modmult", size, "%A0",  a,
                                          "%B0",  b,
                                          "%D0",  m,
                                          "=%A0", c,
                                          (char *)NULL);
   if (rc != 0)
      return CRYPTO_FAILED;

   return CRYPTO_OK;
}


//modular division: c=a/b mod m
//intputs:  a
//    b
//    m (modulus)
//output: c
short clue_bn_moddiv (int fd, unsigned char * a, unsigned char * b, unsigned char * m, unsigned char * c, unsigned short size)
{
  short ret = CRYPTO_OK;

  if ((a == 0) || (b == 0) || (c == 0) || (m == 0)) {
    ret = CRYPTO_INVALID_ARGUMENT;
  } else if ((size == 0) || (size > BN_RSA_BASE_RADIX_SIZE)) {
    ret = CRYPTO_INVALID_ARGUMENT;
  } else if (clue_bn_denominator_gcd_check (b, m, size) != CRYPTO_OK) {
    ret = CRYPTO_INVALID_ARGUMENT;
  }
  else {
    if (m != 0) {
       if (elppka_run(fd, "moddiv", size,
                  "%C0",  a,
                  "%A0",  b,
                  "%D0",  m,
                  "=%C0", c,
                  (char *)NULL) != CRYPTO_OK) {
         ret = CRYPTO_FAILED;
      }
    }
    else {
       if (elppka_run(fd, "moddiv", size,
                  "%C0",  a,
                  "%A0",  b,
                  "=%C0", c,
                  (char *)NULL) != CRYPTO_OK) {
         ret = CRYPTO_FAILED;
      }
    }
  }
  return ret;
}

//modular inversion: c=1/b mod m
//intputs:  a=1
//    b
//    m (modulus)
//output: c
short clue_bn_modinv (int fd, unsigned char * b, unsigned char * m, unsigned char * c, unsigned short size)
{
  short ret = CRYPTO_OK;

  if ((b == 0) || (c == 0) || (m == 0)) {
    ret = CRYPTO_INVALID_ARGUMENT;
  } else if ((size == 0) || (size > BN_RSA_BASE_RADIX_SIZE)) {
    ret = CRYPTO_INVALID_ARGUMENT;
  } else if (clue_bn_denominator_gcd_check (b, m, size) != CRYPTO_OK) {
    ret = CRYPTO_INVALID_ARGUMENT;
  } else {
     if (elppka_run(fd, "modinv", size,
                        "%A0",  b,
                        "%D0",  m,
                        "=%C0", c,
                        (char *)NULL) != CRYPTO_OK) {
         ret = CRYPTO_FAILED;
     }
  }
  return ret;
}


//modular addition:   c=a+b mod m
//intputs:      
//    a
//          b
//          m (modulus)
//output:    
//    c
short clue_bn_modadd (int fd, unsigned char * a, unsigned char * b, unsigned char * m, unsigned char * c, unsigned short size)
{
  short ret = CRYPTO_OK;

  if ((a == 0) || (b == 0) || (c == 0) || (m == 0)) {
    ret = CRYPTO_INVALID_ARGUMENT;
  } else if ((size == 0) || (size > BN_RSA_BASE_RADIX_SIZE)) {
    ret = CRYPTO_INVALID_ARGUMENT;
  }
  else {
    if (m != 0) {
       if (elppka_run(fd, "modadd", size,
                  "%A0",  a,
                  "%B0",  b,
                  "%D0",  m,
                  "=%A0", c,
                  (char *)NULL) != CRYPTO_OK) {
         ret = CRYPTO_FAILED;
      }
    }
    else {
       if (elppka_run(fd, "modadd", size,
                  "%A0",  a,
                  "%B0",  b,
                  "=%A0", c,
                  (char *)NULL) != CRYPTO_OK) {
         ret = CRYPTO_FAILED;
       }
    }
  }
  return ret;
}


//modular subtraction:    c=a-b mod m
//intputs:      a
//          b
//          m (modulus)
//output: c
short clue_bn_modsub (int fd, unsigned char * a, unsigned char * b, unsigned char * m, unsigned char * c, unsigned short size)
{
  short ret = CRYPTO_OK;

  if ((a == 0) || (b == 0) || (c == 0) || (m == 0)) {
    ret = CRYPTO_INVALID_ARGUMENT;
  } else if ((size == 0) || (size > BN_RSA_BASE_RADIX_SIZE)) {
    ret = CRYPTO_INVALID_ARGUMENT;
  }
  else {
    if (m != 0) {
      if (elppka_run(fd, "modsub", size,
                  "%A0",  a,
                  "%B0",  b,
                  "%D0",  m,
                  "=%A0", c,
                  (char *)NULL) != CRYPTO_OK) {
         ret = CRYPTO_FAILED;
      }
    }
    else {
      if (elppka_run(fd, "modsub", size,
                  "%A0",  a,
                  "%B0",  b,
                  "=%A0", c,
                  (char *)NULL) != CRYPTO_OK) {
         ret = CRYPTO_FAILED;
      }
    }
  }
  return ret;
}


//modular reduction:    c=a mod m
//intputs:      a
//          m (modulus)
//output: c
short clue_bn_modred (int fd, unsigned char * a, unsigned char * m, unsigned char * c, unsigned short size)
{
  short ret = CRYPTO_OK;

  if ((a == 0) || (c == 0) || (m == 0)) {
    ret = CRYPTO_INVALID_ARGUMENT;
  } else if ((size == 0) || (size > BN_RSA_BASE_RADIX_SIZE)) {
    ret = CRYPTO_INVALID_ARGUMENT;
  }
  else {
    if (m != 0) {
       if (elppka_run(fd, "reduce", size,
                  "%C0",  a,
                  "%D0",  m,
                  "=%A0", c,
                  (char *)NULL) != CRYPTO_OK) {
          ret = CRYPTO_FAILED;
       }
    }
    else {
       if (elppka_run(fd, "reduce", size,
                  "%A0",  a,
                  "=%A0", c,
                  (char *)NULL) != CRYPTO_OK) {
         ret = CRYPTO_FAILED;
      }
    }
  }
  return ret;
}

#endif /* DO_RSA */
