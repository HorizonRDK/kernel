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
 * Copyright (c) 2018 Synopsys, Inc. and/or its affiliates.
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
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>

#include "elpspaccusr.h"

#define MAX_ENC_LEN 5; // max number of bytes to hold length encoding

static int left_encode( uint8_t *buf, uint8_t *buflen, uint32_t value )
{
   uint8_t i = 1;
   uint8_t size;

   while ((i < sizeof(value)) && (value >> (8 * i) > 0)) i++;

   size = i + 1;
   if (size >= *buflen) {
      return 1;
   } else {
      buf[0] = i;
      *buflen = size;
      while (i > 0) {
         buf[i] = value & 0xFF;
         value >>= 8;
         i--;
      }
   }
   return 0;
}

static int right_encode( uint8_t *buf, uint8_t *buflen, uint32_t value )
{
   uint8_t i = 1;
   uint8_t size;

   while ((i < sizeof(value)) && (value >> (8 * i) > 0)) i++;

   size = i + 1;
   if (size >= *buflen) {
      return 1;
   } else {
      buf[i] = i;
      *buflen = size;
      while (i > 0) {
         i--;
         buf[i] = value & 0xFF;
         value = value >> 8;
      }
   }
   return 0;
}

int tuplehash_demo(void)
{
   struct elp_spacc_usr fd;
   int x, i;
   unsigned char src[128], dst[128], sbuf[64], nbuf[64];

   static const struct {
      char    data[64];
      uint8_t len;
   } tuples[] = {
      {{0x00, 0x01, 0x02}, 3},
      {{0x10, 0x11, 0x12, 0x13, 0x14, 0x15}, 6},
      {{0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28}, 9},
   };
   char S_string[] = "My Tuple App";
   char N_string[] = "TupleHash"; // This is fixed for TupleHash function
   uint8_t slen, nlen, enclen, srclen;
   int icvlen = 32;

   // Prep inputs, assuming here that buffers are large enough for the inputs
   enclen = MAX_ENC_LEN; // limit encode to 5 chars
   left_encode(sbuf, &enclen, strlen(S_string) * 8);
   memcpy(sbuf + enclen, S_string, strlen(S_string));
   slen = enclen + strlen(S_string);

   enclen = MAX_ENC_LEN;
   left_encode(nbuf, &enclen, strlen(N_string) * 8);
   memcpy(nbuf + enclen, N_string, strlen(N_string));
   nlen = enclen + strlen(N_string);

   srclen = 0;
   // Insert the tuples as left_encode(len)||tuple...
   for (i = 0; i < (sizeof tuples/sizeof tuples[0]); i++) {
      enclen = MAX_ENC_LEN;
      left_encode(src + srclen, &enclen, tuples[i].len * 8);
      srclen += enclen;
      memcpy(src + srclen, tuples[i].data, tuples[i].len);
      srclen += tuples[i].len;
   }

   // Add right_encode(L)
   enclen = MAX_ENC_LEN;
   right_encode(src + srclen, &enclen, icvlen * 8);
   srclen += enclen;

   // Open the spacc for cshake128 hashing
   x = spacc_dev_open(&fd, CRYPTO_MODE_NULL, CRYPTO_MODE_HASH_CSHAKE128, 1, ICV_HASH, icvlen, 0, NULL, 0, NULL, 0, nbuf, nlen, sbuf, slen);
   assert(!x);

   // Run the hash process
   spacc_dev_process(&fd, NULL, -1, 0, 0, 0, 0, src, srclen, dst, sizeof dst);

   // Dump the input and the resulting hash
   printf("SRC =\n");
   for (i = 0; i < srclen; i++) {
      printf("%02X ", src[i]);
      if ((i % 16) == 15) printf("\n");
   }
   printf("\n");
   printf("DST =\n");
   for (i = 0; i < 32; i++) {
      printf("%02X ", dst[srclen + i]);
      if ((i % 16) == 15) printf("\n");
   }
   printf("\n");

   // Close/unmap buffers
   spacc_dev_close(&fd);

   return 0;
}

int parallelhash_demo(void)
{
   struct elp_spacc_usr fd;
   int x, i, blocks;
   unsigned char src[256], dst[256], sbuf[64], nbuf[64];
   uint8_t srclen, slen, nlen, enclen;

   unsigned char indata[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                             0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                             0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27};
   int blocksize = 8;
   char S_string[] = "Parallel Data";
   char N_string[] = "ParallelHash"; // This is fixed for ParallelHash function
   int icvlen = 32;

   // Open spacc for shake128 hashing
   x = spacc_dev_open(&fd, CRYPTO_MODE_NULL, CRYPTO_MODE_HASH_SHAKE128, 1, ICV_HASH, icvlen, 0, NULL, 0, NULL, 0, NULL, 0, NULL, 0);
   assert(!x);

   // Build up the cshake input in src
   srclen = 0;
   enclen = MAX_ENC_LEN;
   left_encode(src, &enclen, blocksize);
   srclen += enclen;

   // Hash blocks and push the result into src
   // For a very large input and large blocks, this is the work which could be done in parallel,
   // producing a string of hashes which would then be hashed to produce the final unique result
   blocks = (sizeof indata) / blocksize;
   for (i = 0; i < blocks; i++) {
      spacc_dev_process(&fd, NULL, -1, 0, 0, 0, 0, indata + i * blocksize, blocksize, dst, sizeof dst);
      memcpy(src + srclen, dst + blocksize, icvlen);
      srclen += icvlen;
   }

   // Build up the cshake input in src
   enclen = MAX_ENC_LEN;
   right_encode(src + srclen, &enclen, blocks);
   srclen += enclen;

   enclen = MAX_ENC_LEN;
   right_encode(src + srclen, &enclen, icvlen * 8);
   srclen += enclen;

   // Close/unmap buffers
   spacc_dev_close(&fd);

   // Prep string inputs, assuming here that buffers are large enough for the inputs
   enclen = MAX_ENC_LEN; // limit encode to 5 chars
   left_encode(sbuf, &enclen, strlen(S_string) * 8);
   memcpy(sbuf + enclen, S_string, strlen(S_string));
   slen = enclen + strlen(S_string);

   enclen = MAX_ENC_LEN;
   left_encode(nbuf, &enclen, strlen(N_string) * 8);
   memcpy(nbuf + enclen, N_string, strlen(N_string));
   nlen = enclen + strlen(N_string);

   // Open spacc for cshake128 hash
   x = spacc_dev_open(&fd, CRYPTO_MODE_NULL, CRYPTO_MODE_HASH_CSHAKE128, 1, ICV_HASH, icvlen, 0, NULL, 0, NULL, 0, nbuf, nlen, sbuf, slen);
   assert(!x);

   // Run the hash process
   spacc_dev_process(&fd, NULL, -1, 0, 0, 0, 0, src, srclen, dst, sizeof dst);

   // Dump the input and the resulting hash
   printf("SRC =\n");
   for (i = 0; i < srclen; i++) {
      printf("%02X ", src[i]);
      if ((i % 16) == 15) printf("\n");
   }
   printf("\n");
   printf("DST =\n");
   for (i = 0; i < 32; i++) {
      printf("%02X ", dst[srclen + i]);
      if ((i % 16) == 15) printf("\n");
   }
   printf("\n");

   // Close/unmap buffers
   spacc_dev_close(&fd);

   return 0;
}

// Demonstrates the use of the XOF modes (SHAKE, CSHAKE) of the SPACC
// to run TupleHash and ParallelHash from NIST SP800-185.
int main(int argc, char **argv)
{
   struct elp_spacc_features features;
   int x;

   x = spacc_dev_features(&features);
   assert(x == 0);
   printf("SPAcc userspace driver bound to SPAcc EPN-%04x ver.%02x, max msg size %u, partial support %s, qos %s, iv %s\n\n", features.project, features.version, features.max_msg_size, features.partial ? "on" : "off", features.qos ? "on" : "off", features.ivimport ? "on" : "off");

   printf("Tuple Hash\n");
   tuplehash_demo();
   printf("Parallel Hash\n");
   parallelhash_demo();

   return 0;
}
