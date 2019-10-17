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
 * Copyright (c) 2012-2017 Synopsys, Inc. and/or its affiliates.
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
#include <pthread.h>

#include "elpspaccusr.h"

static struct elp_spacc_features features;


int cipher_only(void)
{
   unsigned char src[128], dst[128], src2[128], dst2[128];
   struct elp_spacc_usr fd, fd2;
   int x;
   unsigned char cipher_only[32] = {
   0x0c,0x47,0x28,0x35,0xcf,0x8b,0xba,0xf7,0xcb,0x9b,0xc4,0x3b,0xbb,0xdc,0xa8,0xc4,
   0x6c,0x5a,0x99,0x2c,0x61,0x46,0xca,0x47,0xd2,0xb3,0x78,0x6a,0xb0,0x89,0x03,0x86, };

   // allocate a handle for AES-128-CBC
   x = spacc_dev_open(&fd, CRYPTO_MODE_AES_CBC, CRYPTO_MODE_NULL, 1, 0, 0, 0, "1234567890ABCDEF", 16, "43218765A098FEDC", 16, "", 0, "", 0);
   assert(x >= 0);

   // 32 bytes of plaintext
   strcpy((char *)src, "abcdefghijklmnopqrstuvwxyz01234");

   // encrypt it, should return length of output
   memset(dst, 0, sizeof dst);
   x = spacc_dev_process(&fd, NULL, -1, 0, 0, 0, 0, src, 32, dst, sizeof dst);
   printf("CIPHER ENCRYPT: return code (32==OK) == %d\n", x);
   if (x != 32 || memcmp(dst, cipher_only, x)) {
      printf("Failed to match cipher_only test vector\n");
      for (x = 0; x  < 32 ; x++ ) { printf("%02x ", dst[x]); } printf("\n");
      exit(-1);
   }

   if (features.ivimport) {
      // encrypt it with IV import, should return length of output
      memcpy(src+32, "43218765A098FEDC", 16); // copy IV

      // we instruct it to load a new IV to ensure that the import is being used
      x = spacc_dev_process(&fd, "ffffaaaakkkkeeee", 32, 0, 0, 0, 0, src, 32, dst, sizeof dst);
      printf("CIPHER ENCRYPT IV IMPORT: return code (32==OK) == %d\n", x);
      if (x != 32 || memcmp(dst, cipher_only, x)) {
         printf("Failed to match cipher_only test vector\n");
         exit(-1);
      }

      // we instruct it to load a new IV to ensure that this works
      memmove(src+16,src,32);
      x = spacc_dev_process(&fd, "43218765A098FEDC", -1, 0, 0, 16, 0, src, 32, dst, sizeof dst);
      printf("CIPHER ENCRYPT SRC OFFSET: return code (32==OK) == %d\n", x);
      if (x != 32 || memcmp(dst, cipher_only, x)) {
         printf("Failed to match cipher_only test vector\n");
         exit(-1);
      }
      memmove(src, src+16, 32);
   } else {
      printf("IV import disabled cannot test\n");
   }

   // open another handle for the same key but in decrypt mode this time
   x = spacc_dev_open(&fd2, CRYPTO_MODE_AES_CBC, CRYPTO_MODE_NULL, 0, 0, 0, 0, "1234567890ABCDEF", 16, "43218765A098FEDC", 16, "", 0, "", 0);
   assert(x == 0);

   // copy ciphertext to new src DMA buffer
   memcpy(src2, dst, 32);

   // decrypt 32 plaintext
   x = spacc_dev_process(&fd2, NULL, -1, 0, 0, 0, 0, src2, 32, dst2, sizeof dst2);

   printf("CIPHER DECRYPT: return code (32==OK) == %d\n", x);
   if (x != 32 || memcmp(dst2, src, 32)) {
      printf("Failed to decrypt\n");
      exit(-1);
   }

   // close/unmap buffers
   spacc_dev_close(&fd);
   spacc_dev_close(&fd2);

   return 0;
}

void run_more_thorough_partial_test(int algo)
{
   struct elp_spacc_usr fd;
   unsigned x, y, z, outsize;
   unsigned char dst[32], src[128];
   unsigned char hash[2][13][20] =
   {
      {
         { 0x6a, 0xa6, 0x52, 0x02, 0x3b, 0xdb, 0xcb, 0x0d, 0x35, 0x09, 0xc5, 0x6d, 0xc5, 0xc6, 0xc6, 0x70,  }, // these are the HMACs of 2**(8+x) - 1 a's
         { 0x4f, 0x13, 0x4b, 0x49, 0x47, 0xaa, 0xcf, 0x10, 0xae, 0x55, 0x87, 0x10, 0xc1, 0x7c, 0xb7, 0xaa,  },
         { 0x6a, 0x67, 0xb6, 0x61, 0x19, 0x8d, 0x80, 0xf7, 0xc3, 0xc9, 0x2d, 0xcd, 0xb6, 0x37, 0x46, 0x54,  },
         { 0xf2, 0x7f, 0xb0, 0x5f, 0xd8, 0xd0, 0x72, 0xfb, 0x4f, 0x31, 0xb7, 0x35, 0x95, 0x66, 0x52, 0x9a,  },
         { 0xa4, 0x7d, 0x5b, 0xb7, 0x83, 0x26, 0x3b, 0x4f, 0x2a, 0xd4, 0xb8, 0x75, 0x2c, 0x15, 0x97, 0xa2,  },
         { 0x28, 0x30, 0x9d, 0x9a, 0x80, 0x5d, 0x14, 0xa3, 0x65, 0x5d, 0x6b, 0xbf, 0xb0, 0x76, 0xd9, 0xfa,  },
         { 0x3c, 0xca, 0xbc, 0x11, 0x78, 0x1c, 0x88, 0x47, 0x79, 0x94, 0x38, 0x11, 0x94, 0x10, 0x1b, 0x8b,  },
         { 0xa1, 0x69, 0xc3, 0x19, 0x89, 0xfa, 0x19, 0xdf, 0x6a, 0xe0, 0x71, 0xdd, 0xf3, 0xcc, 0xd9, 0xf8,  },
         { 0xab, 0x05, 0x5a, 0xf4, 0x11, 0x23, 0xb9, 0xe7, 0xb3, 0xcf, 0x71, 0x6b, 0xc4, 0x18, 0x97, 0xcc,  },
         { 0x4e, 0x00, 0x26, 0x8f, 0xd9, 0x83, 0xe7, 0x20, 0x7c, 0x7a, 0xd3, 0x21, 0xf8, 0x5d, 0xca, 0x16,  },
         { 0x71, 0x25, 0xc0, 0x04, 0x59, 0xdf, 0x68, 0x28, 0x32, 0x05, 0x2c, 0xaa, 0x5b, 0x62, 0x4d, 0xc1,  },
         { 0x44, 0x71, 0x33, 0xe3, 0xb7, 0xba, 0x9c, 0x1a, 0x66, 0x2b, 0xd0, 0x94, 0x67, 0xb2, 0xeb, 0x3e,  },
         { 0x5e, 0x0a, 0x1a, 0xcf, 0x36, 0xf4, 0xd7, 0x2b, 0x93, 0xda, 0x3b, 0x59, 0x95, 0xeb, 0x63, 0xe9,  },
      },
      {
         { 0x16, 0xc3, 0x24, 0x96, 0x50, 0x23, 0x83, 0x61, 0xa4, 0x41, 0x5f, 0xa4, 0xc1, 0x00, 0x2c, 0x14, 0xc8, 0x2e, 0xe1, 0x11,  }, // SHA1 HMACs...
         { 0x47, 0x6e, 0x02, 0x66, 0x1a, 0xe8, 0xb4, 0x38, 0xe5, 0xd3, 0xf2, 0x71, 0x88, 0xa9, 0x51, 0x8c, 0x61, 0x2a, 0xb7, 0x0f,  },
         { 0xdd, 0x5b, 0x7b, 0xcf, 0x96, 0xcf, 0x08, 0x3c, 0x8e, 0x71, 0x67, 0x10, 0x9b, 0x71, 0x24, 0x7c, 0x51, 0xd6, 0x90, 0x06,  },
         { 0x2f, 0x84, 0x13, 0x26, 0x89, 0x77, 0xa8, 0x6f, 0xa7, 0x0f, 0x67, 0x9e, 0x94, 0x5e, 0x13, 0xc0, 0x89, 0xed, 0x93, 0x84,  },
         { 0x85, 0xd2, 0xd9, 0xd8, 0x4d, 0x5c, 0xce, 0xe8, 0xfe, 0xed, 0xc8, 0xbe, 0x8f, 0xd2, 0x6e, 0x9a, 0x3c, 0x15, 0xd1, 0x1f,  },
         { 0x76, 0x88, 0x0a, 0xd4, 0xcb, 0x5b, 0x1f, 0x94, 0x4a, 0x11, 0x99, 0x26, 0x41, 0xf6, 0x18, 0x79, 0x23, 0xed, 0x00, 0xc8,  },
         { 0x01, 0xd6, 0x16, 0x57, 0x6c, 0x88, 0x8c, 0x79, 0x2c, 0x24, 0x12, 0xbc, 0x32, 0x8a, 0x6e, 0x1d, 0x18, 0xfe, 0xcc, 0x7e,  },
         { 0xa1, 0x2a, 0x9e, 0xab, 0x33, 0xb4, 0x3e, 0xff, 0xb8, 0x5f, 0xdc, 0x84, 0xcc, 0xba, 0x53, 0xe3, 0xad, 0xad, 0xd4, 0x1b,  },
         { 0x8e, 0xd5, 0xfb, 0x4f, 0x1c, 0x19, 0x8c, 0xde, 0xfd, 0x6d, 0xc8, 0x20, 0x3f, 0xa6, 0x38, 0x60, 0x47, 0x27, 0xf2, 0x34,  },
         { 0x0b, 0x4c, 0xfd, 0x8b, 0xc7, 0xf2, 0xb5, 0x39, 0x5e, 0x66, 0xeb, 0xf9, 0x2b, 0x10, 0x34, 0x99, 0x56, 0x35, 0xd0, 0x7a,  },
         { 0x1c, 0xd8, 0x11, 0x60, 0x88, 0x7c, 0xf9, 0xcf, 0x28, 0x27, 0x1b, 0xc5, 0xa9, 0x40, 0xa8, 0xed, 0xa3, 0x6f, 0x92, 0xe4,  },
         { 0x8b, 0x6e, 0x97, 0x3a, 0x6f, 0xd5, 0x55, 0x9c, 0xeb, 0x02, 0x83, 0xe8, 0x0a, 0x2b, 0x47, 0x3a, 0x44, 0x09, 0xc6, 0x2e,  },
         { 0xfb, 0xbf, 0x53, 0x23, 0xba, 0x61, 0x19, 0xf7, 0xdd, 0x26, 0xfc, 0x2e, 0x94, 0xa1, 0x6a, 0x53, 0xb1, 0x8b, 0xc9, 0x2d,  },
      }
   };

   printf("Large HMAC test with %s using partial processing...\n", algo ? "sha1" : "md5");

   outsize = 16 + (4 * !!algo);

   // allocate a handle for HMAC-MD5 in encrypt mode using the default ICV size (0)
   x = spacc_dev_open(&fd, CRYPTO_MODE_NULL, (algo==0)?CRYPTO_MODE_HMAC_MD5:CRYPTO_MODE_HMAC_SHA1, 1, 0, 0, 0, "", 0, "", 0, "abcdefghijklmnop", 16, "", 0);
   assert(x >= 0);

   // HMAC large buffers of all 'a's
   memset(src, 'a', 128);

   for (x = 8; x <= 20; x++) {
      if (((1u<<x)-1) > features.max_msg_size) break;

      ELP_SPACC_USR_MSG_FIRST(&fd);
      y = spacc_dev_process(&fd, NULL, -1, 128, 0, 0, 0, src, 128, dst, sizeof dst);           // this call returns zero because PT==AAD and we're not copying AAD
      if (y) {
         printf("HMAC[%d] partial for 2**%d - 1 failed to start...\n", algo, x);
         exit(-1);
      }
      if ((1u<<x) >= 512) {
         ELP_SPACC_USR_MSG_MIDDLE(&fd);
         for (z = 0; z < (((1<<x)/128)-2); z++) {
            y = spacc_dev_process(&fd, NULL, -1, 128, 0, 0, 0, src, 128, dst, sizeof dst);           // this call returns zero because PT==AAD and we're not copying AAD
            if (y) {
               printf("HMAC[%d] partial for 2**%d - 1 failed in middle...\n", algo, x);
               exit(-1);
            }
         }
      }
      ELP_SPACC_USR_MSG_LAST(&fd);
      y = spacc_dev_process(&fd, NULL, -1, 127, 0, 0, 0, src, 127, dst, sizeof dst);            // this call returns the HMAC tag of 16 bytes
      if (y != outsize) {
         printf("HMAC[%d] partial for 2**%d - 1 failed to finish...\n", algo, x);
         exit(-1);
      }
      if (memcmp(hash[algo][x-8], dst, outsize)) {
         printf("HMAC[%d] partial FAILED memcmp for 2**%d - 1\n", algo, x);
         exit(-1);
      }
      printf("HMAC[%s] partial 2**%d - 1 passed\n", algo?"sha1":"md5", x);
   }
   spacc_dev_close(&fd);
   printf("\n");
}


int hash_only(void)
{
   unsigned char src[128], dst[128], src2[128], dst2[128];
   struct elp_spacc_usr fd, fd2;
   int x;
   unsigned char hash_only[16] = { 0x8b, 0x84, 0x32, 0x48, 0x48, 0x9a, 0xfb, 0xb1, 0xee, 0x33, 0xeb, 0x08, 0x39, 0xc5, 0xe1, 0x9e };

   // allocate a handle for HMAC-MD5 in encrypt mode using the default ICV size (0)
   x = spacc_dev_open(&fd, CRYPTO_MODE_NULL, CRYPTO_MODE_HMAC_MD5, 1, 0, 0, 0, "", 0, "", 0, "abcdefghijklmnop", 16, "", 0);
   assert(x >= 0);

   if (features.partial) {
      run_more_thorough_partial_test(0);
   }

   // 32 bytes of message
   strncpy((char *)src, "abcdefghijklmnopqrstuvwxyz01234", sizeof src);

   // hmac it, should return length of output
   x = spacc_dev_process(&fd, NULL, -1, 32, 0, 0, 0, src, 32, dst, sizeof dst);
   printf("HMAC sign: return code (16==OK) == %d\n", x);
   if (x != 16 || memcmp(dst, hash_only, x)) {
      printf("Failed to match hash_only test vector\n");
      exit(-1);
   }


   // hmac it, should return length of output
   memmove(src+5,src,32);
   x = spacc_dev_process(&fd, NULL, -1, 32, 0, 5, 0, src, 32, dst, sizeof dst);
   printf("HMAC sign with offset: return code (16==OK) == %d\n", x);
   if (x != 16 || memcmp(dst, hash_only, x)) {
      printf("Failed to match hash_only test vector\n");
      exit(-1);
   }
   memmove(src,src+5,32);

   // open another handle for the same key but in verify/decrypt mode this time
   x = spacc_dev_open(&fd2, CRYPTO_MODE_NULL, CRYPTO_MODE_HMAC_MD5, 0, 0, 0, 0, "", 0, "", 0, "abcdefghijklmnop", 16, "", 0);
   assert(x >= 0);

   // copy ciphertext + HMAC output to new src DMA buffer
   memcpy(src2, src, 32);
   memcpy(src2+32, dst, 16);

   // verify
   x = spacc_dev_process(&fd2, NULL, -1, 32, 0, 0, 0, src2, 32, dst2, sizeof dst2);
   printf("HMAC verify: return code (0==OK) == %d\n", x);
   if (x) {
      printf("Failed to hash correctly\n");
      exit(-1);
   }

   memmove(src2+7, src2, 48);
   x = spacc_dev_process(&fd2, NULL, -1, 32, 0, 7, 0, src2, 32, dst2, sizeof dst2);
   printf("HMAC verify with offset: return code (0==OK) == %d\n", x);
   if (x) {
      printf("Failed to hash correctly\n");
      exit(-1);
   }
   memmove(src2, src2+7, 48);

   src2[0] ^= 1;
   x = spacc_dev_process(&fd2, NULL, -1, 32, 0, 0, 0, src2, 32, dst2, sizeof dst2);

   printf("HMAC' verify: return code (-1==OK) == %d\n", x);
   if (x != -1) {
      printf("Failed to detect error in hash correctly\n");
      exit(-1);
   }

   // close/unmap buffers
   spacc_dev_close(&fd);
   spacc_dev_close(&fd2);

   return 0;
}

int combined(void)
{
   unsigned char src[128], dst[128], src2[128], dst2[128];
   struct elp_spacc_usr fd, fd2;
   int x;

unsigned char combined_mode[48] = {
0x0c,0x47,0x28,0x35,0xcf,0x8b,0xba,0xf7,0xcb,0x9b,0xc4,0x3b,0xbb,0xdc,0xa8,0xc4,
0x6c,0x5a,0x99,0x2c,0x61,0x46,0xca,0x47,0xd2,0xb3,0x78,0x6a,0xb0,0x89,0x03,0x86,
0x12,0x89,0x5b,0xc5,0x06,0x06,0x67,0x17,0xd6,0xc5,0x74,0xaf,0xee,0x50,0xcb,0x11,
};


unsigned char combined_mode_noenchash[48] = {
0x0c,0x47,0x28,0x35,0xcf,0x8b,0xba,0xf7,0xcb,0x9b,0xc4,0x3b,0xbb,0xdc,0xa8,0xc4,
0x6c,0x5a,0x99,0x2c,0x61,0x46,0xca,0x47,0xd2,0xb3,0x78,0x6a,0xb0,0x89,0x03,0x86,
0x8b,0x84,0x32,0x48,0x48,0x9a,0xfb,0xb1,0xee,0x33,0xeb,0x08,0x39,0xc5,0xe1,0x9e,};


   // allocate a handle for AES-128-CBC/HMAC-MD5 in encrypt mode using the default ICV size (0)
   x = spacc_dev_open(&fd, CRYPTO_MODE_AES_CBC, CRYPTO_MODE_HMAC_MD5, 1, ICV_HASH_ENCRYPT, 0, 0, "1234567890ABCDEF", 16, "43218765A098FEDC", 16, "abcdefghijklmnop", 16, "", 0);
   assert(!x);

   // 32 bytes of plaintext
   strcpy((char *)src, "abcdefghijklmnopqrstuvwxyz01234");

   // encrypt it, should return length of output
   x = spacc_dev_process(&fd, NULL, -1, 0, 0, 0, 0, src, 32, dst, sizeof dst);

   printf("Combined mode with ICV_ENC (48==OK) == %d\n", x);
   if (x != 48 || memcmp(dst, combined_mode, x)) {
      printf("Failed to match combined_mode test vector\n");
      exit(-1);
   }

   // open another handle for the same key but in decrypt mode this time
   x = spacc_dev_open(&fd2, CRYPTO_MODE_AES_CBC, CRYPTO_MODE_HMAC_MD5, 0, ICV_HASH_ENCRYPT, 0, 0, "1234567890ABCDEF", 16, "43218765A098FEDC", 16, "abcdefghijklmnop", 16, "", 0);
   assert(!x);

   // copy ciphertext + HMAC output to new src DMA buffer
   memcpy(src2, dst, 48);

   // decrypt (32 plaintext + 16 for HMAC tag)
   x = spacc_dev_process(&fd2, NULL, -1, 0, 0, 0, 0, src2, 32, dst2, sizeof dst2);

   printf("Combined mode with ICV_ENC return code (32==OK) == %d\n", x);
   if (x != 32 || memcmp(dst2, src, 32)) {
      printf("Failed to decrypt properly");
      exit(-1);
   }

   // modify the src to fail the ICV check
   src2[0] ^= 1;

   // decrypt (32 plaintext + 16 for HMAC tag)
   x = spacc_dev_process(&fd2, NULL, -1, 0, 0, 0, 0, src2, 32, dst2, sizeof dst2);

   printf("Combined mode with ICV_ENC (modified) return code (-1==OK) == %d\n", x);
   if (x != -1) {
      printf("Failed to detect error in decrypt properly");
      exit(-1);
   }
   src2[0] ^= 1; // put it back to normal


   // close/unmap buffers
   spacc_dev_close(&fd);
   spacc_dev_close(&fd2);


   // allocate a handle for AES-128-CBC/HMAC-MD5 in encrypt mode using the default ICV size (0)
   x = spacc_dev_open(&fd, CRYPTO_MODE_AES_CBC, CRYPTO_MODE_HMAC_MD5, 1, ICV_HASH, 0, 0, "1234567890ABCDEF", 16, "43218765A098FEDC", 16, "abcdefghijklmnop", 16, "", 0);
   assert(!x);

   // 32 bytes of plaintext
   strcpy((char *)src, "abcdefghijklmnopqrstuvwxyz01234");

   // encrypt it, should return length of output
   x = spacc_dev_process(&fd, NULL, -1, 0, 0, 0, 0, src, 32, dst, sizeof dst);

   printf("Combined mode without ICV_ENC (48==OK) == %d\n", x);
   if (x != 48 || memcmp(dst, combined_mode_noenchash, x)) {
      printf("Failed to match combined_mode test vector\n");
      exit(-1);
   }

   // open another handle for the same key but in decrypt mode this time
   x = spacc_dev_open(&fd2, CRYPTO_MODE_AES_CBC, CRYPTO_MODE_HMAC_MD5, 0, ICV_HASH, 0, 0, "1234567890ABCDEF", 16, "43218765A098FEDC", 16, "abcdefghijklmnop", 16, "", 0);
   assert(!x);

   // copy ciphertext + HMAC output to new src DMA buffer
   memcpy(src2, dst, 48);

   // decrypt (32 plaintext + 16 for HMAC tag)
   x = spacc_dev_process(&fd2, NULL, -1, 0, 0, 0, 0, src2, 32, dst2, sizeof dst2);

   printf("Combined mode without ICV_ENC return code (32==OK) == %d\n", x);
   if (x != 32 || memcmp(dst2, src, 32)) {
      printf("Failed to decrypt properly");
      exit(-1);
   }

   // close/unmap buffers
   spacc_dev_close(&fd);
   spacc_dev_close(&fd2);


   return 0;
}

struct job_data {
   struct elp_spacc_usr fd;
};

void * speed(void *d)
{
   struct elp_spacc_usr fd2;
   struct job_data *jd = d;
   unsigned char buf[2][4096+16];
   unsigned x;
   const unsigned char speed_data[16] = { 0x1e,0xde,0x60,0x56,0xaf,0xa6,0xfa,0x14,0xfa,0x20,0x72,0xe1,0xda,0x47,0xae,0xeb, };

   #define BLKSIZ (sizeof(buf[0]) - 16)

   printf("Starting thread...\n");
   memset(buf, 0, sizeof buf);

   // bind a new handle to it so we can use both
   x = spacc_dev_open_bind(&jd->fd, &fd2);
   assert(!x);

   // build DDT entries

   // we split the source buffer in two even portions with the IV tacked on to the third entry
   ELP_SPACC_SRC_DDT(&fd2, 0, &buf[0][0],        BLKSIZ/2);
   ELP_SPACC_SRC_DDT(&fd2, 1, &buf[0][BLKSIZ/2], BLKSIZ/2);
   ELP_SPACC_SRC_DDT(&fd2, 2, &buf[0][BLKSIZ],   16); // +16 to include IV
   ELP_SPACC_SRC_TERM(&fd2, 3);
   ELP_SPACC_SRCLEN_SET(&fd2, BLKSIZ);  // we can't use RESET here since the 16 bytes we added for IV import isn't supposed to be counted as plaintext

   // we split the destination into two uneven portions
   ELP_SPACC_DST_DDT(&fd2, 0, &buf[1][0],                 (BLKSIZ/2) - 7);            // break into odd size chunks
   ELP_SPACC_DST_DDT(&fd2, 1, &buf[1][fd2.io.dst[0].len], BLKSIZ - fd2.io.dst[0].len);
   ELP_SPACC_DST_TERM(&fd2, 2);
   ELP_SPACC_DSTLEN_RESET(&fd2); // here we can use RESET since we only setup enough DDT space for our ciphertext

   // copy IV to last block buffer so we can start this loop cleanly
   // here since we are using two handles that both talk to the same key context page we
   // must use IV import.  We use the last 16 bytes of the block to store the IV.
   // We cannot use new_iv since that actually writes to the key context page we MUST
   // use IV import.
   memcpy(&buf[1][BLKSIZ-16], "43218765A098FEDC", 16);

   for (x = 0; x < (((1UL<<25)/BLKSIZ)) - 1; x++) {
      memcpy(&buf[0][BLKSIZ], &buf[1][BLKSIZ-16], 16); // copy last block of ciphertext to end of plaintext so we can do IV import on it
      if (spacc_dev_process_multi(&fd2, NULL, BLKSIZ, 0, 0, 0, 0, SPACC_MAP_HINT_NOLAP) < 0) {
         printf("ERR: %d\n", fd2.io.err);
         return NULL;
      }
   }
   // do last job in place
   memcpy(&buf[0][BLKSIZ], &buf[1][BLKSIZ-16], 16); // copy last block of ciphertext to end of plaintext so we can do IV import on it
   if (spacc_dev_process(&fd2, NULL, BLKSIZ, 0, 0, 0, 0, buf[0], BLKSIZ, buf[0], BLKSIZ) < 0) {
      printf("last err: %d\n", fd2.io.err);
      assert(0 == 1);
   }
   spacc_dev_close(&fd2);
   if (memcmp(&buf[0][BLKSIZ-16], speed_data, 16)) {
      printf("Failed speed test comparison\n");
      exit(-1);
   }
   printf("DONE\n");
   return NULL;
}

#include <sys/uio.h>
#include <fcntl.h>

int main(int argc, char **argv)
{
   int x;
   pthread_t pt[32];
   struct job_data jd;

   x = spacc_dev_features(&features);
   assert(x == 0);
   printf("SPAcc userspace driver bound to SPAcc EPN-%04x ver.%02x, max msg size %u, partial support %s, qos %s, iv %s\n\n\n", features.project, features.version, features.max_msg_size, features.partial ? "on" : "off", features.qos ? "on" : "off", features.ivimport ? "on" : "off");

   if (argc == 1) {
      if (spacc_dev_isenabled(&features, CRYPTO_MODE_AES_CBC, 16)) {
         printf("Cipher only\n"); cipher_only(); printf("\n\n");
      } else {
         printf("Cannot run cipher test since AES=128-CBC is not enabled\n\n");
      }
      if (spacc_dev_isenabled(&features, CRYPTO_MODE_HMAC_MD5, 16)) {
         printf("Hash only\n");   hash_only(); printf("\n\n");
      } else {
         printf("Cannot run hash test since HMAC-MD5-128 is not enabled\n\n");
      }

      if (features.partial && spacc_dev_isenabled(&features, CRYPTO_MODE_HMAC_SHA1, 16)) {
         run_more_thorough_partial_test(1);
      }

      if (spacc_dev_isenabled(&features, CRYPTO_MODE_HMAC_MD5, 16) && spacc_dev_isenabled(&features, CRYPTO_MODE_AES_CBC, 16)) {
         printf("Combined\n");    combined(); printf("\n\n");
      } else {
         printf("Cannot run combined mode test since one (or both) of AES or MD5 are not enabled\n\n");
      }
   } else {
      printf("SPEED\n");

      // open the master handle from which the threads will bind to
      x = spacc_dev_open(&jd.fd, CRYPTO_MODE_AES_CBC, CRYPTO_MODE_NULL, 1, 0, 0, 0, "1234567890ABCDEF", 16, "43218765A098FEDC", 16, "", 0, "", 0);
      assert(!x);

      // register the handle
      x = spacc_dev_register(&jd.fd);
      assert(!x);

      for (x = 0; x < atoi(argv[1]); x++) {
         pthread_create(&pt[x], NULL, &speed, &jd);
      }
      for (x = 0; x < atoi(argv[1]); x++) {
         pthread_join(pt[x], NULL);
      }
      printf("\n\n");
   }
   printf("ALL TESTS PASSED\n");
}
