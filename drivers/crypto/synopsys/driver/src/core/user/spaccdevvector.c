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
 * Copyright (c) 2011-2018 Synopsys, Inc. and/or its affiliates.
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

#include "elpspaccusr.h"
#include "spaccdevvector.h"

#define IV_OFFSET_MASK 0x7fffffff
#define IV_OFFSET_EN_MASK 0x80000000

int getNum (int length, FILE * file)
{
   unsigned int temp, i;
   unsigned char c;
   temp = 0;
   if (length == 0)
      return 0;
   for (i = 0; i < length; i += 1) {
      c = fgetc (file);
      temp |= (c & 0xFF) << 8 * (length - 1 - i);
   }
   return temp;
}

void getChars (unsigned char *buffer, int length, FILE * file)
{
   if (fread (buffer, 1, length, file) != length) {
      printf ("SPACC VECTOR: Warning did not read entire buffer into memory\n");
   }
}

vector_data *init_test_vector (size_t max_packet)
{
   vector_data *vector;

   vector = calloc (1, sizeof *vector);
   if (vector == NULL)
      return NULL;

   memset (vector, 0, sizeof (vector_data));
   vector->max_packet = max_packet;

   vector->mem.pool = calloc (4, max_packet);
   if (vector->mem.pool == NULL) {
      free (vector);
      return NULL;
   }
   vector->mem.pread  = vector->mem.pool + (0 * max_packet);
   vector->mem.postad = vector->mem.pool + (1 * max_packet);
   vector->mem.pt     = vector->mem.pool + (2 * max_packet);
   vector->mem.ct     = vector->mem.pool + (3 * max_packet);
   return vector;
}

int free_test_vector (vector_data * vector)
{
   free (vector->mem.pool);
   free (vector);
   return 1;
}

// This function reads binary test vector content and stores it in the
// vector_data structure. Note that it allocates the input and output data
// pointers so remember to call the free_test_vector function for each
// vector parsed
int parse_test_vector (char *filename, vector_data * vector)
{
   int y = -1;
   FILE *in;

   if ((in = fopen (filename, "rb")) == NULL)
      return -1;

   vector->seed = getNum (4, in);
   vector->flags = getNum (1, in);
   vector->errors = getNum (1, in);
   vector->error_field = getNum (1, in);
   vector->error_mangle = getNum (1, in);
   vector->error_offset = getNum (4, in);
   vector->enc_mode = getNum (1, in);
   vector->hash_mode = getNum (1, in);
   vector->icv_mode = getNum (1, in);
   vector->iv_offset = getNum (4, in);

   if (vector->iv_offset == 0xFFFFFFFF) {
      vector->iv_offset = 0;
   } else {
      vector->iv_offset |= 0x80000000UL;  // if we are using it we must enable the high bit
   }

   if ((vector->flags >> AUXINFO_FLAG) & 0x01) {
      vector->auxinfo_bit_align = getNum (1, in);
      vector->auxinfo_dir = getNum (1, in);
   } else {
      vector->auxinfo_bit_align = 8;
      vector->auxinfo_dir = 0;
   }

   if ((vector->flags >> FAILURE_FLAG) & 0x01) {
      vector->fail_mode = 1;
   } else {
      vector->fail_mode = 0;
   }
   vector->keysize = getNum (2, in);
   vector->seckey = (vector->keysize & SEK_KEY_MASK) >> SEK_KEY_SHIFT;
   vector->mseckey = (vector->keysize & MSEK_KEY_MASK) >> MSEK_KEY_SHIFT;
   vector->keysize &= ~(SEK_KEY_MASK | MSEK_KEY_MASK);
   if (vector->keysize > sizeof (vector->key)) {
      printf ("SPACC VECTOR:  Keysize larger than max key size supported\n");
      goto ERR;
   }
   getChars (vector->key, vector->keysize, in);

   vector->hmac_keysize = getNum (1, in);
   if (vector->hmac_keysize > sizeof (vector->hmac_key)) {
      printf ("SPACC VECTOR:  HASH Keysize larger than max key size supported\n");
      goto ERR;
   }
   getChars (vector->hmac_key, vector->hmac_keysize, in);

   vector->iv_size = getNum (2, in);

   vector->c_iv_size = vector->iv_size & 0xff;
   vector->h_iv_size = (vector->iv_size & 0xff00) >> 8;
   vector->iv_size = vector->c_iv_size + vector->h_iv_size;
   if (vector->iv_offset != 0) {
      vector->c_iv_size = 0;
      vector->h_iv_size = 0;
   }

   if (vector->iv_size > sizeof (vector->iv)) {
      printf ("SPACC VECTOR:  IVsize larger than max IV size supported\n");
      goto ERR;
   }
   getChars (vector->iv, vector->iv_size, in);

   vector->salt_size = getNum (1, in);
   if (vector->salt_size > sizeof (vector->saltkey)) {
      printf ("SPACC VECTOR:  salt_size larger than max salt size supported\n");
      goto ERR;
   }
   getChars (vector->saltkey, vector->salt_size, in);

   vector->pre_aad_size = getNum (4, in);
   if (vector->pre_aad_size > vector->max_packet) {
      printf ("SPACC VECTOR:  PRE AAD size too large\n");
      goto ERR;
   }

   if (vector->pre_aad_size) {
      vector->pre_aad = vector->mem.pread;
      getChars (vector->pre_aad, vector->pre_aad_size, in);
   }

   vector->post_aad_size = getNum (4, in);
   if (vector->post_aad_size > vector->max_packet) {
      printf ("SPACC VECTOR:  POST AAD size too large\n");
      goto ERR;
   }
   if (vector->post_aad_size) {
      vector->post_aad = vector->mem.postad;
      getChars (vector->post_aad, vector->post_aad_size, in);
   }

   vector->pt_size = getNum (4, in);
   if (vector->pt_size > vector->max_packet) {
      printf ("SPACC VECTOR:  PT size too large\n");
      goto ERR;
   }

   vector->pt = vector->mem.pt;
   getChars (vector->pt, vector->pt_size, in);

   vector->ct_size = getNum (4, in);
   if (vector->ct_size > vector->max_packet) {
      printf ("SPACC VECTOR:  CT size too large\n");
      goto ERR;
   }
   vector->ct = vector->mem.ct;
   getChars (vector->ct, vector->ct_size, in);

   vector->icv_size = getNum (1, in);
   if (vector->icv_size > sizeof (vector->icv)) {
      printf ("SPACC VECTOR:  ICV size too large\n");
      goto ERR;
   }
   getChars (vector->icv, vector->icv_size, in);
   // for now, the ICV offset < SPACC_MAX_PARTICLE_SIZE (65536)
   vector->icv_offset = getNum (4, in);

   vector->virt = getNum (4, in);
   vector->epn = getNum (4, in);

   y = 0;
 ERR:
   fclose (in);
   return y;
}

static int context_modes[2][64] = {
   {CRYPTO_MODE_NULL,
    CRYPTO_MODE_RC4_40,
    CRYPTO_MODE_RC4_128,
    CRYPTO_MODE_RC4_KS,
    CRYPTO_MODE_AES_ECB,
    CRYPTO_MODE_AES_CBC,
    CRYPTO_MODE_AES_CTR,
    CRYPTO_MODE_AES_CCM,
    CRYPTO_MODE_AES_GCM,
    CRYPTO_MODE_AES_F8,
    CRYPTO_MODE_AES_XTS,
    CRYPTO_MODE_AES_CFB,
    CRYPTO_MODE_AES_OFB,
    CRYPTO_MODE_AES_CS1,
    CRYPTO_MODE_AES_CS2,
    CRYPTO_MODE_AES_CS3,
    CRYPTO_MODE_MULTI2_ECB,
    CRYPTO_MODE_MULTI2_CBC,
    CRYPTO_MODE_MULTI2_OFB,
    CRYPTO_MODE_MULTI2_CFB,
    CRYPTO_MODE_3DES_CBC,
    CRYPTO_MODE_3DES_ECB,
    CRYPTO_MODE_DES_CBC,
    CRYPTO_MODE_DES_ECB,
    CRYPTO_MODE_KASUMI_ECB,
    CRYPTO_MODE_KASUMI_F8,
    CRYPTO_MODE_SNOW3G_UEA2,
    CRYPTO_MODE_ZUC_UEA3,
    CRYPTO_MODE_CHACHA20_STREAM,
    CRYPTO_MODE_CHACHA20_POLY1305,
    },
   {CRYPTO_MODE_NULL,
    CRYPTO_MODE_HASH_MD5,
    CRYPTO_MODE_HMAC_MD5,
    CRYPTO_MODE_HASH_SHA1,
    CRYPTO_MODE_HMAC_SHA1,
    CRYPTO_MODE_HASH_SHA224,
    CRYPTO_MODE_HMAC_SHA224,
    CRYPTO_MODE_HASH_SHA256,
    CRYPTO_MODE_HMAC_SHA256,
    CRYPTO_MODE_HASH_SHA384,
    CRYPTO_MODE_HMAC_SHA384,
    CRYPTO_MODE_HASH_SHA512,
    CRYPTO_MODE_HMAC_SHA512,
    CRYPTO_MODE_HASH_SHA512_224,
    CRYPTO_MODE_HMAC_SHA512_224,
    CRYPTO_MODE_HASH_SHA512_256,
    CRYPTO_MODE_HMAC_SHA512_256,
    CRYPTO_MODE_MAC_XCBC,
    CRYPTO_MODE_MAC_CMAC,
    CRYPTO_MODE_MAC_KASUMI_F9,
    CRYPTO_MODE_MAC_SNOW3G_UIA2,
    CRYPTO_MODE_MAC_ZUC_UIA3,
    CRYPTO_MODE_SSLMAC_MD5,
    CRYPTO_MODE_SSLMAC_SHA1,
    CRYPTO_MODE_HASH_CRC32,
    CRYPTO_MODE_MAC_MICHAEL,
    CRYPTO_MODE_HASH_SHA3_224,
    CRYPTO_MODE_HASH_SHA3_256,
    CRYPTO_MODE_HASH_SHA3_384,
    CRYPTO_MODE_HASH_SHA3_512,
    CRYPTO_MODE_HASH_SHAKE128,
    CRYPTO_MODE_HASH_SHAKE256,
    CRYPTO_MODE_HASH_CSHAKE128,
    CRYPTO_MODE_HASH_CSHAKE256,
    CRYPTO_MODE_MAC_KMAC128,
    CRYPTO_MODE_MAC_KMAC256,
    CRYPTO_MODE_MAC_KMACXOF128,
    CRYPTO_MODE_MAC_KMACXOF256,
    CRYPTO_MODE_MAC_POLY1305,
    }
};

static char *mode_names[2][64] = {
   {"NULL",
    "RC4_40",
    "RC4_128",
    "RC4_KS",
    "AES_ECB",
    "AES_CBC",
    "AES_CTR",
    "AES_CCM",
    "AES_GCM",
    "AES_F8",
    "AES_XTS",
    "AES_CFB",
    "AES_OFB",
    "AES_CS1",
    "AES_CS2",
    "AES_CS3",
    "MULTI2_ECB",
    "MULTI2_CBC",
    "MULTI2_OFB",
    "MULTI2_CFB",
    "3DES_CBC",
    "3DES_ECB",
    "DES_CBC",
    "DES_ECB",
    "KASUMI_ECB",
    "KASUMI_F8",
    "SNOW3G_UEA2",
    "ZUC_UEA2",
    "CHACHA20_STREAM",
    "CHACHA20_AEAD",
    },
   {"NULL",
    "HASH_MD5",
    "HMAC_MD5",
    "HASH_SHA1",
    "HMAC_SHA1",
    "HASH_SHA224",
    "HMAC_SHA224",
    "HASH_SHA256",
    "HMAC_SHA256",
    "HASH_SHA384",
    "HMAC_SHA384",
    "HASH_SHA512",
    "HMAC_SHA512",
    "HASH_SHA512_224",
    "HMAC_SHA512_224",
    "HASH_SHA512_256",
    "HMAC_SHA512_256",
    "MAC_XCBC",
    "MAC_CMAC",
    "MAC_KASUMI_F9",
    "SNOW3G_UIA2",
    "ZUC_UIA3",
    "SSLMAC_MD5",
    "SSLMAC_SHA1",
    "CRC32",
    "MICHAEL-MIC",
    "HASH_SHA3_224",
    "HASH_SHA3_256",
    "HASH_SHA3_384",
    "HASH_SHA3_512",
    "HASH_SHAKE128",
    "HASH_SHAKE256",
    "HASH_CSHAKE128",
    "HASH_CSHAKE256",
    "MAC_KMAC128",
    "MAC_KMAC256",
    "MAC_KMACXOF128",
    "MAC_KMACXOF256",
    "MAC_POLY1305",
    }
};

enum
{
   DIR_ENC = 0,
   DIR_DEC
};

#define BUF_SIZE (1UL<<16)

unsigned char *dst, *src;

void run_vector(char *name, int dir)
{
   vector_data *vector;
   struct elp_spacc_usr fd;
   int x, y, srclen;

   y = 0;

   memset(src, 0, BUF_SIZE);
   memset(dst, 0, BUF_SIZE);

   vector = init_test_vector (BUF_SIZE);
   assert (!parse_test_vector (name, vector));

   if (context_modes[0][vector->enc_mode] == CRYPTO_MODE_AES_F8) {
       unsigned char tmp[128];
       memcpy(tmp, vector->saltkey, vector->salt_size);
       memcpy(tmp + vector->salt_size, vector->key, vector->keysize);
       memcpy(vector->key, tmp, vector->keysize+vector->salt_size);
   }

   x = spacc_dev_open (&fd,
                       context_modes[0][vector->enc_mode], context_modes[1][vector->hash_mode],
                       !dir,
                       vector->icv_mode,
                       vector->icv_size, 0,
                       vector->key, vector->keysize,
                       vector->iv, vector->c_iv_size,
                       vector->hmac_key, vector->hmac_keysize,
                       vector->iv + vector->c_iv_size, vector->h_iv_size);
   assert (!x);


   // create input packet
   srclen = 0;
   memcpy (src + srclen, vector->pre_aad, vector->pre_aad_size);
   srclen += vector->pre_aad_size;
   if (dir == DIR_ENC) {
      memcpy (src + srclen, vector->pt, vector->pt_size);
      srclen += vector->pt_size;
   } else {
      if ((vector->icv_mode != ICV_HASH_ENCRYPT)) {
         memcpy (src + srclen, vector->ct, vector->pt_size);
         srclen += vector->pt_size;
      } else {
         memcpy (src + srclen, vector->ct, vector->ct_size);
         srclen += vector->ct_size - vector->icv_size; // don't include ICV in this mode
      }
   }
   memcpy (src + srclen, vector->post_aad, vector->post_aad_size);
   srclen += vector->post_aad_size;

   if (vector->icv_size > 0) {
      if (dir == DIR_DEC) {
         if ((vector->icv_mode != ICV_HASH_ENCRYPT)) {
            memcpy (src + srclen, vector->icv, vector->icv_size);
            //srclen += vector->icv_size; // we don't support ICV offset
         }
      }
   } else {
      if ((context_modes[1][vector->hash_mode] != CRYPTO_MODE_NULL)) {
         if ((dir == DIR_DEC) && (vector->ct_size > vector->pt_size)) {
            memcpy (src + srclen, vector->ct + vector->pt_size, vector->ct_size - vector->pt_size);
            //srclen += vector->ct_size - vector->pt_size;  // don't support ICV offset
         }
      }
   }

   // here we cope with the IV offset. for SRC DDT only.
   if (vector->iv_offset & IV_OFFSET_EN_MASK) {
      if (vector->iv_size) {
         memcpy (src + (vector->iv_offset & IV_OFFSET_MASK), vector->iv, vector->iv_size);   // no increase srclen because the driver will handle that for us
      }
   }

   spacc_dev_setaux(&fd, vector->auxinfo_bit_align, vector->auxinfo_dir);

   x = spacc_dev_process (&fd, NULL, (vector->iv_offset & IV_OFFSET_MASK) ? (vector->iv_offset & IV_OFFSET_MASK) : -1,
                          vector->pre_aad_size, vector->post_aad_size, 0, 0, src, srclen, dst, BUF_SIZE);
   spacc_dev_close (&fd);

   if (x < 0) {
      // error y0
      if (!(vector->fail_mode)) {
         printf("Failed to run SPAcc job: ERR == %d\n", fd.io.err);
      }
      y = -1;
      goto end;
   }
   // let's do some compares
   if (dir == DIR_ENC) {
      if (memcmp (dst, vector->ct, vector->ct_size)) {
         if (!(vector->fail_mode)) {
            printf ("Comparing CT failed[%d]\n", __LINE__);
            printf ("fail_mode == %d\n", vector->fail_mode);
         }
         y = 1;
      }
   } else {
      if (memcmp (dst, vector->pt, vector->pt_size)) {
         if (!(vector->fail_mode)) {
            printf ("Comparing PT failed[%d]\n", __LINE__);
            printf ("fail_mode == %d\n", vector->fail_mode);
         }
         y = 1;
      }
   }
 end:

 printf("Vector: [%s] in %s mode running %s/%s/%d: %s (error detected: %s)\n", name, dir?"decrypt":"encrypt", mode_names[0][vector->enc_mode], mode_names[1][vector->hash_mode], vector->icv_mode, (!!y == !!vector->fail_mode) ? "PASSED" : "FAILED", y ? "true" : "false");
 if (!!y != !!vector->fail_mode) {
   {  int xx;

      printf("src[%d] == \n", srclen);
      for (xx = 0; xx < srclen; ) {
         printf("%02x ", src[xx]);
         if (!(++xx & 15)) { printf("\n"); }
      }
      printf("\n");

      printf("dst[%d] == \n", x);
      for (xx = 0; xx < x; ) {
         printf("%02x ", dst[xx]);
         if (!(++xx & 15)) { printf("\n"); }
      }
      printf("\n");
   }
    exit(-1);
 }
 free_test_vector(vector);
}


int main (int argc, char **argv)
{
   int x;

   src = calloc (1, BUF_SIZE);
   assert (src);
   dst = calloc (1, BUF_SIZE);
   assert (dst);

   for (x = 1; x < argc; x++) {
      run_vector (argv[x], 0);
#ifndef MAKEAVECTOR
      run_vector (argv[x], 1);
#endif
   }

   free(src);
   free(dst);

   return 0;

}
