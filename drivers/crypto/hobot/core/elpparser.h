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
 * Copyright (c) 2002-2012 Synopsys, Inc. and/or its affiliates.
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

//-----------------------------------------------------------------------
//
// Project:
//
//   common sdk
//
// Description:
//
//   spacc vector parser header
//
//-----------------------------------------------------------------------*/
#ifndef _ELPPARSER_H
#define _ELPPARSER_H

#define VECTOR_BUF_MAX_SIZE 1048576


enum
{
   FAILURE_FLAG = 0,
   MULTI2_FLAG = 1,
   AUXINFO_FLAG = 2,
};

enum
{
   ERROR_KEY = 0,
   ERROR_IV = 1,
   ERROR_SALT_KEY = 2,
   ERROR_ICV_KEY = 3,
   ERROR_INPUT = 4,
};

#define SEK_KEY_MASK 0x00008000
#define SEK_KEY_SHIFT 15
#define MSEK_KEY_MASK 0x00004000
#define MSEK_KEY_SHIFT 14

/* enc_mode index */
enum
{
   VEC_MODE_NULL = 0,
   VEC_MODE_RC4_40,
   VEC_MODE_RC4_128,
   VEC_MODE_RC4_KS,
   VEC_MODE_AES_ECB,
   VEC_MODE_AES_CBC,
   VEC_MODE_AES_CTR,
   VEC_MODE_AES_CCM,
   VEC_MODE_AES_GCM,
   VEC_MODE_AES_F8,
   VEC_MODE_AES_XTS,
   VEC_MODE_AES_CFB,
   VEC_MODE_AES_OFB,
   VEC_MODE_AES_CS1,
   VEC_MODE_AES_CS2,
   VEC_MODE_AES_CS3,
   VEC_MODE_MULTI2_ECB,
   VEC_MODE_MULTI2_CBC,
   VEC_MODE_MULTI2_OFB,
   VEC_MODE_MULTI2_CFB,
   VEC_MODE_3DES_CBC,
   VEC_MODE_3DES_ECB,
   VEC_MODE_DES_CBC,
   VEC_MODE_DES_ECB,
   VEC_MODE_KASUMI_ECB,
   VEC_MODE_KASUMI_F8,
   VEC_MODE_SNOW3G_UEA2,
   VEC_MODE_ZUC_UEA3,
};

/* hash mode index */
enum
{
   VEC_MODE_HASH_NULL = 0,
   VEC_MODE_HASH_MD5,
   VEC_MODE_HMAC_MD5,
   VEC_MODE_HASH_SHA1,
   VEC_MODE_HMAC_SHA1,
   VEC_MODE_HASH_SHA224,
   VEC_MODE_HMAC_SHA224,
   VEC_MODE_HASH_SHA256,
   VEC_MODE_HMAC_SHA256,
   VEC_MODE_HASH_SHA384,
   VEC_MODE_HMAC_SHA384,
   VEC_MODE_HASH_SHA512,
   VEC_MODE_HMAC_SHA512,
   VEC_MODE_HASH_SHA512_224,
   VEC_MODE_HMAC_SHA512_224,
   VEC_MODE_HASH_SHA512_256,
   VEC_MODE_HMAC_SHA512_256,
   VEC_MODE_MAC_XCBC,
   VEC_MODE_MAC_CMAC,
   VEC_MODE_MAC_KASUMI_F9,
   VEC_MODE_MAC_SNOW3G_UIA2,
   VEC_MODE_MAC_ZUC_UIA3,
   VEC_MODE_SSLMAC_MD5,
   VEC_MODE_SSLMAC_SHA1,
   VEC_MODE_HASH_CRC32,
   VEC_MODE_MICHAEL_MIC,
};



// Generic Test Vector structure collects all control and data fiels from the
// binary vector format

typedef struct vector_data_
{
   int enc_mode;
   int hash_mode;
   int keysize;
   int hmac_keysize;

   int iv_offset;
   int iv_size;
   int c_iv_size;
   int h_iv_size;

   int icv_mode;
   int icv_offset;
   int icv_size;
   int icvpos;                  // this is set interally it's the flag of how the ICV offset is loaded
   unsigned char icv[128];

   int salt_size;
   int flags;
   int errors;
   int fail_mode;
   int outsplit[10];
   int insplit[10];

   int pre_aad_size;
   int post_aad_size;
   int pt_size;
   int ct_size;

   int auxinfo_dir;
   int auxinfo_bit_align;

   int seckey;
   int mseckey;

   unsigned long seed;

   unsigned char key[384];
   unsigned char hmac_key[384];
   unsigned char iv[384];
   unsigned char saltkey[384];

   unsigned char *pre_aad;
   unsigned char *post_aad;
   unsigned char *pt;
   unsigned char *ct;

   unsigned int epn, virt;

   struct
   {
      unsigned char *pool, *pread, *postad, *ct, *pt;
   } mem;

   size_t max_packet;

   int error_field, error_mangle, error_offset;
} vector_data;

// This function reads binary test vector content and stores it in the
// vector_data structure. Note that it allocates the input and output data
// pointers so do not for get to call the free_test_vector function for each
// vector parsed
int parse_test_vector(vector_data * vector);
vector_data *init_test_vector(size_t max_packet);
int free_test_vector(vector_data * vector);
size_t elp_parser_fwrite(unsigned char *src, size_t len);
void elp_parse_rewind(void);
#endif
