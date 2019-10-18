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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include "elpparser.h"

static unsigned char *vector_buf=NULL;
static uint32_t vector_off, vector_size;


void elp_parse_rewind(void)
{
  vector_off = 0;
}

static int elp_fgetc(void)
{
   if (vector_off < vector_size) {
      return vector_buf[vector_off++] & 0xFF;
   } else {
      return -1;
   }
}

static size_t elp_fread(unsigned char *dst, size_t len)
{
   size_t n;

   if (len + vector_off > vector_size) {
      n = vector_size - vector_off;
   } else {
      n = len;
   }

   memcpy(dst, vector_buf + vector_off, n);
   vector_off += n;
   return n;
}

size_t elp_parser_fwrite(unsigned char *src, size_t len)
{
  size_t n, r;
  if (len + vector_size > VECTOR_BUF_MAX_SIZE) {
    n = VECTOR_BUF_MAX_SIZE - vector_size;
  } else {
    n = len;
  }
  memcpy(vector_buf + vector_size, src, n);
  vector_size += n;
#if 0
  printk("Added %u bytes to vector it's now %u bytes long\n", (unsigned)n, (unsigned)vector_size);
  for (r = 0; r < n; ) {
    if (!(r & 15)) { printk("%04u ", r); }
    printk("%02x ", src[r]);
    if (!(++r & 15))  { printk("\n"); }
  }
  printk("\n");
#endif
  return n;
}

#define PARSE_MALLOC(x) vmalloc(x)
#define PARSE_FREE(x) vfree(x)

static int getNum(int length)
{
   unsigned int temp, i;
   unsigned char c;
   temp = 0;
   if (length == 0)
      return 0;
   for (i = 0; i < length; i += 1) {
      c = elp_fgetc();
      temp |= (c & 0xFF) << 8 * (length - 1 - i);
   }
   return temp;
}

static void getChars(unsigned char *buffer, int length)
{
   if (elp_fread(buffer, length) != length) {
      printk("SPACC VECTOR: Warning did not read entire buffer into memory\n");
   }
}

vector_data *init_test_vector(size_t max_packet)
{
   vector_data *vector;

   if (vector_buf == NULL) {
     vector_size = 0;
     vector_off  = 0;
     vector_buf  = PARSE_MALLOC(VECTOR_BUF_MAX_SIZE);
     if (!vector_buf) {
       return NULL;
     }
   }

   vector = PARSE_MALLOC(sizeof *vector);
   if (vector == NULL) {
      PARSE_FREE(vector_buf);
      return NULL;
   }

   memset(vector, 0, sizeof(vector_data));
   vector->max_packet = max_packet;

   vector->mem.pool = PARSE_MALLOC(4 * max_packet);
   if (vector->mem.pool == NULL) {
      PARSE_FREE(vector_buf);
      PARSE_FREE(vector);
      return NULL;
   }
   vector->mem.pread = vector->mem.pool + (0 * max_packet);
   vector->mem.postad = vector->mem.pool + (1 * max_packet);
   vector->mem.pt = vector->mem.pool + (2 * max_packet);
   vector->mem.ct = vector->mem.pool + (3 * max_packet);
   return vector;
}

int free_test_vector(vector_data * vector)
{
   PARSE_FREE(vector_buf);
   vector_buf  = NULL;
   vector_size = 0;
   vector_off = 0;
   PARSE_FREE(vector->mem.pool);
   PARSE_FREE(vector);
   return 1;
}

// This function reads binary test vector content and stores it in the
// vector_data structure. Note that it allocates the input and output data
// pointers so remember to call the free_test_vector function for each
// vector parsed
int parse_test_vector(vector_data * vector)
{
   int y = -1;


   vector->seed = getNum(4);
   vector->flags = getNum(1);
   vector->errors = getNum(1);
   vector->error_field = getNum(1);
   vector->error_mangle = getNum(1);
   vector->error_offset = getNum(4);
   vector->enc_mode = getNum(1);
   vector->hash_mode = getNum(1);
   vector->icv_mode = getNum(1);
   vector->iv_offset = getNum(4);



   if (vector->iv_offset == 0xFFFFFFFF) {
      vector->iv_offset = 0;
   } else {
      vector->iv_offset |= 0x80000000UL;  // if we are using it we must enable the high bit
   }


   if ((vector->flags >> AUXINFO_FLAG) & 0x01) {
      vector->auxinfo_bit_align = getNum(1);
      vector->auxinfo_dir = getNum(1);
   } else {
      vector->auxinfo_bit_align = 8;
      vector->auxinfo_dir = 0;
   }

   if ((vector->flags >> FAILURE_FLAG) & 0x01) {
      vector->fail_mode = 1;
   } else {
      vector->fail_mode = 0;
   }
   vector->keysize = getNum(2);
   vector->seckey = (vector->keysize & SEK_KEY_MASK) >> SEK_KEY_SHIFT;
   vector->mseckey = (vector->keysize & MSEK_KEY_MASK) >> MSEK_KEY_SHIFT;
   vector->keysize &= ~(SEK_KEY_MASK | MSEK_KEY_MASK);
   if (vector->keysize > sizeof(vector->key)) {
      printk("SPACC VECTOR:  Keysize(%d) larger than max key size (%zu) supported\n", vector->keysize, sizeof(vector->key));
      goto ERR;
   }
   getChars(vector->key, vector->keysize);

   vector->hmac_keysize = getNum(1);
   if (vector->hmac_keysize > sizeof(vector->hmac_key)) {
      printk("SPACC VECTOR:  HASH Keysize larger than max key size supported\n");
      goto ERR;
   }
   getChars(vector->hmac_key, vector->hmac_keysize);

   vector->iv_size = getNum(2);

   vector->c_iv_size = vector->iv_size & 0xff;
   vector->h_iv_size = (vector->iv_size & 0xff00) >> 8;
   vector->iv_size = vector->c_iv_size + vector->h_iv_size;
   if (vector->iv_offset != 0) {
      vector->c_iv_size = 0;
      vector->h_iv_size = 0;
   }

   if (vector->iv_size > sizeof(vector->iv)) {
      printk("SPACC VECTOR:  IVsize larger than max IV size supported\n");
      goto ERR;
   }
   getChars(vector->iv, vector->iv_size);

   vector->salt_size = getNum(1);
   if (vector->salt_size > sizeof(vector->saltkey)) {
      printk("SPACC VECTOR:  salt_size larger than max salt size supported\n");
      goto ERR;
   }
   getChars(vector->saltkey, vector->salt_size);

   vector->pre_aad_size = getNum(4);
   if (vector->pre_aad_size > vector->max_packet) {
      printk("SPACC VECTOR:  PRE AAD size too large\n");
      goto ERR;
   }

   if (vector->pre_aad_size) {
      vector->pre_aad = vector->mem.pread;
      getChars(vector->pre_aad, vector->pre_aad_size);
   }

   vector->post_aad_size = getNum(4);
   if (vector->post_aad_size > vector->max_packet) {
      printk("SPACC VECTOR:  POST AAD size too large\n");
      goto ERR;
   }
   if (vector->post_aad_size) {
      vector->post_aad = vector->mem.postad;
      getChars(vector->post_aad, vector->post_aad_size);
   }

   vector->pt_size = getNum(4);
   if (vector->pt_size > vector->max_packet) {
      printk("SPACC VECTOR:  PT size too large\n");
      goto ERR;
   }

   vector->pt = vector->mem.pt;
   getChars(vector->pt, vector->pt_size);

   vector->ct_size = getNum(4);
   if (vector->ct_size > vector->max_packet) {
      printk("SPACC VECTOR:  CT size too large\n");
      goto ERR;
   }
   vector->ct = vector->mem.ct;
   getChars(vector->ct, vector->ct_size);

   vector->icv_size = getNum(1);
   if (vector->icv_size > sizeof(vector->icv)) {
      printk("SPACC VECTOR:  ICV size too large\n");
      goto ERR;
   }
   getChars(vector->icv, vector->icv_size);
   // for now, the ICV offset < SPACC_MAX_PARTICLE_SIZE (65536)
   vector->icv_offset = getNum(4);

   vector->virt = getNum(4);
   vector->epn = getNum(4);

   y = 0;
 ERR:
   return y;
}
