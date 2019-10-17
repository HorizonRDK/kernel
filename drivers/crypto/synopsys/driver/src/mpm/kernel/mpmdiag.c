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

#include "elpspaccdrv.h"
#include "elpspacc.h"
#include "elpparser.h"
#include "elpmpm.h"
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>

#define MODE_ENCRYPT 0
#define MODE_HASH    1

// for epn0409, Effective Master Key decrypt/load is forced to context 0.
#define EPN0409_EMK_CTX 0

#define IV_OFFSET_MASK 0x7fffffff
#define IV_OFFSET_EN_MASK 0x80000000

// valid for all MPMs based on SPAcc v4.14 and below
#define CIPH_MODE(ciph, ciphmode) \
   (((ciph)       <<  _PDU_DESC_CTRL_CIPH_ALG)  | \
   ((ciphmode)   <<  _PDU_DESC_CTRL_CIPH_MODE))

#define HASH_MODE(hash, hashmode) \
   (((hash)       <<  _PDU_DESC_CTRL_HASH_ALG)  | \
   ((hashmode)   << _PDU_DESC_CTRL_HASH_MODE))

static struct test_buffer {
   unsigned char *virt;

   struct page **pages;
   size_t num_pages;

   struct sg_table sgt;
   pdu_ddt ddt;
} inbuf, outbuf;

static uint32_t icv_modes[4] = {
     PDU_DESC_CTRL_ICV_PT,
     PDU_DESC_CTRL_ICV_PT|PDU_DESC_CTRL_ICV_ENC|PDU_DESC_CTRL_ICV_APPEND,
     0,
     0
};

static int context_table_update = 0;
static uint32_t context_modes[2][64] = {
  {
     CIPH_MODE(C_NULL,    CM_ECB),
     CIPH_MODE(C_RC4,     CM_ECB),
     CIPH_MODE(C_RC4,     CM_ECB),
     CIPH_MODE(C_RC4,     CM_ECB),
     CIPH_MODE(C_AES,     CM_ECB),
     CIPH_MODE(C_AES,     CM_CBC),
     CIPH_MODE(C_AES,     CM_CTR),
     CIPH_MODE(C_AES,     CM_CCM),
     CIPH_MODE(C_AES,     CM_GCM),
     CIPH_MODE(C_AES,     CM_F8),
     CIPH_MODE(C_AES,     CM_XTS),
     CIPH_MODE(C_AES,     CM_CFB),
     CIPH_MODE(C_AES,     CM_OFB),
     CIPH_MODE(C_AES,     CM_CBC), //CS1
     CIPH_MODE(C_AES,     CM_CBC), //CS2
     CIPH_MODE(C_AES,     CM_CBC), //CS3
     CIPH_MODE(C_MULTI2,  CM_ECB),
     CIPH_MODE(C_MULTI2,  CM_CBC),
     CIPH_MODE(C_MULTI2,  CM_OFB),
     CIPH_MODE(C_MULTI2,  CM_CFB),
     CIPH_MODE(C_DES,     CM_CBC),
     CIPH_MODE(C_DES,     CM_ECB),
     CIPH_MODE(C_DES,     CM_CBC),
     CIPH_MODE(C_DES,     CM_ECB),
     CIPH_MODE(C_KASUMI,  CM_ECB),
     CIPH_MODE(C_KASUMI,  CM_F8),
     CIPH_MODE(C_SNOW3G_UEA2, CM_ECB),
     CIPH_MODE(C_ZUC_UEA3, CM_ECB),
  },
  {
     HASH_MODE(H_NULL,              HM_RAW),
     HASH_MODE(H_MD5,               HM_RAW),
     HASH_MODE(H_MD5,               HM_HMAC),
     HASH_MODE(H_SHA1,              HM_RAW),
     HASH_MODE(H_SHA1,              HM_HMAC),
     HASH_MODE(H_SHA224,            HM_RAW),
     HASH_MODE(H_SHA224,            HM_HMAC),
     HASH_MODE(H_SHA256,            HM_RAW),
     HASH_MODE(H_SHA256,            HM_HMAC),
     HASH_MODE(H_SHA384,            HM_RAW),
     HASH_MODE(H_SHA384,            HM_HMAC),
     HASH_MODE(H_SHA512,            HM_RAW),
     HASH_MODE(H_SHA512,            HM_HMAC),
     HASH_MODE(H_SHA512_224,        HM_RAW),
     HASH_MODE(H_SHA512_224,        HM_HMAC),
     HASH_MODE(H_SHA512_256,        HM_RAW),
     HASH_MODE(H_SHA512_256,        HM_HMAC),
     HASH_MODE(H_XCBC,              HM_RAW),
     HASH_MODE(H_CMAC,              HM_RAW),
     HASH_MODE(H_KF9,               HM_RAW),
     HASH_MODE(H_SNOW3G_UIA2,       HM_RAW),
     HASH_MODE(H_ZUC_UIA3,          HM_RAW),
     HASH_MODE(H_MD5,               HM_SSLMAC),
     HASH_MODE(H_SHA1,              HM_SSLMAC),
     HASH_MODE(H_CRC32_I3E802_3,    HM_RAW),
     HASH_MODE(H_MICHAEL,           HM_RAW),
     HASH_MODE(H_SHA3_224,          HM_RAW),
     HASH_MODE(H_SHA3_256,          HM_RAW),
     HASH_MODE(H_SHA3_384,          HM_RAW),
     HASH_MODE(H_SHA3_512,          HM_RAW),
  }
};

static int context_tags[2][64] = {
  {CRYPTO_MODE_NULL,
   CRYPTO_MODE_RC4_40,          // 1
   CRYPTO_MODE_RC4_128,
   CRYPTO_MODE_RC4_KS,
   CRYPTO_MODE_AES_ECB,         // 3
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
   CRYPTO_MODE_3DES_CBC,        // 13
   CRYPTO_MODE_3DES_ECB,        // 14
   CRYPTO_MODE_DES_CBC,         // 15
   CRYPTO_MODE_DES_ECB,         // 16
   CRYPTO_MODE_KASUMI_ECB,
   CRYPTO_MODE_KASUMI_F8,
   CRYPTO_MODE_SNOW3G_UEA2,
   CRYPTO_MODE_ZUC_UEA3,
  },
  {  CRYPTO_MODE_NULL,
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
  }
};

enum {
   DIR_ENC=0,
   DIR_DEC
};

static int mpmdiag_verbose=0;
static char *mode_names[][64] = {
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
   "MICHAEL MIC",
   "HASH_SHA3_224",
   "HASH_SHA3_256",
   "HASH_SHA3_384",
   "HASH_SHA3_512",
 }
};

struct platform_device *mpm_get_platform_device(void);

static void outdiff (unsigned char *buf1, unsigned char *buf2, int len)
{
#if 1
  int x, y;
  char buf[16];
  printk("outdiff len == %d\n", len);
  for (x = 0; x < len; x += 4) {
    if (memcmp(buf1, buf2, 4) == 0) { buf1 += 4; buf2 += 4; continue; }
    sprintf (buf, "%04x   ", x);
    printk (buf);
    for (y = 0; y < 4; y++) {
      sprintf (buf, "%02lx", (unsigned long) buf1[y] & 255UL);
      printk (buf);
    }
    printk (" - ");
    for (y = 0; y < 4; y++) {
      sprintf (buf, "%02lx", (unsigned long) buf2[y] & 255UL);
      printk (buf);
    }
    printk (" = ");
    for (y = 0; y < 4; y++) {
      sprintf (buf, "%02lx", (unsigned long) (buf1[y] ^ buf2[y]) & 255UL);
      printk (buf);
    }
    printk ("\n");
    buf1 += 4;
    buf2 += 4;
    /* exit early */
    //x=len;
  }
#endif
}

static volatile uint32_t job_status;

static void mpmdiag_cb(void *mpm, void *data, int pdu, int key, uint32_t status)
{
   if (PDU_DESC_STATUS_RET_CODE(status) != 0) {
      printk(KERN_DEBUG "Status == %p, %d, %d, %.8lx\n", mpm, pdu, key, (unsigned long)status);
   }
   job_status = status;
}


static int mpm_vector(vector_data *vec, int dir)
{
   struct device *dev = &mpm_get_platform_device()->dev;
   uint32_t     in_buff_size = 0;
   uint32_t     aux_info;
   ktime_t starttime;

   mpm_device *mpm;

   int err = 0, y = 0, pdu = -1, key = -1, inv_key, ctx[1];
   uint32_t icvoff = 0;
   uint32_t proc_sz = 0, ckey_keylen, hmac_keylen;

   ctx[0] = -1;

   job_status = 0;

   y = -1;
   //use common parser to get the data
   err = parse_test_vector (vec);
   if (err!=0){
      printk("parse_test_vector FAILED!!!\n");
      return y;
   }
vec->virt = 0;
   mpm = mpm_get_device_by_num(vec->virt);

   // do we need to update context_modes based on v4.15+ spaccs?
   if (mpm->spacc->config.version >= 0x4F) {
      /* Little explanation here ...
         The context_modes[1][] table has CTRL bits for hash alg/mode only.  In the
         <= v4.14 cores the hash algorithm took 4 bits.
         In >= v4.15 cores it takes 5 bits and is shifted down by 1.
         So we mask off the hash mode (~0x1F0) and then shift the alg down one.
         Since the table only has hash related data we can safely assume that 0x1F0 is a valid
         mask for all hash algorithms
      */
      if (context_table_update == 0) {
         unsigned x, y;
         for (x = 0; x < (sizeof(context_modes[1])/sizeof(context_modes[1][0])); x++) {
            y = context_modes[1][x];
            context_modes[1][x] = (y & ~0x1F0) | ((y & 0x1F0) >> 1);
         }
         context_table_update = 1;
      }
   }

   // allocate context to spacc
   err = mpm_req_spacc_ctx(mpm, 1, ctx);
   if (err) { printk(KERN_DEBUG "mpmdiag: Error allocating a context to the MPM\n"); y = 1; goto ERR; }

   // allocate a pdu and key
   pdu = mpm_alloc_pdu(mpm);
   if (pdu < 0) {
      printk("mpmdiag: Out of PDU handles\n");
      y = 1;
      goto ERR;
   }
   key = mpm_alloc_key(mpm);
   if (key < 0) {
      printk("mpmdiag: Out of KEY handles\n");
      y = 1;
      goto ERR;
   }

   // setup XCBC keys for MPM ...
   if (context_tags[MODE_HASH][vec->hash_mode] == CRYPTO_MODE_MAC_XCBC) {
      if (vec->hmac_keysize != 64) {
         memmove(&vec->hmac_key[64-32], &vec->hmac_key[vec->hmac_keysize-32], 32);
      }
      vec->hmac_keysize -= 32;
      hmac_keylen = 64;
   } else {
      hmac_keylen = vec->hmac_keysize;
   }

   // fill in key context
   inv_key = 0;
   // we embed 3GPP IVs in the key buffer
   if (context_tags[MODE_ENCRYPT][vec->enc_mode] == CRYPTO_MODE_SNOW3G_UEA2 ||
       context_tags[MODE_ENCRYPT][vec->enc_mode] == CRYPTO_MODE_ZUC_UEA3) {
       vec->keysize = 16;
       memcpy(vec->iv, vec->key+16, 16);
   }

   if (context_tags[MODE_HASH][vec->hash_mode] == CRYPTO_MODE_MAC_SNOW3G_UIA2 ||
       context_tags[MODE_HASH][vec->hash_mode] == CRYPTO_MODE_MAC_ZUC_UIA3) {
       vec->hmac_keysize = 16;
   }

   // set the last 4 bytes of the IV since it's not stored in the vector
   if (context_tags[MODE_ENCRYPT][vec->enc_mode] == CRYPTO_MODE_AES_GCM) {
      vec->iv[12] = 0;
      vec->iv[13] = 0;
      vec->iv[14] = 0;
      vec->iv[15] = 1;
   }

   if (context_tags[MODE_ENCRYPT][vec->enc_mode] == CRYPTO_MODE_AES_F8) {
       unsigned char tmp[128];
       memcpy(tmp+32,   vec->saltkey, vec->salt_size);
       memcpy(tmp,      vec->key,     vec->keysize);
       memcpy(vec->key, tmp, 64);
       ckey_keylen = 64;
   } else if (context_tags[MODE_ENCRYPT][vec->enc_mode] == CRYPTO_MODE_AES_XTS) {
       unsigned char tmp[128];
       memcpy(tmp,    vec->key, vec->keysize>>1);
       memcpy(tmp+32, vec->key+(vec->keysize>>1), vec->keysize>>1);
       memcpy(vec->key, tmp, 64);
       vec->keysize >>= 1;
       ckey_keylen = 64;
   } else {
       ckey_keylen = vec->keysize;
   }

   mpm_set_key(mpm, key, vec->keysize, ckey_keylen, vec->hmac_keysize, hmac_keylen, 1, inv_key, vec->key, vec->hmac_key);

   /* Add sources */
   if (vec->pre_aad) {
      memcpy(inbuf.virt, vec->pre_aad, vec->pre_aad_size);
      in_buff_size += vec->pre_aad_size;
   }

   if (dir == DIR_ENC) {
     memcpy(inbuf.virt + in_buff_size, vec->pt, vec->pt_size);
     in_buff_size += vec->pt_size;
   } else {
      if ((vec->icv_mode != ICV_HASH_ENCRYPT)) {
         memcpy(inbuf.virt + in_buff_size, vec->ct, vec->pt_size);
         in_buff_size += vec->pt_size;
      } else {
         memcpy(inbuf.virt + in_buff_size, vec->ct, vec->ct_size);
         in_buff_size += vec->ct_size;
      }
   }

   if (vec->post_aad) {
     memcpy(inbuf.virt + in_buff_size, vec->post_aad, vec->post_aad_size);
     in_buff_size += vec->post_aad_size;
   }

   if (vec->icv_offset == -1) {
      vec->icvpos = IP_ICV_APPEND;
      icvoff = vec->pre_aad_size + vec->pt_size + vec->post_aad_size;
   } else {
      vec->icvpos = IP_ICV_OFFSET;
      icvoff = vec->icv_offset;
   }

   // in decrypt mode we stick the icv after all this nonsense
   if (vec->icv_size > 0) {
     if (dir == DIR_DEC) {
        if ((vec->icv_mode != ICV_HASH_ENCRYPT)) {
           memcpy(inbuf.virt + icvoff, vec->icv, vec->icv_size);
           in_buff_size = icvoff + vec->icv_size;
        }
     } else { // dir == DIR_ENC
     }
   } else {
     if ((context_tags[MODE_HASH][vec->hash_mode] != CRYPTO_MODE_NULL)) {
        if ((dir == DIR_DEC) && (vec->ct_size > vec->pt_size)) {
           memcpy(inbuf.virt + icvoff, vec->ct + vec->pt_size, vec->ct_size - vec->pt_size);
        }
         in_buff_size = icvoff + vec->ct_size - vec->pt_size;
     } else {
     }
   }

   if (vec->iv_offset & IV_OFFSET_EN_MASK) {
     if (in_buff_size < ((vec->iv_offset & IV_OFFSET_MASK) + vec->iv_size)) {
         in_buff_size = (vec->iv_offset & IV_OFFSET_MASK) + vec->iv_size;
     }
      memcpy(inbuf.virt + (vec->iv_offset & IV_OFFSET_MASK), vec->iv, vec->iv_size);
   }

   dma_sync_sg_for_device(dev, inbuf.sgt.sgl, inbuf.sgt.nents, DMA_TO_DEVICE);

   // proclen register
   proc_sz = vec->pre_aad_size + vec->pt_size;

   //add icv_len if we're in ICV_ENC mode
   if ((vec->icv_mode==1) && (dir != DIR_ENC)) {
      proc_sz += vec->icv_size;
   }


   // subtract ICV if in decrypt and using GCM/CCM
#warning Should use table lookup for modes to tags ...
   if ((dir != DIR_ENC) &&
       ((vec->hash_mode > 0) || (vec->enc_mode == CRYPTO_MODE_AES_CCM || vec->enc_mode == CRYPTO_MODE_AES_GCM)) &&
       !(vec->icv_mode & 2)) {
//        proc_sz -= vec->icv_size;
   }

   // auxinfo
   aux_info =  (vec->auxinfo_dir << _SPACC_AUX_INFO_DIR) | (vec->auxinfo_bit_align << _SPACC_AUX_INFO_BIT_ALIGN);

   switch (context_tags[MODE_ENCRYPT][vec->enc_mode]) {
      case CRYPTO_MODE_AES_CS1: aux_info |= AUX_INFO_SET_CBC_CS(1); break;
      case CRYPTO_MODE_AES_CS2: aux_info |= AUX_INFO_SET_CBC_CS(2); break;
      case CRYPTO_MODE_AES_CS3: aux_info |= AUX_INFO_SET_CBC_CS(3); break;
   }

   /* insert pdu */
   err = mpm_insert_pdu(mpm,
             pdu, key, 0, mpmdiag_cb, NULL,
             &inbuf.ddt, &outbuf.ddt, vec->pre_aad_size, vec->post_aad_size, proc_sz,
             vec->icv_size, (vec->icv_offset == 0xFFFFFFFF) ? 0 : (vec->icv_offset & 0x7FFFFFFF), (vec->iv_offset == 0xFFFFFFFF) ? 0 : (vec->iv_offset & 0x7FFFFFFF),
             aux_info,
             context_modes[0][vec->enc_mode] | context_modes[1][vec->hash_mode] | ((dir == DIR_ENC) ? PDU_DESC_CTRL_ENCRYPT:0) | icv_modes[vec->icv_mode],
             vec->iv, vec->hmac_key + 16);
   if (err < 0) {
      printk("mpmdiag:  Failed to insert vector\n");
      err = 1;
      goto ERR;
   }

   err = mpm_enqueue_chain(mpm, NULL, 0);
   if (err < 0) {
      printk("mpmdiag:  Failed to enqueue vector\n");
      err = 1;
      goto ERR;
   }
   mpm_kernel_schedule_tasklet_by_num(vec->virt);

   starttime = ktime_get();
   while (!job_status) {
      cond_resched();
      if (ktime_us_delta(ktime_get(), starttime) > 5000000) {
         printk("mpmdiag:  Timed out waiting for job\n");
         y = 1;
         goto ERR;
      }
   }
   mpm_free_key(mpm, key);
   key = -1;

   dma_sync_sg_for_cpu(dev, outbuf.sgt.sgl, outbuf.sgt.nents, DMA_FROM_DEVICE);

   if (PDU_DESC_STATUS_RET_CODE(job_status) != 0) {
      printk("MPM: Vector failed with return code %x\n", PDU_DESC_STATUS_RET_CODE(job_status));
      y = 1;
      goto ERR;
   }


   // zero out unused bits in the last byte returned of the stream cipher for SNOW3G and Kasumi
   if (vec->auxinfo_bit_align != 8 && (vec->hash_mode == CRYPTO_MODE_MAC_KASUMI_F9 || vec->hash_mode == CRYPTO_MODE_MAC_SNOW3G_UIA2) && vec->post_aad_size == 0) {
      if (dir == DIR_ENC) {
         outbuf.virt[vec->ct_size - vec->icv_size - 1] &= 0xFF << (8 - vec->auxinfo_bit_align);
      } else {
         outbuf.virt[vec->pt_size - 1] &= 0xFF << (8 - vec->auxinfo_bit_align);
      }
   }


   // compare data
   y = 0;
   if (dir == DIR_ENC) {
      if (vec->icvpos == IP_ICV_OFFSET) {
         if (memcmp (outbuf.virt, vec->ct, vec->ct_size - vec->icv_size)) {
            printk(KERN_DEBUG "MPM: mpm_diag:: Comparing CT failed[%d]\n", __LINE__);
            if(vec->fail_mode == 0) {
               outdiff (outbuf.virt, vec->ct, vec->ct_size - vec->icv_size);
            }
            printk(KERN_DEBUG "MPM: mpm_diag:: fail_mode == %d\n", vec->fail_mode);
            y = 1;
         }
         if (memcmp (outbuf.virt + icvoff, vec->icv, vec->icv_size)) {
            printk(KERN_DEBUG "MPM: mpm_diag:: Comparing ICV failed[%d]\n", __LINE__);
            if(vec->fail_mode == 0) {
               outdiff (outbuf.virt + icvoff, vec->icv, vec->icv_size);
            }
            printk(KERN_DEBUG "MPM: mpm_diag:: fail_mode == %d\n", vec->fail_mode);
            y = 1;
         }
      } else { // IP_ICV_APPEND
         if (memcmp (outbuf.virt, vec->ct, vec->ct_size)) {
            printk(KERN_DEBUG "MPM: mpm_diag:: Comparing ICV/CT failed[%d]\n", __LINE__);
            if(vec->fail_mode == 0) {
               outdiff (outbuf.virt, vec->ct, vec->ct_size);
            }
            printk(KERN_DEBUG "MPM: mpm_diag:: fail_mode == %d\n", vec->fail_mode);
            y = 1;
         }
      }
   } else {
      if (memcmp (outbuf.virt, vec->pt, vec->pt_size)) {
         printk(KERN_DEBUG "MPM: mpm_diag:: Comparing PT failed[%d]\n", __LINE__);
         if(vec->fail_mode == 0){
            outdiff (outbuf.virt, vec->pt, vec->pt_size);
         }
         printk(KERN_DEBUG "MPM: mpm_diag:: fail_mode == %d\n", vec->fail_mode);
         y = 1;
      }
   }
ERR:
  if (y || mpmdiag_verbose) {
     printk(KERN_DEBUG "MPM: %s Vector %s-%s %s\n", dir == DIR_ENC ? "ENCRYPT" : "DECRYPT", mode_names[0][vec->enc_mode], mode_names[1][vec->hash_mode], (y == 0) ? "[PASSED]" : "[FAILED]");
  }


  if (key > 0) {
     mpm_free_key(mpm, key);
  }

  mpm_free_spacc_ctx(mpm, 1, ctx);

  return y;
}

static vector_data *diag_vec;

static ssize_t store_test_name(struct file *fp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t offset, size_t count)
{
   size_t n;

   if (offset == 0) {
      diag_vec = init_test_vector(SPACC_MAX_MSG_MALLOC_SIZE);
      if (diag_vec == NULL) {
         printk("Error allocating and initializing vector data\n");
         return -ENODEV;
      }
   }

   n = elp_parser_fwrite(buf, count);
   if (n != count || count != PAGE_SIZE) {
      // run vector;
      n  = mpm_vector(diag_vec, 0);
      elp_parse_rewind();
      if (!n) { n = mpm_vector(diag_vec, 1); }
      free_test_vector(diag_vec);
      return n ? -EBADMSG : count;
   } else {
      return n;
   }
}

static BIN_ATTR(test_name, 0222, NULL, store_test_name, 0);

/* Allocate large buffers for testing.  Inspired by v4l. */
static int alloc_buffer_pages(struct test_buffer *buf, size_t size)
{
   size_t last_page = 0;

   buf->num_pages = DIV_ROUND_UP(size, PAGE_SIZE);
   buf->pages = kzalloc(buf->num_pages * sizeof *buf->pages, GFP_KERNEL);
   if (!buf->pages)
      return -ENOMEM;

   while (size > 0) {
      struct page *pages = NULL;
      int order = get_order(size);
      int i;

      /* Prefer more fragments to over-allocation */
      if (order && (PAGE_SIZE << order) > size)
         order--;

      while (!pages) {
         pages = alloc_pages(GFP_KERNEL | __GFP_ZERO | __GFP_NOWARN, order);
         if (pages)
            break;

         if (order == 0)
            goto err_unwind;
         order--;
      }

      split_page(pages, order);
      for (i = 0; i < (1 << order); i++)
         buf->pages[last_page++] = &pages[i];
      size -= min_t(size_t, size, PAGE_SIZE << order);
   }

   return 0;
err_unwind:
   while (last_page--)
      __free_page(buf->pages[last_page]);
   kfree(buf->pages);
   return -ENOMEM;
}

static int alloc_buffer(struct device *dev, struct test_buffer *buf,
                        size_t size, int dir)
{
   struct scatterlist *sg;
   size_t i, ddt_count;
   int rc;

   rc = alloc_buffer_pages(buf, size);
   if (rc < 0)
      return rc;

   buf->virt = vmap(buf->pages, buf->num_pages, 0, PAGE_KERNEL);
   if (!buf->virt) {
      rc = -ENOMEM;
      goto err_free_pages;
   }

   rc = sg_alloc_table_from_pages(&buf->sgt, buf->pages, buf->num_pages,
                                             0, size, GFP_KERNEL);
   if (rc < 0)
      goto err_free_pages;

   rc = dma_map_sg(dev, buf->sgt.sgl, buf->sgt.orig_nents, dir);
   if (rc < 0)
      goto err_free_sg;
   buf->sgt.nents = rc;

   /*
    * Figure out how many 64K chunks are needed
    * (TODO: make this depend on H/W config)
    */
   ddt_count = 0;
   for_each_sg(buf->sgt.sgl, sg, buf->sgt.nents, i) {
      ddt_count += DIV_ROUND_UP(sg_dma_len(sg), 65536);
   }

   rc = pdu_ddt_init(&buf->ddt, ddt_count);
   if (rc < 0)
      goto err_unmap_sg;

   /* Break SG into 64K chunks and add to DDT */
   for_each_sg(buf->sgt.sgl, sg, buf->sgt.nents, i) {
      uint32_t addr = sg_dma_address(sg), size = sg_dma_len(sg);

      while (size > 0) {
         uint32_t x = size;

         if (x > 65536)
            x = 65536;

         pdu_ddt_add(&buf->ddt, addr, x);
         addr += x;
         size -= x;
      }
   }

   return 0;
err_unmap_sg:
   dma_unmap_sg(dev, buf->sgt.sgl, buf->sgt.orig_nents, dir);
err_free_sg:
   sg_free_table(&buf->sgt);
err_free_pages:
   vunmap(buf->virt);
   for (i = 0; i < buf->num_pages; i++)
      __free_page(buf->pages[i]);
   kfree(buf->pages);

   return rc;
}

static void free_buffer(struct device *dev, struct test_buffer *buf, int dir)
{
   size_t i;

   pdu_ddt_free(&buf->ddt);
   dma_unmap_sg(dev, buf->sgt.sgl, buf->sgt.orig_nents, dir);
   sg_free_table(&buf->sgt);

   vunmap(buf->virt);
   for (i = 0; i < buf->num_pages; i++)
      __free_page(buf->pages[i]);
   kfree(buf->pages);
}

static int __init mpmdiag_mod_init(void)
{
   struct device *dev = &mpm_get_platform_device()->dev;
   int rc;

   rc = alloc_buffer(dev, &inbuf, SPACC_MAX_MSG_MALLOC_SIZE, DMA_TO_DEVICE);
   if (rc < 0) {
      return rc;
   }

   rc = alloc_buffer(dev, &outbuf, SPACC_MAX_MSG_MALLOC_SIZE, DMA_FROM_DEVICE);
   if (rc < 0) {
      free_buffer(dev, &inbuf, DMA_TO_DEVICE);
      return rc;
   }

   rc = sysfs_create_bin_file(&dev->kobj, &bin_attr_test_name);
   if (rc < 0) {
      free_buffer(dev, &outbuf, DMA_FROM_DEVICE);
      free_buffer(dev, &inbuf, DMA_TO_DEVICE);
      return rc;
   }

   return 0;
}
module_init (mpmdiag_mod_init);

static void __exit mpmdiag_mod_exit (void)
{
   struct device *dev = &mpm_get_platform_device()->dev;

   sysfs_remove_bin_file(&dev->kobj, &bin_attr_test_name);
   free_buffer(dev, &outbuf, DMA_FROM_DEVICE);
   free_buffer(dev, &inbuf, DMA_TO_DEVICE);
}
module_exit (mpmdiag_mod_exit);

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");

module_param(mpmdiag_verbose, int, 0);
MODULE_PARM_DESC(mpmdiag_verbose, "Enable verbose vector parsing (disabled by default)");
