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
#include "elpspaccdrv.h"
#include "elpspacc.h"
#include "elpparser.h"
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

static spacc_device *myspacc;
static pdu_ddt src_ddt, dst_ddt;
static unsigned char *OUTCT[SPACC_MAX_MSG_MALLOC_SIZE/SPACC_MAX_PARTICLE_SIZE], *IN_BUFF[SPACC_MAX_MSG_MALLOC_SIZE/SPACC_MAX_PARTICLE_SIZE];

#ifndef MIN
   #define MIN(x,y) (((x)<(y))?(x):(y))
#endif

#define MODE_ENCRYPT 0
#define MODE_HASH    1

#define IV_OFFSET_MASK 0x7fffffff
#define IV_OFFSET_EN_MASK 0x80000000

static int context_modes[2][32] = {
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
  }
};

enum {
   DIR_ENC=0,
   DIR_DEC
};

int spaccdiag_verbose=0;
static char *mode_names[][32] = {
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
 }
};

static void outdiff (unsigned char **buf1, unsigned char *buf2, int len)
{
  int x, y, z, t;
  unsigned char buf[16], *p;

  z = 0;
  p = buf1[z];
  t = 0;
  printk("outdiff len == %d\n", len);
  for (x = 0; x < len; x += 4) {
     if (x && (x % SPACC_MAX_PARTICLE_SIZE) == 0) {
        p = buf1[++z];
     }
    if (memcmp(p, buf2, 4) == 0) { p += 4; buf2 += 4; continue; }
    sprintf (buf, "%04x   ", x);
    printk (buf);
    for (y = 0; y < 4; y++) {
      sprintf (buf, "%02lx", (unsigned long) p[y] & 255UL);
      printk (buf);
    }
    printk (" - ");
    for (y = 0; y < 4; y++) {
      sprintf (buf, "%02lx", (unsigned long) buf2[y] & 255UL);
      printk (buf);
    }
    printk (" = ");
    for (y = 0; y < 4; y++) {
      sprintf (buf, "%02lx", (unsigned long) (p[y] ^ buf2[y]) & 255UL);
      printk (buf);
    }
    printk ("\n");
    if (++t == 1000) {
       printk("Suffice it to say the vector failed...\n");
       return;
    }

    p    += 4;
    buf2 += 4;
    /* exit early */
    //x=len;
  }
}



/****** Partial Packet Example code ******/
#ifdef SPACCDIAG_PARTIAL
enum {
   partial_fsm_pre_aad=0,
   partial_fsm_pre_aad_with_pt,
   partial_fsm_pt_only,
   partial_fsm_pt_end,
};

struct partial_job {
   spacc_device   *spacc;
   unsigned long  pre_ad_len,
                  post_ad_len,
                  pt_len,
                  ct_len,
                  icv_len,
                  iv_offset,
                  prio;

   int ciphermode,
       hashmode,
       aad_copy,
       icv_mode,
       dir,
       done,
       handle,
       state;

   unsigned long  pos,
                  dstpos,
                  blksize;
};

static void partial_job_init(
   spacc_device *spacc,
   int ciphermode,
   int hashmode,
   int dir,
   int handle,
   unsigned long pre_ad_len,
   unsigned long post_ad_len,
   int aad_copy,
   unsigned long pt_len,
   unsigned long ct_len,
   unsigned long icv_len,
   int icv_mode,
   unsigned long iv_offset,
   unsigned long prio,
   struct partial_job *job)
{
   *job = (struct partial_job)
                            { .spacc       = spacc,
                              .ciphermode  = ciphermode,
                              .hashmode    = hashmode,
                              .dir         = dir,
                              .handle      = handle,
                              .pre_ad_len  = pre_ad_len,
                              .post_ad_len = post_ad_len,
                              .aad_copy    = aad_copy,
                              .pt_len      = pt_len,
                              .ct_len      = ct_len,
                              .icv_len     = icv_len,
                              .icv_mode    = icv_mode,
                              .iv_offset   = iv_offset,
                              .prio        = prio,
                              .pos         = 0,
                              .dstpos      = 0,
                              .blksize     = 1024,
                              .done        = 0,
                              .state       = partial_fsm_pre_aad
                            };
}

static int partial_next(struct partial_job *job)
{
   int                   ret;
   unsigned long         r, n;
   elp_spacc_handle_ctx *ctx;

   // get context
   ctx = context_lookup (job->spacc, job->handle);
   if ( NULL == ctx ) {
     return CRYPTO_FAILED;
   }

   // reset BEGIN|END flags
   ctx->ctrl &= ~((1UL<<_SPACC_CTRL_MSG_BEGIN)|(1UL<<_SPACC_CTRL_MSG_END));

   // where are we
   if (job->pos < job->pre_ad_len) {
      /* we're either in partial_fsm_pre_aad or  partial_fsm_pre_aad_with_pt
         rule is simple,
            1.  if we're in SSLMAC then partial_fsm_pre_aad_with_pt is required (and must include at least 64 bytes of PT)
            2.  If not SSLMAC
                2.1  if less than 128 bytes remaining, partial_fsm_pre_aad_with_pt
                2.2  otherwise partial_fsm_pre_aad
      */
      if (job->hashmode == CRYPTO_MODE_SSLMAC_MD5 || job->hashmode == CRYPTO_MODE_SSLMAC_SHA1 || job->pre_ad_len < 128) {
         /* SSLMAC mode */
         job->state = partial_fsm_pre_aad_with_pt;
      } else {
         job->state = partial_fsm_pre_aad;
      }
   } else {
      /* we're either in  partial_fsm_pt_only,  partial_fsm_pt_with_postaad, partial_fsm_pt_with_icv
         rules
            1. if remaining pt > blksize, then partial_fsm_pt_only
            2. else
                2.1 if there is postaad, include at least 128 bytes and go partial_fsm_pt_with_postaad
                2.2 if there is no postaad AND we're in decrypt mode go partial_fsm_pt_with_icv
      */
      if ((job->pt_len - (job->pos - job->pre_ad_len)) > job->blksize) {
         job->state = partial_fsm_pt_only;
      } else {
         job->state = partial_fsm_pt_end;
      }
   }

   if (job->pos == 0) {
      ctx->ctrl |= (1UL<<_SPACC_CTRL_MSG_BEGIN);
   }

   // states
   r = n = 0;
   switch (job->state) {
      case partial_fsm_pre_aad:
         // enqueue a job of just AAD
         n = job->pre_ad_len - job->pos;
         if (n > job->blksize) {
            n = job->blksize;
         }

         /* last block? */
         if ((job->pos + n) == job->pre_ad_len && job->pt_len == 0) {
            ctx->ctrl |= (1UL<<_SPACC_CTRL_MSG_END);
            job->done = 1;
         }

         ret = spacc_packet_enqueue_ddt(job->spacc, job->handle, n, (job->pos<<SPACC_OFFSET_SRC_O)|(job->dstpos<<SPACC_OFFSET_DST_O), n, 0, job->iv_offset, job->prio);
         job->pos += n;

         // if we are copying AAD ...
         if (job->aad_copy) {
            job->dstpos += n;
         }

         break;

      case partial_fsm_pre_aad_with_pt:
         n = job->pre_ad_len; // we add the entire pre_aad in

      case partial_fsm_pt_only:
         r = job->pt_len - ((job->pos == 0) ? 0 : (job->pos - job->pre_ad_len));
         if (r > job->blksize) {
            r = job->blksize;
         }

         // payload must be multiple of 128
         if (job->pt_len > 128) {
            r -= (r & 127);
         }

         if (r == job->pt_len) {
            r += job->post_ad_len;
            ctx->ctrl |= (1UL<<_SPACC_CTRL_MSG_END);
            job->done = 1;

            if (job->ciphermode != CRYPTO_MODE_NULL && job->dir == DIR_DEC && job->icv_mode == 1) {
               r += job->icv_len;
            }
         }

         // add any aad to payload
         n += r;

         /* last block? */
         if ((job->pos + n) == (job->pre_ad_len + job->pt_len) && job->post_ad_len == 0 && (job->dir == DIR_ENC || job->icv_len == 0)) {
            ctx->ctrl |= (1UL<<_SPACC_CTRL_MSG_END);
            job->done = 1;
         }

         ret = spacc_packet_enqueue_ddt(job->spacc, job->handle, n, (job->pos<<SPACC_OFFSET_SRC_O)|(job->dstpos<<SPACC_OFFSET_DST_O), n-r, job->done ? job->post_ad_len : 0, job->iv_offset, job->prio);
         job->pos += n;

         /* advance destination position */
         job->dstpos += r;

         // if we are copying AAD ...
         if (job->aad_copy) {
            job->dstpos += (n-r);
         }
         break;

      case partial_fsm_pt_end:
         n  = job->pt_len - (job->pos - job->pre_ad_len);
         n += job->post_ad_len;

         if (job->ciphermode != CRYPTO_MODE_NULL && job->dir == DIR_DEC && job->icv_mode == 1) {
            n += job->icv_len;
         }

         // always last block at this point
         ctx->ctrl |= (1UL<<_SPACC_CTRL_MSG_END);

         ret = spacc_packet_enqueue_ddt(job->spacc, job->handle, n, (job->pos<<SPACC_OFFSET_SRC_O)|(job->dstpos<<SPACC_OFFSET_DST_O), SPACC_AUTO_SIZE, job->post_ad_len, job->iv_offset, job->prio);

         job->done = 1;
         break;
   }

   return ret;
}
/****** End of Partial Packet Example code ******/
#endif

/****** Program the SKP backend ******/
static int skp_backend(spacc_device *spacc, vector_data *vec, int dir, int ctx)
{
   uint32_t alg, mode, size, enc, dec;

   alg = mode = size = enc = dec = 0;

   switch (context_modes[MODE_ENCRYPT][vec->enc_mode]) {
      case CRYPTO_MODE_NULL:          break;
      case CRYPTO_MODE_RC4_40:
      case CRYPTO_MODE_RC4_128:       alg = C_RC4; mode = CM_ECB; break;
      case CRYPTO_MODE_AES_ECB:       alg = C_AES; mode = CM_ECB; break;
      case CRYPTO_MODE_AES_CBC:       alg = C_AES; mode = CM_CBC; break;
      case CRYPTO_MODE_AES_CTR:       alg = C_AES; mode = CM_CTR; break;
      case CRYPTO_MODE_AES_CCM:       alg = C_AES; mode = CM_CCM; break;
      case CRYPTO_MODE_AES_GCM:       alg = C_AES; mode = CM_GCM; break;
      case CRYPTO_MODE_AES_F8:        alg = C_AES; mode = CM_F8;  break;
      case CRYPTO_MODE_AES_XTS:       alg = C_AES; mode = CM_XTS; break;
      case CRYPTO_MODE_AES_CFB:       alg = C_AES; mode = CM_CFB; break;
      case CRYPTO_MODE_AES_OFB:       alg = C_AES; mode = CM_OFB; break;
      case CRYPTO_MODE_MULTI2_ECB:    alg = C_MULTI2; mode = CM_ECB; break;
      case CRYPTO_MODE_MULTI2_CBC:    alg = C_MULTI2; mode = CM_CBC; break;
      case CRYPTO_MODE_MULTI2_OFB:    alg = C_MULTI2; mode = CM_OFB; break;
      case CRYPTO_MODE_MULTI2_CFB:    alg = C_MULTI2; mode = CM_CFB; break;
      case CRYPTO_MODE_3DES_CBC:
      case CRYPTO_MODE_DES_CBC:       alg = C_DES; mode = CM_CBC; break;
      case CRYPTO_MODE_3DES_ECB:
      case CRYPTO_MODE_DES_ECB:       alg = C_DES; mode = CM_ECB; break;
      case CRYPTO_MODE_KASUMI_ECB:    alg = C_KASUMI; mode = CM_ECB; break;
      case CRYPTO_MODE_KASUMI_F8:     alg = C_KASUMI; mode = CM_CBC; break;
      case CRYPTO_MODE_SNOW3G_UEA2:   alg = C_SNOW3G_UEA2; mode = CM_ECB; break;
      default:
         return -1;
   }

   switch (vec->keysize) {
      case 8:
      case 16: size = 0; break;
      case 24: size = 1; break;
      case 32: size = 2; break;
      default:
         return -1;
   }

   // ENCRYPT bit
   if (dir == DIR_ENC) {
      enc = 1;
   } else {
      dec = 1;
   }

   return spacc_load_skp(spacc, (uint32_t *)vec->key, vec->keysize/4, ctx, alg, mode, size, enc, dec);
}

static void spaccback(void *d, void *s)
{
   complete(s);
}

static void buf_insert(spacc_device *spacc, const unsigned char *buf, uint32_t inlen, uint32_t *outlen)
{
   uint32_t idx, off;

//   printk("%zu + %zu == %zu\n", *outlen, inlen, SPACC_MAX_MSG_MALLOC_SIZE - (*outlen + inlen));
   if (*outlen + inlen >= SPACC_MAX_MSG_MALLOC_SIZE) {
      printk("Overflow of buffer\n");
      return;
   }

   idx = *outlen / SPACC_MAX_PARTICLE_SIZE;
   off = *outlen % SPACC_MAX_PARTICLE_SIZE;
   (*outlen) += inlen;
   while (inlen--) {
      IN_BUFF[idx][off++] = *buf++;
      if (off == SPACC_MAX_PARTICLE_SIZE) {
         ++idx;
         off = 0;
      }
   }
}

static int buf_compare(unsigned char **src, const unsigned char *orig, uint32_t len)
{
   uint32_t x, y, d;
   int      r;

   x = 0;
   d = 0;
   while (len) {
      y = MIN(SPACC_MAX_PARTICLE_SIZE, len);
      r = memcmp(src[x], orig, y);
      if (r) {
         printk("diff at %zu\n", d);
         return r;
      }
      orig += y;
      len  -= y;
      ++x;
      d += y;
   }
   return 0;
}

static int buf_compare_offset(unsigned char **src, uint32_t off, const unsigned char *orig, uint32_t len)
{
   uint32_t x, y;

   x = off / SPACC_MAX_PARTICLE_SIZE;
   y = off % SPACC_MAX_PARTICLE_SIZE;

   while (len) {
      if (src[x][y] != *orig) {
         return -1;
      }
      ++orig;
      ++y;
      if (y == SPACC_MAX_PARTICLE_SIZE) {
         y = 0;
         ++x;
      }
      --len;
   }
   return 0;
}

static int alloc_and_map(unsigned char **ptr, pdu_ddt *ddt)
{
   uint32_t x;

   printk("spaccdiag::Allocating %lu buffer\n", SPACC_MAX_MSG_MALLOC_SIZE);
   if (pdu_ddt_init(ddt, 1+(SPACC_MAX_MSG_MALLOC_SIZE/SPACC_MAX_PARTICLE_SIZE)) < 0) {
      return -ENOMEM;
   }
   for (x = 0; x < (SPACC_MAX_MSG_MALLOC_SIZE/SPACC_MAX_PARTICLE_SIZE); x++) {
     PDU_DMA_ADDR_T tmp;
     ptr[x] = kmalloc(SPACC_MAX_PARTICLE_SIZE, GFP_KERNEL);
     if (ptr[x]) {
        tmp = dma_map_single(NULL, ptr[x], SPACC_MAX_PARTICLE_SIZE, DMA_BIDIRECTIONAL);
        pdu_ddt_add(ddt, tmp, SPACC_MAX_PARTICLE_SIZE);
     } else {
        tmp = 0;
     }
     printk("spaccdiag::Frag %2d == %p @ %zu\n", x, ptr[x], ddt->virt[2*x+0]);
     if (!tmp || !ptr[x]) {
        int y;
        for (y = 0; y < x; y++) {
           dma_unmap_single(NULL, ddt->virt[2*y+0], SPACC_MAX_PARTICLE_SIZE, DMA_BIDIRECTIONAL);
           kfree(ptr[y]);
           ptr[y] = NULL;
        }
        if (!tmp && ptr[x]) {
           kfree(ptr[x]);
           ptr[x] = NULL;
        }
        pdu_ddt_free(ddt);
        return -ENOMEM;
     }
     memset(ptr[x], 0, SPACC_MAX_PARTICLE_SIZE);
   }
   return 0;
}

static int parse_test_script (char * filename, int dir, int secure_mode, int normal_mode)
{
  struct completion   comp;
  uint32_t            OUTCTsize;
  uint32_t            in_buff_size = 0;
  uint32_t            out_buff_size = 0;
  vector_data    *vec;  //the current vector

  struct platform_device   *pdev_spacc;
  spacc_device *spacc;

  int            handle = 0, err = 0, y = 0;
  int tries = 0;
#ifdef SPACCDIAG_PARTIAL
   struct partial_job job;
   elp_spacc_handle_ctx *ctx;
#endif
   int ctxid = -1;
   uint32_t sec_key = 0;
   uint32_t icvoff = 0;
   uint32_t proc_sz = 0;

  vec = init_test_vector(SPACC_MAX_MSG_MALLOC_SIZE);
  if (vec == NULL) {
    printk("Error allocating and initializing vector data\n");
    return -ENODEV;
  }

  y = -1;
  //use common parser to get the data
  err = parse_test_vector (filename, vec);
  if (err!=0){
    free_test_vector(vec);
    printk("parse_test_vector FAILED!!![%s]\n", filename);
    return y;
  }

   //attach to the requested spacc
//   printk("\nepn:[%x] virt:[%d]\n", vec->epn, vec->virt);
   pdev_spacc = get_spacc_platdev_by_epn(vec->epn, vec->virt);
   if (!pdev_spacc) {
      printk("Could not find device!\n");
      free_test_vector(vec);
      return -ENODEV;
   }

   /* get the spacc private struct */
   spacc = &((struct spacc_priv *)platform_get_drvdata(pdev_spacc))->spacc;
   myspacc = spacc;

  if (vec->seckey) {
      sec_key = 1;
  } else {
      sec_key = 0;
  }

  if ((handle = spacc_open (spacc, context_modes[MODE_ENCRYPT][vec->enc_mode], context_modes[MODE_HASH][vec->hash_mode], ctxid, secure_mode, spaccback, &comp)) < 0) {
    printk ("ERR[%d]: spacc_open:%d: %s\n", __LINE__, handle,  spacc_error_msg (handle));
    y = 1;
    goto end;
  }

  //take the salt as the first part of the key for F8 mode
  if (context_modes[MODE_ENCRYPT][vec->enc_mode] == CRYPTO_MODE_AES_F8) {
    unsigned char tmp[128];
    memcpy(tmp, vec->saltkey, vec->salt_size);
    memcpy(tmp + vec->salt_size, vec->key, vec->keysize);
    memcpy(vec->key, tmp, vec->keysize+vec->salt_size);
  }

  if ((vec->enc_mode > 0) && (err = spacc_write_context (spacc, handle, SPACC_CRYPTO_OPERATION, vec->key, vec->keysize, vec->iv, vec->c_iv_size)) != CRYPTO_OK) {
    printk ("ERR[%d]: crypto_set_context: %s\n", __LINE__,  spacc_error_msg (err));
    y = 1;
    goto end;
  }
  if ((vec->hash_mode > 0) && (err = spacc_write_context (spacc, handle, SPACC_HASH_OPERATION, vec->hmac_key, vec->hmac_keysize, vec->iv + vec->c_iv_size, vec->h_iv_size)) != CRYPTO_OK) {
    printk ("ERR[%d]: crypto_set_context: %s\n", __LINE__, spacc_error_msg (err));
    y = 1;
    goto end;
  }

  // copy the context into a new handle to test if get/set works
  { int handle2;
    handle2 = spacc_open(spacc, context_modes[MODE_ENCRYPT][vec->enc_mode], context_modes[MODE_HASH][vec->hash_mode], ctxid, secure_mode, spaccback, &comp);
    if (handle2 >= 0) {
       // gracefully carry on if we're out of contexts
       unsigned char keybuf[256], ivbuf[256], hmackeybuf[256], hmacivbuf[256];
       spacc_read_context (spacc, handle, SPACC_CRYPTO_OPERATION, keybuf, vec->keysize, ivbuf, vec->c_iv_size);
       spacc_read_context (spacc, handle, SPACC_HASH_OPERATION, hmackeybuf, vec->hmac_keysize, hmacivbuf, vec->h_iv_size);
       spacc_write_context (spacc, handle2, SPACC_CRYPTO_OPERATION, keybuf, vec->keysize, ivbuf, vec->c_iv_size);
       spacc_write_context (spacc, handle2, SPACC_HASH_OPERATION, hmackeybuf, vec->hmac_keysize, hmacivbuf, vec->h_iv_size);
       spacc_close(spacc, handle);
       handle = handle2;
    }
  }


  if (vec->icv_offset == -1) {
    vec->icvpos = IP_ICV_APPEND;
    icvoff = vec->pre_aad_size + vec->pt_size + vec->post_aad_size;
  } else {
    vec->icvpos = IP_ICV_OFFSET;
    icvoff = vec->icv_offset;
  }


  if ((err = spacc_set_operation (spacc, handle, dir==DIR_ENC?OP_ENCRYPT:OP_DECRYPT, vec->icv_mode, vec->icvpos,
      icvoff, vec->icv_size, sec_key)) != CRYPTO_OK) {
    printk ("ERR[%d]: spacc_set_operation: %s\n", __LINE__, spacc_error_msg (err));
    y = 1;
    goto end;
  }

  // expand key in decrypt mode for everything, then enc mode for RC4 40/128 but not KS
  if ((dir != DIR_ENC || context_modes[MODE_ENCRYPT][vec->enc_mode] == CRYPTO_MODE_RC4_40 || context_modes[MODE_ENCRYPT][vec->enc_mode] == CRYPTO_MODE_RC4_128) &&
      !(context_modes[MODE_ENCRYPT][vec->enc_mode] == CRYPTO_MODE_RC4_KS)) {
     if ((err = spacc_set_key_exp(spacc, handle)) != CRYPTO_OK) {
        printk("ERR[%d]: spacc_set_key_exp: %s\n", __LINE__, spacc_error_msg(err));
        y = 1;
        goto end;
     }
  }

  /* Add sources */
  if (vec->pre_aad) {
      buf_insert(spacc, vec->pre_aad, vec->pre_aad_size, &in_buff_size);
//      memcpy(IN_BUFF, vec->pre_aad, vec->pre_aad_size);
//      in_buff_size += vec->pre_aad_size;
  }

  if (dir == DIR_ENC) {
      buf_insert(spacc, vec->pt, vec->pt_size, &in_buff_size);
//     memcpy(IN_BUFF + in_buff_size, vec->pt, vec->pt_size);
//     in_buff_size += vec->pt_size;
  } else {
      if ((vec->icv_mode != ICV_HASH_ENCRYPT)) {
         buf_insert(spacc, vec->ct, vec->pt_size, &in_buff_size);
//         memcpy(IN_BUFF + in_buff_size, vec->ct, vec->pt_size);
//         in_buff_size += vec->pt_size;
      } else {
         buf_insert(spacc, vec->ct, vec->ct_size, &in_buff_size);
//         memcpy(IN_BUFF + in_buff_size, vec->ct, vec->ct_size);
//         in_buff_size += vec->ct_size;
      }
  }

  if (vec->post_aad) {
      buf_insert(spacc, vec->post_aad, vec->post_aad_size, &in_buff_size);
//     memcpy(IN_BUFF + in_buff_size, vec->post_aad, vec->post_aad_size);
//     in_buff_size += vec->post_aad_size;
  }

  // in decrypt mode we stick the icv after all this nonsense
  if (vec->icv_size > 0) {
     if (dir == DIR_DEC) {
        if ((vec->icv_mode != ICV_HASH_ENCRYPT)) {
           uint32_t tmp = icvoff;
           buf_insert(spacc, vec->icv, vec->icv_size, &tmp);
//           memcpy(IN_BUFF + icvoff, vec->icv, vec->icv_size);
//           in_buff_size = icvoff + vec->icv_size;
        }
        out_buff_size = icvoff + vec->icv_size;

     } else { // dir == DIR_ENC
        out_buff_size = icvoff + vec->icv_size;
     }
  } else {
     if ((context_modes[MODE_HASH][vec->hash_mode] != CRYPTO_MODE_NULL)) {
        if ((dir == DIR_DEC) && (vec->ct_size > vec->pt_size)) {
           uint32_t tmp = icvoff;
           buf_insert(spacc, vec->ct + vec->pt_size, vec->ct_size - vec->pt_size, &tmp);
//           memcpy(IN_BUFF + icvoff, vec->ct + vec->pt_size, vec->ct_size - vec->pt_size);
        }
         in_buff_size = icvoff + vec->ct_size - vec->pt_size;
        out_buff_size = icvoff + vec->ct_size - vec->pt_size;
     } else {
         out_buff_size = in_buff_size;
     }
  }

  // here we cope with the IV offset. for SRC DDT only.
  if (vec->iv_offset & IV_OFFSET_EN_MASK){
     if (vec->iv_size) {
        uint32_t tmp;
        if (in_buff_size < ((vec->iv_offset & IV_OFFSET_MASK) + vec->iv_size)) {
            in_buff_size = (vec->iv_offset & IV_OFFSET_MASK) + vec->iv_size;
        }
        tmp = (vec->iv_offset & IV_OFFSET_MASK);
        buf_insert(spacc, vec->iv, vec->iv_size, &tmp);
//        memcpy(IN_BUFF + (vec->iv_offset & IV_OFFSET_MASK), vec->iv, vec->iv_size);
     }

     // snow3g/zuc have a 16 byte IV
     // encrypt mode we copy from vec->key and change vec->iv_size so we can also do UIA2/UIA3
     if ( (context_modes[MODE_ENCRYPT][vec->enc_mode] == CRYPTO_MODE_SNOW3G_UEA2) ||
          (context_modes[MODE_ENCRYPT][vec->enc_mode] == CRYPTO_MODE_ZUC_UEA3)) {
        uint32_t tmp;
        if (in_buff_size < ((vec->iv_offset & IV_OFFSET_MASK) + 16)) {
            in_buff_size = (vec->iv_offset & IV_OFFSET_MASK) + 16;
        }
        tmp = (vec->iv_offset & IV_OFFSET_MASK);
        buf_insert(spacc, vec->key + 16, 16, &tmp);
        //memcpy(IN_BUFF + (vec->iv_offset & IV_OFFSET_MASK), vec->key + 16, 16);
        vec->iv_size = 16;
     }


     if ( (context_modes[MODE_HASH][vec->hash_mode] == CRYPTO_MODE_MAC_ZUC_UIA3) ||
          (context_modes[MODE_HASH][vec->hash_mode] == CRYPTO_MODE_MAC_SNOW3G_UIA2)) {
        uint32_t tmp;
        if (in_buff_size < ((vec->iv_offset & IV_OFFSET_MASK) + vec->iv_size + 16)) {
            in_buff_size = (vec->iv_offset & IV_OFFSET_MASK) + vec->iv_size + 16;
        }
        tmp = (vec->iv_offset & IV_OFFSET_MASK) + vec->iv_size;
        buf_insert(spacc, vec->hmac_key + 16, 16, &tmp);
//        memcpy(IN_BUFF + (vec->iv_offset & IV_OFFSET_MASK) + vec->iv_size, vec->hmac_key + 16, 16);
     }
  }

//Only set the auxillary register if this is Kasumi F8/F9 or Snow3G UIA2 mode
  if (context_modes[MODE_HASH][vec->hash_mode] == CRYPTO_MODE_MAC_KASUMI_F9 ||
      context_modes[MODE_ENCRYPT][vec->enc_mode] == CRYPTO_MODE_KASUMI_F8 ||
      context_modes[MODE_ENCRYPT][vec->enc_mode] == CRYPTO_MODE_SNOW3G_UEA2 ||
      context_modes[MODE_HASH][vec->hash_mode] == CRYPTO_MODE_MAC_ZUC_UIA3 ||
      context_modes[MODE_HASH][vec->hash_mode] == CRYPTO_MODE_MAC_SNOW3G_UIA2) {
     if ((err = spacc_set_auxinfo(spacc, handle, vec->auxinfo_dir, vec->auxinfo_bit_align)) != CRYPTO_OK) {
        printk("ERR[%d]: spacc_set_auxinfo : %s\n", __LINE__, spacc_error_msg(err));
        y = 1;
        goto end;
     }
  } else if (context_modes[MODE_HASH][vec->hash_mode] == CRYPTO_MODE_HASH_CRC32) {
     if ((err = spacc_set_auxinfo(spacc, handle, vec->auxinfo_dir,
            vec->auxinfo_bit_align)) != CRYPTO_OK) {
        printk("ERR[%d]: spacc_set_auxinfo : %s\n", __LINE__, spacc_error_msg(err));
        y = 1;
        goto end;
     }
  }
  if (sec_key) {
     spacc_device *pspacc;
     struct platform_device *pdev_spacc2;

     if (spacc->config.is_secure_port) {
        // we're on the secure port so talk to it directly
        pspacc = spacc;
     } else {
        // we're on the normal port so we have to look up the other
        // note that this might not work if they are on diff buses ... BEWARNED
        pdev_spacc2 = get_spacc_platdev_by_epn(vec->epn, 0);
        if (!pdev_spacc2) {
           printk("Could not find secure spacc to register SKP\n");
           y = 1;
           goto end;
        }
        /* get the spacc private struct */
        pspacc = &((struct spacc_priv *)platform_get_drvdata(pdev_spacc2))->spacc;
     }
     // load the SKP data
     if ((err = skp_backend(pspacc, vec, dir, spacc->job[handle].ctx_idx))) {
        printk("Error setting up SKP[%d]\n", __LINE__);
        y = 1;
        goto end;
     }
  }

#ifndef SPACCDIAG_PARTIAL
  if ((vec->icv_mode == ICV_HASH_ENCRYPT) && (dir == DIR_DEC)) {
     //proc_sz = SPACC_AUTO_SIZE;
     proc_sz = vec->pre_aad_size + vec->ct_size + vec->post_aad_size;
  } else {
     proc_sz = vec->pre_aad_size + vec->pt_size + vec->post_aad_size;
  }

  { int x;
    for (x = 0; x < (SPACC_MAX_MSG_MALLOC_SIZE/SPACC_MAX_PARTICLE_SIZE); x++) {
       dma_sync_single_for_device(NULL, src_ddt.virt[x*2+0], src_ddt.virt[x*2+1], DMA_BIDIRECTIONAL);
    }
  }

  init_completion(&comp);
//printk("\n\n\nSTART\n");
  if ((err = spacc_packet_enqueue_ddt (spacc, handle, &src_ddt, &dst_ddt, proc_sz, 0, vec->pre_aad_size, vec->post_aad_size, vec->iv_offset, SPACC_SW_CTRL_PRIO_HI)) != CRYPTO_OK) {
     printk ("ERR[%d]: spacc_packet_enqueue : %s\n", __LINE__,
           spacc_error_msg (err));
     y = 1;
     goto end;
  }
  tries = 0;
  do {
     int t;
     wait_for_completion_timeout(&comp, msecs_to_jiffies(1000));
     if (spacc_packet_dequeue(spacc, handle) == CRYPTO_OK) { break; }
     spacc_pop_packets(spacc, &t);
  } while (spacc_packet_dequeue(spacc, handle) != CRYPTO_OK && ++tries < 10);
  if (tries >= 10) {
     err = CRYPTO_FAILED;
  }
  if (err != CRYPTO_OK) {
     if (tries >= 10) {
        printk ("spacc_packet_dequeue : timeout(%d)\n", tries);
        printk ("timeout occurred: %08lx %08lx\n", pdu_io_read32(spacc->regmap + SPACC_REG_IRQ_STAT), pdu_io_read32(spacc->regmap + SPACC_REG_FIFO_STAT));
     }
     printk ("ERR[%d]: spacc_packet_dequeue : %s\n", __LINE__,  spacc_error_msg (err));
     y = 1;
     goto end;
  }

  { int x;
    for (x = 0; x < (SPACC_MAX_MSG_MALLOC_SIZE/SPACC_MAX_PARTICLE_SIZE); x++) {
       dma_sync_single_for_cpu(NULL, dst_ddt.virt[x*2+0], dst_ddt.virt[x*2+1], DMA_BIDIRECTIONAL);
    }
  }


#ifdef MAKEAVECTOR
{ int x;
  spacc_job * job;
  job = &spacc->job[handle];
  printk("VEC: Final cipher context\n");
  for (x = 0; x < spacc->config.ciph_page_size; x += 4) {
     printk("VEC: %08lx\n", pdu_io_read32(spacc->regmap + SPACC_CTX_CIPH_KEY + (spacc->config.ciph_page_size>>2) * job->ctx_idx + x));
  }
  printk("VEC: END\n");
  printk("VEC: Final hash context\n");
  for (x = 0; x < spacc->config.hash_page_size; x += 4) {
     printk("VEC: %08lx\n", pdu_io_read32(spacc->regmap + SPACC_CTX_HASH_KEY + (spacc->config.hash_page_size>>2) * job->ctx_idx + x));
  }
  printk("VEC: END\n");
}
#endif

  y = 0;
#else
//FIXME no partial yet
#if 0

   // setup our job
   partial_job_init(spacc,
                    context_modes[MODE_ENCRYPT][vec->enc_mode],
                    context_modes[MODE_HASH][vec->hash_mode],
                    dir,
                    handle,
                    vec->pre_aad_size,
                    vec->post_aad_size,
                    0, // aadcopy
                    vec->pt_size,
                    vec->ct_size,
                    vec->icv_size,
                    vec->icv_mode,
                    vec->iv_offset,
                    SPACC_SW_ID_PRIO_HI,
                    &job);

   // get context
   ctx = context_lookup (spacc, job.handle);

   while (job.done == 0) {
      // enqueue job
      if (partial_next(&job) != CRYPTO_OK) {
         printk("ERR[%d]: partial_next() : %s\n", __LINE__, spacc_error_msg(err));
         y = 1;
         goto end;
      }

      // dequeue jobs if we fill the fifo (or too many jobs
      while ((*spacc->reg.fifo_stat & SPACC_FIFO_STAT_CMD_FULL) || (ctx->job_idx >= MAX_JOBS)) {
         // a sleep right here would be nice

         // while status is not empty
         while (!(*spacc->reg.fifo_stat & SPACC_FIFO_STAT_STAT_EMPTY)) {
            err = spacc_packet_dequeue(spacc, handle, 0);
            if (err != CRYPTO_OK) {
               printk("ERR[%d]: spacc_packet_dequeue() : %s\n", __LINE__, spacc_error_msg(err));
               y = 1;
               goto end;
            }
         }
      }
   }


   while (ctx->job_idx) {
      // a sleep right here would be nice

      while (!(*spacc->reg.fifo_stat & SPACC_FIFO_STAT_STAT_EMPTY)) {
         err = spacc_packet_dequeue(spacc, handle, 0);
         if (err != CRYPTO_OK) {
            printk("ERR[%d]: spacc_packet_dequeue() : %s\n", __LINE__, spacc_error_msg(err));
            y = 1;
            goto end;
         }
      }
   }
   y = 0;

#endif

#endif

   // zero out unused bits in the last byte returned of the stream cipher for SNOW3G and Kasumi
   if (vec->auxinfo_bit_align != 8 && (vec->hash_mode == CRYPTO_MODE_MAC_KASUMI_F9 || vec->hash_mode == CRYPTO_MODE_MAC_SNOW3G_UIA2) && vec->post_aad_size == 0) {
      if (dir == DIR_ENC) {
#warning fix padding issue with 3GPP
         //         OUTCT[vec->ct_size - vec->icv_size - 1] &= 0xFF << (8 - vec->auxinfo_bit_align);
      } else {
#warning fix padding issue with 3GPP
//         OUTCT[vec->pt_size - 1] &= 0xFF << (8 - vec->auxinfo_bit_align);
      }
   }

// OUTCT[0] ^= 1;

   if (dir == DIR_ENC) {
      if (vec->icvpos == IP_ICV_OFFSET) {
         if (buf_compare(OUTCT, vec->ct, vec->ct_size - vec->icv_size)) {
#ifdef PCI_INDIRECT
elppci_indirect_trigger();
#endif
printk("Comparing CT failed[%d]\n", __LINE__);
            if(vec->fail_mode == 0) {
               outdiff (OUTCT, vec->ct, vec->ct_size - vec->icv_size);
            }
            printk("fail_mode == %d\n", vec->fail_mode);
            y = 1;
         }

         if (buf_compare_offset(OUTCT, icvoff, vec->icv, vec->icv_size)) {
#ifdef PCI_INDIRECT
elppci_indirect_trigger();
#endif

printk("Comparing ICV failed[%d]\n", __LINE__);
            if(vec->fail_mode == 0) {
               //outdiff (OUTCT + icvoff, vec->icv, vec->icv_size);
            }
            printk("fail_mode == %d\n", vec->fail_mode);
            y = 1;
         }
      } else { // IP_ICV_APPEND
         if (buf_compare(OUTCT, vec->ct, vec->ct_size)) {
#ifdef PCI_INDIRECT
elppci_indirect_trigger();
#endif

printk("Comparing CT failed[%d]\n", __LINE__);
            if(vec->fail_mode == 0) {
               //outdiff (OUTCT, vec->ct, vec->ct_size);
            }
            printk("fail_mode == %d\n", vec->fail_mode);
            y = 1;
         }
      }
   } else {
      if (buf_compare(OUTCT, vec->pt, vec->pt_size)) {
#ifdef PCI_INDIRECT
elppci_indirect_trigger();
#endif

printk("Comparing PT failed[%d]\n", __LINE__);
         if(vec->fail_mode == 0){
//            outdiff (OUTCT, vec->pt, vec->pt_size);
         }
         printk("fail_mode == %d\n", vec->fail_mode);
         y = 1;
      }
   }

end:
   if (handle >= 0) {
#if defined(SECURE_MODE) && defined(DO_PCIM)
     if (spacc->testing.secure_mode) {
        printk("spacc::Testing secure context access\n");

        // test #1: can normal mode write/read the context?
        // disable PCI bit
        elppci_set_secure_off(&tif);

        spacc_dev.ctx[handle].ciph_key[0] = 0xDEADBEEF;
        if (spacc_dev.ctx[handle].ciph_key[0] == 0xDEADBEEF) {
           printk("spacc::We should not be able to write/read the context with secure bit off! [%d]\n", __LINE__);
           y = 1;
        }

        elppci_set_secure_on(&tif);

        spacc_dev.ctx[handle].ciph_key[0] = 0xDEADBEEF;
        if (spacc_dev.ctx[handle].ciph_key[0] != 0xDEADBEEF) {
           printk("spacc::We SHOULD be able to write/read the context with secure bit on! [%d]\n", __LINE__);
           y = 1;
        }

        // test #2: can normal mode release the context?
        // write a known value in
        spacc_dev.ctx[handle].ciph_key[0] = 0xDEADBEEF;

        // normal mode
        elppci_set_secure_off(&tif);
        *(spacc_dev.reg.spacc_secure_rel) = handle;
        if (spacc_dev.ctx[handle].ciph_key[0] == 0xDEADBEEF) {
           printk("spacc::We should not be able release the context with secure bit off! [%d]\n", __LINE__);
           y = 1;
        }

        // back to secure mode
        elppci_set_secure_on(&tif);

        if (spacc->testing.normal_mode) {
           // test #3: release context and then try to read in normal mode
           *(spacc_dev.reg.spacc_secure_rel) = handle;
           elppci_set_secure_off(&tif);
           if (spacc_dev.ctx[handle].ciph_key[0] != 0xDEADBEEF) {
              printk("spacc::We should be able read the context with secure bit off when the context is released! [%d]\n", __LINE__);
              y = 1;
           }

           // back to secure mode
           elppci_set_secure_on(&tif);
       }
     }
#endif

   if (y) {
//     spacc_dump_ctx(spacc, spacc->job[handle].ctx_idx);
   }

     spacc_close (spacc, handle);

#if defined(SECURE_MODE) && defined(DO_PCIM)
     if (spacc->testing.secure_mode) {
        if (spacc->ctx[handle].ciph_key[0] == 0xDEADBEEF) {
           printk("spacc::The context should have been zeroed! [%d]\n", __LINE__);
           y = 1;
        }
     }
#endif
   }

if (y || spaccdiag_verbose)
   printk ("SPACC-%d: %s %4s-%-4s KEY:%4d HMAC:%5d DATA:%5d PRE-AD:%4d POST-AD:%4d ICVM:%d %s %s\n", vec->virt, dir == DIR_ENC ? "ENCRYPT" : "DECRYPT",
          mode_names[MODE_ENCRYPT][vec->enc_mode], mode_names[MODE_HASH][vec->hash_mode], vec->keysize, vec->hmac_keysize, vec->pt_size, vec->pre_aad_size, vec->post_aad_size, vec->icv_mode, y ? "[FAILED]" : "[PASSED]",
#ifdef SPACCDIAG_PARTIAL
   "Partial"
#else
   "Entire"
#endif
          );

  // free buffs
  free_test_vector(vec);

   if (y) {
#ifdef PCI_INDIRECT
   elppci_indirect_trigger();
#endif
   }

  return y;
}

static ssize_t
store_spaccdiag(struct class *class, struct class_attribute *classattr,
                                  const char *buf, size_t count)
{
   char *c, test_vector_name[count+1];
   int err;
   int secure_mode, normal_mode;

   secure_mode = normal_mode = 0;

   strcpy(test_vector_name, buf);

   /*
    * Check for trailing newline and remove it.  We do not
    * support filenames containing newlines.
    */
   c = strchr(test_vector_name, '\n');
   if (c) {
      if (c[1] != '\0')
         return -EINVAL;
      *c = '\0';
   }

  err = parse_test_script (test_vector_name, DIR_ENC, secure_mode, normal_mode);
  if (err != 0) return -EINVAL;

#ifndef MAKEAVECTOR
  err = parse_test_script (test_vector_name, DIR_DEC, secure_mode, normal_mode);
#endif
  return (err == 0) ? count : -EINVAL;
}

static struct class_attribute attrs[] = {
   __ATTR(vector, 0200, NULL, store_spaccdiag),
   __ATTR_NULL
};

static struct class spaccdiag_class = {
   .class_attrs = attrs,
   .owner = THIS_MODULE,
   .name = "spaccdiag",
};

static int __init spaccdiag_mod_init (void)
{
  int retval;

  if (alloc_and_map(OUTCT, &dst_ddt) < 0) {
     return -ENOMEM;
  }

  if (alloc_and_map(IN_BUFF, &src_ddt) < 0) {
      { int x;
         for (x = 0; x < (SPACC_MAX_MSG_MALLOC_SIZE/SPACC_MAX_PARTICLE_SIZE); x++) {
            dma_unmap_single(NULL, dst_ddt.virt[2*x+0], SPACC_MAX_PARTICLE_SIZE, DMA_BIDIRECTIONAL);
            kfree(OUTCT[x]);
            OUTCT[x] = NULL;
         }
      }
      pdu_ddt_free(&dst_ddt);
     return -ENOMEM;
  }
  retval = class_register(&spaccdiag_class);
  if (retval) {
     int x;
     for (x = 0; x < (SPACC_MAX_MSG_MALLOC_SIZE/SPACC_MAX_PARTICLE_SIZE); x++) {
        dma_unmap_single(NULL, src_ddt.virt[2*x+0], SPACC_MAX_PARTICLE_SIZE, DMA_BIDIRECTIONAL);
        kfree(IN_BUFF[x]);
        IN_BUFF[x] = NULL;
     }
     for (x = 0; x < (SPACC_MAX_MSG_MALLOC_SIZE/SPACC_MAX_PARTICLE_SIZE); x++) {
        dma_unmap_single(NULL, dst_ddt.virt[2*x+0], SPACC_MAX_PARTICLE_SIZE, DMA_BIDIRECTIONAL);
        kfree(OUTCT[x]);
        OUTCT[x] = NULL;
     }
     pdu_ddt_free(&src_ddt);
     pdu_ddt_free(&dst_ddt);
  }
  return retval;
}

static void __exit spaccdiag_mod_exit (void)
{
   if (myspacc) {
     int x;
     for (x = 0; x < (SPACC_MAX_MSG_MALLOC_SIZE/SPACC_MAX_PARTICLE_SIZE); x++) {
        dma_unmap_single(NULL, src_ddt.virt[2*x+0], SPACC_MAX_PARTICLE_SIZE, DMA_BIDIRECTIONAL);
        kfree(IN_BUFF[x]);
        IN_BUFF[x] = NULL;
     }
     for (x = 0; x < (SPACC_MAX_MSG_MALLOC_SIZE/SPACC_MAX_PARTICLE_SIZE); x++) {
        dma_unmap_single(NULL, dst_ddt.virt[2*x+0], SPACC_MAX_PARTICLE_SIZE, DMA_BIDIRECTIONAL);
        kfree(OUTCT[x]);
        OUTCT[x] = NULL;
     }
     pdu_ddt_free(&src_ddt);
     pdu_ddt_free(&dst_ddt);
   }
   class_unregister(&spaccdiag_class);
}

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init (spaccdiag_mod_init);
module_exit (spaccdiag_mod_exit);

module_param(spaccdiag_verbose, int, 0);
MODULE_PARM_DESC(spaccdiag_verbose, "Enable verbose vector parsing (disabled by default)");

