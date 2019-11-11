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
 * Copyright (c) 2011-2017 Synopsys, Inc. and/or its affiliates.
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
#include <linux/dmapool.h>
#include <crypto/aead.h>
#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <crypto/authenc.h>
#include <linux/rtnetlink.h>
#include "elpspaccdrv.h"
#include "cryptoapi.h"

//#define PRINT_IVS

struct spacc_iv_buf {
   unsigned char iv[SPACC_MAX_IV_SIZE], fulliv[SPACC_MAX_IV_SIZE+16+16]; // 16 for B0 and 16 for IPsec AAD
   struct scatterlist sg[2], fullsg[2];
};

static struct kmem_cache *spacc_iv_pool;

static int spacc_aead_init_dma(struct device *dev, struct aead_request *req, u64 seq, u8 *giv, uint32_t icvlen)
{
   struct crypto_aead *reqtfm      = crypto_aead_reqtfm(req);
   struct spacc_crypto_ctx *tctx   = crypto_aead_ctx(reqtfm);
   struct spacc_crypto_reqctx *ctx = aead_request_ctx(req);

   gfp_t mflags = GFP_ATOMIC;
   struct spacc_iv_buf *iv;
   int rc, i, B0len;
   unsigned ivsize = crypto_aead_ivsize(reqtfm);
   if (!ivsize) ivsize = 1; // always have 1 byte of IV

   if (req->base.flags & CRYPTO_TFM_REQ_MAY_SLEEP)
      mflags = GFP_KERNEL;

   ctx->iv_buf = kmem_cache_alloc(spacc_iv_pool, mflags);
   if (!ctx->iv_buf)
      return -ENOMEM;
   iv = ctx->iv_buf;

   sg_init_table(iv->sg,     ARRAY_SIZE(iv->sg));
   sg_init_table(iv->fullsg, ARRAY_SIZE(iv->fullsg));

   B0len = 0;

   if (tctx->mode != CRYPTO_MODE_NULL) {
      if (giv != NULL) {
         // we're given buffer to generate an IV which they need back...
         unsigned char *p = iv->iv, *pseq;
         __be64 lseq;

         pseq = (unsigned char *)&lseq;

         lseq = cpu_to_be64(seq);

         if (tctx->mode & SPACC_MANGLE_IV_FLAG) {
            switch (tctx->mode & 0x7F00) {
               case SPACC_MANGLE_IV_RFC3686:
               case SPACC_MANGLE_IV_RFC4106:
               case SPACC_MANGLE_IV_RFC4543:
                  {
                     unsigned char *p = iv->fulliv;

                     // we're in RFC3686 mode so the last 4 bytes of the key are the SALT
                     memcpy(p,     tctx->csalt, 4);
                     memcpy(p + 4, &lseq,       8);

                     p[12] = 0;
                     p[13] = 0;
                     p[14] = 0;
                     p[15] = 1;

   #ifdef PRINT_IVS
                     { int x; printk("CTR/GCM CIV == \n"); for (x = 0; x < 16; x++) printk("%02X ", p[x]); printk("\n"); }
   #endif
                     memcpy(giv, &lseq, 8);
                     memcpy(iv->iv, &lseq, 8);
                  }
                  break;
              case SPACC_MANGLE_IV_RFC4309:
                  {
                     unsigned char *p = iv->fulliv;
                     int L, M;
                     uint32_t lm = req->cryptlen;

                     // CCM mode
                     // p[0..15] is the CTR IV
                     // p[16..31] is the CBC-MAC B0 block
                     B0len = 16;

                     // IPsec requires L=4
                     L = 4;
                     M = tctx->auth_size;

                     // CTR block
                     p[0] = L-1;
                     memcpy(p+1, tctx->csalt, 3);
                     memcpy(p+4, &lseq,       8);

                     p[12] = 0;
                     p[13] = 0;
                     p[14] = 0;
                     p[15] = 0;

                     // store B0 block at p[16..31]
                     p[16] = (1<<6)          | // AAD present
                            (((M-2)>>1)<<3)  | // ICV length encoding
                            (L-1);             // length of pt message

                     memcpy(p+1+16, tctx->csalt, 3);
                     memcpy(p+4+16, &lseq,       ivsize);
                     p[16+12+0] = (lm >> 24) & 0xFF;
                     p[16+12+1] = (lm >> 16) & 0xFF;
                     p[16+12+2] = (lm >> 8) & 0xFF;
                     p[16+12+3] = (lm) & 0xFF;

   #ifdef PRINT_IVS
                     { int x; printk("CCM CIV == \n"); for (x = 0; x < 32; x++) printk("%02X ", p[x]); printk("\n"); }
   #endif
                     memcpy(giv, &lseq, 8);
                     memcpy(iv->iv, &lseq, 8);
                  }
                  break;
            }
         } else {
            // assumed here that ivsize == cipher's IV size and not protocol IV size (e.g. 16 for AES, 8 for DES)
            // we have a random salt and we XOR the counter into it so each block starts fresh
            memcpy(p, tctx->csalt, ivsize);
            for (i = 0; i < sizeof lseq; i++) {
               p[i] ^= pseq[i];
            }

            memcpy(giv,        p, ivsize); // copy IV for fallbacks
            memcpy(iv->fulliv, p, ivsize); // copy for IV import
         }
   #ifdef PRINT_IVS
         { int x; printk("giv[..] == "); for (x = 0; x < ivsize; x++) printk("%02X ", giv[x]); printk("\n"); }
   #endif
      } else {
         // we're not given an IV to compute so just take it from req->iv (typically this would be decrypt mode)

         // copy the IV out for AAD
         memcpy(iv->iv,     req->iv, ivsize);

         // now we need to figure out the cipher IV which may or may not be "req->iv" depending on the mode we are
         if (tctx->mode & SPACC_MANGLE_IV_FLAG) {
            switch (tctx->mode & 0x7F00) {
               case SPACC_MANGLE_IV_RFC3686:
               case SPACC_MANGLE_IV_RFC4106:
               case SPACC_MANGLE_IV_RFC4543:
                  {
                     unsigned char *p = iv->fulliv;

                     // we're in RFC3686 mode so the last 4 bytes of the key are the SALT
                     memcpy(p,   tctx->csalt, 4);
                     memcpy(p+4, req->iv, ivsize);

                     p[12] = 0;
                     p[13] = 0;
                     p[14] = 0;
                     p[15] = 1;

   #ifdef PRINT_IVS
                     { int x; printk("CTR/GCM DCIV == \n"); for (x = 0; x < 16; x++) printk("%02X ", p[x]); printk("\n"); }
   #endif
                  }
                  break;
               case SPACC_MANGLE_IV_RFC4309:
                  {
                     unsigned char *p = iv->fulliv;
                     int L, M;
                     uint32_t lm = req->cryptlen;

                     // CCM mode
                     // p[0..15] is the CTR IV
                     // p[16..31] is the CBC-MAC B0 block
                     B0len = 16;

                     // IPsec requires L=4
                     L = 4;
                     M = tctx->auth_size;

                     // CTR block
                     p[0] = L-1;
                     memcpy(p+1, tctx->csalt, 3);
                     memcpy(p+4, req->iv,     ivsize);

                     p[12] = 0;
                     p[13] = 0;
                     p[14] = 0;
                     p[15] = 1;

                     // store B0 block at p[16..31]
                     p[16] = (1<<6)          | // AAD present
                            (((M-2)>>1)<<3) | // ICV length encoding
                            (L-1);            // length of pt message

                     memcpy(p+1+16, tctx->csalt, 3);
                     memcpy(p+4+16, req->iv,     ivsize);

                     // now store length
                     p[16+12+0] = (lm >> 24) & 0xFF;
                     p[16+12+1] = (lm >> 16) & 0xFF;
                     p[16+12+2] = (lm >> 8) & 0xFF;
                     p[16+12+3] = (lm) & 0xFF;

                     // now store the pre-formatted AAD ...
                     p[32] = (req->assoclen >> 8) & 0xFF;
                     p[33] = (req->assoclen) & 0xFF;

                     B0len += 2; // we added the 2 byte header to the AAD


   #ifdef PRINT_IVS
                     { int x; printk("CCM DCIV == \n"); for (x = 0; x < 32; x++) printk("%02X ", p[x]); printk("\n"); }
   #endif
                  }
                  break;

            }
         } else {
            // default is to copy the iv over since the cipher and protocol IV are the same
            memcpy(iv->fulliv, req->iv, ivsize);
         }

   #ifdef PRINT_IVS
        { int x; printk("IV == \n"); for (x = 0; x < ivsize; x++) printk("%02X ", iv->iv[x]); printk("\n"); }
   #endif
      }
   }

   sg_set_buf(iv->sg,     iv->iv,     ivsize);                  // this is part of the AAD
   sg_set_buf(iv->fullsg, iv->fulliv, SPACC_MAX_IV_SIZE+B0len); // this is the actual IV getting fed to the core (via IV IMPORT)

   // GCM and CCM don't include the IV in the AAD ...
   if (tctx->mode == CRYPTO_MODE_AES_GCM_RFC4106 ||
       tctx->mode == CRYPTO_MODE_AES_CCM_RFC4309 ||
       tctx->mode == CRYPTO_MODE_NULL) {
      ctx->iv_nents = 0;
      rc = spacc_sgs_to_ddt(dev,
         iv->fullsg,     SPACC_MAX_IV_SIZE+B0len, &ctx->fulliv_nents,
         req->assoclen ? req->src : NULL,     		 req->assoclen,     &ctx->assoc_nents,
         NULL,           0,                 &ctx->iv_nents,
         req->assoclen ? &req->src[1] : req->src,       req->cryptlen + icvlen, &ctx->src_nents,
         &ctx->src, DMA_BIDIRECTIONAL);
   } else {
      rc = spacc_sgs_to_ddt(dev,
         iv->fullsg,     SPACC_MAX_IV_SIZE+B0len, &ctx->fulliv_nents,
         req->assoclen ? req->src : NULL,     		 req->assoclen,     &ctx->assoc_nents,
         iv->sg,         ivsize,            &ctx->iv_nents,
         req->assoclen ? &req->src[1] : req->src,       req->cryptlen + icvlen, &ctx->src_nents,
         &ctx->src, DMA_BIDIRECTIONAL);
   }

   if (rc < 0) {
      goto err_free_iv;
   }

   if (req->dst != req->src) {
      rc = spacc_sg_to_ddt(dev, req->dst, req->cryptlen + icvlen, &ctx->dst, DMA_FROM_DEVICE);
      if (rc < 0) {
         goto err_free_src;
      }
      ctx->dst_nents = rc;
   }

   return 0;
err_free_src:
   dma_unmap_sg(dev, iv->fullsg, ctx->fulliv_nents, DMA_BIDIRECTIONAL);
   if (ctx->iv_nents) {
      dma_unmap_sg(dev, iv->sg,     ctx->iv_nents,  DMA_BIDIRECTIONAL);
   }
//   dma_unmap_sg(dev, req->assoc, ctx->assoc_nents, DMA_BIDIRECTIONAL);
   dma_unmap_sg(dev, req->src,   ctx->src_nents,   DMA_BIDIRECTIONAL);
   pdu_ddt_free(&ctx->src);
err_free_iv:
   kmem_cache_free(spacc_iv_pool, ctx->iv_buf);
   return rc;
}

static void spacc_aead_cleanup_dma(struct device *dev, struct aead_request *req)
{
   struct spacc_crypto_reqctx *ctx = aead_request_ctx(req);
   struct spacc_iv_buf *iv = ctx->iv_buf;

   if (req->src != req->dst) {
      dma_sync_sg_for_cpu(dev, req->dst, ctx->dst_nents, DMA_FROM_DEVICE);
      dma_unmap_sg(dev, req->dst, ctx->dst_nents, DMA_FROM_DEVICE);
      pdu_ddt_free(&ctx->dst);
   } else {
      dma_sync_sg_for_cpu(dev, req->src, ctx->src_nents, DMA_FROM_DEVICE);
   }

   dma_unmap_sg(dev, iv->fullsg, ctx->fulliv_nents, DMA_BIDIRECTIONAL);
   if (ctx->iv_nents) {
      dma_unmap_sg(dev, iv->sg,     ctx->iv_nents,  DMA_BIDIRECTIONAL);
   }
//   dma_unmap_sg(dev, req->assoc, ctx->assoc_nents,  DMA_BIDIRECTIONAL);
   dma_unmap_sg(dev, req->src,   ctx->src_nents, DMA_BIDIRECTIONAL);
   pdu_ddt_free(&ctx->src);

   kmem_cache_free(spacc_iv_pool, ctx->iv_buf);
}

// XXX: make this shared between cipher/aead at some point
static bool spacc_keylen_ok(const struct spacc_alg *salg, unsigned keylen)
{
   unsigned i, mask = salg->keylen_mask;

   BUG_ON(mask > (1ul << ARRAY_SIZE(salg->mode->keylen))-1);

   for (i = 0; mask; i++, mask >>= 1) {
      if (mask & 1 && salg->mode->keylen[i] == keylen)
         return true;
   }

   return false;
}

static void spacc_aead_cb(void *spacc, void *data)
{
   struct aead_request *req = data;
   struct crypto_aead *tfm      = crypto_aead_reqtfm(req);
   struct spacc_crypto_ctx *tctx   = crypto_aead_ctx(tfm);
   struct spacc_crypto_reqctx *ctx = aead_request_ctx(req);
   struct spacc_priv *priv = dev_get_drvdata(tctx->dev);
   int err;

   spacc_aead_cleanup_dma(tctx->dev, req);

   err = pdu_error_code(priv->spacc.job[ctx->new_handle].job_err);
   spacc_close(&priv->spacc, ctx->new_handle);

   req->base.complete(&req->base, err);
}

static int spacc_aead_setkey(struct crypto_aead *tfm, const u8 *key, unsigned int keylen)
{
   struct spacc_crypto_ctx *ctx  = crypto_aead_ctx(tfm);
   const struct spacc_alg  *salg = spacc_tfm_alg(&tfm->base);
   struct spacc_priv       *priv;
   struct rtattr *rta = (void *)key;
   struct crypto_authenc_key_param *param;
   unsigned int x, authkeylen, enckeylen;
   const unsigned char *authkey, *enckey;
   unsigned char xcbc[64];

   int err = -EINVAL;
   int singlekey = 0;

   // are keylens valid?
   ctx->ctx_valid = false;

   switch (ctx->mode & 0xFF) {
      case CRYPTO_MODE_AES_GCM:
      case CRYPTO_MODE_AES_CCM: // because if there are more than one way of doing things please do both!
         authkey    = key; // set it to "something"
         authkeylen = 0;
         enckey     = key;
         enckeylen  = keylen;
         singlekey  = 1;
         goto skipover;
   }

   if (!RTA_OK(rta, keylen))
      goto badkey;

   if (rta->rta_type != CRYPTO_AUTHENC_KEYA_PARAM)
      goto badkey;

   if (RTA_PAYLOAD(rta) < sizeof(*param))
      goto badkey;

   param = RTA_DATA(rta);
   enckeylen = be32_to_cpu(param->enckeylen);

   key += RTA_ALIGN(rta->rta_len);
   keylen -= RTA_ALIGN(rta->rta_len);

   if (keylen < enckeylen)
      goto badkey;

   authkeylen = keylen - enckeylen;

   // at this point we have authkeylen and enckeylen
   // enckey is at &key[authkeylen] and
   // authkey is at &key[0]
   authkey = &key[0];
   enckey  = &key[authkeylen];
skipover:

   // TODO: detect RFC3686/4106 and trim from enckeylen (and copy salt...)
   if (ctx->mode & SPACC_MANGLE_IV_FLAG) {
      switch (ctx->mode & 0x7F00) {
         case SPACC_MANGLE_IV_RFC3686:
         case SPACC_MANGLE_IV_RFC4106:
         case SPACC_MANGLE_IV_RFC4543:
            memcpy(ctx->csalt, enckey+enckeylen-4, 4);
            enckeylen -= 4;
            break;
         case SPACC_MANGLE_IV_RFC4309:
            memcpy(ctx->csalt, enckey+enckeylen-3, 3);
            enckeylen -= 3;
            break;
      }
   }

   if (!singlekey) {
      if (authkeylen > salg->mode->hashlen) {
         dev_warn(ctx->dev, "Auth key size of %u is not valid\n", authkeylen);
         return 0;
      }
   }

   if (!spacc_keylen_ok(salg, enckeylen)) {
      dev_warn(ctx->dev, "Enc key size of %u is not valid\n", enckeylen);
      return 0;
   }

   // if we're already open close the handle since the size may have changed
   if (ctx->handle != -1) {
      priv = dev_get_drvdata(ctx->dev);
      spacc_close(&priv->spacc, ctx->handle);
      put_device(ctx->dev);
      ctx->handle = -1;
   }

   // Open a handle ...
   // search all devices for an open handle
   priv = NULL;
   for (x = 0; x < ELP_CAPI_MAX_DEV && salg->dev[x]; x++) {
      priv = dev_get_drvdata(salg->dev[x]);

      // increase reference
      ctx->dev = get_device(salg->dev[x]);

      // is it a valid mode ...
      if (spacc_isenabled(&priv->spacc, salg->mode->aead.ciph & 0xFF, enckeylen) && spacc_isenabled(&priv->spacc, salg->mode->aead.hash & 0xFF, authkeylen)) {
         // try to open spacc handle
         ctx->handle = spacc_open(&priv->spacc, salg->mode->aead.ciph & 0xFF, salg->mode->aead.hash & 0xFF, -1, 0, spacc_aead_cb, tfm);
      }

      if (ctx->handle < 0) { 
         put_device(salg->dev[x]); 
      } else { 
         break;
      }
   }

   if (ctx->handle < 0) {
      dev_dbg(salg->dev[0], "failed to open SPAcc context\n");
      return -1;
   }

   // setup XCBC key
   if (salg->mode->aead.hash == CRYPTO_MODE_MAC_XCBC) {
      err = spacc_compute_xcbc_key(&priv->spacc, ctx->handle, authkey, authkeylen, xcbc);
      if (err < 0) {
         dev_warn(ctx->dev, "failed to compute XCBC key: %d\n", err);
         return 0;
      }
      authkey    = xcbc;
      authkeylen = 48;
   }

   err = spacc_write_context(&priv->spacc, ctx->handle, SPACC_CRYPTO_OPERATION, enckey, enckeylen, NULL, 0);
   if (err) {
      dev_warn(ctx->dev, "Could not write ciphering context: %d\n", err);
      return 0;
   }

   if (!singlekey) {
      err = spacc_write_context(&priv->spacc, ctx->handle, SPACC_HASH_OPERATION, authkey, authkeylen, NULL, 0);
      if (err) {
         dev_warn(ctx->dev, "Could not write hashing context: %d\n", err);
         return 0;
      }
   }

   // set expand key
   spacc_set_key_exp(&priv->spacc, ctx->handle);
   ctx->ctx_valid = true;

   memset(xcbc, 0, sizeof xcbc);

   return 0;
badkey:
   return err;
}

static int spacc_aead_setauthsize(struct crypto_aead *tfm, unsigned int authsize)
{
   struct spacc_crypto_ctx *ctx = crypto_aead_ctx(tfm);
   ctx->auth_size = authsize;
   return 0;
}

static int spacc_aead_process(struct aead_request *req, u64 seq, u8 *giv, int encrypt)
{
   struct crypto_aead *reqtfm      = crypto_aead_reqtfm(req);
   struct spacc_crypto_ctx *tctx   = crypto_aead_ctx(reqtfm);
   struct spacc_crypto_reqctx *ctx = aead_request_ctx(req);
   struct spacc_priv *priv = dev_get_drvdata(tctx->dev);
   int ivsize                      = crypto_aead_ivsize(reqtfm);
   int rc;
   int icvremove;
   int ivaadsize;
   int ptaadsize;
   int B0len;
   uint32_t dstoff;

   if (tctx->handle < 0 || !tctx->ctx_valid || (req->cryptlen + req->assoclen) > priv->max_msg_len) {
      return -EINVAL;
   }

   icvremove = (encrypt) ? 0 : tctx->auth_size;

   rc = spacc_aead_init_dma(tctx->dev, req, seq, giv, (encrypt) ? tctx->auth_size : 0);
   if (rc < 0) {
      return -EINVAL;
   }

   /*
    * Note: This won't work if IV_IMPORT has been disabled
    */

   ctx->new_handle = spacc_clone_handle(&priv->spacc, tctx->handle, req);
   if (ctx->new_handle < 0) {
      spacc_aead_cleanup_dma(tctx->dev, req);
      return -EINVAL;
   }

   // CCM and GCM don't include the IV in the AAD ...
   if (tctx->mode == CRYPTO_MODE_AES_GCM_RFC4106 ||
       tctx->mode == CRYPTO_MODE_AES_CCM_RFC4309 ||
       tctx->mode == CRYPTO_MODE_NULL) {
      ivaadsize = 0;
   } else {
      ivaadsize = ivsize;
   }

   // CCM requires an extra block of AAD ...
   if (tctx->mode == CRYPTO_MODE_AES_CCM_RFC4309) {
      B0len = 16;
   } else {
      B0len = 0;
   }

   // GMAC mode uses AAD for the entire message
   // So does NULL cipher
   if (tctx->mode == CRYPTO_MODE_AES_GCM_RFC4543 ||
       tctx->mode == CRYPTO_MODE_NULL) {
      ptaadsize = req->cryptlen - icvremove;
   } else {
      ptaadsize = 0;
   }

   if (req->dst == req->src) {
      dstoff = ((uint32_t)(SPACC_MAX_IV_SIZE + B0len + req->assoclen + ivaadsize));
   } else {
      dstoff = req->assoclen;//dst: assoc || output
   }

   // compute the ICV offset which is different for both encrypt/decrypt
   if (encrypt) {
      rc = spacc_set_operation(&priv->spacc, ctx->new_handle, encrypt ? OP_ENCRYPT : OP_DECRYPT, ICV_ENCRYPT_HASH, IP_ICV_OFFSET, dstoff + req->cryptlen, tctx->auth_size, 0);
   } else {
      // in decrypt mode we have to account for the offset based on the AAD and what not ...
      rc = spacc_set_operation(&priv->spacc, ctx->new_handle, encrypt ? OP_ENCRYPT : OP_DECRYPT, ICV_ENCRYPT_HASH, IP_ICV_OFFSET, req->cryptlen - icvremove + SPACC_MAX_IV_SIZE + B0len + req->assoclen + ivaadsize, tctx->auth_size, 0);
   }

   rc = spacc_packet_enqueue_ddt(&priv->spacc, ctx->new_handle,
       &ctx->src, (req->dst == req->src) ? &ctx->src : &ctx->dst,      // SRC and DST DDTs (we re-use the src if they overlap)
       B0len + req->cryptlen + req->assoclen + ivaadsize - icvremove,  // PROC len
       (dstoff << SPACC_OFFSET_DST_O) | SPACC_MAX_IV_SIZE,             // OFFSET:  SRC is offset by IV import block, DST is potentially offset by IV + AAD
       B0len + req->assoclen + ivaadsize + ptaadsize,                  // PRE_AAD length
       0,                                                              // no POST AAD
       0x80000000,                                                     // IV import from offset 0
       0);                                                             // default PRIO

   if (rc < 0) {
      spacc_aead_cleanup_dma(tctx->dev, req);
      spacc_close(&priv->spacc, ctx->new_handle);

      if (rc != CRYPTO_FIFO_FULL) {
         dev_err(tctx->dev, "failed to enqueue job: %s\n", spacc_error_msg(rc));
      } else if (!(req->base.flags & CRYPTO_TFM_REQ_MAY_BACKLOG)) {
         return -EBUSY;
      }

      return -EINVAL;
   }

   // at this point the job is in flight to the engine ... remove first use so subsequent calls don't expand the key again...
   // ideally we would pump a dummy job through the engine to pre-expand the key so that by time setkey was done we wouldn't
   // have to do this
   priv->spacc.job[tctx->handle].first_use  = 0;
   priv->spacc.job[tctx->handle].ctrl      &= ~(1UL << priv->spacc.config.ctrl_map[SPACC_CTRL_KEY_EXP]);

   return -EINPROGRESS;
}

static int spacc_aead_encrypt(struct aead_request *req)
{
   return spacc_aead_process(req, 0ULL, NULL, 1);
}

static int spacc_aead_decrypt(struct aead_request *req)
{
   return spacc_aead_process(req, 0ULL, NULL, 0);
}

/*
static int spacc_aead_givencrypt(struct aead_givcrypt_request *req)
{
   return spacc_aead_process(&req->areq, req->seq, req->giv, 1);
}
*/

static int spacc_aead_cra_init(struct crypto_tfm *tfm)
{
   struct spacc_crypto_ctx *ctx  = crypto_tfm_ctx(tfm);
   const struct spacc_alg  *salg = spacc_tfm_alg(tfm);

   ctx->fb.aead = __crypto_aead_cast(tfm);

   get_random_bytes(ctx->csalt, sizeof(ctx->csalt));
   ctx->fb.aead->reqsize = sizeof(struct spacc_crypto_reqctx);
   ctx->handle           = -1;
   ctx->mode             = salg->mode->aead.ciph;
   ctx->dev              = get_device(salg->dev[0]);

   return 0;
}

static void spacc_aead_cra_exit(struct crypto_tfm *tfm)
{
   struct spacc_crypto_ctx *ctx = crypto_tfm_ctx(tfm);
   struct spacc_priv *priv = dev_get_drvdata(ctx->dev);

   // close spacc handle
   if (ctx->handle >= 0) {
      spacc_close(&priv->spacc, ctx->handle);
      ctx->handle = -1;
   }

   put_device(ctx->dev);
}


const struct aead_alg spacc_aead_template __devinitconst = {
	.setkey      = spacc_aead_setkey,
	.setauthsize = spacc_aead_setauthsize,
	.encrypt     = spacc_aead_encrypt,
	.decrypt     = spacc_aead_decrypt,
//	.givencrypt  = spacc_aead_givencrypt,

	.base.cra_init = spacc_aead_cra_init,
	.base.cra_exit     = spacc_aead_cra_exit,
	.base.cra_priority = 1301,
	.base.cra_module   = THIS_MODULE,
	.base.cra_ctxsize  = sizeof (struct spacc_crypto_ctx),
//	.base.cra_type     = &crypto_aead_type,//will set cra_type in register_aead
	.base.cra_flags    = CRYPTO_ALG_TYPE_AEAD
				 | CRYPTO_ALG_ASYNC
				 | CRYPTO_ALG_NEED_FALLBACK
				 | CRYPTO_ALG_KERN_DRIVER_ONLY
};


int __init spacc_aead_module_init(void)
{
   size_t alloc_size = max(roundup_pow_of_two(sizeof (struct spacc_iv_buf)),
                           (unsigned long)dma_get_cache_alignment());

   spacc_iv_pool = kmem_cache_create("spacc-aead-iv", alloc_size, alloc_size, 0, NULL);

   if (!spacc_iv_pool)
      return -ENOMEM;

   return 0;
}

void spacc_aead_module_exit(void)
{
   if (spacc_iv_pool)
      kmem_cache_destroy(spacc_iv_pool);
}
