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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/platform_device.h>
#include <linux/pagemap.h>
#include <linux/dmapool.h>
#include <crypto/hash.h>
#include <crypto/md5.h>
#include <crypto/sha.h>
#include "elpspaccdrv.h"
#include "cryptoapi.h"

static struct platform_device *spacc_pdev[ELP_CAPI_MAX_DEV+1];


static LIST_HEAD(spacc_alg_list);
static DEFINE_MUTEX(spacc_alg_mutex);

/* Prepare the SG for DMA mapping.  Returns the number of SG entries. */
static int fixup_sg(struct scatterlist *sg, int nbytes)
{
   int sg_nents = 0;

   while (nbytes > 0) {
      if (sg->length) {
         ++sg_nents;
         nbytes -= sg->length;
         sg = sg_next(sg);

         // WARNING: sg->length may be > nbytes
      } else {
         /*
          * The Linux crypto system uses its own SG chaining method which
          * is slightly incompatible with the generic SG chaining.  In
          * particular, dma_map_sg does not support this method.  Turn
          * them into proper chained SGs here (which dma_map_sg does
          * support) as a workaround.
          */
		 //yu WARNING: if nbytes> 0 and sg->lenght == 0 at the beginning, this will cause deadloop
         spacc_sg_chain(sg, 1, sg_chain_ptr(sg));
         sg = sg_chain_ptr(sg);
      }
   }

   return sg_nents;
}


// TODO MERGE THIS WITH BELOW LATER ON
int spacc_sgs_to_ddt(struct device *dev,
                    struct scatterlist *sg1, int len1, int *ents1,
                    struct scatterlist *sg2, int len2, int *ents2,
                    struct scatterlist *sg3, int len3, int *ents3,
                    struct scatterlist *sg4, int len4, int *ents4,
                    pdu_ddt *ddt, int dma_direction)
{
   struct scatterlist *sg_entry, *foo_sg[4];
   int nents[4], onents[4], tents;
   int i, j, k, rc, *vents[4];
   unsigned foo_len[4];

   foo_sg[0] = sg1; foo_len[0] = len1; vents[0] = ents1;
   foo_sg[1] = sg2; foo_len[1] = len2; vents[1] = ents2;
   foo_sg[2] = sg3; foo_len[2] = len3; vents[2] = ents3;
   foo_sg[3] = sg4; foo_len[3] = len4; vents[3] = ents4;

   // map them all
   tents = 0;
   for (j = 0; j < 4; j ++) {
      if (foo_sg[j]) {
         onents[j] = fixup_sg(foo_sg[j], foo_len[j]);
         *(vents[j]) = nents[j] = dma_map_sg(dev, foo_sg[j], onents[j], dma_direction);
         tents += nents[j];
         if (nents[j] <= 0) {
            dev_err(dev, "failed to map scatterlist for DMA: %d\n", nents[j]);
            for (k = 0; k < j; k++) {
               if (foo_sg[k]) {
                  dma_unmap_sg(dev, foo_sg[k], nents[k], dma_direction);
               }
            }
            return -ENOMEM;
         }
      }
   }

   rc = pdu_ddt_init(ddt, tents|0x80000000); // require ATOMIC operations
   if (rc < 0) {
      for (k = 0; k < 4; k++) {
         if (foo_sg[k]) {
            dma_unmap_sg(dev, foo_sg[k], nents[k], dma_direction);
         }
      }
      return pdu_error_code(rc);
   }

   for (j = 0; j < 4; j++) {
      if (foo_sg[j]) {
         for_each_sg(foo_sg[j], sg_entry, nents[j], i) {
            pdu_ddt_add(ddt, sg_dma_address(sg_entry), min(foo_len[j], sg_dma_len(sg_entry)));
            foo_len[j] -= sg_dma_len(sg_entry);
         }
         dma_sync_sg_for_device(dev, foo_sg[j], nents[j], dma_direction);
      }
   }
   return tents;
}


int spacc_sg_to_ddt(struct device *dev, struct scatterlist *sg,
                    int nbytes, pdu_ddt *ddt, int dma_direction)
{
   struct scatterlist *sg_entry;
   int nents, orig_nents;
   int i, rc;

   orig_nents = fixup_sg(sg, nbytes);

   nents = dma_map_sg(dev, sg, orig_nents, dma_direction);
   if (nents <= 0) {
      dev_err(dev, "failed to map scatterlist for DMA: %d\n", nents);
      return -ENOMEM;
   }

   rc = pdu_ddt_init(ddt, nents|0x80000000); // require ATOMIC operations
   if (rc < 0) {
      dma_unmap_sg(dev, sg, orig_nents, dma_direction);
      return pdu_error_code(rc);
   }

   for_each_sg(sg, sg_entry, nents, i) {
      pdu_ddt_add(ddt, sg_dma_address(sg_entry), sg_dma_len(sg_entry));
   }

   dma_sync_sg_for_device(dev, sg, nents, dma_direction);
   return orig_nents;
}

static void __devinit spacc_init_calg(struct crypto_alg *calg, const struct mode_tab *mode)
{
   snprintf(calg->cra_name,        sizeof calg->cra_name,        "%s",       mode->name);
   snprintf(calg->cra_driver_name, sizeof calg->cra_driver_name, "spacc-%s", mode->name);
   calg->cra_blocksize = mode->blocklen;
   calg->cra_ablkcipher.ivsize = mode->ivlen;
}

static void spacc_unregister_algs(struct device *dev)
{
   struct spacc_alg *salg, *tmp;

   mutex_lock(&spacc_alg_mutex);
   list_for_each_entry_safe(salg, tmp, &spacc_alg_list, list) {
      crypto_unregister_alg(salg->calg);
      list_del(&salg->list);
      kfree(salg);
   }
   mutex_unlock(&spacc_alg_mutex);
}

/* Helper macros for initializing the hash/cipher tables. */
#define MODE_TAB_COMMON(_name, _id_name, _blocklen) \
   .name = _name, .id = CRYPTO_MODE_##_id_name, .blocklen = _blocklen

#define MODE_TAB_HASH(_name, _id_name, _hashlen, _blocklen) \
   MODE_TAB_COMMON(_name, _id_name, _blocklen), .hashlen = _hashlen

#define MODE_TAB_CIPH(_name, _id_name, _ivlen, _blocklen) \
   MODE_TAB_COMMON(_name, _id_name, _blocklen), \
   .ivlen = _ivlen

#define MODE_TAB_HASH_XCBC 0x8000
#define HASH_PROCESS_LEN 0x800 //2048Byte

struct mode_tab possible_hashes[] = {
//   { .keylen[0] = 16,                    MODE_TAB_HASH("cmac(aes)", MAC_CMAC, 16,  16),  },
//   { .keylen[0] = 48|MODE_TAB_HASH_XCBC, MODE_TAB_HASH("xcbc(aes)", MAC_XCBC, 16,  16), },
     { MODE_TAB_HASH("md5",              HASH_MD5,        16,  64), .keylen = { 0 } },
     { MODE_TAB_HASH("sha1",             HASH_SHA1,       20,  64), .keylen = { 0 } },
     { MODE_TAB_HASH("sha256",           HASH_SHA256,     32,  64), .keylen = { 0 } },
     { MODE_TAB_HASH("sha512",           HASH_SHA512,     64, 128), .keylen = { 0 } },
     { MODE_TAB_HASH("hmac(md5)",        HMAC_MD5,        16,  64), .keylen = { 16 } },
     { MODE_TAB_HASH("hmac(sha1)",       HMAC_SHA1,       20,  64), .keylen = { 20 } },
//     { MODE_TAB_HASH("hmac(sha224)",     HMAC_SHA224,     28,  64), .keylen = { 28 } },
     { MODE_TAB_HASH("hmac(sha256)",     HMAC_SHA256,     32,  64), .keylen = { 32 } },
//   { MODE_TAB_HASH("hmac(sha384)",     HMAC_SHA384,     48, 128), .keylen = { 24 } },
     { MODE_TAB_HASH("hmac(sha512)",     HMAC_SHA512,     64, 128), .keylen = { 24 } },
//   { MODE_TAB_HASH("hmac(sha512-224)", HMAC_SHA512_224, 28, 128), .keylen = { 24 } },
//   { MODE_TAB_HASH("hmac(sha512-256)", HMAC_SHA512_256, 32, 128), .keylen = { 24 } },
};

static struct mode_tab possible_ciphers[] = {
#if 1
   { MODE_TAB_CIPH("ecb(aes)",         AES_ECB,  16, 16), .keylen = { 16, 24, 32 } },
   { MODE_TAB_CIPH("cbc(aes)",         AES_CBC,  16, 16), .keylen = { 16, 24, 32 } },
   { MODE_TAB_CIPH("ctr(aes)",         AES_CTR,  16, 16), .keylen = { 16, 24, 32 } },
   { MODE_TAB_CIPH("cbc(des3_ede)",    3DES_CBC, 8, 8), .keylen = { 24 } },
   { MODE_TAB_CIPH("ecb(des3_ede)",    3DES_ECB, 8, 8), .keylen = { 24 } },
   { MODE_TAB_CIPH("cbc(des)",         DES_CBC,  8, 8), .keylen = { 8 } },
   { MODE_TAB_CIPH("ecb(des)",         DES_ECB,  8, 8), .keylen = { 8 } },
#endif
};


#define MODE_TAB_AEAD(_name, _ciph, _hash, _hashlen, _ivlen, _blocklen) \
    .name = _name, .aead = { .ciph = _ciph, .hash = _hash }, .hashlen = _hashlen, .ivlen = _ivlen, .blocklen = _blocklen

static struct mode_tab possible_aeads[] = {

#if 0
// cipher only modes for ESP
   { MODE_TAB_AEAD("authenc(digest_null,cbc(des3_ede))",      CRYPTO_MODE_3DES_CBC,         CRYPTO_MODE_NULL, 0, 8, 8),   .keylen = { 24 } },
   { MODE_TAB_AEAD("authenc(digest_null,cbc(aes))",           CRYPTO_MODE_AES_CBC,          CRYPTO_MODE_NULL, 0, 16, 16), .keylen = { 16, 24, 32 } },
   { MODE_TAB_AEAD("authenc(digest_null,rfc3686(ctr(aes)))",  CRYPTO_MODE_AES_CTR_RFC3686,  CRYPTO_MODE_NULL, 0, 16, 1),  .keylen = { 16, 24, 32 } },

// hash only modes for ESP
   { MODE_TAB_AEAD("authenc(hmac(md5),ecb(cipher_null))",     CRYPTO_MODE_NULL,  CRYPTO_MODE_HMAC_MD5,    16, 0, 1), },
   { MODE_TAB_AEAD("authenc(hmac(sha1),ecb(cipher_null))",    CRYPTO_MODE_NULL,  CRYPTO_MODE_HMAC_SHA1,   20, 0, 1), },
   { MODE_TAB_AEAD("authenc(hmac(sha256),ecb(cipher_null))",  CRYPTO_MODE_NULL,  CRYPTO_MODE_HMAC_SHA256, 32, 0, 1), },
   { MODE_TAB_AEAD("authenc(hmac(sha384),ecb(cipher_null))",  CRYPTO_MODE_NULL,  CRYPTO_MODE_HMAC_SHA384, 48, 0, 1), },
   { MODE_TAB_AEAD("authenc(hmac(sha512),ecb(cipher_null))",  CRYPTO_MODE_NULL,  CRYPTO_MODE_HMAC_SHA512, 64, 0, 1), },
   { MODE_TAB_AEAD("authenc(cmac(aes),ecb(cipher_null))",     CRYPTO_MODE_NULL,  CRYPTO_MODE_MAC_CMAC,    16, 0, 1), },
   { MODE_TAB_AEAD("authenc(xcbc(aes),ecb(cipher_null))",     CRYPTO_MODE_NULL,  CRYPTO_MODE_MAC_XCBC,    16, 0, 1), },


#endif
// combined or AEAD modes
#if 0
   { MODE_TAB_AEAD("authenc(hmac(md5),cbc(des3_ede))",    CRYPTO_MODE_3DES_CBC, CRYPTO_MODE_HMAC_MD5,    16, 8, 8), .keylen = { 24 } },
   { MODE_TAB_AEAD("authenc(hmac(sha1),cbc(des3_ede))",   CRYPTO_MODE_3DES_CBC, CRYPTO_MODE_HMAC_SHA1,   20, 8, 8), .keylen = { 24 } },
   { MODE_TAB_AEAD("authenc(hmac(sha256),cbc(des3_ede))", CRYPTO_MODE_3DES_CBC, CRYPTO_MODE_HMAC_SHA256, 32, 8, 8), .keylen = { 24 } },
   { MODE_TAB_AEAD("authenc(hmac(sha384),cbc(des3_ede))", CRYPTO_MODE_3DES_CBC, CRYPTO_MODE_HMAC_SHA384, 48, 8, 8), .keylen = { 24 } },
   { MODE_TAB_AEAD("authenc(hmac(sha512),cbc(des3_ede))", CRYPTO_MODE_3DES_CBC, CRYPTO_MODE_HMAC_SHA512, 64, 8, 8), .keylen = { 24 } },
   { MODE_TAB_AEAD("authenc(cmac(aes),cbc(des3_ede))",    CRYPTO_MODE_3DES_CBC, CRYPTO_MODE_MAC_CMAC,    16, 8, 8), .keylen = { 24 } },
   { MODE_TAB_AEAD("authenc(xcbc(aes),cbc(des3_ede))",    CRYPTO_MODE_3DES_CBC, CRYPTO_MODE_MAC_XCBC,    16, 8, 8), .keylen = { 24 } },
#endif
   { MODE_TAB_AEAD("authenc(hmac(md5),cbc(aes))",    CRYPTO_MODE_AES_CBC, CRYPTO_MODE_HMAC_MD5,    16, 16, 16), .keylen = { 16, 24, 32 } },
   { MODE_TAB_AEAD("authenc(hmac(sha1),cbc(aes))",   CRYPTO_MODE_AES_CBC, CRYPTO_MODE_HMAC_SHA1,   20, 16, 16), .keylen = { 16, 24, 32 } },
   { MODE_TAB_AEAD("authenc(hmac(sha256),cbc(aes))", CRYPTO_MODE_AES_CBC, CRYPTO_MODE_HMAC_SHA256, 32, 16, 16), .keylen = { 16, 24, 32 } },
#if 0
   { MODE_TAB_AEAD("authenc(hmac(sha384),cbc(aes))", CRYPTO_MODE_AES_CBC, CRYPTO_MODE_HMAC_SHA384, 48, 16, 16), .keylen = { 16, 24, 32 } },
   { MODE_TAB_AEAD("authenc(hmac(sha512),cbc(aes))", CRYPTO_MODE_AES_CBC, CRYPTO_MODE_HMAC_SHA512, 64, 16, 16), .keylen = { 16, 24, 32 } },
   { MODE_TAB_AEAD("authenc(cmac(aes),cbc(aes))",    CRYPTO_MODE_AES_CBC, CRYPTO_MODE_MAC_CMAC,    16, 16, 16), .keylen = { 16, 24, 32 } },
   { MODE_TAB_AEAD("authenc(xcbc(aes),cbc(aes))",    CRYPTO_MODE_AES_CBC, CRYPTO_MODE_MAC_XCBC,    16, 16, 16), .keylen = { 16, 24, 32 } },

   { MODE_TAB_AEAD("authenc(hmac(md5),rfc3686(ctr(aes)))",    CRYPTO_MODE_AES_CTR_RFC3686, CRYPTO_MODE_HMAC_MD5,    16, 16, 1), .keylen = { 16, 24, 32 } },
   { MODE_TAB_AEAD("authenc(hmac(sha1),rfc3686(ctr(aes)))",   CRYPTO_MODE_AES_CTR_RFC3686, CRYPTO_MODE_HMAC_SHA1,   20, 16, 1), .keylen = { 16, 24, 32 } },
   { MODE_TAB_AEAD("authenc(hmac(sha256),rfc3686(ctr(aes)))", CRYPTO_MODE_AES_CTR_RFC3686, CRYPTO_MODE_HMAC_SHA256, 32, 16, 1), .keylen = { 16, 24, 32 } },
   { MODE_TAB_AEAD("authenc(hmac(sha384),rfc3686(ctr(aes)))", CRYPTO_MODE_AES_CTR_RFC3686, CRYPTO_MODE_HMAC_SHA384, 48, 16, 1), .keylen = { 16, 24, 32 } },
   { MODE_TAB_AEAD("authenc(hmac(sha512),rfc3686(ctr(aes)))", CRYPTO_MODE_AES_CTR_RFC3686, CRYPTO_MODE_HMAC_SHA512, 64, 16, 1), .keylen = { 16, 24, 32 } },
   { MODE_TAB_AEAD("authenc(cmac(aes),rfc3686(ctr(aes)))",    CRYPTO_MODE_AES_CTR_RFC3686, CRYPTO_MODE_MAC_CMAC,    16, 16, 1), .keylen = { 16, 24, 32 } },
   { MODE_TAB_AEAD("authenc(xcbc(aes),rfc3686(ctr(aes)))",    CRYPTO_MODE_AES_CTR_RFC3686, CRYPTO_MODE_MAC_XCBC,    16, 16, 1), .keylen = { 16, 24, 32 } },

   { MODE_TAB_AEAD("rfc4106(gcm(aes))",                       CRYPTO_MODE_AES_GCM_RFC4106, CRYPTO_MODE_NULL,        16, 16, 1), .keylen = { 16, 24, 32 } },
   { MODE_TAB_AEAD("rfc4543(gcm(aes))",                       CRYPTO_MODE_AES_GCM_RFC4543, CRYPTO_MODE_NULL,        16, 16, 1), .keylen = { 16, 24, 32 } },

//   { MODE_TAB_AEAD("rfc4309(ccm(aes))",                       CRYPTO_MODE_AES_CCM_RFC4309, CRYPTO_MODE_NULL,        16, 16, 1), .keylen = { 16, 24, 32 } },
#endif
};


static int __devinit spacc_register_hash(struct spacc_alg *salg)
{
   int rc;

   salg->calg     = &salg->alg.hash.halg.base;

   if (salg->mode->keylen[0] == 0)
      salg->alg.hash = spacc_hash_template;
   else
      salg->alg.hash = spacc_hmac_template;

   spacc_init_calg(salg->calg, salg->mode);
   salg->alg.hash.halg.digestsize = salg->mode->hashlen;
   salg->alg.hash.halg.statesize = SPACC_HASH_STATE_SIZE;

   rc = crypto_register_ahash(&salg->alg.hash);
   if (rc < 0) {
      return rc;
   }

   mutex_lock(&spacc_alg_mutex);
   list_add(&salg->list, &spacc_alg_list);
   mutex_unlock(&spacc_alg_mutex);

   return 0;
}

static int __devinit probe_hashes(void)
{
   struct spacc_alg *salg;
   int x, rc;
   int registered = 0;
   unsigned i, j;

   for (i = 0; i < ARRAY_SIZE(possible_hashes); i++) {
      possible_hashes[i].valid = 0;
   }

   for (j = 0; j < ELP_CAPI_MAX_DEV && spacc_pdev[j]; ++j) {
      struct spacc_priv *priv = dev_get_drvdata(&spacc_pdev[j]->dev);
      for (i = 0; i < ARRAY_SIZE(possible_hashes); i++) {
         if (possible_hashes[i].valid == 0) {
            if (spacc_isenabled(&priv->spacc, possible_hashes[i].id&0xFF, possible_hashes[i].keylen[0])) {
               salg = kmalloc(sizeof *salg, GFP_KERNEL);
               if (!salg) {
                  return -ENOMEM;
               }
               salg->mode = &possible_hashes[i];

               // Copy all dev's over to the salg
               for (x = 0; x < ELP_CAPI_MAX_DEV && spacc_pdev[x]; x++) {
                  salg->dev[x]  = &spacc_pdev[x]->dev;
               }
               salg->dev[x] = NULL;

               rc = spacc_register_hash(salg);
               if (rc < 0) {
                  kfree(salg);
                  continue;
               }
               dev_info(&spacc_pdev[j]->dev, "registered %s\n", possible_hashes[i].name);
               printk("registered %s\n", possible_hashes[i].name);
               registered++;
               possible_hashes[i].valid = 1;
            } else {
		       pr_err(" %s is not enabled", possible_hashes[i].name);
			}
         }
      }
   }
   return registered;
}

static int __devinit spacc_register_cipher(struct spacc_alg *salg)
{
   int rc;

   salg->alg.cipher = spacc_cipher_template;
   salg->calg = &salg->alg.cipher.base;
   salg->alg.cipher.ivsize = salg->mode->ivlen;

   spacc_init_calg(salg->calg, salg->mode);

   rc = crypto_register_skcipher(&salg->alg.cipher);
   if (rc < 0) {
      pr_err("register cipher alg failed\n");
      return rc;
   }

   mutex_lock(&spacc_alg_mutex);
   list_add(&salg->list, &spacc_alg_list);
   mutex_unlock(&spacc_alg_mutex);

   return 0;
}

static int __devinit probe_ciphers(void)
{
   struct spacc_alg *salg;
   int x, rc;
   int registered = 0;
   unsigned i, j, k;

   for (i = 0; i < ARRAY_SIZE(possible_ciphers); i++) {
      possible_ciphers[i].valid = 0;
   }

   for (j = 0; j < ELP_CAPI_MAX_DEV && spacc_pdev[j]; ++j) {
      struct spacc_priv *priv = dev_get_drvdata(&spacc_pdev[j]->dev);
      for (i = 0; i < ARRAY_SIZE(possible_ciphers); i++) {
         if (possible_ciphers[i].valid == 0) {

		    possible_ciphers[i].keylen_mask = 0;
			for (k = 0; k < ARRAY_SIZE(possible_ciphers[i].keylen); k++) {
               if (spacc_isenabled(&priv->spacc, possible_ciphers[i].id&0xFF, possible_ciphers[i].keylen[k])) { 
			       possible_ciphers[i].keylen_mask |= 1u<<k;
			   }
			}

            if (possible_ciphers[i].keylen_mask) {

               salg = kmalloc(sizeof *salg, GFP_KERNEL);
               if (!salg) {
                  pr_err("Failed to allocate salg\n");
                  return -ENOMEM;
               }
               salg->mode = &possible_ciphers[i];

               // Copy all dev's over to the salg
               for (x = 0; x < ELP_CAPI_MAX_DEV && spacc_pdev[x]; x++) {
                  salg->dev[x]  = &spacc_pdev[x]->dev;
               }

               salg->dev[x] = NULL;
			   salg->alg.cipher.ivsize = salg->mode->ivlen;
			   salg->alg.cipher.chunksize = salg->mode->blocklen;
			   salg->alg.cipher.walksize = salg->mode->blocklen;

               rc = spacc_register_cipher(salg);
               if (rc < 0) {
                  pr_err("failed to register cipher %s\n", possible_ciphers[i].name);
                  kfree(salg);
                  continue;
               }
               dev_info(&spacc_pdev[j]->dev, "registered %s\n", possible_ciphers[i].name);
               registered++;
               possible_ciphers[i].valid = 1;
            }
         }
      }
   }
   return registered;
}

static int __devinit spacc_register_aead(unsigned aead_mode)
{
   int rc, x;
   struct spacc_alg *salg;

   salg = kmalloc(sizeof *salg, GFP_KERNEL);
   if (!salg) {
      return -ENOMEM;
   }

   salg->mode       = &possible_aeads[aead_mode];

// Copy all dev's over to the salg, TODO: support asymmetrical SPAccs...
   for (x = 0; x < ELP_CAPI_MAX_DEV && spacc_pdev[x] != NULL; x++) {
      salg->dev[x]  = &spacc_pdev[x]->dev;
   }
   salg->dev[x] = NULL;

   salg->calg     = &salg->alg.aead.base;
   salg->alg.aead = spacc_aead_template;

   spacc_init_calg(salg->calg, salg->mode);
   salg->alg.aead.ivsize      = salg->mode->ivlen;
   salg->alg.aead.maxauthsize = salg->mode->hashlen;
   salg->alg.aead.chunksize   = salg->mode->blocklen;

   // TODO: remove salg's keylen_mask
   salg->keylen_mask = possible_aeads[aead_mode].keylen_mask;

   if (salg->mode->aead.ciph & SPACC_MANGLE_IV_FLAG) {
      switch (salg->mode->aead.ciph & 0x7F00) {
         case SPACC_MANGLE_IV_RFC3686: //CTR
         case SPACC_MANGLE_IV_RFC4106: //GCM
         case SPACC_MANGLE_IV_RFC4543: //GMAC
         case SPACC_MANGLE_IV_RFC4309: //CCM
            salg->alg.aead.ivsize  = 8;
            break;
      }
   }

   rc = crypto_register_alg(&salg->alg.aead.base);
   if (rc < 0) {
     kfree(salg);
      return rc;
   }

   dev_info(salg->dev[0], "registered %s\n", salg->mode->name);

   mutex_lock(&spacc_alg_mutex);
   list_add(&salg->list, &spacc_alg_list);
   mutex_unlock(&spacc_alg_mutex);

   return 0;
}

static int __devinit probe_aeads(void)
{
   int err;
   unsigned z, x, y;

   for (x = 0; x < ARRAY_SIZE(possible_aeads); x++) {
      possible_aeads[x].keylen_mask = 0;
      possible_aeads[x].valid       = 0;
   }

   // compute cipher key masks (over all devices)
   for (z = 0; spacc_pdev[z] != NULL; z++) {
      struct spacc_priv *priv = dev_get_drvdata(&spacc_pdev[z]->dev);
      for (x = 0; x < ARRAY_SIZE(possible_aeads); x++) {
         for (y = 0; y < ARRAY_SIZE(possible_aeads[x].keylen); y++) {
            if (spacc_isenabled(&priv->spacc, possible_aeads[x].aead.ciph&0xFF, possible_aeads[x].keylen[y])) {
               possible_aeads[x].keylen_mask |= 1u<<y;
            }
         }
      }
   }

   //scan for combined modes
   for (z = 0; spacc_pdev[z] != NULL; z++) {
      struct spacc_priv *priv = dev_get_drvdata(&spacc_pdev[z]->dev);
      for (x = 0; x < ARRAY_SIZE(possible_aeads); x++) {
         if (!possible_aeads[x].valid && possible_aeads[x].keylen_mask) {
            if (spacc_isenabled(&priv->spacc, possible_aeads[x].aead.hash&0xFF, possible_aeads[x].hashlen)) {
                   // once we register it once we don't need to register it again even if different devices have different key mappings...
                   possible_aeads[x].valid = 1;
                   err = spacc_register_aead(x);
                   if (err < 0) {
                      goto error;
                   }
             }
          }
       }
   }
   return 0;

error:
   return err;
}


static int match_name(struct device *dev, void *name)
{
   struct platform_device *pdev = to_platform_device(dev);
   return !strcmp(pdev->name, name);
}

static void enumerate_spaccs(void)
{
   struct device *dev;
   int x;

   for (x = 0; x < ELP_CAPI_MAX_DEV+1; x++) { spacc_pdev[x] = NULL; }

   x = 0;
   dev = NULL;
   do {
     dev = bus_find_device(&platform_bus_type, dev, (void *)"spacc", match_name);
     if (dev != NULL) {
        spacc_pdev[x++] = to_platform_device(dev);
     }
   } while (dev && (x < ELP_CAPI_MAX_DEV));
}

static int __init spacc_crypto_init(void)
{
   int x, rc;

   enumerate_spaccs();
   if (!spacc_pdev[0])
      return -ENODEV;

   rc = spacc_hash_module_init();
   if (rc < 0)
      goto err;

   rc = probe_hashes();
   if (rc < 0)
      goto err;

   rc = spacc_cipher_module_init();
   if (rc < 0)
      goto err;

   rc = probe_ciphers();
   if (rc < 0)
      goto err;

   rc = spacc_aead_module_init();
   if (rc < 0)
      goto err;

   // now that we have ciphers/hashes probed we can probe combined modes...
   rc = probe_aeads();
   if (rc < 0)
      goto err;

   return 0;
err:
   spacc_unregister_algs(NULL);
   spacc_hash_module_exit();
   spacc_aead_module_exit();
   for (x = 0; spacc_pdev[x]; x++) { put_device(&spacc_pdev[x]->dev); }
   return rc;
}
module_init(spacc_crypto_init);

static void __exit spacc_crypto_exit(void)
{
   int x;
   spacc_unregister_algs(NULL);
   spacc_hash_module_exit();
   spacc_aead_module_exit();
   for (x = 0; spacc_pdev[x]; x++) { put_device(&spacc_pdev[x]->dev); }
}
module_exit(spacc_crypto_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
