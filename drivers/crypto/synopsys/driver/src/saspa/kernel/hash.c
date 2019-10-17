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

#define DRVNAME "saspa-hmac"

#include <linux/module.h>
#include <linux/export.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/semaphore.h>

#include "elpsaspa.h"
#include "elpsaspa_hw.h"
#include "saspa_drv.h"

struct saspa_hash_priv {
   u32 __iomem *regs;

   struct semaphore hash_busy;
};

int saspa_hash_run(struct device *dev, int mode, int sslmac, int keysize,
                  const struct saspa_hash_ctx *ctx, 
                  const void *_msg, size_t msglen, void *_out, size_t outlen)
{
   struct saspa_hash_priv *priv = dev_get_drvdata(dev);
   const unsigned char *msg = _msg;
   unsigned char ctx_buf[64], md[64];
   long key_handle = -1, ctx_handle = -1, buf_handle = -1;
   size_t bufsize=128, i;
   u32 ctrl, ctrl_num_bytes, ctrl_tot_bytes, ctrl_secret_offset, ctrl_secret_length, ctrl_sslmac_seq;
   int rc;

   ctx_handle = saspa_alloc_ctx(dev->parent, 64);
   if (ctx_handle < 0) {
      rc = ctx_handle;
      goto out;
   }
   
   if (keysize > 0) {
      key_handle = saspa_alloc_ctx(dev->parent, 64);
      if (key_handle < 0) {
         rc = key_handle;
         goto out;
      }
      saspa_mem_write(dev->parent, key_handle, ctx->key, 64); 
   }   

   // search for a greedy buffer size to work with (always run this last after allocating key/ctx buffers)
   for (bufsize = saspa_mem_size(dev->parent); bufsize >= 128; bufsize -= 128) {
      buf_handle = saspa_alloc_data(dev->parent, bufsize);
      if (buf_handle >= 0) {
         break;
      }
   }
   if (buf_handle < 0) {
      rc = buf_handle;
      goto out;
   }


   /* Process data in chunks. */
   ctrl = (1ul << HMAC_CTRL_MSG_BEGIN) | (outlen << HMAC_CTRL_ICV_LEN) | (mode << HMAC_CTRL_MODE) | ((!!sslmac)<<HMAC_CTRL_SSLMAC) | ((!!keysize)<<HMAC_CTRL_HMAC);
   ctrl_tot_bytes     = msglen;
   ctrl_secret_offset = saspa_mem_addr(dev->parent, key_handle);
   ctrl_secret_length = keysize;
   ctrl_sslmac_seq    = ctx->sslmac_seq;
      
   for (i = 0; i+bufsize < msglen; i+=bufsize) {
      saspa_mem_write(dev->parent, buf_handle, msg+i, bufsize);
      ctrl_num_bytes = bufsize;

      if (down_interruptible(&priv->hash_busy)) {
         rc = -1;
         goto out;
      }
      
      // HASH critical
         saspa_prepare_job(dev->parent, ctx_handle, buf_handle);
         // SASPA critical
            pdu_io_write32(&priv->regs[SASPA_HMAC_TOT_BYTES],     ctrl_tot_bytes);
            pdu_io_write32(&priv->regs[SASPA_HMAC_SECRET_OFFSET], ctrl_secret_offset);
            pdu_io_write32(&priv->regs[SASPA_HMAC_SECRET_BYTES],  ctrl_secret_length);
            pdu_io_write32(&priv->regs[SASPA_HMAC_SSLMAC_SEQNUM], ctrl_sslmac_seq);
            pdu_io_write32(&priv->regs[SASPA_HMAC_NUM_BYTES],     ctrl_num_bytes);
            pdu_io_write32(&priv->regs[SASPA_HMAC_CTRL],          ctrl);
            rc = saspa_wait(dev->parent);
            if (rc < 0) {
               goto out;
            }
        up(&priv->hash_busy);
      // not on first block anymore
      ctrl &= ~(1ul << HMAC_CTRL_MSG_BEGIN);
   }

   /* Last block */
   ctrl |= (1ul << HMAC_CTRL_MSG_END)|(1ul<<HMAC_CTRL_STR_CTX);
   ctrl_num_bytes = msglen-i;
   saspa_mem_write(dev->parent, buf_handle, msg+i, msglen-i);

   if (down_interruptible(&priv->hash_busy)) {
      rc = -1;
      goto out;
   }

   // HASH critical
      saspa_prepare_job(dev->parent, ctx_handle, buf_handle);
      // SASPA critical
         pdu_io_write32(&priv->regs[SASPA_HMAC_TOT_BYTES],     ctrl_tot_bytes);
         pdu_io_write32(&priv->regs[SASPA_HMAC_SECRET_OFFSET], ctrl_secret_offset);
         pdu_io_write32(&priv->regs[SASPA_HMAC_SECRET_BYTES],  ctrl_secret_length);
         pdu_io_write32(&priv->regs[SASPA_HMAC_SSLMAC_SEQNUM], ctrl_sslmac_seq);
         pdu_io_write32(&priv->regs[SASPA_HMAC_NUM_BYTES],     ctrl_num_bytes);
         pdu_io_write32(&priv->regs[SASPA_HMAC_CTRL],          ctrl);
         rc = saspa_wait(dev->parent);
         if (rc < 0) {
            goto out;
         }
      up(&priv->hash_busy);

   // the hash output is stored interleaved for 32-bit hashes so we have to read it properly   
   saspa_mem_read(dev->parent, ctx_buf, ctx_handle, 64);
   
   memset(md, 0, 64);
   switch (mode) {
   case HMAC_MODE_SHA224:
   case HMAC_MODE_SHA256:
   case HMAC_MODE_SHA1:
   case HMAC_MODE_MD5:
      // 32-bit hash
      {
         unsigned char *p = ctx_buf;
         memcpy(md,    p, 4);
         memcpy(md+4,  p+8, 4);
         memcpy(md+8,  p+16, 4);
         memcpy(md+12, p+24, 4);
         memcpy(md+16, p+32, 4);
         memcpy(md+20, p+40, 4);
         memcpy(md+24, p+48, 4);
         memcpy(md+28, p+56, 4);
      }
      break;
   default:
      // 64-bit hash
      memcpy(md, ctx_buf, 64);
   }
   memcpy(_out, md, (outlen > 64) ? 64 : outlen);
out:
   saspa_free(dev->parent, ctx_handle, 64);
   saspa_free(dev->parent, buf_handle, bufsize);
   saspa_free(dev->parent, key_handle, 64);
   return rc;
}
EXPORT_SYMBOL(saspa_hash_run);

static int __devinit saspa_hash_probe(struct platform_device *pdev)
{
   struct resource *mem_resource;
   struct device *dev = &pdev->dev;
   struct saspa_hash_priv *priv;

   mem_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   if (!mem_resource || resource_size(mem_resource) < SASPA_HMAC_SIZE)
      return -EINVAL;

   priv = devm_kzalloc(dev, sizeof *priv, GFP_KERNEL);
   if (!priv)
      return -ENOMEM;

   sema_init(&priv->hash_busy, 1);
   dev_set_drvdata(dev, priv);

   priv->regs = pdu_linux_map_regs(dev, mem_resource);
   if (IS_ERR(priv->regs))
      return PTR_ERR(priv->regs);

   saspa_enable_irq(pdev->dev.parent, 1ul << SASPA_IRQ_SHA);
   return 0;
}

static int __devexit saspa_hash_remove(struct platform_device *pdev)
{
   saspa_disable_irq(pdev->dev.parent, 1ul << SASPA_IRQ_SHA);

   dev_info(&pdev->dev, "%s\n", __func__);
   return 0;
}

struct platform_driver saspa_hash_driver = {
   .probe = saspa_hash_probe,
   .remove = __devexit_p(saspa_hash_remove),

   .driver = {
      .name = DRVNAME,
      .owner = THIS_MODULE,
   },
};
