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

#define DRVNAME "saspa-aes"

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/export.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include "elpsaspa.h"
#include "elpsaspa_hw.h"
#include "saspa_drv.h"

struct saspa_aes_priv {
   u32 __iomem *regs;

   /*
    * aes_mutex coordinates access to the AES core's CTRL and STAT registers.
    * It must be held across any writes to the CTRL register and subsequent
    * reads of the STAT register.
    */
   struct mutex aes_mutex;

   struct saspa_aes_layout ctx;
};

#define MAX(x, y) ( ((x)>(y)) ? (x) : (y) )
#define MIN(x, y) ( ((x)<(y)) ? (x) : (y) )

int saspa_aes_run(struct device *dev, int encrypt, int mode, int keysize,
                  struct saspa_aes_ctx *ctx, void *_msg, size_t msglen, size_t aadlen, uint32_t *stat)
{
   struct saspa_aes_priv *priv = dev_get_drvdata(dev);
   unsigned char *msg = _msg;
   unsigned char ctx_buf[96];
   long ctx_handle = -1, buf_handle = -1;
   size_t bufsize=128, i;
   uint32_t aadjob, ctrl, ctrl1;
   int rc;
   int nooutput=0;

   ctx_handle = saspa_alloc_ctx(dev->parent, priv->ctx.total_size);
   if (ctx_handle < 0) {
      rc = ctx_handle;
      goto out;
   }

   // search for a greedy buffer size to work with 
   for (bufsize = saspa_mem_size(dev->parent); bufsize >= 16; bufsize -= 128) {
      buf_handle = saspa_alloc_data(dev->parent, bufsize);
      if (buf_handle >= 0) {
         break;
      }
   }
   if (buf_handle < 0) {
      rc = buf_handle;
      goto out;
   }

   rc = elpaes_build_ctx(ctx_buf, mode, ctx, &priv->ctx);
   if (rc < 0) {
      rc = pdu_error_code(rc);
      goto out;
   }

   saspa_mem_write(dev->parent, ctx_handle, ctx_buf, priv->ctx.total_size);
   
   if (mode == AES_MODE_CMAC || mode == AES_MODE_XCBC) {
      nooutput = 1;
   }

   /* Process data in chunks. */
   ctrl = (1ul << AES_CTRL_MSG_BEGIN) | (1ul << AES_CTRL_LOAD_IV) | (1ul << AES_CTRL_STORE_IV);
   ctrl |= (mode & ((1ul << AES_CTRL_MODE_BITS)-1)) << AES_CTRL_MODE;
   ctrl |= (keysize & ((1ul << AES_CTRL_KEY_SZ_BITS)-1)) << AES_CTRL_KEY_SZ;
   if (encrypt)
      ctrl |= (1ul << AES_CTRL_ENCRYPT);

   for (i = 0; i+bufsize < msglen; i+=bufsize) {
      saspa_mem_write(dev->parent, buf_handle, msg+i, bufsize);
      ctrl1 = bufsize;

      rc = mutex_lock_interruptible(&priv->aes_mutex);
      if (rc < 0) {
         goto out;
      }

      saspa_prepare_job(dev->parent, ctx_handle, buf_handle);

      pdu_io_write32(&priv->regs[AES_CTRL4], aadlen);
      aadjob = (i >= aadlen) ? 0 : MIN(aadlen - i, bufsize);
      pdu_io_write32(&priv->regs[AES_CTRL3], aadjob);
      pdu_io_write32(&priv->regs[AES_CTRL2], msglen - aadlen);
      pdu_io_write32(&priv->regs[AES_CTRL1], ctrl1);
      pdu_io_write32(&priv->regs[AES_CTRL], ctrl);

      /*
       * We don't need to read the STAT register for this job, so we can drop
       * the mutex early.
       */
      mutex_unlock(&priv->aes_mutex);

      rc = saspa_wait(dev->parent);
      if (rc < 0) {
         goto out;
      }

      if (!nooutput) {
         saspa_mem_read(dev->parent, msg+i, buf_handle, bufsize);
      }

      // we're not on the first block anymore 
      ctrl &= ~(1ul << AES_CTRL_MSG_BEGIN);
   }

   /* Last block */
   ctrl |= 1ul << AES_CTRL_MSG_END;
   ctrl1 = msglen-i;
   saspa_mem_write(dev->parent, buf_handle, msg+i, msglen-i);

   rc = mutex_lock_interruptible(&priv->aes_mutex);
   if (rc < 0) {
      goto out;
   }
   
   saspa_prepare_job(dev->parent, ctx_handle, buf_handle);

   pdu_io_write32(&priv->regs[AES_CTRL4], aadlen);
   aadjob = (i >= aadlen) ? 0 : MIN(aadlen - i, bufsize);
   pdu_io_write32(&priv->regs[AES_CTRL3], aadjob);
   pdu_io_write32(&priv->regs[AES_CTRL2], msglen - aadlen);
   pdu_io_write32(&priv->regs[AES_CTRL1], ctrl1);
   pdu_io_write32(&priv->regs[AES_CTRL], ctrl);

   rc = saspa_wait(dev->parent);
   if (rc < 0) {
      mutex_unlock(&priv->aes_mutex);
      goto out;
   }

   if (stat) {
      *stat = pdu_io_read32(&priv->regs[AES_STAT]);
   }

   mutex_unlock(&priv->aes_mutex);

   if (!nooutput) {
      saspa_mem_read(dev->parent, msg+i, buf_handle, msglen-i);
   }

   saspa_mem_read(dev->parent, ctx_buf, ctx_handle, priv->ctx.total_size);
   elpaes_read_ctx(ctx_buf, mode, ctx, &(priv->ctx));
   memset(ctx_buf, 0, sizeof ctx_buf);
out:
   saspa_free(dev->parent, ctx_handle, priv->ctx.total_size);
   saspa_free(dev->parent, buf_handle, bufsize);
   return rc;
}
EXPORT_SYMBOL(saspa_aes_run);

static int __devinit saspa_aes_probe(struct platform_device *pdev)
{
   struct resource *mem_resource;
   struct device *dev = &pdev->dev;
   struct saspa_aes_priv *priv;

   mem_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   if (!mem_resource || resource_size(mem_resource) < SASPA_AES_SIZE)
      return -EINVAL;

   priv = devm_kzalloc(dev, sizeof *priv, GFP_KERNEL);
   if (!priv)
      return -ENOMEM;

   mutex_init(&priv->aes_mutex);
   dev_set_drvdata(dev, priv);

   priv->regs = pdu_linux_map_regs(dev, mem_resource);
   if (IS_ERR(priv->regs))
      return PTR_ERR(priv->regs);

   /* Parse out the SASPA config to figure out context details. */
   elpaes_get_ctx_layout(dev->platform_data, &priv->ctx);

   saspa_enable_irq(pdev->dev.parent, 1ul << SASPA_IRQ_AES);
   return 0;
}

static int __devexit saspa_aes_remove(struct platform_device *pdev)
{
   saspa_disable_irq(pdev->dev.parent, 1ul << SASPA_IRQ_AES);

   dev_info(&pdev->dev, "%s\n", __func__);
   return 0;
}

struct platform_driver saspa_aes_driver = {
   .probe = saspa_aes_probe,
   .remove = __devexit_p(saspa_aes_remove),

   .driver = {
      .name = DRVNAME,
      .owner = THIS_MODULE,
   },
};
