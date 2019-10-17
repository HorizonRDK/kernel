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

#define DRVNAME "saspa-des"

#include <linux/module.h>
#include <linux/export.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/semaphore.h>

#include "elpsaspa.h"
#include "elpsaspa_hw.h"
#include "saspa_drv.h"

struct saspa_des_priv {
   u32 __iomem *regs;
   struct semaphore des_busy;
};

#define MAX(x, y) ( ((x)>(y)) ? (x) : (y) )
#define MIN(x, y) ( ((x)<(y)) ? (x) : (y) )

int saspa_des_run(struct device *dev, int encrypt, int mode, int keysize,
                  struct saspa_des_ctx *ctx, void *_msg, size_t msglen)
{
   struct saspa_des_priv *priv = dev_get_drvdata(dev);
   unsigned char *msg = _msg;
   unsigned char ctx_buf[32];
   long ctx_handle = -1, buf_handle = -1;
   size_t bufsize=128, i, blks;
   u32 ctrl, ctrl1;
   int rc;

   ctx_handle = saspa_alloc_ctx(dev->parent, 32);
   if (ctx_handle < 0) {
      rc = ctx_handle;
      goto out;
   }

   // search for a greedy buffer size to work with 
   for (bufsize = saspa_mem_size(dev->parent)&(~7); bufsize >= 8; bufsize -= 32) {
      buf_handle = saspa_alloc_data(dev->parent, bufsize);
      if (buf_handle >= 0) {
         break;
      }
   }
   if (buf_handle < 0) {
      rc = buf_handle;
      goto out;
   }

   rc = elpdes_build_ctx(ctx_buf, ctx);
   if (rc < 0) {
      rc = pdu_error_code(rc);
      goto out;
   }

   saspa_mem_write(dev->parent, ctx_handle, ctx_buf, 32);

   /* Process data in chunks. */
   ctrl = (1ul << DES_CTRL_STR_IV) | (1ul << DES_CTRL_RET_IV) | (1ul << DES_CTRL_RET_KEYS);
   if (keysize == 24) {
      ctrl |= (1ul << DES_CTRL_MODE_3DES);
   }
   if (mode == DES_MODE_CBC) {
      ctrl |= (1ul << DES_CTRL_MODE_CBC);
   }
   if (encrypt) {
      ctrl |= (1ul << DES_CTRL_ENCRYPT);
   }

   for (i = 0; i < msglen; i+=bufsize) {
      if ((i + bufsize) > msglen) {
         blks = (msglen - i) >> 3;
      } else {
         blks = bufsize >> 3;
      }
      saspa_mem_write(dev->parent, buf_handle, msg+i, blks*8);
      ctrl1 = ctrl | (blks << DES_CTRL_N_BLKS);

      if (down_interruptible(&priv->des_busy)) {
         rc = -1;
         goto out;
      }

         saspa_prepare_job(dev->parent, ctx_handle, buf_handle);
            pdu_io_write32(&priv->regs[DES_CTRL], ctrl1);
            rc = saspa_wait(dev->parent);
            if (rc < 0) {
               goto out;
            }
         up(&priv->des_busy);
      saspa_mem_read(dev->parent, msg+i, buf_handle, blks*8);
   }
   saspa_mem_read(dev->parent, ctx_buf, ctx_handle, 32);
   elpdes_read_ctx(ctx_buf, ctx);
   memset(ctx_buf, 0, sizeof ctx_buf);
out:
   saspa_free(dev->parent, ctx_handle, 32);
   saspa_free(dev->parent, buf_handle, bufsize);
   return rc;
}
EXPORT_SYMBOL(saspa_des_run);

static int __devinit saspa_des_probe(struct platform_device *pdev)
{
   struct resource *mem_resource;
   struct device *dev = &pdev->dev;
   struct saspa_des_priv *priv;

   mem_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   if (!mem_resource || resource_size(mem_resource) < SASPA_DES_SIZE)
      return -EINVAL;

   priv = devm_kzalloc(dev, sizeof *priv, GFP_KERNEL);
   if (!priv)
      return -ENOMEM;

   sema_init(&priv->des_busy, 1);
   dev_set_drvdata(dev, priv);

   priv->regs = pdu_linux_map_regs(dev, mem_resource);
   if (IS_ERR(priv->regs))
      return PTR_ERR(priv->regs);

   saspa_enable_irq(pdev->dev.parent, 1ul << SASPA_IRQ_DES);
   return 0;
}

static int __devexit saspa_des_remove(struct platform_device *pdev)
{
   saspa_disable_irq(pdev->dev.parent, 1ul << SASPA_IRQ_DES);

   dev_info(&pdev->dev, "%s\n", __func__);
   return 0;
}

struct platform_driver saspa_des_driver = {
   .probe = saspa_des_probe,
   .remove = __devexit_p(saspa_des_remove),

   .driver = {
      .name = DRVNAME,
      .owner = THIS_MODULE,
   },
};
