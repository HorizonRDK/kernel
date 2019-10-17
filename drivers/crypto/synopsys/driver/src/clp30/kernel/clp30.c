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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ratelimit.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/param.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include "clp30.h"

#ifdef CLP36_ENABLED
   #define DEVNAME "clp36: "
#else
   #define DEVNAME "clp30: "
#endif

static clp30_device clp30;
static struct platform_device *clp30_dev;

clp30_device *clp30_get_device(void)
{
   return &clp30;
}

struct platform_device *clp30_get_platform_device(void)
{
   return clp30_dev;
}


void clp30_pop_jobs(unsigned long _clp30)
{
   clp30_pop_packet(&clp30);
}

static DECLARE_TASKLET(clp30_job_tasklet, clp30_pop_jobs, 0UL);

#ifndef CLP36_ENABLED
/* a function to run callbacks in the IRQ handler */
static irqreturn_t clp30_irq_handler(int irq, void *dev)
{
   uint32_t irq_stat;

   irq_stat = pdu_io_read32(clp30.regmap + CLP30_REG_IRQ_STAT);
   if (irq_stat) {
      pdu_io_write32(clp30.regmap + CLP30_REG_IRQ_STAT, irq_stat);
      tasklet_schedule(&clp30_job_tasklet);
      return IRQ_HANDLED;
   }

   return IRQ_NONE;
}
#else
/* a function to run callbacks in the IRQ handler */
/* TODO: Handle FIFO scaling to throttle back IRQ rate */
static irqreturn_t clp30_irq_handler(int irq, void *dev)
{
   uint32_t irq_stat;

   irq_stat = pdu_io_read32(clp30.regmap + CLP30_REG_IRQ_STAT);
   if (irq_stat) {
      pdu_io_write32(clp30.regmap + CLP30_REG_IRQ_STAT, irq_stat);
      tasklet_schedule(&clp30_job_tasklet);
      return IRQ_HANDLED;
   }

   return IRQ_NONE;
}
#endif

#define HW_ENTRY(x) { x, #x }
static const struct { unsigned addr; char *name; } reg_names[] = {
   HW_ENTRY(CLP30_REG_IRQ_EN),
   HW_ENTRY(CLP30_REG_IRQ_STAT),
   HW_ENTRY(CLP30_REG_DMA_BURST),
   HW_ENTRY(CLP30_REG_IV_RND),
   HW_ENTRY(CLP30_REG_OUT_SRC_PTR),
   HW_ENTRY(CLP30_REG_OUT_DST_PTR),
   HW_ENTRY(CLP30_REG_OUT_OFFSET),
   HW_ENTRY(CLP30_REG_OUT_ID),
   HW_ENTRY(CLP30_REG_OUT_SAI),
   HW_ENTRY(CLP30_REG_OUT_POP),
   HW_ENTRY(CLP30_REG_OUT_STAT),
   HW_ENTRY(CLP30_REG_IN_SRC_PTR),
   HW_ENTRY(CLP30_REG_IN_DST_PTR),
   HW_ENTRY(CLP30_REG_IN_OFFSET),
   HW_ENTRY(CLP30_REG_IN_ID),
   HW_ENTRY(CLP30_REG_IN_SAI),
   HW_ENTRY(CLP30_REG_IN_POP),
   HW_ENTRY(CLP30_REG_IN_STAT),
#ifdef CLP36_ENABLED
      HW_ENTRY(CLP36_REG_IRQ_CTRL),
      HW_ENTRY(CLP36_REG_OUT_FIFO_STAT),
      HW_ENTRY(CLP36_REG_IN_FIFO_STAT),
#endif
   { 0, NULL },
};
#undef HW_ENTRY

static ssize_t show_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   char *name, out[128];
   unsigned x, reg_addr;

   buf[0] = out[0] = 0;
   for (reg_addr = 0; reg_addr < 0x80; reg_addr += 4) {
      name = NULL;
      for (x = 0; reg_names[x].name != NULL; x++) {
         if (reg_names[x].addr == reg_addr) {
            name = reg_names[x].name;
            break;
         }
      }
      if (name == NULL) { continue; }
      sprintf(out, "%-25s = %08lx\n", name, (unsigned long)pdu_io_read32(clp30.regmap + reg_addr));
      strcat(buf, out);
   }
   return strlen(buf);
}

static DEVICE_ATTR(reg,                0400, show_reg, NULL);
static const struct attribute_group clp30_attr_group = {
   .attrs = (struct attribute *[]) {
      &dev_attr_reg.attr,
      NULL
   }
};


static int clp30_probe(struct platform_device *pdev)
{
   void *baseaddr;
   struct resource *res, *irq;
   int err;

   /* Initialize DDT DMA pools based on this device's resources */
   if (pdu_mem_init(&pdev->dev)) {
      dev_err(&pdev->dev, "Could not initialize DMA pools\n");
      return -ENOMEM;
   }

   irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
   res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   if (!res || !irq) {
      return -EINVAL;
   }
   printk("%s clp30_probe: Device at %08lx(%08lx) of size %lu bytes\n", DEVNAME, (unsigned long)res->start, (unsigned long)res->end, (unsigned long)resource_size(res));

   baseaddr = pdu_linux_map_regs(&pdev->dev, res);
   if (IS_ERR(baseaddr)) {
      dev_err(&pdev->dev, "unable to map iomem\n");
      return PTR_ERR(baseaddr);
   }

   /* init the library */
   err = clp30_init(&clp30, baseaddr, 32);
   if (err) {
      return -1;
   }

   err = sysfs_create_group(&pdev->dev.kobj, &clp30_attr_group);
   if (err) {
      clp30_deinit(&clp30);
      return -1;
   }

   // after we call re_init we need to allocate a pool
   printk("%s requires %zu bytes for SA\n", DEVNAME, clp30.sa_ptr_mem_req);
   clp30.sa_ptr_virt = pdu_dma_alloc(clp30.sa_ptr_mem_req, &clp30.sa_ptr_phys);
   if (!clp30.sa_ptr_virt) {
      printk("Cannot allocate SA pool\n");
      sysfs_remove_group(&pdev->dev.kobj, &clp30_attr_group);
      clp30_deinit(&clp30);
      return -1;
   }

   if (devm_request_irq(&pdev->dev, irq->start, clp30_irq_handler, IRQF_SHARED, dev_name(&pdev->dev), &pdev->dev)) {
      dev_err(&pdev->dev, "failed to request IRQ\n");
      pdu_dma_free(clp30.sa_ptr_mem_req, clp30.sa_ptr_virt, clp30.sa_ptr_phys);
      sysfs_remove_group(&pdev->dev.kobj, &clp30_attr_group);
      clp30_deinit(&clp30);
      return -EBUSY;
   }

#ifdef CLP36_ENABLED
   /* IRQ whenever a job appears on the STAT FIFO */
   /* TODO: add scaling to FIFO depth */
   pdu_io_write32(baseaddr + CLP36_REG_IRQ_CTRL, CLP36_IRQ_CTRL_CMD_CNT(0) | CLP36_IRQ_CTRL_STAT_CNT(1));

   // enable interrupts
   pdu_io_write32(baseaddr+CLP30_REG_IRQ_EN,CLP30_IRQ_EN_OUT_STAT_EN|CLP30_IRQ_EN_IN_STAT_EN|CLP30_IRQ_EN_GLBL_EN);
#else
   // enable interrupts
   pdu_io_write32(baseaddr+CLP30_REG_IRQ_EN,CLP30_IRQ_EN_OUT_CMD_EN|CLP30_IRQ_EN_OUT_STAT_EN|CLP30_IRQ_EN_IN_CMD_EN|CLP30_IRQ_EN_IN_STAT_EN|CLP30_IRQ_EN_GLBL_EN);
#endif

   clp30_dev = pdev;
   return 0;
}

static int clp30_remove(struct platform_device *pdev)
{
   printk("%s clp30_mod_exit::Freeing resources\n", DEVNAME);

   // fini the SDK then free the SA pool
   sysfs_remove_group(&pdev->dev.kobj, &clp30_attr_group);
   clp30_deinit(&clp30);
   pdu_dma_free(clp30.sa_ptr_mem_req, clp30.sa_ptr_virt, clp30.sa_ptr_phys);

   pdu_mem_deinit(&pdev->dev);
   return 0;
}

static struct platform_driver clp30_driver = {
   .probe  = clp30_probe,
   .remove = clp30_remove,
   .driver = {
      .name  = "clp30",
      .owner = THIS_MODULE
   }
};

static int __init clp30_mod_init (void)
{
   memset(&clp30, 0, sizeof clp30);
   return platform_driver_register(&clp30_driver);
}

static void __exit clp30_mod_exit (void)
{
   platform_driver_unregister(&clp30_driver);
}

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init (clp30_mod_init);
module_exit (clp30_mod_exit);

// kernel
EXPORT_SYMBOL (clp30_get_device);
EXPORT_SYMBOL (clp30_get_platform_device);

// lib functions
EXPORT_SYMBOL (clp30_init);
EXPORT_SYMBOL (clp30_deinit);
EXPORT_SYMBOL (clp30_open);
EXPORT_SYMBOL (clp30_build_sa);
EXPORT_SYMBOL (clp30_go);
EXPORT_SYMBOL (clp30_close);
EXPORT_SYMBOL (clp30_done);
EXPORT_SYMBOL (clp30_pop_packet);


