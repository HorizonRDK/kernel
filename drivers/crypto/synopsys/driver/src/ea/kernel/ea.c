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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ratelimit.h>

#include <linux/io.h>

#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/param.h>
#include <linux/platform_device.h>
#include "elpea.h"
#include "elpeahw.h"
#include "elpspaccdrv.h"

static int oldtimer=50000,timer=50000,
           no_latency=0;

static ea_device ea;
static struct platform_device *ea_dev;

struct platform_device *ea_get_platform_device(void)
{
   return ea_dev;
}

ea_device *ea_get_device(void)
{
   return &ea;
}

void ea_pop_jobs(unsigned long _ea)
{
   ea_pop_packet(&ea);
}

static DECLARE_TASKLET(ea_job_tasklet, ea_pop_jobs, 0UL);

// interrupt handler for high volume v2.5 cores
static irqreturn_t ea_irq_handler(int irq, void *dev)
{
   uint32_t irq_stat;

   irq_stat = pdu_io_read32(ea.regmap + EA_IRQ_STAT);

   if (irq_stat) {
#ifdef EA_PROF
      ++ea.stats.dc[EA_FIFO_STAT_STAT_CNT(pdu_io_read32(ea.regmap + EA_FIFO_STAT))];
#endif
      pdu_io_write32(ea.regmap + EA_IRQ_STAT, irq_stat);
//      printk("IRQ_STAT==%08lx, FIFO_STAT==%08lx\n", irq_stat, pdu_io_read32(ea.regmap + EA_FIFO_STAT));

      // allow the timer to be changed at runtime
      if (oldtimer != timer) {
         printk("ea::timer now set to %d\n", timer);
         EA_IRQ_ENABLE_STAT_WD(&ea, timer);
         oldtimer = timer;
      }
   }

   if (irq_stat & (EA_IRQ_STAT_STAT|EA_IRQ_STAT_STAT_WD)) {
      tasklet_schedule(&ea_job_tasklet);
      return IRQ_HANDLED;
   }

   return IRQ_NONE;
}

// interrupt handler for low latency and v2.4 cores
static irqreturn_t ea_irq_handler_v24(int irq, void *dev)
{
   uint32_t irq_stat;

   irq_stat = pdu_io_read32(ea.regmap + EA_IRQ_STAT);


   if (irq_stat & (EA_IRQ_STAT_STAT|EA_IRQ_STAT_STAT_V24)) {
#ifdef EA_PROF
   ++(ea.stats.dc[ea.config.cur_depth]);
#endif

      if (ea.config.cur_depth == 1) {
         // we now have the ability to stack up more than 1 job therefore we need the CMD IRQ on
         EA_IRQ_ENABLE_CMD(&ea, 0);
      }

      // ideally you want to raise a STAT IRQ before the STAT FIFO completely fills up thus preventing a CMD IRQ which forces the STAT CNT to reset
      if (ea.config.cur_depth < ea.config.max_stat_limit) {
         ea.config.cur_depth <<= 1;
         if (ea.config.cur_depth > ea.config.max_stat_limit) {
            ea.config.cur_depth = ea.config.max_stat_limit;
         }
         EA_IRQ_SET_STAT(&ea, ea.config.cur_depth);
      }
   }

   if (irq_stat & EA_IRQ_STAT_CMD) {
#ifdef EA_PROF
   ++(ea.stats.cmd);
#endif

      ea.config.cur_depth = 1;
      EA_IRQ_SET_STAT(&ea, ea.config.cur_depth);
      EA_IRQ_DISABLE_CMD(&ea);
   }

   if (irq_stat & (EA_IRQ_STAT_STAT|EA_IRQ_STAT_CMD|EA_IRQ_STAT_STAT_V24)) {
      pdu_io_write32(ea.regmap + EA_IRQ_STAT, irq_stat);
      tasklet_schedule(&ea_job_tasklet);
      return IRQ_HANDLED;
   }

   return IRQ_NONE;
}


#define HW_ENTRY(x) { x, #x }
static const struct { unsigned addr; char *name; } reg_names[] = {
   HW_ENTRY(EA_IRQ_EN),
   HW_ENTRY(EA_IRQ_STAT),
   HW_ENTRY(EA_IRQ_CTRL),
   HW_ENTRY(EA_FIFO_STAT),
   HW_ENTRY(EA_FIFO_STAT_PRIO),
   HW_ENTRY(EA_SRC_PTR),
   HW_ENTRY(EA_DST_PTR),
   HW_ENTRY(EA_OFFSET),
   HW_ENTRY(EA_SA_PTR),
   HW_ENTRY(EA_SW_CTRL),
   HW_ENTRY(EA_CTRL),
   HW_ENTRY(EA_POP),
   HW_ENTRY(EA_STATUS),
   HW_ENTRY(EA_STAT_WD_CTRL),
   HW_ENTRY(EA_VERSION),

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
      sprintf(out, "%-25s = %08lx\n", name, (unsigned long)pdu_io_read32(ea.regmap + reg_addr));
      strcat(buf, out);
   }
   return strlen(buf);
}

static DEVICE_ATTR(reg,                0400, show_reg, NULL);
static const struct attribute_group ea_attr_group = {
   .attrs = (struct attribute *[]) {
      &dev_attr_reg.attr,
      NULL
   }
};

static int spaccea_probe(struct platform_device *pdev)
{
   void *baseaddr;
   struct resource *res, *irq;
   spacc_device *spacc;
   int err;

   irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
   res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   if (!res || !irq) {
      return -EINVAL;
   }
   printk("spaccea_probe: Device at %08lx(%08lx) of size %lu bytes\n", (unsigned long)res->start, (unsigned long)res->end, (unsigned long)resource_size(res));

   baseaddr = pdu_linux_map_regs(&pdev->dev, res);
   if (IS_ERR(baseaddr)) {
      dev_err(&pdev->dev, "unable to map iomem\n");
      return PTR_ERR(baseaddr);
   }

   /* get the associated spacc */
   printk ("spaccea_probe::EA attaching to SPAcc EPN %X\n", ((pdu_info *)(pdev->dev.platform_data))->spacc_version.project);
   spacc = get_spacc_device_by_epn(((pdu_info *)(pdev->dev.platform_data))->spacc_version.project, 0);
   if (spacc == NULL) {
      return -ENODEV;
   }

   err = ea_init(&ea, spacc, baseaddr, 32);
   if (err) {
      return -ENODEV;
   }

   err = sysfs_create_group(&pdev->dev.kobj, &ea_attr_group);
   if (err) {
      ea_deinit(&ea);
      return -1;
   }

   // after we call re_init we need to allocate a pool
   printk("spaccea_probe::EA requires %zu bytes for SA\n", ea.sa_ptr_mem_req);
#ifndef USE_ZYNQ_OCM
   ea.sa_ptr_virt = pdu_dma_alloc(ea.sa_ptr_mem_req, &ea.sa_ptr_phys);
#else
   // assume that the SA only needs 8K which is the default...
   printk("ea::OCM use enabled, using first 8K of 0xFFFC_0000 as SA\n");
   ea.sa_ptr_virt = ioremap_nocache(0xFFFC0000, 0x2000);
   ea.sa_ptr_phys = 0xFFFC0000;
#endif
   if (!ea.sa_ptr_virt) {
      printk("spaccea_probe::Cannot allocate SA pool\n");
      ea_deinit(&ea);
      sysfs_remove_group(&pdev->dev.kobj, &ea_attr_group);
      return -ENODEV;
   }

   if (no_latency || ea.spacc->config.pdu_version <= 0x24) {
      if (devm_request_irq(&pdev->dev, irq->start, ea_irq_handler_v24, IRQF_SHARED, dev_name(&pdev->dev), &pdev->dev)) {
         dev_err(&pdev->dev, "spaccea_probe::failed to request IRQ\n");
         ea_deinit(&ea);
         pdu_dma_free(ea.sa_ptr_mem_req, ea.sa_ptr_virt, ea.sa_ptr_phys);
         sysfs_remove_group(&pdev->dev.kobj, &ea_attr_group);
         return -ENODEV;
      }
      EA_IRQ_ENABLE_CMD(&ea, 0);
   } else {
      if (devm_request_irq(&pdev->dev, irq->start, ea_irq_handler, IRQF_SHARED, dev_name(&pdev->dev), &pdev->dev)) {
         dev_err(&pdev->dev, "spaccea_probe::failed to request IRQ\n");
         ea_deinit(&ea);
         pdu_dma_free(ea.sa_ptr_mem_req, ea.sa_ptr_virt, ea.sa_ptr_phys);
         sysfs_remove_group(&pdev->dev.kobj, &ea_attr_group);
         return -ENODEV;
      }
      EA_IRQ_DISABLE_CMD(&ea);
      EA_IRQ_DISABLE_CMD1(&ea);
      EA_IRQ_ENABLE_STAT_WD(&ea, timer);
   }

   // enable interrupts
   EA_IRQ_ENABLE_STAT(&ea, ea.config.ideal_stat_limit);
   EA_IRQ_ENABLE_GLBL(&ea);

   ea_dev = pdev;

   return 0;
}

static int spaccea_remove(struct platform_device *pdev)
{
   printk("ea_mod_exit::Freeing resources\n");

   // fini the SDK then free the SA pool
   EA_IRQ_DISABLE_GLBL(&ea);
   ea_deinit(&ea);
#ifndef USE_ZYNQ_OCM
   pdu_dma_free(ea.sa_ptr_mem_req, ea.sa_ptr_virt, ea.sa_ptr_phys);
#else
   iounmap(ea.sa_ptr_virt);
#endif
   sysfs_remove_group(&pdev->dev.kobj, &ea_attr_group);

   return 0;
}

static struct platform_driver spaccea_driver = {
   .probe  = spaccea_probe,
   .remove = spaccea_remove,
   .driver = {
      .name  = "spacc-ea",
      .owner = THIS_MODULE
   }
};

static int __init ea_mod_init (void)
{
   memset(&ea, 0, sizeof ea);
   return platform_driver_register(&spaccea_driver);
}

static void __exit ea_mod_exit (void)
{
   platform_driver_unregister(&spaccea_driver);
}

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init (ea_mod_init);
module_exit (ea_mod_exit);

module_param(timer, int, 0600);
MODULE_PARM_DESC(timer, "STAT_WD timer in cycles");

module_param(no_latency, int, 0);
MODULE_PARM_DESC(no_latency, "Set to 1 to have a low latency IRQ mechanism");

// kernel
EXPORT_SYMBOL (ea_get_device);
EXPORT_SYMBOL (ea_get_platform_device);

// lib functions
EXPORT_SYMBOL (ea_init);
EXPORT_SYMBOL (ea_deinit);
EXPORT_SYMBOL (ea_open);
EXPORT_SYMBOL (ea_build_sa);
EXPORT_SYMBOL (ea_go);
EXPORT_SYMBOL (ea_close);
EXPORT_SYMBOL (ea_done);
EXPORT_SYMBOL (ea_pop_packet);
EXPORT_SYMBOL (ea_set_mode);
