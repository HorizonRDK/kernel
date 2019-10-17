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
#include <linux/random.h>

#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/param.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include "eape.h"

#define DEVNAME "eape: "

static eape_device eape;
static struct platform_device *eape_dev;
static int         wd_timer=75000;

eape_device *eape_get_device(void)
{
   return &eape;
}

struct platform_device *eape_get_platform_device(void)
{
   return eape_dev;
}

void eape_pop_jobs(unsigned long _eape)
{
   eape_pop_packet(&eape);
}

static DECLARE_TASKLET(eape_job_tasklet, eape_pop_jobs, 0UL);

/*********************************************************************
 * Function - eape_irq_handler
 *
 * Description - Interrupt handler for th EAPE device.
 *
 * Inputs - Interrupt assigned to device
 *          Device
 *
 * Outputs - Status
 *
 ********************************************************************/
static irqreturn_t eape_irq_handler(int irq, void *dev)
{
   uint32_t irq_stat;

   irq_stat = pdu_io_read32(eape.regmap + EAPE_REG_IRQ_STAT);
   if (irq_stat) {
      pdu_io_write32(eape.regmap + EAPE_REG_IRQ_STAT, irq_stat);

#ifdef EAPE_STATS
      if (irq_stat & EAPE_IRQ_STAT_IN_STAT_MASK)     { EAPE_STATS_INC(eape.stats.irqs.in_stat); }
      if (irq_stat & EAPE_IRQ_STAT_IN_STAT_WD_MASK)  { EAPE_STATS_INC(eape.stats.irqs.in_wd); }
      if (irq_stat & EAPE_IRQ_STAT_OUT_STAT_MASK)    { EAPE_STATS_INC(eape.stats.irqs.out_stat); }
      if (irq_stat & EAPE_IRQ_STAT_OUT_STAT_WD_MASK) { EAPE_STATS_INC(eape.stats.irqs.out_wd); }
#endif

      tasklet_schedule(&eape_job_tasklet);
      return IRQ_HANDLED;
   }
   return IRQ_NONE;
}

#ifdef EAPE_STATS
static ssize_t show_stats(struct device *dev, struct device_attribute *devattr, char *buf)
{
   return sprintf(buf, "IRQ: in.stat=%u, in.wd=%u, out.stat=%u, out.wd=%u\nPassed: in=%u, out=%u\nDropped: in=%u, out=%u\n",
      eape.stats.irqs.in_stat, eape.stats.irqs.in_wd,
      eape.stats.irqs.out_stat, eape.stats.irqs.out_wd,
      eape.stats.pass_in, eape.stats.pass_out, eape.stats.dropped_in, eape.stats.dropped_out);
}

static ssize_t store_stats(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   memset(&eape.stats, 0, sizeof(eape.stats));
   return count;
}

static ssize_t show_wd_timer(struct device *dev, struct device_attribute *devattr, char *buf)
{
   return sprintf(buf, "%lu\n", (unsigned long)pdu_io_read32(eape.regmap+EAPE_REG_STAT_WD_CTL));
}

static ssize_t store_wd_timer(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   unsigned long timer;
   (void)kstrtoul(buf, 10, &timer);
   if (timer < 10000 || timer > 1000000) {
      timer = 25000;
   }
   pdu_io_write32(eape.regmap+EAPE_REG_STAT_WD_CTL, wd_timer=timer);
   return count;
}

#define HW_ENTRY(x) { x, #x }

static const struct { unsigned addr; char *name; } reg_names[] = {
   HW_ENTRY(EAPE_REG_IRQ_EN),
   HW_ENTRY(EAPE_REG_IRQ_STAT),
   HW_ENTRY(EAPE_REG_IRQ_CTRL),
   HW_ENTRY(EAPE_REG_FIFO_STAT),
   HW_ENTRY(EAPE_REG_STAT_WD_CTL),
   HW_ENTRY(EAPE_REG_DMA_BURST),
   HW_ENTRY(EAPE_REG_OUT_SRC_PTR),
   HW_ENTRY(EAPE_REG_OUT_DST_PTR),
   HW_ENTRY(EAPE_REG_OUT_OFFSET),
   HW_ENTRY(EAPE_REG_OUT_ID),
   HW_ENTRY(EAPE_REG_OUT_SAI),
   HW_ENTRY(EAPE_REG_OUT_POP),
   HW_ENTRY(EAPE_REG_OUT_STAT),
   HW_ENTRY(EAPE_REG_IN_SRC_PTR),
   HW_ENTRY(EAPE_REG_IN_DST_PTR),
   HW_ENTRY(EAPE_REG_IN_OFFSET),
   HW_ENTRY(EAPE_REG_IN_ID),
   HW_ENTRY(EAPE_REG_IN_SAI),
   HW_ENTRY(EAPE_REG_IN_POP),
   HW_ENTRY(EAPE_REG_IN_STAT),
   HW_ENTRY(EAPE_REG_CACHE_FLUSH),
   HW_ENTRY(EAPE_REG_CACHE_RDY),
   HW_ENTRY(EAPE_REG_VERSION0),
   HW_ENTRY(EAPE_REG_VERSION1),
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
      sprintf(out, "%-25s = %08lx\n", name, (unsigned long)pdu_io_read32(eape.regmap + reg_addr));
      strcat(buf, out);
   }
   return strlen(buf);
}

static DEVICE_ATTR(stats,              0644, show_stats, store_stats);
static DEVICE_ATTR(wd_timer,           0600, show_wd_timer, store_wd_timer);
static DEVICE_ATTR(reg,                0400, show_reg, NULL);
static const struct attribute_group eape_attr_group = {
   .attrs = (struct attribute *[]) {
      &dev_attr_stats.attr,
      &dev_attr_wd_timer.attr,
      &dev_attr_reg.attr,
      NULL
   }
};
#endif

static int eape_probe(struct platform_device *pdev)
{
   void *baseaddr;
   struct resource *res, *irq;
   int err;
   uint32_t tmp, eape_seed[8];

   /* Initialize DDT DMA pools based on this device's resources */
   if (pdu_mem_init(&pdev->dev)) {
      dev_err(&pdev->dev, "Could not initialize DMA pools\n");
      return -ENOMEM;
   }

   eape_dev = pdev;

   //Standard Linux driver function to get system resources
   irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
   res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   if (!res || !irq) {
      return -EINVAL;
   }
   printk("%s eape_probe: Device at %08lx(%08lx) of size %lu bytes\n", DEVNAME, (unsigned long)res->start, (unsigned long)res->end, (unsigned long)resource_size(res));

   baseaddr = pdu_linux_map_regs(&pdev->dev, res);
   if (IS_ERR(baseaddr)) {
      dev_err(&pdev->dev, "unable to map iomem\n");
      return PTR_ERR(baseaddr);
   }

   /* Initialize the driver*/
   err = eape_init(&eape, baseaddr, EAPE_NUM_CONTEXTS);
   if (err) {
      return -1;
   }
   printk("%s requires %zu bytes for SA\n", DEVNAME, eape.sa.sa_ptr_mem_req);

#ifdef EAPE_STATS
   err = sysfs_create_group(&pdev->dev.kobj, &eape_attr_group);
   if (err) {
      eape_deinit(&eape);
      return -1;
   }
#endif

  /* initialize RNG */
   if (eape.config.version_1.rng) {
      get_random_bytes(&eape_seed, sizeof eape_seed);
      eape_rng_init(&eape, eape_seed);
      memset(eape_seed, 0, sizeof eape_seed);
   }

   //Allocate DMA memory for the SA
   eape.sa.sa_ptr_virt = pdu_dma_alloc(eape.sa.sa_ptr_mem_req, &eape.sa.sa_ptr_phys);
   if (!eape.sa.sa_ptr_virt) {
#ifdef EAPE_STATS
      sysfs_remove_group(&pdev->dev.kobj, &eape_attr_group);
#endif
      eape_deinit(&eape);
#ifdef EAPE_STATS
      sysfs_remove_group(&pdev->dev.kobj, &eape_attr_group);
#endif
      printk("Cannot allocate SA pool\n");
      return -1;
   }

   //Get an IRQ assigned to this device
   if (devm_request_irq(&pdev->dev, irq->start, eape_irq_handler, IRQF_SHARED, dev_name(&pdev->dev), &pdev->dev)) {
#ifdef EAPE_STATS
      sysfs_remove_group(&pdev->dev.kobj, &eape_attr_group);
#endif
      eape_deinit(&eape);
      dev_err(&pdev->dev, "failed to request IRQ\n");
      pdu_dma_free(eape.sa.sa_ptr_mem_req, eape.sa.sa_ptr_virt, eape.sa.sa_ptr_phys);
      return -EBUSY;
   }
   //Set the Watchdog timeout
   pdu_io_write32(eape.regmap+EAPE_REG_STAT_WD_CTL, wd_timer);

   // program STAT_CNT variables
   if (eape.config.version_1.fifo_depth > 2) {
      tmp = eape.config.version_1.fifo_depth - 2;
   } else {
      tmp = 1;
   }
   pdu_io_write32(eape.regmap+EAPE_REG_IRQ_CTRL, (tmp<<EAPE_IRQ_CTRL_OUT_STAT_CNT_OFFSET)|(tmp<<EAPE_IRQ_CTRL_IN_STAT_CNT_OFFSET));

   // Enable interrupts (watchdogs + STAT_CNT)
   pdu_io_write32(eape.regmap+EAPE_REG_IRQ_EN,  EAPE_IRQ_EN_OUT_STAT_WD_EN |
                                                EAPE_IRQ_EN_IN_STAT_WD_EN  |
                                                EAPE_IRQ_EN_IN_STAT_EN     |
                                                EAPE_IRQ_EN_OUT_STAT_EN    |
                                                EAPE_IRQ_EN_GLBL_EN);

   //display some mildly useful stuff
   printk("eape: Bound to a v%u.%u EAPE from EPN %04x\n", eape.config.version_0.major, eape.config.version_0.minor, eape.config.version_0.project);
   printk("eape: Features={ %s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s}\n",
      eape.config.version_1.rng?"rng ":"",
      eape.config.version_0.tx_cache?"tx_cache ":"",eape.config.version_0.rx_cache?"rx_cache ":"",
      eape.config.version_0.sad_64?"sad_64 ":"", eape.config.version_0.pbm_64?"pbm_64 ":"",
      eape.config.version_0.ipv6?"ipv6 ":"", eape.config.version_1.md5?"md5 ":"",
      eape.config.version_1.sha1?"sha1 ":"", eape.config.version_1.sha256?"sha256 ":"",
      eape.config.version_1.sha384?"sha384 ":"", eape.config.version_1.sha512?"sha512":"",
      eape.config.version_1.des_cbc?"des ":"", eape.config.version_1.aes_cbc?"aes_cbc ":"",
      eape.config.version_1.aes_ctr?"aes_ctr ":"", eape.config.version_1.aes_gcm?"aes_gcm ":"",
      eape.config.version_1.aes_128?"aes_128 ":"", eape.config.version_1.aes_192?"aes_192 ":"",
      eape.config.version_1.aes_256?"aes_256 ":"");
   printk("eape: FIFO depth=%u, TX pipes=%u, RX pipes=%u\n", eape.config.version_1.fifo_depth, eape.config.version_1.tx_pipes, eape.config.version_1.rx_pipes);

   return 0;
}

static int eape_remove(struct platform_device *pdev)
{
   printk("%s eape_mod_exit::Freeing resources\n", DEVNAME);

   // fini the SDK then free the SA pool
#ifdef EAPE_STATS
   sysfs_remove_group(&pdev->dev.kobj, &eape_attr_group);
#endif
   eape_deinit(&eape);
   pdu_dma_free(eape.sa.sa_ptr_mem_req, eape.sa.sa_ptr_virt, eape.sa.sa_ptr_phys);
   pdu_mem_deinit(&pdev->dev);

   return 0;
}

static struct platform_driver eape_driver = {
   .probe  = eape_probe,
   .remove = eape_remove,
   .driver = {
      .name  = "eape",
      .owner = THIS_MODULE
   }
};

static int __init eape_mod_init (void)
{
   memset(&eape, 0, sizeof eape);
   return platform_driver_register(&eape_driver);
}

static void __exit eape_mod_exit (void)
{
   platform_driver_unregister(&eape_driver);
}

module_param(wd_timer, int, 0);
MODULE_PARM_DESC(wd_timer, "Watchdog Timer count (def=25000 cycles)");

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init (eape_mod_init);
module_exit (eape_mod_exit);

// kernel
EXPORT_SYMBOL (eape_get_device);
EXPORT_SYMBOL (eape_get_platform_device);

// lib functions
EXPORT_SYMBOL (eape_init);
EXPORT_SYMBOL (eape_deinit);
EXPORT_SYMBOL (eape_open);
EXPORT_SYMBOL (eape_build_sa);
EXPORT_SYMBOL (eape_go);
EXPORT_SYMBOL (eape_close);
EXPORT_SYMBOL (eape_done);
EXPORT_SYMBOL (eape_pop_packet);


