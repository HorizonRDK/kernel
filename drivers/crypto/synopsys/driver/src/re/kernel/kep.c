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
#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/io.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/param.h>
#include <linux/platform_device.h>
#include "elpkep.h"
#include "elpspaccdrv.h"

static kep_device kep;

kep_device *kep_get_device(void)
{
   return &kep;
}

/* a function to run callbacks in the IRQ handler */
static irqreturn_t kep_irq_handler(int irq, void *dev)
{
  uint32_t irq_stat, d;

  irq_stat = pdu_io_read32(kep.regmap + KEP_REG_IRQ_STAT);

  d = 0;

  if (irq_stat & KEP_IRQ_STAT_STAT) {
     d = 1;
     // clear any pending jobs
     kep_done(&kep, -1);
     pdu_io_write32(kep.regmap + KEP_REG_IRQ_STAT, KEP_IRQ_STAT_STAT);
  }

  return d ? IRQ_HANDLED : IRQ_NONE;
}

#define HW_ENTRY(x) { x, #x }
static const struct { unsigned addr; char *name; } reg_names[] = {
   HW_ENTRY(KEP_REG_IRQ_EN),
   HW_ENTRY(KEP_REG_IRQ_STAT),
   HW_ENTRY(KEP_REG_IRQ_CTRL),
   HW_ENTRY(KEP_REG_FIFO_STAT),
   HW_ENTRY(KEP_REG_SRC_PTR),
   HW_ENTRY(KEP_REG_DST_PTR),
   HW_ENTRY(KEP_REG_OFFSET),
   HW_ENTRY(KEP_REG_LEN),
   HW_ENTRY(KEP_REG_SW_CTRL),
   HW_ENTRY(KEP_REG_CTRL),
   HW_ENTRY(KEP_REG_POP),
   HW_ENTRY(KEP_REG_STATUS),
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
      sprintf(out, "%-25s = %08lx\n", name, (unsigned long)pdu_io_read32(kep.regmap + reg_addr));
      strcat(buf, out);
   }
   return strlen(buf);
}

static DEVICE_ATTR(reg,                0400, show_reg, NULL);
static const struct attribute_group kep_attr_group = {
   .attrs = (struct attribute *[]) {
      &dev_attr_reg.attr,
      NULL
   }
};


static int spacckep_probe(struct platform_device *pdev)
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

   printk("spacckep_probe: Device at %08lx(%08lx) of size %lu bytes\n", (unsigned long)res->start, (unsigned long)res->end, (unsigned long)resource_size(res));

   baseaddr = pdu_linux_map_regs(&pdev->dev, res);
   if (IS_ERR(baseaddr)) {
      dev_err(&pdev->dev, "unable to map iomem\n");
      return PTR_ERR(baseaddr);
   }

   /* get the associated spacc */
   printk ("KEP attaching to SPAcc EPN %X\n", ((pdu_info *)(pdev->dev.platform_data))->spacc_version.project);
   spacc = get_spacc_device_by_epn(((pdu_info *)(pdev->dev.platform_data))->spacc_version.project, 0);
   if (spacc == NULL) {
      return -ENODEV;
   }

   err = kep_init(baseaddr, spacc, &kep);
   if (err != CRYPTO_OK) {
      return -1;
   }

   err = sysfs_create_group(&pdev->dev.kobj, &kep_attr_group);
   if (err) {
      kep_fini(&kep);
      return -1;
   }


   if (devm_request_irq(&pdev->dev, irq->start, kep_irq_handler, IRQF_SHARED, dev_name(&pdev->dev), &pdev->dev)) {
      dev_err(&pdev->dev, "failed to request IRQ\n");
      sysfs_remove_group(&pdev->dev.kobj, &kep_attr_group);
      kep_fini(&kep);
      return -EBUSY;
   }

   // enable interrupts
   pdu_io_write32(baseaddr+KEP_REG_IRQ_CTRL, KEP_REG_IRQ_CTRL_STAT(1));
   pdu_io_write32(baseaddr+KEP_REG_IRQ_EN, KEP_IRQ_EN_STAT|KEP_IRQ_EN_GLBL);

   return 0;
}

static int spacckep_remove(struct platform_device *pdev)
{
   printk("kep_mod_exit::Freeing resources\n");

   sysfs_remove_group(&pdev->dev.kobj, &kep_attr_group);
   kep_fini(&kep);
   return 0;
}

static struct platform_driver spacckep_driver = {
   .probe  = spacckep_probe,
   .remove = spacckep_remove,
   .driver = {
      .name  = "spacc-kep",
      .owner = THIS_MODULE
   }
};

static int __init kep_mod_init (void)
{
   memset(&kep, 0, sizeof kep);
   return platform_driver_register(&spacckep_driver);
}

static void __exit kep_mod_exit (void)
{
   platform_driver_unregister(&spacckep_driver);
}


MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init (kep_mod_init);
module_exit (kep_mod_exit);

// kernel
EXPORT_SYMBOL (kep_get_device);

// lib
EXPORT_SYMBOL (kep_open);
EXPORT_SYMBOL (kep_is_valid);
EXPORT_SYMBOL (kep_load_keys);
EXPORT_SYMBOL (kep_go);
EXPORT_SYMBOL (kep_done);
EXPORT_SYMBOL (kep_close);

