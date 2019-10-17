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
#include <linux/ratelimit.h>
#include <linux/io.h>

#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <asm/uaccess.h>
#include <asm/param.h>
#include <linux/platform_device.h>
#include "elpre.h"
#include "elprehw.h"
#include "elpspaccdrv.h"

static re_device re;

re_device *re_get_device(void)
{
   return &re;
}

void re_pop_jobs(unsigned long foo)
{
//   printk("RE tasklet\n");
   re_packet_dequeue(&re, -1);
}

static DECLARE_TASKLET(re_job_tasklet, re_pop_jobs, 0UL);

/* a function to run callbacks in the IRQ handler */
static irqreturn_t re_irq_handler(int irq, void *dev)
{
  uint32_t irq_stat, d;

  irq_stat = pdu_io_read32(re.regmap + RE_IRQ_STAT);
//printk("RE IRQ %08zx\n", irq_stat);
  d = 0;

  if (irq_stat & RE_IRQ_STAT_STAT) {
     d = 1;
     re.fifo_cnt = re.config.fifo_depth-1;
     pdu_io_write32(re.regmap+RE_IRQ_EN, RE_IRQ_EN_CMD|RE_IRQ_EN_STAT|RE_IRQ_EN_GLBL);
  }

  if (irq_stat & RE_IRQ_STAT_CMD) {
     d = 1;
     re.fifo_cnt = 1;
     pdu_io_write32(re.regmap+RE_IRQ_EN, RE_IRQ_EN_STAT|RE_IRQ_EN_GLBL);
  }

  if (d) {
     pdu_io_write32(re.regmap + RE_IRQ_STAT, irq_stat);
     pdu_io_write32(re.regmap+RE_IRQ_CTRL, RE_REG_IRQ_CTRL_STAT(re.fifo_cnt));
     tasklet_schedule(&re_job_tasklet);
  }

  return d ? IRQ_HANDLED : IRQ_NONE;
}

#define HW_ENTRY(x) { x, #x }
static const struct { unsigned addr; char *name; } reg_names[] = {
   HW_ENTRY(RE_IRQ_EN),
   HW_ENTRY(RE_IRQ_STAT),
   HW_ENTRY(RE_IRQ_CTRL),
   HW_ENTRY(RE_FIFO_STAT),
   HW_ENTRY(RE_SRC_PTR),
   HW_ENTRY(RE_DST_PTR),
   HW_ENTRY(RE_OFFSET),
   HW_ENTRY(RE_LEN),
   HW_ENTRY(RE_SA_PTR),
   HW_ENTRY(RE_SW_CTRL),
   HW_ENTRY(RE_CTRL),
   HW_ENTRY(RE_POP),
   HW_ENTRY(RE_STATUS),
   HW_ENTRY(RE_VERSION),
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
      sprintf(out, "%-25s = %08lx\n", name, (unsigned long)pdu_io_read32(re.regmap + reg_addr));
      strcat(buf, out);
   }
   return strlen(buf);
}

static DEVICE_ATTR(reg,                0400, show_reg, NULL);
static const struct attribute_group re_attr_group = {
   .attrs = (struct attribute *[]) {
      &dev_attr_reg.attr,
      NULL
   }
};

static int spaccre_probe(struct platform_device *pdev)
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
   printk("spaccre_probe: Device at %08lx(%08lx) of size %lu bytes\n", (unsigned long)res->start, (unsigned long)res->end, (unsigned long)resource_size(res));

   baseaddr = pdu_linux_map_regs(&pdev->dev, res);
   if (IS_ERR(baseaddr)) {
      dev_err(&pdev->dev, "unable to map iomem\n");
      return PTR_ERR(baseaddr);
   }

   /* get the associated spacc */
   printk ("spaccre_probe::RE attaching to SPAcc EPN %X\n", ((pdu_info *)(pdev->dev.platform_data))->spacc_version.project);
   spacc = get_spacc_device_by_epn(((pdu_info *)(pdev->dev.platform_data))->spacc_version.project, 0);
   if (spacc == NULL) {
      return -ENODEV;
   }

   err = re_init(baseaddr, spacc, &re);
   if (err != CRYPTO_OK) {
      return -1;
   }

   err = sysfs_create_group(&pdev->dev.kobj, &re_attr_group);
   if (err) {
      re_fini(&re);
      return -1;
   }

   // after we call re_init we need to allocate a pool
   re.sa_pool_virt = pdu_dma_alloc(re.sa_pool_size, &re.sa_pool_phys);
   if (!re.sa_pool_virt) {
      printk("spaccre_probe::Cannot allocate SA pool\n");
      sysfs_remove_group(&pdev->dev.kobj, &re_attr_group);
      re_fini(&re);
      return -1;
   }

   if (devm_request_irq(&pdev->dev, irq->start, re_irq_handler, IRQF_SHARED, dev_name(&pdev->dev), &pdev->dev)) {
      dev_err(&pdev->dev, "failed to request IRQ\n");
      sysfs_remove_group(&pdev->dev.kobj, &re_attr_group);
      re_fini(&re);
      pdu_dma_free(re.sa_pool_size, re.sa_pool_virt, re.sa_pool_phys);
      return -EBUSY;
   }

   // enable interrupts
   pdu_io_write32(baseaddr+RE_IRQ_CTRL, RE_REG_IRQ_CTRL_STAT(1));
   pdu_io_write32(baseaddr+RE_IRQ_EN, RE_IRQ_EN_STAT|RE_IRQ_EN_GLBL);

   return 0;
}

static int spaccre_remove(struct platform_device *pdev)
{
   printk("re_mod_exit::Freeing resources\n");

   // fini the SDK then free the SA pool
   pdu_io_write32(re.regmap+RE_IRQ_EN, 0); // disable IRQs
   sysfs_remove_group(&pdev->dev.kobj, &re_attr_group);
   re_fini(&re);
   pdu_dma_free(re.sa_pool_size, re.sa_pool_virt, re.sa_pool_phys);

   return 0;
}

static struct platform_driver spaccre_driver = {
   .probe  = spaccre_probe,
   .remove = spaccre_remove,
   .driver = {
      .name  = "spacc-re",
      .owner = THIS_MODULE
   }
};

static int __init re_mod_init (void)
{
   memset(&re, 0, sizeof re);
   return platform_driver_register(&spaccre_driver);
}

static void __exit re_mod_exit (void)
{
   platform_driver_unregister(&spaccre_driver);
}

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init (re_mod_init);
module_exit (re_mod_exit);

// kernel
EXPORT_SYMBOL (re_get_device);

// lib functions
EXPORT_SYMBOL (re_reset_sa);
EXPORT_SYMBOL (re_set_next_read);
EXPORT_SYMBOL (re_set_next_write);
EXPORT_SYMBOL (re_start_operation);
EXPORT_SYMBOL (re_start_operation_ex);
EXPORT_SYMBOL (re_finish_operation);
EXPORT_SYMBOL (re_error_msg);
EXPORT_SYMBOL (re_retrieve_sa);
EXPORT_SYMBOL (re_write_sa);
EXPORT_SYMBOL (re_get_spacc_context);
EXPORT_SYMBOL (re_init_context_ex);
EXPORT_SYMBOL (re_get_context_ex);
EXPORT_SYMBOL (re_init_context);
EXPORT_SYMBOL (re_get_context);
EXPORT_SYMBOL (re_release_context);
EXPORT_SYMBOL (re_set_next_read_iv);
EXPORT_SYMBOL (re_set_next_read_key);
EXPORT_SYMBOL (re_set_next_read_mackey);
EXPORT_SYMBOL (re_set_next_read_params);
EXPORT_SYMBOL (re_set_next_read_sequence_number);
EXPORT_SYMBOL (re_set_next_write_iv);
EXPORT_SYMBOL (re_set_next_write_key);
EXPORT_SYMBOL (re_set_next_write_mackey);
EXPORT_SYMBOL (re_set_next_write_params);
EXPORT_SYMBOL (re_set_next_write_sequence_number);
EXPORT_SYMBOL (re_packet_dequeue);
EXPORT_SYMBOL (re_print_diag);

