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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pci.h>

#include "elpsaspa_hw.h"
#include "elppci.h"

static bool do_pci_reset = true;
module_param_named(pci_reset, do_pci_reset, bool, 0);

static bool do_pci_rings = true;
module_param_named(pci_rings, do_pci_rings, bool, 0);

static void saspa_pci_set_rings(struct pci_dev *pdev, int set)
{
   u32 cfg;

   pci_read_config_dword(pdev, ELPPCI_CTRL_REG, &cfg);
   if (set)
      cfg |= ELPPCI_TRNG_RINGS_EN_BIT;
   else
      cfg &= ~ELPPCI_TRNG_RINGS_EN_BIT;
   pci_write_config_dword(pdev, ELPPCI_CTRL_REG, cfg);
}

static void saspa_pci_set_dpa(struct pci_dev *pdev, int set)
{
   u32 cfg;

   pci_read_config_dword(pdev, ELPPCI_CTRL_REG, &cfg);
   if (set)
      cfg |= ELPPCI_DPA_EN_BIT;
   else
      cfg &= ~ELPPCI_DPA_EN_BIT;
   pci_write_config_dword(pdev, ELPPCI_CTRL_REG, cfg);
}

static void saspa_pci_enable_irqs(struct pci_dev *pdev)
{
   u32 cfg;

   pci_read_config_dword(pdev, ELPPCI_CTRL_REG, &cfg);
   pci_write_config_dword(pdev, ELPPCI_CTRL_REG, cfg | ELPPCI_IRQ_EN_BIT);
}

static void saspa_pci_reset(struct pci_dev *pdev)
{
   u32 cfg;

   pci_read_config_dword(pdev, ELPPCI_CTRL_REG, &cfg);
   pci_write_config_dword(pdev, ELPPCI_CTRL_REG, cfg | ELPPCI_RST_BIT);
   pci_write_config_dword(pdev, ELPPCI_CTRL_REG, cfg & ~ELPPCI_RST_BIT);
}

static int saspa_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
   struct platform_device *saspa;
   struct resource saspa_res[2];
   u32 __iomem *glbl_regs, tmp;
   int rc;

   rc = pci_enable_device(pdev);
   if (rc < 0) {
      dev_err(&pdev->dev, "failed to enable device\n");
      return rc;
   }

   saspa_res[0] = (struct resource) {
      .parent = &pdev->resource[1],
      .start = pci_resource_start(pdev, 1),
      .end   = pci_resource_start(pdev, 1) + SASPA_MEMORY_BASE - 1,
      .flags = IORESOURCE_MEM,
   };
   saspa_res[1] = (struct resource) {
      .start = pdev->irq,
      .end   = pdev->irq,
      .flags = IORESOURCE_IRQ,
   };

   /*
    * We must size the mem resource appropriately for available SASPA memory.
    * In our lab environment, this amount will alway match the configured
    * address width, which is available in the SASPA memory map.
    */
   glbl_regs = ioremap_nocache(saspa_res[0].start + SASPA_GLOBAL_BASE,
                               SASPA_GLOBAL_SIZE);
   if (!glbl_regs) {
      dev_err(&pdev->dev, "failed to map SASPA registers\n");
      return -ENOMEM;
   }

   tmp = readl(&glbl_regs[SASPA_GLBL_MAX_MEM_SZ]);
   tmp >>= SASPA_MAX_MEM_SZ;
   tmp &= (1ul << SASPA_MAX_MEM_SZ_BITS)-1;

   if (tmp > 0) {
      saspa_res[0].end += 16 << tmp;
   }

   iounmap(glbl_regs);

   if (do_pci_reset) {
      saspa_pci_reset(pdev);
   }

   saspa_pci_set_rings(pdev, do_pci_rings);
   saspa_pci_enable_irqs(pdev);

   if (resource_size(&saspa_res[0]) > pci_resource_len(pdev, 1)) {
      dev_err(&pdev->dev, "SASPA memory map does not fit in PCI BAR\n");
      return -ENODEV;
   }

   saspa = platform_device_register_resndata(&pdev->dev, "saspa", -1,
                                             saspa_res, ARRAY_SIZE(saspa_res),
                                             NULL, 0);
   if (IS_ERR(saspa))
      return PTR_ERR(saspa);

   pci_set_drvdata(pdev, saspa);
   return 0;
}

static void saspa_pci_remove(struct pci_dev *pdev)
{
   struct platform_device *saspa = pci_get_drvdata(pdev);

   platform_device_unregister(saspa);
   pci_disable_device(pdev);
}

static const struct pci_device_id saspa_pci_ids[] = {
   { PCI_DEVICE(0xe117, 0x5a5f) },
   { 0 },
};

static struct pci_driver saspa_pci_driver = {
   .name     = "saspa-pci",
   .probe    = saspa_pci_probe,
   .remove   = saspa_pci_remove,
   .id_table = saspa_pci_ids,
};
module_pci_driver(saspa_pci_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
MODULE_DESCRIPTION("Lab PCI driver for SASPA.");
