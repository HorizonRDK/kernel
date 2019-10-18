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

#include "elppci.h"

struct elppci_device {
   char name[12];
   unsigned mem_size;
};

enum {
   ELPPCI_DEVICE_PKA,
   ELPPCI_DEVICE_TRNG3,
   ELPPCI_DEVICE_EAPE,
};

const struct elppci_device elppci_devices[] = {
   [ELPPCI_DEVICE_PKA]   = { .name = "pka",   .mem_size = 0x4000 },
   [ELPPCI_DEVICE_TRNG3] = { .name = "trng3", .mem_size = 0x80   },
   [ELPPCI_DEVICE_EAPE] = { .name = "eape", .mem_size = 0x100 },
};

static void elppci_enable_irqs(struct pci_dev *pdev)
{
   u32 cfg;

   pci_read_config_dword(pdev, ELPPCI_CTRL_REG, &cfg);
   pci_write_config_dword(pdev, ELPPCI_CTRL_REG, cfg | ELPPCI_IRQ_EN_BIT);
}

static void elppci_reset(struct pci_dev *pdev)
{
   u32 cfg;

   pci_read_config_dword(pdev, ELPPCI_CTRL_REG, &cfg);
   pci_write_config_dword(pdev, ELPPCI_CTRL_REG, cfg | ELPPCI_RST_BIT);
   pci_write_config_dword(pdev, ELPPCI_CTRL_REG, cfg & ~ELPPCI_RST_BIT);
}

static int elppci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
   const struct elppci_device *desc;
   struct platform_device *child;
   struct resource res[2];
   int rc;

   BUG_ON(id->driver_data >= ARRAY_SIZE(elppci_devices));
   desc = &elppci_devices[id->driver_data];

   rc = pci_enable_device(pdev);
   if (rc < 0) {
      dev_err(&pdev->dev, "failed to enable device\n");
      return rc;
   }

   if (pci_resource_len(pdev, 1) < desc->mem_size) {
      dev_err(&pdev->dev, "device BAR size is too small\n");
      return -ENOMEM;
   }

   res[0] = (struct resource) {
      .parent = &pdev->resource[1],
      .start  = pci_resource_start(pdev, 1),
      .end    = pci_resource_start(pdev, 1) + desc->mem_size - 1,
      .flags  = IORESOURCE_MEM,
   };
   res[1] = (struct resource) {
      .start = pdev->irq,
      .end   = pdev->irq,
      .flags = IORESOURCE_IRQ,
   };

   elppci_reset(pdev);
   elppci_enable_irqs(pdev);

   child = platform_device_register_resndata(&pdev->dev, desc->name, -1,
                                             res, ARRAY_SIZE(res), NULL, 0);
   if (IS_ERR(child))
      return PTR_ERR(child);

   pci_set_drvdata(pdev, child);
   return 0;
}

static void elppci_remove(struct pci_dev *pdev)
{
   struct platform_device *child = pci_get_drvdata(pdev);

   platform_device_unregister(child);
   pci_disable_device(pdev);
}

static const struct pci_device_id elppci_ids[] = {
   { PCI_DEVICE(0xe117, 0xc10e), .driver_data = ELPPCI_DEVICE_PKA   },
   { PCI_DEVICE(0xe117, 0xd156), .driver_data = ELPPCI_DEVICE_TRNG3 },
   { PCI_DEVICE(0xe117, 0xE5F2), .driver_data = ELPPCI_DEVICE_EAPE },
   { 0 },
};
MODULE_DEVICE_TABLE(pci, elppci_ids);

static struct pci_driver elppci_driver = {
   .name     = "elppci-standalone",
   .probe    = elppci_probe,
   .remove   = elppci_remove,
   .id_table = elppci_ids,
};
module_pci_driver(elppci_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
MODULE_DESCRIPTION("Lab PCI driver for standalone devices.");
