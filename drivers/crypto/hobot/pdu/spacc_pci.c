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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sizes.h>
#include <linux/platform_device.h>
#include <linux/pci.h>

#include "elppdu.h"
#include "elppci.h"

void pdu_pci_set_dpa_off(struct pci_dev *tif)
{
   unsigned int conf = 0;
   pci_read_config_dword( tif, ELPPCI_CTRL_REG,&conf);
   pci_write_config_dword( tif, ELPPCI_CTRL_REG, conf & ~ELPPCI_DPA_EN_BIT);
}

void pdu_pci_set_dpa_on(struct pci_dev *tif)
{
   unsigned int conf = 0;
   pci_read_config_dword( tif, ELPPCI_CTRL_REG,&conf);
   pci_write_config_dword( tif, ELPPCI_CTRL_REG, conf | ELPPCI_DPA_EN_BIT);
}


void pdu_pci_set_trng_ring_off(struct pci_dev *tif)
{
   unsigned int conf = 0;
   pci_read_config_dword( tif, ELPPCI_CTRL_REG,&conf);
   pci_write_config_dword( tif, ELPPCI_CTRL_REG, conf & ~ELPPCI_TRNG_RINGS_EN_BIT);
}

void pdu_pci_set_trng_ring_on(struct pci_dev *tif)
{
   unsigned int conf = 0;
   pci_read_config_dword( tif, ELPPCI_CTRL_REG,&conf);
   pci_write_config_dword( tif, ELPPCI_CTRL_REG, conf | ELPPCI_TRNG_RINGS_EN_BIT);
}

void pdu_pci_set_little_endian(struct pci_dev *tif)
{
   unsigned int conf = 0;
   pci_read_config_dword( tif, ELPPCI_CTRL_REG,&conf);
   pci_write_config_dword( tif, ELPPCI_CTRL_REG, conf | ELPPCI_LITTLE_ENDIAN_BIT );
}

void pdu_pci_set_big_endian(struct pci_dev *tif)
{
   unsigned int conf = 0;
   pci_read_config_dword( tif, ELPPCI_CTRL_REG,&conf);
   pci_write_config_dword( tif, ELPPCI_CTRL_REG,conf & ~(ELPPCI_LITTLE_ENDIAN_BIT));
}

void pdu_pci_set_secure_off(struct pci_dev *tif)
{
   unsigned int conf = 0;
   pci_read_config_dword( tif, ELPPCI_CTRL_REG,&conf);
   pci_write_config_dword( tif, ELPPCI_CTRL_REG, conf & ~ELPPCI_SECURE_BIT);
}

void pdu_pci_set_secure_on(struct pci_dev *tif)
{
   unsigned int conf = 0;
   pci_read_config_dword( tif, ELPPCI_CTRL_REG,&conf);
   pci_write_config_dword( tif, ELPPCI_CTRL_REG, conf | ELPPCI_SECURE_BIT);
}

void pdu_pci_interrupt_enabled(struct pci_dev *tif)
{
   unsigned int conf = 0;
   pci_read_config_dword( tif, ELPPCI_CTRL_REG, &conf);
   pci_write_config_dword( tif, ELPPCI_CTRL_REG, conf | ELPPCI_IRQ_EN_BIT );
}

void pdu_pci_reset(struct pci_dev *tif)
{
   unsigned int conf = 0;
   pci_read_config_dword( tif, ELPPCI_CTRL_REG, &conf);
   pci_write_config_dword( tif, ELPPCI_CTRL_REG, conf | ELPPCI_RST_BIT);
   pci_read_config_dword( tif, ELPPCI_CTRL_REG, &conf);
   pci_write_config_dword( tif, ELPPCI_CTRL_REG, conf & ~(ELPPCI_RST_BIT));
}

void pdu_pci_or(struct pci_dev *tif, unsigned val)
{
   unsigned int conf;
   pci_read_config_dword( tif, ELPPCI_CTRL_REG, &conf);
   pci_write_config_dword( tif, ELPPCI_CTRL_REG, conf | val);
}

int pci_reset, pci_rings, dpa_on;
int pci_or;

#define MAX_CHILDREN 16

struct spdu_pci_priv {
   u32 __iomem *regs;
   unsigned num_children;
   struct platform_device *children[];
};

static int add_child(struct pci_dev *pdev, const char *name, int id,
                     unsigned reg_offset, unsigned reg_size,
                     pdu_info *info)
{
   struct spdu_pci_priv *priv = pci_get_drvdata(pdev);
   struct platform_device *child;
   struct resource res[] = {
      [0] = {
         .start = pci_resource_start(pdev, 1),
         .end   = pci_resource_end(pdev, 1),
         .flags = pci_resource_flags(pdev, 1),
      },
      [1] = {
         .start = pdev->irq,
         .end   = pdev->irq,
         .flags = IORESOURCE_IRQ,
      }
   };
   struct platform_device_info pdevinfo = {
      .name = name,
      .id = id,
      .res = res,
      .num_res = ARRAY_SIZE(res),
      .data = info,
      .size_data = sizeof *info,
      .dma_mask = pdev->dma_mask,
      .parent = &pdev->dev,
   };

   if (resource_size(&res[0]) < reg_size
       || resource_size(&res[0]) - reg_size < reg_offset)
   {
      dev_err(&pdev->dev, "PCI memory %pR does not cover register area\n",
                          &res[0]);
      return -ENOMEM;
   }

   res[0].start += reg_offset;
   res[0].end = res[0].start + (reg_size - 1);

   child = platform_device_register_full(&pdevinfo);
   if (IS_ERR(child)) {
      char suffix[16];
      snprintf(suffix, sizeof suffix, id >= 0 ? ".%d" : "", id);
      dev_err(&pdev->dev, "Failed to register %s%s\n", name, suffix);
      return PTR_ERR(child);
   }

   dev_info(&pdev->dev, "registered %s\n", dev_name(&child->dev));
   priv->children[priv->num_children++] = child;
   return 0;
}

static int spdu_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
   struct resource regs = pdev->resource[1];
   struct spdu_pci_priv *priv;
   uint32_t irq_en = 0;
   pdu_info info;
   unsigned i;
   int rc;

   rc = pci_enable_device(pdev);
   if (rc < 0) {
      dev_err(&pdev->dev, "failed to enable device\n");
      return rc;
   }

   if (resource_size(&regs) < 512) {
      dev_err(&pdev->dev, "PCI memory %pR does not cover register area\n",
                          &regs);
      return -ENOMEM;
   }
   regs.name = "spdu-regs";
   regs.end = regs.start + (512 - 1);

   priv = devm_kzalloc(&pdev->dev,
                       sizeof *priv + MAX_CHILDREN * sizeof *priv->children,
                       GFP_KERNEL);
   if (!priv) {
      return -ENOMEM;
   }
   pci_set_drvdata(pdev, priv);

   priv->regs = devm_ioremap_resource(&pdev->dev, &regs);
   if (IS_ERR(priv->regs))
      return PTR_ERR(priv->regs);

   /* Configure PCI<->elbone interface */
   #if defined(SDK_ENDIAN_LITTLE)
      pdu_pci_set_little_endian(pdev);
   #elif defined(SDK_ENDIAN_BIG)
      pdu_pci_set_big_endian(pdev);
   #endif

   /* Enable interrupt routing in the PCI bridge */
   pdu_pci_interrupt_enabled(pdev);

   if (pdu_get_version(priv->regs, &info)) {
      dev_err(&pdev->dev, "failed to retrieve configuration from SPAcc\n");
      return -EINVAL;
   }

   rc = pdu_mem_init(&pdev->dev);
   if (rc != 0) {
      dev_err(&pdev->dev, "failed to setup PDU DMA\n");
      return -ENOMEM;
   }

   /* Setup sub-devices on SPAcc-PDU */
   if (info.spacc_version.is_pdu) {
      if (info.pdu_config.is_re) {
         add_child(pdev, "spacc-re", -1, 0x8000, SZ_16K, &info);
         irq_en |= PDU_IRQ_EN_RE;
      }

      if (info.pdu_config.is_mpm) {
         add_child(pdev, "spacc-mpm", 0, 0xc000, SZ_16K, &info);
         irq_en |= PDU_IRQ_EN_MPM;
      }

      if (info.pdu_config.is_kep) {
         add_child(pdev, "spacc-kep", -1, 0x10000, SZ_16K, &info);
         irq_en |= PDU_IRQ_EN_KEP;
      }

      if (info.pdu_config.is_ea) {
         add_child(pdev, "spacc-ea", -1, 0x14000, SZ_16K, &info);
         irq_en |= PDU_IRQ_EN_EA;
      }

      if (info.pdu_config.is_rng) {
         add_child(pdev, "clp800", -1, 0x18000, SZ_32K, &info);
         irq_en |= PDU_IRQ_EN_RNG;
      }

      if (info.pdu_config.is_pka) {
         add_child(pdev, "pka", -1, 0x20000, SZ_128K, &info);
         irq_en |= PDU_IRQ_EN_PKA;
      }
   }

   /* Setup SPAcc sub-devices */
   for (i = 0; i < info.spacc_config.num_vspacc; i++) {
      bool is_pdu = info.spacc_version.is_pdu;
      add_child(pdev, "spacc", (info.spacc_version.project << 16 | i),
                               0x40000 * (is_pdu + i), SZ_256K, &info);
      irq_en |= PDU_IRQ_EN_VSPACC(i);
   }

   /* Enable interrupts for all sub-devices on SPAcc-PDU */
   if (info.spacc_version.is_pdu) {
      pdu_io_write32(&priv->regs[0], PDU_IRQ_EN_GLBL | irq_en);
   }

   return 0;
}

static void spdu_pci_remove(struct pci_dev *pdev)
{
   struct spdu_pci_priv *priv = pci_get_drvdata(pdev);
   unsigned i;

   for (i = 0; i < priv->num_children; i++) {
      struct platform_device *child = priv->children[i];
      dev_info(&pdev->dev, "unregistered %s\n", dev_name(&child->dev));
      platform_device_unregister(child);
   }

   pdu_mem_deinit(&pdev->dev);
   pci_disable_device(pdev);
}

static const struct pci_device_id spdu_pci_ids[] = {
   { PCI_DEVICE(0xe117, 0x59ad) },
   { PCI_DEVICE(0xe117, 0x59ae) },
   { 0 },
};
MODULE_DEVICE_TABLE(pci, spdu_pci_ids);

static struct pci_driver spdu_pci_driver = {
   .name     = "elppci-spacc",
   .probe    = spdu_pci_probe,
   .remove   = spdu_pci_remove,
   .id_table = spdu_pci_ids,
};
module_pci_driver(spdu_pci_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Synopsys, Inc.");

module_param(pci_or, int, 0);
MODULE_PARM_DESC(pci_or, "Value to OR into PCI config register");
module_param (pci_reset, int, 0);
MODULE_PARM_DESC (pci_reset, "Reset the PCI");
module_param (pci_rings, int, 0);
MODULE_PARM_DESC (pci_rings, "Set the PCI rings");

EXPORT_SYMBOL (pdu_pci_set_trng_ring_off);
EXPORT_SYMBOL (pdu_pci_set_trng_ring_on);
EXPORT_SYMBOL (pdu_pci_set_little_endian);
EXPORT_SYMBOL (pdu_pci_set_big_endian);
EXPORT_SYMBOL (pdu_pci_set_secure_off);
EXPORT_SYMBOL (pdu_pci_set_secure_on);
EXPORT_SYMBOL (pdu_pci_interrupt_enabled);
EXPORT_SYMBOL (pdu_pci_reset);
