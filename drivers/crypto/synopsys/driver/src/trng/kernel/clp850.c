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
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <asm/uaccess.h>
#include <asm/param.h>
#include <linux/err.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <linux/crypto.h>
#include <crypto/internal/rng.h>

#include "elpclp850.h"

#define ELLIPTIC_HWRNG_DRIVER_NAME "hwrng-clp850"

typedef struct {
  elpclp850_state elpclp850;
  void *hwrng_drv;
  void *crypto_drv;
}elliptic_elpclp850_driver;

int elpclp850_hwrng_driver_read(struct hwrng *rng, void *buf, size_t max, bool wait);

static int elpclp850_platform_driver_read(struct platform_device *pdev, void *buf, size_t max, bool wait)
{
   elliptic_elpclp850_driver *data = 0;
   int elpclp850_error = -1;
   uint32_t m, n, out[4];
   

   if ((pdev == 0) ||
       (buf == 0) ||
       (max == 0) ) {
      return elpclp850_error;
   }

   data = platform_get_drvdata(pdev);
   if (data == 0) {
      return elpclp850_error;
   }
   
   m = max;
   while (m) {
      n = (m > 16) ? 16 : m;
      elpclp850_error = elpclp850_hw_read(&data->elpclp850, out);
      if (elpclp850_error < 0) {
         return elpclp850_error;
      }
      memcpy(buf, out, n);
      buf += n;
      m   -= n;
   }
   memset(out, 0, sizeof out);

   return max;
}

int elpclp850_hwrng_driver_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
   struct platform_device *pdev = 0;

   if (rng == 0) {
      return -1;
   }

   pdev = (struct platform_device *)rng->priv;
   return elpclp850_platform_driver_read(pdev, buf, max, wait);
}

static ssize_t show_epn(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "0x%.4hx\n", priv->elpclp850.config.build_id.epn);
}

static ssize_t show_stepping(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "0x%.4hx\n", priv->elpclp850.config.build_id.stepping);
}

static ssize_t show_features(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "diag_trng3=%u, diag_st_hlt=%u, secure_rst_state=%u\n", priv->elpclp850.config.features.diag_level_trng3, priv->elpclp850.config.features.diag_level_st_hlt, priv->elpclp850.config.features.secure_rst_state);
}


static ssize_t show_secure(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%s\n", CLP850_REG_MODE_GET_SECURE_EN(pdu_io_read32(priv->elpclp850.base + CLP850_REG_MODE)) ? "on" : "off");
}

static ssize_t store_secure(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   if (sysfs_streq(buf, "on")) {
      elpclp850_set_secure(&priv->elpclp850, 1, 1);
   } else if (sysfs_streq(buf, "off")) {
      elpclp850_set_secure(&priv->elpclp850, 0, 1);
   }
   return count;
}

static ssize_t show_nonce(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%s\n", CLP850_REG_MODE_GET_NONCE(pdu_io_read32(priv->elpclp850.base + CLP850_REG_MODE)) ? "on" : "off");
}

static ssize_t store_nonce(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   if (sysfs_streq(buf, "on")) {
      elpclp850_set_nonce(&priv->elpclp850, 1, 1);
   } else if (sysfs_streq(buf, "off")) {
      elpclp850_set_nonce(&priv->elpclp850, 0, 1);
   }
   return count;
}

static ssize_t show_kat(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%s\n", CLP850_REG_MODE_GET_KAT(pdu_io_read32(priv->elpclp850.base + CLP850_REG_MODE)) ? "on" : "off");
}

static ssize_t store_kat(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   if (sysfs_streq(buf, "on")) {
      elpclp850_set_kat(&priv->elpclp850, 1, 1);
   } else if (sysfs_streq(buf, "off")) {
      elpclp850_set_kat(&priv->elpclp850, 0, 1);
   }
   return count;
}

static ssize_t show_sec_alg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%s\n", CLP850_REG_MODE_GET_SEC_ALG(pdu_io_read32(priv->elpclp850.base + CLP850_REG_MODE)) ? "on" : "off");
}

static ssize_t store_sec_alg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   if (sysfs_streq(buf, "on")) {
      elpclp850_set_keylen(&priv->elpclp850, 1, 1);
   } else if (sysfs_streq(buf, "off")) {
      elpclp850_set_keylen(&priv->elpclp850, 0, 1);
   }
   return count;
}

static ssize_t show_wait_for_ht(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%s\n", CLP850_REG_MODE_GET_WAIT_FOR_HT(pdu_io_read32(priv->elpclp850.base + CLP850_REG_MODE)) ? "on" : "off");
}

static ssize_t store_wait_for_ht(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   if (sysfs_streq(buf, "on")) {
      elpclp850_set_wait_for_ht(&priv->elpclp850, 1, 1);
   } else if (sysfs_streq(buf, "off")) {
      elpclp850_set_wait_for_ht(&priv->elpclp850, 0, 1);
   }
   return count;
}

static ssize_t show_rand_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   unsigned x;
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   for (x = 0; x < 4; x++) {
      sprintf(buf + 8*x, "%08lx", pdu_io_read32(priv->elpclp850.base + CLP850_REG_RAND0 + x));
   }
   strcat(buf, "\n");
   return strlen(buf);
}

static ssize_t show_seed_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   unsigned x;
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   for (x = 0; x < 16; x++) {
      sprintf(buf + 8*x, "%08lx", pdu_io_read32(priv->elpclp850.base + CLP850_REG_SEED0 + x));
   }
   strcat(buf, "\n");
   return strlen(buf);
}

static ssize_t store_seed_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned x, tmp;
   
   // string must be at least 16 32-bit words long in 0 padded hex
   if (count < (2*16*4)) {
      return -1;
   }
   
   foo[8] = 0;
   for (x = 0; x < 16; x++) {
      memcpy(foo, buf + x * 8, 8);
      kstrtouint(foo, 16, &tmp);
      pdu_io_write32(priv->elpclp850.base + CLP850_REG_SEED0 + x, tmp);
   }
   return count;
}

static ssize_t show_ctrl_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp850.base + CLP850_REG_CTRL));
}

static ssize_t store_ctrl_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned tmp;
   
   // string must be at least a 32-bit word in 0 padded hex
   if (count < 8) {
      return -1;
   }
   
   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtouint(foo, 16, &tmp);
   pdu_io_write32(priv->elpclp850.base + CLP850_REG_CTRL, tmp);
   return count;
}

static ssize_t show_istat_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp850.base + CLP850_REG_ISTAT));
}

static ssize_t store_istat_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned tmp;
   
   // string must be at least a 32-bit word in 0 padded hex
   if (count < 8) {
      return -1;
   }
   
   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtouint(foo, 16, &tmp);
   pdu_io_write32(priv->elpclp850.base + CLP850_REG_ISTAT, tmp);
   return count;
}

static ssize_t show_mode_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp850.base + CLP850_REG_MODE));
}

static ssize_t store_mode_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned tmp;
   
   // string must be at least a 32-bit word in 0 padded hex
   if (count < 8) {
      return -1;
   }
   
   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtouint(foo, 16, &tmp);
   pdu_io_write32(priv->elpclp850.base + CLP850_REG_MODE, tmp);
   return count;
}

static ssize_t show_alarm_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp850.base + CLP850_REG_ALARM));
}

static ssize_t show_stat_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp850.base + CLP850_REG_STAT));
}

static ssize_t store_ia_wdata_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned tmp;
   
   // string must be at least a 32-bit word in 0 padded hex
   if (count < 8) {
      return -1;
   }
   
   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtouint(foo, 16, &tmp);
   pdu_io_write32(priv->elpclp850.base + CLP850_REG_IA_WDATA, tmp);
   return count;
}

static ssize_t show_ia_wdata_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp850.base + CLP850_REG_IA_WDATA));
}

static ssize_t show_ia_rdata_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp850.base + CLP850_REG_IA_RDATA));
}

static ssize_t store_ia_addr_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned tmp;
   
   // string must be at least a 32-bit word in 0 padded hex
   if (count < 8) {
      return -1;
   }
   
   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtouint(foo, 16, &tmp);
   pdu_io_write32(priv->elpclp850.base + CLP850_REG_IA_ADDR, tmp);
   return count;
}

static ssize_t show_ia_addr_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp850.base + CLP850_REG_IA_ADDR));
}

static ssize_t store_ia_cmd_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned tmp;
   
   // string must be at least a 32-bit word in 0 padded hex
   if (count < 8) {
      return -1;
   }
   
   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtouint(foo, 16, &tmp);
   pdu_io_write32(priv->elpclp850.base + CLP850_REG_IA_CMD, tmp);
   return count;
}

static ssize_t show_ia_cmd_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp850_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp850.base + CLP850_REG_IA_CMD));
}


static DEVICE_ATTR(epn,              0444, show_epn,         NULL);
static DEVICE_ATTR(stepping,         0444, show_stepping,    NULL);
static DEVICE_ATTR(features,         0444, show_features,    NULL);
static DEVICE_ATTR(secure,           0644, show_secure,      store_secure);
static DEVICE_ATTR(nonce,            0644, show_nonce,       store_nonce);
static DEVICE_ATTR(kat,              0644, show_kat,         store_kat);
static DEVICE_ATTR(sec_alg,          0644, show_sec_alg,     store_sec_alg);
static DEVICE_ATTR(wait_for_ht,      0644, show_wait_for_ht, store_wait_for_ht);

static DEVICE_ATTR(mode_reg,         0644, show_mode_reg,  store_mode_reg);
static DEVICE_ATTR(alarm_reg,        0444, show_alarm_reg, NULL);
static DEVICE_ATTR(rand_reg,         0400, show_rand_reg,  NULL);
static DEVICE_ATTR(seed_reg,         0600, show_seed_reg,  store_seed_reg);
static DEVICE_ATTR(ctrl_reg,         0644, show_ctrl_reg,  store_ctrl_reg);
static DEVICE_ATTR(istat_reg,        0644, show_istat_reg, store_istat_reg);
static DEVICE_ATTR(stat_reg,         0444, show_stat_reg,  NULL);

static DEVICE_ATTR(ia_wdata_reg,     0600, show_ia_wdata_reg, store_ia_wdata_reg);
static DEVICE_ATTR(ia_rdata_reg,     0400, show_ia_rdata_reg, NULL);
static DEVICE_ATTR(ia_addr_reg,      0600, show_ia_addr_reg,  store_ia_addr_reg);
static DEVICE_ATTR(ia_cmd_reg,       0600, show_ia_cmd_reg,   store_ia_cmd_reg);

static const struct attribute_group elpclp850_attr_group = {
   .attrs = (struct attribute *[]) {
      &dev_attr_epn.attr,
      &dev_attr_stepping.attr,
      &dev_attr_features.attr,
      &dev_attr_secure.attr,
      &dev_attr_nonce.attr,
      &dev_attr_kat.attr,
      &dev_attr_sec_alg.attr,
      &dev_attr_wait_for_ht.attr,

      &dev_attr_mode_reg.attr,
      &dev_attr_alarm_reg.attr,
      &dev_attr_rand_reg.attr,
      &dev_attr_seed_reg.attr,
      &dev_attr_ctrl_reg.attr,
      &dev_attr_istat_reg.attr,
      &dev_attr_stat_reg.attr,
      
      &dev_attr_ia_wdata_reg.attr,
      &dev_attr_ia_rdata_reg.attr,
      &dev_attr_ia_addr_reg.attr,
      &dev_attr_ia_cmd_reg.attr,
      NULL
   },
};

static int elpclp850_driver_probe(struct platform_device *pdev)
{
   struct resource *cfg, *irq;
   elliptic_elpclp850_driver *data;
   int ret;
   struct hwrng *hwrng_driver_info = 0;
   uint32_t *base_addr;

   cfg = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

   if (!cfg || !irq) {
      ELPHW_PRINT("no memory or IRQ resource\n");
      return -ENOMEM;
   }

   printk("elpclp850_probe: Device at %08lx(%08lx) of size %lu bytes\n", (unsigned long)cfg->start, (unsigned long)cfg->end, (unsigned long)resource_size(cfg));

   data = devm_kzalloc(&pdev->dev, sizeof(elliptic_elpclp850_driver), GFP_KERNEL);
   if (!data) {
      return -ENOMEM;
   }

   platform_set_drvdata(pdev, data);

   base_addr = pdu_linux_map_regs(&pdev->dev, cfg);
   if (IS_ERR(base_addr)) {
      dev_err(&pdev->dev, "unable to remap io mem\n");
      return PTR_ERR(base_addr);
   }

   if ( (ret = elpclp850_init(&data->elpclp850, (uint32_t*)base_addr)) != 0) {
      ELPHW_PRINT("CLP850 init failed (%d)\n", ret);
      devm_kfree(&pdev->dev, data);
      return ret;
   }
   
   hwrng_driver_info = devm_kzalloc(&pdev->dev, sizeof(struct hwrng), GFP_KERNEL);
   if (!hwrng_driver_info) {
      devm_kfree(&pdev->dev, data);
      return -ENOMEM;
   }

   hwrng_driver_info->name = devm_kzalloc(&pdev->dev, sizeof(ELLIPTIC_HWRNG_DRIVER_NAME) + 1, GFP_KERNEL);
   if (!hwrng_driver_info->name) {
      devm_kfree(&pdev->dev, data);
      devm_kfree(&pdev->dev, hwrng_driver_info);
      return -ENOMEM;
   }

   memset((void *)hwrng_driver_info->name, 0, sizeof(ELLIPTIC_HWRNG_DRIVER_NAME) + 1);
   strcpy((char *)hwrng_driver_info->name, ELLIPTIC_HWRNG_DRIVER_NAME);

   hwrng_driver_info->read = &elpclp850_hwrng_driver_read;
   hwrng_driver_info->data_present = 0;
   hwrng_driver_info->priv = (unsigned long)pdev;

   data->hwrng_drv = hwrng_driver_info;
   ret = hwrng_register(hwrng_driver_info);

   if (ret) {
      ELPHW_PRINT("unable to load HWRNG driver (error %d)\n", ret);
      devm_kfree(&pdev->dev, (void *)hwrng_driver_info->name);
      devm_kfree(&pdev->dev, hwrng_driver_info);
      devm_kfree(&pdev->dev, data);
      return ret;
   }

   ret = sysfs_create_group(&pdev->dev.kobj, &elpclp850_attr_group);
   if (ret < 0) {
      ELPHW_PRINT("unable to initialize sysfs group (error %d)\n", ret);
      hwrng_unregister(hwrng_driver_info);
      devm_kfree(&pdev->dev, (void *)hwrng_driver_info->name);
      devm_kfree(&pdev->dev, hwrng_driver_info);
      devm_kfree(&pdev->dev, data);
      return ret;
   }
   ELPHW_PRINT("ELP CLP850 registering HW_RANDOM\n");
   return 0;
}

static int elpclp850_driver_remove(struct platform_device *pdev)
{
   elliptic_elpclp850_driver *data = platform_get_drvdata(pdev);
   struct hwrng *hwrng_driver_info = (struct hwrng *)data->hwrng_drv;

   ELPHW_PRINT("ELP CLP850 removed\n");

   ELPHW_PRINT("ELP CLP850 unregistering from HW_RANDOM\n");
   hwrng_unregister(hwrng_driver_info);
   sysfs_remove_group(&pdev->dev.kobj, &elpclp850_attr_group);
   devm_kfree(&pdev->dev, (void *)hwrng_driver_info->name);
   devm_kfree(&pdev->dev, hwrng_driver_info);
   devm_kfree(&pdev->dev, data);
   return 0;
}

static struct platform_driver s_elpclp850_platform_driver_info = {
   .probe      = elpclp850_driver_probe,
   .remove     = elpclp850_driver_remove,
   .driver     = {
      .name = "clp850",
      .owner   = THIS_MODULE,
   },
};

static int __init elpclp850_platform_driver_start(void)
{
   ELPHW_PRINT("elpclp850_platform_driver_start\n");
   return platform_driver_register(&s_elpclp850_platform_driver_info);
}

static void __exit elpclp850_platform_driver_end(void)
{
   ELPHW_PRINT("elpclp850_platform_driver_end\n");
   platform_driver_unregister(&s_elpclp850_platform_driver_info);
}

module_init(elpclp850_platform_driver_start);
module_exit(elpclp850_platform_driver_end);

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
