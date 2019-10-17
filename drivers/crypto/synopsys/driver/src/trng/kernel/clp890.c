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

#include "elpclp890.h"

#define ELLIPTIC_HWRNG_DRIVER_NAME "hwrng-clp890"

#define num_gen_bytes 64
static unsigned long int max_reads = 128;

typedef struct {
  elpclp890_state elpclp890;
  void *hwrng_drv;
  void *crypto_drv;
  unsigned char rand_out[num_gen_bytes];
}elliptic_elpclp890_driver;

int elpclp890_hwrng_driver_read(struct hwrng *rng, void *buf, size_t max, bool wait);

static void elpclp890_reinit(elpclp890_state *clp890)
{
   int err;

   if ((err = elpclp890_uninstantiate(clp890)) && (err != CRYPTO_NOT_INSTANTIATED)) { goto ERR; }
   if ((err = elpclp890_instantiate(clp890, 128, 1, NULL)))   { goto ERR; }
   
ERR:
   DEBUG("CLP890:  Trying to reinitialize after a fatal alarm: %d\n", err);
}

static int elpclp890_platform_driver_read(struct platform_device *pdev, void *buf, size_t max, bool wait)
{
   elliptic_elpclp890_driver *data = 0;
   int elpclp890_error = -1;   
   uint32_t *out = (uint32_t *)kmalloc(max, GFP_KERNEL);
   
   if (out == NULL) {
      ELPHW_PRINT("memory not allocated\n"); return -1;
   }

   if ((pdev == 0) ||
       (buf == 0) ||
       (max == 0) ) {
      return elpclp890_error;
   }

   data = platform_get_drvdata(pdev);
   if (data == 0) {
      return elpclp890_error;
   }      

   elpclp890_error = elpclp890_generate(&data->elpclp890, out, max, data->elpclp890.status.sec_strength ? 256 : 128, data->elpclp890.status.pred_resist, NULL);
   if (elpclp890_error < 0) {
      if (data->elpclp890.status.alarm_code) {
         elpclp890_reinit(&data->elpclp890);
      }
      return elpclp890_error;
   }
   memcpy(buf, out, max);
   kfree(out);

   return max;
}

int elpclp890_hwrng_driver_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
   struct platform_device *pdev = 0;

   if (rng == 0) {
      return -1;
   }

   pdev = (struct platform_device *)rng->priv;
   return elpclp890_platform_driver_read(pdev, buf, max, wait);
}

static ssize_t show_epn(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "0x%.4hx\n", priv->elpclp890.config.build_id.epn);
}

static ssize_t show_stepping(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "0x%.4hx\n", priv->elpclp890.config.build_id.stepping);
}

static ssize_t show_features(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "drbg_arch = %u, diag_trng3=%u, diag_st_hlt=%u, diag_ns=%u, secure_rst_state=%u, extra_ps_present=%u\n", 
      priv->elpclp890.config.features.drbg_arch, priv->elpclp890.config.features.diag_level_trng3, 
      priv->elpclp890.config.features.diag_level_stat_hlt, priv->elpclp890.config.features.diag_level_ns, 
      priv->elpclp890.config.features.secure_rst_state, priv->elpclp890.config.features.extra_ps_present);
}

static ssize_t show_secure(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%s\n", CLP890_REG_SMODE_GET_SECURE_EN(pdu_io_read32(priv->elpclp890.base + CLP890_REG_SMODE)) ? "on" : "off");
}

static ssize_t store_secure(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{   
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   int ret;
   
   if ((ret = elpclp890_set_secure_mode(&priv->elpclp890, sysfs_streq(buf, "on") ? 1 : 0))) { return -1; }
   
   return count;
}

static ssize_t show_nonce(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%s\n", CLP890_REG_SMODE_GET_NONCE(pdu_io_read32(priv->elpclp890.base + CLP890_REG_SMODE)) ? "on" : "off");
}

static ssize_t store_nonce(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   int ret;
   
   if ((ret = elpclp890_set_nonce_mode(&priv->elpclp890, sysfs_streq(buf, "on")?1:0))) { return -1; }

	return count;   
}

static ssize_t show_sec_strength(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);

   return sprintf(buf, "%s\n", priv->elpclp890.status.sec_strength ? "256" : "128");
}

static ssize_t store_sec_strength(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   int tmp;
   int ret;
   
   if (count > 8) { return -1; }
   
   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtoint(foo, 10, &tmp);
   if ((ret = elpclp890_set_sec_strength(&priv->elpclp890, tmp))) { return -1; }

	return count;   
}

static ssize_t show_rand_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   unsigned x;
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   for (x = 0; x < 4; x++) {
      sprintf(buf + 8*x, "%08lx", pdu_io_read32(priv->elpclp890.base + CLP890_REG_RAND0 + 3 - x));
   }
   strcat(buf, "\n");
   return strlen(buf);
}

static ssize_t show_seed_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   unsigned x;  
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   for (x = 0; x < 12 ; x++) {      
      sprintf(buf + 8*x, "%08lx", pdu_io_read32(priv->elpclp890.base + CLP890_REG_SEED0 + 11 - x));
   }
   strcat(buf, "\n");
   return strlen(buf);
}

static ssize_t store_seed_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned x, tmp;

   // string must be at least 12 32-bit words long in 0 padded hex
   if (count < (2*12*4)) {
      return -1;
   }

   foo[8] = 0;
   for (x = 0; x < 12; x++) {
      memcpy(foo, buf + x * 8, 8);
      kstrtouint(foo, 16, &tmp);
      pdu_io_write32(priv->elpclp890.base + CLP890_REG_SEED0 + x, tmp);
   }
   return count;
}

static ssize_t show_npa_data_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   unsigned x;
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   for (x = 0; x < 16; x++) {
      sprintf(buf + 8*x, "%08lx", pdu_io_read32(priv->elpclp890.base + CLP890_REG_NPA_DATA0 + 15 - x));
   }
   strcat(buf, "\n");
   return strlen(buf);
}

static ssize_t store_npa_data_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
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
      pdu_io_write32(priv->elpclp890.base + CLP890_REG_NPA_DATA0 + x, tmp);
   }
   return count;
}

static ssize_t show_ctrl_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp890.base + CLP890_REG_CTRL));
}

static ssize_t store_ctrl_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned tmp;

   // string must be at least a 32-bit word in 0 padded hex
   if (count < 8) {
      return -1;
   }

   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtouint(foo, 16, &tmp);
   pdu_io_write32(priv->elpclp890.base + CLP890_REG_CTRL, tmp);
   return count;
}

static ssize_t show_istat_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp890.base + CLP890_REG_ISTAT));
}

static ssize_t store_istat_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned tmp;

   // string must be at least a 32-bit word in 0 padded hex
   if (count < 8) {
      return -1;
   }

   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtouint(foo, 16, &tmp);
   pdu_io_write32(priv->elpclp890.base + CLP890_REG_ISTAT, tmp);
   return count;
}

static ssize_t show_mode_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);   
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp890.base + CLP890_REG_MODE));
}

static ssize_t store_mode_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned tmp;

   // string must be at least a 32-bit word in 0 padded hex
   if (count < 8) {
      return -1;
   }

   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtouint(foo, 16, &tmp);
   pdu_io_write32(priv->elpclp890.base + CLP890_REG_MODE, tmp);   
   
   return count;
}

static ssize_t show_smode_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp890.base + CLP890_REG_SMODE));
}

static ssize_t store_smode_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned tmp;

   // string must be at least a 32-bit word in 0 padded hex
   if (count < 8) {
      return -1;
   }

   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtouint(foo, 16, &tmp);
   pdu_io_write32(priv->elpclp890.base + CLP890_REG_SMODE, tmp);
   return count;
}

static ssize_t show_alarm_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp890.base + CLP890_REG_ALARM));
}

static ssize_t store_alarm_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned tmp;

   // string must be at least a 32-bit word in 0 padded hex
   if (count < 8) {
      return -1;
   }

   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtouint(foo, 16, &tmp);
   pdu_io_write32(priv->elpclp890.base + CLP890_REG_ALARM, tmp);
   return count;
}

static ssize_t show_stat_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp890.base + CLP890_REG_STAT));
}

static ssize_t store_ia_wdata_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned tmp;

   // string must be at least a 32-bit word in 0 padded hex
   if (count < 8) {
      return -1;
   }

   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtouint(foo, 16, &tmp);
   pdu_io_write32(priv->elpclp890.base + CLP890_REG_IA_WDATA, tmp);
   return count;
}

static ssize_t show_ia_wdata_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp890.base + CLP890_REG_IA_WDATA));
}

static ssize_t show_ia_rdata_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp890.base + CLP890_REG_IA_RDATA));
}

static ssize_t store_ia_addr_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned tmp;

   // string must be at least a 32-bit word in 0 padded hex
   if (count < 8) {
      return -1;
   }

   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtouint(foo, 16, &tmp);
   pdu_io_write32(priv->elpclp890.base + CLP890_REG_IA_ADDR, tmp);
   return count;
}

static ssize_t show_ia_addr_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp890.base + CLP890_REG_IA_ADDR));
}

static ssize_t store_ia_cmd_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned tmp;

   // string must be at least a 32-bit word in 0 padded hex
   if (count < 8) {
      return -1;
   }

   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtouint(foo, 16, &tmp);
   pdu_io_write32(priv->elpclp890.base + CLP890_REG_IA_CMD, tmp);
   return count;
}

static ssize_t show_ia_cmd_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", pdu_io_read32(priv->elpclp890.base + CLP890_REG_IA_CMD));
}

static ssize_t show_hw_state(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   
   uint32_t addr;
   int i;
   int tot_char;
   
   addr = 0x20;
   tot_char = sprintf(buf, "Key = ");
   for (i = 0; i < 8; i++) {
      pdu_io_write32(priv->elpclp890.base + CLP890_REG_IA_ADDR, addr + 7 - i);
      pdu_io_write32(priv->elpclp890.base + CLP890_REG_IA_CMD, 0x80000000);
      tot_char += sprintf(buf + tot_char, "%08lx", pdu_io_read32(priv->elpclp890.base + CLP890_REG_IA_RDATA));
   }   
   tot_char += sprintf(buf + tot_char, "\n");
   
   addr = 0x28;
   tot_char += sprintf(buf + tot_char, "V = ");
   for (i = 0; i < 4; i++) {
      pdu_io_write32(priv->elpclp890.base + CLP890_REG_IA_ADDR, addr + 3 - i);
      pdu_io_write32(priv->elpclp890.base + CLP890_REG_IA_CMD, 0x80000000);
      tot_char += sprintf(buf + tot_char, "%08lx", pdu_io_read32(priv->elpclp890.base + CLP890_REG_IA_RDATA));
   }
   
   tot_char += sprintf(buf + tot_char, "\n");
   
   return tot_char;     
}

static ssize_t set_max_bits_per_req(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   char foo[9];
   unsigned long tmp;
   int ret;

   // string must be at least a 32-bit word in 0 padded hex
   if (count < 8) {
      return -1;
   }

   foo[8] = 0;
   memcpy(foo, buf, 8);
   kstrtoul(foo, 16, &tmp);
   if ((ret = elpclp890_set_reminder_max_bits_per_req(&priv->elpclp890, tmp))) { return -1; }   
   
	return count;
}

static ssize_t show_max_bits_per_req(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08lx\n", priv->elpclp890.counters.max_bits_per_req);
}

static ssize_t set_max_req_per_seed(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   char foo[17];
   unsigned long long tmp;
   int ret;

   // string must be at least a 64-bit word in 0 padded hex
   if (count < 16) {
      return -1;
   }

   foo[16] = 0;
   memcpy(foo, buf, 16);   
   kstrtoull(foo, 16, &tmp);  
   if ((ret = elpclp890_set_reminder_max_req_per_seed(&priv->elpclp890, tmp))) { return -1; }  
   
	return count;
}

static ssize_t show_max_req_per_seed(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   return sprintf(buf, "%08llx\n", priv->elpclp890.counters.max_req_per_seed);
}

static ssize_t collect_entropy(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);

   int rep;
   int i, j;
   int ret;

   // generate entropy
   if ((ret = elpclp890_get_entropy_input(&priv->elpclp890, NULL, 0))) { return -1; }
   
   // read NS_OUTPUTx
   // 32 reads if sec_strength is 128 and 48 reads if it is 256
   if (priv->elpclp890.status.sec_strength == SEC_STRNT_AES128) {
      rep = 32;
   } else if (priv->elpclp890.status.sec_strength == SEC_STRNT_AES256) {
      rep = 48;
   }

   for (i = 0; i < rep; i++) {
      pdu_io_write32(priv->elpclp890.base + CLP890_REG_IA_ADDR, 0x70 + rep - 1 - i);
      pdu_io_write32(priv->elpclp890.base + CLP890_REG_IA_CMD, 0x80000000);
      sprintf(buf + 8*i, "%08lx", pdu_io_read32(priv->elpclp890.base + CLP890_REG_IA_RDATA));
   }   

	strcat(buf, "\n");
   return strlen(buf);
}

static ssize_t cmd_nonce_seed_with_df(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);

   char foo[9];
   uint32_t seed[48] = {0};
   int rep;  
   int i;
   int ret;
   
   if (priv->elpclp890.status.sec_strength == SEC_STRNT_AES128) {
      rep = 2;
   } else if (priv->elpclp890.status.sec_strength == SEC_STRNT_AES256) {
      rep = 3;
   }

   DEBUG("Number of char in input = %u\n", count);
   if (count != (rep*128)) { return -1; }
   foo[8] = 0;
   for (i = 0; i < (rep*16); i++) {
      memcpy(foo, buf + i*8, 8);      
      kstrtouint(foo, 16, (seed+(rep*16-1)-i));
   }

   if ((ret = elpclp890_get_entropy_input(&priv->elpclp890, seed, 1))) { return -1; }

	return count;
}

static ssize_t cmd_nonce_seed_direct(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);

   char foo[9];
   uint32_t seed[12] = {0};
   int rep;  
   int i;
   int ret;
   
   if (priv->elpclp890.status.sec_strength == SEC_STRNT_AES128) {
      rep = 2;
   } else if (priv->elpclp890.status.sec_strength == SEC_STRNT_AES256) {
      rep = 3;
   }
   
   DEBUG("Number of char in input = %u\n", count);
   if (count != (rep*32)) { return -1; }
   foo[8] = 0;
   for (i = 0; i < (rep*4); i++) {
      memcpy(foo, buf + i*8, 8);      
      kstrtouint(foo, 16, (seed+(rep*4-1)-i));
   }       

   if ((ret = elpclp890_get_entropy_input(&priv->elpclp890, seed, 0))) { return -1; }

	return count;
}   

static ssize_t cmd_instantiate(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   
   char opts_str[101];
   unsigned opts_int;
   int req_sec_strength = 256;
   int pred_resist = 1;
   bool ps_exists = 0;
   uint32_t ps[12];
   unsigned ps_length;
   int i;
   int ret;
   
   /* First 3 digits: 
         they have to be 0 or 1
         2-1-0 --> 2: predictoin resistance, 1: security strength, 0: personilizatoin string existence
   */
   opts_str[3] = 0;   
   memcpy(opts_str, buf, 3);
   kstrtouint(opts_str, 2, &opts_int);
   
   if (((opts_str[0] != '0') && (opts_str[0] != '1')) ||
       ((opts_str[1] != '0') && (opts_str[1] != '1')) ||
       ((opts_str[2] != '0') && (opts_str[2] != '1'))) { 
      ELPHW_PRINT("Invalid input options: First 3 digits can only be 1 or 0\n");
      return -1;
   }
   
   if (opts_int & 1) {
      ps_exists = 1;
   } else {
      ps_exists = 0;
   }
   if (opts_int & 2) {
      req_sec_strength = 256;
   } else {
      req_sec_strength = 128;
   }
   if (opts_int & 4) {
      pred_resist = 1;
   } else {
      pred_resist = 0;
   }

   /* check input option length */
   if (!ps_exists) {      
      if (count != 3) { ELPHW_PRINT("Invalid input options: If personilization string does not exist, options has to be 3 char.\n"); return -1; }
   } else {
      if (req_sec_strength == 128) {
         if (count != 64+4) { // +4 for options and "-"
            ELPHW_PRINT("Invalid input options: If personilization string exists and security strength is 128-bit, options has to be 68 char (not %d char).\n", count);
            return -1; 
         }
      }
      else if (req_sec_strength == 256) {
         if (count != 96+4) { // +4 for options and "-", +1 because of the termination char that count includes
            ELPHW_PRINT("Invalid input options: If personilization string exists and security strength is 256-bit, options has to be 100 char (not %d char).\n", count);
            return -1;
         }
      }
      else { ELPHW_PRINT("Invalid input options\n"); return -1; }
   }
   
   /* Personilization string */
   for (i = 0; i < 12; i++) ps[i] = 0;
   if (req_sec_strength == 128) {
      ps_length = 64;
   } else if (req_sec_strength == 256) {
      ps_length = 96;
   } else {
      ELPHW_PRINT("Invalid security strength\n");
   }

   if (ps_exists) {
      opts_str[1] = 0;
      memcpy(opts_str, buf+3, 1);
      
      if(opts_str[0] == '-') {
         opts_str[8] = 0;
         for (i = 0; i < ps_length/8; i++) {
            memcpy(opts_str, buf+4+i*8, 8);                     
            kstrtouint(opts_str, 16, ps+(ps_length/8-1)-i);
         }
      } else {
         ELPHW_PRINT("4th character of input has to be \"-\" when personilization string exists\n");
      }
      if ((ret = elpclp890_instantiate(&priv->elpclp890, req_sec_strength, pred_resist, ps))) { return -1; }
   } else {
      if ((ret = elpclp890_instantiate(&priv->elpclp890, req_sec_strength, pred_resist, NULL))) { return -1; }
   }        

	return count;
}

static ssize_t cmd_uninstantiate(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   
   elpclp890_uninstantiate(&priv->elpclp890);
   
   return count;
}

static ssize_t cmd_reseed(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   
   char opts_str[100];
   unsigned opts_int;
   int pred_resist = 1;
   bool addin_exists = 0;
   uint32_t addin[12];
   unsigned addin_length;
   int i;
   int ret;

   /* First 2 digits: 
         they have to be 0 or 1
         1-0 --> 1: predictoin resistance, 0: additional input string existence
   */         
   opts_str[2] = 0;   
   memcpy(opts_str, buf, 2);
   kstrtouint(opts_str, 2, &opts_int);
   
   if (((opts_str[0] != '0') && (opts_str[0] != '1')) ||
       ((opts_str[1] != '0') && (opts_str[1] != '1'))) { 
      ELPHW_PRINT("Invalid input options: First 2 digits can only be 1 or 0\n");
      return -1;
   }   
   
   if (opts_int & 1) {
      addin_exists = 1;
   } else {
      addin_exists = 0;
   }   
   if (opts_int & 2) {
      pred_resist = 1;
   } else {
      pred_resist = 0;
   }

   /* check input option length */
   if (!addin_exists) {      
      if (count != 2) { ELPHW_PRINT("Invalid input options: If additional input does not exist, options has to be 2 char.\n"); return -1; }
   } else {
      if (priv->elpclp890.status.sec_strength == SEC_STRNT_AES128) {
         if (count != 64+3) { // +3 for options and "-"
            ELPHW_PRINT("Invalid input options: If additional input exists and security strength is 128-bit, options has to be 67 char.\n");
            return -1; 
         }
      }
      else if (priv->elpclp890.status.sec_strength == SEC_STRNT_AES256) {
         if (count != 96+3) { // +3 for options and "-"
            ELPHW_PRINT("Invalid input options: If additional input exists and security strength is 256-bit, options has to be 99 char.\n");
            return -1;
         }
      }
      else { ELPHW_PRINT("Invalid input options\n"); return -1; }
   }
   
   /* Additional input */
   for (i = 0; i < 12; i++) addin[i] = 0;
   if (priv->elpclp890.status.sec_strength == SEC_STRNT_AES128) {
      addin_length = 64;
   } else if (priv->elpclp890.status.sec_strength == SEC_STRNT_AES256) {
      addin_length = 96;
   } else {
      ELPHW_PRINT("Invalid security strength\n");
   }   
   
   if (addin_exists) {
      opts_str[1] = 0;
      memcpy(opts_str, buf+2, 1);

      if(opts_str[0] == '-') {
         opts_str[8] = 0;
         for (i = 0; i < addin_length/8; i++) {
            memcpy(opts_str, buf+3+i*8, 8);                     
            kstrtouint(opts_str, 16, addin+(addin_length/8-1)-i);
         }
      } else {
         ELPHW_PRINT("3rd character of input has to be \"-\" when additional input exists\n");
      }
      if ((ret = elpclp890_reseed(&priv->elpclp890, pred_resist, addin))) { return -1; }
   } else {
      if ((ret = elpclp890_reseed(&priv->elpclp890, pred_resist, NULL))) { return -1; }
   }
   
	return count;
}

static ssize_t cmd_generate(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   
   char opts_str[101];
   unsigned opts_int;
   int req_sec_strength = 128;
   int pred_resist = 1;
   bool addin_exists = 0;
   unsigned char out[num_gen_bytes];   
   uint32_t addin[12];
   unsigned addin_length;
   int i;
   int ret;

   /* First 3 digits: 
         they have to be 0 or 1
         2-1-0 --> 2: predictoin resistance, 1: security strength, 0: additional input string existence
   */
   opts_str[3] = 0;   
   memcpy(opts_str, buf, 3);
   kstrtouint(opts_str, 2, &opts_int);
   
   if (((opts_str[0] != '0') && (opts_str[0] != '1')) ||
       ((opts_str[1] != '0') && (opts_str[1] != '1')) ||
       ((opts_str[2] != '0') && (opts_str[2] != '1'))) { 
      ELPHW_PRINT("Invalid input options: First 3 digits can only be 1 or 0\n");
      return -1;
   }
   
   if (opts_int & 1) {
      addin_exists = 1;
   } else {
      addin_exists = 0;
   }
   if (opts_int & 2) {
      req_sec_strength = 256;
   } else {
      req_sec_strength = 128;
   }
   if (opts_int & 4) {
      pred_resist = 1;
   } else {
      pred_resist = 0;
   }
   
   /* check input option length */
   if (!addin_exists) {      
      if (count != 3) { ELPHW_PRINT("Invalid input options: If additional input does not exist, options has to be 3 char.\n"); return -1; }
   } else {
      if (req_sec_strength == 128) {
         if (count != 64+4) { // +4 for options and "-"
            ELPHW_PRINT("Invalid input options: If additional input exists and security strength is 128-bit, options has to be 68 char.\n");
            return -1; 
         }
      }
      else if (req_sec_strength == 256) {
         if (count != 96+4) { // +4 for options and "-"
            ELPHW_PRINT("Invalid input options: If additional input exists and security strength is 256-bit, options has to be 100 char.\n");
            return -1;
         }
      }
      else { ELPHW_PRINT("Invalid input options\n"); return -1; }
   }
   
   /* Additional input */
   for (i = 0; i < 12; i++) addin[i] = 0;
   if (req_sec_strength == 128) {
      addin_length = 64;
   } else if (req_sec_strength == 256) {
      addin_length = 96;
   } else {
      ELPHW_PRINT("Invalid security strength\n");
   }

   if (addin_exists) {
      opts_str[1] = 0;
      memcpy(opts_str, buf+3, 1);

      if(opts_str[0] == '-') {
         opts_str[8] = 0;
         for (i = 0; i < addin_length/8; i++) {
            memcpy(opts_str, buf+4+i*8, 8);                     
            kstrtouint(opts_str, 16, addin+(addin_length/8-1)-i);
         }
      } else {
         ELPHW_PRINT("4th character of input has to be \"-\" when additional input exists\n");
      }
      if ((ret = elpclp890_generate(&priv->elpclp890, (uint32_t*)out, num_gen_bytes, req_sec_strength, pred_resist, addin))) { return -1; }
   } else {
      if ((ret = elpclp890_generate(&priv->elpclp890, (uint32_t*)out, num_gen_bytes, req_sec_strength, pred_resist, NULL))) { return -1; }
   }

   /* store the result */
   memcpy(priv->rand_out, out, sizeof out);

   /*DEBUG("RAND_OUT --  ");
   for (i = 0; i < 16*4; i++) { DEBUG("%02x", priv->rand_out[i]); }
   DEBUG("\n");*/

	return count;
}

/* show_rand_out displays last generated random number (num_gen_bytes number of bytes), not just the last block. */
static ssize_t show_rand_out(struct device *dev, struct device_attribute *devattr, char *buf)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   unsigned i, j;
   unsigned long rand;   
   bool all_zero = true;
   
   /* If all bits of the rand_reg register are 0, display 0 */
   for (i = 0; i < 4; i++) {
      rand = pdu_io_read32(priv->elpclp890.base + CLP890_REG_RAND0 + (3-i));
      if (rand != 0) {
         all_zero = false;
         break;
      }      
   }

   if (all_zero) {
      sprintf(buf + 2*i, "%02x", 0);
   } else {
      for (i = 0; i < (num_gen_bytes/16); i++) {
         for (j = 0; j < 16; j++) {
            sprintf(buf + 2*(i*16+j), "%02x", priv->rand_out[(i+1)*16-1-j]);            
         }
      }
      j = 0;
      while (i*16+j < num_gen_bytes) {
         sprintf(buf + 2*(i*16+j), "%02x", priv->rand_out[num_gen_bytes-1-j]);
         j++;
      }
   }

   strcat(buf, "\n");
   return strlen(buf);
}

static ssize_t cmd_kat(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   int ret;

   if (sysfs_streq(buf, "full")) {
      if ((ret = elpclp890_full_kat(&priv->elpclp890))) { return -1; } 
   } else if (sysfs_streq(buf, "00")) {
      if ((ret = elpclp890_kat(&priv->elpclp890, 0, 0))) { return -1; }
   } else if (sysfs_streq(buf, "01")) {
      if ((ret = elpclp890_kat(&priv->elpclp890, 0, 1))) { return -1; }
   } else if (sysfs_streq(buf, "10")) {
      if ((ret = elpclp890_kat(&priv->elpclp890, 1, 0))) { return -1; }
   } else if (sysfs_streq(buf, "11")) {
      if ((ret = elpclp890_kat(&priv->elpclp890, 1, 1))) { return -1; }
   } else {
      if ((ret = elpclp890_full_kat(&priv->elpclp890))) { return -1; }
   }
   
	return count;
}

static void str_to_384_bit (char *buf, uint32_t *out) {
   char foo[9];
   int i;   

   foo[8] = 0;
   for (i = 0; i < 12; i++) {
      memcpy(foo, buf + i*8, 8);
      kstrtouint(foo, 16, out+11-i);
   }   
}

/* This attribute is only for test purpuses */
static ssize_t test_attr_func(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   elliptic_elpclp890_driver *priv = dev_get_drvdata(dev);
   
   int i;
   int err;
   uint32_t addin[12];
   uint32_t ps[12];
   char *out;
   
   char buf_seed1[96] = "c54805274bde00aa5289e0513579019707666d2fa7a1c8908865891c87c0c652335a4d3cc415bc30742b164647f8820f";
   char buf_ps1[96]   = "d63fb5afa2101fa4b8a6c3b89d9c250ac728fc1ddad0e7585b5d54728ed20c2f940e89155596e3b963635b6d6088164b";
   char buf_addin1[96] = "744bfae3c23a5cc9a3b373b6c50795068d35eb8a339746ac810d16f864e880061082edf9d2687c211960aa83400f85f9";
   char buf_seed2[96] = "b2ad31d1f20dcf30dd526ec9156c07f270216bdb59197325bab180675929888ab699c54fb21819b7d921d6346bff2f7f";
   char buf_addin2[96]   = "ad55c682962aa4fe9ebc227c9402e79b0aa7874844d33eaee7e2d15baf81d9d33936e4d93f28ad109657b512aee115a5";
   char buf_seed3[96] = "eca449048d26fd38f8ca435237dce66eadec7069ee5dd0b70084b819a711c0820a7556bbd0ae20f06e5169278b593b71";
   uint32_t tmp[12];

   for (i = 0; i < 12; i++) addin[i] = i;
   for (i = 0; i < 12; i++) ps[i] = i+100;

   /* SDK doc example - Prediction Resistance not available, no Reseed */
   if ((err = elpclp890_uninstantiate(&priv->elpclp890)) && (err != CRYPTO_NOT_INSTANTIATED)) { return -1; }

   if (elpclp890_instantiate(&priv->elpclp890, 128, 0, ps) < 0) { return -1; }
   
   out = (char *)kmalloc(10, GFP_KERNEL);
   if (elpclp890_generate(&priv->elpclp890, out, 10, 128, 0, addin) < 0) { return -1; }
   DEBUG("----- Generate 10 bytes\n");
   for (i = 0; i < 10; i++) { DEBUG("%02x", out[i]); } DEBUG("\n");
   kfree(out);
   
   out = (char *)kmalloc(512, GFP_KERNEL);
   if (elpclp890_generate(&priv->elpclp890, out, 512, 128, 0, addin) < 0) { return -1; }
   DEBUG("----- Generate 512 bytes\n");
   for (i = 0; i < 512; i++) { DEBUG("%02x", out[i]); } DEBUG("\n");
   kfree(out);
   
   out = (char *)kmalloc(41, GFP_KERNEL);
   if (elpclp890_generate(&priv->elpclp890, out, 41, 128, 0, addin) < 0) { return -1; }
   DEBUG("----- Generate 41 bytes\n");
   for (i = 0; i < 41; i++) { DEBUG("%02x", out[i]); } DEBUG("\n");
   kfree(out);
   
   if (((err = elpclp890_uninstantiate(&priv->elpclp890)) < 0) && (err != CRYPTO_NOT_INSTANTIATED)) { return -1; }


   /* SDK doc example - Prediction Resistance not available, with Reseed */
/*   if ((err = elpclp890_uninstantiate(&priv->elpclp890)) && (err != CRYPTO_NOT_INSTANTIATED)) { return -1; }

   if (elpclp890_instantiate(&priv->elpclp890, 256, 0, ps) < 0) { return -1; }
   
   if (elpclp890_reseed(&priv->elpclp890, 0, addin) < 0) { return -1; }
   
   out = (char *)kmalloc(64, GFP_KERNEL);
   if (elpclp890_generate(&priv->elpclp890, out, 64, 256, 0, addin) < 0) { return -1; }
   DEBUG("----- Generate 64 bytes\n");
   for (i = 0; i < 64; i++) { DEBUG("%02x", out[i]); } DEBUG("\n");
   kfree(out);
   
   out = (char *)kmalloc(64, GFP_KERNEL);
   if (elpclp890_generate(&priv->elpclp890, out, 64, 256, 0, addin) < 0) { return -1; }
   DEBUG("----- Generate 64 bytes\n");
   for (i = 0; i < 64; i++) { DEBUG("%02x", out[i]); } DEBUG("\n");
   kfree(out);
   
   out = (char *)kmalloc(64, GFP_KERNEL);
   if (elpclp890_generate(&priv->elpclp890, out, 64, 256, 0, addin) < 0) { return -1; }
   DEBUG("----- Generate 64 bytes\n");
   for (i = 0; i < 64; i++) { DEBUG("%02x", out[i]); } DEBUG("\n");
   kfree(out);
   
   if (elpclp890_reseed(&priv->elpclp890, 0, addin) < 0) { return -1; }
   
   out = (char *)kmalloc(64, GFP_KERNEL);
   if (elpclp890_generate(&priv->elpclp890, out, 64, 256, 0, addin) < 0) { return -1; }
   DEBUG("----- Generate 64 bytes\n");
   for (i = 0; i < 64; i++) { DEBUG("%02x", out[i]); } DEBUG("\n");
   kfree(out);
   
   if (((err = elpclp890_uninstantiate(&priv->elpclp890)) < 0) && (err != CRYPTO_NOT_INSTANTIATED)) { return -1; }
*/

   /* SDK doc example - Prediction Resistance available, no Reseed */
/*   if ((err = elpclp890_uninstantiate(&priv->elpclp890)) && (err != CRYPTO_NOT_INSTANTIATED)) { return -1; }

   if (elpclp890_instantiate(&priv->elpclp890, 128, 1, ps) < 0) { return -1; }
   
   out = (char *)kmalloc(120, GFP_KERNEL);
   if (elpclp890_generate(&priv->elpclp890, out, 120, 128, 1, addin) < 0) { return -1; }
   DEBUG("----- Generate 120 bytes\n");
   for (i = 0; i < 120; i++) { DEBUG("%02x", out[i]); } DEBUG("\n");
   kfree(out);
   
   out = (char *)kmalloc(120, GFP_KERNEL);
   if (elpclp890_generate(&priv->elpclp890, out, 120, 128, 1, addin) < 0) { return -1; }
   DEBUG("----- Generate 120 bytes\n");
   for (i = 0; i < 120; i++) { DEBUG("%02x", out[i]); } DEBUG("\n");
   kfree(out);
 
   if (((err = elpclp890_uninstantiate(&priv->elpclp890)) < 0) && (err != CRYPTO_NOT_INSTANTIATED)) { return -1; }
*/

   /* SDK doc example - Zeroize */
/*   if ((err = elpclp890_uninstantiate(&priv->elpclp890)) && (err != CRYPTO_NOT_INSTANTIATED)) { return -1; } 
   if ((err = elpclp890_uninstantiate(&priv->elpclp890)) && (err != CRYPTO_NOT_INSTANTIATED)) { return -1; } 
*/
   
   /* SDK doc example - Illegal sequences */
   /* gen without instantiate */
//   if ((err = elpclp890_uninstantiate(&priv->elpclp890)) && (err != CRYPTO_NOT_INSTANTIATED)) { return -1; }   
//   elpclp890_generate(&priv->elpclp890, out, 120, 128, 0, addin);
//   
//   /* reseed without instantiate */
//   if ((err = elpclp890_uninstantiate(&priv->elpclp890)) && (err != CRYPTO_NOT_INSTANTIATED)) { return -1; }   
//   elpclp890_reseed(&priv->elpclp890, 0, addin);
//   
//   /* two consecuitive reseeds */
//   if ((err = elpclp890_uninstantiate(&priv->elpclp890)) && (err != CRYPTO_NOT_INSTANTIATED)) { return -1; }   
//   if (elpclp890_instantiate(&priv->elpclp890, 128, 0, ps) < 0) { return -1; }
//   if (elpclp890_reseed(&priv->elpclp890, 0, addin) < 0) { return -1; }
//   elpclp890_reseed(&priv->elpclp890, 0, addin);
//
//   if (((err = elpclp890_uninstantiate(&priv->elpclp890)) < 0) && (err != CRYPTO_NOT_INSTANTIATED)) { return -1; }

   /* SDK doc example - DRBG Validation */
   if ((err = elpclp890_uninstantiate(&priv->elpclp890)) && (err != CRYPTO_NOT_INSTANTIATED)) { return -1; }

   if (elpclp890_set_nonce_mode(&priv->elpclp890, 1) < 0) { return -1; }

   out = (char *)kmalloc(64, GFP_KERNEL);
   str_to_384_bit(buf_seed1, tmp);
   if (elpclp890_get_entropy_input(&priv->elpclp890, tmp, 0) < 0) { return -1; }
   str_to_384_bit(buf_ps1, tmp);
   if (elpclp890_instantiate(&priv->elpclp890, 256, 1, tmp) < 0) { return -1; }
   str_to_384_bit(buf_seed2, tmp);
   if (elpclp890_get_entropy_input(&priv->elpclp890, tmp, 0) < 0) { return -1; }
   str_to_384_bit(buf_addin1, tmp);
   if (elpclp890_generate(&priv->elpclp890, out, 64, 256, 1, tmp) < 0) { return -1; }
   str_to_384_bit(buf_seed3, tmp);
   if (elpclp890_get_entropy_input(&priv->elpclp890, tmp, 0) < 0) { return -1; }
   str_to_384_bit(buf_addin2, tmp);
   if (elpclp890_generate(&priv->elpclp890, out, 64, 256, 1, tmp) < 0) { return -1; }
   memcpy(priv->rand_out, out, 64);

	return count;
}

static DEVICE_ATTR(epn,                   0444, show_epn,          NULL);
static DEVICE_ATTR(stepping,              0444, show_stepping,     NULL);
static DEVICE_ATTR(features,              0444, show_features,     NULL);
static DEVICE_ATTR(secure,                0644, show_secure,       store_secure);
static DEVICE_ATTR(nonce,                 0644, show_nonce,        store_nonce);
static DEVICE_ATTR(sec_strength,          0644, show_sec_strength, store_sec_strength);
      
static DEVICE_ATTR(mode_reg,              0644, show_mode_reg,     store_mode_reg);
static DEVICE_ATTR(smode_reg,             0644, show_smode_reg,    store_smode_reg);
static DEVICE_ATTR(alarm_reg,             0644, show_alarm_reg,    store_alarm_reg);
static DEVICE_ATTR(rand_reg,              0400, show_rand_reg,     NULL);
static DEVICE_ATTR(rand_out,              0400, show_rand_out,     NULL);
static DEVICE_ATTR(seed_reg,              0600, show_seed_reg,     store_seed_reg);
static DEVICE_ATTR(npa_data_reg,          0600, show_npa_data_reg, store_npa_data_reg);
static DEVICE_ATTR(ctrl_reg,              0644, show_ctrl_reg,     store_ctrl_reg);
static DEVICE_ATTR(istat_reg,             0644, show_istat_reg,    store_istat_reg);
static DEVICE_ATTR(stat_reg,              0444, show_stat_reg,     NULL);

static DEVICE_ATTR(ia_wdata_reg,          0600, show_ia_wdata_reg, store_ia_wdata_reg);
static DEVICE_ATTR(ia_rdata_reg,          0400, show_ia_rdata_reg, NULL);
static DEVICE_ATTR(ia_addr_reg,           0600, show_ia_addr_reg,  store_ia_addr_reg);
static DEVICE_ATTR(ia_cmd_reg,            0600, show_ia_cmd_reg,   store_ia_cmd_reg);
static DEVICE_ATTR(hw_state,              0400, show_hw_state,     NULL);

static DEVICE_ATTR(collect_ent,           0400, collect_entropy,   NULL);
static DEVICE_ATTR(nonce_seed_with_df,    0200, NULL,              cmd_nonce_seed_with_df);
static DEVICE_ATTR(nonce_seed_direct,     0200, NULL,              cmd_nonce_seed_direct);
static DEVICE_ATTR(instantiate,           0200, NULL,              cmd_instantiate);
static DEVICE_ATTR(uninstantiate,         0200, NULL,              cmd_uninstantiate);
static DEVICE_ATTR(reseed,                0200, NULL,              cmd_reseed);
static DEVICE_ATTR(generate,              0200, NULL,              cmd_generate);
static DEVICE_ATTR(kat,                   0200, NULL,              cmd_kat);

static DEVICE_ATTR(max_bits_per_req, 0644, show_max_bits_per_req, set_max_bits_per_req);
static DEVICE_ATTR(max_req_per_seed, 0644, show_max_req_per_seed, set_max_req_per_seed);

static DEVICE_ATTR(test_attr,             0200, NULL,              test_attr_func);

static const struct attribute_group elpclp890_attr_group = {
   .attrs = (struct attribute *[]) {
      &dev_attr_epn.attr,
      &dev_attr_stepping.attr,
      &dev_attr_features.attr,
      &dev_attr_secure.attr,
      &dev_attr_nonce.attr,
      &dev_attr_sec_strength.attr,

      &dev_attr_mode_reg.attr,
      &dev_attr_smode_reg.attr,
      &dev_attr_alarm_reg.attr,
      &dev_attr_rand_reg.attr,
      &dev_attr_rand_out.attr,
      &dev_attr_seed_reg.attr,
      &dev_attr_npa_data_reg.attr,
      &dev_attr_ctrl_reg.attr,
      &dev_attr_istat_reg.attr,
      &dev_attr_stat_reg.attr,

      &dev_attr_ia_wdata_reg.attr,
      &dev_attr_ia_rdata_reg.attr,
      &dev_attr_ia_addr_reg.attr,
      &dev_attr_ia_cmd_reg.attr,
      &dev_attr_hw_state.attr,
      
      &dev_attr_collect_ent.attr,
      &dev_attr_nonce_seed_with_df.attr,
      &dev_attr_nonce_seed_direct.attr,
      &dev_attr_instantiate.attr,
      &dev_attr_uninstantiate.attr,
      &dev_attr_reseed.attr,
      &dev_attr_generate.attr,
      &dev_attr_kat.attr,
      
      &dev_attr_max_bits_per_req.attr,
      &dev_attr_max_req_per_seed.attr,
      
      &dev_attr_test_attr.attr,
      NULL
   },
};

static int elpclp890_self_test(elpclp890_state *clp890)
{
   uint32_t seed[16], out[4], x;
//   Personalization String = 0   
   static const uint32_t exp128[4] = {0x5db79bb2, 0xc3a0df1e, 0x099482b6, 0xc319981e},
                         exp256[4] = {0x1f1a1441, 0xa0865ece, 0x9ff8d5b9, 0x3f78ace6};
   int ret;

   for (x = 0; x < 16; x++) seed[x] = 0x12345679 * (x + 1);
   
   DEBUG("Doing a self-test with security strength of 128\n");
   if ((ret = elpclp890_uninstantiate(clp890)) && (ret != CRYPTO_NOT_INSTANTIATED)) { goto ERR; }
   if ((ret = elpclp890_set_nonce_mode(clp890, 1)))                                 { goto ERR; }
   if ((ret = elpclp890_set_sec_strength(clp890, 128)))                             { goto ERR; }
   if ((ret = elpclp890_get_entropy_input(clp890, seed, 0)))                        { goto ERR; }         
   if ((ret = elpclp890_instantiate(clp890, 128, 0, NULL)))                         { goto ERR; }   
   if ((ret = elpclp890_generate(clp890, out, 16, 128, 0, NULL)))                   { goto ERR; }
   if (clp890->config.features.extra_ps_present) {
      DEBUG("skip KAT with extra_ps_present\n");
   } else {
      DEBUG("clp890: AES-128 Self-test output: ");
      for (x = 0; x < 4; x++) { DEBUG("0x%08lx ", (unsigned long)out[x]); }
      if (memcmp(out, exp128, sizeof exp128)) {
         ELPHW_PRINT("...  FAILED comparison\n");
         ret = -1;
         goto ERR;
      } else {
         DEBUG("...  PASSED\n");
      }
   }

   if (clp890->config.features.drbg_arch == AES256) {
      // test AES-256 mode
      DEBUG("Doing a self-test with security strength of 256\n");
      if ((ret = elpclp890_uninstantiate(clp890)) && (ret != CRYPTO_NOT_INSTANTIATED)) { goto ERR; }
      if ((ret = elpclp890_set_nonce_mode(clp890, 1)))                                 { goto ERR; }
      if ((ret = elpclp890_set_sec_strength(clp890, 256)))                             { goto ERR; }
      if ((ret = elpclp890_get_entropy_input(clp890, seed, 0)))                        { goto ERR; }
      if ((ret = elpclp890_instantiate(clp890, 256, 0, NULL)))                         { goto ERR; }
      if ((ret = elpclp890_generate(clp890, out, 16, 256, 0, NULL)))                   { goto ERR; }
      if (clp890->config.features.extra_ps_present) {
         DEBUG("skip KAT with extra_ps_present\n");
      } else {
         DEBUG("clp890: AES-256 Self-test output: ");
         for (x = 0; x < 4; x++) { DEBUG("0x%08lx ", (unsigned long)out[x]); }
         if (memcmp(out, exp256, sizeof exp256)) {
            ELPHW_PRINT("...  FAILED comparison\n");
            ret = -1;
            goto ERR;
         } else {
            DEBUG("...  PASSED\n");
         }
      }
   }
   
   /* back to the noise mode */
   if ((ret = elpclp890_set_nonce_mode(clp890, 0)))                                 { goto ERR; }
   
   if ((ret = elpclp890_zeroize(clp890))) { goto ERR; }
ERR:
   return ret;
}

static int elpclp890_driver_probe(struct platform_device *pdev)
{
   struct resource *cfg, *irq;
   elliptic_elpclp890_driver *data;
   int ret;
   struct hwrng *hwrng_driver_info = 0;
   uint32_t *base_addr;

   cfg = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

   if (!cfg || !irq) {
      ELPHW_PRINT("no memory or IRQ resource\n");
      return -ENOMEM;
   }

   DEBUG("=================================================================\n");
   DEBUG("elpclp890_probe: Device at %08lx(%08lx) of size %lu bytes\n", (unsigned long)cfg->start, (unsigned long)cfg->end, (unsigned long)resource_size(cfg));

   data = devm_kzalloc(&pdev->dev, sizeof(elliptic_elpclp890_driver), GFP_KERNEL);
   if (!data) {
      return -ENOMEM;
   }

   platform_set_drvdata(pdev, data);

   base_addr = pdu_linux_map_regs(&pdev->dev, cfg);
   if (IS_ERR(base_addr)) {
      dev_err(&pdev->dev, "unable to remap io mem\n");
      return PTR_ERR(base_addr);
   }

   if ( (ret = elpclp890_init(&data->elpclp890, (uint32_t*)base_addr)) != 0) {
      ELPHW_PRINT("CLP890 init failed (%d)\n", ret);
      devm_kfree(&pdev->dev, data);
      return ret;
   }
   
   /* if max_reads is not 0, change the max_req_per_seed according to max_reads */
   if (max_reads) {
      if ( (ret = elpclp890_set_reminder_max_req_per_seed(&data->elpclp890, max_reads)) != 0) {
         ELPHW_PRINT("CLP890 maximum request-per-seed setup failed (%d)\n", ret);
         devm_kfree(&pdev->dev, data);
         return ret;
      }
   }

   // issue quick self test   
   ret = elpclp890_self_test(&data->elpclp890);
   if (ret) {
      devm_kfree(&pdev->dev, data);
      return -ENOMEM;
   }

   // ready the device for use     
   ret = elpclp890_instantiate(&data->elpclp890, data->elpclp890.config.features.drbg_arch ? 256 : 128, 1, NULL);      
   if (ret) {
      ELPHW_PRINT("CLP890 instantiate failed (%d)\n", ret);
      devm_kfree(&pdev->dev, data);
      return -ENOMEM;
   }

   // at this point the device should be ready for a call to gen_random
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

   hwrng_driver_info->read = &elpclp890_hwrng_driver_read;
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

   ret = sysfs_create_group(&pdev->dev.kobj, &elpclp890_attr_group);
   if (ret < 0) {
      ELPHW_PRINT("unable to initialize sysfs group (error %d)\n", ret);
      hwrng_unregister(hwrng_driver_info);
      devm_kfree(&pdev->dev, (void *)hwrng_driver_info->name);
      devm_kfree(&pdev->dev, hwrng_driver_info);
      devm_kfree(&pdev->dev, data);
      return ret;
   }
   ELPHW_PRINT("ELP CLP890 registering HW_RANDOM\n");
   return 0;
}

static int elpclp890_driver_remove(struct platform_device *pdev)
{
   elliptic_elpclp890_driver *data = platform_get_drvdata(pdev);
   struct hwrng *hwrng_driver_info = (struct hwrng *)data->hwrng_drv;

   ELPHW_PRINT("ELP CLP890 unregistering from HW_RANDOM\n");
   hwrng_unregister(hwrng_driver_info);
   sysfs_remove_group(&pdev->dev.kobj, &elpclp890_attr_group);
   devm_kfree(&pdev->dev, (void *)hwrng_driver_info->name);
   devm_kfree(&pdev->dev, hwrng_driver_info);
   devm_kfree(&pdev->dev, data);
   return 0;
}

static struct platform_driver s_elpclp890_platform_driver_info = {
   .probe      = elpclp890_driver_probe,
   .remove     = elpclp890_driver_remove,
   .driver     = {
      .name = "clp890",
      .owner   = THIS_MODULE,
   },
};

static int __init elpclp890_platform_driver_start(void)
{
   return platform_driver_register(&s_elpclp890_platform_driver_info);
}

static void __exit elpclp890_platform_driver_end(void)
{
   platform_driver_unregister(&s_elpclp890_platform_driver_info);
}

module_init(elpclp890_platform_driver_start);
module_exit(elpclp890_platform_driver_end);

module_param(max_reads, ulong, 0);
MODULE_PARM_DESC(max_reads, "Max # of reads between reseeds (default is 128)");

EXPORT_SYMBOL(elpclp890_init);
EXPORT_SYMBOL(elpclp890_instantiate);
EXPORT_SYMBOL(elpclp890_uninstantiate);
EXPORT_SYMBOL(elpclp890_reseed);
EXPORT_SYMBOL(elpclp890_generate);
EXPORT_SYMBOL(elpclp890_wait_on_busy);
EXPORT_SYMBOL(elpclp890_wait_on_done);
EXPORT_SYMBOL(elpclp890_get_alarms);
EXPORT_SYMBOL(elpclp890_reset_counters);
EXPORT_SYMBOL(elpclp890_set_sec_strength);
EXPORT_SYMBOL(elpclp890_set_addin_present);
EXPORT_SYMBOL(elpclp890_set_pred_resist);
EXPORT_SYMBOL(elpclp890_set_secure_mode);
EXPORT_SYMBOL(elpclp890_set_nonce_mode);
EXPORT_SYMBOL(elpclp890_load_ps_addin);
EXPORT_SYMBOL(elpclp890_get_entropy_input);
EXPORT_SYMBOL(elpclp890_refresh_addin);
EXPORT_SYMBOL(elpclp890_gen_random);
EXPORT_SYMBOL(elpclp890_advance_state);
EXPORT_SYMBOL(elpclp890_kat);
EXPORT_SYMBOL(elpclp890_full_kat);
EXPORT_SYMBOL(elpclp890_zeroize);
EXPORT_SYMBOL(elpclp890_set_reminder_max_bits_per_req);
EXPORT_SYMBOL(elpclp890_set_reminder_max_req_per_seed);

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
