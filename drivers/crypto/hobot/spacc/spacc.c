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
 * Copyright (c) 2011-2018 Synopsys, Inc. and/or its affiliates.
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
#include <linux/uaccess.h>
#include <asm/param.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/miscdevice.h>

#include "elppdu.h"
#include "elpspacc.h"
#include "elpspaccdrv.h"

static unsigned long oldtimer = 100000, timer = 100000; // 1ms @45MHz
static bool no_latency;

struct platform_device * get_spacc_platdev_by_epn(uint32_t epn, uint32_t virt)
{
   char name[256];
   struct device *dev;

   snprintf(name, sizeof(name), "spacc.%d", (epn << 16) | virt);
   dev = bus_find_device_by_name(&platform_bus_type, NULL, name);
   if (!dev) {
      printk(KERN_ERR "failed to find device for %s\n", name);
      return NULL;
   }
   return to_platform_device(dev);
}
EXPORT_SYMBOL(get_spacc_platdev_by_epn);

/* This is used by RE and KEP to get the spacc device */
spacc_device * get_spacc_device_by_epn(uint32_t epn, uint32_t virt)
{
   struct platform_device *plat;
   struct spacc_priv *priv;

   plat = get_spacc_platdev_by_epn(epn, virt);
   if (!plat)
      return NULL;

   priv = platform_get_drvdata(plat);
   return &priv->spacc;
}
EXPORT_SYMBOL(get_spacc_device_by_epn);

/* a function to run callbacks in the IRQ handler */
static irqreturn_t spacc_irq_handler(int irq, void *dev)
{
   struct spacc_priv *priv = platform_get_drvdata(to_platform_device(dev));
   spacc_device *spacc = &priv->spacc;

   if (oldtimer != timer) {
      spacc_set_wd_count(&priv->spacc, priv->spacc.config.wd_timer = timer);
      printk("spacc::Changed timer from %lu to %lu\n", oldtimer, timer);
      oldtimer = timer;
   }


/* check irq flags and process as required */
   if (!spacc_process_irq(spacc)) {
      return IRQ_NONE;
   }
   return IRQ_HANDLED;
}

/* callback function to initialize tasklet running */
static void spacc_stat_process(spacc_device *spacc)
{
   struct spacc_priv *priv = container_of(spacc, struct spacc_priv, spacc);

   /* run tasklet to pop jobs off fifo */
   tasklet_schedule(&priv->pop_jobs);
}

static void spacc_cmd_process(spacc_device *spacc, int x)
{
   struct spacc_priv *priv = container_of(spacc, struct spacc_priv, spacc);

   /* run tasklet to pop jobs off fifo */
   tasklet_schedule(&priv->pop_jobs);

}

static void spacc_pop_jobs (unsigned long data)
{
   struct spacc_priv * priv =  (struct spacc_priv *)data;
   spacc_device *spacc = &priv->spacc;
   int num;

   // decrement the WD CNT here since now we're actually going to respond to the IRQ completely
   if (spacc->wdcnt) {
      --(spacc->wdcnt);
   }

   spacc_pop_packets(spacc, &num);
}

#define HW_ENTRY(x) { x, #x }
static const struct { unsigned addr; char *name; } reg_names[] = {
   HW_ENTRY(SPACC_REG_IRQ_EN),
   HW_ENTRY(SPACC_REG_IRQ_STAT),
   HW_ENTRY(SPACC_REG_IRQ_CTRL),
   HW_ENTRY(SPACC_REG_FIFO_STAT),
   HW_ENTRY(SPACC_REG_SDMA_BRST_SZ),
   HW_ENTRY(SPACC_REG_HSM_CMD_REQ),
   HW_ENTRY(SPACC_REG_HSM_CMD_GNT),
   HW_ENTRY(SPACC_REG_SRC_PTR),
   HW_ENTRY(SPACC_REG_DST_PTR),
   HW_ENTRY(SPACC_REG_OFFSET),
   HW_ENTRY(SPACC_REG_PRE_AAD_LEN),
   HW_ENTRY(SPACC_REG_POST_AAD_LEN),
   HW_ENTRY(SPACC_REG_PROC_LEN),
   HW_ENTRY(SPACC_REG_ICV_LEN),
   HW_ENTRY(SPACC_REG_ICV_OFFSET),
   HW_ENTRY(SPACC_REG_IV_OFFSET),
   HW_ENTRY(SPACC_REG_SW_CTRL),
   HW_ENTRY(SPACC_REG_AUX_INFO),
   HW_ENTRY(SPACC_REG_CTRL),
   HW_ENTRY(SPACC_REG_STAT_POP),
   HW_ENTRY(SPACC_REG_STATUS),
   HW_ENTRY(SPACC_REG_STAT_WD_CTRL),
   HW_ENTRY(SPACC_REG_KEY_SZ),
   HW_ENTRY(SPACC_REG_VIRTUAL_RQST),
   HW_ENTRY(SPACC_REG_VIRTUAL_ALLOC),
   HW_ENTRY(SPACC_REG_VIRTUAL_PRIO),
   HW_ENTRY(SPACC_REG_VIRTUAL_RC4_KEY_RQST),
   HW_ENTRY(SPACC_REG_VIRTUAL_RC4_KEY_GNT),
   HW_ENTRY(SPACC_REG_ID),
   HW_ENTRY(SPACC_REG_CONFIG),
   HW_ENTRY(SPACC_REG_CONFIG2),
   HW_ENTRY(SPACC_REG_HSM_VERSION),
   HW_ENTRY(SPACC_REG_SECURE_CTRL),
   HW_ENTRY(SPACC_REG_SECURE_RELEASE),
   HW_ENTRY(SPACC_REG_SK_LOAD),
   HW_ENTRY(SPACC_REG_SK_STAT),
   HW_ENTRY(SPACC_REG_SK_KEY),
   HW_ENTRY(SPACC_REG_HSM_CTX_CMD),
   HW_ENTRY(SPACC_REG_HSM_CTX_STAT),

   { 0, NULL },
};
#undef HW_ENTRY

static uint32_t reg_epn = 0x0, reg_virt = 0;

static ssize_t show_reg(struct device *dev, struct device_attribute *devattr, char *buf)
{
   char *name, out[128];
   unsigned x, reg_addr;
   spacc_device *spacc = get_spacc_device_by_epn(reg_epn, reg_virt);

   if (!spacc) {
      return sprintf(buf, "Could not find SPAcc device (EPN=%lx,%lu), please write to this file first in the form <epn,virt>\n",
                     (unsigned long)reg_epn, (unsigned long)reg_virt);
   }

   buf[0] = out[0] = 0;
   for (reg_addr = 0; reg_addr < 0x300; reg_addr += 4) {
      name = NULL;
      for (x = 0; reg_names[x].name != NULL; x++) {
         if (reg_names[x].addr == reg_addr) {
            name = reg_names[x].name;
            break;
         }
      }
      if (name == NULL) { continue; }
      sprintf(out, "%-35s = %08lx\n", name, (unsigned long)pdu_io_read32(spacc->regmap + reg_addr));
      strcat(buf, out);
   }
   return strlen(buf);
}

static ssize_t store_reg(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
   unsigned long x, y;

   if (sscanf(buf, "%lx,%lu", &x, &y) != 2)
      return -EINVAL;

   reg_epn = x;
   reg_virt = y;
   return count;
}

static DEVICE_ATTR(reg,                0600, show_reg, store_reg);
static const struct attribute_group spacc_attr_group = {
   .attrs = (struct attribute *[]) {
      &dev_attr_reg.attr,
      NULL
   }
};


static int __devinit spacc_probe(struct platform_device *pdev)
{
   void *baseaddr;
   struct resource *mem, *irq;
   int x, err, oldmode;
   struct spacc_priv   *priv;
   pdu_info     info;

   dev_info(&pdev->dev, "probe called!\n");

   /* Initialize DDT DMA pools based on this device's resources */
   if (pdu_mem_init(&pdev->dev)) {
      dev_err(&pdev->dev, "Could not initialize DMA pools\n");
      return -ENOMEM;
   }

   mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
   if (!mem || !irq) {
      dev_err(&pdev->dev, "no memory/irq resource for spacc\n");
      return -ENXIO;
   }

   priv = devm_kzalloc(&pdev->dev, sizeof *priv, GFP_KERNEL);
   if (!priv) {
      dev_err(&pdev->dev, "no memory for spacc private data\n");
      return -ENOMEM;
   }

   printk("spacc_probe: Device at %pR\n", mem);
   baseaddr = pdu_linux_map_regs(&pdev->dev, mem);
   if (IS_ERR(baseaddr)) {
      dev_err(&pdev->dev, "unable to map iomem\n");
      return PTR_ERR(baseaddr);
   }

   x = pdev->id;
   dev_info(&pdev->dev, "EPN %04X : virt [%d] \n", (x >> 16) & 0xFFFF, x & 0xF);

   pdu_get_version(baseaddr, &info);
   if (pdev->dev.platform_data) {
      pdu_info *parent_info = pdev->dev.platform_data;
      memcpy(&info.pdu_config, &parent_info->pdu_config, sizeof info.pdu_config);
   }

   err = spacc_init (baseaddr, &priv->spacc, &info);
   if (err != CRYPTO_OK) {
      printk("spacc_probe::Failed to initialize device %d...\n", x);
      return -ENXIO;
   }

   err = sysfs_create_group(&pdev->dev.kobj, &spacc_attr_group);
   if (err) {
      spacc_fini(&priv->spacc);
      return -1;
   }


   spin_lock_init(&priv->hw_lock);
   spacc_irq_glbl_disable (&priv->spacc);
   tasklet_init(&priv->pop_jobs, spacc_pop_jobs, (unsigned long)priv);
   platform_set_drvdata(pdev, priv);

   /* Determine configured maximum message length. */
   priv->max_msg_len = priv->spacc.config.max_msg_size;

   if (devm_request_irq(&pdev->dev, irq->start, spacc_irq_handler, IRQF_SHARED, dev_name(&pdev->dev), &pdev->dev)) {
      dev_err(&pdev->dev, "failed to request IRQ\n");
      return -EBUSY;
   }

   /* Perform autodetect in no_latency=1 mode */
      priv->spacc.irq_cb_stat = spacc_stat_process;
      priv->spacc.irq_cb_cmdx = spacc_cmd_process;
      oldmode = priv->spacc.op_mode;
      priv->spacc.op_mode     = SPACC_OP_MODE_IRQ;

      spacc_irq_stat_enable (&priv->spacc, 1);
      spacc_irq_cmdx_enable(&priv->spacc, 0, 1);
      spacc_irq_stat_wd_disable (&priv->spacc);
      spacc_irq_glbl_enable (&priv->spacc);

#ifndef MAKEAVECTOR
      spacc_autodetect(&priv->spacc);
#endif
      priv->spacc.op_mode = oldmode;

   /* register irq callback function */
   if (no_latency) {
      // used to set lower latency mode on newer SPAcc device v4.11 and up
      // set above during autodetect
      priv->spacc.op_mode     = SPACC_OP_MODE_IRQ;
      printk("spacc:: Using low latency IRQ mode\n");
   } else {
      if (priv->spacc.op_mode == SPACC_OP_MODE_IRQ) {
         priv->spacc.irq_cb_stat = spacc_stat_process;
         priv->spacc.irq_cb_cmdx = spacc_cmd_process;

         spacc_irq_stat_enable (&priv->spacc, 1);
         spacc_irq_cmdx_enable(&priv->spacc, 0, 1);
         spacc_irq_glbl_enable (&priv->spacc);
      } else {
         priv->spacc.irq_cb_stat    = spacc_stat_process;
         priv->spacc.irq_cb_stat_wd = spacc_stat_process;

         spacc_irq_stat_enable (&priv->spacc, priv->spacc.config.ideal_stat_level);
         spacc_irq_cmdx_disable(&priv->spacc, 0);
         spacc_irq_stat_wd_enable (&priv->spacc);
         spacc_irq_glbl_enable (&priv->spacc);

         /* enable the wd */
         spacc_set_wd_count(&priv->spacc, priv->spacc.config.wd_timer = timer);
      }
   }

   // unlock normal
   if (priv->spacc.config.is_hsm_shared && priv->spacc.config.is_secure_port) {
      uint32_t t;
      t = pdu_io_read32(baseaddr + SPACC_REG_SECURE_CTRL);
      t &= ~(1UL<<31);
      pdu_io_write32(baseaddr + SPACC_REG_SECURE_CTRL, t);
   }

   // unlock device by default
   pdu_io_write32(baseaddr + SPACC_REG_SECURE_CTRL, 0);

   return err;
}

static int __devexit spacc_remove(struct platform_device *pdev)
{
   spacc_device *spacc;

   // free test vector memory
   spacc = &((struct spacc_priv *)platform_get_drvdata(pdev))->spacc;
   spacc_fini(spacc);
   sysfs_remove_group(&pdev->dev.kobj, &spacc_attr_group);

   pdu_mem_deinit(&pdev->dev);

   /* devm functions do proper cleanup */
   dev_info(&pdev->dev, "removed!\n");

   return 0;
}

#include "elpspacc_irq.h"

static long spacc_kernel_irq_ioctl (struct file *fp, unsigned int cmd, unsigned long arg_)
{
   elpspacc_irq_ioctl io;
   spacc_device *spacc;
   void __user *arg = (void __user *) arg_;
   unsigned long flags;

   if (unlikely (copy_from_user (&io, arg, sizeof (io)))) {
      printk ("spacc_irq_ioctl::Cannot copy ioctl buffer from user\n");
      return -EFAULT;
   }

   spacc = get_spacc_device_by_epn (io.spacc_epn, io.spacc_virt);
   if (!spacc) {
      printk("spacc_irq_ioctl::Cannot find SPAcc %lx/%lx\n",
             (unsigned long)io.spacc_epn, (unsigned long)io.spacc_virt);
      return -EIO;
   }

   if (io.command == SPACC_IRQ_CMD_SET) {
      // lock spacc
      PDU_LOCK(&spacc->lock, flags);

      // first disable everything
      spacc_irq_stat_disable(spacc);
      spacc_irq_cmdx_disable(spacc, 0);
      spacc_irq_stat_wd_disable (spacc);
      spacc_irq_glbl_disable (spacc);

      if (io.irq_mode == SPACC_IRQ_MODE_WD) {
         // set WD mode
         spacc->irq_cb_stat    = spacc_stat_process;
         spacc->irq_cb_stat_wd = spacc_stat_process;
         spacc->irq_cb_cmdx    = NULL;

         spacc_irq_stat_enable (spacc, io.stat_value ? io.stat_value : spacc->config.ideal_stat_level);
         spacc_irq_stat_wd_enable (spacc);
         spacc_set_wd_count(spacc, io.wd_value ? io.wd_value : spacc->config.wd_timer);
         spacc->op_mode = SPACC_OP_MODE_WD;
      } else {
         // set STEP mode
         spacc->irq_cb_stat    = spacc_stat_process;
         spacc->irq_cb_cmdx    = spacc_cmd_process;
         spacc->irq_cb_stat_wd = NULL;

         spacc_irq_stat_enable(spacc, io.stat_value ? io.stat_value : 1);
         spacc_irq_cmdx_enable(spacc, 0, io.cmd_value ? io.cmd_value : 1);
         spacc->op_mode = SPACC_OP_MODE_IRQ;
      }
      spacc_irq_glbl_enable (spacc);
      PDU_UNLOCK(&spacc->lock, flags);
   }

   return 0;
}

static struct file_operations spacc_kernel_irq_fops = {
   .owner = THIS_MODULE,
   .unlocked_ioctl = spacc_kernel_irq_ioctl,
};

static struct miscdevice spaccirq_device = {
   .minor = MISC_DYNAMIC_MINOR,
   .name = "spaccirq",
   .fops = &spacc_kernel_irq_fops,
};

static struct platform_driver spacc_driver = {
   .probe  = spacc_probe,
   .remove = __devexit_p(spacc_remove),
   .driver = {
      .name  = "spacc",
      .owner = THIS_MODULE
   },
};

static int __init spacc_mod_init (void)
{
   int err;

   err = misc_register (&spaccirq_device);
   if (err) {
      return err;
   }

   err = platform_driver_register(&spacc_driver);
   if (err) {
      misc_deregister (&spaccirq_device);
   }
   return err;
}

static void __exit spacc_mod_exit (void)
{
   misc_deregister (&spaccirq_device);
   platform_driver_unregister(&spacc_driver);
}

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init (spacc_mod_init);
module_exit (spacc_mod_exit);

// export wrapped library functions
EXPORT_SYMBOL (spacc_open);
EXPORT_SYMBOL (spacc_clone_handle);
EXPORT_SYMBOL (spacc_close);
//EXPORT_SYMBOL (spacc_status);
EXPORT_SYMBOL (spacc_write_context);
EXPORT_SYMBOL (spacc_read_context);
EXPORT_SYMBOL (spacc_write_rc4_context);
EXPORT_SYMBOL (spacc_read_rc4_context);
EXPORT_SYMBOL (spacc_error_msg);
EXPORT_SYMBOL (spacc_set_operation);
EXPORT_SYMBOL (spacc_set_key_exp);
EXPORT_SYMBOL (spacc_set_auxinfo);
EXPORT_SYMBOL (spacc_packet_enqueue_ddt);
EXPORT_SYMBOL (spacc_packet_dequeue);
EXPORT_SYMBOL (spacc_virtual_set_weight);
EXPORT_SYMBOL (spacc_virtual_request_rc4);
EXPORT_SYMBOL (spacc_pop_packets);
EXPORT_SYMBOL (spacc_load_skp);
EXPORT_SYMBOL (spacc_dump_ctx);
EXPORT_SYMBOL (spacc_set_wd_count);

EXPORT_SYMBOL(spacc_irq_cmdx_enable);
EXPORT_SYMBOL(spacc_irq_cmdx_disable);
EXPORT_SYMBOL(spacc_irq_stat_enable);
EXPORT_SYMBOL(spacc_irq_stat_disable);
EXPORT_SYMBOL(spacc_irq_stat_wd_enable);
EXPORT_SYMBOL(spacc_irq_stat_wd_disable);
EXPORT_SYMBOL(spacc_irq_rc4_dma_enable);
EXPORT_SYMBOL(spacc_irq_rc4_dma_disable);
EXPORT_SYMBOL(spacc_irq_glbl_enable);
EXPORT_SYMBOL(spacc_irq_glbl_disable);
EXPORT_SYMBOL(spacc_process_irq);

EXPORT_SYMBOL(spacc_compute_xcbc_key);
EXPORT_SYMBOL(spacc_isenabled);

// used by RE/KEP
EXPORT_SYMBOL (spacc_ctx_request);
EXPORT_SYMBOL (spacc_ctx_release);

int spacc_endian;
module_param(spacc_endian, int, 0);
MODULE_PARM_DESC(spacc_endian, "Endianess of data transfers (0==little)");
EXPORT_SYMBOL(spacc_endian);

module_param(timer, ulong, 0600);
MODULE_PARM_DESC(timer, "Watchdog timer value (default==0xAFC8 which is 1ms@45MHz)");

module_param(no_latency, bool, 0600);
MODULE_PARM_DESC(no_latency, "Set to 1 to have a low latency IRQ mechanism");
