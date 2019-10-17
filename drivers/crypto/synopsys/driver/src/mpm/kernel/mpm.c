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
#include <linux/dma-mapping.h>
#include <linux/completion.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/param.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/sched.h>

#include "elpspaccdrv.h"
#include "elpmpm.h"

static int
   mpm_config_pdus=128,
   mpm_config_chains=16,
   mpm_config_keys=128,
   mpm_config_links=32,
   mpm_config_demand=16;

static mpm_device mpm[2];
static struct platform_device *mpm_pdev;

mpm_device *mpm_get_device(void)
{
   return &mpm[0];
}

struct platform_device *mpm_get_platform_device(void)
{
   return mpm_pdev;
}

mpm_device *mpm_get_device_by_num(int id)
{
   return &mpm[id&1];
}

// Bottom Half tasklet that gets triggered by users and IRQ to clear chains and enqueue the next chain
static void mpm_tasklet(unsigned long data);
static void mpm_tasklet2(unsigned long data);
DECLARE_TASKLET(mpm_tasklet_struct, mpm_tasklet, 0);
DECLARE_TASKLET(mpm_tasklet_struct2, mpm_tasklet2, 1);

static void mpm_tasklet(unsigned long data)
{
   mpm_clear_chains(&mpm[0]);
}

static void mpm_tasklet2(unsigned long data)
{
   mpm_clear_chains(&mpm[1]);
}

void mpm_kernel_schedule_tasklet(void)
{
   tasklet_schedule(&mpm_tasklet_struct);
}

void mpm_kernel_schedule_tasklet_by_num(int id)
{
   tasklet_schedule(id ? &mpm_tasklet_struct2 : &mpm_tasklet_struct);
}

/* a function to run callbacks in the IRQ handler */
static irqreturn_t mpm_irq_handler(int irq, void *dev)
{
  uint32_t irq_stat, d;

  irq_stat  = pdu_io_read32(mpm[0].regmap + MPM_IRQ_STAT);
  d         = 0;

#ifdef MPM_PERF_MON
  mpm[0].perf.active_counter = pdu_io_read32(mpm[0].regmap + MPM_STAT_CNT_ACTIVE);
  mpm[0].perf.spacc_counter  = pdu_io_read32(mpm[0].regmap + MPM_STAT_SPACC_ACTIVE);
#endif


  if (irq_stat & MPM_IRQ_STAT_DEMAND) {
     // handle DEMAND
     pdu_io_write32(mpm[0].regmap + MPM_IRQ_STAT, MPM_IRQ_STAT_DEMAND);
     mpm_process_int_demand(&mpm[0]);
     d = 1;
  }

  if (irq_stat & MPM_IRQ_STAT_EOL) {
     pdu_io_write32(mpm[0].regmap + MPM_IRQ_STAT, MPM_IRQ_STAT_EOL);
     mpm_process_int_eol(&mpm[0]);
     mpm_kernel_schedule_tasklet_by_num(0);
     d = 1;
  }

#ifdef MPM_PERF_MON
  mpm[0].perf.irq_counter = pdu_io_read32(mpm[0].regmap + MPM_STAT_IRQ_ASSERT);
#endif

  return d ? IRQ_HANDLED : IRQ_NONE;
}

// IRQ handler for 2nd MPM [if any]
static irqreturn_t mpm_irq_handler2(int irq, void *dev)
{
  uint32_t irq_stat, d;

  irq_stat  = pdu_io_read32(mpm[1].regmap + MPM_IRQ_STAT);
  d         = 0;

#ifdef MPM_PERF_MON
  mpm[1].perf.active_counter = pdu_io_read32(mpm[1].regmap + MPM_STAT_CNT_ACTIVE);
  mpm[1].perf.spacc_counter  = pdu_io_read32(mpm[1].regmap + MPM_STAT_SPACC_ACTIVE);
#endif


  if (irq_stat & MPM_IRQ_STAT_DEMAND) {
     // handle DEMAND
     pdu_io_write32(mpm[1].regmap + MPM_IRQ_STAT, MPM_IRQ_STAT_DEMAND);
     mpm_process_int_demand(&mpm[1]);
     d = 1;
  }

  if (irq_stat & MPM_IRQ_STAT_EOL) {
     pdu_io_write32(mpm[1].regmap + MPM_IRQ_STAT, MPM_IRQ_STAT_EOL);
     mpm_process_int_eol(&mpm[1]);
     mpm_kernel_schedule_tasklet_by_num(1);
     d = 1;
  }

#ifdef MPM_PERF_MON
  mpm[1].perf.irq_counter = pdu_io_read32(mpm[1].regmap + MPM_STAT_IRQ_ASSERT);
#endif

  return d ? IRQ_HANDLED : IRQ_NONE;
}

#ifdef MPM_BUILT_IN_TESTS


static void self_test_cb(void *mpm_dev, void *data, int pdu_idx, int key_idx, uint32_t status)
{
   pdubuf *pdu = &((mpm_device *)mpm_dev)->pdu.pdus[pdu_idx];
   printk(KERN_DEBUG "self_test_cb::%p, %08zx%08zx (%d:%d:%08zx)\n", mpm_dev, pdu[0][0x7C/4], pdu[0][0x78/4], pdu_idx, key_idx, status);
   if (status & PDU_DESC_STATUS_DONE) {
      complete(data);
   }
}

static int self_test(int id)
{
   static unsigned char key[16] = { 0x4b, 0x1f, 0xe4, 0x2e, 0xf3, 0x57, 0x46, 0xd1, 0x0f, 0xd6, 0xd6, 0xf0, 0x3c, 0xd6, 0x8e, 0xae },
                        iv[16]  = { 0x61, 0xd5, 0x72, 0x4c, 0x16, 0x8d, 0x7c, 0x6a, 0x06, 0x3b, 0xba, 0xb0, 0x53, 0x03, 0xc1, 0x9f },
                        pt[32]  = { 0x22, 0xa5, 0xcd, 0x15, 0xfc, 0x13, 0xe6, 0x0c, 0xe9, 0xbc, 0xfc, 0x26, 0x92, 0x8a, 0xd4, 0xf4,
                                    0x5f, 0x47, 0x40, 0x76, 0xd4, 0xbc, 0xe0, 0xda, 0xf7, 0x9a, 0x8a, 0x4b, 0x9d, 0x4b, 0xea, 0xbf },
                        ct[32]  = { 0xfa, 0x17, 0x9a, 0x06, 0xd3, 0x9c, 0x83, 0xf3, 0x2b, 0xfc, 0x49, 0x43, 0x97, 0x59, 0xe1, 0x29,
                                    0x57, 0x97, 0x3b, 0xab, 0xb1, 0xc7, 0x81, 0xd2, 0x00, 0x11, 0x11, 0x48, 0x5f, 0x63, 0xc6, 0x24 };
  unsigned char  *PT, *CT;
  PDU_DMA_ADDR_T ptphys, ctphys;
  pdu_ddt       src, dst;
  int keyidx, pduidx, ctx[1], err, ddts;
  uint32_t CTRL;
  struct completion self_test_comp;

  PT = CT = NULL;
  ctx[0]  = -1;
  keyidx  = pduidx = -1;
  ddts    = 0;
  err     = -1;

  // alloc dma mem for contents
  PT = pdu_dma_alloc(32, &ptphys);
  CT = pdu_dma_alloc(32, &ctphys);
  if (!PT || !CT) { printk(KERN_DEBUG "MPM.%d: self_test::Out of memory\n", id); goto ERR; }
  memcpy(PT, pt, 32);

  // allocate context to spacc
  err = mpm_req_spacc_ctx(&mpm[id], 1, ctx);
  if (err) { printk(KERN_DEBUG "MPM.%d: self_test::Error allocating a context to the MPM\n", id); goto ERR; }

  // allocate pdu and key
  keyidx = mpm_alloc_key(&mpm[id]);
  if (keyidx < 0) { printk(KERN_DEBUG "MPM.%d: self_test::Error allocating key\n", id); err = -1; goto ERR; }

  // set key
  mpm_set_key(&mpm[id], keyidx, 16, 0, 0, 0, 0, key, NULL);

  // setup ddts
  pdu_ddt_init(&src, 1);
  pdu_ddt_add(&src, ptphys, 32);
  pdu_ddt_init(&dst, 1);
  pdu_ddt_add(&dst, ctphys, 32);
  ddts = 1;

  // insert job
  CTRL = MPM_SET_CTRL(&mpm[id], C_AES, H_NULL, CM_CTR, HM_RAW, 1, 0, 0, 0, 0);

  pduidx = mpm_alloc_pdu(&mpm[id]);
  if (pduidx < 0) { printk(KERN_DEBUG "MPM.%d: self_test::Error allocating PDU\n", id); err = -1; goto ERR; }

  err = mpm_insert_pdu(&mpm[id], pduidx, keyidx, 0, self_test_cb, &self_test_comp, &src, &dst, 0, 0, 32, 0, 0, 0, 0, CTRL, iv, NULL);
  if (err < 0) { printk(KERN_DEBUG "MPM.%d: self_test::Error inserting PDU...\n", id); goto ERR; }

  // enqueue chain and fire tasklet to get ball rolling
  init_completion(&self_test_comp);
  mpm_enqueue_chain(&mpm[id], NULL, NULL);
  mpm_kernel_schedule_tasklet_by_num(id);
  wait_for_completion_interruptible(&self_test_comp);

  // job should be done now
  if (memcmp(ct, CT, 32)) {
    int x;
    for (x = 0; x < 32; x++) {
       printk(KERN_DEBUG "CT[%2d] == %02x\n", x, CT[x]);
    }

    printk(KERN_DEBUG "MPM.%d self_test::ciphertext does not match\n", id);
    err = -1;
    goto ERR;
  }
  printk(KERN_DEBUG "MPM.%d: self_test::[PASSED]\n", id);

  err = 0;
ERR:
  mpm_free_spacc_ctx(&mpm[id], 1, ctx);
  if (ddts) {
     pdu_ddt_free(&src);
     pdu_ddt_free(&dst);
  }
  if (PT) pdu_dma_free(32, PT, ptphys);
  if (CT) pdu_dma_free(32, CT, ctphys);
  if (keyidx > -1) {
     mpm_free_key(&mpm[id], keyidx);
  }
  return err;
}

#endif

static int spaccmpm_probe(struct platform_device *pdev)
{
   pdu_info *info = pdev->dev.platform_data;
   void *baseaddr;
   struct resource *res, *irq;
   spacc_device *spacc;
   int err, id;

   mpm_pdev = pdev;

   id = pdev->id;
   if (id < 0 || id >= ARRAY_SIZE(mpm)) {
      dev_err(&pdev->dev, "id out of bounds\n");
      return -ENODEV;
   }

   // sanity check params
   if (mpm_config_pdus < (mpm_config_chains * mpm_config_links)) {
     printk("MPM: Warning PDU desc below #chains * #links (could be sub-optimal) \n");
   }
   if (mpm_config_keys > mpm_config_pdus) {
     printk("MPM: Warning there are more KEY buffers than PDU descriptors\n");
   }
   if (mpm_config_links < 1) {
     printk("MPM: There must be at least one link per chain\n");
     return -1;
   }
   if (mpm_config_chains < 1) {
     printk("MPM: There must be at least one chain\n");
     return -1;
   }
   if (mpm_config_chains < 4) {
     printk("MPM: Warning there are fewer than 4 chains requested, performance may be sub-optimal\n");
   }

   printk(KERN_DEBUG "MPM: Configuring with %d chains of %d links (%d total) with %d PDU descriptors and %d KEY buffers\n", mpm_config_chains, mpm_config_links, mpm_config_chains * mpm_config_links, mpm_config_pdus, mpm_config_keys);


   irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
   res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   if (!res || !irq) {
      return -EINVAL;
   }

   dev_info(&pdev->dev, "device at %pR\n", res);

   baseaddr = pdu_linux_map_regs(&pdev->dev, res);
   if (IS_ERR(baseaddr)) {
      dev_err(&pdev->dev, "unable to map iomem\n");
      return PTR_ERR(baseaddr);
   }

   /* get the associated spacc */
   dev_info(&pdev->dev, "attaching to SPAcc EPN %x\n", info->spacc_version.project);
   spacc = get_spacc_device_by_epn(info->spacc_version.project, 0);
   if (spacc == NULL) {
      return -ENODEV;
   }

   if (devm_request_irq(&pdev->dev, irq->start, id?mpm_irq_handler2:mpm_irq_handler, IRQF_SHARED, dev_name(&pdev->dev), &pdev->dev)) {
      dev_err(&pdev->dev, "failed to request IRQ\n");
      return -EBUSY;
   }

   err = mpm_init(&mpm[id], baseaddr, spacc, mpm_config_chains, mpm_config_pdus, mpm_config_keys, mpm_config_links, mpm_config_demand);
   if (err != CRYPTO_OK) {
      return -1;
   }
   dev_info(&pdev->dev, "mpm_init::Requires %zu bytes for PDU and %zu bytes for keys\n", mpm[id].pdus_mem_req, mpm[id].keys_mem_req);
   mpm[id].config.id = id;

   // after we call mpm_init we need to allocate a pools
   mpm[id].pdu.pdus = pdu_dma_alloc(mpm[id].pdus_mem_req, &mpm[id].pdu.pdus_phys);
   if (!mpm[id].pdu.pdus) {
      printk(KERN_DEBUG "Cannot allocate PDUs pool\n");
      mpm_deinit(&mpm[id]);
      return -1;
   }
   mpm[id].key.keys = pdu_dma_alloc(mpm[id].keys_mem_req, &mpm[id].key.keys_phys);
   if (!mpm[id].key.keys) {
      printk(KERN_DEBUG "Cannot allocate KEYs pool\n");
      pdu_dma_free(mpm[id].pdus_mem_req, mpm[id].pdu.pdus, mpm[id].pdu.pdus_phys);
      mpm_deinit(&mpm[id]);
      return -1;
   }

   // enable MPM ints
   pdu_io_write32(baseaddr+MPM_IRQ_EN, MPM_IRQ_EN_EOL|MPM_IRQ_EN_DEMAND|MPM_IRQ_EN_GLBL);

#ifdef MPM_BUILT_IN_TESTS
{
err = self_test(id);
return err;
}
#endif

   return 0;
}

static int spaccmpm_remove(struct platform_device *pdev)
{
   int id;
   id = pdev->id;
   printk(KERN_DEBUG "mpm_mod_exit::Freeing resources for MPM.%d\n", id);

   // fini the SDK then free the SA pool
   pdu_io_write32(mpm[id].regmap + MPM_IRQ_EN, 0);
   pdu_dma_free(mpm[id].pdus_mem_req, mpm[id].pdu.pdus, mpm[id].pdu.pdus_phys);
   pdu_dma_free(mpm[id].keys_mem_req, mpm[id].key.keys, mpm[id].key.keys_phys);
   mpm_deinit(&mpm[id]);

   return 0;
}

static struct platform_driver spaccmpm_driver = {
   .probe  = spaccmpm_probe,
   .remove = spaccmpm_remove,
   .driver = {
      .name  = "spacc-mpm",
      .owner = THIS_MODULE
   },
};


static int __init mpm_mod_init (void)
{
   memset(&mpm, 0, sizeof(mpm));
   return platform_driver_register(&spaccmpm_driver);
}

static void __exit mpm_mod_exit (void)
{
   platform_driver_unregister(&spaccmpm_driver);
}

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init (mpm_mod_init);
module_exit (mpm_mod_exit);

// kernel
EXPORT_SYMBOL (mpm_get_device);
EXPORT_SYMBOL (mpm_get_platform_device);
EXPORT_SYMBOL (mpm_get_device_by_num);
EXPORT_SYMBOL (mpm_kernel_schedule_tasklet);
EXPORT_SYMBOL (mpm_kernel_schedule_tasklet_by_num);

EXPORT_SYMBOL (mpm_req_spacc_ctx);
EXPORT_SYMBOL (mpm_free_spacc_ctx);
EXPORT_SYMBOL (mpm_alloc_pdu);
EXPORT_SYMBOL (mpm_alloc_key);
EXPORT_SYMBOL (mpm_free_key);
EXPORT_SYMBOL (mpm_set_key);
EXPORT_SYMBOL (mpm_insert_pdu);
EXPORT_SYMBOL (mpm_enqueue_chain);
EXPORT_SYMBOL (mpm_stats);
EXPORT_SYMBOL (mpm_clear_chains);


module_param (mpm_config_pdus, int, 0);
MODULE_PARM_DESC (mpm_config_pdus, "The number of PDU buffers (max number of jobs in flight)");
module_param (mpm_config_chains, int, 0);
MODULE_PARM_DESC (mpm_config_chains, "The number of PDU chains");
module_param (mpm_config_keys, int, 0);
MODULE_PARM_DESC (mpm_config_keys, "The number of KEY buffers (no more than PDUs)");
module_param (mpm_config_links, int, 0);
MODULE_PARM_DESC (mpm_config_links, "The number of PDUs per chain");
module_param(mpm_config_demand, int, 0);
MODULE_PARM_DESC (mpm_config_demand, "The number of parallel on demand jobs");

