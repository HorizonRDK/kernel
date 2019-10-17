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

#define DRVNAME "saspa"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/bitmap.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <asm/byteorder.h>

#include "elpsaspa_hw.h"
#include "elpsaspa.h"

#include "saspa_drv.h"

#define ALLOC_GRANULARITY 32

struct saspa_priv {
   struct platform_device *aes, *des, *rc4, *hmac, *snow, *trng, *pka;
   struct semaphore core_busy;

   u32 __iomem *regs, *mem;

   spinlock_t glbl_lock, mem_lock;
   unsigned long mem_size, mem_pages, ctx_tail, data_head;

   /* Allocation bitmap */
   unsigned long mem_alloc[];
};

/*
 * Overview of the SASPA allocator.
 *
 * Context allocations are generally small and longer-lived than data
 * allocations, and we need to keep (ideally large) contiguous memory regions
 * around for data allocations.  To help achieve this, we allocate context
 * memory from one end, and data memory from the other, to avoid the
 * possibility of fragmenting the ephemeral data area with small, persistent
 * context allocations.  This is done using a first-fit approach.
 *
 * So after some allocations, the memory might look something like this:
 *
 *   | ctx | free | ctx | large free space | data | free | data |
 *                      ^                  ^
 *                      A                  B
 * The points marked A and B indicate the end of any active context allocations
 * (called the "context tail" hereafter), and the beginning of any active data
 * allocations (called the "data head"), respectively.  By definition, all
 * space between those points is unallocated and the context tail will always
 * have a lower address than the data head.  Initially, the space between these
 * two points is the entire buffer.
 */

/*
 * Allocate context memory.  Returns a handle which can be used to read/write
 * the SASPA's shared memory, or a negative value on failure.
 */
long saspa_alloc_ctx(struct device *dev, size_t size)
{
   struct saspa_priv *priv = dev_get_drvdata(dev);
   unsigned long pages, n;
   long ret = -ENOMEM;

   if (size == 0)
      return 0;

   pages = DIV_ROUND_UP(size, ALLOC_GRANULARITY);
   if (pages > priv->mem_pages)
      return -ENOMEM;

   spin_lock(&priv->mem_lock);

   n = bitmap_find_next_zero_area(priv->mem_alloc, priv->data_head,
                                  0, pages, 0);
   if (n >= priv->data_head)
      goto out_unlock;
   bitmap_set(priv->mem_alloc, n, pages);

   ret = n;

   if (n + pages >= priv->ctx_tail)
      priv->ctx_tail = n + pages;
   BUG_ON(priv->ctx_tail > priv->data_head);

out_unlock:
   spin_unlock(&priv->mem_lock);
   return ret;
}

/*
 * Allocate data memory.  Returns a handle which can be used to read/write the
 * SASPA's shared memory, or a negative value on failure.
 */
long saspa_alloc_data(struct device *dev, size_t size)
{
   struct saspa_priv *priv = dev_get_drvdata(dev);
   unsigned long pages, n;
   long ret = -ENOMEM;

   if (size == 0)
      return 0;

   pages = DIV_ROUND_UP(size, ALLOC_GRANULARITY);
   if (pages > priv->mem_pages)
      return -ENOMEM;

   spin_lock(&priv->mem_lock);

   /* First attempt to satisfy the allocation after the data head. */
   n = bitmap_find_next_zero_area(priv->mem_alloc, priv->mem_pages,
                                  priv->data_head, pages, 0);
   if (n < priv->mem_pages) {
      ret = n;
      bitmap_set(priv->mem_alloc, ret, pages);
      goto out;
   }

   /* Otherwise try to fit it between the context tail and data head. */
   if (priv->data_head - priv->ctx_tail >= pages) {
      ret = priv->data_head - pages;
      bitmap_set(priv->mem_alloc, ret, pages);
      priv->data_head = ret;

      BUG_ON(priv->data_head < priv->ctx_tail);
      goto out;
   }

out:
   spin_unlock(&priv->mem_lock);
   return ret;
}

static void
saspa_free_ctx(struct device *dev, unsigned long n, unsigned long pages)
{
   struct saspa_priv *priv = dev_get_drvdata(dev);

   bitmap_clear(priv->mem_alloc, n, pages);

   /* Move the context tail back if we freed the highest context block. */
   if (n + pages == priv->ctx_tail) {
      n = find_last_bit(priv->mem_alloc, priv->ctx_tail) + 1;
      if (n > priv->ctx_tail) {
         /* There are no active context allocations at all. */
         n = 0;
      }
      priv->ctx_tail = n;
   }
}

static void
saspa_free_data(struct device *dev, unsigned long n, unsigned long pages)
{
   struct saspa_priv *priv = dev_get_drvdata(dev);

   bitmap_clear(priv->mem_alloc, n, pages);

   /* Move the data head forward if we freed the lowest data block. */
   if (n == priv->data_head) {
      n = find_next_bit(priv->mem_alloc, priv->mem_pages, priv->data_head);
      priv->data_head = n;
   }
}

/*
 * Free allocated SASPA memory.  The size parameter provided here must match
 * the actual allocated size of the block.
 */
void saspa_free(struct device *dev, long handle, size_t size)
{
   struct saspa_priv *priv = dev_get_drvdata(dev);
   unsigned long next, pages, n;

   if (handle < 0 || size == 0)
      return;

   pages = DIV_ROUND_UP(size, ALLOC_GRANULARITY);
   n = handle;

   if (pages > priv->mem_pages || n > priv->mem_pages - pages) {
      WARN(1, "Attempt to free non-existent memory");
      return;
   }

   spin_lock(&priv->mem_lock);

   /* Check that the block is fully allocated. */
   next = find_next_zero_bit(priv->mem_alloc, priv->mem_pages, n);
   WARN(next - n < pages, "Possible double free detected");

   if (n < priv->ctx_tail) {
      WARN_ON(n + pages > priv->ctx_tail);
      saspa_free_ctx(dev, n, pages);
   } else if (n >= priv->data_head) {
      saspa_free_data(dev, n, pages);
   } else {
      WARN(1, "Attempt to free unallocated memory");
   }

   spin_unlock(&priv->mem_lock);
}

/* XXX: Need to support DPRAM configurations and async operation. */
int saspa_mem_write(struct device *dev, long handle, const void *src, size_t size)
{
   struct saspa_priv *priv = dev_get_drvdata(dev);
   u32 __iomem *ptr;

   if (handle < 0 || handle >= priv->mem_pages)
      return -EINVAL;

   ptr = priv->mem + handle * (ALLOC_GRANULARITY>>2);

   down(&priv->core_busy);
   elpsaspa_write_mem(ptr, src, size);
   up(&priv->core_busy);

   return 0;
}

int saspa_mem_read(struct device *dev, void *dst, long handle, size_t size)
{
   struct saspa_priv *priv = dev_get_drvdata(dev);
   u32 __iomem *ptr;

   if (handle < 0 || handle >= priv->mem_pages)
      return -EINVAL;

   ptr = priv->mem + handle * (ALLOC_GRANULARITY>>2);

   down(&priv->core_busy);
   elpsaspa_read_mem(dst, ptr, size);
   up(&priv->core_busy);

   return 0;
}

int saspa_mem_addr(struct device *dev, long handle)
{
   struct saspa_priv *priv = dev_get_drvdata(dev);

   if (handle < 0 || handle >= priv->mem_pages)
      return -EINVAL;

   return handle * ALLOC_GRANULARITY;
}

long saspa_mem_size(struct device *dev)
{
   struct saspa_priv *priv = dev_get_drvdata(dev);
   return priv->mem_size;
}

/*
 * IRQ handler for the symmetric crypto cores.  Since only one crypto core can
 * be running at a time, we can group all the crypto interrupts and treat them
 * as one.
 */
static irqreturn_t saspa_irq_handler(int irq, void *dev)
{
   struct saspa_priv *priv = dev_get_drvdata(dev);
   u32 status;

   status = pdu_io_read32(&priv->regs[SASPA_GLBL_IRQ_STAT]);
   status &= SASPA_IRQ_CRYPTO_MASK;

   if (!status)
      return IRQ_NONE;

   pdu_io_write32(&priv->regs[SASPA_GLBL_IRQ_STAT], status);
   up(&priv->core_busy);

   return IRQ_HANDLED;
}

void saspa_enable_irq(struct device *dev, u32 mask)
{
   struct saspa_priv *priv = dev_get_drvdata(dev);
   u32 tmp;

   spin_lock(&priv->glbl_lock);

   tmp = pdu_io_read32(&priv->regs[SASPA_GLBL_IRQ_EN]);
   pdu_io_write32(&priv->regs[SASPA_GLBL_IRQ_EN], tmp | mask);

   spin_unlock(&priv->glbl_lock);
}

void saspa_disable_irq(struct device *dev, u32 mask)
{
   struct saspa_priv *priv = dev_get_drvdata(dev);
   u32 tmp;

   spin_lock(&priv->glbl_lock);

   tmp = pdu_io_read32(&priv->regs[SASPA_GLBL_IRQ_EN]);
   pdu_io_write32(&priv->regs[SASPA_GLBL_IRQ_EN], tmp & ~mask);

   spin_unlock(&priv->glbl_lock);
}

/* XXX: Make this asynchronous. */
int saspa_prepare_job(struct device *dev, long ctx, long data)
{
   struct saspa_priv *priv = dev_get_drvdata(dev);
   int rc;

   BUG_ON(ctx < 0 || ctx >= priv->mem_pages);
   BUG_ON(data < 0 || data >= priv->mem_pages);

   ctx  *= ALLOC_GRANULARITY;
   data *= ALLOC_GRANULARITY;

   down(&priv->core_busy);

   rc = elpsaspa_setup(priv->regs, ctx, data);
   if (rc < 0)
      return pdu_error_code(rc);

   return 0;
}

int saspa_wait(struct device *dev)
{
   struct saspa_priv *priv = dev_get_drvdata(dev);
   int rc;

   rc = down_interruptible(&priv->core_busy);
   if (rc < 0)
      return rc;

   up(&priv->core_busy);

   return 0;
}

static struct platform_device *
register_child(struct device *parent, const char *name, int id,
               resource_size_t reg_base, resource_size_t reg_len,
               resource_size_t irq, struct saspa_config *cfg)
{
   struct platform_device *pdev;
   struct resource res[2] = {
      {
         .start = reg_base,
         .end   = reg_base + reg_len - 1,
         .flags = IORESOURCE_MEM,
      }, {
         .start = irq,
         .end   = irq,
         .flags = IORESOURCE_IRQ,
      },
   };

   pdev = platform_device_register_resndata(parent, name, id, res, 2,
                                            cfg, sizeof *cfg);
   if (IS_ERR(pdev)) {
      dev_warn(parent, "failed to register %s device\n", name);
      return NULL;
   }
   return pdev;
}

/* Helper macro to register a SASPA child to reduce some boilerplate. */
#define CHILD(parent, drivername, macroname, cfg, mem, irq) (\
   register_child((parent), (drivername), -1, \
                  (mem)->start + SASPA_ ## macroname ## _BASE, \
                  SASPA_ ## macroname ## _SIZE, (irq)->start, cfg) \
)

/* Allocate SASPA private state and setup the shared memory allocator. */
static int saspa_init_state(struct device *dev, struct resource *mem_resource)
{
   unsigned long mem_size, mem_pages;
   struct saspa_priv *priv;
   struct resource *res;
   size_t bitmap_size;

   mem_size = resource_size(mem_resource) - SASPA_MEMORY_BASE;
   mem_pages = mem_size / ALLOC_GRANULARITY;
   mem_size = mem_pages * ALLOC_GRANULARITY;

   BUILD_BUG_ON(sizeof (DECLARE_BITMAP(, 1)) != sizeof *priv->mem_alloc);
   bitmap_size = sizeof (DECLARE_BITMAP(, mem_pages));

   priv = devm_kzalloc(dev, sizeof *priv + bitmap_size, GFP_KERNEL);
   if (!priv)
      return -ENOMEM;

   res = devm_request_mem_region(dev, mem_resource->start + SASPA_MEMORY_BASE,
                                 mem_size, DRVNAME "-mem");
   if (!res)
      return -EADDRNOTAVAIL;

   priv->mem = devm_ioremap_nocache(dev, res->start, resource_size(res));
   if (!priv->mem)
      return -ENOMEM;

   dev_info(dev, "Available memory: %lu bytes\n", mem_size);
   priv->mem_size = mem_size;

   priv->data_head = priv->mem_pages = mem_pages;
   spin_lock_init(&priv->mem_lock);

   dev_set_drvdata(dev, priv);
   return 0;
}

static int __devinit saspa_probe(struct platform_device *pdev)
{
   struct resource *mem_resource, *irq_resource;
   struct device *dev = &pdev->dev;
   struct saspa_priv *priv;
   struct saspa_config cfg;
   int registered = 0, rc;

   mem_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   if (!mem_resource || resource_size(mem_resource) < SASPA_MEMORY_BASE)
      return -EINVAL;

   irq_resource = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
   if (!irq_resource)
      return -EINVAL;

   rc = saspa_init_state(dev, mem_resource);
   if (rc < 0)
      return rc;
   priv = dev_get_drvdata(dev);

   sema_init(&priv->core_busy, 1);
   spin_lock_init(&priv->glbl_lock);

   priv->regs = pdu_linux_map_regs(&pdev->dev, mem_resource);
   if (IS_ERR(priv->regs))
      return PTR_ERR(priv->regs);

   rc = devm_request_irq(&pdev->dev, irq_resource->start, saspa_irq_handler,
                         IRQF_SHARED, dev_name(dev), dev);
   if (rc < 0)
      return rc;

   elpsaspa_get_config(priv->regs, &cfg);
   dev_info(dev, "Elliptic Technologies Inc. SASPA (EPN %.4x)\n", cfg.epn);
   dev_info(dev,
"\nFeature bits:\n"
"   AES: %d\n"
"        AES_128:  %d\n"
"        AES_192:  %d\n"
"        AES_256:  %d\n"
"        AES_cbc:  %d\n"
"        AES_ctr:  %d\n"
"        AES_ccm:  %d\n"
"        AES_xcbc: %d\n"
"         xcbc_1k: %d\n"
"        AES_f8:   %d\n"
"        AES_gcm:  %d\n"
"        AES_cmac: %d\n"
"        AES_xts:  %d\n"
"   DES: %d\n"
"   RC4: %d\n"
"  HMAC: %d\n"
"        MD5: %d\n"
"       SHA1: %d\n"
"     SHA224: %d\n"
"     SHA256: %d\n"
"     SHA384: %d\n"
"     SHA512: %d\n"
"  SNOW: %d\n"
"   PKA: %d\n"
"  TRNG: %d\n"
"   DPA: %d\n",
cfg.has_aes,cfg.has_aes_128,cfg.has_aes_192,cfg.has_aes_256,cfg.has_aes_cbc,cfg.has_aes_ctr,cfg.has_aes_ccm,cfg.has_aes_xcbc,cfg.has_aes_xcbc_1k,cfg.has_aes_f8,cfg.has_aes_gcm,cfg.has_aes_cmac,cfg.has_aes_xts,
cfg.has_des,cfg.has_rc4,cfg.has_hmac,cfg.has_md5,cfg.has_sha1,cfg.has_sha224,cfg.has_sha256,cfg.has_sha384,cfg.has_sha512,cfg.has_snow,cfg.has_pka,cfg.has_trng, cfg.has_dpa);

   /* Register all the child devices now. */
   if (cfg.has_pka) {
      priv->pka = CHILD(dev, "pka", PKA, NULL, mem_resource, irq_resource);
      registered += priv->pka != NULL;
   }

   if (cfg.has_trng) {
      priv->trng = CHILD(dev, "trng", TRNG, NULL, mem_resource, irq_resource);
      registered += priv->trng != NULL;
   }

   if (cfg.has_aes) {
      priv->aes = CHILD(dev, "saspa-aes", AES, &cfg, mem_resource, irq_resource);
      registered += priv->aes != NULL;
   }

   if (cfg.has_des) {
      priv->des = CHILD(dev, "saspa-des", DES, &cfg, mem_resource, irq_resource);
      registered += priv->des != NULL;
   }

   if (cfg.has_hmac) {
      priv->hmac = CHILD(dev, "saspa-hmac", HMAC, &cfg, mem_resource, irq_resource);
      registered += priv->hmac != NULL;
   }

   if (cfg.has_snow) {
      priv->snow = CHILD(dev, "saspa-snow", SNOW, &cfg, mem_resource, irq_resource);
      registered += priv->snow != NULL;
   }

   /* XXX: RC4 needs some special handling (ctx memory).  Skip it for now. */

   if (!registered) {
      dev_err(dev, "failed to register any child devices\n");
      return -ENODEV;
   }

   /* Set up IRQs. */
   pdu_io_write32(&priv->regs[SASPA_GLBL_IRQ_EN], 1ul << SASPA_IRQ_GLBL);

   /*
    * XXX: This is less than ideal: device IRQs shouldn't be enabled until
    * their driver's handler is registered.  But the PKA _shouldn't_ give IRQs
    * before a command has been programmed, so it should be safe.
    *
    * TRNG, on the other hand, gives an IRQ shortly after reset.
    */
   saspa_enable_irq(dev, 1ul << SASPA_IRQ_PKA);

#ifdef __LITTLE_ENDIAN
   elpsaspa_set_byteswap(priv->regs, 1);
#else
   elpsaspa_set_byteswap(priv->regs, 0);
#endif

   dev_info(dev, "registered %d child devices\n", registered);
   return 0;
}

static int __devexit saspa_remove(struct platform_device *pdev)
{
   struct saspa_priv *priv = platform_get_drvdata(pdev);

   platform_device_unregister(priv->aes);
   platform_device_unregister(priv->des);
   platform_device_unregister(priv->rc4);
   platform_device_unregister(priv->hmac);
   platform_device_unregister(priv->snow);
   platform_device_unregister(priv->trng);
   platform_device_unregister(priv->pka);

   return 0;
}

static struct platform_driver saspa_driver = {
   .probe = saspa_probe,
   .remove = __devexit_p(saspa_remove),

   .driver = {
      .name = DRVNAME,
      .owner = THIS_MODULE,
   },
};

static int __init saspa_init(void)
{
   int rc;

   rc = platform_driver_register(&saspa_driver);
   if (rc < 0)
      return rc;

   /*
    * Register drivers for individual crypto cores, but do not abort on
    * failure because the toplevel driver may already be doing useful work.
    */
   platform_driver_register(&saspa_aes_driver);
   platform_driver_register(&saspa_des_driver);
   platform_driver_register(&saspa_hash_driver);

   return 0;
}
module_init(saspa_init);

/* Unregister a platform driver only if it's already registered. */
static void __exit saspa_unregister_driver(struct platform_driver *drv)
{
   struct device_driver *driver;

   driver = driver_find(drv->driver.name, &platform_bus_type);
   if (driver) {
      WARN_ON(&drv->driver != driver);
      platform_driver_unregister(drv);
   }
}

static void __exit saspa_exit(void)
{
   saspa_unregister_driver(&saspa_aes_driver);
   saspa_unregister_driver(&saspa_des_driver);
   saspa_unregister_driver(&saspa_hash_driver);

   platform_driver_unregister(&saspa_driver);
}
module_exit(saspa_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
