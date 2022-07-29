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

#include <linux/dmapool.h>
#include <linux/module.h>
#include <linux/slab.h>
#include "elppdu.h"

static bool trace_io;
module_param(trace_io, bool, 0600);
MODULE_PARM_DESC(trace_io, "Trace MMIO reads/writes");

static bool trace_ddt;
module_param(trace_ddt, bool, 0600);
MODULE_PARM_DESC(trace_ddt, "Trace DDT entries");

static struct dma_pool *ddt_pool, *ddt16_pool, *ddt4_pool;
static struct device *ddt_device;

#ifdef PCI_INDIRECT
static void *pci_indirect_bus;
static spinlock_t indirect_lock;

void elppci_set_ptr(void *p) { pci_indirect_bus = p; spin_lock_init(&indirect_lock); }

void elppci_indirect_trigger(void)
{
   writel(0xFA7E1E55, pci_indirect_bus + 0x14);
}


void elppci_indirect_write(void *p, uint32_t data)
{
   uint32_t t, z;
   unsigned long flags;


   PDU_LOCK(&indirect_lock, flags);

   t = p;

   // write data
   writel(data, pci_indirect_bus + 0x10);
   // write addr
   writel(t, pci_indirect_bus + 0x08);
   // enable
   writel(0x80000001, pci_indirect_bus);

   z = 1UL<<16UL;
   while(--z && ((t = readl(pci_indirect_bus + 0x04)) & 0x80000000));
   if (!z) {
      printk("Timed out writing to %p\n", p);
   }
//   printk("IO: Wrote %08lx to %p and got %08lx as STAT\n", data, p, t);
   PDU_UNLOCK(&indirect_lock, flags);
}

uint32_t elppci_indirect_read(void *p)
{
   uint32_t t, z;
   unsigned long flags;

   PDU_LOCK(&indirect_lock, flags);

   t = p;
   // write addr
   writel(t, pci_indirect_bus + 0x08);
   writel(0x80000000, pci_indirect_bus);
   z = 1UL<<16UL;
   while(--z && ((t = readl(pci_indirect_bus + 0x04)) & 0x80000000));
   if (!z) {
      printk("Timed out reading from %p\n", p);
   }
   if ((t&0xFFFF) > 3) printk(KERN_DEBUG "ACK: Got %08lx back as a stat from a read, address %p\n", t, p);

   z = readl(pci_indirect_bus + 0x14);
//   printk("IO: Read %08lx from %p and got %08lx as STAT\n", z, p, t);
   PDU_UNLOCK(&indirect_lock, flags);
   return z;
}

#endif

void *pdu_linux_map_regs(struct device *dev, struct resource *regs)
{
#ifdef PCI_INDIRECT
   if (!devm_request_mem_region(dev, regs->start, resource_size(regs),
                                     "pci-indirect"))
   {
      return ERR_PTR(-EADDRNOTAVAIL);
   }

   return (void *)regs->start;
#endif

   return devm_ioremap_resource(dev, regs);
}
EXPORT_SYMBOL(pdu_linux_map_regs);

void pdu_io_write32(void *addr, unsigned long val)
{
   if (trace_io) {
      ELPHW_PRINT("PDU: write %.8lx -> %p\n", val, addr);
   }

#ifdef PCI_INDIRECT
   elppci_indirect_write(addr, val);
#else
   writel(val, addr);
#endif
}
EXPORT_SYMBOL(pdu_io_write32);

void pdu_io_cached_write32(void *addr, unsigned long val, uint32_t *cache)
{
   if (*cache == val) {
      if (trace_io) {
         ELPHW_PRINT("PDU: write %.8lx -> %p (cached)\n", val, addr);
      }
      return;
   }

   *cache = (uint32_t)val;
   pdu_io_write32(addr, val);
}
EXPORT_SYMBOL(pdu_io_cached_write32);

unsigned long pdu_io_read32(void *addr)
{
   unsigned long val;

#ifdef PCI_INDIRECT
   val = elppci_indirect_read(addr);
#else
   val = readl(addr);
#endif

   if (trace_io) {
      ELPHW_PRINT("PDU: read  %.8lx <- %p\n", val, addr);
   }

   return val;
}
EXPORT_SYMBOL(pdu_io_read32);

/* Platform specific memory allocation */
void *pdu_malloc (unsigned long n)
{
   return vmalloc (n);
}

void pdu_free (void *p)
{
   vfree (p);
}

/* allocate coherent memory */
void *pdu_dma_alloc(size_t bytes, PDU_DMA_ADDR_T *phys)
{
  // void * vir_addr;

   //vir_addr = kmalloc(bytes, GFP_KERNEL);
   //phys = __pa(vir_addr);

   //return vir_addr;
   return dma_alloc_coherent(ddt_device, bytes, phys, GFP_KERNEL);
}

/* free coherent memory */
void pdu_dma_free(size_t bytes, void *virt, PDU_DMA_ADDR_T phys)
{
   //kfree(virt);
   dma_free_coherent(ddt_device, bytes, virt, phys);
}

/* Platform specific DDT routines */

// create a DMA pool for DDT entries this should help from splitting
// pages for DDTs which by default are 520 bytes long meaning we would
// otherwise waste 3576 bytes per DDT allocated...
//
// we also maintain a smaller table of 4 entries common for simple jobs
// which uses 480 fewer bytes of DMA memory.
//
// and for good measure another table for 16 entries saving 384 bytes
int pdu_mem_init(void *device)
{
   if (ddt_device) {
      /* Already setup */
      return 0;
   }

   ddt_device = device;

   ddt_pool = dma_pool_create("elpddt", device, (PDU_MAX_DDT +1 ) * 8, 8, PDU_ALIGN_SIZE); // max of 64 DDT entries
   if (!ddt_pool) {
      return -1;
   }
#if PDU_MAX_DDT > 16
   ddt16_pool = dma_pool_create("elpddt16", device, (16 * 2 + 1) * 8, 8, PDU_ALIGN_SIZE); // max of 16 DDT entries
   if (!ddt16_pool) {
      dma_pool_destroy(ddt_pool);
      return -1;
   }
#else
   ddt16_pool = ddt_pool;
#endif
   ddt4_pool = dma_pool_create("elpddt4", device, (4 * 2 + 1) * 8, 8, PDU_ALIGN_SIZE); // max of 4 DDT entries
   if (!ddt4_pool) {
      dma_pool_destroy(ddt_pool);
#if PDU_MAX_DDT > 16
      dma_pool_destroy(ddt16_pool);
#endif
      return -1;
   }

   return 0;
}

// destroy the pool
void pdu_mem_deinit(void *device)
{
   /* For now, just skip deinit except for matching device */
   if (device != ddt_device)
      return;

   if (ddt_pool) {
      dma_pool_destroy(ddt_pool);
   }
#if PDU_MAX_DDT > 16
   if (ddt16_pool) {
      dma_pool_destroy(ddt16_pool);
   }
#endif
   if (ddt4_pool) {
      dma_pool_destroy(ddt4_pool);
   }

   ddt_device = NULL;
}

int pdu_ddt_init (pdu_ddt * ddt, unsigned long limit)
{
   int flag = (int)(limit & 0x80000000);  // set the MSB if we want to use an ATOMIC allocation required for top half processing
   limit &= 0x3FFFFFFF;

   limit = limit * 2 + 1;

   if (limit+1 >= SIZE_MAX/8) {
      /* Too big to even compute DDT size */
      return -1;
   } else if (limit > PDU_MAX_DDT) {
      size_t len = 8*((size_t)limit+1);

      ddt->virt = dma_alloc_coherent(ddt_device, len, &ddt->phys, flag?GFP_ATOMIC:GFP_KERNEL);
   } else if (limit > 16) {
      ddt->virt = dma_pool_alloc(ddt_pool, flag?GFP_ATOMIC:GFP_KERNEL, &ddt->phys);
   } else if (limit > 4) {
      ddt->virt = dma_pool_alloc(ddt16_pool, flag?GFP_ATOMIC:GFP_KERNEL, &ddt->phys);
   } else {
      ddt->virt = dma_pool_alloc(ddt4_pool, flag?GFP_ATOMIC:GFP_KERNEL, &ddt->phys);
   }
   ddt->idx = 0;
   ddt->len = 0;
   ddt->limit = limit;

   if (!ddt->virt) {
      return -1;
   }

   if (trace_ddt) {
      ELPHW_PRINT("DDT[%.8lx]: allocated %lu fragments\n",
                               (unsigned long)ddt->phys, limit);
   }

   return 0;
}

int pdu_ddt_add (pdu_ddt * ddt, PDU_DMA_ADDR_T phys, unsigned long size)
{
   uint32_t size_before_aligned = 0;

   if (trace_ddt) {
      printk("DDT[%.8lx]: 0x%.8lx size %lu\n", (unsigned long)ddt->phys,
                                               (unsigned long)phys, size);
   }

   if (size >= PDU_ALIGN_SIZE) {
      pr_err("%s max support segment size is %lu\n", __func__, PDU_ALIGN_SIZE - 1);
	  return -1;
   }

   if (ddt->idx >= ddt->limit - 2) {
      pr_err("%s error: ddt->idx %lu, ddt->limit:%lu\n", __func__, ddt->idx, ddt->limit);
      return -1;
   }

   size_before_aligned = PDU_ALIGN_SIZE - (phys & (PDU_ALIGN_SIZE - 1));

   if (size > size_before_aligned) {
      ddt->virt[ddt->idx * 4 + 0] = (uint32_t)phys;
      ddt->virt[ddt->idx * 4 + 1] = (uint32_t)size_before_aligned;
      ddt->virt[ddt->idx * 4 + 2] = (uint32_t)phys + size_before_aligned;
      ddt->virt[ddt->idx * 4 + 3] = (uint32_t)size - size_before_aligned;
      ddt->virt[ddt->idx * 4 + 4] = 0;
      ddt->virt[ddt->idx * 4 + 5] = 0;
      ddt->len += size;
      ddt->idx += 2;
   } else {
      ddt->virt[ddt->idx * 2 + 0] = (uint32_t)phys;
      ddt->virt[ddt->idx * 2 + 1] = (uint32_t)size;
      ddt->virt[ddt->idx * 2 + 2] = 0;
      ddt->virt[ddt->idx * 2 + 3] = 0;
      ddt->len += size;
      ++(ddt->idx);
   }
   return 0;
}

int pdu_ddt_reset (pdu_ddt * ddt)
{
   ddt->idx = 0;
   ddt->len = 0;
   return 0;
}

int pdu_ddt_free (pdu_ddt * ddt)
{
   if (ddt->virt != NULL) {
      if (ddt->limit > PDU_MAX_DDT) {
         size_t len = 8*((size_t)ddt->limit+1);

         dma_free_coherent(ddt_device, len, ddt->virt, ddt->phys);
      } else if (ddt->limit > 16) {
         dma_pool_free(ddt_pool, ddt->virt, ddt->phys);
      } else if (ddt->limit > 4) {
         dma_pool_free(ddt16_pool, ddt->virt, ddt->phys);
      } else {
         dma_pool_free(ddt4_pool, ddt->virt, ddt->phys);
      }
      ddt->virt = NULL;
   }
   return 0;
}

/*
 * convert scatterlist to h/w ddt table format
 * scatterlist must have been previously dma mapped
 * pdu_ddt must have been initialized
 */
int pdu_sg_to_ddt(struct scatterlist *sg, int sg_count, pdu_ddt *ddt)
{
   int err = 0;
   while (sg_count) {
      err = pdu_ddt_add (ddt, (PDU_DMA_ADDR_T)sg_dma_address(sg), sg_dma_len(sg));
      if (err)
         return err;

      sg = sg_next(sg);
      sg_count--;
   }
   return err;
}

void pdu_sync_single_for_device(uint32_t addr, uint32_t size)
{
   dma_sync_single_for_device(ddt_device, addr, size, DMA_BIDIRECTIONAL);
}

void pdu_sync_single_for_cpu(uint32_t addr, uint32_t size)
{
   dma_sync_single_for_cpu(ddt_device, addr, size, DMA_BIDIRECTIONAL);
}

/* Convert SDK error codes to corresponding kernel error codes. */
int pdu_error_code(int code)
{
   switch (code) {
   case CRYPTO_INPROGRESS:
      return -EINPROGRESS;
   case CRYPTO_INVALID_HANDLE:
   case CRYPTO_INVALID_CONTEXT:
      return -ENXIO;
   case CRYPTO_NOT_INITIALIZED:
      return -ENODATA;
   case CRYPTO_INVALID_SIZE:
   case CRYPTO_INVALID_ALG:
   case CRYPTO_INVALID_KEY_SIZE:
   case CRYPTO_INVALID_ARGUMENT:
   case CRYPTO_INVALID_BLOCK_ALIGNMENT:
   case CRYPTO_INVALID_MODE:
   case CRYPTO_INVALID_KEY:
   case CRYPTO_INVALID_IV_SIZE:
   case CRYPTO_INVALID_ICV_KEY_SIZE:
   case CRYPTO_INVALID_PARAMETER_SIZE:
   case CRYPTO_REPLAY:
   case CRYPTO_INVALID_PROTOCOL:
   case CRYPTO_RESEED_REQUIRED:
      return -EINVAL;
   case CRYPTO_NOT_IMPLEMENTED:
   case CRYPTO_MODULE_DISABLED:
      return -ENOTSUPP;
   case CRYPTO_NO_MEM:
      return -ENOMEM;
   case CRYPTO_INVALID_PAD:
   case CRYPTO_INVALID_SEQUENCE:
      return -EILSEQ;
   case CRYPTO_MEMORY_ERROR:
      return -EIO;
   case CRYPTO_TIMEOUT:
      return -ETIMEDOUT;
   case CRYPTO_HALTED:
      return -ECANCELED;
   case CRYPTO_AUTHENTICATION_FAILED:
   case CRYPTO_SEQUENCE_OVERFLOW:
   case CRYPTO_INVALID_VERSION:
      return -EPROTO;
   case CRYPTO_FIFO_FULL:
      return -EBUSY;
   case CRYPTO_SRM_FAILED:
   case CRYPTO_DISABLED:
   case CRYPTO_LAST_ERROR:
      return -EAGAIN;
   case CRYPTO_FAILED:
   case CRYPTO_FATAL:
      return -EIO;
   case CRYPTO_INVALID_FIRMWARE:
      return -ENOEXEC;
   case CRYPTO_NOT_FOUND:
      return -ENOENT;
   }

   /*
    * Any unrecognized code is either success (i.e., zero) or a negative
    * error code, which may be meaningless but at least will still be
    * recognized as an error.
    */
   return code;
}

static int __init pdu_mod_init (void)
{
   return 0;
}

static void __exit pdu_mod_exit (void)
{
}

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init (pdu_mod_init);
module_exit (pdu_mod_exit);

// kernel functions
EXPORT_SYMBOL (pdu_to_dev32);
EXPORT_SYMBOL (pdu_from_dev32);
EXPORT_SYMBOL (pdu_to_dev32_big);
EXPORT_SYMBOL (pdu_from_dev32_big);
EXPORT_SYMBOL (pdu_to_dev32_little);
EXPORT_SYMBOL (pdu_from_dev32_little);
EXPORT_SYMBOL (pdu_to_dev32_s);
EXPORT_SYMBOL (pdu_from_dev32_s);

EXPORT_SYMBOL (pdu_malloc);
EXPORT_SYMBOL (pdu_free);
EXPORT_SYMBOL (pdu_dma_alloc);
EXPORT_SYMBOL (pdu_dma_free);
EXPORT_SYMBOL (pdu_mem_init);
EXPORT_SYMBOL (pdu_mem_deinit);
EXPORT_SYMBOL (pdu_ddt_init);
EXPORT_SYMBOL (pdu_ddt_add);
EXPORT_SYMBOL (pdu_ddt_reset);
EXPORT_SYMBOL (pdu_ddt_free);
EXPORT_SYMBOL (pdu_sg_to_ddt);
EXPORT_SYMBOL (pdu_sync_single_for_device);
EXPORT_SYMBOL (pdu_sync_single_for_cpu);
EXPORT_SYMBOL (pdu_error_code);

// lib functions
EXPORT_SYMBOL (pdu_get_version);

#ifdef PCI_INDIRECT
EXPORT_SYMBOL(elppci_indirect_trigger);
EXPORT_SYMBOL(elppci_set_ptr);
#endif
