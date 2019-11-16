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

#ifndef ELPPDU_H_
#define ELPPDU_H_

/* Platform Specific */
#include <linux/kernel.h>       /* printk() */
#include <linux/types.h>        /* size_t */
#include <linux/string.h>       /* memcpy()/etc */
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/io.h>
#include <linux/ctype.h>
#include <linux/version.h>

#define PDU_BASE_ADDR 0xB2100000
#define PDU_BASE_IRQ  118

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
      #undef __devexit_p
      #undef __devexit
      #undef __devinit
      #undef __devinitconst
      #define __devexit_p(x) x
      #define __devexit
      #define __devinit
      #define __devinitconst
#endif

#if 1
#define ELPHW_PRINT printk
#else
#define ELPHW_PRINT(...)
#endif

#define CPU_YIELD
#define ELPHW_MEMCPY memcpy

// Maximum number of DDT entries allowed
#ifndef PDU_MAX_DDT
   #define PDU_MAX_DDT                  64
#endif

// DMAable address type, usually can be some equivalent of uint32_t but in Linux it must be dma_addr_t
// since on 64-bit and PAE boxes the pointers used by coherant allocation are 64-bits (even though only the lower 32 bits are used)
#define PDU_DMA_ADDR_T               dma_addr_t

// Debug modifier for printing, in linux adding KERN_DEBUG makes the output only show up in debug logs (avoids /var/log/messages)
#define ELPHW_PRINT_DEBUG            KERN_DEBUG

// Locking
#define PDU_LOCK_TYPE                spinlock_t
#define PDU_INIT_LOCK(lock)          spin_lock_init(lock)

// these are for IRQ contexts
#define PDU_LOCK(lock, flags)        spin_lock_irqsave(lock, flags)
#define PDU_UNLOCK(lock, flags)      spin_unlock_irqrestore(lock, flags)

// these are for bottom half BH contexts
#define PDU_LOCK_TYPE_BH                struct mutex
#define PDU_INIT_LOCK_BH(lock)       mutex_init(lock)
#define PDU_LOCK_BH(lock)            mutex_lock(lock)
#define PDU_UNLOCK_BH(lock)          mutex_unlock(lock)

/* Platform Generic */
#define PDU_IRQ_EN_GLBL (1UL<<31)
#define PDU_IRQ_EN_VSPACC(x) (1UL<<x)
#define PDU_IRQ_EN_RNG  (1UL<<16)
#define PDU_IRQ_EN_PKA  (1UL<<17)
#define PDU_IRQ_EN_RE   (1UL<<18)
#define PDU_IRQ_EN_KEP  (1UL<<19)
#define PDU_IRQ_EN_EA   (1UL<<20)
#define PDU_IRQ_EN_MPM  (1UL<<21)
#ifdef  PDU_DUAL_MPM
   #define PDU_IRQ_EN_MPM1 (1UL<<22)
#endif

#include "elppdu_error.h"
#include "elppdu_types.h"

void *pdu_linux_map_regs(struct device *dev, struct resource *regs);

void pdu_io_write32(void *addr, unsigned long val);
void pdu_io_cached_write32(void *addr, unsigned long val, uint32_t *cache);
unsigned long pdu_io_read32(void *addr);

void pdu_to_dev32(void *addr, uint32_t *src, unsigned long nword);
void pdu_from_dev32(uint32_t *dst, void *addr, unsigned long nword);
void pdu_to_dev32_big(void *addr, const unsigned char *src, unsigned long nword);
void pdu_from_dev32_big(unsigned char *dst, void *addr, unsigned long nword);
void pdu_to_dev32_little(void *addr, const unsigned char *src, unsigned long nword);
void pdu_from_dev32_little(unsigned char *dst, void *addr, unsigned long nword);
void pdu_from_dev32_s(unsigned char *dst, void *addr, unsigned long nword, int endian);
void pdu_to_dev32_s(void *addr, const unsigned char *src, unsigned long nword, int endian);

void *pdu_malloc(unsigned long n);
void pdu_free(void *p);

void *pdu_dma_alloc(size_t bytes, PDU_DMA_ADDR_T *phys);
void pdu_dma_free(size_t bytes, void *virt, PDU_DMA_ADDR_T phys);

int pdu_mem_init(void *device);
void pdu_mem_deinit(void *device);

int pdu_ddt_init(pdu_ddt *ddt, unsigned long limit);
int pdu_ddt_add(pdu_ddt *ddt, PDU_DMA_ADDR_T phys, unsigned long size);
int pdu_ddt_reset(pdu_ddt *ddt);
int pdu_ddt_free(pdu_ddt *ddt);

void pdu_sync_single_for_device(uint32_t addr, uint32_t size);
void pdu_sync_single_for_cpu(uint32_t addr, uint32_t size);


int pdu_error_code(int code);

int pdu_get_version(void *dev, pdu_info *inf);

void spdu_boot_trng(pdu_info *info, unsigned long pdu_base);

#ifdef DO_PCIM
#include <linux/pci.h>
void pdu_pci_set_trng_ring_off(struct pci_dev *tif);
void pdu_pci_set_trng_ring_on(struct pci_dev *tif);
void pdu_pci_set_little_endian(struct pci_dev *tif);
void pdu_pci_set_big_endian(struct pci_dev *tif);
void pdu_pci_set_secure_off(struct pci_dev *tif);
void pdu_pci_set_secure_on(struct pci_dev *tif);
void pdu_pci_interrupt_enabled(struct pci_dev *tif);
void pdu_pci_reset(struct pci_dev *tif);
#endif

#ifdef PCI_INDIRECT
void elppci_indirect_trigger(void);
#endif

#endif

