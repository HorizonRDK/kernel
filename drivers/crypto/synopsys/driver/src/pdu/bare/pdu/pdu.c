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

#include "elppdu.h"


/* Platform Specific I/O */
static int debug_on=0;

/* write a 32-bit word to a given address */
void pdu_io_write32 (void *addr, unsigned long val)
{
   if (addr == NULL) {
      debug_on ^= 1;
      return;
   }
   if (debug_on) {
      ELPHW_PRINT("PDU: write %.8lx -> %p\n", val, addr);
   }
   *((uint32_t *)addr) = val;
}

/* read a 32-bit word from a given address */
unsigned long pdu_io_read32 (void *addr)
{
   unsigned long foo;
   foo = *((uint32_t *)addr);
   if (debug_on) {
      ELPHW_PRINT("PDU: read  %.8lx <- %p\n", foo, addr);
   }
   return foo;
}

/* Platform specific DDT routines */

// initialize memory for DDT routines
int pdu_mem_init(void *device)
{
   return 0; // does nothing, here is where you could initialize your heap/etc
}

// cleanup memory used by DDT routines
void pdu_mem_deinit(void)
{
}


int pdu_ddt_init (pdu_ddt * ddt, unsigned long limit)
{
   limit &= 0x7FFFFFFF; // top bit is used for flags which is ignored here

   ddt->virt = ddt->phys = calloc(8, limit + 1);
   ddt->idx = 0;
   ddt->len = 0;
   ddt->limit = limit;
   return ddt->virt == NULL ? -1 : 0;
}

int pdu_ddt_add (pdu_ddt * ddt, PDU_DMA_ADDR_T phys, unsigned long size)
{
   if (ddt->idx == ddt->limit) {
      return -1;
   }
   ddt->virt[ddt->idx * 2 + 0] = (uint32_t) phys;  /* write in address and size */
   ddt->virt[ddt->idx * 2 + 1] = size;
   ddt->virt[ddt->idx * 2 + 2] = 0;                /* ensure list is NULL terminated */
   ddt->virt[ddt->idx * 2 + 3] = 0;
   ddt->len += size;
   ++(ddt->idx);
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
   free(ddt->virt);
   return 0;
}

/* Platform specific memory allocation */
void *pdu_malloc (unsigned long n)
{
   return malloc (n);
}

void pdu_free (void *p)
{
   free (p);
}

/* allocate coherent memory (currently just allocates heap memory) */
void *pdu_dma_alloc(size_t bytes, PDU_DMA_ADDR_T *phys)
{
   *phys = pdu_malloc(bytes);
   return *phys;
}

/* free coherent memory */
void *pdu_dma_free(size_t bytes, void *virt, PDU_DMA_ADDR_T phys)
{
   pdu_free(virt);
}


/* sync memory for the device to read, may be an empty function on cache coherent or cacheless designs */
void pdu_sync_single_for_device(uint32_t addr, uint32_t size)
{
}

/* sync memory for the CPU to read, may be an empty function on cache coherent or cacheless designs */
void pdu_sync_single_for_cpu(uint32_t addr, uint32_t size)
{
}

void pdu_io_cached_write32 (void *addr, unsigned long val, uint32_t *cache)
{
   if (*cache == val) {
      return;
   }
   *cache = val;
   pdu_io_write32 (addr, val);
}

/* Convert SDK error codes to corresponding kernel error codes. */
int pdu_error_code(int code)
{
   return code;
}

