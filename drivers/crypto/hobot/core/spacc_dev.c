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

//
//  This file incorporates zero-copy routines from Linux CryptoDev,
//  covered by the following copyright and permission notice:
//
//    Copyright (c) 2009-2011 Nikos Mavrogiannopoulos <nmav@gnutls.org>
//    Copyright (c) 2010 Phil Sutter
//    Copyright (c) 2011, 2012 OpenSSL Software Foundation, Inc.
//
//    This program is free software; you can redistribute it and/or
//    modify it under the terms of the GNU General Public License
//    as published by the Free Software Foundation; either version 2
//    of the License, or (at your option) any later version.
// 
// ------------------------------------------------------------------------

#include <linux/kernel.h>
#include <linux/scatterlist.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>
#include <linux/completion.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/ratelimit.h>
#include <linux/jiffies.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/pipe_fs_i.h>
#include <linux/slab.h>


#include <linux/list.h>

#include <linux/delay.h>

#include "elpspaccdrv.h"

// this is the maximum number of DDT entries (not including the NULL) for
// either direction of the engine.  Don't set below 8 unless you are really
// constrained for memory.  Don't set above 64 unless you really need to support
// highly fragmented data.
#ifndef SPACC_DEFAULT_DDT_LENGTH
   #define SPACC_DEFAULT_DDT_LENGTH 32
#endif

#ifndef KERNEL
#define KERNEL
#endif
#include "elpspaccusr.h"

struct platform_device *spacc_pdev;
static spacc_device *spacc;

//#define USE_POLL
//#define PERF_MON
#ifdef PERF_MON

struct {
   uint32_t ioctl_total,
            open_total,
            job_map_total,
            deprogram_total,
            spacc_write_total,
            polling_total,
            demap_total,
            job_init_total;

   uint32_t ioctl_ents,
            open_ents,
            job_map_ents,
            deprogram_ents,
            spacc_write_ents,
            polling_ents,
            demap_ents,
            job_init_ents;
} perf;
#endif

#ifdef PERF_MON


// users must supply their own high res monotonically increasing counter routine here...
uint32_t an224_clk_simple_read(void);
static uint32_t perf_counter(void)
{
   return an224_clk_simple_read();
}
#endif

static void (*old_notify_jobs)(struct _spacc_device *spacc);

static struct State
{
   struct device *dev;
   struct file *fp;
   int spacc_handle, state, map;

   volatile int blocked;

   unsigned int array_size;
   unsigned int used_pages;     /* the number of pages that are used */
   /* the number of pages marked as NOT-writable; they preceed writeables */
   unsigned int readonly_pages, write_pages;
   struct page **pages;
   struct scatterlist *sg, *dst_sg;
   unsigned src_ents, dst_ents;

   pdu_ddt src_ddt, dst_ddt;

   struct completion comp, fifo_full;
   struct semaphore lock;

   unsigned char state_key[16];

} *states = NULL;

static int state_num;
//static int spacc_epn = 0x0414;  // EPN of hardware to look up (TODO: change this to your own core)
static int spacc_epn = 0x0;  // EPN of hardware to look up (TODO: change this to your own core)

/*
 * Add an SG entry to DDT, taking care to split entries which straddle
 * H/W addressing boundary.  Note that the addressing boundaries depend
 * on pCLUSTER_ADDR_WIDTH, which unfortunately is not part of the H/W
 * configuration registers.  The following assumes a value of 16 (64 KiB).
 */
static int sg_to_ddt(pdu_ddt *ddt, const struct scatterlist *sg)
{
   dma_addr_t baseaddr = sg_dma_address(sg);
   unsigned long totlen = sg_dma_len(sg);
   int rc = 0;

   while (totlen) {
      dma_addr_t maxaddr = (baseaddr | 0xffff) + 1;
      unsigned long curlen = min_t(unsigned long, totlen, maxaddr - baseaddr);

      rc = pdu_ddt_add(ddt, baseaddr, curlen);
      if (rc != 0)
         break;

      baseaddr += curlen;
      totlen -= curlen;
   }

   return rc;
}

/* map an SG to DDT */
static int map_sgs_to_ddt (struct State *s)
{
   unsigned x, y;
   struct scatterlist *sgtmp;

   // map source pages
   x = dma_map_sg(s->dev, s->sg, (s->sg==s->dst_sg)?s->used_pages:s->readonly_pages, DMA_BIDIRECTIONAL);
   if (!x) {
      return -1;
   }
   pdu_ddt_reset (&s->src_ddt);
   for_each_sg (s->sg, sgtmp, x, y) {
      if (sg_to_ddt(&s->src_ddt, sgtmp) != 0) {
         dma_unmap_sg(s->dev, s->sg, s->readonly_pages, DMA_BIDIRECTIONAL);
         return -1;
      }
   }

   // map dest pages
   if (s->dst_sg != s->sg) {
      x = dma_map_sg(s->dev, s->dst_sg, s->write_pages, DMA_BIDIRECTIONAL);
      if (!x) {
         dma_unmap_sg(s->dev, s->sg, s->readonly_pages, DMA_BIDIRECTIONAL);
         return -1;
      }
   }
   pdu_ddt_reset (&s->dst_ddt);
   for_each_sg (s->dst_sg, sgtmp, x, y) {
      if (sg_to_ddt(&s->dst_ddt, sgtmp) != 0) {
         dma_unmap_sg(s->dev, s->sg, s->readonly_pages, DMA_BIDIRECTIONAL);
         dma_unmap_sg(s->dev, s->dst_sg, s->write_pages, DMA_BIDIRECTIONAL);
         return -1;
      }
   }
   return 0;
}

void unmap_sgs(struct State *s)
{
   dma_unmap_sg(s->dev, s->sg, (s->sg==s->dst_sg)?s->used_pages:s->readonly_pages, DMA_BIDIRECTIONAL);
   if (s->dst_sg != s->sg) {
      dma_unmap_sg(s->dev, s->dst_sg, s->write_pages, DMA_BIDIRECTIONAL);
   }
}

/* offset of buf in it's first page */
#define PAGEOFFSET(buf) ((unsigned long)buf & ~PAGE_MASK)

/* buflen ? (last page - first page + 1) : 0 */
#define PAGECOUNT(buf, buflen) ((buflen) \
   ? ((((unsigned long)(buf + buflen - 1)) >> PAGE_SHIFT) - \
      (((unsigned long)(buf             )) >> PAGE_SHIFT) + 1) \
   : 0)

#define DEFAULT_PREALLOC_PAGES 32


static int zc_get_userbuf (struct elp_spacc_usr_ddt *addr, int *pgcnts, int write,
                                       unsigned int  pgcount, struct page **pg,
                                 struct scatterlist *sg,
                                 struct task_struct *task, struct mm_struct *mm)
{
   unsigned len;

   int ret, pglen, i, x;
   struct scatterlist *sgp;

   if (unlikely (!pgcount || !addr[0].ptr)) {
      sg_mark_end (sg);
      return 0;
   }

   sg_init_table (sg, pgcount);
   sgp = sg;

   for (i = x = 0; x < ELP_SPACC_USR_MAX_DDT && addr[x].ptr != NULL; x++) {
      len = addr[x].len;

      ret = get_user_pages_fast((unsigned long)addr[x].ptr, pgcnts[x], write, pg + i);
      if (ret != pgcnts[x]) {
         return -EINVAL;
      }

      pglen = min ((ptrdiff_t) (PAGE_SIZE - PAGEOFFSET (addr[x].ptr)), (ptrdiff_t) len);
      sg_set_page (sgp, pg[i++], pglen, PAGEOFFSET (addr[x].ptr));

      len -= pglen;
      for (sgp = sg_next (sgp); len; sgp = sg_next (sgp)) {
         pglen = min ((uint32_t) PAGE_SIZE, len);
         sg_set_page (sgp, pg[i++], pglen, 0);
         len -= pglen;
      }
   }
   sg_mark_end (sg_last (sg, pgcount));
   return 0;
}


static int zc_adjust_sg_array (struct State *ses, int pagecount)
{
   struct scatterlist *sg;
   struct page **pages;
   int array_size;

   for (array_size = ses->array_size; array_size < pagecount; array_size *= 2);
//   printk (KERN_DEBUG "reallocating from %d to %d pages\n", ses->array_size, array_size);
   pages = krealloc (ses->pages, array_size * sizeof (struct page *), GFP_KERNEL);
   if (unlikely (!pages))
      return -ENOMEM;
   ses->pages = pages;
   sg = krealloc (ses->sg, array_size * sizeof (struct scatterlist), GFP_KERNEL);
   if (unlikely (!sg))
      return -ENOMEM;
   ses->sg = sg;
   ses->array_size = array_size;

   return 0;
}

static void zc_release_user_pages (struct State *ses)
{
   unsigned int i;

   if (likely(ses->map)) {
      unmap_sgs(ses);
      ses->map = 0;
   }

   for (i = 0; i < ses->used_pages; i++) {
      if (!PageReserved (ses->pages[i]))
         SetPageDirty (ses->pages[i]);

      if (ses->readonly_pages == 0)
         flush_dcache_page (ses->pages[i]);
      else
         ses->readonly_pages--;

      put_page(ses->pages[i]);
   }
   ses->used_pages = 0;
}

static int get_userbufs (struct State *ses, struct elp_spacc_usr_ddt *src, struct elp_spacc_usr_ddt *dst, unsigned hint)
{
   int src_pagecount, dst_pagecount;
   int maptype, rc, x, src_pg_counts[ELP_SPACC_USR_MAX_DDT+1], dst_pg_counts[ELP_SPACC_USR_MAX_DDT+1];

   struct task_struct *task;
   struct mm_struct *mm;

   task = current;
   mm = current->mm;

   // we don't support NULL mappings
   if (src[0].ptr == NULL || dst[0].ptr == NULL) {
      return -1;
   }

   // prevent mapping 0 bytes so we default to 1
   if (src[0].len == 0) {
      src[0].len = 1;
   }

   if (dst[0].len == 0) {
      dst[0].len = 1;
   }


   for (x = src_pagecount = 0; x < ELP_SPACC_USR_MAX_DDT && src[x].ptr != NULL; x++) {
      src_pagecount += (src_pg_counts[x] = (int)PAGECOUNT(src[x].ptr, src[x].len));
   }

   for (x = dst_pagecount = 0; x < ELP_SPACC_USR_MAX_DDT && dst[x].ptr != NULL; x++) {
      dst_pagecount += (dst_pg_counts[x] = (int)PAGECOUNT(dst[x].ptr, dst[x].len));
   }

   // are these the same array
   if (hint == SPACC_MAP_HINT_TEST) {
      maptype = 1;
      for (x = 0; (x < ELP_SPACC_USR_MAX_DDT) && (src[x].ptr || dst[x].ptr); x++) {
         if (dst[x].ptr == NULL || src[x].ptr == NULL || (src[x].ptr == dst[x].ptr && ((src[x+1].ptr == NULL && src[x].len <= dst[x].len) || (dst[x+1].ptr == NULL && dst[x].len <= src[x].len)))) {
            // they're allowed to be different sizes provided one is fully contained in another
            if (src[x].ptr == NULL || (src[x+1].ptr == NULL && src[x].len <= dst[x].len)) {
               // set it to 2 if the source is fully contained inside the dest (this ensures the correct # of pages/bytes are mapped)
               maptype = 2;
            }
            break;
         }
         // do the pointers match and/or lengths
         if (src[x].ptr != dst[x].ptr || src[x].len != dst[x].len) {
            maptype = 0;
            break;
         }
      }
   } else if (hint == SPACC_MAP_HINT_USESRC) {
      maptype = 1;
   } else if (hint == SPACC_MAP_HINT_USEDST) {
      maptype = 2;
   } else {
      maptype = 0;
   }

   ses->used_pages     = (maptype) ? max (src_pagecount, dst_pagecount) : (src_pagecount + dst_pagecount);
   ses->readonly_pages = (maptype) ? 0 : src_pagecount;
   ses->write_pages    = dst_pagecount;

   if (ses->used_pages > ses->array_size) {
      rc = zc_adjust_sg_array (ses, ses->used_pages);
      if (rc)
         return rc;
   }

   if (maptype) {            /* inplace operation */
      rc = zc_get_userbuf ((maptype == 1) ? src : dst, (maptype == 1) ? src_pg_counts : dst_pg_counts, 1, ses->used_pages, ses->pages, ses->sg, task, mm);
      if (unlikely (rc)) {
         printk (KERN_ERR "failed to get user pages for data IO\n");
         return rc;
      }
      ses->dst_sg = ses->sg;
   } else {
      {
         rc = zc_get_userbuf (src, src_pg_counts, 0, ses->readonly_pages, ses->pages, ses->sg, task, mm);
         if (unlikely (rc)) {
            printk (KERN_ERR "failed to get user pages for data input\n");
            return rc;
         }
      }

      {
         const unsigned int writable_pages = ses->used_pages - ses->readonly_pages;
         struct page **dst_pages = ses->pages + ses->readonly_pages;
         ses->dst_sg = ses->sg + ses->readonly_pages;

         rc = zc_get_userbuf (dst, dst_pg_counts, 1, writable_pages, dst_pages, ses->dst_sg, task, mm);
         if (unlikely (rc)) {
            printk (KERN_ERR "failed to get user pages for data output\n");
            zc_release_user_pages (ses);  /* FIXME: use __release_userbuf(src, ...) */
            return rc;
         }
      }
   }

   ses->map = 0;
   if (unlikely(map_sgs_to_ddt(ses))) {
      printk(KERN_ERR "Failed to map SGs to DDT\n");
      zc_release_user_pages(ses);
   }
   ses->map = 1;

   return 0;
}

static void check_fifos(void)
{
    int x;
    for (x = 0; x < state_num; x++) {
        if (states[x].blocked) {
            complete(&states[x].fifo_full);
        }
    }
}

static void spaccdev_callback (void *sprof_dev, void *data)
{
   complete (data);
}

static void spacc_notify_jobs(struct _spacc_device *spacc)
{
   check_fifos();
   if (old_notify_jobs) {
      old_notify_jobs(spacc);
   }
}

// look up a file handle in our state table and return the index if any
static int spacc_fp_lookup (struct file *fp)
{
   int x;
   for (x = 0; x < state_num; x++) {
      if (states[x].fp == fp) {
         return x;
      }
   }
   return -1;
}

static struct semaphore dev_lock;

static int spacc_dev_open (struct inode *in, struct file *fp)
{
   int x;

   pr_debug("spaccdev::open %p\n", fp);

   down(&dev_lock);

   // find a hole
   for (x = 0; x < state_num; x++) {
      if (states[x].fp == NULL) {
         break;
      }
   }
   if (x == state_num) {
      printk ("spacc_dev_open::No more open slots for a new SPAcc handle\n");
      up(&dev_lock);
      return -1;
   }
   // we allocate the spacc handle when a job is fired
   states[x].dev          = get_device(&spacc_pdev->dev);
   states[x].spacc_handle = -1;
   states[x].fp           = fp;
   states[x].map          = 0;
   states[x].blocked      = 0;

   up(&dev_lock);

   return 0;
}

static int spacc_dev_release (struct inode *in, struct file *fp)
{
   int x;

   // printk("spaccdev::close %p\n", fp);
   down(&dev_lock);

   x = spacc_fp_lookup (fp);
   if (x == -1) {
      printk ("spacc_dev_close::invalid 'struct file *' pointer\n");
      up(&dev_lock);
      return -1;
   }

   states[x].fp = NULL;
   if (states[x].spacc_handle >= 0) {
      spacc_close (spacc, states[x].spacc_handle);
      states[x].spacc_handle = -1;
   }

   // complete the lock in case there was a pending job
   complete(&states[x].comp);
   states[x].blocked = 0;

   put_device(states[x].dev);

   // zero the state key used to pair it
   memset(states[x].state_key, 0, sizeof states[x].state_key);

   up(&dev_lock);

#ifdef PERF_MON
   printk("SPAcc-DEV perf mon\n");
   printk("OPEN, IOCTL, JOB INIT, JOB MAP, MMIO WR, DEPROGRAM, DEMAP, POLLING\n");
   printk("%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,\n",
      (unsigned long)(perf.open_total/(perf.open_ents?perf.open_ents:1)),
      (unsigned long)(perf.ioctl_total/(perf.ioctl_ents?perf.ioctl_ents:1)),
      (unsigned long)(perf.job_init_total/(perf.job_init_ents?perf.job_init_ents:1)),
      (unsigned long)(perf.job_map_total/(perf.job_map_ents?perf.job_map_ents:1)),
      (unsigned long)(perf.spacc_write_total/(perf.spacc_write_ents?perf.spacc_write_ents:1)),
      (unsigned long)(perf.deprogram_total/(perf.deprogram_ents?perf.deprogram_ents:1)),
      (unsigned long)(perf.demap_total/(perf.demap_ents?perf.demap_ents:1)),
      (unsigned long)(perf.polling_total/(perf.polling_ents?perf.polling_ents:1)));
#endif
   return 0;
}

static long scan_state(int si, unsigned char *state_lock)
{
   int x;
   static unsigned char zeroes[16] = { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0  };

   // we disallow all zero keys for obvious reasons
   if (!memcmp(zeroes, state_lock, 16)) {
      return -1;
   }

   down(&dev_lock);
   for (x = 0; x < state_num; x++) {
       if (likely(x != si)) {
          if (states[x].spacc_handle >= 0 && !memcmp(states[x].state_key, state_lock, 16)) {

             // copy the state key so the master can be freed and we can still pair to this one at least
             memcpy(states[si].state_key, state_lock, 16);

             // return the context handle being used (note that dev_lock IS STILL BEING HELD!!!)
             // this is required to prevent someone from freeing the master between here and our
             // eventual spacc_open call below
             return spacc->job[states[x].spacc_handle].ctx_idx;
          }
       }
    }
    up(&dev_lock);
    return -1;
 }

static long spacc_dev_ioctl2 (int si, struct file *fp, unsigned int cmd, unsigned long arg_)
{
   void __user *arg = (void __user *) arg_;
   struct elp_spacc_ioctl io;
   struct elp_spacc_features features;
   int withicv, copied, ctxid;
   long outlen, inlen;

#ifdef PERF_MON
   uint32_t t3, t2, t1 = perf_counter();
#endif

   copied = 0;

   switch (cmd) {
      case ELP_SPACC_USR_FEATURES:
          features.project      = spacc->config.project;
          features.partial      = spacc->config.is_partial;
          features.qos          = spacc->config.is_qos;
          features.version      = spacc->config.version;
          features.ivimport     = spacc->config.is_ivimport;
          features.max_msg_size = spacc->config.max_msg_size;
          memcpy(features.modes, spacc->config.modes, sizeof features.modes);
          if (unlikely (copy_to_user (arg, &features, sizeof(features)))) {
             printk ("spacc_dev_ioctl::Could not copy ioctl structure back to user\n");
             return -EIO;
          }
          return 0;
      case ELP_SPACC_USR_INIT:
         // copy struct from user
         if (unlikely (copy_from_user (&io, arg, sizeof (io)))) {
            printk ("spacc_dev_ioctl::Cannot copy ioctl buffer from user\n");
            return -EFAULT;
         }

         // sanity check key/iv
         if (io.ckeylen > sizeof(io.ckey) ||
             io.civlen  > sizeof(io.civ)  ||
             io.hkeylen > sizeof(io.hkey) ||
             io.hivlen  > sizeof(io.hiv)) {
             printk("spacc_dev_ioctl::Invalid key or IV length specified\n");
             return -EFAULT;
          }

         // open spacc handle
         states[si].spacc_handle = spacc_open (spacc, io.cipher_mode, io.hash_mode, -1, 0, spaccdev_callback, &states[si].comp);
         if (states[si].spacc_handle < 0) {
            printk ("spacc_dev_ioctl::Failed to open a spacc handle\n");
            return -1;
         }

         // set context
         if (io.cipher_mode != CRYPTO_MODE_NULL) {
            spacc_write_context (spacc, states[si].spacc_handle, SPACC_CRYPTO_OPERATION, io.ckey, io.ckeylen, io.civ, io.civlen);
         }
         if (io.hash_mode != CRYPTO_MODE_NULL) {
            spacc_write_context (spacc, states[si].spacc_handle, SPACC_HASH_OPERATION, io.hkey, io.hkeylen, io.hiv, io.hivlen);
         }
         // set operation, we append the ICV
         spacc_set_operation (spacc, states[si].spacc_handle, io.encrypt ? OP_ENCRYPT : OP_DECRYPT, io.icv_mode, IP_ICV_APPEND, 0, io.icv_len, 0);

         // if we are decrypting set the expand bit (required for RC4/AES)
         if (!io.encrypt || io.cipher_mode == CRYPTO_MODE_RC4_128 || io.cipher_mode == CRYPTO_MODE_RC4_40) {
            spacc_set_key_exp (spacc, states[si].spacc_handle);
         }

#ifdef PERF_MON
         perf.open_total += (perf_counter() - t1);
         perf.open_ents++;
#endif
         return 0;

      case ELP_SPACC_USR_REGISTER:
         if (unlikely (copy_from_user (&io, arg, sizeof (io)))) {
            printk ("spacc_dev_ioctl::Cannot copy ioctl buffer from user\n");
            return -EFAULT;
         }

         // copy key
         memcpy(states[si].state_key, io.state_key, 16);

         return 0;

      case ELP_SPACC_USR_BIND:
         if (unlikely (copy_from_user (&io, arg, sizeof (io)))) {
            printk ("spacc_dev_ioctl::Cannot copy ioctl buffer from user\n");
            return -EFAULT;
         }
// *** this scans for the matching state, note this function LOCKS dev_lock upon success so we must up it here!!!
         ctxid = (int)(scan_state(si, io.state_key));
         if (ctxid < 0) {
            return ctxid;
         }

         states[si].spacc_handle = spacc_open (spacc, io.cipher_mode, io.hash_mode, ctxid, 0, spaccdev_callback, &states[si].comp);
// *** up the dev_lock this is required if scan_state succeeds before we return to the user after we call spacc_open()
         up(&dev_lock);
         if (states[si].spacc_handle < 0) {
            printk ("spacc_dev_ioctl::Failed to open a spacc handle\n");
            return -1;
         }

         // set operation, we append the ICV
         spacc_set_operation (spacc, states[si].spacc_handle, io.encrypt ? OP_ENCRYPT : OP_DECRYPT, io.icv_mode, IP_ICV_APPEND, 0, io.icv_len, 0);

         // we can't set the KEY_EXP bit because that modifies the key context page
         // so if we're in decrypt mode we have rely on the user to feed at least one job through this before binding to new handles

         return 0;

      case ELP_SPACC_USR_PROCESS_IV:
         if (unlikely(states[si].spacc_handle < 0)) {
            printk("spacc_dev_ioctl::SPAcc state not initialized properly\n");
            return -EFAULT;
         }

         // copy struct from user
         if (unlikely (copy_from_user (&io, arg, offsetof(struct elp_spacc_ioctl,ckey)))) {
            printk ("spacc_dev_ioctl::Cannot copy ioctl buffer from user\n");
            return -EFAULT;
         }

         // sanity check key/iv
         if (unlikely(io.civlen  > sizeof(io.civ))) {
             printk("spacc_dev_ioctl::Invalid IV length specified\n");
             return -EFAULT;
          }

         // set cipher IV (this branch is likely since why else would you call this)
         if (likely(io.cipher_mode != CRYPTO_MODE_NULL)) {
            spacc_write_context (spacc, states[si].spacc_handle, SPACC_CRYPTO_OPERATION, NULL, 0, io.civ, io.civlen);
         }

         if (unlikely(!io.srclen)) {
            // we're not processing data just setting the IV so might as well return right away
            return 0;
         }

         // fall through on purpose
         copied = 1;
      case ELP_SPACC_USR_PROCESS:
         // copy struct from user
         if (likely(!copied)) {
            if (unlikely(states[si].spacc_handle < 0)) {
               printk("spacc_dev_ioctl::SPAcc state not initialized properly\n");
               return -EFAULT;
            }

            if (unlikely (copy_from_user (&io, arg, offsetof(struct elp_spacc_ioctl,civ)))) {
               printk ("spacc_dev_ioctl:  Cannot copy ioctl buffer from user\n");
               return -EFAULT;
            }
         }

         // set auxinfo
         spacc_set_auxinfo(spacc, states[si].spacc_handle, io.auxinfo_dir, io.auxinfo_align);

         // default to not including ICV in proclen
         withicv = 0;

         // copy from user
         inlen = io.srclen + (io.encrypt ? 0 : io.icv_len);
         if (io.ivoffset != -1) {
            inlen = max (inlen, io.ivoffset + io.civlen);
         }

         // program spacc
         init_completion(&states[si].comp);

         // length depends on mode, in !aad_copy mode with !cipher we treat the data as all AAD regardless of how the SPAcc is actually programmed
         if (io.encrypt) {
            unsigned icvlen = 0;

            if (io.partial & ELP_SPACC_USR_MSG_END) {
               icvlen = io.icv_len;
            }

            if (io.aad_copy) {
               outlen = io.srclen + icvlen;
            } else {
               outlen = io.srclen - io.pre_aad_len - io.post_aad_len + icvlen;
            }
         } else {
            if (io.aad_copy) {
               outlen = io.srclen;  // we don't subtract icvlen since it's not added by the user
            } else {
               outlen = io.srclen - io.pre_aad_len - io.post_aad_len;
            }
         }

         // add offsets to mapping since we map from the start of the buffer
         outlen += io.dst_offset;

         // sanity check output length
         if (unlikely(io.dstlen < outlen)) {
            printk("spacc_dev_ioctl::dst buffer shorter than output length %d %d\n", io.dstlen, outlen);
            return -1;
         }

         // include ICV when decrypting final block with encrypted hash
         if (io.icv_mode == ICV_HASH_ENCRYPT && !io.encrypt) {
            if (io.partial & ELP_SPACC_USR_MSG_END) {
               withicv = io.icv_len;
            }
         }

#ifdef PERF_MON
t2 = perf_counter();
t1 = t2 - t1;
perf.job_init_total += t1;
perf.job_init_ents++;
t1 = t2;
#endif

         // map user buf
         if (unlikely(get_userbufs(&states[si], io.src, io.dst, io.map_hint))) { // always map at least 1 byte of the destination
            printk("spacc_dev_ioctl::Could not map user buffer into kernel space\n");
            return -EIO;
         }

#ifdef PERF_MON
t2 = perf_counter();
perf.job_map_total += t2 - t1;
perf.job_map_ents++;
t1 = t2;
#endif

         // set MSG flags
         spacc->job[states[si].spacc_handle].ctrl &= ~(SPACC_CTRL_MASK(SPACC_CTRL_MSG_BEGIN) | SPACC_CTRL_MASK(SPACC_CTRL_MSG_END));
         if (io.partial & 1) spacc->job[states[si].spacc_handle].ctrl |= SPACC_CTRL_MASK(SPACC_CTRL_MSG_BEGIN);
         if (io.partial & 2) spacc->job[states[si].spacc_handle].ctrl |= SPACC_CTRL_MASK(SPACC_CTRL_MSG_END);

         if (!spacc->config.is_partial && io.partial != 3) {
            printk("spacc_dev_ioctl::Requesting non one-shot job of a SPAcc core that does not support partial processing...\n");
            zc_release_user_pages(&states[si]);
            io.err = -1;
            if (unlikely (copy_to_user (arg, &io, offsetof(struct elp_spacc_ioctl,civ)))) {
               printk ("spacc_dev_ioctl::Could not copy ioctl structure back to user\n");
               return -EIO;
            }
            return -EINVAL;
         }

         // sync input to DMA
         dma_sync_sg_for_device(states[si].dev, states[si].sg, states[si].readonly_pages, DMA_BIDIRECTIONAL);

//#define SPACC_DEV_NULLNULL
#ifdef SPACC_DEV_NULLNULL
   if (!(io.hash_mode == CRYPTO_MODE_NULL && io.cipher_mode == CRYPTO_MODE_NULL)) {
#endif

retry:
         io.err = spacc_packet_enqueue_ddt (spacc, states[si].spacc_handle, &states[si].src_ddt, &states[si].dst_ddt,
            (uint32_t)(io.srclen + withicv),// proc_sz (PROC_LEN)
            (io.src_offset<<SPACC_OFFSET_SRC_O)|(io.dst_offset<<SPACC_OFFSET_DST_O),     // aad_offset
            io.pre_aad_len | (io.aad_copy ? SPACC_AADCOPY_FLAG : 0),                     // pre_aad_sz + flag for AAD COPY
            io.post_aad_len,                                                             // post_aad_sz
            io.ivoffset == -1 ? 0 : (uint32_t)(io.ivoffset | 0x80000000UL),                        // iv_offset
            SPACC_SW_CTRL_PRIO_HI);
         // CMD FIFO is full so let's wait until at least one job clears up to see if we can program another
         // we hijack the IRQ callbacks during module init so we get informed when other users of the SDK
         // are having jobs coming off the FIFO
         if (unlikely(io.err == CRYPTO_FIFO_FULL)) {
            states[si].blocked++;
            init_completion(&states[si].fifo_full);
            // it's ok to have an interruptible wait here since we haven't programmed the job yet
            if (unlikely(wait_for_completion_timeout(&states[si].fifo_full, jiffies_to_msecs(10)))) {
               complete(&states[si].fifo_full);
               if (states[si].blocked >= 100) {
                  // fail after 100 retries with EBUSY
                  printk("spacc_dev_ioctl::CMD FIFO is too busy\n");
                  zc_release_user_pages(&states[si]);
                  if (unlikely (copy_to_user (arg, &io, offsetof(struct elp_spacc_ioctl,civ)))) {
                     printk ("spacc_dev_ioctl::Could not copy ioctl structure back to user\n");
                     return -EIO;
                  }
                  return -EBUSY;
               }
            }
            goto retry;
         }
         states[si].blocked = 0;

#ifdef PERF_MON
   t2 = perf_counter();
   perf.spacc_write_total += t2 - t1;
   perf.spacc_write_ents++;
   t1 = t2;
#endif


         if (unlikely(io.err != CRYPTO_OK) && io.err != CRYPTO_USED_JB) {
            printk("spacc_dev_ioctl::Could not enqueue packet: %d\n", io.err);
            zc_release_user_pages(&states[si]);
            if (unlikely (copy_to_user (arg, &io, offsetof(struct elp_spacc_ioctl,civ)))) {
               printk ("spacc_dev_ioctl::Could not copy ioctl structure back to user\n");
               return -EIO;
            }
            return -EIO;
         }

// default to un-interruptible, only switch this over for debugging, otherwise bad things could happen since the SPAcc is still active
#ifndef USE_POLL
   #if 0
      if (unlikely (wait_for_completion_interruptible (&states[si].comp))) {
         printk ("spacc_dev_ioctl::Interrupted IOCTL\n");
         zc_release_user_pages(&states[si]);
         return -EIO;
      }
   #else
      wait_for_completion(&states[si].comp);
   #endif
   io.err = spacc_packet_dequeue(spacc, states[si].spacc_handle);
#else
   do {
      int num;
      spacc_pop_packets(spacc, &num);
      io.err = spacc_packet_dequeue (spacc, states[si].spacc_handle);
      udelay(10);
      cpu_relax();
   } while (io.err == CRYPTO_INPROGRESS);
#ifdef PERF_MON
   t2 = perf_counter();
   perf.polling_total += (t2 - t1);
   perf.polling_ents++;
   t1 = t2;
#endif
#endif

#ifdef MAKEAVECTOR
{ int x;
  spacc_job * job;
  job = &spacc->job[states[si].spacc_handle];
  printk("VEC: Final cipher context\n");
  for (x = 0; x < spacc->config.ciph_page_size; x += 4) {
     printk("VEC: %08lx\n", htonl(pdu_io_read32(spacc->regmap + SPACC_CTX_CIPH_KEY + (spacc->config.ciph_page_size>>2) * job->ctx_idx + x)));
  }
  printk("VEC: END\n");
  printk("VEC: Final hash context\n");
  for (x = 0; x < spacc->config.hash_page_size; x += 4) {
     printk("VEC: %08lx\n", htonl(pdu_io_read32(spacc->regmap + SPACC_CTX_HASH_KEY + (spacc->config.hash_page_size>>2) * job->ctx_idx + x)));
  }
  printk("VEC: END\n");
}
#endif

#ifdef SPACC_DEV_NULLNULL
} else {
   io.err = CRYPTO_OK;
}
#endif
         dma_sync_sg_for_cpu(states[si].dev, states[si].dst_sg, states[si].write_pages, DMA_BIDIRECTIONAL);

#ifdef PERF_MON
   t2 = perf_counter();
   perf.deprogram_total += t2 - t1;
   perf.deprogram_ents++;
   t1 = t2;
#endif

         zc_release_user_pages(&states[si]);

#ifdef PERF_MON
   t2 = perf_counter();
   perf.demap_total += t2 - t1;
   perf.demap_ents++;
#endif

         if (likely (io.err == CRYPTO_OK)) {
            return outlen;
         } else {
            // we only copy upto but not including the key/ivs to the user
            // we don't copy this back to the user if there is no error since we can return the size easily enough
            if (unlikely (copy_to_user (arg, &io, offsetof(struct elp_spacc_ioctl,civ)))) {
               printk ("spacc_dev_ioctl::Could not copy ioctl structure back to user\n");
               return -EIO;
            }
            return pdu_error_code(io.err);
         }
   }

   // shouldn't get here
   return -EIO;
}

static long spacc_dev_ioctl (struct file *fp, unsigned int cmd, unsigned long arg_)
{
   long err;
   int  si;

#ifdef PERF_MON
   uint64_t t1;
   t1 = perf_counter();
#endif

   // look up structure associated with this file handle
   si = spacc_fp_lookup (fp);
   if (si == -1) {
      printk ("spacc_dev_ioctl:invalid 'struct file *' pointer\n");
      return -1;
   }

   // lock this state so another thread can't fire a job on this handle
   if (unlikely(down_interruptible(&states[si].lock))) {
      printk("spacc_dev_ioctl::Interrupted trying to lock state\n");
      return -1;
   }

   err = spacc_dev_ioctl2(si, fp, cmd, arg_);
   up(&states[si].lock);

#ifdef PERF_MON
   t1 = perf_counter() - t1;
   perf.ioctl_total += t1;
   perf.ioctl_ents++;
#endif
   return err;
}

static struct file_operations spacc_dev_fops = {
   .owner = THIS_MODULE,
   .open           = spacc_dev_open,
   .release        = spacc_dev_release,
   .unlocked_ioctl = spacc_dev_ioctl,
};

static struct miscdevice spaccdev_device = {
   .minor = MISC_DYNAMIC_MINOR,
   .name = "spaccusr",
   .fops = &spacc_dev_fops,
};


static int __init spacc_dev_init (void)
{
   int err, x, y;
   unsigned long flags;

#ifdef PERF_MON
   memset(&perf, 0, sizeof perf);
#endif
   sema_init(&dev_lock, 1);

   // sort out SPAcc
   spacc_pdev = get_spacc_platdev_by_epn(spacc_epn, 0);
   if (spacc_pdev == NULL) {
      printk ("spacc_dev_init::module not initialized\n");
      return -1;
   }
   spacc = platform_get_drvdata(spacc_pdev);

   err = misc_register (&spaccdev_device);
   if (err) {
      return err;
   }
   // create states, realistically we only need upto 4 threads per context
   state_num = spacc->config.num_ctx * 4;

   states = vmalloc (state_num * sizeof (*states));
   if (!states) {
      printk ("spacc_dev::Out of memory\n");
      misc_deregister (&spaccdev_device);
      return -1;
   }
   memset (states, 0, sizeof (*states) * state_num);

   for (x = 0; x < state_num; x++) {
      init_completion(&(states[x].comp));
      init_completion(&(states[x].fifo_full));
      sema_init(&states[x].lock, 1);

      states[x].spacc_handle = -1;

      // 32 DDT segments allow for upto 16x4=64KB blocks + some slack for misaligned pages/scattergather from user
      if (pdu_ddt_init(&states[x].src_ddt, SPACC_DEFAULT_DDT_LENGTH)) {
         printk("spacc_dev_init::Failed to initialize src_ddt %d\n", x);
         goto cleanup;
      }
      if (pdu_ddt_init(&states[x].dst_ddt, SPACC_DEFAULT_DDT_LENGTH)) {
         printk("spacc_dev_init::Failed to initialize dst_ddt %d\n", x);
         goto cleanup;
      }

      states[x].array_size = DEFAULT_PREALLOC_PAGES;
      states[x].pages      = kzalloc (states[x].array_size * sizeof (struct page *), GFP_KERNEL);
      states[x].sg         = kzalloc (states[x].array_size * sizeof (struct scatterlist), GFP_KERNEL);
      if (states[x].sg == NULL || states[x].pages == NULL) {
         printk("spacc_dev_init::Failed to initialize pages: %d\n", x);
         goto cleanup;
      }
   }

   // hijack handlers so that we can attempt to see if the FIFO is free during IRQs
   PDU_LOCK(&spacc->lock, flags);

#ifdef USE_POLL
   spacc_irq_glbl_disable(spacc);
#endif

   old_notify_jobs          = spacc->spacc_notify_jobs;
   spacc->spacc_notify_jobs = spacc_notify_jobs;

   PDU_UNLOCK(&spacc->lock, flags);

   return 0;
 cleanup:
   printk ("spacc_dev::Could not allocate sg/pages during init out of memory\n");
   for (y = 0; y < x; y++) {
      kfree (states[x].sg);
      kfree (states[x].pages);
      pdu_ddt_free (&states[x].dst_ddt);
      pdu_ddt_free (&states[x].src_ddt);
   }
   misc_deregister (&spaccdev_device);
   vfree(states);
   return -1;
}

static void __exit spacc_dev_exit (void)
{
   int x;
   unsigned long flags;
   printk ("spacc_dev_exit: removing /dev/spaccusr ...\n");

   PDU_LOCK(&spacc->lock, flags);

   spacc->spacc_notify_jobs = old_notify_jobs;

   PDU_UNLOCK(&spacc->lock, flags);

   misc_deregister (&spaccdev_device);
   if (states) {
      for (x = 0; x < state_num; x++) {
         kfree (states[x].sg);
         kfree (states[x].pages);
         pdu_ddt_free (&states[x].dst_ddt);
         pdu_ddt_free (&states[x].src_ddt);
      }
      vfree (states);
   }
}

MODULE_AUTHOR("Synopsys, Inc.");
MODULE_LICENSE ("GPL");

//module_init (spacc_dev_init);
late_initcall(spacc_dev_init)
module_exit (spacc_dev_exit);
module_param (spacc_epn, int, 0);
MODULE_PARM_DESC (spacc_epn, "Set SPAcc EPN number to use");
