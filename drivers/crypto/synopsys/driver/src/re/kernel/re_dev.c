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
 * Copyright (c) 2015 Synopsys, Inc. and/or its affiliates.
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
#include <linux/slab.h>


#include <linux/list.h>

#include <linux/delay.h>

#include "elpre.h"

// this is the maximum number of DDT entries (not including the NULL) for
// either direction of the engine.  Don't set below 8 unless you are really
// constrained for memory.  Don't set above 24 unless you really need to support
// highly fragmented data.
#ifndef RE_DEFAULT_DDT_LENGTH
   #define RE_DEFAULT_DDT_LENGTH 16
#endif

#if RE_DEFAULT_DDT_LENGTH > PDU_MAX_DDT
  #error RE_DEFAULT_DDT_LENGTH cannot be larger than PDU_MAX_DDT
#endif


#ifndef KERNEL
#define KERNEL
#endif
#include "elpreuser.h"
static spacc_device *spacc;
static re_device    *re;

static void (*old_notify_jobs)(struct _spacc_device *spacc);

static struct State
{
   struct file *fp;
   int re_handle, spacc_handle, map, dstlen;

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

/* map an SG to DDT */
static int map_sgs_to_ddt (struct State *s)
{
   unsigned x, y;
   struct scatterlist *sgtmp;

   // map source pages
   x = dma_map_sg (NULL, s->sg, (s->sg==s->dst_sg)?s->used_pages:s->readonly_pages, DMA_BIDIRECTIONAL);
   if (!x) {
      return -1;
   }
   pdu_ddt_reset (&s->src_ddt);
   for_each_sg (s->sg, sgtmp, x, y) {
      if (pdu_ddt_add (&s->src_ddt, (PDU_DMA_ADDR_T) sg_dma_address (sgtmp), sg_dma_len (sgtmp))) {
         dma_unmap_sg(NULL, s->sg, s->readonly_pages, DMA_BIDIRECTIONAL);
         return -1;
      }
   }

   // map dest pages
   if (s->dst_sg != s->sg) {
      x = dma_map_sg (NULL, s->dst_sg, s->write_pages, DMA_BIDIRECTIONAL);
      if (!x) {
         dma_unmap_sg(NULL, s->sg, s->readonly_pages, DMA_BIDIRECTIONAL);
         return -1;
      }
   }
   pdu_ddt_reset (&s->dst_ddt);
   for_each_sg (s->dst_sg, sgtmp, x, y) {
      if (pdu_ddt_add (&s->dst_ddt, (PDU_DMA_ADDR_T) sg_dma_address (sgtmp), sg_dma_len (sgtmp))) {
         dma_unmap_sg(NULL, s->sg, s->readonly_pages, DMA_BIDIRECTIONAL);
         dma_unmap_sg(NULL, s->dst_sg, s->write_pages, DMA_BIDIRECTIONAL);
         return -1;
      }
   }
   return 0;
}

void unmap_sgs(struct State *s)
{
   dma_unmap_sg(NULL, s->sg, (s->sg==s->dst_sg)?s->used_pages:s->readonly_pages, DMA_BIDIRECTIONAL);
   if (s->dst_sg != s->sg) {
      dma_unmap_sg(NULL, s->dst_sg, s->write_pages, DMA_BIDIRECTIONAL);
   }
}

/* offset of buf in it's first page */
#define PAGEOFFSET(buf) ((unsigned long)buf & ~PAGE_MASK)

/* buflen ? (last page - first page + 1) : 0 */
#define PAGECOUNT(buf, buflen) ((buflen) \
   ? ((((unsigned long)(buf + buflen - 1)) >> PAGE_SHIFT) - \
      (((unsigned long)(buf             )) >> PAGE_SHIFT) + 1) \
   : 0)

// default to 8 pages per direction [16 total] which should handle most any 16K block from userspace
#define DEFAULT_PREALLOC_PAGES 16


static int zc_get_userbuf (struct elp_spacc_re_usr_ddt *addr, int *pgcnts, int write,
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

   for (i = x = 0; x < ELP_SPACC_RE_USR_MAX_DDT && addr[x].ptr != NULL; x++) {
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

      page_cache_release (ses->pages[i]);
   }
   ses->used_pages = 0;
}

static int get_userbufs (struct State *ses, struct elp_spacc_re_usr_ddt *src, struct elp_spacc_re_usr_ddt *dst, unsigned hint)
{
   int src_pagecount, dst_pagecount;
   int maptype, rc, x, src_pg_counts[ELP_SPACC_RE_USR_MAX_DDT+1], dst_pg_counts[ELP_SPACC_RE_USR_MAX_DDT+1];

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


   for (x = src_pagecount = 0; x < ELP_SPACC_RE_USR_MAX_DDT && src[x].ptr != NULL; x++) {
      src_pagecount += (src_pg_counts[x] = PAGECOUNT(src[x].ptr, src[x].len));
   }

   for (x = dst_pagecount = 0; x < ELP_SPACC_RE_USR_MAX_DDT && dst[x].ptr != NULL; x++) {
      dst_pagecount += (dst_pg_counts[x] = PAGECOUNT(dst[x].ptr, dst[x].len));
   }

   // are these the same array
   if (hint == RE_MAP_HINT_TEST) {
      maptype = 1;
      for (x = 0; (x < ELP_SPACC_RE_USR_MAX_DDT) && (src[x].ptr || dst[x].ptr); x++) {
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
   } else if (hint == RE_MAP_HINT_USESRC) {
      maptype = 1;
   } else if (hint == RE_MAP_HINT_USEDST) {
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

static void spaccredev_callback (void *re_dev, void *data, uint32_t retcode, uint32_t datalen)
{
   struct State *state = data;
   state->dstlen = datalen;
   complete (&state->comp);
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

static int spacc_re_dev_open (struct inode *in, struct file *fp)
{
   int x;

   // printk("spaccdev::open %p\n", fp);

   down(&dev_lock);

   // find a hole
   for (x = 0; x < state_num; x++) {
      if (states[x].fp == NULL) {
         break;
      }
   }
   if (x == state_num) {
      printk ("spacc_re_dev_open::No more open slots for a new SPAcc handle\n");
      up(&dev_lock);
      return -1;
   }
   // we allocate the spacc handle when a job is fired
   states[x].spacc_handle = -1;
   states[x].re_handle    = -1;
   states[x].fp           = fp;
   states[x].map          = 0;
   states[x].blocked      = 0;

   up(&dev_lock);

   return 0;
}

static int spacc_re_dev_release (struct inode *in, struct file *fp)
{
   int x;

   // printk("spaccdev::close %p\n", fp);
   down(&dev_lock);

   x = spacc_fp_lookup (fp);
   if (x == -1) {
      printk ("spacc_re_dev_close::invalid 'struct file *' pointer\n");
      up(&dev_lock);
      return -1;
   }

   states[x].fp = NULL;
   if (states[x].re_handle >= 0) {
      re_release_context(re, states[x].re_handle);
      states[x].spacc_handle = -1;
      states[x].re_handle    = -1;
   }

   // complete the lock in case there was a pending job
   complete(&states[x].comp);
   states[x].blocked = 0;

   // zero the state key used to pair it
   memset(states[x].state_key, 0, sizeof states[x].state_key);

   up(&dev_lock);

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
          if (states[x].re_handle >= 0 && !memcmp(states[x].state_key, state_lock, 16)) {

             // copy the state key so the master can be freed and we can still pair to this one at least
             memcpy(states[si].state_key, state_lock, 16);

             // return the context handle being used (note that dev_lock IS STILL BEING HELD!!!)
             // this is required to prevent someone from freeing the master between here and our
             // eventual spacc_open call below
             return states[x].spacc_handle;
          }
       }
    }
    up(&dev_lock);
    return -1;
 }


static long spacc_re_dev_ioctl2 (int si, struct file *fp, unsigned int cmd, unsigned long arg_)
{
   void __user *arg = (void __user *) arg_;
   struct elp_spacc_re_ioctl io;
   int id, ctxid;
   uint32_t outlen;


   switch (cmd) {
      case ELP_SPACC_RE_USR_INIT:
         if (unlikely (copy_from_user (&io, arg, sizeof (io)))) {
            printk ("spacc_re_dev_ioctl::Cannot copy ioctl buffer from user\n");
            return -EFAULT;
         }
         states[si].re_handle = re_get_context(re, spaccredev_callback, &states[si]);
         if (states[si].re_handle < 0) {
            printk("spacc_re_dev_ioctl::Out of context handles\n");
            return -1;
         }
         states[si].spacc_handle = re_get_spacc_context(re, states[si].re_handle);
         re_reset_sa(re, states[si].re_handle, io.version);

         return 0;

      case ELP_SPACC_RE_USR_REGISTER:
         if (unlikely (copy_from_user (&io, arg, sizeof (io)))) {
            printk ("spacc_re_dev_ioctl::Cannot copy ioctl buffer from user\n");
            return -EFAULT;
         }

         // copy key
         memcpy(states[si].state_key, io.state_key, 16);

         return 0;

      case ELP_SPACC_RE_USR_BIND:
         if (unlikely (copy_from_user (&io, arg, sizeof (io)))) {
            printk ("spacc_re_dev_ioctl::Cannot copy ioctl buffer from user\n");
            return -EFAULT;
         }
// *** this scans for the matching state, note this function LOCKS dev_lock upon success so we must up it here!!!
         ctxid =  scan_state(si, io.state_key);
         if (ctxid < 0) {
            printk("spacc_re_dev_ioctl::Could not bind since the state_key was not found...\n");
            return ctxid;
         }

         states[si].re_handle    = re_get_context_ex(re, ctxid, spaccredev_callback, &states[si]);
// *** up the dev_lock this is required if scan_state succeeds before we return to the user after we call re_get_context_ex().
         up(&dev_lock);
         if (states[si].re_handle < 0) {
            printk("spacc_re_dev_ioctl::Out of context handles\n");
            return -1;
         }
         states[si].spacc_handle = ctxid;
         return 0;

      case ELP_SPACC_RE_USR_WRITE_CONTEXT:
      case ELP_SPACC_RE_USR_READ_CONTEXT:
         // we're setting either the write or read SA pages
         if (unlikely (copy_from_user (&io, arg, sizeof (io)))) {
            printk ("spacc_re_dev_ioctl::Cannot copy ioctl buffer from user\n");
            return -EFAULT;
         }

         if (cmd == ELP_SPACC_RE_USR_WRITE_CONTEXT) {
             io.err = re_set_next_write(re, states[si].re_handle,
                                     io.civ,             io.civlen,
                                     io.ckey,            io.ckeylen,
                                     io.hkey,            io.hkeylen,
                                     io.params,          2,
                                     io.sequence_number, 8);
         } else {
             io.err = re_set_next_read(re, states[si].re_handle,
                                     io.civ,             io.civlen,
                                     io.ckey,            io.ckeylen,
                                     io.hkey,            io.hkeylen,
                                     io.params,          2,
                                     io.sequence_number, 8);
         }

         if (io.err) {
            printk("spacc_re_dev_ioctl::Error writing context! %d, %u\n", io.err, cmd);
            if (unlikely (copy_to_user (arg, &io, offsetof(struct elp_spacc_re_ioctl,civ)))) {
               printk ("spacc_re_dev_ioctl::Could not copy ioctl structure back to user\n");
               return -EIO;
            }
            return -1;
         }
         return 0;

      case ELP_SPACC_RE_USR_DATA_OP:
//printk("Using RE handle: %d, SPAcc handle: %d\n", states[si].re_handle, states[si].spacc_handle);

         // we're issuing either an write [encrypt] or read [decrypt] operation
         if (unlikely (copy_from_user (&io, arg, offsetof(struct elp_spacc_re_ioctl,civ)))) {
            printk ("spacc_re_dev_ioctl::Cannot copy ioctl buffer from user\n");
            return -EFAULT;
         }

         // map data
         if (unlikely(get_userbufs(&states[si], io.src, io.dst, io.map_hint))) { // always map at least 1 byte of the destination
            printk("spacc_dev_ioctl::Could not map user buffer into kernel space\n");
            return -EIO;
         }

         // sync input to DMA
         dma_sync_sg_for_device(NULL, states[si].sg, states[si].readonly_pages, DMA_BIDIRECTIONAL);

         // program job
         init_completion (&states[si].comp);
retry:
         io.err = re_start_operation_ex(re, states[si].re_handle, &states[si].src_ddt, &states[si].dst_ddt, io.src_offset, io.dst_offset, io.srclen, io.cmd);

         // FIFO FULL?
         if (io.err == CRYPTO_FIFO_FULL) {
            states[si].blocked++;
            init_completion(&states[si].fifo_full);
            // it's ok to have an interruptible wait here since we haven't programmed the job yet
            if (unlikely(wait_for_completion_timeout(&states[si].fifo_full, jiffies_to_msecs(10)))) {
               complete(&states[si].fifo_full);
               if (states[si].blocked >= 100) {
                  // fail after 100 retries with EBUSY
                  printk("spacc_re_dev_ioctl::CMD FIFO is too busy\n");
                  zc_release_user_pages(&states[si]);
                  if (unlikely (copy_to_user (arg, &io, offsetof(struct elp_spacc_re_ioctl,civ)))) {
                     printk ("spacc_re_dev_ioctl::Could not copy ioctl structure back to user\n");
                     return -EIO;
                  }
                  return -EBUSY;
               }
            }
            goto retry;
         }
         states[si].blocked = 0;

         if (io.err < 0) {
            if (unlikely (copy_to_user (arg, &io, offsetof(struct elp_spacc_re_ioctl,civ)))) {
               printk ("spacc_re_dev_ioctl::Could not copy ioctl structure back to user\n");
               return -EIO;
            }
            return -1;
         }

         // wait for done
         wait_for_completion(&states[si].comp);

         // done
         outlen = io.dstlen;
         io.err = re_finish_operation(re, states[si].re_handle, &outlen, &id);

         // sync for CPU
         dma_sync_sg_for_cpu (NULL, states[si].dst_sg, states[si].write_pages, DMA_BIDIRECTIONAL);
         zc_release_user_pages(&states[si]);

         if (likely (io.err == CRYPTO_OK)) {
            return outlen;
         } else {
            // we only copy upto but not including the key/ivs to the user
            // we don't copy this back to the user if there is no error since we can return the size easily enough
            if (unlikely (copy_to_user (arg, &io, offsetof(struct elp_spacc_re_ioctl,civ)))) {
               printk ("spacc_re_dev_ioctl::Could not copy ioctl structure back to user\n");
               return -EIO;
            }
            return -1;
         }
   }

   // shouldn't get here
   return -EIO;
}

static long spacc_re_dev_ioctl (struct file *fp, unsigned int cmd, unsigned long arg_)
{
   long err;
   int  si;


//printk("re_ioctl::%d %08lx\n", cmd, arg_);

   // look up structure associated with this file handle
   si = spacc_fp_lookup (fp);
   if (si == -1) {
      printk ("spacc_re_dev_ioctl:invalid 'struct file *' pointer\n");
      return -1;
   }

   // lock this state so another thread can't fire a job on this handle
   if (unlikely(down_interruptible(&states[si].lock))) {
      printk("spacc_re_dev_ioctl::Interrupted trying to lock state\n");
      return -1;
   }

   err = spacc_re_dev_ioctl2(si, fp, cmd, arg_);
   up(&states[si].lock);

   return err;
}

static struct file_operations spacc_re_dev_fops = {
   .owner          = THIS_MODULE,
   .open           = spacc_re_dev_open,
   .release        = spacc_re_dev_release,
   .unlocked_ioctl = spacc_re_dev_ioctl,
};

static struct miscdevice spaccdev_device = {
   .minor = MISC_DYNAMIC_MINOR,
   .name = "spaccreusr",
   .fops = &spacc_re_dev_fops,
};


static int __init spacc_re_dev_init (void)
{
   int err, x, y;
   unsigned long flags;

   sema_init(&dev_lock, 1);

   // sort out SPAcc
  re = re_get_device();
  if (!re) {
     printk("Invalid RE device context\n");
     return -1;
  }
  spacc = re->spacc;
  if (spacc == NULL) {
     printk("re_dev_init::RE module not initialized\n");
     return -1;
  }

   err = misc_register (&spaccdev_device);
   if (err) {
      return err;
   }
   // create states, allow upto 4 threads per context page
   state_num = spacc->config.num_ctx * 4;

   states    = vmalloc (state_num * sizeof (*states));
   if (!states) {
      printk ("spacc_re_dev::Out of memory\n");
      misc_deregister (&spaccdev_device);
      return -1;
   }
   memset (states, 0, sizeof (*states) * state_num);

   for (x = 0; x < state_num; x++) {
      init_completion(&(states[x].comp));
      init_completion(&(states[x].fifo_full));
      sema_init(&states[x].lock, 1);

      states[x].spacc_handle = -1;

      // 16 DDT segments allow for upto 4x4=16KB blocks + some slack for misaligned pages/scattergather from users (SSL/TLS has a limit of 16K records...)
      if (pdu_ddt_init(&states[x].src_ddt, RE_DEFAULT_DDT_LENGTH)) {
         printk("re_dev_init::Failed to initialize src_ddt %d\n", x);
         goto cleanup;
      }
      if (pdu_ddt_init(&states[x].dst_ddt, RE_DEFAULT_DDT_LENGTH)) {
         printk("re_dev_init::Failed to initialize dst_ddt %d\n", x);
         goto cleanup;
      }

      states[x].array_size = DEFAULT_PREALLOC_PAGES;
      states[x].pages      = kzalloc (states[x].array_size * sizeof (struct page *), GFP_KERNEL);
      states[x].sg         = kzalloc (states[x].array_size * sizeof (struct scatterlist), GFP_KERNEL);
      if (states[x].sg == NULL || states[x].pages == NULL) {
         printk (KERN_DEBUG "Memory error\n");
         goto cleanup;
      }
   }

   // hijack handlers so that we can attempt to see if the FIFO is free during IRQs
   PDU_LOCK(&spacc->lock, flags);

   old_notify_jobs          = spacc->spacc_notify_jobs;
   spacc->spacc_notify_jobs = spacc_notify_jobs;

   PDU_UNLOCK(&spacc->lock, flags);

   return 0;
 cleanup:
   printk ("spacc_re_dev::Could not allocate sg/pages during init out of memory\n");
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

static void __exit spacc_re_dev_exit (void)
{
   int x;
   unsigned long flags;
   printk ("spacc_re_dev_exit: removing /dev/spaccusr ...\n");

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

module_init (spacc_re_dev_init);
module_exit (spacc_re_dev_exit);
