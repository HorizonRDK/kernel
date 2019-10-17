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
#include <linux/dma-mapping.h>
#include <linux/completion.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/dmapool.h>

#include <linux/list.h>

#include "elpkep.h"
#ifndef KERNEL
   #define KERNEL
#endif
#include "elpkepuser.h"

static spacc_device *spacc;
static kep_device   *kep;

enum {
   KEP_DEV_RAW=0,
   KEP_DEV_WORKING,
};

static struct State {
   struct file *fp;
   int spacc_handle,
       state;

   PDU_DMA_ADDR_T mst_src_phys,  mst_dst_phys;
   void          *mst_src_virt, *mst_dst_virt;

   pdu_ddt src_ddt, dst_ddt;

   struct completion comp;

} *states = NULL;

static int state_num;

static void kepdev_callback(void *kepdev, void *state)
{
   struct State *s = state;
   complete(&(s->comp));
}

static int kep_fp_lookup(struct file *fp)
{
   int x;
   for (x = 0; x < state_num; x++) {
      if (states[x].fp == fp) {
         return x;
      }
   }
   return -1;
}

static int kep_dev_open(struct inode *in, struct file *fp)
{
   int x;

   // find a hole
   for (x = 0; x < state_num; x++) {
      if (states[x].fp == NULL) {
         break;
      }
   }
   if (x == state_num) {
      printk("kep_dev_open::No more open slots for a new RE handle\n");
      return -1;
   }
   states[x].fp    = fp;
   states[x].state = KEP_DEV_RAW;
   return 0;
}

static int kep_dev_release(struct inode *in, struct file *fp)
{
   int x;

   x = kep_fp_lookup(fp);
   if (x == -1) {
      printk("kep_dev_close::invalid 'struct file *' pointer\n");
      return -1;
   }
   states[x].fp = NULL;
   return 0;
}

static ssize_t kep_dev_read(struct file *fp, char __user *up, size_t len, loff_t *off)
{
   int si, err;
   uint32_t cplen;

   si = kep_fp_lookup(fp);
   if (si == -1) {
      printk("kep_dev_read::invalid 'struct file *' pointer\n");
      return -1;
   }

   if (*off != 0) {
      printk("kep_dev_read::Reads must be from offset 0\n");
      return -1;
   }

   if (len < states[si].dst_ddt.len) {
      printk("kep_dev_read::Invalid buffer size, needed %lu\n", states[si].dst_ddt.len);
      return -1;
   }

   switch (states[si].state) {
      case KEP_DEV_RAW:
         printk("kep_dev_read::Cannot read when in RAW mode\n");
         return -1;
      case KEP_DEV_WORKING:
         // anything to read?
         wait_for_completion(&(states[si].comp));
         if ((err = kep_done(kep, states[si].spacc_handle)) != CRYPTO_OK) {
            printk("kep_dev_read::Engine timed out\n");
            kep_close(kep, states[si].spacc_handle);
            return -1;
         }
         kep_close(kep, states[si].spacc_handle);
         cplen = copy_to_user(up, states[si].mst_dst_virt, states[si].dst_ddt.len);
         if (cplen) {
            printk("kep_dev_read::Error copying to user buffer, returned %lu bytes written\n", states[si].dst_ddt.len);
            err = -1;
         }
         return (err == 0) ? states[si].dst_ddt.len : -1;
   }
   return -1;
}

static ssize_t kep_dev_write(struct file *fp, const char __user *up, size_t len, loff_t *off)
{
   int si, err;
   unsigned char tmp[256], *buf, *ms, *ss, *cs, *lb;
   uint32_t opcode, options, x, buflen, mslen, sslen, cslen, lblen, outlen;

   si = kep_fp_lookup(fp);
   if (si == -1) {
      printk("kep_dev_write::invalid 'struct file *' pointer\n");
      return -1;
   }

   if (len > sizeof(tmp)) {
      printk("kep_dev_write::Invalid size %zd passed\n", len);
      return -1;
   }

   if (*off) {
      printk("kep_dev_write::offset 'off' must be zero! %llu\n", (unsigned long long)*off);
      return -1;
   }

   buflen = copy_from_user(buf = tmp, up, len);
   if (buflen && buflen != len) {
      printk("kep_dev_write::Could not copy all user data: %zu of %lu\n", buflen, (unsigned long)len);
      return -1;
   }

   // if the copy went ok it returns 0 so lets assign it len ...
   buflen = len;

   if (buflen < 8) {
      printk("kep_dev_write::Buffer must be at least 8 bytes long\n");
      return -1;
   }

   // packet is formated as
   // 1 byte - opcode
   // 1 byte - options
   // 1 byte - len of master secret
   // 1 byte - len of server secret
   // 1 byte - len of client secret
   // 1 byte - len of label
   // n bytes - master
   // n bytes - server
   // n bytes - client
   // n bytes - label
   opcode  = *buf++;
   options = *buf++;
   mslen   = *buf++;
   sslen   = *buf++;
   cslen   = *buf++;
   lblen   = *buf++;
   outlen  = *buf++; outlen = (outlen << 8) | *buf++;

   if ((mslen + sslen + cslen + lblen + 8) > buflen) {
      printk("kep_dev_write::Invalid buffer\n");
      return -1;
   }

   ms = buf;
   ss = ms + mslen;
   cs = ss + sslen;
   lb = cs + cslen;

#if 0
   printk("ms[%d] = \n", mslen); for (x = 0; x < mslen; ) { printk("%02x", ms[x]); if (!(++x & 15)) printk("\n"); }
   printk("ss[%d] = \n", sslen); for (x = 0; x < sslen; ) { printk("%02x", ss[x]); if (!(++x & 15)) printk("\n"); }
   printk("cs[%d] = \n", cslen); for (x = 0; x < cslen; ) { printk("%02x", cs[x]); if (!(++x & 15)) printk("\n"); }
   printk("lb[%d] = \n", lblen); for (x = 0; x < lblen; ) { printk("%02x", lb[x]); if (!(++x & 15)) printk("\n"); }
   printk("opcode, options, outlen == %d, %d, %d\n", opcode, options, outlen);
#endif


   states[si].spacc_handle  = kep_open(kep, opcode, options, kepdev_callback, &states[si]);
   if (states[si].spacc_handle < 0) {
      printk("kep_dev_write::Error opening KEP handle\n");
      return -1;
   }

   x = 0;
   switch (opcode) {
      case KEP_SSL3_KEYGEN:
         memcpy(states[si].mst_src_virt + x, ms, 48); x += 48;
         memcpy(states[si].mst_src_virt + x, ss, 32); x += 32;
         memcpy(states[si].mst_src_virt + x, cs, 32); x += 32;
         break;
      case KEP_SSL3_SIGN:
         memcpy(states[si].mst_src_virt + x, ss, sslen); x += sslen;
         memcpy(states[si].mst_src_virt + x, ms, 48); x += 48;
         break;
      case KEP_TLS_PRF:
         memcpy(states[si].mst_src_virt + x, lb, lblen); x += lblen;
         memcpy(states[si].mst_src_virt + x, ss, sslen); x += sslen;
         kep_load_keys(kep, states[si].spacc_handle, ms, 24, ms + 24, 24);
         break;
      case KEP_TLS_SIGN:
         memcpy(states[si].mst_src_virt + x, ss, sslen); x += sslen;
         kep_load_keys(kep, states[si].spacc_handle, ms, 24, ms + 24, 24);
         break;
      case KEP_TLS2_PRF:
         memcpy(states[si].mst_src_virt + x, lb, lblen); x += lblen;
         memcpy(states[si].mst_src_virt + x, ss, sslen); x += sslen;
         kep_load_keys(kep, states[si].spacc_handle, ms, mslen, NULL, 0);
         break;
      case KEP_TLS2_SIGN:
         memcpy(states[si].mst_src_virt + x, ss, sslen); x += sslen;
         kep_load_keys(kep, states[si].spacc_handle, ms, mslen, NULL, 0);
         break;
   }

#if 0
{
   int y;
   printk("kep_dev_write::buffer[%d] ==\n", x);
   for (y = 0; y < x; ) { printk("%02x", ((unsigned char *)states[si].mst_src_virt)[y]); if (!(++y & 15)) printk("\n"); }
   printk("\n");
}
#endif

   states[si].src_ddt.len = x;
   states[si].dst_ddt.len = outlen;
   init_completion(&states[si].comp);
   err = kep_go(kep, &states[si].src_ddt, &states[si].dst_ddt, states[si].spacc_handle);
   if (err < 0) {
      printk("kep_dev_write:: Error starting KEP job\n");
      return -1;
   }
   states[si].state = KEP_DEV_WORKING;
   return len;
}


struct file_operations kep_dev_fops = {
  .owner   = THIS_MODULE,
  .open    = kep_dev_open,
  .release = kep_dev_release,

  .read    = kep_dev_read,
  .write   = kep_dev_write,
};

static struct miscdevice kepdev_device = {
   .minor = MISC_DYNAMIC_MINOR,
   .name = "spacckep",
   .fops = &kep_dev_fops,
};

static struct dma_pool *pool;

static int __init kep_dev_init (void)
{
  int err, x, y;

  // sort out SPAcc
  kep = kep_get_device();
  if (!kep) {
     printk("Invalid KEP device context\n");
     return -1;
  }
  spacc = kep->spacc;
  if (!spacc) {
     printk("kep_dev_init::KEP module not initialized\n");
     return -1;
  }

  pool = dma_pool_create("kepdev", NULL, KEP_MAX_SIZE, 4, 0);
  err =  misc_register(&kepdev_device);
  if (err) {
     dma_pool_destroy(pool);
     return err;
  }

  // create states
  state_num = 8;
  states = vmalloc(state_num * sizeof(*states));
  if (!states) {
     printk("kep_dev::Out of memory\n");
     misc_deregister(&kepdev_device);
     dma_pool_destroy(pool);
     return -1;
  }
  memset(states, 0, sizeof(*states) * state_num);

  // allocate phys mem for each possible handle and pre-map the dest DDT/etc
  for (x = 0; x < state_num; x++) {
     states[x].mst_src_virt = dma_pool_alloc(pool, GFP_KERNEL, &states[x].mst_src_phys);
     states[x].mst_dst_virt = dma_pool_alloc(pool, GFP_KERNEL, &states[x].mst_dst_phys);

     if (!states[x].mst_src_virt || !states[x].mst_dst_virt) {
        if (states[x].mst_src_virt) {
           dma_pool_free(pool, states[x].mst_src_virt, states[x].mst_src_phys);
        } else {
           dma_pool_free(pool, states[x].mst_dst_virt, states[x].mst_dst_phys);
        }
        goto cleanup;
     }

     err = pdu_ddt_init(&states[x].src_ddt, 1);
     if (err) {
        dma_pool_free(pool, states[x].mst_src_virt, states[x].mst_src_phys);
        dma_pool_free(pool, states[x].mst_dst_virt, states[x].mst_dst_phys);
        goto cleanup;
     }
     pdu_ddt_add(&states[x].src_ddt, states[x].mst_src_phys, KEP_MAX_SIZE);

     err = pdu_ddt_init(&states[x].dst_ddt, 1);
     if (err) {
        pdu_ddt_free(&states[x].src_ddt);
        dma_pool_free(pool, states[x].mst_src_virt, states[x].mst_src_phys);
        dma_pool_free(pool, states[x].mst_dst_virt, states[x].mst_dst_phys);
        goto cleanup;
     }
     pdu_ddt_add(&states[x].dst_ddt, states[x].mst_dst_phys, KEP_MAX_SIZE);
  }
  return 0;
cleanup:
   printk("kep_dev:: Cannot allocate master buffers for states\n");
   for (y = 0; y < x; y++) {
     pdu_ddt_free (&states[y].dst_ddt);
     pdu_ddt_free (&states[y].src_ddt);

     dma_pool_free(pool, states[y].mst_src_virt, states[y].mst_src_phys);
     dma_pool_free(pool, states[y].mst_dst_virt, states[y].mst_dst_phys);
   }
   misc_deregister(&kepdev_device);
   dma_pool_destroy(pool);
   return -1;

}

static void __exit kep_dev_exit (void)
{
  int x;
  printk ("kep_dev_exit: removing /dev/spacckep\n");
  if (states) {
     for (x = 0; x < state_num; x++) {
        pdu_ddt_free (&states[x].dst_ddt);
        pdu_ddt_free (&states[x].src_ddt);

        if (states[x].mst_src_virt) { dma_pool_free(pool, states[x].mst_src_virt, states[x].mst_src_phys); }
        if (states[x].mst_dst_virt) { dma_pool_free(pool, states[x].mst_dst_virt, states[x].mst_dst_phys); }
     }
     vfree(states);
  }
  dma_pool_destroy(pool);
  misc_deregister(&kepdev_device);
}

MODULE_AUTHOR("Synopsys, Inc.");
MODULE_LICENSE ("GPL");

module_init (kep_dev_init);
module_exit (kep_dev_exit);
