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
 * Copyright (c) 2011-2015 Synopsys, Inc. and/or its affiliates.
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
#include <linux/miscdevice.h>
#include <linux/ratelimit.h>
#include <linux/crypto.h>
#include <linux/io.h>

#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <asm/uaccess.h>
#include <asm/param.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/device.h>

#include "elpspaccdrv.h"
#include "elpmpm.h"

#define CHAN_MAX 64


static int cb_err;

static void mpm_chain_cb(void *mpm_dev, void *data)
{
   complete(data);
}

static int mpm_dev_open(struct inode *in, struct file *fp)
{
   return 0;
}

static int mpm_dev_release(struct inode *in, struct file *fp)
{
   return 0;
}

static ssize_t mpm_dev_read(struct file *fp, char __user *up, size_t len, loff_t *off)
{
   return -1;
}


/* users write their job to this, we do jobs then just silently return all is ok, format of input is

byte #    |   size    | meaning
   0      |    1      | cipher (enum from spacchw.h)
   1      |    1      | cipher mode (enum from spacchw.h)
   2      |    1      | cipher ksize
   3      |    1      | hash   (enum from spacchw.h)
   4      |    1      | hash  mode (enum from spacchw.h)
   5      |    1      | hash ksize
   6      |    2      | payload size

*/

static ssize_t mpm_dev_write(struct file *fp, const char __user *up, size_t len, loff_t *off)
{
   int mpm_keys[2];
   unsigned char cmdbuf[16], key[32], iv[32];
   uint32_t      jobs, payload_size, hash, cipher, hashmode, ciphermode, cipherksize, hashksize, chain_size, epn, runs;

   volatile int job_cnt;
   int pdus[CHAN_MAX], keys[CHAN_MAX], x, ddts, err;
   uint32_t ctrl;

   mpm_device *mpm;
   dma_addr_t srcphys, dstphys;
   unsigned char *srcvirt, *dstvirt;
   pdu_ddt       src, dst;

   struct completion comp;

   if (*off || len < 13 || len > sizeof cmdbuf) {
     printk("Must write at least 13 bytes to mpmtraffic device\n");
     return -EINVAL;
   }
   err = -1;
   if (copy_from_user(cmdbuf, up, len) != 0) {
      return -EFAULT;
   }
   cipher       = cmdbuf[0];
   ciphermode   = cmdbuf[1];
   cipherksize  = cmdbuf[2];
   hash         = cmdbuf[3];
   hashmode     = cmdbuf[4];
   hashksize    = cmdbuf[5];
   payload_size = (cmdbuf[6]<<8)|cmdbuf[7];
   epn          = (cmdbuf[8]<<8)|cmdbuf[9];
   runs         = (cmdbuf[10]<<16)|(cmdbuf[11]<<8)|cmdbuf[12];

   mpm = mpm_get_device_by_num(epn);

   if (len == 14) {
      chain_size = cmdbuf[13];
      if (chain_size == 0 || chain_size > mpm->config.no_chain_links) {
         chain_size = mpm->config.no_chain_links;
      }
   } else {
      chain_size = mpm->config.no_chain_links;
   }

   if (payload_size > 16384) {
      printk("Sanity check failed on payload size: %lu\n", (unsigned long)payload_size);
      return -1;
   }
   if (!payload_size) {
      payload_size = 1;
   }

   ddts = 0;
   srcvirt = NULL;
   dstvirt = NULL;
   for (x = 0; x < CHAN_MAX; x++) {
      keys[x] = -1;
   }

   mpm_req_spacc_ctx(mpm, 2, mpm_keys);

   // allocate a buffer
   srcvirt = pdu_dma_alloc(payload_size, &srcphys);
   dstvirt = pdu_dma_alloc(payload_size, &dstphys);
   if (!srcvirt || !dstvirt) { printk("Could not allocate DMA memory\n"); goto ERR; }

   // make up DDT
   pdu_ddt_init(&src, 1);
   pdu_ddt_add(&src, srcphys, payload_size);
   pdu_ddt_init(&dst, 1);
   pdu_ddt_add(&dst, dstphys, payload_size);
   ddts = 1;

   // allocate pdus/keys
   for (x = 0; x < 1; x++) {
      keys[x] = mpm_alloc_key(mpm);
      if (keys[x] < 0) {
          printk("mpm_thread::Cannot allocate KEY buffer\n");
          goto ERR;
      }
      mpm_set_key(mpm, keys[x], cipherksize, cipherksize, hashksize, hashksize, 0, 0, key, key);
   }

   ctrl = MPM_SET_CTRL(mpm, cipher, hash, ciphermode, hashmode, 1, 0, 0, 0, 0);

   cb_err = 0;
   for (jobs = 0; jobs < (runs/chain_size); jobs++) {
      init_completion(&comp);
      for (x = 0; x <chain_size; x++) {
          job_cnt += 1;
          pdus[x] = mpm_alloc_pdu(mpm);
          err = mpm_insert_pdu(mpm,
             pdus[x], keys[0], 0, NULL, NULL,
             &src, &dst, 0, 0, payload_size, 0, 0, 0, 0, ctrl,
             iv, iv);
          if (err < 0) {
             printk("mpm_thread::Cannot insert pdu\n");
             goto ERR;
          }
      }
      err = mpm_enqueue_chain(mpm, mpm_chain_cb, &comp);
      if (err < 0) {
         printk("mpm_thread: Cannot enqueue chain\n");
         goto ERR;
      }
      mpm_kernel_schedule_tasklet_by_num(epn);
      if (unlikely(wait_for_completion_interruptible(&comp))) {
         printk("mpmtraffic:  Aborted\n");
         err = -1;
         goto ERR;
      }
   }

   err = !cb_err ? len : -1;
ERR:
   mpm_free_spacc_ctx(mpm, 2, mpm_keys);
   for (x = 0; x < 1; x++) {
      if (keys[x] != -1) { mpm_free_key(mpm, keys[x]); }
   }
   if (ddts) {
      pdu_ddt_free(&src);
      pdu_ddt_free(&dst);
   }
   if (srcvirt) { pdu_dma_free(payload_size, srcvirt, srcphys); }
   if (dstvirt) { pdu_dma_free(payload_size, dstvirt, dstphys); }
   return err;
}

struct file_operations mpm_dev_fops = {
  .owner   = THIS_MODULE,
  .open    = mpm_dev_open,
  .release = mpm_dev_release,
  .write   = mpm_dev_write,
  .read    = mpm_dev_read,
};

static struct miscdevice mpm_dev_device = {
   .minor = MISC_DYNAMIC_MINOR,
   .name = "spaccmpm",
   .fops = &mpm_dev_fops,
};

static int __init mpm_mod_init (void)
{
   return misc_register(&mpm_dev_device);
}

static void __exit mpm_mod_exit (void)
{
   misc_deregister(&mpm_dev_device);
}

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init (mpm_mod_init);
module_exit (mpm_mod_exit);
