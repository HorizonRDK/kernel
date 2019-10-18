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
 * Copyright (c) 2012-2015 Synopsys, Inc. and/or its affiliates.
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
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/ratelimit.h>
#include <linux/crypto.h>
#include <linux/io.h>

#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/param.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/random.h>

#include "elpspaccdrv.h"

#define CHAN_MAX 64

static int set_fifo = 0;

static struct timer_list timer;

static struct completion comp, fifofullcomp;

static int cb_err, spacc_chain_size;

static void sprof_job_cb(void *sprof_dev, void *data)
{
   if (data) {
      volatile int *p = data;
      p[0] -= 1;
      if (!p[0]) {
         complete(&comp);
      }
      complete(&fifofullcomp);
   }
}

static int sprof_dev_open(struct inode *in, struct file *fp)
{
   return 0;
}

static int sprof_dev_release(struct inode *in, struct file *fp)
{
   return 0;
}

static ssize_t sprof_dev_read(struct file *fp, char __user *up, size_t len, loff_t *off)
{
   return -1;
}

#define M(x, y) (((x)<<16)|y)
uint32_t cipher_to_spacc_cipher(uint32_t cipher, uint32_t mode, uint32_t keysize)
{
   switch (M(cipher, mode)) {
   case M(C_NULL, CM_ECB): return CRYPTO_MODE_NULL;
   case M(C_RC4, CM_ECB): return keysize==5?CRYPTO_MODE_RC4_40:CRYPTO_MODE_RC4_128;
   case M(C_AES, CM_ECB): return CRYPTO_MODE_AES_ECB;
   case M(C_AES, CM_CBC): return CRYPTO_MODE_AES_CBC;
   case M(C_AES, CM_CTR): return CRYPTO_MODE_AES_CTR;
   case M(C_AES, CM_CCM): return CRYPTO_MODE_AES_CCM;
   case M(C_AES, CM_GCM): return CRYPTO_MODE_AES_GCM;
   case M(C_AES, CM_F8): return CRYPTO_MODE_AES_F8;
   case M(C_AES, CM_XTS): return CRYPTO_MODE_AES_XTS;
   case M(C_AES, CM_CFB): return CRYPTO_MODE_AES_CFB;
   case M(C_AES, CM_OFB): return CRYPTO_MODE_AES_OFB;
   case M(C_DES, CM_ECB): return keysize==8?CRYPTO_MODE_DES_ECB:CRYPTO_MODE_3DES_ECB;
   case M(C_DES, CM_CBC): return keysize==8?CRYPTO_MODE_DES_CBC:CRYPTO_MODE_3DES_CBC;
   case M(C_KASUMI, CM_ECB): return CRYPTO_MODE_KASUMI_ECB;
   case M(C_KASUMI, CM_F8): return CRYPTO_MODE_KASUMI_F8;
   case M(C_SNOW3G_UEA2, CM_ECB): return CRYPTO_MODE_SNOW3G_UEA2;
   case M(C_ZUC_UEA3, CM_ECB): return CRYPTO_MODE_ZUC_UEA3;
   }
   return 0;
}

uint32_t hash_to_spacc_hash(uint32_t hash, uint32_t mode)
{
   switch (M(hash, mode)) {
   case M(H_NULL, HM_RAW):      return CRYPTO_MODE_NULL;
   case M(H_SHA1, HM_RAW):     return CRYPTO_MODE_HASH_SHA1;
   case M(H_SHA1, HM_HMAC):    return CRYPTO_MODE_HMAC_SHA1;
   case M(H_SHA224, HM_RAW):     return CRYPTO_MODE_HASH_SHA1;
   case M(H_SHA224, HM_HMAC):    return CRYPTO_MODE_HMAC_SHA1;
   case M(H_SHA256, HM_RAW):     return CRYPTO_MODE_HASH_SHA1;
   case M(H_SHA256, HM_HMAC):    return CRYPTO_MODE_HMAC_SHA1;
   case M(H_SHA384, HM_RAW):     return CRYPTO_MODE_HASH_SHA1;
   case M(H_SHA384, HM_HMAC):    return CRYPTO_MODE_HMAC_SHA1;
   case M(H_SHA512, HM_RAW):     return CRYPTO_MODE_HASH_SHA512;
   case M(H_SHA512, HM_HMAC):    return CRYPTO_MODE_HMAC_SHA512;
   case M(H_SHA512_224, HM_RAW):   return CRYPTO_MODE_HASH_SHA512_224;
   case M(H_SHA512_224, HM_HMAC):  return CRYPTO_MODE_HMAC_SHA512_224;
   case M(H_SHA512_256, HM_RAW):   return CRYPTO_MODE_HASH_SHA512_256;
   case M(H_SHA512_256, HM_HMAC):  return CRYPTO_MODE_HMAC_SHA512_256;
   case M(H_XCBC, HM_RAW):         return CRYPTO_MODE_MAC_XCBC;
   case M(H_CMAC, HM_RAW):         return CRYPTO_MODE_MAC_CMAC;
   case M(H_KF9, HM_RAW):    return CRYPTO_MODE_MAC_KASUMI_F9;
   case M(H_SNOW3G_UIA2, HM_RAW):  return CRYPTO_MODE_MAC_SNOW3G_UIA2;
   case M(H_ZUC_UIA3, HM_RAW):     return CRYPTO_MODE_MAC_ZUC_UIA3;
   case M(H_MD5, HM_SSLMAC):       return CRYPTO_MODE_SSLMAC_MD5;
   case M(H_SHA1, HM_SSLMAC):      return CRYPTO_MODE_SSLMAC_SHA1;
   }
   return 0;
}
#undef M



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

static ssize_t sprof_dev_write(struct file *fp, const char __user *up, size_t len, loff_t *off)
{
   unsigned char cmdbuf[16], key[128], iv[128];
   uint32_t      epn, payload_size, hash, cipher, hashmode, ciphermode, cipherksize, hashksize, chain_size, spacc_cipher, spacc_hash, runs;
   int           ddts, x, y, err, jobs[128];

   volatile int job_cnt;

   spacc_device *spacc;
   dma_addr_t srcphys, dstphys;
   unsigned char *srcvirt, *dstvirt;
   pdu_ddt       src, dst;

   if (*off || len < 16 || len > sizeof cmdbuf) {
      printk("Must write at least 16 bytes to mpmtraffic device\n");
      return -EINVAL;
   }

   if (copy_from_user(cmdbuf, up, len) != 0) {
      return -EFAULT;
   }

   err = -1;
   cipher       = cmdbuf[0];
   ciphermode   = cmdbuf[1];
   cipherksize  = cmdbuf[2];
   hash         = cmdbuf[3];
   hashmode     = cmdbuf[4];
   hashksize    = cmdbuf[5];
   payload_size = (cmdbuf[6]<<8)|cmdbuf[7];
   epn          = (cmdbuf[8]<<8)|cmdbuf[9];
   runs         = (cmdbuf[10]<<16)|(cmdbuf[11]<<8)|cmdbuf[12];

   spacc = get_spacc_device_by_epn(epn, 0);
   if (!spacc) {
      printk("Can't find spacc by EPN 0x%04x\n", epn);
      return -1;
   }
   chain_size   = spacc->config.stat_fifo_depth*2;

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
   for (x = 0; x < 128; x++) {
      jobs[x] = -1;
   }

   if (set_fifo != 0) { spacc->config.stat_fifo_depth = set_fifo; }

   // allocate a buffer
   srcvirt = dma_alloc_coherent(NULL, payload_size, &srcphys, GFP_ATOMIC);
   dstvirt = dma_alloc_coherent(NULL, payload_size, &dstphys, GFP_ATOMIC);
   if (!srcvirt || !dstvirt) { printk("Could not allocate DMA memory\n"); goto ERR; }

   // make up DDT
   pdu_ddt_init(&src, 1);
   pdu_ddt_add(&src, srcphys, payload_size);
   pdu_ddt_init(&dst, 1);
   pdu_ddt_add(&dst, dstphys, payload_size);
   ddts = 1;

   spacc_cipher = cipher_to_spacc_cipher(cipher, ciphermode, cipherksize);
   spacc_hash   = hash_to_spacc_hash(hash, hashmode);
   spacc_chain_size = chain_size;

   for (x = 0; x < chain_size; x++) {
      jobs[x] = spacc_open(spacc, spacc_cipher, spacc_hash, 0, 0, sprof_job_cb, &job_cnt);
   }
   spacc_write_context(spacc, 0, SPACC_CRYPTO_OPERATION, key, cipherksize, key, 16);
   spacc_write_context(spacc, 0, SPACC_HASH_OPERATION,   key, hashksize, key, 16);
   spacc_set_operation(spacc, 0, OP_ENCRYPT, 0, 0, 0, 0, 0);

   cb_err  = 0;
   job_cnt = chain_size;
   init_completion(&fifofullcomp);
   for (y = 0; y < (runs/chain_size); y++) {
      init_completion(&comp);
      for (x = 0; x < chain_size; x++) {
          err = spacc_packet_enqueue_ddt (spacc, jobs[x], &src, &dst, payload_size, 0, 0, 0, 0, SPACC_SW_CTRL_PRIO_HI);
          if (err == CRYPTO_FIFO_FULL) {
             init_completion(&fifofullcomp);
             if (unlikely(wait_for_completion_interruptible(&fifofullcomp))) {
                printk("spacctraffic:  Aborted inner loop\n");
                err = -1;
                goto ERR;
             }
             --x;
             continue;
          }
          if (err < 0) {
             printk("sprof_thread::Cannot insert pdu\n");
             goto ERR;
          }
      }
#if 0
      if (unlikely(wait_for_completion_interruptible_timeout(&comp, msecs_to_jiffies(1000)))) {
         printk("spacctraffic:  Aborted outer loop %d\n", job_cnt);
         err = -1;
         goto ERR;
      }
#endif
wait_for_completion(&comp);
      job_cnt = chain_size;
   }

   err = !cb_err ? len : -1;
ERR:
   if (srcvirt) { dma_free_coherent(NULL, payload_size, srcvirt, srcphys); }
   if (dstvirt) { dma_free_coherent(NULL, payload_size, dstvirt, dstphys); }

   for (x = 0; x < chain_size; x++) {
      if (jobs[x] != -1) { spacc_close(spacc, jobs[x]); }
   }
   if (ddts) {
      pdu_ddt_free(&src);
      pdu_ddt_free(&dst);
   }
   return err;
}

struct file_operations sprof_dev_fops = {
  .owner   = THIS_MODULE,
  .open    = sprof_dev_open,
  .release = sprof_dev_release,
  .write   = sprof_dev_write,
  .read    = sprof_dev_read,
};

static struct miscdevice sprof_device = {
   .minor = MISC_DYNAMIC_MINOR,
   .name = "spacc",
   .fops = &sprof_dev_fops,
};

static int __init sprof_mod_init (void)
{
   return misc_register(&sprof_device);
}

static void __exit sprof_mod_exit (void)
{
    misc_deregister(&sprof_device);
}

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init (sprof_mod_init);
module_exit (sprof_mod_exit);

module_param(set_fifo, int, 0);
MODULE_PARM_DESC(set_fifo, "Manually set the SW fifo depth to something");
