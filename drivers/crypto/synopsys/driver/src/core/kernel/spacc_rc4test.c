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
#include <linux/completion.h>

#include "elpspaccdrv.h"

#define TEXT_SIZE 32
#define KEY_SIZE 16
#define RC4KEYCTX_SIZE 258

static int spacc_epn=0x9876;

static void dumpk(char *pfx, char *buf, int buflen)
{
   int i;

   printk(pfx);
   for (i = 0; i < buflen; i++) {
      printk("%02X ", buf[i]);
   }
   printk("\n");

}

// simple callback that clears a completion structure
// this is used so that the calling thread can sleep while the job processes
static void job_callback(void *sprof_dev, void *data)
{
   // wake up the calling task
   complete(data);
}

// The purpose of this test is to demonstrate that the RC4 key context can be managed in software
// rather than in the SPAcc. A normal expanded key context is produced by running a 0-byte encryption
// step in RC4 and then that key context is read out of the SPAcc. This key context is loaded into a
// distinct SPAcc context (handle2) and it is used to decode identical Cipher Text. If the same PT results
// from both handle1 and handle2 decode operations, then we know that reading and writing the RC4 context
// is working successfully.
static void run_rc4test(spacc_device *spacc)
{
   unsigned char      pt[TEXT_SIZE] = "Insert plaintext here.";
   unsigned char      ct[TEXT_SIZE], h1out[TEXT_SIZE], h2out[TEXT_SIZE];
   unsigned char      key[KEY_SIZE];
   PDU_DMA_ADDR_T     src_phys, dst_phys;
   void               *src_virt, *dst_virt;
   pdu_ddt            src, dst;
   int                handle1, handle2;
   struct completion  comp;
   int                err, x;
   unsigned char      rc4keyctx[258];

  // create PT and key
   for (x = 0; x < KEY_SIZE; x++) {
      key[x] = x + 1;
   }

   // initial state
   printk("--- RC4TEST ---\n");
   dumpk("PT: ", pt, sizeof(pt));
   dumpk("KEY: ", key, sizeof(key));
   printk("---------------\n");

   // set up dma and copy in the pt
   src_virt = dma_alloc_coherent(NULL, TEXT_SIZE, &src_phys, GFP_ATOMIC);
   dst_virt = dma_alloc_coherent(NULL, TEXT_SIZE, &dst_phys, GFP_ATOMIC);
   memcpy(src_virt, pt, TEXT_SIZE);

   // create DDT for messages
   pdu_ddt_init(&src, 1);
   pdu_ddt_add(&src, src_phys, TEXT_SIZE);
   pdu_ddt_init(&dst, 1);
   pdu_ddt_add(&dst, dst_phys, TEXT_SIZE);

   // open handle1
   handle1 = spacc_open(spacc, CRYPTO_MODE_RC4_128, CRYPTO_MODE_NULL, 0, 0, job_callback, &comp);

   // write initial context
   spacc_write_context(spacc, handle1, SPACC_CRYPTO_OPERATION, key, KEY_SIZE, NULL, 0);

   printk("ENCRYPT 0 bytes to expand key\n");
   // configure operation and set key expansion bit
   spacc_set_operation(spacc, handle1, OP_ENCRYPT, 0, 0, 0, 0, 0);
   spacc_set_key_exp(spacc, handle1);

   init_completion(&comp);

   // encrypt 0 byte PT - this will expand the key so that we can steal it
   spacc_packet_enqueue_ddt(spacc, handle1, &src, &dst, 0, 0, 0, 0, 0, SPACC_SW_CTRL_PRIO_HI);

   // wait for the job to complete and allow the user to abort if need be
   if (unlikely(wait_for_completion_interruptible(&comp))) {
      printk("User aborted task...\n");
      err = -1;
   } else {
      err = spacc_packet_dequeue(spacc, handle1);
   }

   if (err) {
      printk("ERROR: %d\n", err);
   }

   // read in rc4 expanded key context
   spacc_read_rc4_context(spacc, handle1, &rc4keyctx[0], &rc4keyctx[1], rc4keyctx + 2);
   dumpk("RC4KEYCTX: ", rc4keyctx, RC4KEYCTX_SIZE);

   init_completion(&comp);

   // encrypt PT normally
   spacc_packet_enqueue_ddt(spacc, handle1, &src, &dst, TEXT_SIZE, 0, 0, 0, 0, SPACC_SW_CTRL_PRIO_HI);

   // wait for the job to complete and allow the user to abort if need be
   if (unlikely(wait_for_completion_interruptible(&comp))) {
      printk("User aborted task...\n");
      err = -1;
   } else {
      err = spacc_packet_dequeue(spacc, handle1);
   }

   if (!err) {
      memcpy(ct, dst_virt, TEXT_SIZE);
   } else {
      printk("ERROR: %d\n", err);
   }
   dumpk("CT: ", ct, TEXT_SIZE);

   // open a new handle
   handle2 = spacc_open(spacc, CRYPTO_MODE_RC4_KS, CRYPTO_MODE_NULL, 0, 0, job_callback, &comp);

   // write RC4 context read from handle1 directly to RC4 expanded context area of handle2
   spacc_write_context(spacc, handle2, SPACC_CRYPTO_OPERATION, rc4keyctx, RC4KEYCTX_SIZE, NULL, 0);

   printk("DECRYPT CT with copied rc4 context\n");
   memcpy(src_virt, ct, TEXT_SIZE);
   spacc_set_operation(spacc, handle2, OP_DECRYPT, 0, 0, 0, 0, 0);
   init_completion(&comp);
   spacc_packet_enqueue_ddt(spacc, handle2, &src, &dst, TEXT_SIZE, 0, 0, 0, 0, SPACC_SW_CTRL_PRIO_HI);

   // wait for the job to complete and allow the user to abort if need be
   if (unlikely(wait_for_completion_interruptible(&comp))) {
      printk("User aborted task...\n");
      err = -1;
   } else {
      err = spacc_packet_dequeue(spacc, handle2);
   }

   if (!err) {
      memcpy(h2out, dst_virt, TEXT_SIZE);
   } else {
      printk("ERROR: %d\n", err);
   }

   dumpk("PT (handle2): ", h2out, TEXT_SIZE);
   spacc_close(spacc, handle2);

   // decrypt CT normally
   memcpy(src_virt, ct, TEXT_SIZE);

   printk("DECRYPT CT with original rc4 key\n");
   spacc_set_operation(spacc, handle1, OP_DECRYPT, 0, 0, 0, 0, 0);
   spacc_set_key_exp(spacc, handle1);
   init_completion(&comp);
   spacc_packet_enqueue_ddt(spacc, handle1, &src, &dst, TEXT_SIZE, 0, 0, 0, 0, SPACC_SW_CTRL_PRIO_HI);

   // wait for the job to complete and allow the user to abort if need be
   if (unlikely(wait_for_completion_interruptible(&comp))) {
      printk("User aborted task...\n");
      err = -1;
   } else {
      err = spacc_packet_dequeue(spacc, handle1);
   }

   if (!err) {
      memcpy(h1out, dst_virt, TEXT_SIZE);
   } else {
      printk("ERROR: %d\n", err);
   }

   dumpk("PT (handle1): ", h1out, TEXT_SIZE);

   if (memcmp(h1out, h2out, TEXT_SIZE)) {
      printk("RC4TEST FAILED!\n");
   } else {
      printk("RC4TEST PASSED!\n");
   }
   // close handle1
   spacc_close(spacc, handle1);

   // free the DDTs
   pdu_ddt_free(&src);
   pdu_ddt_free(&dst);

   // release dma
   dma_free_coherent(NULL, TEXT_SIZE, src_virt, src_phys);
   dma_free_coherent(NULL, TEXT_SIZE, dst_virt, dst_phys);
}

static int __init rc4test_mod_init(void)
{
   spacc_device *spacc = get_spacc_device_by_epn(spacc_epn, 0);
   if (spacc) {
      run_rc4test(spacc);
   }
   return 0;
}

static void __exit rc4test_mod_exit(void)
{
}

module_param(spacc_epn, int, 0);
MODULE_PARM_DESC(spacc_epn, "SPAcc EPN (defaults to 0x9876)");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init(rc4test_mod_init);
module_exit(rc4test_mod_exit);
