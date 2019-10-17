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

static int spacc_epn=0x605;

// simple callback that clears a completion structure
// this is used so that the calling thread can sleep while the job processes
static void job_callback(void *sprof_dev, void *data)
{
   // wake up the calling task
   complete(data);
}

// hash data that has already been mapped to DMA coherent memory
//
static int spacc_hash_ex(spacc_device *spacc,
   PDU_DMA_ADDR_T src_phys, PDU_DMA_ADDR_T dst_phys, unsigned long src_len, unsigned long dst_len,
   int mode,
   const unsigned char *hmackey, unsigned hmackeysize)
{
   pdu_ddt       src, dst;
   int handle, err;
   struct completion comp;

   // create DDT for message
   pdu_ddt_init(&src, 1);                  // we only have one DDT fragment to the job, if the job were broken up more we'd allocate more DDT entries here
   pdu_ddt_add(&src, src_phys, src_len);   // here we add the source fragment to the source DDT

   pdu_ddt_init(&dst, 1);                  // similarly we only have one DDT fragment for the output
   pdu_ddt_add(&dst, dst_phys, dst_len);   // add the destination fragment to the destination DDT

   // 1.  Allocate a handle
   // we set the cipher mode to CRYPTO_MODE_NULL since we are only hashing
   // The "job_callback()" callback function expects a pointer to a completion structure which we provide here
   handle = spacc_open(spacc, CRYPTO_MODE_NULL, mode, 0, 0, job_callback, &comp);

   // 2.  Set context
   // here we write the HMAC key (if any) to the context page for this job
   spacc_write_context(spacc, handle, SPACC_HASH_OPERATION, hmackey, hmackeysize, NULL, 0);

   // 3.  Set operation
   // OP_ENCRYPT means to produce a hash/hmac output
   // we set all of the ICV options to zero since we want the ICV (hash output) at the start of the output buffer
   // setting icvcmd to IP_ICV_APPEND is handy if we want to cipher/hash in the same SPAcc command
   spacc_set_operation(spacc, handle, OP_ENCRYPT, 0, 0, 0, 0, 0);

   // (linux specific) init our callback completion
   init_completion(&comp);

   // start the job (we pass the job size as pre_aad_sz since we only want the HMAC-MD5 tag as output not the entire message)
   // proc_len   == size of message
   // pre_aad_sz == size of message, since we are not setting AAD_COPY then the output from the SPAcc is simply the HASH tag
   //               if we set pre_aad_sz to less than proc_len then it'll still produce the valid hash but it will also output
   //               some of the plaintext to the output buffer which we do not want
   // post_aad_sz == 0 since we consume all of the message in the pre_aad_sz
   spacc_packet_enqueue_ddt (spacc, handle, &src, &dst, src_len, 0, src_len, 0, 0, SPACC_SW_CTRL_PRIO_HI);

   // sleep the calling thread until the job finishes.  In this context a user can abort the job by pressing CTRL+C
   // in the terminal they used to load the module
   if (unlikely(wait_for_completion_interruptible(&comp))) {
      printk("User aborted task...\n");
      err = -1;
   } else {
      err = spacc_packet_dequeue(spacc, handle);
   }

   // free the DDTs
   pdu_ddt_free(&src);
   pdu_ddt_free(&dst);

   // close/free our SPAcc handle
   spacc_close(spacc, handle);
   return err;
}

// hash data that is in virtual memory (and may not be contiguous in physical memory)
// only needed for platforms that use an MMU
static int spacc_hash(spacc_device *spacc,
   void *src, void *dst, unsigned long src_len, unsigned long dst_len,
   int mode,
   const unsigned char *hmackey, unsigned hmackeysize)
{
   PDU_DMA_ADDR_T src_phys, dst_phys;
   void *src_virt, *dst_virt;
   int err;

   // allocate DMA coherent memory for the input/output
   // this memory is accessible by a both a virtual pointer and physical
   // this function allocates contiguous memory so it's not ideal for very large messages
   // in those cases it's better to allocate smaller blocks
   src_virt = dma_alloc_coherent(NULL, src_len, &src_phys, GFP_ATOMIC);
   dst_virt = dma_alloc_coherent(NULL, dst_len, &dst_phys, GFP_ATOMIC);

   // copy user message here
   // we need to do this since we're not guaranteed that the user message
   // is DMA mappable easily enough (it'd have to be mapped to scattergather buffers otherwise)
   memcpy(src_virt, src, src_len);
   err = spacc_hash_ex(spacc, src_phys, dst_phys, src_len, dst_len, mode, hmackey, hmackeysize);

   if (!err) {
      memcpy(dst, dst_virt, dst_len);
   }

   // free the DMA memory allocated to hodl the data
   dma_free_coherent(NULL, src_len, src_virt, src_phys);
   dma_free_coherent(NULL, dst_len, dst_virt, dst_phys);

   return err;
}


// CIPHER DMA coherent memory
//
static int spacc_cipher_ex(spacc_device *spacc,
   PDU_DMA_ADDR_T src_phys, PDU_DMA_ADDR_T dst_phys, unsigned long src_len,
   int mode, int encrypt,
   const unsigned char *key, unsigned keylen,
   const unsigned char *iv,  unsigned ivlen)
{
   pdu_ddt       src, dst;
   int handle, err;
   struct completion comp;

   // create DDT for message
   pdu_ddt_init(&src, 1);                // allocate a single DDT entry
   pdu_ddt_add(&src, src_phys, src_len); // add our source data to the source ddt

   pdu_ddt_init(&dst, 1);
   pdu_ddt_add(&dst, dst_phys, src_len); // add the destination data to the destination ddt

   // 1.  Allocate a handle
   // here we aren't hashing so we set the hash mode to CRYPTO_MODE_NULL
   handle = spacc_open(spacc, mode, CRYPTO_MODE_NULL, 0, 0, job_callback, &comp);

   // 2.  Set context
   // we are setting the CIPHER context with SPACC_CRYPTO_OPERATION
   spacc_write_context(spacc, handle, SPACC_CRYPTO_OPERATION, key, keylen, iv, ivlen);

   // 3.  Set operation
   spacc_set_operation(spacc, handle, encrypt?OP_ENCRYPT:OP_DECRYPT, 0, 0, 0, 0, 0);

   // 4.  set key exp bit if we are decrypting
   if (!encrypt) {
      spacc_set_key_exp(spacc, handle);
   }

   // (linux specific) init our callback completion
   init_completion(&comp);

   // start the job
   // here we set the pre_aad_sz/post_aad_sz to zero since we are treating
   // the entire source as plaintext
   spacc_packet_enqueue_ddt (spacc, handle, &src, &dst, src_len, 0, 0, 0, 0, SPACC_SW_CTRL_PRIO_HI);

   // wait for the job to complete and allow the user to abort if need be
   if (unlikely(wait_for_completion_interruptible(&comp))) {
      printk("User aborted task...\n");
      err = -1;
   } else {
      err = spacc_packet_dequeue(spacc, handle);
   }

   pdu_ddt_free(&src);
   pdu_ddt_free(&dst);
   spacc_close(spacc, handle);
   return err;
}

// CIPHER virtual memory
//
static int spacc_cipher(spacc_device *spacc,
   void *src, void *dst, unsigned long src_len,
   int mode, int encrypt,
   const unsigned char *key, unsigned keylen,
   const unsigned char *iv,  unsigned ivlen)
{
   PDU_DMA_ADDR_T src_phys, dst_phys;
   void *src_virt, *dst_virt;
   int err;

   src_virt = dma_alloc_coherent(NULL, src_len, &src_phys, GFP_ATOMIC);
   dst_virt = dma_alloc_coherent(NULL, src_len, &dst_phys, GFP_ATOMIC);

   memcpy(src_virt, src, src_len);
   err = spacc_cipher_ex(spacc, src_phys, dst_phys, src_len, mode, encrypt, key, keylen, iv, ivlen);

   if (!err) {
      memcpy(dst, dst_virt, src_len);
   }

   dma_free_coherent(NULL, src_len, src_virt, src_phys);
   dma_free_coherent(NULL, src_len, dst_virt, dst_phys);

   return err;
}







// DEMO CALLS to example wrappers
static void run_hash(spacc_device *spacc)
{
   unsigned char msg[] = "hello world"; // 11 bytes to hash with HMAC-MD5
   unsigned char hmackey[] = "spaccexample";
   int err, x;
   unsigned char buf[64];


   // EXAMPLE 1, HMAC-MD5 processing a message
      err = spacc_hash(spacc, msg, buf, sizeof msg, 16, CRYPTO_MODE_HMAC_MD5, hmackey, sizeof hmackey);
      printk("hash demo cmd #1, %d: ", err);
      for (x = 0; x < 16; x++) {
         printk("%02X ", buf[x]);
      }
      printk("\n");
}

static void run_cipher(spacc_device *spacc)
{
   unsigned char buf[2][32], key[16], iv[16];
   int err, x;

   // make up a message
   for (x = 0; x < 32; x++) {
      buf[0][x] = x;
   }
   for (x = 0; x < 16; x++) {
      key[x] = x;
      iv[x]  = x;
   }

   // EXAMPLE 1, encrypt in AES-128-CBC mode
   err = spacc_cipher(spacc, &buf[0][0], &buf[1][0], 32, CRYPTO_MODE_AES_CBC, 1, key, 16, iv, 16);
   printk("encrypt demo cmd #1, %d\n", err);
   for (x = 0; x < 32; x++) {
      printk("%02X ", buf[1][x]);
   }
   printk("\n");

   // EXAMPLE 2, decrypt in AES-128-CBC mode
   err = spacc_cipher(spacc, &buf[1][0], &buf[0][0], 32, CRYPTO_MODE_AES_CBC, 0, key, 16, iv, 16);
   printk("decrypt demo cmd #1, %d\n", err);
   for (x = 0; x < 32; x++) {
      printk("%02X ", buf[0][x]);
   }
   printk("\n");
}

static int __init example_mod_init (void)
{
   spacc_device *spacc = get_spacc_device_by_epn(spacc_epn, 0);
   if (spacc) {
      run_hash(spacc);
      run_cipher(spacc);
   }
   return 0;
}

static void __exit example_mod_exit (void)
{
}

module_param(spacc_epn, int, 0);
MODULE_PARM_DESC(spacc_epn, "SPAcc EPN (defaults to 0x605)");


MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init (example_mod_init);
module_exit (example_mod_exit);
