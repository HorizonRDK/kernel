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

static int spacc_epn = 0x0;

enum HASH_OPER {
    MD5,
    SHA1,
    SHA224,
    SHA256,
    SHA384,
    SHA512,
    HASH_MAX
};

char *hash_name[HASH_MAX] = {
    [MD5]    = "MD5",
    [SHA1]   = "SHA1",
    [SHA224] = "SHA224",
    [SHA256] = "SHA256",
    [SHA384] = "SHA384",
    [SHA512] = "SHA512"
};


int hash_op_mapping [HASH_MAX][2] = {
    [MD5]    = {CRYPTO_MODE_HASH_MD5,   16},
    [SHA1]   = {CRYPTO_MODE_HASH_SHA1,  20},
    [SHA224] = {CRYPTO_MODE_HASH_SHA224,28},
    [SHA256] = {CRYPTO_MODE_HASH_SHA256,32},
    [SHA384] = {CRYPTO_MODE_HASH_SHA384,48},
    [SHA512] = {CRYPTO_MODE_HASH_SHA512,64}
};

int hmac_op_mapping [HASH_MAX][2] = {
    [MD5]    = {CRYPTO_MODE_HMAC_MD5,    16},
    [SHA1]   = {CRYPTO_MODE_HMAC_SHA1,   20},
    [SHA224] = {CRYPTO_MODE_HMAC_SHA224, 28},
    [SHA256] = {CRYPTO_MODE_HMAC_SHA256, 32},
    [SHA384] = {CRYPTO_MODE_HMAC_SHA384, 48},
    [SHA512] = {CRYPTO_MODE_HMAC_SHA512, 64}
};

enum AES_OPER {
    CBC_128,
    CBC_192,
    CBC_256,
    ECB_256,
    CFB_256,
    OFB_256,
    CTR_256,
    AES_MAX
};

char *algo_name[AES_MAX] = {
    [CBC_128] = "aes_128_cbc",
    [CBC_192] = "aes_192_cbc",
    [CBC_256] = "aes_256_cbc",
    [ECB_256] = "aes_256_ecb",
    [CFB_256] = "aes_256_cfb",
    [OFB_256] = "aes_256_ofb",
    [CTR_256] = "aes_256_ctr",
};

int aes_op_mapping [AES_MAX][2] ={
    [CBC_128] = {CRYPTO_MODE_AES_CBC, 16},
    [CBC_192] = {CRYPTO_MODE_AES_CBC, 24},
    [CBC_256] = {CRYPTO_MODE_AES_CBC, 32},
    [ECB_256] = {CRYPTO_MODE_AES_ECB, 32},
    [CFB_256] = {CRYPTO_MODE_AES_CFB, 32},
    [OFB_256] = {CRYPTO_MODE_AES_OFB, 32},
    [CTR_256] = {CRYPTO_MODE_AES_CTR, 32}
};

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
   src_virt = pdu_dma_alloc(src_len, &src_phys);
   dst_virt = pdu_dma_alloc(dst_len, &dst_phys);
   // copy user message here
   // we need to do this since we're not guaranteed that the user message
   // is DMA mappable easily enough (it'd have to be mapped to scattergather buffers otherwise)
   //
   memcpy(src_virt, src, src_len);
   err = spacc_hash_ex(spacc, src_phys, dst_phys, src_len, dst_len, mode, hmackey, hmackeysize);
   if (!err) {
      memcpy(dst, dst_virt, dst_len);
   }
   // free the DMA memory allocated to hodl the data
   //dma_free_coherent(NULL, src_len, src_virt, src_phys);
   pdu_dma_free(src_len, src_virt, src_phys);
   //dma_free_coherent(NULL, dst_len, dst_virt, dst_phys);
   pdu_dma_free(dst_len, dst_virt, dst_phys);
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

   src_virt = pdu_dma_alloc(src_len, &src_phys);
   dst_virt = pdu_dma_alloc(src_len, &dst_phys);

   memcpy(src_virt, src, src_len);
   err = spacc_cipher_ex(spacc, src_phys, dst_phys, src_len, mode, encrypt, key, keylen, iv, ivlen);

   if (!err) {
      memcpy(dst, dst_virt, src_len);
   }

   pdu_dma_free(src_len, src_virt, src_phys);
   //dma_free_coherent(NULL, src_len, src_virt, src_phys);
   pdu_dma_free(src_len, dst_virt, dst_phys);
   //dma_free_coherent(NULL, src_len, dst_virt, dst_phys);

   return err;
}







// DEMO CALLS to example wrappers
static void run_hash(spacc_device *spacc, int mode, int len)
{
   unsigned char msg[] = 
   "I could stay awake just to hear you breathing Watch you smile while you are sleeping While you're far away dreaming I could spend my life in this sweet surrender I could stay lost in this moment forever Every moment spent with you is a moment I treasure Don't want to close my eyes I don't want to fall asleep 'Cause I'd miss you baby And I don't want to miss a thing 'Cause even when I dream of you The sweetest dream will never do I'd still miss you baby And I don't want to miss a thing And I don't want to mi";
   //unsigned char hmackey[] = "01234567";
   int err, x;
   unsigned char buf[64];

   // EXAMPLE 1, HMAC-MD5 processing a message
   err = spacc_hash(spacc, msg, buf, strlen(msg), len, mode, NULL, NULL);
   for (x = 0; x < len; x++) {
      printk(KERN_CONT "%02x ", buf[x]);
   }
   printk(KERN_CONT "\n");
}

static void run_hmac(spacc_device *spacc, int mode, int len)
{
   unsigned char msg[] = 
   "I could stay awake just to hear you breathing Watch you smile while you are sleeping While you're far away dreaming I could spend my life in this sweet surrender I could stay lost in this moment forever Every moment spent with you is a moment I treasure Don't want to close my eyes I don't want to fall asleep 'Cause I'd miss you baby And I don't want to miss a thing 'Cause even when I dream of you The sweetest dream will never do I'd still miss you baby And I don't want to miss a thing And I don't want to mi";
   unsigned char hmackey[] = "01234567";
   int err, x;
   unsigned char buf[64];

   // EXAMPLE 1, HMAC-MD5 processing a message
   err = spacc_hash(spacc, msg, buf, strlen(msg), len, mode, hmackey, strlen(hmackey));
   for (x = 0; x < len; x++) {
      printk(KERN_CONT "%02x ", buf[x]);
   }
   printk(KERN_CONT "\n");
}
/*
 *  spacc
 *  mode
 *  key size
 * */

#define SPACC_IVLEN 16
#define SPACC_128BIT 16
#define SPACC_192BIT 24
#define SPACC_256BIT 32
#define MSG_SZ  512

static void run_cipher(spacc_device *spacc, int mode, unsigned keylen)
{
   //unsigned char key[keylen], iv[keylen];
   unsigned char buf[2][MSG_SZ] = { {0}, {0} };
   unsigned char *str = {
   "I could stay awake just to hear you breathing Watch you smile while you are sleeping While you're far away dreaming I could spend my life in this sweet surrender I could stay lost in this moment forever Every moment spent with you is a moment I treasure Don't want to close my eyes I don't want to fall asleep 'Cause I'd miss you baby And I don't want to miss a thing 'Cause even when I dream of you The sweetest dream will never do I'd still miss you baby And I don't want to miss a thing And I don't want to mi"
   };
   int err, x;

   unsigned char ret_buf[MSG_SZ];

   strncpy(&buf[0][0], str,MSG_SZ);

   // make up a message
   //unsigned char iv[32] = {0x9,0x8,0xB,0xF,0xC,0x4,0x0,0x2,0x0,0xB,0x2,0x3,0x1,0x2,0x8,0x5,0xA,0xC,0xA,0x2,0x1,0x2,0x7,0xD,0x4,0x1,0x6,0x5,0x4,0x6,0x2,0x7};
   //unsigned char key[32] = {0x1,0x2};
   //unsigned char key[32] = {0xA,0x8,0x9,0x2,0x9,0x3,0xC,0x1,0x0,0x0,0xF,0xD,0x6,0xF,0x9,0x4,0x5,0x7,0x5,0x0,0x8,0x8,0x3,0x7,0x2,0x7,0xA,0x7,0xB,0xC,0xD,0xB};
   unsigned char *key;

   if (keylen == SPACC_128BIT) {
       /* A 128 bit key */
       key  = (unsigned char *)"0123456789012345";
       printk("use 128 bit key\n");
   } else if (keylen == SPACC_192BIT) {
       /* A 192 bit key */
       key = (unsigned char *)"012345678901234567890123";
       printk("use 192 bit key\n");
   } else if (keylen == SPACC_256BIT) {
       /* A 256 bit key */
       key = (unsigned char *)"01234567890123456789012345678901";
       printk("use 256 bit key\n");
   } else {
       printk("unknow mode [%d] !!! abort!!!\n", keylen);
       return;
   }

   /* A 128 bit IV */
   unsigned char *iv = (unsigned char *)"0123456789012345";
   //unsigned char *plaintext = (unsigned char *)"I could stay awake just to hear you breathing Watch you smile while you are sleeping While you're far away dreaming I could spend my life in this sweet surrender I could stay lost in this moment forever Every moment spent with you is a moment I treasure Don't want to close my eyes I don't want to fall asleep 'Cause I'd miss you baby And I don't want to miss a thing 'Cause even when I dream of you The sweetest dream will never do I'd still miss you baby And I don't want to miss a thing aaa";

   // EXAMPLE 1, encrypt in AES-128-CBC mode
   err = spacc_cipher(spacc, &buf[0][0], &buf[1][0], MSG_SZ, mode, 1, key, keylen, iv, SPACC_IVLEN);
   printk("encrypt demo cmd #1, %d\n", err);

   for (x = 0; x < MSG_SZ; x++) {
      if(x % 16 == 0)
         printk(KERN_CONT "%04X - ", x);

      printk(KERN_CONT "%02X  ", buf[1][x]);

      if(x % 16 == 15)
         printk(KERN_CONT "\n");
   }

   printk("\n");
   // EXAMPLE 2, decrypt in AES-128-CBC mode
   err = spacc_cipher(spacc, &buf[1][0], &ret_buf[0], MSG_SZ, mode, 0, key, keylen, iv, SPACC_IVLEN);
   printk("decrypt demo cmd #1, %d\n", err);

   int i = 0;
   for (i = 0;i < MSG_SZ; i++) {
      if (i % 80 == 0)
         printk(KERN_CONT "\n");

      printk(KERN_CONT "%c", ret_buf[i]);
   }
}

static int __init example_mod_init (void)
{
   int i;
   spacc_device *spacc = get_spacc_device_by_epn(spacc_epn, 0);

#if 1
      if (spacc) {

          for (i = 0; i < HASH_MAX ; i++) {
             //printk("mode = %d, keylen = %d hash name = %s\n", hash_op_mapping[i][0], hash_op_mapping[i][1], hash_name[i]);
             printk(KERN_CONT "hash (mac) = %s: ", hash_name[i]);
             run_hash(spacc, hash_op_mapping[i][0], hash_op_mapping[i][1]);
          }

	  for (i = 0; i < HASH_MAX; i++) {
             //printk("mode = %d, keylen = %d hmac name = %s\n", hmac_op_mapping[i][0], hmac_op_mapping[i][1], hash_name[i]);
             printk(KERN_CONT "hash (hmac)= %s: ", hash_name[i]);
             run_hmac(spacc, hmac_op_mapping[i][0], hmac_op_mapping[i][1]);
          }

          for (i = 0; i < AES_MAX; i++) {
             //printk("mode = %d, keylen = %d name = %s\n", aes_op_mapping[i][0], aes_op_mapping[i][1], algo_name[i]);
             run_cipher(spacc, aes_op_mapping[i][0], aes_op_mapping[i][1]);
          }
   }
#endif
   return 0;
}

static void __exit example_mod_exit (void)
{
}

module_param(spacc_epn, int, 0);
MODULE_PARM_DESC(spacc_epn, "SPAcc EPN (defaults to 0x0)");


MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init (example_mod_init);
module_exit (example_mod_exit);
