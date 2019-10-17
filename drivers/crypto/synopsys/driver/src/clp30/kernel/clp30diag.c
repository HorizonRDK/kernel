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
 * Copyright (c) 2013 Synopsys, Inc. and/or its affiliates.
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
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/param.h>
#include <linux/platform_device.h>
#include "clp30.h"

#define MAXHEADERS 16

enum {
   ESP=1,
   AH=2
};

enum {
   INBOUND=1,
   OUTBOUND=2
};

enum {
   eAlgNULL = 0,
   eAlgDES,
   eAlg3DES,
   eAlgAESCBC128,
   eAlgAESCBC192,
   eAlgAESCBC256,
   eAlgAESCTR128,
   eAlgAESCTR192,
   eAlgAESCTR256,
   eAlgAESCCM128,
   eAlgAESCCM192,
   eAlgAESCCM256,
   eAlgAESGCM128,
   eAlgAESGCM192,
   eAlgAESGCM256,
};

enum {
   eAuthNULL = 0,
   eAuthMD5,
   eAuthSHA1,
   eAuthSHA256,
   eAuthSHA384,
   eAuthSHA512,
   eAuthAESXCBC,
   eAuthAESCMAC,
   eAuthAESGMAC,
};

enum {
   TRANSPORT=1,
   TUNNEL
};

int noisy_mode_activated = 0;

static void outdiff (unsigned char *buf1, unsigned char *buf2, int len)
{
  int x, y;
  char buf[16];
  printk("outdiff len == %d\n", len);
  for (x = 0; x < len; x += 4) {
//    if (memcmp(buf1, buf2, 4) == 0) { buf1 += 4; buf2 += 4; continue; }
    sprintf (buf, "%04x   ", x);
    printk (buf);
    for (y = 0; y < 4; y++) {
      sprintf (buf, "%02lx", (unsigned long) buf1[y] & 255UL);
      printk (buf);
    }
    printk (" - ");
    for (y = 0; y < 4; y++) {
      sprintf (buf, "%02lx", (unsigned long) buf2[y] & 255UL);
      printk (buf);
    }
    printk (" = ");
    for (y = 0; y < 4; y++) {
      sprintf (buf, "%02lx", (unsigned long) (buf1[y] ^ buf2[y]) & 255UL);
      printk (buf);
    }
    printk ("\n");
    buf1 += 4;
    buf2 += 4;
  }
}


/* get a line and skip blanks and comment lines */
static char glbuf[512], vector[32768], *vecp;

static char *get_line (void)
{
   unsigned x, l;
   do {
      x = 0;
      memset(glbuf, 0, sizeof glbuf);
      while (*vecp && *vecp != '\n' && x < 512) {
         glbuf[x++] = *vecp++;
      }
      if (*vecp == '\n') { ++vecp; }
      if (noisy_mode_activated) { printk("get_line: [%s]\n", glbuf); }
   } while (*vecp && sscanf(glbuf, "%08x", &l) != 1);
   return glbuf;
}

static void read_words (unsigned char *dst, int nwords, int little_endian)
{
  char *p;
  unsigned long L;
  unsigned long *pdst = (unsigned long *) dst;

  while (nwords-- && (p = get_line ())) {
    if (sscanf (p, "%08lx", &L) != 1) return;
    if (little_endian)
      *pdst++ = htonl(L);
    else
      *pdst++ = (L);
//    printk("Read %08lx\n", pdst[-1]);
  }
}

static int read_conf(int *ip_version, int *no_headers, int *in_size, int *out_size, int *dst_op_mode)
{
   char *p = get_line();
   if (p == NULL) return -1;
   if (sscanf(p, "%d, %d, %d, %d, %d", ip_version, no_headers, in_size, out_size, dst_op_mode) != 5) {
      return -1;
   }
   return 0;
}

static int read_crypto_conf(int *direction, int *mode, int *transform, int *encryption_alg, int *authentication_alg, uint32_t *spi, int *ext_seq, int *aesmaclen)
{
   char *p = get_line();
   if (p == NULL) return -1;
   if (sscanf(p, "%d, %d, %d, %d, %d, %zu, %d, %d", direction, mode, transform, encryption_alg, authentication_alg, spi, ext_seq, aesmaclen) != 8) {
      return -1;
   }
   return 0;
}

static char *authname(int authentication_alg)
{
   switch (authentication_alg) {
      case eAuthNULL:    return "MAC-NULL";
      case eAuthMD5:     return "HMAC-MD5";
      case eAuthSHA1:    return "HMAC-SHA1";
      case eAuthSHA256:  return "HMAC-SHA256";
      case eAuthSHA384:  return "HMAC-SHA384";
      case eAuthSHA512:  return "HMAC-SHA512";
      case eAuthAESXCBC: return "MAC-AES-XCBC";
      case eAuthAESCMAC: return "MAC-AES-CMAC";
      case eAuthAESGMAC: return "MAC-AES-GMAC";
      default: return 0;
   }
}

static char *encname(int encryption_alg)
{
   switch (encryption_alg) {
      case eAlgNULL:      return "NULL";
      case eAlgDES:       return "DES";
      case eAlg3DES:      return "3DES";
      case eAlgAESCBC128: return "AES-CBC-128";
      case eAlgAESCBC192: return "AES-CBC-192";
      case eAlgAESCBC256: return "AES-CBC-256";
      case eAlgAESCTR128: return "AES-CTR-128";
      case eAlgAESCTR192: return "AES-CTR-192";
      case eAlgAESCTR256: return "AES-CTR-256";
      case eAlgAESCCM128: return "AES-CCM-128";
      case eAlgAESCCM192: return "AES-CCM-192";
      case eAlgAESCCM256: return "AES-CCM-256";
      case eAlgAESGCM128: return "AES-GCM-128";
      case eAlgAESGCM192: return "AES-GCM-192";
      case eAlgAESGCM256: return "AES-GCM-256";
      default: return 0;
   }
}

static int authkeysize(int authentication_alg)
{
   switch (authentication_alg) {
      case eAuthMD5:    return 16;
      case eAuthSHA1:   return 20;
      case eAuthSHA256: return 32;
      case eAuthSHA384: return 48;
      case eAuthSHA512: return 64;
      case eAuthAESXCBC: return 64; // 3KEY mode
      case eAuthAESCMAC: return 16;
      case eAuthAESGMAC: return 16;
      default: return 0;
   }
}

static int enckeysize(int encryption_alg)
{
   switch (encryption_alg) {
      case eAlgNULL:      return 0;
      case eAlgDES:       return 8;
      case eAlg3DES:      return 24;
      case eAlgAESCBC128: return 16;
      case eAlgAESCBC192: return 24;
      case eAlgAESCBC256: return 32;
      case eAlgAESCTR128: return 16;
      case eAlgAESCTR192: return 24;
      case eAlgAESCTR256: return 32;
      case eAlgAESCCM128: return 16;
      case eAlgAESCCM192: return 24;
      case eAlgAESCCM256: return 32;
      case eAlgAESGCM128: return 16;
      case eAlgAESGCM192: return 24;
      case eAlgAESGCM256: return 32;
      default: return 0;
   }
}

static int encivsize(int encryption_alg)
{
   switch (encryption_alg) {
      case eAlgNULL:      return 0;
      case eAlgDES:       return 8;
      case eAlg3DES:      return 8;
      case eAlgAESCBC128: return 16;
      case eAlgAESCBC192: return 16;
      case eAlgAESCBC256: return 16;
      case eAlgAESCTR128: return 4;
      case eAlgAESCTR192: return 4;
      case eAlgAESCTR256: return 4;
      case eAlgAESCCM128: return 4;
      case eAlgAESCCM192: return 4;
      case eAlgAESCCM256: return 4;
      case eAlgAESGCM128: return 4;
      case eAlgAESGCM192: return 4;
      case eAlgAESGCM256: return 4;
      default: return 0;
   }
}

static uint32_t vectosdk_enc(int encryption_alg)
{
   switch (encryption_alg) {
      case eAlgNULL:      return CLP30_SA_ALG_CIPH_NULL;
      case eAlgDES:       return CLP30_SA_ALG_CIPH_DES_CBC;
      case eAlg3DES:      return CLP30_SA_ALG_CIPH_3DES_CBC;
      case eAlgAESCBC128: return CLP30_SA_ALG_CIPH_AES128_CBC;
      case eAlgAESCBC192: return CLP30_SA_ALG_CIPH_AES192_CBC;
      case eAlgAESCBC256: return CLP30_SA_ALG_CIPH_AES256_CBC;
#ifndef CLP36_ENABLED
      case eAlgAESCTR128: return CLP30_SA_ALG_CIPH_AES128_CTR;
      case eAlgAESCTR192: return CLP30_SA_ALG_CIPH_AES192_CTR;
      case eAlgAESCTR256: return CLP30_SA_ALG_CIPH_AES256_CTR;
      case eAlgAESGCM128: return CLP30_SA_ALG_CIPH_AES128_GCM;
      case eAlgAESGCM192: return CLP30_SA_ALG_CIPH_AES192_GCM;
      case eAlgAESGCM256: return CLP30_SA_ALG_CIPH_AES256_GCM;
#endif
      default: return 0;
   }
}

static uint32_t vectosdk_auth(int authentication_alg)
{
   switch (authentication_alg) {
      case eAuthNULL:    return MAC_ALG_NULL;
      case eAuthMD5:     return MAC_ALG_HMAC_MD5_96;
      case eAuthSHA1:    return MAC_ALG_HMAC_SHA1_96;
      case eAuthSHA256:  return MAC_ALG_HMAC_SHA256_128;
      default: return 0;
   }
}

/* this function maps a virtual buffer to an SG then allocates a DDT and maps it, it breaks the buffer into 4K chunks */
static int map_buf_to_sg_ddt (const void *src, unsigned long srclen, struct scatterlist **sg, unsigned *sgents, pdu_ddt * ddt)
{
   unsigned x, n, y;
   struct scatterlist *sgtmp;

   *sgents = 0;
   *sg = NULL;

   n = (srclen + 4095) >> 12;
   *sg = kmalloc (n * sizeof (**sg), GFP_KERNEL);
   if (!*sg) {
      printk ("Out of memory allocating SG\n");
      return -1;
   }
   sg_init_table (*sg, n);
   for (x = 0; srclen; x++) {
      y = (srclen > 4096) ? 4096 : srclen;
      sg_set_buf (&(*sg)[x], src, y);
      srclen -= y;
      src += y;
   }
   x = dma_map_sg (NULL, *sg, n, DMA_BIDIRECTIONAL);
   if (!x) {
      kfree (*sg);
      *sg = NULL;
   }
   pdu_ddt_init (ddt, x);
   for_each_sg (*sg, sgtmp, x, y) {
      pdu_ddt_add (ddt, (PDU_DMA_ADDR_T) sg_dma_address (sgtmp), sg_dma_len (sgtmp));
   }
   *sgents = x;
   return 0;
}

int noisy_mode_activated;

struct callbackdata {
  struct completion comp;
  uint32_t outlen,
           retcode,
           swid;
};


static void diag_callback(void *clp30_dev, void *data, uint32_t payload_len, uint32_t retcode, uint32_t swid)
{
   struct callbackdata *cbdata = data;
   cbdata->retcode = retcode;
   if (retcode) {
      printk("Callback retcode is %zu...\n", retcode);
      cbdata->outlen = 0;
   } else {
      cbdata->outlen  = payload_len;
      cbdata->swid    = swid;
   }
   complete(&cbdata->comp);
}

static int parse_test_script(int little_endian)
{
   unsigned char *v_in  = NULL,
                 *v_out = NULL,
                 *out   = NULL,
                  enckey[32],
                  enciv[16],
                  authkey[64], SA[256];

   uint32_t spi;

   int ip_version,
       no_headers,
       in_size,
       out_size,
       direction,
       transform,
       mode,
       encryption_alg,
       authentication_alg,
       dst_op_mode,
       ext_seq,
       aesmaclen;

   int ret    = -1,
       handle = -1;

   unsigned sgents_src, sgents_dst;
   struct scatterlist *sgsrc, *sgdst;
   pdu_ddt src_ddt, dst_ddt;
   clp30_device *clp30;
   elpxfrm_sa      sa;

   struct callbackdata cbdata;

   sgents_src = 0;
   sgents_dst = 0;
   sgsrc = NULL;
   sgdst = NULL;
   v_out = NULL;
   v_in  = NULL;
     out = NULL;

   clp30 = clp30_get_device();

   memset(enckey, 0, sizeof enckey);
   memset(enciv, 0, sizeof enciv);
   memset(authkey, 0, sizeof authkey);
   memset(SA, 0, sizeof SA);

// read in version, # of headers, input/output size
   if ((ret = read_conf(&ip_version, &no_headers, &in_size, &out_size, &dst_op_mode)) != 0) {
      printk("clp30diag: reading configuration failed\n");
      goto end;
   }
   if (no_headers > MAXHEADERS) {
      printk("clp30diag: no_headers is too large\n");
      ret = -1;
      goto end;
   }

// read crypto conf
   if ((ret = read_crypto_conf(&direction, &mode, &transform, &encryption_alg, &authentication_alg, &spi, &ext_seq, &aesmaclen)) != 0) {
      printk("clp30diag: reading crypto conf failed\n");
      goto end;
   }

// read keys/iv
// we always read the AUTH key
//   printk("Reading authkey\n");
   if (authentication_alg == eAuthAESGMAC) {
      read_words(enckey, 16/4,1);                               // GMAC key goes in cipher area ...
   } else {
      read_words(authkey, authkeysize(authentication_alg)/4,1);
   }
   if (transform == ESP) {
//      printk("Reading enckey\n");
      read_words(enckey, enckeysize(encryption_alg)/4,1);
//      printk("Reading enciv\n");
      if (authentication_alg == eAuthAESGMAC) {
         read_words(enciv, 1,1);
      } else {
         read_words(enciv, encivsize(encryption_alg)/4,1);
      }
   } else {
      if (authentication_alg == eAuthAESGMAC) {
         read_words(enciv, 1,1);
      }
   }


// read input
   v_in = kmalloc(in_size, GFP_KERNEL);
   if (!v_in) {
     printk("clp30diag: error allocating memory for v_in\n");
     goto end;
   }

   v_out = kzalloc(out_size, GFP_KERNEL);
   if (!v_out) {
     printk("clp30diag: error allocating memory for v_out\n");
     goto end;
   }

   out = kmalloc(out_size, GFP_KERNEL);
   if (!out) {
     printk("clp30diag: error allocating memory for out\n");
     goto end;
   }

//   printk("READING INPUT\n");
   read_words(v_in,  (in_size+3)/4,little_endian);
//   printk("READING OUTPUT\n");
   read_words(v_out, (out_size+3)/4,little_endian);

   map_buf_to_sg_ddt(v_in, in_size,  &sgsrc, &sgents_src, &src_ddt);
   map_buf_to_sg_ddt(out,  out_size, &sgdst, &sgents_dst, &dst_ddt);

   dma_sync_sg_for_device(NULL, sgsrc, sgents_src, DMA_BIDIRECTIONAL);

   // run vector
   // allocate handle
   handle = clp30_open(clp30);
   if (handle < 0) {
      printk("clp30diag:  Failed to open an ESPAH handle\n");
      ret = -1;
      goto end;
   }

   // buildSA
   memset(&sa, 0, sizeof sa);
   sa.alg   = vectosdk_enc(encryption_alg)        |
              vectosdk_auth(authentication_alg);
   sa.ctrl = SA_CTRL_ACTIVE                             |
              ((ip_version == 6) ? SA_CTRL_IPV6 : 0)     |
              ((transform == AH) ? SA_CTRL_HDR_TYPE : 0) |
              (dst_op_mode ? SA_CTRL_DST_OP_MODE : 0);
   sa.spi   = spi;
   memcpy(sa.ckey, enckey, 32);
   memcpy(sa.csalt, enciv, 4);
   memcpy(sa.mackey, authkey, 64);
   ret = clp30_build_sa(clp30, handle, &sa);
   if (ret< 0) {
      printk("clp30diag::Error building sa %d\n", ret);
      goto end;
   }

   // do the job
   init_completion(&cbdata.comp);
   ret = clp30_go(clp30, handle, (direction == OUTBOUND) ? 1 : 0, &src_ddt, &dst_ddt, diag_callback, &cbdata);
   if (wait_for_completion_interruptible(&cbdata.comp)) {
      ret = 1;
      printk("clp30diag::User aborted vector\n");
      msleep(1000); // sleep here because if the engine isn't locked up it might write to our DMA'ables after we free ... no single job should take >1sec
      goto end;
   }

   dma_sync_sg_for_cpu(NULL, sgdst, sgents_dst, DMA_BIDIRECTIONAL);

// compare
   if (cbdata.retcode || memcmp((uint32_t *)out, (uint32_t *)v_out, cbdata.outlen)) {
      outdiff(out, v_out, cbdata.outlen);
      printk("clp30diag: Output differs from what was expected\n");
      ret = -1;
      goto end;
   }

// at this point we're all good
   ret = 0;
end:
   if (ret || noisy_mode_activated) {
      printk("clp30diag: vector [IP%d, %s, %s, in_size == %d, %s, %s, %s] == [%s]\n", ip_version, encname(encryption_alg), authname(authentication_alg), in_size,
             transform == AH ? "AH" : "ESP", mode == TUNNEL ? "TUNNEL" : "TRANSPORT", direction == OUTBOUND ? "OUTBOUND" : "INBOUND", (ret == 0) ? "PASSED" : "FAILED");
   }

   // have to unmap before free
   if (sgents_src) {
      dma_unmap_sg (NULL, sgsrc, sgents_src, DMA_BIDIRECTIONAL);
      kfree (sgsrc);
      pdu_ddt_free (&src_ddt);
   }

   if (sgents_dst) {
      dma_unmap_sg (NULL, sgdst, sgents_dst, DMA_BIDIRECTIONAL);
      kfree (sgdst);
      pdu_ddt_free (&dst_ddt);
   }

   if (v_in) {
      kfree(v_in);
   }
   if (out) {
      kfree(out);
   }
   if (v_out) {
      kfree(v_out);
   }

   clp30_close(clp30, handle);

   return ret;
}

static ssize_t store_test_name(struct file *fp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t offset, size_t count)
{
   vecp = &vector[0];
   if (offset == 0) {
      memset(vector, 0, sizeof vector);
   }
   if ((offset + count) > sizeof(vector)) {
      printk("clp30diag: error: vector too big!\n");
      return -1;
   }
   memcpy(vector + offset, buf, count);
   if (count == PAGE_SIZE) {
      return count;
   } else {
      return parse_test_script(1) ? -1 : count;
   }
}

static BIN_ATTR(test_name, 0222, NULL, store_test_name, 0);
struct platform_device *clp30_get_platform_device(void);
static int __init clp30diag_mod_init (void)
{
   return sysfs_create_bin_file(&clp30_get_platform_device()->dev.kobj, &bin_attr_test_name);
}
static void __exit clp30diag_mod_exit (void)
{
   sysfs_remove_bin_file(&clp30_get_platform_device()->dev.kobj, &bin_attr_test_name);
}

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
module_init (clp30diag_mod_init);
module_exit (clp30diag_mod_exit);
module_param(noisy_mode_activated, int, 0);
MODULE_PARM_DESC(noisy_mode_activated, "verbose diag");

