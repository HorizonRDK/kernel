#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/fs.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>


#include "elpparser.h"
#include "elpsaspa.h"
#include "elpsaspa_hw.h"
#include "saspa_drv.h"


#define MAXSIZE 262144

static void outdiff (unsigned char *buf1, unsigned char *buf2, int len)
{
  int x, y;
  char buf[16];
  printk("outdiff len == %d\n", len);
  for (x = 0; x < len; x += 4) {
    if (memcmp(buf1, buf2, 4) == 0) { buf1 += 4; buf2 += 4; continue; }
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
    /* exit early */
    //x=len;
  }
}


static int do_aes_vector(vector_data *vec)
{
   unsigned char *buf;
   int err, mode, keysz, dir;
   struct device *aes_dev;
   struct saspa_aes_ctx ctx;
   uint32_t half, aadlen, efflen, stat;

   aes_dev = bus_find_device_by_name(&platform_bus_type, NULL, "saspa-aes");
   if (!aes_dev)
      return -ENODEV;

   buf = vmalloc(MAXSIZE);
   if (!buf) {
      printk("do_aes_vector:Out of memory\n");
      return -1;
   }

   memcpy(buf, vec->pt, vec->pt_size);

   for (dir = 0; dir < 2; dir++) {
      memcpy(ctx.key, vec->key, vec->keysize);

      mode = keysz = -1;
      aadlen = 0;
      switch (vec->enc_mode) {
         case VEC_MODE_AES_ECB: mode = AES_MODE_ECB; break;
         case VEC_MODE_AES_CBC: mode = AES_MODE_CBC; memcpy(ctx.saspa_aes_cbc_iv, vec->iv, 16); break;
         case VEC_MODE_AES_CTR: mode = AES_MODE_CTR; memcpy(ctx.saspa_aes_ctr_iv, vec->iv, 16); break;
         case VEC_MODE_AES_CCM:
            mode = AES_MODE_CCM;
            memcpy(ctx.saspa_aes_ctr_iv, vec->iv, 16);            // copy CTR iv
            memcpy(ctx.saspa_aes_cbc_iv, vec->pre_aad, 16);       // copy B0
            if (dir == 0) {
               // only move on first [encrypt] pass, on 2nd pass the AAD/CT should be where we need it
               memmove(buf+vec->pre_aad_size, buf, vec->pt_size);    // move PT over and make room for AAD
               memcpy(buf, vec->pre_aad, vec->pre_aad_size);         // copy AAD including B0 into buf
            }
            memcpy(ctx.saspa_aes_tag, vec->icv, 16);              // copy tag into CTX
            aadlen = vec->pre_aad_size;                           // set length of aad
            break;
         case VEC_MODE_AES_GCM: mode = AES_MODE_GCM; break;
         case VEC_MODE_AES_F8:  mode = AES_MODE_F8;  break;
         case VEC_MODE_AES_XTS: mode = AES_MODE_XTS; break;
      }

      switch (vec->keysize) {
         case 16: keysz = AES_KEY_SZ_128; break;
         case 24: keysz = AES_KEY_SZ_192; break;
         case 32: keysz = AES_KEY_SZ_256; break;
      }

      if (mode == -1 || keysz == -1) {
         printk("Invalid AES parameters\n");
         err = -1;
         goto out;
      }

      efflen = vec->pt_size + aadlen;
      if (aadlen || efflen <= 32) {
         half = efflen;
      } else {
         half = ((efflen>>4)>>1)<<4; // divide # of 16-byte blocks in half then scale up to bytes
      }
      err = saspa_aes_run(aes_dev, !dir, mode, keysz, &ctx, buf, half, aadlen, &stat);
      if (err < 0) {
         printk("Could not run saspa_aes\n");
         goto out;
      }
      if (half != efflen) {
         err = saspa_aes_run(aes_dev, !dir, mode, keysz, &ctx, buf+half, efflen-half, aadlen, &stat);
         if (err < 0) {
            printk("Could not run saspa_aes\n");
            goto out;
         }
      }

      if (memcmp(buf+aadlen, dir==0?vec->ct:vec->pt, vec->pt_size)) {
         printk("%s did not match\n", dir == 0 ? "Ciphertext" : "Plaintext");
//         outdiff(buf+aadlen, dir==0?vec->ct:vec->pt, vec->pt_size);
         err = -1;
         goto out;
      }

      if (vec->enc_mode == VEC_MODE_AES_CCM) {
         if (dir && !stat) {
             printk("Decrypting gave us a o_valid of false, ICV failure\n");
            err = -1;
            goto out;
         }

         if (memcmp(vec->icv, ctx.saspa_aes_tag, 16)) {
            printk("ICV tag failed to match\n");
//            outdiff(ctx.saspa_aes_tag, vec->icv, 16);
            err = -1;
            goto out;
         }
      }
   }

   err = 0;
out:
   vfree(buf);
   put_device(aes_dev);
   printk("SASPA-AES vector[%d, %d, %d]: %s\n", mode, keysz, vec->pt_size, err ? "FAILED" : "PASSED");
   return err;
}

static int do_des_vector(vector_data *vec)
{
   unsigned char *buf;
   int err, mode, keysz, dir;
   struct device *des_dev;
   struct saspa_des_ctx ctx;
   uint32_t half;

   des_dev = bus_find_device_by_name(&platform_bus_type, NULL, "saspa-des");
   if (!des_dev) {
      printk("Could not find DES device\n");
      return -ENODEV;
   }

   buf = vmalloc(MAXSIZE);
   if (!buf) {
      printk("do_des_vector:Out of memory\n");
      return -1;
   }
   memcpy(buf, vec->pt, vec->pt_size);

   for (dir = 0; dir < 2; dir++) {
      mode = keysz = -1;
      switch (vec->enc_mode) {
         case VEC_MODE_3DES_ECB:
         case VEC_MODE_DES_ECB:  mode = DES_MODE_ECB; break;
         case VEC_MODE_3DES_CBC:
         case VEC_MODE_DES_CBC:  mode = DES_MODE_CBC; memcpy(ctx.iv, vec->iv, 8); break;
      }


      switch (vec->keysize) {
         case 8:
         case 24:  keysz = vec->keysize; memcpy(ctx.key, vec->key, keysz); break;
      }

      if (mode == -1 || keysz == -1) {
         printk("Invalid DES parameters\n");
         err = -1;
         goto out;
      }

      if (vec->pt_size <= 16) {
         half = vec->pt_size;
      } else {
         half = ((vec->pt_size>>3)>>1)<<3; // divide # of 8-byte blocks in half then scale up to bytes
      }
      err = saspa_des_run(des_dev, !dir, mode, keysz, &ctx, buf, half);
      if (err < 0) {
         printk("Could not run saspa_des\n");
         goto out;
      }

      if (half != vec->pt_size) {
         err = saspa_des_run(des_dev, !dir, mode, keysz, &ctx, buf + half, vec->pt_size - half);
         if (err < 0) {
            printk("Could not run saspa_des\n");
            goto out;
         }
      }

      if (memcmp(buf, dir==0?vec->ct:vec->pt, vec->pt_size)) {
         printk("%s did not match\n", dir == 0 ? "Ciphertext" : "Plaintext");
         err = -1;
         goto out;
      }
   }

   err = 0;
out:
   vfree(buf);
   put_device(des_dev);
   printk("SASPA-DES vector[%d, %d, %d]: %s\n", mode, keysz, vec->pt_size, err ? "FAILED" : "PASSED");
   return err;
}

static int do_hash_vector(vector_data *vec, int ssl_vec)
{
   int mode, err;
   void *buf;
   struct device *hash_dev;
   struct saspa_hash_ctx ctx;

   hash_dev = bus_find_device_by_name(&platform_bus_type, NULL, "saspa-hmac");
   if (!hash_dev) {
      printk("Cannot find hash device\n");
      return -1;
   }

   err = -1;

   switch (vec->hash_mode) {
   case VEC_MODE_SSLMAC_SHA1:
   case VEC_MODE_HASH_SHA1:
   case VEC_MODE_HMAC_SHA1:          mode = HMAC_MODE_SHA1; break;
   case VEC_MODE_SSLMAC_MD5:
   case VEC_MODE_HASH_MD5:
   case VEC_MODE_HMAC_MD5:           mode = HMAC_MODE_MD5; break;
   case VEC_MODE_HASH_SHA224:
   case VEC_MODE_HMAC_SHA224:        mode = HMAC_MODE_SHA224; break;
   case VEC_MODE_HASH_SHA256:
   case VEC_MODE_HMAC_SHA256:        mode = HMAC_MODE_SHA256; break;
   case VEC_MODE_HASH_SHA384:
   case VEC_MODE_HMAC_SHA384:        mode = HMAC_MODE_SHA384; break;
   case VEC_MODE_HASH_SHA512:
   case VEC_MODE_HMAC_SHA512:        mode = HMAC_MODE_SHA512; break;
   }

   buf = vmalloc(64);
   if (!buf) {
      printk("Out of memory\n");
      goto out;
   }

   memcpy(ctx.key, vec->hmac_key, vec->hmac_keysize);
   err = saspa_hash_run(hash_dev, mode, ssl_vec, vec->hmac_keysize, &ctx, vec->pre_aad, vec->pre_aad_size, buf, 64);
   if (err) {
      printk("Failed to run hash vector: %d\n", err);
      goto out;
   }
   if (memcmp(buf, vec->icv, vec->icv_size)) {
      printk("ICV compare failure\n");
//      outdiff(vec->icv, buf, vec->icv_size);
      err = -1;
      goto out;
   }

   err = 0;
out:
   printk("SASPA-HMAC: [%d, %d, %d, %d] %s\n", vec->pre_aad_size, mode, vec->hmac_keysize, vec->icv_size, err == 0 ? "PASSED" : "FAILED");
   vfree(buf);
   put_device(hash_dev);
   return err;
}

static int run_vector(char *filename)
{
  vector_data    *vec;  //the current vector
  int err, y;

  vec = init_test_vector(MAXSIZE);
  if (vec == NULL) {
    printk("Error allocating and initializing vector data\n");
    return -ENODEV;
  }

  y = -1;
  //use common parser to get the data
  err = parse_test_vector (filename, vec);
  if (err!=0){
    free_test_vector(vec);
    printk("parse_test_vector FAILED!!![%s]\n", filename);
    return y;
  }

  switch (vec->enc_mode) {
      case VEC_MODE_AES_ECB:
      case VEC_MODE_AES_CBC:
      case VEC_MODE_AES_CTR:
      case VEC_MODE_AES_CCM:
      case VEC_MODE_AES_GCM:
      case VEC_MODE_AES_F8:
      case VEC_MODE_AES_XTS:
         y = do_aes_vector(vec);
         goto out;
      case VEC_MODE_DES_ECB:
      case VEC_MODE_3DES_ECB:
      case VEC_MODE_DES_CBC:
      case VEC_MODE_3DES_CBC:
         y = do_des_vector(vec);
         goto out;
  }

  switch (vec->hash_mode) {
  case VEC_MODE_HASH_SHA1:
  case VEC_MODE_HMAC_SHA1:
  case VEC_MODE_HASH_MD5:
  case VEC_MODE_HMAC_MD5:
  case VEC_MODE_HASH_SHA224:
  case VEC_MODE_HMAC_SHA224:
  case VEC_MODE_HASH_SHA256:
  case VEC_MODE_HMAC_SHA256:
  case VEC_MODE_HASH_SHA384:
  case VEC_MODE_HMAC_SHA384:
  case VEC_MODE_HASH_SHA512:
  case VEC_MODE_HMAC_SHA512:
  case VEC_MODE_HASH_SHA512_224:
  case VEC_MODE_HMAC_SHA512_224:
  case VEC_MODE_HASH_SHA512_256:
  case VEC_MODE_HMAC_SHA512_256:
     y = do_hash_vector(vec, 0);
     goto out;
  case VEC_MODE_SSLMAC_MD5:
  case VEC_MODE_SSLMAC_SHA1:
     y = do_hash_vector(vec, 1);
     goto out;
  }


out:
   free_test_vector(vec);
   return y;
}

static ssize_t
store_saspadiag(struct class *class, struct class_attribute *classattr,
                                  const char *buf, size_t count)
{
   char *c, test_vector_name[count+1];
   int err;

   strcpy(test_vector_name, buf);

   /*
    * Check for trailing newline and remove it.  We do not
    * support filenames containing newlines.
    */
   c = strchr(test_vector_name, '\n');
   if (c) {
      if (c[1] != '\0')
         return -EINVAL;
      *c = '\0';
   }

   err = run_vector (test_vector_name);
   return (err == 0) ? count : -EINVAL;
}

static struct class_attribute attrs[] = {
   __ATTR(vector, 0200, NULL, store_saspadiag),
   __ATTR_NULL
};

static struct class saspadiag_class = {
   .class_attrs = attrs,
   .owner = THIS_MODULE,
   .name = "saspadiag",
};

static int __init saspadiag_init(void)
{
   return class_register(&saspadiag_class);
}
module_init(saspadiag_init);

static void __exit saspadiag_exit(void)
{
   class_unregister(&saspadiag_class);
}
module_exit(saspadiag_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
MODULE_DESCRIPTION("Diagnostic module for SASPA");
