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

// ------------------------------------------------------------------------
//
//  Project:
//
//   Driver SDK
//
//  Description:
//
//   Device class for PKA-like devices.  This handles character device
//   allocation and the toplevel ioctl handling.
//
// ------------------------------------------------------------------------

#define CLASSNAME "pkadev"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/idr.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include "class.h"

static DEFINE_IDR(pka_idr);
static DEFINE_MUTEX(pka_idr_mutex);

static struct class *pka_class;
static int pka_major;

struct pka_chrdev_priv {
   struct device *pka_dev;
   const struct pka_class_ops *ops;
   struct semaphore chrdev_lock;
};

static int pka_fop_open(struct inode *inode, struct file *file)
{
   struct pka_chrdev_priv *priv;
   struct device *chrdev;

   rcu_read_lock();
   chrdev = get_device(idr_find(&pka_idr, MINOR(inode->i_rdev)));
   rcu_read_unlock();

   if (!chrdev)
      return -ENODEV;

   /*
    * In the future, it would be good to support concurrent access to the
    * device.  For now, prevent that from happening.
    */
   priv = dev_get_drvdata(chrdev);
   if (down_trylock(&priv->chrdev_lock))
      return -EAGAIN;

   file->private_data = chrdev;
   return 0;
}

static int pka_fop_release(struct inode *inode, struct file *file)
{
   struct device *chrdev = file->private_data;
   struct pka_chrdev_priv *priv = dev_get_drvdata(chrdev);

   if (priv->ops->pka_abort)
      priv->ops->pka_abort(priv->pka_dev);

   up(&priv->chrdev_lock);
   put_device(chrdev);

   return 0;
}

static long
pka_std_ioctl(struct device *chrdev, unsigned cmd, void __user *arg)
{
   struct pka_chrdev_priv *priv = dev_get_drvdata(chrdev);
   struct pka_param param;
   long rc;

   /*
    * To allow for future extension, we only require that there be as many
    * value bytes as the size parameter implies.  Currently this applies to
    * call operations, too, even though the value bytes are not used in that
    * case.
    */
   rc = copy_from_user(&param, arg, sizeof param);
   if (rc > sizeof param.value)
      return -EFAULT;
   if (param.size > sizeof param.value)
      return -EINVAL;
   if (rc > sizeof param.value - param.size)
      return -EFAULT;

   switch (cmd) {
   case PKA_IOC_SETPARAM:
      if (priv->ops->pka_setparam) {
         return priv->ops->pka_setparam(priv->pka_dev, &param);
      }
      dev_warn(priv->pka_dev, "SETPARAM not implemented\n");
      return -ENOSYS;
   case PKA_IOC_GETPARAM:
      if (priv->ops->pka_getparam) {
         rc = priv->ops->pka_getparam(priv->pka_dev, &param);
         if (rc < 0)
            return rc;
         break;
      }
      dev_warn(priv->pka_dev, "GETPARAM not implemented\n");
      return -ENOSYS;
   case PKA_IOC_CALL:
      if (priv->ops->pka_call) {
         return priv->ops->pka_call(priv->pka_dev, &param);
      }
      dev_warn(priv->pka_dev, "CALL not implemented\n");
      return -ENOSYS;
   default:
      BUG();
   }

   rc = copy_to_user(arg, &param, offsetof(struct pka_param, value)
                                  + param.size);
   if (rc != 0)
      return -EFAULT;

   return 0;
}

static long
pka_flag_ioctl(struct device *chrdev, unsigned cmd, void __user *arg)
{
   struct pka_chrdev_priv *priv = dev_get_drvdata(chrdev);
   struct pka_flag flag;
   long rc;

   rc = copy_from_user(&flag, arg, sizeof flag);
   if (rc > 0)
      return -EFAULT;

   if (flag.op >= PKA_FLAG_OP_MAX)
      return -EINVAL;

   switch (cmd) {
   case PKA_IOC_TESTF_OLD:
   case PKA_IOC_TESTF:
      if (flag.op != PKA_FLAG_OP_NOP)
         return -EINVAL;

      if (priv->ops->pka_testf) {
         return priv->ops->pka_testf(priv->pka_dev, &flag);
      }
      dev_warn(priv->pka_dev, "TESTF not implemented\n");
      return -ENOSYS;
   case PKA_IOC_SETF:
      if (priv->ops->pka_setf) {
         return priv->ops->pka_setf(priv->pka_dev, &flag);
      }
      dev_warn(priv->pka_dev, "SETF not implemented\n");
      return -ENOSYS;
   }

   BUG();
}

static long
pka_fop_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
   struct device *chrdev = file->private_data;
   struct pka_chrdev_priv *priv = dev_get_drvdata(chrdev);

   /* Handle "simple" ioctls. */
   switch (cmd) {
   case PKA_IOC_SETPARAM:
   case PKA_IOC_GETPARAM:
   case PKA_IOC_CALL:
      return pka_std_ioctl(chrdev, cmd, (void __user *)arg);
   case PKA_IOC_TESTF_OLD:
   case PKA_IOC_TESTF:
   case PKA_IOC_SETF:
      return pka_flag_ioctl(chrdev, cmd, (void __user *)arg);
   case PKA_IOC_WAIT:
      if (priv->ops->pka_wait)
         return priv->ops->pka_wait(priv->pka_dev);
      dev_warn(priv->pka_dev, "WAIT not implemented\n");
      return -ENOSYS;
   case PKA_IOC_ABORT:
      if (priv->ops->pka_abort)
         return priv->ops->pka_abort(priv->pka_dev);
      dev_warn(priv->pka_dev, "ABORT not implemented\n");
      return -ENOSYS;
   }

   /* Handle ioctls with variable-length data. */
   switch (cmd & ~(_IOC_SIZEMASK << _IOC_SIZESHIFT)) {
   case PKA_IOC_IDENT(0):
      /* TODO: copy something resembling a device name. */
      return 0;
   }

   if (priv->ops->pka_ioctl)
      return priv->ops->pka_ioctl(priv->pka_dev, cmd, arg);

   return -ENOTTY;
}

static const struct file_operations pka_chrdev_fops = {
   .owner          = THIS_MODULE,
   .open           = pka_fop_open,
   .release        = pka_fop_release,
   .unlocked_ioctl = pka_fop_ioctl,
};

static void pka_chrdev_release(struct device *chrdev)
{
   struct pka_chrdev_priv *priv = dev_get_drvdata(chrdev);

   put_device(priv->pka_dev);
   kfree(chrdev);
   kfree(priv);
}

static struct device *
pka_chrdev_create(struct device *parent, int id, const struct pka_class_ops *ops)
{
   struct pka_chrdev_priv *priv;
   struct device *chrdev;

   priv = kmalloc(sizeof *priv, GFP_KERNEL);
   if (!priv)
      return ERR_PTR(-ENOMEM);

   *priv = (struct pka_chrdev_priv) {
      .pka_dev = get_device(parent),
      .ops = ops,
   };
   sema_init(&priv->chrdev_lock, 1);

   chrdev = device_create(pka_class, parent, MKDEV(pka_major, id),
                          priv, "pka%d", id);
   if (IS_ERR(chrdev)) {
      put_device(parent);
      kfree(priv);
   } else {
      chrdev->release = pka_chrdev_release;
   }

   return chrdev;
}

struct device *pka_chrdev_register(struct device *dev, const struct pka_class_ops *ops)
{
   struct device *chrdev;
   int id;

   mutex_lock(&pka_idr_mutex);
   id = idr_alloc(&pka_idr, NULL, 0, 255, GFP_KERNEL);
   mutex_unlock(&pka_idr_mutex);

   if (id < 0)
      return ERR_PTR(id);

   chrdev = pka_chrdev_create(dev, id, ops);

   mutex_lock(&pka_idr_mutex);

   if (IS_ERR(chrdev))
      idr_remove(&pka_idr, id);
   else
      idr_replace(&pka_idr, get_device(chrdev), id);

   mutex_unlock(&pka_idr_mutex);

   return chrdev;
}

void pka_chrdev_unregister(struct device *chrdev)
{
   int id = MINOR(chrdev->devt);
   struct device *idr_chrdev;

   rcu_read_lock();
   idr_chrdev = idr_find(&pka_idr, id);
   rcu_read_unlock();

   BUG_ON(idr_chrdev != chrdev);

   mutex_lock(&pka_idr_mutex);
   idr_remove(&pka_idr, id);
   mutex_unlock(&pka_idr_mutex);

   synchronize_rcu();

   device_unregister(chrdev);
   put_device(idr_chrdev);
}

int __init pka_class_init(void)
{
   pka_major = register_chrdev(0, CLASSNAME, &pka_chrdev_fops);
   if (pka_major < 0)
      return pka_major;

   pka_class = class_create(THIS_MODULE, CLASSNAME);
   if (!pka_class) {
      unregister_chrdev(pka_major, CLASSNAME);
      return PTR_ERR_OR_ZERO(pka_class);
   }

   return 0;
}

void pka_class_exit(void)
{
   class_destroy(pka_class);
   unregister_chrdev(pka_major, CLASSNAME);
   idr_destroy(&pka_idr);
}
