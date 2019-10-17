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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <limits.h>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <ctype.h>
#include <errno.h>
#include <sys/ioctl.h>

#include "pkadev.h"
#include "pka_ioctl.h"

/* Check that the device name is of the form pkaN. */
static int is_pka_devname(const char *name)
{
   unsigned i;

   if (strncmp(name, "pka", 3) != 0)
      return 0;

   for (i = 3; name[i]; i++) {
      if (!isdigit((unsigned char)name[i]))
         return 0;
   }

   return 1;
}

static int elppka_device_find(const char *dir)
{
   int fd = -1, saved_errno;
   struct dirent *ent;
   char *tmp;
   int len;
   DIR *d;

   d = opendir(dir);
   if (!d) {
      return -1;
   }

   while (fd == -1) {
      saved_errno = errno;
      errno = 0;
      ent = readdir(d);
      if (!ent) {
         if (!errno)
            errno = ENODEV;
         break;
      }
      errno = saved_errno;

      if (!is_pka_devname(ent->d_name))
         continue;

      len = snprintf(NULL, 0, "%s/%s", dir, ent->d_name);
      if (len == INT_MAX || len < 0)
         continue;

      tmp = malloc(len + 1);
      if (!tmp)
         break;
      sprintf(tmp, "%s/%s", dir, ent->d_name);

      fd = elppka_device_open(tmp);

      free(tmp);
   }

   saved_errno = errno;
   closedir(d);
   errno = saved_errno;

   return fd;
}

int elppka_device_open(const char *dev)
{
   int fd, rc;

   if (!dev)
      return elppka_device_find("/dev");

   fd = open(dev, O_RDWR);
   if (fd == -1)
      return -1;

   rc = ioctl(fd, PKA_IOC_IDENT(0), (char *)NULL);
   if (rc == -1) {
      close(fd);
      return -1;
   }

   return fd;
}

int elppka_device_close(int fd)
{
   return close(fd);
}

int elppka_set_operand(int fd, const char *func, const char *name,
                               unsigned size, const void *data)
{
   struct pka_param param;

   if (size > sizeof param.value) {
      errno = EINVAL;
      return -1;
   }

   strncpy((void *)param.func, func ? func : "", sizeof param.func);
   strncpy((void *)param.name, name, sizeof param.name);
   memcpy(param.value, data, size);
   param.size = size;

   return ioctl(fd, PKA_IOC_SETPARAM, &param);
}

int elppka_get_operand(int fd, const char *func, const char *name,
                               unsigned size, void *data)
{
   struct pka_param param;
   int rc;

   if (size > sizeof param.value) {
      errno = EINVAL;
      return -1;
   }

   strncpy((void *)param.func, func ? func : "", sizeof param.func);
   strncpy((void *)param.name, name, sizeof param.name);
   param.size = size;

   rc = ioctl(fd, PKA_IOC_GETPARAM, &param);
   if (rc == -1)
      return -1;

   memcpy(data, param.value, size);
   return 0;
}

int elppka_copy_operand(int fd, const char *dst_func, const char *dst_name,
                                const char *src_func, const char *src_name,
                                unsigned size)
{
   struct pka_param param;
   int rc;

   if (size > sizeof param.value) {
      errno = EINVAL;
      return -1;
   }

   strncpy((void *)param.func, src_func ? src_func : "", sizeof param.func);
   strncpy((void *)param.name, src_name, sizeof param.name);
   param.size = size;

   rc = ioctl(fd, PKA_IOC_GETPARAM, &param);
   if (rc == -1)
      return -1;

   strncpy((void *)param.func, dst_func ? dst_func : "", sizeof param.func);
   strncpy((void *)param.name, dst_name, sizeof param.name);

   return ioctl(fd, PKA_IOC_SETPARAM, &param);
}

int elppka_test_flag(int fd, const char *func, const char *name)
{
   struct pka_param param;

   strncpy((void *)param.func, func ? func : "", sizeof param.func);
   strncpy((void *)param.name, name, sizeof param.name);
   param.size = 0;

   return ioctl(fd, PKA_IOC_TESTF, &param);
}

int elppka_set_flag(int fd, const char *func, const char *name)
{
   struct pka_flag flag;

   strncpy((void *)flag.func, func ? func : "", sizeof flag.func);
   strncpy((void *)flag.name, name, sizeof flag.name);
   flag.op = PKA_FLAG_OP_SET;

   return ioctl(fd, PKA_IOC_SETF, &flag);
}

/*
 * Read the next input or output parameter specification from a va_list,
 * skipping over outputs when looking for inputs and vice-versa.  Returns
 * a pointer to the argument data, or NULL when the end of the argument
 * list is reached.
 */
static void *read_va_param(int output, struct pka_param *param,
                           const char *func, va_list *ap)
{
   while (1) {
      const char *name;
      void *data;

      name = va_arg(*ap, char *);
      if (!name)
         return NULL;
      data = va_arg(*ap, void *);

      /* Skip parameters we're not interested in. */
      if ((output && name[0] != '=') || (!output && name[0] == '='))
         continue;

      if (name[0] == '=')
         name++;

      if (name[0] == '%') {
         /* Absolute register */
         strncpy((void *)param->func, "",     sizeof param->func);
         strncpy((void *)param->name, name+1, sizeof param->name);
      } else {
         strncpy((void *)param->func, func,   sizeof param->func);
         strncpy((void *)param->name, name,   sizeof param->name);
      }

      return data;
   }
}

/*
 * Submit all inputs from a va_list into the PKA driver.
 */
static int load_inputs(int fd, const char *func, unsigned size, va_list *ap)
{
   struct pka_param param;
   const void *data;
   int rc;

   param.size = size;
   while ((data = read_va_param(0, &param, func, ap)) != NULL) {
      memcpy(param.value, data, size);

      rc = ioctl(fd, PKA_IOC_SETPARAM, &param);
      if (rc == -1)
         return -1;
   }

   return 0;
}

/*
 * Fetch all outputs from the driver, storing them into appropriate va_list
 * entries.
 */
static int unload_outputs(int fd, const char *func, unsigned size, va_list *ap)
{
   struct pka_param param;
   void *data;
   int rc;

   param.size = size;
   while ((data = read_va_param(1, &param, func, ap)) != NULL) {
      rc = ioctl(fd, PKA_IOC_GETPARAM, &param);
      if (rc == -1)
         return -1;

      memcpy(data, param.value, size);
   }

   return 0;
}

/*
 * Start a PKA operation and block waiting for the result.  Returns the
 * (non-negative) PKA status on success, or -1 on failure.
 */
static int do_call(int fd, const char *func, unsigned size)
{
   struct pka_param param;
   int rc;

   strncpy((void *)param.func, func, sizeof param.func);
   param.size = size;

   rc = ioctl(fd, PKA_IOC_CALL, &param);
   if (rc == -1)
      return rc;

   while (1) {
      rc = ioctl(fd, PKA_IOC_WAIT);
      if (rc == -1 && errno == EINTR)
         continue;

      return rc;
   }
}

int elppka_vrun(int fd, const char *func, unsigned size, va_list ap)
{
   va_list ap2;
   int rc;

   if (size > sizeof ((struct pka_param*)0)->value) {
      errno = EINVAL;
      return -1;
   }

   va_copy(ap2, ap);
   rc = load_inputs(fd, func, size, &ap2);
   va_end(ap2);
   if (rc == -1)
      return -1;

   rc = do_call(fd, func, size);
   if (rc != 0)
      return rc;

   va_copy(ap2, ap);
   rc = unload_outputs(fd, func, size, &ap2);
   va_end(ap2);
   if (rc == -1)
      return -1;

   return 0;
}

int elppka_run(int fd, const char *func, unsigned size, ...)
{
   va_list ap;
   int ret;

   va_start(ap, size);
   ret = elppka_vrun(fd, func, size, ap);
   va_end(ap);

   return ret;
}
