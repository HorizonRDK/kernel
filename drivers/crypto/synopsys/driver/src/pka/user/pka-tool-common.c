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

#include <config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include <errno.h>

#include "common.h"

static const char *toolname, *invokename;

void pka_tool_init(const char *name, int argc, char **argv)
{
   toolname = invokename = name;
   if (argc > 0)
      invokename = argv[0];
}

const char *pka_tool_name(void)
{
   assert(invokename);
   return invokename;
}

void pka_tool_version(const char *version_str)
{
   assert(toolname);

   printf("%s (%s) %s\n", toolname, PACKAGE_NAME,
                          version_str ? version_str : PACKAGE_VERSION);
   puts("Copyright (C) 2015 Synopsys Inc.");
}

static int pka_tool_vmsg_(FILE *f, int err, const char *fmt, va_list ap)
{
   size_t tool_len, fmt_len = 0, err_len = 0;
   char *newfmt, *errmsg = NULL;
   int ret;

   assert(invokename);
   tool_len = strlen(invokename);

   if (fmt) {
      fmt_len = strlen(": ") + strlen(fmt);
   }

   if (err >= 0) {
      errmsg = strerror(err);
      err_len = strlen(": ") + strlen(errmsg);
   }

   newfmt = malloc(tool_len + fmt_len + err_len + sizeof "\n");
   if (!newfmt)
      return -1;

   if (errmsg && fmt)
      sprintf(newfmt, "%s: %s: %s\n", invokename, fmt, errmsg);
   else if (errmsg)
      sprintf(newfmt, "%s: %s\n", invokename, errmsg);
   else if (fmt)
      sprintf(newfmt, "%s: %s\n", invokename, fmt);
   else
      sprintf(newfmt, "%s\n", invokename);

   ret = vfprintf(f, newfmt, ap);

   free(newfmt);

   return ret;
}

int pka_tool_vmsg(const char *fmt, va_list ap)
{
   return pka_tool_vmsg_(stdout, -1, fmt, ap);
}

int pka_tool_msg(const char *fmt, ...)
{
   va_list ap;
   int ret;

   va_start(ap, fmt);
   ret = pka_tool_vmsg(fmt, ap);
   va_end(ap);

   return ret;
}

int pka_tool_verr(int err, const char *fmt, va_list ap)
{
   if (err == 0)
      err = errno;

   return pka_tool_vmsg_(stderr, err, fmt, ap);
}

int pka_tool_err(int err, const char *fmt, ...)
{
   va_list ap;
   int ret;

   va_start(ap, fmt);
   ret = pka_tool_verr(err, fmt, ap);
   va_end(ap);

   return ret;
}
