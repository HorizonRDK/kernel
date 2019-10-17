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
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "elpreuser.h"

static const struct {
   int k_err, h_err;
} error_mapping[] = {
   { CRYPTO_OK,                      RE_ERR_OK },
   { CRYPTO_AUTHENTICATION_FAILED,   RE_ERR_MAC_FAIL },
   { CRYPTO_MEMORY_ERROR,            RE_ERR_MEM_ERROR },
   { CRYPTO_INVALID_PAD,             RE_ERR_BAD_PADDING },
   { CRYPTO_FATAL,                   RE_ERR_FATAL_ALERT },
   { CRYPTO_INVALID_PROTOCOL,        RE_ERR_UNKNOWN_PROT },
   { CRYPTO_INVALID_VERSION,         RE_ERR_BAD_VERSION },
   { CRYPTO_INVALID_BLOCK_ALIGNMENT, RE_ERR_BAD_LENGTH },
   { CRYPTO_DISABLED,                RE_ERR_INACTIVE },
   { CRYPTO_SEQUENCE_OVERFLOW,       RE_ERR_SEQ_OVFL },
   { CRYPTO_REPLAY,                  RE_ERR_REPLAY },
} ;

int re_dev_error_map(int k_err)
{
   unsigned x;
   for (x = 0; x < (sizeof(error_mapping)/sizeof(error_mapping[0])); x++) {
      if (error_mapping[x].k_err == k_err) {
         return error_mapping[x].h_err;
      }
   }
   return -1;
}

int re_dev_open(struct elp_spacc_re_usr *io, int version)
{
   int err;

   memset(io, 0, sizeof *io);

   io->fd = open("/dev/spaccreusr", O_RDWR);
   if (io->fd < 1) {
      return io->fd;
   }
   io->io.version = version;
   err = ioctl(io->fd, ELP_SPACC_RE_USR_INIT, &io->io);
   if (err < 0) {
      close(io->fd);
      return err;
   }
}

int re_dev_register(struct elp_spacc_re_usr *io)
{
   int fd, x;
   fd = open("/dev/urandom", O_RDWR);
   if (fd < 0) {
      return fd;
   }
   x = read(fd, io->io.state_key, 16);
   close(fd);
   if (x < 16) {
      return -1;
   }
   return ioctl(io->fd, ELP_SPACC_RE_USR_REGISTER, &io->io);
}

int re_dev_open_bind(struct elp_spacc_re_usr *master, struct elp_spacc_re_usr *new)
{
   int err;
   memcpy(new, master, sizeof *new);
   new->fd = open("/dev/spaccreusr", O_RDWR);
   if (new->fd < 0) {
      memset(new, 0, sizeof *new);
      return -1;
   }
   err = ioctl(new->fd, ELP_SPACC_RE_USR_BIND, &new->io);
   if (err < 0) {
      close(new->fd);
      memset(new, 0, sizeof *new);
   }
   return err;
}


int re_dev_set_context(struct elp_spacc_re_usr *io, int dir,
                   unsigned char *iv,              uint32_t ivlen,
                   unsigned char *key,             uint32_t keylen,
                   unsigned char *mackey,          uint32_t mackeylen,
                   unsigned char *params,          uint32_t paramlength,
                   unsigned char *sequence_number, uint32_t seqlength)
{
   if (ivlen     > sizeof(io->io.civ)  ||
       keylen    > sizeof(io->io.ckey) ||
       mackeylen > sizeof(io->io.hkey) ||
       paramlength != 2 ||
       seqlength   != 8) {
      return -1;
   }

   memcpy(io->io.civ,             iv,              ivlen);
   memcpy(io->io.ckey,            key,             keylen);
   memcpy(io->io.hkey,            mackey,          mackeylen);
   memcpy(io->io.params,          params,          2);
   memcpy(io->io.sequence_number, sequence_number, 8);

   io->io.ckeylen = keylen;
   io->io.civlen  = ivlen;
   io->io.hkeylen = mackeylen;

   return ioctl(io->fd, dir == RE_DEV_DIR_WRITE ? ELP_SPACC_RE_USR_WRITE_CONTEXT : ELP_SPACC_RE_USR_READ_CONTEXT, &io->io);
}

int re_dev_do_record(struct elp_spacc_re_usr *io,
                     const unsigned char *in,        uint32_t  inlen,
                           unsigned char *out,       uint32_t  outlen,
                                     int src_offset,      int  dst_offset,
                                     int cmd)
{
   int hint;

   io->io.src[0].ptr = in;
   io->io.src[0].len = inlen + src_offset;
   io->io.src[1].ptr = NULL;
   io->io.srclen     = inlen;

   io->io.dst[0].ptr = out;
   io->io.dst[0].len = outlen + dst_offset;
   io->io.dst[1].ptr = NULL;
   io->io.dstlen     = outlen;

   if (in == out) {
      if (io->io.src[0].len > io->io.dst[0].len) {
         hint = RE_MAP_HINT_USESRC;
      } else {
         hint = RE_MAP_HINT_USEDST;
      }
   } else {
      hint = RE_MAP_HINT_NOLAP;
   }

   return re_dev_do_record_multi(io, src_offset, dst_offset, hint, cmd);
}

int re_dev_do_record_multi(struct elp_spacc_re_usr *io, int src_offset, int dst_offset, int hint, int cmd)
{
   int err, x;

   io->io.src_offset = src_offset;
   io->io.dst_offset = dst_offset;
   io->io.cmd        = cmd;
   io->io.map_hint   = hint;

   if (io->io.srclen == -1) {
      for (io->io.srclen = x = 0; x < ELP_SPACC_RE_USR_MAX_DDT && io->io.src[x].ptr != NULL; ++x) {
         io->io.srclen += io->io.src[x].len;
      }
   }

   if (io->io.dstlen == -1) {
      for (io->io.dstlen = x = 0; x < ELP_SPACC_RE_USR_MAX_DDT && io->io.dst[x].ptr != NULL; ++x) {
         io->io.dstlen += io->io.dst[x].len;
      }
   }

   err = ioctl(io->fd, ELP_SPACC_RE_USR_DATA_OP, &io->io);

   if (err >= 0) {
      return err;
   } else {
      return io->io.err;
   }
}

int re_dev_close(struct elp_spacc_re_usr *io)
{
   close(io->fd);
   memset(io, 0, sizeof *io);
}

