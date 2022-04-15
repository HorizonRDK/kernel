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
 * Copyright (c) 2011-2017 Synopsys, Inc. and/or its affiliates.
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

// include spacc modes if we are in userspace
#ifndef _ELPSPACCMODES_H_
   #include "elpspaccmodes.h"
#endif

// no __user in userspace
#ifndef KERNEL
   #ifndef __user
      #define __user
   #endif
#endif

enum {
   ELP_SPACC_USR_INIT=100,       // set keys/mode
   ELP_SPACC_USR_PROCESS_IV,     // process packet and reset IV (not used with ivimport)
   ELP_SPACC_USR_PROCESS,        // process packet
   ELP_SPACC_USR_REGISTER,       // register a state key with this connection
   ELP_SPACC_USR_BIND,           // bind this connection to another
   ELP_SPACC_USR_FEATURES,       // get features out of the SPAcc config
};

#define ELP_SPACC_USR_MAX_DDT 8  // max userspace segments per src/dst

struct elp_spacc_usr_ddt {
   void __user *ptr;
   int          len;
};

#define ELP_SPACC_SRC_DDT(fd, x, addr, len) do { (fd)->io.src[x] = (struct elp_spacc_usr_ddt){(addr),(len)}; } while (0);
#define ELP_SPACC_DST_DDT(fd, x, addr, len) do { (fd)->io.dst[x] = (struct elp_spacc_usr_ddt){(addr),(len)}; } while (0);
#define ELP_SPACC_SRC_TERM(fd, x) do { (fd)->io.src[x] = (struct elp_spacc_usr_ddt){NULL,0}; } while (0);
#define ELP_SPACC_DST_TERM(fd, x) do { (fd)->io.dst[x] = (struct elp_spacc_usr_ddt){NULL,0}; } while (0);

#define ELP_SPACC_SRCLEN_SET(fd, x) (fd)->io.srclen = x
#define ELP_SPACC_DSTLEN_SET(fd, x) (fd)->io.dstlen = x

#define ELP_SPACC_SRCLEN_RESET(fd) (fd)->io.srclen = -1
#define ELP_SPACC_DSTLEN_RESET(fd) (fd)->io.dstlen = -1

enum {
   SPACC_MAP_HINT_TEST=0,  // test internally if there is valid overlap
   SPACC_MAP_HINT_USESRC,  // there is overlap and use the source mapping (because the destination mapping is contained inside it)
   SPACC_MAP_HINT_USEDST,  // there is overlap and use the destination mapping
   SPACC_MAP_HINT_NOLAP    // there is no overlap so don't bother trying to test for it
};

// Message begin/end flags
#define ELP_SPACC_USR_MSG_BEGIN 1
#define ELP_SPACC_USR_MSG_END   2

// handy macros for setting the various partial processing states
#define ELP_SPACC_USR_MSG_DEFAULT(fd) (fd)->io.partial = ELP_SPACC_USR_MSG_BEGIN|ELP_SPACC_USR_MSG_END
#define ELP_SPACC_USR_MSG_FIRST(fd)   (fd)->io.partial = ELP_SPACC_USR_MSG_BEGIN
#define ELP_SPACC_USR_MSG_MIDDLE(fd)  (fd)->io.partial = 0
#define ELP_SPACC_USR_MSG_LAST(fd)    (fd)->io.partial = ELP_SPACC_USR_MSG_END

struct elp_spacc_features {
   unsigned
      project,
      partial,
      version,
      ivimport,
      qos,
      max_msg_size;
   unsigned char
      modes[CRYPTO_MODE_LAST];
};

struct elp_spacc_ioctl {
   unsigned char
                cmd,           // ioctl command
                auxinfo_align, // bit align for 3G modes
                auxinfo_dir,   // direction for 3G modes
                partial;       // MSG_BEGIN/MSG_END flags
   int
                map_hint,
                aad_copy,      // copy the AAD or not
                icv_len,       // ICV length (0 for default)
                icv_mode,      // how to handle the ICV
                ckeylen,       // cipher key length
                civlen,        // cipher IV length
                hkeylen,       // hash key length
                hivlen,        // hash IV length
                encrypt,       // 1 to set encrypt mode
                cipher_mode,   // cipher mode
                hash_mode,     // hash mode
                err;

   struct elp_spacc_usr_ddt
                src[ELP_SPACC_USR_MAX_DDT+1],
                dst[ELP_SPACC_USR_MAX_DDT+1];

   unsigned long          srclen, dstlen;
// despite the DDTs above these need to be filled in to
// make things simpler inside the kernel
   int
                src_offset, dst_offset,      // offsets into buffers
                pre_aad_len, post_aad_len,   // AAD lengths before and after plaintext (if any)
                ivoffset;                    // offset in packet for IV, or -1 to not use import

   unsigned char civ[64];                    // order is important, civ must come before ckey
   unsigned char ckey[64], hkey[128], hiv[128];
   unsigned char state_key[16];              // state key used to allow multiple file handle use the same SPAcc handle
}__attribute__((packed));

struct elp_spacc_usr {
   int fd;
   struct elp_spacc_ioctl io;
};

#ifndef KERNEL

int spacc_dev_open(struct elp_spacc_usr *io,
                   int cipher_mode, int hash_mode,    // cipher and hash (see elpspaccmodes.h)
                   int encrypt,                       // non-zero for encrypt (sign) modes
                   int icvmode,                       // ICV mode (see elpspaccmodes.h)
                   int icvlen,                        // length of ICV, 0 for default length (algorithm dependent)
                   int aad_copy,                      // non-zero to copy AAD to dst buffer
                   const void *ckey, int ckeylen, const void *civ, int civlen,  // cipher key and IVs
                   const void *hkey, int hkeylen, const void *hiv, int hivlen); // hash key and IVs (if any)

#define spacc_dev_setaux(f, align, dir) do { (f)->io.auxinfo_align = align; (f)->io.auxinfo_dir = dir; } while (0);

int spacc_dev_register(struct elp_spacc_usr *io);
int spacc_dev_open_bind(struct elp_spacc_usr *master, struct elp_spacc_usr *new);

int spacc_dev_process_multi(
   struct elp_spacc_usr *io,
   const void    *new_iv,
   int            iv_offset,
   int            pre_aad,
   int            post_aad,
   int            src_offset,
   int            dst_offset,
   int            map_hint);

int spacc_dev_process(
   struct elp_spacc_usr *io,
   const void    *new_iv,                       //pointer to new IV for this job if any, NULL if not
   int            iv_offset,                    //offset of IV for this job within src, -1 if you don't want to use this
   int            pre_aad,                      //PRE AAD size
   int            post_aad,                     //POST AAD size
   int            src_offset,
   int            dst_offset,
   unsigned char *src, unsigned long src_len,
   unsigned char *dst, unsigned long dst_len);

int spacc_dev_close(struct elp_spacc_usr *io);

int spacc_dev_features(struct elp_spacc_features *features);

int spacc_dev_isenabled(struct elp_spacc_features *features, int mode, int keysize);

#endif
