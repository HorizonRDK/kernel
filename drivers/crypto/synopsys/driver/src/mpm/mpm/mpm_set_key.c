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

#include "elpmpm.h"
int mpm_set_key(mpm_device *mpm, int key_idx, int ckey_sz, int ckey_copysz, int hkey_sz, int hkey_copysz, int no_cache, int inv_ckey, unsigned char *ckey, unsigned char *hkey)
{
   uint32_t ctrl;

   if (key_idx < 0 || key_idx > mpm->config.no_keys) {
      ELPHW_PRINT("mpm_set_key::Invalid key idx\n");
      return -1;
   }

   ctrl = KEY_BUF_CTRL_CKEY_SZ(ckey_sz) |
          KEY_BUF_CTRL_HKEY_SZ(hkey_sz) |
          (no_cache ? KEY_BUF_CTRL_NOCACHE : 0) |
          (inv_ckey ? KEY_BUF_CTRL_INV_CKEY: 0);

   mpm->key.keys[key_idx][0] = ctrl;

   if (ckey) {
      memcpy(&mpm->key.keys[key_idx][0x40/4], ckey, ckey_copysz);
   }
   if (hkey) {
      memcpy(&mpm->key.keys[key_idx][0x80/4], hkey, hkey_copysz);
   }
   pdu_sync_single_for_device(mpm->key.keys_phys + (key_idx * sizeof(keybuf)), sizeof(keybuf));

#if 0
{
   int x;
   for (x = 0; x < 0x100/4; x++) {
       printk(KERN_DEBUG "key[0x%03x] = %08zx\n", x*4, mpm->key.keys[key_idx][x]);
   }
}
#endif

   return 0;
}

