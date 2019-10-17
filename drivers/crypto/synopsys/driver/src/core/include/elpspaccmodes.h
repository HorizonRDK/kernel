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
 * Copyright (c) 2015-2018 Synopsys, Inc. and/or its affiliates.
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

#ifndef _ELPSPACCMODES_H_
#define _ELPSPACCMODES_H_

enum eicvpos
{
  IP_ICV_OFFSET = 0,
  IP_ICV_APPEND = 1,
  IP_ICV_IGNORE = 2,
  IP_MAX
};

enum {
  ICV_HASH = 0,             /* HASH of plaintext */
  ICV_HASH_ENCRYPT = 1,     /* HASH the plaintext and encrypt the plaintext and ICV */
  ICV_ENCRYPT_HASH = 2,     /* HASH the ciphertext */
  ICV_IGNORE=3,
  IM_MAX,
};

enum crypto_modes {
  CRYPTO_MODE_NULL,
  CRYPTO_MODE_RC4_40,
  CRYPTO_MODE_RC4_128,
  CRYPTO_MODE_RC4_KS,
  CRYPTO_MODE_AES_ECB,
  CRYPTO_MODE_AES_CBC,
  CRYPTO_MODE_AES_CTR,
  CRYPTO_MODE_AES_CCM,
  CRYPTO_MODE_AES_GCM,
  CRYPTO_MODE_AES_F8,
  CRYPTO_MODE_AES_XTS,
  CRYPTO_MODE_AES_CFB,
  CRYPTO_MODE_AES_OFB,
  CRYPTO_MODE_AES_CS1,
  CRYPTO_MODE_AES_CS2,
  CRYPTO_MODE_AES_CS3,
  CRYPTO_MODE_MULTI2_ECB,
  CRYPTO_MODE_MULTI2_CBC,
  CRYPTO_MODE_MULTI2_OFB,
  CRYPTO_MODE_MULTI2_CFB,
  CRYPTO_MODE_3DES_CBC,
  CRYPTO_MODE_3DES_ECB,
  CRYPTO_MODE_DES_CBC,
  CRYPTO_MODE_DES_ECB,
  CRYPTO_MODE_KASUMI_ECB,
  CRYPTO_MODE_KASUMI_F8,
  CRYPTO_MODE_SNOW3G_UEA2,
  CRYPTO_MODE_ZUC_UEA3,
  CRYPTO_MODE_CHACHA20_STREAM,
  CRYPTO_MODE_CHACHA20_POLY1305,

  CRYPTO_MODE_HASH_MD5,
  CRYPTO_MODE_HMAC_MD5,
  CRYPTO_MODE_HASH_SHA1,
  CRYPTO_MODE_HMAC_SHA1,
  CRYPTO_MODE_HASH_SHA224,
  CRYPTO_MODE_HMAC_SHA224,
  CRYPTO_MODE_HASH_SHA256,
  CRYPTO_MODE_HMAC_SHA256,
  CRYPTO_MODE_HASH_SHA384,
  CRYPTO_MODE_HMAC_SHA384,
  CRYPTO_MODE_HASH_SHA512,
  CRYPTO_MODE_HMAC_SHA512,
  CRYPTO_MODE_HASH_SHA512_224,
  CRYPTO_MODE_HMAC_SHA512_224,
  CRYPTO_MODE_HASH_SHA512_256,
  CRYPTO_MODE_HMAC_SHA512_256,

  CRYPTO_MODE_MAC_XCBC,
  CRYPTO_MODE_MAC_CMAC,
  CRYPTO_MODE_MAC_KASUMI_F9,
  CRYPTO_MODE_MAC_SNOW3G_UIA2,
  CRYPTO_MODE_MAC_ZUC_UIA3,
  CRYPTO_MODE_MAC_POLY1305,

  CRYPTO_MODE_SSLMAC_MD5,
  CRYPTO_MODE_SSLMAC_SHA1,
  CRYPTO_MODE_HASH_CRC32,
  CRYPTO_MODE_MAC_MICHAEL,

  CRYPTO_MODE_HASH_SHA3_224,
  CRYPTO_MODE_HASH_SHA3_256,
  CRYPTO_MODE_HASH_SHA3_384,
  CRYPTO_MODE_HASH_SHA3_512,

  CRYPTO_MODE_HASH_SHAKE128,
  CRYPTO_MODE_HASH_SHAKE256,
  CRYPTO_MODE_HASH_CSHAKE128,
  CRYPTO_MODE_HASH_CSHAKE256,
  CRYPTO_MODE_MAC_KMAC128,
  CRYPTO_MODE_MAC_KMAC256,
  CRYPTO_MODE_MAC_KMACXOF128,
  CRYPTO_MODE_MAC_KMACXOF256,

  CRYPTO_MODE_LAST
};

#endif
