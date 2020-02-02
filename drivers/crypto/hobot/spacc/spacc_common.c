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

#include <linux/printk.h>
#include "elpspacc.h"

static void spacc_hexdump(unsigned char *buf, unsigned int len)
{
	print_hex_dump_debug("", DUMP_PREFIX_OFFSET, 16, 1, buf, len, false);
}

void spacc_dump_regs(spacc_device *spacc)
{
   pr_debug("--- dump regs ------\n");
   pr_debug("SPACC_REG_PROC_LEN:     0x%lx\n", pdu_io_read32(spacc->regmap + SPACC_REG_PROC_LEN)      );
   pr_debug("SPACC_REG_ICV_LEN:      0x%lx\n", pdu_io_read32(spacc->regmap + SPACC_REG_ICV_LEN)       );
   pr_debug("SPACC_REG_ICV_OFFSET:   0x%lx\n", pdu_io_read32(spacc->regmap + SPACC_REG_ICV_OFFSET)    );
   pr_debug("SPACC_REG_PRE_AAD_LEN:  0x%lx\n", pdu_io_read32(spacc->regmap + SPACC_REG_PRE_AAD_LEN)   );
   pr_debug("SPACC_REG_POST_AAD_LEN: 0x%lx\n", pdu_io_read32(spacc->regmap + SPACC_REG_POST_AAD_LEN)  );
   pr_debug("SPACC_REG_IV_OFFSET:    0x%lx\n", pdu_io_read32(spacc->regmap + SPACC_REG_IV_OFFSET)     );
   pr_debug("SPACC_REG_OFFSET:       0x%lx\n", pdu_io_read32(spacc->regmap + SPACC_REG_OFFSET)        );
   pr_debug("SPACC_REG_AUX_INFO:     0x%lx\n", pdu_io_read32(spacc->regmap + SPACC_REG_AUX_INFO)      );
   pr_debug("SPACC_REG_SW_CTRL:      0x%lx\n", pdu_io_read32(spacc->regmap + SPACC_REG_SW_CTRL)       );
}

void spacc_dump_job(spacc_job *job)
{
   pr_debug("--- dump job -------\n");
   pr_debug("enc_mode: %lu, hash_mode: %lu, icv_len:%lu, icv_offset: %lu\n",
         job->enc_mode, job->hash_mode, job->icv_len, job->icv_offset);
   pr_debug("op:%lx, ctrl:%lx, first_use:%lu, pre_aad_sz:%lu, post_aad_sz:%lu\n",
         job->op, job->ctrl, job->first_use, job->pre_aad_sz, job->post_aad_sz);
   pr_debug("hkey_sz: %lu, ckey_sz: 0x%lx, ctx_idx:%u\n",
         job->hkey_sz, job->ckey_sz, job->ctx_idx);
   pr_debug("job_used: %u, job_swid: %u, job_done: %u, job_err: %u, job_secure: %u\n",
         job->job_used, job->job_swid, job->job_done, job->job_err, job->job_secure);
}


//common error message handling
static unsigned char *elpcommon_error_msg (int err)
{
  unsigned char *msg = NULL;

  switch (err) {
  case CRYPTO_OK:
    msg = (unsigned char *) "Operation has succeded";
    break;
  case CRYPTO_FAILED:
    msg = (unsigned char *) "Operation has failed";
    break;
  case CRYPTO_INPROGRESS:
    msg = (unsigned char *) "Operation in progress";
    break;
  case CRYPTO_INVALID_HANDLE:
    msg = (unsigned char *) "Invalid handle";
    break;
  case CRYPTO_INVALID_CONTEXT:
    msg = (unsigned char *) "Invalid context";
    break;
  case CRYPTO_INVALID_SIZE:
    msg = (unsigned char *) "Invalid size";
    break;
  case CRYPTO_NOT_INITIALIZED:
    msg = (unsigned char *) "Crypto library has not been initialized";
    break;
  case CRYPTO_NO_MEM:
    msg = (unsigned char *) "No context memory";
    break;
  case CRYPTO_INVALID_ALG:
    msg = (unsigned char *) "Algorithm is not supported";
    break;
  case CRYPTO_INVALID_KEY_SIZE:
    msg = (unsigned char *) "Invalid key size";
    break;
  case CRYPTO_INVALID_ARGUMENT:
    msg = (unsigned char *) "Invalid argument";
    break;
  case CRYPTO_MODULE_DISABLED:
    msg = (unsigned char *) "Crypto module disabled";
    break;
  case CRYPTO_NOT_IMPLEMENTED:
    msg = (unsigned char *) "Function is not implemented";
    break;
  case CRYPTO_INVALID_BLOCK_ALIGNMENT:
    msg = (unsigned char *) "Invalid block alignment";
    break;
  case CRYPTO_INVALID_MODE:
    msg = (unsigned char *) "Invalid mode";
    break;
  case CRYPTO_INVALID_KEY:
    msg = (unsigned char *) "Invalid key";
    break;
  case CRYPTO_AUTHENTICATION_FAILED:
    msg = (unsigned char *) "Authentication failed";
    break;
  case CRYPTO_INVALID_IV_SIZE:
    msg = (unsigned char *) "Invalid IV size";
    break;
  case CRYPTO_MEMORY_ERROR:
    msg = (unsigned char *) "Internal Memory Error";
    break;
  case CRYPTO_CMD_FIFO_INACTIVE:
    msg = (unsigned char *) "Command FIFO inactive";
    break;
  case CRYPTO_FIFO_FULL:
    msg = (unsigned char *) "Command FIFO is full";
    break; 
  default:
    msg = (unsigned char *) "Invalid error code";
    break;
  }
  return msg;
}


// Returns a user friendly error message for a correspponding error code
unsigned char *spacc_error_msg (int err)
{
   unsigned char *msg = NULL;
   if (err > CRYPTO_COMMON_ERROR_MAX) {
      msg = (unsigned char *) elpcommon_error_msg (err);
   } else {
      // Error codes specific to the SPAcc
      switch (err) {
         default:
            msg = (unsigned char *) "Invalid error code";
            break;
      }
   }
   return msg;
}

// The following function returns the current library version value.
// XXXXVVRR
// XXXX - Build Date
// VV    - Version
// RR    - Release
#if defined(BDATE) && defined(VERSION) && defined(RELEASE) && 0
uint32_t spacc_get_version (void)
{
   return (uint32_t) (BDATE << 16) | (((uint32_t) (0x000000FF) & (uint32_t) (VERSION)) << 8) | ((0x000000FF) & (uint32_t) (RELEASE));
}
#endif
