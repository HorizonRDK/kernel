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


#include "elpre.h"
#include "elprehw.h"


/*
 *  Dequeue a packet from the STAT FIFO, if it exists
 *  otherwise return CRYPTO_INPROGRESS;
 */
int re_packet_dequeue (re_device * re, int context)
{
   uint32_t handle, status;
   unsigned long lock_flag;
   int ret = CRYPTO_INPROGRESS;
   int jobs;
   elp_re_ctx *ctx = NULL;

   handle = -1;
   if (context >= re->config.num_ctx) {
      return CRYPTO_INVALID_CONTEXT;
   }
   if (context != -1 && re->re_contexts[context].jobdone == 1) {
      return re->re_contexts[context].curjob_swid;
   }

   PDU_LOCK(&re->lock, lock_flag);
   while ((jobs = RE_FIFO_STAT_CNT (pdu_io_read32 (re->regmap + RE_FIFO_STAT))) > 0) {
      while (jobs--) {
         uint32_t jobid;
         pdu_io_write32 (re->regmap + RE_POP, 1);
         status = pdu_io_read32 (re->regmap + RE_STATUS);
         jobid  = RE_STAT_SW_ID (status);
         handle = re->jobid_to_ctx[jobid];
         if (handle == -1) {
            ret = CRYPTO_INVALID_CONTEXT;
            goto ERR;
         }
         re->jobid_to_ctx[jobid] = -1;
         ctx = &re->re_contexts[handle];

         ret = RE_STAT_RET_CODE(status);
         ctx->jobdone = 1;

         switch (ret) {
            case RE_ERR_MAC_FAIL:
               ELPHW_PRINT ("re_error: Mac Failed: read channel disabled\n");
               ctx->ret_stat = CRYPTO_AUTHENTICATION_FAILED;
               break;
            case RE_ERR_MEM_ERROR:
               ELPHW_PRINT ("re_error: Memory Error: Source DDT did not have enough information\n");
               ctx->ret_stat = CRYPTO_MEMORY_ERROR;
               break;
            case RE_ERR_BAD_PADDING:
               ELPHW_PRINT ("re_error: Bad Padding: read channel disabled\n");
               ctx->ret_stat = CRYPTO_INVALID_PAD;
               break;
            case RE_ERR_FATAL_ALERT:
               ELPHW_PRINT ("re_error: Fatal Alert: Payload contained a fatal alert, read and write channels are disabled\n");
               ctx->ret_size = RE_STAT_LEN (pdu_io_read32 (re->regmap + RE_STATUS));
               ctx->ret_prot = RE_STAT_PROT (pdu_io_read32 (re->regmap + RE_STATUS));
               ctx->ret_stat = CRYPTO_FATAL;
               break;
            case RE_ERR_UNKNOWN_PROT:
               ELPHW_PRINT ("re_error: Unknown protocol: Decrypted successfully\n");
               ctx->ret_size = RE_STAT_LEN (pdu_io_read32 (re->regmap + RE_STATUS));
               ctx->ret_prot = RE_STAT_PROT (pdu_io_read32 (re->regmap + RE_STATUS));
               ctx->ret_stat = CRYPTO_INVALID_PROTOCOL;
               break;
            case RE_ERR_BAD_VERSION:
               ELPHW_PRINT ("re_error: Bad Version: Read channel is disabled\n");
               ctx->ret_stat = CRYPTO_INVALID_VERSION;
               break;
            case RE_ERR_BAD_LENGTH:
               ELPHW_PRINT ("re_error: Bad Length: Incorrect length for block cipher, read chanel disabled\n");
               ctx->ret_stat = CRYPTO_INVALID_BLOCK_ALIGNMENT;
               break;
            case RE_ERR_INACTIVE:
               ELPHW_PRINT ("re_error: Read or Write channel has been disabled: record not processed\n");
               ctx->ret_stat = CRYPTO_DISABLED;
               break;
            case RE_ERR_SEQ_OVFL:
               ELPHW_PRINT ("re_error: Sequence Number Overflowed: record processed, but read chanel disabled\n");
               ctx->ret_size = RE_STAT_LEN (pdu_io_read32 (re->regmap + RE_STATUS));
               ctx->ret_prot = RE_STAT_PROT (pdu_io_read32 (re->regmap + RE_STATUS));
               ctx->ret_stat = CRYPTO_SEQUENCE_OVERFLOW;
               break;
            case RE_ERR_REPLAY:
               ELPHW_PRINT ("re_error: Replay Detected\n");
               ctx->ret_size = RE_STAT_LEN (pdu_io_read32 (re->regmap + RE_STATUS));
               ctx->ret_prot = RE_STAT_PROT (pdu_io_read32 (re->regmap + RE_STATUS));
               ctx->ret_stat = CRYPTO_REPLAY;
               break;
            case RE_ERR_OK:
               ctx->ret_size = RE_STAT_LEN (pdu_io_read32 (re->regmap + RE_STATUS));
               ctx->ret_prot = RE_STAT_PROT (pdu_io_read32 (re->regmap + RE_STATUS));
               ctx->ret_stat = CRYPTO_OK;
               break;
            default:
               ELPHW_PRINT ("THIS SHOULD NEVER BE REACHED!!! %d \n", ret);
               ret = CRYPTO_FAILED;
               break;
         }

         if (ctx->cb) {
            PDU_UNLOCK(&re->lock, lock_flag);
            ctx->cb(re, ctx->cbdata, ctx->ret_stat, ctx->ret_size);
            PDU_LOCK(&re->lock, lock_flag);
         }
      }
   }

   if (handle == -1) {
      goto ERR;
   }

   if (handle == context && ctx != NULL) {
      ret = ctx->ret_stat;
      goto ERR;
   }

   if (context == -1) {
      ret = CRYPTO_OK;
      goto ERR;
   }

   PDU_UNLOCK(&re->lock, lock_flag);
   return (re_packet_dequeue (re, context)); //If we do not match, then we try and dequeue the next packet, eventually
   //We will hit the one for our context.
ERR:
   PDU_UNLOCK(&re->lock, lock_flag);
   return ret;
}
