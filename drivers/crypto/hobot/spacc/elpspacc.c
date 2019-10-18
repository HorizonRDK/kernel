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
 * Copyright (c) 2011-2018 Synopsys, Inc. and/or its affiliates.
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

#include "elpspacc.h"

static int spacc_xof_stringsize_autodetect(spacc_device *spacc)
{
   pdu_ddt     ddt;
   dma_addr_t  dma;
   void        *virt;
   int         ss, alg, i;
   unsigned long spacc_ctrl[2] = {0xF400B400, 0xF400D400};
   unsigned char buf[256];
   unsigned long buflen;
   unsigned char test_str[6] = {0x01, 0x20, 0x54, 0x45, 0x53, 0x54};
   unsigned char md[2][16] = {{0xc3, 0x6d, 0x0a, 0x88, 0xfa, 0x37, 0x4c, 0x9b, 0x44, 0x74, 0xeb, 0x00, 0x5f, 0xe8, 0xca, 0x25},
                              {0x68, 0x77, 0x04, 0x11, 0xf8, 0xe3, 0xb0, 0x1e, 0x0d, 0xbf, 0x71, 0x6a, 0xe9, 0x87, 0x1a, 0x0d}};

   // get memory
   virt = pdu_dma_alloc(256, &dma);
   if (!virt) {
      return CRYPTO_FAILED;
   }
   if (pdu_ddt_init(&ddt, 1)) {
      pdu_dma_free(256, virt, dma);
      return CRYPTO_FAILED;
   }
   pdu_ddt_add(&ddt, dma, 256);

   // populate registers for jobs
   pdu_io_write32(spacc->regmap + SPACC_REG_SRC_PTR, (uint32_t)ddt.phys);
   pdu_io_write32(spacc->regmap + SPACC_REG_DST_PTR, (uint32_t)ddt.phys);
   pdu_io_write32(spacc->regmap + SPACC_REG_PROC_LEN,     16);
   pdu_io_write32(spacc->regmap + SPACC_REG_PRE_AAD_LEN,  16);
   pdu_io_write32(spacc->regmap + SPACC_REG_ICV_LEN,      16);
   pdu_io_write32(spacc->regmap + SPACC_REG_KEY_SZ,       6);
   pdu_io_write32(spacc->regmap + SPACC_REG_SW_CTRL,      0);

   // repeat for 2 algorithms, CSHAKE128 and KMAC128
   for (alg = 0; (alg < 2) && (spacc->config.string_size == 0); alg++) {
      // repeat for 4 string_size sizes
      for (ss = 0; ss < 4; ss++) {
         buflen = (32UL << ss);
         if (buflen > spacc->config.hash_page_size) break;

         // clear I/O memory
         memset(virt, 0, 256);

         // clear buf and then insert test string
         memset(buf, 0, sizeof(buf));
         memcpy(buf, test_str, sizeof(test_str));
         memcpy(buf + (buflen >> 1), test_str, sizeof(test_str));

         //write key context
         pdu_to_dev32_s(spacc->regmap + SPACC_CTX_HASH_KEY, buf, spacc->config.hash_page_size >> 2, spacc_endian);

         //write ctrl
         pdu_io_write32(spacc->regmap + SPACC_REG_CTRL, spacc_ctrl[alg]);

         //wait for job complete
         for (i = 0; i < 20; i++) {
            if (!(pdu_io_read32(spacc->regmap + SPACC_REG_FIFO_STAT) & SPACC_FIFO_STAT_STAT_EMPTY)) {
               //check result, if it matches, we have string_size
               pdu_io_write32 (spacc->regmap + SPACC_REG_STAT_POP, 1);
               if ((SPACC_GET_STATUS_RET_CODE(pdu_io_read32 (spacc->regmap + SPACC_REG_STATUS)) == SPACC_OK)
                   && (!memcmp(virt, md[alg], 16))) {
                  spacc->config.string_size = (16 << ss);
               }
               break;
            }
         }
      }
   }

   // reset registers
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_CTRL,     0);
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_EN,       0);
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_STAT,     0xFFFFFFFF);

   pdu_io_write32(spacc->regmap + SPACC_REG_SRC_PTR,      0);
   pdu_io_write32(spacc->regmap + SPACC_REG_DST_PTR,      0);
   pdu_io_write32(spacc->regmap + SPACC_REG_PROC_LEN,     0);
   pdu_io_write32(spacc->regmap + SPACC_REG_ICV_LEN,      0);
   pdu_io_write32(spacc->regmap + SPACC_REG_PRE_AAD_LEN,  0);

   pdu_ddt_free(&ddt);
   pdu_dma_free(256, virt, dma);

   return CRYPTO_OK;
}

static const uint8_t spacc_ctrl_map[SPACC_CTRL_VER_SIZE][SPACC_CTRL_MAPSIZE] =
                       {{ 0, 8, 4, 12, 24, 16, 31, 25, 26, 27, 28, 29, 14, 15 },
                        { 0, 8, 3, 12, 24, 16, 31, 25, 26, 27, 28, 29, 14, 15 },
                        { 0, 4, 8, 13, 15, 16, 24, 25, 26, 27, 28, 29, 30, 31 }};

int spacc_init (void *baseaddr, spacc_device * spacc, pdu_info * info)
{
   unsigned long id;

   if (baseaddr == NULL) {
      ELPHW_PRINT("spacc_init:: baseaddr is NULL\n");
      return -1;
   }
   if (spacc == NULL) {
      ELPHW_PRINT("spacc_init:: spacc is NULL\n");
      return -1;
   }

   memset (spacc, 0, sizeof *spacc);
   PDU_INIT_LOCK(&spacc->lock);
   PDU_INIT_LOCK(&spacc->ctx_lock);

   // assign the baseaddr
   spacc->regmap = baseaddr;

   // version info
   spacc->config.version     = info->spacc_version.version;
   spacc->config.pdu_version = (info->pdu_config.major << 4) | info->pdu_config.minor;
   spacc->config.project     = info->spacc_version.project;
   spacc->config.is_pdu      = info->spacc_version.is_pdu;
   spacc->config.is_qos      = info->spacc_version.qos;

   // hsm specific
   spacc->config.is_hsm_virtual    = info->spacc_version.is_hsm & info->hsm_config.paradigm;
   spacc->config.is_hsm_shared     = info->spacc_version.is_hsm & !(info->hsm_config.paradigm);
   spacc->config.num_sec_ctx       = info->hsm_config.num_ctx;
   spacc->config.sec_ctx_page_size = 4*(1U<<(info->hsm_config.ctx_page_size+2));

   // misc
   spacc->config.is_partial        = info->spacc_version.partial;
   spacc->config.num_ctx           = info->spacc_config.num_ctx;
   spacc->config.num_rc4_ctx       = info->spacc_config.num_rc4_ctx;
   spacc->config.ciph_page_size    = 1U << info->spacc_config.ciph_ctx_page_size;
   spacc->config.hash_page_size    = 1U << info->spacc_config.hash_ctx_page_size;
   spacc->config.dma_type          = info->spacc_config.dma_type;
   spacc->config.idx               = info->spacc_version.vspacc_idx;
   spacc->config.cmd0_fifo_depth   = info->spacc_config.cmd0_fifo_depth;
   spacc->config.cmd1_fifo_depth   = info->spacc_config.cmd1_fifo_depth;
   spacc->config.cmd2_fifo_depth   = info->spacc_config.cmd2_fifo_depth;
   spacc->config.stat_fifo_depth   = info->spacc_config.stat_fifo_depth;
   spacc->config.fifo_cnt          = 1;

   spacc->config.is_ivimport = info->spacc_version.ivimport;

   // ctrl register map
   if (spacc->config.version <= 0x4E) {
      spacc->config.ctrl_map = spacc_ctrl_map[SPACC_CTRL_VER_0];
   } else if (spacc->config.version <= 0x60){
      spacc->config.ctrl_map = spacc_ctrl_map[SPACC_CTRL_VER_1];
   } else {
      spacc->config.ctrl_map = spacc_ctrl_map[SPACC_CTRL_VER_2];
   }

   spacc->job_tally                = 0;
   spacc->wdcnt                    = 0;
   spacc->config.wd_timer          = SPACC_WD_TIMER_INIT;

   // version 4.10 uses IRQ, above uses WD and we don't support below 4.00
   if (spacc->config.version < 0x40) {
      ELPHW_PRINT("spacc_init::Unsupported SPAcc version\n");
      return CRYPTO_FAILED;
   } else if (spacc->config.version < 0x4B) {
      spacc->op_mode                  = SPACC_OP_MODE_IRQ;
   } else {
      spacc->op_mode                  = SPACC_OP_MODE_WD;
   }

   {
     int x;
     spacc->config.ctx_mask = 0;
     for (x = 1; x < spacc->config.num_ctx; x <<= 1) {
       spacc->config.ctx_mask |= x;
     }
   }

   if (spacc->config.is_hsm_virtual || spacc->config.is_hsm_shared) {
      spacc->config.is_secure      = 1;
      spacc->config.is_secure_port = spacc->config.idx;
   } else {
      spacc->config.is_secure      = 0;
      spacc->config.is_secure_port = 0;
   }

/* set threshold and enable irq */
   // on 4.11 and newer cores we can derive this from the HW reported depths.
   if (spacc->config.stat_fifo_depth == 1) {
      spacc->config.ideal_stat_level = 1;
   } else if (spacc->config.stat_fifo_depth <= 4) {
      spacc->config.ideal_stat_level = spacc->config.stat_fifo_depth - 1;
   } else if (spacc->config.stat_fifo_depth <= 8) {
      spacc->config.ideal_stat_level = spacc->config.stat_fifo_depth - 2;
   } else {
      spacc->config.ideal_stat_level = spacc->config.stat_fifo_depth - 4;
   }

/* determine max PROClen value */
   pdu_io_write32(spacc->regmap + SPACC_REG_PROC_LEN, 0xFFFFFFFF);
   spacc->config.max_msg_size = pdu_io_read32(spacc->regmap + SPACC_REG_PROC_LEN);

   // read config info
   if (spacc->config.is_pdu) {
      ELPHW_PRINT("PDU:\n");
      ELPHW_PRINT("   MAJOR      : %u\n", info->pdu_config.major);
      ELPHW_PRINT("   MINOR      : %u\n", info->pdu_config.minor);
   }
   id = pdu_io_read32 (spacc->regmap + SPACC_REG_ID);
   ELPHW_PRINT ("SPACC ID: (%08lx)\n   MAJOR      : %x\n", (unsigned long) id, info->spacc_version.major);
   ELPHW_PRINT ("   MINOR      : %x\n", info->spacc_version.minor);
   ELPHW_PRINT ("   QOS        : %x\n", info->spacc_version.qos);
   ELPHW_PRINT ("   IVIMPORT   : %x\n", spacc->config.is_ivimport);
   if (spacc->config.version >= 0x48) {
      ELPHW_PRINT ("   TYPE       : %lx (%s)\n", SPACC_ID_TYPE (id), (char *[]){"SPACC", "SPACC-PDU", "SPACC-HSM", "TROOT"}[SPACC_ID_TYPE (id)&3]);
   }
   ELPHW_PRINT ("   AUX        : %x\n", info->spacc_version.qos);
   ELPHW_PRINT ("   IDX        : %lx %s\n", SPACC_ID_VIDX (id), spacc->config.is_secure ? ((char *[]){"(Normal Port)", "(Secure Port)"}[spacc->config.is_secure_port&1]) : "");
   ELPHW_PRINT ("   PARTIAL    : %x\n", info->spacc_version.partial);
   ELPHW_PRINT ("   PROJECT    : %x\n", info->spacc_version.project);
   if (spacc->config.version >= 0x48) {
      id = pdu_io_read32 (spacc->regmap + SPACC_REG_CONFIG);
   } else {
      id = 0xFFFFFFFF;
   }
   ELPHW_PRINT ("SPACC CFG: (%08lx)\n", id);
   ELPHW_PRINT ("   CTX CNT    : %u\n", info->spacc_config.num_ctx);
   ELPHW_PRINT ("   RC4 CNT    : %u\n", info->spacc_config.num_rc4_ctx);
   ELPHW_PRINT ("   VSPACC CNT : %u\n", info->spacc_config.num_vspacc);
   ELPHW_PRINT ("   CIPH SZ    : %-3lu bytes\n", 1UL<<info->spacc_config.ciph_ctx_page_size);
   ELPHW_PRINT ("   HASH SZ    : %-3lu bytes\n", 1UL<<info->spacc_config.hash_ctx_page_size);
   ELPHW_PRINT ("   DMA TYPE   : %u (%s)\n", info->spacc_config.dma_type, (char *[]){"Unknown", "Scattergather", "Linear", "Unknown"}[info->spacc_config.dma_type&3]);
   ELPHW_PRINT ("   MAX PROCLEN: %lu bytes\n", (unsigned long)spacc->config.max_msg_size);
   ELPHW_PRINT ("   FIFO CONFIG :\n");
   ELPHW_PRINT ("      CMD0 DEPTH: %d\n", spacc->config.cmd0_fifo_depth);
   if (spacc->config.is_qos) {
      ELPHW_PRINT ("      CMD1 DEPTH: %d\n", spacc->config.cmd1_fifo_depth);
      ELPHW_PRINT ("      CMD2 DEPTH: %d\n", spacc->config.cmd2_fifo_depth);
   }
   ELPHW_PRINT ("      STAT DEPTH: %d\n", spacc->config.stat_fifo_depth);
   if (spacc->config.is_secure) {
      id = pdu_io_read32(spacc->regmap + SPACC_REG_HSM_VERSION);
      ELPHW_PRINT("HSM CFG: (%08lx)\n", id);
      ELPHW_PRINT("   MAJOR     : %x\n", info->hsm_config.major);
      ELPHW_PRINT("   MINOR     : %x\n", info->hsm_config.minor);
      ELPHW_PRINT("   PARADIGM  : %x (%s)\n", info->hsm_config.paradigm, (char *[]){"Shared", "Virtual"}[info->hsm_config.paradigm&1]);
      ELPHW_PRINT("   CTX CNT   : %u (secure key)\n", info->hsm_config.num_ctx);
      ELPHW_PRINT("   CTX SZ    : %-3lu bytes\n", 4*(1UL<<(info->hsm_config.ctx_page_size+2)));
   }

   // Quick sanity check for ptr registers (mask unused bits)
   if (spacc->config.is_hsm_shared && spacc->config.is_secure_port == 0) {
      // request the semaphore
      if (spacc_request_hsm_semaphore(spacc) != CRYPTO_OK) {
         goto ERR;
      }
   }

   if (spacc->config.dma_type == SPACC_DMA_DDT) {
      pdu_io_write32 (baseaddr + SPACC_REG_DST_PTR, 0x1234567F);
      pdu_io_write32 (baseaddr + SPACC_REG_SRC_PTR, 0xDEADBEEF);
      if (((pdu_io_read32 (baseaddr + SPACC_REG_DST_PTR)) != (0x1234567F & SPACC_DST_PTR_PTR)) ||
          ((pdu_io_read32 (baseaddr + SPACC_REG_SRC_PTR)) != (0xDEADBEEF & SPACC_SRC_PTR_PTR))) {
         ELPHW_PRINT("spacc_init::Failed to set pointers\n");
         goto ERR;
      }
   }

   // zero the IRQ CTRL/EN register (to make sure we're in a sane state)
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_CTRL,     0);
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_EN,       0);
   pdu_io_write32(spacc->regmap + SPACC_REG_IRQ_STAT,     0xFFFFFFFF);

   // init cache
   memset(&spacc->cache, 0, sizeof(spacc->cache));
   pdu_io_write32(spacc->regmap + SPACC_REG_SRC_PTR,      0);
   pdu_io_write32(spacc->regmap + SPACC_REG_DST_PTR,      0);
   pdu_io_write32(spacc->regmap + SPACC_REG_PROC_LEN,     0);
   pdu_io_write32(spacc->regmap + SPACC_REG_ICV_LEN,      0);
   pdu_io_write32(spacc->regmap + SPACC_REG_ICV_OFFSET,   0);
   pdu_io_write32(spacc->regmap + SPACC_REG_PRE_AAD_LEN,  0);
   pdu_io_write32(spacc->regmap + SPACC_REG_POST_AAD_LEN, 0);
   pdu_io_write32(spacc->regmap + SPACC_REG_IV_OFFSET,    0);
   pdu_io_write32(spacc->regmap + SPACC_REG_OFFSET,       0);
   pdu_io_write32(spacc->regmap + SPACC_REG_AUX_INFO,     0);

   // free the HSM
   if (spacc->config.is_hsm_shared && spacc->config.is_secure_port == 0) {
      spacc_free_hsm_semaphore(spacc);
   }

   spacc->ctx = pdu_malloc (sizeof (spacc_ctx) * spacc->config.num_ctx);
   if (spacc->ctx == NULL) {
      ELPHW_PRINT ("spacc_init::Out of memory for ctx\n");
      goto ERR;
   }
   spacc->job = pdu_malloc (sizeof (spacc_job) * SPACC_MAX_JOBS);
   if (spacc->job == NULL) {
      ELPHW_PRINT ("spacc_init::Out of memory for job\n");
      goto ERR;
   }

   /* initialize job_idx and lookup table */
   spacc_job_init_all(spacc);

   /* initialize contexts */
   spacc_ctx_init_all (spacc);

   // autodetect and set string size setting
   if (spacc->config.version == 0x61) {
      spacc_xof_stringsize_autodetect(spacc);
   }

   return CRYPTO_OK;
 ERR:
   spacc_fini (spacc);
   return CRYPTO_FAILED;
}

/* free up the memory */
void spacc_fini (spacc_device * spacc)
{
   pdu_free (spacc->ctx);
   pdu_free (spacc->job);
}

