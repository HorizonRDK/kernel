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

#include "elppdu.h"

// some defines
#define PDU_REG_SPACC_VERSION   0x00180UL
#define PDU_REG_SPACC_CONFIG    0x00184UL
#define PDU_REG_PDU_CONFIG      0x00188UL
#define PDU_REG_SPACC_CONFIG2   0x00190UL
#define PDU_REG_SECURE_LOCK     0x001C0UL
#define PDU_REG_SPACC_IV_OFFSET 0x00040UL

#ifndef SPACC_ID_MINOR
#define SPACC_ID_MINOR(x)   ((x)         & 0x0F)
#define SPACC_ID_MAJOR(x)   (((x) >>  4) & 0x0F)
#define SPACC_ID_QOS(x)     (((x) >>  8) & 0x01)
#define SPACC_ID_TYPE(x)    (((x) >>  9) & 0x03)
#define SPACC_ID_AUX(x)     (((x) >> 11) & 0x01)
#define SPACC_ID_VIDX(x)    (((x) >> 12) & 0x07)
#define SPACC_ID_PARTIAL(x) (((x) >> 15) & 0x01)
#define SPACC_ID_PROJECT(x) (uint32_t)((x)>>16)

#define SPACC_TYPE_SPACCQOS 0
#define SPACC_TYPE_PDU      1
#define SPACC_TYPE_HSM      2
#define SPACC_TYPE_TROOT    3

// macros for v4.7 and below
#define SPACC_ID_AUX_V47(x)    (((x) >> 10) & 1)
#define SPACC_ID_QOS_V47(x)    (((x) >> 8) & 1)
#define SPACC_ID_PDU_V47(x)    (((x) >> 9) & 1)

#define SPACC_CFG_CTX_CNT(x)       ((x) & 0x7F)
#define SPACC_CFG_RC4_CTX_CNT(x)   (((x) >> 8) & 0x7F)
#define SPACC_CFG_VSPACC_CNT(x)    (((x) >> 16) & 0x0F)
#define SPACC_CFG_CIPH_CTX_SZ(x)   (((x) >> 20) & 0x07)
#define SPACC_CFG_HASH_CTX_SZ(x)   (((x) >> 24) & 0x0F)
#define SPACC_CFG_DMA_TYPE(x)      (((x) >> 28) & 0x03)

#define SPACC_CFG_CMD0_FIFO_QOS(x)   (((x)>>0)&0x7F)
#define SPACC_CFG_CMD0_FIFO(x)   (((x)>>0)&0x1FF)
#define SPACC_CFG_CMD1_FIFO(x)   (((x)>>8)&0x7F)
#define SPACC_CFG_CMD2_FIFO(x)   (((x)>>16)&0x7F)
#define SPACC_CFG_STAT_FIFO_QOS(x)   (((x)>>24)&0x7F)
#define SPACC_CFG_STAT_FIFO(x)       (((x)>>16)&0x1FF)


#define SPACC_PDU_CFG_MINOR(x)   ((x) & 0x0F)
#define SPACC_PDU_CFG_MAJOR(x)   (((x)>>4)  & 0x0F)
#define SPACC_PDU_CFG_RNG(x)     (((x)>>8)  & 0x01)
#define SPACC_PDU_CFG_PKA(x)     (((x)>>9)  & 0x01)
#define SPACC_PDU_CFG_RE(x)      (((x)>>10) & 0x01)
#define SPACC_PDU_CFG_KEP(x)     (((x)>>11) & 0x01)
#define SPACC_PDU_CFG_EA(x)      (((x)>>12) & 0x01)
#define SPACC_PDU_CFG_MPM(x)     (((x)>>13) & 0x01)

#define SPACC_HSM_CFG_MINOR(x)   ((x) & 0x0F)
#define SPACC_HSM_CFG_MAJOR(x)   (((x)>>4)  & 0x0F)
#define SPACC_HSM_CFG_PARADIGM(x)    (((x)>>8)  & 0x01)
#define SPACC_HSM_CFG_KEY_CNT(x)  (((x)>>16)&0xFF)
#define SPACC_HSM_CFG_KEY_SZ(x)   (((x)>>14)&0x03)

#define PDU_SECURE_LOCK_SPACC(x) (x)
#define PDU_SECURE_LOCK_RNG      (1UL<<16)
#define PDU_SECURE_LOCK_PKA      (1UL<<17)
#define PDU_SECURE_LOCK_RE       (1UL<<18)
#define PDU_SECURE_LOCK_KEP      (1UL<<19)
#define PDU_SECURE_LOCK_EA       (1UL<<20)
#define PDU_SECURE_LOCK_MPM      (1UL<<21)
#define PDU_SECURE_LOCK_CFG      (1UL<<30)
#define PDU_SECURE_LOCK_GLBL     (1UL<<31)

#endif

// configs for SPAccs v4.7 and lower
static const struct {
   unsigned project;
   spacc_config_block cfg;
} spacc_config[] = {

   { 0x0411, (spacc_config_block){10,0,1,6,6,1,0,0,0,0} },
   { 0x0412, (spacc_config_block){4,0,2,7,6,1,0,0,0,0} },
   { 0x0417, (spacc_config_block){2,0,1,6,0,1,0,0,0,0} },
   { 0x0418, (spacc_config_block){2,0,1,6,7,1,0,0,0,0} },
   { 0x0419, (spacc_config_block){4,0,1,6,5,1,0,0,0,0} },
   { 0x1201, (spacc_config_block){8,4,3,7,7,1,0,0,0,0} },
   { 0x1802, (spacc_config_block){16,16,1,7,6,1,0,0,0,0} },
   { 0x1805, (spacc_config_block){64,0,2,7,6,1,0,0,0,0} },

   // end of list
   { 0, (spacc_config_block){0} },
};

// although the spacc_config_blocks above will have the FIFO depths these are
// used for platforms that are in the v4.8 to v4.10 range since they lack
// FIFO depth config registers
// For anything below v4.11 you must set BOTH structures, you can set the FIFO
// depths above to zero if you wish as these below will overwrite it.
static const struct {
   unsigned project;
   int cmd0_depth,
       cmd1_depth,
       cmd2_depth,
       stat_depth;
} spacc_fifos[] = {

   { 0x0411, 10, 0, 0, 10 },
   { 0x0412, 16, 0, 0, 16 },
   { 0x0414,  4, 0, 0,  4 },
   { 0x0417,  2, 0, 0,  2 },
   { 0x0418,  2, 0, 0,  2 },
   { 0x0419,  4, 0, 0,  4 },
   { 0x0601,  8, 0, 0,  8 },
   { 0x0605,  8, 0, 0,  8 },
   { 0x1201,  4, 0, 0,  4 },
   { 0x1802, 16, 0, 0, 16 },
   { 0x1805, 32, 32, 0, 32 },

   { 0x5604,  4,  0, 0,  4 },
   { 0x5407,  4,  0, 0,  4 },
   { 0x9999,  8,  0, 0,  8 },

   // end of list
   { 0 },
};

static int assign_spacc_config_block(unsigned project, pdu_info *inf)
{
   unsigned x;
   for (x = 0; spacc_config[x].project; x++) {
      if (spacc_config[x].project == project) {
         inf->spacc_config = spacc_config[x].cfg;
         return 0;
      }
   }
   return -1;
}

static int assign_spacc_fifo_depth(unsigned project, pdu_info *inf)
{
   unsigned x;
   for (x = 0; spacc_fifos[x].project; x++) {
      if (spacc_fifos[x].project == project) {
         inf->spacc_config.cmd0_fifo_depth = spacc_fifos[x].cmd0_depth;
         inf->spacc_config.cmd1_fifo_depth = spacc_fifos[x].cmd1_depth;
         inf->spacc_config.cmd2_fifo_depth = spacc_fifos[x].cmd2_depth;
         inf->spacc_config.stat_fifo_depth = spacc_fifos[x].stat_depth;
         return 0;
      }
   }
   return -1;
}

int pdu_get_version(void *dev, pdu_info *inf)
{
   unsigned long tmp, ver;

   if (inf == NULL) {
      return -1;
   }

   memset(inf, 0, sizeof *inf);
   tmp = pdu_io_read32(dev + PDU_REG_SPACC_VERSION);

   ver = (SPACC_ID_MAJOR(tmp)<<4)|SPACC_ID_MINOR(tmp);

   if (SPACC_ID_MAJOR(tmp) < 0x04) {
      // we don't support SPAccs before v4.00
      printk("SPAcc MAJOR: %lu not supported\n", SPACC_ID_MAJOR(tmp));
      return -1;
   }

// ***** Read the SPAcc version block this tells us the revision, project, and a few other feature bits *****
   if (ver <= 0x47) {
      // older cores had a different layout
      printk("SPACC::Detected older core, not all configuration values may be correct\n");
      inf->spacc_version = (spacc_version_block){SPACC_ID_MINOR(tmp),
                                                 SPACC_ID_MAJOR(tmp),
                                                 (SPACC_ID_MAJOR(tmp)<<4)|SPACC_ID_MINOR(tmp),
                                                 SPACC_ID_QOS_V47(tmp),
                                                 1,
                                                 SPACC_ID_PDU_V47(tmp),
                                                 0,
                                                 SPACC_ID_AUX_V47(tmp),
                                                 SPACC_ID_VIDX(tmp),
                                                 0,
                                                 SPACC_ID_PROJECT(tmp)};
   } else {
      // layout for v4.8+
      inf->spacc_version = (spacc_version_block) {
         .minor      = SPACC_ID_MINOR(tmp),
         .major      = SPACC_ID_MAJOR(tmp),
         .version    = (SPACC_ID_MAJOR(tmp) << 4) | SPACC_ID_MINOR(tmp),
         .qos        = SPACC_ID_QOS(tmp),
         .is_spacc   = SPACC_ID_TYPE(tmp) == SPACC_TYPE_SPACCQOS,
         .is_pdu     = SPACC_ID_TYPE(tmp) == SPACC_TYPE_PDU
                       || SPACC_ID_TYPE(tmp) == SPACC_TYPE_TROOT,
         .is_hsm     = SPACC_ID_TYPE(tmp) == SPACC_TYPE_HSM,
         .aux        = SPACC_ID_AUX(tmp),
         .vspacc_idx = SPACC_ID_VIDX(tmp),
         .partial    = SPACC_ID_PARTIAL(tmp),
         .project    = SPACC_ID_PROJECT(tmp),
      };
   }

   if (ver <= 0x4C) {
     inf->spacc_version.ivimport = 0;
   } else {
      // try to autodetect
      pdu_io_write32(dev + PDU_REG_SPACC_IV_OFFSET, 0x80000000);
      if (pdu_io_read32(dev + PDU_REG_SPACC_IV_OFFSET) == 0x80000000) {
         inf->spacc_version.ivimport = 1;
      } else {
         inf->spacc_version.ivimport = 0;
      }
   }


// ***** Read the SPAcc config block (v4.8+) which tells us how many contexts there are and context page sizes *****
   // this register only available in v4.8 and up
   if (inf->spacc_version.version >= 0x48) {
      tmp = pdu_io_read32(dev + PDU_REG_SPACC_CONFIG);
      inf->spacc_config = (spacc_config_block){SPACC_CFG_CTX_CNT(tmp),
                                               SPACC_CFG_RC4_CTX_CNT(tmp),
                                               SPACC_CFG_VSPACC_CNT(tmp),
                                               SPACC_CFG_CIPH_CTX_SZ(tmp),
                                               SPACC_CFG_HASH_CTX_SZ(tmp),
                                               SPACC_CFG_DMA_TYPE(tmp),0,0,0,0};
   } else {
      // have to lookup based on Project number
      if (assign_spacc_config_block(inf->spacc_version.project, inf)) {
         printk("SPACC::Failed to find configuration block for this device (%04x), driver will likely not work properly\n", inf->spacc_version.project);
         return -1;
      }
   }


// ***** Read the SPAcc config2 block (v4.11) which tells us the FIFO depths of the core *****
   if (inf->spacc_version.version >= 0x4b) {
      // CONFIG2 only present in v4.11+ cores
      tmp = pdu_io_read32(dev + PDU_REG_SPACC_CONFIG2);
      if (inf->spacc_version.qos) {
         inf->spacc_config.cmd0_fifo_depth = SPACC_CFG_CMD0_FIFO_QOS(tmp);
         inf->spacc_config.cmd1_fifo_depth = SPACC_CFG_CMD1_FIFO(tmp);
         inf->spacc_config.cmd2_fifo_depth = SPACC_CFG_CMD2_FIFO(tmp);
         inf->spacc_config.stat_fifo_depth = SPACC_CFG_STAT_FIFO_QOS(tmp);
      } else {
         inf->spacc_config.cmd0_fifo_depth = SPACC_CFG_CMD0_FIFO(tmp);
         inf->spacc_config.stat_fifo_depth = SPACC_CFG_STAT_FIFO(tmp);
      }
   } else {
      if (assign_spacc_fifo_depth(inf->spacc_version.project, inf)) {
         printk("SPACC::Failed to find FIFO depth configuration for this device (%04x), driver will likely not work properly\n", inf->spacc_version.project);
         return -1;
      }
   }

   /* only read PDU config if it's actually a PDU engine */
   if (inf->spacc_version.is_pdu) {
      tmp = pdu_io_read32(dev + PDU_REG_PDU_CONFIG);
      inf->pdu_config = (pdu_config_block){SPACC_PDU_CFG_MINOR(tmp),
                                           SPACC_PDU_CFG_MAJOR(tmp),
                                           SPACC_PDU_CFG_RNG(tmp),
                                           SPACC_PDU_CFG_PKA(tmp),
                                           SPACC_PDU_CFG_RE(tmp),
                                           SPACC_PDU_CFG_KEP(tmp),
                                           SPACC_PDU_CFG_EA(tmp),
                                           SPACC_PDU_CFG_MPM(tmp)};

      pdu_io_write32(dev + PDU_REG_SECURE_LOCK, 0); // unlock all cores by default
   }

   /* only read HSM config if it's actually a HSM engine */
   if (inf->spacc_version.is_hsm) {
      tmp = pdu_io_read32(dev + PDU_REG_PDU_CONFIG);
      inf->hsm_config = (hsm_config_block){SPACC_HSM_CFG_MINOR(tmp),
                                           SPACC_HSM_CFG_MAJOR(tmp),
                                           SPACC_HSM_CFG_PARADIGM(tmp),
                                           SPACC_HSM_CFG_KEY_CNT(tmp),
                                           SPACC_HSM_CFG_KEY_SZ(tmp)};
   }

   return 0;
}
