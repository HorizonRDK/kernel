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

#include "elpea.h"

static const struct {
   unsigned project;
   int cmd0_depth,
       stat_depth,
       sp_ddt_cnt,
       ah_enable,
       ipv6,
       arwinsize;
} fifo_data[] = {

   { 0x0605, 16, 16, 32, 0, 1, 64 },

   { 0x9999,  4,  4,  8, 1, 1, 64 },

   { 0 },
};

static int ea_assign_fifo_info(ea_device *ea)
{
   int x;

   for (x = 0; fifo_data[x].project; x++) {
      if (fifo_data[x].project == ea->spacc->config.project) {
         ea->config.cmd0_depth  = fifo_data[x].cmd0_depth;    // CMD fifo depth
         ea->config.stat_depth  = fifo_data[x].stat_depth;    // STAT fifo depth
         ea->config.sp_ddt_cnt  = fifo_data[x].sp_ddt_cnt;    // max DDT entries
         ea->config.ah_enable   = fifo_data[x].ah_enable;
         ea->config.ipv6        = fifo_data[x].ipv6;
         ea->config.ar_win_size = fifo_data[x].arwinsize;
         return 0;
      }
   }
   return -1;
}


int ea_init(ea_device *ea, spacc_device *spacc, void *regmap, int no_ctx)
{
   int x;
   uint32_t cfg;

   memset(ea, 0, sizeof *ea);

   ea->spacc  = spacc;
   ea->regmap = regmap;

   ea->num_ctx        = no_ctx;
   ea->sa_ptr_mem_req = SA_SIZE * no_ctx;

   ea->ctx = pdu_malloc(sizeof(*(ea->ctx)) * no_ctx);
   if (!ea->ctx) {
      ELPHW_PRINT("Out of memory\n");
      return -1;
   }
   memset(ea->ctx, 0, sizeof(*(ea->ctx)) * no_ctx);

   for (x = 0; x < no_ctx; x++) {
      ea->ctx[x].spacc_handle = -1;
   }

   for (x = 0; x < 256; x++) {
      ea->swid[x] = 0xFFFFFFFF;
   }

   memset(&(ea->cache), 0, sizeof (ea->cache));
   pdu_io_write32(regmap + EA_SRC_PTR, 0);
   pdu_io_write32(regmap + EA_DST_PTR, 0);
   pdu_io_write32(regmap + EA_SA_PTR,  0);
   pdu_io_write32(regmap + EA_OFFSET,  0);

   // only present in PDU v2.5 and up
   if (spacc->config.pdu_version >= 0x25) {
      cfg = pdu_io_read32(regmap + EA_VERSION);
      ea->config = (struct ea_config){
         EA_VERSION_CMD0_DEPTH(cfg),
         EA_VERSION_CMD1_DEPTH(cfg),
         EA_VERSION_SP_DDT_CNT(cfg),
         EA_VERSION_AH_ENABLE(cfg),
         EA_VERSION_IPV6(cfg),
         EA_VERSION_AR_WIN_SIZE(cfg),
         EA_VERSION_CMD0_DEPTH(cfg) + EA_VERSION_CMD1_DEPTH(cfg),
         0, 0, 0 };

      if (spacc->config.pdu_version == 0x25) {
         // AH was not optional in this version and the VERSION bit is not present...
         ea->config.ah_enable = 1;
      }

      ea->config.ideal_stat_limit = min(ea->config.stat_depth, ea->config.cmd0_depth);
      if (ea->config.cmd1_depth) {
         ea->config.ideal_stat_limit = min(ea->config.ideal_stat_limit, ea->config.cmd1_depth);
      }

      // limit the STAT CNT to at most 16 since anything beyond that is not really going to happen in most sessions
      ea->config.ideal_stat_limit = min(16, ea->config.ideal_stat_limit);
      ea->config.ideal_stat_limit = max(1, ea->config.ideal_stat_limit - 1);
   } else {
      // no config register present
      memset(&ea->config, 0, sizeof (ea->config));

      if (ea_assign_fifo_info(ea)) {
         printk("EA:: Could not find project ID in FIFO data table, EA SDK may not work correctly\n");
      }

      ea->config.ideal_stat_limit = 1;
      ea->config.max_stat_limit   = min(16, ea->config.stat_depth - 1);
   }
   ea->config.cur_depth = 1;

   // default to FAVTX since for a balanced sized FIFO it's equal to FAVRX
   ea_set_mode(ea, EA_OP_MODE_FULL_DUPLEX_FAVTX);

   ELPHW_PRINT("EA configuration detected:\n");
   ELPHW_PRINT("   CMD0_DEPTH: %lu\n", (unsigned long)ea->config.cmd0_depth);
   ELPHW_PRINT("   CMD1_DEPTH: %lu\n", (unsigned long)ea->config.cmd1_depth);
   ELPHW_PRINT("   SP_DDT_CNT: %lu\n", (unsigned long)ea->config.sp_ddt_cnt);
   ELPHW_PRINT("    AH ENABLE: %lu\n", (unsigned long)ea->config.ah_enable);
   ELPHW_PRINT("         IPv6: %lu\n", (unsigned long)ea->config.ipv6);
   ELPHW_PRINT("   STAT_DEPTH: %lu\n", (unsigned long)ea->config.stat_depth);
   ELPHW_PRINT("      TX_FIFO: %lu\n", (unsigned long)ea->config.tx_fifo);
   ELPHW_PRINT("      RX_FIFO: %lu\n", (unsigned long)ea->config.rx_fifo);
   ELPHW_PRINT("   FIFO STRAT: %lu\n", (unsigned long)ea->config.op_mode);

   if (ea->config.cmd1_depth) {
      // default to round robin
      EA_FIFO_PRIO(ea, 0, 0, 0);
   }

   return 0;
}


