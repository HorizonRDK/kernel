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
 * Copyright (c) 2011-2015 Synopsys, Inc. and/or its affiliates.
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

#ifndef ELPPKA_H_
#define ELPPKA_H_

#include "elppdu.h"

struct pka_state {
   uint32_t *regbase;

   struct pka_config {
      unsigned alu_size, rsa_size, ecc_size;
      unsigned fw_ram_size, fw_rom_size;
      unsigned ram_offset, rom_offset;
   } cfg;
};

struct pka_fw {
   unsigned long ram_size, rom_size;
   const char *errmsg;

   struct pka_fw_tag {
      unsigned long origin, tag_length, timestamp, md5_coverage;
      unsigned char md5[16];
   } ram_tag, rom_tag;

   /* For internal use */
   struct elppka_fw_priv *priv;
};

enum {
   PKA_OPERAND_A,
   PKA_OPERAND_B,
   PKA_OPERAND_C,
   PKA_OPERAND_D,
   PKA_OPERAND_MAX
};

int elppka_setup(struct pka_state *pka, uint32_t *regbase);

int elppka_start(struct pka_state *pka, uint32_t entry, uint32_t flags,
                                                        unsigned size);
void elppka_abort(struct pka_state *pka);
int elppka_get_status(struct pka_state *pka, unsigned *code);

int elppka_load_operand(struct pka_state *pka, unsigned bank, unsigned index,
                                         unsigned size, const uint8_t *data);
int elppka_unload_operand(struct pka_state *pka, unsigned bank, unsigned index,
                                                 unsigned size, uint8_t *data);

void elppka_set_byteswap(struct pka_state *pka, int swap);

/* Firmware image handling */
int elppka_fw_parse(struct pka_fw *fw, const unsigned char *data,
                                       unsigned long len);
void elppka_fw_free(struct pka_fw *fw);

int elppka_fw_lookup_entry(struct pka_fw *fw, const char *entry);

int elppka_fw_load(struct pka_state *pka, struct pka_fw *fw);

/* The firmware timestamp epoch (2009-11-11 11:00:00Z) as a UNIX timestamp. */
#define PKA_FW_TS_EPOCH 1257937200ull

/* Resolution of the timestamp, in seconds. */
#define PKA_FW_TS_RESOLUTION 20

#endif
