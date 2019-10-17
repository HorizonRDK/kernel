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
 * Copyright (c) 2012-2016 Synopsys, Inc. and/or its affiliates.
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

#include "elpclp890_hw.h"
#include "elpclp890.h"

/* Initialize the CLP890 state structure */
int elpclp890_init (elpclp890_state *state, uint32_t *base) {
   int err;
   uint32_t tmp;
   
   DEBUG(">> elpclp890_init: initialize the CLP890\n");
   
   memset(state, 0, sizeof *state);
   
   state->base = base;

   /* make sure there is no alarm and the core is not busy */
   if ((err = elpclp890_get_alarms(state)))   { goto ERR; }
   if ((err = elpclp890_wait_on_busy(state))) { goto ERR; }
   
   /* hardware features*/
   tmp = pdu_io_read32(state->base + CLP890_REG_FEATURES);
   state->config.features.drbg_arch           = CLP890_REG_FEATURES_AES_256(tmp);
   state->config.features.extra_ps_present    = CLP890_REG_FEATURES_EXTRA_PS_PRESENT(tmp);
   state->config.features.secure_rst_state    = CLP890_REG_FEATURES_SECURE_RST_STATE(tmp);
   state->config.features.diag_level_trng3    = CLP890_REG_FEATURES_DIAG_LEVEL_CLP800(tmp);
   state->config.features.diag_level_stat_hlt = CLP890_REG_FEATURES_DIAG_LEVEL_ST_HLT(tmp);
   state->config.features.diag_level_ns       = CLP890_REG_FEATURES_DIAG_LEVEL_NS(tmp);
   
   /* build id */
   tmp = pdu_io_read32(state->base + CLP890_REG_BUILD_ID);
   state->config.build_id.stepping            = CLP890_REG_BUILD_ID_STEPPING(tmp);
   state->config.build_id.epn                 = CLP890_REG_BUILD_ID_EPN(tmp);
   
   /* clear registers */
   pdu_io_write32(state->base + CLP890_REG_ALARM, 0xFFFFFFFF);
   pdu_io_write32(state->base + CLP890_REG_ISTAT, 0xFFFFFFFF);

   /* setup the CLP890 in secure mode, self seeding mode, with prediction resistance, maximum possible security strength */
   /* SMODE */
   tmp = 0;
   tmp = CLP890_REG_SMODE_SET_SECURE_EN(tmp, 1);
   tmp = CLP890_REG_SMODE_SET_NONCE(tmp, 0);
   tmp = CLP890_REG_SMODE_SET_MAX_REJECTS(tmp, CLP890_DFLT_MAX_REJECTS);
   pdu_io_write32(state->base + CLP890_REG_SMODE, tmp);   
   state->status.secure_mode = 1;
   state->status.nonce_mode  = 0;
   /* MODE */
   tmp = 0;
   if (state->config.features.drbg_arch == AES256) {
      tmp = CLP890_REG_MODE_SET_SEC_ALG(tmp, 1);
      state->status.sec_strength = SEC_STRNT_AES256;
   } else if (state->config.features.drbg_arch == AES128) {
      tmp = CLP890_REG_MODE_SET_SEC_ALG(tmp, 0);
      state->status.sec_strength = SEC_STRNT_AES128;
   } else {
      ELPHW_PRINT("Invalid DRBG architecture");
      err = CRYPTO_FAILED; goto ERR;
   }   
   tmp = CLP890_REG_MODE_SET_PRED_RESIST(tmp, 1);
   pdu_io_write32(state->base + CLP890_REG_MODE, 0);
   state->status.pred_resist  = 1;
   /* rest of the status */
   state->status.alarm_code   = 0;
   state->status.pad_ps_addin = 0;
   
   /* reminders - set the counters to the standard's maximum values. An API is be provided to change those on demand.*/
   elpclp890_set_reminder_max_bits_per_req(state, CLP890_DFLT_MAX_BITS_PER_REQ);
   elpclp890_set_reminder_max_req_per_seed(state, CLP890_DFLT_MAX_REQ_PER_SEED);
   
   /* display features */
   ELPHW_PRINT("CLP890: (epn.%04x v.%02x)\n", state->config.build_id.epn, state->config.build_id.stepping);
   switch (state->config.features.drbg_arch) {
      case AES128:
         ELPHW_PRINT("CLP890: DRBG Architecture=128-bit AES, Extra Personilization Existence=%u\n", state->config.features.extra_ps_present);
         break;
      case AES256:
         ELPHW_PRINT("CLP890: DRBG Architecture=256-bit AES, Extra Personilization Existence=%u\n", state->config.features.extra_ps_present);
         break;
      default:
         ELPHW_PRINT("Invalid DRBG architecture");
         err = CRYPTO_FAILED; goto ERR;
         break;
   }

   DEBUG("initialization is done, going for a zeroize\n");     
   
   if ((err = elpclp890_zeroize(state))) { goto ERR; }
   
   err = CRYPTO_OK;
   state->status.current_state = CLP890_STATE_INITIALIZE;
ERR:
   DEBUG("--- elpclp890_init Return, err = %i\n", err);
   return err;
} /* elpclp890_init */

/* Instantiate the DRBG state */
int elpclp890_instantiate (elpclp890_state *state, int req_sec_strength, int pred_resist, void *personal_str) {
   int err;
   uint32_t tmp;
   uint32_t zero_ps[12] = {0};
   
   DEBUG(">> elpclp890_instantiate: security strength = %u, pred_resist = %u, personilization string existence = %u\n", req_sec_strength, pred_resist, (personal_str) ? 1 : 0);
   
   /* make sure there is no alarm and the core is not busy */
   if ((err = elpclp890_get_alarms(state)))   { goto ERR; }
   if ((err = elpclp890_wait_on_busy(state))) { goto ERR; }  
   
   /* If DRBG is already instantiated or if current state does not allow an instantiate, return error */   
   if (DRBG_INSTANTIATED(state->status.current_state)) { DEBUG("Initial check: DRBG state is already instantiated\n"); err = CRYPTO_FAILED; goto ERR; }
   if ((state->status.current_state != CLP890_STATE_INITIALIZE) && (state->status.current_state != CLP890_STATE_UNINSTANTIATE)) {
      DEBUG("Cannot instantiate in the current state (%u)\n", state->status.current_state);
      err = CRYPTO_FAILED; goto ERR;
   }
   
   /* if hardware is not configured to accept extra personalization string, but personal_str is not NULL, return error */
   if (!state->config.features.extra_ps_present && personal_str) { DEBUG("HW config does not allow extra PS\n"); err = CRYPTO_FAILED; goto ERR; }
   
   /* Validate and set the security strength */
   if ((err = elpclp890_set_sec_strength(state, req_sec_strength))) { goto ERR; }
   
   /* get entropy - noise seeding. If the mode is nonce, get_entropy must be called by the user prior to the instantiate function */
   DEBUG("Seeding mode is: %s\n", state->status.nonce_mode ? "Nonce" : "Noise");
   if (!state->status.nonce_mode) { /* noise seeding */
      if ((err = elpclp890_get_entropy_input(state, NULL, 0))) { goto ERR; }
   }
   
   /* load the personilization string if hardware is configured to accept it */
   if (state->config.features.extra_ps_present) {
      /* if HW is configured to accept personilizatoin string, it will use whatever is in the NPA_DATAx. So, if the string is NULL, just load 0. */
      if (!personal_str) {
         personal_str = &zero_ps[0];
      }
      
      if ((err = elpclp890_load_ps_addin(state, personal_str))) { goto ERR; } 
   }
  
   /* initiate the Create_State command and wait on done */
   DEBUG("Create the DRBG state\n");
   pdu_io_write32(state->base + CLP890_REG_CTRL, CLP890_REG_CTRL_CMD_CREATE_STATE);
   if ((err = elpclp890_wait_on_done(state))) { goto ERR; }

   /* check STAT register to make sure DRBG is instantiated */
   tmp = pdu_io_read32(state->base + CLP890_REG_STAT);
   if (!CLP890_REG_STAT_GET_DRBG_STATE(tmp)) { err = CRYPTO_FAILED; goto ERR; }
   
   /* reset reminder and alarms counters */
   elpclp890_reset_counters(state);
   
   /* set the prediction resistance */
   if ((err = elpclp890_set_pred_resist(state, pred_resist))) { goto ERR; }      
   
   err = CRYPTO_OK;
   state->status.current_state = CLP890_STATE_INSTANTIATE;   
ERR:
   DEBUG("--- Return elpclp890_instantiate, err = %i\n", err);
   return err;
} /* elpclp890_instantiate */

/* Uninstantiate the DRBG state and zeroize */
int elpclp890_uninstantiate (elpclp890_state *state) {
   int err;
   int err_tmp;
   
   DEBUG(">> elpclp890_uninstantiate: uninstantiate the DRBG and zeroize\n");
   err = CRYPTO_OK;
   err_tmp = CRYPTO_OK;   
   
   /* if DRBG is instantiated, return CRYPTO_NOT_INSTANTIATED, but still do the zeroize */      
   if (!DRBG_INSTANTIATED(state->status.current_state)) { err_tmp = CRYPTO_NOT_INSTANTIATED; }
   
   /* zeroize */
   if ((err = elpclp890_zeroize(state))) { goto ERR; }

   if ((err == CRYPTO_OK) && (err_tmp == CRYPTO_NOT_INSTANTIATED)) { err = CRYPTO_NOT_INSTANTIATED; }   
   state->status.current_state = CLP890_STATE_UNINSTANTIATE;
ERR:
   DEBUG("--- Return elpclp890_uninstantiate, err = %i\n", err);
   return err;
} /* elpclp890_uninstantiate */

/* Reseed */
int elpclp890_reseed (elpclp890_state *state, int pred_resist, void *addin_str) {
   int err;   
   
   DEBUG(">> elpclp890_reseed: pred_resist = %u, additional strign existence = %u\n", pred_resist, (addin_str) ? 1 : 0);
   
   /* make sure there is no alarm and the core is not busy */
   if ((err = elpclp890_get_alarms(state)))   { goto ERR; }
   if ((err = elpclp890_wait_on_busy(state))) { goto ERR; }
   
   /* if the DRBG is not instantiated return error */
   if (!DRBG_INSTANTIATED(state->status.current_state)) { DEBUG("DRBG is not instantiated\n"); err = CRYPTO_FAILED; goto ERR; }
   
   /* if pred_resist is set but, pred_resist that the DRBG is instantiated with is not 1, return error */
   if ((err = elpclp890_set_pred_resist(state, pred_resist))) { goto ERR; }

   /* get entropy - noise seeding. If the mode is nonce, get_entropy must be called by the user prior to the instantiate function */
   if (!state->status.nonce_mode) { /* noise seeding */
      if ((err = elpclp890_get_entropy_input(state, NULL, 0))) { goto ERR; }
   }
   
   /* if addin_str is not NULL, it means that the additionl input is available and has to be loaded */
   if (addin_str) {
      /* set the ADDIN_PRESENT field of the MODE register to 1 */
      if ((err = elpclp890_set_addin_present(state, 1))) { goto ERR; }
      
      /* load the additional input */
      if ((err = elpclp890_load_ps_addin(state, addin_str))) { goto ERR; }
   } else {
      /* set the ADDIN_PRESENT field of the MODE register to 0 */
      if ((err = elpclp890_set_addin_present(state, 0))) { goto ERR; }
   }
   
   /* initiate the reseed and wait on done */
   pdu_io_write32(state->base + CLP890_REG_CTRL, CLP890_REG_CTRL_CMD_RENEW_STATE);
   if ((err = elpclp890_wait_on_done(state))) { goto ERR; }   
   
   /* reset reminder and alarms counters */
   elpclp890_reset_counters(state);
   
   err = CRYPTO_OK;
   state->status.current_state = CLP890_STATE_RESEED;
ERR:
   DEBUG("--- Return elpclp890_reseed, err = %i\n", err);
   return err;
} /* elpclp890_reseed */

/* Generate */
int elpclp890_generate (elpclp890_state *state, void *random_bits, unsigned long req_num_bytes, int req_sec_strength, int pred_resist, void *addin_str) {
   int err;
   int reseed_required;
   
   DEBUG(">> elpclp890_generate: requested number of bytes = %lu, security strength = %u, pred_resist = %u, additional string existence = %u\n", req_num_bytes, req_sec_strength, pred_resist, (addin_str) ? 1 : 0);
   
   /* make sure there is no alarm and the core is not busy */
   if ((err = elpclp890_get_alarms(state)))   { goto ERR; }
   if ((err = elpclp890_wait_on_busy(state))) { goto ERR; }
   
   /* if the DRBG is not intantiated return error */
   if (!DRBG_INSTANTIATED(state->status.current_state)) { DEBUG("DRBG is not instantiated\n"); err = CRYPTO_FAILED; goto ERR; }
   
   /* requested number of bits has to be less that the programmed maximum */
   if ((req_num_bytes*8) > state->counters.max_bits_per_req) { 
      DEBUG("requested number of bits (%lu) is larger than the set maximum (%lu)\n", (req_num_bytes*8), state->counters.max_bits_per_req);
      err = CRYPTO_FAILED; goto ERR; 
   }
   
   /* security strength has to be lower than what the DRBG is instantiated with. set_sec_strength function checks this. */
   if ((err = elpclp890_set_sec_strength(state, req_sec_strength))) { goto ERR; }   
   
   /* set the prediction resistance - if pred_resist is set but, pred_resist that the DRBG is instantiated with is not 1, return error */
   if ((err = elpclp890_set_pred_resist(state, pred_resist))) { goto ERR; }
   
   /* make sure random_bits is not NULL */
   if (!random_bits) { DEBUG("random_bits pointer cannot be NULL\n"); err = CRYPTO_FAILED; goto ERR; }     
   
   /* set the reseed required flag to 0. The loop is to check at the end whether a reseed is required at the end and jump back to reseed and generate if needed. This is the NIST mandated procedure */
   reseed_required = 0;
   
   if (!addin_str) {
      /* set the ADDIN_PRESENT field of the MODE register to 1 */
      if ((err = elpclp890_set_addin_present(state, 0))) { goto ERR; }
   }
   
   do {
      void *generate_addin_str = addin_str;      
      
      if (pred_resist | reseed_required) {
         if ((err = elpclp890_reseed(state, pred_resist, addin_str))) { goto ERR; }
         /* SP800-90a says that if reseed is executed, any additional input string is only used in the reseed phase and replaced by NULL in the generate phase */
         generate_addin_str = NULL;
         if ((err = elpclp890_set_addin_present(state, 0))) { goto ERR; } /* ADDIN_PRESENT field in MODE has to be set back to 0 to avoid illegal cmd sequence */
         reseed_required = 0;
      }
      
      /* generate process */
      if (elpclp890_check_seed_lifetime(state) == CRYPTO_RESEED_REQUIRED) {
         reseed_required = 1;
      } else {
         reseed_required = 0;
         
         /* Refresh_Addin command if additional input is not NULL*/
         if (generate_addin_str) {
            if ((err = elpclp890_refresh_addin (state, generate_addin_str))) { goto ERR; }
         }

         /* Generate all random bits */      
         if((err = elpclp890_gen_random(state, random_bits, req_num_bytes))) { goto ERR; }
         
         /* Advance the state - if it returns CRYPTO_RESEED_REQUIRED, have to jump back and do a reseed and generate */
         if((err = elpclp890_advance_state(state))) { goto ERR; }
      }
   } while (reseed_required);   
   
   err = CRYPTO_OK;
   state->status.current_state = CLP890_STATE_GENERATE;
ERR:
   if (err) { random_bits = NULL; }
   DEBUG("--- Return elpclp890_generate, err = %i\n", err);
   return err;
} /* elpclp890_generate */
