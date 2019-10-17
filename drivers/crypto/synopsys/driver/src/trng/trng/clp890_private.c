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

/* Wait functions */
static int elpclp890_wait_on_ (elpclp890_state *state, uint32_t mask) {
   uint32_t tmp;
   int t;
   
   t = CLP890_RETRY_MAX;

   do { tmp = pdu_io_read32(state->base + CLP890_REG_ISTAT); } while (!(tmp & (mask|CLP890_REG_ISTAT_ALARMS)) && --t);
   if (tmp & CLP890_REG_ISTAT_ALARMS) {
	   return elpclp890_get_alarms(state);
   }

   if (t) {
      pdu_io_write32(state->base + CLP890_REG_ISTAT, mask);
      return CRYPTO_OK;
   } else {
      DEBUG("wait_on_: failed timeout: %08lx\n", (unsigned long)tmp);
      return CRYPTO_TIMEOUT;
   }
} /* elpclp890_wait_on_ */

int elpclp890_wait_on_done (elpclp890_state *state) {
   DEBUG(">> elpclp890_wait_on_done\n");
   return elpclp890_wait_on_(state, CLP890_REG_ISTAT_DONE);
} /* elpclp890_wait_on_done */

static int elpclp890_wait_on_zeroize (elpclp890_state *state) {
   DEBUG(">> elpclp890_wait_on_zeroize\n");
   return elpclp890_wait_on_(state, CLP890_REG_ISTAT_ZEROIZE);
} /* elpclp890_wait_on_zeroize */

static int elpclp890_wait_on_kat_completed (elpclp890_state *state) {
   DEBUG(">> elpclp890_wait_on_kat_completed\n");
   return elpclp890_wait_on_(state, CLP890_REG_ISTAT_KAT_COMPLETE);
} /* elpclp890_wait_on_kat_completed */

int elpclp890_wait_on_busy (elpclp890_state *state) {
   uint32_t tmp, t;
   
   DEBUG(">> elpclp890_wait_on_busy\n");      
   
   t = CLP890_RETRY_MAX;
   
   do { tmp = pdu_io_read32(state->base + CLP890_REG_STAT); } while ((tmp & CLP890_REG_STAT_BUSY) && --t);
   if (t) {
      return CRYPTO_OK;
   } else {
      DEBUG("wait_on_: failed timeout: %08lx\n", (unsigned long)tmp);
      return CRYPTO_TIMEOUT;
   }
} /* elpclp890_wait_on_busy */

/* Read and return alarm. Zeroize if there is an alarm*/
int elpclp890_get_alarms (elpclp890_state *state) {
   uint32_t tmp;
   
   //DEBUG(">> elpclp890_get_alarms\n");
   
   tmp = pdu_io_read32(state->base + CLP890_REG_ISTAT);
   if (tmp & CLP890_REG_ISTAT_ALARMS) {
      // alarm happened
      tmp = pdu_io_read32(state->base + CLP890_REG_ALARM);
      DEBUG("Received alarm: %lx\n", (unsigned long)tmp);
      // clear istat
      pdu_io_write32(state->base + CLP890_REG_ISTAT, CLP890_REG_ISTAT_ALARMS);
      pdu_io_write32(state->base + CLP890_REG_ALARM, 0x1F);
      state->status.alarm_code = tmp & 0x1F;
      
      /* zeroize if there was an alarm */
      if (state->status.alarm_code != CLP890_REG_ALARM_FAILED_TEST_ID_OK) {
         elpclp890_zeroize(state);
      }
   } else {
      state->status.alarm_code = 0;
   }
   
   if (state->status.alarm_code) {
      return CRYPTO_FATAL;
   } else {
      return CRYPTO_OK;
   }
} /* elpclp890_get_alarms */

/* Reset reminder and alarm counters */
int elpclp890_reset_counters (elpclp890_state *state) {
   
   DEBUG(">> elpclp890_reset_counters\n");

   state->counters.bits_per_req_left = state->counters.max_bits_per_req;
   state->counters.req_per_seed_left = state->counters.max_req_per_seed;
   
   DEBUG("--- Return elpclp890_reset_counters, err = %i\n", 0);
   return 0;
} /* elpclp890_reset_counters */

/* When a zeroize happens some of the struct objects should reset */
int elpclp890_reset_state (elpclp890_state *state) {
   
   DEBUG(">> elpclp890_reset_state\n");
   
   elpclp890_reset_counters(state);
   state->status.pad_ps_addin = 0;
   state->status.current_state = CLP890_STATE_UNINSTANTIATE;
   
   DEBUG("--- Return elpclp890_reset_state, err = %i\n", 0);
   return 0;
} /* elpclp890_reset_state */

/* ---------- Set field APIs ---------- */

/*
Sets the security strength of the DRBG instance. 
   > req_sec_strength has to be an integer. The API chooses one of SEC_STRNT_AES128 or SEC_STRNT_AES256 as follows:
      * 0   < req_sec_strength <= 128  --> security strength = SEC_STRNT_AES128
      * 128 < req_sec_strength <= 256  --> security strength = SEC_STRNT_AES256
      * else 			                  --> Invalid security strength
   > If the DRBG is instantiated, a new security strength change request with greater security strength will return error. 
*/
int elpclp890_set_sec_strength (elpclp890_state *state, int req_sec_strength) {
   int err;
   uint32_t tmp;
   elpclp890_sec_strength chosen_sec_strength;
   
   DEBUG(">> elpclp890_set_sec_strength: security strength = %i\n", req_sec_strength);
   
   /* choose the security strength */   
   /* set the security strength to the lowest security strength greater or equal to the req_sec_strenght from the set {128, 256} */
   if (REQ_SEC_STRENGTH_IS_VALID(req_sec_strength)) {
      if ((req_sec_strength > 0) && (req_sec_strength <= 128)) { 
         chosen_sec_strength = SEC_STRNT_AES128;
      } else if (((req_sec_strength > 128) && (req_sec_strength <= 256)) && (state->config.features.drbg_arch == AES256)) { 
         chosen_sec_strength = SEC_STRNT_AES256;      
      } else { /* should not get here, because we have already checked the validity */
         DEBUG("Invalid security strength\n");
         err = CRYPTO_FAILED; goto ERR;
      }
   } else {
      DEBUG("Invalid security strength\n");
      err = CRYPTO_FAILED; goto ERR;
   }
   DEBUG("chosen security strength = %u\n", chosen_sec_strength);
   
   /* set the security strenght - at this point security strength is validated and converted */
   if (DRBG_INSTANTIATED(state->status.current_state) && (chosen_sec_strength != state->status.sec_strength)) { /* security strength can only change when the DRBG is not instantiated. */
      /* if the new security strength is less that what the DRBG is instantiated with, accept it, but don't change in HW. If it's more, return error */
      if (chosen_sec_strength < state->status.sec_strength) {
         DEBUG("Lowering the security strength. DRBG is already instantiated.\n");
         state->status.pad_ps_addin = 4;
         state->status.sec_strength = chosen_sec_strength;         
      } else {
         state->status.pad_ps_addin = 0;
         DEBUG("Cannot select a higher security strenght once the DRBG is instantiated\n");
         err = CRYPTO_FAILED; goto ERR;
      }
   } else {   
      DEBUG("Updating the security strength.\n");
      tmp = pdu_io_read32(state->base + CLP890_REG_MODE);
      tmp = CLP890_REG_MODE_SET_SEC_ALG(tmp, chosen_sec_strength);
      pdu_io_write32(state->base + CLP890_REG_MODE, tmp);
      
      state->status.pad_ps_addin = 0;
      state->status.sec_strength = chosen_sec_strength;
   }/* else {
      DEBUG("No change in the security strength\n");
   }*/

   err = CRYPTO_OK;
ERR:   
   DEBUG("--- Return elpclp890_set_sec_strength, err = %i\n", err);
   return err;
} /* elpclp890_set_sec_strength */

/*
Sets the ADDIN_PRESENT field of the MODE register according to the addin_present input.
*/
int elpclp890_set_addin_present (elpclp890_state *state, int addin_present) {   
   uint32_t tmp;
   
   DEBUG(">> elpclp890_set_addin_present, adding_present = %u\n", addin_present);
   
   tmp = pdu_io_read32(state->base + CLP890_REG_MODE);
   tmp = CLP890_REG_MODE_SET_ADDIN_PRESENT(tmp, addin_present);
   pdu_io_write32(state->base + CLP890_REG_MODE, tmp);
   
   DEBUG("--- Return elpclp890_set_addin_present, err = %i\n", 0);
   return 0;
} /* elpclp890_set_addin_present */

/* 
Sets the PRED_RESIST field of the MODE register according to the pred_resist input. 
   > If the DRBG is instantiated with prediction resistance of 0, and a change to the prediction resistance of 1 is requested, the API will return an error.
*/
int elpclp890_set_pred_resist (elpclp890_state *state, int pred_resist) {
   int err;
   uint32_t tmp;
   
   DEBUG(">> elpclp890_set_pred_resist: pred_resist = %u\n", pred_resist);
   
   /* if DRBG is instantiated, prediction resistance can only change from 1 to 0 and not vice versa. This is a NIST requirement. */
   if (DRBG_INSTANTIATED(state->status.current_state) && pred_resist && !state->status.pred_resist) { err = CRYPTO_FAILED; goto ERR; }
      
   tmp = pdu_io_read32(state->base + CLP890_REG_MODE);
   tmp = CLP890_REG_MODE_SET_PRED_RESIST(tmp, pred_resist);
   pdu_io_write32(state->base + CLP890_REG_MODE, tmp);
   
   state->status.pred_resist = pred_resist;
   
   err = CRYPTO_OK;
ERR:
   DEBUG("--- Return elpclp890_set_pred_resist, err = %i\n", err);
   return err;
} /* elpclp890_set_pred_resist */

/* 
Puts the CLP-890 in either the SECURE or PROMISCUOUS mode.
   > A value of 1 for secure_mode puts the core in the SECURE mode and a value of 0 puts it in the PROMISCUOUS mode. 
   > Any change to the secure mode of the CLP-890 will result in a complete zeroize, and will set the seeding mode to self-seeding. 
     A zeroize will not destroy the programmed mode and ALARM register value. 
     It keeps the programmed mode to avoid re-programming. 
     It also, maintains the ALARM register value, so that the user can read the value to understand the reason of the occurred alarm.
*/
int elpclp890_set_secure_mode (elpclp890_state *state, int secure_mode) {
   int err;
   uint32_t tmp;
   int t;
   
   DEBUG(">> elpclp890_set_secure_mode: secure_mode = %u\n", secure_mode);
   
   t = CLP890_RETRY_MAX;

   tmp = pdu_io_read32(state->base + CLP890_REG_SMODE);
   tmp = CLP890_REG_SMODE_SET_SECURE_EN(tmp, secure_mode);
   pdu_io_write32(state->base + CLP890_REG_SMODE, tmp);
   
   /* wait until STAT register indicates that the mode is applied */
   do {
      tmp = pdu_io_read32(state->base + CLP890_REG_STAT);      
   } while ((CLP890_REG_STAT_GET_SECURE(tmp) != secure_mode) && --t);  
   
   if (!t) { err = CRYPTO_TIMEOUT; goto ERR; }
   
   /* if secure mode changes, a zeroize will happen in HW. */
   if (state->status.secure_mode != secure_mode) {
      DEBUG("secure mode changed. zeroize happened. reset sw state\n");
      /* nonce mode goes back to default. */
      state->status.nonce_mode    = 0;      
      /* reset the SW state */
      elpclp890_reset_state(state);
   }
   
   state->status.secure_mode = secure_mode;
   
   err = CRYPTO_OK;
ERR:
   DEBUG("--- Return elpclp890_set_secure_mode, err = %i\n", err);
   return err;
} /* elpclp890_set_secure_mode */

/*
To change the seeding mode of the CLP-890. 
   > A value of 1 for nonce_mode will put the CLP-890 in the nonce seeding mode, which means that the seed will be provided by the user, 
     unlike the noise or self-seeding mode (normal mode of operation) in which the seed is generated by the internal entropy source.
   > Any transition to or from the nonce mode will zeroize the CLP-890.
*/
int elpclp890_set_nonce_mode (elpclp890_state *state, int nonce_mode) {
   int err;
   uint32_t tmp;
   int t;
   
   DEBUG(">> elpclp890_set_nonce_mode: nonce_mode = %u\n", nonce_mode);
   
   t = CLP890_RETRY_MAX;
   
   tmp = pdu_io_read32(state->base + CLP890_REG_SMODE);
   tmp = CLP890_REG_SMODE_SET_NONCE(tmp, nonce_mode);
   pdu_io_write32(state->base + CLP890_REG_SMODE, tmp);
   
   /* wait until STAT register indicates that the mode is applied */
   do {
      tmp = pdu_io_read32(state->base + CLP890_REG_STAT);      
   } while ((CLP890_REG_STAT_GET_NONCE(tmp) != nonce_mode) && --t);
   
   if (!t) { err = CRYPTO_TIMEOUT; goto ERR; }
   
   /* if nonce mode changes, a zeroize will happen in HW. */
   if (state->status.nonce_mode != nonce_mode) {      
      DEBUG("nonce mode changed. zeroize happened. reset sw state\n");      
      /* reset the SW state */
      elpclp890_reset_state(state);
   }
   
   state->status.nonce_mode = nonce_mode;
   
   err = CRYPTO_OK;
ERR:
   DEBUG("--- Return elpclp890_set_nonce_mode, err = %i\n", err);
   return err;
} /* elpclp890_set_nonce_mode */

/* ---------- Load data APIs ---------- */
/* 
Loads the additional input or personalization string into the NPA_DATAx registers.
   > Loads the proper number of bits (256 or 384) according to the security strength stored in the state handle.
*/
int elpclp890_load_ps_addin (elpclp890_state *state, void *input_str) {
   int err;
   int i, j;
   int str_size;
   
   DEBUG(">> elpclp890_load_ps_addin\n");
   
   /* return error if the pointer is NULL */
   if (!input_str) {err = CRYPTO_FAILED; goto ERR; }   
   
   /* calculate the length based on the security strength */
   if (state->status.sec_strength == SEC_STRNT_AES128) {
      str_size = 8; /* 256/32 */
   } else if (state->status.sec_strength == SEC_STRNT_AES256) {
      str_size = 12; /* 384/32 */
   }
   
   for (i = 0; i < str_size; i++) {
      pdu_io_write32(state->base + CLP890_REG_NPA_DATA0 + i, ((uint32_t*)input_str)[i]);
   }
   j = str_size + state->status.pad_ps_addin;
   /* if security strength is lowered after the DRBG is instantiated, pad PS and ADDIN with 0 at the MSB side */
   DEBUG("pad NPA_DATA with %u zeros at the MSB side\n", state->status.pad_ps_addin);
   for (i = str_size; i < j; i++) {      
      pdu_io_write32(state->base + CLP890_REG_NPA_DATA0 + i, 0);
   }
   
   err = CRYPTO_OK;
ERR:
   DEBUG("--- Return elpclp890_load_ps_addin, err = %i\n", err);
   return err;
} /* elpclp890_load_ps_addin */

/* ---------- Command APIs ---------- */
/*
Provides entropy and is used in both nonce and noise (self) seeding modes of operation:
   > If the CLP-890 is in the nonce mode, entropy must be provided by the user; otherwise (in the self-seeding mode) entropy will be generated by the internal entropy source of the CLP-890.
   > In the noise mode, calling the API will initiate a seeding command. Depending on the programmed security strength, a 256 or 384-bit seed will be generated.   
   > Inputs 2 and 3 are only used when the core is in the nonce mode.      
   > In the nonce mode, the CLP-890 can be seeded either through 2 or 3 blocks of 512-bit nonce values which are passed to the internal derivation function to increase the entropy, 
     or it can be seeded by a 256 or 384-bit nonce written directly into the SEEDx registers. 
     Passing a value of 1 to nonce_operation selects the former scenario and a value of 0 selects the latter. 
   > The input_nonce pointer must point to a memory location with a sufficient number of initialized bits. 
   > Table below shows the required number of bits depending on the nonce_operation and the security strength values.
      nonce_operation	                   |  Security Strength	  |    Bit length requirement
      ------------------------------------------------------------------------------------------
      1 (using the Derivation Function)	 |  SEC_STRNT_AES128	     |    2x512 = 1024
      1 (using the Derivation Function)	 |  SEC_STRNT_AES256	     |    3x512 = 1536
      0 (loading the seed into SEEDx)	    |  SEC_STRNT_AES128	     |    256
      0 (loading the seed into SEEDx)	    |  SEC_STRNT_AES256	     |    384
   > Generated entropy is secret information held securely within the HW and remains inaccessible to the user, unless the HW core is in the PROMISCUOUS mode.
*/
int elpclp890_get_entropy_input (elpclp890_state *state, void *input_nonce, int nonce_operation) {
   int err;   
   int nonce_ld_cntr = 0;
   int i, j;   
   
   DEBUG(">> elpclp890_get_entropy_input: seeding mode = %s, nonce_operation = %u\n", (state->status.nonce_mode ? "Nonce" : "Noise"), nonce_operation);
   
   /* make sure there is no alarm and the core is not busy */
   if ((err = elpclp890_get_alarms(state)))   { goto ERR; }
   if ((err = elpclp890_wait_on_busy(state))) { goto ERR; }

   /* --- Seeding --- */
   if (state->status.nonce_mode) { /* --- nonce mode --- */      
      if (!input_nonce) { err = CRYPTO_FAILED; goto ERR; }
      
      nonce_ld_cntr = 0;
         
      if (state->status.sec_strength == SEC_STRNT_AES128) {
         nonce_ld_cntr = 2;
      } else if (state->status.sec_strength == SEC_STRNT_AES256) {
         nonce_ld_cntr = 3;
      }
         
      if (nonce_operation) { /* load the noise inside NPA_DATAx register and issue gen_nonce command */                          
         for (i = 0; i < nonce_ld_cntr; i++) {
            /* load the nonoce */
            for (j = 0; j < 16; j++) {
               pdu_io_write32(state->base + CLP890_REG_NPA_DATA0 + j, ((uint32_t *)input_nonce)[16*i+j]);
            }
            
            /* issue the command and wait on done */
            pdu_io_write32(state->base + CLP890_REG_CTRL, CLP890_REG_CTRL_CMD_GEN_NONCE);
            if (elpclp890_wait_on_done(state)) { err = CRYPTO_FATAL; goto ERR; };
         }

      } else {
         /* load the nonoce */
         for (i = 0; i < 4*nonce_ld_cntr; i++) {
            pdu_io_write32(state->base + CLP890_REG_SEED0 + i, ((uint32_t *)input_nonce)[i]);
         }
      }
   } else { /* --- noise mode --- */
      /* issue the command and wait on done */
      DEBUG("issue the Gen_Noise command\n");
      pdu_io_write32(state->base + CLP890_REG_CTRL, CLP890_REG_CTRL_CMD_GEN_NOISE);
      if (elpclp890_wait_on_done(state)) { err = CRYPTO_FATAL; goto ERR; };
   }   
   
   err = CRYPTO_OK;
ERR:
   DEBUG("--- Return elpclp890_get_entropy_input, err = %i\n", err);
   return err;
} /* elpclp890_get_entropy_input */

/*
Generate Function: 
   > The Generate function in CLP-890 HW is broken down into 3 steps: Refresh_Addin, Gen_Random and Advance_State. 
      elpclp890_generate incorporates all these steps and some extra checks into one simple API.
   > There is one API for each step, below ||
                                           \/
*/
/*
Generate Part 1 - Refresh_Addin: Additional input string is used to add to the HW state entropy.
   > This API calls elpclp890_set_addin_present to set the ADDIN_PRESENT field of the MODE register to 1. 
   > Then it loads the additional input provided by addin_str pointer into the NPA_DATAx by calling the elpclp890_load_ps_addin. 
   > Then, it issues a Refresh_Addin command to the HW.
   > If the addin_str pointer is NULL, the API will return error.
*/
int elpclp890_refresh_addin (elpclp890_state *state, void *addin_str) {
   int err;
   
   DEBUG(">> elpclp890_refresh_addin\n");
   
   /* if the DRBG is not intantiated return error */
   if (!DRBG_INSTANTIATED(state->status.current_state)) { err = CRYPTO_FAILED; goto ERR; }
   
   /* make sure there is no alarm and the core is not busy */
   if ((err = elpclp890_get_alarms(state)))   { goto ERR; }
   if ((err = elpclp890_wait_on_busy(state))) { goto ERR; }
   
   /* This API should not be called with a NULL additional input string */
   if (!addin_str) { err = CRYPTO_FAILED; goto ERR; }
   
   /* set the ADDIN_PRESENT field of the MODE register to 1 */
   if ((err = elpclp890_set_addin_present(state, 1))) { goto ERR; }

   if ((err = elpclp890_load_ps_addin(state, addin_str))) { goto ERR; }
   
   /* execute the command and wait on done*/
   pdu_io_write32(state->base + CLP890_REG_CTRL, CLP890_REG_CTRL_CMD_REFRESH_ADDIN);
   if ((err = elpclp890_wait_on_done(state))) { goto ERR; }   
   
   err = 0;
ERR:
   DEBUG("--- Return elpclp890_refresh_addin, err = %i\n", err);
   return err;
} /* elpclp890_refresh_addin */

/*
Generate Part 2 - Gen_Random: generates the requested number of bits.
   > This API issues the Gen_Random command to the HW as many times as indicated by req_num_bytes to generate the requested number of bits. 
   > If the requested number of bits (i.e. 128×req_num_blks) is more than the maximum value specified by max_bits_per_req, the API will return error.
   > Random bits will be returned in random_bits.
*/
int elpclp890_gen_random (elpclp890_state *state, void *random_bits, unsigned long req_num_bytes) {
   int err;
   int i, j;
   uint32_t tmp;
   unsigned remained_bytes;
   unsigned long req_num_blks;
   
   DEBUG(">> elpclp890_gen_random: req_num_bytes = %lu\n", req_num_bytes);     
   
   /* if the DRBG is not intantiated return error */
   if (!DRBG_INSTANTIATED(state->status.current_state)) { err = CRYPTO_FAILED; goto ERR; }
   
   /* make sure there is no alarm and the core is not busy */
   if ((err = elpclp890_get_alarms(state)))   { goto ERR; }
   if ((err = elpclp890_wait_on_busy(state))) { goto ERR; }

   /* requested number of bits has to be less that the programmed maximum */
   if ((req_num_bytes*8) > state->counters.max_bits_per_req) { 
      DEBUG("requested number of bits (%lu) is larger than the set maximum (%lu)\n", (req_num_bytes*8), state->counters.max_bits_per_req);
      err = CRYPTO_FAILED; goto ERR; 
   }
   
   /* make sure random_bits is not NULL */
   if (!random_bits) { DEBUG("random_bits pointer cannot be NULL\n"); err = CRYPTO_FAILED; goto ERR; }
   
   /* loop on generate to get the requested number of bits. Each generate gives CLP890_RAND_BLK_SIZE_BITS bits. */   
   req_num_blks = ((req_num_bytes*8)%CLP890_RAND_BLK_SIZE_BITS) ? (((req_num_bytes*8)/CLP890_RAND_BLK_SIZE_BITS) + 1) : ((req_num_bytes*8)/CLP890_RAND_BLK_SIZE_BITS);

   for (i = 0; i < req_num_blks; i++) {            
      /* issue gen_random and wait on done */
      pdu_io_write32(state->base + CLP890_REG_CTRL, CLP890_REG_CTRL_CMD_GEN_RANDOM);
      if ((err = elpclp890_wait_on_done(state))) { goto ERR; }
      
      /* read the generated random number block and store */
      for (j = 0; j < (CLP890_RAND_BLK_SIZE_BITS/32); j++) {
         tmp = pdu_io_read32(state->base + CLP890_REG_RAND0 + j);
         /* copy to random_bits byte-by-byte, until req_num_bytes are copied */
         remained_bytes = req_num_bytes - (i*(CLP890_RAND_BLK_SIZE_BITS/8) + j*4);
         if (remained_bytes > 4) {
            memcpy(random_bits+i*(CLP890_RAND_BLK_SIZE_BITS/8) + j*4, &tmp, 4);
            
            /* decrement the bits counter and return error if generated more than the maximum*/
            state->counters.bits_per_req_left = state->counters.bits_per_req_left - 4*8;            
            if (state->counters.bits_per_req_left < 0 ) { err = CRYPTO_FAILED; goto ERR; }
         } else {
            memcpy(random_bits+i*(CLP890_RAND_BLK_SIZE_BITS/8) + j*4, &tmp, remained_bytes);
            
            /* decrement the bits counter and return error if generated more than the maximum*/
            state->counters.bits_per_req_left = state->counters.bits_per_req_left - remained_bytes*8;            
            if (state->counters.bits_per_req_left < 0 ) { err = CRYPTO_FAILED; goto ERR; }
            break;
         }         
      }     
   }
   
   err = CRYPTO_OK;
ERR:
   DEBUG("--- Return elpclp890_gen_random, err = %i\n", err);
   return err;
} /* elpclp890_gen_random */

/* 
Generate Part 3 - Advance the state: advances the state of the DRBG.
   > This API issues the Advance_State command to the HW. 
   > Then it updates the counter for the number of generate requests per seed. 
   > The counter must be checked every time before starting the Generate process and a reseed must be issued if the limit is reached. This check is incorporated inside elpclp890_generate API.
   > Note that we don't have to provide additional input again for this API, because if it had been provided in refresh_addin stage, HW will lock the NPA_DATAx, so it will be still available
*/
int elpclp890_advance_state (elpclp890_state *state) {
   int err;
   
   DEBUG(">> elpclp890_advance_state\n");
   
   /* if the DRBG is not intantiated return error */
   if (!DRBG_INSTANTIATED(state->status.current_state)) { err = CRYPTO_FAILED; goto ERR; }
   
   /* make sure there is no alarm and the core is not busy */
   if ((err = elpclp890_get_alarms(state)))   { goto ERR; }
   if ((err = elpclp890_wait_on_busy(state))) { goto ERR; }
   
   /* issue advance_state and wait on done */
   pdu_io_write32(state->base + CLP890_REG_CTRL, CLP890_REG_CTRL_CMD_ADVANCE_STATE);
   if ((err = elpclp890_wait_on_done(state))) { goto ERR; }
   
   /* generate is finished, reset the bits_per_req_left counter */
   state->counters.bits_per_req_left = state->counters.max_bits_per_req;
      
   --state->counters.req_per_seed_left;
   if (state->counters.req_per_seed_left < 0) { err = CRYPTO_FAILED; goto ERR; }   /* just a check */
   
   err = CRYPTO_OK;
ERR:
   DEBUG("--- Return elpclp890_advance_state, err = %i\n", err);
   return err;
} /* elpclp890_advance_state */

int elpclp890_check_seed_lifetime (elpclp890_state *state) {
   int err;
   
   if (state->counters.req_per_seed_left <= 0) { DEBUG("maximum number of requests per seed is reached\n"); err = CRYPTO_RESEED_REQUIRED; goto ERR; }
   
   err = CRYPTO_OK;
ERR:   
   return err;
}

/*
Perform Known Answer Test
   > The CLP-890 can perform a KAT on the DRBG and the derivation function inside the entropy source. There are also two different vectors available to do the KAT.
   > The kat_sel input selects whether the KAT should be performed on the DRBG or the derivation function. 
   > The kat_vec input chooses the KAT vector. 
   > Selections are done by writing the values to the MODE register. 
   > If the KAT fails, the API returns error.
*/
int elpclp890_kat (elpclp890_state *state, int kat_sel, int kat_vec) {
   int err;
   uint32_t tmp;
   
   DEBUG(">> elpclp890_kat: kat_sel = %u, kat_vec = %u\n", kat_sel, kat_vec);
   
   /* set KAT_SEL and KAT_VEC */
   tmp = pdu_io_read32(state->base + CLP890_REG_MODE);
   tmp = CLP890_REG_MODE_SET_KAT_SEL(tmp, kat_sel);
   tmp = CLP890_REG_MODE_SET_KAT_VEC(tmp, kat_vec);
   pdu_io_write32(state->base + CLP890_REG_MODE, tmp);
   
   /* issue the command and wait on kat_completed */
   pdu_io_write32(state->base + CLP890_REG_CTRL, CLP890_REG_CTRL_CMD_KAT);
   if ((err = elpclp890_wait_on_kat_completed(state))) { goto ERR; }
   
   /* check for alarms */
   if ((err = elpclp890_get_alarms(state)))   { goto ERR; }
   
   err = CRYPTO_OK;
ERR:
   DEBUG("--- Return elpclp890_kat, err = %i\n", err);
   return err;  
} /* elpclp890_kat */

/*
Performs a full KAT with all four combinations of the kat_sel and kat_vec
   > If any of the KAT fails, the API returns error.
*/
int elpclp890_full_kat (elpclp890_state *state) {
   int err;
   
   DEBUG(">> elpclp890_full_kat\n");
   
   /* SEL = 0, Vec = 0 */
   if ((err = elpclp890_kat(state, 0, 0))) { goto ERR; }
   
   /* SEL = 0, Vec = 1 */
   if ((elpclp890_kat(state, 0, 0))) { goto ERR; }
   
   /* SEL = 1, Vec = 0 */
   if ((elpclp890_kat(state, 1, 0))) { goto ERR; }
   
   /* SEL = 1, Vec = 1 */
   if ((elpclp890_kat(state, 1, 1))) { goto ERR; }
   
   err = CRYPTO_OK;
ERR:
   DEBUG("--- Return elpclp890_full_kat, err = %i\n", err);
   return err;
} /* elpclp890_full_kat */

/* 
max_bits_per_req reminder initialized by elpclp890_init can change using this API.
   > If this API is called when the DRBG is instantiated, an error will be returned. 
   > If the requested maximum is more than the standard's limit (determinded by CLP890_DFLT_MAX_BITS_PER_REQ), the API will return an error. 
*/
int elpclp890_set_reminder_max_bits_per_req (elpclp890_state *state, unsigned long max_bits_per_req) {
   int err;
   
   DEBUG(">> elpclp890_set_reminder_max_bits_per_req: %lu\n", max_bits_per_req);
   
   /* if the DRBG is instantiated, cannot change the value */
   if (DRBG_INSTANTIATED(state->status.current_state)) { DEBUG("cannot change the reminder value when DRBG is already instantiated\n"); err = CRYPTO_FAILED; goto ERR; }
   
   /* requested value cannot be more than NIST's limit */
   if (max_bits_per_req > CLP890_DFLT_MAX_BITS_PER_REQ) { DEBUG("requested max_bits_per_req is more than standard's limit\n"); err = CRYPTO_INVALID_ARGUMENT; goto ERR; }     
   
   state->counters.max_bits_per_req  = max_bits_per_req;
   state->counters.bits_per_req_left = max_bits_per_req;   
   
   err = CRYPTO_OK;
ERR:
   DEBUG("--- Return elpclp890_set_reminder_max_bits_per_req, err = %i\n", err);
   return err;
}

/* 
max_req_per_seed reminder initialized by elpclp890_init can change using this API.
   > If this API is called when the DRBG is instantiated, an error will be returned. 
   > If the requested maximum is more than the standard's limit (determinded by CLP890_DFLT_MAX_REQ_PER_SEED), the API will return an error. 
*/
int elpclp890_set_reminder_max_req_per_seed (elpclp890_state *state, unsigned long long max_req_per_seed) {
   int err;
   
   DEBUG(">> elpclp890_set_reminder_max_req_per_seed: %llu\n", max_req_per_seed);
   
   /* if the DRBG is instantiated, cannot change the value */
   if (DRBG_INSTANTIATED(state->status.current_state)) { DEBUG("cannot change the reminder value when DRBG is already instantiated\n"); err = CRYPTO_FAILED; goto ERR; }
   
   /* requested value cannot be more than NIST's limit */
   if (max_req_per_seed > CLP890_DFLT_MAX_REQ_PER_SEED) { DEBUG("requested max_req_per_seed is more than standard's limit\n"); err = CRYPTO_INVALID_ARGUMENT; goto ERR; }     
   
   state->counters.max_req_per_seed  = max_req_per_seed;
   state->counters.req_per_seed_left = max_req_per_seed;
   
   err = CRYPTO_OK;
ERR:
   DEBUG("--- Return elpclp890_set_reminder_max_req_per_seed, err = %i\n", err);
   return err;
}

/*
Zeroize command
   > A zeroize will not destroy the programmed mode and ALARM register value. 
     It keeps the programmed mode to avoid re-programming. 
     It also, maintains the ALARM register value, so that the user can read the value to understand the reason of the occurred alarm.
*/
int elpclp890_zeroize (elpclp890_state *state) {   
   int err;   
   
   DEBUG(">> elpclp890_zeroize: zeroize the core\n");

   /* issue zeroize command */
   pdu_io_write32(state->base + CLP890_REG_CTRL, CLP890_REG_CTRL_CMD_ZEROIZE);

   /* wait on zeroize done */
   if ((err = elpclp890_wait_on_zeroize(state))) { goto ERR; }   
   
   /* reset the SW state */
   elpclp890_reset_state(state);
   
   err = CRYPTO_OK;
   state->status.current_state = CLP890_STATE_UNINSTANTIATE;
ERR:   
   DEBUG("--- Return elpclp890_zeroize, err = %i\n", err);
   return err;
} /* elpclp890_zeroize */


