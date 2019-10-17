// ------------------------------------------------------------------------
//
//                (C) COPYRIGHT 2012 - 2016 SYNOPSYS, INC.
//                          ALL RIGHTS RESERVED
//
//  (C) COPYRIGHT 2012-2016 Synopsys, Inc. 
//  This Synopsys software and all associated documentation are 
//  proprietary to Synopsys, Inc. and may only be used pursuant 
//  to the terms and conditions of a written license agreement 
//  with Synopsys, Inc. All other use, reproduction, modification, 
//  or distribution of the Synopsys software or the associated 
//  documentation is strictly prohibited.
//
// ------------------------------------------------------------------------

#ifndef ELPCLP890_COMMON_H
#define ELPCLP890_COMMON_H

#define CLP890_RETRY_MAX 5000000UL

#define CLP890_DFLT_MAX_BITS_PER_REQ (1UL<<19)
#define CLP890_DFLT_MAX_REQ_PER_SEED (1ULL<<48)

/* Do not change the following parameters */
#define CLP890_DFLT_MAX_REJECTS 10

#define DEBUG(...)
//#define DEBUG(...) printk(__VA_ARGS__)

typedef enum elpclp890_sec_strength {
   SEC_STRNT_AES128 = 0,
   SEC_STRNT_AES256 = 1
} elpclp890_sec_strength;

typedef enum elpclp890_drbg_arch {
   AES128 = 0,
   AES256 = 1
} elpclp890_drbg_arch;

typedef enum elpclp890_current_state {   
   CLP890_STATE_INITIALIZE = 0,
   CLP890_STATE_UNINSTANTIATE,
   CLP890_STATE_INSTANTIATE,
   CLP890_STATE_RESEED,
   CLP890_STATE_GENERATE
} elpclp890_current_state;

typedef struct {
   uint32_t *base;
   
   /* Hardware features and build ID */
   struct {
      struct {
         elpclp890_drbg_arch drbg_arch;
         unsigned extra_ps_present,
                  secure_rst_state,
                  diag_level_trng3,
                  diag_level_stat_hlt,
                  diag_level_ns;
      } features;

      struct {
         unsigned stepping,
                  epn;
      } build_id;
   } config;

   /* status */
   struct {
      elpclp890_current_state current_state;
      unsigned nonce_mode,
               secure_mode,
               pred_resist;
      elpclp890_sec_strength sec_strength;
      unsigned pad_ps_addin;
      volatile unsigned alarm_code;
   } status;

   /* reminders and alarms */
   struct {
      unsigned long max_bits_per_req;
      unsigned long long max_req_per_seed;
      unsigned long bits_per_req_left;
      unsigned long long req_per_seed_left;
   } counters;
} elpclp890_state;

#define clp890_zero_status(x) memset(&(x->status), 0, sizeof (x->status))

#define DRBG_INSTANTIATED(cs) (((cs == CLP890_STATE_INSTANTIATE) || (cs == CLP890_STATE_RESEED) || (cs == CLP890_STATE_GENERATE)) ? 1 : 0)
#define REQ_SEC_STRENGTH_IS_VALID(sec_st) (((sec_st > 0) && (sec_st <= 256)) ? 1 : 0)

#endif
