#ifndef ELPSASPA_H_
#define ELPSASPA_H_

#include "elppdu.h"

struct saspa_config {
   unsigned epn;
   unsigned mem_size;

   /* Feature bits for individual core presence. */
   unsigned has_aes:1,
            has_des:1,
            has_rc4:1,
            has_md5:1,
            has_sha1:1,
            has_sha224:1,
            has_sha256:1,
            has_sha384:1,
            has_sha512:1,
            has_snow:1,
            has_pka:1,
            has_trng:1;
   unsigned has_hmac;               

   /* Feature bits for AES. */
   unsigned has_aes_128:1,
            has_aes_192:1,
            has_aes_256:1,
            has_aes_cbc:1,
            has_aes_ctr:1,
            has_aes_ccm:1,
            has_aes_xcbc:1,
            has_aes_xcbc_1k:1,
            has_aes_f8:1,
            has_aes_gcm:1,
            has_aes_cmac:1,
            has_aes_xts:1,
            has_dpa:1;
};

/*
 * Describes the AES context offsets for fields that change depending on the
 * core configuration.  Fields that are always in the same place are not
 * described here.
 */
struct saspa_aes_layout {
   unsigned char total_size;
   unsigned char tag_offset;
   unsigned char ctr_offset; /* CCM/CTR initial counter value. */
   unsigned char cbc_offset; /* CCM/CBC/XCBC initial value. */
};

struct saspa_aes_ctx {
   unsigned char key[32];
   unsigned char tag[16];

   /* Don't access these fields directly, use the accessor macros below. */
   unsigned char buf[3][16];
};

#define saspa_aes_key        key
#define saspa_aes_tag        tag
#define saspa_aes_cbc_iv     buf[0]
#define saspa_aes_f8_iv_s    buf[0]
#define saspa_aes_ctr_iv     buf[1]
#define saspa_aes_f8_salt_j  buf[1]
#define saspa_aes_xts_key    buf[1]
#define saspa_aes_xcbc_k1    buf[1] /* 1-key mode only; overlaps k3 */
#define saspa_aes_xcbc_k3    buf[1]
#define saspa_aes_xcbc_k2    buf[2]
#define saspa_aes_f8_salt_iv buf[2]

struct saspa_des_ctx {
   unsigned char key[24];
   unsigned char iv[8];
};

struct saspa_hash_ctx {
   unsigned char curiv[64],
                 key[64];
   uint32_t      sslmac_seq;                 
};


void elpsaspa_get_config(uint32_t *regs, struct saspa_config *config);

void elpsaspa_set_byteswap(uint32_t *regs, int swap);
int elpsaspa_setup(uint32_t *regs, uint32_t ctx, uint32_t msg);

void elpsaspa_write_mem(uint32_t *dst, const void *src, size_t size);
void elpsaspa_read_mem(void *dst, uint32_t *src, size_t size);

void elpaes_get_ctx_layout(const struct saspa_config *config,
                           struct saspa_aes_layout *layout);

int elpaes_build_ctx(void *out, int mode, const struct saspa_aes_ctx *ctx,const struct saspa_aes_layout *layout);
int elpaes_read_ctx(const void *_in, int mode, struct saspa_aes_ctx *ctx, const struct saspa_aes_layout *layout);

int elpdes_build_ctx(void *_out, const struct saspa_des_ctx *ctx);
int elpdes_read_ctx(const void *_in, struct saspa_des_ctx *ctx);


#endif
