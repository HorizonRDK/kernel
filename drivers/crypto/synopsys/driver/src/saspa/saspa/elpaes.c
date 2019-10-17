#include "elppdu.h"
#include "elpsaspa.h"
#include "elpsaspa_hw.h"

void elpaes_get_ctx_layout(const struct saspa_config *cfg,
                           struct saspa_aes_layout *layout)
{
   if (cfg->has_aes_xcbc && !cfg->has_aes_xcbc_1k) {
      layout->total_size = 0x60;
      layout->tag_offset = 0x50;
      layout->cbc_offset = 0x40;
      layout->ctr_offset = 0x30;
   } else if (cfg->has_aes_ccm || cfg->has_aes_f8 || cfg->has_aes_xcbc || cfg->has_aes_xts) {
      layout->total_size = 0x50;
      layout->tag_offset = 0x40;
      layout->cbc_offset = 0x30;
      layout->ctr_offset = 0x20;
   } else if (cfg->has_aes_cmac) {
      layout->total_size = 0x40;
      layout->tag_offset = 0x30;
      layout->cbc_offset = 0x20;
      layout->ctr_offset = 0x20;
   } else if (cfg->has_aes_cbc || cfg->has_aes_ctr) {
      layout->total_size = 0x30;
      layout->tag_offset = -1;
      layout->cbc_offset = 0x20;
      layout->ctr_offset = 0x20;
   } else {
      layout->total_size = 0x20;
      layout->tag_offset = -1;
      layout->cbc_offset = -1;
      layout->ctr_offset = -1;
   }
}

int elpaes_build_ctx(void *_out, int mode, const struct saspa_aes_ctx *ctx,
                     const struct saspa_aes_layout *layout)
{
   unsigned char *out = _out;

   memset(out, 0, layout->total_size);
   memcpy(out, ctx->saspa_aes_key, 32);

   switch (mode) {
   case AES_MODE_ECB:
      break;
   case AES_MODE_CBC:
      if (layout->cbc_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      memcpy(out + layout->cbc_offset, ctx->saspa_aes_cbc_iv, 16);
      break;
   case AES_MODE_CTR:
      if (layout->ctr_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      memcpy(out + layout->ctr_offset, ctx->saspa_aes_ctr_iv, 16);
      break;
   case AES_MODE_CCM:
      if (layout->cbc_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      if (layout->ctr_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      if (layout->tag_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      memcpy(out + layout->cbc_offset, ctx->saspa_aes_cbc_iv, 16);
      memcpy(out + layout->ctr_offset, ctx->saspa_aes_ctr_iv, 16);
      memcpy(out + layout->tag_offset, ctx->saspa_aes_tag, 16);
      break;
   case AES_MODE_CMAC:
      if (layout->cbc_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      if (layout->tag_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      memcpy(out + layout->cbc_offset, ctx->saspa_aes_cbc_iv, 16);
      memcpy(out + layout->tag_offset, ctx->saspa_aes_tag, 16);
      break;
   case AES_MODE_GCM:
      /* XXX: Not implemented */
      return CRYPTO_INVALID_MODE;
   case AES_MODE_XCBC:
      if (layout->ctr_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      if (layout->ctr_offset > 0x20) {
         memcpy(out + 0x20, ctx->saspa_aes_xcbc_k2, 16);
         memcpy(out + layout->ctr_offset, ctx->saspa_aes_xcbc_k3, 16);
      } else {
         memcpy(out + layout->ctr_offset, ctx->saspa_aes_xcbc_k1, 16);
      }
      break;
   case AES_MODE_F8:
      if (layout->total_size < 0x50)
         return CRYPTO_INVALID_MODE;
      memcpy(out + 0x20, ctx->saspa_aes_f8_salt_iv, 16);
      memcpy(out + 0x30, ctx->saspa_aes_f8_salt_j,  16);
      memcpy(out + 0x40, ctx->saspa_aes_f8_iv_s,    16);
      break;
   case AES_MODE_XTS:
      if (layout->total_size < 0x40)
         return CRYPTO_INVALID_MODE;
      memcpy(out + 0x30, ctx->saspa_aes_xts_key, 16);
      break;
   default:
      return CRYPTO_INVALID_ARGUMENT;
   }

   return 0;
}

int elpaes_read_ctx(const void *_in, int mode, struct saspa_aes_ctx *ctx, const struct saspa_aes_layout *layout)
{

   switch (mode) {
   case AES_MODE_ECB:
      break;
   case AES_MODE_CBC:
      if (layout->cbc_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      memcpy(ctx->saspa_aes_cbc_iv, _in + layout->cbc_offset, 16);
      break;
   case AES_MODE_CTR:
      if (layout->ctr_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      memcpy(ctx->saspa_aes_ctr_iv, _in + layout->ctr_offset, 16);
      break;
   case AES_MODE_CCM:
      if (layout->cbc_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      if (layout->ctr_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      if (layout->tag_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      memcpy(ctx->saspa_aes_cbc_iv, _in + layout->cbc_offset, 16);
      memcpy(ctx->saspa_aes_ctr_iv, _in + layout->ctr_offset, 16);
      memcpy(ctx->saspa_aes_tag,    _in + layout->tag_offset, 16);
      break;
   case AES_MODE_CMAC:
      if (layout->cbc_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      if (layout->tag_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      memcpy(ctx->saspa_aes_cbc_iv, _in + layout->cbc_offset, 16);
      memcpy(ctx->saspa_aes_tag,    _in + layout->tag_offset, 16);
      break;
   case AES_MODE_GCM:
      /* XXX: Not implemented */
      return CRYPTO_INVALID_MODE;
   case AES_MODE_XCBC:
      if (layout->ctr_offset >= layout->total_size)
         return CRYPTO_INVALID_MODE;
      if (layout->ctr_offset > 0x20) {
         memcpy(ctx->saspa_aes_xcbc_k2, _in + 0x20, 16);
         memcpy(ctx->saspa_aes_xcbc_k3, _in + layout->ctr_offset, 16);
      } else {
         memcpy(ctx->saspa_aes_xcbc_k1, _in + layout->ctr_offset, 16);
      }
      break;
   case AES_MODE_F8:
      if (layout->total_size < 0x50)
         return CRYPTO_INVALID_MODE;
      memcpy(ctx->saspa_aes_f8_salt_iv, _in + 0x20, 16);
      memcpy(ctx->saspa_aes_f8_salt_j,  _in + 0x30, 16);
      memcpy(ctx->saspa_aes_f8_iv_s,    _in + 0x40, 16);
      break;
   case AES_MODE_XTS:
      if (layout->total_size < 0x40)
         return CRYPTO_INVALID_MODE;
      memcpy(ctx->saspa_aes_xts_key, _in + 0x30, 16);
      break;
   default:
      return CRYPTO_INVALID_ARGUMENT;
   }

   return 0;
}
