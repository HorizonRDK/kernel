#include "elppdu.h"
#include "elpsaspa.h"
#include "elpsaspa_hw.h"

void elpsaspa_get_config(uint32_t *regs, struct saspa_config *out)
{
   struct saspa_config cfg = {0};
   uint32_t tmp;

   tmp = pdu_io_read32(&regs[SASPA_GLBL_EPN_ID]);
   cfg.epn = (tmp >> SASPA_ID_EPN) & ((1ul << SASPA_ID_EPN_BITS)-1);

   /* Flags for each present core. */
   cfg.has_pka  = !!(tmp & (1ul << SASPA_ID_PKA));
   cfg.has_trng = !!(tmp & (1ul << SASPA_ID_TRNG));
   cfg.has_aes  = !!(tmp & (1ul << SASPA_ID_AES));
   cfg.has_des  = !!(tmp & (1ul << SASPA_ID_DES));
   cfg.has_rc4  = !!(tmp & (1ul << SASPA_ID_RC4));
   cfg.has_snow = !!(tmp & (1ul << SASPA_ID_SNOW));
   cfg.has_md5  = !!(tmp & (1ul << SASPA_ID_MD5));
   cfg.has_sha1  = !!(tmp & (1ul << SASPA_ID_SHA1));
   cfg.has_sha224  = !!(tmp & (1ul << SASPA_ID_SHA224));
   cfg.has_sha256  = !!(tmp & (1ul << SASPA_ID_SHA256));
   cfg.has_sha384  = !!(tmp & (1ul << SASPA_ID_SHA384));
   cfg.has_sha512  = !!(tmp & (1ul << SASPA_ID_SHA512));
   cfg.has_hmac  = !!((tmp >> SASPA_ID_HMAC) & ((1<<SASPA_ID_HMAC_BITS)-1));

   tmp = pdu_io_read32(&regs[SASPA_GLBL_FEATURE1]);

   /* Flags for AES. */
   cfg.has_aes_128     = !!(tmp & (1ul << SASPA_FEAT_AES_128));
   cfg.has_aes_192     = !!(tmp & (1ul << SASPA_FEAT_AES_192));
   cfg.has_aes_256     = !!(tmp & (1ul << SASPA_FEAT_AES_256));
   cfg.has_aes_cbc     = !!(tmp & (1ul << SASPA_FEAT_AES_CBC));
   cfg.has_aes_ctr     = !!(tmp & (1ul << SASPA_FEAT_AES_CTR));
   cfg.has_aes_ccm     = !!(tmp & (1ul << SASPA_FEAT_AES_CCM));
   cfg.has_aes_xcbc    = !!(tmp & (1ul << SASPA_FEAT_AES_XCBC));
   cfg.has_aes_xcbc_1k = !!(tmp & (1ul << SASPA_FEAT_AES_XCBC_1KEY));
   cfg.has_aes_f8      = !!(tmp & (1ul << SASPA_FEAT_AES_F8));
   cfg.has_aes_gcm     = !!(tmp & (1ul << SASPA_FEAT_AES_GCM));
   cfg.has_aes_cmac    = !!(tmp & (1ul << SASPA_FEAT_AES_CMAC));
   cfg.has_aes_xts     = !!(tmp & (1ul << SASPA_FEAT_AES_XTS));
   cfg.has_dpa         = !!(tmp & (1ul << SASPA_FEAT_AES_DPA));

   *out = cfg;
}

void elpsaspa_set_byteswap(uint32_t *regs, int swap)
{
   pdu_io_write32(&regs[SASPA_GLBL_ENDIAN_SWAP], !!swap);
}

int elpsaspa_setup(uint32_t *regs, uint32_t ctx, uint32_t msg)
{
   if ((ctx | msg) & 3 || (ctx | msg) >= 0x4000)
      return CRYPTO_INVALID_ARGUMENT;

   pdu_io_write32(&regs[SASPA_GLBL_CTX_OFFSET], ctx);
   pdu_io_write32(&regs[SASPA_GLBL_MSG_OFFSET], msg);

   return 0;
}

void elpsaspa_write_mem(uint32_t *dst, const void *_src, size_t size)
{
   const unsigned char *src = _src;
   uint32_t val;
   size_t i;

   for (i = 0; i+4 < size; i+=4) {
      memcpy(&val, src+i, sizeof val);
      pdu_io_write32(dst + (i>>2), val);
   }

   val = 0;
   memcpy(&val, src+i, size-i);
   pdu_io_write32(dst + (i>>2), val);
}

void elpsaspa_read_mem(void *_dst, uint32_t *src, size_t size)
{
   unsigned char *dst = _dst;
   uint32_t val;
   size_t i;

   for (i = 0; i+4 < size; i+=4) {
      val = pdu_io_read32(src + (i>>2));
      memcpy(dst+i, &val, sizeof val);
   }

   val = pdu_io_read32(src + (i>>2));
   memcpy(dst+i, &val, size-i);
}
