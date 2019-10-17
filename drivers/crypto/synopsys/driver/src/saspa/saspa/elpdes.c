#include "elppdu.h"
#include "elpsaspa.h"
#include "elpsaspa_hw.h"

int elpdes_build_ctx(void *_out, const struct saspa_des_ctx *ctx)
{
   memcpy(_out + 0, ctx->iv,   8);
   memcpy(_out + 8, ctx->key, 24);
   return 0;
}

int elpdes_read_ctx(const void *_in, struct saspa_des_ctx *ctx)
{
   memcpy(ctx->iv,  _in + 0, 8);
   memcpy(ctx->key, _in + 8, 24);
   return 0;
}
