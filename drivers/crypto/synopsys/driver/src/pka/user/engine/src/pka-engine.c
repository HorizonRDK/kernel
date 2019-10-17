#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>
#include <openssl/engine.h>

#include "pkadev.h"

static const char elppka_engine_id[] = "elppka";
static const char elppka_engine_name[] = "pka engine support";

static int rsa_idx = -1;

struct elppka_ctx {
   int fd;
   size_t size;
   unsigned char *m, *e, *x, *y;
};

/* Convert an OpenSSL bignum into a PKA operand, size bytes long. */
static void store_bn(unsigned char *buf, size_t size, const BIGNUM *bn)
{
   size_t len = BN_num_bytes(bn);

   assert(len <= size);
   memset(buf, 0, size-len);
   BN_bn2bin(bn, &buf[size-len]);
}

/*
 * Convert a PKA operand, size bytes long, into an OpenSSL bignum.
 * Returns 1 on success, 0 on failure.
 */
static int read_bn(BIGNUM *bn, const unsigned char *buf, size_t size)
{
   return BN_bin2bn(buf, size, bn) != NULL;
}

static struct elppka_ctx *elppka_ctx_init(size_t opsize)
{
   struct {
      struct elppka_ctx ctx;
      char buf[];
   } *alloc;

   alloc = OPENSSL_malloc(sizeof *alloc + opsize*4);
   if (!alloc)
      return NULL;

   alloc->ctx = (struct elppka_ctx) {
      .fd   = elppka_device_open(NULL),
      .m    = &alloc->buf[0*opsize],
      .e    = &alloc->buf[1*opsize],
      .x    = &alloc->buf[2*opsize],
      .y    = &alloc->buf[3*opsize],
      .size = opsize,
   };

   if (alloc->ctx.fd == -1) {
      perror("elppka_device_open");
      goto err;
   }

   return &alloc->ctx;
err:
   OPENSSL_free(alloc);
   return NULL;
}

static void elppka_ctx_done(struct elppka_ctx *ctx)
{
   elppka_device_close(ctx->fd);
   OPENSSL_free(ctx);
}

static int run_pka(struct elppka_ctx *pka, const char *func, ...)
{
   va_list ap;
   int rc;

   va_start(ap, func);
   rc = elppka_vrun(pka->fd, func, pka->size, ap);
   va_end(ap);

   if (rc == -1) {
      perror(func);
      return 0;
   } else if (rc > 0) {
      fprintf(stderr, "PKA returned error: %d\n", rc);
      return 0;
   }

   return 1;
}

static int do_modexp(BIGNUM *out, struct elppka_ctx *ctx)
{
   /* Montgomery precomputation. */
   if (!run_pka(ctx, "calc_r_inv",
                "%D0",  ctx->m,
                "=%C0", ctx->y,
                (char *)NULL))
      return 0;

   if (!run_pka(ctx, "calc_mp", (char *)NULL))
      return 0;

   if (!run_pka(ctx, "calc_r_sqr",
                "%D0", ctx->m,
                "%C0", ctx->y,
                (char *)NULL))
      return 0;

   /* Modular exponentiation. */
   if (!run_pka(ctx, "modexp",
                "%A0",  ctx->x,
                "%D2",  ctx->e,
                "%D0",  ctx->m,
                "=%A0", ctx->y,
                (char *)NULL))
      return 0;

   return read_bn(out, ctx->y, ctx->size);
}

static int elppka_bn_mod_exp(BIGNUM *r, const BIGNUM *a, const BIGNUM *p,
                             const BIGNUM *m, BN_CTX *bnctx, BN_MONT_CTX *mctx)
{
   struct elppka_ctx *ctx;
   int ret;

   ctx = elppka_ctx_init(BN_num_bytes(m));
   if (!ctx)
      return 0;

   store_bn(ctx->m, ctx->size, m);
   store_bn(ctx->e, ctx->size, p);
   store_bn(ctx->x, ctx->size, a);

   ret = do_modexp(r, ctx);

   elppka_ctx_done(ctx);

   return ret;
}

static int elppka_rsa_mod_exp(BIGNUM *r0, const BIGNUM *I,
                              RSA *rsa, BN_CTX *ctx)
{
   return elppka_bn_mod_exp(r0, I, rsa->d, rsa->n, ctx, NULL);
}

static RSA_METHOD elppka_rsa = {
   .rsa_mod_exp = elppka_rsa_mod_exp,
   .bn_mod_exp = elppka_bn_mod_exp,
};

static int elppka_init(ENGINE *e)
{
   /*
    * XXX: "PKA device handle" needs to be in a variable as this function
    * seems to key based on the /address/ it's passed, not the string.
    */
   if (rsa_idx == -1)
      rsa_idx = RSA_get_ex_new_index(0, "PKA device handle", 0, 0, 0);
   if (rsa_idx == -1)
      return 0;
   return 1;
}

static int elppka_bind(ENGINE *e, const char *id)
{
   if (id && strcmp(id, elppka_engine_id) != 0)
      return 0;
   if (!ENGINE_set_id(e, elppka_engine_id))
      return 0;
   if (!ENGINE_set_name(e, elppka_engine_name))
      return 0;
   if (!ENGINE_set_init_function(e, elppka_init))
      return 0;
   if (!ENGINE_set_RSA(e, &elppka_rsa))
      return 0;

   elppka_rsa.rsa_pub_enc  = RSA_PKCS1_SSLeay()->rsa_pub_enc;
   elppka_rsa.rsa_pub_dec  = RSA_PKCS1_SSLeay()->rsa_pub_dec;
   elppka_rsa.rsa_priv_enc = RSA_PKCS1_SSLeay()->rsa_priv_enc;
   elppka_rsa.rsa_priv_dec = RSA_PKCS1_SSLeay()->rsa_priv_dec;

   return 1;
}
IMPLEMENT_DYNAMIC_CHECK_FN()
IMPLEMENT_DYNAMIC_BIND_FN(elppka_bind)
