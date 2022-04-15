#include "elpclp800.h"
#include "elpclp800_hw.h"

/*
 * Helper to perform a read-modify-write to change a specified register.
 * Any bits in the register which are set in mask are assigned with the
 * corresponding bit in val.  All other bits are unchanged.
 */
static uint32_t elpclp800_rmw(struct elpclp800_state *clp800, unsigned reg,
                                             uint32_t val, uint32_t mask)
{
   uint32_t tmp;

   tmp = elpclp800_readreg(clp800, reg);
   tmp &= ~mask;
   tmp |= (val & mask);
   elpclp800_writereg(clp800, reg, tmp);

   return tmp;
}

static void clp800_mmio_writereg(void *base, unsigned offset, uint32_t val)
{
   uint32_t *regs = base;

   pdu_io_write32(&regs[offset], val);
}

static uint32_t clp800_mmio_readreg(void *base, unsigned offset)
{
   uint32_t *regs = base;

   return (uint32_t)pdu_io_read32(&regs[offset]);
}

void elpclp800_setup(struct elpclp800_state *clp800, uint32_t *regbase)
{
   uint32_t tmp;

   if (regbase) {
      clp800->writereg = clp800_mmio_writereg;
      clp800->readreg  = clp800_mmio_readreg;
      clp800->regbase  = regbase;
   }

   /* Put the device into a sane initial state */
   elpclp800_disable_irqs(clp800, -1);
   elpclp800_writereg(clp800, CLP800_ISTAT, -1);
   elpclp800_rmw(clp800, CLP800_SMODE, 0, 1ul<<CLP800_SMODE_NONCE);

   /* Parse out BUILD_ID register */
   tmp = elpclp800_readreg(clp800, CLP800_BUILD_ID);
   clp800->epn       = (uint16_t)(tmp >> CLP800_BUILD_ID_EPN);
   clp800->epn      &= (uint16_t)((1ul << CLP800_BUILD_ID_EPN_BITS) - 1);
   clp800->stepping  = (uint16_t)(tmp >> CLP800_BUILD_ID_STEPPING);
   clp800->stepping &= (uint16_t)((1ul << CLP800_BUILD_ID_STEPPING_BITS) - 1);

   /* Parse out FEATURES register */
   tmp = elpclp800_readreg(clp800, CLP800_FEATURES);
   clp800->secure_reset = (tmp >> CLP800_FEATURES_SECURE_RST) & 1;
   clp800->rings_avail = (tmp >> CLP800_FEATURES_RAND_SEED) & 1;

   clp800->output_len  = (uint16_t)(tmp >> CLP800_FEATURES_RAND_LEN);
   clp800->output_len &= (uint16_t)((1ul << CLP800_FEATURES_RAND_LEN_BITS) - 1);
   clp800->output_len  = (uint16_t)(16ul << clp800->output_len);

   clp800->diag_level  = (uint16_t)(tmp >> CLP800_FEATURES_DIAG_LEVEL);
   clp800->diag_level &= (uint16_t)((1ul << CLP800_FEATURES_DIAG_LEVEL_BITS)
		 - 1);
}

int elpclp800_reseed(struct elpclp800_state *clp800, const void *nonce)
{
   uint32_t stat, smode, nonce_buf[8];
   size_t i;

   if (nonce) {
      smode = elpclp800_rmw(clp800, CLP800_SMODE, -1, 1ul<<CLP800_SMODE_NONCE);

      memcpy(nonce_buf, nonce, sizeof nonce_buf);
      do {
         stat = elpclp800_readreg(clp800, CLP800_STAT);
      } while (((stat >> CLP800_STAT_NONCE_MODE) & 1) == 0);

      for (i = 0; i < sizeof nonce_buf / sizeof nonce_buf[0]; i++) {
         elpclp800_writereg(clp800, (uint32_t)(CLP800_SEED_BASE+i),
		nonce_buf[i]);
      }
      elpclp800_writereg(clp800, CLP800_CTRL, CLP800_CMD_NONCE_RESEED);
      elpclp800_writereg(clp800, CLP800_SMODE,
	smode & (uint32_t)(~(1ul<<CLP800_SMODE_NONCE)));
   } else {
      if (!clp800->rings_avail)
         return CRYPTO_MODULE_DISABLED;

      elpclp800_rmw(clp800, CLP800_SMODE, 0, 1ul<<CLP800_SMODE_NONCE);
      elpclp800_writereg(clp800, CLP800_CTRL, CLP800_CMD_RAND_RESEED);
   }

   return 0;
}

int elpclp800_get_seed(struct elpclp800_state *clp800, void *out)
{
   uint32_t stat, seed_buf[8];
   size_t i;

   stat = elpclp800_readreg(clp800, CLP800_STAT);
   if (!((stat >> CLP800_STAT_SEEDED) & 1))
      return CRYPTO_NOT_INITIALIZED;

   for (i = 0; i < sizeof seed_buf / sizeof seed_buf[0]; i++) {
      seed_buf[i] = elpclp800_readreg(clp800, (uint32_t)(CLP800_SEED_BASE+i));
   }

   memcpy(out, seed_buf, sizeof seed_buf);
   return 0;
}

void elpclp800_enable_irqs(struct elpclp800_state *clp800, uint32_t irq_mask)
{
   irq_mask &= CLP800_IRQ_ALL_MASK;

   elpclp800_rmw(clp800, CLP800_IE, -1, irq_mask);
}

void elpclp800_disable_irqs(struct elpclp800_state *clp800, uint32_t irq_mask)
{
   irq_mask &= CLP800_IRQ_ALL_MASK;

   elpclp800_rmw(clp800, CLP800_IE, 0, irq_mask);
}

void elpclp800_set_secure(struct elpclp800_state *clp800, int secure)
{
   if (secure)
      elpclp800_rmw(clp800, CLP800_SMODE, -1, 1ul << CLP800_SMODE_SECURE);
   else
      elpclp800_rmw(clp800, CLP800_SMODE, 0, 1ul << CLP800_SMODE_SECURE);
}

int elpclp800_set_max_rejects(struct elpclp800_state *clp800, unsigned rejects)
{
   uint32_t val, mask;

   if (rejects >= (1ul << CLP800_SMODE_MAX_REJECTS_BITS))
      return CRYPTO_INVALID_ARGUMENT;

   val  = rejects << CLP800_SMODE_MAX_REJECTS;
   mask = ((1ul << CLP800_SMODE_MAX_REJECTS_BITS)-1) << CLP800_SMODE_MAX_REJECTS;
   elpclp800_rmw(clp800, CLP800_SMODE, val, mask);

   return 0;
}

int elpclp800_set_output_len(struct elpclp800_state *clp800, unsigned outlen)
{
   switch (outlen) {
   case 16:
      elpclp800_rmw(clp800, CLP800_MODE, 0, 1ul << CLP800_MODE_R256);
      return 0;
   case 32:
      if (clp800->output_len < 32)
         return CRYPTO_MODULE_DISABLED;
      elpclp800_rmw(clp800, CLP800_MODE, -1, 1ul << CLP800_MODE_R256);
      return 0;
   }

   return CRYPTO_INVALID_ARGUMENT;
}

int elpclp800_set_request_reminder(struct elpclp800_state *clp800,
                                  unsigned long val)
{
   if (val > ULONG_MAX - CLP800_AUTO_RQST_RESOLUTION)
      return CRYPTO_INVALID_ARGUMENT;

   val = (val + CLP800_AUTO_RQST_RESOLUTION - 1) / CLP800_AUTO_RQST_RESOLUTION;
   if (val >= 1ul << CLP800_AUTO_RQST_RQSTS_BITS)
      return CRYPTO_INVALID_ARGUMENT;

   elpclp800_writereg(clp800, CLP800_AUTO_RQST, (uint32_t)val);
   return 0;
}

int elpclp800_set_age_reminder(struct elpclp800_state *clp800,
                              unsigned long long val)
{
   if (val > ULLONG_MAX - CLP800_AUTO_AGE_RESOLUTION)
      return CRYPTO_INVALID_ARGUMENT;

   val = (val + CLP800_AUTO_AGE_RESOLUTION - 1) / CLP800_AUTO_AGE_RESOLUTION;
   if (val >= 1ul << CLP800_AUTO_AGE_AGE_BITS)
      return CRYPTO_INVALID_ARGUMENT;

   elpclp800_writereg(clp800, CLP800_AUTO_AGE, (uint32_t)val);
   return 0;
}

int elpclp800_get_random(struct elpclp800_state *clp800, void *out)
{
   uint32_t stat, istat;
   size_t i, outlen;
   uint32_t rand_buf[8];

   stat = elpclp800_readreg(clp800, CLP800_STAT);
   if (!((stat >> CLP800_STAT_SEEDED) & 1))
      return CRYPTO_NOT_INITIALIZED;
   if ((stat >> CLP800_STAT_GENERATING) & 1)
      return CRYPTO_INPROGRESS;

   istat = elpclp800_readreg(clp800, CLP800_ISTAT);
   if (!(istat & CLP800_IRQ_RAND_RDY_MASK))
      return 0;

   outlen = (stat >> CLP800_STAT_R256) & 1 ? 8 : 4;
   for (i = 0; i < outlen; i++) {
      rand_buf[i] = elpclp800_readreg(clp800,
		(uint32_t)(CLP800_RAND_BASE + i));
   }

   elpclp800_writereg(clp800, CLP800_ISTAT, CLP800_IRQ_RAND_RDY_MASK);

   if (out) {
      memcpy(out, rand_buf, i * sizeof rand_buf[0]);
   }

   return (int)(i * sizeof rand_buf[0]);
}
