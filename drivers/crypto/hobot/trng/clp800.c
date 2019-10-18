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
 * Copyright (c) 2012-2017 Synopsys, Inc. and/or its affiliates.
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

#define DRVNAME "clp800"

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/hw_random.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/sched/signal.h>
#include "clp800_driver.h"

/*
 * Return true iff a reseed has been started but not yet acknowledged by
 * the interrupt handler.  If there is no IRQ, then acknowledge seed
 * completion here to support polling operation.
 */
static bool is_reseeding(struct device *dev)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   uint32_t stat = elpclp800_readreg(&priv->elpclp800, CLP800_STAT);

   if ((stat >> CLP800_STAT_RESEEDING) & 1)
      return true;

   if ((stat >> CLP800_STAT_SRVC_RQST) & 1) {
      /* There is a pending IRQ indicated; need to check ISTAT. */
      uint32_t istat = elpclp800_readreg(&priv->elpclp800, CLP800_ISTAT);

      if ((istat >> CLP800_IRQ_SEED_DONE) & 1) {
         if (!priv->has_irq) {
            unsigned long flags;

            /* In no-IRQ mode, handle generation increment here and now. */
            spin_lock_irqsave(&priv->cfg_lock, flags);
            priv->seed_generation++;
            elpclp800_writereg(&priv->elpclp800, CLP800_ISTAT,
                               1ul<<CLP800_IRQ_SEED_DONE);
            spin_unlock_irqrestore(&priv->cfg_lock, flags);
         } else {
            /* ISR will handle generation increment */
            return true;
         }
      }
   }

   return false;
}

/*
 * Wait for completion of an inprogress reseed.  This will be done by polling
 * or by IRQ.  If interruptible is true, the operation will be interruptible,
 * otherwise it is killable.
 */
static int wait_for_reseed(struct device *dev, bool interruptible)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);

   if (priv->has_irq && interruptible) {
      return wait_event_interruptible(priv->reseed_wq, !is_reseeding(dev));
   } else if (priv->has_irq) {
      return wait_event_killable(priv->reseed_wq, !is_reseeding(dev));
   } else if (interruptible) {
      while (is_reseeding(dev)) {
         if (signal_pending(current))
            return -ERESTARTSYS;
         schedule_timeout_interruptible(msecs_to_jiffies(50));
      }
   } else {
      while (is_reseeding(dev)) {
         if (fatal_signal_pending(current))
            return -ERESTARTSYS;
         schedule_timeout_killable(msecs_to_jiffies(50));
      }
   }

   return 0;
}

static ssize_t
show_reseeding(struct device *dev, struct device_attribute *devattr, char *buf) {
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   u32 stat = elpclp800_readreg(&priv->elpclp800, CLP800_STAT);
   stat = stat >> CLP800_STAT_RESEEDING;
   return sprintf(buf, "%d\n",stat);
}


static bool is_secure(struct device *dev)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   u32 stat = elpclp800_readreg(&priv->elpclp800, CLP800_STAT);

   return (stat >> CLP800_STAT_SECURE) & 1;
}

static ssize_t
show_epn(struct device *dev, struct device_attribute *devattr, char *buf)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);

   return sprintf(buf, "0x%.4hx\n", priv->elpclp800.epn);
}

static ssize_t
show_tts(struct device *dev, struct device_attribute *devattr, char *buf)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   u32 tts = elpclp800_readreg(&priv->elpclp800, CLP800_TTS);
   return sprintf(buf, "%d\n",tts);
}

static ssize_t
show_stepping(struct device *dev, struct device_attribute *devattr, char *buf)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);

   return sprintf(buf, "%hu\n", priv->elpclp800.stepping);
}

static ssize_t
show_seed(struct device *dev, struct device_attribute *devattr, char *buf)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   unsigned char tmp[sizeof priv->nonce_buf];
   unsigned long long seed_start, seed_end;
   size_t i;
   int rc;

   /*
    * If a background reseed is in progress while reading the seed registers,
    * the result may contain a mixture of old and new seed materials (and is
    * therefore bogus).  Moreover, S/W does not necessarily have complete
    * control over when the engine is reseeded so we cannot entirely avoid
    * this situation.
    *
    * Since completely preventing the race is impossible, we will instead
    * validate after the fact whether or not a reseed occurred during our
    * read of the seed registers.  If no reseed occurred, then the seed
    * registers were stable and we return that.  Otherwise, we retry the
    * read.  We implement this by maintaining a counter that increments
    * whenever the engine reseeds, plus some checks in this function to
    * deal with IRQ latency.
    */

   do {
      rc = wait_for_reseed(dev, true);
      if (rc < 0)
         return rc;

      device_lock(dev);
      if (is_secure(dev)) {
         /* The seed registers are only readable in promiscuous mode. */
         device_unlock(dev);
         return -EPERM;
      }

      spin_lock_irq(&priv->cfg_lock);
      seed_start = priv->seed_generation;
      spin_unlock_irq(&priv->cfg_lock);

      rc = elpclp800_get_seed(&priv->elpclp800, tmp);
      device_unlock(dev);

      if (rc < 0)
         return pdu_error_code(rc);

      if (is_reseeding(dev)) {
         seed_end = seed_start + 1;
      } else {
         spin_lock_irq(&priv->cfg_lock);
         seed_end = priv->seed_generation;
         spin_unlock_irq(&priv->cfg_lock);
      }
   } while (seed_start != seed_end);

   for (i = 0; i < sizeof tmp; i++) {
      sprintf(buf+2*i, "%.2hhx\n", tmp[i]);
   }

   return 2*i+1;
}

static ssize_t
show_mode(struct device *dev, struct device_attribute *devattr, char *buf)
{
   if (is_secure(dev))
      return sprintf(buf, "secure\n");
   return sprintf(buf, "promiscuous\n");
}

static ssize_t
store_mode(struct device *dev, struct device_attribute *devattr,
                               const char *buf, size_t count)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   bool newmode;

   if (sysfs_streq(buf, "secure"))
      newmode = true;
   else if (sysfs_streq(buf, "promiscuous"))
      newmode = false;
   else
      return -EINVAL;

   device_lock(dev);

   if (is_secure(dev) != newmode) {
      spin_lock_irq(&priv->cfg_lock);
      elpclp800_set_secure(&priv->elpclp800, newmode);

      /* Pending alarms are invalid after a mode change */
      priv->alarm_reason = 0;
      spin_unlock_irq(&priv->cfg_lock);

      kobject_uevent(&dev->kobj, KOBJ_CHANGE);
   }

   device_unlock(dev);

   return count;
}

static ssize_t
store_seed(struct device *dev, struct device_attribute *devattr,
                               const char *buf, size_t count)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   unsigned char tmp[sizeof priv->nonce_buf] = {0};
   size_t i, pos;
   int cur = -1;

   if (count == sizeof tmp) {
      /* Binary input. */
      memcpy(tmp, buf, sizeof tmp);
   } else {
      /* Hexadecimal input. */
      for (i = pos = 0; i < count && pos < sizeof tmp; i++) {
         int rc;

         if (isspace(buf[i]))
            continue;

         rc = hex_to_bin(buf[i]);
         if (rc < 0)
            return -EINVAL;

         if (cur == -1) {
            cur = rc << 4;
         } else {
            tmp[pos++] = cur | rc;
            cur = -1;
         }
      }

      if (pos != sizeof tmp)
         return -EINVAL;

      /* Allow for trailing whitespace. */
      for (; i < count; i++) {
         if (!isspace(buf[i]))
            return -EINVAL;
      }
   }

   device_lock(dev);
   memcpy(priv->nonce_buf, tmp, sizeof priv->nonce_buf);
   priv->nonce_valid = true;
   device_unlock(dev);

   return count;
}

static ssize_t
show_reseed(struct device *dev, struct device_attribute *devattr, char *buf)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   u32 stat = elpclp800_readreg(&priv->elpclp800, CLP800_STAT);

   stat >>= CLP800_STAT_RESEED_REASON;
   stat &= (1ul << CLP800_STAT_RESEED_REASON_BITS) - 1;
   switch (stat) {
   case CLP800_RESEED_HOST:
      return sprintf(buf, "random (host command)\n");
   case CLP800_RESEED_PIN:
      return sprintf(buf, "random (external pin)\n");
   case CLP800_RESEED_NONCE:
      return sprintf(buf, "nonce\n");
   case CLP800_RESEED_UNSEEDED:
      return sprintf(buf, "unseeded\n");
   }

   return sprintf(buf, "unknown (%d)\n", (int)stat);
}

static int reseed_action_random(struct device *dev)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   unsigned long long seed_start, seed_end;
   int rc;

   spin_lock_irq(&priv->cfg_lock);
   seed_start = priv->seed_generation;
   rc = elpclp800_reseed(&priv->elpclp800, NULL);
   spin_unlock_irq(&priv->cfg_lock);

   if (rc < 0)
      return pdu_error_code(rc);

   do {
      rc = wait_for_reseed(dev, false);
      if (rc < 0)
         return rc;

      spin_lock_irq(&priv->cfg_lock);
      seed_end = priv->seed_generation;
      spin_unlock_irq(&priv->cfg_lock);
   } while (seed_start == seed_end);

   return 0;
}

static int reseed_action_nonce(struct device *dev)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   int rc;

   device_lock(dev);
   if (!priv->nonce_valid) {
      device_unlock(dev);
      return -ENODATA;
   }

   /*
    * Nonce reseed does not trigger an IRQ, so we must increment the
    * generation ourselves.
    */

   spin_lock_irq(&priv->cfg_lock);
   rc = elpclp800_reseed(&priv->elpclp800, priv->nonce_buf);
   if (rc < 0) {
      spin_unlock_irq(&priv->cfg_lock);
      device_unlock(dev);
      return pdu_error_code(rc);
   }

   priv->seed_generation++;
   spin_unlock_irq(&priv->cfg_lock);

   priv->nonce_valid = false;
   device_unlock(dev);

   /*
    * Since a nonce reseed aborts any in-progress
    * random reseed, wake up the waiters.
    */
   wake_up_all(&priv->reseed_wq);
   return 0;
}

static ssize_t
store_reseed(struct device *dev, struct device_attribute *devattr,
                                 const char *buf, size_t count)
{
   int rc = -EINVAL;

   if (sysfs_streq(buf, "random")) {
      rc = reseed_action_random(dev);
   } else if (sysfs_streq(buf, "nonce")) {
      rc = reseed_action_nonce(dev);
   }

   if (rc < 0)
      return rc;
   return count;
}

static ssize_t
show_gen(struct device *dev, struct device_attribute *devattr, char *buf)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   unsigned long long val;

   spin_lock_irq(&priv->cfg_lock);
   val = priv->seed_generation;
   spin_unlock_irq(&priv->cfg_lock);

   return sprintf(buf, "%llu\n", val);
}

static ssize_t
show_max_rej(struct device *dev, struct device_attribute *devattr, char *buf)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   u32 val;

   val   = elpclp800_readreg(&priv->elpclp800, CLP800_SMODE);
   val >>= CLP800_SMODE_MAX_REJECTS;
   val  &= (1ul<<CLP800_SMODE_MAX_REJECTS_BITS)-1;

   return sprintf(buf, "%lu\n", (unsigned long)val);
}

static ssize_t
store_max_rej(struct device *dev, struct device_attribute *devattr,
                                  const char *buf, size_t count)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   unsigned val;
   int rc;

   rc = kstrtouint(buf, 0, &val);
   if (rc < 0)
      return rc;

   spin_lock_irq(&priv->cfg_lock);
   rc = elpclp800_set_max_rejects(&priv->elpclp800, val);
   spin_unlock_irq(&priv->cfg_lock);

   if (rc != 0)
      return -ERANGE;

   return count;
}

static ssize_t
show_outlen(struct device *dev, struct device_attribute *devattr, char *buf)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   u32 stat = elpclp800_readreg(&priv->elpclp800, CLP800_STAT);

   if ((stat >> CLP800_STAT_R256) & 1)
      return sprintf(buf, "32\n");
   return sprintf(buf, "16\n");
}

static ssize_t
store_outlen(struct device *dev, struct device_attribute *devattr,
                                 const char *buf, size_t count)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   unsigned val;
   int rc;

   rc = kstrtouint(buf, 0, &val);
   if (rc < 0)
      return rc;

   device_lock(dev);

   spin_lock_irq(&priv->cfg_lock);
   rc = elpclp800_set_output_len(&priv->elpclp800, val);
   spin_unlock_irq(&priv->cfg_lock);

   if (rc < 0) {
      device_unlock(dev);
      return pdu_error_code(rc);
   }

   /* Eat the next random value since its size will be bogus. */
   while (elpclp800_get_random(&priv->elpclp800, NULL) == CRYPTO_INPROGRESS)
      continue;

   device_unlock(dev);

   return count;
}

/*
 * Obtain a single random value from the TRNG.  Returns the number of
 * bytes written, or a negative error code.
 */
static int elpclp800_crank_rng(struct device *dev, unsigned char *out)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   int rc;

   device_lock(dev);
retry:
   rc = elpclp800_get_random(&priv->elpclp800, out);
   switch (rc) {
   case 0:
      elpclp800_writereg(&priv->elpclp800, CLP800_CTRL, CLP800_CMD_GEN_RAND);
   case CRYPTO_INPROGRESS:
      cond_resched();
      goto retry;
   }

   if (rc < 0) {
      device_unlock(dev);
      return pdu_error_code(rc);
   }

   elpclp800_writereg(&priv->elpclp800, CLP800_CTRL, CLP800_CMD_GEN_RAND);
   device_unlock(dev);

   return rc;
}

static ssize_t elpclp800_read(struct device *dev, void *out, size_t outlen)
{
   unsigned char buf[ELPCLP800_MAXLEN];
   size_t ret;
   int rc;

   if (outlen > INT_MAX)
      return -EINVAL;

   for (ret = 0; ret < outlen;) {
      rc = elpclp800_crank_rng(dev, buf);
      if (rc < 0) {
         /* Only return an error if we got no data. */
         if (!ret)
            return rc;
         break;
      }

      if (rc > outlen-ret)
         rc = outlen-ret;

      memcpy(out+ret, buf, rc);
      ret += rc;
   }

   return ret;
}

static ssize_t
show_random(struct device *dev, struct device_attribute *devattr, char *buf)
{
   unsigned char rbuf[ELPCLP800_MAXLEN];
   int i, rlen;

   rlen = elpclp800_crank_rng(dev, rbuf);
   if (rlen < 0)
      return rlen;

   for (i = 0; i < rlen; i++) {
      sprintf(buf+2*i, "%.2hhx\n", rbuf[i]);
   }

   return 2*i+1;
}

static ssize_t
show_rqst(struct device *dev, struct device_attribute *devattr, char *buf)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   unsigned long long val = 0;

   if (priv->has_irq) {
      val  = elpclp800_readreg(&priv->elpclp800, CLP800_AUTO_RQST);
      val &= (1ul << CLP800_AUTO_RQST_RQSTS_BITS) - 1;
      val *= CLP800_AUTO_RQST_RESOLUTION;
   }

   return sprintf(buf, "%llu\n", val);
}

static ssize_t
show_age(struct device *dev, struct device_attribute *devattr, char *buf)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   unsigned long long val = 0;

   /* Alarms only work with IRQs */
   if (priv->has_irq) {
      val  = elpclp800_readreg(&priv->elpclp800, CLP800_AUTO_AGE);
      val &= (1ul << CLP800_AUTO_AGE_AGE_BITS) - 1;
      val *= CLP800_AUTO_AGE_RESOLUTION;
   }

   return sprintf(buf, "%llu\n", val);
}

static ssize_t
store_rqst(struct device *dev, struct device_attribute *devattr,
                               const char *buf, size_t count)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   unsigned long long val;
   int rc;

   rc = kstrtoull(buf, 0, &val);
   if (rc < 0)
      return rc;

   /* Alarms only work with IRQs */
   if (!priv->has_irq)
      return -EOPNOTSUPP;

   rc = elpclp800_set_request_reminder(&priv->elpclp800, val);
   if (rc < 0)
      return pdu_error_code(rc);

   return count;
}

static ssize_t
store_age(struct device *dev, struct device_attribute *devattr,
                              const char *buf, size_t count)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   unsigned long long val;
   int rc;

   rc = kstrtoull(buf, 0, &val);
   if (rc < 0)
      return rc;

   /* Alarms only work with IRQs */
   if (!priv->has_irq)
      return -EOPNOTSUPP;

   rc = elpclp800_set_age_reminder(&priv->elpclp800, val);
   if (rc < 0)
      return pdu_error_code(rc);

   return count;
}

static ssize_t
show_alarm(struct device *dev, struct device_attribute *devattr, char *buf)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   unsigned alarm_reason = 0;

   spin_lock_irq(&priv->cfg_lock);
   if (priv->alarm_generation == priv->seed_generation)
      alarm_reason = priv->alarm_reason;
   spin_unlock_irq(&priv->cfg_lock);

   switch (alarm_reason) {
   case 0:
      return sprintf(buf, "none\n");
   case CLP800_IRQ_AGE_ALARM:
      return sprintf(buf, "age\n");
   case CLP800_IRQ_RQST_ALARM:
      return sprintf(buf, "request\n");
   }

   return sprintf(buf, "unknown\n");
}

static DEVICE_ATTR(reseeding,        0444, show_reseeding,      NULL);
static DEVICE_ATTR(epn,              0444, show_epn,      NULL);
static DEVICE_ATTR(tts,              0444, show_tts,      NULL);
static DEVICE_ATTR(stepping,         0444, show_stepping, NULL);
static DEVICE_ATTR(seed,             0600, show_seed,     store_seed);
static DEVICE_ATTR(seed_generation,  0444, show_gen,      NULL);
static DEVICE_ATTR(reseed_action,    0600, show_reseed,   store_reseed);
static DEVICE_ATTR(mode,             0644, show_mode,     store_mode);
static DEVICE_ATTR(reject_threshold, 0644, show_max_rej,  store_max_rej);
static DEVICE_ATTR(output_len,       0644, show_outlen,   store_outlen);
static DEVICE_ATTR(random,           0444, show_random,   NULL);
static DEVICE_ATTR(wd_requests,      0644, show_rqst,     store_rqst);
static DEVICE_ATTR(wd_cycles,        0644, show_age,      store_age);
static DEVICE_ATTR(wd_alarm,         0444, show_alarm,    NULL);

static const struct attribute_group elpclp800_attr_group = {
   .attrs = (struct attribute *[]) {
      
      &dev_attr_reseeding.attr,
      &dev_attr_epn.attr,
      &dev_attr_tts.attr,
      &dev_attr_stepping.attr,
      &dev_attr_seed.attr,
      &dev_attr_seed_generation.attr,
      &dev_attr_reseed_action.attr,
      &dev_attr_mode.attr,
      &dev_attr_reject_threshold.attr,
      &dev_attr_output_len.attr,
      &dev_attr_random.attr,
      &dev_attr_wd_requests.attr,
      &dev_attr_wd_cycles.attr,
      &dev_attr_wd_alarm.attr,
      NULL
   },
};

static irqreturn_t elpclp800_irq(int irq, void *dev)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);
   u32 istat = elpclp800_readreg(&priv->elpclp800, CLP800_ISTAT);
   irqreturn_t ret = IRQ_NONE;
   u32 ack = 0;

   if (istat & CLP800_IRQ_AGE_ALARM_MASK) {
      spin_lock(&priv->cfg_lock);
      priv->alarm_generation = priv->seed_generation;
      priv->alarm_reason = CLP800_IRQ_AGE_ALARM;
      spin_unlock(&priv->cfg_lock);

      ack |= CLP800_IRQ_AGE_ALARM_MASK;
      schedule_work(&priv->alarm_work);
   }

   if (istat & CLP800_IRQ_RQST_ALARM_MASK) {
      spin_lock(&priv->cfg_lock);
      priv->alarm_generation = priv->seed_generation;
      priv->alarm_reason = CLP800_IRQ_RQST_ALARM;
      spin_unlock(&priv->cfg_lock);

      ack |= CLP800_IRQ_RQST_ALARM_MASK;
      schedule_work(&priv->alarm_work);
   }

   if (istat & CLP800_IRQ_SEED_DONE_MASK) {
      spin_lock(&priv->cfg_lock);
      priv->seed_generation++;
      spin_unlock(&priv->cfg_lock);

      ack |= CLP800_IRQ_SEED_DONE_MASK;
   }

   if (ack) {
      elpclp800_writereg(&priv->elpclp800, CLP800_ISTAT, ack);
      if (ack & CLP800_IRQ_SEED_DONE_MASK)
         wake_up_all(&priv->reseed_wq);

      ret = IRQ_HANDLED;
   }

   return ret;
}

static int
elpclp800_hwrng_read(struct hwrng *rng, void *data, size_t max, bool wait)
{
   struct device *dev = (void *)rng->priv;
   struct elpclp800_priv *priv = dev_get_drvdata(dev);

   /*
    * hwrng_register calls into us from device probe, we can't support
    * that (and RNG needs userspace setup anyway).
    */
   if (!priv->hwrng_setup)
      return -EAGAIN;

   return elpclp800_read(dev, data, max);
}

static int __devinit elpclp800_setup_hwrng(struct device *dev)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);

   priv->hwrng = (struct hwrng) {
      .name = dev_name(dev),
      .read = elpclp800_hwrng_read,
      .priv = (unsigned long)dev,
   };

   if (!IS_ENABLED(CONFIG_HW_RANDOM))
      return 0;

   return hwrng_register(&priv->hwrng);
}

static void __devexit elpclp800_stop_hwrng(struct device *dev)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);

   if (!IS_ENABLED(CONFIG_HW_RANDOM) || !priv->hwrng_setup)
      return;

   hwrng_unregister(&priv->hwrng);
}

static void elpclp800_alarm_notify(struct work_struct *work)
{
   struct elpclp800_priv *priv = container_of(work, struct elpclp800_priv, alarm_work);
   struct device *dev = (void *)priv->hwrng.priv;

   kobject_uevent(&dev->kobj, KOBJ_CHANGE);
}

static void __devinit elpclp800_describe_device(struct device *dev)
{
   struct elpclp800_priv *priv = dev_get_drvdata(dev);

   dev_info(dev, "EPN-%.4hx Random Number Generator (stepping: %hx)\n",
                 priv->elpclp800.epn, priv->elpclp800.stepping);

   if (priv->elpclp800.output_len > 16)
      dev_info(dev, "Supports %u-bit output mode\n", 8u*priv->elpclp800.output_len);
   if (priv->elpclp800.rings_avail)
      dev_info(dev, "Supports true random reseed function\n");
   if (priv->elpclp800.diag_level)
      dev_info(dev, "Supports level %hu diagnostics\n", priv->elpclp800.diag_level);
}

static int __devinit elpclp800_probe(struct platform_device *pdev)
{
   struct resource *mem, *irq;
   struct elpclp800_priv *priv;
   void __iomem *regbase;
   int rc;

   printk("%s\n", __func__);

   mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   if (!mem || resource_size(mem) < CLP800_REG_MAX)
      return -EINVAL;
   irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

   priv = devm_kzalloc(&pdev->dev, sizeof *priv, GFP_KERNEL);
   if (!priv)
      return -ENOMEM;

   priv->has_irq = irq;
   spin_lock_init(&priv->cfg_lock);
   spin_lock_init(&priv->iap_lock);
   INIT_WORK(&priv->alarm_work, elpclp800_alarm_notify);
   init_waitqueue_head(&priv->reseed_wq);
   platform_set_drvdata(pdev, priv);

   regbase = pdu_linux_map_regs(&pdev->dev, mem);
   if (IS_ERR(regbase))
      return PTR_ERR(regbase);
   elpclp800_setup(&priv->elpclp800, regbase);

   if (irq) {
      rc = devm_request_irq(&pdev->dev, irq->start, elpclp800_irq, IRQF_SHARED,
                                        dev_name(&pdev->dev), &pdev->dev);
      if (rc < 0) {
         dev_err(&pdev->dev, "failed to request IRQ %u: %d\n",
                             (unsigned)irq->start, rc);
         priv->has_irq = false;
      }
   }

   rc = sysfs_create_group(&pdev->dev.kobj, &elpclp800_attr_group);
   if (rc < 0)
      return rc;

   rc = elpclp800_create_debug_attrs(&pdev->dev);
   if (rc < 0) {
      sysfs_remove_group(&pdev->dev.kobj, &elpclp800_attr_group);
      return rc;
   }

   /* Notify userspace of new attributes */
   kobject_uevent(&pdev->dev.kobj, KOBJ_CHANGE);

   elpclp800_describe_device(&pdev->dev);

   spin_lock_irq(&priv->cfg_lock);
   elpclp800_set_output_len(&priv->elpclp800, 32);
   if (priv->has_irq) {
      elpclp800_enable_irqs(&priv->elpclp800, CLP800_IRQ_GLBL_EN_MASK
                                       | CLP800_IRQ_RQST_ALARM_MASK
                                       | CLP800_IRQ_AGE_ALARM_MASK
                                       | CLP800_IRQ_SEED_DONE_MASK);
   }
   spin_unlock_irq(&priv->cfg_lock);

   /* Register H/W random interfaces if TRNG functions are available. */
   if (priv->elpclp800.rings_avail) {
      rc = elpclp800_setup_hwrng(&pdev->dev);
      if (rc < 0) {
         dev_err(&pdev->dev, "failed to setup hw_random interfaces\n");
         /* non-fatal */
      } else {
         priv->hwrng_setup = true;
      }
   }

   return 0;
}

static int __devexit elpclp800_remove(struct platform_device *pdev)
{
   struct elpclp800_priv *priv = platform_get_drvdata(pdev);

   elpclp800_stop_hwrng(&pdev->dev);

   spin_lock_irq(&priv->cfg_lock);
   elpclp800_disable_irqs(&priv->elpclp800, -1);
   spin_unlock_irq(&priv->cfg_lock);

   elpclp800_remove_debug_attrs(&pdev->dev);
   sysfs_remove_group(&pdev->dev.kobj, &elpclp800_attr_group);

   return 0;
}

static struct platform_driver elpclp800_driver = {
   .probe  = elpclp800_probe,
   .remove = __devexit_p(elpclp800_remove),

   .driver = {
      .name   = DRVNAME,
      .owner  = THIS_MODULE,
   },
};

static int __init elpclp800_init(void)
{
   return platform_driver_register(&elpclp800_driver);
}
module_init(elpclp800_init);

static void __exit elpclp800_exit(void)
{
   platform_driver_unregister(&elpclp800_driver);
}
module_exit(elpclp800_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
