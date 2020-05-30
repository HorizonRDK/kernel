/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/sched_clock.h>
#include <soc/hobot/hobot_timer.h>

/**
 * struct hobot_timer - This definition defines local timer structure
 *
 * @ce_irq: Interrupt for clockevent
 * @freq:   Timer input clock frequency
 * @iobase: Base address of timer
 * @clk:    Associated clock source
 */
struct hbtimer_pdata {
	int ce_irq;
	u32 ref_clk;
	void __iomem *iobase;
	struct clk *clk;
};

struct hbtimer_cs {
	struct hbtimer_pdata *hbtimer;
	struct clocksource cs;
};

#define to_hbtimer_cs(x) \
	container_of(x, struct hbtimer_cs, cs)

struct hbtimer_ce {
	struct hbtimer_pdata *hbtimer;
	struct clock_event_device ce;
};

#define to_hbtimer_ce(x) \
	container_of(x, struct hbtimer_ce, ce)

static void __iomem *hbtimer_iobase;

#define hbtimer_rd(dev, reg)       ioread32((dev)->iobase + (reg))
#define hbtimer_wr(dev, reg, val)  iowrite32((val), (dev)->iobase + (reg))

static irqreturn_t hobot_timer_ce_interrupt(int irq, void *dev_id)
{
	u32 val;
	struct hbtimer_ce *hobot_tce = (struct hbtimer_ce *)dev_id;
	struct hbtimer_pdata *hbtimer = hobot_tce->hbtimer;

	/* Acknowledge the interrupt and call event handler */
	val = hbtimer_rd(hbtimer, HOBOT_TIMER_TMR_SRCPND_REG);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR_SRCPND_REG, val);
	hobot_tce->ce.event_handler(&hobot_tce->ce);

	return IRQ_HANDLED;
}

/* set next event */
static int hobot_timer_set_ne(unsigned long cycles, struct clock_event_device *evt)
{
	struct hbtimer_ce *hobot_tce = to_hbtimer_ce(evt);
	struct hbtimer_pdata *hbtimer = hobot_tce->hbtimer;

	/* disable T1 interrupt and then stop it for config new timer target value */
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR_SETMASK_REG, HOBOT_TIMER_T1_INTMASK);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMRSTOP_REG, HOBOT_TIMER_T1STOP);

	/* Set new value(cycles) to timer1 */
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR1TGT_REG, cycles);

	/* enable T1 interrupt and then start it */
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR_UNMASK_REG, HOBOT_TIMER_T1_INTMASK);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMRSTART_REG, HOBOT_TIMER_T1START);

	return 0;
}

static int hobot_timer_shutdown (struct clock_event_device *evt)
{
	struct hbtimer_ce *hobot_tce = to_hbtimer_ce(evt);
	struct hbtimer_pdata *hbtimer = hobot_tce->hbtimer;

	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR_SETMASK_REG, HOBOT_TIMER_T1_INTMASK);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMRSTOP_REG, HOBOT_TIMER_T1STOP);

	return 0;
}

static int hobot_timer_set_periodic (struct clock_event_device *evt)
{
	u32 val;
	struct hbtimer_ce *hobot_tce = to_hbtimer_ce(evt);
	struct hbtimer_pdata *hbtimer = hobot_tce->hbtimer;

	/* disable T1 interrupt and then stop it for config new timer target value */
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR_SETMASK_REG, HOBOT_TIMER_T1_INTMASK);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMRSTOP_REG, HOBOT_TIMER_T1STOP);

	/* Set new value(cycles) to timer1 */
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR1TGT_REG, DIV_ROUND_CLOSEST (hbtimer->ref_clk, HZ));

	/* Set work mode to periodic */
	val = hbtimer_rd(hbtimer, HOBOT_TIMER_TMRMODE_REG);
	val &= 0xFFFFFF0F;
	val |= (HOBOT_TIMER_PRD_MODE << HOBOT_TIMER_T1MODE_OFFSET);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMRMODE_REG, val);

	/* enable T1 interrupt and then start it */
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR_UNMASK_REG, HOBOT_TIMER_T1_INTMASK);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMRSTART_REG, HOBOT_TIMER_T1START);

	return 0;
}

static int hobot_timer_set_oneshot (struct clock_event_device *evt)
{
	u32 val;
	struct hbtimer_ce *hobot_tce = to_hbtimer_ce(evt);
	struct hbtimer_pdata *hbtimer = hobot_tce->hbtimer;

	/* disable T1 interrupt and then stop it for config new timer target value */
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR_SETMASK_REG, HOBOT_TIMER_T1_INTMASK);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMRSTOP_REG, HOBOT_TIMER_T1STOP);

	/* Set new value(cycles) to timer1 */
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR1TGT_REG, 0xFFFFFFFFU);

	/* Set work mode to periodic */
	val = hbtimer_rd(hbtimer, HOBOT_TIMER_TMRMODE_REG);
	val &= 0xFFFFFF0F;
	val |= (HOBOT_TIMER_ONE_MODE << HOBOT_TIMER_T1MODE_OFFSET);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMRMODE_REG, val);

	/* enable T1 interrupt and then start it */
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR_UNMASK_REG, HOBOT_TIMER_T1_INTMASK);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMRSTART_REG, HOBOT_TIMER_T1START);

	return 0;
}

static int __init hobot_timer_ce_setup(struct hbtimer_pdata *hbtimer)
{
	int ret;
	u32 val;
	struct hbtimer_ce *hobot_tce;

	hobot_tce = kzalloc (sizeof (*hobot_tce), GFP_KERNEL);
	if (!hobot_tce)
		return -ENOMEM;

	hobot_tce->hbtimer     = hbtimer;
	hobot_tce->ce.name     = "hobot_clockevent";
	hobot_tce->ce.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
	hobot_tce->ce.set_next_event     = hobot_timer_set_ne;
	hobot_tce->ce.set_state_shutdown = hobot_timer_shutdown;
	hobot_tce->ce.set_state_periodic = hobot_timer_set_periodic;
	hobot_tce->ce.set_state_oneshot  = hobot_timer_set_oneshot;
	hobot_tce->ce.tick_resume        = hobot_timer_shutdown;
	hobot_tce->ce.rating  = 500;
	hobot_tce->ce.irq     = hbtimer->ce_irq;
	hobot_tce->ce.cpumask = cpu_possible_mask;

	/* Default timer work mode is one-time mode */
	val = hbtimer_rd(hbtimer, HOBOT_TIMER_TMRMODE_REG);
	val &= 0xFFFFFF0F;
	val |= (HOBOT_TIMER_ONE_MODE << HOBOT_TIMER_T1MODE_OFFSET);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMRMODE_REG, val);

	/* enable T1 interrupt */
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR_UNMASK_REG, HOBOT_TIMER_T1_INTMASK);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR_SRCPND_REG, HOBOT_TIMER_INT_ALLCLR);
	ret = request_irq (hbtimer->ce_irq, hobot_timer_ce_interrupt, IRQF_TIMER, hobot_tce->ce.name, hobot_tce);
	if (ret) {
		kfree (hobot_tce);
		return ret;
	}

	/* start t1 for clock event */
	/* hbtimer_wr(hbtimer, HOBOT_TIMER_TMRSTART_REG, HOBOT_TIMER_T1START); */

	clockevents_config_and_register(&hobot_tce->ce, hbtimer->ref_clk, 1, 0xffffffff);

	return 0;
}

/* clock source read */
static u64 hobot_timer_cs_rd(struct clocksource *cs)
{
	u64 count;
	u32 hv, lv;
	struct hbtimer_cs *hobot_tcs = to_hbtimer_cs(cs);
	struct hbtimer_pdata *hbtimer = hobot_tcs->hbtimer;

	/* reference RTL designed, you should read upper_reg first
	   (to activer lower_reg self-fresh), and then read lower_reg */
	hv = hbtimer_rd(hbtimer, HOBOT_TIMER_TMR0DH_REG);
	lv = hbtimer_rd(hbtimer, HOBOT_TIMER_TMR0DL_REG);
	count = ((u64)hv<<32) | lv;

	return count;
}

/* sched clock read */
static u64 notrace hobot_timer_sc_rd(void)
{
	u64 count;
	u32 hv, lv;

	/* reference RTL designed, you should read upper_reg first
	   (to activer lower_reg self-fresh), and then read lower_reg */
	hv = ioread32(hbtimer_iobase + HOBOT_TIMER_TMR0DH_REG);
	lv = ioread32(hbtimer_iobase + HOBOT_TIMER_TMR0DL_REG);
	count = ((u64)hv<<32) | lv;

	return count;
}

static void hobot_timer_cs_resume(struct clocksource *cs)
{
	u32 val;
	struct hbtimer_cs *hobot_cs = to_hbtimer_cs(cs);
	struct hbtimer_pdata *hbtimer = hobot_cs->hbtimer;
	/*
	 * set timer0 no interrupt, continuous mode and start it
	 */
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR_SETMASK_REG, HOBOT_TIMER_T0_INTMASK);

	val = hbtimer_rd(hbtimer, HOBOT_TIMER_TMRMODE_REG);
	val &= 0xFFFFFFF0;
	val |= (HOBOT_TIMER_CON_MODE << HOBOT_TIMER_T0MODE_OFFSET);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMRMODE_REG, val);

	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR0TGTL_REG, 0xFFFFFFFFU);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR0TGTH_REG, 0xFFFFFFFFU);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMRSTART_REG, HOBOT_TIMER_T0START);
}

static int __init hobot_timer_cs_setup(struct hbtimer_pdata *hbtimer)
{
	int err;
	u32 val;
	struct hbtimer_cs *hobot_tcs;

	hobot_tcs = kzalloc (sizeof (*hobot_tcs), GFP_KERNEL);
	if (!hobot_tcs)
		return -ENOMEM;

	hobot_tcs->hbtimer   = hbtimer;
	hobot_tcs->cs.name   = "hobot_clocksource";
	hobot_tcs->cs.rating = 200;
	hobot_tcs->cs.read   = hobot_timer_cs_rd;
	hobot_tcs->cs.mask   = CLOCKSOURCE_MASK (HOBOT_TIMER_T0_WIDTH);
	hobot_tcs->cs.flags  = CLOCK_SOURCE_IS_CONTINUOUS;
	hobot_tcs->cs.resume = hobot_timer_cs_resume;

	/*
	 * set timer0 no interrupt, continuous mode and start it
	 */
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR_SETMASK_REG, HOBOT_TIMER_T0_INTMASK);

	val = hbtimer_rd(hbtimer, HOBOT_TIMER_TMRMODE_REG);
	val &= 0xFFFFFFF0;
	val |= (HOBOT_TIMER_CON_MODE << HOBOT_TIMER_T0MODE_OFFSET);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMRMODE_REG, val);

	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR0TGTL_REG, 0xFFFFFFFFU);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMR0TGTH_REG, 0xFFFFFFFFU);
	hbtimer_wr(hbtimer, HOBOT_TIMER_TMRSTART_REG, HOBOT_TIMER_T0START);

	err = clocksource_register_hz(&hobot_tcs->cs, hbtimer->ref_clk);
	if (err) {
		kfree (hobot_tcs);
		return err;
	}

	if (hbtimer_iobase == NULL) {
		hbtimer_iobase = hbtimer->iobase;
		sched_clock_register(hobot_timer_sc_rd,
				HOBOT_TIMER_T0_WIDTH, hbtimer->ref_clk);
	}

	return 0;
}

static int __init hobot_timer_init (struct device_node *np)
{
	int ret;
	struct hbtimer_pdata *hbtimer;

	hbtimer = kzalloc(sizeof(*hbtimer), GFP_KERNEL);
	if (!hbtimer)
		return -ENOMEM;

	hbtimer->iobase = of_iomap (np, 0);
	if (!hbtimer->iobase) {
		pr_err ("ERROR: invalid timer base address\n");
		return -ENXIO;
	}

	/* select timer1 for clock envent  */
	hbtimer->ce_irq = irq_of_parse_and_map (np, 1);
	if (hbtimer->ce_irq <= 0) {
		pr_err ("ERROR: invalid interrupt number\n");
		return -EINVAL;
	}

	/* get the module ref-clk and enabel it */
	hbtimer->clk = of_clk_get(np, 0);
	if (IS_ERR(hbtimer->clk)) {
		pr_err("uart_clk clock not found.\n");
		return PTR_ERR(hbtimer->clk);
	}
	ret = clk_prepare_enable(hbtimer->clk);
	if (ret) {
		pr_err("Unable to enable device clock.\n");
		goto probe_clk_failed;
	}


	hbtimer->ref_clk = clk_get_rate(hbtimer->clk);

	ret = hobot_timer_cs_setup(hbtimer);
	if (ret)
		goto probe_clk_failed;

	ret = hobot_timer_ce_setup(hbtimer);
	if (ret)
		goto probe_clk_failed;

	return ret;

probe_clk_failed:
	clk_disable_unprepare(hbtimer->clk);
	return ret;
}

TIMER_OF_DECLARE(hbtimer, "hobot,hobot-timer", hobot_timer_init);
