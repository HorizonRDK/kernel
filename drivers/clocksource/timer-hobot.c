/*
 * Horizon X2 QuadSPI driver.
 *
 * Copyright (C) 2019 Horizon, Inc.
 * Author: Luyang <yang.lu@horizon.ai>
 *
 * This driver configures the 32/64-bit count-up timers as follows:
 * T0: 64bit Timer, used as clocksource
 * T1: 32bit Timer, used as clockevents
 * T2: watchdog Timer(Only for Timer0 controller isn't used in this driver)
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
 * struct x2_timer - This definition defines local timer structure
 *
 * @ce_irq: Interrupt for clockevent
 * @freq:   Timer input clock frequency
 * @iobase: Base address of timer
 * @clk:    Associated clock source
 */
struct x2timer_pdata {
	int ce_irq;
	u32 ref_clk;
	void __iomem *iobase;
	struct clk *clk;
};

struct x2timer_cs {
	struct x2timer_pdata *x2timer;
	struct clocksource cs;
};

#define to_x2timer_cs(x) \
	container_of(x, struct x2timer_cs, cs)

struct x2timer_ce {
	struct x2timer_pdata *x2timer;
	struct clock_event_device ce;
};

#define to_x2timer_ce(x) \
	container_of(x, struct x2timer_ce, ce)

static void __iomem *x2timer_iobase;

#define x2timer_rd(dev, reg)       ioread32((dev)->iobase + (reg))
#define x2timer_wr(dev, reg, val)  iowrite32((val), (dev)->iobase + (reg))

static irqreturn_t x2_timer_ce_interrupt(int irq, void *dev_id)
{
	u32 val;
	struct x2timer_ce *x2_tce = (struct x2timer_ce *)dev_id;
	struct x2timer_pdata *x2timer = x2_tce->x2timer;

	/* Acknowledge the interrupt and call event handler */
	val = x2timer_rd(x2timer, X2_TIMER_TMR_SRCPND_REG);
	x2timer_wr(x2timer, X2_TIMER_TMR_SRCPND_REG, val);
	x2_tce->ce.event_handler(&x2_tce->ce);

	return IRQ_HANDLED;
}

/* set next event */
static int x2_timer_set_ne(unsigned long cycles, struct clock_event_device *evt)
{
	struct x2timer_ce *x2_tce = to_x2timer_ce(evt);
	struct x2timer_pdata *x2timer = x2_tce->x2timer;

	/* disable T1 interrupt and then stop it for config new timer target value */
	x2timer_wr(x2timer, X2_TIMER_TMR_SETMASK_REG, X2_TIMER_T1_INTMASK);
	x2timer_wr(x2timer, X2_TIMER_TMRSTOP_REG, X2_TIMER_T1STOP);

	/* Set new value(cycles) to timer1 */
	x2timer_wr(x2timer, X2_TIMER_TMR1TGT_REG, cycles);

	/* enable T1 interrupt and then start it */
	x2timer_wr(x2timer, X2_TIMER_TMR_UNMASK_REG, X2_TIMER_T1_INTMASK);
	x2timer_wr(x2timer, X2_TIMER_TMRSTART_REG, X2_TIMER_T1START);

	return 0;
}

static int x2_timer_shutdown (struct clock_event_device *evt)
{
	struct x2timer_ce *x2_tce = to_x2timer_ce(evt);
	struct x2timer_pdata *x2timer = x2_tce->x2timer;

	x2timer_wr(x2timer, X2_TIMER_TMR_SETMASK_REG, X2_TIMER_T1_INTMASK);
	x2timer_wr(x2timer, X2_TIMER_TMRSTOP_REG, X2_TIMER_T1STOP);

	return 0;
}

static int x2_timer_set_periodic (struct clock_event_device *evt)
{
	u32 val;
	struct x2timer_ce *x2_tce = to_x2timer_ce(evt);
	struct x2timer_pdata *x2timer = x2_tce->x2timer;

	/* disable T1 interrupt and then stop it for config new timer target value */
	x2timer_wr(x2timer, X2_TIMER_TMR_SETMASK_REG, X2_TIMER_T1_INTMASK);
	x2timer_wr(x2timer, X2_TIMER_TMRSTOP_REG, X2_TIMER_T1STOP);

	/* Set new value(cycles) to timer1 */
	x2timer_wr(x2timer, X2_TIMER_TMR1TGT_REG, DIV_ROUND_CLOSEST (x2timer->ref_clk, HZ));

	/* Set work mode to periodic */
	val = x2timer_rd(x2timer, X2_TIMER_TMRMODE_REG);
	val &= 0xFFFFFF0F;
	val |= (X2_TIMER_PRD_MODE << X2_TIMER_T1MODE_OFFSET);
	x2timer_wr(x2timer, X2_TIMER_TMRMODE_REG, val);

	/* enable T1 interrupt and then start it */
	x2timer_wr(x2timer, X2_TIMER_TMR_UNMASK_REG, X2_TIMER_T1_INTMASK);
	x2timer_wr(x2timer, X2_TIMER_TMRSTART_REG, X2_TIMER_T1START);

	return 0;
}

static int x2_timer_set_oneshot (struct clock_event_device *evt)
{
	u32 val;
	struct x2timer_ce *x2_tce = to_x2timer_ce(evt);
	struct x2timer_pdata *x2timer = x2_tce->x2timer;

	/* disable T1 interrupt and then stop it for config new timer target value */
	x2timer_wr(x2timer, X2_TIMER_TMR_SETMASK_REG, X2_TIMER_T1_INTMASK);
	x2timer_wr(x2timer, X2_TIMER_TMRSTOP_REG, X2_TIMER_T1STOP);

	/* Set new value(cycles) to timer1 */
	x2timer_wr(x2timer, X2_TIMER_TMR1TGT_REG, 0xFFFFFFFFU);

	/* Set work mode to periodic */
	val = x2timer_rd(x2timer, X2_TIMER_TMRMODE_REG);
	val &= 0xFFFFFF0F;
	val |= (X2_TIMER_ONE_MODE << X2_TIMER_T1MODE_OFFSET);
	x2timer_wr(x2timer, X2_TIMER_TMRMODE_REG, val);

	/* enable T1 interrupt and then start it */
	x2timer_wr(x2timer, X2_TIMER_TMR_UNMASK_REG, X2_TIMER_T1_INTMASK);
	x2timer_wr(x2timer, X2_TIMER_TMRSTART_REG, X2_TIMER_T1START);

	return 0;
}

static int __init x2_timer_ce_setup(struct x2timer_pdata *x2timer)
{
	int ret;
	u32 val;
	struct x2timer_ce *x2_tce;

	x2_tce = kzalloc (sizeof (*x2_tce), GFP_KERNEL);
	if (!x2_tce)
		return -ENOMEM;

	x2_tce->x2timer     = x2timer;
	x2_tce->ce.name     = "hobot_clockevent";
	x2_tce->ce.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
	x2_tce->ce.set_next_event     = x2_timer_set_ne;
	x2_tce->ce.set_state_shutdown = x2_timer_shutdown;
	x2_tce->ce.set_state_periodic = x2_timer_set_periodic;
	x2_tce->ce.set_state_oneshot  = x2_timer_set_oneshot;
	x2_tce->ce.tick_resume        = x2_timer_shutdown;
	x2_tce->ce.rating  = 500;
	x2_tce->ce.irq     = x2timer->ce_irq;
	x2_tce->ce.cpumask = cpu_possible_mask;

	/* Default timer work mode is one-time mode */
	val = x2timer_rd(x2timer, X2_TIMER_TMRMODE_REG);
	val &= 0xFFFFFF0F;
	val |= (X2_TIMER_ONE_MODE << X2_TIMER_T1MODE_OFFSET);
	x2timer_wr(x2timer, X2_TIMER_TMRMODE_REG, val);

	/* enable T1 interrupt */
	x2timer_wr(x2timer, X2_TIMER_TMR_UNMASK_REG, X2_TIMER_T1_INTMASK);
	x2timer_wr(x2timer, X2_TIMER_TMR_SRCPND_REG, X2_TIMER_INT_ALLCLR);
	ret = request_irq (x2timer->ce_irq, x2_timer_ce_interrupt, IRQF_TIMER, x2_tce->ce.name, x2_tce);
	if (ret) {
		kfree (x2_tce);
		return ret;
	}

	/* start t1 for clock event */
	/* x2timer_wr(x2timer, X2_TIMER_TMRSTART_REG, X2_TIMER_T1START); */

	clockevents_config_and_register(&x2_tce->ce, x2timer->ref_clk, 1, 0xffffffff);

	return 0;
}

/* clock source read */
static u64 x2_timer_cs_rd(struct clocksource *cs)
{
	u64 count;
	u32 hv, lv;
	struct x2timer_cs *x2_tcs = to_x2timer_cs(cs);
	struct x2timer_pdata *x2timer = x2_tcs->x2timer;

	/* reference RTL designed, you should read upper_reg first
	   (to activer lower_reg self-fresh), and then read lower_reg */
	hv = x2timer_rd(x2timer, X2_TIMER_TMR0DH_REG);
	lv = x2timer_rd(x2timer, X2_TIMER_TMR0DL_REG);
	count = ((u64)hv<<32) | lv;

	return count;
}

/* sched clock read */
static u64 notrace x2_timer_sc_rd(void)
{
	u64 count;
	u32 hv, lv;

	/* reference RTL designed, you should read upper_reg first
	   (to activer lower_reg self-fresh), and then read lower_reg */
	hv = ioread32(x2timer_iobase + X2_TIMER_TMR0DH_REG);
	lv = ioread32(x2timer_iobase + X2_TIMER_TMR0DL_REG);
	count = ((u64)hv<<32) | lv;

	return count;
}

static void x2_timer_cs_resume(struct clocksource *cs)
{
	u32 val;
	struct x2timer_cs *x2_cs = to_x2timer_cs(cs);
	struct x2timer_pdata *x2timer = x2_cs->x2timer;
	/*
	 * set timer0 no interrupt, continuous mode and start it
	 */
	x2timer_wr(x2timer, X2_TIMER_TMR_SETMASK_REG, X2_TIMER_T0_INTMASK);

	val = x2timer_rd(x2timer, X2_TIMER_TMRMODE_REG);
	val &= 0xFFFFFFF0;
	val |= (X2_TIMER_CON_MODE << X2_TIMER_T0MODE_OFFSET);
	x2timer_wr(x2timer, X2_TIMER_TMRMODE_REG, val);

	x2timer_wr(x2timer, X2_TIMER_TMR0TGTL_REG, 0xFFFFFFFFU);
	x2timer_wr(x2timer, X2_TIMER_TMR0TGTH_REG, 0xFFFFFFFFU);
	x2timer_wr(x2timer, X2_TIMER_TMRSTART_REG, X2_TIMER_T0START);
}

static int __init x2_timer_cs_setup(struct x2timer_pdata *x2timer)
{
	int err;
	u32 val;
	struct x2timer_cs *x2_tcs;

	x2_tcs = kzalloc (sizeof (*x2_tcs), GFP_KERNEL);
	if (!x2_tcs)
		return -ENOMEM;

	x2_tcs->x2timer   = x2timer;
	x2_tcs->cs.name   = "hobot_clocksource";
	x2_tcs->cs.rating = 200;
	x2_tcs->cs.read   = x2_timer_cs_rd;
	x2_tcs->cs.mask   = CLOCKSOURCE_MASK (X2_TIMER_T0_WIDTH);
	x2_tcs->cs.flags  = CLOCK_SOURCE_IS_CONTINUOUS;
	x2_tcs->cs.resume = x2_timer_cs_resume;

	/*
	 * set timer0 no interrupt, continuous mode and start it
	 */
	x2timer_wr(x2timer, X2_TIMER_TMR_SETMASK_REG, X2_TIMER_T0_INTMASK);

	val = x2timer_rd(x2timer, X2_TIMER_TMRMODE_REG);
	val &= 0xFFFFFFF0;
	val |= (X2_TIMER_CON_MODE << X2_TIMER_T0MODE_OFFSET);
	x2timer_wr(x2timer, X2_TIMER_TMRMODE_REG, val);

	x2timer_wr(x2timer, X2_TIMER_TMR0TGTL_REG, 0xFFFFFFFFU);
	x2timer_wr(x2timer, X2_TIMER_TMR0TGTH_REG, 0xFFFFFFFFU);
	x2timer_wr(x2timer, X2_TIMER_TMRSTART_REG, X2_TIMER_T0START);

	err = clocksource_register_hz(&x2_tcs->cs, x2timer->ref_clk);
	if (err) {
		kfree (x2_tcs);
		return err;
	}

	if (x2timer_iobase == NULL) {
		x2timer_iobase = x2timer->iobase;
		sched_clock_register(x2_timer_sc_rd,
				X2_TIMER_T0_WIDTH, x2timer->ref_clk);
	}

	return 0;
}

static int __init x2_timer_init (struct device_node *np)
{
	int ret;
	struct x2timer_pdata *x2timer;

	x2timer = kzalloc(sizeof(*x2timer), GFP_KERNEL);
	if (!x2timer)
		return -ENOMEM;

	x2timer->iobase = of_iomap (np, 0);
	if (!x2timer->iobase) {
		pr_err ("ERROR: invalid timer base address\n");
		return -ENXIO;
	}

	/* select timer1 for clock envent  */
	x2timer->ce_irq = irq_of_parse_and_map (np, 1);
	if (x2timer->ce_irq <= 0) {
		pr_err ("ERROR: invalid interrupt number\n");
		return -EINVAL;
	}

	/* get the module ref-clk and enabel it */
	x2timer->clk = of_clk_get(np, 0);
	if (IS_ERR(x2timer->clk)) {
		pr_err("uart_clk clock not found.\n");
		return PTR_ERR(x2timer->clk);
	}
	ret = clk_prepare_enable(x2timer->clk);
	if (ret) {
		pr_err("Unable to enable device clock.\n");
		goto probe_clk_failed;
	}


	x2timer->ref_clk = clk_get_rate(x2timer->clk);

	ret = x2_timer_cs_setup(x2timer);
	if (ret)
		goto probe_clk_failed;

	ret = x2_timer_ce_setup(x2timer);
	if (ret)
		goto probe_clk_failed;

	return ret;

probe_clk_failed:
	clk_disable_unprepare(x2timer->clk);
	return ret;
}

TIMER_OF_DECLARE(x2timer, "hobot,x2-timer", x2_timer_init);
