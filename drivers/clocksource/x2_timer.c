/*
 * This file contains driver for the X2 Timer
 *
 * Copyright (C) 2018 Horizon
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/sched_clock.h>

/*
 * This driver configures the 32/64-bit count-up timers as follows:
 *
 * T0: Timer 0, 64bit Timer, used as clocksource
 * T1: Timer 1, 32bit Timer, used as clockevents
 * T2: Timer 2, watchdog Timer(Not used in this driver)
 */

#define X2_TIMER_TMREN						(0x0)
#define X2_TIMER_TMRSTART					(0x4)
#define X2_TIMER_TMRSTOP					(0x8)
#define X2_TIMER_TMRMODE					(0xC)
#define X2_TIMER_TMR0TGTL					(0x10)
#define X2_TIMER_TMR0TGTH					(0x14)
#define X2_TIMER_TMR0DL						(0x18)
#define X2_TIMER_TMR0DH						(0x1C)
#define X2_TIMER_TMR1TGT					(0x20)
#define X2_TIMER_TMR1D						(0x24)
#define X2_TIMER_WDTGT						(0x28)
#define X2_TIMER_WDWAIT						(0x2C)
#define X2_TIMER_WD1D						(0x30)
#define X2_TIMER_WD2D						(0x34)
#define X2_TIMER_WDCLR						(0x38)
#define X2_TIMER_TMR_SRCPND					(0x3C)
#define X2_TIMER_TMR_INTMASK				(0x40)
#define X2_TIMER_TMR_SETMASK				(0x44)
#define X2_TIMER_TMR_UNMASK					(0x48)

/*************************************************************
 * register bit
 *************************************************************/

/* X2_TIMER_TMREN */
#define X2_TIMER_TMR2_ENABLE_RO				(0x1 << 0x2)
#define X2_TIMER_TMR2_ENABLE_RO_SHIT(n)		(((n) & 0x1) >> 0x2)
#define X2_TIMER_TMR1_ENABLE_RO				(0x1 << 0x1)
#define X2_TIMER_TMR1_ENABLE_RO_SHIT(n)		(((n) & 0x1) >> 0x1)
#define X2_TIMER_TMR0_ENABLE_RO				(0x1 << 0x0)
#define X2_TIMER_TMR0_ENABLE_RO_SHIT(n)		(((n) & 0x1) >> 0x0)

/* X2_TIMER_TMRSTART */
#define X2_TIMER_TMR2START_WO(n)			(((n) & 0x1) << 0x2)
#define X2_TIMER_TMR1START_WO(n)			(((n) & 0x1) << 0x1)
#define X2_TIMER_TMR0START_WO(n)			(((n) & 0x1) << 0x0)

/* X2_TIMER_TMRSTOP */
#define X2_TIMER_TMR2STOP_WO(n)				(((n) & 0x1) << 0x2)
#define X2_TIMER_TMR1STOP_WO(n)				(((n) & 0x1) << 0x1)
#define X2_TIMER_TMR0STOP_WO(n)				(((n) & 0x1) << 0x0)

/* X2_TIMER_TMRMODE */
#define X2_TIMER_TMR2_MODE(n)				(((n) & 0xf) << 0x8)
#define X2_TIMER_TMR2_MODE_MASK				(0xf << 0x8)
#define X2_TIMER_TMR2_MODE_SHIT(n)			(((n) & 0xf) >> 0x8)
#define X2_TIMER_TMR1_MODE(n)				(((n) & 0xf) << 0x4)
#define X2_TIMER_TMR1_MODE_MASK				(0xf << 0x4)
#define X2_TIMER_TMR1_MODE_SHIT(n)			(((n) & 0xf) >> 0x4)
#define X2_TIMER_TMR0_MODE(n)				(((n) & 0xf) << 0x0)
#define X2_TIMER_TMR0_MODE_MASK				(0xf << 0x0)
#define X2_TIMER_TMR0_MODE_SHIT(n)			(((n) & 0xf) >> 0x0)

/* X2_TIMER_TMR0TGTL */
#define X2_TIMER_TMR0TGTL_RW(n)				(((n) & 0xffffffff) << 0x0)
#define X2_TIMER_TMR0TGTL_MASK				(0xffffffff << 0x0)
#define X2_TIMER_TMR0TGTL_SHIT(n) 			(((n) & 0xffffffff) >> 0x0)

/* X2_TIMER_TMR0TGTH */
#define X2_TIMER_TMR0TGTH_RW(n)				(((n) & 0xffffffff) << 0x0)
#define X2_TIMER_TMR0TGTH_MASK				(0xffffffff << 0x0)
#define X2_TIMER_TMR0TGTH_SHIT(n) 			(((n) & 0xffffffff) >> 0x0)

/* X2_TIMER_TMR0DL */
#define X2_TIMER_TMR0DL_RO					(0xffffffff << 0x0)
#define X2_TIMER_TMR0DL_RO_SHIT(n)			(((n) & 0xffffffff) >> 0x0)

/* X2_TIMER_TMR0DH */
#define X2_TIMER_TMR0DH_RO					(0xffffffff << 0x0)
#define X2_TIMER_TMR0DH_RO_SHIT(n)			(((n) & 0xffffffff) >> 0x0)

/* X2_TIMER_TMR1TGT */
#define X2_TIMER_TMR1TGT_RW(n)				(((n) & 0xffffffff) << 0x0)
#define X2_TIMER_TMR1TGT_MASK				(0xffffffff << 0x0)
#define X2_TIMER_TMR1TGT_SHIT(n)			(((n) & 0xffffffff) >> 0x0)

/* X2_TIMER_TMR1D */
#define X2_TIMER_TMR1D_RO 					(0xffffffff << 0x0)
#define X2_TIMER_TMR1D_RO_SHIT(n) 			(((n) & 0xffffffff) >> 0x0)

/* X2_TIMER_WDTGT */
#define X2_TIMER_WD1TGT(n)					(((n) & 0xffffffff) << 0x0)
#define X2_TIMER_WD1TGT_MASK				(0xffffffff << 0x0)
#define X2_TIMER_WD1TGT_SHIT(n)				(((n) & 0xffffffff) >> 0x0)

/* X2_TIMER_WDWAIT */
#define X2_TIMER_WD2TGT(n)					(((n) & 0xffffffff) << 0x0)
#define X2_TIMER_WD2TGT_MASK				(0xffffffff << 0x0)
#define X2_TIMER_WD2TGT_SHIT(n)				(((n) & 0xffffffff) >> 0x0)

/* X2_TIMER_WD1D */
#define X2_TIMER_WD1D_RO					(0xffffffff << 0x0)
#define X2_TIMER_WD1D_RO_SHIT(n)			(((n) & 0xffffffff) >> 0x0)

/*	  X2_TIMER_WD2D    */
#define X2_TIMER_WD2D_RO					(0xffffffff << 0x0)
#define X2_TIMER_WD2D_RO_SHIT(n)			(((n) & 0xffffffff) >> 0x0)

/*	  X2_TIMER_WDCLR	*/
#define X2_TIMER_WDOGCLR_WO(n)				(((n) & 0x1) << 0x0)

/*	  X2_TIMER_TMR_SRCPND	 */
#define X2_TIMER_TMR2SRCPND_W1C				(0x1 << 0x2)
#define X2_TIMER_TMR1SRCPND_W1C				(0x1 << 0x1)
#define X2_TIMER_TMR0SRCPND_W1C				(0x1 << 0x0)

/*	  X2_TIMER_TMR_INTMASK	  */
#define X2_TIMER_TMR_INTMASK2_RO			(0x1 << 0x2)
#define X2_TIMER_TMR_INTMASK2_RO_SHIT(n)	(((n) & 0x1) >> 0x2)
#define X2_TIMER_TMR_INTMASK1_RO			(0x1 << 0x1)
#define X2_TIMER_TMR_INTMASK1_RO_SHIT(n)	(((n) & 0x1) >> 0x1)
#define X2_TIMER_TMR_INTMASK0_RO			(0x1 << 0x0)
#define X2_TIMER_TMR_INTMASK0_RO_SHIT(n)	(((n) & 0x1) >> 0x0)

/*	  X2_TIMER_TMR_SETMASK	  */
#define X2_TIMER_TMR_SETMASK2_WO(n)			(((n) & 0x1) << 0x2)
#define X2_TIMER_TMR_SETMASK1_WO(n)			(((n) & 0x1) << 0x1)
#define X2_TIMER_TMR_SETMASK0_WO(n)			(((n) & 0x1) << 0x0)

/* X2_TIMER_TMR_UNMASK */
#define X2_TIMER_TMR_UNMASK2_WO(n)			(((n) & 0x1) << 0x2)
#define X2_TIMER_TMR_UNMASK1_WO(n)			(((n) & 0x1) << 0x1)
#define X2_TIMER_TMR_UNMASK0_WO(n)			(((n) & 0x1) << 0x0)

#define one_time_mode						0x0
#define periodic_mode						0x1
#define continuous_mode						0x2


/**
 * struct x2_timer - This definition defines local timer structure
 *
 * @base_addr:	Base address of timer
 * @freq:	Timer input clock frequency
 * @clk:	Associated clock source
 * @clk_rate_change_nb	Notifier block for clock rate changes
 */
struct x2_timer {
	void __iomem *base_addr;
	unsigned long freq;
#if 0	//No support at FPGA version
	struct clk *clk;
	struct notifier_block clk_rate_change_nb;
#endif
};

#define to_x2_timer(x) \
	container_of(x, struct x2_timer, clk_rate_change_nb)

struct x2_timer_clocksource {
	struct x2_timer	x2_cpu_timer;
	struct clocksource	cs;
};

#define to_x2_timer_clksrc(x) \
	container_of(x, struct x2_timer_clocksource, cs)

struct x2_timer_clockevent {
	struct x2_timer		x2_cpu_timer;
	struct clock_event_device	ce;
};

#define to_x2_timer_clkevent(x) \
		container_of(x, struct x2_timer_clockevent, ce)

static void __iomem *x2_timer_base;
/*
 * To get the value from the 64bit Timer Counter register proceed as follows:
 * 1. Read the upper 32-bit timer counter register
 * 2. Read the lower 32-bit timer counter register
 * 3. Read the upper 32-bit timer counter register again. If the value is
 *	different to the 32-bit upper value read previously, go back to step 2.
 *	Otherwise the 64-bit timer counter value is correct.
 */
static u64 notrace x2_timer_counter_read (void)
{
	u64 counter;
	u32 lower;
	u32 upper, old_upper;

	upper = readl_relaxed (x2_timer_base + X2_TIMER_TMR0DH);
	do {
		old_upper = upper;
		lower = readl_relaxed (x2_timer_base + X2_TIMER_TMR0DL);
		upper = readl_relaxed (x2_timer_base + X2_TIMER_TMR0DH);
	} while (upper != old_upper);

	counter = upper;
	counter <<= 32;
	counter |= lower;
	return counter;
}

/**
 * x2_clock_event_interrupt - Clock event timer interrupt handler
 *
 * @irq:	IRQ number of the Timer
 * @dev_id:	void pointer to the x2_timer instance
 *
 * returns: Always IRQ_HANDLED - success
 **/
static irqreturn_t x2_clock_event_interrupt (int irq, void *dev_id)
{
	struct x2_timer_clockevent *x2_tce = dev_id;
	struct x2_timer *timer = &x2_tce->x2_cpu_timer;
	uint32_t isr;

	isr = readl_relaxed (timer->base_addr + X2_TIMER_TMR_SRCPND);
	writel_relaxed (isr, timer->base_addr + X2_TIMER_TMR_SRCPND);

	/* Acknowledge the interrupt and call event handler */

	x2_tce->ce.event_handler (&x2_tce->ce);

	return IRQ_HANDLED;
}

/**
 * x2_clocksource_read - Reads the timer counter register
 *
 * returns: Current timer counter register value
 **/
static u64 x2_clocksource_read (struct clocksource *cs)
{
	return (u64) x2_timer_counter_read();
}

static u64 notrace x2_sched_clock_read (void)
{
	return x2_timer_counter_read();
}


static int x2_set_next_event (unsigned long cycles, struct clock_event_device *evt)
{
	struct x2_timer_clockevent *x2_tce = to_x2_timer_clkevent (evt);
	struct x2_timer *timer = &x2_tce->x2_cpu_timer;
	uint32_t tmr_mode;

	/* mask timer1 for config new timer target value */
	writel_relaxed (X2_TIMER_TMR_SETMASK1_WO (1),
	                timer->base_addr + X2_TIMER_TMR_SETMASK);

	/* Stop timer1 */
	writel_relaxed (X2_TIMER_TMR1STOP_WO (1),
	                timer->base_addr + X2_TIMER_TMRSTOP);

	/* Set new value(cycles) to timer1 */
	writel_relaxed (X2_TIMER_TMR1TGT_RW (cycles),
	                timer->base_addr + X2_TIMER_TMR1TGT);

	/* Set timer1 work mode to one time mode */
	tmr_mode = readl_relaxed (timer->base_addr + X2_TIMER_TMRMODE);
	tmr_mode |= X2_TIMER_TMR1_MODE (periodic_mode);
	writel_relaxed (tmr_mode, timer->base_addr + X2_TIMER_TMRMODE);

	/* Unmask timer1 interrupt */
	writel_relaxed (X2_TIMER_TMR_UNMASK1_WO (1),
	                timer->base_addr + X2_TIMER_TMR_UNMASK);

	/* Start timer1 */
	writel_relaxed (X2_TIMER_TMR1START_WO (1),
	                timer->base_addr + X2_TIMER_TMRSTART);

	return 0;
}

static int x2_shutdown (struct clock_event_device *evt)
{
	struct x2_timer_clockevent *x2_tce = to_x2_timer_clkevent (evt);
	struct x2_timer *timer = &x2_tce->x2_cpu_timer;

	writel_relaxed (X2_TIMER_TMR_SETMASK1_WO (1),
	                timer->base_addr + X2_TIMER_TMR_SETMASK);

	writel_relaxed (X2_TIMER_TMR1STOP_WO (1),
	                timer->base_addr + X2_TIMER_TMRSTOP);

	return 0;
}

static int x2_resume (struct clock_event_device *evt)
{
	struct x2_timer_clockevent *x2_tce =
	    to_x2_timer_clkevent (evt);
	struct x2_timer *timer = &x2_tce->x2_cpu_timer;

	writel_relaxed (X2_TIMER_TMR_UNMASK1_WO (1),
	                timer->base_addr + X2_TIMER_TMR_UNMASK);

	writel_relaxed (X2_TIMER_TMR1START_WO (1),
	                timer->base_addr + X2_TIMER_TMRSTOP);

	return 0;
}

static int x2_set_periodic (struct clock_event_device *evt)
{
	struct x2_timer_clockevent *x2_tce = to_x2_timer_clkevent (evt);
	struct x2_timer *timer = &x2_tce->x2_cpu_timer;
	uint32_t tmr_mode;

	/* mask timer1 for config new timer target value */
	writel_relaxed (X2_TIMER_TMR_SETMASK1_WO (1),
	                timer->base_addr + X2_TIMER_TMR_SETMASK);

	/* Stop timer1 */
	writel_relaxed (X2_TIMER_TMR1STOP_WO (1),
	                timer->base_addr + X2_TIMER_TMRSTOP);

	/* Set new value to timer1 */
	writel_relaxed (X2_TIMER_TMR1TGT_RW (DIV_ROUND_CLOSEST (timer->freq, HZ)),
	                timer->base_addr + X2_TIMER_TMR1TGT);

	/* Set timer1 work mode to periodic mode */
	tmr_mode = readl_relaxed (timer->base_addr + X2_TIMER_TMRMODE);
	tmr_mode |= X2_TIMER_TMR1_MODE (periodic_mode);
	writel_relaxed (tmr_mode, timer->base_addr + X2_TIMER_TMRMODE);

	/* Unmask timer1 interrupt */
	writel_relaxed (X2_TIMER_TMR_UNMASK1_WO (1),
	                timer->base_addr + X2_TIMER_TMR_UNMASK);

	/* Start timer1 */
	writel_relaxed (X2_TIMER_TMR1START_WO (1),
	                timer->base_addr + X2_TIMER_TMRSTART);

	return 0;
}

static int x2_timer_set_oneshot (struct clock_event_device *evt)
{
	struct x2_timer_clockevent *x2_tce = to_x2_timer_clkevent (evt);
	struct x2_timer *timer = &x2_tce->x2_cpu_timer;
	uint32_t tmr_mode;

	/* mask timer1 for config new timer target value */
	writel_relaxed (X2_TIMER_TMR_SETMASK1_WO (1),
	                timer->base_addr + X2_TIMER_TMR_SETMASK);

	/* Stop timer1 */
	writel_relaxed (X2_TIMER_TMR1STOP_WO (1),
	                timer->base_addr + X2_TIMER_TMRSTOP);

	/* Set new value to timer1 */
	writel_relaxed (0xFFFFFFFFU, timer->base_addr + X2_TIMER_TMR1TGT);

	/* Set timer1 work mode to one time mode */
	tmr_mode = readl_relaxed (timer->base_addr + X2_TIMER_TMRMODE);
	tmr_mode |= X2_TIMER_TMR1_MODE (one_time_mode);
	writel_relaxed (tmr_mode, timer->base_addr + X2_TIMER_TMRMODE);

	/* Unmask timer1 interrupt */
	writel_relaxed (X2_TIMER_TMR_UNMASK1_WO (1),
	                timer->base_addr + X2_TIMER_TMR_UNMASK);

	/* Start timer1 */
	writel_relaxed (X2_TIMER_TMR1START_WO (1),
	                timer->base_addr + X2_TIMER_TMRSTART);

	return 0;
}

static int __init x2_setup_clockevent (u32 freq,
                                       void __iomem *base, u32 irq)
{
	struct x2_timer_clockevent *x2_tce;
	int err;
	u32 timer_intmask, timer_mode;

	x2_tce = kzalloc (sizeof (*x2_tce), GFP_KERNEL);
	if (!x2_tce)
		return -ENOMEM;

	x2_tce->x2_cpu_timer.freq = freq;

	x2_tce->x2_cpu_timer.base_addr = base;
	x2_tce->ce.name = "x2_clockevent";
	x2_tce->ce.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
	x2_tce->ce.set_next_event = x2_set_next_event;
	x2_tce->ce.set_state_shutdown = x2_shutdown;
	x2_tce->ce.set_state_periodic = x2_set_periodic;
	x2_tce->ce.set_state_oneshot = x2_timer_set_oneshot;
	x2_tce->ce.tick_resume = x2_resume;
	x2_tce->ce.rating = 500;
	x2_tce->ce.irq = irq;
	x2_tce->ce.cpumask = cpu_possible_mask;

	/*
	 * Unmask timer1 interrupt
	 * Default timer work mode is one time mode
	 */
	timer_intmask = readl_relaxed (base + X2_TIMER_TMR_INTMASK);
	if (timer_intmask & X2_TIMER_TMR_INTMASK1_RO)
		writel_relaxed (X2_TIMER_TMR_UNMASK1_WO (1), base + X2_TIMER_TMR_UNMASK);

	timer_mode = readl_relaxed (base + X2_TIMER_TMRMODE);
	timer_mode |= X2_TIMER_TMR1_MODE (periodic_mode);
	writel_relaxed (timer_mode, base + X2_TIMER_TMRMODE);

	writel_relaxed (0x7, base + X2_TIMER_TMR_SRCPND);

	err = request_irq (irq, x2_clock_event_interrupt,
	                   IRQF_TIMER, x2_tce->ce.name, x2_tce);
	if (err) {
		kfree (x2_tce);
		return err;
	}

	writel_relaxed (X2_TIMER_TMR1START_WO (1), base + X2_TIMER_TMRSTART);

	clockevents_config_and_register (&x2_tce->ce,
	                                 x2_tce->x2_cpu_timer.freq, 1, 0xffffffff);

	return 0;
}

static int __init x2_setup_clocksource (u32 freq, void __iomem *base, u32 timer_width)
{
	struct x2_timer_clocksource *x2_tcs;
	int err;
	u32 timer_mode;

	x2_tcs = kzalloc (sizeof (*x2_tcs), GFP_KERNEL);
	if (!x2_tcs)
		return -ENOMEM;

	x2_tcs->x2_cpu_timer.freq = freq;

	x2_tcs->x2_cpu_timer.base_addr = base;
	x2_tcs->cs.name = "x2_clocksource";
	x2_tcs->cs.rating = 500;
	x2_tcs->cs.read = x2_clocksource_read;
	x2_tcs->cs.mask = CLOCKSOURCE_MASK (timer_width);
	x2_tcs->cs.flags = CLOCK_SOURCE_IS_CONTINUOUS;

	/*
	 * set timer0 no interrupt, continuous mode and start it
	 */
	writel_relaxed (X2_TIMER_TMR_SETMASK0_WO (1), base + X2_TIMER_TMR_SETMASK);

	timer_mode = readl_relaxed (base + X2_TIMER_TMRMODE);
	timer_mode |= X2_TIMER_TMR0_MODE (continuous_mode);
	writel_relaxed (timer_mode, base + X2_TIMER_TMRMODE);

	writel_relaxed (0xFFFFFFFFU, base + X2_TIMER_TMR0TGTL);
	writel_relaxed (0xFFFFFFFFU, base + X2_TIMER_TMR0TGTH);

	writel_relaxed (X2_TIMER_TMR0START_WO (1), base + X2_TIMER_TMRSTART);

	err = clocksource_register_hz (&x2_tcs->cs, x2_tcs->x2_cpu_timer.freq);
	if (err) {
		kfree (x2_tcs);
		return err;
	}

	sched_clock_register (x2_sched_clock_read, timer_width,
	                      x2_tcs->x2_cpu_timer.freq);

	return 0;
}

static int __init x2_timer_init (struct device_node *timer)
{
	unsigned int irq;
	static int initialized;
	int ret;
	u32 timer_width = 64;
	u32 clk_cs_freq, clk_ce_freq;
	u32 timer_freq;

	if (initialized)
		return 0;

	initialized = 1;

	x2_timer_base = of_iomap (timer, 0);
	if (!x2_timer_base) {
		pr_err ("ERROR: invalid timer base address\n");
		return -ENXIO;
	}

	/* select timer1 for clock envent  */
	irq = irq_of_parse_and_map (timer, 1);
	if (irq <= 0) {
		pr_err ("ERROR: invalid interrupt number\n");
		return -EINVAL;
	}

	of_property_read_u32 (timer, "timer-freq", &timer_freq);
	clk_cs_freq = clk_ce_freq = timer_freq;

	ret = x2_setup_clocksource (clk_cs_freq, x2_timer_base, timer_width);
	if (ret)
		return ret;

	ret = x2_setup_clockevent (clk_ce_freq, x2_timer_base, irq);
	if (ret)
		return ret;

	pr_info ("%s #0 at %p, irq=%d\n", timer->name, x2_timer_base, irq);

	return 0;
}

CLOCKSOURCE_OF_DECLARE (x2_timer, "hobot,x2-timer", x2_timer_init);
