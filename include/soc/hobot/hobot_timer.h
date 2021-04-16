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

#ifndef __HOBOT_TIMER_H__
#define __HOBOT_TIMER_H__


/* Hobot TIMER register offsets */
#define HOBOT_TIMER_TMREN_REG		  0x00
#define HOBOT_TIMER_TMRSTART_REG	  0x04
#define HOBOT_TIMER_TMRSTOP_REG	  0x08
#define HOBOT_TIMER_TMRMODE_REG	  0x0C
#define HOBOT_TIMER_TMR0TGTL_REG	  0x10
#define HOBOT_TIMER_TMR0TGTH_REG	  0x14
#define HOBOT_TIMER_TMR0DL_REG 	  0x18
#define HOBOT_TIMER_TMR0DH_REG 	  0x1C
#define HOBOT_TIMER_TMR1TGT_REG	  0x20
#define HOBOT_TIMER_TMR1D_REG		  0x24
#define HOBOT_TIMER_WDTGT_REG		  0x28
#define HOBOT_TIMER_WDWAIT_REG 	  0x2C
#define HOBOT_TIMER_WD1D_REG		  0x30
#define HOBOT_TIMER_WD2D_REG		  0x34
#define HOBOT_TIMER_WDCLR_REG		  0x38
#define HOBOT_TIMER_TMR_SRCPND_REG   0x3C
#define HOBOT_TIMER_TMR_INTMASK_REG  0x40
#define HOBOT_TIMER_TMR_SETMASK_REG  0x44
#define HOBOT_TIMER_TMR_UNMASK_REG   0x48

/* Hobot TIMER register op-bit Masks */
#define HOBOT_TIMER_T0START		  BIT(0)
#define HOBOT_TIMER_T1START		  BIT(1)
#define HOBOT_TIMER_T2START		  BIT(2)
#define HOBOT_TIMER_T0STOP 		  BIT(0)
#define HOBOT_TIMER_T1STOP 		  BIT(1)
#define HOBOT_TIMER_T2STOP 		  BIT(2)
#define HOBOT_TIMER_ONE_MODE		  0x0	  /* one-time mode */
#define HOBOT_TIMER_PRD_MODE		  0x1	  /* periodical mode */
#define HOBOT_TIMER_CON_MODE		  0x2	  /* continuous mode */
#define HOBOT_TIMER_WDT_MODE		  0x3	  /* watchdog mode, only for timer2 of timer module0 */
#define HOBOT_TIMER_T0MODE_OFFSET	  0x0
#define HOBOT_TIMER_T1MODE_OFFSET	  0x4
#define HOBOT_TIMER_T2MODE_OFFSET	  0x8
#define HOBOT_TIMER_WDT_RESET		  BIT(0)
#define HOBOT_TIMER_T0_INTMASK		  BIT(0)
#define HOBOT_TIMER_T1_INTMASK		  BIT(1)
#define HOBOT_TIMER_T2_INTMASK		  BIT(2)
#define HOBOT_TIMER_WDT_INTMASK	  HOBOT_TIMER_T2_INTMASK
#define HOBOT_TIMER_INT_ALLCLR		  0x7

#define HOBOT_TIMER_T0_WIDTH		  64
#define HOBOT_TIMER_T1_WIDTH		  32
#define HOBOT_TIMER_T2_WIDTH		  32

#define HOBOT_TIMER_REF_CLOCK		  24000000


#endif
