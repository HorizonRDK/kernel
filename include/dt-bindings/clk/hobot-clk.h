/*
 * The constants defined in this header are used in dts files
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _DT_BINDINGS_CLK_HOBOT_H
#define _DT_BINDINGS_CLK_HOBOT_H

#define BIT(nr)			(1 << (nr))

#define CLK_SET_RATE_GATE		BIT(0)
#define CLK_SET_PARENT_GATE		BIT(1)
#define CLK_SET_RATE_PARENT		BIT(2)
#define CLK_IGNORE_UNUSED		BIT(3)
#define CLK_IS_BASIC			BIT(5)
#define CLK_GET_RATE_NOCACHE		BIT(6)
#define CLK_SET_RATE_NO_REPARENT	BIT(7)
#define CLK_GET_ACCURACY_NOCACHE	BIT(8)
#define CLK_RECALC_NEW_RATES		BIT(9)
#define CLK_SET_RATE_UNGATE		BIT(10)
#define CLK_IS_CRITICAL			BIT(11)
#define CLK_OPS_PARENT_ENABLE		BIT(12)

#define CLK_DIVIDER_ONE_BASED		BIT(0)
#define CLK_DIVIDER_POWER_OF_TWO	BIT(1)
#define CLK_DIVIDER_ALLOW_ZERO		BIT(2)
#define CLK_DIVIDER_HIWORD_MASK		BIT(3)
#define CLK_DIVIDER_ROUND_CLOSEST	BIT(4)
#define CLK_DIVIDER_READ_ONLY		BIT(5)
#define CLK_DIVIDER_MAX_AT_ZERO		BIT(6)
#define CLK_DIVIDER_ROUND_DOWN		BIT(7)

#define CLK_MUX_INDEX_ONE               BIT(0)
#define CLK_MUX_INDEX_BIT               BIT(1)
#define CLK_MUX_HIWORD_MASK             BIT(2)
#define CLK_MUX_READ_ONLY               BIT(3)
#define CLK_MUX_ROUND_CLOSEST           BIT(4)

#endif
