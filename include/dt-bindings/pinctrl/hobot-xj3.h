/*
 * This header provides constants for hobot pinctrl bindings.
 *
 * Copyright (c) 2020 Hobot Limited.
 *
 * This program is served for pinctrl_dtsi.You can see the
 * GNU General Public License for more details.
 */

#ifndef INCLUDE_DT_BINDINGS_PINCTRL_HOBOT_XJ3_H_
#define INCLUDE_DT_BINDINGS_PINCTRL_HOBOT_XJ3_H_

/* MUX functions for pins */
#define MUX_F0		0
#define MUX_F1		1
#define MUX_F2		2
#define MUX_F3		3

/* pin states bits */
#define PULL1_MASK	(3 << 6)
#define PULL2_MASK	(3 << 7)
#define PULL1_EN     (1 << 6)
#define PULL1_DIS	(0)
#define PULL2_DIS	(0)
#define PULL1_UP	(PULL1_EN | (1 << 7))
#define PULL2_UP	(1 << 8)
#define PULL1_DOWN	(PULL1_EN | (0 << 7))
#define PULL2_DOWN	(1 << 7)

/* drive strength definition */
#define DRIVE_MASK	(4 << 2)
#define DRIVE1_03MA	(0 << 2)
#define DRIVE2_06MA	(0 << 2)
#define DRIVE1_06MA	(1 << 2)
#define DRIVE2_09MA	(1 << 2)
#define DRIVE1_09MA	(2 << 2)
#define DRIVE2_12MA	(2 << 2)
#define DRIVE1_12MA	(3 << 2)
#define DRIVE2_15MA	(3 << 2)
#define DRIVE1_17MA	(4 << 2)
#define DRIVE2_18MA	(4 << 2)
#define DRIVE1_20MA	(5 << 2)
#define DRIVE2_21MA	(5 << 2)
#define DRIVE1_22MA	(6 << 2)
#define DRIVE2_24MA	(6 << 2)
#define DRIVE1_25MA	(7 << 2)
#define DRIVE2_27MA	(7 << 2)
#define DRIVE1_33MA	(8 << 2)
#define DRIVE2_30MA	(8 << 2)
#define DRIVE1_35MA	(9 << 2)
#define DRIVE2_33MA	(9 << 2)
#define DRIVE1_37MA	(10 << 2)
#define DRIVE2_36MA	(10 << 2)
#define DRIVE1_39MA	(11 << 2)
#define DRIVE2_39MA	(11 << 2)
#define DRIVE1_41MA	(12 << 2)
#define DRIVE2_41MA	(12 << 2)
#define DRIVE1_42_5MA	(13 << 2)
#define DRIVE2_42_5MA	(13 << 2)
#define DRIVE1_44MA	(14 << 2)
#define DRIVE2_44MA	(14 << 2)
#define DRIVE1_45MA	(15 << 2)
#define DRIVE2_45MA	(15 << 2)

/* pin schmitt */
#define SCHMITT1_ENA	(1 << 8)
#define SCHMITT1_DIS	(0 << 8)

#define SCHMITT2_ENA	(1 << 9)
#define SCHMITT2_DIS	(0 << 9)
#endif //INCLUDE_DT_BINDINGS_PINCTRL_HOBOT_XJ3_H_
