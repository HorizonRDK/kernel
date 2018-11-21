/*
 * Hobot pin controller
 *
 *	Copyright (C) 2018 Hobot
 * *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include "pinctrl-utils.h"
#include "core.h"

#define X2_NUM_IOS	118

struct x2_pctrl_group {
	const char *name;
	const unsigned int *pins;
	const unsigned int *mux_val;
	const unsigned npins;
};

struct x2_pinmux_function {
	const char *name;
	const char *const *groups;
	unsigned int ngroups;
	const unsigned int *mux_val;
};
struct __io_group {
	unsigned int start;
	unsigned int end;
	unsigned int regoffset;
};

struct x2_pinctrl {
	struct pinctrl_dev *pctrl;
	struct gpio_chip *gpio_chip;
	struct pinctrl_gpio_range *gpio_range;
	void __iomem *regbase;
	const struct x2_pctrl_group *groups;
	unsigned int ngroups;
	const struct x2_pinmux_function *funcs;
	unsigned int nfuncs;
};

enum {
	GPIO_IN = 0,
	GPIO_OUT = 1
};

enum {
	GPIO_LOW = 0,
	GPIO_HIGH = 1
};

#define X2_IO_MIN 1
#define X2_IO_MAX 118

#define GROUP0_MAX	14
#define GROUP1_MAX	30
#define GROUP2_MAX	45
#define GROUP3_MAX	61
#define GROUP4_MAX	77
#define GROUP5_MAX	93
#define GROUP6_MAX	109
#define GROUP7_MAX	118

#define X2_IO_CFG	0x0
#define X2_IO_PE	0x4
#define X2_IO_CTL	0x8
#define X2_IO_IN_VALUE	0xc
#define X2_IO_DIR_SHIFT	16

static struct __io_group io_groups[] = {
	[0] = {
	       .start = X2_IO_MIN,
	       .end = GROUP0_MAX,
	       .regoffset = 0x0,
	       },
	[1] = {
	       .start = GROUP0_MAX + 1,
	       .end = GROUP1_MAX,
	       .regoffset = 0x10,
	       },
	[2] = {
	       .start = GROUP1_MAX + 1,
	       .end = GROUP2_MAX,
	       .regoffset = 0x20,
	       },
	[3] = {
	       .start = GROUP2_MAX + 1,
	       .end = GROUP3_MAX,
	       .regoffset = 0x30,
	       },
	[4] = {
	       .start = GROUP3_MAX + 1,
	       .end = GROUP4_MAX,
	       .regoffset = 0x40,
	       },
	[5] = {
	       .start = GROUP4_MAX + 1,
	       .end = GROUP5_MAX,
	       .regoffset = 0x50,
	       },
	[6] = {
	       .start = GROUP5_MAX + 1,
	       .end = GROUP6_MAX,
	       .regoffset = 0x60,
	       },
	[7] = {
	       .start = GROUP6_MAX + 1,
	       .end = GROUP7_MAX,
	       .regoffset = 0x70,
	       },
};

unsigned int find_io_group_index(unsigned int io)
{
	if (io < X2_IO_MIN || io > X2_IO_MAX)
		return -1;

	if (io / (GROUP3_MAX + 1)) {
		if (io / (GROUP5_MAX + 1)) {
			if (io / (GROUP6_MAX + 1))
				return 7;
			else
				return 6;
		} else {
			if (io / (GROUP4_MAX + 1))
				return 5;
			else
				return 4;
		}
	} else {
		if (io / (GROUP1_MAX + 1)) {
			if (io / (GROUP2_MAX + 1))
				return 3;
			else
				return 2;
		} else {
			if (io / (GROUP0_MAX + 1))
				return 1;
			else
				return 0;
		}
	}
}

static const struct pinctrl_pin_desc x2_pins[] = {
	//PINCTRL_PIN(0,  "IO0"),
	PINCTRL_PIN(1, "IO1"),
	PINCTRL_PIN(2, "IO2"),
	PINCTRL_PIN(3, "IO3"),
	PINCTRL_PIN(4, "IO4"),
	PINCTRL_PIN(5, "IO5"),
	PINCTRL_PIN(6, "IO6"),
	PINCTRL_PIN(7, "IO7"),
	PINCTRL_PIN(8, "IO8"),
	PINCTRL_PIN(9, "IO9"),
	PINCTRL_PIN(10, "IO10"),
	PINCTRL_PIN(11, "IO11"),
	PINCTRL_PIN(12, "IO12"),
	PINCTRL_PIN(13, "IO13"),
	PINCTRL_PIN(14, "IO14"),
	PINCTRL_PIN(15, "IO15"),
	PINCTRL_PIN(16, "IO16"),
	PINCTRL_PIN(17, "IO17"),
	PINCTRL_PIN(18, "IO18"),
	PINCTRL_PIN(19, "IO19"),
	PINCTRL_PIN(20, "IO20"),
	PINCTRL_PIN(21, "IO21"),
	PINCTRL_PIN(22, "IO22"),
	PINCTRL_PIN(23, "IO23"),
	PINCTRL_PIN(24, "IO24"),
	PINCTRL_PIN(25, "IO25"),
	PINCTRL_PIN(26, "IO26"),
	PINCTRL_PIN(27, "IO27"),
	PINCTRL_PIN(28, "IO28"),
	PINCTRL_PIN(29, "IO29"),
	PINCTRL_PIN(30, "IO30"),
	PINCTRL_PIN(31, "IO31"),
	PINCTRL_PIN(32, "IO32"),
	PINCTRL_PIN(33, "IO33"),
	PINCTRL_PIN(34, "IO34"),
	PINCTRL_PIN(35, "IO35"),
	PINCTRL_PIN(36, "IO36"),
	PINCTRL_PIN(37, "IO37"),
	PINCTRL_PIN(38, "IO38"),
	PINCTRL_PIN(39, "IO39"),
	PINCTRL_PIN(40, "IO40"),
	PINCTRL_PIN(41, "IO41"),
	PINCTRL_PIN(42, "IO42"),
	PINCTRL_PIN(43, "IO43"),
	PINCTRL_PIN(44, "IO44"),
	PINCTRL_PIN(45, "IO45"),
	PINCTRL_PIN(46, "IO46"),
	PINCTRL_PIN(47, "IO47"),
	PINCTRL_PIN(48, "IO48"),
	PINCTRL_PIN(49, "IO49"),
	PINCTRL_PIN(50, "IO50"),
	PINCTRL_PIN(51, "IO51"),
	PINCTRL_PIN(52, "IO52"),
	PINCTRL_PIN(53, "IO53"),
	PINCTRL_PIN(54, "IO54"),
	PINCTRL_PIN(55, "IO55"),
	PINCTRL_PIN(56, "IO56"),
	PINCTRL_PIN(57, "IO57"),
	PINCTRL_PIN(58, "IO58"),
	PINCTRL_PIN(59, "IO59"),
	PINCTRL_PIN(60, "IO60"),
	PINCTRL_PIN(61, "IO61"),
	PINCTRL_PIN(62, "IO62"),
	PINCTRL_PIN(63, "IO63"),
	PINCTRL_PIN(64, "IO64"),
	PINCTRL_PIN(65, "IO65"),
	PINCTRL_PIN(66, "IO66"),
	PINCTRL_PIN(67, "IO67"),
	PINCTRL_PIN(68, "IO68"),
	PINCTRL_PIN(69, "IO69"),
	PINCTRL_PIN(70, "IO70"),
	PINCTRL_PIN(71, "IO71"),
	PINCTRL_PIN(72, "IO72"),
	PINCTRL_PIN(73, "IO73"),
	PINCTRL_PIN(74, "IO74"),
	PINCTRL_PIN(75, "IO75"),
	PINCTRL_PIN(76, "IO76"),
	PINCTRL_PIN(77, "IO77"),
	PINCTRL_PIN(78, "IO78"),
	PINCTRL_PIN(79, "IO79"),
	PINCTRL_PIN(80, "IO80"),
	PINCTRL_PIN(81, "IO81"),
	PINCTRL_PIN(82, "IO82"),
	PINCTRL_PIN(83, "IO83"),
	PINCTRL_PIN(84, "IO84"),
	PINCTRL_PIN(85, "IO85"),
	PINCTRL_PIN(86, "IO86"),
	PINCTRL_PIN(87, "IO87"),
	PINCTRL_PIN(88, "IO88"),
	PINCTRL_PIN(89, "IO89"),
	PINCTRL_PIN(90, "IO90"),
	PINCTRL_PIN(91, "IO91"),
	PINCTRL_PIN(92, "IO92"),
	PINCTRL_PIN(93, "IO93"),
	PINCTRL_PIN(94, "IO94"),
	PINCTRL_PIN(95, "IO95"),
	PINCTRL_PIN(96, "IO96"),
	PINCTRL_PIN(97, "IO97"),
	PINCTRL_PIN(98, "IO98"),
	PINCTRL_PIN(99, "IO99"),
	PINCTRL_PIN(100, "IO100"),
	PINCTRL_PIN(101, "IO101"),
	PINCTRL_PIN(102, "IO102"),
	PINCTRL_PIN(103, "IO103"),
	PINCTRL_PIN(104, "IO104"),
	PINCTRL_PIN(105, "IO105"),
	PINCTRL_PIN(106, "IO106"),
	PINCTRL_PIN(107, "IO107"),
	PINCTRL_PIN(108, "IO108"),
	PINCTRL_PIN(109, "IO109"),
	PINCTRL_PIN(110, "IO110"),
	PINCTRL_PIN(111, "IO111"),
	PINCTRL_PIN(112, "IO112"),
	PINCTRL_PIN(113, "IO113"),
	PINCTRL_PIN(114, "IO114"),
	PINCTRL_PIN(115, "IO115"),
	PINCTRL_PIN(116, "IO116"),
	PINCTRL_PIN(117, "IO117"),
	PINCTRL_PIN(118, "IO118"),
};

#define DEFINE_X2_PINCTRL_GRP(nm) \
	{ \
		.name = #nm "_grp", \
		.pins = nm ## _pins, \
		.npins = ARRAY_SIZE(nm ## _pins), \
		.mux_val = nm ## _muxs, \
	}

#define DEFINE_X2_PINMUX_FUNCTION(fname)	\
	[x2_pmux_##fname] = {				\
		.name = #fname,				\
		.groups = fname##_groups,		\
		.ngroups = ARRAY_SIZE(fname##_groups),	\
	}

enum x2_pinmux_functions {
	x2_pmux_i2c0,
	x2_pmux_i2s0,
	x2_pmux_i2s1,
	x2_pmux_btin,
	x2_pmux_dvpin,
	x2_pmux_btout,
	x2_pmux_dvpout,
	x2_pmux_max_func,
};

static const unsigned int i2c0_0_pins[] = { 9, 10 };
static const unsigned int i2c0_0_muxs[] = { 0, 0 };

static const unsigned int i2s0_0_pins[] = { 88, 89, 90, 91 };
static const unsigned int i2s0_0_muxs[] = { 0, 0, 0, 0 };

static const unsigned int i2s1_0_pins[] = { 5, 61, 62, 63 };
static const unsigned int i2s1_0_muxs[] = { 1, 2, 2, 2 };

static const unsigned int btin_0_pins[] =
    { 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77 };
static const unsigned int btin_0_muxs[] =
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static const unsigned int dvpin_0_pins[] =
    { 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75 };
static const unsigned int dvpin_0_muxs[] =
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

static const unsigned int btout_0_pins[] =
    { 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108,
109 };
static const unsigned int btout_0_muxs[] =
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static const unsigned int dvpout_0_pins[] =
    { 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107 };
static const unsigned int dvpout_0_muxs[] =
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

/* function groups */
static const char *const i2c0_groups[] = { "i2c0_0_grp" };
static const char *const i2s0_groups[] = { "i2s0_0_grp" };
static const char *const i2s1_groups[] = { "i2s1_0_grp" };
static const char *const btin_groups[] = { "btin_0_grp" };
static const char *const dvpin_groups[] = { "dvpin_0_grp" };
static const char *const btout_groups[] = { "btout_0_grp" };
static const char *const dvpout_groups[] = { "dvpout_0_grp" };

static const struct x2_pctrl_group x2_pctrl_groups[] = {
	DEFINE_X2_PINCTRL_GRP(i2c0_0),
	DEFINE_X2_PINCTRL_GRP(i2s0_0),
	DEFINE_X2_PINCTRL_GRP(i2s1_0),
	DEFINE_X2_PINCTRL_GRP(btin_0),
	DEFINE_X2_PINCTRL_GRP(dvpin_0),
	DEFINE_X2_PINCTRL_GRP(btout_0),
	DEFINE_X2_PINCTRL_GRP(dvpout_0),
};

static const struct x2_pinmux_function x2_pmux_functions[] = {
	DEFINE_X2_PINMUX_FUNCTION(i2c0),
	DEFINE_X2_PINMUX_FUNCTION(i2s0),
	DEFINE_X2_PINMUX_FUNCTION(i2s1),
	DEFINE_X2_PINMUX_FUNCTION(btin),
	DEFINE_X2_PINMUX_FUNCTION(dvpin),
	DEFINE_X2_PINMUX_FUNCTION(btout),
	DEFINE_X2_PINMUX_FUNCTION(dvpout),
};

static int x2_pctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct x2_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->ngroups;
}

static const char *x2_pctrl_get_group_name(struct pinctrl_dev *pctldev,
					   unsigned selector)
{
	struct x2_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->groups[selector].name;
}

static int x2_pctrl_get_group_pins(struct pinctrl_dev *pctldev,
				   unsigned selector,
				   const unsigned **pins, unsigned *num_pins)
{
	struct x2_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	*pins = pctrl->groups[selector].pins;
	*num_pins = pctrl->groups[selector].npins;

	return 0;
}

static int reserve_map(struct device *dev, struct pinctrl_map **map,
		       unsigned *reserved_maps, unsigned *num_maps,
		       unsigned reserve)
{
	unsigned old_num = *reserved_maps;
	unsigned new_num = *num_maps + reserve;
	struct pinctrl_map *new_map;
	if (old_num >= new_num)
		return 0;
	new_map = krealloc(*map, sizeof(*new_map) * new_num, GFP_KERNEL);
	if (!new_map) {
		dev_err(dev, "krealloc(map) failed\n");
		return -ENOMEM;
	}

	memset(new_map + old_num, 0, (new_num - old_num) * sizeof(*new_map));

	*map = new_map;
	*reserved_maps = new_num;
	return 0;
}

static int add_map_mux(struct pinctrl_map **map, unsigned int *reserved_maps,
		       unsigned int *num_maps, const char *group,
		       const char *function)
{
	if (WARN_ON(*num_maps == *reserved_maps))
		return -ENOSPC;

	(*map)[*num_maps].type = PIN_MAP_TYPE_MUX_GROUP;
	(*map)[*num_maps].data.mux.group = group;
	(*map)[*num_maps].data.mux.function = function;
	(*num_maps)++;
	return 0;
}

static int x2_pinctrl_dt_subnode_to_map(struct device *dev,
					struct device_node *np,
					struct pinctrl_map **map,
					unsigned int *reserved_maps,
					unsigned int *num_maps)
{
	int ret;
	const char *function;
	const char *group;
	unsigned int reserve;
	ret = of_property_read_string(np, "hobot,pin-function", &function);
	if (ret < 0) {
		/* EINVAL=missing, which is fine since it's optional */
		if (ret != -EINVAL)
			dev_err(dev, "could not parse property function\n");
		function = NULL;
	}
	ret = of_property_read_string(np, "hobot,pin-group", &group);
	if (ret < 0) {
		/* EINVAL=missing, which is fine since it's optional */
		if (ret != -EINVAL)
			dev_err(dev, "could not parse property group\n");
		group = NULL;
	}
	reserve = 0;
	if (function != NULL)
		reserve++;
	if (group != NULL)
		reserve++;

	ret = reserve_map(dev, map, reserved_maps, num_maps, reserve);
	if (ret < 0)
		goto exit;

	ret = add_map_mux(map, reserved_maps, num_maps, group, function);
	if (ret < 0)
		goto exit;

	ret = 0;
exit:
	return ret;
}

static void x2_pinctrl_dt_free_map(struct pinctrl_dev *pctldev,
				   struct pinctrl_map *map,
				   unsigned int num_maps)
{
	kfree(map);
}

static int x2_pinctrl_dt_node_to_map(struct pinctrl_dev *pctldev,
				     struct device_node *np_config,
				     struct pinctrl_map **map,
				     unsigned int *num_maps)
{
	unsigned int reserved_maps;
	struct device_node *np;
	int ret;

	reserved_maps = 0;
	*map = NULL;
	*num_maps = 0;
	for_each_child_of_node(np_config, np) {
		ret = x2_pinctrl_dt_subnode_to_map(pctldev->dev, np, map,
						   &reserved_maps, num_maps);
		if (ret < 0) {
			x2_pinctrl_dt_free_map(pctldev, *map, *num_maps);
			return ret;
		}
	}

	return 0;
}

static const struct pinctrl_ops x2_pctrl_ops = {
	.get_groups_count = x2_pctrl_get_groups_count,
	.get_group_name = x2_pctrl_get_group_name,
	.get_group_pins = x2_pctrl_get_group_pins,
	.dt_node_to_map = x2_pinctrl_dt_node_to_map,
	.dt_free_map = x2_pinctrl_dt_free_map,
};

static int x2_pmux_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct x2_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->nfuncs;
}

static const char *x2_pmux_get_function_name(struct pinctrl_dev *pctldev,
					     unsigned selector)
{
	struct x2_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->funcs[selector].name;
}

static int x2_pmux_get_function_groups(struct pinctrl_dev *pctldev,
				       unsigned selector,
				       const char *const **groups,
				       unsigned *const num_groups)
{
	struct x2_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	*groups = pctrl->funcs[selector].groups;
	*num_groups = pctrl->funcs[selector].ngroups;
	return 0;
}

static inline void x2_pinctrl_fsel_set(struct x2_pinctrl *pctrl,
				       unsigned pin, unsigned int fsel)
{
	int index, value;
	void __iomem *regaddr;
	index = find_io_group_index(pin);
	if (index < 0)
		return;
	regaddr = pctrl->regbase + io_groups[index].regoffset;
	value = readl(regaddr + X2_IO_CFG);
	value &= ~(0x3 << (pin - io_groups[index].start) * 2);
	value |= (fsel << (pin - io_groups[index].start));
	writel(value, regaddr + X2_IO_CFG);
}

static int x2_pinmux_set_mux(struct pinctrl_dev *pctldev,
			     unsigned int function, unsigned int group)
{
	int i;
	struct x2_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	const struct x2_pctrl_group *pgrp = &pctrl->groups[group];
	//const struct x2_pinmux_function *func = &pctrl->funcs[function];

	for (i = 0; i < pgrp->npins; i++) {
		unsigned int pin = pgrp->pins[i];
		unsigned int mux_val = pgrp->mux_val[i];
		x2_pinctrl_fsel_set(pctrl, pin, mux_val);
	}
	return 0;
}

static const struct pinmux_ops x2_pinmux_ops = {
	.get_functions_count = x2_pmux_get_functions_count,
	.get_function_name = x2_pmux_get_function_name,
	.get_function_groups = x2_pmux_get_function_groups,
	.set_mux = x2_pinmux_set_mux,
	.strict = true,
};

static int x2_pinconf_cfg_get(struct pinctrl_dev *pctldev,
			      unsigned pin, unsigned long *config)
{
	u32 index, value;
	unsigned int arg = 0;
	void __iomem *regaddr;

	unsigned int param = pinconf_to_config_param(*config);
	struct x2_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	if (pin >= X2_NUM_IOS)
		return -ENOTSUPP;

	index = find_io_group_index(pin);
	regaddr = pctrl->regbase + io_groups[index].regoffset;

	switch (param) {
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		{
			value = readl(regaddr + X2_IO_CTL);
			value &=
			    (0x1 <<
			     (pin - io_groups[index].start + X2_IO_DIR_SHIFT));
			if (!value)
				return -EINVAL;
			arg = 1;
		}
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int x2_pinconf_cfg_set(struct pinctrl_dev *pctldev,
			      unsigned pin,
			      unsigned long *configs, unsigned num_configs)
{
	int i, index, value;
	void __iomem *regaddr;
	struct x2_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	if (pin >= X2_NUM_IOS)
		return -ENOTSUPP;
	index = find_io_group_index(pin);
	regaddr = pctrl->regbase + io_groups[index].regoffset;

	for (i = 0; i < num_configs; i++) {
		unsigned int param = pinconf_to_config_param(configs[i]);
		unsigned int arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_DRIVE_PUSH_PULL:
			{
				value = readl(regaddr + X2_IO_CTL);
				if (arg)
					value |=
					    (0x1 <<
					     (pin - io_groups[index].start +
					      X2_IO_DIR_SHIFT));
				else
					value &=
					    ~(0x1 <<
					      (pin - io_groups[index].start +
					       X2_IO_DIR_SHIFT));
				writel(value, regaddr + X2_IO_CTL);
			}
			break;
		default:
			dev_warn(pctldev->dev,
				 "unsupported configuration parameter '%u'\n",
				 param);
			continue;
		}
	}

	return 0;
}

static const struct pinconf_ops x2_pinconf_ops = {
	.pin_config_get = x2_pinconf_cfg_get,
	.pin_config_set = x2_pinconf_cfg_set,
};

static struct pinctrl_desc x2_desc = {
	.name = "x2_pinctrl",
	.pins = x2_pins,
	.npins = ARRAY_SIZE(x2_pins),
	.pctlops = &x2_pctrl_ops,
	.pmxops = &x2_pinmux_ops,
	.confops = &x2_pinconf_ops,
	.owner = THIS_MODULE,
};

static int x2_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	u32 value;
	int index;
	void __iomem *regaddr;
	unsigned int gpio = chip->base + offset;
	struct x2_pinctrl *pctrl = gpiochip_get_data(chip);
	index = find_io_group_index(gpio);
	if (index < 0)
		return -1;
	regaddr = pctrl->regbase + io_groups[index].regoffset;

	value = readl(regaddr + X2_IO_IN_VALUE);
	value &= (0x1 << (gpio - io_groups[index].start));
	if (value)
		return 1;
	return 0;
}

static void x2_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	u32 value;
	int index;
	void __iomem *regaddr;
	unsigned int gpio = chip->base + offset;
	struct x2_pinctrl *pctrl = gpiochip_get_data(chip);
	index = find_io_group_index(gpio);
	if (index < 0)
		return;
	regaddr = pctrl->regbase + io_groups[index].regoffset;
	value = readl(regaddr + X2_IO_CTL);
	if (val == GPIO_LOW)
		value &= ~(0x1 << (gpio - io_groups[index].start));
	else if (val == GPIO_HIGH)
		value |= (0x1 << (gpio - io_groups[index].start));
	writel(value, regaddr + X2_IO_CTL);
}

static int x2_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	u32 value;
	int index;
	void __iomem *regaddr;
	unsigned int gpio = chip->base + offset;
	struct x2_pinctrl *pctrl = gpiochip_get_data(chip);

	index = find_io_group_index(gpio);
	if (index < 0)
		return -ENODEV;
	regaddr = pctrl->regbase + io_groups[index].regoffset;
	value = readl(regaddr + X2_IO_CTL);
	value &= ~(0x1 << (gpio - io_groups[index].start + X2_IO_DIR_SHIFT));
	writel(value, regaddr + X2_IO_CTL);
	return 0;
}

static int x2_gpio_direction_output(struct gpio_chip *chip,
				    unsigned offset, int val)
{
	u32 value;
	int index;
	void __iomem *regaddr;
	unsigned int gpio = chip->base + offset;
	struct x2_pinctrl *pctrl = gpiochip_get_data(chip);

	index = find_io_group_index(gpio);
	if (index < 0)
		return -ENODEV;
	regaddr = pctrl->regbase + io_groups[index].regoffset;
	value = readl(regaddr + X2_IO_CTL);
	value |= (0x1 << (gpio - io_groups[index].start + X2_IO_DIR_SHIFT));

	if (val == GPIO_LOW)
		value &= ~(0x1 << (gpio - io_groups[index].start));
	else if (val == GPIO_HIGH)
		value |= (0x1 << (gpio - io_groups[index].start));

	writel(value, regaddr + X2_IO_CTL);

	return 0;
}

static int x2_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return 0;
}

static int x2_gpio_request(struct gpio_chip *chip, unsigned int offset)
{
	int ret;
	struct x2_pinctrl *pctrl = gpiochip_get_data(chip);
	ret = pinctrl_request_gpio(chip->base + offset);
	if (ret)
		return ret;
	x2_pinctrl_fsel_set(pctrl, chip->base + offset, 0);
	return 0;
}

static void x2_gpio_free(struct gpio_chip *chip, unsigned int offset)
{
	pinctrl_free_gpio(chip->base + offset);
}

static struct gpio_chip x2_gpio = {
	.base = X2_IO_MIN,
	.ngpio = X2_IO_MAX - X2_IO_MIN + 1,
	.direction_input = x2_gpio_direction_input,
	.direction_output = x2_gpio_direction_output,
	.get = x2_gpio_get,
	.set = x2_gpio_set,
	.to_irq = x2_gpio_to_irq,
	.request = x2_gpio_request,
	.free = x2_gpio_free,
};

static struct pinctrl_gpio_range x2_pinctrl_gpio_range = {
	.name = "pinctrl-x2",
	.npins = X2_IO_MAX - X2_IO_MIN + 1,
};

static int x2_pinctrl_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct x2_pinctrl *x2_pctrl;
	int err;

	x2_pctrl =
	    devm_kzalloc(&pdev->dev, sizeof(struct x2_pinctrl), GFP_KERNEL);
	if (!x2_pctrl)
		return -ENOMEM;
	printk("x2_pinctrl_probe\n");
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "missing IO resource\n");
		return -ENODEV;
	}
	x2_pctrl->regbase = devm_ioremap_resource(&pdev->dev, res);

	x2_pctrl->gpio_chip = &x2_gpio;
	x2_pctrl->gpio_chip->parent = &pdev->dev;
	x2_pctrl->gpio_chip->of_node = pdev->dev.of_node;

	err = gpiochip_add_data(x2_pctrl->gpio_chip, x2_pctrl);
	if (err) {
		dev_err(&pdev->dev, "could not add GPIO chip\n");
		return err;
	}

	x2_pctrl->groups = x2_pctrl_groups;
	x2_pctrl->ngroups = ARRAY_SIZE(x2_pctrl_groups);
	x2_pctrl->funcs = x2_pmux_functions;
	x2_pctrl->nfuncs = ARRAY_SIZE(x2_pmux_functions);

	x2_pctrl->pctrl = devm_pinctrl_register(&pdev->dev, &x2_desc, x2_pctrl);
	if (IS_ERR(x2_pctrl->pctrl))
		return PTR_ERR(x2_pctrl->pctrl);

	x2_pctrl->gpio_range = &x2_pinctrl_gpio_range;
	x2_pctrl->gpio_range->base = X2_IO_MIN;
	x2_pctrl->gpio_range->pin_base = X2_IO_MIN;
	x2_pctrl->gpio_range->gc = x2_pctrl->gpio_chip;
	pinctrl_add_gpio_range(x2_pctrl->pctrl, x2_pctrl->gpio_range);

	platform_set_drvdata(pdev, x2_pctrl);

	dev_info(&pdev->dev, "x2 pinctrl initialized\n");

	return 0;
}

static const struct of_device_id x2_pinctrl_of_match[] = {
	{.compatible = "hobot,x2-pinctrl"},
	{}
};

static struct platform_driver x2_pinctrl_driver = {
	.driver = {
		   .name = "pinctrl-x2",
		   .of_match_table = x2_pinctrl_of_match,
		   },
	.probe = x2_pinctrl_probe,
};

static int __init x2_pinctrl_init(void)
{
	return platform_driver_register(&x2_pinctrl_driver);
}

arch_initcall(x2_pinctrl_init);
