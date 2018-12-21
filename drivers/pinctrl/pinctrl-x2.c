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
	unsigned npins;
	struct list_head node;
};

struct x2_pinmux_function {
	const char *name;
	const char *const *groups;
	unsigned int ngroups;
	struct list_head node;
};
struct __io_group {
	unsigned int start;
	unsigned int end;
	unsigned int regoffset;
};

struct x2_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctrl;
	struct mutex mutex;
	struct gpio_chip *gpio_chip;
	struct pinctrl_gpio_range *gpio_range;
	void __iomem *regbase;
	unsigned int ngroups;
	unsigned int nfuncs;
	struct list_head pingroups;
	struct list_head functions;
	struct radix_tree_root pgtree;
	struct radix_tree_root ftree;
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
	       .start = 0,	//X2_IO_MIN,  special case, gpio0 is remove but bit is remain
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

static int x2_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct x2_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	return pctrl->ngroups;
}

static const char *x2_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
					     unsigned selector)
{
	struct x2_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct x2_pctrl_group *group;
	group = radix_tree_lookup(&pctrl->pgtree, selector);
	if (!group) {
		dev_err(pctrl->dev, "%s could not find pingroup%i\n", __func__,
			selector);
		return NULL;
	}
	return group->name;
}

static int x2_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
				     unsigned selector,
				     const unsigned **pins, unsigned *num_pins)
{
	struct x2_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct x2_pctrl_group *group;
	group = radix_tree_lookup(&pctrl->pgtree, selector);
	if (!group) {
		dev_err(pctrl->dev, "%s could not find pingroup%i\n", __func__,
			selector);
		return -EINVAL;
	}
	*pins = group->pins;
	*num_pins = group->npins;
	return 0;
}

static unsigned int x2_add_pingroup(struct x2_pinctrl *x2_pctrl,
				    const char *name,
				    int *pins, int *muxs, unsigned npins)
{
	struct x2_pctrl_group *pingroup;
	pingroup = devm_kzalloc(x2_pctrl->dev, sizeof(*pingroup), GFP_KERNEL);
	if (!pingroup)
		return -ENOMEM;
	pingroup->name = name;
	pingroup->npins = npins;
	pingroup->pins = pins;
	pingroup->mux_val = muxs;

	mutex_lock(&x2_pctrl->mutex);
	list_add_tail(&pingroup->node, &x2_pctrl->pingroups);
	radix_tree_insert(&x2_pctrl->pgtree, x2_pctrl->ngroups, pingroup);
	x2_pctrl->ngroups++;
	mutex_unlock(&x2_pctrl->mutex);
	return 0;

}

static struct x2_pinmux_function *x2_add_function(struct x2_pinctrl *x2_pctrl,
						  const char *name,
						  const char **pgnames,
						  unsigned npgnames)
{
	struct x2_pinmux_function *function;
	function = devm_kzalloc(x2_pctrl->dev, sizeof(*function), GFP_KERNEL);
	if (!function)
		return NULL;
	function->name = name;
	function->groups = pgnames;
	function->ngroups = npgnames;

	mutex_lock(&x2_pctrl->mutex);
	list_add_tail(&function->node, &x2_pctrl->functions);
	radix_tree_insert(&x2_pctrl->ftree, x2_pctrl->nfuncs, function);
	x2_pctrl->nfuncs++;
	mutex_unlock(&x2_pctrl->mutex);
	return function;
}

static void x2_remove_function(struct x2_pinctrl *x2_pctrl,
			       struct x2_pinmux_function *function)
{
	int i;
	mutex_lock(&x2_pctrl->mutex);
	for (i = 0; i < x2_pctrl->nfuncs; i++) {
		struct x2_pinmux_function *found;
		found = radix_tree_lookup(&x2_pctrl->ftree, i);
		if (found == function)
			radix_tree_delete(&x2_pctrl->ftree, i);
	}
	list_del(&function->node);
	x2_pctrl->nfuncs--;
	mutex_unlock(&x2_pctrl->mutex);
}

static void x2_remove_group(struct x2_pinctrl *x2_pctrl,
			    struct x2_pctrl_group *pingroup)
{
	int i;
	mutex_lock(&x2_pctrl->mutex);
	for (i = 0; i < x2_pctrl->ngroups; i++) {
		struct x2_pctrl_group *found;
		found = radix_tree_lookup(&x2_pctrl->pgtree, i);
		if (found == pingroup)
			radix_tree_delete(&x2_pctrl->pgtree, i);
	}
	list_del(&pingroup->node);
	x2_pctrl->ngroups--;
	mutex_unlock(&x2_pctrl->mutex);
}

static int x2_find_func_byname(struct x2_pinctrl *pctrl, const char *func_name)
{
	struct x2_pinmux_function *func;
	int selector = 0;

	while (selector < pctrl->nfuncs) {
		func = radix_tree_lookup(&pctrl->ftree, selector);
		if (func && !strcmp(func_name, func->name))
			return selector;
		selector++;
	}
	return -1;
}

static void x2_pinctrl_free_funcs(struct x2_pinctrl *x2_pctrl)
{
	struct list_head *pos, *tmp;
	int i;

	mutex_lock(&x2_pctrl->mutex);
	for (i = 0; i < x2_pctrl->nfuncs; i++) {
		struct x2_pinmux_function *func;
		func = radix_tree_lookup(&x2_pctrl->ftree, i);
		if (!func)
			continue;
		radix_tree_delete(&x2_pctrl->ftree, i);
	}
	list_for_each_safe(pos, tmp, &x2_pctrl->functions) {
		struct x2_pinmux_function *function;
		function = list_entry(pos, struct x2_pinmux_function, node);
		list_del(&function->node);
	}
	mutex_unlock(&x2_pctrl->mutex);
}

static void x2_pinctrl_free_pingroups(struct x2_pinctrl *x2_pctrl)
{
	struct list_head *pos, *tmp;
	int i;

	mutex_lock(&x2_pctrl->mutex);
	for (i = 0; i < x2_pctrl->ngroups; i++) {
		struct x2_pctrl_group *pingroup;
		pingroup = radix_tree_lookup(&x2_pctrl->pgtree, i);
		if (!pingroup)
			continue;
		radix_tree_delete(&x2_pctrl->pgtree, i);
	}
	list_for_each_safe(pos, tmp, &x2_pctrl->pingroups) {
		struct x2_pctrl_group *pingroup;
		pingroup = list_entry(pos, struct x2_pctrl_group, node);
		list_del(&pingroup->node);
	}
	mutex_unlock(&x2_pctrl->mutex);
}

static int x2_parse_one_pinctrl_entry(struct x2_pinctrl *x2_pctrl,
				      struct device_node *np,
				      struct pinctrl_map **map,
				      unsigned *num_maps, const char **pgnames)
{
	const __be32 *pinmux_group;
	int size, rows, *pins, *muxs, index = 0, found = 0, res = -ENOMEM;
	struct x2_pinmux_function *function = NULL;
	pinmux_group = of_get_property(np, "pinctrl-x2,pins-muxs", &size);
	if ((!pinmux_group) || (size < sizeof(*pinmux_group) * 2)) {
		dev_err(x2_pctrl->dev, "bad data for mux %s\n", np->name);
		return -EINVAL;
	}
	if (x2_find_func_byname(x2_pctrl, np->name) >= 0) {
		pr_debug("existed func group\n");
		goto existed_func;
	}

	size /= sizeof(*pinmux_group);	/* Number of elements in array */
	rows = size / 2;

	pins = devm_kzalloc(x2_pctrl->dev, sizeof(*pins) * rows, GFP_KERNEL);
	if (!pins)
		return -ENOMEM;
	muxs = devm_kzalloc(x2_pctrl->dev, sizeof(*muxs) * rows, GFP_KERNEL);
	if (!muxs)
		goto free_pins;

	while (index < size) {
		unsigned val1, val2;

		val1 = be32_to_cpup(pinmux_group + index++);
		val2 = be32_to_cpup(pinmux_group + index++);
		pins[found] = val1;
		muxs[found++] = val2;
	}

	pgnames[0] = np->name;
	function = x2_add_function(x2_pctrl, np->name, pgnames, 1);
	if (!function)
		goto free_muxs;
	if (x2_add_pingroup(x2_pctrl, np->name, pins, muxs, found))
		goto free_function;

existed_func:
	(*map)->type = PIN_MAP_TYPE_MUX_GROUP;
	(*map)->data.mux.group = np->name;
	(*map)->data.mux.function = np->name;

	*num_maps = 1;
	return 0;

free_function:
	x2_remove_function(x2_pctrl, function);
free_muxs:
	devm_kfree(x2_pctrl->dev, muxs);
free_pins:
	devm_kfree(x2_pctrl->dev, pins);

	return res;
}

static int x2_pinctrl_dt_node_to_map(struct pinctrl_dev *pctldev,
				     struct device_node *np_config,
				     struct pinctrl_map **map,
				     unsigned *num_maps)
{
	struct x2_pinctrl *x2_pctrl;
	const char **pgnames;
	int ret;

	x2_pctrl = pinctrl_dev_get_drvdata(pctldev);

	/* create 2 maps. One is for pinmux, and the other is for pinconf.      pinconf TODO */
	*map = devm_kzalloc(x2_pctrl->dev, sizeof(**map) * 2, GFP_KERNEL);
	if (!*map)
		return -ENOMEM;

	*num_maps = 0;
	pgnames = devm_kzalloc(x2_pctrl->dev, sizeof(*pgnames), GFP_KERNEL);
	if (!pgnames) {
		ret = -ENOMEM;
		goto free_map;
	}
	ret =
	    x2_parse_one_pinctrl_entry(x2_pctrl, np_config, map, num_maps,
				       pgnames);
	if (ret < 0) {
		dev_err(x2_pctrl->dev, "no pins entries for %s ret:%d\n",
			np_config->name, ret);
		goto free_pgnames;
	}
	return 0;

free_pgnames:
	devm_kfree(x2_pctrl->dev, pgnames);
free_map:
	devm_kfree(x2_pctrl->dev, *map);

	return ret;
}

static void x2_pinctrl_dt_free_map(struct pinctrl_dev *pctldev,
				   struct pinctrl_map *map,
				   unsigned int num_maps)
{
	struct x2_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	devm_kfree(pctrl->dev, map);
}

static const struct pinctrl_ops x2_pctrl_ops = {
	.get_groups_count = x2_pinctrl_get_groups_count,
	.get_group_name = x2_pinctrl_get_group_name,
	.get_group_pins = x2_pinctrl_get_group_pins,
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
	struct x2_pinmux_function *func;
	func = radix_tree_lookup(&pctrl->ftree, selector);
	if (!func) {
		dev_err(pctrl->dev, "%s could not find function%i\n", __func__,
			selector);
		return NULL;
	}
	return func->name;
}

static int x2_pmux_get_function_groups(struct pinctrl_dev *pctldev,
				       unsigned selector,
				       const char *const **groups,
				       unsigned *const num_groups)
{
	struct x2_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct x2_pinmux_function *func;
	func = radix_tree_lookup(&pctrl->ftree, selector);
	if (!func) {
		dev_err(pctrl->dev, "%s could not find function%i\n", __func__,
			selector);
		return -EINVAL;
	}
	*groups = func->groups;
	*num_groups = func->ngroups;
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
	value |= (fsel << (pin - io_groups[index].start) * 2);
	writel(value, regaddr + X2_IO_CFG);
	pr_debug("pin:%d fsel:%d add:0x%p value:0x%x\n", pin, fsel,
		 regaddr + X2_IO_CFG, value);
}

static int x2_pinmux_set_mux(struct pinctrl_dev *pctldev,
			     unsigned int function, unsigned int group)
{
	int i;
	struct x2_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	const struct x2_pctrl_group *pgrp;
	pgrp = radix_tree_lookup(&pctrl->pgtree, group);

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
	x2_pinctrl_fsel_set(pctrl, chip->base + offset, 0x3);
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
	int ret = 0;

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
	x2_pctrl->dev = &pdev->dev;
	mutex_init(&x2_pctrl->mutex);
	INIT_LIST_HEAD(&x2_pctrl->pingroups);
	INIT_LIST_HEAD(&x2_pctrl->functions);

	x2_pctrl->gpio_chip = &x2_gpio;
	x2_pctrl->gpio_chip->parent = &pdev->dev;
	x2_pctrl->gpio_chip->of_node = pdev->dev.of_node;

	ret = gpiochip_add_data(x2_pctrl->gpio_chip, x2_pctrl);
	if (ret) {
		dev_err(&pdev->dev, "could not add GPIO chip\n");
		goto free;
	}

	x2_pctrl->pctrl = devm_pinctrl_register(&pdev->dev, &x2_desc, x2_pctrl);
	if (IS_ERR(x2_pctrl->pctrl)) {
		ret = PTR_ERR(x2_pctrl->pctrl);
		goto free;
	}

	x2_pctrl->gpio_range = &x2_pinctrl_gpio_range;
	x2_pctrl->gpio_range->base = X2_IO_MIN;
	x2_pctrl->gpio_range->pin_base = X2_IO_MIN;
	x2_pctrl->gpio_range->gc = x2_pctrl->gpio_chip;
	pinctrl_add_gpio_range(x2_pctrl->pctrl, x2_pctrl->gpio_range);

	platform_set_drvdata(pdev, x2_pctrl);

	dev_info(&pdev->dev, "x2 pinctrl initialized\n");

	return 0;
free:
	x2_pinctrl_free_funcs(x2_pctrl);
	x2_pinctrl_free_pingroups(x2_pctrl);
	return ret;
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
