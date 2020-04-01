/*
 * Copyright (c) 2020
 * pf5024-regulator.c - Voltage regulation for NXP pf5024 PMUs
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/pf5024.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regmap.h>

/*
 * PF5024 Global Register Map.
 */
#define PF5024_DCDC1_VSET	0x4b
#define PF5024_DCDC2_VSET	0x53
#define PF5024_DCDC3_VSET	0x5b
#define PF5024_DCDC4_VSET	0x63
#define PF5024_SYS_CTRL		0x33
#define PF5024_PMIC_OFF		0x2


/*
 * PF5024 Definitions.
 */
#define	PF5024_DCDC1_MODE		0x4a
#define	PF5024_DCDC2_MODE		0x52
#define	PF5024_DCDC3_MODE		0x5a
#define	PF5024_DCDC4_MODE		0x62
#define PF5024_ENA_MASK			0x03
#define	PF5024_ENA			0x01
#define	PF5024_DIS			0x0
#define	PF5024_VSEL_MASK		0xFF	/* VSET - [7:0] */

/*
 * PF5024 voltage number
 */
#define	PF5024_VOLTAGE_NUM	178

struct pf5024 {
	struct regmap *regmap;
	int off_reg;
};

static const struct regmap_config pf5024_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct regulator_linear_range pf5024_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(400000, 0, 175, 6250),
	REGULATOR_LINEAR_RANGE(1500000, 176, 177, 300000),
};

static struct regulator_ops pf5024_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
};

#define PF5024_REG(_name, _id)		\
	[ID_##_id] = {					\
		.name			= _name,			\
		.id			= ID_##_id,		\
		.type			= REGULATOR_VOLTAGE,		\
		.ops			= &pf5024_ops,			\
		.n_voltages		= PF5024_VOLTAGE_NUM,		\
		.linear_ranges		= pf5024_voltage_ranges,	\
		.n_linear_ranges	= ARRAY_SIZE(pf5024_voltage_ranges), \
		.vsel_reg		= PF5024##_##_id##_VSET, \
		.vsel_mask		= PF5024_VSEL_MASK,		\
		.enable_reg		= (PF5024##_##_id##_MODE),	\
		.enable_mask		= PF5024_ENA_MASK,			\
		.enable_val		= PF5024_ENA,			\
		.disable_val		= PF5024_DIS,			\
		.owner			= THIS_MODULE,			\
	}

static const struct regulator_desc pf5024_regulators[] = {
	PF5024_REG("DCDC1", DCDC1),
	PF5024_REG("DCDC2", DCDC2),
	PF5024_REG("DCDC3", DCDC3),
	PF5024_REG("DCDC4", DCDC4),
};

#ifdef CONFIG_OF
static const struct of_device_id pf5024_dt_ids[] = {
	{ .compatible = "nxp,pf5024" },
	{ }
};
MODULE_DEVICE_TABLE(of, pf5024_dt_ids);

static struct of_regulator_match pf5024_matches[] = {
	[ID_DCDC1]	= { .name = "DCDC1"},
	[ID_DCDC2]	= { .name = "DCDC2"},
	[ID_DCDC3]	= { .name = "DCDC3"},
	[ID_DCDC4]	= { .name = "DCDC4"},
};

static int pf5024_pdata_from_dt(struct device *dev,
				 struct pf5024_platform_data *pdata)
{
	int matched, i, num_matches;
	struct device_node *np;
	struct pf5024_regulator_data *regulator;
	struct of_regulator_match *matches;

	matches = pf5024_matches;
	num_matches = ARRAY_SIZE(pf5024_matches);

	np = of_get_child_by_name(dev->of_node, "regulators");
	if (!np) {
		dev_err(dev, "missing 'regulators' subnode in DT\n");
		return -EINVAL;
	}

	matched = of_regulator_match(dev, np, matches, num_matches);
	of_node_put(np);
	if (matched <= 0)
		return matched;

	pdata->regulators = devm_kzalloc(dev,
					 sizeof(struct pf5024_regulator_data) *
					 num_matches, GFP_KERNEL);
	if (!pdata->regulators)
		return -ENOMEM;

	pdata->num_regulators = num_matches;
	regulator = pdata->regulators;

	for (i = 0; i < num_matches; i++) {
		regulator->id = i;
		regulator->name = matches[i].name;
		regulator->init_data = matches[i].init_data;
		regulator->of_node = matches[i].of_node;
		regulator++;
	}

	return 0;
}
#else
static inline int pf5024_pdata_from_dt(struct device *dev,
					struct pf5024_platform_data *pdata)
{
	return 0;
}
#endif

static struct pf5024_regulator_data *pf5024_get_regulator_data(
		int id, struct pf5024_platform_data *pdata)
{
	int i;

	if (!pdata)
		return NULL;

	for (i = 0; i < pdata->num_regulators; i++) {
		if (pdata->regulators[i].id == id)
			return &pdata->regulators[i];
	}

	return NULL;
}

static struct i2c_client *pf5024_i2c_client;
__attribute__((unused)) static void pf5024_power_off(void)
{
	struct pf5024 *pf5024;

	pf5024 = i2c_get_clientdata(pf5024_i2c_client);
	regmap_write(pf5024->regmap, pf5024->off_reg, PF5024_PMIC_OFF);

	while (1)
		continue;
}

static int pf5024_pmic_probe(struct i2c_client *client,
			      const struct i2c_device_id *i2c_id)
{
	const struct regulator_desc *regulators;
	struct pf5024_platform_data pdata_of, *pdata;
	struct device *dev = &client->dev;
	int i, ret, num_regulators;
	struct pf5024 *pf5024;
	const struct regmap_config *regmap_config;
	uint64_t type;

	dev_info(dev, "start probe\n");
	pdata = dev_get_platdata(dev);

	if (dev->of_node && !pdata) {
		const struct of_device_id *id;

		id = of_match_device(of_match_ptr(pf5024_dt_ids), dev);
		if (!id)
			return -ENODEV;

		type = (uint64_t) id->data;
	} else {
		type = i2c_id->driver_data;
	}

	regulators = pf5024_regulators;
	num_regulators = ARRAY_SIZE(pf5024_regulators);
	regmap_config = &pf5024_regmap_config;

	if (dev->of_node && !pdata) {
		ret = pf5024_pdata_from_dt(dev, &pdata_of);
		if (ret < 0)
			return ret;

		pdata = &pdata_of;
	}

	pf5024 = devm_kzalloc(dev, sizeof(struct pf5024), GFP_KERNEL);
	if (!pf5024)
		return -ENOMEM;

	pf5024->regmap = devm_regmap_init_i2c(client, regmap_config);
	if (IS_ERR(pf5024->regmap)) {
		ret = PTR_ERR(pf5024->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	pf5024->off_reg = PF5024_SYS_CTRL;
//	pm_power_off = pf5024_power_off;

	/* Finally register devices */
	for (i = 0; i < num_regulators; i++) {
		const struct regulator_desc *desc = &regulators[i];
		struct regulator_config config = { };
		struct pf5024_regulator_data *rdata;
		struct regulator_dev *rdev;

		config.dev = dev;
		config.driver_data = pf5024;
		config.regmap = pf5024->regmap;

		rdata = pf5024_get_regulator_data(desc->id, pdata);
		if (rdata) {
			config.init_data = rdata->init_data;
			config.of_node = rdata->of_node;
		}

		rdev = devm_regulator_register(dev, desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(dev, "failed to register %s\n", desc->name);
			return PTR_ERR(rdev);
		}
	}

	pf5024_i2c_client = client;
	i2c_set_clientdata(client, pf5024);

	return 0;
}

static const struct i2c_device_id pf5024_ids[] = {
	{ .name = "pf5024", },
	{ },
};
MODULE_DEVICE_TABLE(i2c, pf5024_ids);

static struct i2c_driver pf5024_pmic_driver = {
	.driver	= {
		.name	= "pf5024",
	},
	.probe		= pf5024_pmic_probe,
	.id_table	= pf5024_ids,
};

static int __init pf5024_init(void)
{
	return i2c_add_driver(&pf5024_pmic_driver);
}

subsys_initcall(pf5024_init);

MODULE_DESCRIPTION("nxp pf5024 voltage regulator driver");
MODULE_AUTHOR("Yunqian Wang <yunqian.wang@horizon.ai>");
MODULE_LICENSE("GPL v2");
