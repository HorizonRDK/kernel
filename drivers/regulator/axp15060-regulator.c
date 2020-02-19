/*
 * Copyright (c) 2020
 * axp15060-regulator.c - Voltage regulation for X-Powers axp15060 PMUs
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
#include <linux/regulator/axp15060.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regmap.h>

/*
 * AXP15060 Global Register Map.
 */
#define AXP15060_ON_OFF_CTRL1       0x10  //output power on-off control
#define AXP15060_ON_OFF_CTRL2       0x11  //output power on-off control
#define AXP15060_ON_OFF_CTRL3       0x12  //output power on-off control

#define AXP15060_DCDC1_VSET	    0x13 //DCDC1 voltage set
#define AXP15060_DCDC2_VSET	    0x14 //DCDC2 voltage set
#define AXP15060_DCDC3_VSET	    0x15 //DCDC3 voltage set
#define AXP15060_DCDC4_VSET	    0x16 //DCDC4 voltage set
#define AXP15060_DCDC5_VSET	    0x17 //DCDC5 voltage set
#define AXP15060_DCDC6_VSET	    0x18 //DCDC6 voltage set

#define AXP15060_DCDC_MODE_CTRL1    0x1a  //on-off control
#define AXP15060_DCDC_MODE_CTRL2    0x1b  //DCDC6-1 PFM/PWM control

#define AXP15060_BLDO1_VSET         0x24 //BLDO1 voltage set
#define AXP15060_BLDO2_VSET         0x25
#define AXP15060_BLDO3_VSET         0x26
#define AXP15060_BLDO4_VSET         0x27
#define AXP15060_BLDO5_VSET         0x28

#define AXP15060_CLDO1_VSET         0x29 //CLDO1 voltage set
#define AXP15060_CLDO2_VSET         0x2a
#define AXP15060_CLDO3_VSET         0x2b
#define AXP15060_CLDO4_VSET         0x2d

#define AXP15060_PMIC_OFF           0x32 //pmic power down, BIT[7]

/*
 * AXP15060 Definitions.
 */
#define AXP15060_DCDC6_ENA          0x20
#define AXP15060_DCDC5_ENA          0x10
#define AXP15060_DCDC4_ENA          0x08
#define AXP15060_DCDC3_ENA          0x04
#define AXP15060_DCDC2_ENA          0x02
#define AXP15060_DCDC1_ENA          0x01

#define AXP15060_BLDO3_ENA          0x80
#define AXP15060_BLDO2_ENA          0x40
#define AXP15060_BLDO1_ENA          0x20
#define AXP15060_ALDO5_ENA          0x10
#define AXP15060_ALDO4_ENA          0x08
#define AXP15060_ALDO3_ENA          0x04
#define AXP15060_ALDO2_ENA          0x02
#define AXP15060_ALDO1_ENA          0x01

#define AXP15060_SWITCH_ENA         0x80
#define AXP15060_CPUSLDO_ENA        0x40
#define AXP15060_CLDO4_ENA          0x20
#define AXP15060_CLDO3_ENA          0x10
#define AXP15060_CLDO2_ENA          0x08
#define AXP15060_CLDO1_ENA          0x04
#define AXP15060_BLDO5_ENA          0x02
#define AXP15060_BLDO4_ENA          0x01

#define AXP15060_VSET_MASK5         0x1F /*VSET - [4:0]*/
#define AXP15060_VSET_MASK7         0x7F /*VSET - [6:0]*/

#define AXP15060_POWER_OFF          0x80

/*
 * PF5024 voltage number
 */
#define AXP15060_VOLTAGE_NUM20      20
#define AXP15060_VOLTAGE_NUM27      27
#define AXP15060_VOLTAGE_NUM30      30
#define AXP15060_VOLTAGE_NUM36      36
#define AXP15060_VOLTAGE_NUM69      69
#define AXP15060_VOLTAGE_NUM88      88

struct axp15060 {
	struct regmap *regmap;
	int off_reg;
};

static const struct regmap_config axp15060_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct regulator_linear_range axp15060_voltage_ranges1[] = {
	REGULATOR_LINEAR_RANGE(1500000, 0, 19, 100000),
};

static const struct regulator_linear_range axp15060_voltage_ranges2[] = {
	REGULATOR_LINEAR_RANGE(500000, 0, 70, 10000),
	REGULATOR_LINEAR_RANGE(1220000, 71, 87, 20000),
};

static const struct regulator_linear_range axp15060_voltage_ranges3[] = {
	REGULATOR_LINEAR_RANGE(800000, 0, 32, 10000),
	REGULATOR_LINEAR_RANGE(1140000, 33, 68, 20000),
};

static const struct regulator_linear_range axp15060_voltage_ranges4[] = {
	REGULATOR_LINEAR_RANGE(500000, 0, 29, 100000),
};

static const struct regulator_linear_range axp15060_voltage_ranges5[] = {
	REGULATOR_LINEAR_RANGE(700000, 0, 26, 100000),
};

static const struct regulator_linear_range axp15060_voltage_ranges6[] = {
	REGULATOR_LINEAR_RANGE(700000, 0, 35, 100000),
};


static struct regulator_ops axp15060_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
};

#define AXP15060_REG(_name, _id, _linear, _step, _vset_mask, _enable)		\
	[ID_##_id] = {					\
		.name			= _name,			\
		.id			= ID_##_id,		\
		.type			= REGULATOR_VOLTAGE,		\
		.ops			= &axp15060_ops,			\
		.n_voltages		= AXP15060_VOLTAGE_NUM##_step,		\
		.linear_ranges		= axp15060_voltage_ranges##_linear,	\
		.n_linear_ranges	= ARRAY_SIZE(axp15060_voltage_ranges##_linear), \
		.vsel_reg		= AXP15060##_##_id##_VSET, \
		.vsel_mask		= AXP15060_VSET_MASK##_vset_mask,		\
		.enable_reg		= (AXP15060_ON_OFF_CTRL##_enable),	\
		.enable_mask		= AXP15060##_##_id##_ENA,			\
		.disable_val		= AXP15060_POWER_OFF,			\
		.owner			= THIS_MODULE,			\
	}

static const struct regulator_desc axp15060_regulators[] = {
	AXP15060_REG("DCDC1", DCDC1, 1, 20, 5, 1),
	AXP15060_REG("DCDC2", DCDC2, 2, 88, 7, 1),
	AXP15060_REG("DCDC3", DCDC3, 2, 88, 7, 1),
	AXP15060_REG("DCDC4", DCDC4, 2, 88, 7, 1),
	AXP15060_REG("DCDC5", DCDC5, 3, 69, 7, 1),
	AXP15060_REG("DCDC6", DCDC6, 4, 30, 5, 1),
	AXP15060_REG("BLDO1", BLDO1, 5, 27, 5, 2),
	AXP15060_REG("CLDO2", CLDO2, 5, 27, 5, 3),
};

#ifdef CONFIG_OF
static const struct of_device_id axp15060_dt_ids[] = {
	{ .compatible = "X-Powers,axp15060" },
	{ }
};
MODULE_DEVICE_TABLE(of, axp15060_dt_ids);

static struct of_regulator_match axp15060_matches[] = {
	[ID_DCDC1]	= { .name = "DCDC1"},
	[ID_DCDC2]	= { .name = "DCDC2"},
	[ID_DCDC3]	= { .name = "DCDC3"},
	[ID_DCDC4]	= { .name = "DCDC4"},
	[ID_DCDC5]	= { .name = "DCDC5"},
	[ID_DCDC6]	= { .name = "DCDC6"},
	[ID_BLDO1]	= { .name = "BLDO1"},
	[ID_CLDO2]	= { .name = "CLDO2"},
};

static int axp15060_pdata_from_dt(struct device *dev,
				 struct axp15060_platform_data *pdata)
{
	int matched, i, num_matches;
	struct device_node *np;
	struct axp15060_regulator_data *regulator;
	struct of_regulator_match *matches;

	matches = axp15060_matches;
	num_matches = ARRAY_SIZE(axp15060_matches);

	np = of_get_child_by_name(dev->of_node, "regulators");
	if (!np) {
		dev_err(dev, "missing 'regulators' subnode in DT, %d\n", -EINVAL);
		return -EINVAL;
	}

	matched = of_regulator_match(dev, np, matches, num_matches);
	of_node_put(np);
	if (matched <= 0) {
		dev_err(dev, "regulators not match, %d\n", matched);
		return matched;
	}

	pdata->regulators = devm_kzalloc(dev,
					 sizeof(struct axp15060_regulator_data) *
					 num_matches, GFP_KERNEL);
	if (!pdata->regulators) {
		dev_err(dev, "malloc for regulators fail, %d\n", -ENOMEM);
		return -ENOMEM;
	}

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
static inline int axp15060_pdata_from_dt(struct device *dev,
					struct axp15060_platform_data *pdata)
{
	return 0;
}
#endif

static struct axp15060_regulator_data *axp15060_get_regulator_data(
		int id, struct axp15060_platform_data *pdata)
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

static struct i2c_client *axp15060_i2c_client;
__attribute__((unused)) static void axp15060_power_off(void)
{
	struct axp15060 *axp15060;

	axp15060 = i2c_get_clientdata(axp15060_i2c_client);
	regmap_write(axp15060->regmap, axp15060->off_reg, AXP15060_PMIC_OFF);

	while (1)
		continue;
}

static int axp15060_pmic_probe(struct i2c_client *client,
			      const struct i2c_device_id *i2c_id)
{
	const struct regulator_desc *regulators;
	struct axp15060_platform_data pdata_of, *pdata;
	struct device *dev = &client->dev;
	int i, ret, num_regulators;
	struct axp15060 *axp15060;
	const struct regmap_config *regmap_config;
	uint64_t type;

	dev_info(dev, "AXP15060 start probe\n");
	pdata = dev_get_platdata(dev);

	if (dev->of_node && !pdata) {
		const struct of_device_id *id;

		id = of_match_device(of_match_ptr(axp15060_dt_ids), dev);
		if (!id) {
			dev_err(dev, "No device : %d\n", -ENODEV);
			return -ENODEV;
		}

		type = (uint64_t) id->data;
	} else {
		type = i2c_id->driver_data;
	}

	regulators = axp15060_regulators;
	num_regulators = ARRAY_SIZE(axp15060_regulators);
	regmap_config = &axp15060_regmap_config;

	if (dev->of_node && !pdata) {
		ret = axp15060_pdata_from_dt(dev, &pdata_of);
		if (ret < 0) {
			dev_err(dev, "Get pdata_of from dt error, %d\n", ret);
			return ret;
		}

		pdata = &pdata_of;
	}

	axp15060 = devm_kzalloc(dev, sizeof(struct axp15060), GFP_KERNEL);
	if (!axp15060) {
		dev_err(dev, "No memory for axp15060 : %d\n", -ENOMEM);
		return -ENOMEM;
	}

	axp15060->regmap = devm_regmap_init_i2c(client, regmap_config);
	if (IS_ERR(axp15060->regmap)) {
		ret = PTR_ERR(axp15060->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	axp15060->off_reg = AXP15060_PMIC_OFF;

	/* Finally register devices */
	for (i = 0; i < num_regulators; i++) {
		const struct regulator_desc *desc = &regulators[i];
		struct regulator_config config = { };
		struct axp15060_regulator_data *rdata;
		struct regulator_dev *rdev;

		config.dev = dev;
		config.driver_data = axp15060;
		config.regmap = axp15060->regmap;

		rdata = axp15060_get_regulator_data(desc->id, pdata);
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

	axp15060_i2c_client = client;
	i2c_set_clientdata(client, axp15060);

	dev_info(dev, "AXP15060 probe done\n");

	return 0;
}

static const struct i2c_device_id axp15060_ids[] = {
	{ .name = "axp15060", },
	{ },
};
MODULE_DEVICE_TABLE(i2c, axp15060_ids);

static struct i2c_driver axp15060_pmic_driver = {
	.driver	= {
		.name	= "axp15060",
	},
	.probe		= axp15060_pmic_probe,
	.id_table	= axp15060_ids,
};

static int __init axp15060_init(void)
{
	return i2c_add_driver(&axp15060_pmic_driver);
}

subsys_initcall(axp15060_init);

MODULE_DESCRIPTION("X-Powers axp15060 voltage regulator driver");
MODULE_AUTHOR("v-xiankun.zhu <v-xiankun.zhu@horizon.ai>");
MODULE_LICENSE("GPL v2");
