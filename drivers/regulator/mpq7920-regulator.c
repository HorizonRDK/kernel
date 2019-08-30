/*
 * mpq7920-regulator.c - Voltage regulation for MPS mp5416 and mpq7920 PMUs
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
#include <linux/regulator/mpq7920.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regmap.h>

/*
 * MPQ7920 Global Register Map.
 */
#define MPQ7920_DCDC1_VSET	0x03
#define MPQ7920_DCDC1_SLOT_MODE	0x06
#define MPQ7920_DCDC2_VSET	0x07
#define MPQ7920_DCDC2_SLOT_MODE	0x0a
#define MPQ7920_DCDC3_VSET	0x0b
#define MPQ7920_DCDC3_SLOT_MODE	0x0e
#define MPQ7920_DCDC4_VSET	0x0f
#define MPQ7920_DCDC4_SLOT_MODE	0x12
#define MPQ7920_LDO2_VSET	0x14
#define MPQ7920_LDO3_VSET	0x17
#define MPQ7920_LDO4_VSET	0x1a
#define MPQ7920_LDO5_VSET	0x1d
#define MPQ7920_SYS_CTRL	0x22


/*
 * MPQ7920 Definitions.
 */
#define	MPQ7920_DCDC1_ENA		0x80	/* ON - [7] */
#define	MPQ7920_DCDC2_ENA		0x40	/* ON - [6] */
#define	MPQ7920_DCDC3_ENA		0x20	/* ON - [5] */
#define	MPQ7920_DCDC4_ENA		0x10	/* ON - [4] */
#define	MPQ7920_LDO2_ENA		0x08	/* ON - [3] */
#define	MPQ7920_LDO3_ENA		0x04	/* ON - [2] */
#define	MPQ7920_LDO4_ENA		0x02	/* ON - [1] */
#define	MPQ7920_LDO5_ENA		0x01	/* ON - [0] */
#define	MPQ7920_VSEL_MASK		0xFF	/* VSET - [7:0] */

/*
 * MPQ7920 voltage number
 */
#define	MPQ7920_VOLTAGE_NUM	255

/*
 * MP5416 Global Register Map.
 */
#define MP5416_DCDC1_VSET	0x04
#define MP5416_DCDC2_VSET	0x05
#define MP5416_DCDC3_VSET	0x06
#define MP5416_DCDC4_VSET	0x07
#define MP5416_LDO2_VSET	0x08
#define MP5416_LDO3_VSET	0x09
#define MP5416_LDO4_VSET	0x0a
#define MP5416_LDO5_VSET	0x0b

/*
 * MP5416 Field Definitions.
 */

#define	MP5416_VSEL_MASK		0x4F	/* VSET - [6:0] */
#define	MP5416_VSET_ENA			0x80	/* ON - [7] */

/*
 * MP5416 voltage number
 */
#define	MP5416_VOLTAGE_NUM		128

struct mpq7920 {
	struct regmap *regmap;
	int off_reg;
};

static const struct regmap_config mpq7920_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct regulator_linear_range mpq7920_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(400000, 0, 255, 12500),
};

static const struct regulator_linear_range mp5416_buck1_3_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(600000, 0, 128, 12500),
};

static const struct regulator_linear_range mp5416_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000, 0, 128, 25000),
};

static struct regulator_ops mpq7920_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
};

#define MPQ7920_REG(_name,_id)		\
	[ID_##_id] = {					\
		.name			= _name,			\
		.id			= ID_##_id,		\
		.type			= REGULATOR_VOLTAGE,		\
		.ops			= &mpq7920_ops,			\
		.n_voltages		= MPQ7920_VOLTAGE_NUM,		\
		.linear_ranges		= mpq7920_voltage_ranges,	\
		.n_linear_ranges	= ARRAY_SIZE(mpq7920_voltage_ranges), \
		.vsel_reg		= MPQ7920##_##_id##_VSET, \
		.vsel_mask		= MPQ7920_VSEL_MASK,		\
		.enable_reg		= MPQ7920_SYS_CTRL,	\
		.enable_mask		= (MPQ7920##_##_id##_ENA),			\
		.owner			= THIS_MODULE,			\
	}

#define MP5416_REG(_name, _id)		\
	[ID_##_id] = {					\
		.name			= _name,			\
		.id			= ID_##_id,		\
		.type			= REGULATOR_VOLTAGE,		\
		.ops			= &mpq7920_ops,			\
		.n_voltages		= MP5416_VOLTAGE_NUM,		\
		.linear_ranges		= mp5416_voltage_ranges,	\
		.n_linear_ranges	= ARRAY_SIZE(mp5416_voltage_ranges), \
		.vsel_reg		= MP5416##_##_id##_VSET, \
		.vsel_mask		= MP5416_VSEL_MASK,		\
		.enable_reg		= MP5416##_##_id##_VSET,	\
		.enable_mask		= MP5416_VSET_ENA,			\
		.owner			= THIS_MODULE,			\
	}

static const struct regulator_desc mpq7920_regulators[] = {
	MPQ7920_REG("DCDC1", DCDC1),
	MPQ7920_REG("DCDC2", DCDC2),
	MPQ7920_REG("DCDC3", DCDC3),
	MPQ7920_REG("DCDC4", DCDC4),
	MPQ7920_REG("LDO2", LDO2),
	MPQ7920_REG("LDO3", LDO3),
	MPQ7920_REG("LDO4", LDO4),
	MPQ7920_REG("LDO5", LDO5),
};

static const struct regulator_desc mp5416_regulators[] = {
	MP5416_REG("DCDC1", DCDC1),
	MP5416_REG("DCDC2", DCDC2),
	MP5416_REG("DCDC3", DCDC3),
	MP5416_REG("DCDC4", DCDC4),
	MP5416_REG("LDO2", LDO2),
	MP5416_REG("LDO3", LDO3),
	MP5416_REG("LDO4", LDO4),
	MP5416_REG("LDO5", LDO5),
};

#ifdef CONFIG_OF
static const struct of_device_id mpq7920_dt_ids[] = {
	{ .compatible = "mps,mpq7920", .data = (void *)MPQ7920 },
	{ .compatible = "mps,mp5416", .data = (void *)MP5416 },
	{ }
};
MODULE_DEVICE_TABLE(of, mpq7920_dt_ids);

static struct of_regulator_match mpq7920_matches[] = {
	[ID_DCDC1]	= { .name = "DCDC1"},
	[ID_DCDC2]	= { .name = "DCDC2"},
	[ID_DCDC3]	= { .name = "DCDC3"},
	[ID_DCDC4]	= { .name = "DCDC4"},
	[ID_LDO2]	= { .name = "LDO2"},
	[ID_LDO3]	= { .name = "LDO3"},
	[ID_LDO4]	= { .name = "LDO4"},
	[ID_LDO5]	= { .name = "LDO5"},
};

static int mpq7920_pdata_from_dt(struct device *dev,
				 struct mpq7920_platform_data *pdata,
				 unsigned long type)
{
	int matched, i, num_matches;
	struct device_node *np;
	struct mpq7920_regulator_data *regulator;
	struct of_regulator_match *matches;

	switch (type) {
	case MP5416:
	case MPQ7920:
		matches = mpq7920_matches;
		num_matches = ARRAY_SIZE(mpq7920_matches);
		break;
	default:
		dev_err(dev, "invalid device id %lu\n", type);
		return -EINVAL;
	}

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
					 sizeof(struct mpq7920_regulator_data) *
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
static inline int mpq7920_pdata_from_dt(struct device *dev,
					struct mpq7920_platform_data *pdata,
					unsigned long type)
{
	return 0;
}
#endif

static struct mpq7920_regulator_data *mpq7920_get_regulator_data(
		int id, struct mpq7920_platform_data *pdata)
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

static struct i2c_client *mpq7920_i2c_client;
static void mpq7920_power_off(void)
{
	struct mpq7920 *mpq7920;

	mpq7920 = i2c_get_clientdata(mpq7920_i2c_client);
	regmap_write(mpq7920->regmap, mpq7920->off_reg, 0x0);

	while (1);
}

static void mpq7920_slot_init(void)
{
	struct mpq7920 *mpq7920;

	mpq7920 = i2c_get_clientdata(mpq7920_i2c_client);
	regmap_write(mpq7920->regmap, MPQ7920_DCDC1_SLOT_MODE, 0x0);
	regmap_write(mpq7920->regmap, MPQ7920_DCDC2_SLOT_MODE, 0x0);
	regmap_write(mpq7920->regmap, MPQ7920_DCDC3_SLOT_MODE, 0x0);
	regmap_write(mpq7920->regmap, MPQ7920_DCDC4_SLOT_MODE, 0x0);
}

static int mpq7920_pmic_probe(struct i2c_client *client,
			      const struct i2c_device_id *i2c_id)
{
	const struct regulator_desc *regulators;
	struct mpq7920_platform_data pdata_of, *pdata;
	struct device *dev = &client->dev;
	int i, ret, num_regulators;
	struct mpq7920 *mpq7920;
	const struct regmap_config *regmap_config;
	unsigned long type;
	int off_reg;
	int retry_cnt;

	dev_info(dev, "start probe\n");
	pdata = dev_get_platdata(dev);

	if (dev->of_node && !pdata) {
		const struct of_device_id *id;

		id = of_match_device(of_match_ptr(mpq7920_dt_ids), dev);
		if (!id)
			return -ENODEV;

		type = (unsigned long) id->data;
	} else {
		type = i2c_id->driver_data;
	}

	switch (type) {
	case MP5416:
		regulators = mp5416_regulators;
		num_regulators = ARRAY_SIZE(mp5416_regulators);
		regmap_config = &mpq7920_regmap_config;
		off_reg = MP5416_DCDC4_VSET;
		break;
	case MPQ7920:
		regulators = mpq7920_regulators;
		num_regulators = ARRAY_SIZE(mpq7920_regulators);
		regmap_config = &mpq7920_regmap_config;
		off_reg = MPQ7920_SYS_CTRL;
		break;
	default:
		dev_err(dev, "invalid device id %lu\n", type);
		return -EINVAL;
	}

	if (dev->of_node && !pdata) {
		ret = mpq7920_pdata_from_dt(dev, &pdata_of, type);
		if (ret < 0)
			return ret;

		pdata = &pdata_of;
	}

	mpq7920 = devm_kzalloc(dev, sizeof(struct mpq7920), GFP_KERNEL);
	if (!mpq7920)
		return -ENOMEM;

	mpq7920->regmap = devm_regmap_init_i2c(client, regmap_config);
	if (IS_ERR(mpq7920->regmap)) {
		ret = PTR_ERR(mpq7920->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	if (of_device_is_system_power_controller(dev->of_node)) {
		mpq7920->off_reg = off_reg;
		pm_power_off = mpq7920_power_off;
	}


	/* Finally register devices */
	for (i = 0; i < num_regulators; i++) {
		const struct regulator_desc *desc = &regulators[i];
		struct regulator_config config = { };
		struct mpq7920_regulator_data *rdata;
		struct regulator_dev *rdev;
		retry_cnt = 2;

		config.dev = dev;
		config.driver_data = mpq7920;
		config.regmap = mpq7920->regmap;

		rdata = mpq7920_get_regulator_data(desc->id, pdata);
		if (rdata) {
			config.init_data = rdata->init_data;
			config.of_node = rdata->of_node;
		}

		/*QUAD:mqp7920's fist read has no ack
		after cold reset,so retry*/
		while (retry_cnt--) {
			rdev = devm_regulator_register(dev, desc, &config);
			if (!IS_ERR(rdev)) {
				dev_dbg(dev, "try to register %s,retry cnt(%d)\n",
					desc->name, retry_cnt);
				break;
			} else if (retry_cnt == 0) {
				dev_err(dev, "failed to register %s\n", desc->name);
				return PTR_ERR(rdev);
			}
		}
	}

	mpq7920_i2c_client = client;
	i2c_set_clientdata(client, mpq7920);

	if (type == MPQ7920)
		mpq7920_slot_init();

	return 0;
}

static const struct i2c_device_id mpq7920_ids[] = {
	{ .name = "mpq7920", .driver_data = MPQ7920 },
	{ .name = "mp5416", .driver_data = MP5416 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mpq7920_ids);

static struct i2c_driver mpq7920_pmic_driver = {
	.driver	= {
		.name	= "mpq7920",
	},
	.probe		= mpq7920_pmic_probe,
	.id_table	= mpq7920_ids,
};

static int __init mpq7920_init(void)
{
	return i2c_add_driver(&mpq7920_pmic_driver);
}

subsys_initcall(mpq7920_init);

MODULE_DESCRIPTION("mps mpq7920 voltage regulator driver");
MODULE_AUTHOR("Yuqian Wang <yunqian.wang@horizon.ai>");
MODULE_LICENSE("GPL v2");
