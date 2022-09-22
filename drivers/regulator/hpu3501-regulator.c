/*
 * Copyright (C) 2022 Horizon Robotics
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regmap.h>

#include "./hpu3501.h"

static struct i2c_client *hpu3501_i2c_client;

#ifdef CONFIG_OF
static const struct of_device_id hpu3501_dt_ids[] = {
	{.compatible = "hobot-pmic,hpu3501", },
	{ }
};

static struct of_regulator_match hpu3501_matches[] = {
	[ID_BUCK1] = {.name = "BUCK1"},
	[ID_BUCK2] = {.name = "BUCK2"},
	[ID_BUCK3] = {.name = "BUCK3"},
	[ID_BUCK4] = {.name = "BUCK4"},
	[ID_BUCK5] = {.name = "BUCK5"},

	[ID_LDO1] = {.name = "LDO1"},
	[ID_LDO2_CFG1] = {.name = "LDO2_CFG1"},
	[ID_LDO2_CFG2] = {.name = "LDO2_CFG2"},
	[ID_LDO3] = {.name = "LDO3"},
};

static int hpu3501_pdata_from_dt(struct device *dev,
			struct hpu3501_platform_data *pdata)
{
	int matched, i, num_matches;
	struct device_node *np;
	struct hpu3501_regulator_data *regulator;
	struct of_regulator_match *matches;

	matches = hpu3501_matches;
	num_matches = ARRAY_SIZE(hpu3501_matches);

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
					 sizeof(struct hpu3501_regulator_data) *
					 matched, GFP_KERNEL);
	if (!pdata->regulators) {
		dev_err(dev, "malloc for regulators fail, %d\n", -ENOMEM);
		return -ENOMEM;
	}

	pdata->num_regulators = matched;
	regulator = pdata->regulators;

	for (i = 0; i < num_matches; i++) {
		if (!matches[i].init_data)
			continue;
		regulator->id = i;
		regulator->name = matches[i].name;
		regulator->init_data = matches[i].init_data;
		regulator->of_node = matches[i].of_node;
		regulator++;
	}

	return 0;
}
#else
static int hpu3501_pdata_from_dt(struct device *dev,
			struct hpu3501_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static const struct regmap_config hpu3501_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

/* Buck1-Buck4: step 5mv, range: 0v and [0.6v, 1.275v] */
static const struct regulator_linear_range hpu3501_voltage_ranges1[] = {
	REGULATOR_LINEAR_RANGE(0, 0, 0, 0),
	REGULATOR_LINEAR_RANGE(600000, 1, 120, 0),
	REGULATOR_LINEAR_RANGE(605000, 121, 255, 5000),
};

/* Buck5: step 15mv, range: 0v and [0.6v, 3.825v] */
static const struct regulator_linear_range hpu3501_voltage_ranges2[] = {
	REGULATOR_LINEAR_RANGE(0, 0, 39, 0),
	REGULATOR_LINEAR_RANGE(600000, 40, 255, 15000),
};

/* Ldo1: step 10mv, range: 0v and [0.6v, 1.3v] */
static const struct regulator_linear_range hpu3501_voltage_ranges3[] = {
	REGULATOR_LINEAR_RANGE(0, 0, 0, 0),
	REGULATOR_LINEAR_RANGE(600000, 1, 60, 0),
	REGULATOR_LINEAR_RANGE(610000, 61, 129, 10000),
	REGULATOR_LINEAR_RANGE(1300000, 130, 255, 0),
};

/* Ldo2-Ldo3: step 20mv, range: 0v and [1.2v, 3.7v] */
static const struct regulator_linear_range hpu3501_voltage_ranges4[] = {
	REGULATOR_LINEAR_RANGE(0, 0, 0, 0),
	REGULATOR_LINEAR_RANGE(1200000, 1, 60, 0),
	REGULATOR_LINEAR_RANGE(1220000, 61, 184, 20000),
	REGULATOR_LINEAR_RANGE(3700000, 185, 255, 0),
};

static struct regulator_ops hpu3501_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
};

#define HPU3501_REG(_name, _id, _linear, _step, _vset_mask)		\
	[ID_##_id] = {					\
		.name			= _name,			\
		.id			= ID_##_id,		\
		.type			= REGULATOR_VOLTAGE,		\
		.ops			= &hpu3501_ops,			\
		.n_voltages		= HPU3501_VOLTAGE_NUM##_step,		\
		.linear_ranges		= hpu3501_voltage_ranges##_linear,	\
		.n_linear_ranges	= ARRAY_SIZE(hpu3501_voltage_ranges##_linear), \
		.vsel_reg		= HPU3501##_##_id##_VSET, \
		.vsel_mask		= HPU3501_VSET_MASK##_vset_mask,		\
		.enable_reg		= HPU3501_ON_OFF_CTRL,	\
		.enable_mask		= HPU3501##_##_id##_ENA_BIT,			\
		.disable_val		= HPU3501_POWER_OFF,			\
		.owner			= THIS_MODULE,			\
	}

/*
 * name:regulator name used
 * id: regulator id, define in hpu3501.h without prefix "ID_"
 * linear: which linear id to chose, refer to hpu3501_voltage_rangesxx array above
 * step: how many step this regulator have, refer to datasheet
 * vset_mask: refer to datasheet, when set register how many bits are used to control
 */
static const struct regulator_desc hpu3501_regulators[] = {
	HPU3501_REG("BUCK1", BUCK1, 1, 256, 8),
	HPU3501_REG("BUCK2", BUCK2, 1, 256, 8),
	HPU3501_REG("BUCK3", BUCK3, 1, 256, 8),
	HPU3501_REG("BUCK4", BUCK4, 1, 256, 8),
	HPU3501_REG("BUCK5", BUCK5, 2, 256, 8),

	HPU3501_REG("LDO1", LDO1, 3, 256, 8),
	HPU3501_REG("LDO2_CFG1", LDO2_CFG1, 4, 256, 8),
	HPU3501_REG("LDO2_CFG2", LDO2_CFG2, 4, 256, 8),
	HPU3501_REG("LDO3", LDO3, 4, 256, 8),
};

static struct hpu3501_regulator_data *hpu3501_get_regulator_data(
		int id, struct hpu3501_platform_data *pdata)
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

static void hpu3501_board_init(struct hpu3501 *hpu3501)
{
	if (hpu3501->en_pin_map != 0)
	regmap_write(hpu3501->regmap, HPU3501_EN_PIN_RMPR,
		hpu3501->en_pin_map);

	if (hpu3501->fault_cfgr != 0)
	regmap_write(hpu3501->regmap, HPU3501_FAULT_CFGR,
		hpu3501->fault_cfgr);

	if (hpu3501->ocp_cfg1r != 0)
	regmap_write(hpu3501->regmap, HPU3501_OCP_CFG1R,
		hpu3501->ocp_cfg1r);

	if (hpu3501->ocp_cfg2r != 0)
	regmap_write(hpu3501->regmap, HPU3501_OCP_CFG2R,
		hpu3501->ocp_cfg2r);

	return;
}

static int hpu3501_pmic_probe(struct i2c_client *client,
			const struct i2c_device_id *i2c_id)
{
	const struct regulator_desc *regulators;
	struct hpu3501_platform_data pdata_of, *pdata = NULL;
	struct device *dev = &client->dev;
	struct hpu3501 *hpu3501 = NULL;
	const struct regmap_config *regmap_config;
	int i, ret, num_regulators;
	pdata = dev_get_platdata(dev);

	regulators = hpu3501_regulators;
	num_regulators = ARRAY_SIZE(hpu3501_regulators);
	regmap_config = &hpu3501_regmap_config;

	hpu3501 = devm_kzalloc(dev, sizeof(struct hpu3501), GFP_KERNEL);
	if (!hpu3501) {
		dev_err(dev, "No memory for hpu3501 : %d\n", -ENOMEM);
		return -ENOMEM;
	}

	/* dts parse */
	if (dev->of_node && !pdata) {
		ret = hpu3501_pdata_from_dt(dev, &pdata_of);
		if (ret < 0) {
			dev_err(dev, "Get pdata_of from dt error, %d\n", ret);
			return ret;
		}

		pdata = &pdata_of;

		hpu3501->master = of_property_read_bool(dev->of_node, "master");

		if(of_property_read_u32(dev->of_node, "en_pin_map",
				&hpu3501->en_pin_map)) {
			dev_info(dev, "en_pin_map will be default value\n");
		}

		if(of_property_read_u32(dev->of_node, "fault_cfgr",
				&hpu3501->fault_cfgr)) {
			dev_info(dev, "fault_cfgr will be default value\n");
		}

		if(of_property_read_u32(dev->of_node, "ocp_cfg1r",
				&hpu3501->ocp_cfg1r)) {
			dev_info(dev, "ocp_cfg1r will be default value\n");
		}

		if(of_property_read_u32(dev->of_node, "ocp_cfg2r",
				&hpu3501->ocp_cfg2r)) {
			dev_info(dev, "ocp_cfg2r will be default value\n");
		}
	} else {
		return -ENODEV;
	}

	hpu3501->regmap = devm_regmap_init_i2c(client, regmap_config);
	if (IS_ERR(hpu3501->regmap)) {
		ret = PTR_ERR_OR_ZERO(hpu3501->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	/* Finally register devices */
	for (i = 0; i < num_regulators; i++) {
		const struct regulator_desc *desc = &regulators[i];
		struct regulator_config config = { };
		struct hpu3501_regulator_data *rdata;
		struct regulator_dev *rdev;

		config.dev = dev;
		config.driver_data = hpu3501;
		config.regmap = hpu3501->regmap;

		rdata = hpu3501_get_regulator_data(desc->id, pdata);
		if (!rdata)
			continue;

		config.init_data = rdata->init_data;
		config.of_node = rdata->of_node;

		rdev = devm_regulator_register(dev, desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(dev, "failed to register %s\n", desc->name);
			return PTR_ERR_OR_ZERO(rdev);
		}
	}

	hpu3501_i2c_client = client;

	i2c_set_clientdata(hpu3501_i2c_client, hpu3501);

	hpu3501_board_init(hpu3501);

	return 0;
}

static const struct i2c_device_id hpu3501_ids[] = {
	{ .name = "hpu3501", },
	{ },
};
MODULE_DEVICE_TABLE(i2c, hpu3501_ids);

static struct i2c_driver hpu3501_pmic_driver = {
	.driver	= {
		.name	= "hpu3501",
	},
	.probe		= hpu3501_pmic_probe,
	.id_table	= hpu3501_ids,
};

static int __init hpu3501_init(void)
{
	int ret;
	ret = i2c_add_driver(&hpu3501_pmic_driver);
	if (ret != 0) {
		pr_err("hpu3501_pmic_driver register i2c failed, ret =%d\n", ret);
		return ret;
	}

	return ret;
}

static void __exit hpu3501_exit(void)
{
	i2c_del_driver(&hpu3501_pmic_driver);
}

subsys_initcall(hpu3501_init);

//module_init(hpu3501_init);
//module_exit(hpu3501_exit);


MODULE_DESCRIPTION("hobot-pmic hpu3501 voltage regulator driver");
MODULE_AUTHOR("chaohang.cheng <chaohang.cheng@horizon.ai>");
MODULE_LICENSE("GPL v2");
