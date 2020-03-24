/*
 * Copyright (c) 2020
 * irps5401.c - Voltage regulation for Infineon IRPS5401 PMUs
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/bitops.h>
#include "pmbus.h"

#define IRPS5401_SW_FUNC (PMBUS_HAVE_VIN | PMBUS_HAVE_IIN | \
			  PMBUS_HAVE_STATUS_INPUT | \
			  PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT | \
			  PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT | \
			  PMBUS_HAVE_PIN | PMBUS_HAVE_POUT | \
			  PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP)

#define IRPS5401_LDO_FUNC (PMBUS_HAVE_VIN | \
			   PMBUS_HAVE_STATUS_INPUT | \
			   PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT | \
			   PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT | \
			   PMBUS_HAVE_PIN | PMBUS_HAVE_POUT | \
			   PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP)

#define IRPS5401_VOLTAGE_NUM 846
#define IRPS5401_VSET_MASK   0x3FF

enum {
	ID_DCDC1,
	ID_DCDC2,
	ID_DCDC3,
	ID_DCDC4,
	ID_LDO0,
};

static const struct regulator_linear_range irps5401_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(0, 0, 846, 3906),
};


static int pmbus_regulator_set_voltage(struct regulator_dev *rdev, unsigned sel)
{
	struct device *dev = rdev_get_dev(rdev);
	struct i2c_client *client = to_i2c_client(dev->parent);
	u8 page = rdev_get_id(rdev);
	int ret;

	dev_dbg(&client->dev, "pmbus set voltage sel: 0x%x", sel);

	//sel <<= ffs(rdev->desc->vsel_mask) - 1;
	sel &= rdev->desc->vsel_mask;
	dev_dbg(&client->dev, "pmbus set voltage sel: 0x%x", sel);

	ret = pmbus_write_word_data(client, page, PMBUS_VOUT_COMMAND, sel);
	if (ret < 0) {
		dev_err(&client->dev, "pmbus set voltage error, %d", ret);
	}
	return ret;
}

static int pmbus_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct device *dev = rdev_get_dev(rdev);
	struct i2c_client *client = to_i2c_client(dev->parent);
	u8 page = rdev_get_id(rdev);
	int val;

	val = pmbus_read_word_data(client, page, PMBUS_READ_VOUT);
	dev_dbg(&client->dev, "pmbus get voltage value: 0x%x", val);
	if (val < 0) {
		dev_err(&client->dev, "pmbus get voltage error, %d", val);
		return val;
	}

	val &= rdev->desc->vsel_mask;
	//val >>= ffs(rdev->desc->vsel_mask) - 1;
	dev_dbg(&client->dev, "pmbus convent voltage value: 0x%x", val);

	return val;
}

static int pmbus_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct device *dev = rdev_get_dev(rdev);
	struct i2c_client *client = to_i2c_client(dev->parent);
	u8 page = rdev_get_id(rdev);
	int ret;

	ret = pmbus_read_byte_data(client, page, PMBUS_OPERATION);
	if (ret < 0)
		return ret;

	return !!(ret & PB_OPERATION_CONTROL_ON);
}

static int _pmbus_regulator_on_off(struct regulator_dev *rdev, bool enable)
{
	struct device *dev = rdev_get_dev(rdev);
	struct i2c_client *client = to_i2c_client(dev->parent);
	u8 page = rdev_get_id(rdev);

	return pmbus_update_byte_data(client, page, PMBUS_OPERATION,
				      PB_OPERATION_CONTROL_ON,
				      enable ? PB_OPERATION_CONTROL_ON : 0);
}

static int pmbus_regulator_enable(struct regulator_dev *rdev)
{
	return _pmbus_regulator_on_off(rdev, 1);
}

static int pmbus_regulator_disable(struct regulator_dev *rdev)
{
	return _pmbus_regulator_on_off(rdev, 0);
}


const struct regulator_ops pmbus_regulator_irps5401_ops = {
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
	.get_voltage_sel = pmbus_regulator_get_voltage,
	.set_voltage_sel	= pmbus_regulator_set_voltage,
	.enable = pmbus_regulator_enable,
	.disable = pmbus_regulator_disable,
	.is_enabled = pmbus_regulator_is_enabled,
};

#define PMBUS_REGULATOR_IRPS(_name, _id, _num)				\
	[ID_##_id] = {						\
		.name = _name,				\
		.id = (ID_##_id),					\
		.of_match = of_match_ptr(_name # _num),		\
		.regulators_node = of_match_ptr("regulators"),	\
		.linear_ranges		= irps5401_voltage_ranges,	\
		.n_linear_ranges	= ARRAY_SIZE(irps5401_voltage_ranges), \
		.n_voltages		= IRPS5401_VOLTAGE_NUM,			\
		.vsel_reg		= PMBUS_VOUT_COMMAND, \
		.vsel_mask		= IRPS5401_VSET_MASK,		\
		.ops = &pmbus_regulator_irps5401_ops,			\
		.type = REGULATOR_VOLTAGE,			\
		.owner = THIS_MODULE,				\
	}

static const struct regulator_desc irps5401_reg_desc[] = {
	PMBUS_REGULATOR_IRPS("DCDC", DCDC1, 1),
	PMBUS_REGULATOR_IRPS("DCDC", DCDC2, 2),
	PMBUS_REGULATOR_IRPS("DCDC", DCDC3, 3),
	PMBUS_REGULATOR_IRPS("DCDC", DCDC4, 4),
	//PMBUS_REGULATOR_IRPS("LDO", LDO0, 0),
};

static int irps5401_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct pmbus_driver_info *info;
	int ret = 0;

	printk("irps5401 probe start\n");

	info = devm_kzalloc(&client->dev, \
			sizeof(struct pmbus_driver_info), GFP_KERNEL);
	if (!info) {
		return -ENOMEM;
		dev_err(&client->dev, "malloc pmbus info error!");
	}
	info->pages = 4;
	info->func[0] = IRPS5401_SW_FUNC;
	info->func[1] = IRPS5401_SW_FUNC;
	info->func[2] = IRPS5401_SW_FUNC;
	info->func[3] = IRPS5401_SW_FUNC;
	//info->func[4] = IRPS5401_LDO_FUNC;
	info->num_regulators = info->pages;
	info->reg_desc = irps5401_reg_desc;

	if (info->num_regulators > ARRAY_SIZE(irps5401_reg_desc)) {
		dev_err(&client->dev, "num_regulators too large!");
		info->num_regulators = ARRAY_SIZE(irps5401_reg_desc);
	}

	ret = pmbus_do_probe(client, id, info);
	if (ret != 0 )
		dev_err(&client->dev, "irps5401 probe error : %d!", ret);
	else
		dev_err(&client->dev, "irps5401 probe done!");

	return ret;
}

static const struct i2c_device_id irps5401_id[] = {
	{"irps5401", 0},
	{},
};

static const struct of_device_id irps5401_of_match[] = {
	{ .compatible = "infineon,irps5401" },
	{},
};

static struct i2c_driver irps5401_driver = {
	.driver = {
		.name = "irps5401",
		.of_match_table = of_match_ptr(irps5401_of_match),
	},
	.probe = irps5401_probe,
	.remove = pmbus_do_remove,
	.id_table = irps5401_id,
};

module_i2c_driver(irps5401_driver);
/*
static int __init irps5401_init(void)
{
	return i2c_add_driver(&irps5401_driver);
}

subsys_initcall(irps5401_init);
*/

MODULE_AUTHOR("xiankun zhu");
MODULE_DESCRIPTION("PMBus driver for Power Management Unit IRPS5401");
MODULE_LICENSE("GPL");
