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

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/pwm.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "hobot_lt8618sxb.h"


int lt8618sxb_reset_pin = -1;
struct x2_lt8618sxb_s* g_x2_lt8618sxb;


static int x2_lt8618sxb_probe(struct i2c_client* client,
	const struct i2c_device_id* id)
{
	struct i2c_adapter* adapter = client->adapter;
	int ret = 0;

	pr_debug("x2 lt8618sxb probe start.\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	g_x2_lt8618sxb = devm_kzalloc(&client->dev, sizeof(struct x2_lt8618sxb_s),
		GFP_KERNEL);
	if (!g_x2_lt8618sxb)
		return -ENOMEM;
	g_x2_lt8618sxb->client = client;

	mutex_init(&g_x2_lt8618sxb->lt8618sxb_mutex);

	ret = LT8618SX_Chip_ID();
	if (ret != 0) {
		pr_err("not found lt8618sxb device, exit probe!!!\n");
		goto err;
	}

	display_type = HDMI_TYPE;

	ret = of_property_read_u32(client->dev.of_node, "rst_pin",
			&lt8618sxb_reset_pin);
	if (ret) {
		//dev_err(&client->dev, "Failed to get rst_pin %d\n", ret);
	} else {
		ret = gpio_request(lt8618sxb_reset_pin, "lt8618sxb_rst_pin");
		if (ret) {
			//pr_err("%s() Err get reset pin ret= %d\n", __func__, ret);
		}
	}

	i2c_set_clientdata(client, g_x2_lt8618sxb);

	client->flags = I2C_CLIENT_SCCB;
	LT8618SXB_DEBUG("chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	LT8618SX_Initial();

	pr_debug("x2_lt8618sxb probe OK!!!\n");
	return 0;

err:
	if (g_x2_lt8618sxb)
		devm_kfree(&client->dev, g_x2_lt8618sxb);
	return ret;
}

static int x2_lt8618sxb_remove(struct i2c_client* client)
{
	struct x2_lt8618sxb_s* x2_lt8618sxb = i2c_get_clientdata(client);
	if (x2_lt8618sxb)
		devm_kfree(&client->dev, x2_lt8618sxb);
	return 0;
}

static const struct of_device_id x2_lt8618sxb_of_match[] = {
	{ .compatible = "lt,lt8618sxb", .data = NULL },
	{}
};
MODULE_DEVICE_TABLE(of, x2_lt8618sxb_of_match);

static struct i2c_driver x2_lt8618sxb_driver = {
	.driver = {
		.name = "x2_lt8618sxb",
		.of_match_table = x2_lt8618sxb_of_match,
	},
	.probe = x2_lt8618sxb_probe,
	.remove = x2_lt8618sxb_remove,
};
module_i2c_driver(x2_lt8618sxb_driver);

MODULE_DESCRIPTION("x2_lt8618sxb_converter_driver");
MODULE_AUTHOR("GuoRui <rui.guo@hobot.cc>");
MODULE_LICENSE("GPL v2");
