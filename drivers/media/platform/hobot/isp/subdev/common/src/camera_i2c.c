/*   Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include "inc/camera_dev.h"
#include "inc/camera_subdev.h"
#include "inc/camera_i2c.h"

extern camera_charmod_s *camera_mod[CAMERA_TOTAL_NUMBER];

int camera_i2c_open(uint32_t port, uint32_t i2c_bus,
		char *sensor_name, uint32_t sensor_addr)
{
	struct i2c_adapter *adap;
	uint32_t minor = 0, i;

	pr_info("camera_i2c_open come in port %d\n", port);
	if (sensor_name != NULL) {
		pr_info("camera_i2c_open come in sensor_name %s sensor_addr 0x%x\n",
				sensor_name, sensor_addr);
		strncpy(camera_mod[port]->board_info.type, sensor_name,
				sizeof(camera_mod[port]->board_info.type));
		camera_mod[port]->board_info.addr = (uint16_t)sensor_addr;
		for (i = 0; i < CAMERA_TOTAL_NUMBER; i++) {
			if ((camera_mod[i]->camera_param.sensor_addr == sensor_addr) &&
					(camera_mod[i]->camera_param.bus_num == i2c_bus) &&
					(camera_mod[i]->client)) {
				camera_mod[port]->client = camera_mod[i]->client;
				return 0;
			}
		}
	}
	if(camera_mod[port]->client) {
		camera_i2c_release(port);
	}
	minor = i2c_bus;
	adap = i2c_get_adapter(minor);
	if (!adap) {
		pr_err("can not get i2c_adapter");
		return -ENODEV;
	}
	camera_mod[port]->client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!camera_mod[port]->client) {
		i2c_put_adapter(adap);
		return -ENOMEM;
	}
	camera_mod[port]->client->adapter = adap;
	if (sensor_name != NULL) {
		camera_mod[port]->client->addr = (uint16_t)sensor_addr;
	}

	pr_info("the %s is open success !", camera_mod[port]->client->name);
	return 0;
}


int camera_i2c_release(uint32_t port)
{
	int i;

	if (!camera_mod[port]->client)
		return -ENOMEM;
	for (i = 0; i < CAMERA_TOTAL_NUMBER; i++) {
		if ((camera_mod[i]->client == camera_mod[port]->client) &&
			(i != port)) {
				camera_mod[port]->client = NULL;
				return 0;
		}
	}
	kzfree(camera_mod[port]->client);
	camera_mod[port]->client = NULL;
    return 0;
}

int camera_i2c_read(uint32_t port, uint32_t reg_addr,
		uint32_t bit_width, char *buf, uint32_t count)
{
	char tmp[4];
	struct i2c_msg msg[2];
	int ret = 0;

	if (count > 100)
		count = 100;
	if (!camera_mod[port]->client) {
		pr_err("can not get client[%d]", port);
		return -ENOMEM;
	}

	struct i2c_adapter *adap = camera_mod[port]->client->adapter;

	tmp[0] = (char)((reg_addr >> 8) & 0xff);
	tmp[1] = (char)(reg_addr & 0xff);
	msg[0].len = 2;
	if (bit_width == 8) {
		tmp[0] = tmp[1];
		msg[0].len = 1;
	}

	msg[0].addr = camera_mod[port]->client->addr;
	msg[0].flags = camera_mod[port]->client->flags & I2C_M_TEN;
	msg[0].buf = (char *)tmp;

	msg[1].addr = camera_mod[port]->client->addr;
	msg[1].flags = camera_mod[port]->client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = (uint16_t)count;
	msg[1].buf = buf;

	ret = i2c_transfer(adap, msg, 2);
	if(ret != 2) {
		pr_err("[%d]read failed reg_addr 0x%x! bit_width %d count %d",
			__LINE__, reg_addr, bit_width, count);
		ret = -1;
	}

	return ret;
}

int camrea_i2c_adapter_read(uint32_t port, uint16_t slave_addr,
		uint32_t reg_addr, int bit_width,
		uint16_t *value, int count)
{
	char tmp[4];
	struct i2c_msg msg[2];
	int ret = 0;
	char buf[2];

	if (count > 2)
				count = 2;
	if (!camera_mod[port]->client) {
		pr_err("can not get client[%d]", port);
		return -ENOMEM;
	}

	struct i2c_adapter *adap = camera_mod[port]->client->adapter;

	tmp[0] = (char)((reg_addr >> 8) & 0xff);
	tmp[1] = (char)(reg_addr & 0xff);
	msg[0].len = 2;
	if (bit_width == 8) {
		tmp[0] = tmp[1];
		msg[0].len = 1;
	}

	msg[0].addr = slave_addr;
	msg[0].flags = camera_mod[port]->client->flags & I2C_M_TEN;
	msg[0].buf = (char *)tmp;

	msg[1].addr = slave_addr;
	msg[1].flags = camera_mod[port]->client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = (uint16_t)count;
	msg[1].buf = buf;

	ret = i2c_transfer(adap, msg, 2);
	if(ret != 2) {
		pr_err("[%s %d]read failed reg_addr 0x%x! bit_width %d count %d",
			__func__, __LINE__, reg_addr, bit_width, count);
		ret = -1;
	}

	if (count == 1) {
		*value = buf[0];
	} else if (count == 2) {
		*value = (uint16_t)((buf[0] << 8) | buf[1]);
	} else {
		pr_err("value count is invaild!\n");
	}
	pr_info("value = 0x%x\n", *value);
	return ret;
}

int camera_i2c_write(uint32_t port, uint32_t reg_addr, uint32_t bit_width,
		const char *buf, uint32_t count)
{
	int ret = 0, k = CAM_I2C_RETRY_MAX;
	char tmp[102];
	int ret1 = 0;
	uint32_t r_data = 0;

	if (count > 100)
		count = 100;
	if (!camera_mod[port]->client) {
		pr_err("can not get client[%d]", port);
		return -ENOMEM;
	}

	if (bit_width == 8) {
		tmp[0] = (char)(reg_addr & 0xff);
		memcpy(&tmp[1], buf, count);
		count += 1;
	} else {
		tmp[0] = (char)((reg_addr >> 8) & 0xff);
		tmp[1] = (char)(reg_addr & 0xff);
		memcpy(&tmp[2], buf, count);
		count += 2;
	}

	ret = i2c_master_send(camera_mod[port]->client, tmp, count);
	while ((ret != count) && k--) {
			mdelay(3);
		ret = i2c_master_send(camera_mod[port]->client, tmp, count);
	}
	if (ret != count) {
		pr_err("port %d write failed ! tmp: 0x%x 0x%x 0x%x 0x%x ret %d count %d",
				port, tmp[0], tmp[1], tmp[2], tmp[3], ret, count);
		ret1 = camera_i2c_read(port, reg_addr, bit_width, tmp, count);
		if(ret1 < 0) {
			pr_err("read failed port %d k %d ret1 %d count %d reg_addr 0x%x ",
				port, k, ret1, count, reg_addr);
		} else {
			r_data = (tmp[0] << 8) | tmp[1];
			pr_err("port %d ret1 %d k %d count %d reg_addr 0x%x r_data 0x%x",
				port, ret1, k, count, reg_addr, r_data);
		}
		ret = -1;
	}
	return ret;
}