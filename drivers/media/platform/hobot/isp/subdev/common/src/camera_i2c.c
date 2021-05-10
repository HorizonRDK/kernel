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

	pr_info("camera_i2c_open come in sensor_name %s sensor_addr 0x%x\n",
			sensor_name, sensor_addr);
	strncpy(camera_mod[port]->board_info.type, sensor_name,
			sizeof(camera_mod[port]->board_info.type));
	camera_mod[port]->board_info.addr = sensor_addr;
	for (i = 0; i < CAMERA_TOTAL_NUMBER; i++) {
		if ((camera_mod[i]->camera_param.sensor_addr == sensor_addr) &&
			(camera_mod[i]->camera_param.bus_num == i2c_bus) &&
			(camera_mod[i]->client)) {
				camera_mod[port]->client = camera_mod[i]->client;
				return 0;
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
	camera_mod[port]->client = i2c_new_device(adap, &camera_mod[port]->board_info);
    if (!camera_mod[port]->client) {
		i2c_put_adapter(adap);
        return -ENOMEM;
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
	i2c_unregister_device(camera_mod[port]->client);
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
	msg[1].len = count;
	msg[1].buf = buf;

	ret = i2c_transfer(adap, msg, 2);
	if(ret != 2) {
		pr_err("[%d]read failed reg_addr 0x%x! bit_width %d count %d",
			__LINE__, reg_addr, bit_width, count);
		ret = -1;
	}

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

/* sensor register read */
int camera_user_i2c_read(struct i2c_client *client,
			uint32_t addr, uint16_t reg, uint8_t *val)
{
	struct i2c_msg msg[2];
	uint8_t buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;

	msg[0].addr = addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = buf[0];
		return 0;
	}

	pr_info("x2_camera i2c read error addr: 0x%x"
			"reg: 0x%x ret %d !!!\n", addr, reg, ret);

	return ret;
}

int camera_user_i2c_read_byte(struct i2c_client *client,
			uint32_t addr, uint8_t reg, uint8_t *val)
{
	struct i2c_msg msg[2];
	uint8_t buf[1];
	int ret;

	buf[0] = reg & 0xFF;

	msg[0].addr = addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = buf[0];
		return 0;
	}

	pr_info("x2_camera i2c read error addr: 0x%x"
			"reg: 0x%x ret %d !!!\n", addr, reg, ret);

	return ret;
}

int camera_user_i2c_write_byte(struct i2c_client *client,
		uint32_t addr, uint8_t reg, uint8_t val)
{
	struct i2c_msg msg;
	uint8_t buf[2];
	int ret;

	buf[0] = reg & 0xFF;
	buf[1] = val;

	msg.addr = addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	pr_info("x2_camera i2c write error addr: 0%x"
			"reg: 0x%x ret %d !!!\n", addr, reg, ret);

	return ret;
}

/* sensor register write */
int camera_user_i2c_write(struct i2c_client *client, uint32_t addr,
		uint16_t reg, uint8_t val)
{
	struct i2c_msg msg;
	uint8_t buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val;

	msg.addr = addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	pr_info("x2_camera i2c write error addr: 0%x"
			"reg: 0x%x ret %d !!!\n", addr, reg, ret);

	return ret;
}


