/*********************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
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
 *
 *    @file      lens_i2c.c
 *    @brief     provide i2c control
 *    @details 
 *    @mainpage 
 *    @author    IE&E
 *    @email     yongyong.duan@horizon.ai
 *    @version   v-1.0.0
 *    @date      2020-1-17
 *    @license
 *    @copyright
 *********************************************************************/

#include "acamera_logger.h"
#include "lens_i2c.h"
#include "lens_ops.h"

#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_LENS
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_LENS
#endif

static struct i2c_client *lens_i2c_open(uint32_t i2c_chn,
	struct i2c_board_info *board_info)
{
	struct i2c_client *client = NULL;
	struct i2c_adapter *adap;
	int minor = 0;

	minor = i2c_chn;

	adap = i2c_get_adapter(minor);
	if (!adap) {
		LOG(LOG_ERR, "can not get i2c_adapter");
		return NULL;
	}

	client = i2c_new_device(adap, board_info);
	if (!client) {
		i2c_put_adapter(adap);
		return NULL;
	}

	LOG(LOG_INFO, "the %s is open success !", client->name);

	return client;
}

static int lens_i2c_read(const struct i2c_client *client, uint16_t reg_addr,
	uint8_t bit_width, char *buf, size_t count)
{
	char tmp[4];
	int ret = 0;

	if (count > 100)
		count = 100;

	if (!client) {
		LOG(LOG_ERR, "can not get client");
		return -ENOMEM;
	}

	if (bit_width == 8) {
		tmp[0] = (char)(reg_addr & 0xff);
		ret = i2c_master_send(client, tmp, 1);
		if (ret != 1)
			goto failed;
	} else {
		tmp[0] = (char)((reg_addr >> 8) & 0xff);
		tmp[1] = (char)(reg_addr & 0xff);
		ret = i2c_master_send(client, tmp, 2);
		if (ret != 2)
			goto failed;
	}

	if (ret < 0)
		return ret;

	ret = i2c_master_recv(client, buf, count);
	if(ret != count) {
		LOG(LOG_ERR, "read failed !");
		ret = -1;
	}

	return ret;
failed:
	return -1;
}

static int lens_i2c_write(const struct i2c_client *client, uint16_t reg_addr,
	uint8_t bit_width, const char *buf, size_t count)
{
	int ret;
	char tmp[102];

	if (count > 100)
		count = 100;
	if (!client) {
		LOG(LOG_ERR, "can not get client");
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

	ret = i2c_master_send(client, tmp, count);

	if (ret != count) {
		LOG(LOG_INFO, "write failed !");
		ret = -1;
	}
	return ret;
}


//basic_func
void motor_i2c_move(void *ctx, void *param, uint32_t pos)
{
	int ret = 0;
	char buf[4] = {0, 0, 0, 0};
	struct i2c_client *client = (struct i2c_client *)ctx;

	buf[0] = (char)(pos & 0xff);
	buf[1] = (char)((pos >> 8) & 0xff);
	buf[2] = (char)((pos >> 16) & 0xff);
	buf[3] = (char)((pos >> 24) & 0xff);

	ret = lens_i2c_write(client, 0x0000, 8, buf, 1);
}

void motor_i2c_stop(void *ctx, void *param)
{
	int ret = 0;
	char buf[4] = {0, 0, 0, 0};

	struct i2c_client *client = (struct i2c_client *)ctx;
	ret = lens_i2c_write(client, 0x0000, 8, buf, 1);
}

void motor_i2c_write_reg(void *ctx, void *param, uint32_t addr, uint32_t data)
{
	int ret = 0;
	char buf[4] = {0, 0, 0, 0};
	struct i2c_client *client = (struct i2c_client *)ctx;

	buf[0] = (char)(data & 0xff);
	buf[1] = (char)((data >> 8) & 0xff);
	buf[2] = (char)((data >> 16) & 0xff);
	buf[3] = (char)((data >> 24) & 0xff);

	ret = lens_i2c_write(client, addr, 8, buf, 1);
}

uint32_t motor_i2c_read_reg(void *ctx, void *param, uint32_t addr)
{
	int ret = 0;
	uint32_t data = 0;
	char buf[4] = {0, 0, 0, 0};

	struct i2c_client *client = (struct i2c_client *)ctx;
	ret = lens_i2c_read(client, (uint16_t)addr, 8, buf, 1);

	data = buf[3];
	data = ((data << 8) | buf[2]);
	data = ((data << 8) | buf[1]);
	data = ((data << 8) | buf[0]);

	return data;
}

uint8_t motor_i2c_is_moving(void *ctx)
{
	//struct i2c_client *client = (struct i2c_client *)ctx;
	return 0;
}

static struct basic_control_ops basic_i2c_ops = {
	.move = motor_i2c_move,
	.stop = motor_i2c_stop,
	.is_moving = motor_i2c_is_moving,
	.write_reg = motor_i2c_write_reg,
	.read_reg = motor_i2c_read_reg,
};

int lens_i2c_release(struct i2c_client *client)
{
	if (!client)
		return -ENOMEM;

	i2c_unregister_device(client);
	client = NULL;

	return 0;
}

struct i2c_client *lens_i2c_request(uint32_t i2c_chn, uint32_t i2c_addr,
	const char *name, struct basic_control_ops **ops)
{
	struct i2c_board_info temp_info;
	struct i2c_client *client;

	LOG(LOG_DEBUG, "start request i2c driver!");

	snprintf(temp_info.type, I2C_NAME_SIZE, "%s", name);
	temp_info.addr = i2c_addr;
	client = lens_i2c_open(i2c_chn, &temp_info);

	if (client != NULL) {
		*ops = &basic_i2c_ops;
	}
	return client;
}

