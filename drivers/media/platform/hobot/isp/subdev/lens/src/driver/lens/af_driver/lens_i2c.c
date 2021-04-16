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
	struct i2c_adapter *adap = NULL;
	int minor = 0;

	minor = i2c_chn;

	adap = i2c_get_adapter(minor);
	if (!adap) {
		LOG(LOG_ERR, "can not get i2c_adapter");
		return NULL;
	}

	client = i2c_new_device(adap, board_info);
	if (!client) {
		LOG(LOG_ERR, "can not get i2c_client");
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
	} else if (bit_width == 16) {
		tmp[0] = (char)((reg_addr >> 8) & 0xff);
		tmp[1] = (char)(reg_addr & 0xff);
		ret = i2c_master_send(client, tmp, 2);
		if (ret != 2)
			goto failed;
	} else {
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

static int lens_i2c_writebuf(const struct i2c_client *client, const char *buf, size_t count)
{
	int ret;

	if (count > 10)
		count = 10;
	if (!client) {
		LOG(LOG_ERR, "can not get client");
		return -ENOMEM;
	}

	ret = i2c_master_send(client, buf, count);

	if (ret != count) {
		LOG(LOG_INFO, "write failed !");
		ret = -1;
	}
	return ret;
}

static int lens_i2c_write(const struct i2c_client *client, uint16_t reg_addr,
	uint8_t bit_width, const char *buf, size_t count)
{
	int ret;
	char tmp[10];

	if (count > 10)
		count = 10;
	if (!client) {
		LOG(LOG_ERR, "can not get client");
		return -ENOMEM;
	}

	if (bit_width == 8) {
		tmp[0] = (char)(reg_addr & 0xff);
		memcpy(&tmp[1], buf, count);
		count += 1;
	} else if (bit_width == 16) {
		tmp[0] = (char)((reg_addr >> 8) & 0xff);
		tmp[1] = (char)(reg_addr & 0xff);
		memcpy(&tmp[2], buf, count);
		count += 2;
	} else {
		return -1;
	}

	ret = i2c_master_send(client, tmp, count);

	if (ret != count) {
		LOG(LOG_INFO, "write failed !");
		ret = -1;
	}
	return ret;
}

static void data_tranform_to_send(uint32_t input, char *buf, uint32_t len, uint16_t conversion)
{
	if (conversion == 0) {
		switch (len) {
		case 4:
			buf[3] = (char)((input >> 24) & 0xff);
		case 3:
			buf[2] = (char)((input >> 16) & 0xff);
		case 2:
			buf[1] = (char)((input >> 8) & 0xff);
		case 1:
			buf[0] = (char)(input & 0xff);
		}
	} else {
		switch (len) {
		case 4:
			buf[0] = (char)((input >> 24) & 0xff);
			buf[1] = (char)((input >> 16) & 0xff);
			buf[2] = (char)((input >> 8) & 0xff);
			buf[3] = (char)(input & 0xff);
		break;
		case 3:
			buf[0] = (char)((input >> 16) & 0xff);
			buf[1] = (char)((input >> 8) & 0xff);
			buf[2] = (char)(input & 0xff);
		break;
		case 2:
			buf[0] = (char)((input >> 8) & 0xff);
			buf[1] = (char)(input & 0xff);
		break;
		case 1:
			buf[0] = (char)(input & 0xff);
		break;
		}
	}
	LOG(LOG_DEBUG, "buf[0] 0x%x, buf[1] 0x%x, buf[2] 0x%x, buf[3] 0x%x \n", buf[0], buf[1], buf[2], buf[3]);
}

static void data_tranform_to_send_a(uint32_t base, uint32_t pos, char *buf, uint32_t len, uint16_t conversion)
{
	uint32_t temp = 0;
	temp = base + (pos << conversion);

	switch (len) {
		case 4:
			buf[0] = (char)((temp >> 24) & 0xff);
			buf[1] = (char)((temp >> 16) & 0xff);
			buf[2] = (char)((temp >> 8) & 0xff);
			buf[3] = (char)(temp & 0xff);
		break;
		case 3:
			buf[0] = (char)((temp >> 16) & 0xff);
			buf[1] = (char)((temp >> 8) & 0xff);
			buf[2] = (char)(temp & 0xff);
		break;
		case 2:
			buf[0] = (char)((temp >> 8) & 0xff);
			buf[1] = (char)(temp & 0xff);
		break;
		case 1:
			buf[0] = (char)(temp & 0xff);
		break;
	}
	LOG(LOG_DEBUG, "buf[0] 0x%x, buf[1] 0x%x, buf[2] 0x%x, buf[3] 0x%x \n", buf[0], buf[1], buf[2], buf[3]);
}

//basic_func
void motor_i2c_init(void *ctx, void *param)
{
}

void motor_i2c_move(void *ctx, void *param, uint32_t pos)
{
	int ret = 0;
	char buf[10] = {0};
	struct i2c_client *client = (struct i2c_client *)ctx;
	struct motor_info *info = (struct motor_info *)param;

	if (param == NULL)
		return;

	i2c_ctrl_param_t *i2c_param = (i2c_ctrl_param_t *)info->ctrl_param;

	if (i2c_param == NULL)
		return;

	if (i2c_param->reserved_2 == 1) {
		data_tranform_to_send_a(i2c_param->reg_addr, pos, buf, i2c_param->reg_width, i2c_param->conversion);
		ret = lens_i2c_writebuf(client, buf, i2c_param->reg_width);
		LOG(LOG_DEBUG, "addr 0x%x, width %d, len %d, pos %d \n", i2c_param->reg_addr,
			i2c_param->reg_width, i2c_param->reg_len, pos);
	} else {
		data_tranform_to_send(pos, buf, i2c_param->reg_len, i2c_param->conversion);
		ret = lens_i2c_write(client, i2c_param->reg_addr, i2c_param->reg_width, buf, i2c_param->reg_len);
		LOG(LOG_DEBUG, "addr 0x%x, width %d, len %d, pos %d \n", i2c_param->reg_addr,
			i2c_param->reg_width, i2c_param->reg_len, pos);
	}
}

void motor_i2c_stop(void *ctx, void *param)
{
	char buf[4] = {0, 0, 0, 0};
	struct motor_info *info = (struct motor_info *)param;
	if (param == NULL)
		return;

	i2c_ctrl_param_t *i2c_param = (i2c_ctrl_param_t *)info->ctrl_param;
	if (i2c_param == NULL)
		return;

	data_tranform_to_send(0, buf, i2c_param->reg_len, i2c_param->conversion);
	//ret = lens_i2c_write(client, i2c_param->reg_addr, i2c_param->reg_width, buf, i2c_param->reg_len);
}

void motor_i2c_write_reg(void *ctx, void *param, uint32_t addr, uint32_t data)
{
	int ret = 0;
	char buf[4] = {0, 0, 0, 0};
	struct i2c_client *client = (struct i2c_client *)ctx;
	struct motor_info *info = (struct motor_info *)param;
	if (param == NULL)
		return;

	i2c_ctrl_param_t *i2c_param = (i2c_ctrl_param_t *)info->ctrl_param;
	if (i2c_param == NULL)
		return;

	LOG(LOG_DEBUG, "addr %d, data %d\n", addr, data);
	data_tranform_to_send(data, buf, i2c_param->reg_len, i2c_param->conversion);
	ret = lens_i2c_write(client, addr, i2c_param->reg_width, buf, 1);
}

uint32_t motor_i2c_read_reg(void *ctx, void *param, uint32_t addr)
{
	int ret = 0;
	uint32_t data = 0;
	char buf[4] = {0, 0, 0, 0};
	struct i2c_client *client = (struct i2c_client *)ctx;
	struct motor_info *info = (struct motor_info *)param;
	if (param == NULL)
		return 0;

	i2c_ctrl_param_t *i2c_param = (i2c_ctrl_param_t *)info->ctrl_param;
	if (i2c_param == NULL)
		return 0;

	ret = lens_i2c_read(client, (uint16_t)addr, i2c_param->reg_width, buf, 1);
	LOG(LOG_DEBUG, "addr %d, data %d\n", addr, buf[0]);
	data = buf[0];

	return data;
}

uint8_t motor_i2c_is_moving(void *ctx)
{
	//struct i2c_client *client = (struct i2c_client *)ctx;
	return 0;
}

static struct basic_control_ops basic_i2c_ops = {
	.init = motor_i2c_init,
	.move = motor_i2c_move,
	.stop = motor_i2c_stop,
	.is_moving = motor_i2c_is_moving,
	.write_reg = motor_i2c_write_reg,
	.read_reg = motor_i2c_read_reg,
};

void lens_i2c_release(struct i2c_client *client)
{
	if (!client)
		return;

	i2c_unregister_device(client);
	client = NULL;
}

struct i2c_client *lens_i2c_request(uint32_t i2c_chn, uint32_t i2c_addr,
	const char *name, struct basic_control_ops **ops)
{
	struct i2c_board_info temp_info;
	struct i2c_client *client;

	LOG(LOG_DEBUG, "start request i2c driver!");
	memset(&temp_info, 0, sizeof(struct i2c_board_info));
	snprintf(temp_info.type, I2C_NAME_SIZE, "%s", name);
	temp_info.addr = i2c_addr;

	LOG(LOG_DEBUG, "name %s, chn %d, addr 0x%x\n", name, i2c_chn, i2c_addr);

	client = lens_i2c_open(i2c_chn, &temp_info);

	if (client != NULL) {
		*ops = &basic_i2c_ops;
	}
	return client;
}

