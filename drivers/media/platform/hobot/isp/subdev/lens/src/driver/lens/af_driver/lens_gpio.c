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
 *    @file      lens_gpio.c
 *    @brief     provide gpio control
 *    @details
 *    @mainpage
 *    @author    IE&E
 *    @email     yongyong.duan@horizon.ai
 *    @version   v-1.0.0
 *    @date      2020-1-17
 *    @license
 *    @copyright
 *********************************************************************/

#include <linux/gpio.h>
#include <linux/delay.h>
#include "acamera_logger.h"
#include "lens_gpio.h"
#include "lens_ops.h"

#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_LENS
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_LENS
#endif

struct gpio_group {
	uint16_t a1;
	uint16_t a2;
	uint16_t b1;
	uint16_t b2;
};

static int m_delay_time = 800;

static void montor_standby(uint16_t A1, uint16_t A2, uint16_t B1, uint16_t B2)
{
	uint32_t a1 = (uint32_t) A1;
	uint32_t a2 = (uint32_t) A2;
	uint32_t b1 = (uint32_t) B1;
	uint32_t b2 = (uint32_t) B2;

	LOG(LOG_DEBUG, "montor standy");
	gpio_set_value(a1, 0);
	gpio_set_value(a2, 0);
	gpio_set_value(b1, 0);
	gpio_set_value(b2, 0);
}

static void montor_step_one(uint16_t A1, uint16_t A2, uint16_t B1, uint16_t B2)
{
	uint32_t a1 = (uint32_t) A1;
	uint32_t a2 = (uint32_t) A2;
	uint32_t b1 = (uint32_t) B1;
	uint32_t b2 = (uint32_t) B2;

	LOG(LOG_DEBUG, "montor step one");
	gpio_set_value(a1, 1);
	gpio_set_value(a2, 0);
	gpio_set_value(b1, 0);
	gpio_set_value(b2, 1);
}

static void montor_step_two(uint16_t A1, uint16_t A2, uint16_t B1, uint16_t B2)
{
	uint32_t a1 = (uint32_t) A1;
	uint32_t a2 = (uint32_t) A2;
	uint32_t b1 = (uint32_t) B1;
	uint32_t b2 = (uint32_t) B2;

	LOG(LOG_DEBUG, "montor step two");
	gpio_set_value(a1, 1);
	gpio_set_value(a2, 0);
	gpio_set_value(b1, 1);
	gpio_set_value(b2, 0);
}

static void montor_step_three(uint16_t A1, uint16_t A2, uint16_t B1, uint16_t B2)
{
	uint32_t a1 = (uint32_t) A1;
	uint32_t a2 = (uint32_t) A2;
	uint32_t b1 = (uint32_t) B1;
	uint32_t b2 = (uint32_t) B2;

	LOG(LOG_DEBUG, "montor step three");
	gpio_set_value(a1, 0);
	gpio_set_value(a2, 1);
	gpio_set_value(b1, 1);
	gpio_set_value(b2, 0);
}

static void montor_step_four(uint16_t A1, uint16_t A2, uint16_t B1, uint16_t B2)
{
	uint32_t a1 = (uint32_t) A1;
	uint32_t a2 = (uint32_t) A2;
	uint32_t b1 = (uint32_t) B1;
	uint32_t b2 = (uint32_t) B2;

	LOG(LOG_DEBUG, "montor step four");
	gpio_set_value(a1, 0);
	gpio_set_value(a2, 1);
	gpio_set_value(b1, 0);
	gpio_set_value(b2, 1);
}

static void montor_forward(uint16_t A1, uint16_t A2, uint16_t B1, uint16_t B2, uint32_t num)
{
	uint32_t count = 0;

	montor_step_one(A1, A2, B1, B2);
	udelay(2*1000);
	for (count = 0; count < num; count++) {
		montor_step_one(A1, A2, B1, B2);
		udelay(m_delay_time);
		montor_step_two(A1, A2, B1, B2);
		udelay(m_delay_time);
		montor_step_three(A1, A2, B1, B2);
		udelay(m_delay_time);
		montor_step_four(A1, A2, B1, B2);
		udelay(m_delay_time);
	}
}

static void montor_back(uint16_t A1, uint16_t A2, uint16_t B1, uint16_t B2, uint32_t num)
{
	uint32_t count = 0;

	montor_step_four(A1, A2, B1, B2);
	udelay(2*1000);
	for (count = 0; count < num; count++) {
		montor_step_four(A1, A2, B1, B2);
		udelay(m_delay_time);
		montor_step_three(A1, A2, B1, B2);
		udelay(m_delay_time);
		montor_step_two(A1, A2, B1, B2);
		udelay(m_delay_time);
		montor_step_one(A1, A2, B1, B2);
		udelay(m_delay_time);
	}
}

//basic_func
void motor_gpio_move(void *ctx, void *param, uint32_t pos)
{
	struct gpio_group *dev = NULL;
	struct motor_info *info = NULL;

	if ((ctx == NULL) || (param == NULL))
		return;

	dev = ctx;
	info = param;
	if (info->curr_pos < pos) {
		LOG(LOG_DEBUG, "move forward %d", pos);
		montor_forward(dev->a1, dev->a2, dev->b1, dev->b2, 15);
	} else {
		LOG(LOG_DEBUG, "move back %d", pos);
		montor_back(dev->a1, dev->a2, dev->b1, dev->b2, 15);
	}
	montor_standby(dev->a1, dev->a2, dev->b1, dev->b2);
}

void motor_gpio_stop(void *ctx, void *param)
{
}

void motor_gpio_write_reg(void *ctx, void *param, uint32_t addr, uint32_t data)
{
}

uint32_t motor_gpio_read_reg(void *ctx, void *param, uint32_t addr)
{
	return 0;
}

uint8_t motor_gpio_is_moving(void *ctx)
{
	return 0;
}

static struct basic_control_ops basic_gpio_ops = {
	.move = motor_gpio_move,
	.stop = motor_gpio_stop,
	.is_moving = motor_gpio_is_moving,
	.write_reg = motor_gpio_write_reg,
	.read_reg = motor_gpio_read_reg,
};

static int local_gpio_request(uint16_t gpio_num, const char *name)
{
	int ret = 0;
	char gpio_name[20];
	uint32_t gpio = (uint32_t)gpio_num;

	memset(gpio_name, 0, 20);
	snprintf(gpio_name, 20, "%s_gpio_%d", name, gpio);

	LOG(LOG_DEBUG, "gpio %d, name is %s !", gpio, gpio_name);
	ret = gpio_request(gpio, gpio_name);
	if (ret < 0) {
		LOG(LOG_ERR, "gpio %d request failed!", gpio);
		goto err_request;
	}
	ret = gpio_direction_output(gpio, 0);
	if (ret < 0) {
		LOG(LOG_ERR, "gpio %d set direction failed!", gpio);
		gpio_free(gpio);
		goto err_set;
	}

	LOG(LOG_ERR, "gpio %d request success, name is %s !", gpio, gpio_name);

	return ret;
err_set:
	gpio_free(gpio);
err_request:
	return ret;
}

void lens_gpio_release(uint16_t gpio_a1, uint16_t gpio_a2,
	uint16_t gpio_b1, uint16_t gpio_b2)
{
	gpio_free(gpio_a1);
	gpio_free(gpio_a2);
	gpio_free(gpio_b1);
	gpio_free(gpio_b2);
}

int lens_gpio_request(uint16_t gpio_a1, uint16_t gpio_a2,
	uint16_t gpio_b1, uint16_t gpio_b2, const char *name,
	struct basic_control_ops **ops)
{
	int ret = 0;

	LOG(LOG_DEBUG, "start request gpio driver!");
	ret = local_gpio_request(gpio_a1, name);
	if (ret < 0) {
		goto err_a1;
	}

	ret = local_gpio_request(gpio_a2, name);
	if (ret < 0) {
		goto err_a2;
	}

	ret = local_gpio_request(gpio_b1, name);
	if (ret < 0) {
		goto err_b1;
	}

	ret = local_gpio_request(gpio_b2, name);
	if (ret < 0) {
		goto err_b2;
	}

	*ops = &basic_gpio_ops;

	return ret;
err_b2:
	gpio_free(gpio_b1);
err_b1:
	gpio_free(gpio_a2);
err_a2:
	gpio_free(gpio_a1);
err_a1:
	return ret;
}

