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
 *    @file      lens_pwm.c
 *    @brief     provide pwm control
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
#include "lens_pwm.h"
#include "lens_ops.h"
#include "lens_driver.h"

#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_SENSOR
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_SENSOR
#endif


static int lens_pwm_config(struct pwm_device *pwm_dev,
	int duty_ns, int period_ns)
{
	int ret = 0;

	ret = pwm_config(pwm_dev, duty_ns, period_ns);
	if (ret) {
		LOG(LOG_ERR, "Error config pwm!!!!");
	}

	return ret;
}

static void lens_pwm_set_period(struct pwm_device *pwm_dev, uint32_t period)
{
	pwm_set_period(pwm_dev, period);
}

static void lens_pwm_set_duty(struct pwm_device *pwm_dev, uint32_t duty)
{
	pwm_set_duty_cycle(pwm_dev, duty);
}

static int lens_pwm_enable(struct pwm_device *pwm_dev)
{
	int ret = 0;

	ret = pwm_enable(pwm_dev);

	return ret;
}

static void lens_pwm_disable(struct pwm_device *pwm_dev)
{
	pwm_disable(pwm_dev);
}

static bool lens_pwm_chectstate(const struct pwm_device *pwm_dev)
{
	return pwm_is_enabled(pwm_dev);
}

static int lens_pwm_set_pulsenum(const struct pwm_device *pwm_dev,
	uint32_t pulse_num)
{
	return 0;
}

static int lens_pwm_get_pulsenum(const struct pwm_device *pwm_dev)
{
	return 0;
}

//pwm func
void motor_pwm_move(void *ctx, void *param, uint32_t pos)
{
	int ret = 0;

	struct pwm_device *pwm_dev = (struct pwm_device *)ctx;
	lens_pwm_disable(pwm_dev);
	lens_pwm_set_duty(pwm_dev, pos);
	ret = lens_pwm_enable(pwm_dev);
}

void motor_pwm_stop(void *ctx, void *param)
{
	struct pwm_device *pwm_dev = (struct pwm_device *)ctx;
	lens_pwm_disable(pwm_dev);
}

uint8_t motor_is_moving(void *ctx)
{
	uint8_t temp = 0;
	struct pwm_device *pwm_dev = (struct pwm_device *)ctx;
	struct pwm_state state;

	pwm_get_state(pwm_dev, &state);
	temp = (uint8_t)(state.enabled);

	return temp;
}

void motor_pwm_write_reg(void *ctx, void *param, uint32_t addr, uint32_t data)
{
	return;
}

uint32_t motor_pwm_read_reg(void *ctx, void *param, uint32_t addr)
{
	return 0;
}

static struct basic_control_ops basic_pwm_ops = {
	.move = motor_pwm_move,
	.stop = motor_pwm_stop,
	.is_moving = motor_is_moving,
	.write_reg = motor_pwm_write_reg,
	.read_reg = motor_pwm_read_reg,
};

//pulse func
void motor_pulse_move(void *ctx, void *param, uint32_t pos)
{
	int ret = 0;

	struct pwm_device *pwm_dev = (struct pwm_device *)ctx;
	if (lens_pwm_chectstate(pwm_dev) == false) {
		ret = lens_pwm_set_pulsenum(pwm_dev, pos);
		ret = lens_pwm_enable(pwm_dev);
	} else {
	}
}

void motor_pulse_stop(void *ctx, void *param)
{
	struct pwm_device *pwm_dev = (struct pwm_device *)ctx;
	lens_pwm_disable(pwm_dev);
}

void motor_pulse_write_reg(void *ctx, void *param, uint32_t addr, uint32_t data)
{
	return;
}

uint32_t motor_pulse_read_reg(void *ctx, void *param, uint32_t addr)
{
	return 0;
}

static struct basic_control_ops basic_pulse_ops = {
	.move = motor_pulse_move,
	.stop = motor_pulse_stop,
	.is_moving = motor_is_moving,
	.write_reg = motor_pulse_write_reg,
	.read_reg = motor_pulse_read_reg,
};

struct pwm_device *lens_pwm_request(uint32_t chn, const char *name,
	uint16_t dev_type, void *ops)
{
	struct pwm_device *pwm_dev = NULL;

	pwm_dev = pwm_request(chn, name);
	if (pwm_dev == NULL) {
		LOG(LOG_ERR, "can not get pwm device");
	} else {
		if (dev_type == PWM_TYPE) {
			ops = (void *)&basic_pwm_ops;
		} else if (dev_type == PULSE_TYPE) {
			ops = (void *)&basic_pulse_ops;
		}
	}

	return pwm_dev;
}

void lens_pwm_release(struct pwm_device *pwm_dev)
{
	if (!pwm_dev)
		return;

	pwm_free(pwm_dev);
	pwm_dev = NULL;
}

