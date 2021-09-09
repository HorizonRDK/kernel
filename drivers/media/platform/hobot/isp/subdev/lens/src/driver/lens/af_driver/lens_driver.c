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
 *    @file      lens_driver.c
 *    @brief     provide motor driver logic
 *    @details 
 *    @mainpage 
 *    @author    IE&E
 *    @email     yongyong.duan@horizon.ai
 *    @version   v-1.0.0
 *    @date      2020-1-17
 *    @license
 *    @copyright
 *********************************************************************/

#include <linux/delay.h>
#include "acamera_logger.h"
#include "lens_driver.h"
#include "lens_i2c.h"
#include "lens_pwm.h"
#include "lens_ops.h"
#include "lens_char.h"
#include "lens_gpio.h"
#include "acamera_command_api.h"

#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_LENS
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_LENS
#endif

#define LENS_MONTOR_NAME_LENS 20

/** pwm_dev struct pwm_device*/
struct motor_pwm_param_s {
	uint16_t pwm_num;
	uint32_t pwm_duty;
	uint32_t pwm_period;
	void *pwm_dev;
};

/** pulse_forward_dev struct pwm_device*/
struct motor_pulse_param_s {
	uint16_t pulse_forward_num;
	uint16_t pulse_back_num;
	uint32_t pulse_duty;
	uint32_t pulse_period;
	void *pulse_forward_dev;
	void *pulse_back_dev;
};

/** i2c_dev struct i2c_client*/
struct motro_i2c_param_s {
	uint16_t i2c_num;
	uint32_t i2c_addr;
	void *i2c_dev;
	struct motor_i2c_param reg_param;
};

/** spi_dev struct spi_client*/
struct motro_spi_param_s {
	uint16_t spi_num;
	uint32_t spi_addr;
	void *spi_dev;
};

/** spi_dev struct spi_client*/
struct motro_gpio_param_s {
	struct {
		uint16_t gpio_a1;
		uint16_t gpio_a2;
		uint16_t gpio_b1;
		uint16_t gpio_b2;
	} gpio_dev;
	uint8_t  gpio_enable;
};

/**
 * struct motor_param_s - motor_param_s
 * @motor_type: motor_type - i2c/pwm/pulse
 * @max_step: motor max_step
 */
struct motor_param_s {
	uint16_t motor_type;
	uint16_t max_step;
	uint32_t curr_pos;
	uint32_t next_pos;
	uint32_t init_pos;
	uint32_t min_pos;
	uint32_t max_pos;
	union {
		struct motor_pwm_param_s pwm_param;
		struct motor_pulse_param_s pulse_param;
		struct motro_i2c_param_s i2c_param;
		struct motro_spi_param_s spi_param;
		struct motro_gpio_param_s gpio_param;
	};
};

struct motor_context {
	/** AF_PARAM */
	uint32_t af_busy;
	struct basic_control_ops *af_ops;
	struct motor_param_s af_param;
	/** ZOOM_PRAM */
	uint32_t zoom_busy;
	struct basic_control_ops *zoom_ops;
	struct motor_param_s zoom_param;
	/** IRIS_PARAM, add later*/
};

static struct motor_context motor_ctx[FIRMWARE_CONTEXT_NUMBER];

void motor_ctx_param_init(void)
{
	memset(motor_ctx, 0, FIRMWARE_CONTEXT_NUMBER*sizeof(struct motor_context));
}

/**
 * @brief fill_ctx_param, request dev & ops
 *
 * @param ops control-ops
 * @param ctx motor_param
 * @param param
 * @return
 *   @retval
 */
int fill_ctx_param(struct basic_control_ops **ops, struct motor_param_s *param,
	struct chardev_port_param *ctx, const char *name)
{
	int ret = -1;
	// struct motor_info ctx_p;

	param->motor_type = ctx->motor_type;
	param->max_step = ctx->max_step;
	param->curr_pos = ctx->init_pos;
	param->next_pos = ctx->init_pos;
	param->init_pos = ctx->init_pos;
	param->min_pos = ctx->min_pos;
	param->max_pos = ctx->max_pos;

	if (ctx->motor_type == I2C_TYPE) {
		param->i2c_param.i2c_dev = lens_i2c_request(ctx->i2c_param.i2c_num,
			ctx->i2c_param.i2c_addr, name, ops);
		if (param->i2c_param.i2c_dev != NULL) {
			param->i2c_param.i2c_num = ctx->i2c_param.i2c_num;
			param->i2c_param.i2c_addr = ctx->i2c_param.i2c_addr;
			ret = 0;
		} else {
			LOG(LOG_ERR, "get i2c client failed !");
		}
	} else if (ctx->motor_type == PWM_TYPE) {
		param->pwm_param.pwm_dev = lens_pwm_request(ctx->pwm_param.pwm_num,
			name, ctx->motor_type, ops);
		if (param->pwm_param.pwm_dev != NULL) {
			param->pwm_param.pwm_num = ctx->pwm_param.pwm_num;
			param->pwm_param.pwm_duty = ctx->pwm_param.pwm_duty;
			param->pwm_param.pwm_period = ctx->pwm_param.pwm_period;
			ret = 0;
		} else {
			LOG(LOG_ERR, "get pwm dev failed !");
		}
	} else if (ctx->motor_type == PULSE_TYPE) {
		param->pulse_param.pulse_forward_dev =
			lens_pwm_request(ctx->pulse_param.pulse_forward_num,
			name, ctx->motor_type, ops);
		param->pulse_param.pulse_back_dev =
			lens_pwm_request(ctx->pulse_param.pulse_back_num,
			name, ctx->motor_type, ops);
		if ((param->pulse_param.pulse_forward_dev != NULL) &&
			(param->pulse_param.pulse_back_dev != NULL)) {
			param->pulse_param.pulse_forward_num = ctx->pulse_param.pulse_forward_num;
			param->pulse_param.pulse_back_num = ctx->pulse_param.pulse_back_num;
			param->pulse_param.pulse_duty = ctx->pulse_param.pulse_duty;
			param->pulse_param.pulse_period = ctx->pulse_param.pulse_period;
			ret = 0;
		} else {
			lens_pwm_release(param->pulse_param.pulse_forward_dev);
			param->pulse_param.pulse_forward_dev = NULL;
			lens_pwm_release(param->pulse_param.pulse_back_dev);
			param->pulse_param.pulse_back_dev = NULL;
			LOG(LOG_ERR, "get pulse dev failed !");
		}
	} else if (ctx->motor_type == GPIO_TYPE) {
		ret = lens_gpio_request(ctx->gpio_param.gpio_a1,
			ctx->gpio_param.gpio_a2, ctx->gpio_param.gpio_b1,
			ctx->gpio_param.gpio_b2, name, ops);
		if (ret >= 0) {
			param->gpio_param.gpio_dev.gpio_a1 = ctx->gpio_param.gpio_a1;
			param->gpio_param.gpio_dev.gpio_a2 = ctx->gpio_param.gpio_a2;
			param->gpio_param.gpio_dev.gpio_b1 = ctx->gpio_param.gpio_b1;
			param->gpio_param.gpio_dev.gpio_b2 = ctx->gpio_param.gpio_b2;
			param->gpio_param.gpio_enable = 1;
		} else {
			param->gpio_param.gpio_enable = 0;
			ret = -1;
		}
	}

	return ret;
}

void free_ctx_param(struct basic_control_ops **ops, struct motor_param_s *param)
{
	if (ops == NULL || param == NULL) {
		return;
	}

	if (*ops == NULL) {
		return;
	}

	if (param->motor_type == I2C_TYPE) {
		lens_i2c_release(param->i2c_param.i2c_dev);
		param->i2c_param.i2c_dev = NULL;
		param->i2c_param.i2c_num = 0;
		param->i2c_param.i2c_addr = 0;
	} else if (param->motor_type == PWM_TYPE) {
		lens_pwm_release(param->pwm_param.pwm_dev);
		param->pwm_param.pwm_dev = NULL;
		param->pwm_param.pwm_num = 0;
		param->pwm_param.pwm_duty = 0;
		param->pwm_param.pwm_period = 0;
	} else if (param->motor_type == PULSE_TYPE) {
		lens_pwm_release(param->pulse_param.pulse_forward_dev);
		lens_pwm_release(param->pulse_param.pulse_back_dev);
		param->pulse_param.pulse_forward_dev = NULL;
		param->pulse_param.pulse_back_dev = NULL;
		param->pulse_param.pulse_forward_num = 0;
		param->pulse_param.pulse_forward_num = 0;
		param->pulse_param.pulse_duty = 0;
		param->pulse_param.pulse_period = 0;
	} else if (param->motor_type == GPIO_TYPE) {
		lens_gpio_release(param->gpio_param.gpio_dev.gpio_a1,
			param->gpio_param.gpio_dev.gpio_a2,
			param->gpio_param.gpio_dev.gpio_b1,
			param->gpio_param.gpio_dev.gpio_b2);
		param->gpio_param.gpio_dev.gpio_a1 = 20;
		param->gpio_param.gpio_dev.gpio_a2 = 20;
		param->gpio_param.gpio_dev.gpio_b1 = 20;
		param->gpio_param.gpio_dev.gpio_b2 = 20;
		param->gpio_param.gpio_enable = 0;
	}

	param->motor_type = 0;
	param->max_step = 0;
	param->curr_pos = 0;
	param->next_pos = 0;
	param->init_pos = 0;
	param->min_pos = 0;
	param->max_pos = 0;
	*ops = NULL;
}

void *get_driverdev_info(struct motor_param_s *param, uint32_t pos)
{
	void *dev_info = NULL;

	if (param == NULL) {
		LOG(LOG_ERR, "param is null");
		return NULL;
	}

	if (param->motor_type == I2C_TYPE) {
		dev_info = param->i2c_param.i2c_dev;
	} else if (param->motor_type == PWM_TYPE) {
		dev_info = param->pwm_param.pwm_dev;
	} else if (param->motor_type == PULSE_TYPE) {
		//stop motor
		if (pos >= param->curr_pos) {
			dev_info = param->pulse_param.pulse_forward_dev;
		} else {
			dev_info = param->pulse_param.pulse_back_dev;
		}
	} else if (param->motor_type == SPI_TYPE) {
		dev_info = param->spi_param.spi_dev;
	} else if (param->motor_type == GPIO_TYPE) {
		dev_info = &param->gpio_param.gpio_dev;
	}

	return dev_info;
}

static void lens_basic_init(struct motor_param_s *param,
	struct basic_control_ops *ops)
{
	void *dev_info = NULL;
	struct motor_info ctx_p;

	if ((param == NULL) || (ops == NULL)) {
		LOG(LOG_ERR, "param is null");
		return;
	}

	memset(&ctx_p, 0, sizeof(struct motor_info));
	param->curr_pos = 0;
	param->next_pos = 0;

	dev_info = get_driverdev_info(param, 0);
	if (dev_info != NULL) {
		ctx_p.max_step = param->max_step;
		ctx_p.curr_pos = 0;
		ctx_p.next_pos = 0;
		ctx_p.init_pos = 0;
		ctx_p.min_pos = param->min_pos;
		ctx_p.max_pos = param->max_pos;
		ops->init(dev_info, &ctx_p);
	}
}

static void lens_basic_move(struct motor_param_s *param,
	struct basic_control_ops *ops, uint32_t pos)
{
	void *dev_info = NULL;
	struct motor_info ctx_p;

	if ((param == NULL) || (ops == NULL)) {
		LOG(LOG_INFO, "param is null");
		return;
	}
	memset(&ctx_p, 0, sizeof(struct motor_info));
	dev_info = get_driverdev_info(param, pos);
	if (dev_info != NULL) {
		param->curr_pos = param->next_pos;
		ctx_p.max_step = param->max_step;
		ctx_p.curr_pos = param->curr_pos;
		ctx_p.next_pos = param->next_pos;
		ctx_p.init_pos = param->init_pos;
		ctx_p.min_pos = param->min_pos;
		ctx_p.max_pos = param->max_pos;
		if (param->motor_type == I2C_TYPE) {
			ctx_p.ctrl_param = (void *)(&param->i2c_param.reg_param);
		}
		ops->move(dev_info, &ctx_p, pos);
		param->next_pos = pos;
	}
}

static void lens_basic_stop(struct motor_param_s *param,
	struct basic_control_ops *ops)
{
	void *dev_info = NULL;

	if ((param == NULL) || (ops == NULL)) {
		LOG(LOG_INFO, "param is null");
		return;
	}

	dev_info = get_driverdev_info(param, param->next_pos);
	if (dev_info != NULL) {
		ops->stop(dev_info, NULL);
	}
}

static uint8_t lens_basic_is_moving(struct motor_param_s *param,
	struct basic_control_ops *ops)
{
	uint8_t motor_status = 0;
	void *dev_info = NULL;

	if ((param == NULL) || (ops == NULL)) {
		LOG(LOG_INFO, "param is null");
		return motor_status;
	}

	dev_info = get_driverdev_info(param, param->next_pos);
	if (dev_info != NULL) {
		motor_status = ops->is_moving(dev_info);
	}

	return motor_status;
}

static void lens_basic_write_reg(struct motor_param_s *param,
	struct basic_control_ops *ops, uint32_t addr, uint32_t data)
{
	void *dev_info = NULL;
	struct motor_info ctx_p;

	if ((param == NULL) || (ops == NULL)) {
		LOG(LOG_INFO, "param is null");
		return;
	}

	memset(&ctx_p, 0, sizeof(struct motor_info));
	dev_info = get_driverdev_info(param, param->next_pos);
	if (dev_info != NULL) {
		param->curr_pos = param->next_pos;
		ctx_p.max_step = param->max_step;
		ctx_p.curr_pos = param->curr_pos;
		ctx_p.next_pos = param->next_pos;
		ctx_p.init_pos = param->init_pos;
		ctx_p.min_pos = param->min_pos;
		ctx_p.max_pos = param->max_pos;
		if (param->motor_type == I2C_TYPE) {
			ctx_p.ctrl_param = (void *)(&param->i2c_param.reg_param);
		}
		ops->write_reg(dev_info, &ctx_p, addr, data);
	}
}

static uint32_t lens_basic_read_reg(struct motor_param_s *param,
	struct basic_control_ops *ops, uint32_t addr)
{
	uint32_t data = 0;
	void *dev_info = NULL;
	struct motor_info ctx_p;

	if ((param == NULL) || (ops == NULL)) {
		LOG(LOG_INFO, "param is null");
		return 0;
	}

	memset(&ctx_p, 0, sizeof(struct motor_info));
	dev_info = get_driverdev_info(param, param->next_pos);
	if (dev_info != NULL) {
		param->curr_pos = param->next_pos;
		ctx_p.max_step = param->max_step;
		ctx_p.curr_pos = param->curr_pos;
		ctx_p.next_pos = param->next_pos;
		ctx_p.init_pos = param->init_pos;
		ctx_p.min_pos = param->min_pos;
		ctx_p.max_pos = param->max_pos;
		if (param->motor_type == I2C_TYPE) {
			ctx_p.ctrl_param = (void *)(&param->i2c_param.reg_param);
		}
		data = ops->read_reg(dev_info, &ctx_p, addr);
	}
	return data;
}

int lens_basic_get_param(uint16_t port, uint32_t param_id,
	struct motor_param_s **param, struct basic_control_ops **ops)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "Invalid param: port: %d.", port);
		ret = -1;
		goto err_param;
	}

	if (param_id >= LENS_PARAM_MAX_ID || param_id <= LENS_PARAM_MIN_ID) {
		LOG(LOG_ERR, "Invalid param: param_id: %d.", param_id);
		ret = -1;
		goto err_param;
	}

	if (param_id == LENS_AF_PARAM_ID) {
		LOG(LOG_DEBUG, "get %d af param.", port);
		if (motor_ctx[port].af_ops) {
			*param = &(motor_ctx[port].af_param);
			*ops = motor_ctx[port].af_ops;
			motor_ctx[port].af_busy = 1;
		} else {
			ret = -1;
		}
	} else if (param_id == LENS_ZOOM_PARAM_ID) {
		LOG(LOG_DEBUG, "get %d zoom param.", port);
		if (motor_ctx[port].zoom_ops) {
			*param = &(motor_ctx[port].zoom_param);
			*ops = motor_ctx[port].zoom_ops;
			motor_ctx[port].zoom_busy = 1;
		} else {
			ret = -1;
		}
	} else if (param_id == LENS_IRIS_PARAM_ID) {
		LOG(LOG_DEBUG, "get %d iris param.", port);
		//add later
	}

	if (*param) {
		if (param_id == LENS_AF_PARAM_ID) {
			LOG(LOG_INFO, "af port: %d, curr_pos %d", port, motor_ctx[port].af_param.curr_pos);
			LOG(LOG_INFO, "af port: %d, next_pos %d", port, motor_ctx[port].af_param.next_pos);
		} else if (param_id == LENS_ZOOM_PARAM_ID) {
			LOG(LOG_INFO, "zoom port: %d, curr_pos %d", port, motor_ctx[port].zoom_param.curr_pos);
			LOG(LOG_INFO, "zoom port: %d, next_pos %d", port, motor_ctx[port].zoom_param.next_pos);
		}
	}
err_param:
	return ret;
}

int lens_basic_free_param(uint16_t port, uint32_t param_id)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "Invalid param: port: %d.", port);
		ret = -1;
		goto err_param;
	}

	if (param_id >= LENS_PARAM_MAX_ID || param_id <= LENS_PARAM_MIN_ID) {
		LOG(LOG_ERR, "Invalid param: param_id: %d.", param_id);
		ret = -1;
		goto err_param;
	}

	if (param_id == LENS_AF_PARAM_ID) {
		LOG(LOG_DEBUG, "free %d af param.", port);
		motor_ctx[port].af_busy = 0;
	} else if (param_id == LENS_ZOOM_PARAM_ID) {
		LOG(LOG_DEBUG, "free %d zoom param.", port);
		motor_ctx[port].zoom_busy = 0;
	} else if (param_id == LENS_IRIS_PARAM_ID) {
		LOG(LOG_DEBUG, "free %d iris param.", port);
		//add later
	}
err_param:
	return ret;
}

int lens_driver_init(uint16_t port, uint32_t param_id)
{
	int ret = 0;
	struct motor_param_s *param = NULL;
	struct basic_control_ops *ops = NULL;

	ret = lens_basic_get_param(port, param_id, &param, &ops);
	if (!ret) {
		lens_basic_init(param, ops);
	}
	lens_basic_free_param(port, param_id);

	return ret;
}

int lens_driver_move(uint16_t port, uint32_t param_id, uint32_t pos)
{
	int ret = 0;
	struct motor_param_s *param = NULL;
	struct basic_control_ops *ops = NULL;

	ret = lens_basic_get_param(port, param_id, &param, &ops);
	if (!ret) {
		lens_basic_move(param, ops, pos);
	}
	lens_basic_free_param(port, param_id);

	return ret;
}

uint8_t lens_driver_get_status(uint16_t port, uint32_t param_id)
{
	int ret = 0;
	uint8_t status = 0;
	struct motor_param_s *param = NULL;
	struct basic_control_ops *ops = NULL;

	ret = lens_basic_get_param(port, param_id, &param, &ops);
	if (!ret) {
		status = lens_basic_is_moving(param, ops);
	}
	lens_basic_free_param(port, param_id);

	return status;
}

uint32_t lens_driver_get_pos(uint16_t port, uint32_t param_id)
{
	int ret = 0;
	uint32_t pos = 0;
	struct motor_param_s *param = NULL;
	struct basic_control_ops *ops = NULL;

	ret = lens_basic_get_param(port, param_id, &param, &ops);
	if (param != NULL) {
		pos = param->curr_pos;
	}
	lens_basic_free_param(port, param_id);

	return pos;
}

int lens_driver_get_param(uint16_t port, uint32_t param_id)
{
	int ret = 0;
	struct motor_param_s *param = NULL;
	struct basic_control_ops *ops = NULL;

	ret = lens_basic_get_param(port, param_id, &param, &ops);
	lens_basic_free_param(port, param_id);

	return 0;
}

void lens_driver_write_reg(uint16_t port, uint32_t param_id,
	uint32_t addr, uint32_t data)
{
	int ret = 0;
	struct motor_param_s *param = NULL;
	struct basic_control_ops *ops = NULL;

	ret = lens_basic_get_param(port, param_id, &param, &ops);
	if (!ret) {
		lens_basic_write_reg(param, ops, addr, data);
	}
	lens_basic_free_param(port, param_id);
}

uint32_t lens_driver_read_reg(uint16_t port, uint32_t param_id, uint32_t addr)
{
	int ret = 0;
	uint32_t data = 0;
	struct motor_param_s *param = NULL;
	struct basic_control_ops *ops = NULL;

	ret = lens_basic_get_param(port, param_id, &param, &ops);
	if (!ret) {
		data = lens_basic_read_reg(param, ops, addr);
	}
	lens_basic_free_param(port, param_id);

	return data;
}

void lens_driver_stop(uint16_t port, uint32_t param_id)
{
	int ret = 0;
	struct motor_param_s *param = NULL;
	struct basic_control_ops *ops = NULL;

	ret = lens_basic_get_param(port, param_id, &param, &ops);
	if (!ret) {
		lens_basic_stop(param, ops);
	}
	lens_basic_free_param(port, param_id);
}

//CHAR_DEV
int set_af_param(uint16_t port, struct chardev_port_param *ctx)
{
	int ret = 0;
	char name[20];

	if ((port >= FIRMWARE_CONTEXT_NUMBER) || (ctx == NULL)) {
		LOG(LOG_ERR, "Invalid param: port: %d.", port);
		return -1;
	}

	if (motor_ctx[port].af_ops) {
		LOG(LOG_ERR, "port: %d have init.", port);
		return 0;
	}

	snprintf(name, LENS_MONTOR_NAME_LENS, "%s_%d", "afdev", port);
	ret = fill_ctx_param(&motor_ctx[port].af_ops,
		&motor_ctx[port].af_param, ctx, name);

	return ret;
}

int set_i2c_param(uint16_t port, struct motor_i2c_param *ctx)
{
	int ret = 0;

	if ((port >= FIRMWARE_CONTEXT_NUMBER) || (ctx == NULL)) {
		LOG(LOG_ERR, "Invalid param: port: %d.", port);
		return -1;
	}

	if (ctx->control_type == AF_MODE) {
		memcpy(&motor_ctx[port].af_param.i2c_param.reg_param,
			ctx, sizeof(struct motor_i2c_param));
	} else if (ctx->control_type == ZOOM_MODE) {
		memcpy(&motor_ctx[port].zoom_param.i2c_param.reg_param,
			ctx, sizeof(struct motor_i2c_param));
	}

	LOG(LOG_DEBUG, "i2c param port: %d.", ctx->port);
	LOG(LOG_DEBUG, "i2c param type: %d.", ctx->control_type);
	LOG(LOG_DEBUG, "i2c param reg_width: %d.", ctx->reg_width);
	LOG(LOG_DEBUG, "i2c param conversion: %d.", ctx->conversion);
	LOG(LOG_DEBUG, "i2c param reg_len: %d.", ctx->reg_len);
	LOG(LOG_DEBUG, "i2c param reg_addr: 0x%x.", ctx->reg_addr);
	LOG(LOG_DEBUG, "i2c param reserved1: %d.", ctx->reserved_1);
	LOG(LOG_DEBUG, "i2c param reserved2: %d.", ctx->reserved_2);

	return ret;
}


int free_af_param(uint16_t port)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "Invalid param: port: %d.", port);
		return -1;
	}
	while (motor_ctx[port].af_busy) {
		udelay(10*1000);
	}
	free_ctx_param(&motor_ctx[port].af_ops, &motor_ctx[port].af_param);

	return ret;
}


int set_zoom_param(uint16_t port, struct chardev_port_param *ctx)
{
	int ret = 0;
	char name[20];

	if ((port >= FIRMWARE_CONTEXT_NUMBER) || (ctx == NULL)) {
		LOG(LOG_ERR, "Invalid param: port: %d.", port);
		return -1;
	}

	if (motor_ctx[port].zoom_ops) {
		LOG(LOG_ERR, "port: %d have init.", port);
		return 0;
	}

	snprintf(name, LENS_MONTOR_NAME_LENS, "%s_%d", "zoomdev", port);
	ret = fill_ctx_param(&motor_ctx[port].zoom_ops,
		&motor_ctx[port].zoom_param, ctx, name);

	return ret;
}

int free_zoom_param(uint16_t port)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "Invalid param: port: %d.", port);
		return -1;
	}

	while (motor_ctx[port].zoom_busy) {
		udelay(10*1000);
	}
	free_ctx_param(&motor_ctx[port].zoom_ops, &motor_ctx[port].zoom_param);

	return ret;
}

int set_af_init(uint16_t port)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "Invalid param: port: %d.", port);
		return -1;
	}

	ret = lens_driver_init(port, LENS_AF_PARAM_ID);

	return ret;
}

int set_zoom_init(uint16_t port)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "Invalid param: port: %d.", port);
		return -1;
	}

	ret = lens_driver_init(port, LENS_ZOOM_PARAM_ID);

	return ret;
}

int set_af_pos(uint16_t port, uint32_t pos)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "Invalid param: port: %d.", port);
		return -1;
	}
	ret = set_af_init(port);
	udelay(10*1000);
	ret = lens_driver_move(port, LENS_AF_PARAM_ID, pos);

	return ret;
}

int set_zoom_pos(uint16_t port, uint32_t pos)
{
	int ret = 0;

	if (port >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "Invalid param: port: %d.", port);
		return -1;
	}
	ret = set_zoom_init(port);
	udelay(10*1000);
	ret = lens_driver_move(port, LENS_ZOOM_PARAM_ID, pos);

	return ret;
}
