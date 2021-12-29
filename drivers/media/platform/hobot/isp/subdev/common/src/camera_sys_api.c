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
#define pr_fmt(fmt) "[isp_drv]: %s: " fmt, __func__
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/printk.h>
#include <linux/dmaengine.h>
#include <linux/compiler.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/mman.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include "inc/camera_dev.h"
#include "inc/camera_subdev.h"
#include "inc/camera_i2c.h"
#include "inc/camera_spi.h"
#include "inc/camera_sys_api.h"

// PRQA S ALL ++

extern 	uint32_t sensor_log10(uint32_t val);
extern 	uint32_t sensor_date(uint32_t val);

static void DOFFSET(uint32_t *x, uint32_t n)
{
	switch (n) {
	case 2:
	*x = ((((*x) & 0xff00) >> 8) | (((*x) & 0xff) << 8));
	break;
	case 3:
	 *x = (((*x) & 0x0000ff00) + (((*x) & 0xff0000) >> 16)
		+ (((*x) & 0xff) << 16));
	break;
	case 4:
	 *x = ((((((*x) & 0xff000000) >> 8) +
		(((*x) & 0xff0000) << 8)) >> 16) |
	        (((((*x) & 0xff00) >> 8) +
		(((*x) & 0xff) << 8)) << 16));
	break;
	}
}

/*
 * sensor gain/line ctrl for ar0231 and so on
 * note: have again/dgain
 * note: ae using again to send info
 *
 */
unsigned int cal_again_regval(unsigned int* input_value)
{
	unsigned int again_regval;
	unsigned int value = *input_value;
	if ( (value >= 256) && (value < 320) ) {
		again_regval = 0x7777;
		*input_value = (value << 9) / 256;
	} else if ((value >= 320) && (value < 384)) {
		again_regval = 0x8888;
		*input_value = (value << 9) / 320;
	} else if ((value >= 384) && (value < 512)) {
		again_regval = 0x9999;
		*input_value = (value << 9) / 384;
	} else if ((value >= 512) && (value < 598)) {
		again_regval = 0xaaaa;
		*input_value = (value << 9) / 512;
	} else if ((value >= 598) && (value < 896)) {
		again_regval = 0xbbbb;
		*input_value = (value << 9) / 598;
	} else if ((value >= 896) && (value < 1024)) {
		again_regval = 0xcccc;
		*input_value = (value << 9) / 896;
	} else {
		again_regval = 0xdddd;
		*input_value = (value << 9) / 1024;
	}
	return	again_regval;
}

/*
 * sensor gain/line ctrl for sensor_lut and so on
 * note: have again/dgain
 * note:
 */
void camera_sys_sensor_gain_turning_data(uint32_t port,
	sensor_priv_t *priv_param, uint32_t *a_gain,
	uint32_t *d_gain, uint32_t *line)
{
	int i;
	uint32_t gain_temp = 0;
	uint32_t a_gain_num = 0;
	uint32_t d_gain_num = 0;
	uint32_t *again_lut = NULL;
	uint32_t *dgain_lut = NULL;
	uint32_t temp = 0;

	//mode switch
	switch (camera_mod[port]->camera_param.mode) {
	case NORMAL_M :
		a_gain_num = camera_mod[port]->camera_param.normal.again_control_num;
		d_gain_num = camera_mod[port]->camera_param.normal.dgain_control_num;
		again_lut = camera_mod[port]->camera_param.normal.again_lut;
		dgain_lut = camera_mod[port]->camera_param.normal.dgain_lut;
	break;
	case DOL2_M :
		a_gain_num = camera_mod[port]->camera_param.dol2.again_control_num;
		d_gain_num = camera_mod[port]->camera_param.dol2.dgain_control_num;
		again_lut = camera_mod[port]->camera_param.dol2.again_lut;
		dgain_lut = camera_mod[port]->camera_param.dol2.dgain_lut;
	break;
	case DOL3_M :
		a_gain_num = camera_mod[port]->camera_param.dol3.again_control_num;
		d_gain_num = camera_mod[port]->camera_param.dol3.dgain_control_num;
		again_lut = camera_mod[port]->camera_param.dol3.again_lut;
		dgain_lut = camera_mod[port]->camera_param.dol3.dgain_lut;
	break;
	case DOL4_M :
	break;
	case PWL_M :
		a_gain_num = camera_mod[port]->camera_param.pwl.again_control_num;
		d_gain_num = camera_mod[port]->camera_param.pwl.dgain_control_num;
		again_lut = camera_mod[port]->camera_param.pwl.again_lut;
		dgain_lut = camera_mod[port]->camera_param.pwl.dgain_lut;
	break;
	default:
		pr_debug("%s mode is error\n", __func__);
	break;
	}

	//show again
	for(i = 0; i < a_gain_num; i++) {
		gain_temp = priv_param->gain_buf[0];
		if (gain_temp > 255)
			gain_temp = 255;
		if (again_lut) {
			if (camera_mod[port]->camera_param.sensor_data.conversion) {
				temp = again_lut[i*256 + gain_temp];
				DOFFSET(&temp, 2);
				pr_debug("port %d, a_gain[%d] = 0x%x\n", port, i, temp);
			} else {
				pr_debug("port %d, a_gain[%d] = 0x%x\n", port, i, again_lut[i*256 + gain_temp]);
			}
			a_gain[i] = again_lut[i*256 + gain_temp];
		} else {
			pr_debug("a_gain[%d] gain is null\n", i);
			a_gain[i] = 0;
		}
	}

	//show dgain
	for(i = 0; i < d_gain_num; i++) {
		gain_temp = priv_param->dgain_buf[0];
		if (gain_temp > 255)
			gain_temp = 255;
		if (dgain_lut) {
			if (camera_mod[port]->camera_param.sensor_data.conversion) {
				temp = dgain_lut[i*256 + gain_temp];
				DOFFSET(&temp, 2);
				pr_debug("port %d, d_gain[%d] = 0x%x\n", port, i, temp);
			} else {
				pr_debug("port %d, d_gain[%d] = 0x%x\n", port, i, dgain_lut[i*256 + gain_temp]);
			}
			d_gain[i] = dgain_lut[i*256 + gain_temp];
		} else {
			pr_debug("d_gain[%d] gain is null\n", i);
			d_gain[i] = 0;
		}
	}
}

static uint32_t sensor_line_calculation(int ratio, uint32_t offset,
	uint32_t max, uint32_t input)
{
	uint32_t line = 0;
	uint32_t r_t = 256;
	if (input > max) {
		input = max;
	}

	if (ratio < 0) {
		r_t = (uint32_t)(0 - ratio);
		line = (uint32_t)(offset - ((r_t * input) >> 8));
	} else {
		r_t = (uint32_t)(ratio);
		line = (uint32_t)(offset + ((r_t * input) >> 8));
	}

	pr_debug("%s, ratio 0x%x, offset 0x%x, max 0x%x, input 0x%x, line 0x%x\n",
		__func__, ratio, offset, max, input, line);
	return line;
}

/*
 * sensor gain/line ctrl for sensor_lut and so on
 * note: have again/dgain
 * note:
 */
void camera_sys_sensor_line_turning_data(uint32_t port,
	sensor_priv_t *priv_param, uint32_t *a_line)
{
	uint32_t i = 0;
	uint32_t ratio, lexposure_time_min;

	//data calculation
	switch (camera_mod[port]->camera_param.mode) {
	case DOL4_M :
	case DOL3_M :
		for(i = 0; i < 3; i++) {
			a_line[i] = sensor_line_calculation(
				camera_mod[port]->camera_param.dol3.line_p[i].ratio,
				camera_mod[port]->camera_param.dol3.line_p[i].offset,
				camera_mod[port]->camera_param.dol3.line_p[i].max,
				priv_param->line_buf[i]);
		}
	break;
	case DOL2_M :
		for(i = 0; i < 2; i++) {
			a_line[i] = sensor_line_calculation(
				camera_mod[port]->camera_param.dol2.line_p[i].ratio,
				camera_mod[port]->camera_param.dol2.line_p[i].offset,
				camera_mod[port]->camera_param.dol2.line_p[i].max,
				priv_param->line_buf[i]);
		}
	break;
	case NORMAL_M :
		a_line[0] = sensor_line_calculation(
			camera_mod[port]->camera_param.normal.line_p.ratio,
			camera_mod[port]->camera_param.normal.line_p.offset,
			camera_mod[port]->camera_param.normal.line_p.max,
			priv_param->line_buf[0]);
	break;
	case PWL_M :
		ratio = camera_mod[port]->camera_param_ex.ratio_value;
		lexposure_time_min = camera_mod[port]->camera_param_ex.lexposure_time_min;
		a_line[0] = sensor_line_calculation(
			camera_mod[port]->camera_param.pwl.line_p.ratio,
			camera_mod[port]->camera_param.pwl.line_p.offset,
			camera_mod[port]->camera_param.pwl.line_p.max,
			priv_param->line_buf[0]);
		if(camera_mod[port]->camera_param_ex.ratio_en) {
			a_line[1] = a_line[0]/ratio;
			if(a_line[1] < lexposure_time_min)
				a_line[1] = lexposure_time_min;
		}

	break;
	default:
		pr_debug("sensor_mode is err!\n");
	break;
	}

	//line show
	for(i = 0; i < priv_param->line_num; i++) {
		 pr_debug("%s, a_line[%d] 0x%x\n", __func__, i, a_line[i]);
	}

	//data conversion
	if (camera_mod[port]->camera_param.sensor_data.conversion) {
		switch (camera_mod[port]->camera_param.mode) {
		case DOL4_M:
		break;
		case DOL3_M:
			DOFFSET(&a_line[0], camera_mod[port]->camera_param.dol3.s_line_length);
			DOFFSET(&a_line[1], camera_mod[port]->camera_param.dol3.m_line_length);
			DOFFSET(&a_line[2], camera_mod[port]->camera_param.dol3.l_line_length);
		break;
		case DOL2_M:
			DOFFSET(&a_line[0], camera_mod[port]->camera_param.dol2.s_line_length);
			DOFFSET(&a_line[1], camera_mod[port]->camera_param.dol2.m_line_length);
		break;
		case NORMAL_M:
			DOFFSET(&a_line[0], camera_mod[port]->camera_param.normal.s_line_length);
		break;
		case PWL_M:
			DOFFSET(&a_line[0], camera_mod[port]->camera_param.pwl.line_length);
			if(camera_mod[port]->camera_param_ex.ratio_en) {
				DOFFSET(&a_line[1], camera_mod[port]->camera_param_ex.l_line_length);
			}
		break;
		default:
			pr_debug("sensor_mode is err!\n");
		break;
		}
	}
}


void camera_common_alloc_again(uint32_t port, uint32_t *a_gain)
{
	if(*a_gain >= (camera_mod[port]->camera_param.sensor_data.analog_gain_max >> 13))
		*a_gain = camera_mod[port]->camera_param.sensor_data.analog_gain_max >> 13;
}

uint32_t camera_sys_sensor_gain_alloc(uint32_t port, uint32_t input,
	uint32_t gain_sw)
{
	int i = 0;
	uint32_t count = 0;
	uint32_t gain_temp = input;
	uint32_t a_gain_num = 0;
	uint32_t d_gain_num = 0;
	uint32_t *again_lut = NULL;
	uint32_t *dgain_lut = NULL;

	//mode switch
	switch (camera_mod[port]->camera_param.mode) {
	case NORMAL_M :
		a_gain_num = camera_mod[port]->camera_param.normal.again_control_num;
		d_gain_num = camera_mod[port]->camera_param.normal.dgain_control_num;
		again_lut = camera_mod[port]->camera_param.normal.again_lut;
		dgain_lut = camera_mod[port]->camera_param.normal.dgain_lut;
	break;
	case DOL2_M :
		a_gain_num = camera_mod[port]->camera_param.dol2.again_control_num;
		d_gain_num = camera_mod[port]->camera_param.dol2.dgain_control_num;
		again_lut = camera_mod[port]->camera_param.dol2.again_lut;
		dgain_lut = camera_mod[port]->camera_param.dol2.dgain_lut;
	break;
	case DOL3_M :
		a_gain_num = camera_mod[port]->camera_param.dol3.again_control_num;
		d_gain_num = camera_mod[port]->camera_param.dol3.dgain_control_num;
		again_lut = camera_mod[port]->camera_param.dol3.again_lut;
		dgain_lut = camera_mod[port]->camera_param.dol3.dgain_lut;
	break;
	case DOL4_M :
	break;
	case PWL_M :
		a_gain_num = camera_mod[port]->camera_param.pwl.again_control_num;
		d_gain_num = camera_mod[port]->camera_param.pwl.dgain_control_num;
		again_lut = camera_mod[port]->camera_param.pwl.again_lut;
		dgain_lut = camera_mod[port]->camera_param.pwl.dgain_lut;
	break;
	default:
		pr_debug("%s mode is error\n", __func__);
	break;
	}

	count = 0;
	if (gain_sw == 1) {
		while(gain_temp > 1) {
			for(i = 0; i < a_gain_num; i++) {
				if (again_lut[i * 256 + gain_temp] !=
					again_lut[i * 256 + gain_temp -1]) {
					goto SUCCESS_FLAG;
				}
			}
			gain_temp--;
		}
	} else if (gain_sw == 2) {
		while(gain_temp > 1) {
			for(i = 0; i < d_gain_num; i++) {
				if (dgain_lut[i * 256 + gain_temp] !=
					dgain_lut[i * 256 + gain_temp -1]) {
					goto SUCCESS_FLAG;
				}
			}
			gain_temp--;
		}
	}
	pr_debug("%s gain %d\n", __func__, gain_temp);
SUCCESS_FLAG:
	return gain_temp;
}

void camera_common_alloc_lut_again(uint32_t port, uint32_t *a_gain)
{
	if(*a_gain >= (camera_mod[port]->camera_param.sensor_data.
		analog_gain_max >> 13)) {
		*a_gain = camera_mod[port]->camera_param.sensor_data.analog_gain_max >> 13;
	}

	*a_gain = camera_sys_sensor_gain_alloc(port, *a_gain, 1);
	// lut again
}

void camera_common_alloc_dgain(uint32_t port, uint32_t *d_gain)
{
	if(*d_gain >= (camera_mod[port]->camera_param.sensor_data.digital_gain_max >> 13))
		*d_gain = camera_mod[port]->camera_param.sensor_data.digital_gain_max >> 13;
}

void camera_common_alloc_lut_dgain(uint32_t port, uint32_t *d_gain)
{
	if(*d_gain >= (camera_mod[port]->camera_param.sensor_data.
		digital_gain_max >> 13)) {
		*d_gain = camera_mod[port]->camera_param.sensor_data.digital_gain_max >> 13;
	}

	*d_gain = camera_sys_sensor_gain_alloc(port, *d_gain, 2);
}

static struct sensor_ctrl_ops sensor_ops[] = {
	{
		.ctrl_name = "imx327",
		.camera_gain_control = NULL,
		.camera_line_control = NULL,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},
	{
		.ctrl_name = "os8a10",
		.camera_gain_control = NULL,
		.camera_line_control = NULL,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},
	{
		.ctrl_name = "ar0233",
		.camera_gain_control = NULL,
		.camera_line_control = NULL,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},
	{
		.ctrl_name = "ar0144",
		.camera_gain_control = NULL,
		.camera_line_control = NULL,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},
	{
		.ctrl_name = "ar0231",
		.camera_gain_control = NULL,
		.camera_line_control = NULL,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},
	{
		.ctrl_name = "lut_mode",	//index 5
		.camera_gain_control = camera_sys_sensor_gain_turning_data,
		.camera_line_control = camera_sys_sensor_line_turning_data,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},
	{
		.ctrl_name = "lut_mode_1",	//index 6
		.camera_gain_control = camera_sys_sensor_gain_turning_data,
		.camera_line_control = camera_sys_sensor_line_turning_data,
		.camera_alloc_again = camera_common_alloc_lut_again,
		.camera_alloc_dgain = camera_common_alloc_lut_dgain,
	},
};


void camera_sys_printk_disturing(sensor_turning_data_t *turing_param)
{
	//sensor info

	pr_info("sensor_addr 0x%x, bus_type %d, reg_width %d",
	turing_param->sensor_addr, turing_param->bus_type, turing_param->reg_width);
	pr_info("sensor_mode %d \n", turing_param->mode);
	pr_info("s_line 0x%x s_line_length %d \n", turing_param->normal.s_line,
		turing_param->normal.s_line_length);
	pr_info("sensor_addr 0x%x, bus_type %d, reg_width %d",
	turing_param->sensor_addr, turing_param->bus_type, turing_param->reg_width);
	pr_info("line 0x%x line_length %d \n", turing_param->pwl.line,
		turing_param->pwl.line_length);
	pr_info("active_height %d active_width %d \n",
			turing_param->sensor_data.active_height,
		turing_param->sensor_data.active_width);
	pr_info("normal param_hold 0x%x param_hold_length %d \n",
			turing_param->normal.param_hold,
		turing_param->normal.param_hold_length);
	pr_info("dol2 param_hold 0x%x param_hold_length %d \n",
			turing_param->dol2.param_hold,
		turing_param->dol2.param_hold_length);
}
int camera_sys_read(uint32_t port, uint32_t reg_addr,
		uint32_t reg_width, char *buf, uint32_t length)
{
	int ret = 0;
	uint32_t chip_id;
	chip_id = camera_mod[port]->camera_param.chip_id;

	if(camera_mod[port]->camera_param.bus_type == I2C_BUS) {
		ret = camera_i2c_read(port, reg_addr, reg_width, buf, length);
	} else if (camera_mod[port]->camera_param.bus_type == SPI_BUS) {
		ret = camera_spi_read(port, chip_id, reg_width, reg_addr, buf, length);
	}

	return ret;
}
int camera_sys_write(uint32_t port, uint32_t reg_addr,
		uint32_t reg_width, char *buf, uint32_t length)
{
	int ret = 0;
	uint32_t chip_id;
	chip_id = camera_mod[port]->camera_param.chip_id;

	if(camera_mod[port]->camera_param.bus_type == I2C_BUS) {
		ret = camera_i2c_write(port, reg_addr, reg_width, buf, length);
		if(ret < 0) {
		pr_err("[%d] port %d reg_addr 0x%x reg_width %d length %d error!",
			__LINE__, port, reg_addr, reg_width, length);
		}
	} else if (camera_mod[port]->camera_param.bus_type == SPI_BUS) {
		ret = camera_spi_write(port, chip_id, reg_width, reg_addr, buf, length);
		if(ret < 0) {
		pr_err("[%d] port %d chip_id %d reg_addr 0x%x reg_width %d length %d error!",
			__LINE__, port, chip_id, reg_addr, reg_width, length);
		}
	}

	return ret;
}
int	camera_sys_alloc_again(uint32_t port, uint32_t *a_gain)
{
	int ret = 0;
	if (port >= CAMERA_TOTAL_NUMBER) {
		pr_err("not support %d max port is %d\n", port, CAMERA_TOTAL_NUMBER);
		return -1;
	}
	uint32_t num = camera_mod[port]->camera_param.sensor_data.turning_type;

	if ((num == 0) || (num > sizeof(sensor_ops)/sizeof(struct sensor_ctrl_ops))) {
		pr_info("gain line calculation out range, type is %d\n", num);
		dump_stack();
		return -1;
	} else {
		if (sensor_ops[num - 1].camera_alloc_again)
			sensor_ops[num - 1].camera_alloc_again(port, a_gain);
	}

	return ret;
}

int	camera_sys_alloc_dgain(uint32_t port, uint32_t *d_gain)
{
	int ret = 0;
	if (port >= CAMERA_TOTAL_NUMBER) {
		pr_err("not support %d max port is %d\n", port, CAMERA_TOTAL_NUMBER);
		return -1;
	}
	uint32_t num = camera_mod[port]->camera_param.sensor_data.turning_type;

	if ((num == 0) || (num > sizeof(sensor_ops)/sizeof(struct sensor_ctrl_ops))) {
		pr_info("gain line calculation out range, type is %d\n", num);
		dump_stack();
		return -1;
	} else {
		if (sensor_ops[num - 1].camera_alloc_dgain)
			sensor_ops[num - 1].camera_alloc_dgain(port, d_gain);
	}

	return ret;
}

int camera_sys_alloc_intergration_time(uint32_t port,
		uint32_t *intergration_time)
{
	int ret = 0;
	return ret;
}

static void camera_trans_value(uint32_t *input, char *output)
{
	output[0] = (char)(input[0] & 0xff);
	output[1] = (char)((input[0] >> 8) & 0xff);
	output[2] = (char)((input[0] >> 16) & 0x03);
	return;
}
static int camera_sys_set_normal_gain(uint32_t port, uint32_t *input_gain,
			uint32_t *dinput_gain)
{
	char a_gain[3] = {0};
	uint32_t i = 0;
	int ret = 0;
	uint32_t reg_width, s_gain, s_gain_length;

	for (i = 0; i < camera_mod[port]->
		camera_param.normal.again_control_num; i++) {
		reg_width = camera_mod[port]->camera_param.reg_width;
		s_gain_length = camera_mod[port]->
			camera_param.normal.again_control_length[i];
		s_gain = camera_mod[port]->camera_param.normal.again_control[i];
		camera_trans_value(&input_gain[i], a_gain);
		ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
		if(ret < 0) {
			pr_err("%d] port %d s_gain 0x%x a_gain: 0x%x 0x%x 0x%x error!",
				__LINE__, port, s_gain, a_gain[0], a_gain[1], a_gain[2]);
			return ret;
		}
	}

	for (i = 0; i < camera_mod[port]->
		camera_param.normal.dgain_control_num; i++) {
		reg_width = camera_mod[port]->camera_param.reg_width;
		s_gain_length = camera_mod[port]->
			camera_param.normal.again_control_length[i];
		s_gain = camera_mod[port]->camera_param.normal.dgain_control[i];
		camera_trans_value(&dinput_gain[i], a_gain);
		ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
		if(ret < 0) {
			pr_err("[%d] port %d s_gain 0x%x a_gain: 0x%x 0x%x 0x%x error!",
				__LINE__, port, s_gain, a_gain[0], a_gain[1], a_gain[2]);
		}
	}

    return ret;
}

static int camera_sys_set_normal_line(uint32_t port, uint32_t *input_line)
{
	char line_d[3] = {0};
	int ret = 0;
	uint32_t reg_width, s_line, s_line_length;

	reg_width = camera_mod[port]->camera_param.reg_width;
	s_line = camera_mod[port]->camera_param.normal.s_line;
	s_line_length = camera_mod[port]->camera_param.normal.s_line_length;

	camera_trans_value(input_line, line_d);
	ret = camera_sys_write(port, s_line, reg_width, line_d, s_line_length);
	if(ret < 0) {
		pr_err("[%d] port %d s_line 0x%x line_d: 0x%x 0x%x 0x%x error!",
			__LINE__, port, s_line, line_d[0], line_d[1], line_d[2]);
	}
	return ret;
}

static int camera_sys_set_dol2_gain(uint32_t port, uint32_t gain_num,
		uint32_t *input_gain, uint32_t *dinput_gain)
{
	char a_gain[3] = {0};
	uint32_t i = 0;
	int ret = 0;
	uint32_t reg_width, s_gain_length, s_gain;

	for (i = 0; i < camera_mod[port]->
		camera_param.dol2.again_control_num; i++) {
		reg_width = camera_mod[port]->camera_param.reg_width;
		s_gain_length = camera_mod[port]->camera_param.dol2.again_control_length[i];
		s_gain = camera_mod[port]->camera_param.dol2.again_control[i];
		camera_trans_value(&input_gain[i], a_gain);
		ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
		if(ret < 0) {
		pr_err("[%d] ret %d port %d s_gain 0x%x a_gain: 0x%x 0x%x 0x%x error!",
			__LINE__, ret, port, s_gain, a_gain[0], a_gain[1], a_gain[2]);
			return ret;
		}
	}

	for (i = 0; i < camera_mod[port]->
		camera_param.dol2.dgain_control_num; i++) {
		reg_width = camera_mod[port]->camera_param.reg_width;
		s_gain_length = camera_mod[port]->camera_param.dol2.again_control_length[i];
		s_gain = camera_mod[port]->camera_param.dol2.dgain_control[i];
		camera_trans_value(&dinput_gain[i], a_gain);
		ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
		if(ret < 0) {
		pr_err("[%d] ret %d port %d s_gain 0x%x a_gain: 0x%x 0x%x 0x%x error!",
			__LINE__, ret, port, s_gain, a_gain[0], a_gain[1], a_gain[2]);
			return ret;
		}
	}

	return ret;
}

static int camera_sys_set_dol2_line(uint32_t port, uint32_t gain_num,
		uint32_t *input_line)
{
	char line_d[3] = {0};
	//char rev_d[3] = {0};
	int ret = 0;
	uint32_t reg_width, s_line_length, m_line_length, s_line, m_line;

	reg_width = camera_mod[port]->camera_param.reg_width;
	s_line = camera_mod[port]->camera_param.dol2.s_line;
	m_line = camera_mod[port]->camera_param.dol2.m_line;
	s_line_length = camera_mod[port]->camera_param.dol2.s_line_length;
	m_line_length = camera_mod[port]->camera_param.dol2.m_line_length;

	switch (gain_num) {
		case 2:  // m_line
			camera_trans_value(&input_line[1], line_d);
			ret = camera_sys_write(port, m_line, reg_width, line_d, m_line_length);
			if(ret < 0) {
			pr_err("[%d]ret %d port %d m_line 0x%x line_d: 0x%x 0x%x 0x%x error!",
				__LINE__, ret, port, m_line, line_d[0], line_d[1], line_d[2]);
				return ret;
			}
		case 1:  // s_line
			camera_trans_value(input_line, line_d);
			ret = camera_sys_write(port, s_line, reg_width, line_d, s_line_length);
			if(ret < 0) {
			pr_err("[%d] ret %d port %d s_line 0x%x line_d: 0x%x 0x%x 0x%x error!",
				__LINE__, ret, port, s_line, line_d[0], line_d[1], line_d[2]);
				return ret;
			}
			break;
		default:
			break;
	}

	return ret;
}

static int camera_sys_set_dol3_gain(uint32_t port, uint32_t gain_num,
		uint32_t *input_gain, uint32_t *dinput_gain)
{
	char gain_d[3] = {0};
	uint32_t i = 0;
	int ret = 0;
	uint32_t reg_width, s_gain_length, s_gain;

	for (i = 0; i < camera_mod[port]->
		camera_param.dol3.again_control_num; i++) {
		reg_width = camera_mod[port]->camera_param.reg_width;
		s_gain_length = camera_mod[port]->camera_param.dol3.again_control_length[i];
		s_gain = camera_mod[port]->camera_param.dol3.again_control[i];
		camera_trans_value(&input_gain[i], gain_d);
		ret = camera_sys_write(port, s_gain, reg_width, gain_d, s_gain_length);
		if(ret < 0) {
		pr_err("[%d] ret %d port %d s_gain 0x%x gain_d: 0x%x 0x%x 0x%x error!",
			__LINE__, ret, port, s_gain, gain_d[0], gain_d[1], gain_d[2]);
			return ret;
		}
	}

	for (i = 0; i < camera_mod[port]->
		camera_param.dol3.dgain_control_num; i++) {
		reg_width = camera_mod[port]->camera_param.reg_width;
		s_gain_length = camera_mod[port]->camera_param.dol3.again_control_length[i];
		s_gain = camera_mod[port]->camera_param.dol3.dgain_control[i];
		camera_trans_value(&dinput_gain[i], gain_d);
		ret = camera_sys_write(port, s_gain, reg_width, gain_d, s_gain_length);
		if(ret < 0) {
		pr_err("[%d] ret %d port %d s_gain 0x%x gain_d: 0x%x 0x%x 0x%x error!",
			__LINE__, ret, port, s_gain, gain_d[0], gain_d[1], gain_d[2]);
			return ret;
		}
	}

	return ret;
}

static int camera_sys_set_dol3_line(uint32_t port, uint32_t gain_num,
		uint32_t *input_line)
{
	char line_d[3] = {0};
	int ret = 0;
	uint32_t reg_width, s_line_length, m_line_length, s_line, m_line;
	uint32_t l_line_length, l_line;

	reg_width = camera_mod[port]->camera_param.reg_width;
	s_line = camera_mod[port]->camera_param.dol3.s_line;
	m_line = camera_mod[port]->camera_param.dol3.m_line;
	l_line = camera_mod[port]->camera_param.dol3.l_line;
	s_line_length = camera_mod[port]->camera_param.dol3.s_line_length;
	m_line_length = camera_mod[port]->camera_param.dol3.m_line_length;
	l_line_length = camera_mod[port]->camera_param.dol3.l_line_length;
	switch (gain_num) {
		case 3:  // l_line
			camera_trans_value(&input_line[2], line_d);
			ret = camera_sys_write(port, l_line, reg_width, line_d, l_line_length);
			if(ret < 0) {
			pr_err("[%d] ret %d port %d l_line 0x%x line_d: 0x%x 0x%x 0x%x error!",
				__LINE__, ret, port, l_line, line_d[0], line_d[1], line_d[2]);
				return ret;
			}
		case 2:  // m_line
			camera_trans_value(&input_line[1], line_d);
			ret = camera_sys_write(port, m_line, reg_width, line_d, m_line_length);
			if(ret < 0) {
			pr_err("[%d] ret %d port %d m_line 0x%x line_d: 0x%x 0x%x 0x%x error!",
				__LINE__, ret, port, m_line, line_d[0], line_d[1], line_d[2]);
				return ret;
			}
		case 1:  // s_line
			camera_trans_value(input_line, line_d);
			ret = camera_sys_write(port, s_line, reg_width, line_d, s_line_length);
			if(ret < 0) {
			pr_err("[%d] ret %d port %d s_line 0x%x line_d: 0x%x 0x%x 0x%x error!",
				__LINE__, ret, port, s_line, line_d[0], line_d[1], line_d[2]);
				return ret;
			}
			break;
		default:
			break;
	}

	return ret;
}

int camera_sys_gain_line_process(uint32_t port, sensor_priv_t *priv_param,
				uint32_t *a_gain, uint32_t *d_gain, uint32_t *a_line)
{
	uint32_t num = camera_mod[port]->camera_param.sensor_data.turning_type;

	if ((num == 0) || (num > sizeof(sensor_ops)/sizeof(struct sensor_ctrl_ops))) {
		pr_info("gain line calculation out range, type is %d\n", num);
		dump_stack();
		return -1;
	} else {
		if (sensor_ops[num - 1].camera_gain_control)
			sensor_ops[num - 1].camera_gain_control(port,
				priv_param, a_gain, d_gain, a_line);
		if (sensor_ops[num - 1].camera_line_control)
			sensor_ops[num - 1].camera_line_control(port, priv_param, a_line);
	}

	return 0;
}

static int camera_sys_set_pwl_line(uint32_t port, uint32_t line_num,
						uint32_t *input_line)
{
	int ret = 0;
	char line_data[3] = {0};
	uint32_t reg_width, line_addr, line_length;
	uint32_t l_line_addr, l_line_length;
	uint32_t ratio_en;

	reg_width = camera_mod[port]->camera_param.reg_width;
	line_addr = camera_mod[port]->camera_param.pwl.line;
	line_length = camera_mod[port]->camera_param.pwl.line_length;
	ratio_en = camera_mod[port]->camera_param_ex.ratio_en;
	if(ratio_en) {
		l_line_addr = camera_mod[port]->camera_param_ex.l_line;
		l_line_length = camera_mod[port]->camera_param_ex.l_line_length;
	}

	switch (line_num) {
		case 1:
			if(ratio_en) {
				camera_trans_value(&input_line[1], line_data);
				pr_debug("line_data[0] 0x%x line_data[1] 0x%x\n",
					line_data[0], line_data[1]);
				ret = camera_sys_write(port, l_line_addr, reg_width,
						line_data, l_line_length);
				if(ret < 0) {
				pr_err("[%d] ret %d port %d l_line_addr 0x%x line_data:0x%x 0x%x 0x%x error!",
					__LINE__, ret, port, l_line_addr, line_data[0], line_data[1], line_data[2]);
					return ret;
				}
			}
			camera_trans_value(&input_line[0], line_data);
			pr_debug("input_line[0] 0x%x input_line[1] 0x%x\n",
					line_data[0], line_data[1]);
			ret = camera_sys_write(port, line_addr, reg_width, line_data, line_length);
			if(ret < 0) {
			pr_err("[%d] ret %d port %d line_addr 0x%x line_data:0x%x 0x%x 0x%x error!",
				__LINE__, ret, port, line_addr, line_data[0], line_data[1], line_data[2]);
			}
		default:
			break;
	}
	return ret;
}
static int camera_sys_set_pwl_gain(uint32_t port, uint32_t gain_num,
					uint32_t *input_gain)

{
	int ret = 0;
	uint32_t i = 0;
	char gain_data[3] = {0};
	uint32_t reg_width, gain_addr, gain_length;

	for (i = 0; i < camera_mod[port]->
		camera_param.pwl.again_control_num; i++) {
		reg_width = camera_mod[port]->camera_param.reg_width;
		gain_length = camera_mod[port]->camera_param.pwl.again_control_length[i];
		gain_addr = camera_mod[port]->camera_param.pwl.again_control[i];
		camera_trans_value(&input_gain[i], gain_data);
		ret = camera_sys_write(port, gain_addr, reg_width, gain_data, gain_length);
		if(ret < 0) {
		pr_err("[%d]ret %d port %d gain_addr 0x%x gain_data:0x%x 0x%x 0x%x error!",
			__LINE__, ret, port, gain_addr, gain_data[0], gain_data[1], gain_data[2]);
			return ret;
		}
	}
	return ret;
}

static int camera_sys_set_pwl_dgain(uint32_t port, uint32_t gain_num,
					uint32_t *input_dgain)

{
	int ret = 0;
	uint32_t i = 0;
	char gain_data[3] = {0};
	uint32_t reg_width, gain_addr, gain_length;

	for (i = 0; i < camera_mod[port]->
		camera_param.pwl.dgain_control_num; i++) {
		reg_width = camera_mod[port]->camera_param.reg_width;
		gain_length = camera_mod[port]->camera_param.pwl.dgain_control_length[i];
		gain_addr = camera_mod[port]->camera_param.pwl.dgain_control[i];
		camera_trans_value(&input_dgain[i], gain_data);
		ret = camera_sys_write(port, gain_addr, reg_width, gain_data, gain_length);
		if(ret < 0) {
		pr_err("[%d]ret %d port %d gain_addr 0x%x gain_data:0x%x 0x%x 0x%x error!",
			__LINE__, ret, port, gain_addr, gain_data[0], gain_data[1], gain_data[2]);
			return ret;
		}
	}
	return ret;
}

int camera_sys_set_ex_gain_control(uint32_t port, sensor_priv_t *priv_param,
		uint32_t *input_gain, uint32_t *input_dgain, uint32_t *input_line)
{
	int ret = 0;

	ret = camera_sys_set_pwl_line(port, priv_param->line_num, input_line);
	if(ret < 0) {
		pr_err("[%d] port %d line_num %d error!", __LINE__,
			port, priv_param->line_num);
		return ret;
	}
	ret = camera_sys_set_pwl_gain(port, priv_param->gain_num, input_gain);
	if(ret < 0) {
		pr_err("[%d] port %d gain_num %d error!", __LINE__,
			port, priv_param->gain_num);
		return ret;
	}
	ret = camera_sys_set_pwl_dgain(port, priv_param->gain_num, input_dgain);
	if(ret < 0) {
		pr_err("[%d] port %d gain_num %d error!", __LINE__,
			port, priv_param->gain_num);
	}
	return ret;
}

int  camera_sys_set_param_hold(uint32_t port, uint32_t value)
{
	int ret = 0;
	uint32_t param_hold = 0, param_hold_length = 0;
	int  reg_width;
	char buf[2] = {0};

	reg_width = camera_mod[port]->camera_param.reg_width;
	switch(camera_mod[port]->camera_param.mode) {
		case NORMAL_M:
			param_hold = camera_mod[port]->camera_param.normal.param_hold;
			param_hold_length = camera_mod[port]->camera_param.normal.param_hold_length;
			break;
		case DOL2_M:
			param_hold = camera_mod[port]->camera_param.dol2.param_hold;
			param_hold_length = camera_mod[port]->camera_param.dol2.param_hold_length;
			break;
		case DOL3_M:
			param_hold = camera_mod[port]->camera_param.dol3.param_hold;
			param_hold_length = camera_mod[port]->camera_param.dol3.param_hold_length;
			break;
		case PWL_M:
			param_hold = camera_mod[port]->camera_param.pwl.param_hold;
			param_hold_length = camera_mod[port]->camera_param.pwl.param_hold_length;
			break;
		default:
			pr_err("[%d] mode is err %d !", __LINE__,
					camera_mod[port]->camera_param.mode);
			ret = -1;
			break;
	}
	if(param_hold != 0) {
		if(value) {
			buf[0] = 0x01;
			ret = camera_sys_write(port, param_hold, reg_width, buf, param_hold_length);
			if(ret < 0) {
			pr_err("[%d] mode %d wirte param_hold 0x%x error!", __LINE__,
					camera_mod[port]->camera_param.mode, param_hold);
			}
		} else {
			buf[0] = 0x00;
			ret = camera_sys_write(port, param_hold, reg_width, buf, param_hold_length);
			if(ret < 0) {
			pr_err("[%d] mode %d wirte param_hold 0x%x error!", __LINE__,
					camera_mod[port]->camera_param.mode, param_hold);
			}
		}
	}
	return ret;
}
int  camera_sys_set_gain_line_control(uint32_t port, sensor_priv_t *priv_param)
{
	int ret = 0;
	uint32_t a_gain[3] = {0};
	uint32_t d_gain[6] = {0};
	uint32_t a_line[3] = {0};

	ret = camera_sys_gain_line_process(port, priv_param, a_gain, d_gain, a_line);
	if (ret) {
		pr_err("[%d] param is err!", __LINE__);
		return -1;
	}
	switch(camera_mod[port]->camera_param.mode) {
		case NORMAL_M:
			ret = camera_sys_set_param_hold(port, 0x1);
			ret |= camera_sys_set_normal_gain(port, a_gain, d_gain);
			ret |= camera_sys_set_normal_line(port, a_line);
			ret |= camera_sys_set_param_hold(port, 0x0);
			break;
		case DOL2_M:
			ret = camera_sys_set_param_hold(port, 0x1);
			ret |= camera_sys_set_dol2_gain(port, priv_param->gain_num, a_gain, d_gain);
			ret |= camera_sys_set_dol2_line(port, priv_param->line_num, a_line);
			ret |= camera_sys_set_param_hold(port, 0x0);
			break;
		case DOL3_M:
			ret = camera_sys_set_param_hold(port, 0x1);
			ret |= camera_sys_set_dol3_gain(port, priv_param->gain_num, a_gain, d_gain);
			ret |= camera_sys_set_dol3_line(port, priv_param->line_num, a_line);
			ret |= camera_sys_set_param_hold(port, 0x0);
			break;
		case PWL_M:
			ret = camera_sys_set_param_hold(port, 0x1);
			ret |= camera_sys_set_ex_gain_control(port, priv_param, a_gain, d_gain, a_line);
			ret |= camera_sys_set_param_hold(port, 0x0);
			break;
		default:
			pr_err("[%d ]mode is err %d !", __LINE__,
					camera_mod[port]->camera_param.mode);
			ret = -1;
			break;
	}

	return ret;
}

int  camera_sys_set_awb_control(uint32_t port, sensor_priv_t *priv_param)
{
	int ret = 0;
	char awb_gain[3];
	uint32_t i = 0;
	uint32_t data = 0;

	uint32_t reg_width, rgain_addr, rgain_length, bgain_addr, bgain_length;
	uint32_t grgain_addr, grgain_length, gbgain_addr, gbgain_length;

	reg_width = camera_mod[port]->camera_param.reg_width;

	camera_sys_set_param_hold(port, 0x1);
	for (i = 0; i < 4; i++) {
		rgain_addr = camera_mod[port]->camera_param.sensor_awb.rgain_addr[i];
		rgain_length = camera_mod[port]->camera_param.sensor_awb.rgain_length[i];
		if (rgain_length != 0) {
			if (camera_mod[port]->camera_param.sensor_awb.rb_prec > 8) {
				data = priv_param->rgain << (camera_mod[port]->camera_param.sensor_awb.rb_prec - 8);
			} else {
				data = priv_param->rgain >> (8 - camera_mod[port]->camera_param.sensor_awb.rb_prec);
			}
			if (camera_mod[port]->camera_param.sensor_data.conversion) {
				DOFFSET(&data, rgain_length);
			}
			camera_trans_value(&data, awb_gain);
			pr_debug("rgain[%d] = %x\n", i, data);
			ret = camera_sys_write(port, rgain_addr, reg_width, awb_gain, rgain_length);
		}

		bgain_addr = camera_mod[port]->camera_param.sensor_awb.bgain_addr[i];
		bgain_length = camera_mod[port]->camera_param.sensor_awb.bgain_length[i];
		if (bgain_length != 0) {
			if (camera_mod[port]->camera_param.sensor_awb.rb_prec > 8) {
				data = priv_param->bgain << (camera_mod[port]->camera_param.sensor_awb.rb_prec - 8);
			} else {
				data = priv_param->bgain >> (8 - camera_mod[port]->camera_param.sensor_awb.rb_prec);
			}
			if (camera_mod[port]->camera_param.sensor_data.conversion) {
				DOFFSET(&data, bgain_length);
			}
			camera_trans_value(&data, awb_gain);
			pr_debug("bgain[%d] = %x\n", i, data);
			ret = camera_sys_write(port, bgain_addr, reg_width, awb_gain, bgain_length);
		}

		grgain_addr = camera_mod[port]->camera_param.sensor_awb.grgain_addr[i];
		grgain_length = camera_mod[port]->camera_param.sensor_awb.grgain_length[i];
		if (grgain_length != 0) {
			if (camera_mod[port]->camera_param.sensor_awb.rb_prec > 8) {
				data = priv_param->grgain << (camera_mod[port]->camera_param.sensor_awb.rb_prec - 8);
			} else {
				data = priv_param->grgain >> (8 - camera_mod[port]->camera_param.sensor_awb.rb_prec);
			}
			if (camera_mod[port]->camera_param.sensor_data.conversion) {
				DOFFSET(&data, grgain_length);
			}
			camera_trans_value(&data, awb_gain);
			pr_debug("grgain[%d] = %x\n", i, data);
			ret = camera_sys_write(port, grgain_addr, reg_width, awb_gain, grgain_length);
		}

		gbgain_addr = camera_mod[port]->camera_param.sensor_awb.gbgain_addr[i];
		gbgain_length = camera_mod[port]->camera_param.sensor_awb.gbgain_length[i];
		if (gbgain_length != 0) {
			if (camera_mod[port]->camera_param.sensor_awb.rb_prec > 8) {
				data = priv_param->gbgain << (camera_mod[port]->camera_param.sensor_awb.rb_prec - 8);
			} else {
				data = priv_param->gbgain >> (8 - camera_mod[port]->camera_param.sensor_awb.rb_prec);
			}
			if (camera_mod[port]->camera_param.sensor_data.conversion) {
				DOFFSET(&data, gbgain_length);
			}
			camera_trans_value(&data, awb_gain);
			pr_debug("gbgain[%d] = %x\n", i, data);
			ret = camera_sys_write(port, gbgain_addr, reg_width, awb_gain, gbgain_length);
		}
	}
	camera_sys_set_param_hold(port, 0x0);

	return ret;
}

static uint32_t *camera_sys_lut_fill(uint32_t gain_num, uint32_t *turning_pram)
{
	uint32_t *gain_ptr = NULL;

	gain_ptr = kzalloc(256 * gain_num * sizeof(uint32_t), GFP_KERNEL);
	if (gain_ptr) {
		if (copy_from_user((void *)gain_ptr, (void __user *)turning_pram,
			256 * gain_num * sizeof(uint32_t))) {
				kfree(gain_ptr);
				gain_ptr = NULL;
				pr_err("copy is err !\n");
		}
	}

	return gain_ptr;
}

void camera_print_param_config(uint32_t port,
			sensor_turning_data_ex_t *turning_param_ex)
{
	pr_info("port %d ratio_en %d \n", turning_param_ex->ratio_en,
			turning_param_ex->ratio_value);
	pr_info("l_line 0x%x\n", turning_param_ex->l_line);
	pr_info("l_line_length %d\n", turning_param_ex->l_line_length);
	return;
}

int camera_turning_param_config(uint32_t port,
			sensor_turning_data_ex_t *turning_param_ex)
{
	int ret = 0;

	if (port >= CAMERA_TOTAL_NUMBER) {
		return -1;
	} else {
		if (turning_param_ex) {
			memcpy(&camera_mod[port]->camera_param_ex, turning_param_ex,
						sizeof(sensor_turning_data_ex_t));
		}
	}
	camera_print_param_config(port, &camera_mod[port]->camera_param_ex);
	return ret;
}

void camera_sys_tuning_data_init(uint32_t port,
		sensor_turning_data_t *turning_pram)
{
	memcpy(&camera_mod[port]->camera_param, turning_pram,
					sizeof(sensor_turning_data_t));
	camera_mod[port]->camera_param.normal.again_lut = NULL;
	camera_mod[port]->camera_param.normal.dgain_lut = NULL;
	camera_mod[port]->camera_param.dol2.again_lut = NULL;
	camera_mod[port]->camera_param.dol2.dgain_lut = NULL;
	camera_mod[port]->camera_param.dol3.again_lut = NULL;
	camera_mod[port]->camera_param.dol3.dgain_lut = NULL;
	camera_mod[port]->camera_param.pwl.again_lut = NULL;
	camera_mod[port]->camera_param.pwl.dgain_lut = NULL;
}

int camera_sys_turining_set(uint32_t port, sensor_turning_data_t *turning_pram)
{
	int ret = 0;
	uint32_t *ptr = NULL;

	if (port >= CAMERA_TOTAL_NUMBER) {
		return -1;
	}

	if (turning_pram) {
			camera_sys_tuning_release(port);
			camera_sys_tuning_data_init(port, turning_pram);
	} else {
			return -1;
	}

	// tuning lut map
	switch (camera_mod[port]->camera_param.mode) {
	case NORMAL_M:
		if (turning_pram->normal.again_lut) {
			ptr = NULL;
			ptr = camera_sys_lut_fill(turning_pram->normal.again_control_num,
				turning_pram->normal.again_lut);
			if (ptr) {
				camera_mod[port]->camera_param.normal.again_lut = ptr;
			} else {
				ret = -1;
				goto malloc_failed;
			}
		}

		if (turning_pram->normal.dgain_lut) {
			ptr = NULL;
			ptr = camera_sys_lut_fill(turning_pram->normal.dgain_control_num,
				turning_pram->normal.dgain_lut);
			if (ptr) {
				camera_mod[port]->camera_param.normal.dgain_lut = ptr;
			} else {
				ret = -1;
				if (camera_mod[port]->camera_param.normal.again_lut) {
					kfree(camera_mod[port]->camera_param.normal.again_lut);
					camera_mod[port]->camera_param.normal.again_lut = NULL;
				}
			}
		}
	break;
	case DOL2_M:
		if (turning_pram->dol2.again_lut) {
			ptr = NULL;
			ptr = camera_sys_lut_fill(turning_pram->dol2.again_control_num,
				turning_pram->dol2.again_lut);
			if (ptr) {
				camera_mod[port]->camera_param.dol2.again_lut = ptr;
			} else {
				ret = -1;
				goto malloc_failed;
			}
		}

		if (turning_pram->dol2.dgain_lut) {
			ptr = NULL;
			ptr = camera_sys_lut_fill(turning_pram->dol2.dgain_control_num,
				turning_pram->dol2.dgain_lut);
			if (ptr) {
				camera_mod[port]->camera_param.dol2.dgain_lut = ptr;
			} else {
				ret = -1;
				if (camera_mod[port]->camera_param.dol2.again_lut) {
					kfree(camera_mod[port]->camera_param.dol2.again_lut);
					camera_mod[port]->camera_param.dol2.again_lut = NULL;
				}
			}
		}
	break;
	case DOL3_M:
		if (turning_pram->dol3.again_lut) {
			ptr = NULL;
			ptr = camera_sys_lut_fill(turning_pram->dol3.again_control_num,
				turning_pram->dol3.again_lut);
			if (ptr) {
				camera_mod[port]->camera_param.dol3.again_lut = ptr;
			} else {
				ret = -1;
				goto malloc_failed;
			}
		}

		if (turning_pram->dol3.dgain_lut) {
			ptr = NULL;
			ptr = camera_sys_lut_fill(turning_pram->dol3.dgain_control_num,
				turning_pram->dol3.dgain_lut);
			if (ptr) {
				camera_mod[port]->camera_param.dol3.dgain_lut = ptr;
			} else {
				ret = -1;
				if (camera_mod[port]->camera_param.dol3.again_lut) {
					kfree(camera_mod[port]->camera_param.dol3.again_lut);
					camera_mod[port]->camera_param.dol3.again_lut = NULL;
				}
			}
		}
	break;
	case DOL4_M:
	break;
	case PWL_M:
		if (turning_pram->pwl.again_lut) {
			ptr = NULL;
			ptr = camera_sys_lut_fill(turning_pram->pwl.again_control_num,
				turning_pram->pwl.again_lut);
			if (ptr) {
				camera_mod[port]->camera_param.pwl.again_lut = ptr;
			} else {
				ret = -1;
				goto malloc_failed;
			}
		}

		if (turning_pram->pwl.dgain_lut) {
			ptr = NULL;
			ptr = camera_sys_lut_fill(turning_pram->pwl.dgain_control_num,
				turning_pram->pwl.dgain_lut);
			if (ptr) {
				camera_mod[port]->camera_param.pwl.dgain_lut = ptr;
			} else {
				ret = -1;
				if (camera_mod[port]->camera_param.pwl.again_lut) {
					kfree(camera_mod[port]->camera_param.pwl.again_lut);
					camera_mod[port]->camera_param.pwl.again_lut = NULL;
				}
			}
		}
	break;
	default:
	break;
	}

	camera_sys_printk_disturing(&camera_mod[port]->camera_param);
malloc_failed:
	return ret;
}

void camera_sys_tuning_release(uint32_t port)
{
	//free malloc size
	if (camera_mod[port]->camera_param.normal.again_lut) {
		kfree(camera_mod[port]->camera_param.normal.again_lut);
		camera_mod[port]->camera_param.normal.again_lut = NULL;
	}
	if (camera_mod[port]->camera_param.normal.dgain_lut) {
		kfree(camera_mod[port]->camera_param.normal.dgain_lut);
		camera_mod[port]->camera_param.normal.dgain_lut = NULL;
	}
	if (camera_mod[port]->camera_param.dol2.again_lut) {
		kfree(camera_mod[port]->camera_param.dol2.again_lut);
		camera_mod[port]->camera_param.dol2.again_lut = NULL;
	}
	if (camera_mod[port]->camera_param.dol2.dgain_lut) {
		kfree(camera_mod[port]->camera_param.dol2.dgain_lut);
		camera_mod[port]->camera_param.dol2.dgain_lut = NULL;
	}
	if (camera_mod[port]->camera_param.dol3.again_lut) {
		kfree(camera_mod[port]->camera_param.dol3.again_lut);
		camera_mod[port]->camera_param.dol3.again_lut = NULL;
	}
	if (camera_mod[port]->camera_param.dol3.dgain_lut) {
		kfree(camera_mod[port]->camera_param.dol3.dgain_lut);
		camera_mod[port]->camera_param.dol3.dgain_lut = NULL;
	}
	if (camera_mod[port]->camera_param.pwl.again_lut) {
		kfree(camera_mod[port]->camera_param.pwl.again_lut);
		camera_mod[port]->camera_param.pwl.again_lut = NULL;
	}
	if (camera_mod[port]->camera_param.pwl.dgain_lut) {
		kfree(camera_mod[port]->camera_param.pwl.dgain_lut);
		camera_mod[port]->camera_param.pwl.dgain_lut = NULL;
	}
}

int camera_sys_priv_set(uint32_t port, sensor_priv_t *priv_param)
{
	int ret = 0;

	if (port >= CAMERA_TOTAL_NUMBER)
		return -1;

	mutex_lock(&camera_mod[port]->slock);
	if (camera_mod[port]->client == NULL) {
		ret = -1;
		goto out;
	} else {
		if (priv_param) {
			uint32_t src_port = (camera_mod[port]->ae_share_flag >> 16) & 0xff;
			uint32_t dst_port = camera_mod[port]->ae_share_flag & 0xf;

			ret = camera_sys_set_gain_line_control(port, priv_param);
			if(ret < 0) {
				pr_err("[%d ] port %d src_port %d dst_port %d error!",
					__LINE__, port, src_port, dst_port);
				goto out;
			}

			/* ae share */
			if (src_port == dst_port) {
				goto out;
			}

			if (src_port - 0xA0 >= CAMERA_TOTAL_NUMBER || dst_port >= CAMERA_TOTAL_NUMBER) {
				pr_err("port src %d, dst %d exceed valid range.\n", src_port, dst_port);
				goto out;
			}

			if (src_port - 0xA0 != port) {
				pr_err("src port %d is not sharer, cur port %d\n", src_port, port);
				goto out;
			}
			ret = camera_sys_set_gain_line_control(dst_port, priv_param);
			if(ret < 0) {
				pr_err("[%d ] port %d src_port %d dst_port %d error!",
					__LINE__, port, src_port, dst_port);
				goto out;
			}
#if 0
			uint32_t line = 0;
			sensor_priv_t param;
			memcpy(&param, priv_param, sizeof(sensor_priv_t));

			line = sensor_line_calculation(
							camera_mod[dst_port]->camera_param.normal.line_p.ratio,
							camera_mod[dst_port]->camera_param.normal.line_p.offset,
							camera_mod[dst_port]->camera_param.normal.line_p.max,
							priv_param->line_buf[0]);

			if (line < camera_mod[dst_port]->camera_param.normal.line_p.max
				&& priv_param->gain_buf[0] == 0) {
				;
			} else if (line >= camera_mod[dst_port]->camera_param.normal.line_p.max
				&& priv_param->gain_buf[0] == 0) {

				param.gain_buf[0] = priv_param->line_buf[0] / 500 * 3 + 1/2;

			} else if (line >= camera_mod[dst_port]->camera_param.normal.line_p.max
				&& priv_param->gain_buf[0] > 0) {
				;
			}

			pr_debug("line %d, gain idx %d\n", param.line_buf[0], param.gain_buf[0]);

			camera_sys_set_gain_line_control(dst_port, &param);
#endif
		}
		else
			ret = -1;
	}

out:
	mutex_unlock(&camera_mod[port]->slock);
	return ret;
}

int camera_sys_priv_awb_set(uint32_t port, sensor_priv_t *priv_param)
{
	int ret = 0;

	if (port >= CAMERA_TOTAL_NUMBER) {
		ret = -1;
	} else {
		if (priv_param) {
			sensor_priv_t param;
			memcpy(&param, priv_param, sizeof(sensor_priv_t));
			pr_debug("rgain %d, bgain %d\n", param.rgain, param.bgain);
			ret = camera_sys_set_awb_control(port, &param);
		} else {
			ret = -1;
		}
	}
	return ret;
}

int camera_sys_get_param(uint32_t port, sensor_data_t *sensor_data)
{
	int ret = 0;

	if (port >= CAMERA_TOTAL_NUMBER) {
		pr_err("not support %d max port is %d\n", port, CAMERA_TOTAL_NUMBER);
		return -1;
	}
	memcpy(sensor_data, &camera_mod[port]->camera_param.sensor_data,
			sizeof(sensor_data_t));

	return ret;
}

int camera_sys_stream_on(uint32_t port)
{
	int ret = 0, i;
	char buf[2];
	uint32_t reg_width, setting_size, data_length;

        if (port >= CAMERA_TOTAL_NUMBER) {
	    pr_err("not support %d max port is %d\n", port, CAMERA_TOTAL_NUMBER);
	    return -1;
	}
	uint32_t *stream_on = camera_mod[port]->camera_param.stream_ctrl.stream_on;
    uint32_t size =
    sizeof(camera_mod[port]->camera_param.stream_ctrl.stream_on);

	reg_width = camera_mod[port]->camera_param.reg_width;
	data_length = camera_mod[port]->camera_param.stream_ctrl.data_length;
	setting_size = size/sizeof(uint32_t)/2;
	pr_info("stream on setting_size %d\n", setting_size);
	for(i = 0; i < setting_size; i++) {
		pr_info(" stream_on[i*2] 0x%x stream_on[i*2 + 1] 0x%x \n",
				stream_on[i*2], stream_on[i*2 + 1]);
		if(stream_on[i*2]) {
			if(data_length == 1) {
				buf[0] = (char)(stream_on[i*2 + 1] & 0xff);
				camera_i2c_write(port, stream_on[i*2], reg_width, buf, data_length);
			} else {
				buf[0] = (char)((stream_on[i*2 + 1] >> 8 ) & 0xff);
				buf[1] = (char)(stream_on[i*2 + 1] & 0xff);
				camera_i2c_write(port, stream_on[i*2], reg_width, buf, data_length);
			}
		} else {
			break;
		}
	}

	return ret;
}

int camera_sys_stream_off(uint32_t port)
{
	int ret = 0, i;
	char buf[2];
	uint32_t reg_width, setting_size, data_length;

        if (port >= CAMERA_TOTAL_NUMBER) {
	     pr_err("not support %d max port is %d\n", port, CAMERA_TOTAL_NUMBER);
	     return -1;
	}
	uint32_t *stream_off = camera_mod[port]->camera_param.stream_ctrl.stream_off;
    uint32_t size =
    sizeof(camera_mod[port]->camera_param.stream_ctrl.stream_off);

	reg_width = camera_mod[port]->camera_param.reg_width;
	data_length = camera_mod[port]->camera_param.stream_ctrl.data_length;
	setting_size = size/sizeof(uint32_t)/2;
	// pr_info("stream off setting_size %d\n", setting_size);
	for(i = 0; i < setting_size; i++) {
		// pr_info(" stream_off[i*2] 0x%x stream_off[i*2 + 1] 0x%x \n",
		//		stream_off[i*2], stream_off[i*2 + 1]);
		if(stream_off[i*2]) {
			if(data_length == 1) {
				buf[0] = (char)(stream_off[i*2 + 1] & 0xff);
				camera_i2c_write(port, stream_off[i*2], reg_width, buf, data_length);
			} else {
				buf[0] = (char)((stream_off[i*2 + 1] >> 8 ) & 0xff);
				buf[1] = (char)(stream_off[i*2 + 1] & 0xff);
				camera_i2c_write(port, stream_off[i*2], reg_width, buf, data_length);
			}
		} else {
			break;
		}
	}

	pr_info("camera_sys_stream_off success line %d dev_name %s\n",
		__LINE__, camera_mod[port]->name);
	return ret;
}

int camera_sys_sensor_write(uint32_t port, uint32_t address, uint32_t w_data)
{
    int ret = 0;
    char buf[1];
    uint32_t reg_width;

    if (port >= CAMERA_TOTAL_NUMBER) {
	pr_err("not support %d max port is %d\n", port, CAMERA_TOTAL_NUMBER);
	return -1;
    }
    reg_width = camera_mod[port]->camera_param.reg_width;
    buf[0] = (char)(w_data & 0xff);
    ret = camera_i2c_write(port, address, reg_width, buf, 1);
    return ret;
}

int camera_sys_sensor_read(uint32_t port, uint32_t address, uint32_t *r_data)
{
    int ret = 0;
    char buf[1];
    uint32_t reg_width;

     if (port >= CAMERA_TOTAL_NUMBER) {
	pr_err("not support %d max port is %d\n", port, CAMERA_TOTAL_NUMBER);
	return -1;
    }
    reg_width = camera_mod[port]->camera_param.reg_width;
    ret = camera_i2c_read(port, address, reg_width, buf, 1);
    *r_data = (uint32_t)(buf[0]);
    return ret;
}

// PRQA S ALL --
