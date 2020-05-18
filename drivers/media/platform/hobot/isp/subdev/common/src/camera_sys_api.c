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
 * sensor gain/line ctrl for imx327 and so on
 * note: imx327 only have a_gain(a_gain/d_gain in the same register)
 * note: db
 */

void camera_sys_imxsensor_turning_control(uint32_t port, sensor_priv_t *priv_param,
	uint32_t *a_gain, uint32_t *d_gain, uint32_t *a_line)
{
	int i;
	uint32_t step_gain;
	uint32_t line_buf0,  line_buf1, line_buf2;

	line_buf0 = priv_param->line_buf[0];
	line_buf1 = priv_param->line_buf[1];
	line_buf2 = priv_param->line_buf[2];

	step_gain = camera_mod[port]->camera_param.sensor_data.step_gain;
	for(i = 0; i < priv_param->gain_num; i++) {
		a_gain[i] = sensor_log10(priv_param->gain_buf[i]);
		a_gain[i] = (uint32_t)(((a_gain[i]* 200) / step_gain) >> 8);
	}
	if(priv_param->line_num == 1) {
		a_line[0] = camera_mod[port]->camera_param.sensor_data.VMAX
			- 1 - priv_param->line_buf[0];
	} else if (priv_param->line_num == 2) {
		if (line_buf1 >= camera_mod[port]->camera_param.sensor_data.FSC_DOL2)
				line_buf1 = camera_mod[port]->camera_param.sensor_data.FSC_DOL2 - 1;
		 a_line[1] = camera_mod[port]->camera_param.sensor_data.FSC_DOL2
			 - 1 - line_buf1;
		if (line_buf0 >= camera_mod[port]->camera_param.sensor_data.RHS1)
				line_buf0 = camera_mod[port]->camera_param.sensor_data.RHS1 - 1;
		 a_line[0] = camera_mod[port]->camera_param.sensor_data.RHS1
			 - 1 - line_buf0;
	} else if (priv_param->line_num == 3) {
		 a_line[2] = camera_mod[port]->camera_param.sensor_data.FSC_DOL3
			- 1 - line_buf2;
		 a_line[1] = camera_mod[port]->camera_param.sensor_data.RHS2
			 - 1 - line_buf1;
		 a_line[0] = camera_mod[port]->camera_param.sensor_data.RHS1
			 - 1 - line_buf0;
	}
	return;
}

/*
 * sensor gain/line ctrl for os8a10 and so on
 * note: have again/dgain
 * note:
 *
 */
void camera_sys_os8a10_turning_data(uint32_t port, sensor_priv_t *priv_param,
				uint32_t *a_gain, uint32_t *d_gain, uint32_t *a_line)
{
	int i;
	uint32_t again_prec, dgain_prec;
	uint32_t a_gain_temp[4];
	uint32_t d_gain_temp[4];
	again_prec = camera_mod[port]->camera_param.sensor_data.again_prec;
	dgain_prec = camera_mod[port]->camera_param.sensor_data.dgain_prec;

	for(i = 0; i < priv_param->gain_num; i++) {
		a_gain_temp[i] = sensor_date(priv_param->gain_buf[i]);
	}

	for(i = 0; i < priv_param->dgain_num; i++) {
		d_gain_temp[i] = sensor_date(priv_param->dgain_buf[i]);
	}

	if(camera_mod[port]->camera_param.mode == NORMAL_M) {
		a_gain[0] = a_gain_temp[0];
		d_gain[0] = d_gain_temp[0];

		if(again_prec <= 8)
			a_gain[0] = (a_gain[0] >> (8 - again_prec));
		else
			a_gain[0] = (a_gain[0] << (again_prec - 8));

		if(dgain_prec <= 8)
			d_gain[0] = (d_gain[0] >> (8 - dgain_prec));
		else
			d_gain[0] = (d_gain[0] << (dgain_prec - 8));

	} else if (camera_mod[port]->camera_param.mode == DOL2_M) {
		a_gain[0] = a_gain_temp[0];
		d_gain[0] = d_gain_temp[0];

		if(again_prec <= 8)
			a_gain[0] = (a_gain[0] >> (8 - again_prec));
		else
			a_gain[0] = (a_gain[0] << (again_prec - 8));
		if(dgain_prec <= 8)
			d_gain[0] = (d_gain[0] >> (8 - dgain_prec));
		else
			d_gain[0] = (d_gain[0] << (dgain_prec - 8));

		a_gain[1] = a_gain_temp[1];
		d_gain[1] = d_gain_temp[1];

		if(again_prec <= 8)
			a_gain[1] = (a_gain[1] >> (8 - again_prec));
		else
			a_gain[1] = (a_gain[1] << (again_prec - 8));
		if(dgain_prec <= 8)
			d_gain[1] = (d_gain[1] >> (8 - dgain_prec));
		else
			d_gain[1] = (d_gain[1] << (dgain_prec - 8));
	}

	if(camera_mod[port]->camera_param.mode == NORMAL_M) {
		if(priv_param->line_buf[0] > camera_mod[port]->camera_param.
					sensor_data.VMAX - 8)
			priv_param->line_buf[0] = camera_mod[port]->camera_param.
							sensor_data.VMAX - 8;
	} else if (camera_mod[port]->camera_param.mode == DOL2_M) {
		if((priv_param->line_buf[0] + priv_param->line_buf[1]) >
				(camera_mod[port]->camera_param.sensor_data.VMAX - 4))
			priv_param->line_buf[1] = camera_mod[port]->camera_param.sensor_data.VMAX
				- 4 - priv_param->line_buf[0];
	}

	for(i = 0; i < priv_param->line_num; i++) {
		again_prec = priv_param->line_buf[i];
		a_line[i] = 0;
		a_line[i] = (again_prec & 0xff) << 8;
		a_line[i] |= (again_prec & 0xff00) >> 8;
		pr_debug("line[%d] = 0x%x\n", i, a_line[i]);
	}

	// change
	for(i = 0; i < priv_param->gain_num; i++) {
		again_prec = a_gain[i];
		a_gain[i] = 0;
		a_gain[i] = (again_prec & 0xff) << 8;
		a_gain[i] |= (again_prec & 0xff00) >> 8;
		pr_debug("a_gain[%d] = 0x%x\n", i, a_gain[i]);
	}

	// change
	for(i = 0; i < priv_param->dgain_num; i++) {
		dgain_prec = d_gain[i];
		d_gain[i] = 0;
		d_gain[i] = (dgain_prec & 0xff) << 8;
		d_gain[i] |= (dgain_prec & 0xff00) >> 8;
		pr_debug("d_gain[%d] = 0x%x\n", i, d_gain[i]);
	}

		return;
}

/*
 * sensor gain/line ctrl for 0144 and so on
 * note: have again/dgain
 * note:
 *
 */
void camera_sys_ar0144_turning_control(uint32_t port, sensor_priv_t *priv_param,
                uint32_t *a_gain, uint32_t *d_gain, uint32_t *a_line)
{
	uint32_t tmp = 0;
	uint32_t tmp1 = 0;
	uint32_t coarse = 0;
	uint32_t fine = 0;
	uint32_t analog_gain;
	uint32_t digital_gain;
	analog_gain = sensor_date(priv_param->gain_buf[0]);
	digital_gain = sensor_date(priv_param->dgain_buf[0]);
	pr_debug("a_gain = 0x%x\n", analog_gain);

	//again
	tmp = analog_gain >> 8;
	if (tmp == 1) {
		tmp1 = 0;
		coarse = 0 << 4;
	} else if (tmp >= 2 && tmp < 4) {
		tmp1 = 1;
		coarse = 1 << 4;
	} else if (tmp >= 4 && tmp < 8) {
		tmp1 = 2;
		coarse = 2 << 4;
	} else if (tmp >= 8 && tmp < 16) {
		tmp1 = 3;
		coarse = 3 << 4;
	} else if (tmp == 16) {
		tmp1 = 4;
		coarse = 4 << 4;
	}
	tmp = (analog_gain * 10000) >> (tmp1 + 8);
	fine = 32 - (320000 + tmp - 1) / tmp;
	a_gain[0] = (uint16_t)(coarse | fine);
	a_gain[0] = (uint16_t)(a_gain[0] << 8 | a_gain[0] >> 8);

	//digital
	d_gain[0] = digital_gain >> 1;
	d_gain[0] = (uint16_t)(d_gain[0] << 8 | d_gain[0] >> 8);

	a_line[0] = priv_param->line_buf[0];
	a_line[0] = (uint16_t)(a_line[0] << 8 | a_line[0] >> 8);

	pr_debug("a_gain = 0x%x\n", a_gain[0]);
	pr_debug("d_gain = 0x%x\n", d_gain[0]);
	pr_debug("line = 0x%x\n", a_line[0]);

	return;
}

/*
 * sensor gain/line ctrl for ar0233 and so on
 * note: have dgain
 * note: ae using again to send info
 * note:
 *
 */
void camera_sys_ar0233_turning_control(uint32_t port, sensor_priv_t *priv_param,
				uint32_t *a_gain, uint32_t *d_gain, uint32_t *a_line)
{
	int i;

	for(i = 0; i < priv_param->gain_num; i++) {
		a_gain[i] = sensor_date(priv_param->gain_buf[i]);
		a_gain[i] = a_gain[i] >> 1;
		if ((a_gain[i] >> 7) > 16)
			d_gain[i] = a_gain[i] >> 2;
			pr_info("gain_num %d, a_gain[i] %d, d_gain[i]0x%x\n", a_gain[i],
					d_gain[i], priv_param->gain_num);
	}
	for(i = 0; i < priv_param->line_num; i++) {
		 a_line[i] = priv_param->line_buf[i];
		 pr_info("priv_param->line_num %d a_line[i] 0x%x\n",
				 priv_param->line_num, a_line[i]);
	}
	return;
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

void camera_sys_ar0231_turning_control(uint32_t port, sensor_priv_t *priv_param,
				uint32_t *a_gain, uint32_t *d_gain, uint32_t *a_line)
{
	int i;
	unsigned int value;
	for(i = 0; i < priv_param->gain_num; i++) {
		value = sensor_date(priv_param->gain_buf[i]);
		if (value < 760) {
			d_gain[i + 3] = 0; //dc_gain
			a_gain[i] = cal_again_regval(&value);
			d_gain[i] = value;
			pr_info("dc_gain_reg: 0x%x, again_reg: 0x%x, dgain_reg: 0x%x\n",
					d_gain[i + 3], a_gain[i], d_gain[i]);
		} else {
			d_gain[i + 3] = 0xf;  //dc_gain
			value = (value << 8) / 760;
			a_gain[i] = cal_again_regval(&value);
			d_gain[i] = value;
			pr_info("dcgain_reg: 0x%x, again_reg: 0x%x, dgain_reg: 0x%x\n",
					d_gain[i + 3], a_gain[i], d_gain[i]);
		}
	}
	for(i = 0; i < priv_param->line_num; i++) {
		 a_line[i] = priv_param->line_buf[i];
		 pr_info("priv_param->line_num %d  a_line[i] 0x%x\n",
				 priv_param->line_num, a_line[i]);
	}
	return;
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
				pr_debug("a_gain[%d] = 0x%x\n", i, temp);
			} else {
				pr_debug("a_gain[%d] = 0x%x\n", i, again_lut[i*256 + gain_temp]);
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
				pr_debug("d_gain[%d] = 0x%x\n", i, temp);
			} else {
				pr_debug("d_gain[%d] = 0x%x\n", i, dgain_lut[i*256 + gain_temp]);
			}
			d_gain[i] = dgain_lut[i*256 + gain_temp];
		} else {
			pr_debug("d_gain[%d] gain is null\n", i);
			d_gain[i] = 0;
		}
	}
}

static uint32_t sensor_line_calculation(uint32_t ratio, uint32_t offset,
	uint32_t max, uint32_t input)
{
	uint32_t line = input;
	if (input > max) {
		input = max;
	}

	if (ratio == 0) {
		line = offset - input;
	} else {
		line = offset + input;
	}

	pr_debug("%s, ratio 0x%x, offset 0x%x, max 0x%x, input %x\n",
		__func__, ratio, offset, max, input);
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
		a_line[0] = sensor_line_calculation(
			camera_mod[port]->camera_param.pwl.line_p.ratio,
			camera_mod[port]->camera_param.pwl.line_p.offset,
			camera_mod[port]->camera_param.pwl.line_p.max,
			priv_param->line_buf[0]);
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

struct sensor_ctrl_ops sensor_ops[] = {
	{
		.ctrl_name = "imx327",
		.camera_gain_control = camera_sys_imxsensor_turning_control,
		.camera_line_control = NULL,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},
	{
		.ctrl_name = "os8a10",
		.camera_gain_control = camera_sys_os8a10_turning_data,
		.camera_line_control = NULL,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},
	{
		.ctrl_name = "ar0233",
		.camera_gain_control = camera_sys_ar0233_turning_control,
		.camera_line_control = NULL,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},
	{
		.ctrl_name = "ar0144",
		.camera_gain_control = camera_sys_ar0144_turning_control,
		.camera_line_control = NULL,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},
	{
		.ctrl_name = "ar0231",
		.camera_gain_control = camera_sys_ar0231_turning_control,
		.camera_line_control = NULL,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},
	{
		.ctrl_name = "lut_mode",
		.camera_gain_control = camera_sys_sensor_gain_turning_data,
		.camera_line_control = camera_sys_sensor_line_turning_data,
		.camera_alloc_again = camera_common_alloc_again,
		.camera_alloc_dgain = camera_common_alloc_dgain,
	},
	{
		.ctrl_name = "lut_mode_1",
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
	pr_info("s_gain 0x%x  s_gain_length %d \n", turing_param->normal.s_gain,
		turing_param->normal.s_gain_length);
	pr_info("sensor_mode %d \n", turing_param->mode);
	pr_info("s_line 0x%x s_line_length %d \n", turing_param->normal.s_line,
		turing_param->normal.s_line_length);
	pr_info("sensor_addr 0x%x, bus_type %d, reg_width %d",
	turing_param->sensor_addr, turing_param->bus_type, turing_param->reg_width);
	pr_info("min_gain_time 0x%x  max_gain_time %d \n",
			turing_param->pwl.min_gain_time,
		turing_param->pwl.max_gain_time);
	pr_info("line 0x%x line_length %d \n", turing_param->pwl.line,
		turing_param->pwl.line_length);
	pr_info("gain 0x%x gain_length %d \n", turing_param->pwl.gain,
		turing_param->pwl.gain_length);
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
	} else if (camera_mod[port]->camera_param.bus_type == SPI_BUS) {
		ret = camera_spi_write(port, chip_id, reg_width, reg_addr, buf, length);
	}

	return ret;
}
int	camera_sys_alloc_again(uint32_t port, uint32_t *a_gain)
{
	int ret = 0;
	uint32_t num = camera_mod[port]->camera_param.sensor_data.turning_type;

	if (num  > sizeof(sensor_ops)/sizeof(struct sensor_ctrl_ops)) {
		pr_info("gain line calculation out range\n");
	} else {
		if (sensor_ops[num - 1].camera_alloc_again)
			sensor_ops[num - 1].camera_alloc_again(port, a_gain);
	}

	return ret;
}

int	camera_sys_alloc_dgain(uint32_t port, uint32_t *d_gain)
{
	int ret = 0;
	uint32_t num = camera_mod[port]->camera_param.sensor_data.turning_type;

	if (num > sizeof(sensor_ops)/sizeof(struct sensor_ctrl_ops)) {
		pr_info("gain line calculation out range\n");
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
	char dig_gain[3] = {0};
	uint32_t i = 0;
	int ret = 0;
	uint32_t reg_width, s_gain, s_gain_length, sd_gain, sd_gain_length;

	uint32_t num = camera_mod[port]->camera_param.sensor_data.turning_type;
	if (num == 6) {
		for (i = 0; i < camera_mod[port]->
			camera_param.normal.again_control_num; i++) {
			reg_width = camera_mod[port]->camera_param.reg_width;
			s_gain_length = camera_mod[port]->
				camera_param.normal.again_control_length[i];
			s_gain = camera_mod[port]->camera_param.normal.again_control[i];
			camera_trans_value(&input_gain[i], a_gain);
			ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
		}

		for (i = 0; i < camera_mod[port]->
			camera_param.normal.dgain_control_num; i++) {
			reg_width = camera_mod[port]->camera_param.reg_width;
			s_gain_length = camera_mod[port]->
				camera_param.normal.again_control_length[i];
			s_gain = camera_mod[port]->camera_param.normal.dgain_control[i];
			camera_trans_value(&dinput_gain[i], a_gain);
			ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
		}
	} else {
		reg_width = camera_mod[port]->camera_param.reg_width;
		s_gain = camera_mod[port]->camera_param.normal.s_gain;
		s_gain_length = camera_mod[port]->camera_param.normal.s_gain_length;
		sd_gain = camera_mod[port]->camera_param.normal.sd_gain;
		sd_gain_length = camera_mod[port]->camera_param.normal.sd_gain_length;

		camera_trans_value(input_gain, a_gain);
		if(sd_gain == 0) {
			ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
		} else {
			camera_trans_value(dinput_gain, dig_gain);
			ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
			ret = camera_sys_write(port, sd_gain, reg_width, dig_gain, sd_gain_length);
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

	return ret;
}

static int camera_sys_set_dol2_gain(uint32_t port, uint32_t gain_num,
		uint32_t *input_gain, uint32_t *dinput_gain)
{
	char a_gain[3] = {0};
	char dig_gain[3] = {0};
	uint32_t i = 0;
	int ret = 0;
	uint32_t reg_width, s_gain_length, m_gain_length, s_gain, m_gain;
	uint32_t sd_gain, sd_gain_length, md_gain, md_gain_length;

	uint32_t num = camera_mod[port]->camera_param.sensor_data.turning_type;
	if (num == 6) {
		for (i = 0; i < camera_mod[port]->
			camera_param.dol2.again_control_num; i++) {
			reg_width = camera_mod[port]->camera_param.reg_width;
			s_gain_length = camera_mod[port]->camera_param.dol2.again_control_length[i];
			s_gain = camera_mod[port]->camera_param.dol2.again_control[i];
			camera_trans_value(&input_gain[i], a_gain);
			ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
		}

		for (i = 0; i < camera_mod[port]->
			camera_param.dol2.dgain_control_num; i++) {
			reg_width = camera_mod[port]->camera_param.reg_width;
			s_gain_length = camera_mod[port]->camera_param.dol2.again_control_length[i];
			s_gain = camera_mod[port]->camera_param.dol2.dgain_control[i];
			camera_trans_value(&dinput_gain[i], a_gain);
			ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
		}
	} else {
		reg_width = camera_mod[port]->camera_param.reg_width;
		s_gain = camera_mod[port]->camera_param.dol2.s_gain;
		sd_gain = camera_mod[port]->camera_param.dol2.sd_gain;
		m_gain = camera_mod[port]->camera_param.dol2.m_gain;
		md_gain = camera_mod[port]->camera_param.dol2.md_gain;

		s_gain_length = camera_mod[port]->camera_param.dol2.s_gain_length;
		sd_gain_length = camera_mod[port]->camera_param.dol2.sd_gain_length;
		m_gain_length = camera_mod[port]->camera_param.dol2.m_gain_length;
		md_gain_length = camera_mod[port]->camera_param.dol2.md_gain_length;
		switch (gain_num) {
			case 2:  // m_gain
				camera_trans_value(&input_gain[1], a_gain);
				if(md_gain == 0) {
					ret = camera_sys_write(port, m_gain, reg_width, a_gain, m_gain_length);
				} else {
					camera_trans_value(&dinput_gain[1], dig_gain);
					ret = camera_sys_write(port, m_gain, reg_width, a_gain, m_gain_length);
					ret = camera_sys_write(port, md_gain, reg_width, dig_gain, md_gain_length);
				}
			case 1:  // s_gain
				camera_trans_value(input_gain, a_gain);
				if(s_gain == 0) {
					ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
				} else {
					camera_trans_value(dinput_gain, dig_gain);
					ret = camera_sys_write(port, s_gain, reg_width, a_gain, s_gain_length);
					ret = camera_sys_write(port, sd_gain, reg_width, dig_gain, sd_gain_length);
				}
				break;
			default:
				break;
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
		case 1:  // s_line
			camera_trans_value(input_line, line_d);
			ret = camera_sys_write(port, s_line, reg_width, line_d, s_line_length);
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
	char dig_gain[3] = {0};
	uint32_t i = 0;
	int ret = 0;
	uint32_t reg_width, s_gain_length, m_gain_length, s_gain, m_gain;
	uint32_t l_gain_length, l_gain, ld_gain, ld_gain_length;
	uint32_t sd_gain, sd_gain_length, md_gain, md_gain_length;

	uint32_t num = camera_mod[port]->camera_param.sensor_data.turning_type;
	if (num == 6) {
		for (i = 0; i < camera_mod[port]->
			camera_param.dol3.again_control_num; i++) {
			reg_width = camera_mod[port]->camera_param.reg_width;
			s_gain_length = camera_mod[port]->camera_param.dol3.again_control_length[i];
			s_gain = camera_mod[port]->camera_param.dol3.again_control[i];
			camera_trans_value(&input_gain[i], gain_d);
			ret = camera_sys_write(port, s_gain, reg_width, gain_d, s_gain_length);
		}

		for (i = 0; i < camera_mod[port]->
			camera_param.dol3.dgain_control_num; i++) {
			reg_width = camera_mod[port]->camera_param.reg_width;
			s_gain_length = camera_mod[port]->camera_param.dol3.again_control_length[i];
			s_gain = camera_mod[port]->camera_param.dol3.dgain_control[i];
			camera_trans_value(&dinput_gain[i], gain_d);
			ret = camera_sys_write(port, s_gain, reg_width, gain_d, s_gain_length);
		}
	} else {
		reg_width = camera_mod[port]->camera_param.reg_width;
		s_gain = camera_mod[port]->camera_param.dol3.s_gain;
		m_gain = camera_mod[port]->camera_param.dol3.m_gain;
		l_gain = camera_mod[port]->camera_param.dol3.l_gain;
		sd_gain = camera_mod[port]->camera_param.dol3.sd_gain;
		md_gain = camera_mod[port]->camera_param.dol3.md_gain;
		ld_gain = camera_mod[port]->camera_param.dol3.ld_gain;

		s_gain_length = camera_mod[port]->camera_param.dol3.s_gain_length;
		m_gain_length = camera_mod[port]->camera_param.dol3.m_gain_length;
		l_gain_length = camera_mod[port]->camera_param.dol3.l_gain_length;
		sd_gain_length = camera_mod[port]->camera_param.dol3.sd_gain_length;
		md_gain_length = camera_mod[port]->camera_param.dol3.md_gain_length;
		ld_gain_length = camera_mod[port]->camera_param.dol3.ld_gain_length;
		switch (gain_num) {
			case 3:  // l_gain
				camera_trans_value(&input_gain[2], gain_d);
				if(ld_gain == 0) {
					ret = camera_sys_write(port, l_gain, reg_width, gain_d, l_gain_length);
				} else {
					camera_trans_value(&dinput_gain[2], dig_gain);
					ret = camera_sys_write(port, l_gain, reg_width, gain_d, l_gain_length);
					ret = camera_sys_write(port, ld_gain, reg_width, dig_gain, ld_gain_length);
				}
			case 2:  // m_gain
				camera_trans_value(&input_gain[1], gain_d);
				if(md_gain == 0) {
					ret = camera_sys_write(port, m_gain, reg_width, gain_d, m_gain_length);
				} else {
					camera_trans_value(&dinput_gain[1], dig_gain);
					ret = camera_sys_write(port, m_gain, reg_width, gain_d, m_gain_length);
					ret = camera_sys_write(port, md_gain, reg_width, dig_gain, md_gain_length);
				}
			case 1:  // s_gain
				camera_trans_value(input_gain, gain_d);
				if(sd_gain == 0) {
					ret = camera_sys_write(port, s_gain, reg_width, gain_d, s_gain_length);
				} else {
					camera_trans_value(dinput_gain, dig_gain);
					ret = camera_sys_write(port, s_gain, reg_width, gain_d, s_gain_length);
					ret = camera_sys_write(port, sd_gain, reg_width, dig_gain, sd_gain_length);
				}
				break;
			default:
				break;
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
		case 2:  // m_line
			camera_trans_value(&input_line[1], line_d);
			ret = camera_sys_write(port, m_line, reg_width, line_d, m_line_length);
		case 1:  // s_line
			camera_trans_value(input_line, line_d);
			ret = camera_sys_write(port, s_line, reg_width, line_d, s_line_length);
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

	if (camera_mod[port]->camera_param.sensor_data.turning_type >
		sizeof(sensor_ops)/sizeof(struct sensor_ctrl_ops)) {
		pr_info("gain line calculation out range\n");
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

	reg_width = camera_mod[port]->camera_param.reg_width;
	line_addr = camera_mod[port]->camera_param.pwl.line;
	line_length = camera_mod[port]->camera_param.pwl.line_length;

	switch (line_num) {
		case 1:
			camera_trans_value(&input_line[0], line_data);
			pr_debug("input_line[0] 0x%x input_line[1] 0x%x\n",
					line_data[0], line_data[1]);
			ret = camera_sys_write(port, line_addr, reg_width, line_data, line_length);
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
	uint32_t min_gain_time,	max_gain_time;

	uint32_t num = camera_mod[port]->camera_param.sensor_data.turning_type;
	if (num == 6) {
		for (i = 0; i < camera_mod[port]->
			camera_param.pwl.again_control_num; i++) {
			reg_width = camera_mod[port]->camera_param.reg_width;
			gain_length = camera_mod[port]->camera_param.pwl.again_control_length[i];
			gain_addr = camera_mod[port]->camera_param.pwl.again_control[i];
			camera_trans_value(&input_gain[i], gain_data);
			ret = camera_sys_write(port, gain_addr, reg_width, gain_data, gain_length);
		}
	} else {
		reg_width = camera_mod[port]->camera_param.reg_width;
		gain_addr = camera_mod[port]->camera_param.pwl.gain;
		gain_length = camera_mod[port]->camera_param.pwl.gain_length;
		min_gain_time = camera_mod[port]->camera_param.pwl.min_gain_time;
		max_gain_time = camera_mod[port]->camera_param.pwl.max_gain_time;
		switch(gain_num) {
			case 1:
				if((input_gain[0] < min_gain_time) && (min_gain_time != 0))
					input_gain[0] = min_gain_time;
				if((input_gain[0] > max_gain_time) && (max_gain_time != 0))
					input_gain[0] = max_gain_time;
				gain_data[0] = (char)((input_gain[0] >> 8 )& 0xff);
				gain_data[1] = (char)(input_gain[0] & 0xff);
				pr_info("gain_data[0] 0x%x, gain_data[1] 0x%x\n",
						gain_data[0], gain_data[1]);
				ret = camera_sys_write(port, gain_addr, reg_width, gain_data, gain_length);
				break;
			default:
				break;
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
	uint32_t reg_width, gain_addr, gain_length, dc_gain_addr, dc_gain_length;
	uint32_t min_gain_time, max_gain_time;

	uint32_t num = camera_mod[port]->camera_param.sensor_data.turning_type;
	if (num == 6) {
		for (i = 0; i < camera_mod[port]->
			camera_param.pwl.dgain_control_num; i++) {
			reg_width = camera_mod[port]->camera_param.reg_width;
			gain_length = camera_mod[port]->camera_param.pwl.dgain_control_length[i];
			gain_addr = camera_mod[port]->camera_param.pwl.dgain_control[i];
			camera_trans_value(&input_dgain[i], gain_data);
			ret = camera_sys_write(port, gain_addr, reg_width, gain_data, gain_length);
		}
	} else {
		reg_width = camera_mod[port]->camera_param.reg_width;
		gain_addr = camera_mod[port]->camera_param.pwl.sd_gain;
		gain_length = camera_mod[port]->camera_param.pwl.sd_gain_length;
		min_gain_time = camera_mod[port]->camera_param.pwl.min_dgain_time;
		max_gain_time = camera_mod[port]->camera_param.pwl.max_dgain_time;
		dc_gain_addr = camera_mod[port]->camera_param.pwl.dc_gain;
		dc_gain_length = camera_mod[port]->camera_param.pwl.dc_gain_length;

		switch(gain_num) {
			case 1:
				if(input_dgain[0] < min_gain_time)
					input_dgain[0] = min_gain_time; //512==0x200 1 time
				if(input_dgain[0] > max_gain_time)	//1536
					input_dgain[0] = max_gain_time;
				gain_data[0] = (char)((input_dgain[0] >> 8 )& 0xff);
				gain_data[1] = (char)(input_dgain[0] & 0xff);
				pr_info("dgain_data[0] 0x%x, dgain_data[1] 0x%x\n",
						gain_data[0], gain_data[1]);
				ret = camera_sys_write(port, gain_addr, reg_width, gain_data, gain_length);

				if (dc_gain_addr != 0) {
					gain_data[0] = (char)((input_dgain[3] >> 8 )& 0xff);
					gain_data[1] = (char)(input_dgain[3] & 0xff);
					pr_info("dc_gain_data[0] 0x%x, dc_gain_data[1] 0x%x\n",
								gain_data[0], gain_data[1]);
					ret = camera_sys_write(port, dc_gain_addr, reg_width,
								gain_data, dc_gain_length);
				}
				break;
			default:
				break;
		}
	}
	return ret;
}

int camera_sys_set_ex_gain_control(uint32_t port, sensor_priv_t *priv_param,
		uint32_t *input_gain, uint32_t *input_dgain, uint32_t *input_line)
{
	int ret = 0;
	char buf[2];
	uint32_t param_hold, reg_width, param_hold_length;

	reg_width = camera_mod[port]->camera_param.reg_width;
	param_hold = camera_mod[port]->camera_param.pwl.param_hold;
	param_hold_length = camera_mod[port]->camera_param.pwl.param_hold_length;

	buf[0] = 0x00;
	buf[1] = 0x01;
	ret = camera_sys_write(port, param_hold, reg_width, buf, param_hold_length);
	camera_sys_set_pwl_line(port, priv_param->line_num, input_line);
	camera_sys_set_pwl_gain(port, priv_param->gain_num, input_gain);
	camera_sys_set_pwl_dgain(port, priv_param->gain_num, input_dgain);
	buf[0] = 0x00;
	buf[1] = 0x00;
	ret = camera_sys_write(port, param_hold, reg_width, buf, param_hold_length);

	return ret;
}

int  camera_sys_set_param_hold(uint32_t port, uint32_t value)
{
	int ret = 0;
	uint32_t param_hold = 0, param_hold_length = 0;
	int  reg_width;
	char buf[2];

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
		default:
			pr_err("[%s -- %d ] mode is err %d !", __func__, __LINE__,
					camera_mod[port]->camera_param.mode);
			ret = -1;
			break;
	}
	if(param_hold != 0) {
		if(value) {
			buf[0] = 0x01;
			ret = camera_sys_write(port, param_hold, reg_width, buf, param_hold_length);
		} else {
			buf[0] = 0x00;
			ret = camera_sys_write(port, param_hold, reg_width, buf, param_hold_length);
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

	camera_sys_gain_line_process(port, priv_param, a_gain, d_gain, a_line);
	switch(camera_mod[port]->camera_param.mode) {
		case NORMAL_M:
			camera_sys_set_param_hold(port, 0x1);
			camera_sys_set_normal_gain(port, a_gain, d_gain);
			camera_sys_set_normal_line(port, a_line);
			camera_sys_set_param_hold(port, 0x0);
			break;
		case DOL2_M:
			camera_sys_set_param_hold(port, 0x1);
			camera_sys_set_dol2_gain(port, priv_param->gain_num, a_gain, d_gain);
			camera_sys_set_dol2_line(port, priv_param->line_num, a_line);
			camera_sys_set_param_hold(port, 0x0);
			break;
		case DOL3_M:
			camera_sys_set_param_hold(port, 0x1);
			camera_sys_set_dol3_gain(port, priv_param->gain_num, a_gain, d_gain);
			camera_sys_set_dol3_line(port, priv_param->line_num, a_line);
			camera_sys_set_param_hold(port, 0x0);
			break;
		case PWL_M:
			camera_sys_set_ex_gain_control(port, priv_param, a_gain, d_gain, a_line);
			break;
		default:
			pr_err("[%s -- %d ] mode is err %d !", __func__, __LINE__,
					camera_mod[port]->camera_param.mode);
			ret = -1;
			break;
	}

	return ret;
}

static uint32_t *camera_sys_lut_fill(uint32_t gain_num, uint32_t *turning_pram)
{
	uint32_t *gain_ptr = NULL;

	gain_ptr = kzalloc(256 * gain_num * sizeof(uint32_t), GFP_KERNEL);
	if (gain_ptr) {
		memcpy(gain_ptr, turning_pram, (256 * gain_num * sizeof(uint32_t)));
	}

	return gain_ptr;
}

int camera_sys_turining_set(uint32_t port, sensor_turning_data_t *turning_pram)
{
	int ret = 0;
	uint32_t *ptr = NULL;

	if (port > CAMERA_TOTAL_NUMBER) {
		return -1;
	} else {
		if (turning_pram) {
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

			memcpy(&camera_mod[port]->camera_param, turning_pram,
					sizeof(sensor_turning_data_t));
		} else {
			return -1;
		}
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

int camera_sys_priv_set(uint32_t port, sensor_priv_t *priv_param)
{
	int ret = 0;

	if (port > CAMERA_TOTAL_NUMBER) {
		ret = -1;
	} else {
		if (priv_param) {
			camera_sys_set_gain_line_control(port, priv_param);
		}
		else
			ret = -1;
	}

	return ret;
}

int camera_sys_get_param(uint32_t port, sensor_data_t *sensor_data)
{
	int ret = 0;

	memcpy(sensor_data, &camera_mod[port]->camera_param.sensor_data,
			sizeof(sensor_data_t));

	return ret;
}

int write_register(uint32_t port, uint8_t *pdata, int setting_size)
{
	int ret = 0;
	uint8_t tmp[32];
	uint16_t delay;
	int i, len, count;
	struct i2c_client client;
	client.adapter = camera_mod[port]->client->adapter;
	for (i = 0; i < setting_size;) {
		len = pdata[i];
		if (len == 5) {
			client.addr = pdata[i + 1] >> 1;
			tmp[0] = pdata[i + 2];
			tmp[1] = pdata[i + 3];
			tmp[2] = pdata[i + 4];
			tmp[3] = pdata[i + 5];
			count = 4;
			ret = i2c_master_send(&client, tmp, count);
			if (ret < 0) {
				pr_err("write 0231_register error\n");
				return ret;
			}
			i = i + len + 1;
			pr_info(" stream_on/off reg: 0x%x  stream_on/off value: 0x%x \n",
					tmp[0] << 8 | tmp[1], tmp[2] <<8 | tmp[3]);
		} else if (len == 4) {
			client.addr = pdata[i + 1] >> 1;
			tmp[0] = pdata[i + 2];
			tmp[1] = pdata[i + 3];
			tmp[2] = pdata[i + 4];
			count = 3;
			ret = i2c_master_send(&client, tmp, count);
			if (ret < 0) {
				pr_err("write max9296_register error\n");
				return ret;
			}
			i = i + len + 1;
			pr_info(" stream_on/off reg: 0x%x  stream_on/off value: 0x%x \n",
					tmp[0] << 8 | tmp[1], tmp[2]);

		} else if (len == 3) {
			client.addr = pdata[i + 1] >> 1;
			tmp[0] = pdata[i + 2];
			tmp[1] = pdata[i + 3];
			count = 2;
			ret = i2c_master_send(&client, tmp, count);
			if (ret < 0) {
				pr_err("write max96705_register error\n");
				return ret;
			}
			i = i + len + 1;
			pr_info(" stream_on/off reg: 0x%x  stream_on/off value: 0x%x \n",
					tmp[0], tmp[1]);

		} else if (len == 0) {
			delay = pdata[i + 1];
			msleep(delay);
			i = i + 2;
		}
	}
	return ret;
}

int camera_sys_stream_on(uint32_t port)
{
	int ret = 0, i;
	char buf[2];
	uint32_t reg_width, setting_size, data_length;
	uint32_t *stream_on = camera_mod[port]->camera_param.stream_ctrl.stream_on;
    uint32_t size =
    sizeof(camera_mod[port]->camera_param.stream_ctrl.stream_on);

	if (camera_mod[port]->camera_param.sensor_data.turning_type == 5) {
		ret = write_register(port, (uint8_t *)stream_on, size);
	} else {
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
	}

	return ret;
}

int camera_sys_stream_off(uint32_t port)
{
	int ret = 0, i;
	char buf[2];
	uint32_t reg_width, setting_size, data_length;
	uint32_t *stream_off = camera_mod[port]->camera_param.stream_ctrl.stream_off;
    uint32_t size =
    sizeof(camera_mod[port]->camera_param.stream_ctrl.stream_off);

	if (camera_mod[port]->camera_param.sensor_data.turning_type == 5) {
		ret = write_register(port, (uint8_t *)stream_off, size);
	} else {
		reg_width = camera_mod[port]->camera_param.reg_width;
		data_length = camera_mod[port]->camera_param.stream_ctrl.data_length;
		setting_size = size/sizeof(uint32_t)/2;
		pr_info("stream off setting_size %d\n", setting_size);
		for(i = 0; i < setting_size; i++) {
			pr_info(" stream_off[i*2] 0x%x stream_off[i*2 + 1] 0x%x \n",
					stream_off[i*2], stream_off[i*2 + 1]);
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
	}
	return ret;
}

int camera_sys_sensor_write(uint32_t port, uint32_t address, uint32_t w_data)
{
    int ret = 0;
    char buf[1];
    uint32_t reg_width;

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

    reg_width = camera_mod[port]->camera_param.reg_width;
    ret = camera_i2c_read(port, address, reg_width, buf, 1);
    *r_data = (uint32_t)(buf[0]);
    return ret;
}
