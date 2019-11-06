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
#include <linux/module.h>

#include "inc/camera_dev.h"
#include "inc/camera_subdev.h"
#include "inc/camera_i2c.h"
#include "inc/camera_spi.h"
#include "inc/camera_sys_api.h"

extern 	uint32_t sensor_log10(uint32_t val);

void camera_sys_printk_disturing(sensor_turning_data_t *turing_param)
{
	pr_info("sensor_addr 0x%x, bus_type %d, reg_width %d",
	turing_param->sensor_addr, turing_param->bus_type, turing_param->reg_width);
	pr_info("s_gain 0x%x  s_gain_length %d \n", turing_param->normal.s_gain,
		turing_param->normal.s_gain_length);
	pr_info("s_line 0x%x s_line_length %d \n", turing_param->normal.s_line,
		turing_param->normal.s_line_length);
}
int camera_sys_write(uint32_t port, uint32_t reg_addr,
		uint32_t reg_width, const char *buf, uint32_t length)
{
	int ret = 0;

	if(camera_mod[port]->camera_param.bus_type == I2C_BUS) {
		ret = camera_i2c_write(port, reg_addr, reg_width, buf, length);
	} else if (camera_mod[port]->camera_param.bus_type == SPI_BUS) {
		ret = camera_spi_write(port, buf, length);
	}

	return ret;
}
int	camera_sys_alloc_again(uint32_t port, uint32_t *a_gain)
{
	int ret = 0;

	if(*a_gain >= camera_mod[port]->camera_param.sensor_data.analog_gain_max)
		*a_gain = camera_mod[port]->camera_param.sensor_data.analog_gain_max;

	return ret;
}

int	camera_sys_alloc_dgain(uint32_t port, uint32_t *d_gain)
{
	int ret = 0;

	if(*d_gain >= camera_mod[port]->camera_param.sensor_data.digital_gain_max)
		*d_gain = camera_mod[port]->camera_param.sensor_data.digital_gain_max;

	return ret;
}

int camera_sys_alloc_intergration_time(uint32_t port,
		uint32_t *intergration_time)
{
	int ret = 0;
	return ret;
}

static int camera_sys_set_normal_gain(uint32_t port, uint32_t *input_gain)
{
	char gain_d[3] = {0};
	char rev_d[3] = {0};
	int ret = 0;
	uint32_t reg_width, s_gain, s_gain_length;

	reg_width = camera_mod[port]->camera_param.reg_width;
	s_gain = camera_mod[port]->camera_param.normal.s_gain;
	s_gain_length = camera_mod[port]->camera_param.normal.s_gain_length;

	gain_d[0] = (char)(input_gain[0] & 0xff);
	gain_d[1] = (char)((input_gain[0] >> 8) & 0xff);
	gain_d[2] = (char)((input_gain[0] >> 16) & 0xff);
	ret = camera_sys_write(port, s_gain, reg_width, gain_d, s_gain_length);
	// ret = camera_i2c_read(port, s_gain, reg_width, rev_d, s_gain_length);
	// pr_info("rev_d[0] 0x%x rev_d[1] 0x%x rev_d[2] 0x%x\n",
	// rev_d[0], rev_d[1], rev_d[2]);

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

	if(camera_mod[port]->camera_param.sensor_data.VMAX != 0) {
		 input_line[0] = camera_mod[port]->camera_param.sensor_data.VMAX
			 - 1 - input_line[0];
	}
	line_d[0] = (char)(input_line[0] & 0xff);
	line_d[1] = (char)((input_line[0] >> 8) & 0xff);
	line_d[2] = (char)((input_line[0] >> 16) & 0xff);
	ret = camera_sys_write(port, s_line, reg_width, line_d, s_line_length);
	return ret;
}

static int camera_sys_set_dol2_gain(uint32_t port, uint32_t gain_num,
		uint32_t *input_gain)
{
	char gain_d[3] = {0};
	int ret = 0;
	uint32_t reg_width, s_gain_length, m_gain_length, s_gain, m_gain;

	reg_width = camera_mod[port]->camera_param.reg_width;
	s_gain = camera_mod[port]->camera_param.dol2.s_gain;
	m_gain = camera_mod[port]->camera_param.dol2.m_gain;
	s_gain_length = camera_mod[port]->camera_param.dol2.s_gain_length;
	m_gain_length = camera_mod[port]->camera_param.dol2.m_gain_length;
	switch (gain_num) {
		case 2:  // m_gain
			gain_d[0] = (char)(input_gain[1] & 0xff);
			gain_d[1] = (char)((input_gain[1] >> 8) & 0xff);
			gain_d[2] = (char)((input_gain[1] >> 16) & 0xff);
			ret = camera_sys_write(port, m_gain, reg_width, gain_d, m_gain_length);
		case 1:  // s_gain
			gain_d[0] = (char)(input_gain[0] & 0xff);
			gain_d[1] = (char)((input_gain[0] >> 8) & 0xff);
			gain_d[2] = (char)((input_gain[0] >> 16) & 0xff);
			ret = camera_sys_write(port, s_gain, reg_width, gain_d, s_gain_length);
			break;
		default:
			break;
	}

	return ret;
}

static int camera_sys_set_dol2_line(uint32_t port, uint32_t gain_num,
		uint32_t *input_line)
{
	char line_d[3] = {0};
	int ret = 0;
	uint32_t reg_width, s_line_length, m_line_length, s_line, m_line;

	reg_width = camera_mod[port]->camera_param.reg_width;
	s_line = camera_mod[port]->camera_param.dol2.s_line;
	m_line = camera_mod[port]->camera_param.dol2.m_line;
	s_line_length = camera_mod[port]->camera_param.dol2.s_line_length;
	m_line_length = camera_mod[port]->camera_param.dol2.m_line_length;
	switch (gain_num) {
		case 2:  // m_line
			if(camera_mod[port]->camera_param.sensor_data.VMAX != 0) {
		   		 input_line[1] = camera_mod[port]->camera_param.sensor_data.VMAX
					 - 1 - input_line[1];
			}
			line_d[0] = (char)(input_line[1] & 0xff);
			line_d[1] = (char)((input_line[1] >> 8) & 0xff);
			line_d[2] = (char)((input_line[1] >> 16) & 0xff);
			ret = camera_sys_write(port, m_line, reg_width, line_d, m_line_length);
		case 1:  // s_line
			if(camera_mod[port]->camera_param.sensor_data.RHS1 != 0) {
		   		 input_line[0] = camera_mod[port]->camera_param.sensor_data.RHS1
					 - 1 - input_line[0];
			}
			line_d[0] = (char)(input_line[0] & 0xff);
			line_d[1] = (char)((input_line[0] >> 8) & 0xff);
			line_d[2] = (char)((input_line[0] >> 16) & 0xff);
			ret = camera_sys_write(port, s_line, reg_width, line_d, s_line_length);
			break;
		default:
			break;
	}

	return ret;
}

static int camera_sys_set_dol3_gain(uint32_t port, uint32_t gain_num,
		uint32_t *input_gain)
{
	char gain_d[3] = {0};
	int ret = 0;
	uint32_t reg_width, s_gain_length, m_gain_length, s_gain, m_gain;
	uint32_t l_gain_length, l_gain;

	reg_width = camera_mod[port]->camera_param.reg_width;
	s_gain = camera_mod[port]->camera_param.dol3.s_gain;
	m_gain = camera_mod[port]->camera_param.dol3.m_gain;
	l_gain = camera_mod[port]->camera_param.dol3.l_gain;
	s_gain_length = camera_mod[port]->camera_param.dol3.s_gain_length;
	m_gain_length = camera_mod[port]->camera_param.dol3.m_gain_length;
	l_gain_length = camera_mod[port]->camera_param.dol3.l_gain_length;
	switch (gain_num) {
		case 3:  // l_gain
			gain_d[0] = (char)(input_gain[2] & 0xff);
			gain_d[1] = (char)((input_gain[2] >> 8) & 0xff);
			gain_d[2] = (char)((input_gain[2] >> 16) & 0xff);
			ret = camera_sys_write(port, l_gain, reg_width, gain_d, l_gain_length);
		case 2:  // m_gain
			gain_d[0] = (char)(input_gain[1] & 0xff);
			gain_d[1] = (char)((input_gain[1] >> 8) & 0xff);
			gain_d[2] = (char)((input_gain[1] >> 16) & 0xff);
			ret = camera_sys_write(port, m_gain,  reg_width, gain_d, m_gain_length);
		case 1:  // s_gain
			gain_d[0] = (char)(input_gain[0] & 0xff);
			gain_d[1] = (char)((input_gain[0] >> 8) & 0xff);
			gain_d[2] = (char)((input_gain[0] >> 16) & 0xff);
			ret = camera_sys_write(port, s_gain, reg_width, gain_d, s_gain_length);
			break;
		default:
			break;
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
			if(camera_mod[port]->camera_param.sensor_data.VMAX != 0) {
		   		 input_line[2] = camera_mod[port]->camera_param.sensor_data.VMAX
					 - 1 - input_line[2];
			}
			line_d[0] = (char)(input_line[2] & 0xff);
			line_d[1] = (char)((input_line[2] >> 8) & 0xff);
			line_d[2] = (char)((input_line[2] >> 16) & 0xff);
			ret = camera_sys_write(port, l_line, reg_width, line_d, l_line_length);
		case 2:  // m_line
			if(camera_mod[port]->camera_param.sensor_data.RHS2 != 0) {
		   		 input_line[1] = camera_mod[port]->camera_param.sensor_data.RHS2
					 - 1 - input_line[1];
			}
			line_d[0] = (char)(input_line[1] & 0xff);
			line_d[1] = (char)((input_line[1] >> 8) & 0xff);
			line_d[2] = (char)((input_line[1] >> 16) & 0xff);
			ret = camera_sys_write(port, m_line, reg_width, line_d, m_line_length);
		case 1:  // s_line
			if(camera_mod[port]->camera_param.sensor_data.RHS1 != 0) {
		   		 input_line[0] = camera_mod[port]->camera_param.sensor_data.RHS1
					 - 1 - input_line[0];
			}
			line_d[0] = (char)(input_line[0] & 0xff);
			line_d[1] = (char)((input_line[0] >> 8) & 0xff);
			line_d[2] = (char)((input_line[0] >> 16) & 0xff);
			ret = camera_sys_write(port, s_line, reg_width, line_d, s_line_length);
			break;
		default:
			break;
	}

	return ret;
}

int  camera_sys_set_gain_line_control(uint32_t port, sensor_priv_t *priv_param)
{
	int ret = 0, i;
	uint32_t a_gain[3] = {0};
	uint32_t a_line[3] = {0};

	if(camera_mod[port]->camera_param.sensor_data.gain_type == 1) {
		for(i = 0; i < priv_param->gain_num; i++) {
			a_gain[i] = sensor_log10(priv_param->gain_buf[i]);  // ux.8 转化为DB
			a_gain[i] = (uint32_t)(((a_gain[i]* 200) / 3) >> 8);
		}
	} else if (camera_mod[port]->camera_param.sensor_data.gain_type == 2) {
		for(i = 0; i < priv_param->gain_num; i++) {
			a_gain[i] = sensor_log10(priv_param->gain_buf[i]);
			a_gain[i] = (uint32_t)(a_gain[i] << 8);
		}
	}
	for(i = 0; i < priv_param->line_num; i++) {
		a_line[i] = priv_param->line_buf[i];
	}
	switch(camera_mod[port]->camera_param.mode) {
		case NORMAL_M:
			camera_sys_set_normal_gain(port, a_gain);
			camera_sys_set_normal_line(port, a_line);
			break;
		case DOL2_M:
			camera_sys_set_dol2_gain(port, priv_param->gain_num, a_gain);
			camera_sys_set_dol2_line(port, priv_param->line_num, a_line);
			break;
		case DOL3_M:
			camera_sys_set_dol3_gain(port, priv_param->gain_num, a_gain);
			camera_sys_set_dol3_line(port, priv_param->line_num, a_line);
			break;
		default:
			pr_err("[%s -- %d ] mode is err !", __func__, __LINE__);
			ret = -1;
			break;
	}

	return ret;
}

int camera_sys_turining_set(uint32_t port, sensor_turning_data_t *turning_pram)
{
	int ret = 0;

	if (port > CAMERA_TOTAL_NUMBER) {
		ret = -1;
	} else {
		if (turning_pram)
			memcpy(&camera_mod[port]->camera_param, turning_pram,
					sizeof(sensor_turning_data_t));
		else
			ret = -1;
	}

	camera_sys_printk_disturing(&camera_mod[port]->camera_param);
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

	memcpy(&sensor_data, &camera_mod[port]->camera_param.sensor_data,
			sizeof(sensor_data_t));
	return ret;
}

