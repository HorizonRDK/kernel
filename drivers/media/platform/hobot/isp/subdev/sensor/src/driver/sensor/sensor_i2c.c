/*
 *    driver, vb2_buffer interface
 *
 *    Copyright (C) 2018 Horizon Inc.
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

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include "acamera_logger.h"
#include "sensor_i2c.h"
#include "acamera_command_api.h"


#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME 
#define CUR_MOD_NAME LOG_MODULE_SOC_SENSOR
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_SENSOR
#endif


#ifdef SENSOR_IMX290
#include "./user_sensor/imx_290.h"
#endif

#ifdef SENSOR_IMX390
#include "./user_sensor/imx_390.h"
#endif

#ifdef SENSOR_IMX385
#include "./user_sensor/imx_385.h"
#endif

#ifdef SENSOR_IMX327
#include "./user_sensor/imx_327.h"
#endif

struct i2c_client *client[FIRMWARE_CONTEXT_NUMBER];
struct sensor_data;


static struct i2c_board_info board_info[] = {
	{
		.type = "imx390",
		.addr = 0x21,
	},
	{
		.type = "imx290",
		.addr = 0x36,
	},
	{
		.type = "imx385",
		.addr = 0x1a,
	},
	{
		.type = "imx327",
		.addr = 0x1a,
	},
	{
		.type = "OV8a10",
		.addr = 0x1a,
	},
	{
		.type = "OV5648",
		.addr = 0x36,
	},
	{
		.type = "OV8865",
		.addr = 0x36,
	},
};


struct sensor_operations *sensor_ops_register(uint8_t sensor_sw)
{
	struct sensor_operations *sensor_ctrl = NULL;

	switch (sensor_sw) {
#ifdef SENSOR_IMX290
	case 1:
		sensor_ctrl = imx290_ops_register(); 
		LOG(LOG_INFO, "IMX_290 register !");
		break;
#endif
#ifdef SENSOR_IMX385
	case 2:
		sensor_ctrl = imx385_ops_register();
		break;
		LOG(LOG_INFO, "IMX_385_register !");
#endif
#ifdef SENSOR_IMX327 
	case 3:
		sensor_ctrl = imx327_ops_register();
		LOG(LOG_INFO, "IMX_327_register !");
		break;
#endif
#ifdef SENSOR_IMX390 
	case 4:
		break;
#endif
	default:
		LOG(LOG_INFO, "sensor %d is not register !", sensor_sw);
		break;
	}

	return sensor_ctrl;
}

int sensor_chn_release(uint8_t chn)
{
	if (!client[chn])
		return -ENOMEM;
    
	i2c_unregister_device(client[chn]);
        client[chn] = NULL;
	LOG(LOG_INFO, "the %s  is close success !", client[chn]->name);

        return 0;
}

int sensor_i2c_open(uint8_t chn, uint32_t i2c_chn, uint32_t sensor_type)
{
        struct i2c_adapter *adap;
	int minor = 0;
	int ret = 0;

	if(client[chn]) {
		sensor_chn_release(chn);
	}	

	minor = i2c_chn;

        adap = i2c_get_adapter(minor);
        if (!adap) {
		LOG(LOG_ERR, "can not get i2c_adapter");
                return -ENODEV;
	}

	client[chn] = i2c_new_device(adap, &board_info[sensor_type]);//
        if (!client[chn]) {
		i2c_put_adapter(adap);
                return -ENOMEM;
        }
	
	//isp_mod = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
	//kzfree(isp_mod);
	LOG(LOG_INFO, "the %s is open success !", client[chn]->name);

        return 0;
}

struct sensor_operations *sensor_chn_open(uint8_t chn,
	uint32_t i2c_chn, uint8_t sensor_num)
{
	int ret = 0;
	struct sensor_operations *sensor_ctrl = NULL;
	
	ret = sensor_i2c_open(chn, i2c_chn, sensor_num);
	if (ret < 0)
		return NULL;

	sensor_ctrl = sensor_ops_register(sensor_num);
	if (sensor_ctrl == NULL) {
		LOG(LOG_ERR, " the sensor %d with i2c %d is failed ", sensor_num, i2c_chn);
		ret = sensor_chn_release(chn);
	}

        return sensor_ctrl;
}


int sensor_i2c_read(uint8_t chn, uint16_t reg_addr, uint8_t bit_width, char *buf, size_t count)
{
        char tmp[4];
        int ret = 0;
        
	if (count > 100)
                count = 100;
	if (!client[chn]) {
		LOG(LOG_ERR, "can not get client[%d]", chn);
		return -ENOMEM;
	}
	
	if (bit_width == 8) {
		tmp[0] = (char)(reg_addr & 0xff);
		ret = i2c_master_send(client[chn], tmp, 1);
		if (ret != 1)
			goto failed;
	} else {
		tmp[0] = (char)((reg_addr >> 8) & 0xff); 
		tmp[1] = (char)(reg_addr & 0xff);
		ret = i2c_master_send(client[chn], tmp, 2);
		if (ret != 2)
			goto failed;
	}
	
	if (ret < 0)
		return ret;
	
        ret = i2c_master_recv(client[chn], buf, count);
	
	if(ret != count) {
		LOG(LOG_ERR, "read failed !");
		ret = -1;
	}

        return ret;
failed:
	return -1;
}

int sensor_i2c_write(uint8_t chn, uint16_t reg_addr, uint8_t bit_width, const char *buf, size_t count)
{
        int ret;
        char tmp[102];
        
	if (count > 100)
                count = 100;
	if (!client[chn]) {
		LOG(LOG_ERR, "can not get client[%d]", chn);
		return -ENOMEM;
	}

	if (bit_width == 8) {
		tmp[0] = (char)(reg_addr & 0xff);
		memcpy(&tmp[1], buf, count);
		count += 1;
	}
	else {
		tmp[0] = (char)((reg_addr >> 8) & 0xff); 
		tmp[1] = (char)(reg_addr & 0xff);
		memcpy(&tmp[2], buf, count);
		count += 2;
	}

        ret = i2c_master_send(client[chn], tmp, count);
	
	if (ret != count) {
		LOG(LOG_INFO, "write failed !");
		ret = -1;
	}
        return ret;
}

