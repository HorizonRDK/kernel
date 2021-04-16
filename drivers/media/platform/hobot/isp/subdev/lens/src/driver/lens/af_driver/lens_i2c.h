/*********************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 *********************************************************************/

#ifndef __LENS_I2C_H__
#define __LENS_I2C_H__

#include <linux/types.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include "lens_ops.h"

typedef struct i2c_ctrl_param {
        uint16_t port;
        uint16_t control_type;//af, zoom
        uint16_t reg_width;
        uint16_t conversion;
        uint32_t reg_len;
        uint32_t reg_addr;
        uint32_t reserved_1;
        uint32_t reserved_2;
} i2c_ctrl_param_t;

struct i2c_client *lens_i2c_request(uint32_t i2c_chn, uint32_t i2c_addr,
	const char *name, struct basic_control_ops **ops);
void lens_i2c_release(struct i2c_client *client);

#endif /* __LENS_I2C_H__ */
