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

struct i2c_client *lens_i2c_request(uint32_t i2c_chn, uint32_t i2c_addr,
	const char *name, struct basic_control_ops **ops);
int lens_i2c_release(struct i2c_client *client);

#endif /* __LENS_I2C_H__ */
