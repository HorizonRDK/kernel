/*********************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 *********************************************************************/

#ifndef __LENS_GPIO_H__
#define __LENS_GPIO_H__

#include <linux/types.h>
#include "lens_ops.h"

void lens_gpio_release(uint16_t gpio_a1, uint16_t gpio_a2,
	uint16_t gpio_b1, uint16_t gpio_b2);
int lens_gpio_request(uint16_t gpio_a1, uint16_t gpio_a2,
	uint16_t gpio_b1, uint16_t gpio_b2, const char *name,
	struct basic_control_ops **ops);

#endif /* __LENS_I2C_H__ */
