/*********************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 *********************************************************************/

#ifndef __LENS_PWM_H__
#define __LENS_PWM_H__

#include <linux/types.h>
#include <linux/pwm.h>

struct pwm_device *lens_pwm_request(uint32_t chn, const char *name,
	uint16_t dev_type, void *ops);
void lens_pwm_release(struct pwm_device *pwm_dev);

#endif /* __X3_LENS_PWM_H__ */
