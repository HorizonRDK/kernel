/*********************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 *********************************************************************/

#ifndef __LENS_CHAR_H__
#define __LENS_CHAR_H__

#include <linux/types.h>
#include <linux/mutex.h>

#define CHARDEVNAME_LEN  20

struct chardev_port_param {
	uint16_t port;
	uint16_t motor_type;//pulses, pwm, i2c
	uint32_t max_step;
	uint32_t init_pos;
	uint32_t min_pos;
	uint32_t max_pos;
	union {
		struct {
			uint16_t pwm_num;
			uint32_t pwm_duty;
			uint32_t pwm_period;
		} pwm_param;
		struct {
			uint16_t pulse_forward_num;
			uint16_t pulse_back_num;
			uint32_t pulse_duty;
			uint32_t pulse_period;
		} pulse_param;
		struct {
			uint16_t i2c_num;
			uint32_t i2c_addr;
		} i2c_param;
	};
};

struct motor_i2c_param {
	uint16_t port;
	uint16_t control_type;//af, zoom
	uint32_t reg_len;
	uint32_t reg_addr;
	uint32_t reserved_1;
	uint32_t reserved_2;
};

#define LENS_IOC_MAGIC    'l'
#define LENS_SET_PORT_PARAM        _IOW(LNES_IOC_MAGIC, 0, struct chardev_port_param)
#define LENS_SET_I2C_PARAM         _IOW(LNES_IOC_MAGIC, 1, struct motor_i2c_param)

#endif /* __X3_LENS_CHAR_H__ */
