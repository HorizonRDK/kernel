/*********************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 *********************************************************************/

#ifndef __LENS_CHAR_H__
#define __LENS_CHAR_H__

#include <linux/types.h>
#include <linux/mutex.h>

#define LENS_CHARDEV_NAME "ac_lens"
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
		struct {
			uint16_t gpio_a1;  // A+
			uint16_t gpio_a2;  // A-
			uint16_t gpio_b1;  // B+
			uint16_t gpio_b2;  // B-
		} gpio_param;
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

struct motor_pos_set {
	uint16_t port;
	uint32_t pos;
};

#define LENS_IOC_MAGIC    'l'
#define LENS_AF_INIT          _IOW(LENS_IOC_MAGIC, 0, struct chardev_port_param)
#define LENS_ZOOM_INIT        _IOW(LENS_IOC_MAGIC, 1, struct chardev_port_param)
#define LENS_AF_RESET          _IOW(LENS_IOC_MAGIC, 2, uint32_t)
#define LENS_ZOOM_RESET        _IOW(LENS_IOC_MAGIC, 3, uint32_t)
#define LENS_AF_DEINIT          _IOW(LENS_IOC_MAGIC, 4, uint32_t)
#define LENS_ZOOM_DEINIT        _IOW(LENS_IOC_MAGIC, 5, uint32_t)
#define LENS_SET_I2C_PARAM         _IOW(LENS_IOC_MAGIC, 6, struct motor_i2c_param)
#define LENS_SET_AF_POS         _IOW(LENS_IOC_MAGIC, 7, struct motor_pos_set)
#define LENS_SET_ZOOM_POS         _IOW(LENS_IOC_MAGIC, 8, struct motor_pos_set)

#endif /* __X3_LENS_CHAR_H__ */
