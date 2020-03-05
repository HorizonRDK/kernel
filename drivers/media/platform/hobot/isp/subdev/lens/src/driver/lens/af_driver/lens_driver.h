/*********************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 *********************************************************************/

#ifndef __LENS_DRIVER_H__
#define __LENS_DRIVER_H__

#include <linux/types.h>
#include <lens_char.h>

enum lens_type {
	PWM_TYPE = 0,
	PULSE_TYPE,
	I2C_TYPE,
	SPI_TYPE,
};

enum lens_param_type {
	LENS_PARAM_MIN_ID = 100,
	LENS_AF_PARAM_ID,
	LENS_ZOOM_PARAM_ID,
	LENS_IRIS_PARAM_ID,
	LENS_PARAM_MAX_ID,
};

int lens_driver_init(uint16_t port, uint32_t param_id);
int lens_driver_move(uint16_t port, uint32_t param_id,  uint32_t pos);
uint8_t lens_driver_get_status(uint16_t port, uint32_t param_id);
uint32_t lens_driver_get_pos(uint16_t port, uint32_t param_id);
int lens_driver_get_param(uint16_t port, uint32_t param_id);
void lens_driver_write_reg(uint16_t port, uint32_t param_id,
	uint32_t addr, uint32_t data);
uint32_t lens_driver_read_reg(uint16_t port, uint32_t param_id, uint32_t addr);
void lens_driver_stop(uint16_t port, uint32_t param_id);
int set_af_param(uint16_t port, struct chardev_port_param *ctx);
int set_zoom_param(uint16_t port, struct chardev_port_param *ctx);


#endif /* __X3_LENS_DRIVER_H__ */
