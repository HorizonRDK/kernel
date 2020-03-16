/*********************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 *********************************************************************/

#ifndef __LENS_OPS_H__
#define __LENS_OPS_H__

#include <linux/types.h>

struct motor_info {
	uint16_t max_step;
	uint32_t curr_pos;
	uint32_t next_pos;
	uint32_t init_pos;
	uint32_t min_pos;
	uint32_t max_pos;
};

struct motor_control_ops {
	void (*init)(void *ctx);
	void (*move)(void *ctx, uint32_t pos);
	void (*stop)(void *ctx);
	uint8_t (*is_moving)(void *ctx);
	uint16_t (*get_pos)(void *ctx);
	void (*calculate)(void *ctx, uint32_t param);
	void (*write_reg)(void *ctx, uint32_t addr, uint32_t data);
	uint32_t (*read_reg)(void *ctx, uint32_t addr);
	//const lens_param_t *(*get_param)(void *ctx);
};

struct basic_control_ops {
	void (*init)(void *ctx, void *param);
	void (*move)(void *ctx, void *param, uint32_t pos);
	void (*stop)(void *ctx, void *param);
	uint8_t (*is_moving)(void *ctx);
	void (*write_reg)(void *ctx, void *param, uint32_t addr, uint32_t data);
	uint32_t (*read_reg)(void *ctx, void *param, uint32_t addr);
};

#endif /* __X3_LENS_OPS_H__ */
