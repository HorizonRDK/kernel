/*
 * Copyright (C) 2019 Horizon Robotics
 *
 * Zhang Guoying <guoying.zhang@horizon.ai>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */
#ifndef __HW_IO_H__
#define __HW_IO_H__
#include "bpu_core.h"

#if defined(CONFIG_X2_BPU) || defined(CONFIG_X3_BPU)
#include "hw_io_xj3.h"
#elif defined CONFIG_J5_BPU
#include "hw_io_j5.h"
#else
#define FC_MAX_DEPTH			1024
#define FC_DEPTH				FC_MAX_DEPTH
#define HW_ID_MAX	0xFFF
#define BPU_PRIO_NUM			0x1
#define FC_PRIO_ID(prio, id)	((id) & 0xFF) | ((prio) << 8)
#define FC_ID(id)				((id) & 0xFF)
#define FC_PRIO(id)				(0)
#endif

#define HEXBITS (16u)

/* normal register read/write */
uint32_t bpu_core_reg_read(const struct bpu_core *core, uint32_t offset);
void bpu_core_reg_write(const struct bpu_core *core,
		uint32_t offset, uint32_t val);

/* safe register read/write, use time redundancy method */
uint32_t bpu_core_safe_reg_read(const struct bpu_core *core, uint32_t offset);
int32_t bpu_core_safe_reg_write(const struct bpu_core *core,
		uint32_t offset, uint32_t val);

int32_t bpu_core_hw_reset(const struct bpu_core *core, uint32_t delay_time);

extern struct bpu_core_hw_ops hw_ops;

#endif
