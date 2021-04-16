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

#ifndef _UAPI_HOBOT_BPU_H
#define _UAPI_HOBOT_BPU_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* The magic IOCTL value for this interface. */
#define BPU_IOC_MAGIC 'B'

struct user_bpu_fc {
	/* communicate id with user */
	uint32_t id;

	/* contained slice fc number*/
	uint32_t slice_num;

	/* band data length if the struct is data head */
	uint32_t length;

	/* user set fc can run cores bit */
	uint64_t core_mask;

	/* real fc run cores bit, 0 is anyone*/
	uint64_t run_c_mask;

	/* group id*/
	uint32_t g_id;

	uint32_t priority;

	/* the follow element read after fc process done */
	/* 0: process succeed, not 0: error code */
	int32_t status;

	/* maybe the real time bpu core process */
	uint64_t process_time;
};

struct bpu_group {
	uint32_t group_id;
	uint32_t prop;
	uint32_t ratio;
};

#define BPU_SET_GROUP _IOW(BPU_IOC_MAGIC, 1, struct bpu_group)
#define BPU_GET_RATIO _IOWR(BPU_IOC_MAGIC, 2, uint32_t)
#define BPU_GET_CAP _IOWR(BPU_IOC_MAGIC, 3, uint16_t)
#define BPU_RESET _IOW(BPU_IOC_MAGIC, 4, uint16_t)
#define BPU_GET_GROUP _IOWR(BPU_IOC_MAGIC, 5, struct bpu_group)
#define BPU_OPT_CORE _IOWR(BPU_IOC_MAGIC, 6, uint64_t)
#define BPU_SET_POWER _IOW(BPU_IOC_MAGIC, 7, int16_t)
#define BPU_SET_FREQ_LEVEL _IOW(BPU_IOC_MAGIC, 9, int16_t)
#define BPU_GET_FREQ_LEVEL _IOR(BPU_IOC_MAGIC, 10, int16_t)
#define BPU_GET_FREQ_LEVEL_NUM _IOR(BPU_IOC_MAGIC, 11, int16_t)
#define BPU_SET_LIMIT _IOW(BPU_IOC_MAGIC, 12, uint32_t)
#define BPU_SET_CLK _IOW(BPU_IOC_MAGIC, 13, uint64_t)
#define BPU_GET_CLK _IOR(BPU_IOC_MAGIC, 14, uint64_t)
#define BPU_CORE_TYPE _IOR(BPU_IOC_MAGIC, 15, uint8_t)

#endif
