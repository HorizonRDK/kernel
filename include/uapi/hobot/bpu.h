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

	/* band data length if the struct is data head */
	uint32_t length;

	/* user set fc can run cores bit */
	uint64_t core_mask;

	/* real fc run cores bit, 0 is anyone*/
	uint64_t run_c_mask;

	/* group id*/
	uint32_t g_id;

	/* the follow element read after fc process done */
	/* 0: process succeed, not 0: error code */
	int32_t status;

	/* maybe the real time bpu core process */
	uint64_t process_time;
};

struct bpu_group {
	__u32 group_id;
	__u32 prop;
};
#define BPU_SET_GROUP _IOWR(BPU_IOC_MAGIC, 1, struct bpu_group)
#define BPU_GET_RATIO _IOWR(BPU_IOC_MAGIC, 2, __u32)
#define BPU_GET_CAP _IOWR(BPU_IOC_MAGIC, 3, __u16)
#define BPU_RESET _IOW(BPU_IOC_MAGIC, 4, __u16)

#endif
