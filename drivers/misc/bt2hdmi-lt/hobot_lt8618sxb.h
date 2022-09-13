/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __HOBOT_LT8618SXBDEV_H__
#define __HOBOT_LT8618SXBDEV_H__

#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/fb.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#ifndef CONFIG_HOBOT_X3_UBUNTU
#include <soc/hobot/hobot_iar.h>
#endif
#define ENABLE 0x1
#define DISABLE 0x0

#define CONFIG_LT8618SXB_DEBUG (DISABLE)

#if (CONFIG_LT8618SXB_DEBUG == ENABLE)
#define LT8618SXB_DEBUG(format, args...) \
    pr_info("LT8618SXB debug: " format, ##args)
#else
#define LT8618SXB_DEBUG(format, args...)
#endif
#ifndef CONFIG_HOBOT_X3_UBUNTU
extern int display_type;
#endif
struct x2_lt8618sxb_s {
	struct i2c_client *client;
	struct mutex lt8618sxb_mutex;
};

typedef enum {
	_HDMI_480P60_ = 0,
	_HDMI_576P50_,

	_HDMI_720P60_,
	_HDMI_720P50_,
	_HDMI_720P30_,
	_HDMI_720P25_,

	_HDMI_1080P60_,
	_HDMI_1080P50_,
	_HDMI_1080P30_,
	_HDMI_1080P25_,

	_HDMI_1080i60_,
	_HDMI_1080i50_,

	_HDMI_4K30_,

	_HDMI_800x600P60_,
	_HDMI_1024x768P60_,
	_HDMI_1024x600_,
	_HDMI_800x480_,
	_HDMI_1366x768_
}LT8618_Resolution_Ratio;

typedef struct hobot_lt8618_sync {
	int hfp;
	int hs;
	int hbp;
	int hact;
	int htotal;
	int vfp;
	int vs;
	int vbp;
	int vact;
	int vtotal;
	int vic;
	int pic_ratio;
	int clk;
} hobot_lt8618_sync_t;

typedef struct hobot_lt8618_ioctl {
	spinlock_t		lock;
    LT8618_Resolution_Ratio ratio;
} hobot_lt8618_ioctl_t;

extern int LT8618SXB_Chip_ID(void);
extern void LT8618SX_Initial(void);
void Resolution_change(u8 Resolution);
int LT8618SXB_Read_EDID(hobot_lt8618_sync_t * sync);

#endif
