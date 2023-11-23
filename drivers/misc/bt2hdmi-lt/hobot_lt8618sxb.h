/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-03-23 15:04:36
 * @LastEditTime: 2023-03-24 16:10:58
 ***************************************************************************/
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
#ifdef CONFIG_HOBOT_X3_UBUNTU
#include <soc/hobot/hobot_iar.h>
#endif
#define ENABLE 0x1
#define DISABLE 0x0

#define CONFIG_LT8618SXB_DEBUG (ENABLE)

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



typedef struct hobot_lt8618_ioctl {
	spinlock_t		lock;
    LT8618_Resolution_Ratio ratio;
	hobot_hdmi_sync_t user_timing;
} hobot_lt8618_ioctl_t;

typedef struct edid_raw
{
	int block_num;
	u8 edid_data[256];
	u8 edid_data2[256];
} edid_raw_t;


extern edid_raw_t edid_raw_data;
extern int LT8618SXB_Chip_ID(void);
extern void LT8618SX_Initial(void);
void Resolution_change(hobot_hdmi_sync_t* timing);
int LT8618SXB_Read_EDID(hobot_hdmi_sync_t * sync);
#endif
