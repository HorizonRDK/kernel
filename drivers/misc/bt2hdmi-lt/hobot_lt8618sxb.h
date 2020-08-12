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
#include <soc/hobot/hobot_iar.h>

#define ENABLE 0x1
#define DISABLE 0x0

#define CONFIG_LT8618SXB_DEBUG (DISABLE)

#if (CONFIG_LT8618SXB_DEBUG == ENABLE)
#define LT8618SXB_DEBUG(format, args...) \
    pr_info("LT8618SXB debug: " format, ##args)
#else
#define LT8618SXB_DEBUG(format, args...)
#endif

extern int display_type;

struct x2_lt8618sxb_s {
    struct i2c_client* client;
    struct mutex lt8618sxb_mutex;
};

extern int LT8618SX_Chip_ID(void);
extern void LT8618SX_Initial(void);

#endif
