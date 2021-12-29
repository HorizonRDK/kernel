/*    Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#ifndef DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_SERDES_DEV_H_
#define DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_SERDES_DEV_H_

#include <linux/miscdevice.h>
#include "inc/camera_subdev.h"

#define SERDES_IOC_MAGIC    	's'
#define SERDES_SET_INIT_CNT  	_IOW(SERDES_IOC_MAGIC, 1, int)
#define SERDES_GET_INIT_CNT  	_IOR(SERDES_IOC_MAGIC, 2, int)
#define SERDES_USER_LOCK      	_IOW(SERDES_IOC_MAGIC, 3, int)
#define SERDES_USER_UNLOCK    	_IOW(SERDES_IOC_MAGIC, 4, int)
#define SERDES_GPIO_CONTROL   	_IOW(SERDES_IOC_MAGIC, 5, gpio_info_t)
#define CHAR_DEVNAME_LEN  20
#define GPIO_MAX_NUM	  256		/*max gpio number*/

typedef enum _serdes_pre_state_t {
	SERDES_PRE_STATE_LOCK = 0,
	SERDES_PRE_STATE_UNLOCK,
} serdes_pre_state_t;

typedef struct _serdes_charmod_s {
	char name[CHAR_DEVNAME_LEN];
	uint32_t devflag;
	struct mutex slock;
	struct mutex user_mutex;
	uint32_t user_num;
	uint32_t init_num;
	int dev_minor_id;
	struct miscdevice serdes_chardev;
	uint32_t serdes_index;
	uint32_t pre_state;
	DECLARE_BITMAP(gpio_req_mask, GPIO_MAX_NUM);
} serdes_charmod_s;

int serdes_gpio_info_config(serdes_charmod_s *serdes_cdev,
		gpio_info_t *gpio_info);
void serdes_gpio_all_free(serdes_charmod_s *serdes_cdev);
int serdes_cdev_init(void);
void serdes_cdev_exit(void);
#endif // DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_SERDES_DEV_H_
