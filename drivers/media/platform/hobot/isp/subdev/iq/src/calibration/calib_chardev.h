/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

#ifndef __CALIB_CHARDEV_H__
#define __CALIB_CHARDEV_H__


#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/ioctl.h>
#include <asm-generic/io.h>
#include <linux/string.h>
#include <linux/wait.h>


#include <linux/miscdevice.h>
#include <linux/sched.h>
#include "acamera_logger.h"
#include "acamera_types.h"
#include "acamera_firmware_config.h"
#include "acamera_firmware_settings.h"

#define CAMERA_CALIBMODE_NUM 1
#define CALIB_NUM_LENGTH 20


#define CALIB_INIT_ERR  (1)
#define CALIB_DESTORY_ERR (2)
#define CALIB_PORT_ERR (3)
#define CALIB_MALLOC_ERR (4)
#define CALIB_COPY_ERR (5)
#define CALIB_BUSY_ERR (6)
#define CALIB_SIZE_ERR (7)
#define CALIB_NULL_ERR (8)
#define CALIB_EXIST_ERR (9)



typedef struct camra_calib_s {
        char name[CALIB_NUM_LENGTH];
        void *pstr;
	void *plut;
        uint32_t tsize;
        uint32_t num;
	uint8_t port;
} camera_calib_t;

#define AC_CALIB_IOC_MAGIC      'd'
#define AC_CALIB_INIT           _IOW(AC_CALIB_IOC_MAGIC, 0, camera_calib_t)
#define AC_CALIB_RELEASE        _IOW(AC_CALIB_IOC_MAGIC, 1, camera_calib_t)
#define AC_CALIB_SETPART        _IOW(AC_CALIB_IOC_MAGIC, 2, camera_calib_t)
#define AC_CALIB_GETPART        _IOWR(AC_CALIB_IOC_MAGIC, 3, camera_calib_t)

struct calib_dev_s {
    uint8_t dev_inited;
    int dev_minor_id;
    char *dev_name;
    int dev_opened;

    struct miscdevice calib_dev;
    struct mutex fops_lock;
};

struct calib_data_s {
        char name[CALIB_NUM_LENGTH];
	LookupTable plut[CALIBRATION_TOTAL_SIZE];
        uint32_t tsize;
	uint8_t enable;
	uint8_t busy;
};

struct calib_param_s {
	uint8_t p_num;
        struct calib_data_s *plist[FIRMWARE_CONTEXT_NUMBER];
};




#endif /* __CALIB_CHARDEV_H__ */
