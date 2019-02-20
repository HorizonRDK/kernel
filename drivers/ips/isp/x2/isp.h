/*
 * isp.h - interfaces internal to the isp framework
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __X2_ISP_H__
#define __X2_ISP_H__

#ifndef ISP_MAJOR
#define ISP_MAJOR   235
#endif /* ISP_MAJOR */

#ifndef ISP_NR_DEVS
#define ISP_NR_DEVS 1
#endif /* ISP_NR_DEVS */

#ifndef ISP_DEV_SIZE
#define ISP_DEV_SIZE    4096
#endif /* ISP_DEV_SIZE */

#include <linux/types.h>

struct isp_mod_s {
	int *pData;
	unsigned long size;
	const char *name;
	int major;
	int minor;
	struct cdev cdev;
	dev_t dev_num;
	struct class *isp_classes;
};

#endif /* __X2_ISP_H__ */
