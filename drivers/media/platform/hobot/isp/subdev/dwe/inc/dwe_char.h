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

#ifndef __DWE_CHAR_H__
#define __DWE_CHAR_H__

#include <linux/types.h>

#define CHARDEVNAME_LEN  16


#define DWE_IOC_MAGIC           'h'

#define DWEC_SET_DIS_PARAM               _IO(DWE_IOC_MAGIC, 0)
#define DWEC_SET_LDC_PARAM               _IO(DWE_IOC_MAGIC, 1)
#define DWEC_GET_DIS_PARAM               _IO(DWE_IOC_MAGIC, 2)
#define DWEC_GET_LDC_PARAM               _IO(DWE_IOC_MAGIC, 3)
//#define DWEC_GET_STATUS         _IOWR(DWE_IOC_MAGIC, 2, uint32_t)


typedef struct _dwe_charmod_s {
	const char name[CHARDEVNAME_LEN];
	uint32_t devflag;
	spinlock_t slock;
	uint32_t user_num;

	int dev_minor_id;
	struct miscdevice dwe_chardev;
} dwe_charmod_s;


#endif /* __X2_ISP_H__ */
