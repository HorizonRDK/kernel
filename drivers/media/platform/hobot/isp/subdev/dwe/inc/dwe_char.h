/*
 *    dwe_char.h interface
 *
 *    Copyright (C) 2018 Horizon Inc.
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

#ifndef __DWE_CHAR_H__
#define __DWE_CHAR_H__

#include <linux/types.h>
#include <linux/mutex.h>
#include "buffer_vb2.h"

#define CHARDEVNAME_LEN  20

#define DWE_IOC_MAGIC    'h'
#define DWEC_SET_DIS_PARAM        _IOW(DWE_IOC_MAGIC, 0, dis_param_s)
#define DWEC_GET_DIS_PARAM        _IOR(DWE_IOC_MAGIC, 1, dis_param_s)
#define DWEC_SET_LDC_PARAM        _IOW(DWE_IOC_MAGIC, 2, ldc_param_s)
#define DWEC_GET_LDC_PARAM        _IOR(DWE_IOC_MAGIC, 3, ldc_param_s)
#define DWEC_SET_PG_PARAM         _IOW(DWE_IOC_MAGIC, 4, pg_param_s)
#define DWEC_GET_PG_PARAM         _IOR(DWE_IOC_MAGIC, 5, pg_param_s)
#define DWEC_START_PG             _IO(DWE_IOC_MAGIC, 6)
//used by buffer
#define DWEC_REQBUFS    _IOWR(DWE_IOC_MAGIC,  7, struct v4l2_requestbuffers)
#define DWEC_QUERYBUF   _IOWR(DWE_IOC_MAGIC,  8, struct v4l2_buffer)
#define DWEC_DQBUF      _IOWR(DWE_IOC_MAGIC,  9, struct v4l2_buffer)
#define DWEC_QBUF       _IOWR(DWE_IOC_MAGIC, 10, struct v4l2_buffer)
#define DWEC_FORMAT     _IOW(DWE_IOC_MAGIC, 11, struct v4l2_format)

typedef struct _dwe_charmod_s {
	const char name[CHARDEVNAME_LEN];
	uint32_t devflag;
	spinlock_t slock;
	uint32_t user_num;
	int dev_minor_id;
	struct miscdevice dwe_chardev;
	uint32_t port;
	//buffer
	struct vb2_queue vb2_q;
	struct mutex mlock;
	dwe_v4l2_stream_t *pstream;
} dwe_charmod_s;


#endif /* __X2_ISP_H__ */
