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

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/completion.h>
#include <linux/seqlock.h>

#ifndef ISP_MAJOR
#define ISP_MAJOR   235
#endif /* ISP_MAJOR */

#ifndef ISP_NR_DEVS
#define ISP_NR_DEVS 1
#endif /* ISP_NR_DEVS */

#ifndef ISP_DEV_SIZE
#define ISP_DEV_SIZE    4096
#endif /* ISP_DEV_SIZE */

#define ISP_RUNNING         0x01
#define ISP_STOPPING        0x00

#include <linux/types.h>
#include <linux/wait.h>
#include "isp_base.h"
#include "x2/x2_ips.h"

struct isp_cfg_s {
	uint32_t isp_cfg;
};

struct isp_iodata_s {
	uint32_t u_datalen;
	uint32_t *p_databuffer;
};

struct isp_ioreg_s {
	uint32_t udatalen;
	uint32_t udataaddr;
	uint32_t *p_databuffer;
};

struct isp_stf_s {
	phys_addr_t isp_tile_addr;
	uint32_t isp_tile_size;
	phys_addr_t isp_grid_addr;
	uint32_t isp_grid_size;
	phys_addr_t isp_rsum_addr;
	uint32_t isp_rsum_size;
	phys_addr_t isp_hist_addr;
	uint32_t isp_hist_size;
	phys_addr_t isp_crd_addr;
	uint32_t isp_crd_size;
};

struct isp_mod_s {
	const char *name;
	int *pData;
	unsigned long size;

	struct completion isp_completion;
	wait_queue_head_t isp_waitq;
	int isp_condition;
	spinlock_t slock;
//      seqlock_t          seqlock;

	uint32_t get_data_delay;
	struct isp_stf_s isp_stf_memory;
	struct isp_cfg_s config;
	enum PING_PANG { ping = 1, pang } cdr_sw;
	uint32_t reserved_mem;

	uint32_t runflags;
	uint32_t irq_status;

	struct class *class;
	struct cdev mcdev;
	dev_t dev_num;
};

typedef struct _reg_s {
	uint32_t offset;
	uint32_t value;
} reg_t;

#define ISP_IOC_MAGIC       'h'

#define ISPC_START			_IO(ISP_IOC_MAGIC, 12)
#define ISPC_STOP			_IO(ISP_IOC_MAGIC, 13)
#define ISPC_GET_STATUS		_IOR(ISP_IOC_MAGIC, 2, uint32_t)
#define ISPC_READ_REG		_IOWR(ISP_IOC_MAGIC, 3, struct con_reg_s)
#define ISPC_WRITE_REG		_IOW(ISP_IOC_MAGIC, 4, struct con_reg_s)
#define ISPC_SET_WBG		_IOW(ISP_IOC_MAGIC, 5, struct ae_input_s)
#define ISPC_GET_ADDR		_IOR(ISP_IOC_MAGIC, 6, struct isp_stf_s)
#define ISPC_WRITE_STRING   _IOW(ISP_IOC_MAGIC, 7, struct isp_iodata_s)
#define ISPC_READ_STRING    _IOWR(ISP_IOC_MAGIC, 8, struct isp_iodata_s)
#define ISPC_READ_REGS      _IOWR(ISP_IOC_MAGIC, 9, struct isp_ioreg_s)
#define ISPC_WRITE_REGS     _IOWR(ISP_IOC_MAGIC, 10, struct isp_ioreg_s)
#define ISPC_WRITE_CDR      _IO(ISP_IOC_MAGIC, 11)

#define ISP_READ        _IOWR('p', 0, reg_t)
#define ISP_WRITE       _IOW('p', 1, reg_t)

int isp_model_init(void);
int isp_model_exit(void);


#endif /* __X2_ISP_H__ */
