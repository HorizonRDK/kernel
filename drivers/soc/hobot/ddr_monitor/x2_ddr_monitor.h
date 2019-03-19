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

#define DDR_PORT_READ_QOS_CTRL 0x0
#define DDR_PORT_WRITE_QOS_CTRL 0x04
#define DDR_PHY_DFI1_ENABLE_CTRL 0x08

#define PERF_MONITOR_ENABLE 0x20
#define PERF_MONITOR_PERIOD 0x24
#define PERF_MONITOR_SRCPND 0x28
#define PERF_MONITOR_ENABLE_INTMASK 0x2c
#define PERF_MONITOR_ENABLE_SETMASK 0x30
#define PERF_MONITOR_ENABLE_UNMASK 0x34

//#define MP_BASE_RID_CFG 0x100
#define MP_BASE_RADDR_TX_NUM 0x110
#define MP_BASE_RDATA_TX_NUM 0x114
#define MP_BASE_RADDR_ST_CYC 0x118
#define MP_BASE_RA2LSTRD_LATENCY 0x11c
//#define MP_BASE_RID_CFG 0x130
#define MP_BASE_WADDR_TX_NUM 0x140
#define MP_BASE_WDATA_TX_NUM 0x144
#define MP_BASE_WADDR_ST_CYC 0x148
#define MP_BASE_WA2BRESP_LATENCY 0x14c

#define MP_REG_OFFSET 0X100

#define RD_CMD_TX_NUM 0x700
#define WR_CMD_TX_NUM 0x704
#define MWR_CMD_TX_NUM 0x708
#define RDWR_SWITCH_NUM 0x70c
#define ACT_CMD_TX_NUM 0x710
#define ACT_CMD_TX_FOR_RD_TX_NUM 0x714
#define WAR_HAZARD_NUM 0x718
#define RAW_HAZARD_NUM 0x71c
#define WAW_HAZARD_NUM 0x720
#define PERCHARGE_CMD_TX_NUM 0x724
#define PERCHARGE_CMD_FOR_RDWR_TX_NUM 0x728

int ddr_monitor_start(void);
int ddr_monitor_stop(void);

#endif /* __X2_ISP_H__ */
