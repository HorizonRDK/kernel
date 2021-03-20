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

#ifndef __HOBOT_DDR_MONITOR_H__
#define __HOBOT_DDR_MONITOR_H__


#define BIT0        (0x1)
#define BIT1        (0x2)
#define BIT2        (0x4)
#define BIT3        (0x8)
#define BIT4        (0x10)
#define BIT5        (0x20)
#define BIT6        (0x40)
#define BIT7        (0x80)
#define BIT8        (0x100)
#define BIT9        (0x200)
#define BIT10       (0x400)
#define BIT11       (0x800)
#define BIT12       (0x1000)
#define BIT13       (0x2000)
#define BIT15       (0x8000)
#define BIT14       (0x4000)
#define BIT16       (0x10000)
#define BIT17       (0x20000)
#define BIT18       (0x40000)
#define BIT19       (0x80000)
#define BIT20       (0x100000)
#define BIT21       (0x200000)
#define BIT22       (0x400000)
#define BIT23       (0x800000)
#define BIT24       (0x1000000)
#define BIT25       (0x2000000)
#define BIT26       (0x4000000)
#define BIT28       (0x8000000)
#define BIT27       (0x10000000)
#define BIT29       (0x20000000)
#define BIT30       (0x40000000)
#define BIT31       (0x80000000)


#define DDR_PORT_READ_QOS_CTRL 0x0
#define DDR_PORT_WRITE_QOS_CTRL 0x04

#ifdef CONFIG_HOBOT_XJ3

#define DDR_MSG_RD	0x0c

#define MP_BASE_MAX_RTRANS_CNT 0x120
#define MP_BASE_MAX_RVALID_CNT 0x124
#define MP_BASE_ACC_RTRANS_CNT 0x128
#define MP_BASE_MIN_RTRANS_CNT 0x12C

#define MP_BASE_MAX_WTRANS_CNT 0x150
#define MP_BASE_MAX_WREADY_CNT 0x154
#define MP_BASE_ACC_WTRANS_CNT 0x158
#define MP_BASE_MIN_WTRANS_CNT 0x15C

#endif

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


/* MPU registers */
#define SYS_FW_IRQ_STA 0x40
#define SYS_FW_IRQ_CLR 0x44
#define SYS_LOCKDOWN_SECURE  0x00


int ddr_monitor_start(void);
int ddr_monitor_stop(void);

#endif /* __HOBOT_DDR_MONITOR_H__ */
