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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/timer.h>
#include <linux/sched/signal.h>
#include <soc/hobot/hobot_mipi_host.h>
#include <soc/hobot/hobot_mipi_dphy.h>
#ifdef CONFIG_HOBOT_DIAG
#include <soc/hobot/diag.h>
#endif

#include "hobot_mipi_host_regs.h"
#include "hobot_mipi_utils.h"

#ifdef CONFIG_HOBOT_IPS_X2
#include "soc/hobot/hobot_ips_x2.h"
#endif

#define MIPI_HOST_DNAME		"mipi_host"
#define MIPI_HOST_MAX_NUM	CONFIG_HOBOT_MIPI_HOST_MAX_NUM
#define MIPI_HOST_CFGCLK_NAME	"mipi_cfg_host"
#define MIPI_HOST_CFGCLK_MHZ	24000000UL
#define MIPI_HOST_REFCLK_NAME	"mipi_host_ref"
#define MIPI_HOST_REFCLK_MHZ	24000000UL
#define MIPI_HOST_IPICLK_NAME(x)	"mipi_rx" __stringify(x) "_ipi"
#define MIPI_HOST_SNRCLK_NAME(x)	"sensor" __stringify(x) "_mclk"
#define MIPI_HOST_HW_MODE_DEF	0


#define MIPI_IPI_MASK_ALL (0xffffu)


static unsigned int port_num;
module_param(port_num, uint, 0444);

#ifdef MODULE
/* driver as ko without dts platform */
#define CONFIG_HOBOT_MIPI_PHY
#ifdef CONFIG_ARCH_HOBOT
#define MIPI_HOST_REG_ADDR (0xA4350000)
#else
#define MIPI_HOST_REG_ADDR (0xA0100000)
#endif
#define MIPI_HOST_REG_SIZE (0x00000800)

static unsigned int reg_addr = MIPI_HOST_REG_ADDR;
static unsigned int reg_size = MIPI_HOST_REG_SIZE;
static unsigned int init_num = MIPI_HOST_MAX_NUM;
static unsigned int hw_mode = 1;
module_param(reg_addr, uint, 0644);
module_param(reg_size, uint, 0644);
module_param(init_num, uint, 0644);
module_param(hw_mode, uint, 0644);
#endif

/* only hobot platform driver use irq */
#if defined MODULE || !defined CONFIG_ARCH_HOBOT
#define MIPI_HOST_INT_USE_TIMER
#endif

#define MIPI_HOST_INT_DBG		   (1)
#define MIPI_HOST_INT_DBG_ERRSTR   (1)
#define MIPI_HOST_SYSFS_FATAL_EN   (1)

#define MIPI_HOST_INT_PHY_FATAL    (0x1)
#define MIPI_HOST_INT_PKT_FATAL    (0x1<<1)
#define MIPI_HOST_INT_FRM_FATAL    (0x1<<2)
#define MIPI_HOST_INT_PHY		   (0x1<<16)
#define MIPI_HOST_INT_PKT		   (0x1<<17)
#define MIPI_HOST_INT_LINE		   (0x1<<18)
#define MIPI_HOST_INT_IPI		   (0x1<<19)
#define MIPI_HOST_INT_IPI2         (0x1<<20)
#define MIPI_HOST_INT_IPI3         (0x1<<21)
#define MIPI_HOST_INT_IPI4         (0x1<<22)

#define MIPI_HOST_1P4_INT_PHY_FATAL       (0x1)
#define MIPI_HOST_1P4_INT_PKT_FATAL       (0x1<<1)
#define MIPI_HOST_1P4_INT_BNDRY_FRM_FATAL (0x1<<2)
#define MIPI_HOST_1P4_INT_SEQ_FRM_FATAL   (0x1<<3)
#define MIPI_HOST_1P4_INT_CRC_FRM_FATAL   (0x1<<4)
#define MIPI_HOST_1P4_INT_PLD_CRC_FATAL   (0x1<<5)
#define MIPI_HOST_1P4_INT_DATA_ID         (0x1<<6)
#define MIPI_HOST_1P4_INT_ECC_CORRECTED   (0x1<<7)
#define MIPI_HOST_1P4_INT_PHY             (0x1<<16)
#define MIPI_HOST_1P4_INT_LINE            (0x1<<17)
#define MIPI_HOST_1P4_INT_IPI             (0x1<<18)
#define MIPI_HOST_1P4_INT_IPI2            (0x1<<19)
#define MIPI_HOST_1P4_INT_IPI3            (0x1<<20)
#define MIPI_HOST_1P4_INT_IPI4            (0x1<<21)

#define MIPI_HOST_CSI2_RAISE	   (0x01)
#define MIPI_HOST_CSI2_RESETN	   (0x00)
#define MIPI_HOST_MEMFLUSN_ENABLE  (0x01 << 8)
#define MIPI_HOST_IPI_DT_MASK	   (0x3f)
#define MIPI_HOST_EMB_DATA		   (0x01 << 8)
#define MIPI_HOST_BITWIDTH_48	   (0x00 << 8)
#define MIPI_HOST_BITWIDTH_16	   (0x01 << 8)
#define MIPI_HOST_CUT_THROUGH	   (0x01 << 16)
#define MIPI_HOST_IPI_ENABLE	   (0x01 << 24)
#define MIPI_HOST_IPI_DISABLE	   (0x00)
#define MIPI_HOST_LEGCYMODE_ENABLE (0x01 << 24)
#define MIPI_HOST_HSATIME		   (0x04)
#define MIPI_HOST_HBPTIME		   (0x04)
#define MIPI_HOST_HSDTIME		   (0x5f4)
#define MIPI_HOST_HSDTIME_MAX	   (0xfff)
#define MIPI_HOST_CFGCLK_DEFAULT   (0x1C)
#define MIPI_HOST_IPI1_SOFTRSTN	   (0x01 << 0)
#define MIPI_HOST_IPI2_SOFTRSTN	   (0x01 << 4)
#define MIPI_HOST_IPI3_SOFTRSTN	   (0x01 << 8)
#define MIPI_HOST_IPI4_SOFTRSTN	   (0x01 << 12)
#define MIPI_HOST_ALLE_SOFTRSTN    ((uint32_t)(0x1111UL))
#define MIPI_HOST_VC_EXT_LEGACY    (0x01)
#define MIPI_HOST_VC_EXT_ENABLE    (0x00)

#define MIPI_HOST_ADV_DEFAULT      (0x3 << 16)
#define MIPI_HOST_CUT_DEFAULT      (1)
#define MIPI_HOST_IPILIMIT_DEFAULT (102000000UL)
#define MIPI_HOST_SIGFUNCFG_DEFAULT (0x0)
#define MIPI_HOST_SIGWAITMS_DEFAULT (0)
#define MIPI_HOST_IRQ_CNT          (10)
#define MIPI_HOST_IRQ_DEBUG_PRERR  (0x1)
#define MIPI_HOST_IRQ_DEBUG_ERRSTR (0x2)
#define MIPI_HOST_IRQ_DEBUG        (0x1)
#define MIPI_HOST_SNRCLK_DISABLE   (0)
#define MIPI_HOST_SNRCLK_ENABLE    (1)
#define MIPI_HOST_SNRCLK_NOUSED    (2)
#define MIPI_HOST_SNRCLK_FREQ_MIN  (9281250UL)
#define MIPI_HOST_SNRCLK_FREQ_BASE (10000UL)

#define HOST_DPHY_LANE_MAX         (4)
#define HOST_DPHY_CHECK_MAX        (3000)
#define HOST_DPHY_LANE_STOP(l)     (0xF>>(HOST_DPHY_LANE_MAX-(l)))
#define HOST_DPHY_RX_HS            (0x030000)

#define MIPI_CSI2_DT_YUV420_8   (0x18)
#define MIPI_CSI2_DT_YUV420_10  (0x19)
#define MIPI_CSI2_DT_YUV422_8   (0x1E)
#define MIPI_CSI2_DT_YUV422_10  (0x1F)
#define MIPI_CSI2_DT_RGB565     (0x22)
#define MIPI_CSI2_DT_RGB888     (0x24)
#define MIPI_CSI2_DT_RAW_8      (0x2A)
#define MIPI_CSI2_DT_RAW_10     (0x2B)
#define MIPI_CSI2_DT_RAW_12     (0x2C)
#define MIPI_CSI2_DT_RAW_14     (0x2D)

#ifdef CONFIG_HOBOT_MIPI_REG_OPERATE
typedef struct _reg_s {
	uint32_t offset;
	uint32_t value;
} reg_t;

#define MIPIHOSTIOC_READ		_IOWR(MIPIHOSTIOC_MAGIC, 16, reg_t)
#define MIPIHOSTIOC_WRITE		_IOW(MIPIHOSTIOC_MAGIC, 17, reg_t)
#endif

typedef enum _mipi_state_t {
	MIPI_STATE_DEFAULT = 0,
	MIPI_STATE_INIT,
	MIPI_STATE_START,
	MIPI_STATE_STOP,
	MIPI_STATE_MAX,
} mipi_state_t;

static const char *g_mh_state[MIPI_STATE_MAX] = {
	"default",
	"init",
	"start",
	"stop",
};

typedef enum _mipi_pre_state_t {
	MIPI_PRE_STATE_DEFAULT = 0,
	MIPI_PRE_STATE_INITING,
	MIPI_PRE_STATE_INITED,
	MIPI_PRE_STATE_STARTING,
	MIPI_PRE_STATE_STARTED,
	MIPI_PRE_STATE_MAX,
} mipi_pre_state_t;

static const char *g_mh_pre_state[MIPI_PRE_STATE_MAX] = {
	"default",
	"initing",
	"inited",
	"starting",
	"started",
};

static const char *g_mh_ipiclk_name[] = {
	MIPI_HOST_IPICLK_NAME(0),
	MIPI_HOST_IPICLK_NAME(1),
	MIPI_HOST_IPICLK_NAME(2),
	MIPI_HOST_IPICLK_NAME(3),
};

static const char *g_mh_snrclk_name[] = {
	MIPI_HOST_SNRCLK_NAME(0),
	MIPI_HOST_SNRCLK_NAME(1),
	MIPI_HOST_SNRCLK_NAME(2),
	MIPI_HOST_SNRCLK_NAME(3),
};

typedef struct _mipi_host_snrclk_s {
	int                   index;
	int                   en_cnt;
	int                   sys_cnt;
	struct pinctrl       *pinctrl;
	struct pinctrl_state *enable;
	struct pinctrl_state *disable;
} mipi_host_snrclk_t;

typedef struct _mipi_host_param_s {
	/* type must be: uint32_t */
	uint32_t nocheck;
	uint32_t notimeout;
	uint32_t wait_ms;
	uint32_t dbg_value;
	uint32_t adv_value;
	uint32_t need_stop_check;
	uint32_t stop_check_instart;
	uint32_t cut_through;
	uint32_t ipi_16bit;
	uint32_t ipi_force;
	uint32_t ipi_limit;
	uint32_t snrclk_en;
	uint32_t snrclk_freq;
	uint32_t sigfun_cfg;
	uint32_t sigwait_ms;
	uint32_t sigfatal_mask;
	uint32_t vcext_en;
	uint32_t ipi1_dt;
#if MIPIHOST_CHANNEL_NUM >= 2
	uint32_t ipi2_dt;
#endif
#if MIPIHOST_CHANNEL_NUM >= 3
	uint32_t ipi3_dt;
#endif
#if MIPIHOST_CHANNEL_NUM >= 4
	uint32_t ipi4_dt;
#endif
#if MIPI_HOST_INT_DBG
	uint32_t irq_cnt;
	uint32_t irq_debug;
#endif
} mipi_host_param_t;

static const char *g_mh_param_names[] = {
	"nocheck",
	"notimeout",
	"wait_ms",
	"dbg_value",
	"adv_value",
	"need_stop_check",
	"stop_check_instart",
	"cut_through",
	"ipi_16bit",
	"ipi_force",
	"ipi_limit",
	"snrclk_en",
	"snrclk_freq",
	"sigfun_cfg",
	"sigwait_ms",
	"sigfatal_mask",
	"vcext_en",
	"ipi1_dt",
#if MIPIHOST_CHANNEL_NUM >= 2
	"ipi2_dt",
#endif
#if MIPIHOST_CHANNEL_NUM >= 3
	"ipi3_dt",
#endif
#if MIPIHOST_CHANNEL_NUM >= 4
	"ipi4_dt",
#endif
#if MIPI_HOST_INT_DBG
	"irq_cnt",
	"irq_debug",
#endif
};

#if MIPI_HOST_INT_DBG
typedef struct _mipi_host_icnt_s {
	/* type must be: uint32_t */
	uint32_t st_main;
	uint32_t phy_fatal;
	uint32_t pkt_fatal;
	uint32_t frm_fatal;
	uint32_t bndry_frm_fatal;
	uint32_t seq_frm_fatal;
	uint32_t crc_frm_fatal;
	uint32_t pld_crc_fatal;
	uint32_t data_id;
	uint32_t ecc_corrected;
	uint32_t phy;
	uint32_t pkt;
	uint32_t line;
	uint32_t ipi;
	uint32_t ipi2;
	uint32_t ipi3;
	uint32_t ipi4;
} mipi_host_icnt_t;

static const char *g_mh_icnt_names[] = {
	"st_main",
	"phy_fatal",
	"pkt_fatal",
	"frm_fatal",
	"bndry_frm_fatal",
	"seq_frm_fatal",
	"crc_frm_fatal",
	"pld_crc_fatal",
	"data_id",
	"ecc_corrected",
	"phy",
	"pkt",
	"line",
	"ipi",
	"ipi2",
	"ipi3",
	"ipi4",
};

typedef struct _mipi_host_ireg_s {
	uint32_t icnt_n;
	uint32_t st_mask;
	uint32_t reg_st;
	uint32_t reg_mask;
	uint32_t reg_force;
	uint32_t err_mask;
#if MIPI_HOST_INT_DBG_ERRSTR
	const char* err_str[32];
#endif
} mipi_host_ireg_t;

static const mipi_host_ireg_t mh_int_regs_1p3[] = {
	{ 1, MIPI_HOST_INT_PHY_FATAL, REG_MIPI_HOST_INT_ST_PHY_FATAL,
		REG_MIPI_HOST_INT_MSK_PHY_FATAL, REG_MIPI_HOST_INT_FORCE_PHY_FATAL,
		0x0000000f,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "phy_errsotsynchs_0", "phy_errsotsynchs_1",
		"phy_errsotsynchs_2", "phy_errsotsynchs_3",
		"phy_errsotsynchs_4", "phy_errsotsynchs_5",
		"phy_errsotsynchs_6", "phy_errsotsynchs_7" },
#endif
	},
	{ 2, MIPI_HOST_INT_PKT_FATAL, REG_MIPI_HOST_INT_ST_PKT_FATAL,
		REG_MIPI_HOST_INT_MSK_PKT_FATAL, REG_MIPI_HOST_INT_FORCE_PKT_FATAL,
		0x0000010f,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "vc0_err_crc", "vc1_err_crc", "vc2_err_crc", "vc3_err_crc",
		NULL, NULL, NULL, NULL,
		NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
		"err_ecc_double" },
#endif
	},
	{ 3, MIPI_HOST_INT_FRM_FATAL, REG_MIPI_HOST_INT_ST_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_FRAME_FATAL,
		0x000f0f0f,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "err_f_bndry_match_vc0", "err_f_bndry_match_vc1",
		"err_f_bndry_match_vc2", "err_f_bndry_match_vc3",
		NULL, NULL, NULL, NULL,
		"err_f_seq_vc0", "err_f_seq_vc1",
		"err_f_seq_vc2", "err_f_seq_vc3",
		NULL, NULL, NULL, NULL,
		"err_frame_data_vc0", "err_frame_data_vc1",
		"err_frame_data_vc2", "err_frame_data_vc3" },
#endif
	},
	{ 10, MIPI_HOST_INT_PHY, REG_MIPI_HOST_INT_ST_PHY,
		REG_MIPI_HOST_INT_MSK_PHY, REG_MIPI_HOST_INT_FORCE_PHY,
		0x00ff00ff,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "phy_errsoths_0", "phy_errsoths_1",
		"phy_errsoths_2", "phy_errsoths_3",
		"phy_errsoths_4", "phy_errsoths_5",
		"phy_errsoths_6", "phy_errsoths_7",
		NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
		"phy_erresc_0", "phy_erresc_1",
		"phy_erresc_2", "phy_erresc_3",
		"phy_erresc_4", "phy_erresc_5",
		"phy_erresc_6", "phy_erresc_7" },
#endif
	},
	{ 12, MIPI_HOST_1P4_INT_LINE, REG_MIPI_HOST_INT_ST_LINE,
		REG_MIPI_HOST_INT_MSK_LINE, REG_MIPI_HOST_INT_FORCE_LINE,
		0x00ff00ff,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "err_l_bndry_match_di0", "err_l_bndry_match_di1",
		"err_l_bndry_match_di2", "err_l_bndry_match_di3",
		"err_l_bndry_match_di4", "err_l_bndry_match_di5",
		"err_l_bndry_match_di6", "err_l_bndry_match_di7",
		NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
		"err_l_seq_di0", "err_l_seq_di1",
		"err_l_seq_di2", "err_l_seq_di3",
		"err_l_seq_di4", "err_l_seq_di5",
		"err_l_seq_di6", "err_l_seq_di7" },
#endif
	},
	{ 13, MIPI_HOST_INT_IPI, REG_MIPI_HOST_INT_ST_IPI,
		REG_MIPI_HOST_INT_MSK_IPI, REG_MIPI_HOST_INT_FORCE_IPI,
		0x0000003f,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "pixel_if_fifo_underflow", "pixel_if_fifo_overflow",
		"pixel_if_frame_sync_err", "pixel_if_fifo_nempty_fs",
		"pixel_if_hline_err", "int_event_fifo_overflow" },
#endif
	},
	{ 14, MIPI_HOST_INT_IPI2, REG_MIPI_HOST_INT_ST_IPI2,
		REG_MIPI_HOST_INT_MSK_IPI2, REG_MIPI_HOST_INT_FORCE_IPI2,
		0x0000003f,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "pixel_if_fifo_underflow", "pixel_if_fifo_overflow",
		"pixel_if_frame_sync_err", "pixel_if_fifo_nempty_fs",
		"pixel_if_hline_err", "int_event_fifo_overflow" },
#endif
	},
	{ 15, MIPI_HOST_INT_IPI3, REG_MIPI_HOST_INT_ST_IPI3,
		REG_MIPI_HOST_INT_MSK_IPI3, REG_MIPI_HOST_INT_FORCE_IPI3,
		0x0000003f,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "pixel_if_fifo_underflow", "pixel_if_fifo_overflow",
		"pixel_if_frame_sync_err", "pixel_if_fifo_nempty_fs",
		"pixel_if_hline_err", "int_event_fifo_overflow" },
#endif
	},
	{ 16, MIPI_HOST_INT_IPI4, REG_MIPI_HOST_INT_ST_IPI4,
		REG_MIPI_HOST_INT_MSK_IPI4, REG_MIPI_HOST_INT_FORCE_IPI4,
		0x0000003f,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "pixel_if_fifo_underflow", "pixel_if_fifo_overflow",
		"pixel_if_frame_sync_err", "pixel_if_fifo_nempty_fs",
		"pixel_if_hline_err", "int_event_fifo_overflow" },
#endif
	}
};

static const mipi_host_ireg_t mh_int_regs_1p4[] = {
	{ 1, MIPI_HOST_1P4_INT_PHY_FATAL, REG_MIPI_HOST_INT_ST_PHY_FATAL,
		REG_MIPI_HOST_INT_MSK_PHY_FATAL, REG_MIPI_HOST_INT_FORCE_PHY_FATAL,
		0x0000000f, /* ignore err_deskew */
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "phy_errsotsynchs_0", "phy_errsotsynchs_1",
		"phy_errsotsynchs_2", "phy_errsotsynchs_3",
		"phy_errsotsynchs_4", "phy_errsotsynchs_5",
		"phy_errsotsynchs_6", "phy_errsotsynchs_7",
		"err_deskew" },
#endif
	},
	{ 2, MIPI_HOST_1P4_INT_PKT_FATAL, REG_MIPI_HOST_INT_ST_PKT_FATAL,
		REG_MIPI_HOST_INT_MSK_PKT_FATAL, REG_MIPI_HOST_INT_FORCE_PKT_FATAL,
		0x00000001,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "err_ecc_double" },
#endif
	},
	{ 4, MIPI_HOST_1P4_INT_BNDRY_FRM_FATAL, REG_MIPI_HOST_INT_ST_BNDRY_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_BNDRY_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_BNDRY_FRAME_FATAL,
		0x0000ffff,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "err_f_bndry_match_vc0", "err_f_bndry_match_vc1",
		"err_f_bndry_match_vc2", "err_f_bndry_match_vc3",
		"err_f_bndry_match_vc4", "err_f_bndry_match_vc5",
		"err_f_bndry_match_vc6", "err_f_bndry_match_vc7",
		"err_f_bndry_match_vc8", "err_f_bndry_match_vc9",
		"err_f_bndry_match_vc10", "err_f_bndry_match_vc11",
		"err_f_bndry_match_vc12", "err_f_bndry_match_vc13",
		"err_f_bndry_match_vc14", "err_f_bndry_match_vc15",
		"err_f_bndry_match_vc16", "err_f_bndry_match_vc17",
		"err_f_bndry_match_vc18", "err_f_bndry_match_vc19",
		"err_f_bndry_match_vc20", "err_f_bndry_match_vc21",
		"err_f_bndry_match_vc22", "err_f_bndry_match_vc23",
		"err_f_bndry_match_vc24", "err_f_bndry_match_vc25",
		"err_f_bndry_match_vc26", "err_f_bndry_match_vc27",
		"err_f_bndry_match_vc28", "err_f_bndry_match_vc29",
		"err_f_bndry_match_vc30", "err_f_bndry_match_vc31" },
#endif
	},
	{ 5, MIPI_HOST_1P4_INT_SEQ_FRM_FATAL, REG_MIPI_HOST_INT_ST_SEQ_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_SEQ_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_SEQ_FRAME_FATAL,
		0x0000ffff,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "err_f_seq_vc0", "err_f_seq_vc1", "err_f_seq_vc2",
		"err_f_seq_vc3", "err_f_seq_vc4", "err_f_seq_vc5",
		"err_f_seq_vc6", "err_f_seq_vc7",
		"err_f_seq_vc8", "err_f_seq_vc9", "err_f_seq_vc10",
		"err_f_seq_vc11", "err_f_seq_vc12", "err_f_seq_vc13",
		"err_f_seq_vc14", "err_f_seq_vc15",
		"err_f_seq_vc16", "err_f_seq_vc17", "err_f_seq_vc18",
		"err_f_seq_vc19", "err_f_seq_vc20", "err_f_seq_vc21",
		"err_f_seq_vc22", "err_f_seq_vc23",
		"err_f_seq_vc24", "err_f_seq_vc25", "err_f_seq_vc26",
		"err_f_seq_vc27", "err_f_seq_vc28", "err_f_seq_vc29",
		"err_f_seq_vc30", "err_f_seq_vc31" },
#endif
	},
	{ 6, MIPI_HOST_1P4_INT_CRC_FRM_FATAL, REG_MIPI_HOST_INT_ST_CRC_FRAME_FATAL,
		REG_MIPI_HOST_INT_MSK_CRC_FRAME_FATAL, REG_MIPI_HOST_INT_FORCE_CRC_FRAME_FATAL,
		0x0000ffff,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "err_frame_data_vc0", "err_frame_data_vc1",
		"err_frame_data_vc2", "err_frame_data_vc3",
		"err_frame_data_vc4", "err_frame_data_vc5",
		"err_frame_data_vc6", "err_frame_data_vc7",
		"err_frame_data_vc8", "err_frame_data_vc9",
		"err_frame_data_vc10", "err_frame_data_vc11",
		"err_frame_data_vc12", "err_frame_data_vc13",
		"err_frame_data_vc14", "err_frame_data_vc15",
		"err_frame_data_vc16", "err_frame_data_vc17",
		"err_frame_data_vc18", "err_frame_data_vc19",
		"err_frame_data_vc20", "err_frame_data_vc21",
		"err_frame_data_vc22", "err_frame_data_vc23",
		"err_frame_data_vc24", "err_frame_data_vc25",
		"err_frame_data_vc26", "err_frame_data_vc27",
		"err_frame_data_vc28", "err_frame_data_vc29",
		"err_frame_data_vc30", "err_frame_data_vc31" },
#endif
	},
	{ 7, MIPI_HOST_1P4_INT_PLD_CRC_FATAL, REG_MIPI_HOST_INT_ST_PLD_CRC_FATAL,
		REG_MIPI_HOST_INT_MSK_PLD_CRC_FATAL, REG_MIPI_HOST_INT_FORCE_PLD_CRC_FATAL,
		0x0000ffff,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "err_crc_vc0", "err_crc_vc1", "err_crc_vc2",
		"err_crc_vc3", "err_crc_vc4", "err_crc_vc5",
		"err_crc_vc6", "err_crc_vc7",
		"err_crc_vc8", "err_crc_vc9", "err_crc_vc10",
		"err_crc_vc11", "err_crc_vc12", "err_crc_vc13",
		"err_crc_vc14", "err_crc_vc15",
		"err_crc_vc16", "err_crc_vc17", "err_crc_vc18",
		"err_crc_vc19", "err_crc_vc20", "err_crc_vc21",
		"err_crc_vc22", "err_crc_vc23",
		"err_crc_vc24", "err_crc_vc25", "err_crc_vc26",
		"err_crc_vc27", "err_crc_vc28", "err_crc_vc29",
		"err_crc_vc30", "err_crc_vc31" },
#endif
	},
	{ 8, MIPI_HOST_1P4_INT_DATA_ID, REG_MIPI_HOST_INT_ST_DATA_ID,
		REG_MIPI_HOST_INT_MSK_DATA_ID, REG_MIPI_HOST_INT_FORCE_DATA_ID,
		0x0000ffff,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "err_id_vc0", "err_id_vc1", "err_id_vc2",
		"err_id_vc3", "err_id_vc4", "err_id_vc5",
		"err_id_vc6", "err_id_vc7",
		"err_id_vc8", "err_id_vc9", "err_id_vc10",
		"err_id_vc11", "err_id_vc12", "err_id_vc13",
		"err_id_vc14", "err_id_vc15",
		"err_id_vc16", "err_id_vc17", "err_id_vc18",
		"err_id_vc19", "err_id_vc20", "err_id_vc21",
		"err_id_vc22", "err_id_vc23",
		"err_id_vc24", "err_id_vc25", "err_id_vc26",
		"err_id_vc27", "err_id_vc28", "err_id_vc29",
		"err_id_vc30", "err_id_vc31" },
#endif
	},
	{ 9, MIPI_HOST_1P4_INT_ECC_CORRECTED, REG_MIPI_HOST_INT_ST_ECC_CORRECT,
		REG_MIPI_HOST_INT_MSK_ECC_CORRECT, REG_MIPI_HOST_INT_FORCE_ECC_CORRECT,
		0x0000ffff,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "err_ecc_corrected0", "err_ecc_corrected1",
		"err_ecc_corrected2", "err_ecc_corrected3",
		"err_ecc_corrected4", "err_ecc_corrected5",
		"err_ecc_corrected6", "err_ecc_corrected7",
		"err_ecc_corrected8", "err_ecc_corrected9",
		"err_ecc_corrected10", "err_ecc_corrected11",
		"err_ecc_corrected12", "err_ecc_corrected13",
		"err_ecc_corrected14", "err_ecc_corrected15",
		"err_ecc_corrected16", "err_ecc_corrected17",
		"err_ecc_corrected18", "err_ecc_corrected19",
		"err_ecc_corrected20", "err_ecc_corrected21",
		"err_ecc_corrected22", "err_ecc_corrected23",
		"err_ecc_corrected24", "err_ecc_corrected25",
		"err_ecc_corrected26", "err_ecc_corrected27",
		"err_ecc_corrected28", "err_ecc_corrected29",
		"err_ecc_corrected30", "err_ecc_corrected31" },
#endif
	},
	{ 10, MIPI_HOST_1P4_INT_PHY, REG_MIPI_HOST_INT_ST_PHY,
		REG_MIPI_HOST_INT_MSK_PHY, REG_MIPI_HOST_INT_FORCE_PHY,
		0x000f000f,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "phy_errsoths_0", "phy_errsoths_1",
		"phy_errsoths_2", "phy_errsoths_3",
		"phy_errsoths_4", "phy_errsoths_5",
		"phy_errsoths_6", "phy_errsoths_7",
		NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
		"phy_erresc_0", "phy_erresc_1",
		"phy_erresc_2", "phy_erresc_3",
		"phy_erresc_4", "phy_erresc_5",
		"phy_erresc_6", "phy_erresc_7" },
#endif
	},
	{ 12, MIPI_HOST_1P4_INT_LINE, REG_MIPI_HOST_INT_ST_LINE,
		REG_MIPI_HOST_INT_MSK_LINE, REG_MIPI_HOST_INT_FORCE_LINE,
		0x00ff00ff,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "err_l_bndry_match_di0", "err_l_bndry_match_di1",
		"err_l_bndry_match_di2", "err_l_bndry_match_di3",
		"err_l_bndry_match_di4", "err_l_bndry_match_di5",
		"err_l_bndry_match_di6", "err_l_bndry_match_di7",
		NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
		"err_l_seq_di0", "err_l_seq_di1",
		"err_l_seq_di2", "err_l_seq_di3",
		"err_l_seq_di4", "err_l_seq_di5",
		"err_l_seq_di6", "err_l_seq_di7" },
#endif
	},
	{ 13, MIPI_HOST_1P4_INT_IPI, REG_MIPI_HOST_INT_ST_IPI,
		REG_MIPI_HOST_INT_MSK_IPI, REG_MIPI_HOST_INT_FORCE_IPI,
		0x0000003f,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "pixel_if_fifo_underflow", "pixel_if_fifo_overflow",
		"pixel_if_frame_sync_err", "pixel_if_fifo_nempty_fs",
		"pixel_if_hline_err", "int_event_fifo_overflow" },
#endif
	},
	{ 14, MIPI_HOST_1P4_INT_IPI2, REG_MIPI_HOST_INT_ST_IPI2,
		REG_MIPI_HOST_INT_MSK_IPI2, REG_MIPI_HOST_INT_FORCE_IPI2,
		0x0000003f,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "pixel_if_fifo_underflow", "pixel_if_fifo_overflow",
		"pixel_if_frame_sync_err", "pixel_if_fifo_nempty_fs",
		"pixel_if_hline_err", "int_event_fifo_overflow" },
#endif
	},
	{ 15, MIPI_HOST_1P4_INT_IPI3, REG_MIPI_HOST_INT_ST_IPI3,
		REG_MIPI_HOST_INT_MSK_IPI3, REG_MIPI_HOST_INT_FORCE_IPI3,
		0x0000003f,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "pixel_if_fifo_underflow", "pixel_if_fifo_overflow",
		"pixel_if_frame_sync_err", "pixel_if_fifo_nempty_fs",
		"pixel_if_hline_err", "int_event_fifo_overflow" },
#endif
	},
	{ 16, MIPI_HOST_1P4_INT_IPI4, REG_MIPI_HOST_INT_ST_IPI4,
		REG_MIPI_HOST_INT_MSK_IPI4, REG_MIPI_HOST_INT_FORCE_IPI4,
		0x0000003f,
#if MIPI_HOST_INT_DBG_ERRSTR
	  { "pixel_if_fifo_underflow", "pixel_if_fifo_overflow",
		"pixel_if_frame_sync_err", "pixel_if_fifo_nempty_fs",
		"pixel_if_hline_err", "int_event_fifo_overflow" },
#endif
	}
};

typedef struct _mipi_host_ierr_s {
	const mipi_host_ireg_t *iregs;
	uint32_t num;
} mipi_host_ierr_t;
#endif

typedef struct _mipi_host_port_hw_s {
	uint16_t group;
	uint16_t index;
	uint16_t lane_alone;
	uint16_t lane_group;
	uint16_t ipi;
} mipi_host_port_hw_t;

typedef struct _mipi_host_port_hw_mode_s {
	const mipi_host_port_hw_t *unit;
	uint16_t unum;
	uint16_t ugrp;
} mipi_host_hw_mode_t;

/**
 * comb 1 group by 2 ports:
 * port group index lane ipi
 * 0    0     0     2/4  4
 * 1    1     0     2/4  4
 * 2    0     1     2/0  2
 * 3    1     1     2/0  2
 * ...
 */
static const mipi_host_port_hw_t mh_port_hw_comb2[] = {
	{ 0, 0, 2, 4, 4 },
	{ 1, 0, 2, 4, 4 },
	{ 0, 1, 2, 0, 2 },
	{ 1, 1, 2, 0, 2 },
};

/**
 * alone 1 group with 1 port:
 * port group index lane ipi
 * 0    0     0     4    4
 * ...
 */
static const mipi_host_port_hw_t mh_port_hw_alone[] = {
	{ 0, 0, 4, 4, 4 },
};

/**
 * port hw mode:
 * 0: comb2, as default.
 * 1: alone.
 */
static const mipi_host_hw_mode_t g_mh_hw_modes[] = {
	{ mh_port_hw_comb2, ARRAY_SIZE(mh_port_hw_comb2), 2 },
	{ mh_port_hw_alone, ARRAY_SIZE(mh_port_hw_alone), 1 },
};

typedef struct _mipi_host_s {
	void __iomem     *iomem;
	int			      irq;
	mipi_state_t      state;
	mipi_host_cfg_t   cfg;
	mipi_host_param_t param;
	mipi_host_snrclk_t snrclk;
	/*
	 * spin_lock: reglock
	 * protect: iomem reg r&w.
	 * init: probe, see: hobot_mipi_host_probe_param, hobot_mipi_host_probe.
	 * call: ipi reset, see mipi_host_ipi_reset.
	 */
	spinlock_t 		  reglock;
#if MIPI_HOST_INT_DBG
	mipi_host_ierr_t ierr;
	mipi_host_icnt_t icnt;
#endif
} mipi_host_t;

typedef struct _mipi_user_s {
	/*
	 * mutex: user.open_mutex
	 * protect: user.open_cnt and operations when first open and last close.
	 * init: probe, see: hobot_mipi_host_probe_cdev.
	 * call: open/close, see: hobot_mipi_host_open, hobot_mipi_host_close.
	 */
	struct mutex open_mutex;
	/*
	 * mutex: user.mutex
	 * protect: user.init_cnt user.start_cnt user.pre_state and operations of mipi host.
	 * init: first open, see hobot_mipi_host_open.
	 * call: ioctl, see hobot_mipi_host_ioctl.
	 */
	struct mutex mutex;
	uint32_t open_cnt;
	uint32_t init_cnt;
	uint32_t start_cnt;
	uint32_t pre_state;
	bool pre_done;
	wait_queue_head_t pre_wq;
} mipi_user_t;

typedef struct _mipi_hdev_s {
	int               port;
	int               lane_mode;
	dev_t             devno;
	struct cdev       cdev;
	struct device    *dev;
	void             *ex_hdev;
	int               is_ex;
	uint64_t          ipi_clock;
	mipi_host_t       host;
	mipi_user_t       user;
	const mipi_host_hw_mode_t *hw_mode;
	/*
	 * spin_lock: siglock
	 * protect: struct sigpid.
	 * init: first open, see hobot_mipi_host_open.
	 * call: ioctl/irq, see hobot_mipi_host_ioctl, mipi_host_send_sig_fatal.
	 */
	spinlock_t 		  siglock;
	mipi_host_sigpid_t sigpid;
	unsigned long     sigts_ms;
#ifdef CONFIG_HOBOT_DIAG
	struct timer_list diag_timer;
	uint32_t          last_err_tm_ms;
#endif
#if MIPI_HOST_INT_DBG && defined MIPI_HOST_INT_USE_TIMER
	struct            timer_list irq_timer;
	uint32_t          irq_timer_en;
	uint32_t          irq_st_main;
#endif
} mipi_hdev_t;

static int g_mh_major;
static struct class *g_mh_class;
static mipi_hdev_t *g_hdev[MIPI_HOST_MAX_NUM];

static int mipi_host_port_group(mipi_hdev_t *hdev)
{
	int units, idx;

	if (!hdev->hw_mode)
		return (hdev->port);

	units = hdev->port / hdev->hw_mode->unum;
	idx = hdev->port % hdev->hw_mode->unum;

	return ((units * hdev->hw_mode->ugrp) + hdev->hw_mode->unit[idx].group);
}

static int mipi_host_port_index(mipi_hdev_t *hdev)
{
	int idx;

	if (!hdev->hw_mode)
		return 0;

	idx = hdev->port % hdev->hw_mode->unum;

	return (hdev->hw_mode->unit[idx].index);
}

static int mipi_host_port_other(mipi_hdev_t *hdev)
{
	int units, idx, i;

	if (!hdev->hw_mode)
		return (-1);

	units = hdev->port / hdev->hw_mode->unum;
	idx = hdev->port % hdev->hw_mode->unum;
	for (i = 0; i < hdev->hw_mode->unum; i++) {
		if (i != idx && hdev->hw_mode->unit[idx].group == hdev->hw_mode->unit[i].group)
			return ((units * hdev->hw_mode->unum) + i);
	}

	return (-1);
}

static int mipi_host_port_lane(mipi_hdev_t *hdev, int mode)
{
	int idx;

	if (!hdev->hw_mode)
		return 4;

	idx = hdev->port % hdev->hw_mode->unum;

	if (mode)
		return (hdev->hw_mode->unit[idx].lane_group);
	else
		return (hdev->hw_mode->unit[idx].lane_alone);
}

static int mipi_host_port_ipi(mipi_hdev_t *hdev)
{
	int idx;

	if (!hdev->hw_mode)
		return 4;

	idx = hdev->port % hdev->hw_mode->unum;

	return (hdev->hw_mode->unit[idx].ipi);
}

/**
 * @brief mipi_host_configure_lanemode: configure dphy for lane mode
 *
 * @param [in] lane : mipi host lane
 *
 * @return int32_t: 0/-1
 */
static int32_t mipi_host_configure_lanemode(mipi_hdev_t *hdev, int lane)
{
	if (!hdev)
		return -1;
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	int group, poth;
	int i, ret, target_mode = -1;

	for (i = 0; i < 2; i++) {
		if (lane <= mipi_host_port_lane(hdev, i)) {
			target_mode = i;
			break;
		}
	}

	/* upd hw lane_mode */
	hdev->lane_mode = mipi_dphy_get_lanemode(MIPI_DPHY_TYPE_HOST,
							hdev->port);

	group = mipi_host_port_group(hdev);
	poth = mipi_host_port_other(hdev);
	if(poth < 0)
		return -1;
	if (target_mode < 0) {
		mipierr("port%d not support %dlane",
				hdev->port, lane);
		return -1;
	} else if (target_mode == hdev->lane_mode) {
		goto match_ex_hdev;
	}

	mipiinfo("change group%d lane_mode to %d for %dlane",
			group, target_mode, lane);
	if (host->state != MIPI_STATE_DEFAULT) {
		mipierr("port%d busy error", hdev->port);
		return -1;
	}
	if (poth < MIPI_HOST_MAX_NUM && g_hdev[poth] &&
		g_hdev[poth]->host.state != MIPI_STATE_DEFAULT) {
		mipierr("port%d busy error", poth);
		return -1;
	}
	ret = mipi_dphy_set_lanemode(MIPI_DPHY_TYPE_HOST, hdev->port,
				target_mode);
	if (ret) {
		mipierr("dphy lane_mode ctrl error %d", ret);
		return -1;
	}

	/* sync lane_mode */
	hdev->lane_mode = target_mode;
	if (poth < MIPI_HOST_MAX_NUM && g_hdev[poth])
		g_hdev[poth]->lane_mode = target_mode;

match_ex_hdev:
	if (target_mode) {
		if (poth < MIPI_HOST_MAX_NUM && g_hdev[poth] &&
			g_hdev[poth]->host.state == MIPI_STATE_DEFAULT) {
			/* match ex_hdev and mark it as ex */
			hdev->ex_hdev = g_hdev[poth];
			hdev->is_ex = 0;
			(g_hdev[poth])->is_ex = 1;
		}
	} else {
		hdev->ex_hdev = NULL;
		hdev->is_ex = 0;
	}

	return 0;
}

/**
 * @brief mipi_host_configure_ipi : configure ipi mode of mipi host
 *
 * @param [in] cfg : mipi host cfgler's setting
 *
 * @return int32_t : 0/-1
 */
static int32_t mipi_host_configure_ipi(mipi_hdev_t *hdev, mipi_host_cfg_t *cfg)
{
	if (!hdev)
		return -1;
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	mipi_host_param_t *param = &host->param;
	void __iomem *iomem = host->iomem;
	int ipi_num, ipi_max;
	int vcid, datatype;
	uint32_t ipi_mode;

	if (!iomem)
		return -1;

	ipi_max = mipi_host_port_ipi(hdev);
	if (cfg->channel_num > ipi_max) {
		mipierr("channel_num %d error, max %d", cfg->channel_num, ipi_max);
		return -1;
	}

	ipi_mode = MIPI_HOST_IPI_ENABLE;
	ipi_mode |= (param->cut_through) ? MIPI_HOST_CUT_THROUGH : 0;
	ipi_mode |= (param->ipi_16bit) ? MIPI_HOST_BITWIDTH_16 : MIPI_HOST_BITWIDTH_48;
	mipidbg("ipi_mode %dbit%s", (param->ipi_16bit) ? 16 : 48,
		(param->cut_through) ? " cut_through" : "");

	ipi_num = 0;
	switch (ipi_max) {
	case 4:
#if MIPIHOST_CHANNEL_NUM >= 4
		/* ipi4 config */
		vcid = cfg->channel_sel[3];
		if (cfg->channel_num >= 4 && vcid < 4) {
			datatype = (param->ipi4_dt) ? param->ipi4_dt : cfg->datatype;
			if (param->ipi4_dt || vcid != 3)
				mipiinfo("ipi4 vc%d datatype 0x%02x%s", vcid,
					datatype & MIPI_HOST_IPI_DT_MASK,
					(datatype & MIPI_HOST_EMB_DATA) ? " as embedded" : "");
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_MEM_FLUSH,
				MIPI_HOST_MEMFLUSN_ENABLE);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_VCID, vcid);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_DATA_TYPE, datatype);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_MODE, ipi_mode);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_HSA_TIME,
				cfg->hsaTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_HBP_TIME,
				cfg->hbpTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_HSD_TIME,
				cfg->hsdTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_ADV_FEATURES,
				param->adv_value);
			ipi_num ++;
		} else {
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_MODE,
					MIPI_HOST_IPI_DISABLE);
		}
#endif
		/* no break */
	case 3:
#if MIPIHOST_CHANNEL_NUM >= 3
		/* ipi3 config */
		vcid = cfg->channel_sel[2];
		if (cfg->channel_num >= 3 && vcid < 4) {
			datatype = (param->ipi3_dt) ? param->ipi3_dt : cfg->datatype;
			if (param->ipi3_dt || vcid != 2)
				mipiinfo("ipi3 vc%d datatype 0x%02x%s", vcid,
					datatype & MIPI_HOST_IPI_DT_MASK,
					(datatype & MIPI_HOST_EMB_DATA) ? " as embedded" : "");
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_MEM_FLUSH,
				MIPI_HOST_MEMFLUSN_ENABLE);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_VCID, vcid);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_DATA_TYPE, datatype);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_MODE, ipi_mode);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_HSA_TIME,
				cfg->hsaTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_HBP_TIME,
				cfg->hbpTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_HSD_TIME,
				cfg->hsdTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_ADV_FEATURES,
				param->adv_value);
			ipi_num ++;
		} else {
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_MODE,
					MIPI_HOST_IPI_DISABLE);
		}
#endif
		/* no break */
	case 2:
#if MIPIHOST_CHANNEL_NUM >= 2
		/* ipi2 config */
		vcid = cfg->channel_sel[1];
		if (cfg->channel_num >= 2 && vcid < 4) {
			datatype = (param->ipi2_dt) ? param->ipi2_dt : cfg->datatype;
			if (param->ipi2_dt || vcid != 1)
				mipiinfo("ipi2 vc%d datatype 0x%02x%s", vcid,
					datatype & MIPI_HOST_IPI_DT_MASK,
					(datatype & MIPI_HOST_EMB_DATA) ? " as embedded" : "");
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_MEM_FLUSH,
				MIPI_HOST_MEMFLUSN_ENABLE);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_VCID, vcid);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_DATA_TYPE, datatype);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_MODE, ipi_mode);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_HSA_TIME,
				cfg->hsaTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_HBP_TIME,
				cfg->hbpTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_HSD_TIME,
				cfg->hsdTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_ADV_FEATURES,
				param->adv_value);
			ipi_num ++;
		} else {
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_MODE,
					MIPI_HOST_IPI_DISABLE);
		}
#endif
		/* no break */
	case 1:
	default:
		/* ipi1 config */
		vcid = cfg->channel_sel[0];
		if (vcid < 4) {
			datatype = (param->ipi1_dt) ? param->ipi1_dt : cfg->datatype;
			if (param->ipi1_dt || vcid != 0)
				mipiinfo("ipi1 vc%d datatype 0x%02x%s", vcid,
					datatype & MIPI_HOST_IPI_DT_MASK,
					(datatype & MIPI_HOST_EMB_DATA) ? " as embedded" : "");
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_MEM_FLUSH,
				MIPI_HOST_MEMFLUSN_ENABLE);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_VCID, vcid);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_DATA_TYPE, datatype);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_MODE, ipi_mode);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_HSA_TIME,
				cfg->hsaTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_HBP_TIME,
				cfg->hbpTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_HSD_TIME,
				cfg->hsdTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_ADV_FEATURES,
				param->adv_value);
			ipi_num ++;
		} else {
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_MODE,
					MIPI_HOST_IPI_DISABLE);
		}
		break;
	}

	/* default enable all ipis of mipi host */
	mipi_putreg(iomem + REG_MIPI_HOST_IPI_SOFTRSTN, MIPI_HOST_ALLE_SOFTRSTN);

	mipiinfo("config %d/%d ipi done", ipi_num, cfg->channel_num);
	return 0;
}

/**
 * @brief mipi_host_configure_cmp: configure compare.
 *
 * @param [in] scfg dcfg: mipi host configure
 *
 * @return int32_t: 0/-1
 */
static int32_t mipi_host_configure_cmp(mipi_host_cfg_t *scfg, mipi_host_cfg_t *dcfg)
{
	uint32_t i;

	if ((scfg == NULL) || (dcfg == NULL)) {
		/* do not need report */
		return -1;
	}

	if ((scfg->lane != dcfg->lane) ||
		(scfg->datatype != dcfg->datatype) ||
		(scfg->fps != dcfg->fps) ||
		(scfg->mclk != dcfg->mclk) ||
		(scfg->mipiclk != dcfg->mipiclk) ||
		(scfg->width != dcfg->width) ||
		(scfg->height != dcfg->height) ||
		(scfg->linelenth != dcfg->linelenth) ||
		(scfg->framelenth != dcfg->framelenth) ||
		(scfg->settle != dcfg->settle) ||
		((dcfg->hsaTime != 0U) && (scfg->hsaTime != dcfg->hsaTime)) ||
		((dcfg->hbpTime != 0U) && (scfg->hbpTime != dcfg->hbpTime)) ||
		((dcfg->hsdTime != 0U) && (scfg->hsdTime != dcfg->hsdTime)) ||
		(scfg->channel_num != dcfg->channel_num)) {
		/* do not need report */
		return -1;
	}
	for (i = 0U; (i < scfg->channel_num) && (i < (uint32_t)MIPIHOST_CHANNEL_NUM); i++) {
		if (scfg->channel_sel[i] != dcfg->channel_sel[i]) {
			/* do not need report */
			return -1;
		}
	}

	return 0;
}

/**
 * @brief mipi_host_ipi_reset: reset ipi of mipi host
 *
 * @param [in] ipi_reset: ipi reset struct.
 *
 * @return int32_t : 0/-1
 */
static int32_t mipi_host_ipi_reset(mipi_hdev_t *hdev, mipi_host_ipi_reset_t *ipi_reset)
{
	mipi_host_t *host = &hdev->host;
	mipi_host_cfg_t *cfg = &host->cfg;
	mipi_host_param_t *param = &host->param;
	struct device *dev = hdev->dev;
	void __iomem *iomem = host->iomem;
	int32_t i, ipi_max, done = 0;
	uint32_t softrstn, rstn_bit, reg_mode, ipi_mode;
	unsigned long flags;

	if (!ipi_reset || !iomem)
		return -1;

	ipi_max = mipi_host_port_ipi(hdev);
	for (i = 0; i < MIPIHOST_CHANNEL_NUM; i++) {
		if ((ipi_reset->mask & (0x1 << i)) == 0)
			continue;
		if (i >= ipi_max) {
			mipiinfo("%d:ipi%d reset: not support drop", i, i + 1);
			continue;
		} else if (i >= cfg->channel_num) {
			mipidbg("%d:ipi%d reset: not inited warning", i, i + 1);
		}
		switch (i) {
		case 3:
			reg_mode = REG_MIPI_HOST_IPI4_MODE;
			rstn_bit = MIPI_HOST_IPI4_SOFTRSTN;
			break;
		case 2:
			reg_mode = REG_MIPI_HOST_IPI3_MODE;
			rstn_bit = MIPI_HOST_IPI3_SOFTRSTN;
			break;
		case 1:
			reg_mode = REG_MIPI_HOST_IPI2_MODE;
			rstn_bit = MIPI_HOST_IPI2_SOFTRSTN;
			break;
		case 0:
		default:
			reg_mode = REG_MIPI_HOST_IPI_MODE;
			rstn_bit = MIPI_HOST_IPI1_SOFTRSTN;
			break;
		}
		mipiinfo("%d:ipi%d reset: %s", i, i + 1, ipi_reset->enable ? "enable" : "disable");
		spin_lock_irqsave(&host->reglock, flags);
		softrstn = mipi_getreg(iomem + REG_MIPI_HOST_IPI_SOFTRSTN);
		ipi_mode = mipi_getreg(iomem + reg_mode);
		if (ipi_reset->enable) {
			softrstn |= rstn_bit;
			ipi_mode |= MIPI_HOST_IPI_ENABLE;
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_SOFTRSTN, softrstn);
			mipi_putreg(iomem + reg_mode, ipi_mode);
		} else {
			softrstn &= ~rstn_bit;
			ipi_mode &= ~MIPI_HOST_IPI_ENABLE;
			mipi_putreg(iomem + reg_mode, ipi_mode);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_SOFTRSTN, softrstn);
		}
		spin_unlock_irqrestore(&host->reglock, flags);
		done++;
	}
	if (done == 0) {
		mipierr("ipi reset error: none");
		return -1;
	}

	return 0;
}

/**
 * @brief mipi_host_reset_ipi: reset ipi of mipi host
 *
 * @param [in] port: host port index.
 * @param [in] ipi: ipi index.
 * @param [in] enable: ipi enable.
 *
 * @return int32_t : 0/-1
 */
int32_t mipi_host_reset_ipi(uint32_t port, int32_t ipi, int32_t enable)
{
	mipi_hdev_t *hdev;
	mipi_host_ipi_reset_t ipi_reset;
	int32_t ipi_max;

	if (port >= MIPI_HOST_MAX_NUM)
		return -ENODEV;
	hdev = g_hdev[port];
	ipi_max = mipi_host_port_ipi(hdev);
	if (ipi >= ipi_max)
		return-EINVAL;

	if (ipi < 0)
		ipi_reset.mask = MIPI_IPI_MASK_ALL;
	else
		ipi_reset.mask = (uint16_t)(0x1 << (uint8_t)ipi);
	ipi_reset.enable = !!enable;
	return mipi_host_ipi_reset(hdev, &ipi_reset);
}
EXPORT_SYMBOL_GPL(mipi_host_reset_ipi);

/**
 * @brief mipi_host_ipi_get_info: get ipi info of mipi host
 *
 * @param [in/out] ipi_info: ipi info struct.
 *
 * @return int32_t : 0/-1
 */
static int32_t mipi_host_ipi_get_info(mipi_hdev_t *hdev, mipi_host_ipi_info_t *ipi_info)
{
	mipi_host_t *host = &hdev->host;
	mipi_host_cfg_t *cfg = &host->cfg;
	mipi_host_param_t *param = &host->param;
	struct device *dev = hdev->dev;
	void __iomem *iomem = host->iomem;
	int32_t index, ipi_max;
	int32_t r_fatal, r_mode, r_vc, r_datatype, r_hsa, r_hbp, r_hsd, r_adv;

	if (!ipi_info || !iomem)
		return -1;

	index = ipi_info->index;
	ipi_max = mipi_host_port_ipi(hdev);
	if (index >= ipi_max) {
		mipierr("%d:ipi%d get: not suppor error", index, index + 1);
		return -1;
	} else if (index >= cfg->channel_num) {
		mipidbg("%d:ipi%d get: not inited warning", index, index + 1);
	}
	switch (index) {
	case 3:
		r_fatal = REG_MIPI_HOST_INT_ST_IPI4;
		r_mode = REG_MIPI_HOST_IPI4_MODE;
		r_vc = REG_MIPI_HOST_IPI4_VCID;
		r_datatype = REG_MIPI_HOST_IPI4_DATA_TYPE;
		r_hsa = REG_MIPI_HOST_IPI4_HSA_TIME;
		r_hbp = REG_MIPI_HOST_IPI4_HBP_TIME;
		r_hsd = REG_MIPI_HOST_IPI4_HSD_TIME;
		r_adv = REG_MIPI_HOST_IPI4_ADV_FEATURES;
		break;
	case 2:
		r_fatal = REG_MIPI_HOST_INT_ST_IPI3;
		r_mode = REG_MIPI_HOST_IPI3_MODE;
		r_vc = REG_MIPI_HOST_IPI3_VCID;
		r_datatype = REG_MIPI_HOST_IPI3_DATA_TYPE;
		r_hsa = REG_MIPI_HOST_IPI3_HSA_TIME;
		r_hbp = REG_MIPI_HOST_IPI3_HBP_TIME;
		r_hsd = REG_MIPI_HOST_IPI3_HSD_TIME;
		r_adv = REG_MIPI_HOST_IPI3_ADV_FEATURES;
		break;
	case 1:
		r_fatal = REG_MIPI_HOST_INT_ST_IPI2;
		r_mode = REG_MIPI_HOST_IPI2_MODE;
		r_vc = REG_MIPI_HOST_IPI2_VCID;
		r_datatype = REG_MIPI_HOST_IPI2_DATA_TYPE;
		r_hsa = REG_MIPI_HOST_IPI2_HSA_TIME;
		r_hbp = REG_MIPI_HOST_IPI2_HBP_TIME;
		r_hsd = REG_MIPI_HOST_IPI2_HSD_TIME;
		r_adv = REG_MIPI_HOST_IPI2_ADV_FEATURES;
		break;
	case 0:
	default:
		r_fatal = REG_MIPI_HOST_INT_ST_IPI;
		r_mode = REG_MIPI_HOST_IPI_MODE;
		r_vc = REG_MIPI_HOST_IPI_VCID;
		r_datatype = REG_MIPI_HOST_IPI_DATA_TYPE;
		r_hsa = REG_MIPI_HOST_IPI_HSA_TIME;
		r_hbp = REG_MIPI_HOST_IPI_HBP_TIME;
		r_hsd = REG_MIPI_HOST_IPI_HSD_TIME;
		r_adv = REG_MIPI_HOST_IPI_ADV_FEATURES;
		break;
	}
	ipi_info->fatal = (uint16_t)mipi_getreg(iomem + r_fatal);
	ipi_info->mode = (uint16_t)mipi_getreg(iomem + r_mode);
	ipi_info->vc = (uint8_t)mipi_getreg(iomem + r_vc);
	ipi_info->datatype = (uint16_t)mipi_getreg(iomem + r_datatype);
	ipi_info->hsa = (uint16_t)mipi_getreg(iomem + r_hsa);
	ipi_info->hbp = (uint16_t)mipi_getreg(iomem + r_hbp);
	ipi_info->hsd = (uint16_t)mipi_getreg(iomem + r_hsd);
	ipi_info->adv = (uint32_t)mipi_getreg(iomem + r_adv);

	return 0;
}

/**
 * @brief mipi_host_ipi_set_info: set ipi info of mipi host
 *
 * @param [in] ipi_info: ipi info struct.
 *
 * @return int32_t : 0/-1
 */
static int32_t mipi_host_ipi_set_info(mipi_hdev_t *hdev, mipi_host_ipi_info_t *ipi_info)
{
	mipi_host_t *host = &hdev->host;
	mipi_host_cfg_t *cfg = &host->cfg;
	mipi_host_param_t *param = &host->param;
	struct device *dev = hdev->dev;
	void __iomem *iomem = host->iomem;
	int32_t index, ipi_max, set_mask;
	int32_t r_mode, r_vc, r_datatype, r_hsa, r_hbp, r_hsd, r_adv;

	if (!ipi_info || !iomem)
		return -1;

	index = ipi_info->index;
	ipi_max = mipi_host_port_ipi(hdev);
	if (index >= ipi_max) {
		mipierr("%d:ipi%d set: not suppor error", index, index + 1);
		return -1;
	} else if (index >= cfg->channel_num) {
		mipidbg("%d:ipi%d set: not inited warning", index, index + 1);
	}
	switch (index) {
	case 3:
		r_mode = REG_MIPI_HOST_IPI4_MODE;
		r_vc = REG_MIPI_HOST_IPI4_VCID;
		r_datatype = REG_MIPI_HOST_IPI4_DATA_TYPE;
		r_hsa = REG_MIPI_HOST_IPI4_HSA_TIME;
		r_hbp = REG_MIPI_HOST_IPI4_HBP_TIME;
		r_hsd = REG_MIPI_HOST_IPI4_HSD_TIME;
		r_adv = REG_MIPI_HOST_IPI4_ADV_FEATURES;
		break;
	case 2:
		r_mode = REG_MIPI_HOST_IPI3_MODE;
		r_vc = REG_MIPI_HOST_IPI3_VCID;
		r_datatype = REG_MIPI_HOST_IPI3_DATA_TYPE;
		r_hsa = REG_MIPI_HOST_IPI3_HSA_TIME;
		r_hbp = REG_MIPI_HOST_IPI3_HBP_TIME;
		r_hsd = REG_MIPI_HOST_IPI3_HSD_TIME;
		r_adv = REG_MIPI_HOST_IPI3_ADV_FEATURES;
		break;
	case 1:
		r_mode = REG_MIPI_HOST_IPI2_MODE;
		r_vc = REG_MIPI_HOST_IPI2_VCID;
		r_datatype = REG_MIPI_HOST_IPI2_DATA_TYPE;
		r_hsa = REG_MIPI_HOST_IPI2_HSA_TIME;
		r_hbp = REG_MIPI_HOST_IPI2_HBP_TIME;
		r_hsd = REG_MIPI_HOST_IPI2_HSD_TIME;
		r_adv = REG_MIPI_HOST_IPI2_ADV_FEATURES;
		break;
	case 0:
	default:
		r_mode = REG_MIPI_HOST_IPI_MODE;
		r_vc = REG_MIPI_HOST_IPI_VCID;
		r_datatype = REG_MIPI_HOST_IPI_DATA_TYPE;
		r_hsa = REG_MIPI_HOST_IPI_HSA_TIME;
		r_hbp = REG_MIPI_HOST_IPI_HBP_TIME;
		r_hsd = REG_MIPI_HOST_IPI_HSD_TIME;
		r_adv = REG_MIPI_HOST_IPI_ADV_FEATURES;
		break;
	}
	/* set some masked if fatal.b15, or set all
	 * ps: disable first, enable last */
	set_mask = (ipi_info->fatal & 0x8000) ? ipi_info->fatal : 0xffff;
	if ((set_mask & (0x1 << 0)) &&
		((ipi_info->mode & MIPI_HOST_IPI_ENABLE) == 0))
		mipi_putreg(iomem + r_mode, ipi_info->mode);
	if (set_mask & (0x1 << 1))
		mipi_putreg(iomem + r_vc, ipi_info->vc);
	if (set_mask & (0x1 << 2))
		mipi_putreg(iomem + r_datatype, ipi_info->datatype);
	if (set_mask & (0x1 << 3))
		mipi_putreg(iomem + r_hsa, ipi_info->hsa);
	if (set_mask & (0x1 << 4))
		mipi_putreg(iomem + r_hbp, ipi_info->hbp);
	if (set_mask & (0x1 << 5))
		mipi_putreg(iomem + r_hsd, ipi_info->hsd);
	if (set_mask & (0x1 << 6))
		mipi_putreg(iomem + r_adv, ipi_info->adv);
	if ((set_mask & (0x1 << 0)) &&
		((ipi_info->mode & MIPI_HOST_IPI_ENABLE) != 0))
		mipi_putreg(iomem + r_mode, ipi_info->mode);

	mipiinfo("%d:ipi%d set: mode=0x%x,vc=0x%x,dt=0x%x,hsa=%d,hbp=%d,hsd=%d,adv=0x%x",
		index, index + 1,
		mipi_getreg(iomem + r_mode),
		mipi_getreg(iomem + r_vc),
		mipi_getreg(iomem + r_datatype),
		mipi_getreg(iomem + r_hsa),
		mipi_getreg(iomem + r_hbp),
		mipi_getreg(iomem + r_hsd),
		mipi_getreg(iomem + r_adv));
	return 0;
}

#ifdef CONFIG_HOBOT_XJ3
int vio_clk_enable(const char *name);
int vio_clk_disable(const char *name);
int vio_set_clk_rate(const char *name, ulong frequency);
ulong vio_get_clk_rate(const char *name);
#endif
/**
 * @brief mipi_host_configure_clk: configure clk of mipi host
 *
 * @param [in] name/freq/checkequ: mipi host clk's setting
 *
 * @return int32_t : 0/-1
 */
static int32_t mipi_host_configure_clk(mipi_hdev_t *hdev, const char *name,
				ulong freq, int checkequ)
{
	int32_t ret = 0;
	struct device *dev = hdev->dev;
#ifdef CONFIG_HOBOT_XJ3
	ulong clk;

	if (freq == 0)
		return vio_clk_disable(name);

	clk = vio_get_clk_rate(name);
	if (clk != freq) {
		vio_set_clk_rate(name, freq);
		if (checkequ) {
			clk = vio_get_clk_rate(name);
			if (clk != freq) {
				mipierr("%s = %lu != %lu", name, clk, freq);
				return (-1);
			} else {
				mipiinfo("%s = %lu", name, clk);
			}
		}
	}
	ret = vio_clk_enable(name);
#else
	mipiinfo("should: %s %s %lu", __func__, name, freq);
#endif
	return ret;
}

/**
 * @brief mipi_host_get_clk: configure clk of mipi host
 *
 * @param [in] name: mipi host clk's getting
 *
 * @return int32_t : 0/-1
 */
static ulong mipi_host_get_clk(mipi_hdev_t *hdev, const char *name)
{
	ulong clk = 0;
#ifdef CONFIG_HOBOT_XJ3
	clk = vio_get_clk_rate(name);
#else
	struct device *dev = hdev->dev;

	mipiinfo("should: %s %s %lu", __func__, name, clk);
#endif
	return clk;
}

/**
 * @brief mipi_host_snrclk_set_freq: configure snrclk of mipi host
 *
 * @param [in] freq: mipi host snrclk's setting
 *
 * @return int32_t : 0/-1
 */
static int32_t mipi_host_snrclk_set_freq(mipi_hdev_t *hdev, uint32_t freq)
{
	int32_t ret = 0;
	struct device *dev = hdev->dev;
	mipi_host_snrclk_t *snrclk = &hdev->host.snrclk;
	mipi_host_param_t *param = &hdev->host.param;
	const char *name;
	uint32_t diff;

	if (snrclk->index < 0) {
		mipierr("snrclk set freq not support");
		return -1;
	}
	name = g_mh_snrclk_name[snrclk->index];
	if (freq > 0 || snrclk->en_cnt > 0) {
		/* enable: first time with default freq */
	#if 0
		if (freq > 0 && snrclk->en_cnt == 0) {
			ret = mipi_host_configure_clk(hdev, name, 37125000UL, 0);
			if (ret == 0) {
				snrclk->en_cnt++;
			}
		}
	#endif
		/* enable: then change real freq / disable: freq = 0 */
		ret = mipi_host_configure_clk(hdev, name, freq, 0);
		if (ret == 0) {
			if (freq)
				snrclk->en_cnt++;
			else
				snrclk->en_cnt--;
		}
		param->snrclk_freq = (uint32_t)(mipi_host_get_clk(hdev, name));
		if (freq != 0) {
			diff = (freq > param->snrclk_freq) ? (freq - param->snrclk_freq) :
				param->snrclk_freq - freq;
			if ((diff * 10) > freq) {
				if (ret == 0) {
					(void)mipi_host_configure_clk(hdev, name, 0, 0);
					snrclk->en_cnt--;
				}
				ret = -1;
			}
		}
		if (ret)
			mipierr("%s set(%d) %u but %u error", name, snrclk->en_cnt,
				freq, param->snrclk_freq);
		else
			mipiinfo("%s set(%d) %u as %u", name, snrclk->en_cnt,
				freq, param->snrclk_freq);
	} else {
		mipiinfo("%s set(%d) %u done", name, snrclk->en_cnt, freq);
	}

	return ret;
}

/**
 * @brief mipi_host_snrclk_close_freq: close snrclk of mipi host
 *
 * @param [in] keep_sys: close all or keep sysfs cnt?
 *
 * @return void : none
 */
static void mipi_host_snrclk_close_freq(mipi_hdev_t *hdev, int32_t keep_sys)
{
	mipi_host_snrclk_t *snrclk = &hdev->host.snrclk;
	int32_t ret = 0;

	if (snrclk->index < 0) {
		return;
	}

	while (ret == 0 && snrclk->en_cnt > snrclk->sys_cnt) {
		ret = mipi_host_snrclk_set_freq(hdev, 0);
	}

	if (keep_sys == 0) {
		while (ret == 0 && snrclk->sys_cnt > 0) {
			ret = mipi_host_snrclk_set_freq(hdev, 0);
			if (ret == 0)
				snrclk->sys_cnt--;
		}
	}
}

/**
 * @brief mipi_host_snrclk_set_en: enable/disable snrclk of mipi host
 *
 * @param [in] enable: mipi host snrclk's setting
 *
 * @return int32_t : 0/-1
 */

static int32_t mipi_host_snrclk_set_en(mipi_hdev_t *hdev, int enable)
{
	int32_t ret = 0;
	struct device *dev = hdev->dev;
	mipi_host_snrclk_t *snrclk = &hdev->host.snrclk;
	mipi_host_param_t *param = &hdev->host.param;

	if (!snrclk->pinctrl) {
		mipierr("snrclk set en not support");
		return -1;
	}

	if (enable) {
		if (snrclk->enable) {
			mipiinfo("snrclk set enable");
			ret = pinctrl_select_state(snrclk->pinctrl,
					snrclk->enable);
			if (ret == 0)
				param->snrclk_en = MIPI_HOST_SNRCLK_ENABLE;
		} else {
			mipierr("snrclk set enable not support");
			ret = -1;
		}

		/* enable clk if not enabled */
		if (ret == 0 && snrclk->en_cnt == 0) {
			mipi_host_snrclk_set_freq(hdev,
				(uint32_t)mipi_host_get_clk(hdev, g_mh_snrclk_name[snrclk->index]));
		}
	} else {
		if (snrclk->disable) {
			mipiinfo("snrclk set disable");
			ret = pinctrl_select_state(snrclk->pinctrl,
					snrclk->disable);
			if (ret == 0)
				param->snrclk_en = MIPI_HOST_SNRCLK_DISABLE;
		} else {
			mipierr("snrclk set disable not support");
			ret = -1;
		}

		/* disable clk if enabled once */
		if (ret == 0 && snrclk->en_cnt == 1) {
			mipi_host_snrclk_set_freq(hdev, 0);
		}
	}

	return ret;
}

/* mipi host functions */
static unsigned long mipi_host_pixel_clk_select(mipi_hdev_t *hdev, mipi_host_cfg_t *cfg)
{
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	mipi_host_param_t *param = &host->param;
	unsigned long pixclk = (unsigned long)(cfg->linelenth * cfg->framelenth * cfg->fps);
	unsigned long pixclk_act = pixclk;
	unsigned long linelenth = cfg->linelenth;
	unsigned long framelenth = cfg->framelenth;

	if (!cfg->fps) {
		mipiinfo("input FPS can't be zero!!!");
		return 0;
	}
	if (!cfg->linelenth)
		linelenth = cfg->width;
	if (!cfg->framelenth)
		framelenth = cfg->height;

	if (param->ipi_force >= 10000000) {
		pixclk = param->ipi_force;
		mipiinfo("ipiclk force as %lu", pixclk);
	} else {
		pixclk = linelenth * framelenth * cfg->fps;
		if (cfg->datatype < MIPI_CSI2_DT_RAW_8)
			pixclk = pixclk;
		else
			pixclk = (pixclk + 2) / 3;

		if (param->ipi_limit && pixclk < param->ipi_limit) {
			mipiinfo("ipiclk limit %lu up to %u", pixclk, param->ipi_limit);
			pixclk = param->ipi_limit;
		}
	}
#ifdef CONFIG_HOBOT_IPS_X2
	if (ips_set_mipi_ipi_clk(pixclk) < 0) {
		mipiinfo("ips_set_mipi_ipi_clk %lu error", pixclk);
		pixclk_act = 0;
	} else {
		mipiinfo("host fifo clk pixclk: %lu", pixclk);
		pixclk_act = ips_get_mipi_ipi_clk();
		mipiinfo("host fifo clk pixclk: %lu", pixclk_act);
	}
#else
	if (hdev->port >= ARRAY_SIZE(g_mh_ipiclk_name) ||
		mipi_host_configure_clk(hdev, g_mh_ipiclk_name[hdev->port], pixclk, 0) < 0) {
		pixclk_act = 0;
		mipierr("mipi_host_configure_clk %lu error", pixclk);
	} else {
		pixclk_act = mipi_host_get_clk(hdev, g_mh_ipiclk_name[hdev->port]);
		if (pixclk_act == 0)
			pixclk_act = pixclk;
		mipiinfo("ipiclk set %lu get %lu", pixclk, pixclk_act);
	}
#endif
	return pixclk_act;
}

/**
 * @brief mipi_host_get_hsd :
 *
 * @param [] cfg :
 *
 * @return uint16_t
 */
static uint16_t mipi_host_get_hsd(mipi_hdev_t *hdev, mipi_host_cfg_t *cfg,
		unsigned long pixclk)
{
	/**
	 * Rxbyteclk = (Rxbitclk / Number_of_lanes) / 8
	 * Rxbyteclk_per = 1 / Rxbyteclk
	 * Bytes_to_transmi = BitsPerPixel * Line_size / 8
	 * time to transmit last pixel in PPI
	 * = (Bytes_to_transmit / Number_of_lanes) *  Rxbyteclk_per
	 * = ((BitsPerPixel * Line_size / 8) / Number_of_lanes) /((Rxbitclk /  Number_of_lanes) / 8)
	 * = (BitsPerPixel * Line_size) / Rxbitclk
	 *
	 * pixel_clk_per = 1 / pixel_clk
	 * time to transmit last pixel in IPI
	 * = (hsa + hbp + hsd + cycles_to_transmit_data) * pixel_clk_per
	 * = (hsa + hbp + hsd + cycles_to_transmit_data) / pixel_clk
	 *
	 * T(ppi) < T(ipi) ==>
	 * BitsPerPixel * Line_size * pixel_clk / Rxbitclk < (hsa + hbp + hsd + cycles_to_trans) ==>
	 *  hsd > BitsPerPixel * Line_size * pixel_clk / Rxbitclk - (cycles_to_trans+hsa+ hbp)
	 *  cycles_to_trans = Line_size in 48 bits IPI
	 *
	 */
	struct device *dev = hdev->dev;
	mipi_host_param_t *param = &hdev->host.param;
	unsigned long rx_bit_clk = 0;
	unsigned long bits_per_pixel = 0;
	unsigned long line_size = 0;
	unsigned long cycles_to_trans = 0;
	unsigned long time_ppi = 0;
	unsigned long time_ipi = 0;
	int32_t  hsdtime = 0;
	int yuv_cycle = (param->ipi_16bit) ? 2 : 1;
	int raw_pixel = (param->ipi_16bit) ? 1 : 3;

	switch (cfg->datatype) {
	case MIPI_CSI2_DT_YUV420_8:
		bits_per_pixel = 8 * 3 / 2;
		cycles_to_trans = (unsigned long)(cfg->width * yuv_cycle);
		break;
	case MIPI_CSI2_DT_YUV420_10:
		bits_per_pixel = 16 * 3 / 2;
		cycles_to_trans = (unsigned long)(cfg->width * yuv_cycle);
		break;
	case MIPI_CSI2_DT_YUV422_8:
		bits_per_pixel = 8 * 2;
		cycles_to_trans = (unsigned long)(cfg->width * yuv_cycle);
		break;
	case MIPI_CSI2_DT_YUV422_10:
		bits_per_pixel = 16 * 2;
		cycles_to_trans = (unsigned long)(cfg->width * yuv_cycle);
		break;
	case MIPI_CSI2_DT_RAW_8:
		bits_per_pixel = 8;
		cycles_to_trans = (unsigned long)((cfg->width + 2) / raw_pixel);
		break;
	case MIPI_CSI2_DT_RAW_10:
		bits_per_pixel = 10;
		cycles_to_trans = (unsigned long)((cfg->width + 2) / raw_pixel);
		break;
	case MIPI_CSI2_DT_RAW_12:
		bits_per_pixel = 12;
		cycles_to_trans = (unsigned long)((cfg->width + 2) / raw_pixel);
		break;
	case MIPI_CSI2_DT_RAW_14:
		bits_per_pixel = 14;
		cycles_to_trans = (unsigned long)((cfg->width + 2) / raw_pixel);
		break;
	default:
		bits_per_pixel = 16;
		break;
	}
	if (!cfg->linelenth) {
		rx_bit_clk = (unsigned long)cfg->mipiclk * 1000000;
		line_size = cfg->width;
	} else {
		rx_bit_clk = ((unsigned long)(cfg->linelenth * cfg->framelenth * cfg->fps)) * bits_per_pixel;
		line_size = cfg->linelenth;
	}
	mipiinfo("linelenth: %d, framelenth: %d, fps: %d, bits_per_pixel: %lu, pixclk: %lu, rx_bit_clk: %lu",
			 cfg->linelenth, cfg->framelenth, cfg->fps, bits_per_pixel, pixclk, rx_bit_clk);
	time_ppi = (1000 * bits_per_pixel * line_size * 1000000 / rx_bit_clk);
	mipiinfo("time to transmit last pixel in ppi: %lu", time_ppi);
	hsdtime = (uint32_t)((bits_per_pixel * line_size * pixclk / rx_bit_clk) - ((uint64_t)cfg->hsaTime + (uint64_t)cfg->hbpTime + cycles_to_trans));
	mipiinfo("minium hsdtime: %d", hsdtime);
#ifndef ADJUST_CLK_RECALCULATION
	if (hsdtime < 0) {
		hsdtime = 1;
	}
#else
	if (hsdtime < 0)
		hsdtime = 4;
	else
		hsdtime += 4;
#endif
	if (hsdtime > MIPI_HOST_HSDTIME_MAX) {
		hsdtime = MIPI_HOST_HSDTIME_MAX;
		mipiinfo("hsdtime max as: %d", hsdtime);
	}
	time_ipi = 1000 * (unsigned long)(cfg->hsaTime + cfg->hbpTime + hsdtime + cycles_to_trans) * 1000000 / pixclk;
	mipiinfo("time to transmit last pixel in ipi: %lu", time_ipi);
	return (uint16_t)hsdtime;
}

#if MIPI_HOST_INT_DBG
/**
 * @brief mipi_host_irq_enable : Enale mipi host IRQ
 *
 * @param [in] irq : IRQ Flag
 *
 * @return void
 */
static void mipi_host_irq_enable(mipi_hdev_t *hdev)
{
	mipi_host_t *host = &hdev->host;
	mipi_host_ierr_t *ierr = &host->ierr;
	const mipi_host_ireg_t *ireg = NULL;
	void __iomem *iomem = host->iomem;
	uint32_t temp = 0;
	int i;

	if (!hdev || !iomem)
		return;

	temp = mipi_getreg(iomem + REG_MIPI_HOST_INT_ST_MAIN);
	for (i = 0; i < ierr->num; i++) {
		ireg = &ierr->iregs[i];
		temp = mipi_getreg(iomem + ireg->reg_st);
		temp = mipi_getreg(iomem + ireg->reg_mask);
		temp &= ~(ireg->err_mask);
		temp |= ireg->err_mask;
		mipi_putreg(iomem + ireg->reg_mask, temp);
	}

#ifdef MIPI_HOST_INT_USE_TIMER
	hdev->irq_timer_en = 1;
#endif

	return;
}

/**
 * @brief mipi_host_irq_disable : Disable mipi host IRQ
 *
 * @param [in] irq : IRQ Flag
 *
 * @return void
 */
static void mipi_host_irq_disable(mipi_hdev_t *hdev)
{
	mipi_host_t *host = &hdev->host;
	mipi_host_ierr_t *ierr = &host->ierr;
	const mipi_host_ireg_t *ireg = NULL;
	void __iomem *iomem = host->iomem;
	uint32_t temp = 0;
	int i;

	if (!hdev || !iomem)
		return;

	for (i = 0; i < ierr->num; i ++) {
		ireg = &ierr->iregs[i];
		temp = mipi_getreg(iomem + ireg->reg_mask);
		temp &= ~(ireg->err_mask);
		mipi_putreg(iomem + ireg->reg_mask, temp);
	}

#if defined MIPI_HOST_INT_USE_TIMER
	hdev->irq_timer_en = 0;
#endif

	return;
}

#ifdef CONFIG_HOBOT_DIAG
static void mipi_host_diag_report(mipi_hdev_t *hdev,
		uint8_t errsta, uint8_t errchn, uint8_t errtype, uint32_t total_irq)
{
	uint8_t env_data[8];

	hdev->last_err_tm_ms = jiffies_to_msecs(get_jiffies_64());
	if (errsta) {
		env_data[0] = errchn;
		env_data[1] = errtype;
		env_data[2] = 0;
		env_data[3] = 4;
		memcpy(&env_data[4], &total_irq, 4);
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_VIO,
				(uint16_t)(EventIdVioMipiHost0Err + hdev->port),
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				env_data,
				sizeof(env_data));
	}
}

static void mipi_host_diag_timer_func(unsigned long data)
{
	mipi_hdev_t *hdev = (mipi_hdev_t *)data;
	uint32_t now_tm_ms;
	unsigned long jiffi;

	now_tm_ms = jiffies_to_msecs(get_jiffies_64());
	if (now_tm_ms - hdev->last_err_tm_ms > 6500) {
		diag_send_event_stat(
				DiagMsgPrioMid,
				ModuleDiag_VIO,
				(uint16_t)(EventIdVioMipiHost0Err + hdev->port),
				DiagEventStaSuccess);
	}
	jiffi = get_jiffies_64() + msecs_to_jiffies(2000);
	mod_timer(&hdev->diag_timer, jiffi); // trriger again.
}

static void mipi_host_diag_test(void *p, size_t len)
{
	mipi_hdev_t *hdev;
	void __iomem *iomem;
	int i;

	for (i = 0; i < MIPI_HOST_MAX_NUM; i ++) {
		hdev = g_hdev[i];
		if (!hdev)
			continue;
		iomem = hdev->host.iomem;
		if (!iomem)
			continue;
		mipi_putreg(iomem + REG_MIPI_HOST_INT_FORCE_PHY, 0x1);
	}
}
#endif

static int mipi_host_send_sig_info(int32_t pid, int32_t signo, int errno, uint64_t addr)
{
	int ret = -1;
	struct siginfo info;
	struct task_struct *p_task;

	p_task = pid_task(find_vpid(pid), PIDTYPE_PID);
	if (p_task) {
		memset(&info, 0, sizeof(struct siginfo));
		info.si_signo = signo;
		info.si_errno = errno;
		info.si_code = SI_KERNEL;
		info.si_addr = (void __user *) addr;
		ret = send_sig_info(signo, &info, p_task);
	}
	return ret;
}


static int mipi_host_send_sig_fatal(mipi_hdev_t *hdev, uint32_t fatal, uint32_t sub)
{
	int32_t ret = -1;
	unsigned long flags;
	mipi_host_sigpid_t *spid = &hdev->sigpid;
	mipi_host_param_t *param = &hdev->host.param;
	struct device *dev = hdev->dev;
	uint32_t fatal_mask;
	int signo;
	unsigned long cur_ms = jiffies_to_msecs(get_jiffies_64());

	spin_lock_irqsave(&hdev->siglock, flags);
	fatal_mask = (param->sigfatal_mask) ? param->sigfatal_mask : spid->fatal_mask;
	if ((spid->pid > 0) && ((fatal_mask == 0) ||
		((fatal_mask & fatal) != 0))) {
		/* sigts_ms: sigusr1 last send timestamp.
		 * 0   -- ready to send sigusr1;
		 * !0  -- send sigusr1 again if > sigwait_ms, else send sigusr2;
		 * =0  -- clean after send sigusr2 --> ready to send sigusr1 again.
		 *  */
		if (hdev->sigts_ms == 0 || (cur_ms - hdev->sigts_ms) > param->sigwait_ms)
			signo = SIGUSR1;
		else
			signo = SIGUSR2;
		ret = mipi_host_send_sig_info(spid->pid, signo, hdev->port, (((uint64_t)sub << 32) | fatal));
		if (ret < 0) {
			mipidbg("mipihost%d sig %d 0x%x:0x%x to pid %d error %d",
				hdev->port, signo, fatal, sub, spid->pid, ret);
		} else {
			mipidbg("mipihost%d sig %d 0x%x:0x%x to pid %d",
				hdev->port, signo, fatal, sub, spid->pid);
			hdev->sigts_ms = (signo == SIGUSR2) ? 0 : cur_ms;
			ret = 0;
		}
	}
	spin_unlock_irqrestore(&hdev->siglock, flags);
	return ret;
}

static int mipi_host_int_fatal_pr(mipi_hdev_t *hdev)
{
	struct device *dev;
	mipi_user_t *user;
	mipi_host_t *host;
	mipi_host_ierr_t *ierr;
	const mipi_host_ireg_t *ireg;
	mipi_host_param_t *param;
	mipi_host_icnt_t *icnt;
	void __iomem *iomem;
	uint32_t *icnt_p;
	uint32_t reg, mask, icnt_n;
	uint32_t irq = 0, irq_do;
	uint32_t subirq = 0;
	int i;
	int irq_occurred = 0;
	char *perr = "";
#if MIPI_HOST_INT_DBG_ERRSTR
	int j, l;
	uint32_t subirq_do;
	char err_str[256];
#endif



	if (!hdev)
		return -EINVAL;

	user = &hdev->user;
	dev = hdev->dev;
	host = &hdev->host;
	ierr = &host->ierr;
	param = &host->param;
	icnt = &host->icnt;
	iomem = host->iomem;
	icnt_p = (uint32_t *)icnt;
	if (!iomem)
		return -EFAULT;
	if (user->init_cnt == 0)
		return -EACCES;

#ifdef MIPI_HOST_INT_USE_TIMER
	irq = hdev->irq_st_main;
#else
	irq = mipi_getreg(iomem + REG_MIPI_HOST_INT_ST_MAIN);
#endif
	if(irq) {
		if (param->irq_debug & MIPI_HOST_IRQ_DEBUG_PRERR)
			mipierr("irq status 0x%x (%d)", irq, icnt->st_main);
		else
			mipidbg("irq status 0x%x (%d)", irq, icnt->st_main);

		irq_do = irq;
		icnt->st_main++;
		for (i = 0; i < ierr->num; i++) {
			ireg = &ierr->iregs[i];
			mask = ireg->st_mask;
			if (!(irq_do & mask))
				continue;

			reg = ireg->reg_st;
			icnt_n = ireg->icnt_n;
			subirq = mipi_getreg(iomem + reg) & ireg->err_mask;
			if (!subirq) {
				irq_do &= ~mask;
				continue;
			}

#if MIPI_HOST_INT_DBG_ERRSTR
			err_str[0] = '\0';
			perr = err_str;
			if (param->irq_debug & MIPI_HOST_IRQ_DEBUG_ERRSTR) {
				subirq_do = subirq;
				j = 0;
				l = 0;
				while(subirq_do && j < 32 && l < sizeof(err_str)) {
					if (subirq_do & (0x1 << j)) {
						l += snprintf(&err_str[l], sizeof(err_str) - l,
								" %d:%s", j, (ireg->err_str[j]) ? ireg->err_str[j] : "rsv");
						subirq_do &= ~(0x1 << j);
					}
					j++;
				}
			}
#endif
			if (param->irq_debug & MIPI_HOST_IRQ_DEBUG_PRERR)
				mipierr("  %s: 0x%x%s",
					g_mh_icnt_names[icnt_n], subirq, perr);
			else
				mipidbg("  %s: 0x%x%s",
					g_mh_icnt_names[icnt_n], subirq, perr);
			icnt_p[icnt_n]++;
			irq_occurred |= mask;
			irq_do &= ~mask;
			if (!irq_do)
				break;
		}
	} else {
		mipidbg("irq status 0x%x (%d)", irq, icnt->st_main);
	}

	return irq_occurred;
}

int mipi_host_int_fatal_show(int port)
{
	if (port < 0 || port >= MIPI_HOST_MAX_NUM)
		return -ENODEV;

	return mipi_host_int_fatal_pr(g_hdev[port]);
}
EXPORT_SYMBOL(mipi_host_int_fatal_show);

/**
 * @brief mipi_host_irq_func : irq func
 *
 * @param [in] this_irq : irq num
 * @param [in] data : user data
 *
 * @return irqreturn_t
 */
static irqreturn_t mipi_host_irq_func(int this_irq, void *data)
{
	mipi_hdev_t *hdev = (mipi_hdev_t *)data;
	if (!hdev)
		return IRQ_NONE;
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	mipi_host_ierr_t *ierr = &host->ierr;
	const mipi_host_ireg_t *ireg = NULL;
	mipi_host_param_t *param = &host->param;
	mipi_host_icnt_t *icnt = &host->icnt;
	void __iomem *iomem = host->iomem;
	uint32_t *icnt_p = (uint32_t *)icnt;
	uint32_t reg, mask, icnt_n;
	uint32_t irq = 0, irq_do;
	uint32_t subirq = 0;
	int i;
	uint8_t err_occurred = 0;
	char *perr = "";
#if MIPI_HOST_INT_DBG_ERRSTR
	int j, l;
	uint32_t subirq_do;
	char err_str[256];
#endif
#ifdef CONFIG_HOBOT_DIAG
	uint8_t errchn, errtype;
#endif

	if (!iomem)
		return IRQ_NONE;

	if (this_irq >= 0)
		disable_irq_nosync(this_irq);

#ifdef MIPI_HOST_INT_USE_TIMER
	irq = hdev->irq_st_main;
#else
	irq = mipi_getreg(iomem + REG_MIPI_HOST_INT_ST_MAIN);
#endif
	if (param->irq_debug & MIPI_HOST_IRQ_DEBUG_PRERR)
		mipierr("irq status 0x%x", irq);
	else
		mipidbg("irq status 0x%x", irq);
	if(irq) {
		irq_do = irq;
		icnt->st_main++;
		for (i = 0; i < ierr->num; i++) {
			ireg = &ierr->iregs[i];
			mask = ireg->st_mask;
			if (!(irq_do & mask))
				continue;

			reg = ireg->reg_st;
			icnt_n = ireg->icnt_n;
			subirq = mipi_getreg(iomem + reg) & ireg->err_mask;
			if (!subirq) {
				irq_do &= ~mask;
				continue;
			}

#if MIPI_HOST_INT_DBG_ERRSTR
			err_str[0] = '\0';
			perr = err_str;
			if (param->irq_debug & MIPI_HOST_IRQ_DEBUG_ERRSTR) {
				subirq_do = subirq;
				j = 0;
				l = 0;
				while(subirq_do && j < 32 && l < sizeof(err_str)) {
					if (subirq_do & (0x1 << j)) {
						l += snprintf(&err_str[l], sizeof(err_str) - l,
								" %d:%s", j, (ireg->err_str[j]) ? ireg->err_str[j] : "rsv");
						subirq_do &= ~(0x1 << j);
					}
					j++;
				}
			}
#endif
#ifdef CONFIG_HOBOT_DIAG
			errchn = 0xff;
			errtype = 0xff;
			if (icnt_n >= 4 && icnt_n <= 9) {
				/* bndry_frm seq_frm crc_frm pld_crc data_id ecc_corrected */
				for (errchn = 0; errchn < 32; errchn++) {
					if (subirq & (0x1 << errchn))
						break;
				}
				errtype = (uint8_t)icnt_n;
			} else if (icnt_n >= 13) {
				/* ipi fatal */
				errchn = (uint8_t)(icnt_n - 13);
				errtype = (uint8_t)(0x80 | subirq);
			}
#endif
			if (param->irq_debug & MIPI_HOST_IRQ_DEBUG_PRERR)
				mipierr("  %s: 0x%x%s",
					g_mh_icnt_names[icnt_n], subirq, perr);
			else
				mipidbg("  %s: 0x%x%s",
					g_mh_icnt_names[icnt_n], subirq, perr);
			icnt_p[icnt_n]++;
			err_occurred = 1;
			irq_do &= ~mask;
			if (!irq_do)
				break;
		}
	}

	i = 1;
	if (param->sigfun_cfg & 0x1) {
		if (mipi_host_send_sig_fatal(hdev, irq, subirq) == 0 &&
			(param->sigfun_cfg & 0x2))
			i = 0;
	}
	if (icnt->st_main > param->irq_cnt || i == 0)
		mipi_host_irq_disable(hdev);

	if (this_irq >= 0)
		enable_irq(this_irq);

#ifdef CONFIG_HOBOT_DIAG
	mipi_host_diag_report(hdev, err_occurred, errchn, errtype, irq);
#endif
	return IRQ_HANDLED;
}

#ifdef MIPI_HOST_INT_USE_TIMER
static void mipi_host_irq_timer_func(unsigned long data)
{
	mipi_hdev_t *hdev = (mipi_hdev_t *)data;
	mipi_host_t *host = &hdev->host;
	void __iomem *iomem = host->iomem;
	unsigned long jiffi;

	if (!hdev || !iomem)
		return;

	if (hdev->irq_timer_en) {
		if (iomem) {
			hdev->irq_st_main = mipi_getreg(iomem + REG_MIPI_HOST_INT_ST_MAIN);
			if (hdev->irq_st_main)
				mipi_host_irq_func(-1, hdev);
		}
		jiffi = get_jiffies_64() + msecs_to_jiffies(50);
		mod_timer(&hdev->irq_timer, jiffi);
	} else {
		jiffi = get_jiffies_64() + msecs_to_jiffies(500);
		mod_timer(&hdev->irq_timer, jiffi);
	}
}
#endif
#endif

static int32_t mipi_host_dphy_wait_stop(mipi_hdev_t *hdev, mipi_host_cfg_t *cfg)
{
	if (!hdev)
		return -1;
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	mipi_host_param_t *param = &host->param;
	void __iomem *iomem = host->iomem;
	uint16_t ncount = 0;
	uint32_t stopstate = 0;

	if (!iomem || !cfg)
		return -1;

	mipiinfo("check phy stop state");
	/*Check that data lanes are in Stop state*/
	do {
		stopstate = mipi_getreg(iomem + REG_MIPI_HOST_PHY_STOPSTATE);
		if ((stopstate & 0xF) == HOST_DPHY_LANE_STOP(cfg->lane))
			return 0;
		mdelay(1);
		ncount++;
	} while (param->notimeout || ncount <= param->wait_ms);

	mipierr("lane state of host phy is error: 0x%x", stopstate);
	return -1;
}

/**
 * @brief mipi_host_dphy_start_hs_reception : check if mipi host in hs mode
 *
 * @param []
 *
 * @return int32_t : 0/-1
 */
static int32_t mipi_host_dphy_start_hs_reception(mipi_hdev_t *hdev)
{
	if (!hdev)
		return -1;
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	mipi_host_param_t *param;
	void __iomem *iomem = host->iomem;
	uint16_t ncount = 0;
	uint32_t state = 0;
	int poth;

	if (!iomem)
		return -1;

	mipiinfo("check hs reception");
	/* param from main host if ex */
	if (hdev->is_ex) {
		poth = mipi_host_port_other(hdev);
		if(poth < 0)
			return -1;
		param = &(g_hdev[poth]->host.param);
	} else {
		param = &host->param;
	}
	/*Check that clock lane is in HS mode*/
	do {
		state = mipi_getreg(iomem + REG_MIPI_HOST_PHY_RX);
		if ((state & HOST_DPHY_RX_HS) == HOST_DPHY_RX_HS) {
			mipiinfo("entry hs reception");
			return 0;
		}
		ncount++;
		mdelay(1);
	} while (param->notimeout || ncount <= param->wait_ms);

	mipiinfo("hs reception check error 0x%x", state);
	return -1;
}

/**
 * @brief mipi_host_start : set mipi host start working
 *
 * @param [] void :
 *
 * @return int32_t : 0/-1
 */
static int32_t mipi_host_start(mipi_hdev_t *hdev)
{
	if (!hdev)
		return -1;
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	mipi_host_param_t *param = &host->param;
	void __iomem *iomem = host->iomem;
	uint32_t nocheck;
	int poth;

	if (!iomem)
		return -1;

	if (!hdev->is_ex && !param->need_stop_check && param->stop_check_instart) {
		if (0 != mipi_host_dphy_wait_stop(hdev, &host->cfg)) {
			/*Release DWC_mipi_csi2_host from reset*/
			mipierr("wait phy stop state error!!!");
			return -1;
		}
	}
	poth = mipi_host_port_other(hdev);
	if(poth < 0)
		return -1;
	nocheck = (hdev->is_ex) ?
		g_hdev[poth]->host.param.nocheck : param->nocheck;
	if (!nocheck) {
		if (0 != mipi_host_dphy_start_hs_reception(hdev)) {
			mipierr("hs reception state error!!!");
			return -1;
		}
	}

#if MIPI_HOST_INT_DBG
	if(!hdev->is_ex) {
		mipi_host_irq_enable(hdev);
	}
#endif

	return 0;
}

/**
 * @brief mipi_host_stop : set mipi host stop working
 *
 * @param [] void :
 *
 * @return int32_t : 0/-1
 */
static int32_t mipi_host_stop(mipi_hdev_t *hdev)
{
	if (!hdev)
		return -1;

	/*stop mipi host here?*/
#if MIPI_HOST_INT_DBG
	if(!hdev->is_ex) {
		mipi_host_irq_disable(hdev);
	}
#endif

	return 0;
}

/**
 * @brief mipi_host_deinit : mipi host deinit function
 *
 * @param [] void :
 *
 * @return int32_t : 0/-1
 */
static void mipi_host_deinit(mipi_hdev_t *hdev)
{
	mipi_host_t *host = &hdev->host;
	void __iomem  *iomem = host->iomem;

	if (!hdev || !iomem)
		return;

#ifdef CONFIG_HOBOT_MIPI_PHY
	mipi_host_dphy_reset(iomem);
#endif

	/*Set Synopsys D-PHY Reset*/
	mipi_putreg(iomem + REG_MIPI_HOST_DPHY_RSTZ, MIPI_HOST_CSI2_RESETN);
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_SHUTDOWNZ, MIPI_HOST_CSI2_RESETN);
	/*Release DWC_mipi_csi2_host from reset*/
	mipi_putreg(iomem + REG_MIPI_HOST_CSI2_RESETN, MIPI_HOST_CSI2_RESETN);

	if (hdev->ipi_clock != 0) {
		mipi_host_configure_clk(hdev, g_mh_ipiclk_name[hdev->port], 0, 0);
		hdev->ipi_clock = 0;
	}

	return;
}

/**
 * @brief mipi_host_init : mipi host init function
 *
 * @param [in] cfg : mipi host cfgler's setting
 *
 * @return int32_t : 0/-1
 */
static int32_t mipi_host_init(mipi_hdev_t *hdev, mipi_host_cfg_t *cfg)
{
	if (!hdev)
		return -1;
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	mipi_host_param_t *param;
	void __iomem  *iomem = host->iomem;
	unsigned long pixclk = 0;
	int poth;

	if (!iomem)
		return -1;

	mipiinfo("init begin");
	mipiinfo("%d lane %dx%d %dfps datatype 0x%x",
			 cfg->lane, cfg->width, cfg->height, cfg->fps, cfg->datatype);

	/* param from main host if ex */
	if (hdev->is_ex) {
		poth = mipi_host_port_other(hdev);
		if(poth < 0)
			return -1;
		param = &(g_hdev[poth]->host.param);
	} else {
		param = &host->param;
	}

	if (!hdev->is_ex) {
		/* cfg->mclk:
		 * 0      : disable only
		 * 1      : enable only
		 * 2~24   : invalid and drop
		 * 25~9280: invalid and error
		 * 9281~  : *10K = freq -> 92.81~655.35MHz
		 */
		if (cfg->mclk >= (MIPI_HOST_SNRCLK_FREQ_MIN / MIPI_HOST_SNRCLK_FREQ_BASE)) {
			if (mipi_host_snrclk_set_freq(hdev, (uint32_t)(cfg->mclk * MIPI_HOST_SNRCLK_FREQ_BASE)) ||
					mipi_host_snrclk_set_en(hdev, 1)) {
				return -1;
			}
		} else if (cfg->mclk > 24) {
			mipiinfo("mclk %d should >= %lu(%luHz)", cfg->mclk,
				(MIPI_HOST_SNRCLK_FREQ_MIN / MIPI_HOST_SNRCLK_FREQ_BASE), MIPI_HOST_SNRCLK_FREQ_MIN);
			return -1;
		} else if (cfg->mclk > 1) {
			mipiinfo("mclk %d drop", cfg->mclk);
		} else if (cfg->mclk == 1) {
			mipi_host_snrclk_set_en(hdev, 1);
		} else {
			mipi_host_snrclk_set_en(hdev, 0);
		}
		/* ipi clock */
		pixclk = mipi_host_pixel_clk_select(hdev, cfg);
		if (0 == pixclk) {
			mipierr("pixel clk config error!");
			return -1;
		}
		hdev->ipi_clock = pixclk;
	}
#ifdef CONFIG_HOBOT_MIPI_PHY
	mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_HOST, hdev->port,
			MIPI_CFGCLKFREQRANGE, MIPI_HOST_CFGCLK_DEFAULT);
#endif
	/*Set DWC_mipi_csi2_host reset*/
	mipi_putreg(iomem + REG_MIPI_HOST_CSI2_RESETN, MIPI_HOST_CSI2_RESETN);
	/*Set Synopsys D-PHY Reset*/
	mipi_putreg(iomem + REG_MIPI_HOST_DPHY_RSTZ, MIPI_HOST_CSI2_RESETN);
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_SHUTDOWNZ, MIPI_HOST_CSI2_RESETN);

#ifdef CONFIG_HOBOT_MIPI_PHY
	if (0 != mipi_host_dphy_initialize(cfg->mipiclk, cfg->lane, cfg->settle, iomem)) {
		mipierr("dphy initialize error!!!");
		mipi_host_deinit(hdev);
		return -1;
	}
#endif
	/*Clear Synopsys D-PHY Reset*/
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_SHUTDOWNZ, MIPI_HOST_CSI2_RAISE);
	mipi_putreg(iomem + REG_MIPI_HOST_DPHY_RSTZ, MIPI_HOST_CSI2_RAISE);
	/*Configure the number of active lanes*/
	mipi_putreg(iomem + REG_MIPI_HOST_N_LANES, cfg->lane - 1);
	udelay(1000);
	/*Configure the vc extersion function*/
	if (!param->vcext_en)
		mipi_putreg(iomem + REG_MIPI_HOST_VC_EXTENSION, MIPI_HOST_VC_EXT_LEGACY);
	else
		mipi_putreg(iomem + REG_MIPI_HOST_VC_EXTENSION, MIPI_HOST_VC_EXT_ENABLE);
	/*Release DWC_mipi_csi2_host from reset*/
	mipi_putreg(iomem + REG_MIPI_HOST_CSI2_RESETN, MIPI_HOST_CSI2_RAISE);
#if MIPI_HOST_INT_DBG
	memset(&host->icnt, 0x0, sizeof(host->icnt));
#endif
	if (!hdev->is_ex && !param->need_stop_check && !param->stop_check_instart) {
		if (0 != mipi_host_dphy_wait_stop(hdev, cfg)) {
			/*Release DWC_mipi_csi2_host from reset*/
			mipierr("wait phy stop state error!!!");
			mipi_host_deinit(hdev);
			return -1;
		}
	}
	if (!hdev->is_ex) {
		cfg->hsaTime = cfg->hsaTime ? cfg->hsaTime : MIPI_HOST_HSATIME;
		cfg->hbpTime = cfg->hbpTime ? cfg->hbpTime : MIPI_HOST_HBPTIME;
		cfg->hsdTime = cfg->hsdTime ? cfg->hsdTime : mipi_host_get_hsd(hdev, cfg, pixclk);
		if (0 != mipi_host_configure_ipi(hdev, cfg)) {
			mipierr("configure ipi error!!!");
			mipi_host_deinit(hdev);
			return -1;
		}
	}
	memcpy(&host->cfg, cfg, sizeof(mipi_host_cfg_t));
	mipiinfo("init end");
	return 0;
}

static int hobot_mipi_host_open(struct inode *inode, struct file *file)
{
	mipi_hdev_t *hdev = container_of(inode->i_cdev, mipi_hdev_t, cdev);
	mipi_user_t *user = &hdev->user;
	mipi_host_param_t *param = &hdev->host.param;
	struct device *dev = hdev->dev;

	mutex_lock(&user->open_mutex);
	mipidbg("open as %d", user->open_cnt);
	if (user->open_cnt == 0) {
		mutex_init(&user->mutex);
		spin_lock_init(&hdev->siglock);
		memset(&hdev->sigpid, 0, sizeof(mipi_host_sigpid_t));
		init_waitqueue_head(&user->pre_wq);
		user->pre_state = MIPI_PRE_STATE_DEFAULT;
		user->init_cnt = 0;
		user->start_cnt = 0;
		mipi_host_configure_clk(hdev, MIPI_HOST_CFGCLK_NAME, MIPI_HOST_CFGCLK_MHZ, 1);
		mipi_host_configure_clk(hdev, MIPI_HOST_REFCLK_NAME, MIPI_HOST_REFCLK_MHZ, 1);
	}
	user->open_cnt++;
	mutex_unlock(&user->open_mutex);

	file->private_data = hdev;
	return 0;
}

static int hobot_mipi_host_close(struct inode *inode, struct file *file)
{
	mipi_hdev_t *hdev = file->private_data;
	mipi_hdev_t *ex_hdev;
	mipi_host_t *host = &hdev->host;
	mipi_user_t *user = &hdev->user;
	mipi_host_param_t *param = &host->param;
	struct device *dev = hdev->dev;

	mutex_lock(&user->open_mutex);
	if (user->open_cnt > 0)
		user->open_cnt--;
	mipiinfo("close as %d", user->open_cnt);
	if (user->open_cnt == 0) {
		if (host->state != MIPI_STATE_DEFAULT) {
			mipi_host_stop(hdev);
			mipi_host_deinit(hdev);
			if (hdev->ex_hdev) {
				ex_hdev = (mipi_hdev_t *)(hdev->ex_hdev);
				mipi_host_stop(ex_hdev);
				mipi_host_deinit(ex_hdev);
			}
			host->state = MIPI_STATE_DEFAULT;
		}
		mipi_host_configure_clk(hdev, MIPI_HOST_CFGCLK_NAME, 0, 0);
		mipi_host_configure_clk(hdev, MIPI_HOST_REFCLK_NAME, 0, 0);
		mipi_host_snrclk_close_freq(hdev, 1);
	}
	mutex_unlock(&user->open_mutex);
	return 0;
}

static long hobot_mipi_host_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	mipi_hdev_t *hdev = file->private_data;
	mipi_hdev_t *ex_hdev;
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	mipi_user_t *user = &hdev->user;
	int ret = 0;
#ifdef CONFIG_HOBOT_MIPI_REG_OPERATE
	void __iomem *iomem = host->iomem;
	reg_t reg;
	uint32_t regv;
#endif

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != MIPIHOSTIOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case MIPIHOSTIOC_INIT:
		{
			mipi_host_cfg_t mipi_host_cfg;
			unsigned long flags;
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("init user mutex lock error");
				return -EINVAL;
			}
			mipiinfo("init cmd: %d %s", user->init_cnt,
					(user->init_cnt) ? "drop" : "real");
			if (!arg || copy_from_user((void *)&mipi_host_cfg,
				   (void __user *)arg, sizeof(mipi_host_cfg_t))) {
				mipierr("init error, config %p from user error", (void __user *)arg);
				mutex_unlock(&user->mutex);
				return -EINVAL;
			}
			if (user->init_cnt == 0) {
				if (MIPI_STATE_DEFAULT != host->state) {
					mipiinfo("re-init, pre state: %d(%s)",
							 host->state, g_mh_state[host->state]);
				}
				if (0 != mipi_host_configure_lanemode(hdev, mipi_host_cfg.lane)) {
					mipierr("require %dlane error!!!", mipi_host_cfg.lane);
					mutex_unlock(&user->mutex);
					return -EACCES;
				}
				if (hdev->ex_hdev) {
					mipi_host_cfg_t mipi_exth_cfg;
					memcpy(&mipi_exth_cfg, &mipi_host_cfg, sizeof(mipi_host_cfg_t));
					ex_hdev = (mipi_hdev_t *)(hdev->ex_hdev);
					if (0 != (ret = mipi_host_init(ex_hdev, &mipi_exth_cfg))) {
						mutex_unlock(&user->mutex);
						return ret;
					}
				}
				if (0 != (ret = mipi_host_init(hdev, &mipi_host_cfg))) {
					mipierr("init error: %d", ret);
					mutex_unlock(&user->mutex);
					return ret;
				}
				spin_lock_irqsave(&hdev->siglock, flags);
				hdev->sigpid.pid = pid_nr(get_task_pid(current, PIDTYPE_PID));
				hdev->sigpid.fatal_mask = 0;
				spin_unlock_irqrestore(&hdev->siglock, flags);
				if (user->pre_state == MIPI_PRE_STATE_INITING) {
					user->pre_state = MIPI_PRE_STATE_INITED;
					user->pre_done = true;
					wake_up_interruptible(&user->pre_wq);
				} else {
					user->pre_state = MIPI_PRE_STATE_INITED;
				}
				host->state = MIPI_STATE_INIT;
			} else if (mipi_host_configure_cmp(&host->cfg, &mipi_host_cfg)) {
				mipiinfo("warning: init config mismatch");
			}
			user->init_cnt++;
			mutex_unlock(&user->mutex);
		}
		break;
	case MIPIHOSTIOC_DEINIT:
		{
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("deinit user mutex lock error");
				return -EINVAL;
			}
			if (user->init_cnt > 0)
				user->init_cnt--;
			mipiinfo("deinit cmd: %d %s", user->init_cnt,
					(user->init_cnt) ? "drop" : "real");
			if (user->init_cnt == 0) {
				if (MIPI_STATE_START == host->state) {
					mipi_host_stop(hdev);
					if (hdev->ex_hdev) {
						ex_hdev = (mipi_hdev_t *)(hdev->ex_hdev);
						mipi_host_stop(ex_hdev);
					}
					user->start_cnt = 0;
				}
				mipi_host_deinit(hdev);
				if (hdev->ex_hdev) {
					ex_hdev = (mipi_hdev_t *)(hdev->ex_hdev);
					mipi_host_deinit(ex_hdev);
					ex_hdev->is_ex = 0;
					hdev->ex_hdev = NULL;
				}
				user->pre_state = MIPI_PRE_STATE_DEFAULT;
				host->state = MIPI_STATE_DEFAULT;
			}
			mutex_unlock(&user->mutex);
		}
		break;
	case MIPIHOSTIOC_START:
		{
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("start user mutex lock error");
				return -EINVAL;
			}
			mipiinfo("start cmd: %d %s", user->start_cnt,
					(user->start_cnt) ? "drop" : "real");
			if (user->start_cnt == 0) {
				if (MIPI_STATE_START == host->state) {
					mipiinfo("already in start state");
					user->start_cnt++;
					mutex_unlock(&user->mutex);
					break;
				} else if (MIPI_STATE_INIT != host->state &&
						   MIPI_STATE_STOP != host->state) {
					mipierr("state error, current state: %d(%s)",
							host->state, g_mh_state[host->state]);
					mutex_unlock(&user->mutex);
					return -EBUSY;
				}
				if (hdev->ex_hdev) {
					ex_hdev = (mipi_hdev_t *)(hdev->ex_hdev);
					if (0 != (ret = mipi_host_start(ex_hdev))) {
						mutex_unlock(&user->mutex);
						return ret;
					}
				}
				if (0 != (ret = mipi_host_start(hdev))) {
					mipierr("start error: %d", ret);
					mutex_unlock(&user->mutex);
					return ret;
				}
				if (user->pre_state == MIPI_PRE_STATE_STARTING) {
					user->pre_state = MIPI_PRE_STATE_STARTED;
					user->pre_done = true;
					wake_up_interruptible(&user->pre_wq);
				} else {
					user->pre_state = MIPI_PRE_STATE_STARTED;
				}
				host->state = MIPI_STATE_START;
			}
			user->start_cnt++;
			mutex_unlock(&user->mutex);
		}
		break;
	case MIPIHOSTIOC_STOP:
		{
			int start_cnt_save;
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("stop user mutex lock error");
				return -EINVAL;
			}
			start_cnt_save = user->start_cnt;
			if (user->start_cnt > 0)
				user->start_cnt--;
			mipiinfo("stop cmd: %d %s", user->start_cnt,
					(user->start_cnt) ? "drop" : "real");
			if (user->start_cnt == 0) {
				if (MIPI_STATE_STOP == host->state) {
					mipiinfo("already in stop state");
					mutex_unlock(&user->mutex);
					break;
				} else if (MIPI_STATE_START != host->state) {
					mipierr("state error, current state: %d(%s)",
							host->state, g_mh_state[host->state]);
					user->start_cnt = start_cnt_save;
					mutex_unlock(&user->mutex);
					return -EBUSY;
				}
				if (hdev->ex_hdev) {
					ex_hdev = (mipi_hdev_t *)(hdev->ex_hdev);
					mipi_host_stop(ex_hdev);
				}
				if (0 != (ret = mipi_host_stop(hdev))) {
					mipierr("stop error: %d", ret);
					user->start_cnt = start_cnt_save;
					mutex_unlock(&user->mutex);
					return ret;
				}
				user->pre_state = MIPI_PRE_STATE_INITED;
				host->state = MIPI_STATE_STOP;
			}
			mutex_unlock(&user->mutex);
		}
		break;
	case MIPIHOSTIOC_SNRCLK_SET_EN:
		{
			uint32_t enable;

			mipiinfo("snrclk set en cmd");
			if (!arg || get_user(enable, (uint32_t *)arg)) {
				mipierr("get data from user failed");
				return -EFAULT;
			}
			if (mipi_host_snrclk_set_en(hdev, enable))
				return -EACCES;
		}
		break;
	case MIPIHOSTIOC_SNRCLK_SET_FREQ:
		{
			uint32_t freq;

			mipiinfo("snrclk set freq cmd");
			if (!arg || get_user(freq, (uint32_t *)arg)) {
				mipierr("get data from user failed");
				return -EFAULT;
			}
			if (mipi_host_snrclk_set_freq(hdev, freq))
				return -EACCES;
		}
		break;
	case MIPIHOSTIOC_PRE_INIT_REQUEST:
		{
			uint32_t timeout_ms = 0, wait = 0;

			if (arg && get_user(timeout_ms, (uint32_t *)arg)) {
				mipierr("get data from user failed");
				return -EFAULT;
			}
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("pre_init_request user mutex lock error");
				return -EINVAL;
			}
			if (user->init_cnt == 0) {
				if (user->pre_state == MIPI_PRE_STATE_DEFAULT) {
					mipiinfo("pre_init_request cmd: initing");
					user->pre_state = MIPI_PRE_STATE_INITING;
				} else if (user->pre_state == MIPI_PRE_STATE_INITING) {
					wait = 1;
				} else {
					mipiinfo("pre_init_request cmd: preinited drop");
					ret = -EACCES;
				}
			} else {
				mipiinfo("pre_init_request cmd: inited drop");
				ret = -EACCES;
			}
			mutex_unlock(&user->mutex);
			if (wait) {
				user->pre_done = false;
				if (timeout_ms)
					wait_event_interruptible_timeout(user->pre_wq,
						user->pre_done, msecs_to_jiffies(timeout_ms));
				else
					wait_event_interruptible(user->pre_wq,
						user->pre_done);
				if (mutex_lock_interruptible(&user->mutex)) {
					mipierr("pre init wait user mutex lock error");
					return -EINVAL;
				}
				if (user->init_cnt == 0 &&
					user->pre_state == MIPI_PRE_STATE_DEFAULT) {
					mipiinfo("pre_init_request cmd: wait & initing");
					user->pre_state = MIPI_PRE_STATE_INITING;
				} else {
					mipiinfo("pre_init_request cmd: wait & drop");
					ret = -EACCES;
				}
				mutex_unlock(&user->mutex);
			}
		}
		break;
	case MIPIHOSTIOC_PRE_START_REQUEST:
		{
			uint32_t timeout_ms = 0, wait = 0;

			if (arg && get_user(timeout_ms, (uint32_t *)arg)) {
				mipierr("get data from user failed");
				return -EFAULT;
			}
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("pre_start_request user mutex lock error");
				return -EINVAL;
			}
			if (user->start_cnt == 0) {
				if (user->pre_state == MIPI_PRE_STATE_INITED) {
					mipiinfo("pre_start_request cmd: starting");
					user->pre_state = MIPI_PRE_STATE_STARTING;
				} else if (user->pre_state == MIPI_PRE_STATE_STARTING) {
					wait = 1;
				} else {
					mipiinfo("pre_start_request cmd: prestarted drop");
					ret = -EACCES;
				}
			} else {
				mipiinfo("pre_start_request cmd: started drop");
				ret = -EACCES;
			}
			mutex_unlock(&user->mutex);
			if (wait) {
				user->pre_done = false;
				if (timeout_ms)
					wait_event_interruptible_timeout(user->pre_wq,
						user->pre_done, msecs_to_jiffies(timeout_ms));
				else
					wait_event_interruptible(user->pre_wq,
						user->pre_done);
				if (mutex_lock_interruptible(&user->mutex)) {
					mipierr("pre_start_request wait user mutex lock error");
					return -EINVAL;
				}
				if (user->start_cnt == 0 &&
					user->pre_state == MIPI_PRE_STATE_INITED) {
					mipiinfo("pre_start_request cmd: wait & starting");
					user->pre_state = MIPI_PRE_STATE_STARTING;
				} else {
					mipiinfo("pre_start_request cmd: wait & drop");
					ret = -EACCES;
				}
				mutex_unlock(&user->mutex);
			}
		}
		break;
	case MIPIHOSTIOC_PRE_INIT_RESULT:
		{
			uint32_t result = 0, wake = 0;

			if (!arg || get_user(result, (uint32_t *)arg)) {
				mipierr("get data from user failed");
				return -EFAULT;
			}
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("pre_init_result user mutex lock error");
				return -EINVAL;
			}
			if (user->init_cnt == 0 &&
				user->pre_state == MIPI_PRE_STATE_INITING) {
				if (result)
					user->pre_state = MIPI_PRE_STATE_DEFAULT;
				else
					user->pre_state = MIPI_PRE_STATE_INITED;
				wake = 1;
			}
			mutex_unlock(&user->mutex);
			if (wake) {
				mipiinfo("pre_init_result cmd: %s wake",
					(result) ? "falied" : "done");
				user->pre_done = true;
				wake_up_interruptible(&user->pre_wq);
			} else {
				mipiinfo("pre_init_result cmd: %s drop",
					(result) ? "falied" : "done");
			}
		}
		break;
	case MIPIHOSTIOC_PRE_START_RESULT:
		{
			uint32_t result = 0, wake = 0;

			if (!arg || get_user(result, (uint32_t *)arg)) {
				mipierr("get data from user failed");
				return -EFAULT;
			}
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("pre_start_result user mutex lock error");
				return -EINVAL;
			}
			if (user->start_cnt == 0 &&
				user->pre_state == MIPI_PRE_STATE_STARTING) {
				if (result)
					user->pre_state = MIPI_PRE_STATE_INITED;
				else
					user->pre_state = MIPI_PRE_STATE_STARTED;
				wake = 1;
			}
			mutex_unlock(&user->mutex);
			if (wake) {
				mipiinfo("pre_start_result cmd: %s wake",
					(result) ? "falied" : "done");
				user->pre_done = true;
				wake_up_interruptible(&user->pre_wq);
			} else {
				mipiinfo("pre_start_result cmd: %s drop",
					(result) ? "falied" : "done");
			}
		}
		break;
	case MIPIHOSTIOC_IPI_RESET:
		{
			mipi_host_ipi_reset_t ipi_reset;
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("init user mutex lock error");
				return -EINVAL;
			}
			mipiinfo("ipi reset cmd");
			if (!arg || copy_from_user((void *)&ipi_reset,
				   (void __user *)arg, sizeof(mipi_host_ipi_reset_t))) {
				mipierr("ipi reset erorr, %p from user error", (void __user *)arg);
				mutex_unlock(&user->mutex);
				return -EINVAL;
			}
			if (user->init_cnt == 0) {
				mipierr("state error: not inited");
				mutex_unlock(&user->mutex);
				return -EACCES;
			}
			if (mipi_host_ipi_reset(hdev, &ipi_reset))
				ret = -EFAULT;
			mutex_unlock(&user->mutex);
		}
		break;
	case MIPIHOSTIOC_IPI_GET_INFO:
		{
			mipi_host_ipi_info_t ipi_info;
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("init user mutex lock error");
				return -EINVAL;
			}
			mipiinfo("ipi get info cmd");
			if (!arg || copy_from_user((void *)&ipi_info,
				   (void __user *)arg, sizeof(mipi_host_ipi_info_t))) {
				mipierr("ipi get erorr, %p from user error", (void __user *)arg);
				mutex_unlock(&user->mutex);
				return -EINVAL;
			}
			if (user->init_cnt == 0) {
				mipierr("state error: not inited");
				mutex_unlock(&user->mutex);
				return -EACCES;
			}
			if (mipi_host_ipi_get_info(hdev, &ipi_info)) {
				ret = -EFAULT;
			} else if (copy_to_user((void __user *)arg,
					(void *)&ipi_info, sizeof(mipi_host_ipi_info_t))) {
				mipierr("ipi get erorr, %p to user error", (void __user *)arg);
				ret = -EINVAL;
			}
			mutex_unlock(&user->mutex);
		}
		break;
	case MIPIHOSTIOC_IPI_SET_INFO:
		{
			mipi_host_ipi_info_t ipi_info;
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("init user mutex lock error");
				return -EINVAL;
			}
			mipiinfo("ipi set info cmd");
			if (!arg || copy_from_user((void *)&ipi_info,
				   (void __user *)arg, sizeof(mipi_host_ipi_info_t))) {
				mipierr("ipi set erorr, %p from user error", (void __user *)arg);
				mutex_unlock(&user->mutex);
				return -EINVAL;
			}
			if (user->init_cnt == 0) {
				mipierr("state error: not inited");
				mutex_unlock(&user->mutex);
				return -EACCES;
			}
			if (mipi_host_ipi_set_info(hdev, &ipi_info))
				ret = -EFAULT;
			mutex_unlock(&user->mutex);
		}
		break;
	case MIPIHOSTIOC_SIGPID_SET:
		{
			unsigned long flags;
			mipi_host_sigpid_t sigpid;
			if (!arg || copy_from_user((void *)&sigpid,
			    (void __user *)arg, sizeof(mipi_host_sigpid_t))) {
				mipierr("pidsiig set erorr, %p from user error", (void __user *)arg);
				return -EINVAL;
			}
			spin_lock_irqsave(&hdev->siglock, flags);
			memcpy(&hdev->sigpid, &sigpid, sizeof(mipi_host_sigpid_t));
			spin_unlock_irqrestore(&hdev->siglock, flags);
		}
		break;
#ifdef CONFIG_HOBOT_MIPI_REG_OPERATE
	case MIPIHOSTIOC_READ:
		{
			reg_t reg;
			if (!arg) {
				mipierr("reg read error, reg should not be NULL");
				return -EINVAL;
			}
			if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
				mipierr("reg read error, copy data from user failed");
				return -EFAULT;
			}
			reg.value = mipi_getreg(iomem + reg.offset);
			if (copy_to_user((void __user *)arg, (void *)&reg, sizeof(reg))) {
				mipierr("reg read error, copy data to user failed");
				return -EFAULT;
			}
		}
		break;
	case MIPIHOSTIOC_WRITE:
		{
			reg_t reg;
			uint32_t regv;
			if (!arg) {
				mipierr("reg write error, reg should not be NULL");
				return -EINVAL;
			}
			if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
				mipierr("reg write error, copy data from user failed");
				return -EFAULT;
			}
			mipi_putreg(iomem + reg.offset, reg.value);
			regv = mipi_getreg(iomem + reg.offset);
			if (regv != reg.value) {
				mipierr("reg write error, write 0x%x got 0x%x", reg.value, regv);
				return -EFAULT;
			}
		}
		break;
#endif
	default:
		return -ERANGE;
	}
	return ret;
}

static const struct file_operations hobot_mipi_host_fops = {
	.owner		= THIS_MODULE,
	.open		= hobot_mipi_host_open,
	.release	= hobot_mipi_host_close,
	.unlocked_ioctl = hobot_mipi_host_ioctl,
	.compat_ioctl = hobot_mipi_host_ioctl,
};

/* sysfs dec & add macro */
#define MIPI_HOST_ATTR_DEC(a, m, fsh, fst) \
static struct device_attribute mh_##a##_attr = { \
	.attr   = { \
		.name = __stringify(a), \
		.mode = m, \
	}, \
	.show   = fsh, \
	.store  = fst, \
}
#define MIPI_HOST_ATTR_ADD(a) \
	&mh_##a##_attr.attr

/* sysfs for mipi host devices' params */
static int mipi_host_param_idx(const char *name)
{
	int i;
	int param_num = ARRAY_SIZE(g_mh_param_names);

	for (i = 0; i < param_num; i++) {
		if (strcmp(g_mh_param_names[i], name) == 0)
			return i;
	}

	return -1;
}

static ssize_t mipi_host_param_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	mipi_hdev_t *hdev = dev_get_drvdata(dev);
	uint32_t *param = (uint32_t *)(&hdev->host.param);
	char *s = buf;
	int idx;

	idx = mipi_host_param_idx(attr->attr.name);
	if (idx >= 0) {
		/* update the real value for snrclk_freq */
		if(!strcmp(attr->attr.name, "snrclk_freq") &&
			hdev->host.snrclk.index >= 0) {
			hdev->host.param.snrclk_freq =
				(uint32_t)(mipi_host_get_clk(hdev,
				g_mh_snrclk_name[hdev->host.snrclk.index]));
		}
		s += sprintf(s, "%u\n", param[idx]);
	}

	return (s - buf);
}

static ssize_t mipi_host_param_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	mipi_hdev_t *hdev = dev_get_drvdata(dev);
	uint32_t *param = (uint32_t *)(&hdev->host.param);
	int ret, error = -EINVAL;
	int idx;
	uint32_t val, pbak;

	idx = mipi_host_param_idx(attr->attr.name);
	if (idx >= 0) {
		ret = kstrtouint(buf, 0, &val);
		if (!ret) {
			pbak = param[idx];
			param[idx] = val;
			error = 0;
		}
	}

	if (error == 0) {
		if(!strcmp(attr->attr.name, "snrclk_en")) {
			param[idx] = pbak;
			error = mipi_host_snrclk_set_en(hdev, val);
		} else if (!strcmp(attr->attr.name, "snrclk_freq")) {
			if (val == 0xffffffff) {
				mipi_host_snrclk_close_freq(hdev, 0);
			} else if (val > 0 || hdev->host.snrclk.sys_cnt > 0) {
				error = mipi_host_snrclk_set_freq(hdev, val);
				if (error == 0) {
					if (val)
						hdev->host.snrclk.sys_cnt++;
					else
						hdev->host.snrclk.sys_cnt--;
				}
			} else {
				error = -EPERM;
			}
		}
	}

	return (error ? error : count);
}

#define MIPI_HOST_PARAM_DEC(a) \
	MIPI_HOST_ATTR_DEC(a, 0644, \
		mipi_host_param_show, mipi_host_param_store)
#define MIPI_HOST_PARAM_ADD(a) \
	MIPI_HOST_ATTR_ADD(a)

MIPI_HOST_PARAM_DEC(nocheck);
MIPI_HOST_PARAM_DEC(notimeout);
MIPI_HOST_PARAM_DEC(wait_ms);
MIPI_HOST_PARAM_DEC(dbg_value);
MIPI_HOST_PARAM_DEC(adv_value);
MIPI_HOST_PARAM_DEC(need_stop_check);
MIPI_HOST_PARAM_DEC(stop_check_instart);
MIPI_HOST_PARAM_DEC(cut_through);
MIPI_HOST_PARAM_DEC(ipi_16bit);
MIPI_HOST_PARAM_DEC(ipi_force);
MIPI_HOST_PARAM_DEC(ipi_limit);
MIPI_HOST_PARAM_DEC(snrclk_en);
MIPI_HOST_PARAM_DEC(snrclk_freq);
MIPI_HOST_PARAM_DEC(sigfun_cfg);
MIPI_HOST_PARAM_DEC(sigwait_ms);
MIPI_HOST_PARAM_DEC(sigfatal_mask);
MIPI_HOST_PARAM_DEC(vcext_en);
MIPI_HOST_PARAM_DEC(ipi1_dt);
#if MIPIHOST_CHANNEL_NUM >= 2
MIPI_HOST_PARAM_DEC(ipi2_dt);
#endif
#if MIPIHOST_CHANNEL_NUM >= 3
MIPI_HOST_PARAM_DEC(ipi3_dt);
#endif
#if MIPIHOST_CHANNEL_NUM >= 4
MIPI_HOST_PARAM_DEC(ipi4_dt);
#endif
#if MIPI_HOST_INT_DBG
MIPI_HOST_PARAM_DEC(irq_cnt);
MIPI_HOST_PARAM_DEC(irq_debug);
#endif

static struct attribute *param_attr[] = {
	MIPI_HOST_PARAM_ADD(nocheck),
	MIPI_HOST_PARAM_ADD(notimeout),
	MIPI_HOST_PARAM_ADD(wait_ms),
	MIPI_HOST_PARAM_ADD(dbg_value),
	MIPI_HOST_PARAM_ADD(adv_value),
	MIPI_HOST_PARAM_ADD(need_stop_check),
	MIPI_HOST_PARAM_ADD(stop_check_instart),
	MIPI_HOST_PARAM_ADD(cut_through),
	MIPI_HOST_PARAM_ADD(ipi_16bit),
	MIPI_HOST_PARAM_ADD(ipi_force),
	MIPI_HOST_PARAM_ADD(ipi_limit),
	MIPI_HOST_PARAM_ADD(snrclk_en),
	MIPI_HOST_PARAM_ADD(snrclk_freq),
	MIPI_HOST_PARAM_ADD(sigfun_cfg),
	MIPI_HOST_PARAM_ADD(sigwait_ms),
	MIPI_HOST_PARAM_ADD(sigfatal_mask),
	MIPI_HOST_PARAM_ADD(vcext_en),
	MIPI_HOST_PARAM_ADD(ipi1_dt),
#if MIPIHOST_CHANNEL_NUM >= 2
	MIPI_HOST_PARAM_ADD(ipi2_dt),
#endif
#if MIPIHOST_CHANNEL_NUM >= 3
	MIPI_HOST_PARAM_ADD(ipi3_dt),
#endif
#if MIPIHOST_CHANNEL_NUM >= 4
	MIPI_HOST_PARAM_ADD(ipi4_dt),
#endif
#if MIPI_HOST_INT_DBG
	MIPI_HOST_PARAM_ADD(irq_cnt),
	MIPI_HOST_PARAM_ADD(irq_debug),
#endif
	NULL,
};

static const struct attribute_group param_attr_group = {
	.name = __stringify(param),
	.attrs = param_attr,
};

/* sysfs for mipi host devices' status */
static ssize_t mipi_host_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	mipi_hdev_t *hdev = dev_get_drvdata(dev);
	mipi_host_t *host = &hdev->host;
	mipi_user_t *user = &hdev->user;
	mipi_host_cfg_t *cfg = &host->cfg;
	mipi_host_snrclk_t *snrclk = &host->snrclk;
	mipi_host_param_t *param = &host->param;
	void __iomem *iomem = host->iomem;
	char *s = buf;
	int i;

#define MH_STA_SHOW(n, fmt, ...) \
		s += sprintf(s, "%-15s: " fmt "\n", #n, \
			## __VA_ARGS__)
#define MH_REG_SHOW(r) \
		s += sprintf(s, "0x%03x: 0x%08x - %s\n", \
			REG_MIPI_HOST_##r, \
			mipi_getreg(iomem + REG_MIPI_HOST_##r), \
			#r)

	if (strcmp(attr->attr.name, "info") == 0) {
		MH_STA_SHOW(port, "%d", hdev->port);
		MH_STA_SHOW(hw_mode, "g%d:%d l%d/%d i%d",
			mipi_host_port_group(hdev), mipi_host_port_index(hdev),
			mipi_host_port_lane(hdev, 0), mipi_host_port_lane(hdev, 1),
			mipi_host_port_ipi(hdev));
		MH_STA_SHOW(mode, "%s", (hdev->is_ex) ? "group_ext" :
			((hdev->ex_hdev) ? "group_mst" : "alone"));
		MH_STA_SHOW(lane_mode, "%d(%dlane)", hdev->lane_mode,
			mipi_host_port_lane(hdev, hdev->lane_mode));
		MH_STA_SHOW(iomem, "%p", host->iomem);
#if MIPI_HOST_INT_DBG && defined MIPI_HOST_INT_USE_TIMER
		MH_STA_SHOW(timer, "%s",
			(hdev->irq_timer_en) ? "enable" : "disable");
#else
		MH_STA_SHOW(irq, "%d", host->irq);
#endif
		MH_STA_SHOW(state, "%d(%s)", host->state,
			g_mh_state[host->state]);
		MH_STA_SHOW(ipi_clock, "%llu", hdev->ipi_clock);
	} else if (strcmp(attr->attr.name, "cfg") == 0) {
		if (host->state >= MIPI_STATE_INIT) {
			MH_STA_SHOW(lane, "%d", cfg->lane);
			MH_STA_SHOW(datatype, "0x%02x", cfg->datatype);
			MH_STA_SHOW(fps, "%d", cfg->fps);
			MH_STA_SHOW(mclk, "%d MHz", cfg->mclk);
			MH_STA_SHOW(mipiclk, "%d Mbps", cfg->mipiclk);
			MH_STA_SHOW(width, "%d", cfg->width);
			MH_STA_SHOW(height, "%d", cfg->height);
			MH_STA_SHOW(linelenth, "%d", cfg->linelenth);
			MH_STA_SHOW(framelenth, "%d", cfg->framelenth);
			MH_STA_SHOW(settle, "%d", cfg->settle);
			MH_STA_SHOW(hsaTime, "%d", cfg->hsaTime);
			MH_STA_SHOW(hbpTime, "%d", cfg->hbpTime);
			MH_STA_SHOW(hsdTime, "%d", cfg->hsdTime);
			MH_STA_SHOW(channel_num, "%d", cfg->channel_num);
			for (i = 0; i < MIPIHOST_CHANNEL_NUM; i++)
				s += sprintf(s, "channel_sel[%d] : %d\n",
						i, cfg->channel_sel[i]);
		} else {
			s += sprintf(s, "not inited\n" );
		}
	} else if (strcmp(attr->attr.name, "regs") == 0) {
		if (iomem) {
			MH_REG_SHOW(VERSION);
			MH_REG_SHOW(N_LANES);
			MH_REG_SHOW(CSI2_RESETN);
			MH_REG_SHOW(PHY_SHUTDOWNZ);
			MH_REG_SHOW(DPHY_RSTZ);
			MH_REG_SHOW(PHY_RX);
			MH_REG_SHOW(PHY_STOPSTATE);
			MH_REG_SHOW(VC_EXTENSION);
			MH_REG_SHOW(PHY_CAL);
			MH_REG_SHOW(INT_ST_MAIN);
			MH_REG_SHOW(INT_ST_PHY_FATAL);
			MH_REG_SHOW(INT_ST_PKT_FATAL);
			MH_REG_SHOW(INT_ST_PHY);
			MH_REG_SHOW(INT_ST_LINE);
			MH_REG_SHOW(INT_ST_IPI);
			MH_REG_SHOW(INT_ST_IPI2);
			MH_REG_SHOW(INT_ST_IPI3);
			MH_REG_SHOW(INT_ST_IPI4);
			if (MIPI_HOST_IS_1P4(iomem)) {
				MH_REG_SHOW(INT_ST_BNDRY_FRAME_FATAL);
				MH_REG_SHOW(INT_ST_SEQ_FRAME_FATAL);
				MH_REG_SHOW(INT_ST_CRC_FRAME_FATAL);
				MH_REG_SHOW(INT_ST_PLD_CRC_FATAL);
				MH_REG_SHOW(INT_ST_DATA_ID);
				MH_REG_SHOW(INT_ST_ECC_CORRECT);
			} else {
				MH_REG_SHOW(INT_ST_FRAME_FATAL);
				MH_REG_SHOW(INT_ST_PKT);
			}
			MH_REG_SHOW(INT_MSK_PHY_FATAL);
			MH_REG_SHOW(INT_MSK_PKT_FATAL);
			MH_REG_SHOW(INT_MSK_PHY);
			MH_REG_SHOW(INT_MSK_LINE);
			MH_REG_SHOW(INT_MSK_IPI);
			MH_REG_SHOW(INT_MSK_IPI2);
			MH_REG_SHOW(INT_MSK_IPI3);
			MH_REG_SHOW(INT_MSK_IPI4);
			if (MIPI_HOST_IS_1P4(iomem)) {
				MH_REG_SHOW(INT_MSK_BNDRY_FRAME_FATAL);
				MH_REG_SHOW(INT_MSK_SEQ_FRAME_FATAL);
				MH_REG_SHOW(INT_MSK_CRC_FRAME_FATAL);
				MH_REG_SHOW(INT_MSK_PLD_CRC_FATAL);
				MH_REG_SHOW(INT_MSK_DATA_ID);
				MH_REG_SHOW(INT_MSK_ECC_CORRECT);
			} else {
				MH_REG_SHOW(INT_MSK_FRAME_FATAL);
				MH_REG_SHOW(INT_MSK_PKT);
			}

			MH_REG_SHOW(IPI_MODE);
			MH_REG_SHOW(IPI_VCID);
			MH_REG_SHOW(IPI_DATA_TYPE);
			MH_REG_SHOW(IPI_MEM_FLUSH);
			MH_REG_SHOW(IPI_HSA_TIME);
			MH_REG_SHOW(IPI_HBP_TIME);
			MH_REG_SHOW(IPI_HSD_TIME);
			MH_REG_SHOW(IPI_HLINE_TIME);
			MH_REG_SHOW(IPI_ADV_FEATURES);
			MH_REG_SHOW(IPI_VSA_LINES);
			MH_REG_SHOW(IPI_VBP_LINES);
			MH_REG_SHOW(IPI_VFP_LINES);
			MH_REG_SHOW(IPI_VACTIVE_LINES);

			if (cfg->channel_num > 1) {
				MH_REG_SHOW(IPI2_MODE);
				MH_REG_SHOW(IPI2_VCID);
				MH_REG_SHOW(IPI2_DATA_TYPE);
				MH_REG_SHOW(IPI2_MEM_FLUSH);
				MH_REG_SHOW(IPI2_HSA_TIME);
				MH_REG_SHOW(IPI2_HBP_TIME);
				MH_REG_SHOW(IPI2_HSD_TIME);
				MH_REG_SHOW(IPI2_ADV_FEATURES);
			}
			if (cfg->channel_num > 2) {
				MH_REG_SHOW(IPI3_MODE);
				MH_REG_SHOW(IPI3_VCID);
				MH_REG_SHOW(IPI3_DATA_TYPE);
				MH_REG_SHOW(IPI3_MEM_FLUSH);
				MH_REG_SHOW(IPI3_HSA_TIME);
				MH_REG_SHOW(IPI3_HBP_TIME);
				MH_REG_SHOW(IPI3_HSD_TIME);
				MH_REG_SHOW(IPI3_ADV_FEATURES);
			}
			if (cfg->channel_num > 3) {
				MH_REG_SHOW(IPI4_MODE);
				MH_REG_SHOW(IPI4_VCID);
				MH_REG_SHOW(IPI4_DATA_TYPE);
				MH_REG_SHOW(IPI4_MEM_FLUSH);
				MH_REG_SHOW(IPI4_HSA_TIME);
				MH_REG_SHOW(IPI4_HBP_TIME);
				MH_REG_SHOW(IPI4_HSD_TIME);
				MH_REG_SHOW(IPI4_ADV_FEATURES);
			}
		} else {
			s += sprintf(s, "not ioremap\n" );
		}
	} else if (strcmp(attr->attr.name, "snrclk") == 0) {
		if (snrclk->index >= 0)
			MH_STA_SHOW(snrclk, "%s", g_mh_snrclk_name[snrclk->index]);
		else
			MH_STA_SHOW(snrclk, "%s", "none");
		if (snrclk->pinctrl)
			MH_STA_SHOW(support, "%s %s",
				(snrclk->enable) ? "enable" : "",
				(snrclk->disable) ? "disable" : "");
		else
			MH_STA_SHOW(support, "%s", "none");
		MH_STA_SHOW(state, "%s",
			(param->snrclk_en == MIPI_HOST_SNRCLK_NOUSED) ? "noused" :
			((param->snrclk_en == MIPI_HOST_SNRCLK_ENABLE) ? "enable" : "disable"));
		MH_STA_SHOW(freq, "%u", param->snrclk_freq);
		MH_STA_SHOW(cnt, "%u(sys) / %u(en)", snrclk->sys_cnt, snrclk->en_cnt);
	} else if (strcmp(attr->attr.name, "user") == 0) {
		MH_STA_SHOW(user, "%d", user->open_cnt);
		MH_STA_SHOW(init, "%d", user->init_cnt);
		MH_STA_SHOW(start, "%d", user->start_cnt);
		MH_STA_SHOW(pre_state, "%s", g_mh_pre_state[user->pre_state]);
#if MIPI_HOST_INT_DBG
	} else if (strcmp(attr->attr.name, "icnt") == 0) {
		for (i = 0; i < sizeof(host->icnt)/sizeof(uint32_t); i++) {
			s += sprintf(s, "%-15s: %u\n", g_mh_icnt_names[i],
						 ((uint32_t *)(&host->icnt))[i]);
		}
#endif
	}

	return (s - buf);
}

#define MIPI_HOST_STATUS_DEC(a) \
	MIPI_HOST_ATTR_DEC(a, 0444, mipi_host_status_show, NULL)
#define MIPI_HOST_STATUS_ADD(a) \
	MIPI_HOST_ATTR_ADD(a)

MIPI_HOST_STATUS_DEC(info);
MIPI_HOST_STATUS_DEC(cfg);
MIPI_HOST_STATUS_DEC(regs);
MIPI_HOST_STATUS_DEC(snrclk);
MIPI_HOST_STATUS_DEC(user);
#if MIPI_HOST_INT_DBG
MIPI_HOST_STATUS_DEC(icnt);
#endif

static struct attribute *status_attr[] = {
	MIPI_HOST_STATUS_ADD(info),
	MIPI_HOST_STATUS_ADD(cfg),
	MIPI_HOST_STATUS_ADD(regs),
	MIPI_HOST_STATUS_ADD(snrclk),
	MIPI_HOST_STATUS_ADD(user),
#if MIPI_HOST_INT_DBG
	MIPI_HOST_STATUS_ADD(icnt),
#endif
	NULL,
};

static const struct attribute_group status_attr_group = {
	.name = __stringify(status),
	.attrs = status_attr,
};

#if MIPI_HOST_INT_DBG && MIPI_HOST_SYSFS_FATAL_EN
/* sysfs for mipi host devices' fatal */
static const mipi_host_ireg_t* mipi_host_get_ireg(mipi_host_ierr_t *ierr, const char *name)
{
	const mipi_host_ireg_t *ireg = NULL, *itmp = NULL;
	int i;

	for (i = 0; i < ierr->num; i++) {
		itmp = &ierr->iregs[i];
		if (itmp->icnt_n < ARRAY_SIZE(g_mh_icnt_names) &&
			strcmp(g_mh_icnt_names[itmp->icnt_n], name) == 0) {
			ireg = itmp;
			break;
		}
	}

	return ireg;
}

static ssize_t mipi_host_fatal_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	mipi_hdev_t *hdev = dev_get_drvdata(dev);
	mipi_host_ierr_t *ierr = &hdev->host.ierr;
	const mipi_host_ireg_t *ireg = NULL;
	void __iomem *iomem = hdev->host.iomem;
	char *s = buf;
	uint32_t reg = 0;

	if (strcmp(g_mh_icnt_names[0], attr->attr.name) == 0) {
		reg = REG_MIPI_HOST_INT_ST_MAIN;
	} else {
		ireg = mipi_host_get_ireg(ierr, attr->attr.name);
		if (ireg)
			reg = ireg->reg_st;
	}
	if (reg > 0)
		s += sprintf(s, "0x%08x\n", mipi_getreg(iomem + reg));

	return (s - buf);
}

static ssize_t mipi_host_fatal_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	mipi_hdev_t *hdev = dev_get_drvdata(dev);
	mipi_host_ierr_t *ierr = &hdev->host.ierr;
	const mipi_host_ireg_t *ireg = NULL;
	void __iomem *iomem = hdev->host.iomem;
	int ret, error = -EINVAL;
	uint32_t val;
	int i;

	ret = kstrtouint(buf, 0, &val);
	if (ret) {
		return error;
	}

	if (strcmp(g_mh_icnt_names[0], attr->attr.name) == 0) {
		for (i = 0; i < ierr->num; i++) {
			ireg = &ierr->iregs[i];
			if (val & ireg->st_mask) {
				mipi_putreg(iomem + ireg->reg_force, ireg->err_mask);
				val &= ~ireg->st_mask;
				if (val == 0)
					break;
			}
		}
		error = 0;
	} else {
		ireg = mipi_host_get_ireg(ierr, attr->attr.name);
		if (ireg) {
			mipi_putreg(iomem + ireg->reg_force, val);
			error = 0;
		}
	}

	return ((error) ? error : count);
}

#define MIPI_HOST_FATAL_DEC(a) \
	MIPI_HOST_ATTR_DEC(a, 0644, \
		mipi_host_fatal_show, mipi_host_fatal_store)
#define MIPI_HOST_FATAL_ADD(a) \
	MIPI_HOST_ATTR_ADD(a)

MIPI_HOST_FATAL_DEC(st_main);
MIPI_HOST_FATAL_DEC(phy_fatal);
MIPI_HOST_FATAL_DEC(pkt_fatal);
MIPI_HOST_FATAL_DEC(frm_fatal);
MIPI_HOST_FATAL_DEC(bndry_frm_fatal);
MIPI_HOST_FATAL_DEC(seq_frm_fatal);
MIPI_HOST_FATAL_DEC(crc_frm_fatal);
MIPI_HOST_FATAL_DEC(pld_crc_fatal);
MIPI_HOST_FATAL_DEC(data_id);
MIPI_HOST_FATAL_DEC(ecc_corrected);
MIPI_HOST_FATAL_DEC(phy);
MIPI_HOST_FATAL_DEC(pkt);
MIPI_HOST_FATAL_DEC(line);
MIPI_HOST_FATAL_DEC(ipi);
MIPI_HOST_FATAL_DEC(ipi2);
MIPI_HOST_FATAL_DEC(ipi3);
MIPI_HOST_FATAL_DEC(ipi4);

static struct attribute *fatal_attr[] = {
	MIPI_HOST_FATAL_ADD(st_main),
	MIPI_HOST_FATAL_ADD(phy_fatal),
	MIPI_HOST_FATAL_ADD(pkt_fatal),
	MIPI_HOST_FATAL_ADD(frm_fatal),
	MIPI_HOST_FATAL_ADD(bndry_frm_fatal),
	MIPI_HOST_FATAL_ADD(seq_frm_fatal),
	MIPI_HOST_FATAL_ADD(crc_frm_fatal),
	MIPI_HOST_FATAL_ADD(pld_crc_fatal),
	MIPI_HOST_FATAL_ADD(data_id),
	MIPI_HOST_FATAL_ADD(ecc_corrected),
	MIPI_HOST_FATAL_ADD(phy),
	MIPI_HOST_FATAL_ADD(pkt),
	MIPI_HOST_FATAL_ADD(line),
	MIPI_HOST_FATAL_ADD(ipi),
	MIPI_HOST_FATAL_ADD(ipi2),
	MIPI_HOST_FATAL_ADD(ipi3),
	MIPI_HOST_FATAL_ADD(ipi4),
	NULL,
};

static const struct attribute_group fatal_attr_group = {
	.name = __stringify(fatal),
	.attrs = fatal_attr,
};
#endif

/* sysfs attr groups */
static const struct attribute_group *attr_groups[] = {
	&param_attr_group,
	&status_attr_group,
#if MIPI_HOST_INT_DBG && MIPI_HOST_SYSFS_FATAL_EN
	&fatal_attr_group,
#endif
	NULL,
};

static int hobot_mipi_host_class_get(void)
{
	int ret = 0;

	if (!g_mh_class) {
#ifdef CONFIG_HOBOT_MIPI_PHY
		g_mh_class = mipi_dphy_class();
		if (!g_mh_class) {
			pr_err("[%s] dphy class null\n", __func__);
			ret = -ENODEV;
		}
#else
		g_mh_class = class_create(THIS_MODULE, MIPI_HOST_DNAME);
		if (IS_ERR(g_mh_class)) {
			ret = (int32_t)PTR_ERR(g_mh_class);
			g_mh_class = NULL;
			pr_err("[%s] class error %d\n", __func__,
					ret);
		}
#endif
	}
	return ret;
}

static int hobot_mipi_host_class_put(void)
{
#ifndef CONFIG_HOBOT_MIPI_PHY
	if (g_mh_class)
		class_destroy(g_mh_class);
#endif
	g_mh_class = NULL;
	return 0;
}

static int hobot_mipi_host_probe_cdev(mipi_hdev_t *hdev)
{
	int ret = 0;
	dev_t devno;
	struct cdev	*p_cdev;

	if (g_mh_major == 0) {
		ret = alloc_chrdev_region(&devno, 0, MIPI_HOST_MAX_NUM, MIPI_HOST_DNAME);
		if (ret < 0) {
			pr_err("[%s] alloc chrdev %s error %d\n", __func__,
					MIPI_HOST_DNAME, ret);
			return ret;
		}
		g_mh_major = MAJOR(devno);
	}
	ret = hobot_mipi_host_class_get();
	if (ret)
		return ret;
	devno = MKDEV(g_mh_major, hdev->port);
	hdev->devno = devno;
	p_cdev = &hdev->cdev;
	cdev_init(p_cdev, &hobot_mipi_host_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		pr_err("[%s] cdev add error %d\n", __func__, ret);
		goto err_add;
	}
	hdev->dev = device_create_with_groups(g_mh_class, NULL, devno,
			(void *)hdev, attr_groups,
			"%s%d", MIPI_HOST_DNAME, hdev->port);
	if (IS_ERR(hdev->dev)) {
		ret = (int32_t)PTR_ERR(hdev->dev);
		hdev->dev = NULL;
		pr_err("[%s] deivce create error %d\n", __func__, ret);
		goto err_creat;
	}

#ifdef CONFIG_HOBOT_DIAG
	/* diag */
	if (diag_register(ModuleDiag_VIO, (uint16_t)(EventIdVioMipiHost0Err + hdev->port),
			4, DIAG_MSG_INTERVAL_MIN, DIAG_MSG_INTERVAL_MAX,
			mipi_host_diag_test) < 0) {
		pr_err("mipi host %d diag register fail\n", hdev->port);
	} else {
		hdev->last_err_tm_ms = 0;
		init_timer(&hdev->diag_timer);
		hdev->diag_timer.expires =
			get_jiffies_64() + msecs_to_jiffies(1000);
		hdev->diag_timer.data = (unsigned long)hdev;
		hdev->diag_timer.function = mipi_host_diag_timer_func;
		add_timer(&hdev->diag_timer);
	}
#endif
	mutex_init(&hdev->user.open_mutex);
	hdev->user.open_cnt = 0;

	return 0;
err_creat:
	cdev_del(&hdev->cdev);
err_add:
	return ret;
}

static int hobot_mipi_host_remove_cdev(mipi_hdev_t *hdev)
{
#ifdef CONFIG_HOBOT_DIAG
	del_timer_sync(&hdev->diag_timer);
#endif
	if (g_mh_class)
		device_destroy(g_mh_class, hdev->devno);
	cdev_del(&hdev->cdev);

	if (port_num <= 1) {
		hobot_mipi_host_class_put();
		if (g_mh_major) {
			unregister_chrdev_region(MKDEV(g_mh_major, 0),
				MIPI_HOST_MAX_NUM);
			g_mh_major = 0;
		}
	}
	hdev->dev = NULL;
	return 0;
}

static int hobot_mipi_host_phy_register(mipi_hdev_t *hdev)
{
	int ret = 0;

#ifdef CONFIG_HOBOT_MIPI_PHY
	mipi_phy_sub_t sub;

	sub.iomem = hdev->host.iomem;
	sub.dev = hdev->dev;
	sub.param = NULL;
	sub.port = hdev->port;
	ret = mipi_dphy_register(MIPI_DPHY_TYPE_HOST, hdev->port, &sub);
#else
	struct device *dev = hdev->dev;

	mipiinfo("no dphy driver");
#endif

	return ret;
}

static int hobot_mipi_host_phy_unregister(mipi_hdev_t *hdev)
{
#ifdef CONFIG_HOBOT_MIPI_PHY
	return mipi_dphy_unregister(0, hdev->port);
#else
	return 0;
#endif
}

#ifdef MODULE
static int hobot_mipi_host_remove_param(void)
{
	mipi_hdev_t *hdev;
	mipi_host_t *host;
	int i;

	for (i = 0; i < MIPI_HOST_MAX_NUM; i ++) {
		hdev = g_hdev[i];
		if (!hdev) {
			continue;
		}
		host = &hdev->host;

#if MIPI_HOST_INT_DBG && defined MIPI_HOST_INT_USE_TIMER
		del_timer_sync(&hdev->irq_timer);
#endif
		hobot_mipi_host_remove_cdev(hdev);
		if (host->iomem)
			iounmap(host->iomem);
		hobot_mipi_host_phy_unregister(hdev);
		kfree(hdev);
		g_hdev[i] = NULL;
		port_num--;
	}
	return 0;
}

static int hobot_mipi_host_probe_param(void)
{
	mipi_hdev_t *hdev;
	mipi_host_t *host;
	mipi_host_param_t *param;
	unsigned long reg_addr_dev;
	int i, group, index;
	int ret = 0;

	if (init_num <= 0)
		return -ENODEV;

	if (init_num > MIPI_HOST_MAX_NUM)
		init_num = MIPI_HOST_MAX_NUM;

	if (hw_mode > ARRAY_SIZE(g_mh_hw_modes))
		hw_mode = MIPI_HOST_HW_MODE_DEF;

	port_num = 0;
	for (i = 0; i < init_num; i++) {
		hdev = kmalloc(sizeof(mipi_hdev_t), GFP_KERNEL);
		if (hdev == NULL) {
		pr_err("[%s] malloc failed\n", __func__);
			ret = -ENOMEM;
			goto err_kmalloc;
		}
		memset(hdev, 0x0, sizeof(mipi_hdev_t));
		hdev->port = i;
		hdev->hw_mode = &g_mh_hw_modes[hw_mode];
		host = &hdev->host;
		param = &host->param;
		group = mipi_host_port_group(hdev);
		index = mipi_host_port_index(hdev);
		reg_addr_dev = reg_addr + (reg_size * 2) * group + (reg_size * index);
		host->iomem = ioremap_nocache(reg_addr_dev, reg_size);
		if (IS_ERR(host->iomem)) {
			pr_err("[%s] ioremap error\n", __func__);
			ret = (int32_t)PTR_ERR(host->iomem);
			host->iomem = NULL;
			goto err_ioremap;
		}
		hdev->lane_mode = mipi_dphy_get_lanemode(MIPI_DPHY_TYPE_HOST,
								hdev->port);
		ret = hobot_mipi_host_probe_cdev(hdev);
		if (ret) {
			goto err_cdev;
		}

#if MIPI_HOST_INT_DBG && defined MIPI_HOST_INT_USE_TIMER
		init_timer(&hdev->irq_timer);
		hdev->irq_timer.data = (unsigned long)hdev;
		hdev->irq_timer.function = mipi_host_irq_timer_func;
		add_timer(&hdev->irq_timer);
		param->irq_cnt = MIPI_HOST_IRQ_CNT;
		param->irq_debug = MIPI_HOST_IRQ_DEBUG;
		if (MIPI_HOST_IS_1P4(host->iomem)) {
			host->ierr.iregs = mh_int_regs_1p4;
			host->ierr.num = ARRAY_SIZE(mh_int_regs_1p4);
		} else {
			host->ierr.iregs = mh_int_regs_1p3;
			host->ierr.num = ARRAY_SIZE(mh_int_regs_1p3);
		}
#else
		pr_info("[%s] no int timer\n", __func__);
#endif
		param->adv_value = MIPI_HOST_ADV_DEFAULT;
		param->cut_through = MIPI_HOST_CUT_DEFAULT;
		param->ipi_limit = MIPI_HOST_IPILIMIT_DEFAULT;
		param->wait_ms = HOST_DPHY_CHECK_MAX;
		param->sigfun_cfg = MIPI_HOST_SIGFUNCFG_DEFAULT;
		param->sigwait_ms = MIPI_HOST_SIGWAITMS_DEFAULT;

		hobot_mipi_host_phy_register(hdev);
		g_hdev[i] = hdev;
		port_num ++;

		spin_lock_init(&host->reglock);
		ret = mipi_getreg(host->iomem + REG_MIPI_HOST_VERSION);
		dev_info(hdev->dev, "ver %c%c%c%c port%d(%d:%d)\n",
			(ret >> 24), (ret >> 16), (ret >> 8), ret,
			i, mipi_host_port_group(hdev), mipi_host_port_index(hdev));
	}
	return 0;
err_cdev:
	if (host && host->iomem)
		iounmap(host->iomem);
err_ioremap:
	if (hdev)
		kfree(hdev);
err_kmalloc:
	hobot_mipi_host_remove_param();
	return ret;
}

#else
static int hobot_mipi_host_remove(struct platform_device *pdev)
{
	mipi_hdev_t *hdev = platform_get_drvdata(pdev);
	mipi_host_t *host = &hdev->host;

	hobot_mipi_host_remove_cdev(hdev);
#if MIPI_HOST_INT_DBG
#ifdef MIPI_HOST_INT_USE_TIMER
	del_timer_sync(&hdev->irq_timer);
#else
	free_irq(host->irq, hdev);
#endif
#endif
	devm_iounmap(&pdev->dev, host->iomem);
	hobot_mipi_host_phy_unregister(hdev);
	devm_kfree(&pdev->dev, hdev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static int hobot_mipi_host_probe(struct platform_device *pdev)
{
	mipi_hdev_t *hdev;
	mipi_host_t *host;
	mipi_host_param_t *param;
	int port, ret = 0;
	uint32_t node_val;
	struct resource *res;

	port = of_alias_get_id(pdev->dev.of_node, "mipihost");
	if (port >= MIPI_HOST_MAX_NUM) {
		pr_err("[%s] port %d >= %d overflow error\n", __func__, port,
				MIPI_HOST_MAX_NUM);
		return -ERANGE;
	} else {
		if (port < 0)
			port = 0;
		if (g_hdev[port]) {
			pr_err("[%s] port %d duplicate error\n", __func__, port);
			return -EEXIST;
		}
	}
	hdev = devm_kmalloc(&pdev->dev, sizeof(mipi_hdev_t), GFP_KERNEL);
	if (hdev == NULL) {
		pr_err("[%s] malloc failed\n", __func__);
		return -ENOMEM;
	}
	memset(hdev, 0x0, sizeof(mipi_hdev_t));
	host = &hdev->host;
	param = &host->param;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->iomem = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->iomem)) {
		pr_err("[%s] get mem res error\n", __func__);
		ret = (int32_t)PTR_ERR(host->iomem);
		host->iomem = NULL;
		goto err_ioremap;
	}

#if MIPI_HOST_INT_DBG
#ifdef MIPI_HOST_INT_USE_TIMER
	init_timer(&hdev->irq_timer);
	hdev->irq_timer.data = (unsigned long)hdev;
	hdev->irq_timer.function = mipi_host_irq_timer_func;
	add_timer(&hdev->irq_timer);
#else
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		pr_err("[%s] get irq res error\n", __func__);
		ret = -ENODEV;
		goto err_irq;
	}
	host->irq = (uint32_t)res->start;
	ret = request_threaded_irq(host->irq,
							   mipi_host_irq_func,
							   NULL,
							   IRQF_TRIGGER_HIGH,
							   dev_name(&pdev->dev),
							   hdev);
	if (ret) {
		pr_err("[%s] request irq error %d\n", __func__, ret);
		goto err_irq;
	}
#endif
	param->irq_cnt = MIPI_HOST_IRQ_CNT;
	param->irq_debug = MIPI_HOST_IRQ_DEBUG;
	if (MIPI_HOST_IS_1P4(host->iomem)) {
		host->ierr.iregs = mh_int_regs_1p4;
		host->ierr.num = ARRAY_SIZE(mh_int_regs_1p4);
	} else {
		host->ierr.iregs = mh_int_regs_1p3;
		host->ierr.num = ARRAY_SIZE(mh_int_regs_1p3);
	}
#endif

	param->adv_value = MIPI_HOST_ADV_DEFAULT;
	param->cut_through = MIPI_HOST_CUT_DEFAULT;
	param->ipi_limit = MIPI_HOST_IPILIMIT_DEFAULT;
	param->wait_ms = HOST_DPHY_CHECK_MAX;
	param->sigfun_cfg = MIPI_HOST_SIGFUNCFG_DEFAULT;
	param->sigwait_ms = MIPI_HOST_SIGWAITMS_DEFAULT;

	platform_set_drvdata(pdev, hdev);

	hdev->port = port;
	hdev->lane_mode = mipi_dphy_get_lanemode(MIPI_DPHY_TYPE_HOST,
							hdev->port);
	ret = of_property_read_u32(pdev->dev.of_node,
			"hw-mode", &node_val);
	if (ret || node_val >= ARRAY_SIZE(g_mh_hw_modes)) {
		node_val = MIPI_HOST_HW_MODE_DEF;
	}
	hdev->hw_mode = &g_mh_hw_modes[node_val];

	ret = hobot_mipi_host_probe_cdev(hdev);
	if (ret) {
		goto err_cdev;
	}

	host->snrclk.pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(host->snrclk.pinctrl)) {
		host->snrclk.pinctrl = NULL;
	} else {
		host->snrclk.enable = pinctrl_lookup_state(host->snrclk.pinctrl,
			"enable");
		if (IS_ERR(host->snrclk.enable))
			host->snrclk.enable = NULL;
		host->snrclk.disable = pinctrl_lookup_state(host->snrclk.pinctrl,
			"disable");
		if (IS_ERR(host->snrclk.disable))
			host->snrclk.disable = NULL;
	}
	host->param.snrclk_en = MIPI_HOST_SNRCLK_NOUSED;
	ret = of_property_read_u32(pdev->dev.of_node,
			"snrclk-idx", &node_val);
	if (ret || node_val >= ARRAY_SIZE(g_mh_snrclk_name)) {
		host->snrclk.index = -1;
	} else {
		host->snrclk.index = (int)node_val;
	}
	ret = of_property_read_u32(pdev->dev.of_node,
			"clock-frequency", &node_val);
	if (ret == 0) {
		if (node_val <= 1) {
			mipi_host_snrclk_set_en(hdev, node_val);
			if (host->snrclk.index >= 0)
				host->param.snrclk_freq = (uint32_t)(mipi_host_get_clk(hdev,
					g_mh_snrclk_name[host->snrclk.index]));
		} else {
			mipi_host_snrclk_set_freq(hdev, node_val);
			mipi_host_snrclk_set_en(hdev, 1);
		}
	} else {
		if (host->snrclk.index >= 0)
			host->param.snrclk_freq = (uint32_t)(mipi_host_get_clk(hdev,
						g_mh_snrclk_name[host->snrclk.index]));
	}

	hobot_mipi_host_phy_register(hdev);
	g_hdev[port] = hdev;
	port_num ++;

	spin_lock_init(&host->reglock);
	ret = mipi_getreg(host->iomem + REG_MIPI_HOST_VERSION);
	dev_info(hdev->dev, "ver %c%c%c%c port%d(%d:%d)\n",
		(ret >> 24), (ret >> 16), (ret >> 8), ret,
		port, mipi_host_port_group(hdev), mipi_host_port_index(hdev));
	return 0;
err_cdev:
#if MIPI_HOST_INT_DBG
#ifdef MIPI_HOST_INT_USE_TIMER
	del_timer_sync(&hdev->irq_timer);
#else
	free_irq(host->irq, hdev);
err_irq:
#endif
#endif
	devm_iounmap(&pdev->dev, host->iomem);
err_ioremap:
	devm_kfree(&pdev->dev, hdev);
	platform_set_drvdata(pdev, NULL);
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int hobot_mipi_host_suspend(struct device *dev)
{
	mipi_hdev_t *hdev = dev_get_drvdata(dev);

	if (pm_suspend_target_state == PM_SUSPEND_TO_IDLE)
		return 0;

	mipiinfo("%s:%s enter suspend...", __FILE__, __func__);

	mipi_host_stop(hdev);
	mipi_host_deinit(hdev);

	return 0;
}

static int hobot_mipi_host_resume(struct device *dev)
{
	mipi_hdev_t *hdev = dev_get_drvdata(dev);
	mipi_host_t *host = &hdev->host;
	int ret = 0;

	if (pm_suspend_target_state == PM_SUSPEND_TO_IDLE)
		return 0;

	mipiinfo("%s:%s enter resume...", __FILE__, __func__);

	if (host->state == MIPI_STATE_DEFAULT) {
		mipi_host_stop(hdev);
		mipi_host_deinit(hdev);
	} else if (host->state == MIPI_STATE_START) {
		/* if state == MIPI_STATE_START, it has been initialized. */
		if (0 != (ret = mipi_host_init(hdev, &host->cfg))) {
			mipierr("[%s] init error %d", __func__, ret);
			return ret;
		}

		/* start again */
		if (0 != (ret = mipi_host_start(hdev))) {
			mipierr("[%s] start error %d", __func__, ret);
			return ret;
		}
	} else if (host->state == MIPI_STATE_STOP) {
		/* if state == MIPI_STATE_STOP, it has been initialized. */
		if (0 != (ret = mipi_host_init(hdev, &host->cfg))) {
			mipierr("[%s] init error %d", __func__, ret);
			return ret;
		}
		mipi_host_stop(hdev);
	} else if (host->state == MIPI_STATE_INIT) {
		if (0 != (ret = mipi_host_init(hdev, &host->cfg))) {
			mipierr("[%s] init error %d", __func__, ret);
			return ret;
		}
	}

	return ret;
}

static const struct dev_pm_ops hobot_mipi_host_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hobot_mipi_host_suspend,
			hobot_mipi_host_resume)
};
#endif

static const struct of_device_id hobot_mipi_host_match[] = {
	{.compatible = "hobot,mipi-host"},
	{}
};

MODULE_DEVICE_TABLE(of, hobot_mipi_host_match);

static struct platform_driver hobot_mipi_host_driver = {
	.probe	= hobot_mipi_host_probe,
	.remove = hobot_mipi_host_remove,
	.driver = {
		.name = MIPI_HOST_DNAME,
		.of_match_table = hobot_mipi_host_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &hobot_mipi_host_dev_pm_ops,
#endif
	},
};
#endif

static int __init hobot_mipi_host_module_init(void)
{
	int           ret = 0;

#ifdef MODULE
	ret = hobot_mipi_host_probe_param();
#else
	ret = platform_driver_register(&hobot_mipi_host_driver);
#endif

	return ret;
}

static void __exit hobot_mipi_host_module_exit(void)
{
#ifdef MODULE
	hobot_mipi_host_remove_param();
#else
	platform_driver_unregister(&hobot_mipi_host_driver);
#endif
}

late_initcall_sync(hobot_mipi_host_module_init);
module_exit(hobot_mipi_host_module_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Zhang Tianyu <tianyu.zhang@hobot.cc>");
MODULE_DESCRIPTION("HOBOT MIPI Host Driver");
