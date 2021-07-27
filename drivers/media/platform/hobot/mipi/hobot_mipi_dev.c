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

#include <soc/hobot/hobot_mipi_dev.h>
#include <soc/hobot/hobot_mipi_dphy.h>
#ifdef CONFIG_HOBOT_DIAG
#include <soc/hobot/diag.h>
#endif

#include "hobot_mipi_dev_regs.h"
#include "hobot_mipi_utils.h"

#define MIPI_DEV_DNAME		"mipi_dev"
#define MIPI_DEV_MAX_NUM	CONFIG_HOBOT_MIPI_DEV_MAX_NUM
#define MIPI_DEV_CFGCLK_NAME	"mipi_cfg_host"
#define MIPI_DEV_CFGCLK_MHZ		24000000UL
#define MIPI_DEV_REFCLK_NAME	"mipi_dev_ref"
#define MIPI_DEV_REFCLK_MHZ		24000000UL
#define MIPI_DEV_IPICLK_NAME(x)	"mipi_tx_ipi"

static unsigned int port_num;
module_param(port_num, uint, 0444);

#ifdef MODULE
/* driver as ko without dts platform */
#define CONFIG_HOBOT_MIPI_PHY
#ifdef CONFIG_ARCH_HOBOT
#define MIPI_DEV_REG_ADDR	(0xA4354000)
#else
#define MIPI_DEV_REG_ADDR	(0xB0000000)
#endif
#define MIPI_DEV_REG_SIZE 0x00001000

static unsigned int reg_addr = MIPI_DEV_REG_ADDR;
static unsigned int reg_size = MIPI_DEV_REG_SIZE;
static unsigned int init_num = MIPI_DEV_MAX_NUM;
module_param(reg_addr, uint, 0644);
module_param(reg_size, uint, 0644);
module_param(init_num, uint, 0644);
#endif

/* only hobot platform driver use irq */
#if defined MODULE || !defined CONFIG_ARCH_HOBOT
#define MIPI_DEV_INT_USE_TIMER
#endif

#define MIPI_DEV_INT_DBG			(1)
#define MIPI_DEV_INT_DBG_ERRSTR		(1)
#define MIPI_DEV_SYSFS_FATAL_EN     (0)

#define MIPI_DEV_CSI2_RAISE			(0x01)
#define MIPI_DEV_CSI2_RESETN		(0x00)
#define MIPI_DEV_CLKMGR_RAISE		(0x09)
#define MIPI_DEV_LPCLK_CONT			(0x01)
#define MIPI_DEV_LPCLK_NCONT		(0x00)
#define MIPI_DEV_LPCLK_CTRL			(MIPI_DEV_LPCLK_CONT)

#define MIPI_DEV_INT_VPG			(0x1)
#define MIPI_DEV_INT_IDI			(0x1<<1)
#define MIPI_DEV_INT_IPI			(0x1<<2)
#define MIPI_DEV_INT_PHY			(0x1<<3)
#define MIPI_DEV_INT_MT_IPI			(0x1<<4)
#define MIPI_DEV_INT_IDI_VCX_DMY	(0x1<<5)
#define MIPI_DEV_INT_IDI_VCX2_DMY	(0x1<<6)

#define MIPI_DEV_IPI_PKT_CFG		(0xa00)
#define MIPI_DEV_IPI_HSYNC_PKT_EN	(0x01 << 8)
#define MIPI_DEV_IPI_VC(x)			(((((x) / 4) & 0x7) << 12) | (((x) % 4) << 6))
#define MIPI_DEV_IPI_MAX_FRAME		(0xffff)

#define MIPI_DEV_VPG_DISABLE		(0x00)
#define MIPI_DEV_VPG_ENABLE			(0x01)
#define MIPI_DEV_VPG_ORI_VERT		(0)
#define MIPI_DEV_VPG_ORI_HOR		(1)
#define MIPI_DEV_VPG_ORI(ori)		((ori)<< 16)
#define MIPI_DEV_VPG_MODE_BER		(0x01)
#define MIPI_DEV_VPG_MODE_BAR		(0x00)
#define MIPI_DEV_VPG_MODE(mode)		((mode)<< 0)
#define MIPI_DEV_VPG_LINENUM		(0x00 << 9)
#define MIPI_DEV_VPG_HSYNC_DIS		(0x00)
#define MIPI_DEV_VPG_HSYNC_EN		(0x01)
#define MIPI_DEV_VPG_HSYNC(hsync)	((hsync)<< 8)
#define MIPI_DEV_VPG_VCX(vc)		(((vc) & 0x1C) << 10)
#define MIPI_DEV_VPG_VC(vc)			(((vc) & 0x3) << 6)
#define MIPI_DEV_VPG_HSA_TIME		(0x4e)
#define MIPI_DEV_VPG_HBP_TIME		(0x04)
#define MIPI_DEV_VPG_HFP_TIME		(0x5f4)
#define MIPI_DEV_VPG_HLINE_TIME		(0x1000)
#define MIPI_DEV_VPG_VSA_LINES		(0x02)
#define MIPI_DEV_VPG_VBP_LINES		(0x01)
#define MIPI_DEV_VPG_VFP_LINES		(0x01)
#define MIPI_DEV_VPG_VFP_LINES_MAX	((0x01 << 10) - 1)
#define MIPI_DEV_VPG_BK_LINES		(0x00)
#define MIPI_DEV_VPG_BK_LINES_MAX	((0x01 << 10) - 1)
#define MIPI_DEV_VPG_START_LINE		(0x00)
#define MIPI_DEV_VPG_STEP_LINE		(0x00)
#define MIPI_DEV_CHECK_MAX			(500)

#define MIPI_DEV_CFGCLK_DEFAULT		(0x1C)
#define MIPI_DEV_VPG_DEF_MCLK		(24)
#define MIPI_DEV_VPG_DEF_PCLK		(384)
#define MIPI_DEV_VPG_DEF_SETTLE		(0x30)
#define MIPI_DEV_VPG_DEF_FPS		(30)
#define MIPI_DEV_VPG_DEF_BLANK		(0x5f4)

#define MIPI_DEV_HSYNC_PKT_DEFAULT  (1)
#define MIPI_DEV_IPILIMIT_DEFAULT   (102000000UL)
#define MIPI_DEV_IRQ_CNT            (10)
#define MIPI_DEV_IRQ_DEBUG_PRERR    (0x1)
#define MIPI_DEV_IRQ_DEBUG_ERRSTR   (0x2)
#define MIPI_DEV_IRQ_DEBUG          (0x1)
#define MIPI_DEV_INIT_RETRY_DEFAULT (3)

#define DEV_DPHY_LANE_MAX			(4)
#define DEV_DPHY_CHECK_MAX			(500)
#define DEV_DPHY_STATE_BASIC		(0x3F)
#define DEV_DPHY_STATE_NLANE		(0x38|(3)) /*b'(01 01 01 01) 00 1xxx*/ /*max lane num is 4 now*/
#define DEV_DPHY_STATE(s)			((s>>6)) /*b'00 1xxx*/
#define DEV_DPHY_STATE_STOP(l)		(0xFF>>((DEV_DPHY_LANE_MAX-l)<<1)) /*b'11 11 11 11 (00 1xxx)*/
#define DEV_DPHY_SHUTDOWNZ			(0x01)
#define DEV_DPHY_RSTZ				(0x02)
#define DEV_DPHY_ENABLEZ			(0x04)
#define DEV_DPHY_FORCEPOLL			(0x08)
#define DEV_DPHY_CAL_EN_MHZ			(1500)
#define DEV_DPHY_CAL_DISABLE		(0x0)
#define DEV_DPHY_CAL_ENABLE			(0x1)
#define DEV_DPHY_CAL_DELAY_US		(50)

#define MIPI_CSI2_DT_YUV420_8	(0x18)
#define MIPI_CSI2_DT_YUV420_10	(0x19)
#define MIPI_CSI2_DT_YUV422_8	(0x1E)
#define MIPI_CSI2_DT_YUV422_10	(0x1F)
#define MIPI_CSI2_DT_RGB565		(0x22)
#define MIPI_CSI2_DT_RGB888		(0x24)
#define MIPI_CSI2_DT_RAW_8		(0x2A)
#define MIPI_CSI2_DT_RAW_10		(0x2B)
#define MIPI_CSI2_DT_RAW_12		(0x2C)
#define MIPI_CSI2_DT_RAW_14		(0x2D)

#ifdef CONFIG_HOBOT_MIPI_REG_OPERATE
typedef struct _reg_s {
	uint32_t offset;
	uint32_t value;
} reg_t;

#define MIPIDEVIOC_READ		   _IOWR(MIPIDEVIOC_MAGIC, 16, reg_t)
#define MIPIDEVIOC_WRITE	   _IOW(MIPIDEVIOC_MAGIC, 17, reg_t)
#endif

typedef enum _mipi_state_e {
	MIPI_STATE_DEFAULT = 0,
	MIPI_STATE_INIT,
	MIPI_STATE_START,
	MIPI_STATE_STOP,
	MIPI_STATE_MAX,
} mipi_state_t;

static const char *g_md_state[MIPI_STATE_MAX] = {
	"default",
	"init",
	"start",
	"stop",
};

static const char *g_md_ipiclk_name[] = {
	MIPI_DEV_IPICLK_NAME(0),
};

typedef struct _mipi_dev_param_s {
	/* type must be: uint32_t */
	uint32_t nocheck;
	uint32_t notimeout;
	uint32_t wait_ms;
	uint32_t dbg_value;
	uint32_t power_instart;
	uint32_t hsync_pkt;
	uint32_t init_retry;
	uint32_t ipi_force;
	uint32_t ipi_limit;
	uint32_t ipi1_dt;
#if MIPIDEV_CHANNEL_NUM >= 2
	uint32_t ipi2_dt;
#endif
#if MIPIDEV_CHANNEL_NUM >= 3
	uint32_t ipi3_dt;
#endif
#if MIPIDEV_CHANNEL_NUM >= 4
	uint32_t ipi4_dt;
#endif
#if MIPI_DEV_INT_DBG
	uint32_t irq_cnt;
	uint32_t irq_debug;
#endif
#ifdef CONFIG_HOBOT_MIPI_PHY
	mipi_dphy_tx_param_t phy;
#endif
} mipi_dev_param_t;

static const char *g_md_param_names[] = {
	"nocheck",
	"notimeout",
	"wait_ms",
	"dbg_value",
	"power_instart",
	"hsync_pkt",
	"init_retry",
	"ipi_force",
	"ipi_limit",
	"ipi1_dt",
#if MIPIDEV_CHANNEL_NUM >= 2
	"ipi2_dt",
#endif
#if MIPIDEV_CHANNEL_NUM >= 3
	"ipi3_dt",
#endif
#if MIPIDEV_CHANNEL_NUM >= 4
	"ipi4_dt",
#endif
#if MIPI_DEV_INT_DBG
	"irq_cnt",
	"irq_debug",
#endif
#ifdef CONFIG_HOBOT_MIPI_PHY
	MIPI_DPHY_TX_PARAM_NAMES,
#endif
};

#if MIPI_DEV_INT_DBG
typedef struct _mipi_dev_icnt_s {
	/* type must be: uint32_t */
	uint32_t st_main;
	uint32_t vpg;
	uint32_t idi;
	uint32_t ipi;
	uint32_t phy;
	uint32_t mt_ipi;
	uint32_t idi_vcx;
	uint32_t idi_vcx2;
} mipi_dev_icnt_t;
#define MIPI_DEV_ICNT_NUM (sizeof(mipi_dev_icnt_t)/sizeof(uint32_t))

static const char *g_md_icnt_names[] = {
	"st_main",
	"vpg",
	"idi",
	"ipi",
	"phy",
	"mt_ipi",
	"idi_vcx",
	"idi_vcx2",
};

typedef struct _mipi_dev_ireg_s {
	uint32_t icnt_n;
	uint32_t st_mask;
	uint32_t reg_st;
	uint32_t reg_mask;
	uint32_t reg_force;
	uint32_t err_mask;
#if MIPI_DEV_INT_DBG_ERRSTR
	const char* err_str[32];
#endif
} mipi_dev_ireg_t;

static const mipi_dev_ireg_t md_int_regs_1p2[] = {
	{ 1, MIPI_DEV_INT_VPG, REG_MIPI_DEV_INT_ST_VPG,
		REG_MIPI_DEV_INT_MASK_N_VPG, REG_MIPI_DEV_INT_FORCE_VPG,
		0x00000001,
#if MIPI_DEV_INT_DBG_ERRSTR
	  { "vpg_pkt_lost" },
#endif
	},
	{ 2, MIPI_DEV_INT_IDI, REG_MIPI_DEV_INT_ST_IDI,
		REG_MIPI_DEV_INT_MASK_N_IDI, REG_MIPI_DEV_INT_FORCE_IDI,
		0x000003ff,
#if MIPI_DEV_INT_DBG_ERRSTR
	  { "idi_errwc", "idi_vc0_errf_seq", "idi_vc1_errf_seq",
		"idi_vc2_errf_seq", "idi_vc3_errf_seq", "idi_vc0_errl_seq",
		"idi_vc1_errl_seq", "idi_vc2_errl_seq",
		"idi_vc3_errl_seq", "idi_fifo_overflow" },
#endif
	},
	{ 3, MIPI_DEV_INT_IPI, REG_MIPI_DEV_INT_ST_IPI,
		REG_MIPI_DEV_INT_MASK_N_IPI, REG_MIPI_DEV_INT_FORCE_IPI,
		0x1f1f1f1f,
#if MIPI_DEV_INT_DBG_ERRSTR
	  { "ipi_errpixel", "ipi_fifo_overflow", "ipi_errline",
		"ipi_fifo_underflow", "ipi_trans_conflict", NULL, NULL, NULL,
		"ipi2_errpixel", "ipi2_fifo_overflow", "ipi2_errline",
		"ipi2_fifo_underflow", "ipi2_trans_conflict", NULL, NULL, NULL,
		"ipi3_errpixel", "ipi3_fifo_overflow", "ipi3_errline",
		"ipi3_fifo_underflow", "ipi3_trans_conflict", NULL, NULL, NULL,
		"ipi4_errpixel", "ipi4_fifo_overflow", "ipi4_errline",
		"ipi4_fifo_underflow", "ipi4_trans_conflict", NULL, NULL, NULL },
#endif
	},
	{ 4, MIPI_DEV_INT_PHY, REG_MIPI_DEV_INT_ST_PHY,
		REG_MIPI_DEV_INT_MASK_N_PHY, REG_MIPI_DEV_INT_FORCE_PHY,
		0x00000007,
#if MIPI_DEV_INT_DBG_ERRSTR
	  { "to_hs_tx", "errcontentionlp0", "errcontentionlp1" },
#endif
	},
	{ 5, MIPI_DEV_INT_MT_IPI, REG_MIPI_DEV_INT_ST_MT_IPI,
		REG_MIPI_DEV_INT_MASK_N_MT_IPI, REG_MIPI_DEV_INT_FORCE_MT_IPI,
		0x00000001,
#if MIPI_DEV_INT_DBG_ERRSTR
	  { "mt_ipi_fifo_overflow" },
#endif
	},
	{ 6, MIPI_DEV_INT_IDI_VCX_DMY, REG_MIPI_DEV_INT_ST_IDI_VCX,
		REG_MIPI_DEV_INT_MASK_N_IDI_VCX, REG_MIPI_DEV_INT_FORCE_IDI_VCX,
		0x0fff0fff,
#if MIPI_DEV_INT_DBG_ERRSTR
	  { "idi_vc4_errf_seq", "idi_vc5_errf_seq", "idi_vc6_errf_seq",
		"idi_vc7_errf_seq", "idi_vc8_errf_seq", "idi_vc9_errf_seq",
		"idi_vc10_errf_seq", "idi_vc11_errf_seq",
		"idi_vc12_errf_seq", "idi_vc13_errf_seq", "idi_vc14_errf_seq",
		"idi_vc15_errf_seq", NULL, NULL, NULL, NULL,
		"idi_vc4_errl_seq", "idi_vc5_errl_seq", "idi_vc6_errl_seq",
		"idi_vc7_errl_seq", "idi_vc8_errl_seq", "idi_vc9_errl_seq",
		"idi_vc10_errl_seq", "idi_vc11_errl_seq",
		"idi_vc12_errl_seq", "idi_vc13_errl_seq", "idi_vc14_errl_seq",
		"idi_vc15_errl_seq", NULL, NULL, NULL, NULL },
#endif
	},
	{ 6, MIPI_DEV_INT_IDI_VCX2_DMY, REG_MIPI_DEV_INT_ST_IDI_VCX2,
		REG_MIPI_DEV_INT_MASK_N_IDI_VCX2, REG_MIPI_DEV_INT_FORCE_IDI_VCX2,
		0xffffffff,
#if MIPI_DEV_INT_DBG_ERRSTR
	  { "idi_vc17_errf_seq", "idi_vc18_errf_seq", "idi_vc19_errf_seq",
		"idi_vc20_errf_seq", "idi_vc21_errf_seq", "idi_vc21_errf_seq",
		"idi_vc22_errf_seq", "idi_vc23_errf_seq",
		"idi_vc24_errf_seq", "idi_vc25_errf_seq", "idi_vc26_errf_seq",
		"idi_vc27_errf_seq", "idi_vc28_errf_seq", "idi_vc29_errf_seq",
		"idi_vc30_errf_seq", "idi_vc31_errf_seq",
		"idi_vc17_errl_seq", "idi_vc18_errl_seq", "idi_vc19_errl_seq",
		"idi_vc20_errl_seq", "idi_vc21_errl_seq", "idi_vc21_errl_seq",
		"idi_vc22_errl_seq", "idi_vc23_errl_seq",
		"idi_vc24_errl_seq", "idi_vc25_errl_seq", "idi_vc26_errl_seq",
		"idi_vc27_errl_seq", "idi_vc28_errl_seq", "idi_vc29_errl_seq",
		"idi_vc30_errl_seq", "idi_vc31_errl_seq" },
#endif
	}
};

typedef struct _mipi_dev_ierr_s {
	const mipi_dev_ireg_t *iregs;
	uint32_t num;
} mipi_dev_ierr_t;
#endif

typedef struct _mipi_dev_s {
	void __iomem     *iomem;
	int               irq;
	mipi_state_t      state;   /* mipi dev state */
	mipi_dev_cfg_t    cfg;
	mipi_dev_param_t  param;
#if MIPI_DEV_INT_DBG
	mipi_dev_ierr_t   ierr;
	mipi_dev_icnt_t   icnt;
#endif
} mipi_dev_t;

typedef struct _mipi_user_s {
	/*
	 * mutex: user.open_mutex
	 * protect: user.open_cnt and operations when first open and last close.
	 * init: probe, see: hobot_mipi_dev_probe_cdev.
	 * call: open/close, see: hobot_mipi_dev_open, hobot_mipi_dev_close.
	 */
	struct mutex open_mutex;
	/*
	 * mutex: user.mutex
	 * protect: user.init_cnt user.start_cnt and operations of mipi dev.
	 * init: first open, see hobot_mipi_dev_open.
	 * call: ioctl, see hobot_mipi_dev_ioctl.
	 */
	struct mutex mutex;
	uint32_t open_cnt;
	uint32_t init_cnt;
	uint32_t start_cnt;
} mipi_user_t;

typedef struct _mipi_ddev_s {
	int               port;
	int               lane_mode;
	dev_t             devno;
	struct cdev       cdev;
	struct device    *dev;
	uint64_t          ipi_clock;
	mipi_dev_t        mdev;
	mipi_user_t       user;
#ifdef CONFIG_HOBOT_DIAG
	struct timer_list diag_timer;
	uint32_t          last_err_tm_ms;
#endif
#if MIPI_DEV_INT_DBG && defined MIPI_DEV_INT_USE_TIMER
	struct timer_list irq_timer;
	uint32_t          irq_timer_en;
	uint32_t          irq_st_main;
#endif
} mipi_ddev_t;

static int g_md_major;
static struct class *g_md_class;
static mipi_ddev_t *g_ddev[MIPI_DEV_MAX_NUM];

/**
 * @brief mipi_dev_configure_lanemode: configure dphy as csi tx mode
 *
 * @param [in] lane : mipi dev lane
 *
 * @return int32_t: 0/-1
 */
static int32_t mipi_dev_configure_lanemode(mipi_ddev_t *ddev, int lane)
{
	struct device *dev = ddev->dev;
	mipi_dev_t *mdev = &ddev->mdev;
	int ret;

	if (!ddev)
		return -1;

	/* sync lane_mode */
	ddev->lane_mode = mipi_dphy_get_lanemode(MIPI_DPHY_TYPE_DEV,
							ddev->port);
	if (ddev->lane_mode == 0)
		return 0;

	mipiinfo("change lane_mode to csi");
	if (mdev->state != MIPI_STATE_DEFAULT) {
		mipierr("error: port %d is busy", ddev->port);
		return -1;
	}

	ret = mipi_dphy_set_lanemode(MIPI_DPHY_TYPE_DEV, ddev->port, 0);
	if (ret) {
		mipierr("error: vio dphy ctrl %d", ret);
		return -1;
	}

	return 0;
}

/**
 * @brief mipi_dev_configure_ipi : configure dev ipi
 *
 * @param [in] cfg : the dev cfgler's setting
 *
 * @return int32_t: 0/-1
 */
static int32_t mipi_dev_configure_ipi(mipi_ddev_t *ddev, mipi_dev_cfg_t *cfg)
{
	struct device *dev = ddev->dev;
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_dev_param_t *param = &mdev->param;
	void __iomem *iomem = mdev->iomem;
	int ipi_num;
	int vcid, datatype;

	if (!ddev || !iomem)
		return -1;

#ifdef CONFIG_HOBOT_XJ2
	ipi_num = 1;
#else
	ipi_num = 4;
#endif
	if (cfg->channel_num > ipi_num) {
		mipierr("channel_num %d error, max %d", cfg->channel_num, ipi_num);
		return -1;
	}

	ipi_num = 0;
	switch (cfg->channel_num) {
	case 4:
#if MIPIDEV_CHANNEL_NUM >= 4
		/* ipi4 config */
		vcid = cfg->channel_sel[3];
		if (vcid < 32) {
			datatype = (param->ipi4_dt) ? param->ipi4_dt : cfg->datatype;
			if (param->ipi4_dt || vcid != 3)
				mipiinfo("ipi4 vc%d datatype 0x%02x", vcid, datatype);
			if (param->hsync_pkt)
				mipi_putreg(iomem + REG_MIPI_DEV_IPI4_PKT_CFG,
					MIPI_DEV_IPI_PKT_CFG | MIPI_DEV_IPI_VC(vcid) |
					datatype | MIPI_DEV_IPI_HSYNC_PKT_EN);
			else
				mipi_putreg(iomem + REG_MIPI_DEV_IPI4_PKT_CFG,
					MIPI_DEV_IPI_PKT_CFG | MIPI_DEV_IPI_VC(vcid) |
					datatype);
			mipi_putreg(iomem + REG_MIPI_DEV_IPI4_MAX_FRAME_NUM,
				MIPI_DEV_IPI_MAX_FRAME);
			mipi_putreg(iomem + REG_MIPI_DEV_IPI4_PIXELS,
				cfg->width);
			mipi_putreg(iomem + REG_MIPI_DEV_IPI4_LINES,
				cfg->ipi_lines);
			ipi_num++;
		}
#endif
		/* no break */
	case 3:
#if MIPIDEV_CHANNEL_NUM >= 3
		/* ipi3 config */
		vcid = cfg->channel_sel[2];
		if (vcid < 32) {
			datatype = (param->ipi3_dt) ? param->ipi3_dt : cfg->datatype;
			if (param->ipi3_dt || vcid != 2)
				mipiinfo("ipi3 vc%d datatype 0x%02x", vcid, datatype);
			if (param->hsync_pkt)
				mipi_putreg(iomem + REG_MIPI_DEV_IPI3_PKT_CFG,
					MIPI_DEV_IPI_PKT_CFG | MIPI_DEV_IPI_VC(vcid) |
					datatype | MIPI_DEV_IPI_HSYNC_PKT_EN);
			else
				mipi_putreg(iomem + REG_MIPI_DEV_IPI3_PKT_CFG,
					MIPI_DEV_IPI_PKT_CFG | MIPI_DEV_IPI_VC(vcid) |
					datatype);
			mipi_putreg(iomem + REG_MIPI_DEV_IPI3_MAX_FRAME_NUM,
				MIPI_DEV_IPI_MAX_FRAME);
			mipi_putreg(iomem + REG_MIPI_DEV_IPI3_PIXELS,
				cfg->width);
			mipi_putreg(iomem + REG_MIPI_DEV_IPI3_LINES,
				cfg->ipi_lines);
			ipi_num++;
		}
#endif
		/* no break */
	case 2:
#if MIPIDEV_CHANNEL_NUM >= 2
		/* ipi2 config */
		vcid = cfg->channel_sel[1];
		if (vcid < 32) {
			datatype = (param->ipi2_dt) ? param->ipi2_dt : cfg->datatype;
			if (param->ipi2_dt || vcid != 1)
				mipiinfo("ipi2 vc%d datatype 0x%02x", vcid, datatype);
			if (param->hsync_pkt)
				mipi_putreg(iomem + REG_MIPI_DEV_IPI2_PKT_CFG,
					MIPI_DEV_IPI_PKT_CFG | MIPI_DEV_IPI_VC(vcid) |
					datatype | MIPI_DEV_IPI_HSYNC_PKT_EN);
			else
				mipi_putreg(iomem + REG_MIPI_DEV_IPI2_PKT_CFG,
					MIPI_DEV_IPI_PKT_CFG | MIPI_DEV_IPI_VC(vcid) |
					datatype);
			mipi_putreg(iomem + REG_MIPI_DEV_IPI2_MAX_FRAME_NUM,
				MIPI_DEV_IPI_MAX_FRAME);
			mipi_putreg(iomem + REG_MIPI_DEV_IPI2_PIXELS,
				cfg->width);
			mipi_putreg(iomem + REG_MIPI_DEV_IPI2_LINES,
				cfg->ipi_lines);
			ipi_num++;
		}
#endif
		/* no break */
	case 1:
	default:
		/* ipi1 config */
		vcid = cfg->channel_sel[0];
		if (vcid < 32) {
			datatype = (param->ipi1_dt) ? param->ipi1_dt : cfg->datatype;
			if (param->ipi1_dt || vcid != 0)
				mipiinfo("ipi1 vc%d datatype 0x%02x", vcid, datatype);
			if (param->hsync_pkt)
				mipi_putreg(iomem + REG_MIPI_DEV_IPI_PKT_CFG,
					MIPI_DEV_IPI_PKT_CFG | MIPI_DEV_IPI_VC(vcid) |
					datatype | MIPI_DEV_IPI_HSYNC_PKT_EN);
			else
				mipi_putreg(iomem + REG_MIPI_DEV_IPI_PKT_CFG,
					MIPI_DEV_IPI_PKT_CFG | MIPI_DEV_IPI_VC(vcid) |
					datatype);
			mipi_putreg(iomem + REG_MIPI_DEV_IPI_MAX_FRAME_NUM,
				MIPI_DEV_IPI_MAX_FRAME);
			mipi_putreg(iomem + REG_MIPI_DEV_IPI_PIXELS,
				cfg->width);
			mipi_putreg(iomem + REG_MIPI_DEV_IPI_LINES,
				cfg->ipi_lines);
			ipi_num++;
		}
		break;
	}

	mipiinfo("config %d/%d ipi done", ipi_num, cfg->channel_num);
	return 0;
}

/**
 * @brief mipi_dev_configure_cmp: configure compare.
 *
 * @param [in] scfg dcfg: mipi dev configure
 *
 * @return int32_t: 0/-1
 */
static int32_t mipi_dev_configure_cmp(mipi_dev_cfg_t *scfg, mipi_dev_cfg_t *dcfg)
{
	mipi_dev_cfg_t bcfg;

	if (!scfg || !dcfg)
		return -1;

	memcpy(&bcfg, scfg, sizeof(mipi_dev_cfg_t));
	if (dcfg->ipi_lines == 0)
		bcfg.ipi_lines = 0;

	return memcmp(&bcfg, dcfg, sizeof(mipi_dev_cfg_t));
}

/**
 * @brief mipi_dev_ipi_get_info: get ipi info of mipi dev
 *
 * @param [in/out] ipi_info: ipi info struct.
 *
 * @return int32_t : 0/-1
 */
static int32_t mipi_dev_ipi_get_info(mipi_ddev_t *ddev, mipi_dev_ipi_info_t *ipi_info)
{
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_dev_cfg_t *cfg = &mdev->cfg;
	mipi_dev_param_t *param = &mdev->param;
	struct device *dev = ddev->dev;
	void __iomem *iomem = mdev->iomem;
	int32_t index, ipi_max;
	uint32_t v_pktcfg;
	int32_t r_fatal, r_pktcfg, r_maxfnum, r_pixels, r_lines;

	if (!ipi_info || !iomem)
		return -1;

#ifdef CONFIG_HOBOT_XJ2
	ipi_max = 1;
#else
	ipi_max = 4;
#endif
	index = ipi_info->index;
	if (index >= ipi_max) {
		mipierr("%d:ipi%d get: not suppor error", index, index + 1);
		return -1;
	} else if (index >= cfg->channel_num) {
		mipidbg("%d:ipi%d get: not inited warning", index, index + 1);
	}
	r_fatal = REG_MIPI_DEV_INT_ST_IPI;
	switch (index) {
	case 3:
		r_pktcfg = REG_MIPI_DEV_IPI4_PKT_CFG;
		r_maxfnum = REG_MIPI_DEV_IPI4_MAX_FRAME_NUM;
		r_pixels = REG_MIPI_DEV_IPI4_PIXELS;
		r_lines = REG_MIPI_DEV_IPI4_LINES;
		break;
	case 2:
		r_pktcfg = REG_MIPI_DEV_IPI3_PKT_CFG;
		r_maxfnum = REG_MIPI_DEV_IPI3_MAX_FRAME_NUM;
		r_pixels = REG_MIPI_DEV_IPI3_PIXELS;
		r_lines = REG_MIPI_DEV_IPI3_LINES;
		break;
	case 1:
		r_pktcfg = REG_MIPI_DEV_IPI2_PKT_CFG;
		r_maxfnum = REG_MIPI_DEV_IPI2_MAX_FRAME_NUM;
		r_pixels = REG_MIPI_DEV_IPI2_PIXELS;
		r_lines = REG_MIPI_DEV_IPI2_LINES;
		break;
	case 0:
	default:
		r_pktcfg = REG_MIPI_DEV_IPI_PKT_CFG;
		r_maxfnum = REG_MIPI_DEV_IPI_MAX_FRAME_NUM;
		r_pixels = REG_MIPI_DEV_IPI_PIXELS;
		r_lines = REG_MIPI_DEV_IPI_LINES;
		break;
	}
	ipi_info->fatal = (uint16_t)((mipi_getreg(iomem + r_fatal) >> (index * 8)) & 0xff);
	v_pktcfg = mipi_getreg(iomem + r_pktcfg);
	ipi_info->mode = (uint16_t)(v_pktcfg >> 8);
	ipi_info->vc = (uint16_t)((v_pktcfg >> 6) & 0x3);
	ipi_info->datatype = (uint16_t)(v_pktcfg & 0x3F);
	ipi_info->maxfnum = (uint16_t)mipi_getreg(iomem + r_maxfnum);
	ipi_info->pixels = (uint32_t)mipi_getreg(iomem + r_pixels);
	ipi_info->lines = (uint32_t)mipi_getreg(iomem + r_lines);

	return 0;
}

/**
 * @brief mipi_dev_ipi_set_info: set ipi info of mipi dev
 *
 * @param [in] ipi_info: ipi info struct.
 *
 * @return int32_t : 0/-1
 */
static int32_t mipi_dev_ipi_set_info(mipi_ddev_t *ddev, mipi_dev_ipi_info_t *ipi_info)
{
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_dev_cfg_t *cfg = &mdev->cfg;
	mipi_dev_param_t *param = &mdev->param;
	struct device *dev = ddev->dev;
	void __iomem *iomem = mdev->iomem;
	int32_t index, ipi_max, set_mask;
	uint32_t v_pktcfg;
	int32_t r_pktcfg, r_maxfnum, r_pixels, r_lines;

	if (!ipi_info || !iomem)
		return -1;

#ifdef CONFIG_HOBOT_XJ2
	ipi_max = 1;
#else
	ipi_max = 4;
#endif
	index = ipi_info->index;
	if (index >= ipi_max) {
		mipierr("%d:ipi%d get: not suppor error", index, index + 1);
		return -1;
	} else if (index >= cfg->channel_num) {
		mipidbg("%d:ipi%d get: not inited warning", index, index + 1);
	}
	switch (index) {
	case 3:
		r_pktcfg = REG_MIPI_DEV_IPI4_PKT_CFG;
		r_maxfnum = REG_MIPI_DEV_IPI4_MAX_FRAME_NUM;
		r_pixels = REG_MIPI_DEV_IPI4_PIXELS;
		r_lines = REG_MIPI_DEV_IPI4_LINES;
		break;
	case 2:
		r_pktcfg = REG_MIPI_DEV_IPI3_PKT_CFG;
		r_maxfnum = REG_MIPI_DEV_IPI3_MAX_FRAME_NUM;
		r_pixels = REG_MIPI_DEV_IPI3_PIXELS;
		r_lines = REG_MIPI_DEV_IPI3_LINES;
		break;
	case 1:
		r_pktcfg = REG_MIPI_DEV_IPI2_PKT_CFG;
		r_maxfnum = REG_MIPI_DEV_IPI2_MAX_FRAME_NUM;
		r_pixels = REG_MIPI_DEV_IPI2_PIXELS;
		r_lines = REG_MIPI_DEV_IPI2_LINES;
		break;
	case 0:
	default:
		r_pktcfg = REG_MIPI_DEV_IPI_PKT_CFG;
		r_maxfnum = REG_MIPI_DEV_IPI_MAX_FRAME_NUM;
		r_pixels = REG_MIPI_DEV_IPI_PIXELS;
		r_lines = REG_MIPI_DEV_IPI_LINES;
		break;
	}

	/* set some masked if fatal.b15, or set all */
	set_mask = (ipi_info->fatal & 0x8000) ? ipi_info->fatal : 0xffff;
	v_pktcfg = mipi_getreg(iomem + r_pktcfg);
	if (set_mask & (0x1 << 0))
		v_pktcfg = (ipi_info->mode << 8) | (v_pktcfg & 0xff);
	if (set_mask & (0x1 << 1))
		v_pktcfg = ((ipi_info->vc & 0x3) << 6) | (v_pktcfg & ~0xc);
	if (set_mask & (0x1 << 2))
		v_pktcfg = (ipi_info->datatype & 0x3f) | (v_pktcfg & ~0x3f);
	if (set_mask & 0x7)
		mipi_putreg(iomem + r_pktcfg, v_pktcfg);
	if (set_mask & (0x1 << 3))
		mipi_putreg(iomem + r_maxfnum, ipi_info->maxfnum);
	if (set_mask & (0x1 << 4))
		mipi_putreg(iomem + r_pixels, ipi_info->pixels);
	if (set_mask & (0x1 << 5))
		mipi_putreg(iomem + r_lines, ipi_info->lines);

	v_pktcfg = mipi_getreg(iomem + r_pktcfg);
	mipiinfo("%d:ipi%d set: mode=0x%x,vc=0x%x,dt=0x%x,maxfnum=%d,pixels=%d,lines=%d",
		index, index + 1,
		v_pktcfg >> 8,
		(v_pktcfg >> 6) & 0x3,
		v_pktcfg & 0x3f,
		mipi_getreg(iomem + r_maxfnum),
		mipi_getreg(iomem + r_pixels),
		mipi_getreg(iomem + r_lines));

	return 0;
}

#ifdef CONFIG_HOBOT_XJ3
int vio_clk_enable(const char *name);
int vio_clk_disable(const char *name);
int vio_set_clk_rate(const char *name, ulong frequency);
ulong vio_get_clk_rate(const char *name);
#endif
/**
 * @brief mipi_dev_configure_clk: configure clk of mipi dev
 *
 * @param [in] cfg : mipi dev cfgler's setting
 *
 * @return int32_t : 0/-1
 */
static int32_t mipi_dev_configure_clk(mipi_ddev_t *ddev, const char *name,
				ulong freq, int checkequ)
{
	int32_t ret = 0;
	struct device *dev = ddev->dev;
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
 * @brief mipi_dev_get_clk: configure clk of mipi dev
 *
 * @param [in] name: mipi dev clk's getting
 *
 * @return int32_t : 0/-1
 */
static ulong mipi_dev_get_clk(mipi_ddev_t *ddev, const char *name)
{
	ulong clk = 0;
#ifdef CONFIG_HOBOT_XJ3
	clk = vio_get_clk_rate(name);
#else
	struct device *dev = ddev->dev;

	mipiinfo("should: %s %s %lu", __func__, name, clk);
#endif
	return clk;
}

/* mipi dev functions */
static unsigned long mipi_dev_pixel_clk_select(mipi_ddev_t *ddev, mipi_dev_cfg_t *cfg)
{
	struct device *dev = ddev->dev;
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_dev_param_t *param = &mdev->param;
	unsigned long pixclk = cfg->linelenth * cfg->framelenth * cfg->fps;
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
	if (ddev->port >= ARRAY_SIZE(g_md_ipiclk_name) ||
		mipi_dev_configure_clk(ddev, g_md_ipiclk_name[ddev->port], pixclk, 0) < 0) {
		pixclk_act = 0;
		mipierr("mipi_dev_configure_clk %lu error", pixclk);
	} else {
		pixclk_act = mipi_dev_get_clk(ddev, g_md_ipiclk_name[ddev->port]);
		if (pixclk_act == 0)
			pixclk_act = pixclk;
		mipiinfo("ipiclk set %lu get %lu", pixclk, pixclk_act);
	}
	return pixclk_act;
}


/**
 * @brief mipi_dev_vpg_get_hline : get hline time in vpg mode
 *
 * @param [in] control : mipi dev config
 *
 * @return uint16_t hline
 */
static uint16_t mipi_dev_vpg_get_hline(mipi_ddev_t *ddev, mipi_dev_cfg_t *cfg)
{
	struct device *dev = ddev->dev;
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_dev_param_t *param = &mdev->param;
	uint16_t hline = 0;

	switch (cfg->datatype) {
	case MIPI_CSI2_DT_YUV420_8:
		hline = (12 * cfg->linelenth) / 8;
		break;
	case MIPI_CSI2_DT_YUV422_8:
		hline = (16 * cfg->linelenth) / 8;
		break;
	case MIPI_CSI2_DT_YUV420_10:
		hline = (12 * 2 * cfg->linelenth) / 8;
		break;
	case MIPI_CSI2_DT_YUV422_10:
		hline = (16 * 2 * cfg->linelenth) / 8;
		break;
	case MIPI_CSI2_DT_RAW_8:
		hline = (8 * cfg->linelenth) / 8;
		break;
	case MIPI_CSI2_DT_RAW_10:
		hline = (10 * cfg->linelenth) / 8;
		break;
	case MIPI_CSI2_DT_RAW_12:
		hline = (12 * cfg->linelenth) / 8;
		break;
	case MIPI_CSI2_DT_RAW_14:
		hline = (14 * cfg->linelenth) / 8;
		break;
	default:
		mipierr("data type 0x%x not support", cfg->datatype);
		return 0;
	}
	if (cfg->linelenth == 0 || cfg->linelenth == cfg->width)
		hline = MIPI_DEV_VPG_HLINE_TIME;

	hline = hline / cfg->lane;
	mipidbg("vpg hline: %d", hline);
	return hline;
}

/**
 * @brief mipi_dev_vpg_get_vfp : get mipi dev vfp in vpg mode
 *
 * @param [in] cfg : mipi dev config
 *
 * @return uint16_t vfp
 */
static uint16_t mipi_dev_vpg_get_vfp(mipi_ddev_t *ddev, mipi_dev_cfg_t *cfg)
{
	struct device *dev = ddev->dev;
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_dev_param_t *param = &mdev->param;
	uint16_t vfp;

	vfp	= cfg->framelenth - MIPI_DEV_VPG_VSA_LINES -
			MIPI_DEV_VPG_VBP_LINES - cfg->height;
	if (cfg->framelenth == 0 || cfg->framelenth == cfg->height)
		vfp = MIPI_DEV_VPG_VFP_LINES;

	if (vfp > MIPI_DEV_VPG_VFP_LINES_MAX) {
		mipidbg("vpg vfp %d overflow as: %d", vfp, MIPI_DEV_VPG_VFP_LINES_MAX);
		return MIPI_DEV_VPG_VFP_LINES_MAX;
	}

	mipidbg("vpg vfp: %d", vfp);
	return vfp;
}

/**
 * @brief mipi_dev_vpg_get_bk : get mipi dev bk in vpg mode
 *
 * @param [in] cfg : mipi dev config
 *
 * @return uint16_t bk
 */
static uint16_t mipi_dev_vpg_get_bk(mipi_ddev_t *ddev, mipi_dev_cfg_t *cfg)
{
	struct device *dev = ddev->dev;
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_dev_param_t *param = &mdev->param;
	uint16_t vfp, bk = 0;

	vfp	= cfg->framelenth - MIPI_DEV_VPG_VSA_LINES -
			MIPI_DEV_VPG_VBP_LINES - cfg->height;
	if (cfg->framelenth == 0 || cfg->framelenth == cfg->height)
		vfp = MIPI_DEV_VPG_VFP_LINES;

	if (vfp > MIPI_DEV_VPG_VFP_LINES_MAX) {
		bk = vfp - MIPI_DEV_VPG_VFP_LINES_MAX;
		if (bk > MIPI_DEV_VPG_BK_LINES_MAX) {
			mipiinfo("vpg bk %d overflow, set to %d", bk, MIPI_DEV_VPG_BK_LINES_MAX);
			bk = MIPI_DEV_VPG_BK_LINES_MAX;
		}
		mipidbg("vpg bk: %d", bk);
	}

	return bk;
}

/**
 * @brief mipi_dev_configure_vpg : configure&enable dev vpg mode
 *
 * @param [in] cfg : the dev cfgler's setting
 *
 * @return int32_t: 0/-1
 */
static int32_t mipi_dev_configure_vpg(mipi_ddev_t *ddev, mipi_dev_cfg_t *cfg)
{
	struct device *dev = ddev->dev;
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_dev_param_t *param = &mdev->param;
	void __iomem *iomem = mdev->iomem;
	uint8_t    ncount = 0;
	uint32_t   status = 0;
	uint8_t	   vc, mode, ori;

	if (!ddev || !iomem)
		return -1;

	mipiinfo("configure vpg begin");
	/*Disable the Video Pattern Generator*/
	mipi_putreg(iomem + REG_MIPI_DEV_VPG_CTRL, MIPI_DEV_VPG_DISABLE);

	/*Wait for VPG idle*/
	do {
		status = mipi_getreg(iomem + REG_MIPI_DEV_VPG_STATUS);
		if (ncount >= MIPI_DEV_CHECK_MAX) {
			mipierr("vga status of dev is error: 0x%x", status);
			return -1;
		}
		mdelay(1);
		ncount++;
	} while (MIPI_DEV_VPG_DISABLE != status);

	/*Configure the VPG mode*/
	/*cfg->vpg: bit0:enable, bit1:ori, bit2:mode, bit3~:vc*/
	vc = (cfg->vpg >> 3) & 0x1F;
	mode = (cfg->vpg >> 2) & 0x1;
	ori = (cfg->vpg >> 1) & 0x1;
	mipi_putreg(iomem + REG_MIPI_DEV_VPG_MODE_CFG, MIPI_DEV_VPG_ORI(ori) |
		MIPI_DEV_VPG_MODE(mode));
	if (param->hsync_pkt)
		mipi_putreg(iomem + REG_MIPI_DEV_VPG_PKT_CFG, MIPI_DEV_VPG_LINENUM |
			MIPI_DEV_VPG_HSYNC(MIPI_DEV_VPG_HSYNC_EN) |
			MIPI_DEV_VPG_VCX(vc) | MIPI_DEV_VPG_VC(vc) | cfg->datatype);
	else
		mipi_putreg(iomem + REG_MIPI_DEV_VPG_PKT_CFG, MIPI_DEV_VPG_LINENUM |
			MIPI_DEV_VPG_HSYNC(MIPI_DEV_VPG_HSYNC_DIS) |
			MIPI_DEV_VPG_VCX(vc) | MIPI_DEV_VPG_VC(vc) | cfg->datatype);
	mipi_putreg(iomem + REG_MIPI_DEV_VPG_PKT_SIZE, cfg->width);
	mipi_putreg(iomem + REG_MIPI_DEV_VPG_HSA_TIME, MIPI_DEV_VPG_HSA_TIME);
	mipi_putreg(iomem + REG_MIPI_DEV_VPG_HBP_TIME, MIPI_DEV_VPG_HBP_TIME);
	mipi_putreg(iomem + REG_MIPI_DEV_VPG_HLINE_TIME, mipi_dev_vpg_get_hline(ddev, cfg));
	mipi_putreg(iomem + REG_MIPI_DEV_VPG_VSA_LINES, MIPI_DEV_VPG_VSA_LINES);
	mipi_putreg(iomem + REG_MIPI_DEV_VPG_VBP_LINES, MIPI_DEV_VPG_VBP_LINES);
	mipi_putreg(iomem + REG_MIPI_DEV_VPG_VFP_LINES, mipi_dev_vpg_get_vfp(ddev, cfg));
	mipi_putreg(iomem + REG_MIPI_DEV_VPG_BK_LINES, mipi_dev_vpg_get_bk(ddev, cfg));
	mipi_putreg(iomem + REG_MIPI_DEV_VPG_ACT_LINES, cfg->height);
	mipi_putreg(iomem + REG_MIPI_DEV_VPG_MAX_FRAME_NUM, MIPI_DEV_IPI_MAX_FRAME);
	//mipi_putreg(iomem + REG_MIPI_DEV_VPG_START_LINE_NUM, MIPI_DEV_VPG_START_LINE);
	//mipi_putreg(iomem + REG_MIPI_DEV_VPG_STEP_LINE_NUM, MIPI_DEV_VPG_STEP_LINE);

	/*Enalbe the Video Pattern Generator*/
	mipi_putreg(iomem + REG_MIPI_DEV_VPG_CTRL, MIPI_DEV_VPG_ENABLE);

	mipiinfo("configure vpg 0x%02x vc%d %s %s end", cfg->datatype, vc,
		(mode) ? "ber" : "bar", (ori) ? "horizontal" : "vertical");
	return 0;
}

#if MIPI_DEV_INT_DBG
/**
 * @brief mipi_dev_irq_enable : Enale mipi dev IRQ
 *
 * @param [in] irq : IRQ Flag
 *
 * @return void
 */
static void mipi_dev_irq_enable(mipi_ddev_t *ddev)
{
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_dev_ierr_t *ierr = &mdev->ierr;
	const mipi_dev_ireg_t *ireg = NULL;
	void __iomem *iomem = mdev->iomem;
	uint32_t temp = 0;
	int i;

	if (!ddev || !iomem)
		return;

	temp = mipi_getreg(iomem + REG_MIPI_DEV_INT_ST_MAIN);
	for (i = 0; i < ierr->num; i++) {
		ireg = &ierr->iregs[i];
		temp = mipi_getreg(iomem + ireg->reg_st);
		temp = mipi_getreg(iomem + ireg->reg_mask);
		temp &= ~(ireg->err_mask);
		temp |= ireg->err_mask;
		mipi_putreg(iomem + ireg->reg_mask, temp);
	}

#ifdef MIPI_DEV_INT_USE_TIMER
	ddev->irq_timer_en = 1;
#endif

	return;
}

/**
 * @brief mipi_dev_irq_disable : Disable mipi dev IRQ
 *
 * @param [in] irq : IRQ Flag
 *
 * @return void
 */
static void mipi_dev_irq_disable(mipi_ddev_t *ddev)
{
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_dev_ierr_t *ierr = &mdev->ierr;
	const mipi_dev_ireg_t *ireg = NULL;
	void __iomem *iomem = mdev->iomem;
	uint32_t temp = 0;
	int i;

	if (!ddev || !iomem)
		return;

	for (i = 0; i < ierr->num; i ++) {
		ireg = &ierr->iregs[i];
		temp = mipi_getreg(iomem + ireg->reg_mask);
		temp &= ~(ireg->err_mask);
		mipi_putreg(iomem + ireg->reg_mask, temp);
	}

#ifdef MIPI_DEV_INT_USE_TIMER
	ddev->irq_timer_en = 0;
#endif

	return;
}

#ifdef CONFIG_HOBOT_DIAG
static void mipi_dev_diag_report(mipi_ddev_t *ddev,
		uint8_t errsta, uint32_t total_irq,
		uint32_t *sub_irq_data, uint32_t elem_cnt)
{
	uint32_t buff[MIPI_DEV_ICNT_NUM];
	int i;

	ddev->last_err_tm_ms = jiffies_to_msecs(get_jiffies_64());
	if (errsta) {
		buff[0] = total_irq;
		for (i = 0; i < elem_cnt && i < (MIPI_DEV_ICNT_NUM - 1); i++)
			buff[1 + i] = sub_irq_data[i];

		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_VIO,
				EventIdVioMipiDevErr,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				(uint8_t *)buff,
				sizeof(buff));
	}
}

static void mipi_dev_diag_timer_func(unsigned long data)
{
	mipi_ddev_t *ddev = (mipi_ddev_t *)data;
	uint32_t now_tm_ms;
	unsigned long jiffi;

	now_tm_ms = jiffies_to_msecs(get_jiffies_64());
	if (now_tm_ms - ddev->last_err_tm_ms > 6000) {
		diag_send_event_stat(
				DiagMsgPrioMid,
				ModuleDiag_VIO,
				EventIdVioMipiDevErr,
				DiagEventStaSuccess);
	}
	jiffi = get_jiffies_64() + msecs_to_jiffies(2000);
	mod_timer(&ddev->diag_timer, jiffi); // trriger again.
}

static void mipi_dev_diag_test(void *p, size_t len)
{
	mipi_ddev_t *ddev;
	void __iomem *iomem;
	int i;

	for (i = 0; i < MIPI_DEV_MAX_NUM; i ++) {
		ddev = g_ddev[i];
		if (!ddev)
			continue;
		iomem = ddev->mdev.iomem;
		if (!iomem)
			continue;
		mipi_putreg(iomem + REG_MIPI_DEV_INT_FORCE_PHY, 0x1);
	}
}
#endif

/**
 * @brief mipi_dev_irq_func : irq func
 *
 * @param [in] this_irq : irq num
 * @param [in] data : user data
 *
 * @return irqreturn_t
 */
static irqreturn_t mipi_dev_irq_func(int this_irq, void *data)
{
	mipi_ddev_t *ddev = (mipi_ddev_t *)data;
	struct device *dev = ddev->dev;
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_dev_ierr_t *ierr = &mdev->ierr;
	const mipi_dev_ireg_t *ireg = NULL;
	mipi_dev_param_t *param = &mdev->param;
	mipi_dev_icnt_t *icnt = &mdev->icnt;
	void __iomem *iomem = mdev->iomem;
	uint32_t *icnt_p = (uint32_t *)icnt;
	uint32_t reg, mask, icnt_n;
	uint32_t irq = 0, irq_do;
	uint32_t subirq = 0;
	int i;
	uint8_t err_occurred = 0;
	uint32_t env_subirq[MIPI_DEV_ICNT_NUM - 1] = {0};
	char *perr = "";
#if MIPI_DEV_INT_DBG_ERRSTR
	int j, l;
	uint32_t subirq_do;
	char err_str[256];
#endif

	if (!ddev || !iomem)
		return IRQ_NONE;

	if (this_irq >= 0)
		disable_irq_nosync(this_irq);

#ifdef MIPI_DEV_INT_USE_TIMER
	irq = ddev->irq_st_main;
#else
	irq = mipi_getreg(iomem + REG_MIPI_DEV_INT_ST_MAIN);
#endif
	if (param->irq_debug & MIPI_DEV_IRQ_DEBUG_PRERR)
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
			if (mask == MIPI_DEV_INT_IDI)
				irq_do |= (MIPI_DEV_INT_IDI_VCX_DMY |
					MIPI_DEV_INT_IDI_VCX2_DMY);

			reg = ireg->reg_st;
			icnt_n = ireg->icnt_n;
			subirq = mipi_getreg(iomem + reg) & ireg->err_mask;
			if (!subirq) {
				irq_do &= ~mask;
				continue;
			}

			if (!subirq)
				continue;
#if MIPI_DEV_INT_DBG_ERRSTR
			err_str[0] = '\0';
			perr = err_str;
			if (param->irq_debug & MIPI_DEV_IRQ_DEBUG_ERRSTR) {
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
			if (param->irq_debug & MIPI_DEV_IRQ_DEBUG_PRERR)
				mipierr("  %s: 0x%x%s",
					g_md_icnt_names[icnt_n], subirq, perr);
			else
				mipidbg("  %s: 0x%x%s",
					g_md_icnt_names[icnt_n], subirq, perr);
			icnt_p[icnt_n]++;
			err_occurred = 1;
			env_subirq[icnt_n - 1] = subirq;
			irq_do &= ~mask;
			if (!irq_do)
				break;
		}
	}

	if (icnt->st_main > param->irq_cnt)
		mipi_dev_irq_disable(ddev);

	if (this_irq >= 0)
		enable_irq(this_irq);

#ifdef CONFIG_HOBOT_DIAG
	mipi_dev_diag_report(ddev, err_occurred, irq, env_subirq,
				 sizeof(env_subirq)/sizeof(uint32_t));
#endif
	return IRQ_HANDLED;
}

#ifdef MIPI_DEV_INT_USE_TIMER
static void mipi_dev_irq_timer_func(unsigned long data)
{
	mipi_ddev_t *ddev = (mipi_ddev_t *)data;
	mipi_dev_t *mdev = &ddev->mdev;
	void __iomem *iomem = mdev->iomem;
	unsigned long jiffi;

	if (ddev->irq_timer_en) {
		if (iomem) {
			ddev->irq_st_main = mipi_getreg(iomem + REG_MIPI_DEV_INT_ST_MAIN);
			if (ddev->irq_st_main)
				mipi_dev_irq_func(-1, ddev);
		}
		jiffi = get_jiffies_64() + msecs_to_jiffies(50);
		mod_timer(&ddev->irq_timer, jiffi); // trriger again.
	} else {
		jiffi = get_jiffies_64() + msecs_to_jiffies(200);
		mod_timer(&ddev->irq_timer, jiffi); // trriger again.
	}
}
#endif
#endif

static int32_t mipi_dev_wait_phy_powerup(mipi_ddev_t *ddev, mipi_dev_cfg_t *cfg)
{
	struct device *dev = ddev->dev;
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_dev_param_t *param = &mdev->param;
	void __iomem *iomem = mdev->iomem;
	uint16_t ncount = 0;
	uint32_t state = 0;

	if (!ddev || !iomem || !cfg)
		return -1;

	/*Wait for the PHY power-up*/
	do {
		state = mipi_getreg(iomem + REG_MIPI_DEV_PHY_STATUS);
		mipiinfo("dphy state 0x%x", state);
		if ((state & DEV_DPHY_STATE_BASIC) == DEV_DPHY_STATE_NLANE) {
			if ((DEV_DPHY_STATE(state) & DEV_DPHY_STATE_STOP(cfg->lane)) ==
						DEV_DPHY_STATE_STOP(cfg->lane))
				return 0;
		}
		ncount++;
		mdelay(1);
	} while (param->notimeout || ncount <= param->wait_ms);

	mipierr("lane state of dev phy is error: 0x%x", state);
	return -1;
}

static int32_t mipi_dev_start(mipi_ddev_t *ddev)
{
	struct device *dev = ddev->dev;
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_dev_param_t *param = &mdev->param;
	mipi_dev_cfg_t *cfg = &mdev->cfg;
	void __iomem *iomem = mdev->iomem;

	if (!ddev || !iomem)
		return -1;

	if (param->power_instart) {
		/*Wake up DWC_mipicsi2_device*/
		mipi_putreg(iomem + REG_MIPI_DEV_CSI2_RESETN, MIPI_DEV_CSI2_RAISE);

		if ((cfg->mipiclk / cfg->lane) > DEV_DPHY_CAL_EN_MHZ) {
			mipi_putreg(iomem + REG_MIPI_DEV_PHY_CAL, DEV_DPHY_CAL_ENABLE);
			udelay(DEV_DPHY_CAL_DELAY_US);
			mipi_putreg(iomem + REG_MIPI_DEV_PHY_CAL, DEV_DPHY_CAL_DISABLE);
		}

		if (!param->nocheck) {
			if (0 != mipi_dev_wait_phy_powerup(ddev, &mdev->cfg)) {
				mipierr("phy stop state error!!!");
				mipi_putreg(iomem + REG_MIPI_DEV_CSI2_RESETN, MIPI_DEV_CSI2_RESETN);
				return -1;
			}
		}
	}

	/*Configure the High-Speed clock*/
	mipi_putreg(iomem + REG_MIPI_DEV_LPCLK_CTRL, MIPI_DEV_LPCLK_CONT);

#if MIPI_DEV_INT_DBG
	mipi_dev_irq_enable(ddev);
#endif

	return 0;
}

static int32_t mipi_dev_stop(mipi_ddev_t *ddev)
{
	mipi_dev_t *mdev = &ddev->mdev;
	void __iomem *iomem = mdev->iomem;

	if (!ddev || !iomem)
		return -1;

#if MIPI_DEV_INT_DBG
	mipi_dev_irq_disable(ddev);
#endif

	/*stop mipi dev here*/
	mipi_putreg(iomem + REG_MIPI_DEV_LPCLK_CTRL, MIPI_DEV_LPCLK_NCONT);

	return 0;
}

/**
 * @brief mipi_dev_deinit : mipi dev deinit function
 *
 * @param [] void :
 *
 * @return int32_t: 0/-1
 */
static int32_t mipi_dev_deinit(mipi_ddev_t *ddev)
{
	mipi_dev_t *mdev = &ddev->mdev;
	void __iomem *iomem = mdev->iomem;

	if (!ddev || !iomem)
		return -1;

	/*stop mipi dev here*/
	mipi_putreg(iomem + REG_MIPI_DEV_LPCLK_CTRL, MIPI_DEV_LPCLK_NCONT);
#ifdef CONFIG_HOBOT_MIPI_PHY
	mipi_dev_dphy_reset(iomem);
#endif
	/*Set DWC_mipi_csi2_dev reset*/
	mipi_putreg(iomem + REG_MIPI_DEV_PHY_IF_CFG, MIPI_DEV_CSI2_RESETN);
	mipi_putreg(iomem + REG_MIPI_DEV_PHY_RSTZ, MIPI_DEV_CSI2_RESETN);
	mipi_putreg(iomem + REG_MIPI_DEV_VPG_CTRL, MIPI_DEV_VPG_DISABLE);
	mipi_putreg(iomem + REG_MIPI_DEV_CSI2_RESETN, MIPI_DEV_CSI2_RESETN);

	if (ddev->ipi_clock != 0) {
		mipi_dev_configure_clk(ddev, g_md_ipiclk_name[ddev->port], 0, 0);
		ddev->ipi_clock = 0;
	}

	return 0;
}

/**
 * @brief mipi_dev_init : mipi dev init function
 *
 * @param [in] cfg : the dev cfgler's setting
 *
 * @return int32_t: 0/-1
 */
static int32_t mipi_dev_init(mipi_ddev_t *ddev, mipi_dev_cfg_t *cfg)
{
	struct device *dev = ddev->dev;
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_dev_param_t *param = &mdev->param;
	void __iomem *iomem = mdev->iomem;
	uint32_t power = 0;
	unsigned long pixclk = 0;
	int retry = 0;

	if (!ddev || !iomem)
		return -1;

	mipiinfo("init begin");
	if (0 != mipi_dev_configure_lanemode(ddev, cfg->lane)) {
		mipierr("configure lane error!!!");
		return -1;
	}
	pixclk = mipi_dev_pixel_clk_select(ddev, cfg);
	if (0 == pixclk) {
		mipierr("pixel clk config error!");
		return -1;
	}
	ddev->ipi_clock = pixclk;

init_retry:
#ifdef CONFIG_HOBOT_MIPI_PHY
	mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_DEV, ddev->port,
				MIPI_CFGCLKFREQRANGE, MIPI_DEV_CFGCLK_DEFAULT);
	/*Shut down and reset SNPS D-PHY*/
	mipi_dev_dphy_reset(iomem);
#endif
	/*Reset DWC_mipicsi2_device*/
	mipi_putreg(iomem + REG_MIPI_DEV_CLKMGR_CFG, MIPI_DEV_CSI2_RESETN);
	mipi_putreg(iomem + REG_MIPI_DEV_PHY_IF_CFG, MIPI_DEV_CSI2_RESETN);
	mipi_putreg(iomem + REG_MIPI_DEV_PHY_RSTZ, MIPI_DEV_CSI2_RESETN);
	mipi_putreg(iomem + REG_MIPI_DEV_CSI2_RESETN, MIPI_DEV_CSI2_RESETN);
	/*Configure the number of lanes*/
	mipi_putreg(iomem + REG_MIPI_DEV_PHY_IF_CFG, cfg->lane - 1);
	/*Configure the Escape mode transmit clock*/
	mipi_putreg(iomem + REG_MIPI_DEV_CLKMGR_CFG, MIPI_DEV_CLKMGR_RAISE);
	mipi_putreg(iomem + REG_MIPI_DEV_LPCLK_CTRL, MIPI_DEV_LPCLK_NCONT);

#ifdef CONFIG_HOBOT_MIPI_PHY
	/*Initialize the PHY*/
	if (0 != mipi_dev_dphy_initialize(iomem, cfg->mipiclk, cfg->lane, cfg->settle)) {
		mipi_dev_deinit(ddev);
		mipierr("dphy initialize error!!!");
		return -1;
	}
#endif
	/*Power on PHY*/
	power = DEV_DPHY_ENABLEZ;
	mipi_putreg(iomem + REG_MIPI_DEV_PHY_RSTZ, power);
	udelay(1);
	power |= DEV_DPHY_SHUTDOWNZ;
	mipi_putreg(iomem + REG_MIPI_DEV_PHY_RSTZ, power);
	udelay(1);
	power |= DEV_DPHY_RSTZ;
	mipi_putreg(iomem + REG_MIPI_DEV_PHY_RSTZ, power);
	power |= DEV_DPHY_FORCEPOLL;
	mipi_putreg(iomem + REG_MIPI_DEV_PHY_RSTZ, power);

	cfg->ipi_lines = cfg->ipi_lines ? cfg->ipi_lines : (cfg->height + 1);
	if (!cfg->vpg) {
		if (0 != mipi_dev_configure_ipi(ddev, cfg)) {
			mipi_dev_deinit(ddev);
			mipierr("configure ipi error!!!");
			return -1;
		}
	} else {
		if (0 != mipi_dev_configure_vpg(ddev, cfg)) {
			mipi_dev_deinit(ddev);
			mipierr("configure vpg error!!!");
			return -1;
		}
	}
#if MIPI_DEV_INT_DBG
	memset(&mdev->icnt, 0x0, sizeof(mdev->icnt));
#endif
	if (!param->power_instart) {
		/*Wake up DWC_mipicsi2_device*/
		mipi_putreg(iomem + REG_MIPI_DEV_CSI2_RESETN, MIPI_DEV_CSI2_RAISE);

		if ((cfg->mipiclk / cfg->lane) > DEV_DPHY_CAL_EN_MHZ) {
			mipi_putreg(iomem + REG_MIPI_DEV_PHY_CAL, DEV_DPHY_CAL_ENABLE);
			udelay(DEV_DPHY_CAL_DELAY_US);
			mipi_putreg(iomem + REG_MIPI_DEV_PHY_CAL, DEV_DPHY_CAL_DISABLE);
		}

		if (!param->nocheck) {
			if (0 != mipi_dev_wait_phy_powerup(ddev, cfg)) {
				if (retry < param->init_retry) {
					retry++;
					mipiinfo("init lock failed, retry %d...", retry);
					goto init_retry;
				}
				mipi_dev_deinit(ddev);
				mipierr("phy stop state error!!!");
				return -1;
			}
		}
	}
	memcpy(&mdev->cfg, cfg, sizeof(mipi_dev_cfg_t));
	mipiinfo("init end");
	return 0;
}

static int hobot_mipi_dev_close(struct inode *inode, struct file *file)
{
	mipi_ddev_t *ddev = file->private_data;
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_user_t *user = &ddev->user;
	mipi_dev_param_t *param = &mdev->param;
	struct device *dev = ddev->dev;

	if (mutex_lock_interruptible(&user->open_mutex)) {
		mipierr("open_mutex lock error");
		return -EACCES;
	}
	if (user->open_cnt > 0)
		user->open_cnt--;
	mipidbg("close as %d", user->open_cnt);
	if (user->open_cnt == 0) {
		if (mdev->state != MIPI_STATE_DEFAULT) {
			mipi_dev_stop(ddev);
			mipi_dev_deinit(ddev);
			mdev->state = MIPI_STATE_DEFAULT;
		}
		mipi_dev_configure_clk(ddev, MIPI_DEV_CFGCLK_NAME, 0, 0);
		mipi_dev_configure_clk(ddev, MIPI_DEV_REFCLK_NAME, 0, 0);
	}
	mutex_unlock(&user->open_mutex);

	return 0;
}

static int hobot_mipi_dev_open(struct inode *inode, struct file *file)
{
	mipi_ddev_t *ddev = container_of(inode->i_cdev, mipi_ddev_t, cdev);
	mipi_user_t *user = &ddev->user;
	mipi_dev_param_t *param = &ddev->mdev.param;
	struct device *dev = ddev->dev;

	if (mutex_lock_interruptible(&user->open_mutex)) {
		mipierr("open_mutex lock error");
		return -EACCES;
	}
	mipidbg("open as %d", user->open_cnt);
	if (user->open_cnt == 0) {
		mutex_init(&user->mutex);
		user->init_cnt = 0;
		user->start_cnt = 0;
		mipi_dev_configure_clk(ddev, MIPI_DEV_CFGCLK_NAME, MIPI_DEV_CFGCLK_MHZ, 1);
		mipi_dev_configure_clk(ddev, MIPI_DEV_REFCLK_NAME, MIPI_DEV_REFCLK_MHZ, 1);
	}
	user->open_cnt++;
	mutex_unlock(&user->open_mutex);

	file->private_data = ddev;
	return 0;
}

static long hobot_mipi_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	mipi_ddev_t *ddev = file->private_data;
	struct device *dev = ddev->dev;
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_user_t *user = &ddev->user;
	int ret = 0;
#ifdef CONFIG_HOBOT_MIPI_REG_OPERATE
	void __iomem *iomem = mdev->iomem;
	reg_t reg;
	uint32_t regv;
#endif

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != MIPIDEVIOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case MIPIDEVIOC_INIT:
		{
			mipi_dev_cfg_t mipi_dev_cfg;
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("init user mutex lock error");
				return -EINVAL;
			}
			mipiinfo("init cmd: %d %s", user->init_cnt,
					(user->init_cnt) ? "drop" : "real");
			if (!arg || copy_from_user((void *)&mipi_dev_cfg,
				   (void __user *)arg, sizeof(mipi_dev_cfg_t))) {
				mipierr("init error, config %p from user error", (void __user *)arg);
				mutex_unlock(&user->mutex);
				return -EINVAL;
			}
			if (user->init_cnt == 0) {
				if (MIPI_STATE_DEFAULT != mdev->state) {
					mipiinfo("re-init, pre state: %d(%s)",
							 mdev->state, g_md_state[mdev->state]);
				}
				if (copy_from_user((void *)&mipi_dev_cfg,
								   (void __user *)arg, sizeof(mipi_dev_cfg_t))) {
					mipierr("copy data from user failed\n");
					mutex_unlock(&user->mutex);
					return -EFAULT;
				}
				if (0 != (ret = mipi_dev_init(ddev, &mipi_dev_cfg))) {
					mipierr("init error: %d", ret);
					mutex_unlock(&user->mutex);
					return ret;
				}
				mdev->state = MIPI_STATE_INIT;
			} else if (mipi_dev_configure_cmp(&mdev->cfg, &mipi_dev_cfg)) {
				mipiinfo("warning: init config mismatch");
			}
			user->init_cnt++;
			mutex_unlock(&user->mutex);
		}
		break;
	case MIPIDEVIOC_DEINIT:
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
				if (MIPI_STATE_DEFAULT == mdev->state) {
					mipiinfo("has not been init");
					mutex_unlock(&user->mutex);
					break;
				}
				if (MIPI_STATE_START == mdev->state) {
					mipi_dev_stop(ddev);
				}
				mipi_dev_deinit(ddev);
				mdev->state = MIPI_STATE_DEFAULT;
			}
			mutex_unlock(&user->mutex);
		}
		break;
	case MIPIDEVIOC_START:
		{
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("start user mutex lock error");
				return -EINVAL;
			}
			mipiinfo("start cmd: %d %s", user->start_cnt,
					(user->start_cnt) ? "drop" : "real");
			if (user->start_cnt == 0) {
				if (MIPI_STATE_START == mdev->state) {
					mipiinfo("already in start state");
					user->start_cnt++;
					mutex_unlock(&user->mutex);
					break;
				} else if (MIPI_STATE_INIT != mdev->state &&
						   MIPI_STATE_STOP != mdev->state) {
					mipierr("state error, current state: %d(%s)",
							mdev->state, g_md_state[mdev->state]);
					mutex_unlock(&user->mutex);
					return -EBUSY;
				}
				if (0 != (ret = mipi_dev_start(ddev))) {
					mipierr("start error: %d", ret);
					mutex_unlock(&user->mutex);
					return ret;
				}
				mdev->state = MIPI_STATE_START;
			}
			user->start_cnt++;
			mutex_unlock(&user->mutex);
		}
		break;
	case MIPIDEVIOC_STOP:
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
				if (MIPI_STATE_STOP == mdev->state) {
					mipiinfo("already in stop state");
					mutex_unlock(&user->mutex);
					break;
				} else if (MIPI_STATE_START != mdev->state) {
					mipierr("state error, current state: %d(%s)",
							mdev->state, g_md_state[mdev->state]);
					user->start_cnt = start_cnt_save;
					mutex_unlock(&user->mutex);
					return -EBUSY;
				}
				if (0 != (ret = mipi_dev_stop(ddev))) {
					mipierr("stop error: %d", ret);
					user->start_cnt = start_cnt_save;
					mutex_unlock(&user->mutex);
					return ret;
				}
				mdev->state = MIPI_STATE_STOP;
			}
			mutex_unlock(&user->mutex);
		}
		break;
	case MIPIDEVIOC_IPI_GET_INFO:
		{
			mipi_dev_ipi_info_t ipi_info;
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("init user mutex lock error");
				return -EINVAL;
			}
			mipiinfo("ipi get info cmd");
			if (!arg || copy_from_user((void *)&ipi_info,
				   (void __user *)arg, sizeof(mipi_dev_ipi_info_t))) {
				mipierr("ipi get erorr, %p from user error", (void __user *)arg);
				mutex_unlock(&user->mutex);
				return -EINVAL;
			}
			if (user->init_cnt == 0) {
				mipierr("state error: not inited");
				mutex_unlock(&user->mutex);
				return -EACCES;
			}
			if (mipi_dev_ipi_get_info(ddev, &ipi_info)) {
				ret = -EFAULT;
			} else if (copy_to_user((void __user *)arg,
					(void *)&ipi_info, sizeof(mipi_dev_ipi_info_t))) {
				mipierr("ipi get erorr, %p to user error", (void __user *)arg);
				ret = -EINVAL;
			}
			mutex_unlock(&user->mutex);
		}
		break;
	case MIPIDEVIOC_IPI_SET_INFO:
		{
			mipi_dev_ipi_info_t ipi_info;
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("init user mutex lock error");
				return -EINVAL;
			}
			mipiinfo("ipi set info cmd");
			if (!arg || copy_from_user((void *)&ipi_info,
				   (void __user *)arg, sizeof(mipi_dev_ipi_info_t))) {
				mipierr("ipi set erorr, %p from user error", (void __user *)arg);
				mutex_unlock(&user->mutex);
				return -EINVAL;
			}
			if (user->init_cnt == 0) {
				mipierr("state error: not inited");
				mutex_unlock(&user->mutex);
				return -EACCES;
			}
			if (mipi_dev_ipi_set_info(ddev, &ipi_info))
				ret = -EFAULT;
			mutex_unlock(&user->mutex);
		}
		break;
#ifdef CONFIG_HOBOT_MIPI_REG_OPERATE
	case MIPIDEVIOC_READ:
		{
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
	case MIPIDEVIOC_WRITE:
		{
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
	return 0;
}

static const struct file_operations hobot_mipi_dev_fops = {
	.owner		= THIS_MODULE,
	.open		= hobot_mipi_dev_open,
	.release	= hobot_mipi_dev_close,
	.unlocked_ioctl = hobot_mipi_dev_ioctl,
	.compat_ioctl = hobot_mipi_dev_ioctl,
};

/* sysfs dec & add macro */
#define MIPI_DEV_ATTR_DEC(a, m, fsh, fst) \
static struct device_attribute md_##a##_attr = { \
	.attr   = { \
		.name = __stringify(a), \
		.mode = m, \
	}, \
	.show   = fsh, \
	.store  = fst, \
}
#define MIPI_DEV_ATTR_ADD(a) \
	&md_##a##_attr.attr

/* sysfs for mipi dev devices' params */
static int mipi_dev_param_idx(const char *name)
{
	int i;
	int param_num = ARRAY_SIZE(g_md_param_names);

	for (i = 0; i < param_num; i++) {
		if (strcmp(g_md_param_names[i], name) == 0)
			return i;
	}

	return -1;
}

static ssize_t mipi_dev_param_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	mipi_ddev_t *ddev = dev_get_drvdata(dev);
	uint32_t *param = (uint32_t *)(&ddev->mdev.param);
	char *s = buf;
	int idx;

	idx = mipi_dev_param_idx(attr->attr.name);
	if (idx >= 0) {
		s += sprintf(s, "%u\n", param[idx]);
	}

	return (s - buf);
}

static ssize_t mipi_dev_param_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	mipi_ddev_t *ddev = dev_get_drvdata(dev);
	uint32_t *param = (uint32_t *)(&ddev->mdev.param);
	int ret, error = -EINVAL;
	int idx;
	uint32_t val;

	idx = mipi_dev_param_idx(attr->attr.name);
	if (idx >= 0) {
		ret = kstrtouint(buf, 0, &val);
		if (!ret) {
			param[idx] = val;
			error = 0;
		}
	}

	return (error ? error : count);
}

#define MIPI_DEV_PARAM_DEC(a) \
	MIPI_DEV_ATTR_DEC(a, 0644, \
		mipi_dev_param_show, mipi_dev_param_store)
#define MIPI_DEV_PARAM_ADD(a) \
	MIPI_DEV_ATTR_ADD(a)

MIPI_DEV_PARAM_DEC(nocheck);
MIPI_DEV_PARAM_DEC(notimeout);
MIPI_DEV_PARAM_DEC(wait_ms);
MIPI_DEV_PARAM_DEC(dbg_value);
MIPI_DEV_PARAM_DEC(power_instart);
MIPI_DEV_PARAM_DEC(hsync_pkt);
MIPI_DEV_PARAM_DEC(init_retry);
MIPI_DEV_PARAM_DEC(ipi_force);
MIPI_DEV_PARAM_DEC(ipi_limit);
MIPI_DEV_PARAM_DEC(ipi1_dt);
#if MIPIDEV_CHANNEL_NUM >= 2
MIPI_DEV_PARAM_DEC(ipi2_dt);
#endif
#if MIPIDEV_CHANNEL_NUM >= 3
MIPI_DEV_PARAM_DEC(ipi3_dt);
#endif
#if MIPIDEV_CHANNEL_NUM >= 4
MIPI_DEV_PARAM_DEC(ipi4_dt);
#endif
#if MIPI_DEV_INT_DBG
MIPI_DEV_PARAM_DEC(irq_cnt);
MIPI_DEV_PARAM_DEC(irq_debug);
#endif
#ifdef CONFIG_HOBOT_MIPI_PHY
MIPI_DEV_PARAM_DEC(txout_param_valid);
MIPI_DEV_PARAM_DEC(txout_freq_mode);
MIPI_DEV_PARAM_DEC(txout_freq_autolarge_enbale);
MIPI_DEV_PARAM_DEC(txout_freq_gain_precent);
MIPI_DEV_PARAM_DEC(txout_freq_force);
#endif

static struct attribute *param_attr[] = {
	MIPI_DEV_PARAM_ADD(nocheck),
	MIPI_DEV_PARAM_ADD(notimeout),
	MIPI_DEV_PARAM_ADD(wait_ms),
	MIPI_DEV_PARAM_ADD(dbg_value),
	MIPI_DEV_PARAM_ADD(power_instart),
	MIPI_DEV_PARAM_ADD(hsync_pkt),
	MIPI_DEV_PARAM_ADD(init_retry),
	MIPI_DEV_PARAM_ADD(ipi_force),
	MIPI_DEV_PARAM_ADD(ipi_limit),
	MIPI_DEV_PARAM_ADD(ipi1_dt),
#if MIPIDEV_CHANNEL_NUM >= 2
	MIPI_DEV_PARAM_ADD(ipi2_dt),
#endif
#if MIPIDEV_CHANNEL_NUM >= 3
	MIPI_DEV_PARAM_ADD(ipi3_dt),
#endif
#if MIPIDEV_CHANNEL_NUM >= 4
	MIPI_DEV_PARAM_ADD(ipi4_dt),
#endif
#if MIPI_DEV_INT_DBG
	MIPI_DEV_PARAM_ADD(irq_cnt),
	MIPI_DEV_PARAM_ADD(irq_debug),
#endif
#ifdef CONFIG_HOBOT_MIPI_PHY
	MIPI_DEV_PARAM_ADD(txout_param_valid),
	MIPI_DEV_PARAM_ADD(txout_freq_mode),
	MIPI_DEV_PARAM_ADD(txout_freq_autolarge_enbale),
	MIPI_DEV_PARAM_ADD(txout_freq_gain_precent),
	MIPI_DEV_PARAM_ADD(txout_freq_force),
#endif
	NULL,
};

static const struct attribute_group param_attr_group = {
	.name = __stringify(param),
	.attrs = param_attr,
};

/* sysfs for mipi dev devices' status */
static ssize_t mipi_dev_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	mipi_ddev_t *ddev = dev_get_drvdata(dev);
	mipi_dev_t *mdev = &ddev->mdev;
	mipi_user_t *user = &ddev->user;
	mipi_dev_cfg_t *cfg = &mdev->cfg;
	void __iomem *iomem = mdev->iomem;
	char *s = buf;
	int i;

#define MD_STA_SHOW(n, fmt, ...) \
		s += sprintf(s, "%-15s: " fmt "\n", #n, \
			## __VA_ARGS__)
#define MD_REG_SHOW(r) \
		s += sprintf(s, "0x%03x: 0x%08x - %s\n", \
			REG_MIPI_DEV_##r, \
			mipi_getreg(iomem + REG_MIPI_DEV_##r), \
			#r)

	if (strcmp(attr->attr.name, "info") == 0) {
		MD_STA_SHOW(port, "%d", ddev->port);
		MD_STA_SHOW(lane_mode, "%d(%s)", ddev->lane_mode,
			(ddev->lane_mode) ? "dsi" : "csi");
		MD_STA_SHOW(iomem, "%p", mdev->iomem);
#if MIPI_DEV_INT_DBG && defined MIPI_DEV_INT_USE_TIMER
		MD_STA_SHOW(timer, "%s",
			(ddev->irq_timer_en) ? "enable" : "disable");
#else
		MD_STA_SHOW(irq, "%d", mdev->irq);
#endif
		MD_STA_SHOW(state, "%d(%s)", mdev->state,
			g_md_state[mdev->state]);
		MD_STA_SHOW(ipi_clock, "%llu", ddev->ipi_clock);
	} else if (strcmp(attr->attr.name, "cfg") == 0) {
		if (mdev->state >= MIPI_STATE_INIT) {
			MD_STA_SHOW(lane, "%d", cfg->lane);
			MD_STA_SHOW(datatype, "0x%02x", cfg->datatype);
			MD_STA_SHOW(fps, "%d", cfg->fps);
			MD_STA_SHOW(mclk, "%d MHz", cfg->mclk);
			MD_STA_SHOW(mipiclk, "%d Mbps", cfg->mipiclk);
			MD_STA_SHOW(width, "%d", cfg->width);
			MD_STA_SHOW(height, "%d", cfg->height);
			MD_STA_SHOW(linelenth, "%d", cfg->linelenth);
			MD_STA_SHOW(framelenth, "%d", cfg->framelenth);
			MD_STA_SHOW(settle, "%d", cfg->settle);
			MD_STA_SHOW(vpg, "%d", cfg->vpg);
			MD_STA_SHOW(ipi_lines, "%d", cfg->ipi_lines);
		} else {
			s += sprintf(s, "not inited\n" );
		}
	} else if (strcmp(attr->attr.name, "regs") == 0) {
		if (iomem) {
			MD_REG_SHOW(VERSION);
			MD_REG_SHOW(CSI2_RESETN);
			MD_REG_SHOW(INT_ST_MAIN);
			MD_REG_SHOW(INT_ST_VPG);
			MD_REG_SHOW(INT_ST_IDI);
			MD_REG_SHOW(INT_ST_IPI);
			MD_REG_SHOW(INT_ST_PHY);
			MD_REG_SHOW(INT_ST_IDI_VCX);
			MD_REG_SHOW(INT_ST_IDI_VCX2);
			MD_REG_SHOW(INT_ST_MT_IPI);
			MD_REG_SHOW(INT_MASK_N_VPG);
			MD_REG_SHOW(INT_MASK_N_IDI);
			MD_REG_SHOW(INT_MASK_N_IPI);
			MD_REG_SHOW(INT_MASK_N_PHY);
			MD_REG_SHOW(INT_MASK_N_IDI_VCX);
			MD_REG_SHOW(INT_MASK_N_IDI_VCX2);
			MD_REG_SHOW(INT_MASK_N_MT_IPI);
			if (cfg->vpg) {
				MD_REG_SHOW(VPG_CTRL);
				MD_REG_SHOW(VPG_STATUS);
				MD_REG_SHOW(VPG_MODE_CFG);
				MD_REG_SHOW(VPG_PKT_CFG);
				MD_REG_SHOW(VPG_PKT_SIZE);
				MD_REG_SHOW(VPG_HSA_TIME);
				MD_REG_SHOW(VPG_HBP_TIME);
				MD_REG_SHOW(VPG_HLINE_TIME);
				MD_REG_SHOW(VPG_VSA_LINES);
				MD_REG_SHOW(VPG_VBP_LINES);
				MD_REG_SHOW(VPG_VFP_LINES);
				MD_REG_SHOW(VPG_ACT_LINES);
				MD_REG_SHOW(VPG_BK_LINES);
				MD_REG_SHOW(VPG_MAX_FRAME_NUM);
				MD_REG_SHOW(VPG_START_LINE_NUM);
				MD_REG_SHOW(VPG_STEP_LINE_NUM);
			}
			MD_REG_SHOW(PHY_RSTZ);
			MD_REG_SHOW(PHY_IF_CFG);
			MD_REG_SHOW(LPCLK_CTRL);
			MD_REG_SHOW(PHY_ULPS_CTRL);
			MD_REG_SHOW(CLKMGR_CFG);
			MD_REG_SHOW(PHY_TX_TRIGGERS);
			MD_REG_SHOW(PHY_CAL);
			MD_REG_SHOW(TO_CNT_CFG);
			MD_REG_SHOW(PHY_STATUS);
			MD_REG_SHOW(IDI_FIFO_STATUS);
			MD_REG_SHOW(IPI_INSERT_CTRL);
			if (cfg->channel_num >= 0 && cfg->channel_sel[0] >= 0) {
				MD_REG_SHOW(IPI_PKT_CFG);
				MD_REG_SHOW(IPI_PIXELS);
				MD_REG_SHOW(IPI_MAX_FRAME_NUM);
				MD_REG_SHOW(IPI_START_LINE_NUM);
				MD_REG_SHOW(IPI_STEP_LINE_NUM);
				MD_REG_SHOW(IPI_LINES);
				MD_REG_SHOW(IPI_DATA_SEND_START);
				MD_REG_SHOW(IPI_FIFO_STATUS);
				MD_REG_SHOW(IPI_TRANS_STATUS);
				MD_REG_SHOW(IPI_HSA_HBP_PPI_TIME);
				MD_REG_SHOW(IPI_HLINE_PPI_TIME);
				MD_REG_SHOW(IPI_VSA_LINES);
				MD_REG_SHOW(IPI_VBP_LINES);
				MD_REG_SHOW(IPI_VFP_LINES);
				MD_REG_SHOW(IPI_ACT_LINES);
				MD_REG_SHOW(IPI_FB_LINES);
				MD_REG_SHOW(IPI1_HSA_HBP_TIME);
				MD_REG_SHOW(IPI1_LP_TIME);
			}
			if (cfg->channel_num > 1 && cfg->channel_sel[1] >= 0) {
				MD_REG_SHOW(IPI2_PKT_CFG);
				MD_REG_SHOW(IPI2_PIXELS);
				MD_REG_SHOW(IPI2_MAX_FRAME_NUM);
				MD_REG_SHOW(IPI2_START_LINE_NUM);
				MD_REG_SHOW(IPI2_STEP_LINE_NUM);
				MD_REG_SHOW(IPI2_LINES);
				MD_REG_SHOW(IPI2_DATA_SEND_START);
				MD_REG_SHOW(IPI2_FIFO_STATUS);
				MD_REG_SHOW(IPI2_HSA_HBP_TIME);
				MD_REG_SHOW(IPI2_LP_TIME);
			}
			if (cfg->channel_num > 2 && cfg->channel_sel[2] >= 0) {
				MD_REG_SHOW(IPI3_PKT_CFG);
				MD_REG_SHOW(IPI3_PIXELS);
				MD_REG_SHOW(IPI3_MAX_FRAME_NUM);
				MD_REG_SHOW(IPI3_START_LINE_NUM);
				MD_REG_SHOW(IPI3_STEP_LINE_NUM);
				MD_REG_SHOW(IPI3_LINES);
				MD_REG_SHOW(IPI3_DATA_SEND_START);
				MD_REG_SHOW(IPI3_FIFO_STATUS);
				MD_REG_SHOW(IPI3_HSA_HBP_TIME);
				MD_REG_SHOW(IPI3_LP_TIME);
			}
			if (cfg->channel_num > 3 && cfg->channel_sel[3] >= 0) {
				MD_REG_SHOW(IPI4_PKT_CFG);
				MD_REG_SHOW(IPI4_PIXELS);
				MD_REG_SHOW(IPI4_MAX_FRAME_NUM);
				MD_REG_SHOW(IPI4_START_LINE_NUM);
				MD_REG_SHOW(IPI4_STEP_LINE_NUM);
				MD_REG_SHOW(IPI4_LINES);
				MD_REG_SHOW(IPI4_DATA_SEND_START);
				MD_REG_SHOW(IPI4_FIFO_STATUS);
				MD_REG_SHOW(IPI4_HSA_HBP_TIME);
				MD_REG_SHOW(IPI4_LP_TIME);
			}
			if (cfg->channel_num > 1) {
				MD_REG_SHOW(MT_IPI_CFG);
				MD_REG_SHOW(MT_IPI_DF_TIME);
				MD_REG_SHOW(MT_IPI_FIFO_STATUS);
				MD_REG_SHOW(MT_IPI1_TRANS_CFG);
				MD_REG_SHOW(MT_IPI2_TRANS_CFG);
				MD_REG_SHOW(MT_IPI3_TRANS_CFG);
				MD_REG_SHOW(MT_IPI4_TRANS_CFG);
			}
		} else {
			s += sprintf(s, "not ioremap\n" );
		}
	} else if (strcmp(attr->attr.name, "user") == 0) {
		MD_STA_SHOW(user, "%d", user->open_cnt);
		MD_STA_SHOW(init, "%d", user->init_cnt);
		MD_STA_SHOW(start, "%d", user->start_cnt);
#if MIPI_DEV_INT_DBG
	} else if (strcmp(attr->attr.name, "icnt") == 0) {
		for (i = 0; i < sizeof(mdev->icnt)/sizeof(uint32_t); i++) {
			s += sprintf(s, "%-15s: %u\n", g_md_icnt_names[i],
						 ((uint32_t *)(&mdev->icnt))[i]);
		}
#endif
	}

	return (s - buf);
}

#define MIPI_DEV_STATUS_DEC(a) \
	MIPI_DEV_ATTR_DEC(a, 0444, mipi_dev_status_show, NULL)
#define MIPI_DEV_STATUS_ADD(a) \
	MIPI_DEV_ATTR_ADD(a)

MIPI_DEV_STATUS_DEC(info);
MIPI_DEV_STATUS_DEC(cfg);
MIPI_DEV_STATUS_DEC(regs);
MIPI_DEV_STATUS_DEC(user);
#if MIPI_DEV_INT_DBG
MIPI_DEV_STATUS_DEC(icnt);
#endif

static struct attribute *status_attr[] = {
	MIPI_DEV_STATUS_ADD(info),
	MIPI_DEV_STATUS_ADD(cfg),
	MIPI_DEV_STATUS_ADD(regs),
	MIPI_DEV_STATUS_ADD(user),
#if MIPI_DEV_INT_DBG
	MIPI_DEV_STATUS_ADD(icnt),
#endif
	NULL,
};

static const struct attribute_group status_attr_group = {
	.name = __stringify(status),
	.attrs = status_attr,
};

#if MIPI_DEV_INT_DBG && MIPI_DEV_SYSFS_FATAL_EN
/* sysfs for mipi dev devices' fatal */
static const mipi_dev_ireg_t* mipi_dev_get_ireg(mipi_dev_ierr_t *ierr, const char *name)
{
	const mipi_dev_ireg_t *ireg = NULL, *itmp = NULL;
	int i;

	for (i = 0; i < ierr->num; i++) {
		itmp = &ierr->iregs[i];
		if (itmp->icnt_n < ARRAY_SIZE(g_md_icnt_names) &&
			strcmp(g_md_icnt_names[itmp->icnt_n], name) == 0) {
			ireg = itmp;
			break;
		}
	}

	return ireg;
}

static ssize_t mipi_dev_fatal_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	mipi_ddev_t *ddev = dev_get_drvdata(dev);
	mipi_dev_ierr_t *ierr = &ddev->mdev.ierr;
	const mipi_dev_ireg_t *ireg = NULL;
	void __iomem *iomem = ddev->mdev.iomem;
	char *s = buf;
	uint32_t reg = 0;

	if (strcmp(g_md_icnt_names[0], attr->attr.name) == 0) {
		reg = REG_MIPI_DEV_INT_ST_MAIN;
	} else {
		ireg = mipi_dev_get_ireg(ierr, attr->attr.name);
		if (ireg)
			reg = ireg->reg_st;
	}
	if (reg > 0)
		s += sprintf(s, "0x%08x\n", mipi_getreg(iomem + reg));

	return (s - buf);
}

static ssize_t mipi_dev_fatal_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	mipi_ddev_t *ddev = dev_get_drvdata(dev);
	mipi_dev_ierr_t *ierr = &ddev->mdev.ierr;
	const mipi_dev_ireg_t *ireg = NULL;
	void __iomem *iomem = ddev->mdev.iomem;
	int ret, error = -EINVAL;
	uint32_t val;
	int i;

	ret = kstrtouint(buf, 0, &val);
	if (ret) {
		return error;
	}

	if (strcmp(g_md_icnt_names[0], attr->attr.name) == 0) {
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
		ireg = mipi_dev_get_ireg(ierr, attr->attr.name);
		if (ireg) {
			mipi_putreg(iomem + ireg->reg_force, val);
			error = 0;
		}
	}

	return (error ? error : count);
}

#define MIPI_DEV_FATAL_DEC(a) \
	MIPI_DEV_ATTR_DEC(a, 0644, \
		mipi_dev_fatal_show, mipi_dev_fatal_store)
#define MIPI_DEV_FATAL_ADD(a) \
	MIPI_DEV_ATTR_ADD(a)

MIPI_DEV_FATAL_DEC(st_main);
MIPI_DEV_FATAL_DEC(vpg);
MIPI_DEV_FATAL_DEC(idi);
MIPI_DEV_FATAL_DEC(ipi);
MIPI_DEV_FATAL_DEC(phy);
MIPI_DEV_FATAL_DEC(mt_ipi);
MIPI_DEV_FATAL_DEC(idi_vcx);
MIPI_DEV_FATAL_DEC(idi_vcx2);

static struct attribute *fatal_attr[] = {
	MIPI_DEV_FATAL_ADD(st_main),
	MIPI_DEV_FATAL_ADD(vpg),
	MIPI_DEV_FATAL_ADD(idi),
	MIPI_DEV_FATAL_ADD(ipi),
	MIPI_DEV_FATAL_ADD(phy),
	MIPI_DEV_FATAL_ADD(mt_ipi),
	MIPI_DEV_FATAL_ADD(idi_vcx),
	MIPI_DEV_FATAL_ADD(idi_vcx2),
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
#if MIPI_DEV_INT_DBG && MIPI_DEV_SYSFS_FATAL_EN
	&fatal_attr_group,
#endif
	NULL,
};

static int hobot_mipi_dev_class_get(void)
{
	int ret = 0;

	if (!g_md_class) {
#ifdef CONFIG_HOBOT_MIPI_PHY
		g_md_class = mipi_dphy_class();
		if (!g_md_class) {
			pr_err("[%s] dphy class null\n", __func__);
			ret = -ENODEV;
		}
#else
		g_md_class = class_create(THIS_MODULE, MIPI_DEV_DNAME);
		if (IS_ERR(g_md_class)) {
			ret = PTR_ERR(g_md_class);
			g_md_class = NULL;
			pr_err("[%s] class error %d\n", __func__,
					ret);
		}
#endif
	}
	return ret;
}

static int hobot_mipi_dev_class_put(void)
{
#ifndef CONFIG_HOBOT_MIPI_PHY
	if (g_md_class)
		class_destroy(g_md_class);
#endif
	g_md_class = NULL;
	return 0;
}

static int hobot_mipi_dev_probe_cdev(mipi_ddev_t *ddev)
{
	int ret = 0;
	dev_t devno;
	struct cdev *p_cdev;

	if (g_md_major == 0) {
		ret = alloc_chrdev_region(&devno, 0, MIPI_DEV_MAX_NUM, MIPI_DEV_DNAME);
		if (ret < 0) {
			pr_err("[%s] alloc chrdev %s error %d\n", __func__,
					MIPI_DEV_DNAME, ret);
			return ret;
		}
		g_md_major = MAJOR(devno);
	}
	ret = hobot_mipi_dev_class_get();
	if (ret)
		return ret;
	devno = MKDEV(g_md_major, ddev->port);
	ddev->devno = devno;
	p_cdev = &ddev->cdev;
	cdev_init(p_cdev, &hobot_mipi_dev_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		pr_err("[%s] cdev add error %d\n", __func__, ret);
		goto err_add;
	}
	ddev->dev = device_create_with_groups(g_md_class, NULL, devno,
			(void *)ddev, attr_groups,
			"%s%d", MIPI_DEV_DNAME, ddev->port);
	if (IS_ERR(ddev->dev)) {
		ret = PTR_ERR(ddev->dev);
		ddev->dev = NULL;
		pr_err("[%s] deivce create error %d\n", __func__, ret);
		goto err_creat;
	}

#ifdef CONFIG_HOBOT_DIAG
	/* diag */
	if (diag_register(ModuleDiag_VIO, EventIdVioMipiDevErr,
			sizeof(mipi_dev_icnt_t), DIAG_MSG_INTERVAL_MIN,
			DIAG_MSG_INTERVAL_MAX, mipi_dev_diag_test) < 0) {
		pr_err("mipi dev %d diag register fail\n", ddev->port);
	} else {
		ddev->last_err_tm_ms = 0;
		init_timer(&ddev->diag_timer);
		ddev->diag_timer.expires =
			get_jiffies_64() + msecs_to_jiffies(1000);
		ddev->diag_timer.data = (unsigned long)ddev;
		ddev->diag_timer.function = mipi_dev_diag_timer_func;
		add_timer(&ddev->diag_timer);
	}
#endif
	mutex_init(&ddev->user.open_mutex);
	ddev->user.open_cnt = 0;

	return 0;
err_creat:
	cdev_del(&ddev->cdev);
err_add:
	return ret;
}

static int hobot_mipi_dev_remove_cdev(mipi_ddev_t *ddev)
{
#ifdef CONFIG_HOBOT_DIAG
	del_timer_sync(&ddev->diag_timer);
#endif
	if (g_md_class)
		device_destroy(g_md_class, ddev->devno);
	cdev_del(&ddev->cdev);

	if (port_num <= 1) {
		hobot_mipi_dev_class_put();
		if (g_md_major) {
			unregister_chrdev_region(MKDEV(g_md_major, 0),
				MIPI_DEV_MAX_NUM);
			g_md_major = 0;
		}
	}
	ddev->dev = NULL;
	return 0;
}

static int hobot_mipi_dev_phy_register(mipi_ddev_t *ddev)
{
	int ret = 0;

#ifdef CONFIG_HOBOT_MIPI_PHY
	mipi_phy_sub_t sub;

	sub.iomem = ddev->mdev.iomem;
	sub.dev = ddev->dev;
	sub.param = &ddev->mdev.param.phy;
	sub.port = ddev->port;
	ret = mipi_dphy_register(MIPI_DPHY_TYPE_DEV, ddev->port, &sub);
#else
	struct device *dev = ddev->dev;

	mipiinfo("no dphy driver");
#endif

	return ret;
}

static int hobot_mipi_dev_phy_unregister(mipi_ddev_t *ddev)
{
#ifdef CONFIG_HOBOT_MIPI_PHY
	return mipi_dphy_unregister(1, ddev->port);
#else
	return 0;
#endif
}

#ifdef MODULE
static int hobot_mipi_dev_remove_param(void)
{
	mipi_ddev_t *ddev;
	mipi_dev_t *mdev;
	int i;

	for (i = 0; i < MIPI_DEV_MAX_NUM; i ++) {
		ddev = g_ddev[i];
		if (!ddev) {
			continue;
		}
		mdev = &ddev->mdev;

#if MIPI_DEV_INT_DBG && defined MIPI_DEV_INT_USE_TIMER
		del_timer_sync(&ddev->irq_timer);
#endif
		hobot_mipi_dev_remove_cdev(ddev);
		if (mdev->iomem)
			iounmap(mdev->iomem);
		hobot_mipi_dev_phy_unregister(ddev);
		kfree(ddev);
		g_ddev[i] = NULL;
		port_num--;
	}
	return 0;
}

static int hobot_mipi_dev_probe_param(void)
{
	mipi_ddev_t *ddev;
	mipi_dev_t *mdev;
	mipi_dev_param_t *param;
	unsigned long reg_addr_dev;
	int i;
	int ret = 0;

	if (init_num <= 0)
		return -ENODEV;

	if (init_num > MIPI_DEV_MAX_NUM)
		init_num = MIPI_DEV_MAX_NUM;

	port_num = 0;
	for (i = 0; i < init_num; i++) {
		ddev = kmalloc(sizeof(mipi_ddev_t), GFP_KERNEL);
		if (ddev == NULL) {
		pr_err("[%s] malloc failed\n", __func__);
			ret = -ENOMEM;
			goto err_kmalloc;
		}
		memset(ddev, 0x0, sizeof(mipi_ddev_t));
		mdev = &ddev->mdev;
		param = &mdev->param;
		reg_addr_dev = reg_addr + (reg_size * i);
		mdev->iomem = ioremap_nocache(reg_addr_dev, reg_size);
		if (IS_ERR(mdev->iomem)) {
			pr_err("[%s] ioremap error\n", __func__);
			ret = PTR_ERR(mdev->iomem);
			mdev->iomem = NULL;
			goto err_ioremap;
		}
		ddev->port = i;
		ddev->lane_mode = mipi_dphy_get_lanemode(MIPI_DPHY_TYPE_DEV,
								ddev->port);
		ret = hobot_mipi_dev_probe_cdev(ddev);
		if (ret) {
			goto err_cdev;
		}

#if MIPI_DEV_INT_DBG && defined MIPI_DEV_INT_USE_TIMER
		init_timer(&ddev->irq_timer);
		ddev->irq_timer.data = (unsigned long)ddev;
		ddev->irq_timer.function = mipi_dev_irq_timer_func;
		add_timer(&ddev->irq_timer);
		param->irq_cnt = MIPI_DEV_IRQ_CNT;
		param->irq_debug = MIPI_DEV_IRQ_DEBUG;
		mdev->ierr.iregs = md_int_regs_1p2;
		mdev->ierr.num = ARRAY_SIZE(md_int_regs_1p2);
#else
		pr_info("[%s] no int timer\n", __func__);
#endif
#ifdef ADJUST_CLK_RECALCULATION
		param->power_instart = 1;
#endif
		param->hsync_pkt = MIPI_DEV_HSYNC_PKT_DEFAULT;
		param->ipi_limit = MIPI_DEV_IPILIMIT_DEFAULT;
		param->init_retry = MIPI_DEV_INIT_RETRY_DEFAULT;
		param->wait_ms = DEV_DPHY_CHECK_MAX;

		hobot_mipi_dev_phy_register(ddev);
		g_ddev[i] = ddev;
		port_num ++;

		ret = mipi_getreg(mdev->iomem + REG_MIPI_DEV_VERSION);
		dev_info(ddev->dev, "ver %c%c%c%c port%d\n",
				 (ret >> 24), (ret >> 16), (ret >> 8), ret, i);
	}
	return 0;
err_cdev:
	if (mdev && mdev->iomem)
		iounmap(mdev->iomem);
err_ioremap:
	if (ddev)
		kfree(ddev);
err_kmalloc:
	hobot_mipi_dev_remove_param();
	return ret;
}

#else
static int hobot_mipi_dev_remove(struct platform_device *pdev)
{
	mipi_ddev_t *ddev = platform_get_drvdata(pdev);
	mipi_dev_t *mdev = &ddev->mdev;

	hobot_mipi_dev_remove_cdev(ddev);
#if MIPI_DEV_INT_DBG
#ifdef MIPI_DEV_INT_USE_TIMER
	del_timer_sync(&ddev->irq_timer);
#else
	free_irq(mdev->irq, ddev);
#endif
#endif
	devm_iounmap(&pdev->dev, mdev->iomem);
	hobot_mipi_dev_phy_unregister(ddev);
	devm_kfree(&pdev->dev, ddev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static int hobot_mipi_dev_probe(struct platform_device *pdev)
{
	mipi_ddev_t *ddev;
	mipi_dev_t *mdev;
	mipi_dev_param_t *param;
	int port, ret = 0;
	struct resource *res;

	port = of_alias_get_id(pdev->dev.of_node, "mipidev");
	if (port >= MIPI_DEV_MAX_NUM) {
		pr_err("[%s] port %d >= %d overflow error\n", __func__, port,
				MIPI_DEV_MAX_NUM);
		return -ERANGE;
	} else {
		if (port < 0)
			port = 0;
		if (g_ddev[port]) {
			pr_err("[%s] port %d duplicate error\n", __func__, port);
			return -EEXIST;
		}
	}
	ddev = devm_kmalloc(&pdev->dev, sizeof(mipi_ddev_t), GFP_KERNEL);
	if (ddev == NULL) {
		pr_err("[%s] malloc failed\n", __func__);
		return -ENOMEM;
	}
	memset(ddev, 0x0, sizeof(mipi_ddev_t));
	mdev = &ddev->mdev;
	param = &mdev->param;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mdev->iomem = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mdev->iomem)) {
		pr_err("[%s] get mem res error\n", __func__);
		ret = PTR_ERR(mdev->iomem);
		mdev->iomem = NULL;
		goto err_ioremap;
	}

#if MIPI_DEV_INT_DBG
#ifdef MIPI_DEV_INT_USE_TIMER
	init_timer(&ddev->irq_timer);
	ddev->irq_timer.data = (unsigned long)ddev;
	ddev->irq_timer.function = mipi_dev_irq_timer_func;
	add_timer(&ddev->irq_timer);
#else
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		pr_err("[%s] get irq res error\n", __func__);
		ret = -ENODEV;
		goto err_irq;
	}
	mdev->irq = res->start;
	ret = request_threaded_irq(mdev->irq,
							   mipi_dev_irq_func,
							   NULL,
							   IRQF_TRIGGER_HIGH,
							   dev_name(&pdev->dev),
							   ddev);
	if (ret) {
		pr_err("[%s] request irq error %d\n", __func__, ret);
		goto err_irq;
	}
#endif
	param->irq_cnt = MIPI_DEV_IRQ_CNT;
	param->irq_debug = MIPI_DEV_IRQ_DEBUG;
	mdev->ierr.iregs = md_int_regs_1p2;
	mdev->ierr.num = ARRAY_SIZE(md_int_regs_1p2);
#endif
#ifdef ADJUST_CLK_RECALCULATION
	param->power_instart = 1;
#endif
	param->hsync_pkt = MIPI_DEV_HSYNC_PKT_DEFAULT;
	param->ipi_limit = MIPI_DEV_IPILIMIT_DEFAULT;
	param->init_retry = MIPI_DEV_INIT_RETRY_DEFAULT;
	param->wait_ms = DEV_DPHY_CHECK_MAX;

	platform_set_drvdata(pdev, ddev);

	ddev->port = port;
	ddev->lane_mode = mipi_dphy_get_lanemode(MIPI_DPHY_TYPE_DEV,
								ddev->port);
	ret = hobot_mipi_dev_probe_cdev(ddev);
	if (ret) {
		goto err_cdev;
	}

	hobot_mipi_dev_phy_register(ddev);
	g_ddev[port] = ddev;
	port_num ++;

	ret = mipi_getreg(mdev->iomem + REG_MIPI_DEV_VERSION);
	dev_info(ddev->dev, "ver %c%c%c%c port%d\n",
		(ret >> 24), (ret >> 16), (ret >> 8), ret, port);
	return 0;
err_cdev:
#if MIPI_DEV_INT_DBG
#ifdef MIPI_DEV_INT_USE_TIMER
	del_timer_sync(&ddev->irq_timer);
#else
	free_irq(mdev->irq, ddev);
#endif
err_irq:
#endif
	devm_iounmap(&pdev->dev, mdev->iomem);
err_ioremap:
	devm_kfree(&pdev->dev, ddev);
	platform_set_drvdata(pdev, NULL);
	return ret;
}

#ifdef CONFIG_PM_SLEEP
int hobot_mipi_dev_suspend(struct device *dev)
{
	mipi_ddev_t *ddev = dev_get_drvdata(dev);

	if (pm_suspend_target_state == PM_SUSPEND_TO_IDLE)
		return 0;

	mipiinfo("%s:%s enter suspend...", __FILE__, __func__);

	mipi_dev_stop(ddev);
	mipi_dev_deinit(ddev);

	return 0;
}

int hobot_mipi_dev_resume(struct device *dev)
{
	mipi_ddev_t *ddev = dev_get_drvdata(dev);
	mipi_dev_t *mdev = &ddev->mdev;
	int ret = 0;

	if (pm_suspend_target_state == PM_SUSPEND_TO_IDLE)
		return 0;

	mipiinfo("%s:%s enter resume...", __FILE__, __func__);

	if (mdev->state == MIPI_STATE_DEFAULT) {
		mipi_dev_stop(ddev);
		mipi_dev_deinit(ddev);
	} else if (mdev->state == MIPI_STATE_START) {
		/* if state == MIPI_STATE_START, it has been initialized. */
		if (0 != (ret = mipi_dev_init(ddev, &mdev->cfg))) {
			mipierr("[%s] init error %d", __func__, ret);
			return ret;
		}

		/* start again */
		if (0 != (ret = mipi_dev_start(ddev))) {
			mipierr("[%s] start error %d", __func__, ret);
			return ret;
		}
	} else if (mdev->state == MIPI_STATE_STOP) {
		/* if state == MIPI_STATE_STOP, it has been initialized. */
		if (0 != (ret = mipi_dev_init(ddev, &mdev->cfg))) {
			mipierr("[%s] init error %d", __func__, ret);
			return ret;
		}
		mipi_dev_stop(ddev);
	} else if (mdev->state == MIPI_STATE_INIT) {
		if (0 != (ret = mipi_dev_init(ddev, &mdev->cfg))) {
			mipierr("[%s] init error %d", __func__, ret);
			return ret;
		}
	}

	return ret;
}

static const struct dev_pm_ops hobot_mipi_dev_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hobot_mipi_dev_suspend,
			hobot_mipi_dev_resume)
};
#endif

static const struct of_device_id hobot_mipi_dev_match[] = {
	{.compatible = "hobot,mipi-dev"},
	{}
};

MODULE_DEVICE_TABLE(of, hobot_mipi_dev_match);

static struct platform_driver hobot_mipi_dev_driver = {
	.probe	= hobot_mipi_dev_probe,
	.remove = hobot_mipi_dev_remove,
	.driver = {
		.name = MIPI_DEV_DNAME,
		.of_match_table = hobot_mipi_dev_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &hobot_mipi_dev_dev_pm_ops,
#endif
	},
};
#endif

static int __init hobot_mipi_dev_module_init(void)
{
	int           ret = 0;

#ifdef MODULE
	ret = hobot_mipi_dev_probe_param();
#else
	ret = platform_driver_register(&hobot_mipi_dev_driver);
#endif

	return ret;
}

static void __exit hobot_mipi_dev_module_exit(void)
{
#ifdef MODULE
	hobot_mipi_dev_remove_param();
#else
	platform_driver_unregister(&hobot_mipi_dev_driver);
#endif
}

late_initcall_sync(hobot_mipi_dev_module_init);
module_exit(hobot_mipi_dev_module_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Zhang Tianyu <tianyu.zhang@hobot.cc>");
MODULE_DESCRIPTION("HOBOT MIPI Dev Driver");
