/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
/**
 * @file	 hobot_mipi_host.c
 * @brief	 Hobot MIPI HOST controller driver
 * @author	 tarryzhang (tianyu.zhang@hobot.cc)
 * @date	 2017/7/6
 * @version  V1.0
 * @par		 Horizon Robotics
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

#include <soc/hobot/hobot_mipi_host.h>
#include <soc/hobot/hobot_mipi_dphy.h>

#include "hobot_mipi_host_regs.h"
#include "hobot_mipi_utils.h"

#ifdef CONFIG_X2_IPS
#include "x2/x2_ips.h"
#endif

#ifdef CONFIG_X2_DIAG
#include <x2/diag.h>

#define EventIdVioMipiHostError 80
#endif

#define MIPI_HOST_DNAME		"mipi_host"
#define MIPI_HOST_MAX_NUM	CONFIG_HOBOT_MIPI_HOST_MAX_NUM
#define MIPI_HOST_CFGCLK_NAME	"mipi_cfg_host"
#define MIPI_HOST_CFGCLK_MHZ	24000000UL
#define MIPI_HOST_REFCLK_NAME	"mipi_host_ref"
#define MIPI_HOST_REFCLK_MHZ	24000000UL
#define MIPI_HOST_IPICLK_NAME(x)	"mipi_rx" __stringify(x) "_ipi"
#define MIPI_HOST_SNRCLK_NAME(x)	"sensor" __stringify(x) "_mclk"

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
module_param(reg_addr, uint, 0644);
module_param(reg_size, uint, 0644);
module_param(init_num, uint, 0644);
#endif

/* only hobot platform driver use irq */
#if defined MODULE || !defined CONFIG_ARCH_HOBOT
#define MIPI_HOST_INT_USE_TIMER
#endif

#define MIPI_HOST_INT_DBG		   (1)

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
#define MIPI_HOST_BITWIDTH_48	   (0)
#define MIPI_HOST_BITWIDTH_16	   (1)
#define MIPI_HOST_BITWIDTH_OFFSET  (8)
#define MIPI_HOST_MEMFLUSN_ENABLE  (0x01 << 8)
#define MIPI_HOST_EMB_DATA		   (0x01 << 8)
#define MIPI_HOST_CUT_THROUGH	   (0x01 << 16)
#define MIPI_HOST_IPI_ENABLE	   (0x01 << 24)
#define MIPI_HOST_LEGCYMODE_ENABLE (0x01 << 24)
#define MIPI_HOST_HSATIME		   (0x04)
#define MIPI_HOST_HBPTIME		   (0x04)
#define MIPI_HOST_HSDTIME		   (0x5f4)
#define MIPI_HOST_CFGCLK_DEFAULT   (0x1C)

#define MIPI_HOST_ADV_DEFAULT      (0x3 << 16)
#define MIPI_HOST_CUT_DEFAULT      (1)
#define MIPI_HOST_IPILIMIT_DEFAULT (102000000UL)
#define MIPI_HOST_IRQ_CNT          (10)
#define MIPI_HOST_IRQ_DEBUG        (1)
#define MIPI_HOST_SNRCLK_FREQ_MIN  (9281250UL)

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

#define MIPIHOSTIOC_READ		_IOWR(MIPIHOSTIOC_MAGIC, 4, reg_t)
#define MIPIHOSTIOC_WRITE		_IOW(MIPIHOSTIOC_MAGIC, 5, reg_t)
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
	struct pinctrl       *pinctrl;
	struct pinctrl_state *enable;
	struct pinctrl_state *disable;
} mipi_host_snrclk_t;

typedef struct _mipi_host_param_s {
	/* type must be: uint32_t */
	uint32_t nocheck;
	uint32_t notimeout;
	uint32_t dbg_value;
	uint32_t adv_value;
	uint32_t need_stop_check;
	uint32_t stop_check_instart;
	uint32_t cut_through;
	uint32_t ipi_force;
	uint32_t ipi_limit;
	uint32_t snrclk_en;
	uint32_t snrclk_freq;
#if MIPI_HOST_INT_DBG
	uint32_t irq_cnt;
	uint32_t irq_debug;
#endif
} mipi_host_param_t;

static const char *g_mh_param_names[] = {
	"nocheck",
	"notimeout",
	"dbg_value",
	"adv_value",
	"need_stop_check",
	"stop_check_instart",
	"cut_through",
	"ipi_force",
	"ipi_limit",
	"snrclk_en",
	"snrclk_freq",
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
#endif

typedef struct _mipi_host_s {
	void __iomem     *iomem;
	int			      irq;
	mipi_state_t      state;
	mipi_host_cfg_t   cfg;
	mipi_host_param_t param;
	mipi_host_snrclk_t snrclk;
#if MIPI_HOST_INT_DBG
	mipi_host_icnt_t icnt;
#endif
} mipi_host_t;

typedef struct _mipi_user_s {
	spinlock_t slock;
	struct mutex mutex;
	uint32_t open_cnt;
	uint32_t init_cnt;
	uint32_t start_cnt;
} mipi_user_t;

typedef struct _mipi_hdev_s {
	int               port;
	int               lane_mode;
	dev_t             devno;
	struct cdev       cdev;
	struct device    *dev;
	void             *ex_hdev;
	int               is_ex;
	mipi_host_t       host;
	mipi_user_t       user;
#ifdef CONFIG_X2_DIAG
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

/*
 * mipi host port num to group and index:
 * port	group	index  lane  ipi
 * 0	0		0	   2/4   4
 * 1	1		0      2/0   2
 * 2	0		1      2/4   4
 * 3	1		1      2/0   2
 * ...
 */
#define mipi_port_group(p)	((((p) / 4) * 2) + ((p) % 2))
#define mipi_port_index(p)	(((p) % 4) / 2)
#define mipi_port_lane(p,m)	(mipi_port_index(p) ? \
								((m) ? 0 : 2) : ((m) ? 4 : 2))
#define mipi_port_ipi(p)	(mipi_port_index(p) ? 2 : 4)

/**
 * @brief mipi_host_configure_lanemode: configure dphy for lane mode
 *
 * @param [in] lane : mipi host lane
 *
 * @return int32_t: 0/-1
 */
static int32_t mipi_host_configure_lanemode(mipi_hdev_t *hdev, int lane)
{
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	int group, index, poth;
	int i, ret, target_mode = -1;

	if (!hdev)
		return -1;

	for (i = 0; i < 2; i++) {
		if (lane <= mipi_port_lane(hdev->port, i)) {
			target_mode = i;
			break;
		}
	}

	group = mipi_port_group(hdev->port);
	index = mipi_port_index(hdev->port);
	poth = (index) ? (hdev->port - 2) : (hdev->port + 2);
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
	if (lane > 2) {
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
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	mipi_host_param_t *param = &host->param;
	void __iomem *iomem = host->iomem;
	int ipi_num;

	if (!hdev || !iomem)
		return -1;

	ipi_num = mipi_port_ipi(hdev->port);
	if (cfg->channel_num > ipi_num) {
		mipierr("channel_num %d error, max %d", cfg->channel_num, ipi_num);
		return -1;
	}


	ipi_num = 0;
	switch (cfg->channel_num) {
	case 4:
#if MIPIHOST_CHANNEL_NUM >= 4
		/* ipi4 config */
		if (cfg->channel_sel[3] < 4) {
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_MEM_FLUSH,
				MIPI_HOST_MEMFLUSN_ENABLE);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_VCID,
				cfg->channel_sel[3]);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_DATA_TYPE,
				cfg->datatype);
			if (param->cut_through)
				mipi_putreg(iomem + REG_MIPI_HOST_IPI4_MODE,
					MIPI_HOST_IPI_ENABLE |
					(MIPI_HOST_BITWIDTH_48 << MIPI_HOST_BITWIDTH_OFFSET) |
					MIPI_HOST_CUT_THROUGH);
			else
				mipi_putreg(iomem + REG_MIPI_HOST_IPI4_MODE,
					MIPI_HOST_IPI_ENABLE |
					(MIPI_HOST_BITWIDTH_48 << MIPI_HOST_BITWIDTH_OFFSET));
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_HSA_TIME,
				cfg->hsaTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_HBP_TIME,
				cfg->hbpTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_HSD_TIME,
				cfg->hsdTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI4_ADV_FEATURES,
				param->adv_value);
			ipi_num ++;
		}
#endif
		/* no break */
	case 3:
#if MIPIHOST_CHANNEL_NUM >= 3
		/* ipi4 config */
		if (cfg->channel_sel[2] < 4) {
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_MEM_FLUSH,
				MIPI_HOST_MEMFLUSN_ENABLE);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_VCID,
				cfg->channel_sel[2]);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_DATA_TYPE,
				cfg->datatype);
			if (param->cut_through)
				mipi_putreg(iomem + REG_MIPI_HOST_IPI3_MODE,
					MIPI_HOST_IPI_ENABLE |
					(MIPI_HOST_BITWIDTH_48 << MIPI_HOST_BITWIDTH_OFFSET) |
					MIPI_HOST_CUT_THROUGH);
			else
				mipi_putreg(iomem + REG_MIPI_HOST_IPI3_MODE,
					MIPI_HOST_IPI_ENABLE |
					(MIPI_HOST_BITWIDTH_48 << MIPI_HOST_BITWIDTH_OFFSET));
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_HSA_TIME,
				cfg->hsaTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_HBP_TIME,
				cfg->hbpTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_HSD_TIME,
				cfg->hsdTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI3_ADV_FEATURES,
				param->adv_value);
			ipi_num ++;
		}
#endif
		/* no break */
	case 2:
#if MIPIHOST_CHANNEL_NUM >= 2
		/* ipi4 config */
		if (cfg->channel_sel[1] < 4) {
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_MEM_FLUSH,
				MIPI_HOST_MEMFLUSN_ENABLE);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_VCID,
				cfg->channel_sel[1]);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_DATA_TYPE,
				cfg->datatype);
			if (param->cut_through)
				mipi_putreg(iomem + REG_MIPI_HOST_IPI2_MODE,
					MIPI_HOST_IPI_ENABLE |
					(MIPI_HOST_BITWIDTH_48 << MIPI_HOST_BITWIDTH_OFFSET) |
					MIPI_HOST_CUT_THROUGH);
			else
				mipi_putreg(iomem + REG_MIPI_HOST_IPI2_MODE,
					MIPI_HOST_IPI_ENABLE |
					(MIPI_HOST_BITWIDTH_48 << MIPI_HOST_BITWIDTH_OFFSET));
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_HSA_TIME,
				cfg->hsaTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_HBP_TIME,
				cfg->hbpTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_HSD_TIME,
				cfg->hsdTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI2_ADV_FEATURES,
				param->adv_value);
			ipi_num ++;
		}
#endif
		/* no break */
	case 1:
	default:
		/* ipi4 config */
		if (cfg->channel_sel[0] < 4) {
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_MEM_FLUSH,
				MIPI_HOST_MEMFLUSN_ENABLE);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_VCID,
				cfg->channel_sel[0]);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_DATA_TYPE,
				cfg->datatype);
			if (param->cut_through)
				mipi_putreg(iomem + REG_MIPI_HOST_IPI_MODE,
					MIPI_HOST_IPI_ENABLE |
					(MIPI_HOST_BITWIDTH_48 << MIPI_HOST_BITWIDTH_OFFSET) |
					MIPI_HOST_CUT_THROUGH);
			else
				mipi_putreg(iomem + REG_MIPI_HOST_IPI_MODE,
					MIPI_HOST_IPI_ENABLE |
					(MIPI_HOST_BITWIDTH_48 << MIPI_HOST_BITWIDTH_OFFSET));
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_HSA_TIME,
				cfg->hsaTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_HBP_TIME,
				cfg->hbpTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_HSD_TIME,
				cfg->hsdTime);
			mipi_putreg(iomem + REG_MIPI_HOST_IPI_ADV_FEATURES,
				param->adv_value);
			ipi_num ++;
		}
		break;
	}

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
	mipi_host_cfg_t bcfg;

	if (!scfg || !dcfg)
		return -1;

	memcpy(&bcfg, scfg, sizeof(mipi_host_cfg_t));
	if (dcfg->hsaTime == 0)
		bcfg.hsaTime = 0;
	if (dcfg->hbpTime == 0)
		bcfg.hbpTime = 0;
	if (dcfg->hsdTime == 0)
		bcfg.hsdTime = 0;

	return memcmp(&bcfg, dcfg, sizeof(mipi_host_cfg_t));
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
#ifdef CONFIG_HOBOT_XJ3
	struct device *dev = hdev->dev;
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
				ret = -1;
			} else {
				mipiinfo("%s = %lu", name, clk);
			}
		}
	}
	ret = vio_clk_enable(name);
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
	ret = mipi_host_configure_clk(hdev, name, freq, 0);
	param->snrclk_freq = (uint32_t)(mipi_host_get_clk(hdev, name));
	diff = (freq > param->snrclk_freq) ? (freq - param->snrclk_freq) :
			param->snrclk_freq - freq;
	if ((diff * 10) > freq) {
		ret = -1;
	}
	if (ret)
		mipierr("%s set %u but %u error", name, freq, param->snrclk_freq);
	else
		mipiinfo("%s set %u as %u", name, freq, param->snrclk_freq);

	return ret;
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
				param->snrclk_en = 1;
		} else {
			mipierr("snrclk set enable not support");
			ret = -1;
		}
	} else {
		if (snrclk->disable) {
			mipiinfo("snrclk set disable");
			ret = pinctrl_select_state(snrclk->pinctrl,
					snrclk->disable);
			if (ret == 0)
				param->snrclk_en = 0;
		} else {
			mipierr("snrclk set disable not support");
			ret = -1;
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
		mipiinfo("ipi clk force as %lu", pixclk);
	} else {
		pixclk = linelenth * framelenth * cfg->fps;
		if (cfg->datatype < MIPI_CSI2_DT_RAW_8)
			pixclk = pixclk;
		else
			pixclk = (pixclk + 2) / 3;

		if (param->ipi_limit && pixclk < param->ipi_limit) {
			mipiinfo("ipi clk limit %lu up to %u", pixclk, param->ipi_limit);
			pixclk = param->ipi_limit;
		}
	}
#ifdef CONFIG_HOBOT_XJ3
	if (hdev->port >= ARRAY_SIZE(g_mh_ipiclk_name) ||
		mipi_host_configure_clk(hdev, g_mh_ipiclk_name[hdev->port], pixclk, 0) < 0) {
		mipierr("mipi_host_configure_clk %lu error", pixclk);
	} else {
		mipiinfo("host fifo clk pixclk: %lu", pixclk);
		pixclk_act = mipi_host_get_clk(hdev, g_mh_ipiclk_name[hdev->port]);
		mipiinfo("host fifo clk pixclk: %lu", pixclk_act);
	}
#elif defined CONFIG_X2_IPS
	if (ips_set_mipi_ipi_clk(pixclk) < 0) {
		mipiinfo("ips_set_mipi_ipi_clk %lu error", pixclk);
	} else {
		mipiinfo("host fifo clk pixclk: %lu", pixclk);
		pixclk_act = ips_get_mipi_ipi_clk();
		mipiinfo("host fifo clk pixclk: %lu", pixclk_act);
	}
#else
	mipiinfo("should: ips_set_mipi_ipi_clk(%lu)", pixclk);
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
	unsigned long rx_bit_clk = 0;
	unsigned long bits_per_pixel = 0;
	unsigned long line_size = 0;
	unsigned long cycles_to_trans = 0;
	unsigned long time_ppi = 0;
	unsigned long time_ipi = 0;
	int32_t  hsdtime = 0;

	switch (cfg->datatype) {
	case MIPI_CSI2_DT_YUV420_8:
		bits_per_pixel = 8 * 3 / 2;
		cycles_to_trans = cfg->width;
		break;
	case MIPI_CSI2_DT_YUV420_10:
		bits_per_pixel = 16 * 3 / 2;
		cycles_to_trans = cfg->width;
		break;
	case MIPI_CSI2_DT_YUV422_8:
		bits_per_pixel = 8 * 2;
		cycles_to_trans = cfg->width;
		break;
	case MIPI_CSI2_DT_YUV422_10:
		bits_per_pixel = 16 * 2;
		cycles_to_trans = cfg->width;
		break;
	case MIPI_CSI2_DT_RAW_8:
		bits_per_pixel = 8;
		cycles_to_trans = (cfg->width + 2) / 3;
		break;
	case MIPI_CSI2_DT_RAW_10:
		bits_per_pixel = 10;
		cycles_to_trans = (cfg->width + 2) / 3;
		break;
	case MIPI_CSI2_DT_RAW_12:
		bits_per_pixel = 12;
		cycles_to_trans = (cfg->width + 2) / 3;
		break;
	case MIPI_CSI2_DT_RAW_14:
		bits_per_pixel = 14;
		cycles_to_trans = (cfg->width + 2) / 3;
		break;
	default:
		bits_per_pixel = 16;
		break;
	}
	if (!cfg->linelenth) {
		rx_bit_clk = (unsigned long)cfg->mipiclk * 1000000;
		line_size = cfg->width;
	} else {
		rx_bit_clk = cfg->linelenth * cfg->framelenth * cfg->fps * bits_per_pixel;
		line_size = cfg->linelenth;
	}
	mipiinfo("linelenth: %d, framelenth: %d, fps: %d, bits_per_pixel: %lu, pixclk: %lu, rx_bit_clk: %lu",
			 cfg->linelenth, cfg->framelenth, cfg->fps, bits_per_pixel, pixclk, rx_bit_clk);
	time_ppi = (1000 * bits_per_pixel * line_size * 1000000 / rx_bit_clk);
	mipiinfo("time to transmit last pixel in ppi: %lu", time_ppi);
	hsdtime = (bits_per_pixel * line_size * pixclk / rx_bit_clk) - (cfg->hsaTime + cfg->hbpTime + cycles_to_trans);
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
	//hsdtime = (hsdtime + 16) & (~0xf);
	time_ipi = 1000 * (unsigned long)(cfg->hsaTime + cfg->hbpTime + hsdtime + cycles_to_trans) * 1000000 / pixclk;
	mipiinfo("time to transmit last pixel in ipi: %lu", time_ipi);
	return (uint16_t)hsdtime;
}

#if MIPI_HOST_INT_DBG
static const uint32_t mipi_host_int_msk[] = {
	/* reg offset,                           mask,  */
	REG_MIPI_HOST_INT_MSK_PHY_FATAL,         0x000f,
	REG_MIPI_HOST_INT_MSK_PKT_FATAL,         0x000f,
	REG_MIPI_HOST_INT_MSK_FRAME_FATAL,       0x000f,
	REG_MIPI_HOST_INT_MSK_PHY,               0x000f,
	REG_MIPI_HOST_INT_MSK_PKT,               0x000f,
	REG_MIPI_HOST_INT_MSK_LINE,              0x00ff,
	REG_MIPI_HOST_INT_MSK_IPI,               0x003f,
	REG_MIPI_HOST_INT_MSK_IPI2,              0x003f,
	REG_MIPI_HOST_INT_MSK_IPI3,              0x003f,
	REG_MIPI_HOST_INT_MSK_IPI4,              0x003f,
};

static const uint32_t mipi_host_1p4_int_msk[] = {
	/* reg offset,                           mask,      */
	REG_MIPI_HOST_INT_MSK_PHY_FATAL,         0x000001ff,
	REG_MIPI_HOST_INT_MSK_PKT_FATAL,         0x00000001,
	REG_MIPI_HOST_INT_MSK_LINE,              0x00ff00ff,
	REG_MIPI_HOST_INT_MSK_BNDRY_FRAME_FATAL, 0xffffffff,
	REG_MIPI_HOST_INT_MSK_SEQ_FRAME_FATAL,   0xffffffff,
	REG_MIPI_HOST_INT_MSK_CRC_FRAME_FATAL,   0xffffffff,
	REG_MIPI_HOST_INT_MSK_PLD_CRC_FATAL,     0xffffffff,
	REG_MIPI_HOST_INT_MSK_DATA_ID,           0xffffffff,
    REG_MIPI_HOST_INT_MSK_ECC_CORRECT,       0xffffffff,
	REG_MIPI_HOST_INT_MSK_PHY,               0x00ff00ff,
	REG_MIPI_HOST_INT_MSK_IPI,               0x0000003f,
	REG_MIPI_HOST_INT_MSK_IPI2,              0x0000003f,
	REG_MIPI_HOST_INT_MSK_IPI3,              0x0000003f,
	REG_MIPI_HOST_INT_MSK_IPI4,              0x0000003f,
};

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
	void __iomem *iomem = host->iomem;
	uint32_t reg = 0;
	uint32_t mask = 0;
	uint32_t temp = 0;
	const uint32_t *msk;
	int i, num;

	if (!hdev || !iomem)
		return;

	if (MIPI_HOST_IS_1P4(iomem)) {
		msk = mipi_host_1p4_int_msk;
		num = ARRAY_SIZE(mipi_host_1p4_int_msk);
	} else {
		msk = mipi_host_int_msk;
		num = ARRAY_SIZE(mipi_host_int_msk);
	}

	for (i = 0; i < num; i += 2) {
		reg = msk[i];
		mask = msk[i + 1];
		temp = mipi_getreg(iomem + reg);
		temp &= ~(mask);
		temp |= mask;
		mipi_putreg(iomem + reg, temp);
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
	void __iomem *iomem = host->iomem;
	uint32_t reg = 0;
	uint32_t mask = 0;
	uint32_t temp = 0;
	const uint32_t *msk;
	int i, num;

	if (!hdev || !iomem)
		return;

	if (MIPI_HOST_IS_1P4(iomem)) {
		msk = mipi_host_1p4_int_msk;
		num = ARRAY_SIZE(mipi_host_1p4_int_msk);
	} else {
		msk = mipi_host_int_msk;
		num = ARRAY_SIZE(mipi_host_int_msk);
	}

	for (i = 0; i < num; i+=2) {
		reg = msk[i];
		mask = msk[i + 1];
		temp = mipi_getreg(iomem + reg);
		temp &= ~(mask);
		mipi_putreg(iomem + reg, temp);
	}

#if defined MIPI_HOST_INT_USE_TIMER
	hdev->irq_timer_en = 0;
#endif

	return;
}

#ifdef CONFIG_X2_DIAG
static void mipi_host_diag_report(mipi_hdev_t *hdev,
		uint8_t errsta, uint32_t total_irq,
		uint32_t *sub_irq_data, uint32_t elem_cnt)
{
	uint32_t buff[8];
	int i;

	hdev->last_err_tm_ms = jiffies_to_msecs(get_jiffies_64());
	if (errsta) {
		buff[0] = total_irq;
		for (i = 0; i < elem_cnt; i++)
			buff[1 + i] = sub_irq_data[i];

		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_VIO,
				EventIdVioMipiHostErr,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				(uint8_t *)buff,
				32);
	}
}

static void mipi_host_error_report(mipi_hdev_t *hdev,
		uint8_t errsta, uint32_t total_irq,
		uint32_t *sub_irq_data, uint32_t elem_cnt)
{
		diag_send_event_stat_and_env_data(
				DiagMsgPrioLow,
				ModuleDiag_VIO,
				EventIdVioMipiHostError,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				NULL,
				32);
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
				EventIdVioMipiHostErr,
				DiagEventStaSuccess);
	}
	jiffi = get_jiffies_64() + msecs_to_jiffies(2000);
	mod_timer(&hdev->diag_timer, jiffi); // trriger again.
}
#endif

static const uint32_t mipi_host_int_st[] = {
	/* reg offset,                          mask,                    icnt */
	REG_MIPI_HOST_INT_ST_PHY_FATAL,         MIPI_HOST_INT_PHY_FATAL, 1,
	REG_MIPI_HOST_INT_ST_PKT_FATAL,         MIPI_HOST_INT_PKT_FATAL, 2,
	REG_MIPI_HOST_INT_ST_FRAME_FATAL,       MIPI_HOST_INT_FRM_FATAL, 3,
	REG_MIPI_HOST_INT_ST_PHY,               MIPI_HOST_INT_PHY,       10,
	REG_MIPI_HOST_INT_ST_PKT,               MIPI_HOST_INT_PKT,       11,
	REG_MIPI_HOST_INT_ST_LINE,              MIPI_HOST_INT_LINE,      12,
	REG_MIPI_HOST_INT_ST_IPI,               MIPI_HOST_INT_IPI,       13,
	REG_MIPI_HOST_INT_ST_IPI2,              MIPI_HOST_INT_IPI2,      14,
	REG_MIPI_HOST_INT_ST_IPI3,              MIPI_HOST_INT_IPI3,      15,
	REG_MIPI_HOST_INT_ST_IPI4,              MIPI_HOST_INT_IPI4,      16,
};

static const uint32_t mipi_host_1p4_int_st[] = {
	/* reg offset,                          mask,                              icnt */
	REG_MIPI_HOST_INT_ST_PHY_FATAL,         MIPI_HOST_1P4_INT_PHY_FATAL,       1,
	REG_MIPI_HOST_INT_ST_PKT_FATAL,         MIPI_HOST_1P4_INT_PKT_FATAL,       2,
	REG_MIPI_HOST_INT_ST_BNDRY_FRAME_FATAL, MIPI_HOST_1P4_INT_BNDRY_FRM_FATAL, 4,
	REG_MIPI_HOST_INT_ST_SEQ_FRAME_FATAL,   MIPI_HOST_1P4_INT_SEQ_FRM_FATAL,   5,
	REG_MIPI_HOST_INT_ST_CRC_FRAME_FATAL,   MIPI_HOST_1P4_INT_CRC_FRM_FATAL,   6,
	REG_MIPI_HOST_INT_ST_PLD_CRC_FATAL,     MIPI_HOST_1P4_INT_PLD_CRC_FATAL,   7,
	REG_MIPI_HOST_INT_ST_DATA_ID,           MIPI_HOST_1P4_INT_DATA_ID,         8,
	REG_MIPI_HOST_INT_ST_ECC_CORRECT,       MIPI_HOST_1P4_INT_ECC_CORRECTED,   9,
	REG_MIPI_HOST_INT_ST_PHY,               MIPI_HOST_1P4_INT_PHY,             10,
	REG_MIPI_HOST_INT_ST_LINE,              MIPI_HOST_1P4_INT_LINE,            12,
	REG_MIPI_HOST_INT_ST_IPI,               MIPI_HOST_1P4_INT_IPI,             13,
	REG_MIPI_HOST_INT_ST_IPI2,              MIPI_HOST_1P4_INT_IPI2,            14,
	REG_MIPI_HOST_INT_ST_IPI3,              MIPI_HOST_1P4_INT_IPI3,            15,
	REG_MIPI_HOST_INT_ST_IPI4,              MIPI_HOST_1P4_INT_IPI4,            16,
};

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
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	mipi_host_param_t *param = &host->param;
	mipi_host_icnt_t *icnt = &host->icnt;
	void __iomem *iomem = host->iomem;
	uint32_t *icnt_p = (uint32_t *)icnt;
	uint32_t reg, mask, icnt_n;
	uint32_t irq = 0, irq_do;
	uint32_t subirq = 0;
	const uint32_t *st;
	int i, num;
#ifdef CONFIG_X2_DIAG
	uint8_t err_occurred = 0;
	uint32_t env_subirq[sizeof(mipi_host_icnt_t)/sizeof(uint32_t)] = {0};
#endif

	if (!hdev || !iomem)
		return IRQ_NONE;

	if (this_irq >= 0)
		disable_irq_nosync(this_irq);

	if (MIPI_HOST_IS_1P4(iomem)) {
		st = mipi_host_1p4_int_st;
		num = ARRAY_SIZE(mipi_host_1p4_int_st);
	} else {
		st = mipi_host_int_st;
		num = ARRAY_SIZE(mipi_host_int_st);
	}

#ifdef MIPI_HOST_INT_USE_TIMER
	irq = hdev->irq_st_main;
#else
	irq = mipi_getreg(iomem + REG_MIPI_HOST_INT_ST_MAIN);
#endif
	if (param->irq_debug)
		mipierr("irq status 0x%x", irq);
	else
		mipidbg("irq status 0x%x", irq);
	if(irq) {
		irq_do = irq;
		icnt->st_main++;
		for (i = 0; i < num; i += 3) {
			mask = st[i + 1];
			if (!(irq_do & mask))
				continue;

			reg = st[i];
			icnt_n = st[i + 2];
			subirq = mipi_getreg(iomem + reg);
			if (param->irq_debug)
				mipierr("  %s: 0x%x",
					g_mh_icnt_names[icnt_n], subirq);
			else
				mipidbg("  %s: 0x%x",
					g_mh_icnt_names[icnt_n], subirq);
			icnt_p[icnt_n]++;
#ifdef CONFIG_X2_DIAG
			err_occurred = 1;
			env_subirq[icnt_n] = subirq;
#endif
			irq_do &= ~mask;
			if (!irq_do)
				break;
		}
	}

	if (icnt->st_main > param->irq_cnt)
		mipi_host_irq_disable(hdev);

	if (this_irq >= 0)
		enable_irq(this_irq);

#ifdef CONFIG_X2_DIAG
	mipi_host_diag_report(hdev, err_occurred, irq, env_subirq,
				 sizeof(env_subirq)/sizeof(uint32_t));
	mipi_host_error_report(hdev, err_occurred, irq, env_subirq,
				  sizeof(env_subirq)/sizeof(uint32_t));
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
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	mipi_host_param_t *param = &host->param;
	void __iomem *iomem = host->iomem;
	uint16_t ncount = 0;
	uint32_t stopstate = 0;

	if (!hdev || !iomem || !cfg)
		return -1;

	mipiinfo("check phy stop state");
	/*Check that data lanes are in Stop state*/
	do {
		stopstate = mipi_getreg(iomem + REG_MIPI_HOST_PHY_STOPSTATE);
		if ((stopstate & 0xF) == HOST_DPHY_LANE_STOP(cfg->lane))
			return 0;
		mdelay(1);
		ncount++;
	} while (param->notimeout || ncount <= HOST_DPHY_CHECK_MAX);

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
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	mipi_host_param_t *param = &host->param;
	void __iomem *iomem = host->iomem;
	uint16_t ncount = 0;
	uint32_t state = 0;

	if (!hdev || !iomem)
		return -1;

	mipiinfo("check hs reception");
	/*Check that clock lane is in HS mode*/
	do {
		state = mipi_getreg(iomem + REG_MIPI_HOST_PHY_RX);
		if ((state & HOST_DPHY_RX_HS) == HOST_DPHY_RX_HS) {
			mipiinfo("entry hs reception");
			return 0;
		}
		ncount++;
		mdelay(1);
	} while (param->notimeout || ncount <= HOST_DPHY_CHECK_MAX);

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
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	mipi_host_param_t *param = &host->param;
	void __iomem *iomem = host->iomem;

	if (!hdev || !iomem)
		return -1;

	if (!hdev->is_ex && !param->need_stop_check && param->stop_check_instart) {
		if (0 != mipi_host_dphy_wait_stop(hdev, &host->cfg)) {
			/*Release DWC_mipi_csi2_host from reset*/
			mipierr("wait phy stop state error!!!");
			return -1;
		}
	}
	if (!param->nocheck) {
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
	struct device *dev = hdev->dev;
	mipi_host_t *host = &hdev->host;
	mipi_host_param_t *param = &host->param;
	void __iomem  *iomem = host->iomem;
	unsigned long pixclk = 0;

	if (!hdev || !iomem)
		return -1;

	mipiinfo("init begin");
	mipiinfo("%d lane %dx%d %dfps datatype 0x%x",
			 cfg->lane, cfg->width, cfg->height, cfg->fps, cfg->datatype);
	if (!hdev->is_ex) {
		/* cfg->mclk:
		 * 0      : disable only
		 * 1      : enable only
		 * 2~24   : invalid and drop
		 * 25~9280: invalid and error
		 * 9281~  : *10K = freq -> 92.81~655.35MHz
		 */
		if (cfg->mclk >= (MIPI_HOST_SNRCLK_FREQ_MIN / 10000UL)) {
			if (mipi_host_snrclk_set_freq(hdev, cfg->mclk * 10000UL) ||
					mipi_host_snrclk_set_en(hdev, 1)) {
				return -1;
			}
		} else if (cfg->mclk > 24) {
			mipiinfo("mclk %d should >= %lu(%luHz)", cfg->mclk,
				(MIPI_HOST_SNRCLK_FREQ_MIN / 10000UL), MIPI_HOST_SNRCLK_FREQ_MIN);
			return -1;
		} else if (cfg->mclk > 1) {
			mipiinfo("mclk %d drop", cfg->mclk);
		} else if (cfg->mclk == 1) {
			mipi_host_snrclk_set_en(hdev, 1);
		} else {
			mipi_host_snrclk_set_en(hdev, 0);
		}
	}
	pixclk = mipi_host_pixel_clk_select(hdev, cfg);
	if (0 == pixclk) {
		mipierr("pixel clk config error!");
		return -1;
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
	cfg->hsaTime = cfg->hsaTime ? cfg->hsaTime : MIPI_HOST_HSATIME;
	cfg->hbpTime = cfg->hbpTime ? cfg->hbpTime : MIPI_HOST_HBPTIME;
	cfg->hsdTime = cfg->hsdTime ? cfg->hsdTime : mipi_host_get_hsd(hdev, cfg, pixclk);
	if (!hdev->is_ex) {
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

	spin_lock(&user->slock);
	mipidbg("open as %d", user->open_cnt);
	if (user->open_cnt == 0) {
		mutex_init(&user->mutex);
		user->init_cnt = 0;
		user->start_cnt = 0;
		mipi_host_configure_clk(hdev, MIPI_HOST_CFGCLK_NAME, MIPI_HOST_CFGCLK_MHZ, 1);
		mipi_host_configure_clk(hdev, MIPI_HOST_REFCLK_NAME, MIPI_HOST_REFCLK_MHZ, 1);
	}
	user->open_cnt++;
	spin_unlock(&user->slock);

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

	spin_lock(&user->slock);
	if (user->open_cnt > 0)
		user->open_cnt--;
	mipidbg("close as %d", user->open_cnt);
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
	}
	spin_unlock(&user->slock);

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
				host->state = MIPI_STATE_START;
			}
			user->start_cnt++;
			mutex_unlock(&user->mutex);
		}
		break;
	case MIPIHOSTIOC_STOP:
		{
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("stop user mutex lock error");
				return -EINVAL;
			}
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
					user->start_cnt++;
					mutex_unlock(&user->mutex);
					return -EBUSY;
				}
				if (hdev->ex_hdev) {
					ex_hdev = (mipi_hdev_t *)(hdev->ex_hdev);
					mipi_host_stop(ex_hdev);
				}
				if (0 != (ret = mipi_host_stop(hdev))) {
					mipierr("stop error: %d", ret);
					user->start_cnt++;
					mutex_unlock(&user->mutex);
					return ret;
				}
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
	case MIPIHOSTIOC_GET_INIT_CNT:
		{
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("get int_cnt user mutex lock error");
				return -EINVAL;
			}
			if (put_user(user->init_cnt, (uint32_t *)arg)) {
				mipierr("put data to user failed");
				mutex_unlock(&user->mutex);
				return -EFAULT;
			}
		}
		break;
	case MIPIHOSTIOC_GET_START_CNT:
		{
			if (mutex_lock_interruptible(&user->mutex)) {
				mipierr("get start_cnt user mutex lock error");
				return -EINVAL;
			}
			if (put_user(user->start_cnt, (uint32_t *)arg)) {
				mipierr("put data to user failed");
				mutex_unlock(&user->mutex);
				return -EFAULT;
			}
		}
		break;
	case MIPIHOSTIOC_PUT_CNT:
		{
			mutex_unlock(&user->mutex);
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
	return 0;
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
	uint32_t val;

	idx = mipi_host_param_idx(attr->attr.name);
	if (idx >= 0) {
		ret = kstrtouint(buf, 0, &val);
		if (!ret) {
			param[idx] = val;
			error = 0;
		}
	}

	if (error == 0) {
		if(!strcmp(attr->attr.name, "snrclk_en"))
			error = mipi_host_snrclk_set_en(hdev, val);
		else if (!strcmp(attr->attr.name, "snrclk_freq"))
			error = mipi_host_snrclk_set_freq(hdev, val);
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
MIPI_HOST_PARAM_DEC(dbg_value);
MIPI_HOST_PARAM_DEC(adv_value);
MIPI_HOST_PARAM_DEC(need_stop_check);
MIPI_HOST_PARAM_DEC(stop_check_instart);
MIPI_HOST_PARAM_DEC(cut_through);
MIPI_HOST_PARAM_DEC(ipi_force);
MIPI_HOST_PARAM_DEC(ipi_limit);
MIPI_HOST_PARAM_DEC(snrclk_en);
MIPI_HOST_PARAM_DEC(snrclk_freq);
#if MIPI_HOST_INT_DBG
MIPI_HOST_PARAM_DEC(irq_cnt);
MIPI_HOST_PARAM_DEC(irq_debug);
#endif

static struct attribute *param_attr[] = {
	MIPI_HOST_PARAM_ADD(nocheck),
	MIPI_HOST_PARAM_ADD(notimeout),
	MIPI_HOST_PARAM_ADD(dbg_value),
	MIPI_HOST_PARAM_ADD(adv_value),
	MIPI_HOST_PARAM_ADD(need_stop_check),
	MIPI_HOST_PARAM_ADD(stop_check_instart),
	MIPI_HOST_PARAM_ADD(cut_through),
	MIPI_HOST_PARAM_ADD(ipi_force),
	MIPI_HOST_PARAM_ADD(ipi_limit),
	MIPI_HOST_PARAM_ADD(snrclk_en),
	MIPI_HOST_PARAM_ADD(snrclk_freq),
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
		MH_STA_SHOW(mode, "%s", (hdev->is_ex) ? "group_ext" :
			((hdev->ex_hdev) ? "group_mst" : "alone"));
		MH_STA_SHOW(lane_mode, "%d(%dlane)", hdev->lane_mode,
			mipi_port_lane(hdev->port, hdev->lane_mode));
		MH_STA_SHOW(iomem, "%p", host->iomem);
#if MIPI_HOST_INT_DBG && defined MIPI_HOST_INT_USE_TIMER
		MH_STA_SHOW(timer, "%s",
			(hdev->irq_timer_en) ? "enable" : "disable");
#else
		MH_STA_SHOW(irq, "%d", host->irq);
#endif
		MH_STA_SHOW(state, "%d(%s)", host->state,
			g_mh_state[host->state]);
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
			MH_REG_SHOW(DATA_IDS_1);
			MH_REG_SHOW(DATA_IDS_2);
			MH_REG_SHOW(DATA_IDS_2);
			MH_REG_SHOW(PHY_SHUTDOWNZ);
			MH_REG_SHOW(DPHY_RSTZ);
			MH_REG_SHOW(PHY_RX);
			MH_REG_SHOW(PHY_STOPSTATE);
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
			if (cfg->channel_num > 0 && cfg->channel_sel[0] >= 0) {
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
			}
			if (cfg->channel_num > 1 && cfg->channel_sel[1] >= 0) {
				MH_REG_SHOW(IPI2_MODE);
				MH_REG_SHOW(IPI2_VCID);
				MH_REG_SHOW(IPI2_DATA_TYPE);
				MH_REG_SHOW(IPI2_MEM_FLUSH);
				MH_REG_SHOW(IPI2_HSA_TIME);
				MH_REG_SHOW(IPI2_HBP_TIME);
				MH_REG_SHOW(IPI2_HSD_TIME);
				MH_REG_SHOW(IPI2_ADV_FEATURES);
			}
			if (cfg->channel_num > 2 && cfg->channel_sel[2] >= 0) {
				MH_REG_SHOW(IPI3_MODE);
				MH_REG_SHOW(IPI3_VCID);
				MH_REG_SHOW(IPI3_DATA_TYPE);
				MH_REG_SHOW(IPI3_MEM_FLUSH);
				MH_REG_SHOW(IPI3_HSA_TIME);
				MH_REG_SHOW(IPI3_HBP_TIME);
				MH_REG_SHOW(IPI3_HSD_TIME);
				MH_REG_SHOW(IPI3_ADV_FEATURES);
			}
			if (cfg->channel_num > 3 && cfg->channel_sel[3] >= 0) {
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
		MH_STA_SHOW(state, "%s", (param->snrclk_en ? "enable" : "disable"));
		MH_STA_SHOW(freq, "%u", param->snrclk_freq);
	} else if (strcmp(attr->attr.name, "user") == 0) {
		MH_STA_SHOW(user, "%d", user->open_cnt);
		MH_STA_SHOW(init, "%d", user->init_cnt);
		MH_STA_SHOW(start, "%d", user->start_cnt);
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

/* sysfs attr groups */
static const struct attribute_group *attr_groups[] = {
	&param_attr_group,
	&status_attr_group,
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
			ret = PTR_ERR(g_mh_class);
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
		ret = PTR_ERR(hdev->dev);
		hdev->dev = NULL;
		pr_err("[%s] deivce create error %d\n", __func__, ret);
		goto err_creat;
	}

#ifdef CONFIG_X2_DIAG
	/* diag */
	if (diag_register(ModuleDiag_VIO, EventIdVioMipiHostErr,
						32, 400, 6000, NULL) < 0)
		pr_err("mipi host %d diag register fail\n", hdev->port);
	else {
		hdev->last_err_tm_ms = 0;
		init_timer(&hdev->diag_timer);
		hdev->diag_timer.expires =
			get_jiffies_64() + msecs_to_jiffies(1000);
		hdev->diag_timer.data = (unsigned long)hdev;
		hdev->diag_timer.function = mipi_host_diag_timer_func;
		add_timer(&hdev->diag_timer);
	}
	if (diag_register(ModuleDiag_VIO, EventIdVioMipiHostError,
						32, 300, 5000, NULL) < 0)
		pr_err("mipi host %d diag register fail\n", hdev->port);
#endif
	spin_lock_init(&hdev->user.slock);

	return 0;
err_creat:
	cdev_del(&hdev->cdev);
err_add:
	return ret;
}

static int hobot_mipi_host_remove_cdev(mipi_hdev_t *hdev)
{
#ifdef CONFIG_X2_DIAG
	del_timer_sync(&hdev->host.diag_timer);
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

	port_num = 0;
	for (i = 0; i < init_num; i++) {
		hdev = kmalloc(sizeof(mipi_hdev_t), GFP_KERNEL);
		if (hdev == NULL) {
		pr_err("[%s] malloc failed\n", __func__);
			ret = -ENOMEM;
			goto err_kmalloc;
		}
		memset(hdev, 0x0, sizeof(mipi_hdev_t));
		host = &hdev->host;
		param = &host->param;
		group = mipi_port_group(i);
		index = mipi_port_index(i);
		reg_addr_dev = reg_addr + (reg_size * 2) * group + (reg_size * index);
		host->iomem = ioremap_nocache(reg_addr_dev, reg_size);
		if (IS_ERR(host->iomem)) {
			pr_err("[%s] ioremap error\n", __func__);
			ret = PTR_ERR(host->iomem);
			host->iomem = NULL;
			goto err_ioremap;
		}
		hdev->port = i;
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
#else
		pr_info("[%s] no int timer\n", __func__);
#endif
		param->adv_value = MIPI_HOST_ADV_DEFAULT;
		param->cut_through = MIPI_HOST_CUT_DEFAULT;
		param->ipi_limit = MIPI_HOST_IPILIMIT_DEFAULT;

		hobot_mipi_host_phy_register(hdev);
		g_hdev[i] = hdev;
		port_num ++;

		ret = mipi_getreg(host->iomem + REG_MIPI_HOST_VERSION);
		dev_info(hdev->dev, "ver %c%c%c%c port%d(%d:%d)\n",
			(ret >> 24), (ret >> 16), (ret >> 8), ret,
			i, mipi_port_group(i), mipi_port_index(i));
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
		ret = PTR_ERR(host->iomem);
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
	host->irq = res->start;
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
#endif

	param->adv_value = MIPI_HOST_ADV_DEFAULT;
	param->cut_through = MIPI_HOST_CUT_DEFAULT;
	param->ipi_limit = MIPI_HOST_IPILIMIT_DEFAULT;

	platform_set_drvdata(pdev, hdev);

	hdev->port = port;
	hdev->lane_mode = mipi_dphy_get_lanemode(MIPI_DPHY_TYPE_HOST,
							hdev->port);
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
		if (host->snrclk.pinctrl)
			host->param.snrclk_en = 2;
		if (host->snrclk.index >= 0)
			host->param.snrclk_freq = (uint32_t)(mipi_host_get_clk(hdev,
						g_mh_snrclk_name[host->snrclk.index]));
	}

	hobot_mipi_host_phy_register(hdev);
	g_hdev[port] = hdev;
	port_num ++;

	ret = mipi_getreg(host->iomem + REG_MIPI_HOST_VERSION);
	dev_info(hdev->dev, "ver %c%c%c%c port%d(%d:%d)\n",
		(ret >> 24), (ret >> 16), (ret >> 8), ret,
		port, mipi_port_group(port), mipi_port_index(port));
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
MODULE_DESCRIPTION("X2 MIPI Host Driver");
