/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
/**
 * @file     mipi_host.c
 * @brief    MIPI HOST control function, includeing controller and D-PHY control
 * @author   tarryzhang (tianyu.zhang@hobot.cc)
 * @date     2017/7/6
 * @version  V1.0
 * @par      Horizon Robotics
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>

#include "x2/x2_mipi_host.h"
#include "x2_mipi_host_regs.h"
#include "x2_mipi_dphy.h"
#include "x2_mipi_utils.h"
#include "x2/x2_ips.h"

#define MIPI_HOST_INT_DBG          (1)

#define MIPI_HOST_INT_PHY_FATAL    (0x1)
#define MIPI_HOST_INT_PKT_FATAL    (0x1<<1)
#define MIPI_HOST_INT_FRM_FATAL    (0x1<<2)
#define MIPI_HOST_INT_PHY          (0x1<<16)
#define MIPI_HOST_INT_PKT          (0x1<<17)
#define MIPI_HOST_INT_LINE         (0x1<<18)
#define MIPI_HOST_INT_IPI          (0x1<<19)

#define MIPI_HOST_CSI2_RAISE       (0x01)
#define MIPI_HOST_CSI2_RESETN      (0x00)
#define MIPI_HOST_BITWIDTH_48      (0)
#define MIPI_HOST_BITWIDTH_16      (1)
#define MIPI_HOST_BITWIDTH_OFFSET  (8)
#define MIPI_HOST_MEMFLUSN_ENABLE  (0x01 << 8)
#define MIPI_HOST_EMB_DATA         (0x01 << 8)
#define MIPI_HOST_IPI_ENABLE       (0x01 << 24)
#define MIPI_HOST_LEGCYMODE_ENABLE (0x01 << 24)
#define MIPI_HOST_HSATIME          (0x04)
#define MIPI_HOST_HBPTIME          (0x04)
#define MIPI_HOST_HSDTIME          (0x5f4)
#define MIPI_HOST_CFGCLK_DEFAULT   (0x1C)

#define HOST_DPHY_LANE_MAX         (4)
#define HOST_DPHY_CHECK_MAX        (1000)
#define HOST_DPHY_LANE_STOP(l)     (0xF>>(HOST_DPHY_LANE_MAX-(l)))
#define HOST_DPHY_RX_HS            (0x030000)

#define MIPIHOST_IPI_PORT_0        (0)
#define MIPIHOST_IPI_PORT_1        (1)

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

typedef struct _reg_s {
	uint32_t offset;
	uint32_t value;
} reg_t;

typedef enum _mipi_state_t {
	MIPI_STATE_DEFAULT = 0,
	MIPI_STATE_INIT,
	MIPI_STATE_START,
	MIPI_STATE_STOP,
	MIPI_STATE_MAX,
} mipi_state_t;

unsigned int mipi_host_nocheck = 0;
unsigned int mipi_host_notimeout = 0;
module_param(mipi_host_nocheck, uint, S_IRUGO | S_IWUSR);
module_param(mipi_host_notimeout, uint, S_IRUGO | S_IWUSR);

/* new add */
uint32_t mipi_host_states;
unsigned int need_phy_reset;
unsigned int mipi_delay_time;
unsigned int dbg_value;
unsigned int need_stop_check;
module_param(mipi_host_states, uint, 0644);
module_param(need_phy_reset, uint, 0644);
module_param(mipi_delay_time, uint, 0644);
module_param(dbg_value, uint, 0644);
module_param(need_stop_check, uint, 0644);
#define mipidbg(format, ...)	\
	do {						\
		if ((dbg_value >= 1))	\
			printk(KERN_INFO format "\n", ##__VA_ARGS__);	\
	} while (0)
/* new add */

#define MIPIHOSTIOC_READ        _IOWR(MIPIHOSTIOC_MAGIC, 4, reg_t)
#define MIPIHOSTIOC_WRITE       _IOW(MIPIHOSTIOC_MAGIC, 5, reg_t)

typedef struct _mipi_host_s {
	void __iomem  *iomem;
	int            irq;
	mipi_state_t   state;   /* mipi host state */
} mipi_host_t;

mipi_host_t  *g_mipi_host = NULL;
static int    mipi_host_major = 0;
struct cdev   mipi_host_cdev;
static struct class  *x2_mipi_host_class;
static struct device *g_mipi_host_dev;

static unsigned long mipi_host_pixel_clk_select(mipi_host_cfg_t *control)
{
	unsigned long pixclk = control->linelenth * control->framelenth * control->fps;
	unsigned long pixclk_act = pixclk;
	unsigned long linelenth = control->linelenth;
	unsigned long framelenth = control->framelenth;
	int  ret = 0;

	if (!control->fps) {
		mipiinfo("input FPS can't be zero!!!");
		return 0;
	}
	if (!control->linelenth)
		linelenth = control->width;
	if (!control->framelenth)
		framelenth = control->height;
	pixclk = linelenth * framelenth * control->fps;
	//pixclk = 1000000 * control->mipiclk / control->lane;
	if (control->datatype < MIPI_CSI2_DT_RAW_8)
		pixclk = pixclk;
	else
		pixclk = pixclk / 3;
	ret = ips_set_mipi_ipi_clk(pixclk);
	if (ret < 0)
		mipiinfo("ips_set_mipi_ipi_clk error");
	mipiinfo("host fifo clk pixclk: %lu", pixclk);
	pixclk_act = ips_get_mipi_ipi_clk();
	mipiinfo("host fifo clk pixclk: %lu\n", pixclk_act);
	return pixclk_act;
}

/**
 * @brief mipi_host_get_hsd :
 *
 * @param [] control :
 *
 * @return uint16_t
 */
static uint16_t mipi_host_get_hsd(mipi_host_cfg_t *control, unsigned long pixclk)
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
	unsigned long rx_bit_clk = 0;
	unsigned long bits_per_pixel = 0;
	unsigned long line_size = 0;
	unsigned long cycles_to_trans = 0;
	unsigned long time_ppi = 0;
	unsigned long time_ipi = 0;
	int32_t  hsdtime = 0;

	switch (control->datatype) {
	case MIPI_CSI2_DT_YUV420_8:
		bits_per_pixel = 8 * 3 / 2;
		cycles_to_trans = control->width;
		break;
	case MIPI_CSI2_DT_YUV420_10:
		bits_per_pixel = 16 * 3 / 2;
		cycles_to_trans = control->width;
		break;
	case MIPI_CSI2_DT_YUV422_8:
		bits_per_pixel = 8 * 2;
		cycles_to_trans = control->width;
		break;
	case MIPI_CSI2_DT_YUV422_10:
		bits_per_pixel = 16 * 2;
		cycles_to_trans = control->width;
		break;
	case MIPI_CSI2_DT_RAW_8:
		bits_per_pixel = 8;
		cycles_to_trans = (control->width + 2) / 3;
		break;
	case MIPI_CSI2_DT_RAW_10:
		bits_per_pixel = 10;
		cycles_to_trans = (control->width + 2) / 3;
		break;
	case MIPI_CSI2_DT_RAW_12:
		bits_per_pixel = 12;
		cycles_to_trans = (control->width + 2) / 3;
		break;
	case MIPI_CSI2_DT_RAW_14:
		bits_per_pixel = 14;
		cycles_to_trans = (control->width + 2) / 3;
		break;
	default:
		bits_per_pixel = 16;
		break;
	}
	if (!control->linelenth) {
		rx_bit_clk = (unsigned long)control->mipiclk * 1000000;
        line_size = control->width;
	} else {
		rx_bit_clk = control->linelenth * control->framelenth * control->fps * bits_per_pixel;
		line_size = control->linelenth;
	}
	mipiinfo("linelenth: %d, framelenth: %d, fps: %d, bits_per_pixel: %lu, pixclk: %lu, rx_bit_clk: %lu",
			 control->linelenth, control->framelenth, control->fps, bits_per_pixel, pixclk, rx_bit_clk);
	time_ppi = (1000 * bits_per_pixel * line_size * 1000000 / rx_bit_clk);
	mipiinfo("time to transmit last pixel in ppi: %lu", time_ppi);
	hsdtime = (bits_per_pixel * line_size * pixclk / rx_bit_clk) - (control->hsaTime + control->hbpTime + cycles_to_trans);
	mipiinfo("mipi host minium hsdtime: %d", hsdtime);
	if (hsdtime < 0) {
		hsdtime = 1;
	}
	//hsdtime = (hsdtime + 16) & (~0xf);
	time_ipi = 1000 * (unsigned long)(control->hsaTime + control->hbpTime + hsdtime + cycles_to_trans) * 1000000 / pixclk;
	mipiinfo("time to transmit last pixel in ipi: %lu", time_ipi);
	return (uint16_t)hsdtime;
}

/**
 * @brief mipi_host_configure_ipi : configure ipi mode of mipi host
 *
 * @param [in] control : mipi host controller's setting
 *
 * @return int32_t : 0/-1
 */
static int32_t mipi_host_configure_ipi(mipi_host_cfg_t *control)
{
	void __iomem  *iomem = NULL;
	if (NULL == g_mipi_host) {
		mipierr("mipi host not inited!");
		return -1;
	}
	iomem = g_mipi_host->iomem;
	mipiinfo("mipi host config ipi");
	mipi_putreg(iomem + REG_MIPI_HOST_IPI_MEM_FLUSH, MIPI_HOST_MEMFLUSN_ENABLE);
	/*Select virtual channel and data type to be processed by IPI*/
	mipi_putreg(iomem + REG_MIPI_HOST_IPI_VCID, control->channel_sel[MIPIHOST_IPI_PORT_0]);
	mipi_putreg(iomem + REG_MIPI_HOST_IPI_DATA_TYPE, control->datatype);
	/*Select the IPI mode*/
	mipi_putreg(iomem + REG_MIPI_HOST_IPI_MODE, MIPI_HOST_IPI_ENABLE | (MIPI_HOST_BITWIDTH_48 << MIPI_HOST_BITWIDTH_OFFSET));
	/*Configure the IPI horizontal frame information*/
	mipi_putreg(iomem + REG_MIPI_HOST_IPI_HSA_TIME, control->hsaTime);
	mipi_putreg(iomem + REG_MIPI_HOST_IPI_HBP_TIME, control->hbpTime);
	mipi_putreg(iomem + REG_MIPI_HOST_IPI_HSD_TIME, control->hsdTime);
	mipi_putreg(iomem + REG_MIPI_HOST_IPI_ADV_FEATURES, 0x03 << 16);

	if (MIPIHOST_CHANNEL_NUM == control->channel_num) {
		/*Select virtual channel and data type to be processed by IPI*/
		mipi_putreg(iomem + REG_MIPI_HOST_IPI2_VCID, control->channel_sel[MIPIHOST_IPI_PORT_1]);
		mipi_putreg(iomem + REG_MIPI_HOST_IPI2_DATA_TYPE, control->datatype);
		/*Select the IPI mode*/
		mipi_putreg(iomem + REG_MIPI_HOST_IPI2_MODE, MIPI_HOST_IPI_ENABLE | (MIPI_HOST_BITWIDTH_48 << MIPI_HOST_BITWIDTH_OFFSET));
		/*Configure the IPI horizontal frame information*/
		mipi_putreg(iomem + REG_MIPI_HOST_IPI2_HSA_TIME, control->hsaTime);
		mipi_putreg(iomem + REG_MIPI_HOST_IPI2_HBP_TIME, control->hbpTime);
		mipi_putreg(iomem + REG_MIPI_HOST_IPI2_HSD_TIME, control->hsdTime);
		//mipi_putreg(iomem + REG_MIPI_HOST_IPI2_ADV_FEATURES, MIPI_HOST_LEGCYMODE_ENABLE);
	}
	return 0;
}

#if MIPI_HOST_INT_DBG
/**
 * @brief mipi_host_irq_enable : Enale mipi host IRQ
 *
 * @param [in] irq : IRQ Flag
 *
 * @return void
 */
static void mipi_host_irq_enable(void)
{
	uint32_t reg = 0;
	uint32_t mask = 0;
	uint32_t temp = 0;
	void __iomem  *iomem = NULL;
	if (NULL == g_mipi_host) {
		mipierr("mipi host not inited!");
		return;
	}
	iomem = g_mipi_host->iomem;
	reg = REG_MIPI_HOST_INT_MSK_PHY_FATAL;
	mask = 0xff;
	temp = mipi_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	mipi_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_PKT_FATAL;
	mask = 0xff;
	temp = mipi_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	mipi_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_FRAME_FATAL;
	mask = 0xff;
	temp = mipi_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	mipi_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_PHY;
	mask = 0xff;
	temp = mipi_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	mipi_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_PKT;
	mask = 0xff;
	temp = mipi_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	mipi_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_LINE;
	mask = 0xff;
	temp = mipi_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	mipi_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_IPI;
	mask = 0xff;
	temp = mipi_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	mipi_putreg(iomem + (reg), temp);
	return;
}

/**
 * @brief mipi_host_irq_disable : Disable mipi host IRQ
 *
 * @param [in] irq : IRQ Flag
 *
 * @return void
 */
static void mipi_host_irq_disable(void)
{
	uint32_t reg = 0;
	uint32_t mask = 0;
	uint32_t temp = 0;
	void __iomem  *iomem = NULL;
	if (NULL == g_mipi_host) {
		mipierr("mipi host not inited!");
		return;
	}
	iomem = g_mipi_host->iomem;
	reg = REG_MIPI_HOST_INT_MSK_PHY_FATAL;
	mask = 0xff;
	temp = mipi_getreg(iomem + reg);
	temp &= ~(mask);
	mipi_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_PKT_FATAL;
	mask = 0xff;
	temp = mipi_getreg(iomem + reg);
	temp &= ~(mask);
	mipi_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_FRAME_FATAL;
	mask = 0xff;
	temp = mipi_getreg(iomem + reg);
	temp &= ~(mask);
	mipi_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_PHY;
	mask = 0xff;
	temp = mipi_getreg(iomem + reg);
	temp &= ~(mask);
	mipi_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_PKT;
	mask = 0xff;
	temp = mipi_getreg(iomem + reg);
	temp &= ~(mask);
	mipi_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_LINE;
	mask = 0xff;
	temp = mipi_getreg(iomem + reg);
	temp &= ~(mask);
	mipi_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_IPI;
	mask = 0xff;
	temp = mipi_getreg(iomem + reg);
	temp &= ~(mask);
	mipi_putreg(iomem + (reg), temp);
	return;
}

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
	uint32_t      irq = 0;
	uint32_t      subirq = 0;
	void __iomem  *iomem = NULL;
	mipi_host_t   *mipi_host = (mipi_host_t *)data;
	if (NULL == g_mipi_host) {
		mipierr("mipi host not inited!");
		return IRQ_NONE;
	}
	disable_irq_nosync(this_irq);
	iomem = mipi_host->iomem;
	irq = mipi_getreg(iomem + REG_MIPI_HOST_INT_ST_MAIN);
	/* new add */
	mipi_host_states = irq;
	/* new add */
	mipidbg("mipi host irq status 0x%x\n", irq);
	if (irq & MIPI_HOST_INT_PHY_FATAL) {
		subirq = mipi_getreg(iomem + REG_MIPI_HOST_INT_ST_PHY_FATAL);
		mipidbg("mipi host PHY FATAL: 0x%x", subirq);
	}
	if (irq & MIPI_HOST_INT_PKT_FATAL) {
		subirq = mipi_getreg(iomem + REG_MIPI_HOST_INT_ST_PKT_FATAL);
		mipidbg("mipi host PKT FATAL: 0x%x", subirq);
	}
	if (irq & MIPI_HOST_INT_FRM_FATAL) {
		subirq = mipi_getreg(iomem + REG_MIPI_HOST_INT_ST_FRAME_FATAL);
		mipidbg("mipi host FRAME FATAL: 0x%x", subirq);
	}
	if (irq & MIPI_HOST_INT_PHY) {
		subirq = mipi_getreg(iomem + REG_MIPI_HOST_INT_ST_PHY);
		mipidbg("mipi host PHY ST: 0x%x", subirq);
	}
	if (irq & MIPI_HOST_INT_PKT) {
		subirq = mipi_getreg(iomem + REG_MIPI_HOST_INT_ST_PKT);
		mipidbg("mipi host PKT ST: 0x%x", subirq);
	}
	if (irq & MIPI_HOST_INT_LINE) {
		subirq = mipi_getreg(iomem + REG_MIPI_HOST_INT_ST_LINE);
		mipidbg("mipi host LINE ST: 0x%x", subirq);
	}
	if (irq & MIPI_HOST_INT_IPI) {
		subirq = mipi_getreg(iomem + REG_MIPI_HOST_INT_ST_IPI);
		mipidbg("mipi host IPI ST: 0x%x", subirq);
	}
	enable_irq(this_irq);
	return IRQ_HANDLED;
}
#endif

static int32_t mipi_host_dphy_wait_stop(mipi_host_cfg_t *control)
{
	uint16_t       ncount = 0;
	uint32_t       stopstate = 0;
	void __iomem  *iomem = NULL;
	if (NULL == g_mipi_host) {
		mipierr("mipi host not inited!");
		return -1;
	}
	iomem = g_mipi_host->iomem;
	mipiinfo("mipi host check phy stop state");
	/*Check that data lanes are in Stop state*/
	do {
		stopstate = mipi_getreg(iomem + REG_MIPI_HOST_PHY_STOPSTATE);
		if ((stopstate & 0xF) == HOST_DPHY_LANE_STOP(control->lane))
			return 0;
		mdelay(1);
		ncount++;
	} while ( mipi_host_notimeout || ncount <= HOST_DPHY_CHECK_MAX );
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
static int32_t mipi_host_dphy_start_hs_reception(void)
{
	uint16_t       ncount = 0;
	uint32_t       state = 0;
	void __iomem  *iomem = NULL;
	if (NULL == g_mipi_host) {
		mipierr("mipi host not inited!");
		return -1;
	}
	iomem = g_mipi_host->iomem;
	mipiinfo("mipi host check hs reception");
	/*Check that clock lane is in HS mode*/
	do {
		state = mipi_getreg(iomem + REG_MIPI_HOST_PHY_RX);
		if ((state & HOST_DPHY_RX_HS) == HOST_DPHY_RX_HS) {
			mipiinfo("mipi host entry hs reception");
			return 0;
		}
		ncount++;
		mdelay(1);
	} while ( mipi_host_notimeout || ncount <= HOST_DPHY_CHECK_MAX );
	mipiinfo("mipi host hs reception check error 0x%x", state);
	return -1;
}

/**
 * @brief mipi_host_start : set mipi host start working
 *
 * @param [] void :
 *
 * @return int32_t : 0/-1
 */
int32_t mipi_host_start(void)
{
	/* new add */
	mipi_host_states = 0;
	/* new add */

	if (NULL == g_mipi_host) {
		mipierr("mipi host not inited!");
		return -1;
	}
	if (!mipi_host_nocheck) {
		if (0 != mipi_host_dphy_start_hs_reception()) {
			mipierr("mipi host hs reception state error!!!");
			return -1;
		}
	}
#if MIPI_HOST_INT_DBG
	mipi_host_irq_enable();
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
int32_t mipi_host_stop(void)
{
	/* new add */
	mipi_host_states = 0;
	/* new add */
	/*stop mipi host here?*/
#if MIPI_HOST_INT_DBG
	mipi_host_irq_disable();
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
void mipi_host_deinit(void)
{
	void __iomem  *iomem = NULL;
	if (NULL == g_mipi_host) {
		mipierr("mipi host not inited!");
		return;
	}
	iomem = g_mipi_host->iomem;
#ifdef CONFIG_X2_MIPI_PHY
	mipi_host_dphy_reset();
#endif
	/* new add */
	mipi_host_states = 0;
	/* new add */
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
 * @param [in] control : mipi host controller's setting
 *
 * @return int32_t : 0/-1
 */
int32_t mipi_host_init(mipi_host_cfg_t *control)
{
	unsigned long pixclk = 0;
	void __iomem  *iomem = NULL;
	if (NULL == g_mipi_host) {
		mipierr("mipi host not inited!");
		return -1;
	}
	/* new add */
	mipi_host_states = 0;
	need_phy_reset = 0;
	mipi_delay_time = 0;
	dbg_value = 0;
	need_stop_check = 0;
	/* new add */
	iomem = g_mipi_host->iomem;
	mipiinfo("mipi host init begin");
	mipiinfo("%d lane %dx%d %dfps datatype 0x%x",
			 control->lane, control->width, control->height, control->fps, control->datatype);
	pixclk = mipi_host_pixel_clk_select(control);
	if (0 == pixclk) {
		mipierr("host pixel clk config error!");
		return -1;
	}
	ips_set_mipi_freqrange(MIPI_HOST_CFGCLKFREQRANGE, MIPI_HOST_CFGCLK_DEFAULT);
	/*Set DWC_mipi_csi2_host reset*/
	mipi_putreg(iomem + REG_MIPI_HOST_CSI2_RESETN, MIPI_HOST_CSI2_RESETN);
	/*Set Synopsys D-PHY Reset*/
	mipi_putreg(iomem + REG_MIPI_HOST_DPHY_RSTZ, MIPI_HOST_CSI2_RESETN);
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_SHUTDOWNZ, MIPI_HOST_CSI2_RESETN);

#ifdef CONFIG_X2_MIPI_PHY
	if (0 != mipi_host_dphy_initialize(control->mipiclk, control->lane, control->settle, iomem)) {
		mipierr("mipi host dphy initialize error!!!");
		mipi_host_deinit();
		return -1;
	}
#endif
	/*Clear Synopsys D-PHY Reset*/
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_SHUTDOWNZ, MIPI_HOST_CSI2_RAISE);
	mipi_putreg(iomem + REG_MIPI_HOST_DPHY_RSTZ, MIPI_HOST_CSI2_RAISE);
	/*Configure the number of active lanes*/
	mipi_putreg(iomem + REG_MIPI_HOST_N_LANES, control->lane - 1);
	udelay(1000);
	/*Release DWC_mipi_csi2_host from reset*/
	mipi_putreg(iomem + REG_MIPI_HOST_CSI2_RESETN, MIPI_HOST_CSI2_RAISE);

	if (!need_stop_check) {
		if (0 != mipi_host_dphy_wait_stop(control)) {
			/*Release DWC_mipi_csi2_host from reset*/
			mipierr("mipi host wait phy stop state error!!!");
			mipi_host_deinit();
			return -1;
		}
	}
	control->hsaTime = control->hsaTime ? control->hsaTime : MIPI_HOST_HSATIME;
	control->hbpTime = control->hbpTime ? control->hbpTime : MIPI_HOST_HBPTIME;
	control->hsdTime = control->hsdTime ? control->hsdTime : mipi_host_get_hsd(control, pixclk);
	if (0 != mipi_host_configure_ipi(control)) {
		mipierr("mipi host configure ipi error!!!");
		mipi_host_deinit();
		return -1;
	}
	mipiinfo("mipi host init end");
	return 0;
}

static int x2_mipi_host_open(struct inode *inode, struct file *file)
{
	mipi_host_t *mipi_host = dev_get_drvdata(g_mipi_host_dev);
	file->private_data = mipi_host;
	return 0;
}

static int x2_mipi_host_close(struct inode *inode, struct file *file)
{
	mipi_host_t *mipi_host = (mipi_host_t *)file->private_data;
	if (mipi_host->state != MIPI_STATE_DEFAULT) {
		mipi_host_stop();
		mipi_host_deinit();
		mipi_host->state = MIPI_STATE_DEFAULT;
	}
	return 0;
}
static long x2_mipi_host_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	mipi_host_t   *mipi_host = (mipi_host_t *)file->private_data;
	void __iomem  *iomem = mipi_host->iomem;
	reg_t          reg;
	uint32_t       regv = 0;
	int            ret = 0;
	/* Check type and command number */
	if (_IOC_TYPE(cmd) != MIPIHOSTIOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case MIPIHOSTIOC_INIT:
		{
			mipi_host_cfg_t     mipi_host_cfg;
			printk(KERN_INFO "mipi host init cmd\n");
			if (!arg) {
				mipierr("ERROR: mipi host init error, config should not be NULL");
				ret = EINVAL;
				break;
			}
			if (MIPI_STATE_DEFAULT != mipi_host->state) {
				mipierr("WARNING: mipi host re-init, pre state: 0x%x", mipi_host->state);
			}
			if (copy_from_user((void *)&mipi_host_cfg, (void __user *)arg, sizeof(mipi_host_cfg_t))) {
				mipierr("ERROR: mipi host copy data from user failed\n");
				return -EINVAL;
			}
			if (0 != (ret = mipi_host_init(&mipi_host_cfg))) {
				mipierr("ERROR: mipi host init error: %d", ret);
				ret = -1;
				return ret;
			}
			mipi_host->state = MIPI_STATE_INIT;
		}
		break;
	case MIPIHOSTIOC_DEINIT:
		{
			printk(KERN_INFO "mipi host deinit cmd\n");
			if (MIPI_STATE_DEFAULT == mipi_host->state) {
				mipiinfo("mipi host has not been init");
				break;
			}
			if (MIPI_STATE_START == mipi_host->state) {
				mipi_host_stop();
			}
			mipi_host_deinit();
			mipi_host->state = MIPI_STATE_DEFAULT;
		}
		break;
	case MIPIHOSTIOC_START:
		{
			printk(KERN_INFO "mipi host start cmd\n");
			if (MIPI_STATE_START == mipi_host->state) {
				mipiinfo("mipi already in start state");
				break;
			} else if (MIPI_STATE_INIT != mipi_host->state && MIPI_STATE_STOP != mipi_host->state) {
				mipierr("ERROR: mipi host start state error, current state: 0x%x", mipi_host->state);
				ret = EINVAL;
				break;
			}
			if (0 != (ret = mipi_host_start())) {
				mipierr("ERROR: mipi host start error: %d", ret);
				ret = -1;
				return ret;
			}
			mipi_host->state = MIPI_STATE_START;
		}
		break;
	case MIPIHOSTIOC_STOP:
		{
			printk(KERN_INFO "mipi host stop cmd\n");
			if (MIPI_STATE_STOP == mipi_host->state) {
				mipiinfo("mipi host already in stop state");
				break;
			} else if (MIPI_STATE_START != mipi_host->state) {
				mipierr("ERROR: mipi host stop state error, current state: 0x%x", mipi_host->state);
				ret = EINVAL;
				break;
			}
			if (0 != (ret = mipi_host_stop())) {
				mipierr("ERROR: mipi host stop error: %d", ret);
				ret = -1;
				return ret;
			}
			mipi_host->state = MIPI_STATE_STOP;
		}
		break;
	case MIPIHOSTIOC_READ:
		if (!arg) {
			printk(KERN_ERR "x2 mipi_host reg read error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
			printk(KERN_ERR "x2 mipi_host reg read error, copy data from user failed\n");
			return -EINVAL;
		}
		reg.value = readl(iomem + reg.offset);
		if (copy_to_user((void __user *)arg, (void *)&reg, sizeof(reg))) {
			printk(KERN_ERR "x2 mipi_host reg read error, copy data to user failed\n");
			return -EINVAL;
		}
		break;
	case MIPIHOSTIOC_WRITE:
		if (!arg) {
			printk(KERN_ERR "x2 mipi_host reg write error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
			printk(KERN_ERR "x2 mipi_host reg write error, copy data from user failed\n");
			return -EINVAL;
		}
		writel(reg.value, iomem + reg.offset);
		regv = readl(iomem + reg.offset);
		if (regv != reg.value) {
			printk(KERN_ERR "x2 mipi_host reg write error, write 0x%x got 0x%x\n", reg.value, regv);
			return -EINVAL;
		}
		break;
	default:
		break;
	}
	return 0;
}
static const struct file_operations x2_mipi_host_fops = {
	.owner		= THIS_MODULE,
	.open		= x2_mipi_host_open,
	.release	= x2_mipi_host_close,
	.unlocked_ioctl = x2_mipi_host_ioctl,
	.compat_ioctl = x2_mipi_host_ioctl,
};

static int x2_mipi_host_probe(struct platform_device *pdev)
{
	mipi_host_t     *mipi_host = NULL;
	int              ret = 0;
	dev_t            devno;
	struct resource *res;
	struct cdev     *p_cdev = &mipi_host_cdev;

	mipi_host = kzalloc(sizeof(mipi_host_t), GFP_KERNEL);
	if (mipi_host == NULL) {
		printk(KERN_ERR "mipi_host malloc failed");
		return -1;
	}

	ret = alloc_chrdev_region(&devno, 0, 1, "x2_mipi_host");
	if (ret < 0) {
		printk(KERN_ERR "Error %d while alloc chrdev x2_mipi_host", ret);
		goto err;
	}
	mipi_host_major = MAJOR(devno);
	cdev_init(p_cdev, &x2_mipi_host_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		printk(KERN_ERR "Error %d while adding x2 mipi_host cdev", ret);
		goto err;
	}
	x2_mipi_host_class = class_create(THIS_MODULE, "x2_mipi_host");
	if (IS_ERR(x2_mipi_host_class)) {
		printk(KERN_INFO "[%s:%d] class_create error\n", __func__, __LINE__);
		ret = PTR_ERR(x2_mipi_host_class);
		goto err;
	}
	g_mipi_host_dev = device_create(x2_mipi_host_class, NULL, MKDEV(mipi_host_major, 0), (void *)mipi_host, "x2_mipi_host");
	if (IS_ERR(g_mipi_host_dev)) {
		printk(KERN_ERR "[%s] deivce create error\n", __func__);
		ret = PTR_ERR(g_mipi_host_dev);
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mipi_host->iomem = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mipi_host->iomem)) {
		printk(KERN_ERR "[%s] get mem res error\n", __func__);
		ret = PTR_ERR(mipi_host->iomem);
		goto err;
	}

#if MIPI_HOST_INT_DBG
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		printk(KERN_ERR "[%s] get irq res error\n", __func__);
		ret = -ENODEV;
		goto err;
	}
	mipi_host->irq = res->start;
	ret = request_threaded_irq(mipi_host->irq,
							   mipi_host_irq_func,
							   NULL,
							   IRQF_TRIGGER_HIGH,
							   dev_name(&pdev->dev),
							   mipi_host);
	if (ret) {
		printk(KERN_ERR "[%s] request irq error %d\n", __func__, ret);
		goto err;
	}
#endif

	platform_set_drvdata(pdev, mipi_host);
	g_mipi_host = mipi_host;
	dev_info(&pdev->dev, "X2 mipi host prop OK\n");
	return 0;
err:
	class_destroy(x2_mipi_host_class);
	cdev_del(&mipi_host_cdev);
	unregister_chrdev_region(MKDEV(mipi_host_major, 0), 1);
	kzfree(mipi_host);
	return ret;
}

static int x2_mipi_host_remove(struct platform_device *pdev)
{
	mipi_host_t *mipi_host = platform_get_drvdata(pdev);
	class_destroy(x2_mipi_host_class);
	cdev_del(&mipi_host_cdev);
	unregister_chrdev_region(MKDEV(mipi_host_major, 0), 1);
	kzfree(mipi_host);
	g_mipi_host = NULL;
	return 0;
}

static const struct of_device_id x2_mipi_host_match[] = {
	{.compatible = "hobot,x2-mipi-host"},
	{}
};

MODULE_DEVICE_TABLE(of, x2_mipi_host_match);

static struct platform_driver x2_mipi_host_driver = {
	.probe  = x2_mipi_host_probe,
	.remove = x2_mipi_host_remove,
	.driver	= {
		.name = "x2_mipi_host",
		.of_match_table = x2_mipi_host_match,
	},
};

module_platform_driver(x2_mipi_host_driver);
MODULE_AUTHOR("Zhang Tianyu <tianyu.zhang@hobot.cc>");
MODULE_DESCRIPTION("X2 MIPI Host Driver");
