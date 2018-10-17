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

#include "x2_mipi_host.h"
#include "x2_mipi_host_regs.h"
#include "x2_mipi_dphy.h"
#include "x2_sif_utils.h"

#define MIPI_HOST_INT_DBG          (0)

#define MIPI_HOST_INT_PHY_FATAL    (0x1)
#define MIPI_HOST_INT_PKT_FATAL    (0x1<<1)
#define MIPI_HOST_INT_FRM_FATAL    (0x1<<2)
#define MIPI_HOST_INT_PHY          (0x1<<16)
#define MIPI_HOST_INT_PKT          (0x1<<17)
#define MIPI_HOST_INT_LINE         (0x1<<18)
#define MIPI_HOST_INT_IPI          (0x1<<19)

#define MIPI_HOST_PIXCLK_DEFAULT   (288)
#define MIPI_HOST_HSATIME          (0x04)
#define MIPI_HOST_HBPTIME          (0x04)
#define MIPI_HOST_HSDTIME          (0x5f4)
#define MIPI_HOST_HLINE_TIME(l)    (MIPI_HOST_HSATIME+MIPI_HOST_HBPTIME+MIPI_HOST_HSDTIME+(l))

#define MIPI_HOST_BITWIDTH_OFFSET  (8)

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

#define MIPIHOSTIOC_READ        _IOWR('v', 0, reg_t)
#define MIPIHOSTIOC_WRITE       _IOW('v', 1, reg_t)

typedef struct _mipi_host_s {
	void __iomem *iomem;
	int irq;
} mipi_host_t;

mipi_host_t *g_mipi_host = NULL;

#ifdef SYSC_VIDEO_DIV_REG
static void mipi_host_pix_clk_div(uint32_t div)
{
	uint32_t reg_value;

	reg_value = sif_getreg(iomem + SYSC_VIDEO_DIV_REG);
	sifinfo("host read sysc video div reg: 0x%x", reg_value);

	reg_value &= (~(0x7UL << 6));
	reg_value |= ((div - 1) << 6);

	sif_putreg(iomem + SYSC_VIDEO_DIV_REG, reg_value);
	sifinfo("host write sysc video div reg: 0x%x", reg_value);
}
#endif

static int32_t mipi_host_pixel_clk_select(mipi_host_control_t * control)
{
	uint16_t pixclk =
	    control->linelenth * control->framelenth * control->fps / 1000000;
	uint16_t pixclkdiv = 0;

	if (!control->linelenth || control->linelenth == control->width) {
		switch (control->datatype) {
		case MIPI_CSI2_DT_YUV420_8:
			pixclk = control->mipiclk / (8 * 3 / 2);
			break;
		case MIPI_CSI2_DT_YUV420_10:
			pixclk = control->mipiclk / (16 * 3 / 2);
			break;
		case MIPI_CSI2_DT_YUV422_8:
			pixclk = control->mipiclk / (8 * 2);
			break;
		case MIPI_CSI2_DT_YUV422_10:
			pixclk = control->mipiclk / (16 * 2);
			break;
		case MIPI_CSI2_DT_RAW_8:
			pixclk = control->mipiclk / 8;
			break;
		case MIPI_CSI2_DT_RAW_10:
			pixclk = control->mipiclk / 10;
			break;
		default:
			pixclk = control->mipiclk / 16;
			break;
		}
	}
	pixclkdiv = MIPI_HOST_PIXCLK_DEFAULT / pixclk;
	if (pixclkdiv) {
#ifdef SYSC_VIDEO_DIV_REG
		if (pixclkdiv > 1)
			mipi_host_pix_clk_div(pixclkdiv);
#endif
		control->pixclk = MIPI_HOST_PIXCLK_DEFAULT / pixclkdiv;
		sifinfo("host fifo clk pixclkdiv: %d, pixclk: %d", pixclkdiv,
			control->pixclk);
	} else {
		sifinfo("host fifo clk error ! pixclkdiv: %d, pixclk: %d",
			pixclkdiv, pixclk);
		control->pixclk = MIPI_HOST_PIXCLK_DEFAULT;
		sifinfo("host fifo use default pixclk: %d",
			MIPI_HOST_PIXCLK_DEFAULT);
	}
	return 0;
}

/**
 * @brief mipi_host_get_hsd :
 *
 * @param [] control :
 *
 * @return uint16_t
 */
static uint16_t mipi_host_get_hsd(mipi_host_control_t * control)
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
	uint32_t rx_bit_clk = 0;
	uint32_t bits_per_pixel = 0;
	uint32_t line_size = control->width;
	uint32_t cycles_to_trans = 0;
	uint32_t time_ppi = 0;
	uint32_t time_ipi = 0;
	int32_t hsdtime = 0;

	switch (control->datatype) {
	case MIPI_CSI2_DT_YUV420_8:
		bits_per_pixel = 8 * 3 / 2;
		break;
	case MIPI_CSI2_DT_YUV420_10:
		bits_per_pixel = 16 * 3 / 2;
		break;
	case MIPI_CSI2_DT_YUV422_8:
		bits_per_pixel = 8 * 2;
		break;
	case MIPI_CSI2_DT_YUV422_10:
		bits_per_pixel = 16 * 2;
		break;
	case MIPI_CSI2_DT_RAW_8:
		bits_per_pixel = 8;
		break;
	case MIPI_CSI2_DT_RAW_10:
		bits_per_pixel = 10;
		break;
	default:
		bits_per_pixel = 16;
		break;
	}
	if (!control->linelenth || control->linelenth == control->width)
		rx_bit_clk = control->mipiclk;
	else
		rx_bit_clk =
		    control->linelenth * control->framelenth * control->fps *
		    bits_per_pixel / 1000000;
	cycles_to_trans = control->width;
	sifinfo
	    ("linelenth: %d, framelenth: %d, fps: %d, bits_per_pixel: %d, pixclk: %d",
	     control->linelenth, control->framelenth, control->fps,
	     bits_per_pixel, control->pixclk);
	time_ppi = (bits_per_pixel * line_size * 1000 / rx_bit_clk);
	sifinfo("time to transmit last pixel in ppi: %d", time_ppi);
	hsdtime =
	    (bits_per_pixel * line_size * control->pixclk / rx_bit_clk) -
	    (control->hsaTime + control->hbpTime + cycles_to_trans);
	sifinfo("mipi host minium hsdtime: %d", hsdtime);
	if (hsdtime < 0) {
		hsdtime = 0;
	}
	hsdtime = (hsdtime + 16) & (~0xf);
	time_ipi =
	    (control->hsaTime + control->hbpTime + hsdtime +
	     cycles_to_trans) * 1000 / control->pixclk;
	sifinfo("time to transmit last pixel in ipi: %d", time_ipi);
	return (uint16_t) hsdtime;
}

/**
 * @brief mipi_host_configure_ipi : configure ipi mode of mipi host
 *
 * @param [in] control : mipi host controller's setting
 *
 * @return int32_t : 0/-1
 */
static int32_t mipi_host_configure_ipi(mipi_host_control_t * control)
{
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_host) {
		siferr("mipi host not inited!");
		return -1;
	}
	iomem = g_mipi_host->iomem;
	sifinfo("mipi host config ipi");
	/*Select virtual channel and data type to be processed by IPI */
	sif_putreg(iomem + REG_MIPI_HOST_IPI_DATA_TYPE, control->datatype);
	/*Select the IPI mode */
	sif_putreg(iomem + REG_MIPI_HOST_IPI_MODE,
		   control->bitWidth << MIPI_HOST_BITWIDTH_OFFSET);
	/*Configure the IPI horizontal frame information */
	sif_putreg(iomem + REG_MIPI_HOST_IPI_HSA_TIME, control->hsaTime);
	sif_putreg(iomem + REG_MIPI_HOST_IPI_HBP_TIME, control->hbpTime);
	sif_putreg(iomem + REG_MIPI_HOST_IPI_HSD_TIME, control->hsdTime);

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
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_host) {
		siferr("mipi host not inited!");
		return -1;
	}
	iomem = g_mipi_host->iomem;
	reg = REG_MIPI_HOST_INT_MSK_PHY_FATAL;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	sif_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_PKT_FATAL;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	sif_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_FRAME_FATAL;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	sif_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_PHY;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	sif_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_PKT;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	sif_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_LINE;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	sif_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_IPI;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	sif_putreg(iomem + (reg), temp);
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
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_host) {
		siferr("mipi host not inited!");
		return -1;
	}
	iomem = g_mipi_host->iomem;
	reg = REG_MIPI_HOST_INT_MSK_PHY_FATAL;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(mask);
	sif_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_PKT_FATAL;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(mask);
	sif_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_FRAME_FATAL;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(mask);
	sif_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_PHY;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(mask);
	sif_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_PKT;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(mask);
	sif_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_LINE;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(mask);
	sif_putreg(iomem + (reg), temp);
	reg = REG_MIPI_HOST_INT_MSK_IPI;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(mask);
	sif_putreg(iomem + (reg), temp);
	return;
}

/**
 * @brief mipi_host_irq_func : mipi host irq notify function
 *
 * @param []  :
 *
 * @return void
 */
static void mipi_host_irq_func(void)
{
	uint32_t irq = 0;
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_host) {
		siferr("mipi host not inited!");
		return -1;
	}
	iomem = g_mipi_host->iomem;
	irq = sif_getreg(iomem + REG_MIPI_HOST_INT_ST_MAIN);
	uart_printf("mipi host irq status 0x%x\n", irq);
	if (irq & MIPI_HOST_INT_PHY_FATAL) {
		irq = sif_getreg(iomem + REG_MIPI_HOST_INT_ST_PHY_FATAL);
		sifinfo("mipi host PHY FATAL: 0x%x", irq);
	}
	if (irq & MIPI_HOST_INT_PKT_FATAL) {
		irq = sif_getreg(iomem + REG_MIPI_HOST_INT_ST_PKT_FATAL);
		sifinfo("mipi host PKT FATAL: 0x%x", irq);
	}
	if (irq & MIPI_HOST_INT_FRM_FATAL) {
		irq = sif_getreg(iomem + REG_MIPI_HOST_INT_ST_FRAME_FATAL);
		sifinfo("mipi host FRAME FATAL: 0x%x", irq);
	}
	if (irq & MIPI_HOST_INT_PHY) {
		irq = sif_getreg(iomem + REG_MIPI_HOST_INT_ST_PHY);
		sifinfo("mipi host PHY ST: 0x%x", irq);
	}
	if (irq & MIPI_HOST_INT_PKT) {
		irq = sif_getreg(iomem + REG_MIPI_HOST_INT_ST_PKT);
		sifinfo("mipi host PKT ST: 0x%x", irq);
	}
	if (irq & MIPI_HOST_INT_LINE) {
		irq = sif_getreg(iomem + REG_MIPI_HOST_INT_ST_LINE);
		sifinfo("mipi host LINE ST: 0x%x", irq);
	}
	if (irq & MIPI_HOST_INT_IPI) {
		irq = sif_getreg(iomem + REG_MIPI_HOST_INT_ST_IPI);
		sifinfo("mipi host IPI ST: 0x%x", irq);
	}
	return;
}
#endif

/**
 * @brief mipi_host_start : set mipi host start working
 *
 * @param [] void :
 *
 * @return int32_t : 0/-1
 */
int32_t mipi_host_start(void)
{
#ifdef CONFIG_X2_MIPI_PHY
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_host) {
		siferr("mipi host not inited!");
		return -1;
	}
	iomem = g_mipi_host->iomem;
	if (0 != mipi_host_dphy_start_hs_reception()) {
		siferr("mipi host hs reception state error!!!");
		return -1;
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
int32_t mipi_host_stop(void)
{
	/*stop mipi host here? */
	return 0;
}

/**
 * @brief mipi_host_init : mipi host init function
 *
 * @param [in] control : mipi host controller's setting
 *
 * @return int32_t : 0/-1
 */
int32_t mipi_host_init(mipi_host_control_t * control)
{
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_host) {
		siferr("mipi host not inited!");
		return -1;
	}
	iomem = g_mipi_host->iomem;
	sifinfo("mipi host init begin");
	if (0 != mipi_host_pixel_clk_select(control)) {
		siferr("host pixel clk config error!");
		return -1;
	}
	/*Set DWC_mipi_csi2_host reset */
	sif_putreg(iomem + REG_MIPI_HOST_CSI2_RESETN, MIPI_HOST_CSI2_RESETN);

#ifdef CONFIG_X2_MIPI_PHY
	if (0 != mipi_host_dphy_initialize(control, iomem)) {
		siferr("mipi host dphy initialize error!!!");
		goto err;
	}
#endif
	/*Configure the number of active lanes */
	sif_putreg(iomem + REG_MIPI_HOST_N_LANES, control->lane - 1);
	/*Release DWC_mipi_csi2_host from reset */
	sif_putreg(iomem + REG_MIPI_HOST_CSI2_RESETN, MIPI_HOST_CSI2_RAISE);

#ifdef CONFIG_X2_MIPI_PHY
	if (0 != mipi_host_dphy_wait_stop(control)) {
		/*Release DWC_mipi_csi2_host from reset */
		siferr("mipi host wait phy stop state error!!!");
		goto err;
	}
#endif

	control->hsaTime = MIPI_HOST_HSATIME;
	control->hbpTime = MIPI_HOST_HBPTIME;
	control->hsdTime = MIPI_HOST_HSDTIME;
	control->hsdTime = mipi_host_get_hsd(control);
	sifinfo("mipi host hsdtime: %d", control->hsdTime);

	if (0 != mipi_host_configure_ipi(control)) {
		siferr("mipi host configure ipi error!!!");
		goto err;
	}
#if MIPI_HOST_INT_DBG
	register_irq(ISR_TYPE_IRQ, INTC_MIPI_HOST_INT_NUM, mipi_host_irq_func);
	mipi_host_irq_enable();
	enable_irq(ISR_TYPE_IRQ, INTC_MIPI_HOST_INT_NUM);
	unmask_irq(ISR_TYPE_IRQ, INTC_MIPI_HOST_INT_NUM);
#endif
	sifinfo("mipi host init end");
	return 0;
err:
#ifdef CONFIG_X2_MIPI_PHY
	mipi_dev_dphy_reset();
#endif
	sif_putreg(iomem + REG_MIPI_HOST_CSI2_RESETN, MIPI_HOST_CSI2_RESETN);
	return -1;
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
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_host) {
		siferr("mipi host not inited!");
		return;
	}
	iomem = g_mipi_host->iomem;
#if MIPI_HOST_INT_DBG
	mipi_host_irq_disable();
#endif
#ifdef CONFIG_X2_MIPI_PHY
	mipi_dev_dphy_reset();
#endif
	/*Release DWC_mipi_csi2_host from reset */
	sif_putreg(iomem + REG_MIPI_HOST_CSI2_RESETN, MIPI_HOST_CSI2_RESETN);
	return;
}

static int x2_mipi_host_regs_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "x2 mipi_host reg ctrl\n");
	return 0;
}

static int x2_mipi_host_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, x2_mipi_host_regs_show, inode->i_private);
}

static long x2_mipi_host_regs_ioctl(struct file *file, unsigned int cmd,
				    unsigned long arg)
{
	void __iomem *iomem = g_mipi_host->iomem;
	reg_t reg;
	uint32_t regv = 0;
	/* Check type and command number */
	if (_IOC_TYPE(cmd) != 'v')
		return -ENOTTY;

	switch (cmd) {
	case MIPIHOSTIOC_READ:
		if (!arg) {
			printk(KERN_ERR
			       "x2 mipi_host reg read error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user
		    ((void *)&reg, (void __user *)arg, sizeof(reg))) {
			printk(KERN_ERR
			       "x2 mipi_host reg read error, copy data from user failed\n");
			return -EINVAL;
		}
		reg.value = readl(iomem + reg.offset);
		if (copy_to_user((void __user *)arg, (void *)&reg, sizeof(reg))) {
			printk(KERN_ERR
			       "x2 mipi_host reg read error, copy data to user failed\n");
			return -EINVAL;
		}
		break;
	case MIPIHOSTIOC_WRITE:
		if (!arg) {
			printk(KERN_ERR
			       "x2 mipi_host reg write error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user
		    ((void *)&reg, (void __user *)arg, sizeof(reg))) {
			printk(KERN_ERR
			       "x2 mipi_host reg write error, copy data from user failed\n");
			return -EINVAL;
		}
		writel(reg.value, iomem + reg.offset);
		regv = readl(iomem + reg.offset);
		if (regv != reg.value) {
			printk(KERN_ERR
			       "x2 mipi_host reg write error, write 0x%x got 0x%x\n",
			       reg.value, regv);
			return -EINVAL;
		}
		break;
	default:
		break;
	}
	return 0;
}

static const struct file_operations x2_mipi_host_regs_fops = {
	.owner = THIS_MODULE,
	.open = x2_mipi_host_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.unlocked_ioctl = x2_mipi_host_regs_ioctl,
	.compat_ioctl = x2_mipi_host_regs_ioctl,
};

static int mipi_host_major = 0;
struct cdev mipi_host_cdev;
static struct class *x2_mipi_host_class;
static struct device *g_mipi_host_dev;

static int x2_mipi_host_probe(struct platform_device *pdev)
{
	mipi_host_t *pack_dev;
	int ret = 0;
	dev_t devno;
	struct resource *res;
	struct cdev *p_cdev = &mipi_host_cdev;

	pack_dev = devm_kmalloc(&pdev->dev, sizeof(mipi_host_t), GFP_KERNEL);
	if (!pack_dev) {
		dev_err(&pdev->dev, "Unable to allloc mipi_host pack dev.\n");
		return -ENOMEM;
	}

	ret = alloc_chrdev_region(&devno, 0, 1, "x2_mipi_host");
	if (ret < 0) {
		printk(KERN_ERR "Error %d while alloc chrdev x2_mipi_host",
		       ret);
		goto err;
	}
	mipi_host_major = MAJOR(devno);
	cdev_init(p_cdev, &x2_mipi_host_regs_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		printk(KERN_ERR "Error %d while adding x2 mipi_host cdev", ret);
		goto err;
	}
	x2_mipi_host_class = class_create(THIS_MODULE, "x2_mipi_host");
	if (IS_ERR(x2_mipi_host_class)) {
		printk(KERN_INFO "[%s:%d] class_create error\n", __func__,
		       __LINE__);
		ret = PTR_ERR(x2_mipi_host_class);
		goto err;
	}
	g_mipi_host_dev =
	    device_create(x2_mipi_host_class, NULL, MKDEV(mipi_host_major, 0),
			  (void *)pack_dev, "x2_mipi_host");
	if (IS_ERR(g_mipi_host_dev)) {
		printk(KERN_ERR "[%s] deivce create error\n", __func__);
		ret = PTR_ERR(g_mipi_host_dev);
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pack_dev->iomem = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pack_dev->iomem))
		return PTR_ERR(pack_dev->iomem);

	platform_set_drvdata(pdev, pack_dev);
	g_mipi_host = pack_dev;
	dev_info(&pdev->dev, "X2 mipi host prop OK\n");
	return 0;
err:
	class_destroy(x2_mipi_host_class);
	cdev_del(&mipi_host_cdev);
	unregister_chrdev_region(MKDEV(mipi_host_major, 0), 1);
	if (pack_dev) {
		devm_kfree(&pdev->dev, pack_dev);
	}
	return ret;
}

static int x2_mipi_host_remove(struct platform_device *pdev)
{
	mipi_host_t *pack_dev = platform_get_drvdata(pdev);
	class_destroy(x2_mipi_host_class);
	cdev_del(&mipi_host_cdev);
	unregister_chrdev_region(MKDEV(mipi_host_major, 0), 1);
	devm_kfree(&pdev->dev, pack_dev);
	g_mipi_host = NULL;
	return 0;
}

static const struct of_device_id x2_mipi_host_match[] = {
	{.compatible = "hobot,x2-mipi-host"},
	{}
};

MODULE_DEVICE_TABLE(of, x2_mipi_host_match);

static struct platform_driver x2_mipi_host_driver = {
	.probe = x2_mipi_host_probe,
	.remove = x2_mipi_host_remove,
	.driver = {
		   .name = "x2_mipi_host",
		   .of_match_table = x2_mipi_host_match,
		   },
};

module_platform_driver(x2_mipi_host_driver);
MODULE_AUTHOR("Zhang Tianyu <tianyu.zhang@hobot.cc>");
MODULE_DESCRIPTION("X2 MIPI Host Driver");
