/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
/**
 * @file     mipi_dev_control.c
 * @brief    MIPI DEV controller control function
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

#include "x2_mipi_dev.h"
#include "x2_mipi_dev_regs.h"
#include "x2_mipi_dphy.h"
#include "x2_sif_utils.h"

#define MIPI_DEV_INT_DBG            (0)

#define MIPI_DEV_INT_VPG            (0x1)
#define MIPI_DEV_INT_IDI            (0x1<<1)
#define MIPI_DEV_INT_IPI            (0x1<<2)
#define MIPI_DEV_INT_PHY            (0x1<<3)

#define MIPI_DEV_IPI_PKT_CFG        (0xa00)
#define MIPI_DEV_IPI_MAX_FRAME      (0xffffffff)

#define MIPI_DEV_VPG_DISABLE        (0x00)
#define MIPI_DEV_VPG_ENABLE         (0x01)
#define MIPI_DEV_VPG_ORI_VERT       (0)
#define MIPI_DEV_VPG_ORI_HOR        (1)
#define MIPI_DEV_VPG_ORI            (MIPI_DEV_VPG_ORI_VERT << 16)
#define MIPI_DEV_VPG_MODE_BER       (0x01)
#define MIPI_DEV_VPG_MODE_BAR       (0x00)
#define MIPI_DEV_VPG_MODE           (MIPI_DEV_VPG_MODE_BAR << 0)
#define MIPI_DEV_VPG_LINENUM        (0x00 << 9)
#define MIPI_DEV_VPG_HSYNC_DIS      (0x00)
#define MIPI_DEV_VPG_HSYNC_EN       (0x01)
#define MIPI_DEV_VPG_HSYNC          (MIPI_DEV_VPG_HSYNC_DIS << 8)
#define MIPI_DEV_VPG_VC             (0x00 << 6)
#define MIPI_DEV_VPG_HSA_TIME       (0x4e)
#define MIPI_DEV_VPG_HBP_TIME       (0x04)
#define MIPI_DEV_VPG_HFP_TIME       (0x5f4)
#define MIPI_DEV_VPG_HLINE_TIME     (0x1000)
#define MIPI_DEV_VPG_VSA_LINES      (0x02)
#define MIPI_DEV_VPG_VBP_LINES      (0x01)
#define MIPI_DEV_VPG_VFP_LINES      (0x01)
#define MIPI_DEV_VPG_VLINE_TIME(h)  (MIPI_DEV_VPG_VSA_LINES+MIPI_DEV_VPG_VBP_LINES+MIPI_DEV_VPG_VFP_LINES+(h))
#define MIPI_DEV_VPG_START_LINE     (0x00)
#define MIPI_DEV_VPG_STEP_LINE      (0x00)
#define MIPI_DEV_CHECK_MAX          (500)

#define MIPI_DEV_VPG_DEF_MCLK       (24)
#define MIPI_DEV_VPG_DEF_PCLK       (384)
#define MIPI_DEV_VPG_DEF_SETTLE     (0x30)
#define MIPI_DEV_VPG_DEF_FPS        (30)
#define MIPI_DEV_VPG_DEF_BLANK      (0x5f4)

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

#define MIPIDEVIOC_READ        _IOWR('v', 0, reg_t)
#define MIPIDEVIOC_WRITE       _IOW('v', 1, reg_t)

typedef struct _mipi_dev_s {
	void __iomem *iomem;
	int irq;
} mipi_dev_t;

mipi_dev_t *g_mipi_dev = NULL;

static uint16_t mipi_dev_vpg_get_hline(mipi_dev_control_t * control)
{
	uint16_t tx_byte_clk = control->mipiclk / control->lane / 8;
	uint16_t hfptime = 0;
	uint16_t vfptime = 0;
	uint32_t bytes_to_trans = 0;
	uint32_t hline = 0;
	uint32_t pixclk = 0;

	switch (control->datatype) {
	case MIPI_CSI2_DT_YUV420_8:
	case MIPI_CSI2_DT_YUV422_8:
		bytes_to_trans = (16 * control->width) / 8;
		break;
	case MIPI_CSI2_DT_RAW_10:
		bytes_to_trans = (10 * control->width) / 8;
		break;
	default:
		bytes_to_trans = (16 * control->width) / 8;
		break;
	}
	hfptime = bytes_to_trans / tx_byte_clk;
	hline =
	    MIPI_DEV_VPG_HSA_TIME + MIPI_DEV_VPG_HBP_TIME + bytes_to_trans +
	    hfptime;
	hline = (hline + 16) & (~0xf);
	//hline = hline > MIPI_DEV_VPG_HLINE_TIME ? MIPI_DEV_VPG_HLINE_TIME : hline;
	sifinfo("mipi dev vpg hline: %d", hline);
	switch (control->datatype) {
	case MIPI_CSI2_DT_YUV420_8:
	case MIPI_CSI2_DT_YUV422_8:
		control->linelenth = (hline * 8) / 16;
		pixclk = control->mipiclk * 1000000 / 16;
		break;
	case MIPI_CSI2_DT_RAW_10:
		control->linelenth = (hline * 8) / 10;
		pixclk = control->mipiclk * 1000000 / 10;
		break;
	default:
		control->linelenth = (hline * 8) / 16;
		pixclk = control->mipiclk * 1000000 / 16;
		break;
	}
	vfptime = hfptime * control->height / control->width;
	control->framelenth =
	    MIPI_DEV_VPG_VSA_LINES + MIPI_DEV_VPG_VBP_LINES + control->height +
	    vfptime;
	control->framelenth = (control->framelenth >> 1) << 1;
	control->fps = pixclk / control->linelenth / control->framelenth;
	sifinfo("mipi dev vpg fps: %d, linelenth: %d, framelenth: %d",
		control->fps, control->linelenth, control->framelenth);
	return (uint16_t) hline;
}

/**
 * @brief mipi_dev_initialize_ipi : initialize dev ipi
 *
 * @param [in] control : the dev controller's setting
 *
 * @return int32_t: 0/-1
 */
static int32_t mipi_dev_initialize_ipi(mipi_dev_control_t * control)
{
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_dev) {
		siferr("mipi dev not inited!");
		return -1;
	}
	iomem = g_mipi_dev->iomem;
	/*Configure the IPI packet header */
	/*Configure the IPI line numbering */
	/*Configure IPI frame number */
	sif_putreg(iomem + REG_MIPI_DEV_IPI_PKT_CFG, MIPI_DEV_IPI_PKT_CFG | control->datatype);	/*vc change from 2'b11 to 2'b00 */
	/*Configure IPI maximum frame number */
	sif_putreg(iomem + REG_MIPI_DEV_IPI_MAX_FRAME_NUM,
		   MIPI_DEV_IPI_MAX_FRAME);
	/*Configure IPI horizontal resolution */
	sif_putreg(iomem + REG_MIPI_DEV_IPI_PIXELS, control->width);
	/*Configure IPI vertical resolution */
	sif_putreg(iomem + REG_MIPI_DEV_IPI_LINES, control->height + 2);

	return 0;
}

/**
 * @brief mipi_dev_initialize_vgp : initialize&enable dev vgp mode
 *
 * @param [in] control : the dev controller's setting
 *
 * @return int32_t: 0/-1
 */
static int32_t mipi_dev_initialize_vgp(mipi_dev_control_t * control)
{
	uint8_t ncount = 0;
	uint32_t status = 0;
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_dev) {
		siferr("mipi dev not inited!");
		return -1;
	}
	iomem = g_mipi_dev->iomem;

	sifinfo("mipi device initialize vgp begin");
	/*Disable the Video Pattern Generator */
	sif_putreg(iomem + REG_MIPI_DEV_VPG_CTRL, MIPI_DEV_VPG_DISABLE);

	/*Wait for VPG idle */
	do {
		status = sif_getreg(iomem + REG_MIPI_DEV_VPG_STATUS);
		if (ncount >= MIPI_DEV_CHECK_MAX) {
			siferr("vga status of dev is error: 0x%x", status);
			return -1;
		}
		ncount++;
	} while (MIPI_DEV_VPG_DISABLE != status);

	/*Configure the VPG mode */
	sif_putreg(iomem + REG_MIPI_DEV_VPG_MODE_CFG,
		   MIPI_DEV_VPG_ORI | MIPI_DEV_VPG_MODE);
	sif_putreg(iomem + REG_MIPI_DEV_VPG_PKT_CFG,
		   MIPI_DEV_VPG_LINENUM | MIPI_DEV_VPG_HSYNC | MIPI_DEV_VPG_VC |
		   control->datatype);
	sif_putreg(iomem + REG_MIPI_DEV_VPG_PKT_SIZE, control->width);
	sif_putreg(iomem + REG_MIPI_DEV_VPG_HSA_TIME, MIPI_DEV_VPG_HSA_TIME);
	sif_putreg(iomem + REG_MIPI_DEV_VPG_HBP_TIME, MIPI_DEV_VPG_HBP_TIME);
	sif_putreg(iomem + REG_MIPI_DEV_VPG_HLINE_TIME,
		   mipi_dev_vpg_get_hline(control));
	sif_putreg(iomem + REG_MIPI_DEV_VPG_VSA_LINES, MIPI_DEV_VPG_VSA_LINES);
	sif_putreg(iomem + REG_MIPI_DEV_VPG_VBP_LINES, MIPI_DEV_VPG_VBP_LINES);
	sif_putreg(iomem + REG_MIPI_DEV_VPG_VFP_LINES, MIPI_DEV_VPG_VFP_LINES);
	//sif_putreg(iomem+REG_MIPI_DEV_VPG_HLINE_TIME, MIPI_DEV_VPG_VLINE_TIME(control->width));
	sif_putreg(iomem + REG_MIPI_DEV_VPG_ACT_LINES, control->height);
	sif_putreg(iomem + REG_MIPI_DEV_VPG_MAX_FRAME_NUM,
		   MIPI_DEV_IPI_MAX_FRAME);
	sif_putreg(iomem + REG_MIPI_DEV_VPG_START_LINE_NUM,
		   MIPI_DEV_VPG_START_LINE);
	sif_putreg(iomem + REG_MIPI_DEV_VPG_STEP_LINE_NUM,
		   MIPI_DEV_VPG_STEP_LINE);

	/*Enalbe the Video Pattern Generator */
	sif_putreg(iomem + REG_MIPI_DEV_VPG_CTRL, MIPI_DEV_VPG_ENABLE);

	sifinfo("mipi device initialize vgp end");
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
static void mipi_dev_irq_enable(void)
{
	uint32_t reg = 0;
	uint32_t mask = 0;
	uint32_t temp = 0;
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_dev) {
		siferr("mipi dev not inited!");
		return -1;
	}
	iomem = g_mipi_dev->iomem;
	reg = REG_MIPI_DEV_INT_MASK_N_PHY;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	sif_putreg(iomem + (reg), temp);
	reg = REG_MIPI_DEV_INT_MASK_N_IPI;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	sif_putreg(iomem + (reg), temp);
#ifdef CHIP_TEST
	reg = REG_MIPI_DEV_INT_MASK_N_VPG;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(0xff);
	temp |= mask;
	sif_putreg(iomem + (reg), temp);
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
static void mipi_dev_irq_disable(void)
{
	uint32_t reg = 0;
	uint32_t mask = 0;
	uint32_t temp = 0;
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_dev) {
		siferr("mipi dev not inited!");
		return -1;
	}
	iomem = g_mipi_dev->iomem;
	reg = REG_MIPI_DEV_INT_MASK_N_PHY;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(mask);
	sif_putreg(iomem + (reg), temp);
	reg = REG_MIPI_DEV_INT_MASK_N_IPI;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(mask);
	sif_putreg(iomem + (reg), temp);
#ifdef CHIP_TEST
	reg = REG_MIPI_DEV_INT_MASK_N_VPG;
	mask = 0xff;
	temp = sif_getreg(iomem + reg);
	temp &= ~(mask);
	sif_putreg(iomem + (reg), temp);
#endif
	return;
}

/**
 * @brief mipi_dev_irq_func : mipi dev irq notify function
 *
 * @param []  :
 *
 * @return void
 */
static void mipi_dev_irq_func(void)
{
	uint32_t irq = 0;
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_dev) {
		siferr("mipi dev not inited!");
		return -1;
	}
	iomem = g_mipi_dev->iomem;
	irq = sif_getreg(iomem + REG_MIPI_DEV_INT_ST_MAIN);
	sifinfo("mipi dev irq status 0x%x\n", irq);
	if (irq & MIPI_DEV_INT_VPG) {
		irq = sif_getreg(iomem + REG_MIPI_DEV_INT_ST_VPG);
		sifinfo("mipi dev VPG ST: 0x%x", irq);
	}
	if (irq & MIPI_DEV_INT_IDI) {
		irq = sif_getreg(iomem + REG_MIPI_DEV_INT_ST_IDI);
		sifinfo("mipi dev IDI ST: 0x%x", irq);
	}
	if (irq & MIPI_DEV_INT_IPI) {
		irq = sif_getreg(iomem + REG_MIPI_DEV_INT_ST_IPI);
		sifinfo("mipi dev IPI ST: 0x%x", irq);
	}
	if (irq & MIPI_DEV_INT_PHY) {
		irq = sif_getreg(iomem + REG_MIPI_DEV_INT_ST_PHY);
		sifinfo("mipi dev PHY ST: 0x%x", irq);
	}
	return;
}
#endif

int32_t mipi_dev_start(void)
{
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_dev) {
		siferr("mipi dev not inited!");
		return -1;
	}
	iomem = g_mipi_dev->iomem;
	/*Configure the High-Speed clock */
	sif_putreg(iomem + REG_MIPI_DEV_LPCLK_CTRL, MIPI_DEV_LPCLK_CONT);
	return 0;
}

int32_t mipi_dev_stop(void)
{
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_dev) {
		siferr("mipi dev not inited!");
		return -1;
	}
	iomem = g_mipi_dev->iomem;
	/*stop mipi dev here */
	sif_putreg(iomem + REG_MIPI_DEV_LPCLK_CTRL, MIPI_DEV_LPCLK_NCONT);
	return 0;
}

/**
 * @brief mipi_dev_init : mipi dev init function
 *
 * @param [in] control : the dev controller's setting
 *
 * @return int32_t: 0/-1
 */
int32_t mipi_dev_init(mipi_dev_control_t * control)
{
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_dev) {
		siferr("mipi dev not inited!");
		return -1;
	}
	iomem = g_mipi_dev->iomem;
	sifinfo("mipi device init begin");

	/*Reset DWC_mipicsi2_device */
	sif_putreg(iomem + REG_MIPI_DEV_CSI2_RESETN, MIPI_DEV_CSI2_RESETN);

#ifdef CONFIG_X2_MIPI_PHY
	/*Initialize the PHY */
	if (0 != mipi_dev_dphy_initialize(control, iomem)) {
		siferr("mipi dev initialize error!!!");
		return -1;
	}
#endif

	/*Wake up DWC_mipicsi2_device */
	sif_putreg(iomem + REG_MIPI_DEV_CSI2_RESETN, MIPI_DEV_CSI2_RAISE);

	if (!control->vpg) {
		if (0 != mipi_dev_initialize_ipi(control)) {
			mipi_dev_deinit();
			siferr("mipi dev initialize ipi error!!!");
			return -1;
		}
	} else {
		if (0 != mipi_dev_initialize_vgp(control)) {
			mipi_dev_deinit();
			siferr("mipi dev initialize vpg error!!!");
			return -1;
		}
	}
#if MIPI_DEV_INT_DBG
	register_irq(ISR_TYPE_IRQ, INTC_MIPI_DEVI_INT_NUM, mipi_dev_irq_func);
	mipi_dev_irq_enable();
	enable_irq(ISR_TYPE_IRQ, INTC_MIPI_DEVI_INT_NUM);
	unmask_irq(ISR_TYPE_IRQ, INTC_MIPI_DEVI_INT_NUM);
#endif
	sifinfo("mipi device init end");
	return 0;
}

/**
 * @brief mipi_dev_deinit : mipi dev deinit function
 *
 * @param [] void :
 *
 * @return int32_t: 0/-1
 */
int32_t mipi_dev_deinit(void)
{
	void __iomem *iomem = NULL;
	if (NULL == g_mipi_dev) {
		siferr("mipi dev not inited!");
		return -1;
	}
	iomem = g_mipi_dev->iomem;
#if MIPI_DEV_INT_DBG
	mipi_dev_irq_disable();
#endif
#ifdef CONFIG_X2_MIPI_PHY
	mipi_dev_dphy_reset();
#endif
	sif_putreg(iomem + REG_MIPI_DEV_VPG_CTRL, MIPI_DEV_VPG_DISABLE);
	sif_putreg(iomem + REG_MIPI_DEV_CSI2_RESETN, MIPI_DEV_CSI2_RESETN);
	return 0;
}

static int x2_mipi_dev_regs_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "x2 mipi_dev reg ctrl\n");
	return 0;
}

static int x2_mipi_dev_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, x2_mipi_dev_regs_show, inode->i_private);
}

static long x2_mipi_dev_regs_ioctl(struct file *file, unsigned int cmd,
				   unsigned long arg)
{
	void __iomem *iomem = g_mipi_dev->iomem;
	reg_t reg;
	uint32_t regv = 0;
	/* Check type and command number */
	if (_IOC_TYPE(cmd) != 'v')
		return -ENOTTY;

	switch (cmd) {
	case MIPIDEVIOC_READ:
		if (!arg) {
			printk(KERN_ERR
			       "x2 mipi_dev reg read error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user
		    ((void *)&reg, (void __user *)arg, sizeof(reg))) {
			printk(KERN_ERR
			       "x2 mipi_dev reg read error, copy data from user failed\n");
			return -EINVAL;
		}
		reg.value = readl(iomem + reg.offset);
		if (copy_to_user((void __user *)arg, (void *)&reg, sizeof(reg))) {
			printk(KERN_ERR
			       "x2 mipi_dev reg read error, copy data to user failed\n");
			return -EINVAL;
		}
		break;
	case MIPIDEVIOC_WRITE:
		if (!arg) {
			printk(KERN_ERR
			       "x2 mipi_dev reg write error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user
		    ((void *)&reg, (void __user *)arg, sizeof(reg))) {
			printk(KERN_ERR
			       "x2 mipi_dev reg write error, copy data from user failed\n");
			return -EINVAL;
		}
		writel(reg.value, iomem + reg.offset);
		regv = readl(iomem + reg.offset);
		if (regv != reg.value) {
			printk(KERN_ERR
			       "x2 mipi_dev reg write error, write 0x%x got 0x%x\n",
			       reg.value, regv);
			return -EINVAL;
		}
		break;
	default:
		break;
	}
	return 0;
}

static const struct file_operations x2_mipi_dev_regs_fops = {
	.owner = THIS_MODULE,
	.open = x2_mipi_dev_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.unlocked_ioctl = x2_mipi_dev_regs_ioctl,
	.compat_ioctl = x2_mipi_dev_regs_ioctl,
};

static int mipi_dev_major = 0;
struct cdev mipi_dev_cdev;
static struct class *x2_mipi_dev_class;
static struct device *g_mipi_dev_dev;

static int x2_mipi_dev_probe(struct platform_device *pdev)
{
	mipi_dev_t *pack_dev;
	int ret = 0;
	dev_t devno;
	struct resource *res;
	struct cdev *p_cdev = &mipi_dev_cdev;

	pack_dev = devm_kmalloc(&pdev->dev, sizeof(mipi_dev_t), GFP_KERNEL);
	if (!pack_dev) {
		dev_err(&pdev->dev, "Unable to allloc mipi_dev pack dev.\n");
		return -ENOMEM;
	}

	ret = alloc_chrdev_region(&devno, 0, 1, "x2_mipi_dev");
	if (ret < 0) {
		printk(KERN_ERR "Error %d while alloc chrdev x2_mipi_dev", ret);
		goto err;
	}
	mipi_dev_major = MAJOR(devno);
	cdev_init(p_cdev, &x2_mipi_dev_regs_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		printk(KERN_ERR "Error %d while adding x2 mipi_dev cdev", ret);
		goto err;
	}
	x2_mipi_dev_class = class_create(THIS_MODULE, "x2_mipi_dev");
	if (IS_ERR(x2_mipi_dev_class)) {
		printk(KERN_INFO "[%s:%d] class_create error\n", __func__,
		       __LINE__);
		ret = PTR_ERR(x2_mipi_dev_class);
		goto err;
	}
	g_mipi_dev_dev =
	    device_create(x2_mipi_dev_class, NULL, MKDEV(mipi_dev_major, 0),
			  (void *)pack_dev, "x2_mipi_dev");
	if (IS_ERR(g_mipi_dev_dev)) {
		printk(KERN_ERR "[%s] deivce create error\n", __func__);
		ret = PTR_ERR(g_mipi_dev_dev);
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pack_dev->iomem = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pack_dev->iomem))
		return PTR_ERR(pack_dev->iomem);

	platform_set_drvdata(pdev, pack_dev);
	g_mipi_dev = pack_dev;
	dev_info(&pdev->dev, "X2 mipi dev prop OK\n");
	return 0;
err:
	class_destroy(x2_mipi_dev_class);
	cdev_del(&mipi_dev_cdev);
	unregister_chrdev_region(MKDEV(mipi_dev_major, 0), 1);
	if (pack_dev) {
		devm_kfree(&pdev->dev, pack_dev);
	}
	return ret;
}

static int x2_mipi_dev_remove(struct platform_device *pdev)
{
	mipi_dev_t *pack_dev = platform_get_drvdata(pdev);
	class_destroy(x2_mipi_dev_class);
	cdev_del(&mipi_dev_cdev);
	unregister_chrdev_region(MKDEV(mipi_dev_major, 0), 1);
	devm_kfree(&pdev->dev, pack_dev);
	g_mipi_dev = NULL;
	return 0;
}

static const struct of_device_id x2_mipi_dev_match[] = {
	{.compatible = "hobot,x2-mipi-dev"},
	{}
};

MODULE_DEVICE_TABLE(of, x2_mipi_dev_match);

static struct platform_driver x2_mipi_dev_driver = {
	.probe = x2_mipi_dev_probe,
	.remove = x2_mipi_dev_remove,
	.driver = {
		   .name = "x2_mipi_dev",
		   .of_match_table = x2_mipi_dev_match,
		   },
};

module_platform_driver(x2_mipi_dev_driver);
MODULE_AUTHOR("Zhang Tianyu <tianyu.zhang@hobot.cc>");
MODULE_DESCRIPTION("X2 MIPI Dev Driver");
