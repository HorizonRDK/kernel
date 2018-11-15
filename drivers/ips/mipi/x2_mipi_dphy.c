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

#include "x2_mipi_host_regs.h"
#include "x2_mipi_dev_regs.h"
#include "x2_mipi_dphy.h"
#include "x2_mipi_utils.h"

#define DPHY_RAISE               (1)
#define DPHY_RESETN              (0)

#define HOST_REFCLK_DEFAULT      (24)
#define DEV_REFSCLK_DEFAULT      (288)

#define DPHY_TEST_CLEAR          (0x00000001)
#define DPHY_TEST_RESETN         (0x00000000)
#define DPHY_TEST_CLK            (0x00000002)
#define DPHY_TEST_ENABLE         (0x00010000)
#define DPHY_TEST_DATA_MAX       (4)

/*test code: addr*/
#define REGS_DPHY_RX_CTL_LANE0   (0x44)
#define REGS_DPHY_PLL_INPUT_DIV  (0x17)
#define REGS_DPHY_PLL_LOOP_DIV   (0x18)
#define REGS_DPHY_PLL_CTL        (0x19)
#define REGS_DPHY_RX_HS_SETTLE   (0x75)

/*test code: data*/
#define DPHY_PLL_RANGE           (0x0A)
#define DPHY_PLL_INPUT_DIV_MIN   (2)
#define DPHY_PLL_INPUT_FEQ_MIN   (5)
#define DPHY_PLL_INPUT_FEQ_MAX   (40)
#define DPHY_PLL_LOOP_DIV_MIN    (2)
#define DPHY_PLL_LOOP_DIV_MAX    (300)
#define DPHY_PLL_RANGE           (0x0A)
#define DPHY_PLL_LOOP_DIV_L(m)   ((m)&0x1F)
#define DPHY_PLL_LOOP_DIV_H(m)   (0x80|((m)>>5))
#define DPHY_PLL_CTL             (0x30)
#define DPHY_RX_HS_SETTLE(s)     (0x80|((s)&0x7F))

typedef struct _reg_s {
	uint32_t offset;
	uint32_t value;
} reg_t;

#define MIPIDPHYTIOC_READ        _IOWR('v', 0, reg_t)
#define MIPIDPHYTIOC_WRITE       _IOW('v', 1, reg_t)

typedef struct _mipi_dphy_s {
	void __iomem *iomem;
} mipi_dphy_t;

mipi_dphy_t *g_mipi_dphy = NULL;

void __iomem *g_hostmem = NULL;
void __iomem *g_devmem = NULL;

typedef struct _pll_range_table_s {
	uint16_t low;
	uint16_t high;
	uint32_t value;
} pll_range_table_t;

static const pll_range_table_t g_pll_range_table[] = {
	{80, 89, 0x00},
	{90, 99, 0x10},
	{100, 109, 0x20},
	{110, 129, 0x01},
	{130, 139, 0x11},
	{140, 149, 0x21},
	{150, 169, 0x02},
	{170, 179, 0x12},
	{180, 199, 0x22},
	{200, 219, 0x03},
	{220, 239, 0x13},
	{240, 249, 0x23},
	{250, 269, 0x04},
	{270, 299, 0x14},
	{300, 329, 0x05},
	{330, 359, 0x15},
	{360, 399, 0x25},
	{400, 449, 0x06},
	{450, 499, 0x16},
	{500, 549, 0x07},
	{550, 599, 0x17},
	{600, 649, 0x08},
	{650, 699, 0x18},
	{700, 749, 0x09},
	{750, 799, 0x19},
	{800, 849, 0x29},
	{850, 899, 0x39},
	{900, 949, 0x0A},
	{950, 999, 0x1A},
	{1000, 1049, 0x2A},
	{1050, 1099, 0x3A},
	{1100, 1149, 0x0B},
	{1150, 1199, 0x1B},
	{1200, 1249, 0x2B},
	{1250, 1299, 0x3B},
	{1300, 1349, 0x0C},
	{1350, 1399, 0x1C},
	{1400, 1449, 0x2C},
	{1450, 1500, 0x3C},
};

static int32_t mipi_dphy_pll_div(uint16_t refsclk, uint16_t pixclk, uint8_t * n,
				 uint16_t * m)
{
	uint16_t n_tmp = DPHY_PLL_INPUT_DIV_MIN;
	uint16_t m_tmp = DPHY_PLL_LOOP_DIV_MIN;
	uint16_t outclk = 0;
	if (!refsclk || !pixclk || NULL == n || NULL == m) {
		siferr("pll input error!!!");
		return -1;
	}
	if ((DPHY_PLL_INPUT_FEQ_MIN > refsclk / DPHY_PLL_INPUT_DIV_MIN) ||
	    (pixclk < refsclk) ||
	    (pixclk / DPHY_PLL_LOOP_DIV_MAX >
	     refsclk / DPHY_PLL_INPUT_DIV_MIN)) {
		siferr("pll parameter error!!! refsclk: %d, pixclk: %d",
		       refsclk, pixclk);
	}
	while (refsclk / n_tmp > DPHY_PLL_INPUT_FEQ_MIN) {
		n_tmp++;
	}
	n_tmp--;
	outclk = refsclk / n_tmp;
	while ((outclk * m_tmp) < pixclk) {
		m_tmp += 2;
	}
	m_tmp -= 2;
	*n = n_tmp - 1;
	if (pixclk % m_tmp)
		*m = m_tmp + 1;
	else
		*m = m_tmp - 1;
	outclk = (refsclk / (*n + 1)) * (*m + 1);
	sifinfo("pll div refsclk: %d, pixclk: %d, n: %d, m: %d, outclk: %d",
		refsclk, pixclk, *n, *m, outclk);
	return 0;
}

static uint32_t mipi_dphy_pll_range(uint32_t pixclk)
{
	uint8_t index = 0;
	for (index = 0;
	     index < sizeof(g_pll_range_table) / sizeof(pll_range_table_t);
	     index++) {
		if (pixclk <= g_pll_range_table[index].high)
			break;
	}
	sifinfo("pll div pixclk: %d, range value: %d, selected range: %d-%d",
		pixclk, g_pll_range_table[index].value,
		g_pll_range_table[index].low, g_pll_range_table[index].high);
	return g_pll_range_table[index].value << 1;
}

/**
 * @brief mipi_host_dphy_testcode : write testcode to host phy
 *
 * @param [in] testcode : test code
 *
 * @return void
 */
static void mipi_host_dphy_testcode(uint16_t testcode)
{
	sif_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN);
	sif_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL1,
		   DPHY_TEST_ENABLE | testcode);
	sif_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);
	sif_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN);
}

/**
 * @brief mipi_host_dphy_testdata : write test data
 *
 * @param [in] testdata : testdatas' array
 * @param [in] size : size of testdata's array
 *
 * @return void
 */
static void mipi_host_dphy_testdata(uint8_t * testdata, uint8_t size)
{
	uint8_t count = 0;
	while (count < size) {
		sif_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0,
			   DPHY_TEST_RESETN);
		sifinfo("mipi host dphy test data: 0x%x", testdata[count]);
		sif_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL1,
			   testdata[count]);
		sif_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0,
			   DPHY_TEST_CLK);
		sif_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0,
			   DPHY_TEST_RESETN);
		count++;
	}
}

/**
 * @brief mipi_host_initialize : initialize mipi host
 *
 * @param [in] control : mipi host controller's setting
 *
 * @return int32_t : 0/-1
 */
int32_t mipi_host_dphy_initialize(uint16_t mipiclk, uint16_t lane,
				  uint16_t settle, void __iomem * iomem)
{
	uint8_t n = 0;
	uint16_t m = 0;
	uint8_t testdata[DPHY_TEST_DATA_MAX] = { 0, };
	g_hostmem = iomem;

	sifinfo("mipi host initialize begin");
	/*Release Synopsys-PHY test codes from reset */
	sif_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_RESETN);
	sif_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLEAR);
	/*Configure the D-PHY frequency range */
	if (0 !=
	    mipi_dphy_pll_div(HOST_REFCLK_DEFAULT, (mipiclk / lane), &n, &m)) {
		siferr("pll control error!");
		return -1;
	}
	mipi_host_dphy_testcode(REGS_DPHY_RX_CTL_LANE0);
	testdata[0] = mipi_dphy_pll_range(mipiclk / lane);
	mipi_host_dphy_testdata(testdata, 1);

	mipi_host_dphy_testcode(REGS_DPHY_PLL_CTL);
	testdata[0] = DPHY_PLL_CTL;
	mipi_host_dphy_testdata(testdata, 1);

	mipi_host_dphy_testcode(REGS_DPHY_PLL_INPUT_DIV);
	testdata[0] = n;
	mipi_host_dphy_testdata(testdata, 1);

	mipi_host_dphy_testcode(REGS_DPHY_PLL_LOOP_DIV);
	testdata[0] = DPHY_PLL_LOOP_DIV_L(m);
	testdata[1] = DPHY_PLL_LOOP_DIV_H(m);
	mipi_host_dphy_testdata(testdata, 2);

	mipi_host_dphy_testcode(REGS_DPHY_RX_HS_SETTLE);
	testdata[0] = DPHY_RX_HS_SETTLE(settle);
	mipi_host_dphy_testdata(testdata, 1);
	return 0;
}

/**
 * @brief mipi_host_dphy_reset : reset host dphy
 *
 * @param [] void :
 *
 * @return void
 */
void mipi_host_dphy_reset(void)
{
	/*Set Synopsys D-PHY Reset */
	sif_putreg(g_hostmem + REG_MIPI_HOST_DPHY_RSTZ, DPHY_RESETN);
	sif_putreg(g_hostmem + REG_MIPI_HOST_PHY_SHUTDOWNZ, DPHY_RESETN);
	/*Release Synopsys-PHY test codes from reset */
	sif_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_RESETN);
	sif_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLEAR);
}

/**
 * @brief mipi_dev_dphy_testcode : write testcode to phy
 *
 * @param [in] testcode : test code
 *
 * @return void
 */
static void mipi_dev_dphy_testcode(uint16_t testcode)
{
	sif_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN);
	sif_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL1,
		   DPHY_TEST_ENABLE | testcode);
	sif_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLK);
	sif_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN);
}

/**
 * @brief mipi_dev_dphy_testdata : write test data
 *
 * @param [in] testdata : testdatas' array
 * @param [in] size : size of testdatas' array
 *
 * @return void
 */
static void mipi_dev_dphy_testdata(uint8_t * testdata, uint8_t size)
{
	uint8_t count = 0;
	while (count < size) {
		sif_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0,
			   DPHY_TEST_RESETN);
		sifinfo("mipi dev dphy test data: 0x%x", testdata[count]);
		sif_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL1,
			   testdata[count]);
		sif_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0,
			   DPHY_TEST_CLK);
		sif_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0,
			   DPHY_TEST_RESETN);
		count++;
	}
}

/**
 * @brief mipi_dev_initialize_dphy : initialize dev phy
 *
 * @param [in] control : the dev controller's setting
 *
 * @return int32_t : 0/-1
 */
int32_t mipi_dev_dphy_initialize(uint16_t mipiclk, uint16_t lane,
				 void __iomem * iomem)
{
	uint8_t testdata[DPHY_TEST_DATA_MAX] = { 0, };
	uint8_t n = 0;
	uint16_t m = 0;
	uint16_t outclk = 0;
	g_devmem = iomem;

	sifinfo("mipi device initialize dphy begin");
	/*Configure the D-PHY PLL */
	sif_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN);
	if (0 !=
	    mipi_dphy_pll_div(DEV_REFSCLK_DEFAULT, (mipiclk / lane), &n, &m)) {
		siferr("pll control error!");
		return -1;
	}
	outclk = (DEV_REFSCLK_DEFAULT / (n + 1)) * (m + 1);

	mipi_dev_dphy_testcode(REGS_DPHY_RX_CTL_LANE0);
	testdata[0] = mipi_dphy_pll_range(outclk);
	mipi_dev_dphy_testdata(testdata, 1);

	mipi_dev_dphy_testcode(REGS_DPHY_PLL_CTL);
	testdata[0] = DPHY_PLL_CTL;
	mipi_dev_dphy_testdata(testdata, 1);

	mipi_dev_dphy_testcode(REGS_DPHY_PLL_INPUT_DIV);
	testdata[0] = n;
	mipi_dev_dphy_testdata(testdata, 1);

	mipi_dev_dphy_testcode(REGS_DPHY_PLL_LOOP_DIV);
	testdata[0] = DPHY_PLL_LOOP_DIV_L(m);
	testdata[1] = DPHY_PLL_LOOP_DIV_H(m);
	mipi_dev_dphy_testdata(testdata, 2);
	return 0;
}

/**
 * @brief mipi_dev_initialize_dphy : reset dev phy
 *
 * @param [] void :
 *
 * @return void
 */
void mipi_dev_dphy_reset(void)
{
	sif_putreg(g_devmem + REG_MIPI_DEV_PHY_RSTZ, DPHY_RESETN);
	sif_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLEAR);
}

static int x2_mipi_dphy_regs_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "x2 mipi_dphy reg ctrl\n");
	return 0;
}

static int x2_mipi_dphy_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, x2_mipi_dphy_regs_show, inode->i_private);
}

static long x2_mipi_dphy_regs_ioctl(struct file *file, unsigned int cmd,
				    unsigned long arg)
{
	void __iomem *iomem = g_mipi_dphy->iomem;
	reg_t reg;
	uint32_t regv = 0;
	/* Check type and command number */
	if (_IOC_TYPE(cmd) != 'v')
		return -ENOTTY;

	switch (cmd) {
	case MIPIDPHYTIOC_READ:
		if (!arg) {
			printk(KERN_ERR
			       "x2 mipi_dphy reg read error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user
		    ((void *)&reg, (void __user *)arg, sizeof(reg))) {
			printk(KERN_ERR
			       "x2 mipi_dphy reg read error, copy data from user failed\n");
			return -EINVAL;
		}
		reg.value = readl(iomem + reg.offset);
		if (copy_to_user((void __user *)arg, (void *)&reg, sizeof(reg))) {
			printk(KERN_ERR
			       "x2 mipi_dphy reg read error, copy data to user failed\n");
			return -EINVAL;
		}
		break;
	case MIPIDPHYTIOC_WRITE:
		if (!arg) {
			printk(KERN_ERR
			       "x2 mipi_dphy reg write error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user
		    ((void *)&reg, (void __user *)arg, sizeof(reg))) {
			printk(KERN_ERR
			       "x2 mipi_dphy reg write error, copy data from user failed\n");
			return -EINVAL;
		}
		writel(reg.value, iomem + reg.offset);
		regv = readl(iomem + reg.offset);
		if (regv != reg.value) {
			printk(KERN_ERR
			       "x2 mipi_dphy reg write error, write 0x%x got 0x%x\n",
			       reg.value, regv);
			return -EINVAL;
		}
		break;
	default:
		break;
	}
	return 0;
}

static const struct file_operations x2_mipi_dphy_regs_fops = {
	.owner = THIS_MODULE,
	.open = x2_mipi_dphy_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.unlocked_ioctl = x2_mipi_dphy_regs_ioctl,
	.compat_ioctl = x2_mipi_dphy_regs_ioctl,
};

static int mipi_dphy_major = 0;
struct cdev mipi_dphy_cdev;
static struct class *x2_mipi_dphy_class;
static struct device *g_mipi_dphy_dev;

static int x2_mipi_dphy_probe(struct platform_device *pdev)
{
	mipi_dphy_t *pack_dev;
	int ret = 0;
	dev_t devno;
	struct resource *res;
	struct cdev *p_cdev = &mipi_dphy_cdev;

	pack_dev = devm_kmalloc(&pdev->dev, sizeof(mipi_dphy_t), GFP_KERNEL);
	if (!pack_dev) {
		dev_err(&pdev->dev, "Unable to allloc mipi_dphy pack dev.\n");
		return -ENOMEM;
	}

	ret = alloc_chrdev_region(&devno, 0, 1, "x2_mipi_dphy");
	if (ret < 0) {
		printk(KERN_ERR "Error %d while alloc chrdev x2_mipi_dphy",
		       ret);
		goto err;
	}
	mipi_dphy_major = MAJOR(devno);
	cdev_init(p_cdev, &x2_mipi_dphy_regs_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		printk(KERN_ERR "Error %d while adding x2 mipi_dphy cdev", ret);
		goto err;
	}
	x2_mipi_dphy_class = class_create(THIS_MODULE, "x2_mipi_dphy");
	if (IS_ERR(x2_mipi_dphy_class)) {
		printk(KERN_INFO "[%s:%d] class_create error\n", __func__,
		       __LINE__);
		ret = PTR_ERR(x2_mipi_dphy_class);
		goto err;
	}
	g_mipi_dphy_dev =
	    device_create(x2_mipi_dphy_class, NULL, MKDEV(mipi_dphy_major, 0),
			  (void *)pack_dev, "x2_mipi_dphy");
	if (IS_ERR(g_mipi_dphy_dev)) {
		printk(KERN_ERR "[%s] deivce create error\n", __func__);
		ret = PTR_ERR(g_mipi_dphy_dev);
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pack_dev->iomem = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pack_dev->iomem))
		return PTR_ERR(pack_dev->iomem);

	platform_set_drvdata(pdev, pack_dev);
	g_mipi_dphy = pack_dev;
	dev_info(&pdev->dev, "X2 mipi dev prop OK\n");
	return 0;
err:
	class_destroy(x2_mipi_dphy_class);
	cdev_del(&mipi_dphy_cdev);
	unregister_chrdev_region(MKDEV(mipi_dphy_major, 0), 1);
	if (pack_dev) {
		devm_kfree(&pdev->dev, pack_dev);
	}
	return ret;
}

static int x2_mipi_dphy_remove(struct platform_device *pdev)
{
	mipi_dphy_t *pack_dev = platform_get_drvdata(pdev);
	class_destroy(x2_mipi_dphy_class);
	cdev_del(&mipi_dphy_cdev);
	unregister_chrdev_region(MKDEV(mipi_dphy_major, 0), 1);
	devm_kfree(&pdev->dev, pack_dev);
	g_mipi_dphy = NULL;
	return 0;
}

static const struct of_device_id x2_mipi_dphy_match[] = {
	{.compatible = "x2,mipi_dphy"},
	{}
};

MODULE_DEVICE_TABLE(of, x2_mipi_dphy_match);

static struct platform_driver x2_mipi_dphy_driver = {
	.probe = x2_mipi_dphy_probe,
	.remove = x2_mipi_dphy_remove,
	.driver = {
		   .name = "x2_mipi_dphy",
		   .of_match_table = x2_mipi_dphy_match,
		   },
};

module_platform_driver(x2_mipi_dphy_driver);
MODULE_AUTHOR("Zhang Tianyu <tianyu.zhang@hobot.cc>");
MODULE_DESCRIPTION("X2 MIPI DPHY Driver");
