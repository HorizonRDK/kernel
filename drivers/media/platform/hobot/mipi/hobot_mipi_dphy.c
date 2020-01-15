/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
/**
 * @file     hobot_mipi_dphy.c
 * @brief	 Hobot MIPI DPHY function
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
#include <linux/cdev.h>
#include <linux/uaccess.h>

#include <soc/hobot/hobot_mipi_dphy.h>

#include "hobot_mipi_host_regs.h"
#include "hobot_mipi_dev_regs.h"
#include "hobot_mipi_dphy_regs.h"
#include "hobot_mipi_utils.h"

#define MIPI_DPHY_DNAME		"mipi_dphy"
#ifdef CONFIG_HOBOT_MIPI_HOST_MAX_NUM
#define MIPI_HOST_MAX_NUM	CONFIG_HOBOT_MIPI_HOST_MAX_NUM
#else
#define MIPI_HOST_MAX_NUM	(4)
#endif
#ifdef CONFIG_HOBOT_MIPI_HOST_MAX_NUM
#define MIPI_DEV_MAX_NUM	CONFIG_HOBOT_MIPI_DEV_MAX_NUM
#else
#define MIPI_DEV_MAX_NUM	(1)
#endif

static unsigned int host_num;
module_param(host_num, uint, 0444);
static unsigned int dev_num;
module_param(dev_num, uint, 0444);

#ifdef MODULE
/* driver as ko without dts platform */
#ifdef CONFIG_HOBOT_XJ2
/* x2ips */
#define MIPI_DPHY_REG_ADDR (0xA4000000)
#elif defined CONFIG_HOBOT_XJ3
/* x3vio */
#define MIPI_DPHY_REG_ADDR (0xA4300000)
#else
#define MIPI_DPHY_REG_ADDR (0x00000000)
#endif
#define MIPI_DPHY_REG_SIZE (0x00000100)

static unsigned int reg_addr = MIPI_DPHY_REG_ADDR;
static unsigned int reg_size = MIPI_DPHY_REG_SIZE;
module_param(reg_addr, uint, 0644);
module_param(reg_size, uint, 0644);
#endif

/* module params */
unsigned int txout_freq_autolarge_enbale;
unsigned int txout_freq_gain_precent = 5;
unsigned int txout_freq_force;

module_param(txout_freq_autolarge_enbale, uint, 0644);
module_param(txout_freq_gain_precent, uint, 0644);
module_param(txout_freq_force, uint, 0644);

#define DPHY_RAISE               (1)
#define DPHY_RESETN              (0)

#define DPHY_TEST_CLEAR          (0x00000001)
#define DPHY_TEST_RESETN         (0x00000000)
#define DPHY_TEST_CLK            (0x00000002)
#define DPHY_TEST_ENABLE         (0x00010000)
#define DPHY_TEST_DATA_MAX       (4)

#define TX_REFSCLK_DEFAULT       (24)
#define TX_PLL_INPUT_DIV_MIN     (1)
#define TX_PLL_INPUT_DIV_MAX     (16)
#define TX_PLL_FB_MULTI_MIN      (64)
#define TX_PLL_FB_MULTI_MAX      (625)
#define TX_PLL_INPUT_FEQ_MIN     (2)
#define TX_PLL_INPUT_FEQ_MAX     (8)

/*test code: addr*/
#define REGS_RX_SYS_7            (0x08)
#define REGS_RX_STARTUP_OVR_4    (0xE4)
#define REGS_RX_STARTUP_OVR_5    (0xE5)
#define REGS_RX_STARTUP_OVR_17   (0xF1)
#define REGS_RX_LANE0_DDL_4      (0x60A)
#define REGS_RX_LANE0_DDL_5      (0x60B)
#define REGS_RX_LANE0_DDL_6      (0x60C)
#define REGS_RX_LANE1_DDL_4      (0x80A)
#define REGS_RX_LANE1_DDL_5      (0x80B)
#define REGS_RX_LANE1_DDL_6      (0x80C)
#define REGS_RX_LANE2_DDL_4      (0xA0A)
#define REGS_RX_LANE2_DDL_5      (0xA0B)
#define REGS_RX_LANE2_DDL_6      (0xA0C)
#define REGS_RX_LANE3_DDL_4      (0xC0A)
#define REGS_RX_LANE3_DDL_5      (0xC0B)
#define REGS_RX_LANE3_DDL_6      (0xC0C)

#define REGS_TX_SYSTIMERS_23     (0x65)
#define REGS_TX_PLL_2            (0x160)
#define REGS_TX_PLL_1            (0x15E)
#define REGS_TX_PLL_2            (0x160)
#define REGS_TX_PLL_3            (0x161)
#define REGS_TX_PLL_4            (0x162)
#define REGS_TX_PLL_17           (0x16E)
#define REGS_TX_PLL_19           (0x170)
#define REGS_TX_PLL_27           (0x178)
#define REGS_TX_PLL_28           (0x179)
#define REGS_TX_PLL_29           (0x17A)
#define REGS_TX_PLL_30           (0x17B)
#define REGS_TX_CB_2             (0x1AC)
#define REGS_TX_SLEW_5           (0x270)
#define REGS_TX_SLEW_7           (0x272)

/*test code: data*/
#define RX_CLK_SETTLE_EN         (0x01)
#define RX_CLK_SETTLE            (0x1 << 4)
#define RX_HS_SETTLE(s)          (0x80 | ((s) & 0x7F))
#define RX_SYSTEM_CONFIG         (0x38)
#define RX_OSCFREQ_HIGH(f)       (((f) & 0xF00) >> 8)
#define RX_OSCFREQ_LOW(f)        ((f) & 0xFF)
#define RX_OSCFREQ_EN            (0x1)

#define TX_HS_ZERO(s)            (0x80 | ((s) & 0x7F))
#define TX_SLEW_RATE_CAL         (0x5E)
#define TX_SLEW_RATE_CTL         (0x11)
#define TX_PLL_DIV(n)            (0x80 | ((n) << 3) | 0x2)
#define TX_PLL_MULTI_L(m)        ((m) & 0xFF)
#define TX_PLL_MULTI_H(m)        (((m) & 0x300) >> 8)
#define TX_PLL_VCO(v)            (0x81 | (((v) & 0x3F) << 1))
#define TX_PLL_CPBIAS            (0x10)
#define TX_PLL_INT_CTL           (0x4)
#define TX_PLL_PROP_CNTRL        (0x0C)
#define TX_PLL_RST_TIME_L        (0xFF)
#define TX_PLL_GEAR_SHIFT_L      (0x6)
#define TX_PLL_GEAR_SHIFT_H      (0x1)
#define TX_PLL_CLKDIV_CLK_LMT    (450)
#define TX_PLL_CLKDIV_CLK_EN     (0x10)

#ifdef CONFIG_HOBOT_MIPI_REG_OPERATE
typedef struct _reg_s {
	uint32_t offset;
	uint32_t value;
} reg_t;

#define MIPIDPHYTIOC_READ        _IOWR('v', 0, reg_t)
#define MIPIDPHYTIOC_WRITE       _IOW('v', 1, reg_t)
#endif

typedef struct _mipi_dphy_param_s {
	/* type must be: uint32_t */
	uint32_t dbg_value;
} mipi_dphy_param_t;

static const char *g_mp_param_names[] = {
	"dbg_value",
};

typedef struct _mipi_phy_s {
	mipi_phy_sub_t sub;
	uint16_t       reged;
	uint16_t       lane;
	uint16_t       mipiclk;
	uint16_t       settle;
	void          *pll_sel;
	uint32_t       init_cnt;
	uint32_t       reset_cnt;
} mipi_phy_t;

typedef struct _mipi_dphy_s {
	void __iomem     *iomem;
	spinlock_t        lock;
	mipi_dphy_param_t param;
} mipi_dphy_t;

typedef struct _mipi_pdev_s {
	dev_t             devno;
	struct cdev       cdev;
	struct device    *dev;
	mipi_dphy_t	      dphy;
	mipi_phy_t        phy_host[MIPI_HOST_MAX_NUM];
	mipi_phy_t        phy_dev[MIPI_DEV_MAX_NUM];
} mipi_pdev_t;

static int g_mp_major;
static struct class *g_mp_class;
static mipi_pdev_t g_pdev;

int32_t mipi_dphy_register(int type, int port, mipi_phy_sub_t *sub)
{
	mipi_phy_t *phy;
	struct device *dev;
	int phy_max;
	uint32_t *num;

	if (!sub)
		return -EINVAL;
	dev = sub->dev;

	if (type != MIPI_DPHY_TYPE_HOST) {
		phy = g_pdev.phy_dev;
		phy_max = MIPI_DEV_MAX_NUM;
		num = &dev_num;
	} else {
		phy = g_pdev.phy_host;
		phy_max = MIPI_HOST_MAX_NUM;
		num = &host_num;
	}
	if (port >= phy_max || phy[port].reged) {
		mipiinfo("dphy register error");
		return -EFAULT;
	}
	memset(&phy[port], 0, sizeof(mipi_phy_t));
	memcpy(&phy[port].sub, sub, sizeof(mipi_phy_sub_t));
	phy[port].reged = 1;
	*num = *num + 1;

	return 0;
}
EXPORT_SYMBOL_GPL(mipi_dphy_register);

int32_t mipi_dphy_unregister(int type, int port)
{
	mipi_phy_t *phy;
	int phy_max;
	uint32_t *num;

	if (type) {
		phy = g_pdev.phy_dev;
		phy_max = MIPI_DEV_MAX_NUM;
		num = &dev_num;
	} else {
		phy = g_pdev.phy_host;
		phy_max = MIPI_HOST_MAX_NUM;
		num = &host_num;
	}
	if (port >= phy_max) {
		return -EFAULT;
	}
	phy[port].reged = 0;
	*num = *num - 1;

	return 0;
}
EXPORT_SYMBOL_GPL(mipi_dphy_unregister);

static mipi_phy_t *mipi_dphy_get_phy(int type, void __iomem  *iomem)
{
	mipi_phy_t *phy;
	int phy_max;
	int i;

	if (type) {
		phy = g_pdev.phy_dev;
		phy_max = MIPI_DEV_MAX_NUM;
	} else {
		phy = g_pdev.phy_host;
		phy_max = MIPI_HOST_MAX_NUM;
	}

	for (i = 0; i < phy_max; i ++) {
		if (phy[i].reged && phy[i].sub.iomem == iomem)
			return &phy[i];
	}

	return NULL;
}

#ifdef CONFIG_HOBOT_MIPI_REG_OPERATE
static mipi_phy_t *mipi_dphy_get_phy_byport(int type, int port)
{
	mipi_phy_t *phy;
	int phy_max;

	if (type) {
		phy = g_pdev.phy_dev;
		phy_max = MIPI_DEV_MAX_NUM;
	} else {
		phy = g_pdev.phy_host;
		phy_max = MIPI_HOST_MAX_NUM;
	}

	if (port < phy_max && phy[port].reged)
		return &phy[port];

	return NULL;
}
#endif

typedef struct _pll_range_table_s {
	uint16_t     low;
	uint16_t     high;
	uint32_t     value;
} pll_range_table_t;

static const pll_range_table_t g_pll_range_table[] = {
	{80,   97,  0x00},
	{80,   107, 0x10},
	{83,   118, 0x20},
	{92,   128, 0x30},
	{102,  139, 0x01},
	{111,  149, 0x11},
	{121,  160, 0x21},
	{131,  170, 0x31},
	{140,  181, 0x02},
	{149,  191, 0x12},
	{159,  202, 0x22},
	{168,  212, 0x32},
	{182,  228, 0x03},
	{197,  244, 0x13},
	{211,  259, 0x23},
	{225,  275, 0x33},
	{249,  301, 0x04},
	{273,  328, 0x14},
	{297,  354, 0x25},
	{320,  380, 0x35},
	{368,  433, 0x05},
	{415,  485, 0x16},
};

typedef struct _pll_sel_table_s {
	uint16_t     osc_freq;
	uint16_t     freq;
	uint32_t     value;
} pll_sel_table_t;

static const pll_sel_table_t g_pll_sel_table[] = {
	{438, 80, 0x00},
	{438, 90, 0x10},
	{438, 100, 0x20},
	{438, 110, 0x30},
	{438, 120, 0x01},
	{438, 130, 0x11},
	{438, 140, 0x21},
	{438, 150, 0x31},
	{438, 160, 0x02},
	{438, 170, 0x12},
	{438, 180, 0x22},
	{438, 190, 0x32},
	{438, 205, 0x03},
	{438, 220, 0x13},
	{438, 235, 0x23},
	{438, 250, 0x33},
	{438, 270, 0x04},
	{438, 290, 0x14},
	{438, 310, 0x25},
	{438, 330, 0x35},
	{438, 375, 0x05},
	{438, 425, 0x16},
	{438, 475, 0x26},
	{438, 525, 0x37},
	{438, 575, 0x07},
	{438, 630, 0x18},
	{438, 680, 0x28},
	{438, 720, 0x39},
	{438, 780, 0x09},
	{438, 820, 0x19},
	{438, 880, 0x29},
	{438, 920, 0x3A},
	{438, 980, 0x0A},
	{438, 1020, 0x1A},
	{438, 1100, 0x2A},
	{438, 1150, 0x3B},
	{438, 1200, 0x0B},
	{438, 1250, 0x1B},
	{438, 1300, 0x2B},
	{438, 1350, 0x3C},
	{438, 1400, 0x0C},
	{438, 1450, 0x1C},
	{438, 1500, 0x2C},
	{271, 1550, 0x3D},
	{280, 1600, 0x0D},
	{289, 1650, 0x1D},
	{298, 1700, 0x2D},
	{306, 1750, 0x3E},
	{315, 1800, 0x0E},
	{324, 1850, 0x1E},
	{333, 1900, 0x2F},
	{341, 1950, 0x3F},
	{350, 2000, 0x0F},
	{359, 2050, 0x40},
	{368, 2100, 0x41},
	{376, 2150, 0x42},
	{385, 2200, 0x43},
	{394, 2250, 0x44},
	{403, 2300, 0x45},
	{411, 2350, 0x46},
	{420, 2400, 0x47},
	{429, 2450, 0x48},
	{438, 2500, 0x49},
	{438, 2501, 0x49},
};

static uint32_t mipi_dphy_clk_range(mipi_phy_t *phy, uint32_t mipiclk, uint16_t *osc_freq)
{
	struct device *dev = (phy) ? phy->sub.dev : g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	uint8_t  index = 0;

	for (index = 0; index < (ARRAY_SIZE(g_pll_sel_table) - 1); index++) {
		if (mipiclk >= g_pll_sel_table[index].freq &&
			mipiclk < g_pll_sel_table[index + 1].freq) {
			mipidbg("pll div mipiclk: %d, clk: %d-%d, range: 0x%x",
			 mipiclk, g_pll_sel_table[index].freq,
			 g_pll_sel_table[index + 1].freq,
			 g_pll_sel_table[index].value);
			if (osc_freq) {
				mipidbg("pll osc_freq: %d",
					g_pll_sel_table[index].osc_freq);
				*osc_freq = g_pll_sel_table[index].osc_freq;
			}
			if (phy)
				phy->pll_sel = (void *)&g_pll_sel_table[index];
			return g_pll_sel_table[index].value;
		}
	}

	mipidbg("mipi clock %d not supported", mipiclk);
	return 0;
}

/**
 * @brief mipi_host_dphy_testdata : write test data
 *
 * @param [in] testdata : testdatas' array
 * @param [in] size : size of testdata's array
 *
 * @return void
 */
static void mipi_host_dphy_testdata(mipi_phy_t *phy, void __iomem *iomem, uint16_t testcode, uint8_t testdata)
{
	struct device *dev = (phy) ? phy->sub.dev : g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	uint32_t regv = 0;

	if (!iomem)
		return;

	/*write test code*/
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_ENABLE); /*set testen to high*/
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL1, testcode >> 8);    /*set testen to low, set test code MSBS*/
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/

	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
	regv = mipi_getreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL1);
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_ENABLE | regv); /*set testen to high*/
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_ENABLE | (testcode & 0xff));  /*set test code LSBS*/
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/

	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL1, testdata);         /*set test data*/
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/

	mipidbg("mipi host dphy test code:0x%x, data: 0x%x", testcode, testdata);
}

/**
 * @brief mipi_host_initialize : initialize mipi host
 *
 * @param [in] control : mipi host controller's setting
 *
 * @return int32_t : 0/-1
 */
int32_t mipi_host_dphy_initialize(uint16_t mipiclk, uint16_t lane, uint16_t settle, void __iomem *iomem)
{
	mipi_phy_t *phy = mipi_dphy_get_phy(0, iomem);
	struct device *dev = (phy) ? phy->sub.dev : g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	uint16_t osc_freq = 438;

	mipidbg("host dphy initialize begin");
	/*Release Synopsys-PHY test codes from reset*/
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_RESETN);
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLEAR);
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN);
	/*Ensure that testclk and testen is set to low*/

	mipi_host_dphy_testdata(phy, iomem, REGS_RX_STARTUP_OVR_5, RX_CLK_SETTLE_EN);
	mipi_host_dphy_testdata(phy, iomem, REGS_RX_STARTUP_OVR_4, RX_CLK_SETTLE);
	mipi_host_dphy_testdata(phy, iomem, REGS_RX_STARTUP_OVR_17, RX_HS_SETTLE(settle));
	/*Configure the D-PHY frequency range*/
	mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_HOST, (phy) ? (phy->sub.port) : 0,
		MIPI_HSFREQRANGE, mipi_dphy_clk_range(phy, mipiclk / lane, &osc_freq));
	mipi_host_dphy_testdata(phy, iomem, REGS_RX_SYS_7, RX_SYSTEM_CONFIG);
	mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE0_DDL_4, RX_OSCFREQ_LOW(osc_freq));
	mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE0_DDL_5, RX_OSCFREQ_HIGH(osc_freq));
	mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE0_DDL_6, RX_OSCFREQ_EN);
	mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE1_DDL_4, RX_OSCFREQ_LOW(osc_freq));
	mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE1_DDL_5, RX_OSCFREQ_HIGH(osc_freq));
	mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE1_DDL_6, RX_OSCFREQ_EN);
	mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE2_DDL_4, RX_OSCFREQ_LOW(osc_freq));
	mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE2_DDL_5, RX_OSCFREQ_HIGH(osc_freq));
	mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE2_DDL_6, RX_OSCFREQ_EN);
	mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE3_DDL_4, RX_OSCFREQ_LOW(osc_freq));
	mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE3_DDL_5, RX_OSCFREQ_HIGH(osc_freq));
	mipi_host_dphy_testdata(phy, iomem, REGS_RX_LANE3_DDL_6, RX_OSCFREQ_EN);

	/* record host */
	if (phy) {
		phy->mipiclk = mipiclk;
		phy->lane= lane;
		phy->settle = settle;
		phy->init_cnt ++;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(mipi_host_dphy_initialize);

/**
 * @brief mipi_host_dphy_reset : reset host dphy
 *
 * @param [in] iomem : IO memory
 *
 * @return void
 */
void mipi_host_dphy_reset(void __iomem *iomem)
{
	mipi_phy_t *phy = mipi_dphy_get_phy(0, iomem);

	/*Release Synopsys-PHY test codes from reset*/
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_RESETN);
	mipi_putreg(iomem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLEAR);

	/* record host */
	if (phy) {
		phy->reset_cnt ++;
	}
}
EXPORT_SYMBOL_GPL(mipi_host_dphy_reset);

/**
 * @brief mipi_dev_dphy_testdata : write test data
 *
 * @param [in] testdata : testdatas' array
 * @param [in] size : size of testdata's array
 *
 * @return void
 */
static void mipi_dev_dphy_testdata(mipi_phy_t *phy, void __iomem *iomem, uint16_t testcode, uint8_t testdata)
{
	struct device *dev = (phy) ? phy->sub.dev : g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	uint32_t regv = 0;

	if (!iomem)
		return;

	/*write test code*/
	mipi_putreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL1, DPHY_TEST_ENABLE); /*set testen to high*/
	mipi_putreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
	mipi_putreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL1, testcode >> 8);    /*set testen to low, set test code MSBS*/
	mipi_putreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/

	mipi_putreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
	regv = mipi_getreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL1);
	mipi_putreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL1, DPHY_TEST_ENABLE | regv); /*set testen to high*/
	mipi_putreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL1, DPHY_TEST_ENABLE | (testcode & 0xff));  /*set test code LSBS*/
	mipi_putreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/

	mipi_putreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL1, testdata);         /*set test data*/
	mipi_putreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
	mipidbg("mipi dev dphy test code:0x%x, data: 0x%x", testcode, testdata);
}

static const pll_range_table_t g_vco_range_table[] = {
	{1150, 1250, 0x01},
	{1100, 1152, 0x01},
	{630,  1149, 0x03},
	{420,  660,  0x09},
	{320,  440,  0x0F},
	{210,  330,  0x19},
	{160,  220,  0x1F},
	{105,  165,  0x29},
	{80,   110,  0x2F},
	{53,   83,   0x39},
	{40,   55,   0x3F},
};

static uint32_t mipi_tx_vco_range(mipi_phy_t *phy, uint16_t outclk, uint16_t *outmax)
{
	struct device *dev = (phy) ? phy->sub.dev : g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	uint8_t  index = 0;

	for (index = 0; index < ARRAY_SIZE(g_vco_range_table); index++) {
		if (outclk >= g_vco_range_table[index].low &&
			outclk <= g_vco_range_table[index].high) {
			mipidbg("outclk: %d, selected: %d-%d, range: %d",
				outclk, g_vco_range_table[index].low,
				g_vco_range_table[index].high,
				g_vco_range_table[index].value);
			*outmax = g_vco_range_table[index].high;
			return g_vco_range_table[index].value;
		}
	}
	mipidbg("out clock %d not supported", outclk);
	return 0;
}

static int32_t mipi_tx_pll_div(mipi_phy_t *phy, uint16_t refsclk, uint16_t laneclk, uint8_t *n, uint16_t *m, uint16_t *vco)
{
	struct device *dev = (phy) ? phy->sub.dev : g_pdev.dev;
	mipi_dphy_tx_param_t *ptx = (phy) ? phy->sub.param : NULL;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	int32_t vco_tmp = 0;
	uint16_t n_tmp = TX_PLL_INPUT_DIV_MAX;
	uint16_t m_tmp = TX_PLL_FB_MULTI_MIN;
	uint16_t fout = 0;
	uint16_t fvco = 0;
	uint16_t fout_max = 0;
	uint16_t vco_div = 0;
	uint16_t outclk = 0;
	uint32_t s_txout_freq_autolarge_enbale =
		(ptx && ptx->txout_param_valid) ?
			ptx->txout_freq_autolarge_enbale :
			txout_freq_autolarge_enbale;

	if (!refsclk || !laneclk || NULL == n || NULL == m) {
		mipierr("pll input error!!!");
		return 0;
	}
	fout = laneclk >> 1; /* data rate(Gbps) = PLL Fout(GHz)*2 */
	vco_tmp = mipi_tx_vco_range(phy, fout, &fout_max);
	if (vco_tmp == 0) {
		mipierr("pll output clk error!!! laneclk: %d", laneclk);
		return 0;
	}
	if (s_txout_freq_autolarge_enbale) {
		mipidbg("txout freq autolarge to %dMHz\n", fout_max);
		fout = fout_max;
		laneclk = fout << 1;
	}
	vco_div = (vco_tmp >> 4) & 0x3;
	fvco = fout << vco_div;
	if ((TX_PLL_INPUT_FEQ_MIN > refsclk / TX_PLL_INPUT_DIV_MIN) ||
		(TX_PLL_INPUT_FEQ_MAX < refsclk / TX_PLL_INPUT_DIV_MAX)) {
		mipierr("pll parameter error!!! refsclk: %d, laneclk: %d", refsclk, laneclk);
		return 0;
	}
	while (n_tmp >= TX_PLL_INPUT_DIV_MIN) {
		if (refsclk / n_tmp < TX_PLL_INPUT_FEQ_MIN) {
			n_tmp--;
			continue;
		}
		m_tmp = fvco * n_tmp / refsclk;
		if (m_tmp >= TX_PLL_FB_MULTI_MIN &&
			m_tmp < TX_PLL_FB_MULTI_MAX) {
			m_tmp++;
			break;
		}
		n_tmp--;
	}
	if (n_tmp < TX_PLL_INPUT_DIV_MIN) {
		mipierr("pll output clk error!!! fvco: %d, refsclk: %d",
			fvco, refsclk);
		return 0;
	}
	*vco = vco_tmp;
	*n = n_tmp - 1;
	*m = m_tmp - 2;
	fvco = (refsclk * (*m + 2)) / (*n + 1);
	fout = fvco >> vco_div;
	outclk = fout << 1;
	mipidbg("pll div refsclk: %d, laneclk: %dMbps, n: %d, m: %d",
			 refsclk, laneclk, *n, *m);
	mipidbg("pll vco: %d, outclk: %dMbps(%dMHz)",
			 *vco, outclk, fout);
	return outclk;
}

/**
 * @brief mipi_dev_initialize_dphy : initialize dev phy
 *
 * @param [in] control : the dev controller's setting
 *
 * @return int32_t : 0/-1
 */
int32_t mipi_dev_dphy_initialize(void __iomem *iomem, uint16_t mipiclk, uint16_t lane, uint16_t settle)
{
	mipi_phy_t *phy = mipi_dphy_get_phy(1, iomem);
	struct device *dev = (phy) ? phy->sub.dev : g_pdev.dev;
	mipi_dphy_tx_param_t *ptx = (phy) ? phy->sub.param : NULL;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	uint8_t    n = 0;
	uint16_t   m = 0;
	uint16_t   vco = 0;
	uint16_t   outclk = 0;
	uint32_t s_txout_freq_force =
		(ptx && ptx->txout_param_valid) ?
			ptx->txout_freq_force:
			txout_freq_force;
	uint32_t s_txout_freq_gain_precent=
		(ptx && ptx->txout_param_valid) ?
			ptx->txout_freq_gain_precent:
			txout_freq_gain_precent;

	mipidbg("device dphy initialize dphy begin");

	if (s_txout_freq_force >= 40) {
		mipidbg("txout freq force as %dMHz", s_txout_freq_force);
		mipiclk = s_txout_freq_force * 2 * lane;
	} else if (s_txout_freq_gain_precent) {
		outclk = mipiclk * (100 + s_txout_freq_gain_precent) / 100;
		mipidbg("txout freq %dMHz gain %d%c to %dMHz",
			mipiclk / lane / 2,
			s_txout_freq_gain_precent, '%',
			outclk / lane / 2);
		mipiclk = outclk;
	}

	/*Configure the D-PHY PLL*/
	mipi_putreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN);
	/*Ensure that testclk and testen is set to low*/
	outclk = mipi_tx_pll_div(phy, TX_REFSCLK_DEFAULT, (mipiclk / lane), &n, &m, &vco);
	if (0 == outclk) {
		mipierr("pll control error!");
		return -1;
	}
	/*Configure the D-PHY frequency range*/
	mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_DEV, (phy) ? (phy->sub.port) : 0,
		MIPI_HSFREQRANGE, mipi_dphy_clk_range(phy, mipiclk / lane, NULL));
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_SLEW_5, TX_SLEW_RATE_CAL);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_SLEW_7, TX_SLEW_RATE_CTL);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_27, TX_PLL_DIV(n));
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_28, TX_PLL_MULTI_L(m));
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_29, TX_PLL_MULTI_H(m));
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_30, TX_PLL_VCO(vco));
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_SYSTIMERS_23, TX_HS_ZERO(settle));
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_1,  TX_PLL_CPBIAS);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_4,  TX_PLL_INT_CTL);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_17, TX_PLL_PROP_CNTRL);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_19, TX_PLL_RST_TIME_L);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_2,  TX_PLL_GEAR_SHIFT_L);
	mipi_dev_dphy_testdata(phy, iomem, REGS_TX_PLL_3,  TX_PLL_GEAR_SHIFT_H);
	if (outclk < TX_PLL_CLKDIV_CLK_LMT)
		mipi_dev_dphy_testdata(phy, iomem, REGS_TX_CB_2, TX_PLL_CLKDIV_CLK_EN);

	/* record host */
	if (phy) {
		phy->mipiclk = mipiclk;
		phy->lane= lane;
		phy->settle = settle;
		phy->init_cnt ++;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(mipi_dev_dphy_initialize);

/**
 * @brief mipi_dev_initialize_dphy : reset dev phy
 *
 * @param [] void :
 *
 * @return void
 */
void mipi_dev_dphy_reset(void __iomem *iomem)
{
	mipi_phy_t *phy = mipi_dphy_get_phy(1 ,iomem);

	mipi_putreg(iomem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLEAR);

	/* record host */
	if (phy) {
		phy->reset_cnt ++;
	}
}
EXPORT_SYMBOL_GPL(mipi_dev_dphy_reset);

#ifdef CONFIG_HOBOT_XJ2
static int x2ips_mipi_get_ctl(int type, int port, int region)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;
	unsigned long flags;
	u32 val;

	if (!dphy->iomem || type != MIPI_DPHY_TYPE_DEV || port > 0)
		return -1;

	spin_lock_irqsave(&dphy->lock, flags);
	val = readl(dphy->iomem + REG_X2IPS_MIPI_CTRL);
	spin_unlock_irqrestore(&dphy->lock, flags);
	switch (region) {
	case MIPI_BYPASS_GEN_HSYNC_DLY_CNT:
		val = DP_REG2V(X2IPS, BPASS_GEN_DLY, val);
		break;
	case MIPI_BYPASS_GEN_HSYNC_EN:
		val = DP_REG2V(X2IPS, BPASS_GEN_HSYNC, val);
		break;
	case MIPI_DEV_SHADOW_CLEAR:
		val = DP_REG2V(X2IPS, DEV_SHADOW_CLR, val);
		break;
	default:
		val = -1;
		break;
	}
	return val;
}

static int x2ips_mipi_set_ctl(int type, int port, int region, int value)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;
	struct device *dev = g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	unsigned long flags;
	u32 val;
	int ret = 0;

	if (!dphy->iomem || type != MIPI_DPHY_TYPE_DEV || port > 0)
		return -1;

	spin_lock_irqsave(&dphy->lock, flags);
	val = readl(dphy->iomem + REG_X2IPS_MIPI_CTRL);
	switch (region) {
	case MIPI_BYPASS_GEN_HSYNC_DLY_CNT:
		val &= ~DP_VMASK(X2IPS, BPASS_GEN_DLY);
		val |= DP_V2REG(X2IPS, BPASS_GEN_DLY, value);
		break;
	case MIPI_BYPASS_GEN_HSYNC_EN:
		val &= ~DP_VMASK(X2IPS, BPASS_GEN_HSYNC);
		val |= DP_V2REG(X2IPS, BPASS_GEN_HSYNC, value);
		break;
	case MIPI_DEV_SHADOW_CLEAR:
		val &= ~DP_VMASK(X2IPS, DEV_SHADOW_CLR);
		val |= DP_V2REG(X2IPS, DEV_SHADOW_CLR, value);
		break;
	default:
		ret = -1;
		break;
	}
	writel(val, dphy->iomem + REG_X2IPS_MIPI_CTRL);
	spin_unlock_irqrestore(&dphy->lock, flags);

	mipidbg("set mipi%s%d ctl region %d value %d, regv 0x%x\n",
			(type == MIPI_DPHY_TYPE_HOST) ? "host" : "dev",
			port, region, value, val);
	return ret;
}

static int x2ips_mipi_get_freqrange(int type, int port, int region)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;
	unsigned long flags;
	u32 val;

	if (!dphy->iomem || port > 0)
		return -1;

	spin_lock_irqsave(&dphy->lock, flags);
	val = readl(dphy->iomem + REG_X2IPS_MIPI_FREQRANGE);
	spin_unlock_irqrestore(&dphy->lock, flags);
	if (type == MIPI_DPHY_TYPE_HOST) {
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			val = DP_REG2V(X2IPS, HOST_CFGCLK_FRANGE, val);
			break;
		case MIPI_HSFREQRANGE:
			val = DP_REG2V(X2IPS, HOST_HS_FRANGE, val);
			break;
		default:
			val = -1;
			break;
		}
	} else if (type == MIPI_DPHY_TYPE_DEV) {
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			val = DP_REG2V(X2IPS, DEV_CFGCLK_FRANGE, val);
			break;
		case MIPI_HSFREQRANGE:
			val = DP_REG2V(X2IPS, DEV_HS_FRANGE, val);
			break;
		default:
			val = -1;
			break;
		}
	} else {
		val = -1;
	}

	return val;
}

static int x2ips_mipi_set_freqrange(int type, int port, int region, int value)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;
	struct device *dev = g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	unsigned long flags;
	int ret = 0;
	u32 val;

	if (!dphy->iomem || port > 0)
		return -1;

	spin_lock_irqsave(&dphy->lock, flags);
	if (type == MIPI_DPHY_TYPE_HOST) {
		val = readl(dphy->iomem + REG_X2IPS_MIPI_FREQRANGE);
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			val &= ~DP_VMASK(X2IPS, HOST_CFGCLK_FRANGE);
			val |= DP_V2REG(X2IPS, HOST_CFGCLK_FRANGE, value);
			break;
		case MIPI_HSFREQRANGE:
			val &= ~DP_VMASK(X2IPS, HOST_HS_FRANGE);
			val |= DP_V2REG(X2IPS, HOST_HS_FRANGE, value);
			break;
		default:
			ret = -1;
			break;
		}
		writel(val, dphy->iomem + REG_X2IPS_MIPI_FREQRANGE);
	} else if (type == MIPI_DPHY_TYPE_DEV) {
		val = readl(dphy->iomem + REG_X2IPS_MIPI_DEV_PLL_CTRL2);
		val &= ~DP_VMASK(X2IPS, PLL_SEL_CLR);
		val |= DP_V2REG(X2IPS, PLL_SEL_CLR, 1);
		writel(val, dphy->iomem + REG_X2IPS_MIPI_DEV_PLL_CTRL2);

		val = readl(dphy->iomem + REG_X2IPS_MIPI_FREQRANGE);
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			val &= ~DP_VMASK(X2IPS, DEV_CFGCLK_FRANGE);
			val |= DP_V2REG(X2IPS, DEV_CFGCLK_FRANGE, value);
			break;
		case MIPI_HSFREQRANGE:
			val &= ~DP_VMASK(X2IPS, DEV_HS_FRANGE);
			val |= DP_V2REG(X2IPS, DEV_HS_FRANGE, value);
			break;
		default:
			ret = -1;
			break;
		}
		writel(val, dphy->iomem + REG_X2IPS_MIPI_FREQRANGE);
	} else {
		ret = -1;
	}
	spin_unlock_irqrestore(&dphy->lock, flags);

	mipidbg("set mipi%s%d freq region %d range %d, regv 0x%x\n",
			(type == MIPI_DPHY_TYPE_HOST) ? "host" : "dev",
			port, region, value, val);
	return ret;
}

static int x2ips_mipi_get_lanemode(int type, int port, int lanemode)
{
	return 0;
}

static int x2ips_mipi_set_lanemode(int type, int port, int lanemode)
{
	return 0;
}

#else
static int x3vio_mipi_get_ctl(int type, int port, int region)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;
	unsigned long flags;
	u32 val;

	if (!dphy->iomem || type != MIPI_DPHY_TYPE_DEV || port > 0)
		return -1;

	spin_lock_irqsave(&dphy->lock, flags);
	val = readl(dphy->iomem + REG_X3VIO_MIPI_DEV_CTRL);
	spin_unlock_irqrestore(&dphy->lock, flags);
	switch (region) {
	case MIPI_BYPASS_GEN_HSYNC_DLY_CNT:
		val = DP_REG2V(X3VIO, BPASS_GEN_DLY, val);
		break;
	case MIPI_BYPASS_GEN_HSYNC_EN:
		val = DP_REG2V(X3VIO, BPASS_GEN_HSYNC, val);
		break;
	case MIPI_DEV_SHADOW_CLEAR:
		val = DP_REG2V(X3VIO, DEV_SHADOW_CLR, val);
		break;
	default:
		val = -1;
		break;
	}
	return val;
}

static int x3vio_mipi_set_ctl(int type, int port, int region, int value)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;
	struct device *dev = g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	unsigned long flags;
	u32 val;
	int ret = 0;

	if (!dphy->iomem || type != MIPI_DPHY_TYPE_DEV || port > 0)
		return -1;

	spin_lock_irqsave(&dphy->lock, flags);
	val = readl(dphy->iomem + REG_X3VIO_MIPI_DEV_CTRL);
	switch (region) {
	case MIPI_BYPASS_GEN_HSYNC_DLY_CNT:
		val &= ~DP_VMASK(X3VIO, BPASS_GEN_DLY);
		val |= DP_V2REG(X3VIO, BPASS_GEN_DLY, value);
		break;
	case MIPI_BYPASS_GEN_HSYNC_EN:
		val &= ~DP_VMASK(X3VIO, BPASS_GEN_HSYNC);
		val |= DP_V2REG(X3VIO, BPASS_GEN_HSYNC, value);
		break;
	case MIPI_DEV_SHADOW_CLEAR:
		val &= ~DP_VMASK(X3VIO, DEV_SHADOW_CLR);
		val |= DP_V2REG(X3VIO, DEV_SHADOW_CLR, value);
		break;
	default:
		ret = -1;
		break;
	}
	writel(val, dphy->iomem + REG_X3VIO_MIPI_DEV_CTRL);
	spin_unlock_irqrestore(&dphy->lock, flags);

	mipidbg("set mipi%s%d ctl region %d value %d, regv 0x%x, %d\n",
			(type == MIPI_DPHY_TYPE_HOST) ? "host" : "dev",
			port, region, value, val, ret);
	return ret;
}

static int x3vio_mipi_get_freqrange(int type, int port, int region)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;
	unsigned long flags;
	u32 reg, val;

	if (!dphy->iomem)
		return -1;

	if (type == MIPI_DPHY_TYPE_HOST && port < 4) {
		switch (port) {
		case 0:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX0;
			break;
		case 1:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX1;
			break;
		case 2:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX2;
			break;
		case 3:
		default:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX3;
			break;
		}
		spin_lock_irqsave(&dphy->lock, flags);
		val = readl(dphy->iomem + reg);
		spin_unlock_irqrestore(&dphy->lock, flags);
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			val = DP_REG2V(X3VIO, RXN_CFGCLK_FRANGE, val);
			break;
		case MIPI_HSFREQRANGE:
			val = DP_REG2V(X3VIO, RXN_HS_FRANGE, val);
			break;
		default:
			val = -1;
			break;
		}
	} else if (type == MIPI_DPHY_TYPE_DEV && port == 0) {
		spin_lock_irqsave(&dphy->lock, flags);
		val = readl(dphy->iomem + REG_X3VIO_MIPI_DEV_FREQRANGE);
		spin_unlock_irqrestore(&dphy->lock, flags);
		switch (region) {
			case MIPI_CFGCLKFREQRANGE:
				val = DP_REG2V(X3VIO, DEV_CFGCLK_FRANGE, val);
				break;
			case MIPI_HSFREQRANGE:
				val = DP_REG2V(X3VIO, DEV_HS_FRANGE, val);
				break;
			default:
				val = -1;
				break;
		}
	} else {
		val = -1;
	}

	return val;
}

static int x3vio_mipi_set_freqrange(int type, int port, int region, int value)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;
	struct device *dev = g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	unsigned long flags;
	int ret = 0;
	u32 reg, val = 0;

	if (!dphy->iomem)
		return -1;

	if (type == MIPI_DPHY_TYPE_HOST && port < 4) {
		switch (port) {
		case 0:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX0;
			break;
		case 1:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX1;
			break;
		case 2:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX2;
			break;
		case 3:
		default:
			reg = REG_X3VIO_MIPI_FREQRANGE_RX3;
			break;
		}
		spin_lock_irqsave(&dphy->lock, flags);
		val = readl(dphy->iomem + reg);
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			val &= ~DP_VMASK(X3VIO, RXN_CFGCLK_FRANGE);
			val |= DP_V2REG(X3VIO, RXN_CFGCLK_FRANGE, value);
			break;
		case MIPI_HSFREQRANGE:
			val &= ~DP_VMASK(X3VIO, RXN_HS_FRANGE);
			val |= DP_V2REG(X3VIO, RXN_HS_FRANGE, value);
			break;
		default:
			ret = -1;
			break;
		}
		writel(val, dphy->iomem + reg);
		spin_unlock_irqrestore(&dphy->lock, flags);
	} else if (type == MIPI_DPHY_TYPE_DEV && port == 0) {
		spin_lock_irqsave(&dphy->lock, flags);
		val = readl(dphy->iomem + REG_X3VIO_MIPI_DEV_PLL_CTRL2);
		val &= ~DP_VMASK(X3VIO, PLL_SEL_CLR);
		val |= DP_V2REG(X3VIO, PLL_SEL_CLR, 1);
		writel(val, dphy->iomem + REG_X3VIO_MIPI_DEV_PLL_CTRL2);

		val = readl(dphy->iomem + REG_X3VIO_MIPI_DEV_FREQRANGE);
		switch (region) {
		case MIPI_CFGCLKFREQRANGE:
			val &= ~DP_VMASK(X3VIO, DEV_CFGCLK_FRANGE);
			val |= DP_V2REG(X3VIO, DEV_CFGCLK_FRANGE, value);
			break;
		case MIPI_HSFREQRANGE:
			val &= ~DP_VMASK(X3VIO, DEV_HS_FRANGE);
			val |= DP_V2REG(X3VIO, DEV_HS_FRANGE, value);
			break;
		default:
			ret = -1;
			break;
		}
		writel(val, dphy->iomem + REG_X3VIO_MIPI_DEV_FREQRANGE);
		spin_unlock_irqrestore(&dphy->lock, flags);
	} else {
		ret = -1;
	}

	mipidbg("set mipi%s%d freq region %d range %d, regv 0x%x, %d\n",
			(type == MIPI_DPHY_TYPE_HOST) ? "host" : "dev",
			port, region, value, val, ret);
	return ret;
}

static int x3vio_mipi_get_lanemode(int type, int port)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;
	unsigned long flags;
	u32 val;

	if (!dphy->iomem)
		return -1;

	if (type == MIPI_DPHY_TYPE_HOST && port < 4) {
		spin_lock_irqsave(&dphy->lock, flags);
		val = readl(dphy->iomem + REG_X3VIO_MIPI_RX_DPHY_CTRL);
		spin_unlock_irqrestore(&dphy->lock, flags);
		if (port == 0 || port == 2)
			val = DP_REG2V(X3VIO, RX_DPHY_MODE02, val);
		else
			val = DP_REG2V(X3VIO, RX_DPHY_MODE13, val);
	} else if (type == MIPI_DPHY_TYPE_DEV && port == 0) {
		spin_lock_irqsave(&dphy->lock, flags);
		val = readl(dphy->iomem + REG_X3VIO_MIPI_TX_DPHY_CTRL);
		spin_unlock_irqrestore(&dphy->lock, flags);
		val = DP_REG2V(X3VIO, TX_DPHY_SEL, val);
	} else {
		val = -1;
	}
	return val;
}

static int x3vio_mipi_set_lanemode(int type, int port, int lanemode)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;
	struct device *dev = g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	unsigned long flags;
	int ret = 0;
	u32 val = 0;

	if (!dphy->iomem)
		return -1;

	if (type == MIPI_DPHY_TYPE_HOST && port < 4) {
		spin_lock_irqsave(&dphy->lock, flags);
		val = readl(dphy->iomem + REG_X3VIO_MIPI_RX_DPHY_CTRL);
		if (port == 0 || port == 2) {
			val &= ~DP_RMASK(X3VIO, RX_DPHY_MODE02);
			val |= DP_V2REG(X3VIO, RX_DPHY_MODE02, lanemode);
		} else {
			val &= ~DP_RMASK(X3VIO, RX_DPHY_MODE02);
			val |= DP_V2REG(X3VIO, RX_DPHY_MODE02, lanemode);
		}
		writel(val, dphy->iomem + REG_X3VIO_MIPI_RX_DPHY_CTRL);
		spin_unlock_irqrestore(&dphy->lock, flags);
	} else if (type == MIPI_DPHY_TYPE_DEV && port == 0) {
		spin_lock_irqsave(&dphy->lock, flags);
		val = readl(dphy->iomem + REG_X3VIO_MIPI_TX_DPHY_CTRL);
		val &= ~DP_RMASK(X3VIO, TX_DPHY_SEL);
		val = DP_V2REG(X3VIO, TX_DPHY_SEL, lanemode);
		writel(val, dphy->iomem + REG_X3VIO_MIPI_TX_DPHY_CTRL);
		spin_unlock_irqrestore(&dphy->lock, flags);
	} else {
		ret = -1;
	}

	mipidbg("set mipi%s%d lanemode %d, regv 0x%x, %d\n",
			(type == MIPI_DPHY_TYPE_HOST) ? "host" : "dev",
			port, lanemode, val, ret);
	return ret;
}
#endif

typedef struct _dummy_region_s {
	uint8_t region[4];
} dummy_region_t;
static dummy_region_t region_host_dummy[MIPI_HOST_MAX_NUM][3];
static dummy_region_t region_dev_dummy[MIPI_DEV_MAX_NUM][3];

static int dummy_mipi_get_ctl(int type, int port, int region)
{
	struct device *dev = g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	u32 val;

	if (type == MIPI_DPHY_TYPE_HOST && port < MIPI_HOST_MAX_NUM) {
		val = region_host_dummy[port][0].region[region];
	} else if (type == MIPI_DPHY_TYPE_HOST && port < MIPI_HOST_MAX_NUM) {
		val = region_dev_dummy[port][0].region[region];
	} else {
		val = -1;
	}

	mipidbg("get mipi%s%d ctl region %d value %d, dummp\n",
			(type == MIPI_DPHY_TYPE_HOST) ? "host" : "dev",
			port, region, val);
	return val;
}

static int dummy_mipi_set_ctl(int type, int port, int region, int value)
{
	struct device *dev = g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	int ret = 0;

	if (type == MIPI_DPHY_TYPE_HOST && port < MIPI_HOST_MAX_NUM) {
		region_host_dummy[port][0].region[region] = value;
	} else if (type == MIPI_DPHY_TYPE_HOST && port < MIPI_HOST_MAX_NUM) {
		region_dev_dummy[port][0].region[region] = value;
	} else {
		ret = -1;
	}

	mipidbg("set mipi%s%d ctl region %d value %d, dummp %d\n",
			(type == MIPI_DPHY_TYPE_HOST) ? "host" : "dev",
			port, region, value, ret);
	return ret;
}

static int dummy_mipi_get_freqrange(int type, int port, int region)
{
	struct device *dev = g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	u32 val;

	if (type == MIPI_DPHY_TYPE_HOST && port < MIPI_HOST_MAX_NUM) {
		val = region_host_dummy[port][1].region[region];
	} else if (type == MIPI_DPHY_TYPE_HOST && port < MIPI_HOST_MAX_NUM) {
		val = region_dev_dummy[port][1].region[region];
	} else {
		val = -1;
	}

	mipidbg("get mipi%s%d freq region %d value %d, dummp\n",
			(type == MIPI_DPHY_TYPE_HOST) ? "host" : "dev",
			port, region, val);
	return val;
}

static int dummy_mipi_set_freqrange(int type, int port, int region, int value)
{
	struct device *dev = g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	int ret = 0;

	if (type == MIPI_DPHY_TYPE_HOST && port < MIPI_HOST_MAX_NUM) {
		region_host_dummy[port][1].region[region] = value;
	} else if (type == MIPI_DPHY_TYPE_HOST && port < MIPI_HOST_MAX_NUM) {
		region_dev_dummy[port][1].region[region] = value;
	} else {
		ret = -1;
	}

	mipidbg("set mipi%s%d freq region %d value %d, dummp %d\n",
			(type == MIPI_DPHY_TYPE_HOST) ? "host" : "dev",
			port, region, value, ret);
	return ret;
}

static int dummy_mipi_get_lanemode(int type, int port)
{
	struct device *dev = g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	u32 val;

	if (type == MIPI_DPHY_TYPE_HOST && port < MIPI_HOST_MAX_NUM) {
		val = region_host_dummy[port][2].region[0];
	} else if (type == MIPI_DPHY_TYPE_HOST && port < MIPI_HOST_MAX_NUM) {
		val = region_dev_dummy[port][2].region[0];
	} else {
		val = -1;
	}

	mipidbg("get mipi%s%d lanemode region %d value %d, dummp\n",
			(type == MIPI_DPHY_TYPE_HOST) ? "host" : "dev",
			port, 2, val);
	return val;
}

static int dummy_mipi_set_lanemode(int type, int port, int lanemode)
{
	struct device *dev = g_pdev.dev;
	mipi_dphy_param_t *param = &g_pdev.dphy.param;
	int ret = 0;

	if (type == MIPI_DPHY_TYPE_HOST && port < MIPI_HOST_MAX_NUM) {
		region_host_dummy[port][2].region[0] = lanemode;
	} else if (type == MIPI_DPHY_TYPE_HOST && port < MIPI_HOST_MAX_NUM) {
		region_dev_dummy[port][2].region[0] = lanemode;
	} else {
		ret = -1;
	}

	mipidbg("set mipi%s%d lanemode region %d value %d, dummp %d\n",
			(type == MIPI_DPHY_TYPE_HOST) ? "host" : "dev",
			port, 2, lanemode, ret);
	return ret;
}

int mipi_dphy_get_ctl(int type, int port, int region)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;

	if (!dphy->iomem) {
		return dummy_mipi_get_ctl(type, port, region);
	} else {
#ifdef CONFIG_HOBOT_XJ2
		return x2ips_mipi_get_ctl(type, port, region);
#else
		return x3vio_mipi_get_ctl(type, port, region);
#endif
	}
}
EXPORT_SYMBOL_GPL(mipi_dphy_get_ctl);

int mipi_dphy_set_ctl(int type, int port, int region, int value)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;

	if (!dphy->iomem) {
		return dummy_mipi_set_ctl(type, port, region, value);
	} else {
#ifdef CONFIG_HOBOT_XJ2
		return x2ips_mipi_set_ctl(type, port, region, value);
#else
		return x3vio_mipi_set_ctl(type, port, region, value);
#endif
	}
}
EXPORT_SYMBOL_GPL(mipi_dphy_set_ctl);

int mipi_dphy_get_freqrange(int type, int port, int region)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;

	if (!dphy->iomem) {
		return dummy_mipi_get_freqrange(type, port, region);
	} else {
#ifdef CONFIG_HOBOT_XJ2
		return x2ips_mipi_get_freqrange(type, port, region);
#else
		return x3vio_mipi_get_freqrange(type, port, region);
#endif
	}
}
EXPORT_SYMBOL_GPL(mipi_dphy_get_freqrange);

int mipi_dphy_set_freqrange(int type, int port, int region, int value)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;

	if (!dphy->iomem) {
		return dummy_mipi_set_freqrange(type, port, region, value);
	} else {
#ifdef CONFIG_HOBOT_XJ2
		return x2ips_mipi_set_freqrange(type, port, region, value);
#else
		return x3vio_mipi_set_freqrange(type, port, region, value);
#endif
	}
}
EXPORT_SYMBOL_GPL(mipi_dphy_set_freqrange);

int mipi_dphy_get_lanemode(int type, int port)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;

	if (!dphy->iomem) {
		return dummy_mipi_get_lanemode(type, port);
	} else {
#ifdef CONFIG_HOBOT_XJ2
		return x2ips_mipi_get_lanemode(type, port);
#else
		return x3vio_mipi_get_lanemode(type, port);
#endif
	}
}
EXPORT_SYMBOL_GPL(mipi_dphy_get_lanemode);

int mipi_dphy_set_lanemode(int type, int port, int lanemode)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;

	if (!dphy->iomem) {
		return dummy_mipi_set_lanemode(type, port, lanemode);
	} else {
#ifdef CONFIG_HOBOT_XJ2
		return x2ips_mipi_set_lanemode(type, port, lanemode);
#else
		return x3vio_mipi_set_lanemode(type, port, lanemode);
#endif
	}
}
EXPORT_SYMBOL_GPL(mipi_dphy_set_lanemode);

static int hobot_mipi_dphy_regs_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "%s for %d host %d dev reg control\n",
			MIPI_DPHY_DNAME, host_num, dev_num);
	return 0;
}

static int hobot_mipi_dphy_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, hobot_mipi_dphy_regs_show, inode->i_private);
}

static long hobot_mipi_dphy_regs_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
#ifdef CONFIG_HOBOT_MIPI_REG_OPERATE
	struct device *dev = g_pdev.dev;
	mipi_phy_t *phy;
	void __iomem  *iomem;
    int offset, type, port;
#endif

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != 'v')
		return -ENOTTY;

	switch (cmd) {
#ifdef CONFIG_HOBOT_MIPI_REG_OPERATE
	case MIPIDPHYTIOC_READ:
		{
			if (!arg) {
				mipierr("reg read error, reg should not be NULL");
				return -EINVAL;
			}
			if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
				mipierr("reg read error, copy data from user failed\n");
				return -EFAULT;
			}
			offset = reg.offset & 0x00FFFFFF;
			type = (reg.offset & 0xF0000000) >> 28;
			port = (reg.offset & 0x0F000000) >> 24;
			phy = mipi_dphy_get_phy_byport(type, port);
			if (!phy || phy->sub.iomem) {
				mipierr("reg read error, %s%d not registered\n",
						(type) ? "dev" : "host", port);
				return -ENODEV;
			}
			iomem = phy->sub.iomem;
			reg.value = mipi_getreg(iomem + offset);
			if (copy_to_user((void __user *)arg, (void *)&reg, sizeof(reg))) {
				mipierr("reg read error, copy data to user failed\n");
				return -EFAULT;
			}
		}
		break;
	case MIPIDPHYTIOC_WRITE:
		{
			if (!arg) {
				mipierr("reg write error, reg should not be NULL");
				return -EINVAL;
			}
			if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
				mipierr("reg write error, copy data from user failed");
				return -EFAULT;
			}
			offset = reg.offset & 0x00FFFFFF;
			type = (reg.offset & 0xF0000000) >> 28;
			port = (reg.offset & 0x0F000000) >> 24;
			phy = mipi_dphy_get_phy_byport(type, port);
			if (!phy || phy->sub.iomem) {
				mipierr("reg write error, %s%d not registered\n",
						(type) ? "dev" : "host", port);
				return -ENODEV;
			}
			iomem = phy->sub.iomem;
			mipi_putreg(iomem + offset, reg.value);
			regv = mipi_getreg(iomem + offset);
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
static const struct file_operations hobot_mipi_dphy_regs_fops = {
	.owner		= THIS_MODULE,
	.open		= hobot_mipi_dphy_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.unlocked_ioctl = hobot_mipi_dphy_regs_ioctl,
	.compat_ioctl = hobot_mipi_dphy_regs_ioctl,
};

/* sysfs dec & add macro */
#define MIPI_DPHY_ATTR_DEC(a, m, fsh, fst) \
static struct device_attribute mp_##a##_attr = { \
	.attr   = { \
		.name = __stringify(a), \
		.mode = m, \
	}, \
	.show   = fsh, \
	.store  = fst, \
}
#define MIPI_DPHY_ATTR_ADD(a) \
	&mp_##a##_attr.attr

/* sysfs for mipi dphy devices' params */
static int mipi_dphy_param_idx(const char *name)
{
	int i;
	int param_num = ARRAY_SIZE(g_mp_param_names);

	for (i = 0; i < param_num; i++) {
		if (strcmp(g_mp_param_names[i], name) == 0)
			return i;
	}

	return -1;
}

static ssize_t mipi_dphy_param_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	mipi_pdev_t *pdev = dev_get_drvdata(dev);
	uint32_t *param = (uint32_t *)(&pdev->dphy.param);
	char *s = buf;
	int idx;

	idx = mipi_dphy_param_idx(attr->attr.name);
	if (idx >= 0) {
		s += sprintf(s, "%u\n", param[idx]);
	}

	return (s - buf);
}

static ssize_t mipi_dphy_param_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	mipi_pdev_t *pdev = dev_get_drvdata(dev);
	uint32_t *param = (uint32_t *)(&pdev->dphy.param);
	int ret, error = -EINVAL;
	int idx;
	uint32_t val;

	idx = mipi_dphy_param_idx(attr->attr.name);
	if (idx >= 0) {
		ret = kstrtouint(buf, 0, &val);
		if (!ret) {
			param[idx] = val;
			error = 0;
		}
	}

	return (error ? error : count);
}

#define MIPI_DPHY_PARAM_DEC(a) \
	MIPI_DPHY_ATTR_DEC(a, 0644, \
		mipi_dphy_param_show, mipi_dphy_param_store)
#define MIPI_DPHY_PARAM_ADD(a) \
	MIPI_DPHY_ATTR_ADD(a)

MIPI_DPHY_PARAM_DEC(dbg_value);

static struct attribute *param_attr[] = {
	MIPI_DPHY_PARAM_ADD(dbg_value),
	NULL,
};

static const struct attribute_group param_attr_group = {
	.name = __stringify(param),
	.attrs = param_attr,
};

/* sysfs for mipi dphy devices' status */
static ssize_t mipi_dphy_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	mipi_pdev_t *pdev = dev_get_drvdata(dev);
	mipi_phy_t *phy;
	pll_sel_table_t *pll_sel;
	mipi_dphy_tx_param_t *ptx;
	void __iomem *iomem = pdev->dphy.iomem;
	char *s = buf;
	int i;

#define MP_STA_SHOW(n, fmt, ...) \
		s += sprintf(s, "%-15s: " fmt "\n", #n, \
			## __VA_ARGS__)
#define MP_REG_SHOW(r, n) \
		s += sprintf(s, "0x%03x: 0x%08x - %s:MIPI_%s\n", \
			REG_##r##_MIPI_##n, \
			mipi_getreg(iomem + REG_##r##_MIPI_##n), \
			#r, #n)

	if (strcmp(attr->attr.name, "info") == 0) {
		MP_STA_SHOW(host, "%d", host_num);
		for (i = 0; i < MIPI_HOST_MAX_NUM; i++) {
			phy = &pdev->phy_host[i];
			if (!phy->reged)
				continue;
			MP_STA_SHOW(____, "%s", dev_name(phy->sub.dev));
		}
		MP_STA_SHOW(dev, "%d", dev_num);
		for (i = 0; i < MIPI_DEV_MAX_NUM; i++) {
			phy = &pdev->phy_dev[i];
			if (!phy->reged)
				continue;
			MP_STA_SHOW(____, "%s", dev_name(phy->sub.dev));
		}
	} else if (strcmp(attr->attr.name, "host") == 0) {
		for (i = 0; i < MIPI_HOST_MAX_NUM; i++) {
			phy = &pdev->phy_host[i];
			if (!phy->reged)
				continue;
			MP_STA_SHOW(host, "%d: %s --------",
					i, dev_name(phy->sub.dev));
			MP_STA_SHOW(iomem, "%p", phy->sub.iomem);
			if (phy->init_cnt) {
				MP_STA_SHOW(lane, "%u", phy->lane);
				MP_STA_SHOW(mipiclk, "%u Mbps", phy->mipiclk);
				MP_STA_SHOW(settle, "%u", phy->settle);
				pll_sel = (pll_sel_table_t *)phy->pll_sel;
				if (pll_sel) {
					MP_STA_SHOW(laneclk, "%u Mbps", pll_sel->freq);
					MP_STA_SHOW(osc_freq, "%u", pll_sel->osc_freq);
					MP_STA_SHOW(range, "%u", pll_sel->value);
				}
				MP_STA_SHOW(init_cnt, "%u", phy->init_cnt);
				MP_STA_SHOW(reset_cnt, "%u", phy->reset_cnt);
			}
		}
	} else if (strcmp(attr->attr.name, "dev") == 0) {
		for (i = 0; i < MIPI_DEV_MAX_NUM; i++) {
			phy = &pdev->phy_dev[i];
			if (!phy->reged)
				continue;
			MP_STA_SHOW(dev, "%d: %s --------",
					i, dev_name(phy->sub.dev));
			MP_STA_SHOW(iomem, "%p", phy->sub.iomem);
			if (phy->init_cnt) {
				MP_STA_SHOW(lane, "%u", phy->lane);
				MP_STA_SHOW(mipiclk, "%u Mbps", phy->mipiclk);
				MP_STA_SHOW(settle, "%u", phy->settle);
				pll_sel = (pll_sel_table_t *)phy->pll_sel;
				if (pll_sel) {
					MP_STA_SHOW(laneclk, "%u Mbps", pll_sel->freq);
					MP_STA_SHOW(osc_freq, "%u", pll_sel->osc_freq);
					MP_STA_SHOW(range, "%u", pll_sel->value);
				}
				MP_STA_SHOW(init_cnt, "%u", phy->init_cnt);
				MP_STA_SHOW(reset_cnt, "%u", phy->reset_cnt);
			}
			ptx = phy->sub.param;
			if (ptx) {
				MP_STA_SHOW(txout_param_valid, "%s",
					(ptx->txout_param_valid) ? "dev" : "dphy");
				MP_STA_SHOW(txout_freq_autolarge_enbale, "%u",
					(ptx->txout_param_valid) ?
						ptx->txout_freq_autolarge_enbale :
						txout_freq_autolarge_enbale);
				MP_STA_SHOW(txout_freq_gain_precent, "%u",
					(ptx->txout_param_valid) ?
						ptx->txout_freq_gain_precent:
						txout_freq_gain_precent);
				MP_STA_SHOW(txout_freq_force, "%u",
					(ptx->txout_param_valid) ?
						ptx->txout_freq_force:
						txout_freq_force);
			}
		}
	} else if (strcmp(attr->attr.name, "regs") == 0) {
		if (iomem) {
#ifdef CONFIG_HOBOT_XJ2
			MP_REG_SHOW(X2IPS, DEV_PLL_CTRL1);
			MP_REG_SHOW(X2IPS, DEV_PLL_CTRL2);
			MP_REG_SHOW(X2IPS, HOST_PLL_CTRL1);
			MP_REG_SHOW(X2IPS, HOST_PLL_CTRL2);
			MP_REG_SHOW(X2IPS, CTRL);
			MP_REG_SHOW(X2IPS, FREQRANGE);
#else
			MP_REG_SHOW(X3VIO, DEV_PLL_CTRL1);
			MP_REG_SHOW(X3VIO, DEV_PLL_CTRL2);
			MP_REG_SHOW(X3VIO, DEV_CTRL);
			MP_REG_SHOW(X3VIO, DEV_FREQRANGE);
			MP_REG_SHOW(X3VIO, RX0_PLL_CTRL1);
			MP_REG_SHOW(X3VIO, RX0_PLL_CTRL2);
			MP_REG_SHOW(X3VIO, RX1_PLL_CTRL1);
			MP_REG_SHOW(X3VIO, RX1_PLL_CTRL2);
			MP_REG_SHOW(X3VIO, RX2_PLL_CTRL1);
			MP_REG_SHOW(X3VIO, RX2_PLL_CTRL2);
			MP_REG_SHOW(X3VIO, RX3_PLL_CTRL1);
			MP_REG_SHOW(X3VIO, RX3_PLL_CTRL2);
			MP_REG_SHOW(X3VIO, RX0);
			MP_REG_SHOW(X3VIO, FREQRANGE_RX0);
			MP_REG_SHOW(X3VIO, RX1);
			MP_REG_SHOW(X3VIO, FREQRANGE_RX1);
			MP_REG_SHOW(X3VIO, RX2);
			MP_REG_SHOW(X3VIO, FREQRANGE_RX2);
			MP_REG_SHOW(X3VIO, RX3);
			MP_REG_SHOW(X3VIO, FREQRANGE_RX3);
			MP_REG_SHOW(X3VIO, TX_DPHY_CTRL);
			MP_REG_SHOW(X3VIO, RX_DPHY_CTRL);
#endif
		} else {
			s += sprintf(s, "not ioremap\n" );
		}
	}

	return (s - buf);
}

#define MIPI_DPHY_STATUS_DEC(a) \
	MIPI_DPHY_ATTR_DEC(a, 0444, mipi_dphy_status_show, NULL)
#define MIPI_DPHY_STATUS_ADD(a) \
	MIPI_DPHY_ATTR_ADD(a)

MIPI_DPHY_STATUS_DEC(info);
MIPI_DPHY_STATUS_DEC(host);
MIPI_DPHY_STATUS_DEC(dev);
MIPI_DPHY_STATUS_DEC(regs);

static struct attribute *status_attr[] = {
	MIPI_DPHY_STATUS_ADD(info),
	MIPI_DPHY_STATUS_ADD(host),
	MIPI_DPHY_STATUS_ADD(dev),
	MIPI_DPHY_STATUS_ADD(regs),
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

static int hobot_mipi_dphy_class_get(void)
{
	int ret = 0;

	if (!g_mp_class) {
#if defined CONFIG_ARCH_HOBOT && defined CONFIG_VIDEO_V4L2
		extern struct class *vps_class;
		g_mp_class = vps_class;
		if (!g_mp_class) {
			pr_err("[%s] vps class null\n", __func__);
			ret = -ENODEV;
		}
#else
		/* creat vps class for mipi devices */
		g_mp_class = class_create(THIS_MODULE, "vps");
		if (IS_ERR(g_mp_class)) {
			ret = PTR_ERR(g_mp_class);
			g_mp_class = NULL;
			pr_err("[%s] class create error %d\n", __func__,
					ret);
		}
#endif
	}
	return ret;
}

static int hobot_mipi_dphy_class_put(void)
{
#if !(defined CONFIG_ARCH_HOBOT && defined CONFIG_VIDEO_V4L2)
	if (g_mp_class)
		class_destroy(g_mp_class);
#endif
	g_mp_class = NULL;
	return 0;
}

struct class* mipi_dphy_class(void)
{
	hobot_mipi_dphy_class_get();
	return g_mp_class;
}
EXPORT_SYMBOL_GPL(mipi_dphy_class);

static int hobot_mipi_dphy_probe_cdev(void)
{
	int ret = 0;
	dev_t devno;
	mipi_pdev_t *pdev = &g_pdev;
	struct cdev	*p_cdev = &pdev->cdev;

	if (g_mp_major == 0) {
		ret = alloc_chrdev_region(&devno, 0, 1, MIPI_DPHY_DNAME);
		if (ret < 0) {
			pr_err("[%s] alloc chrdev %s error %d\n", __func__,
					MIPI_DPHY_DNAME, ret);
			return ret;
		}
		g_mp_major = MAJOR(devno);
	}
	ret = hobot_mipi_dphy_class_get();
	if (ret)
		return ret;
	pdev->devno = MKDEV(g_mp_major, 0);
	cdev_init(p_cdev, &hobot_mipi_dphy_regs_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		pr_err("[%s] cdev add error %d\n", __func__, ret);
		goto err_add;
	}
	pdev->dev = device_create_with_groups(g_mp_class, NULL,
			pdev->devno, (void *)pdev,
			attr_groups, MIPI_DPHY_DNAME);
	if (IS_ERR(pdev->dev)) {
		ret = PTR_ERR(pdev->dev);
		pdev->dev = NULL;
		pr_err("[%s] deivce create error %d\n", __func__, ret);
		goto err_creat;
	}

	return 0;
err_creat:
	cdev_del(&pdev->cdev);
err_add:
	return ret;
}

static int hobot_mipi_dphy_remove_cdev(void)
{
	mipi_pdev_t *pdev = &g_pdev;

	if (g_mp_class)
		device_destroy(g_mp_class, pdev->devno);
	cdev_del(&pdev->cdev);
	hobot_mipi_dphy_class_put();
	if (g_mp_major) {
		unregister_chrdev_region(MKDEV(g_mp_major, 0), 1);
		g_mp_major = 0;
	}
	pdev->dev = NULL;
	return 0;
}

#ifdef MODULE
static int hobot_mipi_dphy_remove_param(void)
{
	mipi_dphy_t *dphy = &g_pdev.dphy;

	hobot_mipi_dphy_remove_cdev();
	if (dphy->iomem)
		iounmap(dphy->iomem);
	dphy->iomem = NULL;
	return 0;
}

static int hobot_mipi_dphy_probe_param(void)
{
	mipi_pdev_t *pdev = &g_pdev;
	mipi_dphy_t *dphy = &g_pdev.dphy;
	int ret = 0;

	if (pdev->dev) {
		pr_err("[%s] pdev has been probed\n", __func__);
		return -EBUSY;
	}

	spin_lock_init(&dphy->lock);

	if (reg_addr) {
		dphy->iomem = ioremap_nocache(reg_addr, reg_size);
		if (IS_ERR(dphy->iomem)) {
			dphy->iomem = NULL;
		}
	} else {
		dphy->iomem = NULL;
	}

	ret = hobot_mipi_dphy_probe_cdev();
	if (ret) {
		goto err_cdev;
	}

	if (!dphy->iomem) {
		dev_info(pdev->dev, "probe done\n");
	} else {
#ifdef CONFIG_HOBOT_XJ2
		dev_info(pdev->dev, "probe with x2ips done\n");
#else
		dev_info(pdev->dev, "probe with x3vio done\n");
#endif
	}

	return 0;
err_cdev:
	if (dphy->iomem)
		iounmap(dphy->iomem);
	return ret;
}

#else
static int hobot_mipi_dphy_remove(struct platform_device *pdev)
{
	mipi_pdev_t *p_pdev = platform_get_drvdata(pdev);
	mipi_dphy_t *dphy = &p_pdev->dphy;

	hobot_mipi_dphy_remove_cdev();
	platform_set_drvdata(pdev, NULL);
	if (dphy->iomem)
		devm_iounmap(&pdev->dev, dphy->iomem);
	dphy->iomem = NULL;
	return 0;
}

static int hobot_mipi_dphy_probe(struct platform_device *pdev)
{
	int ret = 0;
	mipi_dphy_t *dphy = &g_pdev.dphy;
	struct resource *res;

	if (g_pdev.dev) {
		pr_err("[%s] dphy has been probed\n", __func__);
		return -EBUSY;
	}

	spin_lock_init(&dphy->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dphy->iomem = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dphy->iomem)) {
		dphy->iomem = NULL;
	}

	platform_set_drvdata(pdev, &g_pdev);

	ret = hobot_mipi_dphy_probe_cdev();
	if (ret) {
		goto err_cdev;
	}

	if (!dphy->iomem) {
		dev_info(g_pdev.dev, "probe done\n");
	} else {
#ifdef CONFIG_HOBOT_XJ2
		dev_info(g_pdev.dev, "probe with x2ips done\n");
#else
		dev_info(g_pdev.dev, "probe with x3vio done\n");
#endif
	}

	return 0;
err_cdev:
	if (dphy->iomem)
		devm_iounmap(&pdev->dev, dphy->iomem);
	return ret;
}

static const struct of_device_id hobot_mipi_dphy_match[] = {
	{.compatible = "hobot,mipi-dphy"},
	{}
};

MODULE_DEVICE_TABLE(of, hobot_mipi_dphy_match);

static struct platform_driver hobot_mipi_dphy_driver = {
	.probe  = hobot_mipi_dphy_probe,
	.remove = hobot_mipi_dphy_remove,
	.driver	= {
		.name = MIPI_DPHY_DNAME,
		.of_match_table = hobot_mipi_dphy_match,
	},
};
#endif

static int __init hobot_mipi_dphy_module_init(void)
{
	int           ret = 0;

#ifdef MODULE
	ret = hobot_mipi_dphy_probe_param();
#else
	ret = platform_driver_register(&hobot_mipi_dphy_driver);
#endif

	return ret;
}

static void __exit hobot_mipi_dphy_module_exit(void)
{
#ifdef MODULE
	hobot_mipi_dphy_remove_param();
#else
	platform_driver_unregister(&hobot_mipi_dphy_driver);
#endif
}

late_initcall(hobot_mipi_dphy_module_init);
module_exit(hobot_mipi_dphy_module_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Zhang Tianyu <tianyu.zhang@hobot.cc>");
MODULE_DESCRIPTION("X2 MIPI DPHY Driver");
