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
#include "x2/x2_ips.h"

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
#define RX_OSCFREQ_HIGH          (0x1)
#define RX_OSCFREQ_LOW           (0xB6)
#define RX_OSCFREQ_EN            (0x1)

#define TX_HS_ZERO(s)            (0x80 | ((s) & 0x7F))
#define TX_SLEW_RATE_CAL         (0x5E)
#define TX_SLEW_RATE_CTL         (0x11)
#define TX_PLL_DIV(n)            (0x80 | ((n) << 3) | 0x2)
#define TX_PLL_MULTI_L(m)        ((m) & 0xFF)
#define TX_PLL_MULTI_H(m)        (((m) & 0x300) >> 8)
#define TX_PLL_VCO(v)            (0x81 | (((v) & 0x1F) << 1))
#define TX_PLL_CPBIAS            (0x10)
#define TX_PLL_INT_CTL           (0x4)
#define TX_PLL_PROP_CNTRL        (0x0C)
#define TX_PLL_RST_TIME_L        (0xFF)
#define TX_PLL_GEAR_SHIFT_L      (0x6)
#define TX_PLL_GEAR_SHIFT_H      (0x1)
#define TX_PLL_CLKDIV_CLK_LMT    (450)
#define TX_PLL_CLKDIV_CLK_EN     (0x10)

typedef struct _reg_s {
	uint32_t offset;
	uint32_t value;
} reg_t;

#define MIPIDPHYTIOC_READ        _IOWR('v', 0, reg_t)
#define MIPIDPHYTIOC_WRITE       _IOW('v', 1, reg_t)

typedef struct _mipi_dphy_s {
	void __iomem  *iomem;
} mipi_dphy_t;

mipi_dphy_t  *g_mipi_dphy = NULL;


void __iomem *g_hostmem = NULL;
void __iomem *g_devmem = NULL;

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
	uint16_t     freq;
	uint32_t     value;
} pll_sel_table_t;

static const pll_sel_table_t g_pll_sel_table[] = {
	{80, 0x00},
	{90, 0x10},
	{100, 0x20},
	{110, 0x30},
	{120, 0x01},
	{130, 0x11},
	{140, 0x21},
	{150, 0x31},
	{160, 0x02},
	{170, 0x12},
	{180, 0x22},
	{190, 0x32},
	{205, 0x03},
	{220, 0x13},
	{235, 0x23},
	{250, 0x33},
	{275, 0x04},
	{300, 0x14},
	{325, 0x25},
	{350, 0x35},
	{400, 0x05},
	{450, 0x16},
	{500, 0x26},
	{550, 0x37},
	{600, 0x07},
	{650, 0x18},
	{700, 0x28},
	{750, 0x39},
	{800, 0x09},
	{850, 0x19},
	{900, 0x29},
	{950, 0x3A},
	{1000, 0x0A},
	{1050, 0x1A},
	{1100, 0x2A},
	{1150, 0x3B},
	{1200, 0x0B},
	{1250, 0x1B},
	{1300, 0x2B},
	{1350, 0x3C},
	{1400, 0x0C},
	{1450, 0x1C},
	{1500, 0x2C},
	{1550, 0x3D},
	{1600, 0x0D},
	{1650, 0x1D},
	{1700, 0x2D},
	{1750, 0x3E},
	{1800, 0x0E},
	{1850, 0x1E},
	{1900, 0x2F},
	{1950, 0x3F},
	{2000, 0x0F},
	{2050, 0x40},
	{2100, 0x41},
	{2150, 0x42},
	{2200, 0x43},
	{2250, 0x44},
	{2300, 0x45},
	{2350, 0x46},
	{2400, 0x47},
	{2450, 0x48},
	{2500, 0x49},
};

static uint32_t mipi_dphy_clk_range(uint32_t mipiclk)
{
	uint8_t  index = 0;
	for (index = 0; index < sizeof(g_pll_sel_table) / sizeof(pll_sel_table_t); index++) {
		if (mipiclk <= g_pll_sel_table[index].freq) {
			mipidbg("pll div mipiclk: %d, selected clk: %d, range value: %d",
					 mipiclk, g_pll_sel_table[index].freq, g_pll_sel_table[index].value);
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
static void mipi_host_dphy_testdata(uint16_t testcode, uint8_t testdata)
{
	uint32_t regv = 0;
	/*write test code*/
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_ENABLE); /*set testen to high*/
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL1, testcode >> 8);    /*set testen to low, set test code MSBS*/
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/

	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
	regv = mipi_getreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL1);
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_ENABLE | regv); /*set testen to high*/
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_ENABLE | (testcode & 0xff));  /*set test code LSBS*/
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/

	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL1, testdata);         /*set test data*/
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
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
	g_hostmem = iomem;

	mipidbg("mipi host initialize begin");
	/*Release Synopsys-PHY test codes from reset*/
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_RESETN);
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLEAR);
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_RESETN);
	/*Ensure that testclk and testen is set to low*/

	mipi_host_dphy_testdata(REGS_RX_STARTUP_OVR_5, RX_CLK_SETTLE_EN);
	mipi_host_dphy_testdata(REGS_RX_STARTUP_OVR_4, RX_CLK_SETTLE);
	mipi_host_dphy_testdata(REGS_RX_STARTUP_OVR_17, RX_HS_SETTLE(settle));
	/*Configure the D-PHY frequency range*/
	ips_set_mipi_freqrange(MIPI_HOST_HSFREQRANGE, mipi_dphy_clk_range(mipiclk / lane));
	mipi_host_dphy_testdata(REGS_RX_SYS_7, RX_SYSTEM_CONFIG);
	mipi_host_dphy_testdata(REGS_RX_LANE0_DDL_4, RX_OSCFREQ_LOW);
	mipi_host_dphy_testdata(REGS_RX_LANE0_DDL_5, RX_OSCFREQ_HIGH);
	mipi_host_dphy_testdata(REGS_RX_LANE0_DDL_6, RX_OSCFREQ_EN);
	mipi_host_dphy_testdata(REGS_RX_LANE1_DDL_4, RX_OSCFREQ_LOW);
	mipi_host_dphy_testdata(REGS_RX_LANE1_DDL_5, RX_OSCFREQ_HIGH);
	mipi_host_dphy_testdata(REGS_RX_LANE1_DDL_6, RX_OSCFREQ_EN);
	mipi_host_dphy_testdata(REGS_RX_LANE2_DDL_4, RX_OSCFREQ_LOW);
	mipi_host_dphy_testdata(REGS_RX_LANE2_DDL_5, RX_OSCFREQ_HIGH);
	mipi_host_dphy_testdata(REGS_RX_LANE2_DDL_6, RX_OSCFREQ_EN);
	mipi_host_dphy_testdata(REGS_RX_LANE3_DDL_4, RX_OSCFREQ_LOW);
	mipi_host_dphy_testdata(REGS_RX_LANE3_DDL_5, RX_OSCFREQ_HIGH);
	mipi_host_dphy_testdata(REGS_RX_LANE3_DDL_6, RX_OSCFREQ_EN);

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
	/*Release Synopsys-PHY test codes from reset*/
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL1, DPHY_TEST_RESETN);
	mipi_putreg(g_hostmem + REG_MIPI_HOST_PHY_TEST_CTRL0, DPHY_TEST_CLEAR);
}

/**
 * @brief mipi_dev_dphy_testdata : write test data
 *
 * @param [in] testdata : testdatas' array
 * @param [in] size : size of testdata's array
 *
 * @return void
 */
static void mipi_dev_dphy_testdata(uint16_t testcode, uint8_t testdata)
{
	uint32_t regv = 0;
	/*write test code*/
	mipi_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL1, DPHY_TEST_ENABLE); /*set testen to high*/
	mipi_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
	mipi_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL1, testcode >> 8);    /*set testen to low, set test code MSBS*/
	mipi_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/

	mipi_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
	regv = mipi_getreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL1);
	mipi_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL1, DPHY_TEST_ENABLE | regv); /*set testen to high*/
	mipi_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL1, DPHY_TEST_ENABLE | (testcode & 0xff));  /*set test code LSBS*/
	mipi_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/

	mipi_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL1, testdata);         /*set test data*/
	mipi_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLK);    /*set testclk to high*/
	mipi_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN); /*set testclk to low*/
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

static uint32_t mipi_tx_vco_range(uint16_t vcoclk, uint16_t *vcomax)
{
	uint8_t  index = 0;
	for (index = 0; index < sizeof(g_vco_range_table) / sizeof(pll_range_table_t); index++) {
		if (vcoclk >= g_vco_range_table[index].low && vcoclk <= g_vco_range_table[index].high) {
			mipidbg("vcoclk: %d, selected range: %d-%d, range value: %d",
					 vcoclk, g_vco_range_table[index].low, g_vco_range_table[index].high, g_vco_range_table[index].value);
			*vcomax = g_vco_range_table[index].high;
			return g_vco_range_table[index].value;
		}
	}
	mipidbg("vco clock %d not supported", vcoclk);
	return 0;
}

static int32_t mipi_tx_pll_div(uint16_t refsclk, uint16_t laneclk, uint8_t *n, uint16_t *m, uint16_t *vco)
{
	uint16_t n_tmp = TX_PLL_INPUT_DIV_MIN;
	uint16_t m_tmp = TX_PLL_FB_MULTI_MIN;
	uint16_t fout = 0;
	uint16_t fvco = 0;
	uint16_t fvco_max = 0;
	uint16_t vco_div = 0;
	uint16_t outclk = 0;
	if (!refsclk || !laneclk || NULL == n || NULL == m) {
		mipierr("pll input error!!!");
		return 0;
	}
	fout = laneclk >> 1; /* data rate(Gbps) = PLL Fout(GHz)*2 */
	if (fout >= 320  && fout <= 1250) {
		vco_div = 0;
	} else if (fout >= 160  && fout < 320) {
		vco_div = 1;
	} else if (fout >= 80  && fout < 160) {
		vco_div = 2;
	} else if (fout >= 40  && fout < 80) {
		vco_div = 3;
	} else {
		mipierr("pll output clk error!!! laneclk: %d", laneclk);
		return 0;
	}
	fvco = fout << vco_div;
	if ((TX_PLL_INPUT_FEQ_MIN > refsclk / TX_PLL_INPUT_DIV_MIN) ||
		(TX_PLL_INPUT_FEQ_MAX < refsclk / TX_PLL_INPUT_DIV_MAX)) {
		mipierr("pll parameter error!!! refsclk: %d, laneclk: %d", refsclk, laneclk);
		return 0;
	}
	while (TX_PLL_INPUT_FEQ_MIN * n_tmp < refsclk) {
		n_tmp++;
	}
	n_tmp -= 1;
	outclk = refsclk / n_tmp;
	while ((outclk * m_tmp) <= fvco) {
		m_tmp++;
	}
	m_tmp -= 1;
	*n = n_tmp - 1;
	//*n = n_tmp - 2; /*device's clk must be higher than host's clk*/
	*m = m_tmp - 2;
	fvco = (refsclk * (*m + 2)) / (*n + 1);
	fout = fvco >> vco_div;
	*vco = mipi_tx_vco_range(fvco, &fvco_max);
	*m = fvco_max * (*n + 1) / refsclk - 2;
	outclk = fout << 1;
	mipidbg("pll div refsclk: %d, laneclk: %d, n: %d, m: %d, outclk: %d",
			 refsclk, laneclk, *n, *m, outclk);
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
	uint8_t    n = 0;
	uint16_t   m = 0;
	uint16_t   vco = 0;
	uint16_t   outclk = 0;
	g_devmem = iomem;

	mipidbg("mipi device initialize dphy begin");

	/*Configure the D-PHY PLL*/
	mipi_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_RESETN);
	/*Ensure that testclk and testen is set to low*/
	outclk = mipi_tx_pll_div(TX_REFSCLK_DEFAULT, (mipiclk / lane), &n, &m, &vco);
	if (0 == outclk) {
		mipierr("pll control error!");
		return -1;
	}
	/*Configure the D-PHY frequency range*/
	ips_set_mipi_freqrange(MIPI_DEV_HSFREQRANGE, mipi_dphy_clk_range(mipiclk / lane));

	mipi_dev_dphy_testdata(REGS_TX_SLEW_5, TX_SLEW_RATE_CAL);
	mipi_dev_dphy_testdata(REGS_TX_SLEW_7, TX_SLEW_RATE_CTL);
	mipi_dev_dphy_testdata(REGS_TX_PLL_27, TX_PLL_DIV(n));
	mipi_dev_dphy_testdata(REGS_TX_PLL_28, TX_PLL_MULTI_L(m));
	mipi_dev_dphy_testdata(REGS_TX_PLL_29, TX_PLL_MULTI_H(m));
	mipi_dev_dphy_testdata(REGS_TX_PLL_30, TX_PLL_VCO(vco));
	mipi_dev_dphy_testdata(REGS_TX_SYSTIMERS_23, TX_HS_ZERO(settle));
	mipi_dev_dphy_testdata(REGS_TX_PLL_1,  TX_PLL_CPBIAS);
	mipi_dev_dphy_testdata(REGS_TX_PLL_4,  TX_PLL_INT_CTL);
	mipi_dev_dphy_testdata(REGS_TX_PLL_17, TX_PLL_PROP_CNTRL);
	mipi_dev_dphy_testdata(REGS_TX_PLL_19, TX_PLL_RST_TIME_L);
	mipi_dev_dphy_testdata(REGS_TX_PLL_2,  TX_PLL_GEAR_SHIFT_L);
	mipi_dev_dphy_testdata(REGS_TX_PLL_3,  TX_PLL_GEAR_SHIFT_H);
	if (outclk < TX_PLL_CLKDIV_CLK_LMT)
		mipi_dev_dphy_testdata(REGS_TX_CB_2, TX_PLL_CLKDIV_CLK_EN);

	return 0;
}

/**
 * @brief mipi_dev_initialize_dphy : reset dev phy
 *
 * @param [] void :
 *
 * @return void
 */
void mipi_dev_dphy_reset(void __iomem *iomem)
{
	g_devmem = iomem;
	mipi_putreg(g_devmem + REG_MIPI_DEV_PHY0_TST_CTRL0, DPHY_TEST_CLEAR);
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

static long x2_mipi_dphy_regs_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __iomem  *iomem = g_mipi_dphy->iomem;
	reg_t          reg;
	uint32_t       regv = 0;
	/* Check type and command number */
	if (_IOC_TYPE(cmd) != 'v')
		return -ENOTTY;

	switch (cmd) {
	case MIPIDPHYTIOC_READ:
		if (!arg) {
			printk(KERN_ERR "x2 mipi_dphy reg read error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
			printk(KERN_ERR "x2 mipi_dphy reg read error, copy data from user failed\n");
			return -EINVAL;
		}
		reg.value = readl(iomem + reg.offset);
		if (copy_to_user((void __user *)arg, (void *)&reg, sizeof(reg))) {
			printk(KERN_ERR "x2 mipi_dphy reg read error, copy data to user failed\n");
			return -EINVAL;
		}
		break;
	case MIPIDPHYTIOC_WRITE:
		if (!arg) {
			printk(KERN_ERR "x2 mipi_dphy reg write error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
			printk(KERN_ERR "x2 mipi_dphy reg write error, copy data from user failed\n");
			return -EINVAL;
		}
		writel(reg.value, iomem + reg.offset);
		regv = readl(iomem + reg.offset);
		if (regv != reg.value) {
			printk(KERN_ERR "x2 mipi_dphy reg write error, write 0x%x got 0x%x\n", reg.value, regv);
			return -EINVAL;
		}
		break;
	default:
		break;
	}
	return 0;
}
static const struct file_operations x2_mipi_dphy_regs_fops = {
	.owner		= THIS_MODULE,
	.open		= x2_mipi_dphy_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.unlocked_ioctl = x2_mipi_dphy_regs_ioctl,
	.compat_ioctl = x2_mipi_dphy_regs_ioctl,
};

static int    mipi_dphy_major = 0;
struct cdev   mipi_dphy_cdev;
static struct class  *x2_mipi_dphy_class;
static struct device *g_mipi_dphy_dev;

static int x2_mipi_dphy_probe(struct platform_device *pdev)
{
	mipi_dphy_t        *pack_dev;
	int              ret = 0;
	dev_t            devno;
	struct resource *res;
	struct cdev     *p_cdev = &mipi_dphy_cdev;

	pack_dev = devm_kmalloc(&pdev->dev, sizeof(mipi_dphy_t), GFP_KERNEL);
	if (!pack_dev) {
		dev_err(&pdev->dev, "Unable to allloc mipi_dphy pack dev.\n");
		return -ENOMEM;
	}

	ret = alloc_chrdev_region(&devno, 0, 1, "x2_mipi_dphy");
	if (ret < 0) {
		printk(KERN_ERR "Error %d while alloc chrdev x2_mipi_dphy", ret);
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
		printk(KERN_INFO "[%s:%d] class_create error\n", __func__, __LINE__);
		ret = PTR_ERR(x2_mipi_dphy_class);
		goto err;
	}
	g_mipi_dphy_dev = device_create(x2_mipi_dphy_class, NULL, MKDEV(mipi_dphy_major, 0), (void *)pack_dev, "x2_mipi_dphy");
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
	.probe  = x2_mipi_dphy_probe,
	.remove = x2_mipi_dphy_remove,
	.driver	= {
		.name = "x2_mipi_dphy",
		.of_match_table = x2_mipi_dphy_match,
	},
};

module_platform_driver(x2_mipi_dphy_driver);
MODULE_AUTHOR("Zhang Tianyu <tianyu.zhang@hobot.cc>");
MODULE_DESCRIPTION("X2 MIPI DPHY Driver");
