/***************************************************************************
 *						COPYRIGHT NOTICE
 *			   Copyright 2018 Horizon Robotics, Inc.
 *					   All rights reserved.
 ***************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpumask.h>
#include <linux/err.h>
//#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <linux/mm.h>
#include <linux/types.h>
#include <linux/major.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <asm/cacheflush.h>
#include <soc/hobot/hobot_iar.h>
#include <soc/hobot/hobot_mipi_dphy.h>

extern struct iar_dev_s *g_iar_dev;
//int panel_reset_pin;

#define VERSION	0x0
#define PWR_UP	0x4
#define CLKMGR_CFG	0x8
#define DPI_VCID	0xc
#define DPI_COLOR_CODING	0x10
#define DPI_CFG_POL	0x14
#define DPI_LP_CMD_TIM	0x18
#define DBI_VCID	0x1c
#define DBI_CFG	0x20
#define DBI_PARTITIONING_EN	0x24
#define DBI_CMDSIZE	0x28
#define PCKHDL_CFG	0x2c
#define GEN_VCID	0x30
#define MODE_CFG	0x34
#define VID_MODE_CFG	0x38
#define VID_PKT_SIZE	0x3c
#define VID_NUM_CHUNKS	0x40
#define VID_NULL_SIZE	0x44
#define VID_HSA_TIME	0x48
#define VID_HBP_TIME	0x4c
#define VID_HLINE_TIME	0x50
#define VID_VSA_LINES	0x54
#define VID_VBP_LINES	0x58
#define VID_VFP_LINES	0x5c
#define VID_VACTIVE_LINES	0x60
#define EDPI_CMD_SIZE	0x64
#define CMD_MODE_CFG	0x68
#define GEN_HDR		0x6c
#define GEN_PLD_DATA	0x70
#define CMD_PKT_STATUS	0x74
#define TO_CNT_CFG	0x78
#define HS_RD_TO_CNT	0x7c
#define LP_RD_TO_CNT	0x80
#define HS_WR_TO_CNT	0x84
#define LP_WR_TO_CNT	0x88
#define BTA_TO_CNT	0x8c
#define SDF_3D		0x90
#define LPCLK_CTRL	0x94
#define PHY_TMR_LPCLK_CFG	0x98
#define PHY_TMR_CFG	0x9c
#define PHY_RSTZ	0xa0
#define PHY_IF_CFG	0xa4
#define PHY_ULPS_CTRL	0xa8
#define PHY_TX_TRIGGERS	0xac
#define PHY_STATUS	0xb0
#define PHY_TST_CTRL0	0xb4
#define PHY_TST_CTRL1	0xb8
#define INT_ST0		0xbc
#define INT_ST1		0xc0
#define INT_MSK0	0xc4
#define INT_MSK1	0xc8
#define PHY_CAL		0xcc
#define INT_FORCE0	0xd8
#define INT_FORCE1	0xdc
#define AUTO_ULPS_MODE	0xe0
#define AUTO_ULPS_ENTRY_DELAY	0xe4
#define AUTO_ULPS_WAKEUP_TIME	0xe8
#define DSC_PARAMETER	0xf0
#define PHY_TMR_RD_CFG	0xf4
#define AUTO_ULPS_MIN_TIME	0xf8
#define PHY_MODE	0xfc
#define VID_SHADOW_CTRL	0x100
#define DPI_VCID_ACT	0x10c
#define DPI_COLOR_CODING_ACT	0x110
#define DPI_LP_CMD_TIM_ACT	0x118
#define EDPI_TE_HW_CFG		0x11c
#define VID_MODE_CFG_ACT	0x138
#define VID_PKT_SIZE_ACT	0x13c
#define VID_NUM_CHUNKS_ACT	0x140
#define VID_NULL_SIZE_ACT	0x144
#define VID_HSA_TIME_ACT	0x148
#define VID_HBP_TIME_ACT	0x14c
#define VID_HLINE_TIME_ACT	0x150
#define VID_VSA_LINES_ACT	0x154
#define VID_VBP_LINES_ACT	0x158
#define VID_VFP_LINES_ACT	0x15c
#define VID_VACTIVE_LINES_ACT	0x160
#define VID_PKT_STATUS		0x168
#define SDF_3D_ACT		0x190
#define DSC_ENC_COREID		0x200
#define DSC_ENC_VERSION		0x204
#define DSC_ENC_FLATNESS_DET_THRES	0x208
#define DSC_ENC_DELAY		0x20c
#define DSC_ENC_COMPRESSED_LINE_SIZE	0x210
#define DSC_ENC_LINES_IN_EXCESS	0x214
#define DSC_ENC_RBUF_ADDR_LAST_LINE_ADJ	0x218
#define DSC_MODE		0x21c
#define DSC_ENC_INT_ST		0x220
#define DSC_ENC_INT_MSK		0x224
#define DSC_ENC_INT_FORCE	0x228
#define DSC_FIFO_STATUS_SELECT	0x22c
#define DSC_FIFO_STATUS		0x230
#define DSC_FIFO_STATUS2	0x234
#define DSC_ENC_PPS_0_3		0x260
#define DSC_ENC_PPS_4_7		0x264
#define DSC_ENC_PPS_8_11	0x268
#define DSC_ENC_PPS_12_15	0x26c
#define DSC_ENC_PPS_16_19	0x270
#define DSC_ENC_PPS_20_23	0x274
#define DSC_ENC_PPS_24_27	0x278
#define DSC_ENC_PPS_28_31	0x27c
#define DSC_ENC_PPS_32_35	0x280
#define DSC_ENC_PPS_36_39	0x284
#define DSC_ENC_PPS_40_43	0x288
#define DSC_ENC_PPS_44_47	0x28c
#define DSC_ENC_PPS_48_51	0x290
#define DSC_ENC_PPS_52_55	0x294
#define DSC_ENC_PPS_56_59	0x298
#define DSC_ENC_PPS_60_63	0x29c
#define DSC_ENC_PPS_64_67	0x2a0
#define DSC_ENC_PPS_68_71	0x2a4
#define DSC_ENC_PPS_72_75	0x2a8
#define DSC_ENC_PPS_76_79	0x2ac
#define DSC_ENC_PPS_80_83	0x2b0
#define DSC_ENC_PPS_84_87	0x2b4

enum mipi_dsi_reg_cfg_e {
	SHUTDOWNZ_FILED,//0
	TX_ESC_CLK_DIVISION_FILED,//1
	TO_CLK_DIVISION_FILED,//2
	PHY_TX_REQUEST_CLK_HS_FILED,//3
	AUTO_CLKLANE_CTRL_FILED,//4
	N_LANES_FILED,//5
	PHY_STOP_WAIT_TIME_FILED,//6
	PHY_SHUTDOWNZ_FILED,//7
	PHY_RSTZ_FILED,//8
	PHY_ENABLE_CLK_FILED,//9
	PHY_FORCE_PLL_FILED,//10
	DPI_VCID_FILED,//11
	DPI_COLOR_CODING_FILED,//12
	VID_PKT_SIZE_FILED,//13
	VID_NUM_CHUNKS_FILED,//14
	VID_NULL_SIZE_FILED,//15
	VID_HSA_TIME_FILED,//16
	VID_HBP_TIME_FILED,//17
	VID_HLINE_TIME_FILED,//18
	VSA_LINES_FILED,//19
	VBP_LINES_FILED,//20
	VFP_LINES_FILED,//21
	V_ACTIVE_LINES_FILED,//22
	CMD_VIDEO_MODE_FILED,//23
	VID_MODE_TYPE_FILED,//24
	LP_VSA_EN_FILED,//25
	LP_VBP_EN_FILED,//26
	LP_VFP_EN_FILED,//27
	LP_VACT_EN_FILED,//28
	LP_HBP_EN_FILED,//29
	LP_HFP_EN_FILED,//30
	FRAME_BTA_ACK_EN_FILED,//31
	LP_CMD_EN_FILED,//32
	VPG_EN_FILED,//33
	VPG_MODE_FILED,//34
	VPG_ORIENTATION_FILED,//35
	TEAR_FX_EN_FILED,//36
	ACK_RQST_EN_FILED,//37
	GEN_SW_0P_TX_FILED,//38
	GEN_SW_1P_TX_FILED,//39
	GEN_SW_2P_TX_FILED,//40
	GEN_SR_0P_TX_FILED,//41
	GEN_SR_1P_TX_FILED,//42
	GEN_SR_2P_TX_FILED,//43
	GEN_LW_TX_FILED,//44
	DCS_SW_0P_TX_FILED,//45
	DCS_SW_1P_TX_FILED,//46
	DCS_SR_0P_TX_FILED,//47
	DCS_LW_TX_FILED,//48
	MAX_RD_PKT_SIZE_FILED,//49
	GEN_DT_FILED,//50
	GEN_VC_FILED,//51
	GEN_WC_LS_BYTE_FILED,//52
	GEN_WC_MS_BYTE_FILED,//53
};

const unsigned int g_mipi_dsi_reg_cfg_table[][3] = {
	/*reg mask      reg offset*/
	{0x1, 0x0},//0
	{0xff, 0x0},//1
	{0xff, 0x8},//2
	{0x1, 0x0},//3
	{0x1, 0x1},//4
	{0x3, 0x0},//5
	{0xff, 0x8},//6
	{0x1, 0x0},//7
	{0x1, 0x1},//8
	{0x1, 0x2},//9
	{0x1, 0x3},//10
	{0x3, 0x0},//11
	{0xf, 0x0},//12
	{0x3fff, 0x0},//13
	{0x1fff, 0x0},//14
	{0x1fff, 0x0},//15
	{0xfff, 0x0},//16
	{0xfff, 0x0},//17
	{0x7fff, 0x0},//18
	{0x3ff, 0x0},//19
	{0x3ff, 0x0},//20
	{0x3ff, 0x0},//21
	{0x3fff, 0x0},//22
	{0x1, 0x0},//23
	{0x3, 0x0},//24
	{0x1, 0x8},//25
	{0x1, 0x9},//26
	{0x1, 0xa},//27
	{0x1, 0xb},//28
	{0x1, 0xc},//29
	{0x1, 0xd},//30
	{0x1, 0xe},//31
	{0x1, 0xf},//32
	{0x1, 0x10},//33
	{0x1, 0x14},//34
	{0x1, 0x18},//35
	{0x1, 0x0},//36
	{0x1, 0x1},//37
	{0x1, 0x8},//38
	{0x1, 0x9},//39
	{0x1, 0xa},//40
	{0x1, 0xb},//41
	{0x1, 0xc},//42
	{0x1, 0xd},//43
	{0x1, 0xe},//44
	{0x1, 0x10},//45
	{0x1, 0x11},//46
	{0x1, 0x12},//47
	{0x1, 0x13},//48
	{0x1, 0x18},//49
	{0x3f, 0x0},//50
	{0x3, 0x6},//51
	{0xff, 0x8},//52
	{0xff, 0x10},//53
};

enum _mipi_dsi_table_e {
	TABLE_MASK = 0,
	TABLE_OFFSET,
	TABLE_MAX,
};

#define REG_MIPI_DSI_PHY_TST_CTRL0  0xb4
#define REG_MIPI_DSI_PHY_TST_CTRL1  0xb8

#define DPHY_TEST_ENABLE    0x10000
#define DPHY_TEST_CLK       0x2
#define DPHY_TEST_RESETN    0x0

#define VALUE_SET(value, mask, offset, regvalue)	\
	((((value) & (mask)) << (offset)) | ((regvalue)&~((mask) << (offset))))
#define VALUE_GET(mask, offset, regvalue) (((regvalue) >> (offset)) & (mask))

#define MIPI_DSI_REG_SET_FILED(key, value, regvalue)	\
	VALUE_SET(value, g_mipi_dsi_reg_cfg_table[key][TABLE_MASK], \
		g_mipi_dsi_reg_cfg_table[key][TABLE_OFFSET], regvalue)
#define MIPI_DSI_REG_GET_FILED(key, regvalue)	\
	VALUE_GET(g_mipi_dsi_reg_cfg_table[key][TABLE_MASK], \
		g_mipi_dsi_reg_cfg_table[key][TABLE_OFFSET], regvalue)

static void mipi_dphy_write(uint16_t addr, uint8_t data)
{
	uint32_t regv = 0;

	/*write test code*/
	/*set testen to high*/
	writel(DPHY_TEST_ENABLE,
			g_iar_dev->mipi_dsi_regaddr + REG_MIPI_DSI_PHY_TST_CTRL1);
	/*set testclk to high*/
	writel(DPHY_TEST_CLK,
			g_iar_dev->mipi_dsi_regaddr + REG_MIPI_DSI_PHY_TST_CTRL0);
	/*set testclk to low*/
	writel(DPHY_TEST_RESETN,
			g_iar_dev->mipi_dsi_regaddr + REG_MIPI_DSI_PHY_TST_CTRL0);
	/*set testen to low, set test code MSBS*/
	writel(addr >> 8, g_iar_dev->mipi_dsi_regaddr + REG_MIPI_DSI_PHY_TST_CTRL1);
	/*set testclk to high*/
	writel(DPHY_TEST_CLK,
		g_iar_dev->mipi_dsi_regaddr + REG_MIPI_DSI_PHY_TST_CTRL0);

	/*set testclk to low*/
	writel(DPHY_TEST_RESETN,
		g_iar_dev->mipi_dsi_regaddr + REG_MIPI_DSI_PHY_TST_CTRL0);
	regv = readl(g_iar_dev->mipi_dsi_regaddr + REG_MIPI_DSI_PHY_TST_CTRL1);
	/*set testen to high*/
	writel(DPHY_TEST_ENABLE | regv,
		g_iar_dev->mipi_dsi_regaddr + REG_MIPI_DSI_PHY_TST_CTRL1);
	/*set testclk to high*/
	writel(DPHY_TEST_CLK,
		g_iar_dev->mipi_dsi_regaddr + REG_MIPI_DSI_PHY_TST_CTRL0);
	/*set test code LSBS*/
	writel(DPHY_TEST_ENABLE | (addr & 0xff),
		g_iar_dev->mipi_dsi_regaddr + REG_MIPI_DSI_PHY_TST_CTRL1);
	/*set testclk to low*/
	writel(DPHY_TEST_RESETN,
		g_iar_dev->mipi_dsi_regaddr + REG_MIPI_DSI_PHY_TST_CTRL0);

	/*set test data*/
	writel(data, g_iar_dev->mipi_dsi_regaddr + REG_MIPI_DSI_PHY_TST_CTRL1);
	/*set testclk to high*/
	writel(DPHY_TEST_CLK,
		g_iar_dev->mipi_dsi_regaddr + REG_MIPI_DSI_PHY_TST_CTRL0);
	/*set testclk to low*/
	writel(DPHY_TEST_RESETN,
		g_iar_dev->mipi_dsi_regaddr + REG_MIPI_DSI_PHY_TST_CTRL0);
}

static void mipi_dphy_config(void)
{
	mipi_dphy_write(0x270, 0x5e);
	mipi_dphy_write(0x272, 0x11);
	mipi_dphy_write(0x179, 0xda);
	mipi_dphy_write(0x17a, 0x0);
	mipi_dphy_write(0x178, 0xda);
	mipi_dphy_write(0x17b, 0xbf);
	mipi_dphy_write(0x65, 0x80);
	mipi_dphy_write(0x15e, 0x10);
	mipi_dphy_write(0x162, 0x4);
	mipi_dphy_write(0x16e, 0xc);
	mipi_dphy_write(0x170, 0xff);
	mipi_dphy_write(0x160, 0x6);
	mipi_dphy_write(0x161, 0x1);
	mipi_dphy_write(0x1ac, 0x10);
	mipi_dphy_write(0x1d, 0x4);
	mipi_dphy_write(0x8, 0x3);
	mipi_dphy_write(0x72, 0x11);
	mipi_dphy_write(0x1b, 0xaa);
	mipi_dphy_write(0x1c, 0xaa);
}

static int mipi_dsi_core_reset(void)
{
	uint32_t value;

	value = MIPI_DSI_REG_SET_FILED(SHUTDOWNZ_FILED, 0x0, 0x0);
	writel(value, g_iar_dev->mipi_dsi_regaddr + PWR_UP);//0x4
	return 0;
}

static int mipi_dsi_core_start(void)
{
	uint32_t value;

	value = MIPI_DSI_REG_SET_FILED(SHUTDOWNZ_FILED, 0x1, 0x0);
	writel(value, g_iar_dev->mipi_dsi_regaddr + PWR_UP);//0x4
	return 0;
}

static int mipi_dsi_core_pre_init(void)
{
	void __iomem *hitm1_reg_addr;

/*	hitm1_reg_addr = ioremap_nocache(0xA4300000 + 0xe0, 4);
	writel(0x1, hitm1_reg_addr);
	hitm1_reg_addr = ioremap_nocache(0xA4300000 + 0x84, 4);
	writel(0x08100000, hitm1_reg_addr);//20bit
	hitm1_reg_addr = ioremap_nocache(0xA4300000 + 0x8c, 4);
	writel(0x1c23, hitm1_reg_addr);
*/
	mipi_dphy_set_lanemode(MIPI_DPHY_TYPE_DSI, 0, 1);
	mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_DSI, 0, MIPI_CFGCLKFREQRANGE, 0x1c);
	mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_DSI, 0, MIPI_HSFREQRANGE, 0x23);

	writel(0x0, g_iar_dev->mipi_dsi_regaddr + PHY_RSTZ);//0xa0
	mipi_dsi_core_reset();
	writel(0x0, g_iar_dev->mipi_dsi_regaddr + MODE_CFG);//0X34
	writel(0x1, g_iar_dev->mipi_dsi_regaddr + PHY_TST_CTRL0);//0xb4
        usleep_range(900, 1000);
	writel(0x0, g_iar_dev->mipi_dsi_regaddr + PHY_TST_CTRL0);//0xb4
	writel(0x3203, g_iar_dev->mipi_dsi_regaddr + PHY_IF_CFG);//0XA4
	writel(0x2, g_iar_dev->mipi_dsi_regaddr + CLKMGR_CFG);//0X8
	writel(0x3, g_iar_dev->mipi_dsi_regaddr + LPCLK_CTRL);//0X94
	usleep_range(900, 1000);
	writel(0x0, g_iar_dev->mipi_dsi_regaddr + PHY_TST_CTRL0);//0XB4
	mipi_dphy_config();

	writel(0x7, g_iar_dev->mipi_dsi_regaddr + PHY_RSTZ);//0XA0
	mipi_dsi_core_start();
	return 0;
}
/*
static int mipi_dsi_core_pre_init(void)
{
	uint32_t value;

	value = readl(g_iar_dev->mipi_dsi_regaddr + CLKMGR_CFG);
	value = MIPI_DSI_REG_SET_FILED(TX_ESC_CLK_DIVISION_FILED, 0x2, value);
	writel(value, g_iar_dev->mipi_dsi_regaddr + CLKMGR_CFG);//0x8

	value = readl(g_iar_dev->mipi_dsi_regaddr + LPCLK_CTRL);
	value = MIPI_DSI_REG_SET_FILED(PHY_TX_REQUEST_CLK_HS_FILED, 0x1, value);
	value = MIPI_DSI_REG_SET_FILED(AUTO_CLKLANE_CTRL_FILED, 0x1, value);
	writel(value, g_iar_dev->mipi_dsi_regaddr + LPCLK_CTRL);//0x94

	value = readl(g_iar_dev->mipi_dsi_regaddr + PHY_IF_CFG);
	value = MIPI_DSI_REG_SET_FILED(N_LANES_FILED, 0x3, value);
	value = MIPI_DSI_REG_SET_FILED(PHY_STOP_WAIT_TIME_FILED, 0x4, value);
	writel(value, g_iar_dev->mipi_dsi_regaddr + PHY_IF_CFG);//0xa4

	value = readl(g_iar_dev->mipi_dsi_regaddr + PHY_RSTZ);
	value = MIPI_DSI_REG_SET_FILED(PHY_SHUTDOWNZ_FILED, 0x1, value);
	value = MIPI_DSI_REG_SET_FILED(PHY_RSTZ_FILED, 0x1, value);
	value = MIPI_DSI_REG_SET_FILED(PHY_ENABLE_CLK_FILED, 0x1, value);
	value = MIPI_DSI_REG_SET_FILED(PHY_FORCE_PLL_FILED, 0x1, value);
	writel(value, g_iar_dev->mipi_dsi_regaddr + PHY_RSTZ);//0xa0
	return 0;
}
*/
static int mipi_dsi_dpi_config(void)
{
	uint32_t value;

	value = readl(g_iar_dev->mipi_dsi_regaddr + DPI_VCID);
	value = MIPI_DSI_REG_SET_FILED(DPI_VCID_FILED, 0x0, value);
	writel(value, g_iar_dev->mipi_dsi_regaddr + DPI_VCID);//0xc

	value = readl(g_iar_dev->mipi_dsi_regaddr + DPI_COLOR_CODING);
	value = MIPI_DSI_REG_SET_FILED(DPI_COLOR_CODING_FILED, 0x5, value);
	writel(value, g_iar_dev->mipi_dsi_regaddr + DPI_COLOR_CODING);//0x10
	return 0;
}

static int mipi_dsi_set_mode(uint8_t mode)
{
	uint32_t value;

	value = readl(g_iar_dev->mipi_dsi_regaddr + MODE_CFG);//0x34
	value = MIPI_DSI_REG_SET_FILED(CMD_VIDEO_MODE_FILED, mode, value);
	writel(value, g_iar_dev->mipi_dsi_regaddr + MODE_CFG);
	return 0;
}

static int mipi_dsi_vid_mode_cfg(uint8_t mode)
{
	uint32_t value;

	//burst mode
	value = MIPI_DSI_REG_SET_FILED(VID_MODE_TYPE_FILED, 0x3, 0x0);
	value = MIPI_DSI_REG_SET_FILED(LP_VSA_EN_FILED, 0x1, value);
	value = MIPI_DSI_REG_SET_FILED(LP_VBP_EN_FILED, 0x1, value);
	value = MIPI_DSI_REG_SET_FILED(LP_VFP_EN_FILED, 0x1, value);
	value = MIPI_DSI_REG_SET_FILED(LP_VACT_EN_FILED, 0x1, value);
	value = MIPI_DSI_REG_SET_FILED(LP_HBP_EN_FILED, 0x1, value);
	value = MIPI_DSI_REG_SET_FILED(LP_HFP_EN_FILED, 0x1, value);

	value = MIPI_DSI_REG_SET_FILED(VPG_EN_FILED, mode, value);
	//0:normal 1:pattern

	writel(value, g_iar_dev->mipi_dsi_regaddr + VID_MODE_CFG);//38
	return 0;
}

struct video_timing {
	uint32_t vid_pkt_size;
	uint32_t vid_num_chunks;
	uint32_t vid_null_size;
	uint32_t vid_hsa;
	uint32_t vid_hbp;
	uint32_t vid_hline_time;
	uint32_t vid_vsa;
	uint32_t vid_vbp;
	uint32_t vid_vfp;
	uint32_t vid_vactive_line;
};

//struct video_timing video_1080_1920 = {
//	1080, 0, 0, 16, 512, 1736, 4, 4, 100, 1920,
//};
struct video_timing video_1080_1920 = {
        1080, 0, 0, 16, 77, 1296, 4, 4, 100, 1920,
};

static int mipi_dsi_video_config(struct video_timing *video_timing_config)
{
	uint32_t value;

	value = readl(g_iar_dev->mipi_dsi_regaddr + VID_PKT_SIZE);
	value = MIPI_DSI_REG_SET_FILED(VID_PKT_SIZE_FILED,
			video_timing_config->vid_pkt_size, value);//1080
	writel(value, g_iar_dev->mipi_dsi_regaddr + VID_PKT_SIZE);//0x3c

	value = readl(g_iar_dev->mipi_dsi_regaddr + VID_NUM_CHUNKS);
	value = MIPI_DSI_REG_SET_FILED(VID_NUM_CHUNKS_FILED,
			video_timing_config->vid_num_chunks, value);//0
	writel(value, g_iar_dev->mipi_dsi_regaddr + VID_NUM_CHUNKS);//0x40

	value = readl(g_iar_dev->mipi_dsi_regaddr + VID_NULL_SIZE);
	value = MIPI_DSI_REG_SET_FILED(VID_NULL_SIZE_FILED,
			video_timing_config->vid_null_size, value);//0
	writel(value, g_iar_dev->mipi_dsi_regaddr + VID_NULL_SIZE);//0x44

	value = readl(g_iar_dev->mipi_dsi_regaddr + VID_HSA_TIME);
	value = MIPI_DSI_REG_SET_FILED(VID_HSA_TIME_FILED,
			video_timing_config->vid_hsa, value);//16
	writel(value, g_iar_dev->mipi_dsi_regaddr + VID_HSA_TIME);//0x48

	value = readl(g_iar_dev->mipi_dsi_regaddr + VID_HBP_TIME);
	value = MIPI_DSI_REG_SET_FILED(VID_HBP_TIME_FILED,
			video_timing_config->vid_hbp, value);//512
	writel(value, g_iar_dev->mipi_dsi_regaddr + VID_HBP_TIME);//0x4c

	value = readl(g_iar_dev->mipi_dsi_regaddr + VID_HLINE_TIME);
	value = MIPI_DSI_REG_SET_FILED(VID_HLINE_TIME_FILED,
			video_timing_config->vid_hline_time, value);//1736
	writel(value, g_iar_dev->mipi_dsi_regaddr + VID_HLINE_TIME);//0x50

	value = readl(g_iar_dev->mipi_dsi_regaddr + VID_VSA_LINES);
	value = MIPI_DSI_REG_SET_FILED(VSA_LINES_FILED,
			video_timing_config->vid_vsa, value);//4
	writel(value, g_iar_dev->mipi_dsi_regaddr + VID_VSA_LINES);//0x54

	value = readl(g_iar_dev->mipi_dsi_regaddr + VID_VBP_LINES);
	value = MIPI_DSI_REG_SET_FILED(VBP_LINES_FILED,
			video_timing_config->vid_vbp, value);//4
	writel(value, g_iar_dev->mipi_dsi_regaddr + VID_VBP_LINES);//0x58

	value = readl(g_iar_dev->mipi_dsi_regaddr + VID_VFP_LINES);
	value = MIPI_DSI_REG_SET_FILED(VFP_LINES_FILED,
			video_timing_config->vid_vfp, value);//100
	writel(value, g_iar_dev->mipi_dsi_regaddr + VID_VFP_LINES);//0x5c

	value = readl(g_iar_dev->mipi_dsi_regaddr + VID_VACTIVE_LINES);
	value = MIPI_DSI_REG_SET_FILED(V_ACTIVE_LINES_FILED,
			video_timing_config->vid_vactive_line, value);//1920
	writel(value, g_iar_dev->mipi_dsi_regaddr + VID_VACTIVE_LINES);//0x60

	mipi_dsi_set_mode(0);//video mode
	mipi_dsi_vid_mode_cfg(0);//normal mode
	return 0;
}

void dsi_panel_write_cmd(uint8_t cmd, uint8_t data, uint8_t header)
{
	uint32_t value;

	value = (uint32_t)header | (uint32_t)cmd << 8 | (uint32_t)data << 16;
	writel(value, g_iar_dev->mipi_dsi_regaddr + GEN_HDR);
	msleep(5);
}

int mipi_dsi_panel_init(uint8_t panel_no)
{
	mipi_dsi_set_mode(1);//cmd mode
	writel(0xfffffffc, g_iar_dev->mipi_dsi_regaddr + CMD_MODE_CFG);// 0x68
	panel_hardware_reset();
	msleep(2000);

	if (panel_no == 0) {
		dsi_panel_write_cmd(0xff, 0x01, 0x15);//change to cmd2_page0
		dsi_panel_write_cmd(0xfb, 0x01, 0x15);
		dsi_panel_write_cmd(0xff, 0x02, 0x15);//change to cmd2_page1
		dsi_panel_write_cmd(0xfb, 0x01, 0x15);
		dsi_panel_write_cmd(0xff, 0x03, 0x15);//change to cmd2_page2
		dsi_panel_write_cmd(0xfb, 0x01, 0x15);
		dsi_panel_write_cmd(0xff, 0x04, 0x15);//change to cmd2_page3
		dsi_panel_write_cmd(0xfb, 0x01, 0x15);
		dsi_panel_write_cmd(0xff, 0x05, 0x15);//change to cmd2_page4
		dsi_panel_write_cmd(0xfb, 0x01, 0x15);
		dsi_panel_write_cmd(0xd7, 0x31, 0x15);
		//VSOUT & HSOUT @CMD2_Page4 (LTPS Dsiplay Timing)
		dsi_panel_write_cmd(0xd8, 0x7e, 0x15);
		//VSOUT & HSOUT @CMD2_Page4 (LTPS Dsiplay Timing)
		msleep(1000);

		dsi_panel_write_cmd(0xff, 0x00, 0x15);//change to cmd1 [*]
		dsi_panel_write_cmd(0xfb, 0x01, 0x15);//reload cmd1
		dsi_panel_write_cmd(0xba, 0x03, 0x15);
		//mipi-4lanes; ecc_crc mode
		dsi_panel_write_cmd(0x36, 0x00, 0x15);
		//dir_mode: Increase in Horizontal_Vertical; order: RGB
		dsi_panel_write_cmd(0xb0, 0x00, 0x15);
		dsi_panel_write_cmd(0xd3, 0x08, 0x15);//VBP = 8
		dsi_panel_write_cmd(0xd4, 0x0e, 0x15);//VFP = 14
		dsi_panel_write_cmd(0xd5, 0x0f, 0x15);//HBP = 15
		dsi_panel_write_cmd(0xd6, 0x48, 0x15);//HFP = 72
		dsi_panel_write_cmd(0xd7, 0x00, 0x15);//HBP8&HFP8
		dsi_panel_write_cmd(0xd9, 0x00, 0x15);//CLK_CTLK: Internal OSC
		dsi_panel_write_cmd(0xfb, 0x01, 0x15);
		dsi_panel_write_cmd(0xff, 0xee, 0x15);//change cmd?
		dsi_panel_write_cmd(0x40, 0x00, 0x15);
		dsi_panel_write_cmd(0x41, 0x00, 0x15);
		dsi_panel_write_cmd(0x42, 0x00, 0x15);
		dsi_panel_write_cmd(0xfb, 0x01, 0x15);
		dsi_panel_write_cmd(0xff, 0x01, 0x15);
		//change cmd2_page0 (power/MTP/gamma)
		dsi_panel_write_cmd(0xfb, 0x01, 0x15);
		dsi_panel_write_cmd(0x01, 0x55, 0x15);
		dsi_panel_write_cmd(0x04, 0x0c, 0x15);
		dsi_panel_write_cmd(0x05, 0x3a, 0x15);
		dsi_panel_write_cmd(0x06, 0x50, 0x15);
		dsi_panel_write_cmd(0x07, 0xd0, 0x15);
		dsi_panel_write_cmd(0x0a, 0x0f, 0x15);
		dsi_panel_write_cmd(0x0c, 0x06, 0x15);
		dsi_panel_write_cmd(0x0d, 0x7f, 0x15);
		dsi_panel_write_cmd(0x0e, 0x7f, 0x15);
		dsi_panel_write_cmd(0x0f, 0x70, 0x15);
		dsi_panel_write_cmd(0x10, 0x63, 0x15);
		dsi_panel_write_cmd(0x11, 0x3c, 0x15);
		dsi_panel_write_cmd(0x12, 0x5c, 0x15);
		dsi_panel_write_cmd(0x13, 0x5a, 0x15);
		dsi_panel_write_cmd(0x14, 0x5a, 0x15);
		dsi_panel_write_cmd(0x15, 0x60, 0x15);
		dsi_panel_write_cmd(0x16, 0x15, 0x15);
		dsi_panel_write_cmd(0x17, 0x15, 0x15);
		dsi_panel_write_cmd(0x23, 0x00, 0x15);
		dsi_panel_write_cmd(0x24, 0x00, 0x15);
		dsi_panel_write_cmd(0x25, 0x00, 0x15);
		dsi_panel_write_cmd(0x26, 0x00, 0x15);
		dsi_panel_write_cmd(0x27, 0x00, 0x15);
		dsi_panel_write_cmd(0x28, 0x00, 0x15);
		dsi_panel_write_cmd(0x44, 0x00, 0x15);
		dsi_panel_write_cmd(0x45, 0x00, 0x15);
		dsi_panel_write_cmd(0x46, 0x00, 0x15);
		dsi_panel_write_cmd(0x5b, 0xca, 0x15);
		dsi_panel_write_cmd(0x5c, 0x00, 0x15);
		dsi_panel_write_cmd(0x5d, 0x00, 0x15);
		dsi_panel_write_cmd(0x5e, 0x2d, 0x15);
		dsi_panel_write_cmd(0x5f, 0x1b, 0x15);
		dsi_panel_write_cmd(0x60, 0xd5, 0x15);
		dsi_panel_write_cmd(0x61, 0xf7, 0x15);
		dsi_panel_write_cmd(0x6c, 0xab, 0x15);
		dsi_panel_write_cmd(0x6d, 0x44, 0x15);
		dsi_panel_write_cmd(0xff, 0x05, 0x15);
		//change cmd2_page4 (LTPS Display Timing)
		dsi_panel_write_cmd(0xfb, 0x01, 0x15);
		dsi_panel_write_cmd(0x00, 0x3f, 0x15);
		dsi_panel_write_cmd(0x01, 0x3f, 0x15);
		dsi_panel_write_cmd(0x02, 0x3f, 0x15);
		dsi_panel_write_cmd(0x03, 0x3f, 0x15);
		dsi_panel_write_cmd(0x04, 0x38, 0x15);
		dsi_panel_write_cmd(0x03, 0x3f, 0x15);
		dsi_panel_write_cmd(0x06, 0x3f, 0x15);
		dsi_panel_write_cmd(0x07, 0x19, 0x15);
		dsi_panel_write_cmd(0x08, 0x1d, 0x15);
		dsi_panel_write_cmd(0x09, 0x3f, 0x15);
		dsi_panel_write_cmd(0x0a, 0x3f, 0x15);
		dsi_panel_write_cmd(0x0b, 0x1b, 0x15);
		dsi_panel_write_cmd(0x0c, 0x17, 0x15);
		dsi_panel_write_cmd(0x0d, 0x3f, 0x15);
		dsi_panel_write_cmd(0x0e, 0x3f, 0x15);
		dsi_panel_write_cmd(0x0f, 0x08, 0x15);
		dsi_panel_write_cmd(0x10, 0x3f, 0x15);
		dsi_panel_write_cmd(0x11, 0x10, 0x15);
		dsi_panel_write_cmd(0x12, 0x3f, 0x15);
		dsi_panel_write_cmd(0x13, 0x3f, 0x15);
		dsi_panel_write_cmd(0x14, 0x3f, 0x15);
		dsi_panel_write_cmd(0x15, 0x3f, 0x15);
		dsi_panel_write_cmd(0x16, 0x3f, 0x15);
		dsi_panel_write_cmd(0x17, 0x3f, 0x15);
		dsi_panel_write_cmd(0x18, 0x38, 0x15);
		dsi_panel_write_cmd(0x19, 0x18, 0x15);
		dsi_panel_write_cmd(0x1a, 0x1c, 0x15);
		dsi_panel_write_cmd(0x1b, 0x3f, 0x15);
		dsi_panel_write_cmd(0x1c, 0x3f, 0x15);
		dsi_panel_write_cmd(0x1d, 0x1a, 0x15);
		dsi_panel_write_cmd(0x1e, 0x16, 0x15);
		dsi_panel_write_cmd(0x1f, 0x3f, 0x15);
		dsi_panel_write_cmd(0x20, 0x3f, 0x15);
		dsi_panel_write_cmd(0x21, 0x3f, 0x15);
		dsi_panel_write_cmd(0x22, 0x3f, 0x15);
		dsi_panel_write_cmd(0x23, 0x06, 0x15);
		dsi_panel_write_cmd(0x24, 0x3f, 0x15);
		dsi_panel_write_cmd(0x25, 0x0e, 0x15);
		dsi_panel_write_cmd(0x26, 0x3f, 0x15);
		dsi_panel_write_cmd(0x27, 0x3f, 0x15);
		dsi_panel_write_cmd(0x54, 0x06, 0x15);
		dsi_panel_write_cmd(0x55, 0x05, 0x15);
		dsi_panel_write_cmd(0x56, 0x04, 0x15);
		dsi_panel_write_cmd(0x58, 0x03, 0x15);
		dsi_panel_write_cmd(0x59, 0x1b, 0x15);
		dsi_panel_write_cmd(0x5a, 0x1b, 0x15);
		dsi_panel_write_cmd(0x5b, 0x01, 0x15);
		dsi_panel_write_cmd(0x5c, 0x32, 0x15);
		dsi_panel_write_cmd(0x5e, 0x18, 0x15);
		dsi_panel_write_cmd(0x5f, 0x20, 0x15);
		dsi_panel_write_cmd(0x60, 0x2b, 0x15);
		dsi_panel_write_cmd(0x61, 0x2c, 0x15);
		dsi_panel_write_cmd(0x62, 0x18, 0x15);
		dsi_panel_write_cmd(0x63, 0x01, 0x15);
		dsi_panel_write_cmd(0x64, 0x32, 0x15);
		dsi_panel_write_cmd(0x65, 0x00, 0x15);
		dsi_panel_write_cmd(0x66, 0x44, 0x15);
		dsi_panel_write_cmd(0x67, 0x11, 0x15);
		dsi_panel_write_cmd(0x68, 0x01, 0x15);
		dsi_panel_write_cmd(0x69, 0x01, 0x15);
		dsi_panel_write_cmd(0x6a, 0x04, 0x15);
		dsi_panel_write_cmd(0x6b, 0x2c, 0x15);
		dsi_panel_write_cmd(0x6c, 0x08, 0x15);
		dsi_panel_write_cmd(0x6d, 0x08, 0x15);
		dsi_panel_write_cmd(0x78, 0x00, 0x15);
		dsi_panel_write_cmd(0x79, 0x00, 0x15);
		dsi_panel_write_cmd(0x7e, 0x00, 0x15);
		dsi_panel_write_cmd(0x7f, 0x00, 0x15);
		dsi_panel_write_cmd(0x80, 0x00, 0x15);
		dsi_panel_write_cmd(0x81, 0x00, 0x15);
		dsi_panel_write_cmd(0x8d, 0x00, 0x15);
		dsi_panel_write_cmd(0x8e, 0x00, 0x15);
		dsi_panel_write_cmd(0x8f, 0xc0, 0x15);
		dsi_panel_write_cmd(0x90, 0x73, 0x15);
		dsi_panel_write_cmd(0x91, 0x10, 0x15);
		dsi_panel_write_cmd(0x92, 0x07, 0x15);
		dsi_panel_write_cmd(0x96, 0x11, 0x15);
		dsi_panel_write_cmd(0x97, 0x14, 0x15);
		dsi_panel_write_cmd(0x98, 0x00, 0x15);
		dsi_panel_write_cmd(0x99, 0x00, 0x15);
		dsi_panel_write_cmd(0x9a, 0x00, 0x15);
		dsi_panel_write_cmd(0x9b, 0x61, 0x15);
		dsi_panel_write_cmd(0x9c, 0x15, 0x15);
		dsi_panel_write_cmd(0x9d, 0x30, 0x15);
		dsi_panel_write_cmd(0x9f, 0x0f, 0x15);
		dsi_panel_write_cmd(0xa2, 0xb0, 0x15);
		dsi_panel_write_cmd(0xa7, 0x0a, 0x15);
		dsi_panel_write_cmd(0xa9, 0x00, 0x15);
		dsi_panel_write_cmd(0xaa, 0x70, 0x15);
		dsi_panel_write_cmd(0xab, 0xda, 0x15);
		dsi_panel_write_cmd(0xac, 0xff, 0x15);
		dsi_panel_write_cmd(0xae, 0xf4, 0x15);
		dsi_panel_write_cmd(0xaf, 0x40, 0x15);
		dsi_panel_write_cmd(0xb0, 0x7f, 0x15);
		dsi_panel_write_cmd(0xb1, 0x16, 0x15);
		dsi_panel_write_cmd(0xb2, 0x53, 0x15);
		dsi_panel_write_cmd(0xb3, 0x00, 0x15);
		dsi_panel_write_cmd(0xb4, 0x2a, 0x15);
		dsi_panel_write_cmd(0xb5, 0x3a, 0x15);
		dsi_panel_write_cmd(0xb6, 0xf0, 0x15);
		dsi_panel_write_cmd(0xbc, 0x85, 0x15);
		dsi_panel_write_cmd(0xbd, 0xf4, 0x15);
		dsi_panel_write_cmd(0xbe, 0x33, 0x15);
		dsi_panel_write_cmd(0xbf, 0x13, 0x15);
		dsi_panel_write_cmd(0xc0, 0x77, 0x15);
		dsi_panel_write_cmd(0xc1, 0x77, 0x15);
		dsi_panel_write_cmd(0xc2, 0x77, 0x15);
		dsi_panel_write_cmd(0xc3, 0x77, 0x15);
		dsi_panel_write_cmd(0xc4, 0x77, 0x15);
		dsi_panel_write_cmd(0xc5, 0x77, 0x15);
		dsi_panel_write_cmd(0xc6, 0x77, 0x15);
		dsi_panel_write_cmd(0xc7, 0x77, 0x15);
		dsi_panel_write_cmd(0xc8, 0xaa, 0x15);
		dsi_panel_write_cmd(0xc9, 0x2a, 0x15);
		dsi_panel_write_cmd(0xca, 0x00, 0x15);
		dsi_panel_write_cmd(0xcb, 0xaa, 0x15);
		dsi_panel_write_cmd(0xcc, 0x92, 0x15);
		dsi_panel_write_cmd(0xcd, 0x00, 0x15);
		dsi_panel_write_cmd(0xce, 0x18, 0x15);
		dsi_panel_write_cmd(0xcf, 0x88, 0x15);
		dsi_panel_write_cmd(0xd0, 0xaa, 0x15);
		dsi_panel_write_cmd(0xd1, 0x00, 0x15);
		dsi_panel_write_cmd(0xd2, 0x00, 0x15);
		dsi_panel_write_cmd(0xd3, 0x00, 0x15);
		dsi_panel_write_cmd(0xd6, 0x02, 0x15);
		dsi_panel_write_cmd(0xed, 0x00, 0x15);
		dsi_panel_write_cmd(0xee, 0x00, 0x15);
		dsi_panel_write_cmd(0xef, 0x70, 0x15);
		dsi_panel_write_cmd(0xfa, 0x03, 0x15);
		dsi_panel_write_cmd(0xff, 0x00, 0x15);
		//change cmd1
		dsi_panel_write_cmd(0xff, 0x01, 0x15);
		//change cmd2_page0 (Power/MFP/Gamma)
		dsi_panel_write_cmd(0xfb, 0x01, 0x15);
		dsi_panel_write_cmd(0x75, 0x00, 0x15);
		dsi_panel_write_cmd(0x76, 0x00, 0x15);
		dsi_panel_write_cmd(0x77, 0x00, 0x15);
		dsi_panel_write_cmd(0x78, 0x2c, 0x15);
		dsi_panel_write_cmd(0x79, 0x00, 0x15);
		dsi_panel_write_cmd(0x7a, 0x4f, 0x15);
		dsi_panel_write_cmd(0x7b, 0x00, 0x15);
		dsi_panel_write_cmd(0x7c, 0x69, 0x15);
		dsi_panel_write_cmd(0x7d, 0x00, 0x15);
		dsi_panel_write_cmd(0x7e, 0x7f, 0x15);
		dsi_panel_write_cmd(0x7f, 0x00, 0x15);
		dsi_panel_write_cmd(0x80, 0x92, 0x15);
		dsi_panel_write_cmd(0x81, 0x00, 0x15);
		dsi_panel_write_cmd(0x82, 0xa3, 0x15);
		dsi_panel_write_cmd(0x83, 0x00, 0x15);
		dsi_panel_write_cmd(0x84, 0xb3, 0x15);
		dsi_panel_write_cmd(0x85, 0x00, 0x15);
		dsi_panel_write_cmd(0x86, 0xc1, 0x15);
		dsi_panel_write_cmd(0x87, 0x00, 0x15);
		dsi_panel_write_cmd(0x88, 0xf3, 0x15);
		dsi_panel_write_cmd(0x89, 0x01, 0x15);
		dsi_panel_write_cmd(0x8a, 0x1b, 0x15);
		dsi_panel_write_cmd(0x8b, 0x01, 0x15);
		dsi_panel_write_cmd(0x8c, 0x5a, 0x15);
		dsi_panel_write_cmd(0x8d, 0x01, 0x15);
		dsi_panel_write_cmd(0x8e, 0x8b, 0x15);
		dsi_panel_write_cmd(0x8f, 0x01, 0x15);
		dsi_panel_write_cmd(0x90, 0xd9, 0x15);
		dsi_panel_write_cmd(0x91, 0x02, 0x15);
		dsi_panel_write_cmd(0x92, 0x16, 0x15);
		dsi_panel_write_cmd(0x93, 0x02, 0x15);
		dsi_panel_write_cmd(0x94, 0x18, 0x15);
		dsi_panel_write_cmd(0x95, 0x02, 0x15);
		dsi_panel_write_cmd(0x96, 0x4e, 0x15);
		dsi_panel_write_cmd(0x97, 0x02, 0x15);
		dsi_panel_write_cmd(0x98, 0x88, 0x15);
		dsi_panel_write_cmd(0x99, 0x02, 0x15);
		dsi_panel_write_cmd(0x9a, 0xac, 0x15);
		dsi_panel_write_cmd(0x9b, 0x02, 0x15);
		dsi_panel_write_cmd(0x9c, 0xdd, 0x15);
		dsi_panel_write_cmd(0x9d, 0x03, 0x15);
		dsi_panel_write_cmd(0x9e, 0x01, 0x15);
		dsi_panel_write_cmd(0x9f, 0x03, 0x15);
		dsi_panel_write_cmd(0xa0, 0x2e, 0x15);
		dsi_panel_write_cmd(0xa2, 0x03, 0x15);
		dsi_panel_write_cmd(0xa3, 0x3c, 0x15);
		dsi_panel_write_cmd(0xa4, 0x03, 0x15);
		dsi_panel_write_cmd(0xa5, 0x4c, 0x15);
		dsi_panel_write_cmd(0xa6, 0x03, 0x15);
		dsi_panel_write_cmd(0xa7, 0x5d, 0x15);
		dsi_panel_write_cmd(0xa9, 0x03, 0x15);
		dsi_panel_write_cmd(0xaa, 0x70, 0x15);
		dsi_panel_write_cmd(0xab, 0x03, 0x15);
		dsi_panel_write_cmd(0xac, 0x88, 0x15);
		dsi_panel_write_cmd(0xad, 0x03, 0x15);
		dsi_panel_write_cmd(0xae, 0xa8, 0x15);
		dsi_panel_write_cmd(0xaf, 0x03, 0x15);
		dsi_panel_write_cmd(0xb0, 0xc8, 0x15);
		dsi_panel_write_cmd(0xb1, 0x03, 0x15);
		dsi_panel_write_cmd(0xb2, 0xff, 0x15);
		dsi_panel_write_cmd(0xb3, 0x00, 0x15);
		dsi_panel_write_cmd(0xb4, 0x00, 0x15);
		dsi_panel_write_cmd(0xb5, 0x00, 0x15);
		dsi_panel_write_cmd(0xb6, 0x2c, 0x15);
		dsi_panel_write_cmd(0xb7, 0x00, 0x15);
		dsi_panel_write_cmd(0xb8, 0x4f, 0x15);
		dsi_panel_write_cmd(0xb9, 0x00, 0x15);
		dsi_panel_write_cmd(0xba, 0x69, 0x15);
		dsi_panel_write_cmd(0xbb, 0x00, 0x15);
		dsi_panel_write_cmd(0xbc, 0x7f, 0x15);
		dsi_panel_write_cmd(0xbd, 0x00, 0x15);
		dsi_panel_write_cmd(0xbe, 0x92, 0x15);
		dsi_panel_write_cmd(0xbf, 0x00, 0x15);
		dsi_panel_write_cmd(0xc0, 0xa3, 0x15);
		dsi_panel_write_cmd(0xc1, 0x00, 0x15);
		dsi_panel_write_cmd(0xc2, 0xb3, 0x15);
		dsi_panel_write_cmd(0xc3, 0x00, 0x15);
		dsi_panel_write_cmd(0xc4, 0xc1, 0x15);
		dsi_panel_write_cmd(0xc5, 0x00, 0x15);
		dsi_panel_write_cmd(0xc6, 0xf3, 0x15);
		dsi_panel_write_cmd(0xc7, 0x01, 0x15);
		dsi_panel_write_cmd(0xc8, 0x1b, 0x15);
		dsi_panel_write_cmd(0xc9, 0x01, 0x15);
		dsi_panel_write_cmd(0xca, 0x5a, 0x15);
		dsi_panel_write_cmd(0xcb, 0x01, 0x15);
		dsi_panel_write_cmd(0xcc, 0x8b, 0x15);
		dsi_panel_write_cmd(0xcd, 0x01, 0x15);
		dsi_panel_write_cmd(0xce, 0xd9, 0x15);
		dsi_panel_write_cmd(0xcf, 0x02, 0x15);
		dsi_panel_write_cmd(0xd0, 0x16, 0x15);
		dsi_panel_write_cmd(0xd1, 0x02, 0x15);
		dsi_panel_write_cmd(0xd2, 0x18, 0x15);
		dsi_panel_write_cmd(0xd3, 0x02, 0x15);
		dsi_panel_write_cmd(0xd4, 0x4e, 0x15);
		dsi_panel_write_cmd(0xd5, 0x02, 0x15);
		dsi_panel_write_cmd(0xd6, 0x88, 0x15);
		dsi_panel_write_cmd(0xd7, 0x02, 0x15);
		dsi_panel_write_cmd(0xd8, 0xac, 0x15);
		dsi_panel_write_cmd(0xd9, 0x02, 0x15);
		dsi_panel_write_cmd(0xda, 0xdd, 0x15);
		dsi_panel_write_cmd(0xdb, 0x03, 0x15);
		dsi_panel_write_cmd(0xdc, 0x01, 0x15);
		dsi_panel_write_cmd(0xdd, 0x03, 0x15);
		dsi_panel_write_cmd(0xde, 0x2e, 0x15);
		dsi_panel_write_cmd(0xdf, 0x03, 0x15);
		dsi_panel_write_cmd(0xe0, 0x3c, 0x15);
		dsi_panel_write_cmd(0xe1, 0x03, 0x15);
		dsi_panel_write_cmd(0xe2, 0x4c, 0x15);
		dsi_panel_write_cmd(0xe3, 0x03, 0x15);
		dsi_panel_write_cmd(0xe4, 0x5d, 0x15);
		dsi_panel_write_cmd(0xe5, 0x03, 0x15);
		dsi_panel_write_cmd(0xe6, 0x70, 0x15);
		dsi_panel_write_cmd(0xe7, 0x03, 0x15);
		dsi_panel_write_cmd(0xe8, 0x88, 0x15);
		dsi_panel_write_cmd(0xe9, 0x03, 0x15);
		dsi_panel_write_cmd(0xea, 0xa8, 0x15);
		dsi_panel_write_cmd(0xeb, 0x03, 0x15);
		dsi_panel_write_cmd(0xec, 0xc8, 0x15);
		dsi_panel_write_cmd(0xed, 0x03, 0x15);
		dsi_panel_write_cmd(0xee, 0xff, 0x15);
		dsi_panel_write_cmd(0xef, 0x00, 0x15);
		dsi_panel_write_cmd(0xf0, 0x00, 0x15);
		dsi_panel_write_cmd(0xf1, 0x00, 0x15);
		dsi_panel_write_cmd(0xf2, 0x2c, 0x15);
		dsi_panel_write_cmd(0xf3, 0x00, 0x15);
		dsi_panel_write_cmd(0xf4, 0x4f, 0x15);
		dsi_panel_write_cmd(0xf5, 0x00, 0x15);
		dsi_panel_write_cmd(0xf6, 0x69, 0x15);
		dsi_panel_write_cmd(0xf7, 0x00, 0x15);
		dsi_panel_write_cmd(0xf8, 0x7f, 0x15);
		dsi_panel_write_cmd(0xf9, 0x00, 0x15);
		dsi_panel_write_cmd(0xfa, 0x92, 0x15);
		dsi_panel_write_cmd(0xff, 0x02, 0x15);
		//change cmd2_page1 (gamma)
		dsi_panel_write_cmd(0xfb, 0x01, 0x15);
		dsi_panel_write_cmd(0x00, 0x00, 0x15);
		dsi_panel_write_cmd(0x01, 0xa3, 0x15);
		dsi_panel_write_cmd(0x02, 0x00, 0x15);
		dsi_panel_write_cmd(0x03, 0xb3, 0x15);
		dsi_panel_write_cmd(0x04, 0x00, 0x15);
		dsi_panel_write_cmd(0x05, 0xc1, 0x15);
		dsi_panel_write_cmd(0x06, 0x00, 0x15);
		dsi_panel_write_cmd(0x07, 0xf3, 0x15);
		dsi_panel_write_cmd(0x08, 0x01, 0x15);
		dsi_panel_write_cmd(0x09, 0x1b, 0x15);
		dsi_panel_write_cmd(0x0a, 0x01, 0x15);
		dsi_panel_write_cmd(0x0b, 0x5a, 0x15);
		dsi_panel_write_cmd(0x0c, 0x01, 0x15);
		dsi_panel_write_cmd(0x0d, 0x8b, 0x15);
		dsi_panel_write_cmd(0x0e, 0x01, 0x15);
		dsi_panel_write_cmd(0x0f, 0xd9, 0x15);
		dsi_panel_write_cmd(0x10, 0x02, 0x15);
		dsi_panel_write_cmd(0x11, 0x16, 0x15);
		dsi_panel_write_cmd(0x12, 0x02, 0x15);
		dsi_panel_write_cmd(0x13, 0x18, 0x15);
		dsi_panel_write_cmd(0x14, 0x02, 0x15);
		dsi_panel_write_cmd(0x15, 0x4e, 0x15);
		dsi_panel_write_cmd(0x16, 0x02, 0x15);
		dsi_panel_write_cmd(0x17, 0x88, 0x15);
		dsi_panel_write_cmd(0x18, 0x02, 0x15);
		dsi_panel_write_cmd(0x19, 0xac, 0x15);
		dsi_panel_write_cmd(0x1a, 0x02, 0x15);
		dsi_panel_write_cmd(0x1b, 0xdd, 0x15);
		dsi_panel_write_cmd(0x1c, 0x03, 0x15);
		dsi_panel_write_cmd(0x1d, 0x01, 0x15);
		dsi_panel_write_cmd(0x1e, 0x03, 0x15);
		dsi_panel_write_cmd(0x1f, 0x2e, 0x15);
		dsi_panel_write_cmd(0x20, 0x03, 0x15);
		dsi_panel_write_cmd(0x21, 0x3c, 0x15);
		dsi_panel_write_cmd(0x22, 0x03, 0x15);
		dsi_panel_write_cmd(0x23, 0x4c, 0x15);
		dsi_panel_write_cmd(0x24, 0x03, 0x15);
		dsi_panel_write_cmd(0x25, 0x5d, 0x15);
		dsi_panel_write_cmd(0x26, 0x03, 0x15);
		dsi_panel_write_cmd(0x27, 0x70, 0x15);
		dsi_panel_write_cmd(0x28, 0x03, 0x15);
		dsi_panel_write_cmd(0x29, 0x88, 0x15);
		dsi_panel_write_cmd(0x2a, 0x03, 0x15);
		dsi_panel_write_cmd(0x2b, 0xa8, 0x15);
		dsi_panel_write_cmd(0x2d, 0x03, 0x15);
		dsi_panel_write_cmd(0x2f, 0xc8, 0x15);
		dsi_panel_write_cmd(0x30, 0x03, 0x15);
		dsi_panel_write_cmd(0x31, 0xff, 0x15);
		dsi_panel_write_cmd(0x32, 0x00, 0x15);
		dsi_panel_write_cmd(0x33, 0x00, 0x15);
		dsi_panel_write_cmd(0x34, 0x00, 0x15);
		dsi_panel_write_cmd(0x35, 0x2c, 0x15);
		dsi_panel_write_cmd(0x36, 0x00, 0x15);
		dsi_panel_write_cmd(0x37, 0x4f, 0x15);
		dsi_panel_write_cmd(0x38, 0x00, 0x15);
		dsi_panel_write_cmd(0x39, 0x69, 0x15);
		dsi_panel_write_cmd(0x3a, 0x00, 0x15);
		dsi_panel_write_cmd(0x3b, 0x7f, 0x15);
		dsi_panel_write_cmd(0x3d, 0x00, 0x15);
		dsi_panel_write_cmd(0x3f, 0x92, 0x15);
		dsi_panel_write_cmd(0x40, 0x00, 0x15);
		dsi_panel_write_cmd(0x41, 0xa3, 0x15);
		dsi_panel_write_cmd(0x42, 0x00, 0x15);
		dsi_panel_write_cmd(0x43, 0xb3, 0x15);
		dsi_panel_write_cmd(0x44, 0x00, 0x15);
		dsi_panel_write_cmd(0x45, 0xc1, 0x15);
		dsi_panel_write_cmd(0x46, 0x00, 0x15);
		dsi_panel_write_cmd(0x47, 0xf3, 0x15);
		dsi_panel_write_cmd(0x48, 0x01, 0x15);
		dsi_panel_write_cmd(0x49, 0x1b, 0x15);
		dsi_panel_write_cmd(0x4a, 0x01, 0x15);
		dsi_panel_write_cmd(0x4b, 0x5a, 0x15);
		dsi_panel_write_cmd(0x4c, 0x01, 0x15);
		dsi_panel_write_cmd(0x4d, 0x8b, 0x15);
		dsi_panel_write_cmd(0x4e, 0x01, 0x15);//
		dsi_panel_write_cmd(0x4f, 0xd9, 0x15);
		dsi_panel_write_cmd(0x50, 0x02, 0x15);
		dsi_panel_write_cmd(0x51, 0x16, 0x15);
		dsi_panel_write_cmd(0x52, 0x02, 0x15);
		dsi_panel_write_cmd(0x53, 0x18, 0x15);
		dsi_panel_write_cmd(0x54, 0x02, 0x15);
		dsi_panel_write_cmd(0x55, 0x4e, 0x15);
		dsi_panel_write_cmd(0x56, 0x02, 0x15);
		dsi_panel_write_cmd(0x58, 0x88, 0x15);
		dsi_panel_write_cmd(0x59, 0x02, 0x15);
		dsi_panel_write_cmd(0x5a, 0xac, 0x15);
		dsi_panel_write_cmd(0x5b, 0x02, 0x15);
		dsi_panel_write_cmd(0x5c, 0xdd, 0x15);
		dsi_panel_write_cmd(0x5d, 0x03, 0x15);
		dsi_panel_write_cmd(0x5e, 0x01, 0x15);
		dsi_panel_write_cmd(0x5f, 0x03, 0x15);
		dsi_panel_write_cmd(0x60, 0x2e, 0x15);
		dsi_panel_write_cmd(0x61, 0x03, 0x15);
		dsi_panel_write_cmd(0x62, 0x3c, 0x15);
		dsi_panel_write_cmd(0x63, 0x03, 0x15);
		dsi_panel_write_cmd(0x64, 0x4c, 0x15);
		dsi_panel_write_cmd(0x65, 0x03, 0x15);
		dsi_panel_write_cmd(0x66, 0x5d, 0x15);
		dsi_panel_write_cmd(0x67, 0x03, 0x15);
		dsi_panel_write_cmd(0x68, 0x70, 0x15);
		dsi_panel_write_cmd(0x69, 0x03, 0x15);
		dsi_panel_write_cmd(0x6a, 0x88, 0x15);
		dsi_panel_write_cmd(0x6b, 0x03, 0x15);
		dsi_panel_write_cmd(0x6c, 0xa8, 0x15);
		dsi_panel_write_cmd(0x6d, 0x03, 0x15);
		dsi_panel_write_cmd(0x6e, 0xc8, 0x15);
		dsi_panel_write_cmd(0x6f, 0x03, 0x15);
		dsi_panel_write_cmd(0x70, 0xff, 0x15);
		dsi_panel_write_cmd(0x71, 0x00, 0x15);
		dsi_panel_write_cmd(0x72, 0x00, 0x15);
		dsi_panel_write_cmd(0x73, 0x00, 0x15);
		dsi_panel_write_cmd(0x74, 0x2c, 0x15);
		dsi_panel_write_cmd(0x75, 0x00, 0x15);
		dsi_panel_write_cmd(0x76, 0x4f, 0x15);
		dsi_panel_write_cmd(0x77, 0x00, 0x15);
		dsi_panel_write_cmd(0x78, 0x69, 0x15);
		dsi_panel_write_cmd(0x79, 0x00, 0x15);
		dsi_panel_write_cmd(0x7a, 0x7f, 0x15);
		dsi_panel_write_cmd(0x7b, 0x00, 0x15);
		dsi_panel_write_cmd(0x7c, 0x92, 0x15);
		dsi_panel_write_cmd(0x7d, 0x00, 0x15);
		dsi_panel_write_cmd(0x7e, 0xa3, 0x15);
		dsi_panel_write_cmd(0x7f, 0x00, 0x15);
		dsi_panel_write_cmd(0x80, 0xb3, 0x15);
		dsi_panel_write_cmd(0x81, 0x00, 0x15);
		dsi_panel_write_cmd(0x82, 0xc1, 0x15);
		dsi_panel_write_cmd(0x83, 0x00, 0x15);
		dsi_panel_write_cmd(0x84, 0xf3, 0x15);
		dsi_panel_write_cmd(0x85, 0x01, 0x15);
		dsi_panel_write_cmd(0x86, 0x1b, 0x15);
		dsi_panel_write_cmd(0x87, 0x01, 0x15);
		dsi_panel_write_cmd(0x88, 0x5a, 0x15);
		dsi_panel_write_cmd(0x89, 0x01, 0x15);
		dsi_panel_write_cmd(0x8a, 0x8b, 0x15);
		dsi_panel_write_cmd(0x8b, 0x01, 0x15);
		dsi_panel_write_cmd(0x8c, 0xd9, 0x15);
		dsi_panel_write_cmd(0x8d, 0x02, 0x15);
		dsi_panel_write_cmd(0x8e, 0x16, 0x15);
		dsi_panel_write_cmd(0x8f, 0x02, 0x15);
		dsi_panel_write_cmd(0x90, 0x18, 0x15);
		dsi_panel_write_cmd(0x91, 0x02, 0x15);
		dsi_panel_write_cmd(0x92, 0x4e, 0x15);
		dsi_panel_write_cmd(0x93, 0x02, 0x15);
		dsi_panel_write_cmd(0x94, 0x88, 0x15);
		dsi_panel_write_cmd(0x95, 0x02, 0x15);
		dsi_panel_write_cmd(0x96, 0xac, 0x15);
		dsi_panel_write_cmd(0x97, 0x02, 0x15);
		dsi_panel_write_cmd(0x98, 0xdd, 0x15);
		dsi_panel_write_cmd(0x99, 0x03, 0x15);
		dsi_panel_write_cmd(0x9a, 0x01, 0x15);
		dsi_panel_write_cmd(0x9b, 0x03, 0x15);
		dsi_panel_write_cmd(0x9c, 0x2e, 0x15);
		dsi_panel_write_cmd(0x9d, 0x03, 0x15);
		dsi_panel_write_cmd(0x9e, 0x3c, 0x15);
		dsi_panel_write_cmd(0x9f, 0x03, 0x15);
		dsi_panel_write_cmd(0xa0, 0x4c, 0x15);
		dsi_panel_write_cmd(0xa2, 0x03, 0x15);
		dsi_panel_write_cmd(0xa3, 0x5d, 0x15);
		dsi_panel_write_cmd(0xa4, 0x03, 0x15);
		dsi_panel_write_cmd(0xa5, 0x70, 0x15);
		dsi_panel_write_cmd(0xa6, 0x03, 0x15);
		dsi_panel_write_cmd(0xa7, 0x88, 0x15);
		dsi_panel_write_cmd(0xa9, 0x03, 0x15);
		dsi_panel_write_cmd(0xaa, 0xa8, 0x15);
		dsi_panel_write_cmd(0xab, 0x03, 0x15);
		dsi_panel_write_cmd(0xac, 0xc8, 0x15);
		dsi_panel_write_cmd(0xad, 0x03, 0x15);
		dsi_panel_write_cmd(0xae, 0xff, 0x15);
		dsi_panel_write_cmd(0xaf, 0x00, 0x15);
		dsi_panel_write_cmd(0xb0, 0x00, 0x15);
		dsi_panel_write_cmd(0xb1, 0x00, 0x15);
		dsi_panel_write_cmd(0xb2, 0x2c, 0x15);
		dsi_panel_write_cmd(0xb3, 0x00, 0x15);
		dsi_panel_write_cmd(0xb4, 0x4f, 0x15);
		dsi_panel_write_cmd(0xb5, 0x00, 0x15);//
		dsi_panel_write_cmd(0xb6, 0x69, 0x15);
		dsi_panel_write_cmd(0xb7, 0x00, 0x15);
		dsi_panel_write_cmd(0xb8, 0x7f, 0x15);
		dsi_panel_write_cmd(0xb9, 0x00, 0x15);
		dsi_panel_write_cmd(0xba, 0x92, 0x15);
		dsi_panel_write_cmd(0xbb, 0x00, 0x15);
		dsi_panel_write_cmd(0xbc, 0xa3, 0x15);
		dsi_panel_write_cmd(0xbd, 0x00, 0x15);
		dsi_panel_write_cmd(0xbe, 0xb3, 0x15);
		dsi_panel_write_cmd(0xc0, 0xc1, 0x15);
		dsi_panel_write_cmd(0xc1, 0x00, 0x15);
		dsi_panel_write_cmd(0xc2, 0xf3, 0x15);
		dsi_panel_write_cmd(0xc3, 0x01, 0x15);
		dsi_panel_write_cmd(0xc4, 0x1b, 0x15);
		dsi_panel_write_cmd(0xc5, 0x01, 0x15);
		dsi_panel_write_cmd(0xc6, 0x5a, 0x15);
		dsi_panel_write_cmd(0xc7, 0x01, 0x15);
		dsi_panel_write_cmd(0xc8, 0x8b, 0x15);
		dsi_panel_write_cmd(0xc9, 0x01, 0x15);
		dsi_panel_write_cmd(0xca, 0xd9, 0x15);
		dsi_panel_write_cmd(0xcb, 0x02, 0x15);
		dsi_panel_write_cmd(0xcc, 0x16, 0x15);
		dsi_panel_write_cmd(0xcd, 0x02, 0x15);
		dsi_panel_write_cmd(0xce, 0x18, 0x15);
		dsi_panel_write_cmd(0xcf, 0x02, 0x15);
		dsi_panel_write_cmd(0xd0, 0x4e, 0x15);
		dsi_panel_write_cmd(0xd1, 0x02, 0x15);
		dsi_panel_write_cmd(0xb5, 0x00, 0x15);
		dsi_panel_write_cmd(0xd2, 0x88, 0x15);
		dsi_panel_write_cmd(0xd3, 0x02, 0x15);
		dsi_panel_write_cmd(0xd4, 0xac, 0x15);
		dsi_panel_write_cmd(0xd5, 0x02, 0x15);
		dsi_panel_write_cmd(0xd6, 0xdd, 0x15);
		dsi_panel_write_cmd(0xd7, 0x03, 0x15);
		dsi_panel_write_cmd(0xd8, 0x01, 0x15);
		dsi_panel_write_cmd(0xd9, 0x03, 0x15);
		dsi_panel_write_cmd(0xda, 0x2e, 0x15);
		dsi_panel_write_cmd(0xdb, 0x03, 0x15);
		dsi_panel_write_cmd(0xdc, 0x3c, 0x15);
		dsi_panel_write_cmd(0xdd, 0x03, 0x15);
		dsi_panel_write_cmd(0xde, 0x4d, 0x15);
		dsi_panel_write_cmd(0xdf, 0x03, 0x15);
		dsi_panel_write_cmd(0xe0, 0x5d, 0x15);
		dsi_panel_write_cmd(0xe1, 0x03, 0x15);
		dsi_panel_write_cmd(0xe2, 0x70, 0x15);
		dsi_panel_write_cmd(0xe3, 0x03, 0x15);
		dsi_panel_write_cmd(0xe4, 0x88, 0x15);
		dsi_panel_write_cmd(0xe5, 0x03, 0x15);
		dsi_panel_write_cmd(0xe6, 0xa8, 0x15);
		dsi_panel_write_cmd(0xe7, 0x03, 0x15);
		dsi_panel_write_cmd(0xe8, 0xc8, 0x15);
		dsi_panel_write_cmd(0xe9, 0x03, 0x15);
		dsi_panel_write_cmd(0xea, 0xff, 0x15);
		dsi_panel_write_cmd(0xff, 0x00, 0x15);//change cmd1
		dsi_panel_write_cmd(0xfb, 0x01, 0x15);

		dsi_panel_write_cmd(0x11, 0x00, 0x05);
		msleep(120);
		dsi_panel_write_cmd(0x29, 0x00, 0x05);
		msleep(100);

		mipi_dsi_set_mode(0);//video mode
                pr_info("mipi dsi panel config end!\n");
		return 0;
	}
}

int set_mipi_display(uint8_t panel_no)
{
	pr_info("mipi: set mipi display begin!\n");
	mipi_dsi_core_pre_init();
	mipi_dsi_dpi_config();
	mipi_dsi_video_config(&video_1080_1920);
	msleep(2000);
	//mipi_dsi_set_mode(0);//video mode
	//mipi_dsi_vid_mode_cfg(0);//normal mode
	mipi_dsi_panel_init(panel_no);//0:1080*1920
	//mipi_dsi_set_mode(0);//video mode
	//mipi_dsi_vid_mode_cfg(1);//pattern mode
	pr_info("mipi: set mipi display end!\n");
	return 0;
}
EXPORT_SYMBOL_GPL(set_mipi_display);

