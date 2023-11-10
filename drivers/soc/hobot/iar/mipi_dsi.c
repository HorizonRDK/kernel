/***************************************************************************
 *						COPYRIGHT NOTICE
 *			   Copyright 2018 Horizon Robotics, Inc.
 *					   All rights reserved.
 ***************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpumask.h>
#include <linux/err.h>
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
#include <linux/iopoll.h>
#include <asm/cacheflush.h>
#include <soc/hobot/hobot_iar.h>
#include <soc/hobot/hobot_mipi_dphy.h>

extern struct iar_dev_s *g_iar_dev;
struct video_timing mipi_timing;
EXPORT_SYMBOL(mipi_timing);
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

#define REG_MIPI_DSI_PHY_TST_CTRL0  0xb4
#define REG_MIPI_DSI_PHY_TST_CTRL1  0xb8

#define DPHY_TEST_ENABLE    0x10000
#define DPHY_TEST_CLK       0x2
#define DPHY_TEST_RESETN    0x0

#define MIPI_DSI_REG_SET_FILED(key, value, regvalue)	\
	VALUE_SET(value, g_mipi_dsi_reg_cfg_table[key][TABLE_MASK], \
		g_mipi_dsi_reg_cfg_table[key][TABLE_OFFSET], regvalue)
#define MIPI_DSI_REG_GET_FILED(key, regvalue)	\
	VALUE_GET(g_mipi_dsi_reg_cfg_table[key][TABLE_MASK], \
		g_mipi_dsi_reg_cfg_table[key][TABLE_OFFSET], regvalue)

struct mipi_dsi_config{
	uint32_t dsi_pkt_size;
	uint32_t dsi_num_chunks;
	uint32_t dsi_null_size;
	uint32_t dsi_hsa;
	uint32_t dsi_hbp;
	uint32_t dsi_hline_time;
	uint32_t dsi_vsa;
	uint32_t dsi_vbp;
	uint32_t dsi_vfp;
	uint32_t dsi_vactive_line;
	struct mipi_init_para *dsi_cmd;

};



void mipi_config_update(void)
{
	writel(0x101, g_iar_dev->mipi_dsi_regaddr + VID_SHADOW_CTRL);
}
EXPORT_SYMBOL_GPL(mipi_config_update);

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

static void mipi_dphy_config(uint8_t panel_no)
{
	if (panel_no == 0) {
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
	} else if (panel_no == 1 || panel_no == 2) {
		mipi_dphy_write(0x1d, 0x4);
		mipi_dphy_write(0x1c, 0xaa);
	} else if (panel_no == 4 ) {
		mipi_dphy_write(0x1d, 0x4);
		mipi_dphy_write(0x1c, 0xaa);
#if 0
		mipi_dphy_write(0x17, 0x0B); //n = 11 +1 =12
		mipi_dphy_write(0x19, 0x01); //override n
		mipi_dphy_write(0x19, 0x20); //override m
		mipi_dphy_write(0x18, 0x0c); //m =172 +2 = 174 fvco =24*(174/12) =348MHz fout = fvco= 348MHz
		mipi_dphy_write(0x18, 0x85); //m =172 +2 = 174 fvco =24*(174/12) =348MHz fout = fvco= 348MHz
		mipi_dphy_write(0x12, 0x4F); //vco_cntrl b[5:0] 0xf 320~440MHz 0x1f 160~320 , b[6] override enable
		mipi_dphy_write(0x1C, 0x10); //pll_cpbias_cntrl
		mipi_dphy_write(0x13, 0x10); //pll_gmp_cntrl
		mipi_dphy_write(0x0E, 0x0B); //pll_prop_cntrl
		mipi_dphy_write(0x0F, 0x00); //pll_int_cntrl
#endif
	}
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

static int mipi_dsi_core_pre_init(uint8_t panel_no)
{
	mipi_dphy_set_lanemode(MIPI_DPHY_TYPE_DSI, 0, 1);
	mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_DSI, 0, MIPI_CFGCLKFREQRANGE, 0x1c);
	if (panel_no == 0)
		mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_DSI, 0, MIPI_HSFREQRANGE, 0x23);
	else if (panel_no == 1 || panel_no == 2)
		mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_DSI, 0, MIPI_HSFREQRANGE, 0x16);
	//add by jiale01.luo
	else if (panel_no == 4) {
		//mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_DSI, 0, MIPI_HSFREQRANGE, 0x35);
		mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_DSI, 0, MIPI_HSFREQRANGE, 0x28);

	}

	writel(0x0, g_iar_dev->mipi_dsi_regaddr + PHY_RSTZ);//0xa0
	mipi_dsi_core_reset();
	writel(0x0, g_iar_dev->mipi_dsi_regaddr + MODE_CFG);//0X34
	writel(0x1, g_iar_dev->mipi_dsi_regaddr + PHY_TST_CTRL0);//0xb4
        usleep_range(900, 1000);
	writel(0x0, g_iar_dev->mipi_dsi_regaddr + PHY_TST_CTRL0);//0xb4
	//add by jiale01.luo
	if(panel_no == 4){
		pr_debug("%s:Using one lane...\n",__func__);
		writel(0x3200, g_iar_dev->mipi_dsi_regaddr + PHY_IF_CFG);//0XA4
	}else{
		writel(0x3203, g_iar_dev->mipi_dsi_regaddr + PHY_IF_CFG);//0XA4
	}
	writel(0x2, g_iar_dev->mipi_dsi_regaddr + CLKMGR_CFG);//0X8
	if(panel_no == 4){
		//add by jiale01.luo
		//If you are using ICN6211 DSI to RGB bridge,clocks often require continuous.
		//otherwise this bridge maybe not working.
		pr_debug("mipi_dsi_clock_continuous mode...\n");
		writel(0x1, g_iar_dev->mipi_dsi_regaddr + LPCLK_CTRL);//0X94
	}else{
		pr_debug("mipi_dsi_clock_non_continuous mode...\n");
		writel(0x3, g_iar_dev->mipi_dsi_regaddr + LPCLK_CTRL);//0X94
	}
	usleep_range(900, 1000);
	writel(0x0, g_iar_dev->mipi_dsi_regaddr + PHY_TST_CTRL0);//0XB4
	mipi_dphy_config(panel_no);

//	mipi_dev_dphy_initialize((void *)((char *)g_iar_dev->mipi_dsi_regaddr - 0x60), 698, 1, 1);
	writel(0x7, g_iar_dev->mipi_dsi_regaddr + PHY_RSTZ);//0XA0
	mipi_dsi_core_start();

	return 0;
}

// static int mipi_dsi_core_pre_init_test(uint8_t panel_no,int val)
// {
// 	mipi_dphy_set_lanemode(MIPI_DPHY_TYPE_DSI, 0, 1);
// 	mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_DSI, 0, MIPI_CFGCLKFREQRANGE, 0x1c);
// 	if (panel_no == 0)
// 		mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_DSI, 0, MIPI_HSFREQRANGE, 0x23);
// 	else if (panel_no == 1 || panel_no == 2)
// 		mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_DSI, 0, MIPI_HSFREQRANGE, 0x16);
// 	//add by jiale01.luo
// 	else if (panel_no == 4)
// 		mipi_dphy_set_freqrange(MIPI_DPHY_TYPE_DSI, 0, MIPI_HSFREQRANGE, val);

// 	// writel(0x0, g_iar_dev->mipi_dsi_regaddr + PHY_RSTZ);//0xa0
// 	// mipi_dsi_core_reset();
// 	// writel(0x0, g_iar_dev->mipi_dsi_regaddr + MODE_CFG);//0X34
// 	// writel(0x1, g_iar_dev->mipi_dsi_regaddr + PHY_TST_CTRL0);//0xb4
//     //     usleep_range(900, 1000);
// 	// writel(0x0, g_iar_dev->mipi_dsi_regaddr + PHY_TST_CTRL0);//0xb4
// 	// //add by jiale01.luo
// 	// if(panel_no == 4){
// 	// 	pr_err("1 LANE init...\n");
// 	// 	writel(0x3200, g_iar_dev->mipi_dsi_regaddr + PHY_IF_CFG);//0XA4
// 	// }else{
// 	// 	writel(0x3203, g_iar_dev->mipi_dsi_regaddr + PHY_IF_CFG);//0XA4
// 	// }
// 	// writel(0x2, g_iar_dev->mipi_dsi_regaddr + CLKMGR_CFG);//0X8
// 	// pr_err("ct time...\n");
// 	// writel(0x1, g_iar_dev->mipi_dsi_regaddr + LPCLK_CTRL);//0X94
// 	// usleep_range(900, 1000);
// 	// writel(0x0, g_iar_dev->mipi_dsi_regaddr + PHY_TST_CTRL0);//0XB4
// 	// mipi_dphy_config(panel_no);

// 	// writel(0x7, g_iar_dev->mipi_dsi_regaddr + PHY_RSTZ);//0XA0
// 	// mipi_dsi_core_start();

// 	return 0;
// }
// EXPORT_SYMBOL(mipi_dsi_core_pre_init_test);

static int mipi_dsi_dpi_config(uint8_t panel_no)
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

int mipi_dsi_set_mode(uint8_t mode)
{
	uint32_t value;

	value = readl(g_iar_dev->mipi_dsi_regaddr + MODE_CFG);//0x34
	value = MIPI_DSI_REG_SET_FILED(CMD_VIDEO_MODE_FILED, mode, value);
	writel(value, g_iar_dev->mipi_dsi_regaddr + MODE_CFG);
	return 0;
}
EXPORT_SYMBOL_GPL(mipi_dsi_set_mode);

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

//struct video_timing video_1080_1920 = {
//	1080, 0, 0, 16, 512, 1736, 4, 4, 100, 1920,
//};
struct video_timing video_1080_1920 = {
        1080, 0, 0, 16, 77, 1296, 4, 4, 100, 1920,
};
struct video_timing video_720_1280 = {
        720, 0, 0, 10, 30, 900, 10, 20, 10, 1320,
};

struct video_timing video_720_1280_sdb = {
	720, 0, 0, 10, 40, 810, 3, 11, 16, 1310,
};

struct video_timing video_1280_720 = {
	1280, 0, 0, 10, 40, 1370, 3, 11, 16, 750,
};

struct video_timing video_800_480_cm = {
	800, 0, 0, 10, 177, 2850, 2, 21, 7, 480,
};

#define MIPI_SNP        0x05
#define MIPI_S1P        0x15
#define MIPI_LCP        0x39

#define MIPI_SLP        0xFE
#define MIPI_OFF        0xFF

struct mipi_init_para {
	uint8_t cmd_type;
	uint8_t cmd_len;
	uint8_t cmd[128];
};

static struct mipi_init_para init_para_720x1280_sdb[] = {
	{MIPI_LCP, 0x04, {0xB9, 0xF1, 0x12, 0x83}},
	{MIPI_LCP, 0x1C, {0xBA, 0x33, 0x81, 0x05, 0xF9, 0x0E, 0x0E,
				0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x44, 0x25, 0x00, 0x91,
				0x0A, 0x00, 0x00, 0x02, 0x4F, 0xD1, 0x00, 0x00, 0x37}},
	{MIPI_S1P, 0x02, {0xB8, 0x26}},
	{MIPI_LCP, 0x04, {0xBF, 0x02, 0x10, 0x00}},
	{MIPI_LCP, 0x0B, {0xB3, 0x07, 0x0B, 0x1E, 0x1E, 0x03, 0xFF,
				0x00, 0x00, 0x00, 0x00}},
	{MIPI_LCP, 0x0A, {0xC0, 0x73, 0x73, 0x50, 0x50, 0x00, 0x00,
				0x08, 0x70, 0x00}},
	{MIPI_S1P, 0x02, {0xBC, 0x46}},
	{MIPI_S1P, 0x02, {0xCC, 0x0B}}, //0x0B
	{MIPI_S1P, 0x02, {0xB4, 0x80}},
	{MIPI_LCP, 0x04, {0xB2, 0xC8, 0x12, 0xA0}},
	{MIPI_LCP, 0x0F, {0xE3, 0x07, 0x07, 0x0B, 0x0B, 0x03, 0x0B,
				0x00, 0x00, 0x00, 0x00, 0xFF, 0x80,
				0xC0, 0x10}},
	{MIPI_LCP, 0x0D, {0xC1, 0x53, 0x00, 0x32, 0x32, 0x77, 0xF1,
				0xFF, 0xFF, 0xCC, 0xCC, 0x77, 0x77}},
	{MIPI_LCP, 0x03, {0xB5, 0x09, 0x09}},
	{MIPI_LCP, 0x03, {0xB6, 0xB7, 0xB7}},
	{MIPI_LCP, 0x40, {0xE9, 0xC2, 0x10, 0x0A, 0x00, 0x00, 0x81,
				0x80, 0x12, 0x30, 0x00, 0x37, 0x86, 0x81,
				0x80, 0x37, 0x18, 0x00, 0x05, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
				0x00, 0xF8, 0xBA, 0x46, 0x02, 0x08, 0x28,
				0x88, 0x88, 0x88, 0x88, 0x88, 0xF8, 0xBA,
				0x57, 0x13, 0x18, 0x38, 0x88, 0x88, 0x88,
				0x88, 0x88, 0x00, 0x00, 0x00, 0x03, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{MIPI_LCP, 0x3E, {0xEA, 0x07, 0x12, 0x01, 0x01, 0x02, 0x3C,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8F,
				0xBA, 0x31, 0x75, 0x38, 0x18, 0x88, 0x88,
				0x88, 0x88, 0x88, 0x8F, 0xBA, 0x20, 0x64,
				0x28, 0x08, 0x88, 0x88, 0x88, 0x88, 0x88,
				0x23, 0x10, 0x00, 0x00, 0x04, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{MIPI_LCP, 0x23, {0xE0, 0x00, 0x02, 0x04, 0x1A, 0x23, 0x3F, 0x2C,
				0x28, 0x05, 0x09, 0x0B, 0x10, 0x11, 0x10,
				0x12, 0x12, 0x19, 0x00, 0x02, 0x04, 0x1A,
				0x23, 0x3F, 0x2C, 0x28, 0x05, 0x09, 0x0B,
				0x10, 0x11, 0x10, 0x12, 0x12, 0x19}},
	{MIPI_SNP, 0x01, {0x11}},
	{MIPI_SLP, 250, {0x00}},
	{MIPI_SNP, 0x01, {0x29}},
	{MIPI_SLP, 50, {0x00}},

	{MIPI_OFF, 0x00, {0x00}},
};

static struct mipi_init_para init_para_800x480_cm[] = {
	{MIPI_LCP, 0x06, {0x10, 0x02, 0x03, 0x00, 0x00, 0x00}},
	{MIPI_LCP, 0x06, {0x64, 0x01, 0x0c, 0x00, 0x00, 0x00}},
	{MIPI_LCP, 0x06, {0x68, 0x01, 0x0c, 0x00, 0x00, 0x00}},
	{MIPI_LCP, 0x06, {0x44, 0x01, 0x00, 0x00, 0x00, 0x00}},
	{MIPI_LCP, 0x06, {0x48, 0x01, 0x00, 0x00, 0x00, 0x00}},
	{MIPI_LCP, 0x06, {0x14, 0x01, 0x15, 0x00, 0x00, 0x00}},
	{MIPI_LCP, 0x06, {0x50, 0x04, 0x60, 0x00, 0x00, 0x00}},
	{MIPI_LCP, 0x06, {0x20, 0x04, 0x52, 0x01, 0x10, 0x00}},

	{MIPI_LCP, 0x06, {0x24,0x04,0x14,0x00,0x1A,0x00}},
	{MIPI_LCP, 0x06, {0x28,0x04,0x20,0x03,0x69,0x00}},
	{MIPI_LCP, 0x06, {0x2C,0x04,0x02,0x00,0x15,0x00}},
	{MIPI_LCP, 0x06, {0x30,0x04,0xE0,0x01,0x07,0x00}},
	{MIPI_LCP, 0x06, {0x34,0x04,0x01,0x00,0x00,0x00}},


	{MIPI_LCP, 0x06, {0x64, 0x04, 0x0f, 0x04, 0x00, 0x00}},
	{MIPI_LCP, 0x06, {0x04, 0x01, 0x01, 0x00, 0x00, 0x00}},
	{MIPI_LCP, 0x06, {0x04, 0x02, 0x01, 0x00, 0x00, 0x00}},
	{MIPI_SLP, 50, {0x00}},
	{MIPI_OFF, 0x00, {0x00}},

};

int mipi_dsi_video_config(struct video_timing *video_timing_config)
{
	uint32_t value;

	pr_debug("++++++ set timing is %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                                video_timing_config->vid_pkt_size,
                                video_timing_config->vid_num_chunks,
                                video_timing_config->vid_null_size,
                                video_timing_config->vid_hsa,
                                video_timing_config->vid_hbp,
                                video_timing_config->vid_hline_time,
                                video_timing_config->vid_vsa,
                                video_timing_config->vid_vbp,
                                video_timing_config->vid_vfp,
                                video_timing_config->vid_vactive_line);
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

	mipi_config_update();
	mipi_dsi_set_mode(0);//video mode
	mipi_dsi_vid_mode_cfg(0);//normal mode
	mipi_config_update();
	return 0;
}
EXPORT_SYMBOL(mipi_dsi_video_config);

void dsi_panel_write_cmd(uint8_t cmd, uint8_t data, uint8_t header)
{
	uint32_t value;

	value = (uint32_t)header | (uint32_t)cmd << 8 | (uint32_t)data << 16;
	writel(value, g_iar_dev->mipi_dsi_regaddr + GEN_HDR);
	usleep_range(900, 1000);
}

int dsi_panel_write_cmd_poll(uint8_t cmd, uint8_t data, uint8_t header)
{
	int ret;
	u32 val, mask;
	uint32_t value = 0;

	ret = readl_poll_timeout(g_iar_dev->mipi_dsi_regaddr + CMD_PKT_STATUS,
			val, !(val & BIT(1)), 1000, 20000);
	if (ret < 0) {
		pr_err("failed to get available command FIFO\n");
		return ret;
	}
	value = (uint32_t)header | (uint32_t)cmd << 8 | (uint32_t)data << 16;
	writel(value, g_iar_dev->mipi_dsi_regaddr + GEN_HDR);
	mask = BIT(0) | BIT(2);
	ret = readl_poll_timeout(g_iar_dev->mipi_dsi_regaddr + CMD_PKT_STATUS,
			val, (val & mask) == mask, 1000, 20000);
	if (ret < 0) {
		pr_err("failed to write command FIFO\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(dsi_panel_write_cmd_poll);

int32_t user_set_dsi_panel_long_cmd(uint32_t len, uint8_t *tx_buf)
{
	int32_t pld_data_bytes = sizeof(uint32_t), ret;
	uint8_t len_tmp;
	uint32_t remainder;
	uint32_t val;

	if (len < 3 || len > 256) {
		pr_err("wrong tx buf length %u for long write\n", len);
		return -EINVAL;
	}

	len_tmp = (uint8_t)len;
	while (DIV_ROUND_UP(len, pld_data_bytes)) {
		if (len < pld_data_bytes) {
			remainder = 0;
			if (copy_from_user(&remainder, tx_buf, len))
				return -EFAULT;
			writel(remainder, g_iar_dev->mipi_dsi_regaddr + GEN_PLD_DATA);
			len = 0;
		} else {
			if (copy_from_user(&remainder, tx_buf, pld_data_bytes))
				return -EFAULT;
			writel(remainder, g_iar_dev->mipi_dsi_regaddr + GEN_PLD_DATA);
			tx_buf += pld_data_bytes;
			len -= pld_data_bytes;
		}

		ret = readl_poll_timeout(g_iar_dev->mipi_dsi_regaddr + CMD_PKT_STATUS,
				val, !(val & BIT(3)), 1000, 20000);
		if (ret < 0) {
			pr_err("failed to get available write payload FIFO\n");
			return ret;
		}
	}

	dsi_panel_write_cmd_poll(len_tmp, 0x00, 0x39);//byte1 byte2 byte0

	return 0;
}
EXPORT_SYMBOL_GPL(user_set_dsi_panel_long_cmd);

static int32_t dsi_panel_write_long_cmd(uint32_t len, uint8_t *tx_buf)
{
	int32_t pld_data_bytes = sizeof(uint32_t), ret;
	uint8_t len_tmp;
	uint32_t remainder;
	uint32_t val;

	if (len < 3 || len > 256) {
		pr_err("wrong tx buf length %u for long write\n", len);
		return -EINVAL;
	}

	len_tmp = (uint8_t)len;
	while (DIV_ROUND_UP(len, pld_data_bytes)) {
		if (len < pld_data_bytes) {
			remainder = 0;
			memcpy(&remainder, tx_buf, len);
			writel(remainder, g_iar_dev->mipi_dsi_regaddr + GEN_PLD_DATA);
			len = 0;
		} else {
			memcpy(&remainder, tx_buf, pld_data_bytes);
			writel(remainder, g_iar_dev->mipi_dsi_regaddr + GEN_PLD_DATA);
			tx_buf += pld_data_bytes;
			len -= pld_data_bytes;
		}

		ret = readl_poll_timeout(g_iar_dev->mipi_dsi_regaddr + CMD_PKT_STATUS,
				val, !(val & BIT(3)), 1000, 20000);
		if (ret < 0) {
			pr_err("failed to get available write payload FIFO\n");
			return ret;
		}
	}

	dsi_panel_write_cmd_poll(len_tmp, 0x00, 0x39);//byte1 byte2 byte0

	return 0;
}


void panel_enter_standby(void)
{
	dsi_panel_write_cmd(0x28, 0x00, 0x05);
	msleep(10);
	dsi_panel_write_cmd(0x10, 0x00, 0x05);
	msleep(120);
}
EXPORT_SYMBOL_GPL(panel_enter_standby);

void panel_exit_standby(void)
{
	msleep(10);
	dsi_panel_write_cmd(0x11, 0x00, 0x05);
	msleep(120);
	dsi_panel_write_cmd(0x29, 0x00, 0x05);
}
EXPORT_SYMBOL_GPL(panel_exit_standby);

void mipi_dsi_panel_config_begin(void)
{
	mipi_dsi_set_mode(1);//cmd mode
        writel(0xfffffffc, g_iar_dev->mipi_dsi_regaddr + CMD_MODE_CFG);// 0x68
        panel_hardware_reset();
}
EXPORT_SYMBOL_GPL(mipi_dsi_panel_config_begin);

int mipi_dsi_panel_init(uint8_t panel_no)
{
	mipi_dsi_set_mode(1);//cmd mode
	writel(0xfffffffc, g_iar_dev->mipi_dsi_regaddr + CMD_MODE_CFG);// 0x68
	panel_hardware_reset();

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
	} else if (panel_no == 1) {
		writel(0x038198ff, g_iar_dev->mipi_dsi_regaddr + GEN_PLD_DATA);// 0x70
		dsi_panel_write_cmd(0x04, 0x00, 0x39);//byte1 byte2 byte0
		dsi_panel_write_cmd(0x01, 0x00, 0x15);
		dsi_panel_write_cmd(0x02, 0x00, 0x15);
		dsi_panel_write_cmd(0x03, 0x72, 0x15);
		dsi_panel_write_cmd(0x04, 0x00, 0x15);
		dsi_panel_write_cmd(0x05, 0x00, 0x15);
		dsi_panel_write_cmd(0x06, 0x09, 0x15);
		dsi_panel_write_cmd(0x07, 0x00, 0x15);
		dsi_panel_write_cmd(0x08, 0x00, 0x15);
		dsi_panel_write_cmd(0x09, 0x01, 0x15);
		dsi_panel_write_cmd(0x0a, 0x00, 0x15);
		dsi_panel_write_cmd(0x0b, 0x00, 0x15);
		dsi_panel_write_cmd(0x0c, 0x01, 0x15);
		dsi_panel_write_cmd(0x0d, 0x00, 0x15);
		dsi_panel_write_cmd(0x0e, 0x00, 0x15);
		dsi_panel_write_cmd(0x0f, 0x00, 0x15);
		dsi_panel_write_cmd(0x10, 0x00, 0x15);
		dsi_panel_write_cmd(0x11, 0x00, 0x15);
		dsi_panel_write_cmd(0x12, 0x00, 0x15);
		dsi_panel_write_cmd(0x13, 0x00, 0x15);
		dsi_panel_write_cmd(0x14, 0x00, 0x15);
		dsi_panel_write_cmd(0x15, 0x00, 0x15);
		dsi_panel_write_cmd(0x16, 0x00, 0x15);
		dsi_panel_write_cmd(0x17, 0x00, 0x15);
		dsi_panel_write_cmd(0x18, 0x00, 0x15);
		dsi_panel_write_cmd(0x19, 0x00, 0x15);
		dsi_panel_write_cmd(0x1a, 0x00, 0x15);
		dsi_panel_write_cmd(0x1b, 0x00, 0x15);
		dsi_panel_write_cmd(0x1c, 0x00, 0x15);
		dsi_panel_write_cmd(0x1d, 0x00, 0x15);
		dsi_panel_write_cmd(0x1e, 0x40, 0x15);
		dsi_panel_write_cmd(0x1f, 0x80, 0x15);
		dsi_panel_write_cmd(0x20, 0x05, 0x15);
		dsi_panel_write_cmd(0x20, 0x05, 0x15);
		dsi_panel_write_cmd(0x21, 0x02, 0x15);
		dsi_panel_write_cmd(0x22, 0x00, 0x15);
		dsi_panel_write_cmd(0x23, 0x00, 0x15);
		dsi_panel_write_cmd(0x24, 0x00, 0x15);
		dsi_panel_write_cmd(0x25, 0x00, 0x15);
		dsi_panel_write_cmd(0x26, 0x00, 0x15);
		dsi_panel_write_cmd(0x27, 0x00, 0x15);
		dsi_panel_write_cmd(0x28, 0x33, 0x15);
		dsi_panel_write_cmd(0x29, 0x02, 0x15);
		dsi_panel_write_cmd(0x2a, 0x00, 0x15);
		dsi_panel_write_cmd(0x2b, 0x00, 0x15);
		dsi_panel_write_cmd(0x2c, 0x00, 0x15);
		dsi_panel_write_cmd(0x2d, 0x00, 0x15);
		dsi_panel_write_cmd(0x2e, 0x00, 0x15);
		dsi_panel_write_cmd(0x2f, 0x00, 0x15);
		dsi_panel_write_cmd(0x30, 0x00, 0x15);
		dsi_panel_write_cmd(0x31, 0x00, 0x15);
		dsi_panel_write_cmd(0x32, 0x00, 0x15);
		dsi_panel_write_cmd(0x32, 0x00, 0x15);
		dsi_panel_write_cmd(0x33, 0x00, 0x15);
		dsi_panel_write_cmd(0x34, 0x04, 0x15);
		dsi_panel_write_cmd(0x35, 0x00, 0x15);
		dsi_panel_write_cmd(0x36, 0x00, 0x15);
		dsi_panel_write_cmd(0x37, 0x00, 0x15);
		dsi_panel_write_cmd(0x38, 0x3c, 0x15);
		dsi_panel_write_cmd(0x39, 0x00, 0x15);
		dsi_panel_write_cmd(0x3a, 0x40, 0x15);
		dsi_panel_write_cmd(0x3b, 0x40, 0x15);
		dsi_panel_write_cmd(0x3c, 0x00, 0x15);
		dsi_panel_write_cmd(0x3d, 0x00, 0x15);
		dsi_panel_write_cmd(0x3e, 0x00, 0x15);
		dsi_panel_write_cmd(0x3f, 0x00, 0x15);
		dsi_panel_write_cmd(0x40, 0x00, 0x15);
		dsi_panel_write_cmd(0x41, 0x00, 0x15);
		dsi_panel_write_cmd(0x42, 0x00, 0x15);
		dsi_panel_write_cmd(0x43, 0x00, 0x15);
		dsi_panel_write_cmd(0x44, 0x00, 0x15);
		dsi_panel_write_cmd(0x50, 0x01, 0x15);
		dsi_panel_write_cmd(0x51, 0x23, 0x15);
		dsi_panel_write_cmd(0x52, 0x45, 0x15);
		dsi_panel_write_cmd(0x53, 0x67, 0x15);
		dsi_panel_write_cmd(0x54, 0x89, 0x15);
		dsi_panel_write_cmd(0x55, 0xab, 0x15);
		dsi_panel_write_cmd(0x56, 0x01, 0x15);
		dsi_panel_write_cmd(0x57, 0x23, 0x15);
		dsi_panel_write_cmd(0x58, 0x45, 0x15);
		dsi_panel_write_cmd(0x59, 0x67, 0x15);
		dsi_panel_write_cmd(0x5a, 0x89, 0x15);
		dsi_panel_write_cmd(0x5b, 0xab, 0x15);
		dsi_panel_write_cmd(0x5c, 0xcd, 0x15);
		dsi_panel_write_cmd(0x5d, 0xef, 0x15);
		dsi_panel_write_cmd(0x5e, 0x11, 0x15);
		dsi_panel_write_cmd(0x5f, 0x01, 0x15);
		dsi_panel_write_cmd(0x60, 0x00, 0x15);
		dsi_panel_write_cmd(0x61, 0x15, 0x15);
		dsi_panel_write_cmd(0x62, 0x14, 0x15);
		dsi_panel_write_cmd(0x63, 0x0e, 0x15);
		dsi_panel_write_cmd(0x64, 0x0f, 0x15);
		dsi_panel_write_cmd(0x65, 0x0c, 0x15);
		dsi_panel_write_cmd(0x66, 0x0d, 0x15);
		dsi_panel_write_cmd(0x67, 0x06, 0x15);
		dsi_panel_write_cmd(0x68, 0x02, 0x15);
		dsi_panel_write_cmd(0x69, 0x07, 0x15);
		dsi_panel_write_cmd(0x6a, 0x02, 0x15);
		dsi_panel_write_cmd(0x6b, 0x02, 0x15);
		dsi_panel_write_cmd(0x6c, 0x02, 0x15);
		dsi_panel_write_cmd(0x6d, 0x02, 0x15);
		dsi_panel_write_cmd(0x6e, 0x02, 0x15);
		dsi_panel_write_cmd(0x6f, 0x02, 0x15);
		dsi_panel_write_cmd(0x70, 0x02, 0x15);
		dsi_panel_write_cmd(0x71, 0x02, 0x15);
		dsi_panel_write_cmd(0x72, 0x02, 0x15);
		dsi_panel_write_cmd(0x73, 0x02, 0x15);
		dsi_panel_write_cmd(0x74, 0x02, 0x15);
		dsi_panel_write_cmd(0x75, 0x01, 0x15);
		dsi_panel_write_cmd(0x76, 0x00, 0x15);
		dsi_panel_write_cmd(0x77, 0x14, 0x15);
		dsi_panel_write_cmd(0x78, 0x15, 0x15);
		dsi_panel_write_cmd(0x79, 0x0e, 0x15);
		dsi_panel_write_cmd(0x7a, 0x0f, 0x15);
		dsi_panel_write_cmd(0x7b, 0x0c, 0x15);
		dsi_panel_write_cmd(0x7c, 0x0d, 0x15);
		dsi_panel_write_cmd(0x7d, 0x06, 0x15);
		dsi_panel_write_cmd(0x7e, 0x02, 0x15);
		dsi_panel_write_cmd(0x7f, 0x07, 0x15);
		dsi_panel_write_cmd(0x80, 0x02, 0x15);
		dsi_panel_write_cmd(0x81, 0x02, 0x15);
		dsi_panel_write_cmd(0x83, 0x02, 0x15);
		dsi_panel_write_cmd(0x84, 0x02, 0x15);
		dsi_panel_write_cmd(0x85, 0x02, 0x15);
		dsi_panel_write_cmd(0x86, 0x02, 0x15);
		dsi_panel_write_cmd(0x87, 0x02, 0x15);
		dsi_panel_write_cmd(0x88, 0x02, 0x15);
		dsi_panel_write_cmd(0x89, 0x02, 0x15);
		dsi_panel_write_cmd(0x8a, 0x02, 0x15);
		writel(0x048198ff, g_iar_dev->mipi_dsi_regaddr + GEN_PLD_DATA);// 0x70
		dsi_panel_write_cmd(0x04, 0x00, 0x39);
		dsi_panel_write_cmd(0x6c, 0x15, 0x15);
		dsi_panel_write_cmd(0x6e, 0x2a, 0x15);
		dsi_panel_write_cmd(0x6f, 0x34, 0x15);
		dsi_panel_write_cmd(0x3a, 0x94, 0x15);
		dsi_panel_write_cmd(0x8d, 0x15, 0x15);
		dsi_panel_write_cmd(0x87, 0xba, 0x15);
		dsi_panel_write_cmd(0x26, 0x76, 0x15);
		dsi_panel_write_cmd(0xb2, 0xd1, 0x15);
		dsi_panel_write_cmd(0xb5, 0x06, 0x15);
		writel(0x018198ff, g_iar_dev->mipi_dsi_regaddr + GEN_PLD_DATA);// 0x70
		dsi_panel_write_cmd(0x04, 0x00, 0x39);
		dsi_panel_write_cmd(0x22, 0x0a, 0x15);
		dsi_panel_write_cmd(0x31, 0x00, 0x15);
		dsi_panel_write_cmd(0x53, 0x90, 0x15);
		dsi_panel_write_cmd(0x55, 0xa2, 0x15);
		dsi_panel_write_cmd(0x50, 0xb7, 0x15);
		dsi_panel_write_cmd(0x51, 0xb7, 0x15);
		dsi_panel_write_cmd(0x60, 0x22, 0x15);
		dsi_panel_write_cmd(0x61, 0x00, 0x15);
		dsi_panel_write_cmd(0x62, 0x19, 0x15);
		dsi_panel_write_cmd(0x63, 0x10, 0x15);
		dsi_panel_write_cmd(0xa0, 0x08, 0x15);
		dsi_panel_write_cmd(0xa1, 0x1a, 0x15);
		dsi_panel_write_cmd(0xa2, 0x27, 0x15);
		dsi_panel_write_cmd(0xa3, 0x15, 0x15);
		dsi_panel_write_cmd(0xa4, 0x17, 0x15);
		dsi_panel_write_cmd(0xa5, 0x2a, 0x15);
		dsi_panel_write_cmd(0xa6, 0x1e, 0x15);
		dsi_panel_write_cmd(0xa7, 0x1f, 0x15);
		dsi_panel_write_cmd(0xa8, 0x8b, 0x15);
		dsi_panel_write_cmd(0xa9, 0x1b, 0x15);
		dsi_panel_write_cmd(0xaa, 0x27, 0x15);
		dsi_panel_write_cmd(0xab, 0x78, 0x15);
		dsi_panel_write_cmd(0xac, 0x18, 0x15);
		dsi_panel_write_cmd(0xad, 0x18, 0x15);
		dsi_panel_write_cmd(0xae, 0x4c, 0x15);
		dsi_panel_write_cmd(0xaf, 0x21, 0x15);
		dsi_panel_write_cmd(0xb0, 0x27, 0x15);
		dsi_panel_write_cmd(0xb1, 0x54, 0x15);
		dsi_panel_write_cmd(0xb2, 0x67, 0x15);
		dsi_panel_write_cmd(0xb3, 0x39, 0x15);
		dsi_panel_write_cmd(0xc0, 0x08, 0x15);
		dsi_panel_write_cmd(0xc1, 0x1a, 0x15);
		dsi_panel_write_cmd(0xc2, 0x27, 0x15);
		dsi_panel_write_cmd(0xc3, 0x15, 0x15);
		dsi_panel_write_cmd(0xc4, 0x17, 0x15);
		dsi_panel_write_cmd(0xc5, 0x2a, 0x15);
		dsi_panel_write_cmd(0xc6, 0x1e, 0x15);
		dsi_panel_write_cmd(0xc7, 0x1f, 0x15);
		dsi_panel_write_cmd(0xc8, 0x8b, 0x15);
		dsi_panel_write_cmd(0xc9, 0x1b, 0x15);
		dsi_panel_write_cmd(0xca, 0x27, 0x15);
		dsi_panel_write_cmd(0xcb, 0x78, 0x15);
		dsi_panel_write_cmd(0xcc, 0x18, 0x15);
		dsi_panel_write_cmd(0xcd, 0x18, 0x15);
		dsi_panel_write_cmd(0xce, 0x4c, 0x15);
		dsi_panel_write_cmd(0xcf, 0x21, 0x15);
		dsi_panel_write_cmd(0xd0, 0x27, 0x15);
		dsi_panel_write_cmd(0xd1, 0x54, 0x15);
		dsi_panel_write_cmd(0xd2, 0x67, 0x15);
		dsi_panel_write_cmd(0xd3, 0x39, 0x15);
		writel(0x008198ff, g_iar_dev->mipi_dsi_regaddr + GEN_PLD_DATA);// 0x70
		dsi_panel_write_cmd(0x04, 0x00, 0x39);
		dsi_panel_write_cmd(0x3a, 0x07, 0x15);
		dsi_panel_write_cmd(0x11, 0x00, 0x05);
		msleep(120);
		dsi_panel_write_cmd(0x29, 0x00, 0x05);
		msleep(100);

		mipi_dsi_set_mode(0);//video mode
		pr_info("mipi dsi panel config end!\n");
		return 0;
	} else if (panel_no == 2) {
		int i = 0;
		pr_info("mipi 720p svb portrait dsi panel config start!\n");
		while (init_para_720x1280_sdb[i].cmd_type != MIPI_OFF) {
			if (init_para_720x1280_sdb[i].cmd_type == MIPI_LCP) {
				dsi_panel_write_long_cmd(init_para_720x1280_sdb[i].cmd_len,
						init_para_720x1280_sdb[i].cmd);
			} else if (init_para_720x1280_sdb[i].cmd_type == MIPI_S1P) {
				dsi_panel_write_cmd_poll(init_para_720x1280_sdb[i].cmd[0],
						init_para_720x1280_sdb[i].cmd[1], 0x15);
			} else if (init_para_720x1280_sdb[i].cmd_type == MIPI_SNP) {
				dsi_panel_write_cmd_poll(init_para_720x1280_sdb[i].cmd[0],
						0x00, 0x05);
			} else if (init_para_720x1280_sdb[i].cmd_type == MIPI_SLP) {
				msleep(init_para_720x1280_sdb[i].cmd_len);
			}
			i++;
		}
		mipi_dsi_set_mode(0);//video mode
		pr_info("mipi dsi panel config end!\n");
		return 0;
        } 
		//add by jiale01.luo
		else if(panel_no == 4){
			pr_debug("Nothing to do on panel_no:%d\n",panel_no);
		// 	pr_err("START INIT DSI CMD...\n");
		// 	pr_err("****************************************\n");
		// 	int i = 0;
		// 	while (init_para_800x480_cm[i].cmd_type != MIPI_OFF) {
		// 	if (init_para_800x480_cm[i].cmd_type == MIPI_LCP) {
		// 		dsi_panel_write_long_cmd(init_para_800x480_cm[i].cmd_len,
		// 				init_para_800x480_cm[i].cmd);
		// 	} else if (init_para_800x480_cm[i].cmd_type == MIPI_S1P) {
		// 		dsi_panel_write_cmd_poll(init_para_800x480_cm[i].cmd[0],
		// 				init_para_800x480_cm[i].cmd[1], 0x15);
		// 	} else if (init_para_800x480_cm[i].cmd_type == MIPI_SNP) {
		// 		dsi_panel_write_cmd_poll(init_para_800x480_cm[i].cmd[0],
		// 				0x00, 0x05);
		// 	} else if (init_para_800x480_cm[i].cmd_type == MIPI_SLP) {
		// 		msleep(init_para_800x480_cm[i].cmd_len);
		// 	}
		// 	i++;
		// }		
		// 	mipi_dsi_set_mode(0);//video mode
		// 	pr_err("mipi dsi panel config end!\n");
		// 	return 0;
		}
		else {
		pr_err("%s: not support panel type,", __func__);
		pr_err("please add panel init code!!\n");
	}
	return 0;
}

int set_mipi_display(uint8_t panel_no)
{
	pr_info("mipi: set mipi display begin!\n");
	mipi_dsi_core_pre_init(panel_no);
	mipi_dsi_dpi_config(panel_no);
	if (panel_no == 0) {
		mipi_dsi_video_config(&video_1080_1920);
		memcpy(&mipi_timing, &video_1080_1920, sizeof(struct video_timing));
	} else if (panel_no == 1) {
		mipi_dsi_video_config(&video_720_1280);
		memcpy(&mipi_timing, &video_720_1280, sizeof(struct video_timing));
	} else if (panel_no == 2) {
		mipi_dsi_video_config(&video_720_1280_sdb);
		memcpy(&mipi_timing, &video_720_1280_sdb, sizeof(struct video_timing));
	} else if (panel_no == 3) {
		mipi_dsi_video_config(&video_1280_720);
		memcpy(&mipi_timing, &video_1280_720, sizeof(struct video_timing));
	//add by jiale01.luo
	} else if (panel_no == 4) {
		mipi_dsi_video_config(&video_800_480_cm);
		memcpy(&mipi_timing, &video_800_480_cm, sizeof(struct video_timing));
		pr_debug("*****dsi timing is %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
				video_800_480_cm.vid_pkt_size,
				video_800_480_cm.vid_num_chunks,
				video_800_480_cm.vid_null_size,
				video_800_480_cm.vid_hsa,
				video_800_480_cm.vid_hbp,
				video_800_480_cm.vid_hline_time,
				video_800_480_cm.vid_vsa,
				video_800_480_cm.vid_vbp,
				video_800_480_cm.vid_vfp,
				video_800_480_cm.vid_vactive_line);
	}

	msleep(100);
	if(panel_no != 4){
		//TODO: Find out why waveshare lcd can't enter CMD mode
		mipi_dsi_panel_init(panel_no);
	}
	mipi_dsi_set_mode(0);//video mode
	mipi_dsi_vid_mode_cfg(0);//normal mode
	mipi_config_update();
	pr_info("mipi: set mipi display end!\n");
	return 0;
}
EXPORT_SYMBOL_GPL(set_mipi_display);

int user_init_mipi_dsi_core(struct mipi_dsi_core_init_data *init)
{
	if (init == NULL) {
		pr_err("%s: input init data is NULL,exit!!\n", __func__);
		return -1;
	}

	pr_debug("%s: user init mipi dsi core begin!\n", __func__);
	if (init->width == 1080 && init->height == 1920) {
		mipi_dsi_core_pre_init(0);
		mipi_dsi_dpi_config(0);
	} else if (init->width == 720 && init->height == 1280) {
		mipi_dsi_core_pre_init(1);
		mipi_dsi_dpi_config(1);
	} else if (init->width == 1280 && init->height == 720) {
		mipi_dsi_core_pre_init(1);
		mipi_dsi_dpi_config(1);
	} else {
		pr_err("%s: unsupport panel resolution!!\n", __func__);
		return -1;
	}
	mipi_dsi_video_config(&init->timing);
	memcpy(&mipi_timing, &init->timing, sizeof(struct video_timing));
	msleep(100);
	return 0;
}
EXPORT_SYMBOL_GPL(user_init_mipi_dsi_core);
