/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
/**
 * @author   Jesse.Huang(Jesse.Huang@hobot.cc)
 * @date     2019/11/27
 * @version  V1.0
 * @par      Horizon Robotics
 */

#ifndef DRIVERS_IPS_USRDRV_INC_CLOCK_DUMP_LAYOUT_H_
#define DRIVERS_IPS_USRDRV_INC_CLOCK_DUMP_LAYOUT_H_
/****************************************************************
* MACROS TOOL
****************************************************************/
#define TYPE_MP			0	/*MP:         Mux input from Parent */
#define TYPE_MPD		1	/*MPD:        Mux input from Parent with Div */
#define TYPE_MV			2	/*MV:         Mux Value */
#define TYPE_CST		3	/*CST:        Constant */
#define TYPE_PLL		4	/*PLL:*/
/*freq=(z[0:0]==1)? 0:((((24mhz/y[17:12])*y[11:0]))/y[22:20])/y[26/24] */
/*y=VIOPLL2_FREQ_CTRL, z=VIOPLL2_PD_CTRL */

#define CK_BEG(type, reg_name, offset, msb, lsb)	\
	{reg_name,	TYPE_##type, {offset, msb, lsb, INV}, INV_CLK, {
#define CK_END()									{INV, XV, XS}, }, },
#define CK_MP_ADD(key, parent_reg_name)				\
	{key, XV, parent_reg_name},
#define CK_MPD_ADD(key, parent_reg_name, div)		\
	{key, div, parent_reg_name},
#define CK_MV_ADD(key, val)							{key, val, XS},
/****************************************************************
* ADVANCE Macro OFFSTA_DIV_DIV_PLL
****************************************************************/
//STA_DIV_DIV_PLL value of sta==1 is poweroff
#define STA_DIV_DIV_PLL(sta, reg0, msb0, lsb0, div2, \
	reg1, msb1, lsb1, div1, reg2, msb2, lsb2, pll) \
	CK_BEG(MP, sta, reg0, msb0, lsb0) \
	CK_MP_ADD(0, div2) \
	CK_MP_ADD(1, NO_CLK) \
	CK_END() \
    \
	CK_BEG(MPD, div2, reg1, msb1, lsb1) \
	CK_MPD_ADD(0, div1, 0) /*both 0 will not be use*/\
	CK_END() \
	\
	CK_BEG(MPD, div1, reg2, msb2, lsb2) \
	CK_MPD_ADD(0, pll, 0) /*both 0 will not be use*/\
	CK_END()

/****************************************************************
* CLOCK ENDPOINT DEFINE(for final display)
****************************************************************/
char clk_endpoint[][MAX_CLK_NAME] = {
	"pym_ddrclk_gate_en",
	"pym_ipuclk_gate_en",
	"pym_uvclk_gate_en0",
	"pym_usclk_gate_en0",
	"pym_uvclk_gate_en1",
	"pym_usclk_gate_en1",
	"pym_uvclk_gate_en2",
	"pym_usclk_gate_en2",
	"pym_uvclk_gate_en3",
	"pym_usclk_gate_en3",
	"sw_sif_cg_en",
	"sw_ipu0_cg_en",
	"sw_ipu1_cg_en",
	"mipi_tx_ipi_clk_clkoff_sta",
	"mipi_rx0_ipi_clk_clkoff_sta",
	"mipi_rx1_ipi_clk_clkoff_sta",
	"mipi_rx2_ipi_clk_clkoff_sta",
	"mipi_rx3_ipi_clk_clkoff_sta",
	"sw_mipi_host_cfg_cg_en",
	"mipi_dev_fsfreqrange",
	"mipi_rx0_fsfreqrange",
	"mipi_rx1_fsfreqrange",
	"mipi_rx2_fsfreqrange",
	"mipi_rx3_fsfreqrange",
	"mipi_dev_cfgclkfreqrange",
	"mipi_rx0_cfgclkfreqrange",
	"mipi_rx1_cfgclkfreqrange",
	"mipi_rx2_cfgclkfreqrange",
	"mipi_rx3_cfgclkfreqrange",
	""
};

/****************************************************************
* CLOCK TABLE DEFINE
****************************************************************/

#define VIOSYS_CLKEN 			0x01000140
#define VIOSYS_CLK_DIV_SEL1		0x01000240
#define VIOSYS_CLK_DIV_SEL2		0x01000244
#define VIOSYS_CLKOFF_STA		0x01000248
#define VIOSYS_CLK_DIV_SEL3		0x0100024C
#define PLLCLK_SEL				0x01000300
#define SYSPLL_FREQ_CTRL		0x01000010
#define SYSPLL_PD_CTRL			0x01000014
#define VIOPLL_FREQ_CTRL		0x01000040
#define VIOPLL_PD_CTRL			0x01000044
#define VIOPLL2_FREQ_CTRL		0x010000B0
#define VIOPLL2_PD_CTRL			0x010000B4

#define IPS_CLK_CTRL			0x0400000C
#define PYM_PYRAMID_CTRL		0x04042000
#define PYM_CFG0				0x04042204
#define PYM_CFG1				0x04042404
#define PYM_CFG2				0x04042604
#define PYM_CFG3				0x04042804

#define VIO_CLK_CTRL            0x0430000C
#define MIPI_DEV_FREQRANGE		0x0430008C
#define MIPI_RX0_FREQRANGE		0x043000C4
#define MIPI_RX1_FREQRANGE		0x043000CC
#define MIPI_RX2_FREQRANGE		0x043000D4
#define MIPI_RX3_FREQRANGE		0x043000DC

#define MIPI_FREQRANGE_TLB  \
	CK_MV_ADD(0x00,   80)\
	CK_MV_ADD(0x10,   90)\
	CK_MV_ADD(0x20,  100)\
	CK_MV_ADD(0x30,  110)\
	CK_MV_ADD(0x01,  120)\
	CK_MV_ADD(0x11,  130)\
	CK_MV_ADD(0x21,  140)\
	CK_MV_ADD(0x31,  150)\
	CK_MV_ADD(0x02,  160)\
	CK_MV_ADD(0x12,  170)\
	CK_MV_ADD(0x22,  180)\
	CK_MV_ADD(0x32,  190)\
	CK_MV_ADD(0x03,  205)\
	CK_MV_ADD(0x13,  220)\
	CK_MV_ADD(0x23,  235)\
	CK_MV_ADD(0x33,  250)\
	CK_MV_ADD(0x04,  275)\
	CK_MV_ADD(0x14,  300)\
	CK_MV_ADD(0x25,  325)\
	CK_MV_ADD(0x35,  350)\
	CK_MV_ADD(0x05,  400)\
	CK_MV_ADD(0x16,  450)\
	CK_MV_ADD(0x26,  500)\
	CK_MV_ADD(0x37,  550)\
	CK_MV_ADD(0x07,  600)\
	CK_MV_ADD(0x18,  650)\
	CK_MV_ADD(0x28,  700)\
	CK_MV_ADD(0x39,  750)\
	CK_MV_ADD(0x09,  800)\
	CK_MV_ADD(0x19,  850)\
	CK_MV_ADD(0x29,  900)\
	CK_MV_ADD(0x3A,  950)\
	CK_MV_ADD(0x0A, 1000)\
	CK_MV_ADD(0x1A, 1050)\
	CK_MV_ADD(0x2A, 1100)\
	CK_MV_ADD(0x3B, 1150)\
	CK_MV_ADD(0x0B, 1200)\
	CK_MV_ADD(0x1B, 1250)\
	CK_MV_ADD(0x2B, 1300)\
	CK_MV_ADD(0x3C, 1350)\
	CK_MV_ADD(0x0C, 1400)\
	CK_MV_ADD(0x1C, 1450)\
	CK_MV_ADD(0x2C, 1500)\
	CK_MV_ADD(0x3D, 1550)\
	CK_MV_ADD(0x0D, 1600)\
	CK_MV_ADD(0x1D, 1650)\
	CK_MV_ADD(0x2D, 1700)\
	CK_MV_ADD(0x3E, 1750)\
	CK_MV_ADD(0x0E, 1800)\
	CK_MV_ADD(0x1E, 1850)\
	CK_MV_ADD(0x2F, 1900)\
	CK_MV_ADD(0x3F, 1950)\
	CK_MV_ADD(0x0F, 2000)\
	CK_MV_ADD(0x40, 2050)\
	CK_MV_ADD(0x41, 2100)\
	CK_MV_ADD(0x42, 2150)\
	CK_MV_ADD(0x43, 2200)\
	CK_MV_ADD(0x44, 2250)\
	CK_MV_ADD(0x45, 2300)\
	CK_MV_ADD(0x46, 2350)\
	CK_MV_ADD(0x47, 2400)\
	CK_MV_ADD(0x48, 2450)\
	CK_MV_ADD(0x49, 2500)

clk_element_t g_clk_element[MAX_CLK_ELEMENT] = {
/*===MIPI===*/
	STA_DIV_DIV_PLL("mipi_tx_ipi_clk_clkoff_sta", VIOSYS_CLKOFF_STA, 11, 11,
			"mipi_tx_ipi_clk_2nd_div_sel", VIOSYS_CLK_DIV_SEL2, 31,
			29,
			"mipi_tx_ipi_clk_1nd_div_sel", VIOSYS_CLK_DIV_SEL2, 28,
			24, "vio_pllmux_clk")
	    STA_DIV_DIV_PLL("mipi_rx3_ipi_clk_clkoff_sta", VIOSYS_CLKOFF_STA,
			    10, 10, "mipi_rx3_ipi_clk_2nd_div_sel",
			    VIOSYS_CLK_DIV_SEL3, 31, 29,
			    "mipi_rx3_ipi_clk_1nd_div_sel", VIOSYS_CLK_DIV_SEL3,
			    28, 24, "vio_pllmux_clk")
	    STA_DIV_DIV_PLL("mipi_rx2_ipi_clk_clkoff_sta", VIOSYS_CLKOFF_STA, 9,
			    9, "mipi_rx2_ipi_clk_2nd_div_sel",
			    VIOSYS_CLK_DIV_SEL3, 23, 21,
			    "mipi_rx2_ipi_clk_1nd_div_sel", VIOSYS_CLK_DIV_SEL3,
			    20, 16, "vio_pllmux_clk")
	    STA_DIV_DIV_PLL("mipi_rx1_ipi_clk_clkoff_sta", VIOSYS_CLKOFF_STA, 8,
			    8, "mipi_rx1_ipi_clk_2nd_div_sel",
			    VIOSYS_CLK_DIV_SEL3, 15, 13,
			    "mipi_rx1_ipi_clk_1nd_div_sel", VIOSYS_CLK_DIV_SEL3,
			    12, 8, "vio_pllmux_clk")
	    STA_DIV_DIV_PLL("mipi_rx0_ipi_clk_clkoff_sta", VIOSYS_CLKOFF_STA, 7,
			    7, "mipi_rx0_ipi_clk_2nd_div_sel",
			    VIOSYS_CLK_DIV_SEL3, 7, 5,
			    "mipi_rx0_ipi_clk_1nd_div_sel", VIOSYS_CLK_DIV_SEL3,
			    4, 0, "vio_pllmux_clk")

	    CK_BEG(MP, "sw_mipi_host_cfg_cg_en", VIO_CLK_CTRL, 1, 1)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "mipi_rx0_fsfreqrange")
	    CK_END()

	    CK_BEG(MV, "mipi_dev_fsfreqrange", MIPI_DEV_FREQRANGE, 6, 0)
	    MIPI_FREQRANGE_TLB CK_END()

	    CK_BEG(MV, "mipi_rx0_fsfreqrange", MIPI_RX0_FREQRANGE, 6, 0)
	    MIPI_FREQRANGE_TLB CK_END()

	    CK_BEG(MV, "mipi_rx1_fsfreqrange", MIPI_RX1_FREQRANGE, 6, 0)
	    MIPI_FREQRANGE_TLB CK_END()

	    CK_BEG(MV, "mipi_rx2_fsfreqrange", MIPI_RX2_FREQRANGE, 6, 0)
	    MIPI_FREQRANGE_TLB CK_END()

	    CK_BEG(MV, "mipi_rx3_fsfreqrange", MIPI_RX3_FREQRANGE, 6, 0)
	    MIPI_FREQRANGE_TLB CK_END()

	    CK_BEG(MV, "mipi_dev_cfgclkfreqrange", MIPI_DEV_FREQRANGE, 15, 8)
	    CK_MV_ADD(0x1c, 28)
	    CK_END()

	    CK_BEG(MV, "mipi_rx0_cfgclkfreqrange", MIPI_RX0_FREQRANGE, 15, 8)
	    CK_MV_ADD(0x1c, 28)
	    CK_END()

	    CK_BEG(MV, "mipi_rx1_cfgclkfreqrange", MIPI_RX1_FREQRANGE, 15, 8)
	    CK_MV_ADD(0x1c, 28)
	    CK_END()

	    CK_BEG(MV, "mipi_rx2_cfgclkfreqrange", MIPI_RX2_FREQRANGE, 15, 8)
	    CK_MV_ADD(0x1c, 28)
	    CK_END()

	    CK_BEG(MV, "mipi_rx3_cfgclkfreqrange", MIPI_RX3_FREQRANGE, 15, 8)
	    CK_MV_ADD(0x1c, 28)
	    CK_END()
/*===SIF===*/
	    CK_BEG(MP, "sw_sif_cg_en", IPS_CLK_CTRL, 0, 0)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "sif_mclk")
	    CK_END()

	    CK_BEG(MP, "sif_mclk", VIOSYS_CLKEN, 1, 1)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "SIF_MCLK_DIV_SEL")
	    CK_END()

	    CK_BEG(MPD, "SIF_MCLK_DIV_SEL", VIOSYS_CLK_DIV_SEL1, 15, 12)
	    CK_MPD_ADD(0, "vio_pllmux_clk", 0)	//both 0 will not be use
	    CK_END()
/*===IPU===*/
	    CK_BEG(MP, "sw_ipu0_cg_en", IPS_CLK_CTRL, 11, 11)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "sif_mclk")	/*TODO: need ask ipu is from sif_mclk?? */
	    CK_END()

	    CK_BEG(MP, "sw_ipu1_cg_en", IPS_CLK_CTRL, 12, 12)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "sif_mclk")	/*TODO: need ask ipu is from sif_mclk?? */
	    CK_END()
/*===PYM===*/
	    CK_BEG(MP, "pym_ddrclk_gate_en", PYM_PYRAMID_CTRL, 29, 29)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "pym_clk")
	    CK_END()

	    CK_BEG(MP, "pym_ipuclk_gate_en", PYM_PYRAMID_CTRL, 28, 28)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "pym_clk")
	    CK_END()

	    CK_BEG(MP, "pym_uvclk_gate_en0", PYM_CFG0, 17, 17)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "pym_clk")
	    CK_END()

	    CK_BEG(MP, "pym_usclk_gate_en0", PYM_CFG0, 16, 16)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "pym_clk")
	    CK_END()

	    CK_BEG(MP, "pym_uvclk_gate_en1", PYM_CFG1, 17, 17)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "pym_clk")
	    CK_END()

	    CK_BEG(MP, "pym_usclk_gate_en1", PYM_CFG1, 16, 16)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "pym_clk")
	    CK_END()

	    CK_BEG(MP, "pym_uvclk_gate_en2", PYM_CFG2, 17, 17)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "pym_clk")
	    CK_END()

	    CK_BEG(MP, "pym_usclk_gate_en2", PYM_CFG2, 16, 16)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "pym_clk")
	    CK_END()

	    CK_BEG(MP, "pym_uvclk_gate_en3", PYM_CFG3, 17, 17)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "pym_clk")
	    CK_END()

	    CK_BEG(MP, "pym_usclk_gate_en3", PYM_CFG3, 16, 16)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "pym_clk")
	    CK_END()

	    CK_BEG(MP, "pym_clk", VIOSYS_CLKEN, 5, 5)
	    CK_MP_ADD(0, NO_CLK)
	    CK_MP_ADD(1, "PYM_MCLK_DIV_SEL")
	    CK_END()

	    CK_BEG(MPD, "PYM_MCLK_DIV_SEL", VIOSYS_CLK_DIV_SEL2, 11, 8)
	    CK_MPD_ADD(0, "pym_mclk_src_sel", 0)	//both 0 will not be use
	    CK_END()

	    CK_BEG(MP, "pym_mclk_src_sel", VIOSYS_CLK_DIV_SEL2, 13, 12)
	    CK_MP_ADD(0, "vio_pllmux_clk")
	    CK_MP_ADD(1, "vio2_pllmux_clk")
	    CK_MP_ADD(2, "sys_vco_pllmux_clk")
	    CK_END()
/*====================================PLL====================================*/
//viopll
	    CK_BEG(MP, "vio_pllmux_clk", PLLCLK_SEL, 16, 16)
	    CK_MP_ADD(0, MHZ24)
	    CK_MP_ADD(1, "VIOPLL_PD_CTRL")
	    CK_END()

	    CK_BEG(MP, "VIOPLL_PD_CTRL", VIOPLL_PD_CTRL, 0, 0)
	    CK_MP_ADD(0, "VIOPLL_FREQ_CTRL")
	    CK_MP_ADD(1, NO_CLK)
	    CK_END()

	    CK_BEG(PLL, "VIOPLL_FREQ_CTRL", VIOPLL_FREQ_CTRL, 26, 0)
	    CK_END()
//viopll2
	    CK_BEG(MP, "vio2_pllmux_clk", PLLCLK_SEL, 17, 17)
	    CK_MP_ADD(0, MHZ24)
	    CK_MP_ADD(1, "VIOPLL2_PD_CTRL")
	    CK_END()

	    CK_BEG(MP, "VIOPLL2_PD_CTRL", VIOPLL2_PD_CTRL, 0, 0)
	    CK_MP_ADD(0, "VIOPLL2_FREQ_CTRL")
	    CK_MP_ADD(1, NO_CLK)
	    CK_END()

	    CK_BEG(PLL, "VIOPLL2_FREQ_CTRL", VIOPLL2_FREQ_CTRL, 26, 0)
	    CK_END()
//sys_vco
	    CK_BEG(MP, "sys_vco_pllmux_clk", PLLCLK_SEL, 25, 25)
	    CK_MP_ADD(0, MHZ24)
	    CK_MP_ADD(1, "SYSPLL_PD_CTRL")
	    CK_END()

	    CK_BEG(MP, "SYSPLL_PD_CTRL", SYSPLL_PD_CTRL, 0, 0)
	    CK_MP_ADD(0, "SYSPLL_FREQ_CTRL")
	    CK_MP_ADD(1, NO_CLK)
	    CK_END()

	    CK_BEG(PLL, "SYSPLL_FREQ_CTRL", SYSPLL_FREQ_CTRL, 26, 0)
		CK_END()
        {""},
};
#endif //DRIVERS_IPS_USRDRV_INC_CLOCK_DUMP_LAYOUT_H_
