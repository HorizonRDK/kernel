/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_IPS_X2_H__
#define __HOBOT_IPS_X2_H__

#define MOT_DET                 (1 << 28)
#define ISP_HIST_FRAME_DONE     (1 << 27)
#define ISP_RSUM_FRAME_DONE     (1 << 26)
#define ISP_GRID_FRAME_DONE     (1 << 25)
#define ISP_TILE_FRAME_DONE     (1 << 24)
#define ISP_HIST_FRAME_DROP     (1 << 23)
#define ISP_RSUM_FRAME_DROP     (1 << 22)
#define ISP_GRID_FRAME_DROP     (1 << 21)
#define ISP_TILE_FRAME_DROP     (1 << 20)
#define ISP_RCCB_FRAME_FINISH   (1 << 19)
#define SIF_SIZE_ERR1           (1 << 18)
#define SIF_SIZE_ERR0           (1 << 17)
#define ISP_STF_FRAME_DROP      (1 << 16)
#define ISP_HMP_FRAME_DROP      (1 << 15)
#define ISP_STF_FRAME_FINISH    (1 << 14)
#define ISP_HMP_FRAME_FINISH    (1 << 13)
#define ISP_FRAME_START     (1 << 12)
#define PYM_US_FRAME_DROP   (1 << 11)
#define PYM_DS_FRAME_DROP   (1 << 10)
#define PYM_FRAME_DONE      (1 << 9)
#define PYM_FRAME_START     (1 << 8)
#define IPU_FRAME_DONE      (1 << 7)
#define IPU_FRAME_START     (1 << 6)
#define IPU_BUS23_TRANSMIT_ERRORS   (1 << 5)
#define IPU_BUS01_TRANSMIT_ERRORS   (1 << 4)
#define RESERVED        (1 << 3)
#define SIF_SOFT_DROP   (1 << 2)
#define SIF_FRAME_END_INTERRUPT     (1 << 1)
#define SIF_FRAME_START_INTERRUPT   (1 << 0)

#define ISP_INT_BITS    (ISP_RSUM_FRAME_DONE|ISP_GRID_FRAME_DONE|ISP_TILE_FRAME_DONE|ISP_HIST_FRAME_DROP|ISP_RSUM_FRAME_DROP|ISP_GRID_FRAME_DROP|ISP_TILE_FRAME_DROP|ISP_RCCB_FRAME_FINISH)
#define SIF_INT_BITS    (SIF_SIZE_ERR1|SIF_SIZE_ERR0|SIF_SOFT_DROP|SIF_FRAME_END_INTERRUPT|SIF_FRAME_START_INTERRUPT|MOT_DET)
#define IPU_INT_BITS    (IPU_FRAME_DONE|IPU_FRAME_START|IPU_BUS23_TRANSMIT_ERRORS|IPU_BUS01_TRANSMIT_ERRORS|PYM_FRAME_START|PYM_FRAME_DONE|PYM_DS_FRAME_DROP|PYM_US_FRAME_DROP)

enum busctl_region_e {
	BUSCTL_REGION_PRI,
	BUSCTL_REGION_ENDIAN,
	BUSCTL_REGION_MAXLEN,
};
enum busctl_type_e {
	BUSCTL_WM,
	BUSCTL_RM,
};
enum ips_ctl_region_e {
	MIPI_DEV_CFG_CLK_GATE_EN,
	MIPI_HOST_CFG_CLK_GATE_EN,
	MIPI_DEV_IPI_CLK_GATE_EN,
	MIPI_HOST_IPI_CLK_GATE_EN,
	PDM_SRAM_CLK_GATE_EN,
	TILE_SRAM_CLK_GATE_EN,
	MOT_DET_CLK_GATE_EN,
	RCCB_CLK_GATE_EN,
	IPU_CLK_GATE_EN,
	ISP_CLK_GATE_EN,
};
enum ips_status_region_e {
	PYM_STATUS,
	IPU_STATUS,
	ISP_STATUS,
	SIF_STATUS,
};
enum ips_mipi_freqrange_region_e {
	MIPI_DEV_CFGCLKFREQRANGE,
	MIPI_DEV_HSFREQRANGE,
	MIPI_HOST_CFGCLKFREQRANGE,
	MIPI_HOST_HSFREQRANGE,
};
enum ips_mipictl_region_e {
	MIPI_BYPASS_GEN_HSYNC_DLY_CNT,
	MIPI_BYPASS_GEN_HSYNC_EN,
	MIPI_DEV_SHADOW_CLEAR,
};
enum {
	SIF_INT,
	ISP_INT,
	IPU_INT,
};
enum {
	IAR_CLK,
	BYPASS_CLK,
};
enum {
	RST_MIPI_IPI = 0x01,
	RST_MIPI_CFG = 0x02,
	RST_SIF      = 0x04,
	RST_IPU      = 0x08,
	RST_DVP      = 0x10,
	RST_BT       = 0x20,
};
typedef void (*ips_irqhandler_t)(unsigned int intstatus, void *data);
int ips_irq_enable(int irq);
int ips_irq_disable(int irq);
int ips_mask_int(unsigned int mask);
int ips_unmask_int(unsigned int mask);
int ips_register_irqhandle(int irq, ips_irqhandler_t handle, void *data);
unsigned int ips_get_intstatus(void);

int ips_mipi_ctl_set(unsigned int region, unsigned int value);
int ips_busctl_set(unsigned int type, unsigned int index, unsigned int region, unsigned int value);
int ips_busctl_get(unsigned int type, unsigned int index, unsigned int region);
int ips_mipi_ctl_set(unsigned int region, unsigned int value);
int ips_mipi_ctl_get(unsigned int region);
int ips_control_set(unsigned int region, unsigned int state);
int ips_control_get(unsigned int region);
int ips_get_status(unsigned int region);
int ips_set_mipi_freqrange(unsigned int region, unsigned int value);
int ips_get_mipi_freqrange(unsigned int region);
int ips_set_mipi_ipi_clk(unsigned long clk);
unsigned long ips_get_mipi_ipi_clk(void);
int ips_pinmux_bt(void);
int ips_pinmux_dvp(void);
int ips_set_btout_clksrc(unsigned int mode, uint8_t invert);
int ips_set_btin_clksrc(uint8_t invert);
void ips_module_reset(unsigned int module);

#endif /* __HOBOT_IPS_X2_H__ */
