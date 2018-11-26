/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __X2_IPS_H
#define __X2_IPS_H

#define IPSINTMASK          0x0
#define IPSINTSTATE         0x4

#define MOT_DET     (1 << 28)
#define IPS_HIST_FRAME_DONE     (1 << 27)
#define IPS_RSUM_FRAME_DONE     (1 << 26)
#define IPS_GRID_FRAME_DONE     (1 << 25)
#define IPS_TILE_FRAME_DONE     (1 << 24)
#define IPS_HIST_FRAME_DROP     (1 << 23)
#define IPS_RSUM_FRAME_DROP     (1 << 22)
#define IPS_GRID_FRAME_DROP     (1 << 21)
#define IPS_TILE_FRAME_DROP     (1 << 20)
#define IPS_RCCB_FRAME_FINISH   (1 << 19)
#define SIF_SIZE_ERR1       (1 << 18)
#define SIF_SIZE_ERR0       (1 << 17)
#define IPS_STF_FRAME_DROP      (1 << 16)
#define IPS_HMP_FRAME_DROP      (1 << 15)
#define IPS_STF_FRAME_FINISH    (1 << 14)
#define IPS_HMP_FRAME_FINISH    (1 << 13)
#define IPS_FRAME_START     (1 << 12)
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

#define ISP_INT_BITS    (IPS_RSUM_FRAME_DONE|IPS_GRID_FRAME_DONE|IPS_TILE_FRAME_DONE|IPS_HIST_FRAME_DROP|IPS_RSUM_FRAME_DROP|IPS_GRID_FRAME_DROP|IPS_TILE_FRAME_DROP|IPS_RCCB_FRAME_FINISH)
#define SIF_INT_BITS    (SIF_SIZE_ERR1|SIF_SIZE_ERR0|SIF_SOFT_DROP|SIF_FRAME_END_INTERRUPT|SIF_FRAME_START_INTERRUPT|MOT_DET)
#define IPU_INT_BITS    (IPU_FRAME_DONE|IPU_FRAME_START|IPU_BUS23_TRANSMIT_ERRORS|IPU_BUS01_TRANSMIT_ERRORS|PYM_FRAME_START|PYM_FRAME_DONE|PYM_DS_FRAME_DROP|PYM_US_FRAME_DROP)


#define IPBUSCFG            0x8

#define IPSBUSCTL_WM0       0xc
#define IPSBUSCTL_WM1       0x10
#define IPSBUSCTL_WM2       0x14
#define IPSBUSCTL_WM3       0x18
#define IPSBUSCTL_WM4       0x1C
#define IPSBUSCTL_WM5       0x20
#define IPSBUSCTL_WM6       0x24
#define IPSBUSCTL_WM7       0x28
#define IPSBUSCTL_WM8       0x2C
#define IPSBUSCTL_WM9       0x30
#define IPSBUSCTL_WM10      0x34
#define IPSBUSCTL_WM11      0x38
#define IPSBUSCTL_WM12      0x3C
#define IPSBUSCTL_WM13      0x40
#define IPSBUSCTL_WM14      0x44
#define IPSBUSCTL_WM15      0x48


#define BUSCTL_MAXLEN_X     (0xff << 8)
#define BUSCTL_ENDIAN_X     (0xf << 4)
#define BUSCTL_PRI_X        (0xf << 0)

#define BUSCTL_MAXLEN_SHIFT_X       8
#define BUSCTL_ENDIAN_SHIFT_X       4
#define BUSCTLD_PRI_SHIFT_X         0

#define IPSBUSCTL_RM0       0x4C
#define IPSBUSCTL_RM1       0x50
#define IPSBUSCTL_RM2       0x54
#define IPSBUSCTL_RM3       0x58
#define IPSBUSCTL_RM4       0x5C
#define IPSBUSCTL_RM5       0x60
#define IPSBUSCTL_RM6       0x64
#define IPSBUSCTL_RM7       0x68
#define IPSBUSCTL_RM8       0x6C

#define IPS_MIPI_CTRL           0XF0
#define MIPI_BPASS_GEN_DLY      (0x3f << 4)
#define MIPI_BPASS_GEN_HSYNC    (1 << 1)
#define MIPI_DEV_SHADOW_CLR     (1 << 0)

#define IPS_MIPI_FREQRANGE      0xF4
#define MIPI_DEV_CFGCLK_FRANGE  (0xff << 24)
#define MIPI_DEV_HS_FRANGE      (0x7f << 16)
#define MIPI_HOST_CFGCLK_FRANGE (0xff << 8)
#define MIPI_HOST_HS_FRANGE     (0x3f << 0)

#define IPS_STATUS      0xF8
#define STATUS_PYM      (1 << 3)
#define STATUS_IPU      (1 << 2)
#define STATUS_IPS      (1 << 1)
#define STATUS_SIF      (1 << 0)

#define IPS_CTL     0xFC
#define MIPI_DEV_CFG_CLK    (1 << 9)
#define MIPI_HOST_CFG_CLK   (1 << 8)
#define MIPI_DEV_IPI_CLK    (1 << 7)
#define MIPI_HOST_IPI_CLK   (1 << 6)
#define PDM_SRAM_CLK        (1 << 5)
#define TILE_SRAM_CLK       (1 << 4)
#define MOT_SRAM_CLK        (1 << 3)
#define RCCB_SRAM_CLK       (1 << 2)
#define IPU_SRAM_CLK        (1 << 1)
#define IPS_SRAM_CLK        (1 << 0)

#define VIOSYS_CLK_CTRL 0x310

enum busctl_region_e
{
    BUSCTL_REGION_PRI,
    BUSCTL_REGION_ENDIAN,
    BUSCTL_REGION_MAXLEN,
};
enum busctl_type_e
{
    BUSCTL_WM,
    BUSCTL_RM,
};
enum ips_ctl_region_e
{
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
enum ips_status_region_e
{
    PYM_STATUS,
    IPU_STATUS,
    ISP_STATUS,
    SIF_STATUS,
};
enum ips_mipi_freqrange_region_e
{
    MIPI_DEV_CFGCLKFREQRANGE,
    MIPI_DEV_HSFREQRANGE,
    MIPI_HOST_CFGCLKFREQRANGE,
    MIPI_HOST_HSFREQRANGE,
};
enum ips_mipictl_region_e
{
    MIPI_BYPASS_GEN_HSYNC_DLY_CNT,
    MIPI_BYPASS_GEN_HSYNC_EN,
    MIPI_DEV_SHADOW_CLEAR,
};
enum
{
    SIF_INT,
    ISP_INT,
    IPU_INT,
};
enum
{
    IAR_CLK,
    BYPASS_CLK,
};
enum
{
    RST_MIPI_IPI,
    RST_MIPI_CFG,
    RST_SIF,
    RST_IPU,
    RST_DVP,
    RST_BT,
    RST_MAX,
};
typedef void (*ips_irqhandler_t)(unsigned int intstatus, void* data);
int ips_irq_enable(int irq);
int ips_irq_disable(int irq);
int ips_mask_int(unsigned int mask);
int ips_unmask_int(unsigned int mask);
int ips_register_irqhandle(int irq, ips_irqhandler_t handle, void* data);
unsigned int ips_get_intstatus(void);

int ips_mipi_ctl_set(unsigned int region, unsigned int value);
int ips_busctl_set(unsigned int type, unsigned int index, unsigned int region, unsigned int value);
int ips_busctl_get(unsigned int type, unsigned int index, unsigned int region);
int ips_mipi_ctl_set(unsigned int region, unsigned int value);
int ips_mipi_ctl_get(unsigned int region);
int ips_control_set(unsigned int region, unsigned int state);
int ips_control_get(unsigned int region);
int ips_get_status(unsigned int region);
int ips_get_mipi_freqrange(unsigned int region);
int ips_pinmux_bt(void);
int ips_pinmux_dvp(void);
int ips_set_btout_clksrc(unsigned int mode);
void ips_module_reset(unsigned int module);

#endif /* __X2_IPS_H */
