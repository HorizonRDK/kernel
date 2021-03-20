/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __HOBOT_DDR_MPU_H__
#define __HOBOT_DDR_MPU_H__


enum FW_USR_ID {
    FW_USR_ID_CPU,              //0
    FW_USR_ID_CR5,
    FW_USR_ID_BIFSPI,
    FW_USR_ID_BIFSD,
    FW_USR_ID_DMAC,
    FW_USR_ID_USB,              //5
    FW_USR_ID_AES,
    FW_USR_ID_CNN0_Fetch = 0xc, //12
    FW_USR_ID_CNN1_Fetch,
    FW_USR_ID_CNN0_Other,
    FW_USR_ID_CNN1_Other,
    FW_USR_ID_UART0,
    FW_USR_ID_UART1,            //17
    FW_USR_ID_UART2,
    FW_USR_ID_UART3,
    FW_USR_ID_SPI0,
    FW_USR_ID_SPI1,
    FW_USR_ID_SPI2,             //22
    FW_USR_ID_I2S0,
    FW_USR_ID_I2S1,
    FW_USR_ID_SDIO0,
    FW_USR_ID_SDIO1,
    FW_USR_ID_SDIO2,
    FW_USR_ID_GMAC,             //28
    FW_USR_ID_VIO_M0,
    FW_USR_ID_VPU_JPG,
    FW_USR_ID_VIO_M1,           //31
    FW_USR_ID_FW_USR_ID_MAX
};

enum SLV_DIS_ID {
    SLV_DIS_ID_DDR,
    SLV_DIS_ID_SRAM,
    SLV_DIS_ID_VIOLT_IRAM,
    SLV_DIS_ID_QSPI,
    SLV_DIS_ID_DBG_SYS,
    SLV_DIS_ID_CPU_SYS,
    SLV_DIS_ID_DDR_SYS,
    SLV_DIS_ID_CNN_SYS,
    SLV_DIS_ID_VIOLT_SYS,
    SLV_DIS_ID_PERI_SYS,
    SLV_DIS_ID_PMU_SYS,
    SLV_DIS_ID_SEC_REG,
    SLV_DIS_ID_VSP_SYS,
    SLV_DIS_ID_USB,
    SLV_DIS_ID_MAX
};


/* sec reg MPU base 0xA6008200*/
#define MPU_BPU0_FETCH_S_RANGE        0x00
#define MPU_BPU0_FETCH_E_RANGE        0x04
#define MPU_BPU1_FETCH_S_RANGE        0x08
#define MPU_BPU1_FETCH_E_RANGE        0x0c
#define MPU_DRAM_BPU_S_RANGE          0x10
#define MPU_DRAM_BPU_E_RANGE          0x14
#define MPU_DRAM_MAX_S_RANGE          0x18
#define MPU_DRAM_MAX_E_RANGE          0x1c
#define MPU_DRAM_BPU_MAX_S_RANGE      0x20
#define MPU_DRAM_BPU_MAX_E_RANGE      0x24
#define MPU_BPU0_FETCH_USER           0x40
#define MPU_BPU1_FETCH_USER           0x44
#define MPU_DRAM_BPU_USER             0x48
#define MPU_DRAM_MAX_USER             0x4c
#define MPU_DRAM_BPU_MAX_USER         0x50
#define MPU_S_RANGE0                  0x100
#define MPU_E_RANGE0                  0x104
#define MPU_S_RANGE1                  0x108
#define MPU_E_RANGE1                  0x10c
#define MPU_S_RANGE2                  0x110
#define MPU_E_RANGE2                  0x114
#define MPU_S_RANGE3                  0x118
#define MPU_E_RANGE3                  0x11c
#define MPU_RANGE0_RUSER              0x120
#define MPU_RANGE1_RUSER              0x124
#define MPU_RANGE2_RUSER              0x128
#define MPU_RANGE3_RUSER              0x12c
#define MPU_RANGE0_WUSER              0x130
#define MPU_RANGE1_WUSER              0x134
#define MPU_RANGE2_WUSER              0x138
#define MPU_RANGE3_WUSER              0x13c
#define MPU_DEFAULT_ADDR              0x180

/* sec reg base 0xA6008200*/
#define RPU_BIFSPI_SLV_DIS            0x200
#define RPU_BIFSD_SLV_DIS             0x204
#define RPU_DEFAULT_ADDR              0x280

/* Sysctrl Base 0xA1000900 */
#define SYSCTL_LOCKDOWN_SECURE        0x0
#define SYSCTL_MSK_AES_KEY            0x4
#define SYSCTL_MSK_ROM                0x8
#define SYSCTL_DBG_PORT_EN_LOCK       0xc
#define SYSCTL_DBG_PORT_EN            0x10
#define SYSCTL_S_EFS_LOCK             0x14


#define FW_IRQ_STA             0x40
#define FW_IRQ_CLR             0x44

#define RPU_BIFSPI_VIOLT_SLV     0x50
#define RPU_BIFSD_VIOLT_SLV      0x54

#define MPU_CNN0_VIOLT_USER      0x80
#define MPU_CNN0_VIOLT_ADDR      0x84
#define MPU_CNN1_VIOLT_USER      0x88
#define MPU_CNN1_VIOLT_ADDR      0x8c
#define MPU_CPU_VIOLT_USER       0x90
#define MPU_CPU_VIOLT_ADDR       0x94
#define MPU_R5_VIOLT_USER        0x98
#define MPU_R5_VIOLT_ADDR        0x9c
#define MPU_CPUSS_VIOLT_USER     0xa0
#define MPU_CPUSS_VIOLT_ADDR     0xa4
#define MPU_VIOSS_VIOLT_USER     0xa8
#define MPU_VIOSS_VIOLT_ADDR     0xac
#define MPU_VSPSS_VIOLT_USER     0xb0
#define MPU_VSPSS_VIOLT_ADDR     0xb4
#define MPU_VSPVIOSS_VIOLT_USER  0xb8
#define MPU_VSPVIOSS_VIOLT_ADDR  0xbc
#define MPU_PERISS_VIOLT_USER    0xc0
#define MPU_PERISS_VIOLT_ADDR    0xc4

#define FW_PORT_ID_RPU_BSPI        (1<<0)
#define FW_PORT_ID_RPU_BSD         (1<<1)
#define FW_PORT_ID_RPU_CPU         (1<<2)
#define FW_PORT_ID_MPU_CNN0        (1<<8)
#define FW_PORT_ID_MPU_CNN1        (1<<9)
#define FW_PORT_ID_MPU_CPU         (1<<10)
#define FW_PORT_ID_MPU_R5          (1<<11)
#define FW_PORT_ID_MPU_CPUSS       (1<<12)
#define FW_PORT_ID_MPU_VIOSS       (1<<13)
#define FW_PORT_ID_MPU_VSPSS       (1<<14)
#define FW_PORT_ID_MPU_VSPVIO      (1<<15)
#define FW_PORT_ID_MPU_PERI        (1<<16)
#define FW_PORT_ID_NUM             17

#define BPU0_FETCH_RANGE_VIOLT       (1<<0)
#define BPU1_FETCH_RANGE_VIOLT       (1<<1)
#define DRAM_BPU_RANGE_VIOLT         (1<<2)
#define DRAM_MAX_RANGE_VIOLT         (1<<3)
#define MPU_ADDR0_RANGE_VIOLT        (1<<4)
#define MPU_ADDR1_RANGE_VIOLT        (1<<5)
#define MPU_ADDR2_RANGE_VIOLT        (1<<6)
#define MPU_ADDR3_RANGE_VIOLT        (1<<7)
#define MPU_BLOCK_NUM               8

#define EFUSE_NS_OFF               0x0
#define NS_EFS_RDONE               0x80
#define EFUSE_S_OFF                0x100
#define S_EFS_RDONE                0x180

#define NS_EFUSE_NUM               32
#define S_EFUSE_NUM                32

#define FW_PORT_MPU_Reserved       0xffffffff

struct mpu_protection {
	int irq;
	int enabled;
	void __iomem *sysctrl;
	void __iomem *secreg;
	struct workqueue_struct *wq;
};

extern int ddr_mpu_init(struct platform_device *pdev);
extern int ddr_mpu_deinit(struct platform_device *pdev);

#endif /* __HOBOT_DDR_MPU_H__ */
