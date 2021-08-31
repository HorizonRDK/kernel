/***************************************************************************
 *   Copyright (C) 2021 by horizon.                                        *
 *   dinggao.pan@horizon.ai                                                *
 *                                                                         *
 *   Diag inject test file.                                                *
 *                                                                         *
 ***************************************************************************/
#ifndef DRIVERS_SOC_HOBOT_DIAG_DIAG_INJECT_H_
#define DRIVERS_SOC_HOBOT_DIAG_DIAG_INJECT_H_
#include <linux/ioctl.h>
#include <soc/hobot/diag.h>

#define DIAG_SW_DRV_BASE	(0U)
#define DIAG_SW_I2C_BASE	(DIAG_SW_DRV_BASE + EventIdDrvMax - 1U)
#define DIAG_SW_VIO_BASE	(DIAG_SW_I2C_BASE + EventIdI2cMax - 1U)
#define DIAG_SW_BPU_BASE	(DIAG_SW_VIO_BASE + EventIdVioMax - 1U)
#define DIAG_SW_I2S_BASE	(DIAG_SW_BPU_BASE + EventIdBpuMax - 1U)
#define DIAG_SW_BIF_BASE	(DIAG_SW_I2S_BASE + EventIdI2sMax - 1U)
#define DIAG_SW_ETH_BASE	(DIAG_SW_BIF_BASE + EventIdBifMax - 1U)
#define DIAG_SW_SPI_BASE	(DIAG_SW_ETH_BASE + EventIdEthMax - 1U)
#define DIAG_SW_EMMC_BASE	(DIAG_SW_SPI_BASE + EventIdSpiMax - 1U)
#define DIAG_SW_QSPI_BASE	(DIAG_SW_EMMC_BASE + EventIdEmmcMax - 1U)
#define DIAG_SW_ALU_BASE	(DIAG_SW_QSPI_BASE + EventIdqspiMax - 1U)
#define DIAG_SW_MPU_BASE	(DIAG_SW_ALU_BASE + EventIdAluMax - 1U)
#define DIAG_SW_UART_BASE	(DIAG_SW_MPU_BASE + EventIdMpuMax - 1U)
#define DIAG_SW_MAX			(DIAG_SW_UART_BASE + EventIdUartMax - 1U)

#define DIAG_SW_DRV_CEIL	(DIAG_SW_I2C_BASE - 1U)
#define DIAG_SW_I2C_CEIL	(DIAG_SW_VIO_BASE - 1U)
#define DIAG_SW_VIO_CEIL	(DIAG_SW_BPU_BASE - 1U)
#define DIAG_SW_BPU_CEIL	(DIAG_SW_I2S_BASE - 1U)
#define DIAG_SW_I2S_CEIL	(DIAG_SW_BIF_BASE - 1U)
#define DIAG_SW_BIF_CEIL	(DIAG_SW_ETH_BASE - 1U)
#define DIAG_SW_ETH_CEIL	(DIAG_SW_SPI_BASE - 1U)
#define DIAG_SW_SPI_CEIL	(DIAG_SW_EMMC_BASE - 1U)
#define DIAG_SW_EMMC_CEIL	(DIAG_SW_QSPI_BASE - 1U)
#define DIAG_SW_QSPI_CEIL	(DIAG_SW_ALU_BASE - 1U)
#define DIAG_SW_ALU_CEIL	(DIAG_SW_MPU_BASE - 1U)
#define DIAG_SW_MPU_CEIL	(DIAG_SW_UART_BASE - 1U)
#define DIAG_SW_UART_CEIL	(DIAG_SW_MAX - 1U)

#define DIAG_SW_RANGE(mod)	\
{ \
	.base = DIAG_SW_##mod##_BASE, \
	.ceil = DIAG_SW_##mod##_CEIL, \
}

typedef struct {
    uint16_t module_id;
    uint16_t event_id;
    uint32_t inject_val;
} diag_inject_data_t;
#define INJECT_EN _IO('q', 0x1)
#define INJECT_VAL_SET _IOW('q', 0x2, diag_inject_data_t *)
#define INJECT_VAL_CHK _IOR('q', 0x3, diag_inject_data_t *)
#define INJECT_EN_CHK _IOR('q', 0x4, diag_inject_data_t *)
#define INJECT_VAL_CLR _IOR('q', 0x5, diag_inject_data_t *)

#endif  // DRIVERS_SOC_HOBOT_DIAG_DIAG_INJECT_H_
