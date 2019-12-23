#ifndef __HOBOT_BIFSPI_H__
#define __HOBOT_BIFSPI_H__


/* modules status */
#define	BIF_SPI_BIT	(0x1L<<0)
#define	BIF_SD_BIT	(0x1L<<1)
#define	BIF_ETH_BIT	(0x1L<<2)

/* system status */
#define	STAGE_SPL		0xFED00000
#define	STAGE_UBOOT		0xFED10000
#define STAGE_KERNEL		0xFED20000
#define	STAGE_PANIC		0xFED30000

/* uboot panic run */
#define SYS_STATUS_REG		0

int bifspi_read_share_reg(unsigned int num, unsigned int *value);
int bifspi_write_share_reg(unsigned int num, unsigned int value);
#endif
