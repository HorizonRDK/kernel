/************************************************************
 ****			 COPYRIGHT NOTICE			 ****
 ****		 Copyright	2018 Horizon Robotics, Inc.		 ****
 ****			 All rights reserved.			 ****
 ************************************************************/
/**
 * BIF API for BIF driver
 * @author		haibo.guo(haibo.guo@horizon.ai)
 * @date		2018/12/26
 */

#include "bif_base.h"
#include "bif_api.h"

/*
 *AP side export symbol functions
 *@addr: phy addr
 *@count: bytes, 512 times, eg. 512,1024,1536...
 *@return: 0, ok; other ,err
 *int bifsd_read(void *addr, unsigned int count, unsigned char *buf);
 *int bifsd_write(void *addr, unsigned int count, unsigned char *buf);

 *@addr: phy addr
 *@count: bytes
 *@return: 0, ok; other ,err
 *int bifspi_read(void *addr, unsigned int count, unsigned char *buf);
 *int bifspi_write(void *addr, unsigned int count, unsigned char *buf);
 */

#ifdef CONFIG_HOBOT_BIF_AP

#ifdef CONFIG_HOBOT_BIFSD
int bif_sd_read(void *addr, unsigned int count, unsigned char *buf)
{
#ifdef CONFIG_HOBOT_BIF_TEST
	return t_bif_sd_read(addr, count, buf);
#else
	return bifsd_read(addr, count, buf);
#endif
}
EXPORT_SYMBOL(bif_sd_read);

int bif_sd_write(void *addr, unsigned int count, unsigned char *buf)
{
#ifdef CONFIG_HOBOT_BIF_TEST
	return t_bif_sd_write(addr, count, buf);
#else
	return bifsd_write(addr, count, buf);
#endif
}
EXPORT_SYMBOL(bif_sd_write);

#else
#ifdef CONFIG_HOBOT_BIFSPI
#if 1
int bifspi_read(void *addr, unsigned int count, unsigned char *buf)
{
	return bifdev_get_cpchip_ddr((uint32_t)addr, (uint16_t)count,
		(uint8_t *)buf);
}
int bifspi_write(void *addr, unsigned int count, unsigned char *buf)
{
	return bifdev_set_cpchip_ddr((uint32_t)addr, (uint16_t)count,
		(uint8_t *)buf);
}
#endif
int bif_spi_read(void *addr, unsigned int count, unsigned char *buf)
{
#ifdef CONFIG_HOBOT_BIF_TEST
	return t_bif_spi_read(addr, count, buf);
#else
	return bifspi_read(addr, count, buf);
#endif
}
EXPORT_SYMBOL(bif_spi_read);
int bif_spi_write(void *addr, unsigned int count, unsigned char *buf)
{
#ifdef CONFIG_HOBOT_BIF_TEST
	return t_bif_spi_write(addr, count, buf);
#else
	return bifspi_write(addr, count, buf);
#endif
}
EXPORT_SYMBOL(bif_spi_write);
#endif
#endif

#endif
