/*************************************************************
 ****			 COPYRIGHT NOTICE
 ****		 Copyright	2019 Horizon Robotics, Inc.
 ****			 All rights reserved.
 *************************************************************/
/**
 * hardware and api driver, for platform and bifbase
 * @version	2.0
 * @author	haibo.guo(haibo.guo@horizon.ai)
 * @date	2019/04/01
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/interrupt.h>

#ifdef CONFIG_HISI
#include <mach/io.h>
#include "hisi_reg.h"
#endif

#include "bif_base.h"
#include "bif_api.h"

int bifdebug;	//pr_bif
EXPORT_SYMBOL(bifdebug);
#define pr_bif(fmt, args...) do {if (bifdebug) pr_info(fmt, ##args); } while (0)
module_param(bifdebug, int, 0644);
int bifbase_irq_pin;
module_param(bifbase_irq_pin, int, 0644);
int bifbase_tri_pin;
module_param(bifbase_tri_pin, int, 0644);

#define CPSIDE_DDR_ADDR		(0x02000000)	//same to x2 reserved memory
#define CPSIDE_DDR_ADDRSIZE	(0x00100000)

#define WRITE_REG(Addr, Value) ((*(uint *)(Addr)) = (Value))
#define READ_REG(Addr) (*(uint *)(Addr))
#ifdef CONFIG_HISI
#define MUX_CTL_BASE_ADDR	(0x12040000)
#define GPIO_MAP_RANGE		(0x4000)
#define BIFTRI_PIN		(2*8 + 5)	//GPIO2_5
#define BIFIRQ_PIN		(2*8 + 3)	//GPIO2_3
void __iomem *muxctrl_base_va;
#endif

#ifdef CONFIG_HOBOT_BIF_TEST
#ifdef CPSIDE_DDR_ADDR
#undef CPSIDE_DDR_ADDR
#define CPSIDE_DDR_ADDR		(0x7ff00000) //vmware reversed address
#endif
extern int t_bif_netlink_init(void);
extern void t_bif_netlink_exit(void);
/*extern int t_bif_register_address(BUFF_ID buffer_id, void *address);*/
extern int t_bif_register_irq(enum BUFF_ID buffer_id,
	irq_handler_t irq_handler);
extern int t_bif_send_irq(int irq);
/*extern void *t_bif_query_address(BUFF_ID buffer_id);*/
/*extern void *t_bif_query_address_wait(BUFF_ID buffer_id);*/
extern int t_bif_sd_read(void *addr, unsigned int count, char *buf);
extern int t_bif_sd_write(void *addr, unsigned int count, char *buf);
extern int t_bif_spi_read(void *addr, unsigned int count, char *buf);
extern int t_bif_spi_write(void *addr, unsigned int count, char *buf);
#endif

#if defined(CONFIG_HOBOT_BIFSD) || \
	defined(CONFIG_HOBOT_BIFSD_MODULE)
extern int bifsd_read(void *addr, unsigned int count, char *buf);
extern int bifsd_write(void *addr, unsigned int count, char *buf);
#else
static int bifsd_read(void *addr, unsigned int count, char *buf)
{
#if defined(CONFIG_HOBOT_BIF_AP) || \
	defined(CONFIG_HOBOT_BIF_AP_MODULE)
	return -BIFNOSD;
#else
	return BIFOK;
#endif
}
static int bifsd_write(void *addr, unsigned int count, char *buf)
{
#if defined(CONFIG_HOBOT_BIF_AP) || \
	defined(CONFIG_HOBOT_BIF_AP_MODULE)
	return -BIFNOSD;
#else
	return BIFOK;
#endif
}
#endif

#if defined(CONFIG_HOBOT_BIFSPI) || \
	defined(CONFIG_HOBOT_BIFSPI_MODULE)
extern int
bifdev_get_cpchip_ddr(uint32_t addr, uint16_t size, uint8_t *value);
extern int
bifdev_set_cpchip_ddr(uint32_t addr, uint16_t size, uint8_t *value);
#else
static int
bifdev_get_cpchip_ddr(uint32_t addr, uint16_t size, uint8_t *value)
{
#if defined(CONFIG_HOBOT_BIF_AP) || \
	defined(CONFIG_HOBOT_BIF_AP_MODULE)
	return -BIFNOSPI;
#else
	return BIFOK;
#endif
}
static int
bifdev_set_cpchip_ddr(uint32_t addr, uint16_t size, uint8_t *value)
{
#if defined(CONFIG_HOBOT_BIF_AP) || \
	defined(CONFIG_HOBOT_BIF_AP_MODULE)
	return -BIFNOSPI;
#else
	return BIFOK;
#endif
}
#endif

int bif_sd_read(void *addr, unsigned int count, unsigned char *buf)
{
	unsigned int len = MULTI(count, BIFSD_BLOCK);

	pr_bif("bifapi: %s()-%d %p,%d\n", __func__, __LINE__, addr, len);
	if (addr == NULL || buf == NULL)
		return -BIFERR;

#ifdef CONFIG_HOBOT_BIF_TEST
	return t_bif_sd_read(addr, len, buf);
#else
	return bifsd_read(addr, len, buf);
#endif
}
EXPORT_SYMBOL(bif_sd_read);

int bif_sd_write(void *addr, unsigned int count, unsigned char *buf)
{
	unsigned int len = MULTI(count, BIFSD_BLOCK);

	pr_bif("bifapi: %s()-%d %p,%d\n", __func__, __LINE__, addr, len);
	if (addr == NULL || buf == NULL)
		return -BIFERR;

#ifdef CONFIG_HOBOT_BIF_TEST
	return t_bif_sd_write(addr, len, buf);
#else
	return bifsd_write(addr, len, buf);
#endif
}
EXPORT_SYMBOL(bif_sd_write);

int bif_spi_read(void *addr, unsigned int count, unsigned char *buf)
{
	unsigned short len = MULTI(count, BIFSPI_BLOCK);

	pr_bif("bifapi: %s()-%d %p,%d\n", __func__, __LINE__, addr, len);
	if (addr == NULL || buf == NULL)
		return -BIFERR;

#ifdef CONFIG_HOBOT_BIF_TEST
	return t_bif_spi_read(addr, len, buf);
#else
	return bifdev_get_cpchip_ddr((uint32_t)addr, len, (uint8_t *)buf);
#endif
}
EXPORT_SYMBOL(bif_spi_read);

int bif_spi_write(void *addr, unsigned int count, unsigned char *buf)
{
	unsigned short len = MULTI(count, BIFSPI_BLOCK);

	pr_bif("bifapi: %s()-%d %p,%d\n", __func__, __LINE__, addr, len);
	if (addr == NULL || buf == NULL)
		return -BIFERR;

#ifdef CONFIG_HOBOT_BIF_TEST
	return t_bif_spi_write(addr, len, buf);
#else
	return bifdev_set_cpchip_ddr((uint32_t)addr, len, (uint8_t *)buf);
#endif
}
EXPORT_SYMBOL(bif_spi_write);

int bifread(int channel, void *addr, unsigned int count, unsigned char *buf)
{
//	pr_bif("bifapi: %s()-%d %d,%p,%d\n",
//		__func__, __LINE__, channel, addr, count);
	if (channel == BIFBUS_SPI)
		return bif_spi_read(addr, count, buf);
	else if (channel == BIFBUS_SD)
		return bif_sd_read(addr, count, buf);
	else
		return -BIFERR;
}
EXPORT_SYMBOL(bifread);

int bifwrite(int channel, void *addr, uint count, unchar *buf)
{
//	pr_bif("bifapi: %s()-%d %d,%p,%d\n",
//		__func__, __LINE__, channel, addr, count);
	if (channel == BIFBUS_SPI)
		return bif_spi_write(addr, count, buf);
	else if (channel == BIFBUS_SD)
		return bif_sd_write(addr, count, buf);
	else
		return -BIFERR;
}
EXPORT_SYMBOL(bifwrite);

int bifget_supportbus(char *str_bus)
{
//	pr_bif("bifapi: %s()-%d %s\n", __func__, __LINE__, str_bus);

	if (!str_bus)
		return SUPPORT_MAX;

	if (strncmp(str_bus, STR_SUPPORT_NO, strlen(STR_SUPPORT_NO)) == 0)
		return SUPPORT_NO;
	else if (strncmp(str_bus, STR_SUPPORT_YES,
		strlen(STR_SUPPORT_YES)) == 0)
		return SUPPORT_YES;
	else
		return SUPPORT_MAX;
}
EXPORT_SYMBOL(bifget_supportbus);

int bifget_bifbustype(char *str_bustype)
{
//	pr_bif("bifapi: %s()-%d %s\n", __func__, __LINE__, str_bustype);

	if (!str_bustype)
		return BIFBUS_MAX;

	if (strncmp(str_bustype, STR_BIFBUS_NO, strlen(STR_BIFBUS_NO)) == 0)
		return BIFBUS_NO;
	else if (strncmp(str_bustype, STR_BIFBUS_SPI,
		strlen(STR_BIFBUS_SPI)) == 0)
		return BIFBUS_SPI;
	else if (strncmp(str_bustype, STR_BIFBUS_SD,
		strlen(STR_BIFBUS_SD)) == 0)
		return BIFBUS_SD;
	else
		return BIFBUS_MAX;
}
EXPORT_SYMBOL(bifget_bifbustype);

void bifplat_get_macro_config(void *p)
{
	struct bifplat_info *pl = (struct bifplat_info *)p;

	if (!pl)
		return;

	pl->kernel_ver = LINUX_VERSION_CODE;
	memset(pl->platform, 0, PLATFORM_SIZE);
#if defined(CONFIG_HOBOT_BIF_AP) || \
	defined(CONFIG_HOBOT_BIF_AP_MODULE)
	//pr_info("bifapi: CONFIG_HOBOT_BIF_AP\n");
	sprintf(pl->platform, "%s", "x2j2 AP");
	pl->plat_type = PLAT_AP;
	pl->param = PARAM_MODULE;
#else
	sprintf(pl->platform, "%s", "x2j2 CP");
	pl->plat_type = PLAT_CP;
	pl->param = PARAM_DTS;
#endif

#if defined(CONFIG_HOBOT_BIFSPI) || \
	defined(CONFIG_HOBOT_BIFSPI_MODULE)
	//pr_info("bifapi: CONFIG_HOBOT_BIFSPI\n");
	pl->bifspi = SUPPORT_YES;
#else
	pl->bifspi = SUPPORT_NO;
#endif

#if defined(CONFIG_HOBOT_BIFSD) || \
	defined(CONFIG_HOBOT_BIFSD_MODULE)
	//pr_info("bifapi: CONFIG_HOBOT_BIFSD\n");
	pl->bifsd = SUPPORT_YES;
#else
	pl->bifsd = SUPPORT_NO;
#endif

	pl->bifbase_phyaddr = CPSIDE_DDR_ADDR;
	pl->bifbase_phyaddrsize = CPSIDE_DDR_ADDRSIZE;
	pl->irq_pin_absent = 0;
	if (bifbase_irq_pin)
		pl->irq_pin = bifbase_irq_pin;
	else
		pl->irq_pin = -1;
	if (bifbase_tri_pin)
		pl->tri_pin = bifbase_tri_pin;
	else
		pl->tri_pin = -1;
	pl->tri_val = 0;
	pl->rev_pin1 = -1;
	pl->rev_pin2 = -1;
}

void bifplat_print_info(void *p)
{
	struct bifplat_info *pl = (struct bifplat_info *)p;

	if (!pl)
		return;

	pr_info("==BIF Application platform information\n");
	pr_info("platform: %s\n", pl->platform);
	pr_info("kernel version: %x\n", pl->kernel_ver);
	pr_info("plat type: %s\n", (pl->plat_type == PLAT_CP) ? "CP" : "AP");
	pr_info("plat param: %s\n",
		(pl->param == PARAM_DTS) ? "dts" : "module param");
	pr_info("support bifspi: %s\n",
		(pl->bifspi == SUPPORT_NO) ? "no" : "yes");
	pr_info("support bifsd: %s\n",
		(pl->bifsd == SUPPORT_NO) ? "no" : "yes");
	pr_info("cp side ddr phyaddr: 0x%lx\n", pl->bifbase_phyaddr);
	pr_info("cp side ddr phyaddrsize: 0x%x\n", pl->bifbase_phyaddrsize);
	pr_info("irq pin absent: %s\n", pl->irq_pin_absent > 0 ? "yes" : "no");
	pr_info("irq pin: %d\n", pl->irq_pin);
	pr_info("irq num: %d\n", pl->irq_num);
	pr_info("tri pin: %d\n", pl->tri_pin);
//	pr_info("tri val: %d\n", pl->tri_val);
	pr_info("rev pin1: %d\n", pl->rev_pin1);
	pr_info("rev pin2: %d\n", pl->rev_pin2);
}
EXPORT_SYMBOL(bifplat_print_info);

void bifplat_trigger_irq(void *p)
{
	struct bifplat_info *pl = (struct bifplat_info *)p;

	if (!pl)
		return;

#ifdef CONFIG_HOBOT_BIF_TEST
	t_bif_send_irq(BUFF_BASE);
#else
	if (pl->irq_pin_absent)
		return;

	pr_bif("bifapi: tri_pin: %d, tri_val: %d\n", pl->tri_pin, pl->tri_val);
	if (pl->tri_val)
		pl->tri_val = 0;
	else
		pl->tri_val = 1;
	if (pl->tri_pin)
		gpio_direction_output(pl->tri_pin, pl->tri_val);
	else
		pr_err("bifapi: Err tri_pin: %d, tri_val: %d\n",
			pl->tri_pin, pl->tri_val);

#endif
}
EXPORT_SYMBOL(bifplat_trigger_irq);

int bifplat_gpio_init(void *p)
{
	int ret;
	struct bifplat_info *pl = (struct bifplat_info *)p;

	if (!pl) {
		ret = -BIFERR;
		goto exit_1;
	}

	//bifplat_print_info((void *)pl);

#ifdef CONFIG_HOBOT_BIF_TEST
	ret = BIFOK;
#else
	if (pl->irq_pin_absent)
		ret = BIFOK;
	else {
		if (pl->tri_pin < 0) {
			ret = -EIO;
			pr_err("bifapi: Err trigger pin %d\n", pl->tri_pin);
			goto exit_1;
		}
		ret = gpio_request(pl->tri_pin, "tri_pin");
		if (ret < 0) {
			pr_err("bifapi: Err get trigger pin ret = %d\n", ret);
			goto exit_1;
		}
		gpio_direction_output(pl->tri_pin, pl->tri_val);
		bifplat_trigger_irq((void *)pl);

		if (pl->irq_pin < 0) {
			ret = -EIO;
			pr_err("bifapi: Err irq pin %d\n", pl->irq_pin);
			goto exit_1;
		}
		ret = gpio_request(pl->irq_pin, "irq_pin");
		if (ret < 0) {
			pr_err("bifapi: Err get irq pin ret = %d\n", ret);
			gpio_free(pl->tri_pin);
			goto exit_1;
		}
		pl->irq_num = gpio_to_irq(pl->irq_pin);
		if (pl->irq_num < 0) {
			pr_err("bifapi: Err get irq num = %d\n", pl->irq_num);
			gpio_free(pl->tri_pin);
			gpio_free(pl->irq_pin);
			ret = -ENODEV;
		}
	}
#endif
exit_1:
	return ret;
}
EXPORT_SYMBOL(bifplat_gpio_init);

void bifplat_gpio_deinit(void *p)
{
	struct bifplat_info *pl = (struct bifplat_info *)p;

	if (!pl)
		return;

	pr_bif("bifapi: gpio deinit begin...\n");

#ifdef CONFIG_HOBOT_BIF_TEST

#else
	if (!pl->irq_pin_absent) {
		gpio_free(pl->tri_pin);
		gpio_free(pl->irq_pin);
	}
#endif
}
EXPORT_SYMBOL(bifplat_gpio_deinit);

int bifplat_register_irq(enum BUFF_ID buffer_id,
	irq_handler_t irq_handler)
{
#ifdef CONFIG_HOBOT_BIF_TEST
	t_bif_register_irq(buffer_id, irq_handler);
	return buffer_id;
#else
	return -BIFERR;
#endif
}
EXPORT_SYMBOL(bifplat_register_irq);
#ifdef CONFIG_HISI
static int  hisi_gpio_pin_mux(void __iomem *muxctrl_base, int gpiopin)
{
	unsigned int offset = 0;
	void __iomem *reg = NULL;
	unsigned char gpio_grp = 0;
	unsigned char gpio_pin = 0;
	int ret = 0;

	gpio_grp = gpiopin / 8;
	gpio_pin = gpiopin % 8;
	reg = muxctrl_base;

	if ((gpio_grp < 17) && (gpio_grp >= 0)
		&& (gpio_pin < 8) && (gpio_pin >= 0)) {

		if (!g_gpio_mux_ctl_addr[gpio_grp][gpio_pin]) {
			ret = -1;
			goto exit_1;
		}
		offset = g_gpio_mux_ctl_addr[gpio_grp][gpio_pin] & 0xFFFF;
		reg = (void __iomem *)(muxctrl_base + offset);

		switch (gpio_grp) {
		case 0:
			if (gpio_pin == 1)
				WRITE_REG(reg, 0x1);
			else
				WRITE_REG(reg, 0x1);
			break;
		case 3:
			if (gpio_pin < 6)
				WRITE_REG(reg, 0);
			else
				WRITE_REG(reg, 0x1);
			break;
		case 4:
			if (gpio_pin < 2)
				WRITE_REG(reg, 0x1);
			else
				WRITE_REG(reg, 0);
			break;
		case 6:
			if (gpio_pin == 5)
				WRITE_REG(reg, 2);
			else
				WRITE_REG(reg, 0);
			break;
		case 12:
			if (gpio_pin == 1)
				WRITE_REG(reg, 0x1);
			else
				WRITE_REG(reg, 0);
			break;
		case 16:
			if (gpio_pin >= 1 && gpio_pin <= 3)
				WRITE_REG(reg, 0x1);

			break;
		default:
			WRITE_REG(reg, 0);
			break;
		}
	} else
		ret = -1;
exit_1:

	pr_bif("bifapi: reg=0x%x offset=0x%x grp=%d pin=%d\n",
		(uint)reg, offset, gpio_grp, gpio_pin);

	return ret;
}
#endif
int bifplat_config(void *p)
{
	int ret = BIFOK;
	struct bifplat_info *pl = (struct bifplat_info *)p;

	if (!pl) {
		ret = -BIFERR;
		goto exit_1;
	}

	bifplat_get_macro_config((void *)pl);

	if (pl->plat_type == PLAT_AP) {
#ifdef CONFIG_HISI
		memset(pl->platform, 0, PLATFORM_SIZE);
		sprintf(pl->platform, "%s", "hisi");
		pl->param = PARAM_MODULE;
		pl->bifbase_phyaddr = CPSIDE_DDR_ADDR;
		pl->bifbase_phyaddrsize = CPSIDE_DDR_ADDRSIZE;
		pl->irq_pin_absent = 0;
		if (bifbase_irq_pin)
			pl->irq_pin = bifbase_irq_pin;
		else
			pl->irq_pin = BIFIRQ_PIN;
		if (bifbase_tri_pin)
			pl->tri_pin = bifbase_tri_pin;
		else
			pl->tri_pin = BIFTRI_PIN;

		muxctrl_base_va = (void __iomem *)IO_ADDRESS(MUX_CTL_BASE_ADDR);
		//GPIO_WRITE_REG(muxctrl_base_va + BIFIRQ_OFFSET, 0x0);
		//GPIO_WRITE_REG(muxctrl_base_va + BIFTRI_OFFSET, 0x0);
		hisi_gpio_pin_mux(muxctrl_base_va, pl->irq_pin);
		hisi_gpio_pin_mux(muxctrl_base_va, pl->tri_pin);
#endif
	}

#ifdef CONFIG_HOBOT_BIF_TEST
	memset(pl->platform, 0, PLATFORM_SIZE);
	sprintf(pl->platform, "%s", "vmware");
	pl->param = PARAM_MODULE;
	pl->bifbase_phyaddr = CPSIDE_DDR_ADDR;
	pl->bifbase_phyaddrsize = CPSIDE_DDR_ADDRSIZE;
	pl->irq_pin_absent = 1;
	pl->irq_pin = -1;
	pl->irq_num = -1;
	pl->tri_pin = -1;
	pl->tri_val = 0;
	ret = t_bif_netlink_init();
#endif

exit_1:
	return ret;
}
EXPORT_SYMBOL(bifplat_config);

void bifplat_unconfig(void *p)
{
	struct bifplat_info *pl = (struct bifplat_info *)p;

	if (!pl)
		return;

	pr_bif("bifapi: unconfig begin...\n");

#ifdef CONFIG_HOBOT_BIF_TEST
	pr_bif("bifapi: exit bif netlink...\n");
	t_bif_netlink_exit();
#else

#endif
}
EXPORT_SYMBOL(bifplat_unconfig);
