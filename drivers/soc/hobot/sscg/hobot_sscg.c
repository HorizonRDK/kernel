/***************************************************************************
*                                              COPYRIGHT NOTICE
*                         Copyright 2018 Horizon Robotics, Inc.
*                                         All rights reserved.
***************************************************************************/
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/string.h>
#include "hobot_sscg.h"

#define DYNAMIC_SSCG 0
#define PRINTK_READ_WRITE 0

// address and offset
enum
{
	BASE_ADDR = 0xa1000000,
	MAPPING_SIZE = 0x1000,
	SSCG_BASE_OFFSET = 0x90,
	PLL_SEL_BASE_OFFSET = 0x300,
	VIO2_PD_BIAS = 0x50,
	VIO2_SS_BIAS = 0x20,
};

// pll sscg table index
enum
{
	CPUPLL,
	SYSPLL,
	CNNPLL,
	DDRPLL,
	VIOPLL,
	PERIPLL,
	VIO2PLL,
	PLL_SIZE = 7,
};

// sscg value table index
enum
{
	SPREAD,
	DIVVAL,
	PLLENABLE,
	DOWNSPREAD = 1,
	SSCG_SETTING_NUM = 3,
};

// sscg pllenable value
enum
{
	DISABLE,
	ENABLE,
};

// sscg_default command
enum
{
	CUSTOMIZE,
	LEVEL1,
	LEVEL2,
	UNDEFINED,
};

// sscg value offset
enum
{
	spread_off = 8,
    divval_off = 2,
    dspread_off = 1,
};

// undefined sscg value in ssccg table
enum
{
	UNDEFINED_SPREAD,
	UNDEFINED_DIVVAL,
	UNDEFINED_PLLENABLE,
};

static void __iomem *sysctl_base, *sscg_base;
uint32_t orig_val = 0;
int command_sscg_default = UNDEFINED; // record current command (sscg_default)
int is_null_pll_sscg_status = 1; // 0: not null, 1: null

uint32_t pll_sscg_level1[PLL_SIZE][SSCG_SETTING_NUM - 1] =
{
	{10, 3},
	{10, 3},
	{10, 3},
	{UNDEFINED_SPREAD, UNDEFINED_DIVVAL}, // DDR PLL
	{10, 3},
	{10, 3},
	{10, 3},
};

uint32_t pll_sscg_level2[PLL_SIZE][SSCG_SETTING_NUM - 1] =
{
	{20, 6},
	{20, 6},
	{20, 6},
	{UNDEFINED_SPREAD, UNDEFINED_DIVVAL}, // DDR PLL
	{20, 6},
	{20, 6},
	{20, 6},
};

uint32_t pll_sscg_status[PLL_SIZE][SSCG_SETTING_NUM] =
{
	{0, 0, DISABLE},
	{0, 0, DISABLE},
	{0, 0, DISABLE},
	{UNDEFINED_SPREAD, UNDEFINED_DIVVAL, UNDEFINED_PLLENABLE}, // DDR PLL
	{0, 0, DISABLE},
	{0, 0, DISABLE},
	{0, 0, DISABLE},
};

#define SHOW_PLL_STATUS(_name)                                                                                                    \
if (pll_sscg_status[_name][PLLENABLE] == ENABLE) {                                                                                \
	printk("[%s]:[ENABLE][SPREAD: %d][DIVVAL: %d]\n", #_name, pll_sscg_status[_name][SPREAD], pll_sscg_status[_name][DIVVAL]);    \
} else {                                                                                                                            \
	printk("[%s]:[DISABLE]\n", #_name);                                                                                           \
}                                                                                                                                 \

#define ATTR_ASSIGN(_name)              \
	&dev_attr_##_name##_spread.attr,    \
	&dev_attr_##_name##_divval.attr,    \
	&dev_attr_##_name##_pllenable.attr,

#define SSCG_RW_ATTR(_name, _mod, _sscg_index)                                                                                     \
static ssize_t _name##_spread_show(struct device *dev,                                                                           \
			     struct device_attribute *attr, char *buf)                                                                        \
{                                                                                                                                 \
	return sprintf(buf, "%s:unsupported function\n", __func__);                                                                   \
}                                                                                                                                 \
static ssize_t _name##_spread_store(struct device *dev,                                                                          \
			      struct device_attribute *attr,                                                                                  \
			      const char *buf, size_t count)                                                                                  \
{                                                                                                                                 \
	if (command_sscg_default == CUSTOMIZE) {                                                                                      \
		if (!DYNAMIC_SSCG) {                                                                                                      \
			if (pll_sscg_status[_sscg_index][PLLENABLE] == DISABLE) {                                                             \
				if (kstrtou32(buf, 0, &pll_sscg_status[_sscg_index][SPREAD])) {                                                   \
					dev_warn(dev, "%s: %d buffer string covert to integer error\n", __func__, __LINE__);                      \
				}                                                                                                                 \
			}                             	                                                                                     \
			else {															\
				printk("You have setted the sscg value\n");                                                                      \
			}															\
		}                                                                                                                         \
		else {                                                                                                                    \
			if (kstrtou32(buf, 0, &pll_sscg_status[_sscg_index][SPREAD])) {                                                       \
				dev_warn(dev, "%s: %d buffer string covert to integer error\n", __func__, __LINE__);                              \
			}                                                                                                                     \
		}                                                                                                                         \
	}                                                                                                                             \
	else {                                                                                                                        \
		printk("please enter 0 to sscg_default to customize pll\n");                                                              \
	}                                                                                                                             \
	return count;                                                                                                                 \
}                                                                                                                                 \
static ssize_t _name##_divval_show (struct device *dev,                                                                           \
			     struct device_attribute *attr, char *buf)                                                                        \
{                                                                                                                                 \
	return sprintf(buf, "%s:unsupported function\n", __func__);                                                                   \
}                                                                                                                                 \
static ssize_t _name##_divval_store (struct device *dev,                                                                          \
			      struct device_attribute *attr,                                                                                  \
			      const char *buf, size_t count)                                                                                  \
{                                                                                                                                 \
	if (command_sscg_default == CUSTOMIZE) {                                                                                      \
		if (!DYNAMIC_SSCG) {                                                                                                      \
			if (pll_sscg_status[_sscg_index][PLLENABLE] == DISABLE) {                                                             \
				if (kstrtou32(buf, 0, &pll_sscg_status[_sscg_index][DIVVAL])) {                                                   \
					dev_warn(dev, "%s: %d buffer string covert to integer error\n", __func__, __LINE__);                          \
				}                                                                                                                 \
			}                                                                                                                     \
			else { 															\
				printk("You have setted the sscg value\n");                                                                      \
			}															\
		}                                                                                                                         \
		else {                                                                                                                    \
			if (kstrtou32(buf, 0, &pll_sscg_status[_sscg_index][DIVVAL])) {                                                       \
				dev_warn(dev, "%s: %d buffer string covert to integer error\n", __func__, __LINE__);                              \
			}                                                                                                                     \
		}                                                                                                                         \
	}                                                                                                                             \
	else {                                                                                                                        \
		printk("please enter 0 to sscg_default to customize pll\n");                                                              \
	}                                                                                                                             \
	return count;                                                                                                                 \
}                                                                                                                                 \
static ssize_t _name##_pllenable_show(struct device *dev,                                                                        \
			     struct device_attribute *attr, char *buf)                                                                        \
{                                                                                                                                 \
	return sprintf(buf, "%s:unsupported function\n", __func__);                                                                   \
}                                                                                                                                 \
static ssize_t _name##_pllenable_store(struct device *dev,                                                                       \
			      struct device_attribute *attr,                                                                                  \
			      const char *buf, size_t count)                                                                                  \
{                                                                                                                                 \
	if (command_sscg_default == CUSTOMIZE) {                                                                                      \
		if (!strncmp(buf, "0", 1)) {                                                                                              \
			pll_sscg_status[_sscg_index][PLLENABLE] = DISABLE;                                                                    \
		}                                                                                                                         \
		else if (!strncmp(buf, "1", 1)) {                                                                                         \
			if (!DYNAMIC_SSCG) {                                                                                                  \
				if (pll_sscg_status[_sscg_index][PLLENABLE] == DISABLE) {                                                         \
					pll_sscg_status[_sscg_index][PLLENABLE] = ENABLE;                                                             \
					do_ss(_sscg_index, pll_sscg_status[_sscg_index][SPREAD], pll_sscg_status[_sscg_index][DIVVAL], DOWNSPREAD);   \
				}                                                                                                                 \
				else {														\
					printk("You have setted the sscg value\n");                                                              \
				}														\
			}                                                                                                                     \
			else {                                                                                                                \
				pll_sscg_status[_sscg_index][PLLENABLE] = ENABLE;                                                                 \
				do_ss(_sscg_index, pll_sscg_status[_sscg_index][SPREAD], pll_sscg_status[_sscg_index][DIVVAL], DOWNSPREAD);       \
			}                                                                                                                     \
		}                                                                                                                         \
		else {                                                                                                                    \
			printk("%s: %d undefined input %s\n", __func__, __LINE__, buf);                                                       \
		}                                                                                                                         \
	}                                                                                                                             \
	else {                                                                                                                        \
		printk("please enter 0 to sscg_default to customize pll\n");                                                              \
	}                                                                                                                             \
	return count;                                                                                                                 \
}                                                                                                                                 \
static DEVICE_ATTR(_name##_spread, _mod, _name##_spread_show, _name##_spread_store);                                              \
static DEVICE_ATTR(_name##_divval, _mod, _name##_divval_show, _name##_divval_store);                                              \
static DEVICE_ATTR(_name##_pllenable, _mod, _name##_pllenable_show, _name##_pllenable_store);

SSCG_RW_ATTR(cpupll, (S_IWUSR | S_IRUSR), CPUPLL)
SSCG_RW_ATTR(syspll, (S_IWUSR | S_IRUSR), SYSPLL)
SSCG_RW_ATTR(cnnpll, (S_IWUSR | S_IRUSR), CNNPLL)
SSCG_RW_ATTR(viopll, (S_IWUSR | S_IRUSR), VIOPLL)
SSCG_RW_ATTR(peripll, (S_IWUSR | S_IRUSR), PERIPLL)
SSCG_RW_ATTR(vio2pll, (S_IWUSR | S_IRUSR), VIO2PLL)

static ssize_t status_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	printk("======================================================\n");
	SHOW_PLL_STATUS(CPUPLL)
	SHOW_PLL_STATUS(SYSPLL)
	SHOW_PLL_STATUS(CNNPLL)
	SHOW_PLL_STATUS(VIOPLL)
	SHOW_PLL_STATUS(PERIPLL)
	SHOW_PLL_STATUS(VIO2PLL)
	printk("======================================================\n");
	return 0;
}

static ssize_t status_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	printk("%s:unsupported function\n", __func__);
	return count;
}

static ssize_t sscg_default_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s:unsupported function\n", __func__);
}

static ssize_t sscg_default_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	if (!strncmp(buf, "0", 1)) {
		int sscg_index;
		command_sscg_default = CUSTOMIZE;
		
		if (is_null_pll_sscg_status) {
			for (sscg_index = 0; sscg_index < PLL_SIZE; sscg_index ++) {
				if (sscg_index == DDRPLL) continue;
				// fill in level 1 sscg values to prevent user from forgetting enter certain sscg values
				pll_sscg_status[sscg_index][SPREAD] = pll_sscg_level1[sscg_index][SPREAD];
				pll_sscg_status[sscg_index][DIVVAL] = pll_sscg_level1[sscg_index][DIVVAL];
			}
			is_null_pll_sscg_status = 0;
		}
	}
	else if (!strncmp(buf, "1", 1)) {
		int sscg_index;
		command_sscg_default = LEVEL1;
		for (sscg_index = 0; sscg_index < PLL_SIZE; sscg_index ++) {
			if (sscg_index == DDRPLL) continue;
#if !DYNAMIC_SSCG
			if (pll_sscg_status[sscg_index][PLLENABLE] == DISABLE) {
#endif
			pll_sscg_status[sscg_index][SPREAD] = pll_sscg_level1[sscg_index][SPREAD];
			pll_sscg_status[sscg_index][DIVVAL] = pll_sscg_level1[sscg_index][DIVVAL];
			pll_sscg_status[sscg_index][PLLENABLE] = ENABLE;
			do_ss(sscg_index, pll_sscg_status[sscg_index][SPREAD], pll_sscg_status[sscg_index][DIVVAL], DOWNSPREAD);

#if !DYNAMIC_SSCG
			}
			else printk("You have setted the sscg value\n");
#endif
		}
		
		// all of plls have sscg values
		if (is_null_pll_sscg_status) is_null_pll_sscg_status = 0;
	}
	else if (!strncmp(buf, "2", 1)) {
		int sscg_index;
		command_sscg_default = LEVEL2;
		for (sscg_index = 0; sscg_index < PLL_SIZE; sscg_index ++) {
			if (sscg_index == DDRPLL) continue;
#if !DYNAMIC_SSCG
			if (pll_sscg_status[sscg_index][PLLENABLE] == DISABLE) {
#endif
			pll_sscg_status[sscg_index][SPREAD] = pll_sscg_level2[sscg_index][SPREAD];
			pll_sscg_status[sscg_index][DIVVAL] = pll_sscg_level2[sscg_index][DIVVAL];
			pll_sscg_status[sscg_index][PLLENABLE] = ENABLE;
			do_ss(sscg_index, pll_sscg_status[sscg_index][SPREAD], pll_sscg_status[sscg_index][DIVVAL], DOWNSPREAD);
			
#if !DYNAMIC_SSCG
			}
			else printk("You have setted the sscg value\n");
#endif
		}
		
		// all of plls have sscg values
		if (is_null_pll_sscg_status) is_null_pll_sscg_status = 0;
	}
	else {
		printk("%s: %d undefined input %s\n", __func__, __LINE__, buf);
	}
	return count;
}
static DEVICE_ATTR(status, (S_IWUSR | S_IRUSR), status_show, status_store);
static DEVICE_ATTR(sscg_default, (S_IWUSR | S_IRUSR), sscg_default_show, sscg_default_store);

static struct attribute *hobot_sscg_attributes[] = {
	ATTR_ASSIGN(cpupll)
	ATTR_ASSIGN(syspll)
	ATTR_ASSIGN(cnnpll)
	ATTR_ASSIGN(viopll)
	ATTR_ASSIGN(peripll)
	ATTR_ASSIGN(vio2pll)
	&dev_attr_status.attr,
	&dev_attr_sscg_default.attr,
	NULL,
};

static const struct attribute_group hobot_sscg_attr_group = {
	.attrs = hobot_sscg_attributes,
};
 
static int hobot_sscg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = 0;
	dev_info(dev, "%s: %d start\n", __func__, __LINE__);

	sysctl_base = ioremap_nocache(BASE_ADDR, MAPPING_SIZE); 
	sscg_base = sysctl_base + SSCG_BASE_OFFSET;
 
	ret = sysfs_create_group(&dev->kobj, &hobot_sscg_attr_group);
	if (ret < 0)
		dev_warn(dev, "attr group create failed\n");
	else
		dev_info(dev, "attr group create success!\n");

	dev_info(dev, "%s: %d sussess\n", __func__, __LINE__);
 
	return 0;
}

static int hobot_sscg_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	dev_info(dev, "%s: %d \n", __func__, __LINE__);
	sysfs_remove_group(&dev->kobj, &hobot_sscg_attr_group);
	iounmap(sscg_base);
	return 0;
}

static const struct of_device_id hobot_sscg_of_match[] = {
	{ .compatible = "hobot,sscg", },
	{},
};
MODULE_DEVICE_TABLE(of, hobot_sscg_of_match);

static struct platform_driver hobot_sscg_driver = {
	.probe		= hobot_sscg_probe,
	.remove		= hobot_sscg_remove,
	.driver		= {
		.name	= "sscg",
		.of_match_table = hobot_sscg_of_match,
	},
};

void switch_pll_source(int switch_back_out, int sscg_index) {
	/*###############################################
    # sscg_index: 
    # 0: CPUPLL,  (PLL sel at bit  0, 0 -> 1 -> 0)
    # 1: SYSPLL,  (PLL sel at bit 24, 1 -> 0 -> 1)
    # 2: CNNPLL,  (PLL sel at bit  9, 0 -> 1 -> 0) 
    # 3: DDRPLL,  (PLL sel at bit 13, 0 -> 1 -> 0)
    # 4: VIOPLL,  (PLL sel at bit 16, 1 -> 0 -> 1)
    # 5: PERIPLL, (PLL sel at bit 20, 1 -> 0 -> 1)
    # 6: VIOPLL2, (PLL sel at bit 17, 1 -> 0 -> 1)
    ###############################################*/

	uint32_t bf = 0, pll_sw_val = 0;
	volatile void __iomem *pll_sel_base = sysctl_base + PLL_SEL_BASE_OFFSET;

	switch (sscg_index) {
		case CPUPLL:
			bf = 0;
			pll_sw_val = 1;
			break;
		case SYSPLL:
			bf = 24;
			pll_sw_val = 0;
			break;
		case CNNPLL:
			bf = 9;
			pll_sw_val = 1;
			break;
		case VIOPLL:
			bf = 16;
			pll_sw_val = 0;
			break;
		case PERIPLL:
			bf = 20;
			pll_sw_val = 0;
			break;
		case VIO2PLL:
			bf = 17;
			pll_sw_val = 0;
			break;
		default:
			printk("undefined index %d\n", sscg_index);
			break;
	}

	if (switch_back_out == 1) {
		// switch back
		if (pll_sw_val == 1) {
			pll_sw_val = 0;
		} else {
			pll_sw_val = 1;
		}
	} else {
		;// switch out
	}

	read_reg(pll_sel_base);

	if (pll_sw_val == 1) {
		write_reg(pll_sel_base, pll_sw_val << bf | orig_val);
	} else {
		write_reg(pll_sel_base, ~(1 << bf) & orig_val);
	}
}

void do_ss(int sscg_index, uint32_t spread, uint32_t divval, uint32_t dspread) {
	switch_pll_source(0, sscg_index);
	_do_ss(sscg_index, spread, divval, dspread);
	switch_pll_source(1, sscg_index);
}

unsigned long sscg_v2p(volatile unsigned char *vaddr) {
	return (unsigned long) (BASE_ADDR + ((unsigned long long) vaddr - (unsigned long long) sysctl_base));
}

ssize_t read_reg(volatile unsigned char *vaddr) {
	orig_val = readl(vaddr);
#if PRINTK_READ_WRITE
	printk("<<< read address: %lx, value: %x\n", sscg_v2p(vaddr), readl(vaddr));
#endif
	return orig_val;
}

void write_reg(volatile unsigned char *vaddr, uint32_t val) {
	writel(val, vaddr);
#if PRINTK_READ_WRITE
	printk(">>> write address: %lx, value %x\n", sscg_v2p(vaddr), val);
#endif
}

void _do_ss(int sscg_index, uint32_t spread, uint32_t divval, uint32_t dspread) {
	uint32_t vio2_pd_bias = VIO2_PD_BIAS, vio2_ss_bias = VIO2_SS_BIAS;
	volatile void __iomem *sysctl_base_temp = sysctl_base;
	volatile void __iomem *sscg_base_temp = sscg_base;

	if (sscg_index == VIO2PLL) {
		sysctl_base_temp = sysctl_base_temp + vio2_pd_bias;
		sscg_base_temp = sscg_base_temp + vio2_ss_bias;
	}

	// 1. Initialize the modullartor with DISABLE_SSCG
	read_reg(sscg_base_temp + (4*sscg_index));
	write_reg(sscg_base_temp + (4*sscg_index), orig_val | 1);

	// 2. Initialze the PLL with PD=1
	read_reg(sysctl_base_temp + (16*sscg_index) + 4);
	write_reg(sysctl_base_temp + (16*sscg_index) + 4, orig_val | 1);

	// 3. Set spread/downspread value
	read_reg(sscg_base_temp + (4*sscg_index));
	write_reg(sscg_base_temp + (4*sscg_index), orig_val | (spread << spread_off) | (divval << divval_off) |  (dspread << dspread_off));

	// down spread disabled ( make it center spread )

	// 4. DSMPD/DACPD
	read_reg(sysctl_base_temp + (16*sscg_index) + 4);
	write_reg(sysctl_base_temp + (16*sscg_index) + 4, ~(3 << 4) & orig_val);

	// 5. sleep (PD=1)
	mdelay(100);

	// 6. Enable DISABLE_SSCG
	read_reg(sscg_base_temp + (4*sscg_index));
	orig_val = orig_val | (spread << spread_off) | (divval << divval_off) |  (dspread << dspread_off);
	write_reg(sscg_base_temp + (4*sscg_index), orig_val & ~1);

	// 7. PD=1 and wait lock state
	read_reg(sysctl_base_temp + (16*sscg_index) + 4);
	write_reg(sysctl_base_temp + (16*sscg_index) + 4, orig_val & ~1);

	mdelay(100);
}

static int __init _hobot_sscg_init(void)
{
	return platform_driver_register(&hobot_sscg_driver);
}

static void __exit _hobot_sscg_exit(void)
{
	platform_driver_unregister(&hobot_sscg_driver);
}

module_init(_hobot_sscg_init);
module_exit(_hobot_sscg_exit);

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("hobot spread specturm module");
MODULE_LICENSE("GPL");
