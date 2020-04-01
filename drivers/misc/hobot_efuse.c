/***************************************************************************
 *			COPYRIGHT NOTICE
 *		Copyright 2020 Horizon Robotics, Inc.
 *			All rights reserved.
 ***************************************************************************/
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <soc/hobot/hobot_efuse.h>

#define X3_EFUSE_NAME "hobot_efuse"

#define mmio_write_32(reg, data)	writel(data, reg)
#define mmio_read_32(reg)		readl(reg)

static void __iomem *normal_efuse_base;
static void __iomem *sec_efuse_base;
static void __iomem *fw_base_addr;
static void __iomem *sysctrl_base_addr;

static unsigned int _hw_efuse_store(enum EFS_TPE type, unsigned int bnk_num,
		unsigned int data)
{
	void __iomem *efs_bank_base;
	unsigned int sta;

	if (type == EFS_NS) {
		efs_bank_base = normal_efuse_base;
	} else if (type == EFS_S) {
		efs_bank_base = sec_efuse_base;
	} else {
		pr_err("[ERR] %s no such type %d!!!", __FUNCTION__, type);
		return HW_EFUSE_READ_FAIL;
	}

	// enable efuse
	mmio_write_32(efs_bank_base + HW_EFS_CFG, HW_EFS_CFG_EFS_EN);

    //// Check lock bank before write data
    // set bank addr to bnk31
	mmio_write_32(efs_bank_base + HW_EFS_BANK_ADDR, HW_EFUSE_LOCK_BNK);

	// trigger read action
	mmio_write_32(efs_bank_base + HW_EFS_CFG,
				  HW_EFS_CFG_EFS_EN | HW_EFS_CFG_EFS_PROG_RD);

	// polling efuse ready (check efs_rdy == 1)
	do {
		sta = mmio_read_32(efs_bank_base + HW_EFS_STATUS);

		if (sta & HW_EFS_EFS_RDY)
			break;
	} while (1);

	// check if bank is valid programmed data
	sta = mmio_read_32(efs_bank_base + HW_EFS_READ_DATA);

	if (sta & (1 << bnk_num)) {
		pr_err("[ERR] bank %d was locked, it can not be writed.\n", bnk_num);
		return HW_EFUSE_WRITE_LOCKED_BNK;
	}

    //// efuse write data
	// Set bank address to what we want
	mmio_write_32(efs_bank_base + HW_EFS_BANK_ADDR, bnk_num);

	// program data
	mmio_write_32(efs_bank_base + HW_EFS_PROG_DATA, data);

	// Trigger program action
	mmio_write_32(efs_bank_base + HW_EFS_CFG,
				  HW_EFS_CFG_EFS_EN | HW_EFS_CFG_EFS_PROG_WR);

	// polling efuse ready (check efs_rdy == 1)
	do {
		sta = mmio_read_32(efs_bank_base + HW_EFS_STATUS);

		if (sta & HW_EFS_EFS_RDY_PASS)
			break;
	} while (1);

    //// efuse read data for varification
    // Set bank address to what we want
    mmio_write_32(efs_bank_base + HW_EFS_BANK_ADDR, bnk_num);

	// Trigger read action
	mmio_write_32(efs_bank_base + HW_EFS_CFG,
				  HW_EFS_CFG_EFS_EN | HW_EFS_CFG_EFS_PROG_RD);

	// polling efuse ready (check efs_rdy == 1)
	do {
		sta = mmio_read_32(efs_bank_base + HW_EFS_STATUS);

		if (sta & HW_EFS_EFS_RDY)
			break;
	} while (1);

	// read data
	sta = mmio_read_32(efs_bank_base + HW_EFS_READ_DATA);

    // verify writed data
    if (sta != data) {
        pr_err("[ERR] bank %d write fail.\n", bnk_num);
		return HW_EFUSE_WRITE_FAIL;
    }

    //// efuse lock bank
	// set program bank addr to bnk31
	mmio_write_32(efs_bank_base + HW_EFS_BANK_ADDR, HW_EFUSE_LOCK_BNK);

	// Write lock to lock_bank (31) for specfic ank HW_EFS_PROG_DATA
	mmio_write_32(efs_bank_base + HW_EFS_PROG_DATA, 1 << bnk_num);

	// Trigger program action
	mmio_write_32(efs_bank_base + HW_EFS_CFG,
				  HW_EFS_CFG_EFS_EN | HW_EFS_CFG_EFS_PROG_WR);

	// polling efuse ready (check efs_rdy == 1)
	do {
		sta = mmio_read_32(efs_bank_base + HW_EFS_STATUS);

		if (sta & HW_EFS_EFS_RDY_PASS)
			break;
	} while (1);

	return HW_EFUSE_READ_OK;
}

static unsigned int _hw_efuse_load(enum EFS_TPE type, unsigned int bnk_num,
								   unsigned int *ret)
{
	void __iomem *efs_bank_base;
	unsigned int sta, val;

	if (type == EFS_NS) {
		efs_bank_base = normal_efuse_base;
	} else if (type == EFS_S) {
		efs_bank_base = sec_efuse_base;
	} else {
		pr_err("[ERR] %s no such type %d!!!", __FUNCTION__, type);
		return HW_EFUSE_READ_FAIL;
	}

	// enable efuse
	mmio_write_32(efs_bank_base + HW_EFS_CFG, HW_EFS_CFG_EFS_EN);

	// set bank addr to bnk31
	mmio_write_32(efs_bank_base + HW_EFS_BANK_ADDR, HW_EFUSE_LOCK_BNK);

	// trigger read action
	mmio_write_32(efs_bank_base + HW_EFS_CFG,
				  HW_EFS_CFG_EFS_EN | HW_EFS_CFG_EFS_PROG_RD);

	// polling efuse ready (check efs_rdy == 1)
	do {
		sta = mmio_read_32(efs_bank_base + HW_EFS_STATUS);

		if (sta & HW_EFS_EFS_RDY)
			break;
	} while (1);

	// check if bank is valid programmed data
	val = mmio_read_32(efs_bank_base + HW_EFS_READ_DATA);

	if (bnk_num == HW_EFUSE_LOCK_BNK) {
		*ret = val;
		return HW_EFUSE_READ_OK;
	}

	if (!(val & (1 << bnk_num))) {
		pr_err("bank %d not locked, it is not a valid data\n", bnk_num);
		*ret = 0;
		return HW_EFUSE_READ_UNLOCK;
	}
	// Set bank address to what we want
	mmio_write_32(efs_bank_base + HW_EFS_BANK_ADDR, bnk_num);

	// Trigger read action
	mmio_write_32(efs_bank_base + HW_EFS_CFG,
				  HW_EFS_CFG_EFS_EN | HW_EFS_CFG_EFS_PROG_RD);

	// polling efuse ready (check efs_rdy == 1)
	do {
		sta = mmio_read_32(efs_bank_base + HW_EFS_STATUS);

		if (sta & HW_EFS_EFS_RDY)
			break;
	} while (1);

	// read data
	val = mmio_read_32(efs_bank_base + HW_EFS_READ_DATA);

	*ret = val;

	return HW_EFUSE_READ_OK;
}

static unsigned int _sw_efuse_set_register(enum EFS_TPE type)
{
	void __iomem *efs_bank_off;
	unsigned int i, efs_bank_num, val;


	if (type == EFS_NS) {
		efs_bank_off = fw_base_addr + EFUSE_NS_OFF;
		efs_bank_num = NS_EFUSE_NUM;
	} else if (type == EFS_S) {
		efs_bank_off = fw_base_addr + EFUSE_S_OFF;
		efs_bank_num = S_EFUSE_NUM;
	} else {
		pr_err("[ERR] %s no such type %d!!!", __FUNCTION__, type);
		return SW_EFUSE_FAIL;
	}

	for (i = 0; i < efs_bank_num; i++) {
		if (_hw_efuse_load(type, i, &val) == HW_EFUSE_READ_FAIL) {
			pr_err("[ERR] HW Efuse read fail type: %d, bnk: %d\n", type, i);
			return SW_EFUSE_FAIL;
		}
		mmio_write_32(efs_bank_off + i * 4, val);
	}

	return SW_EFUSE_OK;
}

unsigned int scomp_sw_efuse_set_readdone(enum EFS_TPE type,
										 unsigned int enable)
{
	void __iomem *efs_rdone_reg_off = 0;

	if (type == EFS_NS) {
		efs_rdone_reg_off = fw_base_addr + NS_EFS_RDONE;
	} else if (type == EFS_S) {
		efs_rdone_reg_off = fw_base_addr + S_EFS_RDONE;
	} else {
		pr_err("[ERR] %s no such type %d!!!", __FUNCTION__, type);
		return SW_EFUSE_FAIL;
	}

	mmio_write_32(efs_rdone_reg_off, enable);

	return SW_EFUSE_OK;
}

void scomp_load_efuse(void)
{
	_sw_efuse_set_register(EFS_NS);
	_sw_efuse_set_register(EFS_S);
}

int read_sw_efuse_bnk(enum EFS_TPE type, unsigned int bnk_num)
{
	void __iomem *efs_reg_off;

	if (type == EFS_NS) {
		efs_reg_off = fw_base_addr + EFUSE_NS_OFF;
	} else if (type == EFS_S) {
		efs_reg_off = fw_base_addr + EFUSE_S_OFF;
	} else {
		pr_err("[ERR] %s no such type %d!!!", __FUNCTION__, type);
		return SW_EFUSE_FAIL;
	}

	return mmio_read_32(efs_reg_off + (bnk_num << 2));
}

int write_hw_efuse_bnk(enum EFS_TPE type, unsigned int bnk_num,
		unsigned int val)
{
	return _hw_efuse_store(type, bnk_num, val);
}

int write_sw_efuse_bnk(enum EFS_TPE type, unsigned int bnk_num,
		unsigned int val)
{
	void __iomem *efs_reg_off = 0;

	if (type == EFS_NS) {
		efs_reg_off = fw_base_addr + EFUSE_NS_OFF;
	} else if (type == EFS_S) {
		efs_reg_off = fw_base_addr + EFUSE_S_OFF;
	} else {
		pr_err("[ERR] %s no such type %d!!!", __FUNCTION__, type);
		return SW_EFUSE_FAIL;
	}

	mmio_write_32(efs_reg_off + (bnk_num << 2), val);

	return SW_EFUSE_OK;
}

void scomp_sw_efuse_set_lock(void)
{
	mmio_write_32(sysctrl_base_addr + SYSCTL_S_EFS_LOCK, 1);
}

void scomp_sec_lock_down(void)
{
	mmio_write_32(sysctrl_base_addr + SYSCTL_LOCKDOWN_SECURE, 1);
}

//****************************************
// Efuse API
//****************************************
int efuse_write_data(enum EFS_TPE type, unsigned int bnk_num, unsigned int val)
{
    int nRet = SW_EFUSE_OK;

    /// 1st step write hw efuse
    nRet = write_hw_efuse_bnk(type, bnk_num, val);
    if (HW_EFUSE_READ_OK != nRet) {
        pr_err("[ERR] %s write hw efuse error:%d.\n", __FUNCTION__, nRet);
        return RET_API_EFUSE_FAIL;
    }

    /// 2nd write sw efuse data when 1st step sccess.
    nRet = write_sw_efuse_bnk(type, bnk_num, val);
    if (SW_EFUSE_OK != nRet) {
        pr_err("[ERR] %s write sw efuse error:%d.\n", __FUNCTION__, nRet);
        return RET_API_EFUSE_FAIL;
    }

    return RET_API_EFUSE_OK;
}
////////////// End of efuse API

static ssize_t secure_show(struct device_driver *drv, char *buf)
{
	unsigned int i, val, bank = 0;
	char temp[50];

	for (i = 0; i < 32; i++, bank++) {
		if (!(i % 4)) {
			sprintf(temp, "\nsecure_bank 0x%.8x:", bank);
			strcat(buf, temp);
		}

		val = read_sw_efuse_bnk(EFS_S, bank);

		sprintf(temp, " %.8x", val);
		strcat(buf, temp);
	}

	strcat(buf, "\n");

	return strlen(buf);
}

static ssize_t secure_store(struct device_driver *drv,
				const char *buf, size_t count)
{
	return count;
}

static ssize_t normal_show(struct device_driver *drv, char *buf)
{
	unsigned int i, val = 0, bank = 0;
	char temp[50];

	for (i = 0; i < 32; i++, bank++) {
		if (!(i % 4)) {
			sprintf(temp, "\nnormal_bank 0x%.8x:", bank);
			strcat(buf, temp);
		}

		val = read_sw_efuse_bnk(EFS_NS, bank);

		sprintf(temp, " %.8x", val);
		strcat(buf, temp);
	}

	strcat(buf, "\n");

	return strlen(buf);
}

static ssize_t normal_store(struct device_driver *drv,
				const char *buf, size_t count)
{
	return count;
}

static struct driver_attribute secure_attribute =
	__ATTR(secure_efuse, 0644, secure_show, secure_store);

static struct driver_attribute normal_attribute =
	__ATTR(normal_efuse, 0644, normal_show, normal_store);

static struct attribute *efuse_attributes[] = {
	&secure_attribute.attr,
	&normal_attribute.attr,
	NULL
};

static const struct attribute_group efuse_group = {
	.name = "efuse",
	.attrs = efuse_attributes,
};

static const struct attribute_group *efuse_attr_group[] = {
	&efuse_group,
	NULL,
};

/* Match table for of_platform binding */
static const struct of_device_id x3_efuse_of_match[] = {
	{ .compatible = "hobot,x3-efuse", },
	{}
};
MODULE_DEVICE_TABLE(of, x3_efuse_of_match);

/**
 * x3_efuse_probe - Platform driver probe
 * @pdev: Pointer to the platform device structure
 *
 * Return: 0 on success, negative errno otherwise
 */
static int x3_efuse_probe(struct platform_device *pdev)
{
	int rc;
	struct resource *reg;

	reg = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!reg) {
		rc = -ENODEV;
		goto err_out;
	}

	normal_efuse_base = ioremap(reg->start, resource_size(reg));
	if (!normal_efuse_base) {
		pr_err("Unable to map normal_efuse_base registers\n");
		return -ENOMEM;
	}

	reg = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!reg) {
		rc = -ENODEV;
		goto err_out;
	}

	sec_efuse_base = ioremap(reg->start, resource_size(reg));
	if (!sec_efuse_base)
		return -ENOMEM;

	reg = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!reg) {
		rc = -ENODEV;
		goto err_out;
	}

	fw_base_addr = ioremap(reg->start, resource_size(reg));
	if (!fw_base_addr)
		return -ENOMEM;

	sysctrl_base_addr = ioremap(SYSCTL_REG_BASE, SYSCTL_REG_LEN);
	if (!sysctrl_base_addr)
		return -ENOMEM;

	pr_info("hobot_efuse probe success\n");
	return 0;
err_out:
	return rc;
}

static int x3_efuse_remove(struct platform_device *pdev)
{
	iounmap(normal_efuse_base);
	iounmap(sec_efuse_base);
	iounmap(fw_base_addr);
	iounmap(sysctrl_base_addr);
	return 0;
}

static struct platform_driver x3_efuse_platform_driver = {
	.probe   = x3_efuse_probe,
	.remove  = x3_efuse_remove,
	.driver  = {
		.name = X3_EFUSE_NAME,
		.of_match_table = x3_efuse_of_match,
		.groups = efuse_attr_group,
		},
};

static int __init x3_efuse_init(void)
{
	int retval = 0;

	/* Register the platform driver */
	retval = platform_driver_register(&x3_efuse_platform_driver);
	if (retval)
		pr_err("Unable to register platform driver\n");

	return retval;
}

static void __exit x3_efuse_exit(void)
{
	/* Unregister the platform driver */
	platform_driver_unregister(&x3_efuse_platform_driver);
}

module_init(x3_efuse_init);
module_exit(x3_efuse_exit);

MODULE_DESCRIPTION("Driver for X3 EFUSE");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL");
