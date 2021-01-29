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
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <soc/hobot/hobot_efuse.h>

#define X3_EFUSE_NAME "hobot_efuse"

#define mmio_write_32(reg, data)	writel(data, reg)
#define mmio_read_32(reg)		readl(reg)

static void __iomem *normal_efuse_base;
static void __iomem *fw_base_addr;
static DEFINE_MUTEX(hobot_efuse_mutex);
static struct hobot_efuse_dev efuse_dev = {0};

static unsigned int _hw_efuse_store(unsigned int bnk_num,
		unsigned int data, bool lock)
{
	void __iomem *efs_bank_base;
	unsigned int sta;

	efs_bank_base = normal_efuse_base;

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
	if ((sta & data) != data) {
		pr_err("[ERR] bank %d write fail.\n", bnk_num);
		return HW_EFUSE_WRITE_FAIL;
	}
	if (lock == 0)
		return HW_EFUSE_READ_OK;
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

static unsigned int _hw_efuse_load(unsigned int bnk_num,
				   unsigned int *ret)
{
	void __iomem *efs_bank_base;
	unsigned int sta, val;

	efs_bank_base = normal_efuse_base;

	// enable efuse
	mmio_write_32(efs_bank_base + HW_EFS_CFG, HW_EFS_CFG_EFS_EN);
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

static unsigned int _sw_efuse_set_register(void)
{
	void __iomem *efs_bank_off;
	unsigned int i, efs_bank_num, val;


	efs_bank_off = fw_base_addr + EFUSE_NS_OFF;
	efs_bank_num = NS_EFUSE_NUM;

	for (i = 0; i < efs_bank_num; i++) {
		if (_hw_efuse_load(i, &val) == HW_EFUSE_READ_FAIL) {
			pr_err("[ERR] HW Efuse read fail bnk: %d\n", i);
			return SW_EFUSE_FAIL;
		}
		mmio_write_32(efs_bank_off + i * 4, val);
	}

	return SW_EFUSE_OK;
}

unsigned int scomp_sw_efuse_set_readdone(unsigned int enable)
{
	void __iomem *efs_rdone_reg_off = 0;

	efs_rdone_reg_off = fw_base_addr + NS_EFS_RDONE;

	mmio_write_32(efs_rdone_reg_off, enable);

	return SW_EFUSE_OK;
}

void scomp_load_efuse(void)
{
	_sw_efuse_set_register();
}

int read_sw_efuse_bnk(unsigned int bnk_num)
{
	void __iomem *efs_reg_off;

	efs_reg_off = fw_base_addr + EFUSE_NS_OFF;

	return mmio_read_32(efs_reg_off + (bnk_num << 2));
}

int write_hw_efuse_bnk(unsigned int bnk_num,
		unsigned int val, bool lock)
{
	return _hw_efuse_store(bnk_num, val, lock);
}

int write_sw_efuse_bnk(unsigned int bnk_num,
		unsigned int val)
{
	void __iomem *efs_reg_off = 0;

	efs_reg_off = fw_base_addr + EFUSE_NS_OFF;

	mmio_write_32(efs_reg_off + (bnk_num << 2), val);

	return SW_EFUSE_OK;
}

//****************************************
// Efuse API
//****************************************
int efuse_write_data(unsigned int bnk_num,
		     unsigned int val, bool lock)
{
    int nRet = SW_EFUSE_OK;

    /// 1st step write hw efuse
    nRet = write_hw_efuse_bnk(bnk_num, val, lock);
    if (HW_EFUSE_READ_OK != nRet) {
        pr_err("[ERR] %s write hw efuse error:%d.\n", __FUNCTION__, nRet);
        return RET_API_EFUSE_FAIL;
    }

    /// 2nd write sw efuse data when 1st step sccess.
    nRet = write_sw_efuse_bnk(bnk_num, val);
    if (SW_EFUSE_OK != nRet) {
        pr_err("[ERR] %s write sw efuse error:%d.\n", __FUNCTION__, nRet);
        return RET_API_EFUSE_FAIL;
    }

    return RET_API_EFUSE_OK;
}
////////////// End of efuse API

static ssize_t normal_show(struct device_driver *drv, char *buf)
{
	unsigned int i, val = 0, bank = 0;
	char temp[50];

	for (i = 0; i < 32; i++, bank++) {
		if (!(i % 4)) {
			sprintf(temp, "\nnormal_bank 0x%.8x:", bank);
			strcat(buf, temp);
		}

		val = read_sw_efuse_bnk(bank);

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

static struct driver_attribute normal_attribute =
	__ATTR(normal_efuse, 0644, normal_show, normal_store);

static struct attribute *efuse_attributes[] = {
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

static int hobot_efuse_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int hobot_efuse_release(struct inode *inode, struct file *filp)
{
	return 0;
}


static int32_t check_and_efuse_bank(uint32_t bank,
				    uint32_t value,
				    uint32_t lock)
{
	uint32_t efuse_value = 0;
	uint32_t efuse_condition = 0;
	int32_t ret = 0;
	uint32_t lock_bank;
	uint32_t type = EFS_NS;

	lock_bank = read_sw_efuse_bnk(HW_EFUSE_LOCK_BNK);
	if ((((lock_bank >> bank) & 0x01) == 0)) {
		efuse_value = read_sw_efuse_bnk(bank);
		if ((!(efuse_value & value) && (value != 0)) ||
		     (value == 0 && lock == 1)) {
			/*efuse condition
			 *1, value==0 and lock==1
			 *2, value!=0 and (efuse_value & value)==0
			 */
			efuse_condition = 1;
		} else {
			pr_info("efuse type:%d bank[%d] mismatch, bypass this bank\n", type, bank);
			efuse_condition = 0;
			return 0;
		}
		if (efuse_condition) {
			ret = efuse_write_data(bank, value, lock);
			if (ret != RET_API_EFUSE_OK) {
				pr_info("burn efuse type: %d secure bank:0x%x failed\n", type, bank);
				return -1;
			}
			if (efuse_value != 0 || lock == 1) {
				ret = _sw_efuse_set_register();
				if (ret != RET_API_EFUSE_OK)
					return -1;
			}
			pr_info("burn efuse type:%d bank:0x%x, value:0x%x, read:0x%x\n",
				type,
				bank, value,
				read_sw_efuse_bnk(bank));
		}
	} else {
		pr_err("[%s,%d],bank[%d] has locked\n", __FILE__, __LINE__, bank);
		return -1;
	}
	return 0;
}
static long hobot_efuse_ioctl(struct file *file,
			      unsigned int cmd,
			      unsigned long arg)
{
	struct  io_efuse_data data;
	int32_t ret = 0;

	if (_IOC_TYPE(cmd) != EFUSE_IOCTL_MAGIC) {
		pr_err("ioctl command magic number does not match.\n");
		return -EINVAL;
	}

	if (_IOC_SIZE(cmd) > sizeof(data))
		return -EINVAL;
	if (copy_from_user(&data, (void __user *)arg, _IOC_SIZE(cmd))) {
		pr_err("%s: copy data failed from userspace\n", __func__);
		return -EFAULT;
	}
	if (data.bank > 30) {
		return -1;
	}
	mutex_lock(&hobot_efuse_mutex);
	switch (cmd) {
		case EFUSE_IOC_SET_BANK:
			pr_debug("bank:%d,value:0x%x, lock:%d\n",
				  data.bank, data.bank_value, data.lock);
			if (data.bank > 11 || data.bank < 7) {
				pr_err("only can write bank7--bank11\n");
				ret = -1;
				break;
			}
			ret = check_and_efuse_bank(data.bank,
						   data.bank_value,
						   data.lock);
			break;
		case EFUSE_IOC_GET_BANK:
			if (data.bank > 30) {
				pr_err("only can read bank0--bank30\n");
				ret = -1;
				break;
			}
			data.bank_value = read_sw_efuse_bnk(data.bank);
			data.lock = (read_sw_efuse_bnk(HW_EFUSE_LOCK_BNK) >>
				     data.bank) & 0x1;
			break;
		default:
			break;
	}
	mutex_unlock(&hobot_efuse_mutex);

        if (_IOC_DIR(cmd) == _IOC_READ) {
                if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd))) {
                        pr_err("%s: copy data to userspace failed\n", __func__);
                        return -EFAULT;
                }
        }

	return ret;
}

static const struct file_operations hb_efuse_fops = {
	.owner		= THIS_MODULE,
	.open		= hobot_efuse_open,
	.release	= hobot_efuse_release,
	.unlocked_ioctl = hobot_efuse_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= hobot_efuse_ioctl,
#endif
};

/* Match table for of_platform binding */
static const struct of_device_id x3_efuse_of_match[] = {
	{ .compatible = "hobot,x3-efuse", },
	{}
};
MODULE_DEVICE_TABLE(of, x3_efuse_of_match);

static int hobot_efuse_init_chrdev(struct hobot_efuse_dev *dev)
{
	int32_t rc;

	// Allocate a major and minor number region for the character device
	rc = alloc_chrdev_region(&dev->dev_num, 0, 1, "efuse");
	if (rc < 0) {
		pr_err("Unable to allocate character device region.\n");
		goto ret;
	}

	// Create a device class for our device
	dev->dev_class = class_create(THIS_MODULE, "efuse");
	if (IS_ERR(dev->dev_class)) {
		pr_err("Unable to create a device class.\n");
		rc = PTR_ERR(dev->dev_class);
		goto free_chrdev_region;
	}

	/* Create a device for our module. This will create a file on the
	 * filesystem, under "/dev/dev->chrdev_name".
	 */
	device_create(dev->dev_class, dev->dev, dev->dev_num, NULL,
		"efuse");

	// Register our character device with the kernel
	cdev_init(&dev->i_cdev, &hb_efuse_fops);
	rc = cdev_add(&dev->i_cdev, dev->dev_num, 1);
	if (rc < 0) {
		pr_err("Unable to add a character device.\n");
		goto device_cleanup;
	}

	return 0;

device_cleanup:
	device_destroy(dev->dev_class, dev->dev_num);
free_chrdev_region:
	unregister_chrdev_region(dev->dev_num, 1);
ret:
	return rc;
}

static void efuse_dev_remove(struct hobot_efuse_dev *dev)
{
	cdev_del(&dev->i_cdev);
	device_destroy(dev->dev_class, dev->dev_num);
	class_destroy(dev->dev_class);
	unregister_chrdev_region(dev->dev_num, 1);
}

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

	fw_base_addr = ioremap(FW_BASE_REG_ADDR, FW_BASE_SIZE);
	if (!fw_base_addr)
		return -ENOMEM;

	rc = hobot_efuse_init_chrdev(&efuse_dev);
	if (rc) {
		dev_err(&pdev->dev, "failed to create char dev for efuse\n");
		goto err_out;
	}
	pr_info("hobot_efuse probe success\n");
	return 0;
err_out:
	return rc;
}

static int x3_efuse_remove(struct platform_device *pdev)
{
	iounmap(normal_efuse_base);
	iounmap(fw_base_addr);
	efuse_dev_remove(&efuse_dev);
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
