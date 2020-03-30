/*************************************************************
 ****			 COPYRIGHT NOTICE
 ****		 Copyright 2020 Horizon Robotics, Inc.
 ****			 All rights reserved.
 *************************************************************/
/**
 * @FileName	hobot-bifspi.c
 * @version	2.0
 * @author	yang01.bai@hobot.cc
 * @CreatedTime 2018-07-27-16:56:21
 * @Log		2020-02 support xj3
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/reset.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/pgtable.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <soc/hobot/diag.h>
#include <soc/hobot/hobot_bifspi.h>

#define BIF_ACCESS_FIRST	(0x00000110)
#define BIF_ACCESS_LAST		(0x00000114)
#define BIF_EN_CLEAR		(0x00000104)
#define BIF_INT_STATE		(0x00000108)
#define MEM_MAX_ACCESS		(0xfffffffc)
#define BIF_CLR_INT		(0x0FC0)
#define BIF_RST_NAME		"bifspi"

#define VER		"HOBOT-bifspi_V20.200330"

static int ap_access_first;
module_param(ap_access_first, uint, 0644);
MODULE_PARM_DESC(ap_access_first, "ap_access_first");

static int ap_access_last = MEM_MAX_ACCESS;
module_param(ap_access_last, uint, 0644);
MODULE_PARM_DESC(ap_access_last, "ap_access_last");


static int bif_open(struct inode *inode, struct file *filp);
static int bif_release(struct inode *inode, struct file *filp);
static ssize_t bif_read(struct file *filp, char __user *buf, size_t count,
			loff_t *offset);
static ssize_t bif_write(struct file *filp, const char __user *buf,
			 size_t count, loff_t *offset);
static long bif_unlocked_ioctl(struct file *filp, unsigned int cmd,
			       unsigned long args);
static long bif_compat_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long args);

static int major;
module_param(major, int, 0644);
MODULE_PARM_DESC(major, "major num");

static uint32_t bifspi_last_err_tm_ms;
struct timer_list bifspi_diag_timer;

#ifdef CONFIG_PM_SLEEP
uint32_t g_bif_spi_regs[3];
#endif

struct bifspi_t {
	int irq;
	struct class *class;
	struct device *device;
	struct cdev cdev;
	unsigned int intstatus;
	spinlock_t lock;
	void __iomem *regs_base;
	struct reset_control *bif_rst;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_bif;

	// ddr limited scope
	unsigned int first;
	unsigned int last;
	struct pinctrl_state *pins_gpio;
	struct pinctrl_state *pins_reset;
};
struct bifspi_t *bif_info;

const struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = bif_open,
	.release = bif_release,
	.read = bif_read,
	.write = bif_write,
	.unlocked_ioctl = bif_unlocked_ioctl,
	.compat_ioctl = bif_compat_ioctl,
};

struct bif_acc_reg {
	size_t num;
	int value;
};
struct bif_acc_space {
	unsigned int first;
	unsigned int last;
};
#define MAGIC_NUM 'k'
#define BIF_GET_SHREG		_IO(MAGIC_NUM, 1)
#define BIF_SET_SHREG		_IO(MAGIC_NUM, 2)
#define BIF_SET_ACCESS		_IO(MAGIC_NUM, 3)
#define BIF_UNSET_ACCESS	_IO(MAGIC_NUM, 4)
#define BIF_RESET		_IO(MAGIC_NUM, 5)

static void bif_set_access(struct bifspi_t *pbif, unsigned int first,
			   unsigned int last);

ssize_t bifspi_show_ap_first(struct kobject *driver,
			     struct kobj_attribute *attr, char *buf)
{
	if (bif_info == NULL) {
		pr_err("%s bifspi is not enabled\n", __func__);
		return 0;
	}
	return snprintf(buf, PAGE_SIZE, "0x%08x\n", bif_info->first);
}

ssize_t bifspi_store_ap_first(struct kobject *driver,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	long out;

	if (bif_info == NULL) {
		pr_err("%s bifspi is not enabled\n", __func__);
		return 0;
	}
	ret = (kstrtol(buf, 0, &out));
	if (ret != 0) {
		pr_err("%s input data err\n", __func__);
		return 0;
	}
	bif_info->first = (out & 0xFFFFFFFF);
	bif_set_access(bif_info, bif_info->first, bif_info->last);
	return count;
}

ssize_t bifspi_show_ap_last(struct kobject *driver,
			     struct kobj_attribute *attr, char *buf)
{
	if (bif_info == NULL) {
		pr_err("%s bifspi is not enabled\n", __func__);
		return 0;
	}
	return snprintf(buf, PAGE_SIZE, "0x%08x\n", bif_info->last);
}

ssize_t bifspi_store_ap_last(struct kobject *driver,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	long out;

	if (bif_info == NULL) {
		pr_err("%s bifspi is not enabled\n", __func__);
		return 0;
	}
	ret = (kstrtol(buf, 0, &out));
	if (ret != 0) {
		pr_err("%s input data err\n", __func__);
		return 0;
	}
	ap_access_last = (out & 0xFFFFFFFF);
	bif_info->last = ap_access_last;
	bif_set_access(bif_info, bif_info->first, bif_info->last);
	return count;
}

static struct kobj_attribute ap_access_first_attr = {
	.attr   = {
		.name = __stringify(ap_access_first_attr),
		.mode = 0644,
	},
	.show   = bifspi_show_ap_first,
	.store  = bifspi_store_ap_first,
};

static struct kobj_attribute ap_access_last_attr = {
	.attr   = {
		.name = __stringify(ap_access_last_attr),
		.mode = 0644,
	},
	.show   = bifspi_show_ap_last,
	.store  = bifspi_store_ap_last,
};


static struct attribute *bifspi_attributes[] = {
	&ap_access_first_attr.attr,
	&ap_access_last_attr.attr,
	NULL,
};

static struct attribute_group bifspi_group = {
	.attrs = bifspi_attributes,
};

static int bif_open(struct inode *inode, struct file *filp)
{
	struct bifspi_t *pbif =
	    (struct bifspi_t *)container_of(inode->i_cdev, struct bifspi_t,
					    cdev);
	filp->private_data = pbif;
	return 0;
}

static int bif_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t bif_read(struct file *filp, char __user *buf, size_t count,
			loff_t *offset)
{
	// struct bifspi_t * pbif  = filp->private_data;

	return count;
}

static ssize_t bif_write(struct file *filp, const char __user *buf,
			 size_t count, loff_t *offset)
{
	// struct bifspi_t  * pbif = filp->private_data;

	return count;
}

static int bif_reset(struct bifspi_t *pbif)
{
	int rc = 0;

	rc = reset_control_assert(pbif->bif_rst);
	if (rc < 0) {
		pr_err("%s assert: failed\n", __func__);
		return rc;
	}
	rc = reset_control_deassert(pbif->bif_rst);
	if (rc < 0) {
		pr_err("%s: deassert failed\n", __func__);
		return rc;
	}
	pr_info("bif reset success\n");
	return 0;
}

static int bif_change_reset2gpio(void)
{
	if (!bif_info->pins_gpio)
		return -ENODEV;
	return pinctrl_select_state(bif_info->pinctrl, bif_info->pins_gpio);
}

static int bif_recover_reset_func(void)
{
	if (!bif_info->pins_reset)
		return -ENODEV;
	return pinctrl_select_state(bif_info->pinctrl, bif_info->pins_reset);
}

static void bif_set_access(struct bifspi_t *pbif, unsigned int first,
			   unsigned int last)
{
	int value;
	unsigned long flags;

	bif_change_reset2gpio();
	spin_lock_irqsave(&pbif->lock, flags);
	// set access, enable irq
	writel(first, (void *)(pbif->regs_base + BIF_ACCESS_FIRST));
	writel(last, (void *)(pbif->regs_base + BIF_ACCESS_LAST));

	value = readl((void *)(pbif->regs_base + BIF_EN_CLEAR));
	value |= (1 << 0x04);
	writel(value, (void *)(pbif->regs_base + BIF_EN_CLEAR));

	pbif->first = first;
	pbif->last = last;
	spin_unlock_irqrestore(&pbif->lock, flags);

	bif_recover_reset_func();
	pr_info
	     ("bif set access limit:start add:%p->%#x(%d) end add:%p-%#x(%d)\n",
	     (void *)(pbif->regs_base + BIF_ACCESS_FIRST), first, first,
	     (void *)(pbif->regs_base + BIF_ACCESS_LAST), last, last);
}

static void bif_unset_access(struct bifspi_t *pbif)
{
	int en_val = 0;
	unsigned long flags;

	bif_change_reset2gpio();
	spin_lock_irqsave(&pbif->lock, flags);
	writel(0, (void *)(pbif->regs_base + BIF_ACCESS_FIRST));
	writel(MEM_MAX_ACCESS, (void *)(pbif->regs_base + BIF_ACCESS_LAST));
	en_val &= (~(1 << 0x04));
	writel(en_val, (void *)(pbif->regs_base + BIF_EN_CLEAR));

	pbif->first = 0;
	pbif->last = MEM_MAX_ACCESS;
	spin_unlock_irqrestore(&pbif->lock, flags);

	bif_recover_reset_func();
	pr_info("bif unset access limit ddr\n");
}

static long bif_compat_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long args)
{
	int value;
	struct bifspi_t *pbif = (struct bifspi_t *)filp->private_data;
	struct bif_acc_reg *pmsg = (struct bif_acc_reg *)args;
	struct bif_acc_space *pspace = (struct bif_acc_space *)args;

	if (_IOC_TYPE(cmd) != MAGIC_NUM) {
		pr_err("Invaild MAGIC_NUM\n");
		return -EINVAL;
	}
	bif_change_reset2gpio();

	switch (cmd) {
	case BIF_GET_SHREG:
		if (pmsg->num > 31) {
			pr_err("Invaild share reg number [%lu]\n", pmsg->num);
			return -EINVAL;
		}
		value = readl((void *)(pbif->regs_base + pmsg->num * 4));
		if (copy_to_user(&pmsg->value, &value, 4) != 0) {
			pr_err("copy to user failed\n");
			return -EINVAL;
		}
		break;
	case BIF_SET_SHREG:
		if (pmsg->num > 30) {
			pr_err("Invaild share reg number [%lu]\n", pmsg->num);
			return -EINVAL;
		}
		if (copy_from_user(&value, &pmsg->value, 4) != 0) {
			pr_err("copy to user failed\n");
			return -EINVAL;
		}
		writel(value, (void *)(pbif->regs_base + pmsg->num * 4));
		break;
	case BIF_SET_ACCESS:
		if (pspace->first > pspace->last
		    || pspace->first > MEM_MAX_ACCESS
		    || pspace->last > MEM_MAX_ACCESS) {
			pr_err("invaild param\n");
			return -EINVAL;
		}
		bif_set_access(pbif, pspace->first, pspace->last);
		break;

	case BIF_UNSET_ACCESS:
		bif_unset_access(pbif);
		break;
	case BIF_RESET:
		bif_reset(pbif);
		break;
	}

	bif_recover_reset_func();
	return 0;
}

static long bif_unlocked_ioctl(struct file *filp, unsigned int cmd,
			       unsigned long args)
{
	// struct inode * pinode = filp->f_inode;
	// _t * pbif = filp->private_data;
	// pr_info("%s-%d\n",__func__, __LINE__);

	return bif_compat_ioctl(filp, cmd, args);
}

int bifspi_read_share_reg(unsigned int num, unsigned int *value)
{
	if (bif_info == NULL) {
		pr_err("%s bifspi is not enabled\n", __func__);
		return -1;
	}
	bif_change_reset2gpio();
	*value = readl((void *)(bif_info->regs_base + num * 4));
	bif_recover_reset_func();
	return 0;
}
EXPORT_SYMBOL(bifspi_read_share_reg);

int bifspi_write_share_reg(unsigned int num, unsigned int value)
{
	if (bif_info == NULL) {
		pr_err("%s bifspi is not enabled\n", __func__);
		return -1;
	}
	bif_change_reset2gpio();
	writel(value, (void *)(bif_info->regs_base + num * 4));
	bif_recover_reset_func();
	return 0;
}
EXPORT_SYMBOL(bifspi_write_share_reg);

static int bif_alloc_chardev(struct bifspi_t *pbif, const char *name)
{
	int ret = 0;
	dev_t devno;

	if (major) {
		devno = MKDEV(major, 0);
		ret = register_chrdev_region(devno, 1, name);
	} else {
		ret = alloc_chrdev_region(&devno, 0, 1, name);
		major = MAJOR(devno);
	}
	if (ret < 0) {
		pr_err("Failed Register char device\n");
		return -1;
	}

	cdev_init(&bif_info->cdev, &fops);
	pbif->cdev.owner = THIS_MODULE;

	ret = cdev_add(&pbif->cdev, devno, 1);
	if (ret < 0) {
		pr_err("Failed cdev add\n");
		goto failed_cdev;
	}

	pbif->class = class_create(THIS_MODULE, name);
	if (!pbif->class) {
		pr_err("Failed  class create\n");
		goto failed_class;
	}
	pbif->device = device_create(pbif->class, NULL, devno, NULL, name);
	if (!pbif->device) {
		pr_err("Failed  device create\n");
		goto failed_device;
	}

	return 0;

	device_destroy(pbif->class, devno);
failed_device:
	class_destroy(pbif->class);
failed_class:
	cdev_del(&pbif->cdev);
failed_cdev:
	unregister_chrdev_region(devno, 1);
	return -1;
}

static void bif_free_chardev(struct bifspi_t *pbif)
{
	dev_t devno;

	devno = MKDEV(major, 0);
	device_destroy(pbif->class, devno);
	class_destroy(pbif->class);
	cdev_del(&pbif->cdev);
	unregister_chrdev_region(devno, 1);
}

static void bifspi_diag_report(uint8_t errsta, uint32_t irqsta)
{
	uint32_t irqstatmp;

	irqstatmp = irqsta;
	bifspi_last_err_tm_ms = jiffies_to_msecs(get_jiffies_64());
	if (errsta) {
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_bif,
				EventIdBifSpiErr,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				(uint8_t *)&irqstatmp,
				4);
	}
}

static void bifspi_diag_timer_func(unsigned long data)
{
	uint32_t now_tm_ms;
	unsigned long jiffi;

	now_tm_ms = jiffies_to_msecs(get_jiffies_64());
	if (now_tm_ms - bifspi_last_err_tm_ms > 6000) {
		diag_send_event_stat(
				DiagMsgPrioMid,
				ModuleDiag_bif,
				EventIdBifSpiErr,
				DiagEventStaSuccess);
	}
	jiffi = get_jiffies_64() + msecs_to_jiffies(2000);
	mod_timer(&bifspi_diag_timer, jiffi); // trriger again.
}

static irqreturn_t bifspi_interrupt(int irq, void *dev_id)
{
	unsigned int value;
	struct bifspi_t *pbif = dev_id;
	unsigned long flags;

	bif_change_reset2gpio();
	// read interrupt
	disable_irq_nosync(irq);
	spin_lock_irqsave(&pbif->lock, flags);
	pbif->intstatus = readl((void *)(pbif->regs_base + BIF_INT_STATE));
	spin_unlock_irqrestore(&pbif->lock, flags);
	pr_info("BIFSPI: Catch interrupt value %#x\n", pbif->intstatus);

	if (pbif->intstatus & BIT(1) ||
		pbif->intstatus & BIT(4) ||
		pbif->intstatus & BIT(5)) {
		bifspi_diag_report(1, pbif->intstatus);
		bif_reset(pbif);
		bif_set_access(pbif, pbif->first, pbif->last);
	}
	// clear int
	spin_lock_irqsave(&pbif->lock, flags);
	value = readl((void *)(pbif->regs_base + BIF_EN_CLEAR));
	value |= BIF_CLR_INT;
	value |= ((pbif->intstatus & 0x3F) << 6);
	writel(value, (void *)(pbif->regs_base + BIF_EN_CLEAR));
	spin_unlock_irqrestore(&pbif->lock, flags);

	enable_irq(irq);
	pbif->intstatus = 0;
	bif_recover_reset_func();

	return 0;
}

static int bifspi_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	unsigned int value32;
	unsigned long flags;
	int value;

	bif_info = kzalloc(sizeof(struct bifspi_t), GFP_KERNEL);
	if (bif_info == NULL) {
		//pr_err("bif_info malloc failed");
		return -ENOMEM;
	}

	bif_info->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(bif_info->pinctrl)) {
		dev_warn(&pdev->dev, "pinctrl get none\n");
		bif_info->pinctrl = NULL;
		bif_info->pins_gpio = NULL;
		bif_info->pins_reset = NULL;
	} else {
		bif_info->pins_gpio = pinctrl_lookup_state(bif_info->pinctrl,
							   "default");
		if (IS_ERR(bif_info->pins_gpio)) {
			dev_warn(&pdev->dev, "bifspi default get error %ld\n",
					PTR_ERR(bif_info->pins_gpio));
			bif_info->pins_gpio = NULL;
		}
		bif_info->pins_reset = pinctrl_lookup_state(bif_info->pinctrl,
							    "reset");
		if (IS_ERR(bif_info->pins_reset)) {
			dev_warn(&pdev->dev, "bifspi reset get error %ld\n",
					PTR_ERR(bif_info->pins_reset));
			bif_info->pins_reset = NULL;
		}
	}

	platform_set_drvdata(pdev, bif_info);
	spin_lock_init(&bif_info->lock);
	bif_info->first = ap_access_first;
	bif_info->last = ap_access_last;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	bif_info->regs_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(bif_info->regs_base)) {
		ret = PTR_ERR(bif_info->regs_base);
		bif_info->regs_base = 0;
		goto bif_free;
	}

	bif_info->irq = platform_get_irq(pdev, 0);
	if (bif_info->irq < 0) {
		ret = -ENXIO;
		dev_err(&pdev->dev, "irq number is invalid\n");
		bif_info->irq = 0;
		goto bif_iounmap;
	}

	ret = devm_request_irq(&pdev->dev, bif_info->irq, bifspi_interrupt,
			       0, pdev->name, bif_info);
	if (ret != 0) {
		ret = -ENXIO;
		dev_err(&pdev->dev, "request_irq failed\n");
		goto bif_iounmap;
	}

	bif_info->bif_rst = devm_reset_control_get(&pdev->dev, BIF_RST_NAME);
	if (IS_ERR(bif_info->bif_rst))
		goto failed_chardev;

	ret = of_property_read_u32(pdev->dev.of_node,
				   "ap_access_first", &value32);
	if (ret)
		dev_err(&pdev->dev, "get ap_access_first failed\n");
	else {
		bif_info->first = value32;
		ret = of_property_read_u32(pdev->dev.of_node,
				   "ap_access_last", &value32);
		if (ret)
			dev_err(&pdev->dev, "get ap_access_last failed\n");
		else {
			bif_info->last = value32;
			writel(bif_info->first, (void *)(bif_info->regs_base
							+ BIF_ACCESS_FIRST));
			writel(bif_info->last, (void *)(bif_info->regs_base
							+ BIF_ACCESS_LAST));
			value = readl((void *)(bif_info->regs_base
							+ BIF_EN_CLEAR));
			value |= (1 << 0x04);
			writel(value, (void *)(bif_info->regs_base
							+ BIF_EN_CLEAR));
		}
	}
	writel(STAGE_KERNEL,
	      (void *)(bif_info->regs_base + 4 * SYS_STATUS_REG));

	ret = bif_alloc_chardev(bif_info, "bifspi");
	if (ret != 0) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "alloc char device failed\n");
		goto failed_chardev;
	}

	/* diag init, enalbe error interrupt
	 * and creator timer for report OK
	 */
	spin_lock_irqsave(&bif_info->lock, flags);
	value = readl((void *)(bif_info->regs_base + BIF_EN_CLEAR));
	value |= (1 << 0x01); // enalbe check_cmd_err int
	value |= (1 << 0x05); // enalbe wr_same_sharereg int
	writel(value, (void *)(bif_info->regs_base + BIF_EN_CLEAR));
	spin_unlock_irqrestore(&bif_info->lock, flags);
	bifspi_last_err_tm_ms = 0;
	if (diag_register(ModuleDiag_bif, EventIdBifSpiErr,
						8, 300, 5000, NULL) < 0) {
		dev_err(&pdev->dev, "bifspi char dev diag register fail\n");
	} else {
		init_timer(&bifspi_diag_timer);
		bifspi_diag_timer.expires = get_jiffies_64()
					+ msecs_to_jiffies(1000);
		bifspi_diag_timer.data = 0;
		bifspi_diag_timer.function = bifspi_diag_timer_func;
		add_timer(&bifspi_diag_timer);
	}
	ret = sysfs_create_group(&pdev->dev.kobj, &bifspi_group);
	if (ret != 0)
		dev_err(&pdev->dev, "sysfs_create_group failed\n");

	bif_recover_reset_func();
	dev_err(&pdev->dev, "Suc %s\n", VER);
	return 0;

failed_chardev:
	if (bif_info->irq > 0)
		devm_free_irq(&pdev->dev, bif_info->irq, bif_info);
bif_iounmap:
	if (bif_info->regs_base > 0)
		devm_iounmap(&pdev->dev, bif_info->regs_base);
bif_free:
	kfree(bif_info);
	dev_err(&pdev->dev, "Err %s\n", VER);
	return -EINVAL;
}

static int bifspi_remove(struct platform_device *pdev)
{
	struct bifspi_t *pbif;

	sysfs_remove_group(&pdev->dev.kobj, &bifspi_group);
	pbif = platform_get_drvdata(pdev);
	bif_free_chardev(pbif);
	if (pbif->irq > 0)
		devm_free_irq(&pdev->dev, pbif->irq, pbif);
	if (pbif->regs_base > 0)
		devm_iounmap(&pdev->dev, bif_info->regs_base);

	kfree(pbif);
	del_timer_sync(&bifspi_diag_timer);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
int x2_bif_spi_suspend(struct device *dev)
{
	struct bifspi_t *bifspi = dev_get_drvdata(dev);

	pr_info("%s:%s, enter suspend...\n", __FILE__, __func__);
	bif_change_reset2gpio();
	g_bif_spi_regs[0] = readl(bifspi->regs_base + BIF_ACCESS_FIRST);
	g_bif_spi_regs[1] = readl(bifspi->regs_base + BIF_ACCESS_LAST);
	g_bif_spi_regs[2] = readl(bifspi->regs_base + BIF_EN_CLEAR);

	//bif_unset_access(bifspi);
	bif_recover_reset_func();
	return 0;
}

int x2_bif_spi_resume(struct device *dev)
{
	struct bifspi_t *bifspi = dev_get_drvdata(dev);

	pr_info("%s:%s, enter resume...\n", __FILE__, __func__);

	bif_reset(bifspi);
	bif_change_reset2gpio();
	writel(g_bif_spi_regs[0], bifspi->regs_base + BIF_ACCESS_FIRST);
	writel(g_bif_spi_regs[1], bifspi->regs_base + BIF_ACCESS_LAST);
	writel(g_bif_spi_regs[2], bifspi->regs_base + BIF_EN_CLEAR);
	bif_recover_reset_func();
	return 0;
}
#endif

static const struct dev_pm_ops x2_bif_spi_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(x2_bif_spi_suspend,
			x2_bif_spi_resume)
};

static const struct of_device_id bifspi_hobot_of_match[] = {
	/* SoC-specific compatible strings w/ soc_ctl_map */
	{.compatible = "hobot,x2-bifspi"},
	{.compatible = "hobot,hobot-bifspi"},
	{ /* end of table */ }
};

static struct platform_driver bifspi_hobot_driver = {
	.driver = {
		   .name = "bifspi",
		   .of_match_table = bifspi_hobot_of_match,
		   .pm = &x2_bif_spi_dev_pm_ops,
		   },
	.probe = bifspi_probe,
	.remove = bifspi_remove,
};

static int __init x2_bifspi_init(void)
{
	int retval = 0;

	/* Register the platform driver */
	retval = platform_driver_register(&bifspi_hobot_driver);
	if (retval)
		pr_err("hobot bifspi driver register failed\n");

	return retval;
}

static void __exit x2_bifspi_exit(void)
{
	/* Unregister the platform driver */
	platform_driver_unregister(&bifspi_hobot_driver);
}

module_init(x2_bifspi_init);
module_exit(x2_bifspi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hobot Inc.");
MODULE_DESCRIPTION("Driver for the HobotRobotics Bifspi Controller");
MODULE_VERSION("2.0");
