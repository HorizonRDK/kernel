/*************************************************************************
	> File Name: bif.c
	> Author: baiy
	> Mail: yang01.bai@hobot.cc
	> Created Time: 2018-07-27-16:56:21
	> Func: x2 bif spi driver: r/w register
 ************************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/reset.h>

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <linux/interrupt.h>

#define BIF_BASE_ADDRESS  (0x00000000)
#define BIF_ACCESS_FIRST  (0x00000110)
#define BIF_ACCESS_LAST   (0x00000114)
#define BIF_EN_CLEAR	  (0x00000104)
#define BIF_INT_STATE     (0x00000108)
#define MEM_MAX_ACCESS	  (0xfffffffc)
#define BIF_CLR_INT 	  (0x0FC0)
#define BIF_RST_NAME	  "bifspi"

static int bif_open(struct inode *inode, struct file *filp);
static int bif_release(struct inode *inode, struct file *filp);
static ssize_t bif_read(struct file *filp, char __user * buf, size_t count,
			loff_t * offset);
static ssize_t bif_write(struct file *filp, const char __user * buf,
			 size_t count, loff_t * offset);
static long bif_unlocked_ioctl(struct file *filp, unsigned int cmd,
			       unsigned long args);
static long bif_compat_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long args);

static int major = 0;
module_param(major, int, 0644);
MODULE_PARM_DESC(major, "major num");

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
};
struct bifspi_t *bif_info;

struct file_operations fops = {
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
#define BIF_GET_SHREG 		_IO(MAGIC_NUM,1)
#define BIF_SET_SHREG		_IO(MAGIC_NUM,2)
#define BIF_SET_ACCESS	    _IO(MAGIC_NUM,3)
#define BIF_UNSET_ACCESS    _IO(MAGIC_NUM,4)
#define BIF_RESET           _IO(MAGIC_NUM,5)

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

static ssize_t bif_read(struct file *filp, char __user * buf, size_t count,
			loff_t * offset)
{
	// struct bifspi_t * pbif  = filp->private_data;

	return count;
}

static ssize_t bif_write(struct file *filp, const char __user * buf,
			 size_t count, loff_t * offset)
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

static void bif_set_access(struct bifspi_t *pbif, unsigned int first,
			   unsigned int last)
{
	int value;
	unsigned long flags;

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

	pr_info
	    ("bif set access limit ddr:start add:%p->%#x(%d)  end add:%p-%#x(%d)\n",
	     (void *)(pbif->regs_base + BIF_ACCESS_FIRST), first, first,
	     (void *)(pbif->regs_base + BIF_ACCESS_LAST), last, last);
}

static void bif_unset_access(struct bifspi_t *pbif)
{
	int en_val = 0;
	unsigned long flags;

	spin_lock_irqsave(&pbif->lock, flags);
	writel(0, (void *)(pbif->regs_base + BIF_ACCESS_FIRST));
	writel(MEM_MAX_ACCESS, (void *)(pbif->regs_base + BIF_ACCESS_LAST));
	en_val &= (~(1 << 0x04));
	writel(en_val, (void *)(pbif->regs_base + BIF_EN_CLEAR));

	pbif->first = 0;
	pbif->last = MEM_MAX_ACCESS;
	spin_unlock_irqrestore(&pbif->lock, flags);

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

static irqreturn_t bifspi_interrupt(int irq, void *dev_id)
{
	unsigned int value;
	struct bifspi_t *pbif = dev_id;
	unsigned long flags;

	// read interrupt
	disable_irq_nosync(irq);
	spin_lock_irqsave(&pbif->lock, flags);
	pbif->intstatus = readl((void *)(pbif->regs_base + BIF_INT_STATE));
	spin_unlock_irqrestore(&pbif->lock, flags);
	pr_info("BIFSPI: Catch interrupt value %#x\n", pbif->intstatus);

	if (pbif->intstatus & BIT(4)) {
		bif_reset(pbif);
		bif_set_access(pbif, pbif->first, pbif->last);
	}
	// clear int
	spin_lock_irqsave(&pbif->lock, flags);
	value = readl((void *)(pbif->regs_base + BIF_EN_CLEAR));
	value |= BIF_CLR_INT;
	writel(value, (void *)(pbif->regs_base + BIF_EN_CLEAR));
	spin_unlock_irqrestore(&pbif->lock, flags);

	enable_irq(irq);
	pbif->intstatus = 0;

	return 0;
}

static int bifspi_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;

	bif_info = kzalloc(sizeof(struct bifspi_t), GFP_KERNEL);
	if (bif_info == NULL) {
		pr_err("bif_info malloc failed");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, bif_info);
	spin_lock_init(&bif_info->lock);
	bif_info->first = 0;
	bif_info->last = MEM_MAX_ACCESS;

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
	if (IS_ERR(bif_info->bif_rst)) {
		goto failed_chardev;
	}

	ret = bif_alloc_chardev(bif_info, "bifspi");
	if (ret != 0) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "alloc char device failed\n");
		goto failed_chardev;
	}

	pr_info("BIF_INIT FINISHED\n");
	return 0;

failed_chardev:
	if (bif_info->irq > 0)
		devm_free_irq(&pdev->dev, bif_info->irq, bif_info);
bif_iounmap:
	if (bif_info->regs_base > 0)
		devm_iounmap(&pdev->dev, bif_info->regs_base);
bif_free:
	if (bif_info)
		kfree(bif_info);
	return -EINVAL;
}

static int bifspi_remove(struct platform_device *pdev)
{
	struct bifspi_t *pbif;

	pbif = platform_get_drvdata(pdev);
	bif_free_chardev(pbif);
	if (pbif->irq > 0)
		devm_free_irq(&pdev->dev, pbif->irq, pbif);
	if (pbif->regs_base > 0)
		devm_iounmap(&pdev->dev, bif_info->regs_base);
	if (pbif)
		kfree(pbif);
	return 0;
}

static const struct of_device_id bifspi_hobot_of_match[] = {
	/* SoC-specific compatible strings w/ soc_ctl_map */
	{.compatible = "hobot,x2-bifspi",},
	{ /* end of table */ }
};

static struct platform_driver bifspi_hobot_driver = {
	.driver = {
		   .name = "bifspi",
		   .of_match_table = bifspi_hobot_of_match,
		   },
	.probe = bifspi_probe,
	.remove = bifspi_remove,
};

module_platform_driver(bifspi_hobot_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("baiy <yang01.bai@horizon,ai>");
MODULE_DESCRIPTION("Driver for the HobotRobotics Bifspi Controller");
MODULE_VERSION("1.0.0.0");
