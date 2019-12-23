/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <linux/mm.h>
#include <linux/types.h>
#include <linux/major.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <asm/io.h>

#include "soc/hobot/hobot_ips_x2.h"

#define IPS_CDEV_MAGIC 'I'
#define IPS_GETVALUE	_IOR(IPS_CDEV_MAGIC,0x11, unsigned int)
#define IPS_SETVALUE	_IOW(IPS_CDEV_MAGIC,0x12, unsigned int)
#define IPS_GETSTATUS	_IOWR(IPS_CDEV_MAGIC,0x13, unsigned int)
#define IPS_BUSCTL_SET	_IOW(IPS_CDEV_MAGIC,0x14, cmdinfo_t)

typedef struct _cmdinfo_t {
	unsigned int type;
	unsigned int index;
	unsigned int region;
	unsigned int value;
} cmdinfo_t;
struct ips_cdev_s {
	const char *name;
	int major;
	int minor;
	struct cdev cdev;
	dev_t dev_num;
	struct class *ips_classes;
};

struct ips_cdev_s g_ipscdev = {
	.name = "ips_cdev",
};

static int ips_cdev_open(struct inode *inode, struct file *filp)
{
	//dev_t device = inode->i_rdev;
	struct ips_cdev_s *ipscdev_p;

	ipscdev_p = container_of(inode->i_cdev, struct ips_cdev_s, cdev);
	filp->private_data = ipscdev_p;

	return 0;
}

static long ips_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long p)
{
	int ret;
	unsigned int result, region;
	cmdinfo_t setcmd;
	void __user *arg = (void __user *)p;
	switch (cmd) {
	case IPS_GETSTATUS:
		ret = copy_from_user(&region, arg, sizeof(region));
		result = ips_get_status(region);
		ret |= copy_to_user(arg, &result, sizeof(result));
		return ret;
	case IPS_BUSCTL_SET:
		ret = copy_from_user(&setcmd, arg, sizeof(setcmd));
		ips_busctl_set(setcmd.type, setcmd.index, setcmd.region, setcmd.value);
		return ret;
	default:
		return -EPERM;
	}

	return 0;
}

static ssize_t ips_cdev_write(struct file *filp, const char __user *ubuf,
							  size_t len, loff_t *ppos)
{
	return len;
}

static ssize_t ips_cdev_read(struct file *filp, char __user *ubuf,
							 size_t len, loff_t *offp)
{
	return 0;
}


int ips_cdev_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;

	return 0;
}

static const struct file_operations ips_cdev_ops = {
	.owner		= THIS_MODULE,
	.open		= ips_cdev_open,
	.release	= ips_cdev_release,
	.write		= ips_cdev_write,
	.read		= ips_cdev_read,
	.unlocked_ioctl = ips_cdev_ioctl,
	.compat_ioctl = ips_cdev_ioctl,
};

int __init ips_cdev_init(void)
{
	int error;
	struct device *dev = NULL;
	struct ips_cdev_s *ipsdev;

	ipsdev = &g_ipscdev;

	ipsdev->ips_classes = class_create(THIS_MODULE, ipsdev->name);
	if (IS_ERR(ipsdev->ips_classes))
		return PTR_ERR(ipsdev->ips_classes);

	error = alloc_chrdev_region(&ipsdev->dev_num, 0, 1, ipsdev->name);
	if (!error) {
		ipsdev->major = MAJOR(ipsdev->dev_num);
		ipsdev->minor = MINOR(ipsdev->dev_num);
	}

	cdev_init(&ipsdev->cdev, &ips_cdev_ops);

	error = cdev_add(&ipsdev->cdev, ipsdev->dev_num, 1);
	if (error) {
		unregister_chrdev_region(ipsdev->dev_num, 1);
		return error;
	}

	dev = device_create(ipsdev->ips_classes, NULL, ipsdev->dev_num, NULL, ipsdev->name);
	if (IS_ERR(dev))
		return PTR_ERR(dev);

	return 0;
}

void __exit ips_cdev_exit(void)
{
	device_destroy(g_ipscdev.ips_classes, g_ipscdev.dev_num);
	class_destroy(g_ipscdev.ips_classes);
	cdev_del(&g_ipscdev.cdev);
	unregister_chrdev_region(g_ipscdev.dev_num, 1);
}

module_init(ips_cdev_init);
module_exit(ips_cdev_exit);

MODULE_DESCRIPTION("X2 IPS Interface");
MODULE_ALIAS("platform:x2-ips");
MODULE_LICENSE("GPL");
