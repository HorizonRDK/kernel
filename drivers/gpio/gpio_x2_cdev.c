/***************************************************************************
 *						COPYRIGHT NOTICE
 *			   Copyright 2018 Horizon Robotics, Inc.
 *					   All rights reserved.
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
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/slab.h>

struct gpio_cdev_s {
	const char *name;
	int major;
	int minor;
	struct cdev cdev;
	dev_t dev_num;
	struct class *gpio_classes;
};
struct gpio_cdev_s *g_gpio_cdev;

typedef struct gpio_cfg {
	int gpio;
	int value;
} gpio_cfg_t;

#define GPIO_CDEV_MAGIC 'G'
#define GPIO_REQUEST		 _IOWR(GPIO_CDEV_MAGIC,0x11, unsigned int)
#define GPIO_RELEASE		 _IOWR(GPIO_CDEV_MAGIC,0x12, unsigned int)

#define GPIO_GETVALUE		 _IOWR(GPIO_CDEV_MAGIC,0x13, gpio_cfg_t)
#define GPIO_SETVALUE		 _IOWR(GPIO_CDEV_MAGIC,0x14, gpio_cfg_t)

static int gpio_cdev_open(struct inode *inode, struct file *filp)
{
	struct gpio_cdev_s *gpiocdev_p;

	gpiocdev_p = container_of(inode->i_cdev, struct gpio_cdev_s, cdev);
	filp->private_data = &gpiocdev_p;
	return 0;
}

static long gpio_cdev_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long p)
{
	int ret;
	void __user *arg = (void __user *)p;
	switch (cmd) {
	case GPIO_REQUEST:
		{
			int gpio = 0;
			if (!arg
			    || copy_from_user(&gpio, (const char *)arg,
					      sizeof(gpio)))
				return -EFAULT;
			gpio_request(gpio, NULL);
			return 0;
		}
		break;
	case GPIO_RELEASE:
		{
			int gpio = 0;
			if (!arg
			    || copy_from_user(&gpio, (const char *)arg,
					      sizeof(gpio)))
				return -EFAULT;
			gpio_free(gpio);
			return 0;
		}
		break;
	case GPIO_SETVALUE:
		{
			gpio_cfg_t g_cfg;
			if (!arg
			    || copy_from_user(&g_cfg, (const char *)arg,
					      sizeof(gpio_cfg_t)))
				return -EFAULT;

			gpio_direction_output(g_cfg.gpio, g_cfg.value);
			gpio_set_value(g_cfg.gpio, g_cfg.value);
			return 0;
		}
		break;
	case GPIO_GETVALUE:
		{
			gpio_cfg_t g_cfg;
			if (!arg
			    || copy_from_user(&g_cfg, (const char *)arg,
					      sizeof(gpio_cfg_t)))
				return -EFAULT;
			gpio_direction_input(g_cfg.gpio);
			g_cfg.value = gpio_get_value(g_cfg.gpio);

			if (copy_to_user
			    (arg, (const void *)&g_cfg, sizeof(gpio_cfg_t)))
				return 0;
		}
		break;
	}
	return ret;
}

static ssize_t gpio_cdev_write(struct file *filp, const char __user * ubuf,
			       size_t len, loff_t * ppos)
{
	return len;
}

static ssize_t gpio_cdev_read(struct file *filp, char __user * ubuf,
			      size_t len, loff_t * offp)
{
	unsigned int size = 0;

	return size;
}

int gpio_cdev_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

static const struct file_operations gpio_cdev_ops = {
	.owner = THIS_MODULE,
	.open = gpio_cdev_open,
	.release = gpio_cdev_release,
	.write = gpio_cdev_write,
	.read = gpio_cdev_read,
	.unlocked_ioctl = gpio_cdev_ioctl,
};

struct kobject *x2_gpio_kobj;
static ssize_t x2_gpio_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	char *s = buf;
	//int i = 0;

	return (s - buf);
}

static ssize_t x2_gpio_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t n)
{
	int error = -EINVAL;
	return error ? error : n;
}

static struct kobj_attribute gpio_test_attr = {
	.attr = {
		 .name = __stringify(gpio_test_attr),
		 .mode = 0644,
		 },
	.show = x2_gpio_show,
	.store = x2_gpio_store,
};

static struct attribute *attributes[] = {
	&gpio_test_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attributes,
};

int __init gpio_cdev_init(void)
{
	int error;
	int ret;

	g_gpio_cdev = kmalloc(sizeof(struct gpio_cdev_s), GFP_KERNEL);
	if (!g_gpio_cdev) {
		printk(KERN_ERR "Unable to alloc GPIO DEV\n");
		return -ENOMEM;
	}
	g_gpio_cdev->name = "gpio_cdev";
	g_gpio_cdev->gpio_classes =
	    class_create(THIS_MODULE, g_gpio_cdev->name);
	if (IS_ERR(g_gpio_cdev->gpio_classes))
		return PTR_ERR(g_gpio_cdev->gpio_classes);

	error =
	    alloc_chrdev_region(&g_gpio_cdev->dev_num, 0, 1, g_gpio_cdev->name);
	if (!error) {
		g_gpio_cdev->major = MAJOR(g_gpio_cdev->dev_num);
		g_gpio_cdev->minor = MINOR(g_gpio_cdev->dev_num);
	}

	cdev_init(&g_gpio_cdev->cdev, &gpio_cdev_ops);

	error = cdev_add(&g_gpio_cdev->cdev, g_gpio_cdev->dev_num, 1);
	if (error) {
		unregister_chrdev_region(g_gpio_cdev->dev_num, 1);
		return error;
	}

	ret =
	    device_create(g_gpio_cdev->gpio_classes, NULL, g_gpio_cdev->dev_num,
			  NULL, g_gpio_cdev->name);

	x2_gpio_kobj = kobject_create_and_add("x2_gpio", NULL);
	if (!x2_gpio_kobj)
		return -ENOMEM;
	return sysfs_create_group(x2_gpio_kobj, &attr_group);

	if (ret)
		return ret;

	return 0;
}

void __exit gpio_cdev_exit(void)
{
	device_destroy(g_gpio_cdev->gpio_classes, g_gpio_cdev->dev_num);
	class_destroy(g_gpio_cdev->gpio_classes);
	cdev_del(&g_gpio_cdev->cdev);
	unregister_chrdev_region(g_gpio_cdev->dev_num, 1);

	if (x2_gpio_kobj) {
		sysfs_remove_group(x2_gpio_kobj, &attr_group);
		kobject_del(x2_gpio_kobj);
	}
}

module_init(gpio_cdev_init);
module_exit(gpio_cdev_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: x2");
