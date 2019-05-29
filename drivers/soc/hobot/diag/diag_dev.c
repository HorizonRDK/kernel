#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/eventpoll.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/param.h>
#include <linux/random.h>
#include <x2/diag.h>
#include "diag_dev.h"

#define DEBUG

#ifdef DEBUG
#define diag_dev_debug printk
#else
#define diag_dev_debug(format, ...) do {} while (0)
#endif

#define diag_dev_error printk

static struct class  *g_diag_dev_class;
struct device *g_diag_dev;

static DEFINE_MUTEX(diag_dev_open_mutex);
static int diag_dev_open(struct inode *inode, struct file *file)
{
	//mutex_lock(&diag_dev_open_mutex);
	//do something here.
	//mutex_unlock(&diag_dev_open_mutex);

	return 0;
}

static DEFINE_MUTEX(diag_dev_write_mutex);
static ssize_t diag_dev_write(struct file *file, const char __user *buf,
size_t count, loff_t *ppos)
{
	return 0;
}

static DEFINE_MUTEX(diag_dev_read_mutex);
static ssize_t diag_dev_read(struct file *file, char __user *buf, size_t size,
loff_t *ppos)
{
	return 0;
}

static DEFINE_MUTEX(diag_dev_ioctl_mutex);
static long diag_dev_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	int ret = 0;
	unsigned char *envdata;
	size_t count;

	mutex_lock(&diag_dev_ioctl_mutex);

	switch (cmd) {
	case IOC_IOC_DIAG_DEV_SELF_TEST:
		count = 2*FRAGMENT_SIZE;
		envdata = vmalloc(count);
		if (envdata == NULL) {
			ret = -EFAULT;
			diag_dev_error("diag dev ioctrl vmalloc error\n");
			goto error;
		}

		get_random_bytes(envdata, count);

		/*
		 * send image frame error and it's env data.
		 */
		if (diag_send_event_stat_and_env_data(
					DIAG_DEV_KERNEL_TO_USER_SELFTEST,
					DIAG_EVENT_FAIL,
					GEN_ENV_DATA_WHEN_ERROR, envdata, count
					) < 0) {
			ret = -EFAULT;
			diag_dev_error("diag dev snd env data error\n");
			goto error;
		}

		ret = wait_for_completion_timeout(
			&diag_dev_completion,
			5*HZ);
		if (!ret) {
			ret = -EFAULT;
			diag_dev_error("diag dev:completion timeout\n");
			goto error;
		}

		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&diag_dev_ioctl_mutex);
	return 0;

error:
	if (envdata)
		vfree(envdata);

	mutex_unlock(&diag_dev_ioctl_mutex);
	return ret;
}

static int diag_dev_close(struct inode *inode, struct file *file)
{
	return 0;
}

static const  struct file_operations diag_dev_fops = {
	.owner		=	THIS_MODULE,
	.open		=	diag_dev_open,
	.write		=	diag_dev_write,
	.read		=	diag_dev_read,
	.release	=	diag_dev_close,
	.unlocked_ioctl	=	diag_dev_ioctl,
	.compat_ioctl	=	diag_dev_ioctl,
};

static int    diag_dev_major;
struct cdev   diag_dev_cdev;
int  diag_dev_init(void)
{
	int           ret = 0;
	dev_t         devno;
	struct cdev  *p_cdev = &diag_dev_cdev;

	diag_dev_major = 0;
	diag_dev_debug("diag dev init enter\n");
	ret = alloc_chrdev_region(&devno, 0, 1, "diag dev");
	if (ret < 0) {
		diag_dev_error("Error %d while alloc chrdev diag dev", ret);
		goto alloc_chrdev_error;
	}
	diag_dev_major = MAJOR(devno);
	cdev_init(p_cdev, &diag_dev_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		diag_dev_error("Error %d while adding example diag cdev", ret);
		goto cdev_add_error;
	}
	g_diag_dev_class = class_create(THIS_MODULE, "diag_dev");
	if (IS_ERR(g_diag_dev_class)) {
		diag_dev_error("[%s:%d] class_create error\n",
			__func__, __LINE__);
		ret = PTR_ERR(g_diag_dev_class);
		goto class_create_error;
	}
	g_diag_dev = device_create(g_diag_dev_class, NULL,
		MKDEV(diag_dev_major, 0), NULL, "diag_dev");
	if (IS_ERR(g_diag_dev)) {
		diag_dev_error("[%s] device create error\n", __func__);
		ret = PTR_ERR(g_diag_dev);
		goto device_create_error;
	}

	init_completion(&diag_dev_completion);
	diag_netlink_init();
	diag_dev_debug("diag dev init exit\n");

	return 0;

device_create_error:
	class_destroy(g_diag_dev_class);
class_create_error:
	cdev_del(&diag_dev_cdev);
cdev_add_error:
	unregister_chrdev_region(MKDEV(diag_dev_major, 0), 1);
alloc_chrdev_error:
		return ret;
}

void diag_dev_release(void)
{
	diag_dev_debug("diag dev exit\n");
	device_destroy(g_diag_dev_class, MKDEV(diag_dev_major, 0));
	class_destroy(g_diag_dev_class);
	cdev_del(&diag_dev_cdev);
	unregister_chrdev_region(MKDEV(diag_dev_major, 0), 1);
	diag_netlink_exit();
}

module_init(diag_dev_init);
module_exit(diag_dev_release);

MODULE_AUTHOR("bo01.chen@horizon.ai");
MODULE_DESCRIPTION("diag netlink module");
MODULE_LICENSE("GPL");
