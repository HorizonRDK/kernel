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
#include <soc/hobot/diag.h>
#include "diag_dev.h"

//#define DEBUG

static struct class  *g_diag_dev_class;
struct device *g_diag_dev;
static int diag_dev_ver[2] __initdata = {1, 1};

ssize_t test_enable_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return strlen(buf);
}

static void enable_callback(uint16_t module_id, uint16_t event_id)
{
	struct diag_msg_id reg_id;
	struct id_register_struct *regisid = NULL;
	uint8_t *paylod = NULL;
	uint32_t paylodlen = 0;

	reg_id.module_id = module_id;
	reg_id.event_id = event_id;
	regisid = diag_id_in_register_list(&reg_id);
	if (regisid == NULL)
		return;
	if (regisid->msg_rcvcallback)
		regisid->msg_rcvcallback(paylod, paylodlen);
	else
		pr_debug("module_id:%d, event_id:%d callback is NULL\n",
				reg_id.module_id, reg_id.event_id);
}

ssize_t test_enable_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	enable_callback(ModuleDiag_eth, EventIdEthDmaBusErr);
	enable_callback(ModuleDiag_bpu, EventIdBpu0Err);
	enable_callback(ModuleDiag_bpu, EventIdBpu1Err);
	return count;
}

static struct class_attribute eth_attribute =
	__ATTR(test_enable, 0644, test_enable_show, test_enable_store);
static struct attribute *diag_attributes[] = {
	&eth_attribute.attr,
	NULL
};

static const struct attribute_group diag_group = {
	.attrs = diag_attributes,
};

static const struct attribute_group *diag_attr_group[] = {
	&diag_group,
	NULL,
};

static struct class diag_class = {
	.name = "diag_test",
	.class_groups = diag_attr_group,
};

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
	int retval;
	unsigned char *envdata;
	size_t count;
	//unsigned long flags;
	uint16_t buf[2];
	uint8_t op;
	struct diag_msg_id_unmask_struct *p;
	struct diag_msg_id unmaskid;
	//uint8_t ready_flag;

	mutex_lock(&diag_dev_ioctl_mutex);
	switch (cmd) {
	case IOCS_DIAG_DEV_SELF_TEST:
		count = 2*FRAGMENT_SIZE;
		envdata = vmalloc(count);
		if (envdata == NULL) {
			ret = -EFAULT;
			pr_err("diag dev ioctrl vmalloc error\n");
			goto self;
		}
		get_random_bytes(envdata, count);

		/* send image frame error and it's env data.*/
		ret = diag_send_event_stat_and_env_data(DiagMsgPrioMid,
					ModuleDiagDriver, EventIdKernelToUserSelfTest,
					DiagEventStaFail, DiagGenEnvdataWhenErr,
					envdata, count
					);
		if (ret < 0){
			pr_err("diag dev snd env data error,due to:%d\n", ret);
			ret = -EFAULT;
			goto self;
		}
		retval = wait_for_completion_timeout(&diag_dev_completion,
			5*HZ);
		if (!retval) {
			ret = -EFAULT;
			pr_err("diag dev:completion timeout\n");
			goto self;
		}
self:
		if (envdata)
			vfree(envdata);
		break;

	case IOCS_DIAG_DEV_UNMASK_ID:
		//spin_lock_irqsave(&diag_id_unmask_list_spinlock, flags);
		memset((uint8_t *)buf, 0x00, sizeof(buf));
		if (copy_from_user(buf, (uint8_t __user *)arg, 4)) {
			pr_err("diag dev: IOCS_DIAG_DEV_MASK_ID copy from user error\n");
			ret = -EINVAL;
			goto mask_id;
		}
		if ((buf[0] > ModuleIdMax) || (buf[1] > EVENT_ID_MAX)) {
			pr_err("diag dev: IOCS_DIAG_DEV_MASK_ID moudle id or event id invalid!!!\n");
			ret = -EINVAL;
			goto mask_id;
		}
		unmaskid.module_id = buf[0];
		unmaskid.event_id = buf[1];
		if (diag_unmask_id_in_list(&unmaskid)) {
			pr_debug("moduleid:%d, eventid:%d had exsit in unmask list\n",
					buf[0], buf[1]);
			goto mask_id;
		}
		if (diag_id_unmask_list_num > DIAG_UNMASK_ID_MAX_NUM) {
			pr_err("diag dev: diag unmask id num over range\n");
			ret = -EINVAL;
			goto mask_id;
		}
		p = (struct diag_msg_id_unmask_struct *)kmalloc(
			sizeof(struct diag_msg_id_unmask_struct), GFP_ATOMIC);
		if (!p) {
			pr_err("diag dev: IOCS_DIAG_DEV_MASK_ID kmalloc fail\n");
			ret = -EINVAL;
			goto mask_id;
		}
		p->module_id = buf[0];
		p->event_id = buf[1];
		list_add_tail(&(p->mask_lst), &diag_id_unmask_list);
		diag_id_unmask_list_num++;
		pr_debug("module id:%x event id:%x had add to unmaskid list\n",
				 buf[0], buf[1]);
mask_id:
		//spin_unlock_irqrestore(&diag_id_unmask_list_spinlock, flags);
		break;

	case IOCS_DIAGDRIVER_STA:
		if (copy_from_user((uint8_t *)&op, (uint8_t __user *)arg, 1)) {
			pr_err("diag dev: IOCS_DIAG_STATUS copy from user error\n");
			ret = -EINVAL;
			goto diag_sta;
		}
		if (op == (uint8_t)DIAGDRIVER_STOPWORK) {
			diag_app_ready -= 1;
			if (diag_app_ready < 0)
				diag_app_ready = 0;
			if (diag_app_ready == 0) {
				INIT_LIST_HEAD(&diag_id_unmask_list);
				diag_id_unmask_list_num = 0;
			}
		} else if (op == (uint8_t)DIAGDRIVER_STARTWORK)
			diag_app_ready += 1;

diag_sta:
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&diag_dev_ioctl_mutex);
	return ret;
}

static int diag_dev_close(struct inode *inode, struct file *file)
{
	return 0;
}

static void daig_dev_driver_selftest_event_callback(void *p, size_t len)
{
	pr_info("enter diag driver selftest callback\n");
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
	pr_debug("diag dev init enter\n");
	ret = alloc_chrdev_region(&devno, 0, 1, "diag dev");
	if (ret < 0) {
		pr_err("Error %d while alloc chrdev diag dev", ret);
		goto alloc_chrdev_error;
	}
	diag_dev_major = MAJOR(devno);
	cdev_init(p_cdev, &diag_dev_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		pr_err("Error %d while adding example diag cdev", ret);
		goto cdev_add_error;
	}
	g_diag_dev_class = class_create(THIS_MODULE, "diag_dev");
	if (IS_ERR(g_diag_dev_class)) {
		pr_err("[%s:%d] class_create error\n",
			__func__, __LINE__);
		ret = PTR_ERR(g_diag_dev_class);
		goto class_create_error;
	}
	g_diag_dev = device_create(g_diag_dev_class, NULL,
		MKDEV(diag_dev_major, 0), NULL, "diag_dev");
	if (IS_ERR(g_diag_dev)) {
		pr_err("[%s] device create error\n", __func__);
		ret = PTR_ERR(g_diag_dev);
		goto device_create_error;
	}

	init_completion(&diag_dev_completion);
	ret = diag_register(ModuleDiagDriver, EventIdKernelToUserSelfTest, 2*FRAGMENT_SIZE,
		20, 2000, daig_dev_driver_selftest_event_callback);
	if (ret < 0)
		pr_err("[%s] diag driver register fail\n", __func__);

	diag_netlink_init();
	pr_info("diag dev [ver:%d.%d] init done\n",
			diag_dev_ver[0], diag_dev_ver[1]);

	ret = class_register(&diag_class);
	if(ret < 0)
		pr_err("diag sys register error\n");
	pr_err("after diag sys register success\n");
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
	pr_info("diag dev exit\n");
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
