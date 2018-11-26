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

#include "x2/x2_ips.h"
#include "x2/x2_sif.h"
#ifdef CONFIG_X2_SIF_DEV
#include "x2_sif_dev.h"
#endif
#include "x2_sif_utils.h"

/*reg driver info*/
#define X2_SIF_MAJOR 253
/*reg mode attr*/

typedef enum _sif_state_e {
	SIF_STATE_DEFAULT = 0,
	SIF_STATE_INIT,
	SIF_STATE_START,
	SIF_STATE_STOP,
	SIF_STATE_MAX,
} sif_state_t;

typedef struct sif_file_s {
	spinlock_t event_lock;
	uint32_t event;
	uint32_t receive_frame;
	wait_queue_head_t event_queue;
} sif_file_t;

typedef struct sif_s {
	sif_cfg_t config;
	sif_state_t state;	/* sif state */
	struct sock *nl_sk;
	uint32_t usr_pid;
	sif_file_t sif_file;
} sif_t;

static struct class *g_sif_class;
static struct device *g_sif_dev;

#define sifdrv(f) (dev_get_drvdata((f)->private_data))

static int sif_start(sif_t * dev)
{
	int ret = 0;
#ifdef CONFIG_X2_SIF_DEV
	if (0 != (ret = sif_dev_start())) {
		siferr("ERROR: sif dev start error: %d", ret);
		return ret;
	}
#endif
	ips_irq_enable(SIF_INT);
	return ret;
}

static int sif_stop(sif_t * dev)
{
	int ret = 0;
	unsigned long flags;
	ips_irq_disable(SIF_INT);
	spin_lock_irqsave(&dev->sif_file.event_lock, flags);
	dev->sif_file.event = SIF_STOP;
	dev->sif_file.receive_frame = false;
	spin_unlock_irqrestore(&dev->sif_file.event_lock, flags);
	wake_up_interruptible(&dev->sif_file.event_queue);
#ifdef CONFIG_X2_SIF_DEV
	sif_dev_stop();
#endif
	return ret;
}

static void x2_sif_irq(unsigned int status, void *data)
{
	sif_t *sif = NULL;
	if (NULL == data) {
		siferr("sif irq input data error!");
		return;
	}
	sif = (sif_t *) data;
	//TODO
	spin_lock(&sif->sif_file.event_lock);
	if (!sif->sif_file.receive_frame && (status & SIF_FRAME_END_INTERRUPT)) {
		sif->sif_file.event |= SIF_START;
		sif->sif_file.receive_frame = true;
	}
	if (status & (SIF_SIZE_ERR0 | SIF_SIZE_ERR1))
		sif->sif_file.event |= SIF_ERROR;
	if (status & MOT_DET)
		sif->sif_file.event |= SIF_MOTDET;
	spin_unlock(&sif->sif_file.event_lock);
	if (sif->sif_file.event)
		wake_up_interruptible(&sif->sif_file.event_queue);
}

static int sif_init(sif_t * dev, sif_cfg_t * cfg)
{
	int ret = 0;
	memset(&dev->config, 0, sizeof(sif_cfg_t));
	memcpy(&dev->config, cfg, sizeof(sif_cfg_t));
	sifinfo("sif config format: %d pixlen: %d bus: %d width: %d height: %d",
		cfg->sif_init.format,
		cfg->sif_init.pix_len,
		cfg->sif_init.bus_type,
		cfg->sif_init.width, cfg->sif_init.height);
	if (BUS_TYPE_DVP == cfg->sif_init.bus_type) {
		ips_pinmux_dvp();
	} else if (BUS_TYPE_BT1120 == cfg->sif_init.bus_type) {
		ips_pinmux_bt();
		if (cfg->sif_init.bypass_en)
			ips_set_btout_clksrc(BYPASS_CLK);
	}
#ifdef CONFIG_X2_SIF_DEV
	if (0 != (ret = sif_dev_init(&dev->config.sif_init))) {
		siferr("ERROR: sif dev init error: %d", ret);
		ret = -1;
		return ret;
	}
	if (dev->config.mot_det.enable) {
		if (0 != (ret = sif_dev_mot_det_cfg(&dev->config.mot_det))) {
			siferr("ERROR: sif dev init error: %d", ret);
			ret = -1;
			return ret;
		}
	}
	if (dev->config.frame_id.enable) {
		if (0 != (ret = sif_dev_frame_id_cfg(&dev->config.frame_id))) {
			siferr("ERROR: sif dev init error: %d", ret);
			ret = -1;
			return ret;
		}
	}
#endif
	dev->sif_file.receive_frame = false;
	ips_irq_disable(SIF_INT);
	ips_register_irqhandle(SIF_INT, x2_sif_irq, dev);
	return 0;
}

static void sif_deinit(sif_t * dev)
{
#ifdef CONFIG_X2_SIF_DEV
	sif_dev_stop();
#endif
	dev->sif_file.receive_frame = false;
}

static int x2_sif_open(struct inode *inode, struct file *file)
{
	sif_t *sif = dev_get_drvdata(g_sif_dev);
	spin_lock_init(&sif->sif_file.event_lock);
	init_waitqueue_head(&sif->sif_file.event_queue);
	sif->sif_file.event = 0;
	file->private_data = g_sif_dev;
	return 0;
}

static ssize_t x2_sif_write(struct file *file, const char __user * buf,
			    size_t count, loff_t * ppos)
{
	return 0;
}

static ssize_t x2_sif_read(struct file *file, char __user * buf, size_t size,
			   loff_t * ppos)
{
	sif_t *dev = sifdrv(file);
	sif_file_t *priv = &dev->sif_file;
	int ret = 0;
	unsigned long flags;

	sifinfo("sif read");
	spin_lock_irqsave(&priv->event_lock, flags);
	while (!priv->event) {
		spin_unlock_irqrestore(&priv->event_lock, flags);

		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (wait_event_interruptible(priv->event_queue, priv->event))
			return -ERESTARTSYS;

		/* If device was disassociated and no event exists set an error */
		if (!priv->event)
			return -EIO;

		spin_lock_irqsave(&priv->event_lock, flags);
	}
	spin_unlock_irqrestore(&priv->event_lock, flags);

	if (copy_to_user(buf, &priv->event, sizeof(priv->event)))
		ret = -EFAULT;
	else
		ret = sizeof(priv->event);
	priv->event = 0;
	return ret;
}

static int x2_sif_close(struct inode *inode, struct file *file)
{
	return 0;
}

static unsigned int x2_sif_poll(struct file *file,
				struct poll_table_struct *wait)
{
	sif_t *dev = sifdrv(file);
	sif_file_t *priv = &dev->sif_file;
	unsigned int mask = 0;
	unsigned long flags;

	poll_wait(file, &priv->event_queue, wait);
	spin_lock_irqsave(&priv->event_lock, flags);

	if (priv->event)
		mask |= (POLLIN | POLLRDNORM);

	spin_unlock_irqrestore(&priv->event_lock, flags);

	return mask;
}

static long x2_sif_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	sif_t *dev = sifdrv(file);
	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SIF_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case SIFIOC_INIT:
		{
			sif_cfg_t sif_cfg;
			printk(KERN_INFO "sif init cmd\n");
			if (!arg) {
				siferr
				    ("ERROR: sif init error, config should not be NULL");
				ret = EINVAL;
				break;
			}
			if (SIF_STATE_DEFAULT != dev->state) {
				siferr("WARNING: sif re-init, pre state: 0x%x",
				       dev->state);
			}
			if (copy_from_user
			    ((void *)&sif_cfg, (void __user *)arg,
			     sizeof(sif_cfg_t))) {
				siferr
				    ("ERROR: sif copy data from user failed\n");
				return -EINVAL;
			}
			if (0 != (ret = sif_init(dev, &sif_cfg))) {
				siferr("ERROR: sif init error: %d", ret);
				ret = -1;
				return ret;
			}
			dev->state = SIF_STATE_INIT;
		}
		break;
	case SIFIOC_DEINIT:
		{
			printk(KERN_INFO "sif deinit cmd\n");
			if (SIF_STATE_DEFAULT == dev->state) {
				sifinfo("sif has not been init");
				break;
			}
			if (SIF_STATE_START == dev->state) {
				sif_stop(dev);
			}
			sif_deinit(dev);
			dev->state = SIF_STATE_DEFAULT;
		}
		break;
	case SIFIOC_START:
		{
			printk(KERN_INFO "sif start cmd\n");
			if (SIF_STATE_START == dev->state) {
				sifinfo("sif already in start state");
				break;
			} else if (SIF_STATE_INIT != dev->state
				   && SIF_STATE_STOP != dev->state) {
				siferr
				    ("ERROR: sif start state error, current state: 0x%x",
				     dev->state);
				ret = EINVAL;
				break;
			}
			if (0 != (ret = sif_start(dev))) {
				siferr("ERROR: sif start error: %d", ret);
				ret = -1;
				return ret;
			}
			dev->state = SIF_STATE_START;
		}
		break;
	case SIFIOC_STOP:
		{
			printk(KERN_INFO "sif stop cmd\n");
			if (SIF_STATE_STOP == dev->state) {
				sifinfo("sif already in stop state");
				break;
			} else if (SIF_STATE_START != dev->state) {
				siferr
				    ("ERROR: sif dev stop state error, current state: 0x%x",
				     dev->state);
				ret = EINVAL;
				break;
			}
			if (0 != (ret = sif_stop(dev))) {
				siferr("ERROR: sif stop error: %d", ret);
				ret = -1;
				return ret;
			}
			dev->state = SIF_STATE_STOP;
		}
		break;
	case SIFIOC_GET_STATUS:
		{
			sif_status_t status;
			if (!arg) {
				siferr
				    ("ERROR: sif get status error, input should not be NULL");
				ret = EINVAL;
				break;
			}
#ifdef CONFIG_X2_SIF_DEV
			sif_dev_get_status(&status);
#endif
			if (copy_to_user
			    ((void __user *)arg, (void *)&status,
			     sizeof(sif_status_t))) {
				printk(KERN_ERR
				       "sif copy data from user failed\n");
				return -EINVAL;
			}
		}
		break;
	case SIFIOC_GET_INFO:
		{
			sif_info_t info;
			if (!arg) {
				siferr
				    ("ERROR: sif get info error, input should not be NULL");
				ret = EINVAL;
				break;
			}
#ifdef CONFIG_X2_SIF_DEV
			sif_dev_get_info(&info);
#endif
			if (copy_to_user
			    ((void __user *)arg, (void *)&info,
			     sizeof(sif_info_t))) {
				printk(KERN_ERR
				       "sif copy data to user failed\n");
				return -EINVAL;
			}
		}
		break;
	case SIFIOC_GET_FRAME_ID:
		{
			frame_id_info_t frame_id = { 0, };
			if (!arg) {
				siferr
				    ("ERROR: sif get frame id error, input should not be NULL");
				ret = EINVAL;
				break;
			}
#ifdef CONFIG_X2_SIF_DEV
			sif_dev_frame_id_get(&frame_id);
#endif
			if (copy_to_user
			    ((void __user *)arg, (void *)&frame_id,
			     sizeof(frame_id_info_t))) {
				printk(KERN_ERR
				       "sif copy data to user failed\n");
				return -EINVAL;
			}
		}
		break;
	default:
		siferr("sif cmd 0x%x not support\n", cmd);
		break;
	}
	return 0;
}

static struct file_operations sif_fops = {
	.owner = THIS_MODULE,
	.open = x2_sif_open,
	.write = x2_sif_write,
	.read = x2_sif_read,
	.poll = x2_sif_poll,
	.release = x2_sif_close,
	.unlocked_ioctl = x2_sif_ioctl,
	.compat_ioctl = x2_sif_ioctl,
};

static int sif_major = 0;
struct cdev sif_cdev;
static int __init sif_module_init(void)
{
	int ret = 0;
	dev_t devno;
	struct cdev *p_cdev = &sif_cdev;
	sif_t *sif = NULL;

	printk(KERN_INFO "sif driver init enter\n");
	sif = kzalloc(sizeof(sif_t), GFP_KERNEL);
	if (sif == NULL) {
		printk(KERN_ERR "sif malloc failed\n");
		return -1;
	}
	ret = alloc_chrdev_region(&devno, 0, 1, "x2_sif");
	if (ret < 0) {
		printk(KERN_ERR "Error %d while alloc chrdev sif", ret);
		goto err;
	}
	sif_major = MAJOR(devno);
	cdev_init(p_cdev, &sif_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		printk(KERN_ERR "Error %d while adding x2 sif cdev", ret);
		goto err;
	}
	g_sif_class = class_create(THIS_MODULE, "x2_sif");
	if (IS_ERR(g_sif_class)) {
		printk(KERN_INFO "[%s:%d] class_create error\n", __func__,
		       __LINE__);
		ret = PTR_ERR(g_sif_class);
		goto err;
	}
	g_sif_dev =
	    device_create(g_sif_class, NULL, MKDEV(sif_major, 0), (void *)sif,
			  "x2_sif");
	if (IS_ERR(g_sif_dev)) {
		printk(KERN_ERR "[%s] deivce create error\n", __func__);
		ret = PTR_ERR(g_sif_dev);
		goto err;
	}

	printk(KERN_INFO "sif driver init exit\n");
	return 0;
err:
	class_destroy(g_sif_class);
	cdev_del(&sif_cdev);
	unregister_chrdev_region(MKDEV(sif_major, 0), 1);
	kzfree(sif);
	return ret;
}

static void __exit sif_module_exit(void)
{
	sif_t *sif = dev_get_drvdata(g_sif_dev);
	printk(KERN_INFO "[%s:%d] sif_exit\n", __func__, __LINE__);
	device_destroy(g_sif_class, MKDEV(sif_major, 0));
	class_destroy(g_sif_class);
	cdev_del(&sif_cdev);
	unregister_chrdev_region(MKDEV(sif_major, 0), 1);
	kzfree(sif);
	return;
}

module_init(sif_module_init);
module_exit(sif_module_exit);
MODULE_LICENSE("GPL v2");
