/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

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
#include <x2/diag.h>
#include <linux/suspend.h>

#include "x2/x2_ips.h"
#include "x2/x2_sif.h"
#include "x2_sif_dev.h"
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
	spinlock_t		  event_lock;
	uint32_t		  event;
	uint32_t		  receive_frame;
	uint32_t		  status;
	wait_queue_head_t event_queue;
} sif_file_t;

typedef struct sif_s {
	sif_cfg_t		config;
	sif_state_t		state;	 /* sif state */
	struct sock    *nl_sk;
	uint32_t        usr_pid;
	sif_file_t      sif_file;
	uint32_t        bypass_inited;
	bool			suspended;
	int users;
} sif_t;

static struct class  *g_sif_class;
static struct device *g_sif_dev;
unsigned int sif_debug_level = 0;
module_param(sif_debug_level, uint, 0644);
unsigned int sif_irq_debug = 0;
module_param(sif_irq_debug, uint, 0644);

#ifdef CONFIG_PM_SLEEP
static sif_cfg_t g_sif_cfg;
#endif

#define sifdrv(f) (dev_get_drvdata((f)->private_data))

static int sif_start(sif_t *dev)
{
	int ret = 0;
	if (0 != (ret = sif_dev_start(&dev->config.sif_init))) {
		siferr("ERROR: sif dev start error: %d", ret);
		return ret;
	}
	ips_unmask_int(MOT_DET | SIF_SOFT_DROP | SIF_FRAME_END_INTERRUPT
			| SIF_FRAME_START_INTERRUPT | SIF_SIZE_ERR0 | SIF_SIZE_ERR1);
	ips_irq_enable(SIF_INT);
	return ret;
}

static int sif_stop(sif_t *dev)
{
	int			  ret = 0;
	unsigned long flags;
	ips_irq_disable(SIF_INT);
	spin_lock_irqsave(&dev->sif_file.event_lock, flags);
	dev->sif_file.event = SIF_STOP;
	dev->sif_file.receive_frame = false;
	spin_unlock_irqrestore(&dev->sif_file.event_lock, flags);
	wake_up_interruptible(&dev->sif_file.event_queue);
	sif_dev_stop(&dev->config.sif_init);
	return ret;
}

static void sif_diag_report(uint8_t errsta, unsigned int status)
{
	unsigned int sta;

	sta = status;
	if (errsta) {
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_VIO,
				EventIdVioSifErr,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				(uint8_t *)&sta,
				sizeof(unsigned int));
	} else {
		diag_send_event_stat(
				DiagMsgPrioMid,
				ModuleDiag_VIO,
				EventIdVioSifErr,
				DiagEventStaSuccess);
	}
}

extern int32_t actual_enable_bypass(void);
static void x2_sif_irq(unsigned int status, void *data)
{
	sif_t          *sif = NULL;
	uint8_t err_occurred = 0;

	if (NULL == data) {
		siferr("sif irq input data error!");
		return;
	}
	sif = (sif_t *)data;
	//TODO
	if (sif_irq_debug) {
		printk(KERN_INFO "[sif][irq]: 0x%x\n", status & SIF_INT_BITS);
	}
	spin_lock(&sif->sif_file.event_lock);
	if (!sif->sif_file.receive_frame && (status & SIF_FRAME_START_INTERRUPT)) {
		sif->sif_file.event |= SIF_START;
		sif->sif_file.receive_frame = true;
	}
	if (status & SIF_FRAME_END_INTERRUPT) {
		ips_mask_int(SIF_FRAME_END_INTERRUPT);
		actual_enable_bypass();
	}
	if (status & (SIF_SIZE_ERR0 | SIF_SIZE_ERR1)) {
		sif->sif_file.event |= SIF_ERROR;
		sif->sif_file.status = status & (SIF_SIZE_ERR0 | SIF_SIZE_ERR1);
		err_occurred = 1;
	}
	if (status & MOT_DET) {
		sif->sif_file.event |= SIF_MOTDET;
		if (sif->suspended)
			pm_system_wakeup();
	}
	spin_unlock(&sif->sif_file.event_lock);
	if (sif->sif_file.event)
		wake_up_interruptible(&sif->sif_file.event_queue);

	sif_diag_report(err_occurred, status);
}

static int sif_update(sif_t *dev, sif_cfg_t *cfg)
{
	int			  ret = 0;
	memset(&dev->config, 0, sizeof(sif_cfg_t));
	memcpy(&dev->config, cfg, sizeof(sif_cfg_t));
	sifinfo("sif update config format: %d pixlen: %d bus: %d width: %d height: %d",
			cfg->sif_init.format,
			cfg->sif_init.pix_len,
			cfg->sif_init.bus_type,
			cfg->sif_init.width,
			cfg->sif_init.height);
	if (0 != (ret = sif_dev_update(&dev->config.sif_init))) {
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
	return 0;
}

static int sif_bypass_init(sif_t *dev)
{
	sif_cfg_t *cfg = &dev->config;

	if (BUS_TYPE_BT1120 == cfg->sif_init.bus_type) {
		ips_set_btout_clksrc(BYPASS_CLK, cfg->sif_init.pclk_out_inv);
	} else if (BUS_TYPE_DVP != cfg->sif_init.bus_type) {
		ips_mipi_ctl_set(MIPI_BYPASS_GEN_HSYNC_EN, true);
		ips_mipi_ctl_set(MIPI_BYPASS_GEN_HSYNC_DLY_CNT, 4);
	}
	dev->bypass_inited = true;

	return 0;
}

static int sif_bypass_ctrl(sif_t *dev, bypass_ctrl_info_t *bypass_ctrl)
{
	if (!dev->bypass_inited)
		sif_bypass_init(dev);

	return sif_dev_bypass_ctrl(bypass_ctrl->port, bypass_ctrl->enable);
}

static int sif_init(sif_t *dev, sif_cfg_t *cfg)
{
	int			  ret = 0;
	memset(&dev->config, 0, sizeof(sif_cfg_t));
	memcpy(&dev->config, cfg, sizeof(sif_cfg_t));
	sifinfo("sif config format: %d pixlen: %d bus: %d width: %d height: %d",
			cfg->sif_init.format,
			cfg->sif_init.pix_len,
			cfg->sif_init.bus_type,
			cfg->sif_init.width,
			cfg->sif_init.height);
	ips_module_reset(RST_SIF);
	if (BUS_TYPE_DVP == cfg->sif_init.bus_type) {
		ips_pinmux_dvp();
	} else if (BUS_TYPE_BT1120 == cfg->sif_init.bus_type) {
		ips_pinmux_bt();
		ips_set_btin_clksrc(cfg->sif_init.pclk_in_inv);
	}
	if (!dev->bypass_inited && cfg->sif_init.bypass_en)
		sif_bypass_init(dev);
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
	dev->sif_file.receive_frame = false;
	ips_irq_disable(SIF_INT);
	ips_register_irqhandle(SIF_INT, x2_sif_irq, dev);
	return 0;
}

static void sif_deinit(sif_t *dev)
{
	sif_dev_stop(&dev->config.sif_init);
	dev->sif_file.receive_frame = false;
	dev->bypass_inited = false;
}

static int x2_sif_open(struct inode *inode, struct file *file)
{
	sif_t *sif = dev_get_drvdata(g_sif_dev);
	sif->sif_file.event = 0;
	sif->users++;
	file->private_data = g_sif_dev;
	return 0;
}
static ssize_t x2_sif_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t x2_sif_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	sif_t		   *dev = sifdrv(file);
	sif_file_t	   *priv = &dev->sif_file;
	int				ret = 0;

	if (0 == priv->status) {
		sifinfo("sif status OK");
	}
	if (copy_to_user(buf, &priv->status, sizeof(priv->status)))
		ret = -EFAULT;
	else
		ret = sizeof(priv->status);
	priv->status = 0;
	return ret;
}

static int x2_sif_close(struct inode *inode, struct file *file)
{
	sif_t *dev = sifdrv(file);
	dev->users--;
	if (dev->users)
		return 0;
	if (dev->state != SIF_STATE_DEFAULT) {
		sif_stop(dev);
		sif_deinit(dev);
		dev->state = SIF_STATE_DEFAULT;
	}
	return 0;
}

static unsigned int x2_sif_poll(struct file *file, struct poll_table_struct *wait)
{
	sif_t		   *dev = sifdrv(file);
	sif_file_t	   *priv = &dev->sif_file;
	unsigned int	mask = 0;
	unsigned long	flags;

	poll_wait(file, &priv->event_queue, wait);
	spin_lock_irqsave(&priv->event_lock, flags);
	if (SIF_STOP == priv->event)
		mask = EPOLLHUP;
	else if (SIF_MOTDET == priv->event) {
		mask = EPOLLPRI;
	} else if (SIF_ERROR == priv->event) {
		mask = EPOLLERR;
	} else if (priv->event) {
		mask = EPOLLIN | EPOLLET;
	}
	priv->event = 0;
	spin_unlock_irqrestore(&priv->event_lock, flags);

	return mask;
}

static long x2_sif_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int    ret = 0;
	sif_t *dev = sifdrv(file);
	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SIF_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case SIFIOC_INIT:
		{
			sif_cfg_t	  sif_cfg;
			sifinfo("sif init cmd\n");
			if (!arg) {
				siferr("ERROR: sif init error, config should not be NULL");
				ret = -EINVAL;
				break;
			}
			if (SIF_STATE_DEFAULT != dev->state && SIF_STATE_INIT != dev->state) {
				sifinfo("sif has been init before");
				ret = -EINVAL;
				break;
			}
			if (copy_from_user((void *)&sif_cfg, (void __user *)arg, sizeof(sif_cfg_t))) {
				siferr("ERROR: sif copy data from user failed\n");
				return -EINVAL;
			}
#ifdef CONFIG_PM_SLEEP
			if (copy_from_user((void *)&g_sif_cfg, (void __user *)arg, sizeof(sif_cfg_t))) {
				siferr("ERROR: sif copy data from user to g_sif_cfg failed\n");
				return -EINVAL;
			}
#endif
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
			sifinfo("sif deinit cmd\n");
			if (dev->users > 1)
				break;
			if (SIF_STATE_DEFAULT == dev->state) {
				sifinfo("sif has not been init");
				break;
			}
			if (SIF_STATE_START == dev->state) {
				sifinfo("sif stop\n");
				sif_stop(dev);
			}
			sif_deinit(dev);
			dev->state = SIF_STATE_DEFAULT;
		}
		break;
	case SIFIOC_START:
		{
			sifinfo("sif start cmd\n");
			if (SIF_STATE_START == dev->state) {
				sifinfo("sif already in start state");
				break;
			} else if (SIF_STATE_INIT != dev->state && SIF_STATE_STOP != dev->state) {
				siferr("ERROR: sif start state error, current state: 0x%x", dev->state);
				ret = -EINVAL;
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
			if (dev->users > 1)
				break;
			sifinfo("sif stop cmd\n");
			if (SIF_STATE_STOP == dev->state) {
				sifinfo("sif already in stop state");
				break;
			} else if (SIF_STATE_START != dev->state) {
				siferr("ERROR: sif dev stop state error, current state: 0x%x", dev->state);
				ret = -EINVAL;
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
				siferr("ERROR: sif get status error, input should not be NULL");
				ret = -EINVAL;
				break;
			}
			sif_dev_get_status(&status);
			if (copy_to_user((void __user *)arg, (void *)&status, sizeof(sif_status_t))) {
				siferr("sif copy data from user failed\n");
				return -EINVAL;
			}
		}
		break;
	case SIFIOC_GET_INFO:
		{
			sif_info_t info;
			if (!arg) {
				siferr("ERROR: sif get info error, input should not be NULL");
				ret = -EINVAL;
				break;
			}
			sif_dev_get_info(&info);
			if (copy_to_user((void __user *)arg, (void *)&info, sizeof(sif_info_t))) {
				siferr("sif copy data to user failed\n");
				return -EINVAL;
			}
		}
		break;
	case SIFIOC_GET_FRAME_ID:
		{
			frame_id_info_t frame_id = {0, };
			if (!arg) {
				siferr("ERROR: sif get frame id error, input should not be NULL");
				ret = -EINVAL;
				break;
			}
			sif_dev_frame_id_get(&frame_id);
			if (copy_to_user((void __user *)arg, (void *)&frame_id, sizeof(frame_id_info_t))) {
				siferr("sif copy data to user failed\n");
				return -EINVAL;
			}
		}
		break;
	case SIFIOC_UPDATE:
		{
			sif_cfg_t	  sif_cfg;
			sifinfo("sif update cmd\n");
			if (!arg) {
				siferr("ERROR: sif init error, config should not be NULL");
				ret = -EINVAL;
				break;
			}
			if (SIF_STATE_DEFAULT == dev->state) {
				sifinfo("sif has not been init");
				ret = -EINVAL;
				break;
			}
			if (copy_from_user((void *)&sif_cfg, (void __user *)arg, sizeof(sif_cfg_t))) {
				siferr("ERROR: sif copy data from user failed\n");
				return -EINVAL;
			}
#ifdef CONFIG_PM_SLEEP
			if (copy_from_user((void *)&g_sif_cfg, (void __user *)arg, sizeof(sif_cfg_t))) {
				siferr("ERROR: sif copy data from user to g_sif_cfg failed\n");
				return -EINVAL;
			}
#endif
			if (0 != (ret = sif_update(dev, &sif_cfg))) {
				siferr("ERROR: sif init error: %d", ret);
				ret = -1;
				return ret;
			}
		}
		break;
	case SIFIOC_BYPASS_CTRL:
		{
			bypass_ctrl_info_t bypass_ctrl;
			if (!arg) {
				siferr("ERROR: ctrl info should not be NULL");
				ret = -EINVAL;
				break;
			}
			if (SIF_STATE_DEFAULT == dev->state) {
				sifinfo("sif has not been init");
				ret = -EINVAL;
				break;
			}
			if (copy_from_user((void *)&bypass_ctrl,
						(void __user *)arg,
						sizeof(bypass_ctrl_info_t))) {
				siferr("ERROR: sif copy data from user failed\n");
				return -EINVAL;
			}
			ret = sif_bypass_ctrl(dev, &bypass_ctrl);
			if (ret != 0) {
				siferr("ERROR: sif bypass ctrl error: %d", ret);
				ret = -1;
				return ret;
			}
		}
		break;
	case SIFIOC_MOT_DET_CFG: {
			mot_det_t mot_det_cfg;

			sifinfo("sif mot_det_cfg cmd\n");
			if (!arg) {
				siferr("ERROR: config should not be NULL");
				ret = -EINVAL;
				break;
			}

			if (copy_from_user((void *)&mot_det_cfg,
			    (void __user *)arg, sizeof(mot_det_cfg))) {
				siferr("ERROR: sif copy from user failed\n");
				return -EINVAL;
			}
			if (sif_dev_mot_det_cfg(&mot_det_cfg) < 0) {
				siferr("ERROR: sif mot_det config failed!\n");
				return -1;
			}
		}
		break;
	case SIFIOC_MOT_DET_CTRL: {
			mot_det_t mot_det_ctrl;

			sifinfo("sif mot_det_ctrl cmd\n");
			if (!arg) {
				siferr("ERROR: config should not be NULL");
				ret = -EINVAL;
				break;
			}
			if (copy_from_user((void *)&mot_det_ctrl,
			   (void __user *)arg, sizeof(mot_det_ctrl))) {
				siferr("ERROR: sif copy from user failed\n");
				return -EINVAL;
			}
			if (sif_dev_mot_det_ctrl(&mot_det_ctrl) < 0) {
				siferr("ERROR: sif mot_det control failed!\n");
				return -1;
			}
		}
		break;

	default:
		siferr("sif cmd 0x%x not support\n", cmd);
		break;
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
int x2_sif_suspend(void)
{
	sif_t *sif = dev_get_drvdata(g_sif_dev);
	sif->suspended = true;

	if (pm_suspend_target_state == PM_SUSPEND_TO_IDLE)
		return 0;

	pr_info("%s:%s enter suspend...\n", __FILE__, __func__);

	sif_stop(sif);
	sif_deinit(sif);

	return 0;
}
EXPORT_SYMBOL(x2_sif_suspend);

int x2_sif_resume(void)
{
	int ret = 0;
	sif_t *sif = dev_get_drvdata(g_sif_dev);
	sif->suspended = false;

	if (pm_suspend_target_state == PM_SUSPEND_TO_IDLE)
		return 0;

	pr_info("%s:%s enter resume...\n", __FILE__, __func__);

	if (sif->state == SIF_STATE_DEFAULT) {
		sif_stop(sif);
		sif_deinit(sif);
	} else if (sif->state == SIF_STATE_START) {
		/* it need to initialize first */
		if (0 != (ret = sif_init(sif, &g_sif_cfg))) {
			siferr("ERROR: sif init error: %d", ret);
			ret = -1;
			return ret;
		}

		/* restart sif */
		if (0 != (ret = sif_start(sif))) {
			siferr("ERROR: sif start error: %d", ret);
			ret = -1;
			return ret;
		}
	} else if (sif->state == SIF_STATE_STOP) {
		/* it need to initialize first */
		if (0 != (ret = sif_init(sif, &g_sif_cfg))) {
			siferr("ERROR: sif init error: %d", ret);
			ret = -1;
			return ret;
		}

		if (0 != (ret = sif_stop(sif))) {
			siferr("ERROR: sif stop error: %d", ret);
			ret = -1;
			return ret;
		}
	}

	return ret;
}
EXPORT_SYMBOL(x2_sif_resume);
#endif

static struct file_operations sif_fops = {
	.owner	= THIS_MODULE,
	.open	= x2_sif_open,
	.write	= x2_sif_write,
	.read = x2_sif_read,
	.poll = x2_sif_poll,
	.release  = x2_sif_close,
	.unlocked_ioctl = x2_sif_ioctl,
	.compat_ioctl = x2_sif_ioctl,
};

static int	  sif_major = 0;
struct cdev   sif_cdev;
static int __init sif_module_init(void)
{
	int			  ret = 0;
	dev_t		  devno;
	struct cdev  *p_cdev = &sif_cdev;
	sif_t *sif = NULL;

	sifinfo("sif driver init enter");
	sif = kzalloc(sizeof(sif_t), GFP_KERNEL);
	if (sif == NULL) {
		siferr("sif malloc failed");
		return -1;
	}
	ret = alloc_chrdev_region(&devno, 0, 1, "x2_sif");
	if (ret < 0) {
		siferr("Error %d while alloc chrdev sif", ret);
		goto err;
	}
	sif_major = MAJOR(devno);
	cdev_init(p_cdev, &sif_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		siferr("Error %d while adding x2 sif cdev", ret);
		goto err;
	}
	g_sif_class = class_create(THIS_MODULE, "x2_sif");
	if (IS_ERR(g_sif_class)) {
		siferr("[%s:%d] class_create error", __func__, __LINE__);
		ret = PTR_ERR(g_sif_class);
		goto err;
	}
	g_sif_dev = device_create(g_sif_class, NULL, MKDEV(sif_major, 0), (void *)sif, "x2_sif");
	if (IS_ERR(g_sif_dev)) {
		siferr("[%s] deivce create error", __func__);
		ret = PTR_ERR(g_sif_dev);
		goto err;
	}
	spin_lock_init(&sif->sif_file.event_lock);
	init_waitqueue_head(&sif->sif_file.event_queue);
	ips_mask_int(SIF_FRAME_START_INTERRUPT | SIF_FRAME_END_INTERRUPT);
	if (diag_register(ModuleDiag_VIO, EventIdVioSifErr,
					8, 400, 7000, NULL) < 0)
		pr_err("sif diag register fail\n");

	sifinfo("sif driver init done");
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
	sifinfo("[%s:%d] sif_exit\n", __func__, __LINE__);
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

