/*
 * Copyright (C) 2019 Horizon Robotics
 *
 * Zhang Guoying <guoying.zhang@horizon.ai>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */

#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/reset.h>
#include <asm/irq.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include "bpu.h"
#include "bpu_core.h"
#include "bpu_ctrl.h"

/* tasklet just to do statistics work */
static void bpu_core_tasklet(unsigned long data)
{
	/* core ratio will be update by group check */
	bpu_sched_seed_update();
}

static irqreturn_t bpu_core_irq_handler(int irq, void *dev_id)
{
	struct bpu_core *core = (struct bpu_core *)dev_id;
	struct bpu_user *tmp_user;
	struct bpu_fc tmp_bpu_fc;
	uint32_t tmp_hw_id, err;
	int32_t lost_report = 0;
	int32_t ret;

	if (atomic_read(&core->host->open_counter) == 0)
		return IRQ_HANDLED;

	bpu_prio_trig_out(core->prio_sched);
	spin_lock(&core->spin_lock);
	ret = core->hw_ops->read_fc(core, &tmp_hw_id, &err);
	spin_unlock(&core->spin_lock);
	pr_debug("BPU Core[%d] irq %d come\n", core->index, tmp_hw_id);
	/* err irq report the the error to the right user */
	if (err != 0) {
		ret = kfifo_peek(&core->run_fc_fifo[FC_PRIO(tmp_hw_id)], &tmp_bpu_fc);
		if (ret < 1) {
			dev_err(core->dev,
					"bpu core no fc bufferd, when error[%d]\n", ret);
			return IRQ_HANDLED;
		}
		tmp_bpu_fc.info.status = err;
		goto do_report;
	}

repush:
	lost_report = 0;
	ret = kfifo_get(&core->run_fc_fifo[FC_PRIO(tmp_hw_id)], &tmp_bpu_fc);
	if (ret < 1) {
		dev_err(core->dev,
				"bpu core no fc bufferd, when normal[%d], lost = %d\n",
				tmp_hw_id, lost_report);
		return IRQ_HANDLED;
	}

	/* maybe lost a hw irq */
	if (tmp_bpu_fc.hw_id < tmp_hw_id)
		lost_report = 1;

	if (tmp_bpu_fc.hw_id > tmp_hw_id) {
		if (tmp_bpu_fc.hw_id - tmp_hw_id > HW_ID_MAX - 10)
			lost_report = 1;
	}

do_report:
	/* update the statistics element */
	bpu_core_update(core, &tmp_bpu_fc);

	tmp_bpu_fc.info.run_c_mask = (0x1 << core->index);
	tmp_user = bpu_get_user(&tmp_bpu_fc);
	if (tmp_user) {
		if (tmp_user->is_alive) {
			if (kfifo_is_full(&tmp_user->done_fcs)) {
				dev_err(core->dev, "user[%d] (%d)read data too Slow\n",
						kfifo_len(&tmp_user->done_fcs), tmp_user->id);
				kfifo_skip(&tmp_user->done_fcs);
			}

			ret = kfifo_in(&tmp_user->done_fcs, &tmp_bpu_fc.info, 1);
			if (ret < 1) {
				dev_err(core->dev, "bpu buffer bind user error\n");
				return IRQ_HANDLED;
			}
			wake_up_interruptible(&tmp_user->poll_wait);
		}
		tmp_user->running_task_num--;

		if ((tmp_user->running_task_num <= 0) && (!tmp_user->is_alive))
			complete(&tmp_user->no_task_comp);
	}

	core->running_task_num--;
	if (core->running_task_num <= 0)
		complete(&core->no_task_comp);

	if (kfifo_is_full(&core->done_fc_fifo))
		kfifo_skip(&core->done_fc_fifo);

	ret = kfifo_in(&core->done_fc_fifo, &tmp_bpu_fc, 1);
	if (ret < 1) {
		dev_err(core->dev, "bpu buffer bind user error\n");
		return IRQ_HANDLED;
	}

	if (lost_report)
		goto repush;

	tasklet_schedule(&core->tasklet);

	return IRQ_HANDLED;
}

static long bpu_core_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;
	struct bpu_core *core = (struct bpu_core *)user->host;
	union bpu_ioctl_arg data;
	int32_t ret, i;

	switch (cmd) {
	case BPU_GET_RATIO:
		data.ratio = bpu_core_ratio(core);
		if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd))) {
			dev_err(core->dev, "copy data to userspace failed\n");
			return -EFAULT;
		}
		break;
	case BPU_GET_CAP:
		/* get the lowest prio fifo size to user */
		if (core->hw_enabled)
			data.cap = kfifo_avail(&core->run_fc_fifo[0]);
		else
			data.cap = 0;
		if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd))) {
			dev_err(core->dev, "copy data to userspace failed\n");
			return -EFAULT;
		}
		break;
	case BPU_SET_POWER:
		data.level = 0;
		if (copy_from_user(&data, (void __user *)arg, _IOC_SIZE(cmd))) {
			dev_err(core->dev, "copy data from userspace failed\n");
			return -EFAULT;
		}
		if (data.level <= 0) {
			ret = bpu_core_disable(core);
			if (ret) {
				dev_err(core->dev, "Disable BPU core%d failed\n", core->index);
				return ret;
			}
		} else {
			ret = bpu_core_enable(core);
			if (ret) {
				dev_err(core->dev, "Enable BPU core%d failed\n", core->index);
				return ret;
			}
		}
		break;
	case BPU_SET_FREQ_LEVEL:
		data.level = 0;
		if (copy_from_user(&data, (void __user *)arg, _IOC_SIZE(cmd))) {
			dev_err(core->dev, "copy data from userspace failed\n");
			return -EFAULT;
		}
		ret = bpu_core_set_freq_level(core, data.level);
		if (ret) {
			dev_err(core->dev, "Set BPU core%d freq level(%d)failed\n",
					core->index, data.level);
			return ret;
		}
		break;
	case BPU_GET_FREQ_LEVEL:
		data.level = 0;
		if (core->dvfs) {
			for (i = 0; i < core->dvfs->level_num; i++) {
				if (core->dvfs->rate == core->dvfs->profile.freq_table[i])
					data.level = i - core->dvfs->level_num + 1;
			}
		}
		if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd))) {
			dev_err(core->dev, "copy data to userspace failed\n");
			return -EFAULT;
		}
		break;
	case BPU_GET_FREQ_LEVEL_NUM:
		data.level = 1;
		if (core->dvfs) {
			data.level = core->dvfs->level_num;
		}
		if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd))) {
			dev_err(core->dev, "copy data to userspace failed\n");
			return -EFAULT;
		}
		break;
	case BPU_RESET:
		bpu_core_reset(core);
		break;
	case BPU_SET_LIMIT:
		if (copy_from_user(&data, (void __user *)arg, _IOC_SIZE(cmd))) {
			pr_err("%s: copy data failed from userspace\n", __func__);
			return -EFAULT;
		}
		bpu_core_set_limit(core, data.limit);
		break;
	default:
		pr_err("%s: BPU invalid ioctl argument\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static unsigned int bpu_core_poll(struct file *filp, poll_table *wait)
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;
	uint32_t mask = 0;

	poll_wait(filp, &user->poll_wait, wait);
	mutex_lock(&user->mutex_lock);

	if (kfifo_len(&user->done_fcs))
		mask |= POLLIN | POLLRDNORM;

	mutex_unlock(&user->mutex_lock);

	return mask;
}

static ssize_t bpu_core_read(struct file *filp,
		char __user *buf, size_t len, loff_t *f_pos)
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;
	struct bpu_core *core = (struct bpu_core *)user->host;

	return bpu_read_with_user(core, user, buf, len);

}

static ssize_t bpu_core_write(struct file *filp,
		const char __user *buf, size_t len, loff_t *f_pos)
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;
	struct bpu_core *core = (struct bpu_core *)user->host;

	return bpu_write_with_user(core, user, buf, len);
}

static int bpu_core_open(struct inode *inode, struct file *filp)
{
	struct bpu_core *core =
		(struct bpu_core *)container_of(filp->private_data,
				struct bpu_core, miscdev);
	struct bpu_user *user;
	int32_t ret, i;

	if (atomic_read(&core->open_counter) == 0) {
		/* first open init something files */
		core->prio_sched = bpu_prio_init(core, CONFIG_BPU_PRIO_NUM);
		if (!core->prio_sched) {
			dev_err(core->dev, "Init bpu core prio sched failed\n");
			return -EINVAL;
		}
		ret = bpu_core_enable(core);
		if (ret) {
			dev_err(core->dev, "Can't bpu core hw enable failed\n");
			return ret;
		}
		for (i = 0; i < BPU_PRIO_NUM; i++)
			atomic_set(&core->hw_id_counter[i], 1);

		core->fc_buf_limit = 0;
	} else {
		if (!core->hw_enabled) {
			ret = bpu_core_enable(core);
			if (ret) {
				dev_err(core->dev, "Can't bpu core hw enable failed\n");
				return ret;
			}
		}
	}

	user = kzalloc(sizeof(struct bpu_user), GFP_KERNEL);
	if (!user) {
		dev_err(core->dev, "Can't bpu user mem\n");
		return -ENOMEM;
	}

	user->id = task_pid_nr(current->group_leader);

	/* init fifo which report to userspace */
	ret = kfifo_alloc(&user->done_fcs, BPU_CORE_RECORE_NUM, GFP_KERNEL);
	if (ret) {
		kfree(user);
		dev_err(core->dev, "Can't bpu user fifo buffer\n");
		return -ret;
	}

	init_waitqueue_head(&user->poll_wait);
	user->host = core;
	user->p_file_private = &filp->private_data;
	list_add((struct list_head *)user, &core->user_list);
	spin_lock_init(&user->spin_lock);
	mutex_init(&user->mutex_lock);
	init_completion(&user->no_task_comp);
	user->is_alive = 1;
	user->running_task_num = 0;
	/* replace user the private to store user */
	filp->private_data = user;

	atomic_inc(&core->open_counter);
	return 0;
}

static int bpu_core_release(struct inode *inode, struct file *filp)
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;
	struct bpu_core *core = (struct bpu_core *)user->host;
	unsigned long flags;
	int32_t i;

	user->is_alive = 0;
	/* wait user running fc done */
	while(user->running_task_num > 0) {
		/* timeout to prevent bpu hung make system hung*/
		if(!wait_for_completion_timeout(&user->no_task_comp, HZ))
			user->running_task_num--;
	}

	atomic_dec(&core->open_counter);

	if (atomic_read(&core->open_counter) == 0) {
		/* release the real bpu core */
		bpu_prio_exit(core->prio_sched);
		bpu_core_disable(core);

		spin_lock_irqsave(&core->spin_lock, flags);
		core->p_run_time = 0;
		core->ratio = 0;
		for (i = 0; i < BPU_PRIO_NUM; i++)
			kfifo_reset(&core->run_fc_fifo[i]);
		spin_unlock_irqrestore(&core->spin_lock, flags);
	}

	spin_lock_irqsave(&core->spin_lock, flags);
	list_del((struct list_head *)user);
	kfifo_free(&user->done_fcs);
	kfree(user);
	filp->private_data = NULL;
	spin_unlock_irqrestore(&core->spin_lock, flags);

	return 0;
}

static const struct file_operations bpu_core_fops = {
	.owner		= THIS_MODULE,
	.open		= bpu_core_open,
	.release	= bpu_core_release,
	.read		= bpu_core_read,
	.write		= bpu_core_write,
	.poll		= bpu_core_poll,
	.unlocked_ioctl = bpu_core_ioctl,
	.compat_ioctl = bpu_core_ioctl,
};

#ifdef CONFIG_PM
static int bpu_core_suspend(struct device *dev)
{
	struct bpu_core *core = dev_get_drvdata(dev);
	uint32_t tmp_hw_state;
	int32_t ret;

	mutex_lock(&core->mutex_lock);
	tmp_hw_state = core->hw_enabled;
	ret = bpu_core_disable(core);
	if (ret) {
		mutex_unlock(&core->mutex_lock);
		dev_err(dev, "BPU core%d suspend failed\n", core->index);
		return ret;
	}
	core->hw_enabled = tmp_hw_state;
	mutex_unlock(&core->mutex_lock);

	return 0;
}

static int bpu_core_resume(struct device *dev)
{
	struct bpu_core *core = dev_get_drvdata(dev);
	int32_t ret;

	mutex_lock(&core->mutex_lock);
	if (!core->hw_enabled) {
		mutex_unlock(&core->mutex_lock);
		return 0;
	}
	ret = bpu_core_enable(core);
	if (ret) {
		mutex_unlock(&core->mutex_lock);
		dev_err(dev, "BPU core%d resume failed\n", core->index);
		return ret;
	}

	ret = bpu_core_process_recover(core);
	if (ret) {
		mutex_unlock(&core->mutex_lock);
		dev_err(dev, "BPU core%d recovery failed\n", core->index);
		return ret;
	}
	mutex_unlock(&core->mutex_lock);
	bpu_prio_trig_out(core->prio_sched);

	return 0;
}

static SIMPLE_DEV_PM_OPS(bpu_core_pm_ops,
			 bpu_core_suspend, bpu_core_resume);
#endif

static int bpu_core_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource *resource;
	char name[10];
	struct bpu_core *core;
	int32_t ret;
	int32_t i, j;

	core = devm_kzalloc(&pdev->dev, sizeof(struct bpu_core), GFP_KERNEL);
	if (!core) {
		dev_err(&pdev->dev, "Can't alloc bpu core mem\n");
		return -ENOMEM;
	}

	mutex_init(&core->mutex_lock);
	spin_lock_init(&core->spin_lock);
	INIT_LIST_HEAD(&core->user_list);

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource) {
		dev_err(&pdev->dev, "Can't get bpu core resource error\n");
		return -ENODEV;
	}

	core->base = devm_ioremap_resource(&pdev->dev, resource);
	if (IS_ERR(core->base)) {
		dev_err(&pdev->dev, "Can't get bpu core resource failed\n");
		return PTR_ERR(core->base);
	}

	ret = of_property_read_u32(np, "cnn-id", &core->index);
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't get bpu core index\n");
		return ret;
	}

	core->regulator = devm_regulator_get(&pdev->dev, "cnn");
	if (IS_ERR(core->regulator)) {
		dev_err(&pdev->dev, "Can't get bpu core regulator\n");
		return PTR_ERR(core->regulator);
	}

	core->aclk = devm_clk_get(&pdev->dev, "cnn_aclk");
	if (IS_ERR(core->aclk) || !core->aclk) {
		dev_err(&pdev->dev, "Can't get bpu core aclk\n");
		return PTR_ERR(core->aclk);
	}

	core->mclk = devm_clk_get(&pdev->dev, "cnn_mclk");
	if (IS_ERR(core->mclk) || !core->mclk) {
		dev_err(&pdev->dev, "Can't get bpu core mclk\n");
		return PTR_ERR(core->mclk);
	}

	core->rst = devm_reset_control_get(&pdev->dev, "cnn_rst");
	if (IS_ERR(core->rst)) {
		dev_err(&pdev->dev, "Can't get bpu core rst\n");
		return PTR_ERR(core->rst);
	}

	for (i = 0; i < BPU_PRIO_NUM; i++) {
		ret = kfifo_alloc(&core->run_fc_fifo[i], FC_MAX_DEPTH, GFP_KERNEL);
		if (ret) {
			for (j = 0; j < i; j++)
				kfifo_free(&core->run_fc_fifo[j]);
			dev_err(&pdev->dev, "Can't bpu core run fifo buffer\n");
			return ret;
		}
	}

	/* done fc fifo mainly for debug */
	ret = kfifo_alloc(&core->done_fc_fifo, BPU_CORE_RECORE_NUM, GFP_KERNEL);
	if (ret) {
		dev_err(&pdev->dev, "Can't bpu core done fifo buffer\n");
		goto err0;
	}

	core->irq = irq_of_parse_and_map(np, 0);
	if (core->irq < 0) {
		dev_err(&pdev->dev, "Can't find bpu core irq\n");
		ret = core->irq;
		goto err1;
	}

	ret = devm_request_irq(&pdev->dev, core->irq, bpu_core_irq_handler,
			0, NULL, core);
	if (ret) {
		dev_err(&pdev->dev, "request '%d' for bpu core failed with %d\n",
			core->irq, ret);
		goto err1;
	}

	disable_irq(core->irq);

	snprintf(name, 10, "bpu_core%d", core->index);

	core->miscdev.minor	= MISC_DYNAMIC_MINOR;
	core->miscdev.name	= name;
	core->miscdev.fops	= &bpu_core_fops;

	ret = misc_register(&core->miscdev);
	if (ret) {
		dev_err(&pdev->dev, "Register bpu core device failed\n");
		goto err1;
	}

	atomic_set(&core->open_counter, 0);

	atomic_set(&core->pend_flag, 0);
	core->running_task_num = 0;
	init_completion(&core->no_task_comp);

	tasklet_init(&core->tasklet, bpu_core_tasklet, (unsigned long)core);

	ret = bpu_core_register(core);
	if (ret) {
		dev_err(&pdev->dev, "Register bpu core to bpu failed\n");
		goto err2;
	}

#ifdef CONFIG_X2_BPU
	core->hw_ops = &x2_hw_ops;
#elif defined CONFIG_J5_BPU
	core->hw_ops = &j5_hw_ops;
#endif
	if (!core->hw_ops) {
		dev_err(&pdev->dev, "No bpu core hardware ops\n");
		goto err3;
	}

	if (!core->hw_ops->write_fc || !core->hw_ops->read_fc) {
		dev_err(&pdev->dev, "No bpu core hardware fc ops\n");
		goto err3;
	}

	/* 0 is not limit, just by the max fifo length */
	core->fc_buf_limit = 0;
	core->dev = &pdev->dev;

	bpu_core_create_sys(core);

	dev_set_drvdata(&pdev->dev, core);

	bpu_core_dvfs_register(core, NULL);

	return 0;

err3:
	bpu_core_unregister(core);
err2:
	misc_deregister(&core->miscdev);
err1:
	kfifo_free(&core->done_fc_fifo);
err0:
	for (i = 0; i < BPU_PRIO_NUM; i++)
		kfifo_free(&core->run_fc_fifo[i]);

	return ret;
}

static int bpu_core_remove(struct platform_device *pdev)
{
	struct bpu_core *core = dev_get_drvdata(&pdev->dev);
	int32_t i;

	bpu_core_dvfs_unregister(core);
	bpu_core_discard_sys(core);

	for (i = 0; i < BPU_PRIO_NUM; i++)
		kfifo_free(&core->run_fc_fifo[i]);
	kfifo_free(&core->done_fc_fifo);

	misc_deregister(&core->miscdev);

	bpu_core_unregister(core);

	return 0;
}

static const struct of_device_id bpu_core_of_match[] = {
	{ .compatible = "hobot,x2-cnn-host",},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bpu_core_of_match);

static struct platform_driver bpu_core_platform_driver = {
	.probe	 = bpu_core_probe,
	.remove  = bpu_core_remove,
	.driver  = {
		.name = "bpu-core",
		.of_match_table = bpu_core_of_match,
#ifdef CONFIG_PM
		.pm = &bpu_core_pm_ops,
#endif
	},
};

static int __init bpu_core_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&bpu_core_platform_driver);
	if (ret)
		pr_err("BPU Core driver register failed\n");

	return ret;
}

static void __exit bpu_core_exit(void)
{
	platform_driver_unregister(&bpu_core_platform_driver);
}

module_init(bpu_core_init);
module_exit(bpu_core_exit);

MODULE_DESCRIPTION("Driver for Horizon BPU Process Core");
MODULE_AUTHOR("Zhang Guoying<guoying.zhang@horizon.ai>");
MODULE_LICENSE("GPL v2");
