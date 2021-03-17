/*
 * Copyright (C) 2019 Horizon Robotics
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

#define NAME_LEN		10
#define ID_LOOP_GAP		10
#define ID_MASK			(0xFFFFFFFFu)
#define STATE_SHIFT		(32u)

/* tasklet just to do statistics work */
static void bpu_core_tasklet(unsigned long data)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)data;/*PRQA S ALL*/
	struct bpu_user *tmp_user;
	struct bpu_fc tmp_bpu_fc;
	uint32_t tmp_hw_id, err;
	int32_t lost_report;
	int32_t ret;

	if (core == NULL) {
		return;
	}

	tmp_hw_id = (uint32_t)(core->done_hw_id & ID_MASK);
	err = (uint32_t)(core->done_hw_id >> STATE_SHIFT);

	do {
		lost_report = 0;
		if (err == 0u) {
			ret = kfifo_get(&core->run_fc_fifo[FC_PRIO(tmp_hw_id)], &tmp_bpu_fc);/*PRQA S ALL*/
			if (ret < 1) {
				bpu_prio_trig_out(core->prio_sched);
				core->done_hw_id = 0;
				bpu_sched_seed_update();
				dev_err(core->dev,
						"bpu core no fc bufferd, when normal[%d], lost = %d\n",
						tmp_hw_id, lost_report);
				return;
			}
			bpu_prio_trig_out(core->prio_sched);
			/* data has been no use fow hw, so clear data */
			bpu_fc_clear(&tmp_bpu_fc);

			/* maybe lost a hw irq */
			if (tmp_bpu_fc.hw_id < tmp_hw_id) {
				lost_report = 1;
			}

			if (tmp_bpu_fc.hw_id > tmp_hw_id) {
				if (((int32_t)tmp_bpu_fc.hw_id - (int32_t)tmp_hw_id) > (HW_ID_MAX - ID_LOOP_GAP)) {
					lost_report = 1;
				}
			}
		} else {
			ret = kfifo_peek(&core->run_fc_fifo[FC_PRIO(tmp_hw_id)], &tmp_bpu_fc);/*PRQA S ALL*/
			if (ret < 1) {
				core->done_hw_id = 0;
				bpu_sched_seed_update();
				dev_err(core->dev,
						"bpu core no fc bufferd, when error[%d]\n", ret);
				return;
			}
			tmp_bpu_fc.info.status = err;
		}

		/* update the statistics element */
		bpu_core_update(core, &tmp_bpu_fc);

		tmp_bpu_fc.info.run_c_mask = ((uint64_t)0x1 << (uint32_t)core->index);
		tmp_user = bpu_get_user(&tmp_bpu_fc);
		if (tmp_user != NULL) {
			if (tmp_user->is_alive > 0u) {
				if (kfifo_is_full(&tmp_user->done_fcs)) {/*PRQA S ALL*/
					dev_err(core->dev, "user[%d] (%d)read data too Slow\n",
							kfifo_len(&tmp_user->done_fcs), tmp_user->id);/*PRQA S ALL*/
					kfifo_skip(&tmp_user->done_fcs);/*PRQA S ALL*/
				}

				ret = kfifo_in(&tmp_user->done_fcs, &tmp_bpu_fc.info, 1);/*PRQA S ALL*/
				if (ret < 1) {
					core->done_hw_id = 0;
					bpu_sched_seed_update();
					dev_err(core->dev, "bpu buffer bind user error\n");
					return;
				}
				wake_up_interruptible(&tmp_user->poll_wait);/*PRQA S ALL*/
			}
			tmp_user->running_task_num--;

			if ((tmp_user->running_task_num <= 0) && (tmp_user->is_alive == 0u)) {
				complete(&tmp_user->no_task_comp);
			}
		}

		core->running_task_num--;
		if (core->running_task_num <= 0) {
			complete(&core->no_task_comp);
		}

		if (kfifo_is_full(&core->done_fc_fifo)) {/*PRQA S ALL*/
			kfifo_skip(&core->done_fc_fifo);/*PRQA S ALL*/
		}

		ret = kfifo_in(&core->done_fc_fifo, &tmp_bpu_fc, 1);/*PRQA S ALL*/
		if (ret < 1) {
			core->done_hw_id = 0;
			bpu_sched_seed_update();
			dev_err(core->dev, "bpu buffer bind user error\n");
			return;
		}
	} while (lost_report != 0);

	core->done_hw_id = 0;
	/* core ratio will be update by group check */
	bpu_sched_seed_update();
}

static irqreturn_t bpu_core_irq_handler(int irq, void *dev_id)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)dev_id;/*PRQA S ALL*/
	uint32_t tmp_hw_id, err;
	int32_t ret;

	if (atomic_read(&core->host->open_counter) == 0) {/*PRQA S ALL*/
		return IRQ_HANDLED;
	}

	bpu_prio_trig_out(core->prio_sched);

	spin_lock(&core->spin_lock);/*PRQA S ALL*/
	ret = core->hw_ops->read_fc(core, &tmp_hw_id, &err);
	if (ret <= 0) {
		spin_unlock(&core->spin_lock);
		if (ret < 0) {
			dev_err(core->dev, "BPU read hardware core error\n");
		}
		return IRQ_HANDLED;
	}
	spin_unlock(&core->spin_lock);
	pr_debug("BPU Core[%d] irq %d come\n", core->index, tmp_hw_id);/*PRQA S ALL*/

	if (tmp_hw_id == (uint32_t)HW_ID_MAX) {
		return IRQ_HANDLED;
	}

	core->done_hw_id = ((uint64_t)err << STATE_SHIFT) | (uint64_t)tmp_hw_id;

	tasklet_schedule(&core->tasklet);

	return IRQ_HANDLED;
}

static long bpu_core_ioctl(struct file *filp,/*PRQA S ALL*/
		unsigned int cmd, unsigned long arg)/*PRQA S ALL*/
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;/*PRQA S ALL*/
	struct bpu_core *core = (struct bpu_core *)user->host;/*PRQA S ALL*/
	uint32_t ratio;
	uint16_t cap;
	int16_t level;
	uint32_t limit;
	uint64_t clock;
	uint8_t type;

	int32_t ret = 0;
	int32_t i;

	switch (cmd) {
	case BPU_GET_RATIO:/*PRQA S ALL*/
		ratio = bpu_core_ratio(core);
		if (copy_to_user((void __user *)arg, &ratio, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			dev_err(core->dev, "copy data to userspace failed\n");
			return -EFAULT;
		}
		break;
	case BPU_GET_CAP:/*PRQA S ALL*/
		/* get the lowest prio fifo size to user */
		if (bpu_core_is_online(core) || (core->hotplug > 0u)) {
			cap = kfifo_avail(&core->run_fc_fifo[0]);/*PRQA S ALL*/
		} else {
			cap = 0;
		}
		if (copy_to_user((void __user *)arg, &cap, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			dev_err(core->dev, "copy data to userspace failed\n");
			return -EFAULT;
		}
		break;
	case BPU_SET_POWER:/*PRQA S ALL*/
		level = 0;
		if (copy_from_user(&level, (void __user *)arg, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			dev_err(core->dev, "copy data from userspace failed\n");
			return -EFAULT;
		}
		if (level <= 0) {
			mutex_lock(&core->mutex_lock);
			ret = bpu_core_disable(core);
			mutex_unlock(&core->mutex_lock);
			if (ret != 0) {
				dev_err(core->dev, "Disable BPU core%d failed\n", core->index);
				return ret;
			}
		} else {
			mutex_lock(&core->mutex_lock);
			ret = bpu_core_enable(core);
			mutex_unlock(&core->mutex_lock);
			if (ret != 0) {
				dev_err(core->dev, "Enable BPU core%d failed\n", core->index);
				return ret;
			}
		}
		break;
	case BPU_SET_FREQ_LEVEL:/*PRQA S ALL*/
		level = 0;
		if (copy_from_user(&level, (void __user *)arg, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			dev_err(core->dev, "copy data from userspace failed\n");
			return -EFAULT;
		}
		mutex_lock(&core->mutex_lock);
		ret = bpu_core_set_freq_level(core, level);
		mutex_unlock(&core->mutex_lock);
		if (ret != 0) {
			dev_err(core->dev, "Set BPU core%d freq level(%d)failed\n",
					core->index, level);
			return ret;
		}
		break;
	case BPU_GET_FREQ_LEVEL:/*PRQA S ALL*/
		level = 0;
#if defined(CONFIG_PM_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
		if (core->dvfs != NULL) {
			for (i = 0; i < (int32_t)core->dvfs->level_num; i++) {
				if (core->dvfs->rate == core->dvfs->profile.freq_table[i]) {
					level = (int16_t)(i - (int32_t)core->dvfs->level_num + 1);
				}
			}
		}
#endif
		if (copy_to_user((void __user *)arg, &level, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			dev_err(core->dev, "copy data to userspace failed\n");
			return -EFAULT;
		}
		break;
	case BPU_GET_FREQ_LEVEL_NUM:/*PRQA S ALL*/
		level = 1;
#if defined(CONFIG_PM_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
		if (core->dvfs != NULL) {
			level = (int16_t)core->dvfs->level_num;
		}
#endif
		if (copy_to_user((void __user *)arg, &level, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			dev_err(core->dev, "copy data to userspace failed\n");
			return -EFAULT;
		}
		break;
	case BPU_SET_CLK:/*PRQA S ALL*/
		clock = 0;
		if (copy_from_user(&clock, (void __user *)arg, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			dev_err(core->dev, "copy data from userspace failed\n");
			return -EFAULT;
		}
		mutex_lock(&core->mutex_lock);
		ret = bpu_core_set_clk(core, clock);
		mutex_unlock(&core->mutex_lock);
		if (ret != 0) {
			dev_err(core->dev, "Set BPU core%d clock(%lld)failed\n",
					core->index, clock);
			return ret;
		}
		break;
	case BPU_GET_CLK:/*PRQA S ALL*/
		clock = bpu_core_get_clk(core);
		if (copy_to_user((void __user *)arg, &clock, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			dev_err(core->dev, "copy data to userspace failed\n");
			return -EFAULT;
		}
		break;
	case BPU_RESET:/*PRQA S ALL*/
		mutex_lock(&core->mutex_lock);
		ret = bpu_core_reset(core);
		mutex_unlock(&core->mutex_lock);
		break;
	case BPU_SET_LIMIT:/*PRQA S ALL*/
		if (copy_from_user(&limit, (void __user *)arg, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			pr_err("%s: copy data failed from userspace\n", __func__);/*PRQA S ALL*/
			return -EFAULT;
		}
		ret = bpu_core_set_limit(core, (int32_t)limit);
		if (ret != 0) {
			dev_err(core->dev, "BPU core set prio limit failed\n");
		}
		break;
	case BPU_CORE_TYPE:
		type = bpu_core_type(core);
		if (copy_to_user((void __user *)arg, &type, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			dev_err(core->dev, "copy data to userspace failed\n");
			return -EFAULT;
		}
		break;
	default:
		pr_err("%s: BPU invalid ioctl argument\n", __func__);/*PRQA S ALL*/
		ret = -EINVAL;
		break;
	}

	return ret;
}

static unsigned int bpu_core_poll(struct file *filp, poll_table *wait)/*PRQA S ALL*/
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;/*PRQA S ALL*/
	uint32_t mask = 0;

	poll_wait(filp, &user->poll_wait, wait);
	mutex_lock(&user->mutex_lock);

	if (kfifo_len(&user->done_fcs)) {/*PRQA S ALL*/
		mask |= POLLIN | POLLRDNORM;/*PRQA S ALL*/
	}

	mutex_unlock(&user->mutex_lock);

	return mask;
}

static ssize_t bpu_core_read(struct file *filp,/*PRQA S ALL*/
		char __user *buf, size_t len, loff_t *f_pos)/*PRQA S ALL*/
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;/*PRQA S ALL*/
	struct bpu_core *core = (struct bpu_core *)user->host;/*PRQA S ALL*/

	return bpu_read_with_user(core, user, buf, len);
}

static ssize_t bpu_core_write(struct file *filp,/*PRQA S ALL*/
		const char __user *buf, size_t len, loff_t *f_pos)/*PRQA S ALL*/
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;/*PRQA S ALL*/
	struct bpu_core *core = (struct bpu_core *)user->host;/*PRQA S ALL*/

	return bpu_write_with_user(core, user, buf, len);
}

static int bpu_core_open(struct inode *inode, struct file *filp)/*PRQA S ALL*/
{
	struct bpu_core *core =
		(struct bpu_core *)container_of(filp->private_data, struct bpu_core, miscdev);/*PRQA S ALL*/
	struct bpu_user *user;
	int32_t ret;
	uint32_t i;

	mutex_lock(&core->mutex_lock);
	if (atomic_read(&core->open_counter) == 0) {/*PRQA S ALL*/
		/* first open init something files */
		core->prio_sched = bpu_prio_init(core, CONFIG_BPU_PRIO_NUM);
		if (core->prio_sched == NULL) {
			mutex_unlock(&core->mutex_lock);
			dev_err(core->dev, "Init bpu core prio sched failed\n");
			return -EINVAL;
		}
#ifdef CONFIG_X3_BPU
		pm_qos_add_request(&core->pm_qos_req, PM_QOS_DEVFREQ, 10000);
#endif
		ret = bpu_core_enable(core);
		if (ret != 0) {
			mutex_unlock(&core->mutex_lock);
			dev_err(core->dev, "Can't bpu core hw enable failed\n");
			return ret;
		}
		for (i = 0; i < BPU_PRIO_NUM; i++) {
			atomic_set(&core->hw_id_counter[i], 1);/*PRQA S ALL*/
		}

		core->fc_buf_limit = 0;
	} else {
		if (core->hw_enabled == 0u) {
			ret = bpu_core_enable(core);
			if (ret != 0) {
				mutex_unlock(&core->mutex_lock);
				dev_err(core->dev, "Can't bpu core hw enable failed\n");
				return ret;
			}
		}
	}

	user = (struct bpu_user *)kzalloc(sizeof(struct bpu_user), GFP_KERNEL);/*PRQA S ALL*/
	if (user == NULL) {
		mutex_unlock(&core->mutex_lock);
		dev_err(core->dev, "Can't bpu user mem\n");
		return -ENOMEM;
	}

	user->id = (uint32_t)task_pid_nr(current->group_leader);

	/* init fifo which report to userspace */
	ret = kfifo_alloc(&user->done_fcs, BPU_CORE_RECORE_NUM, GFP_KERNEL);/*PRQA S ALL*/
	if (ret != 0) {
		mutex_unlock(&core->mutex_lock);
		kfree((void *)user);/*PRQA S ALL*/
		dev_err(core->dev, "Can't bpu user fifo buffer\n");
		return -ret;
	}

	init_waitqueue_head(&user->poll_wait);/*PRQA S ALL*/
	user->host = (void *)core;/*PRQA S ALL*/
	user->p_file_private = &filp->private_data;
	list_add((struct list_head *)user, &core->user_list);/*PRQA S ALL*/
	spin_lock_init(&user->spin_lock);/*PRQA S ALL*/
	mutex_init(&user->mutex_lock);/*PRQA S ALL*/
	init_completion(&user->no_task_comp);/*PRQA S ALL*/
	user->is_alive = 1;
	user->running_task_num = 0;
	/* replace user the private to store user */
	filp->private_data = (void *)user;/*PRQA S ALL*/

	atomic_inc(&core->open_counter);/*PRQA S ALL*/
	mutex_unlock(&core->mutex_lock);
	return 0;
}

static int bpu_core_release(struct inode *inode, struct file *filp)/*PRQA S ALL*/
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;/*PRQA S ALL*/
	struct bpu_core *core = (struct bpu_core *)user->host;/*PRQA S ALL*/
	unsigned long flags;/*PRQA S ALL*/
	int32_t core_work_state = 1;
	struct bpu_fc tmp_bpu_fc;
	int32_t ret;
	uint32_t i;

	user->is_alive = 0;
	/* wait user running fc done */
	while((user->running_task_num > 0) && (core->hw_enabled > 0)) {
		if (core->hw_ops->status != NULL) {
			(void)core->hw_ops->status(core, UPDATE_STATE);
			core_work_state = core->hw_ops->status(core, WORK_STATE);
		}

		/* timeout to prevent bpu hung make system hung*/
		if((wait_for_completion_timeout(&user->no_task_comp, HZ) == 0u)
				&& (core_work_state > 0)) {
			if (core->hw_ops->status != NULL) {
				(void)core->hw_ops->status(core, UPDATE_STATE);
				ret = core->hw_ops->status(core, WORK_STATE);
				if (ret == core_work_state) {
					/* if states between wait are same, break to not wait*/
					break;
				}
			}
			user->running_task_num--;
		} else {
			break;
		}
	}

	mutex_lock(&core->mutex_lock);
	atomic_dec(&core->open_counter);/*PRQA S ALL*/

	if (atomic_read(&core->open_counter) == 0) {/*PRQA S ALL*/
		/* release the real bpu core */
		bpu_prio_exit(core->prio_sched);
		core->prio_sched = NULL;
		ret = bpu_core_disable(core);
		if (ret != 0) {
			dev_err(core->dev, "BPU core disable failed\n");
		}
#ifdef CONFIG_X3_BPU
		pm_qos_remove_request(&core->pm_qos_req);
#endif

		spin_lock_irqsave(&core->spin_lock, flags);/*PRQA S ALL*/
		core->p_run_time = 0;
		core->ratio = 0;
		for (i = 0; i < BPU_PRIO_NUM; i++) {
			while (!kfifo_is_empty(&core->run_fc_fifo[i])) {
				ret = kfifo_get(&core->run_fc_fifo[i], &tmp_bpu_fc);/*PRQA S ALL*/
				if (ret < 1) {
					continue;
				}
				bpu_fc_clear(&tmp_bpu_fc);
			}
			kfifo_reset(&core->run_fc_fifo[i]);/*PRQA S ALL*/
		}
		spin_unlock_irqrestore(&core->spin_lock, flags);
	}
	mutex_unlock(&core->mutex_lock);

	spin_lock_irqsave(&core->spin_lock, flags);/*PRQA S ALL*/
	list_del((struct list_head *)user);/*PRQA S ALL*/
	kfifo_free(&user->done_fcs);/*PRQA S ALL*/
	kfree((void *)user);/*PRQA S ALL*/
	filp->private_data = NULL;
	spin_unlock_irqrestore(&core->spin_lock, flags);

	return 0;
}

static const struct file_operations bpu_core_fops = {/*PRQA S ALL*/
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
static int bpu_core_suspend(struct device *dev)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/
	uint16_t tmp_hw_state;
	int32_t ret;

	mutex_lock(&core->mutex_lock);
	tmp_hw_state = core->hw_enabled;
	ret = bpu_core_disable(core);
	if (ret != 0) {
		mutex_unlock(&core->mutex_lock);
		dev_err(dev, "BPU core%d suspend failed\n", core->index);
		return ret;
	}
	core->hw_enabled = tmp_hw_state;
	mutex_unlock(&core->mutex_lock);

	return 0;
}

static int bpu_core_resume(struct device *dev)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/
	int32_t ret;

	mutex_lock(&core->mutex_lock);
	if (core->hw_enabled == 0u) {
		mutex_unlock(&core->mutex_lock);
		return 0;
	}
	ret = bpu_core_enable(core);
	if (ret != 0) {
		mutex_unlock(&core->mutex_lock);
		dev_err(dev, "BPU core%d resume failed\n", core->index);
		return ret;
	}

	ret = bpu_core_process_recover(core);
	if (ret != 0) {
		mutex_unlock(&core->mutex_lock);
		dev_err(dev, "BPU core%d recovery failed\n", core->index);
		return ret;
	}
	mutex_unlock(&core->mutex_lock);
	bpu_prio_trig_out(core->prio_sched);

	return ret;
}

static SIMPLE_DEV_PM_OPS(bpu_core_pm_ops,/*PRQA S ALL*/
			 bpu_core_suspend, bpu_core_resume);
#endif

static int32_t bpu_core_parse_dts(struct platform_device *pdev, struct bpu_core *core)
{
	struct device_node *np;
	struct resource *resource;
	int32_t ret;

	if (core == NULL) {
		pr_err("NO BPU Core parse DeviceTree\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	if (pdev == NULL) {
		pr_err("BPU Core not Bind Device\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	np = pdev->dev.of_node;
	core->dev = &pdev->dev;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (resource == NULL) {
		dev_err(&pdev->dev, "Can't get bpu core resource error\n");
		return -ENODEV;
	}

	core->base = devm_ioremap_resource(&pdev->dev, resource);
	if (IS_ERR(core->base)) {/*PRQA S ALL*/
		dev_err(&pdev->dev, "Can't get bpu core resource failed\n");
		return PTR_ERR(core->base);/*PRQA S ALL*/
	}

	/* try to get the extra mem resorece */
	resource = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (resource != NULL) {
		core->reserved_base = devm_ioremap(&pdev->dev,
			resource->start, resource_size(resource));
		if (IS_ERR(core->reserved_base)) {/*PRQA S ALL*/
			dev_err(&pdev->dev, "Can't get bpu core extra resource failed[%ld]\n",
					PTR_ERR(core->reserved_base));
			core->reserved_base = NULL;
		}
	} else {
		core->reserved_base = NULL;
	}

	ret = of_property_read_u32(np, "cnn-id", &core->index);/*PRQA S ALL*/
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't get bpu core index\n");
		return ret;
	}

	core->regulator = devm_regulator_get(&pdev->dev, "cnn");
	if (IS_ERR(core->regulator)) {/*PRQA S ALL*/
		/* some platform not has regulator, so just report error info */
		core->regulator = NULL;
		dev_err(&pdev->dev, "Can't get bpu core regulator\n");
	}

	core->aclk = devm_clk_get(&pdev->dev, "cnn_aclk");
	if (IS_ERR(core->aclk) || (core->aclk == NULL)) {/*PRQA S ALL*/
		/* some platform not has aclk, so just report error info */
		core->aclk = NULL;
		dev_err(&pdev->dev, "Can't get bpu core aclk\n");
	}

	core->mclk = devm_clk_get(&pdev->dev, "cnn_mclk");
	if (IS_ERR(core->mclk) || (core->mclk == NULL)) {/*PRQA S ALL*/
		/* some platform not has mclk, so just report error info */
		core->mclk = NULL;
		dev_err(&pdev->dev, "Can't get bpu core mclk\n");
	}

	core->rst = devm_reset_control_get(&pdev->dev, "cnn_rst");
	if (IS_ERR(core->rst)) {/*PRQA S ALL*/
		/* some platform not has rst, so just report error info */
		core->rst = NULL;
		dev_err(&pdev->dev, "Can't get bpu core rst\n");
	}

	core->irq = irq_of_parse_and_map(np, 0);
	if (core->irq <= 0u) {
		dev_err(&pdev->dev, "Can't find bpu core irq\n");
		return -EFAULT;
	}

	return 0;
}

static int32_t bpu_core_alloc_fifos(struct bpu_core *core)
{
	uint32_t i, j;
	int32_t ret;

	if (core == NULL) {
		pr_err("NO BPU Core to alloc fifos\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	for (i = 0; i < BPU_PRIO_NUM; i++) {
		ret = kfifo_alloc(&core->run_fc_fifo[i], FC_MAX_DEPTH, GFP_KERNEL);/*PRQA S ALL*/
		if (ret != 0) {
			for (j = 0; j < i; j++) {/*PRQA S ALL*/
				kfifo_free(&core->run_fc_fifo[j]);/*PRQA S ALL*/
			}
			dev_err(core->dev, "Can't bpu core run fifo buffer\n");
			return ret;
		}
	}

	/* done fc fifo mainly for debug */
	ret = kfifo_alloc(&core->done_fc_fifo, BPU_CORE_RECORE_NUM, GFP_KERNEL);/*PRQA S ALL*/
	if (ret != 0) {
		for (i = 0; i < BPU_PRIO_NUM; i++) {
			kfifo_free(&core->run_fc_fifo[i]);/*PRQA S ALL*/
		}
		dev_err(core->dev, "Can't bpu core done fifo buffer\n");
		return ret;
	}

	return ret;
}

static void bpu_core_free_fifos(struct bpu_core *core)
{
	struct bpu_fc tmp_bpu_fc;
	uint32_t i;
	int32_t ret;

	if (core == NULL) {
		return;
	}

	kfifo_free(&core->done_fc_fifo);/*PRQA S ALL*/
	for (i = 0; i < BPU_PRIO_NUM; i++) {
		while (!kfifo_is_empty(&core->run_fc_fifo[i])) {
			ret = kfifo_get(&core->run_fc_fifo[i], &tmp_bpu_fc);/*PRQA S ALL*/
			if (ret < 1) {
				continue;
			}
			bpu_fc_clear(&tmp_bpu_fc);
		}
		kfifo_free(&core->run_fc_fifo[i]);/*PRQA S ALL*/
	}
}

static int bpu_core_probe(struct platform_device *pdev)/*PRQA S ALL*/
{
	struct bpu_core *core;
	char name[NAME_LEN];
	int32_t ret;

	core = (struct bpu_core *)devm_kzalloc(&pdev->dev, sizeof(struct bpu_core), GFP_KERNEL);/*PRQA S ALL*/
	if (core == NULL) {
		dev_err(&pdev->dev, "Can't alloc bpu core mem\n");
		return -ENOMEM;
	}

	core->dev = &pdev->dev;

	mutex_init(&core->mutex_lock);/*PRQA S ALL*/
	spin_lock_init(&core->spin_lock);/*PRQA S ALL*/
	INIT_LIST_HEAD(&core->user_list);

	ret = bpu_core_parse_dts(pdev, core);
	if (ret != 0) {
		dev_err(&pdev->dev, "BPU Core parse dts failed\n");
		return ret;
	}

	ret = bpu_core_alloc_fifos(core);
	if (ret != 0) {
		dev_err(&pdev->dev, "BPU Core alloc fifos failed\n");
		return ret;
	}

	ret = devm_request_irq(&pdev->dev, core->irq, bpu_core_irq_handler,
			0, NULL, core);/*PRQA S ALL*/
	if (ret != 0) {
		bpu_core_free_fifos(core);
		dev_err(&pdev->dev, "request '%d' for bpu core failed with %d\n",
			core->irq, ret);
		return ret;
	}

	disable_irq(core->irq);

	ret = snprintf(name, NAME_LEN, "bpu_core%d", core->index);
	if (ret <= 0) {
		dev_err(&pdev->dev, "BPU core%d name create failed\n", core->index);
	}

	core->miscdev.minor	= MISC_DYNAMIC_MINOR;
	core->miscdev.name	= name;
	core->miscdev.fops	= &bpu_core_fops;

	ret = misc_register(&core->miscdev);
	if (ret != 0) {
		bpu_core_free_fifos(core);
		dev_err(&pdev->dev, "Register bpu core device failed\n");
		return ret;
	}

	atomic_set(&core->open_counter, 0);/*PRQA S ALL*/

	atomic_set(&core->pend_flag, 0);/*PRQA S ALL*/
	core->running_task_num = 0;
	init_completion(&core->no_task_comp);/*PRQA S ALL*/

	tasklet_init(&core->tasklet, bpu_core_tasklet, (unsigned long)core);/*PRQA S ALL*/

	ret = bpu_core_register(core);
	if (ret != 0) {
		misc_deregister(&core->miscdev);
		bpu_core_free_fifos(core);
		dev_err(&pdev->dev, "Register bpu core to bpu failed\n");
		return ret;
	}

	core->hw_ops = &hw_ops;
	if (core->hw_ops == NULL) {
		bpu_core_unregister(core);
		misc_deregister(&core->miscdev);
		bpu_core_free_fifos(core);
		dev_err(&pdev->dev, "No bpu core hardware ops\n");
		return ret;
	}

	if ((core->hw_ops->write_fc == NULL) || (core->hw_ops->read_fc == NULL)) {
		bpu_core_unregister(core);
		misc_deregister(&core->miscdev);
		bpu_core_free_fifos(core);
		dev_err(&pdev->dev, "No bpu core hardware fc ops\n");
		return ret;
	}

	/* 0 is not limit, just by the max fifo length */
	core->fc_buf_limit = 0;

	ret = bpu_core_create_sys(core);
	if (ret != 0) {
		dev_err(&pdev->dev, "BPU core registe sys failed\n");
	}

	dev_set_drvdata(&pdev->dev, core);/*PRQA S ALL*/

	ret = bpu_core_dvfs_register(core, NULL);
	if (ret != 0) {
		dev_err(&pdev->dev, "BPU core registe dvfs failed\n");
	}

	return 0;
}

static int bpu_core_remove(struct platform_device *pdev)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(&pdev->dev);/*PRQA S ALL*/

	bpu_core_dvfs_unregister(core);
	bpu_core_discard_sys(core);

	bpu_core_free_fifos(core);

	misc_deregister(&core->miscdev);

	bpu_core_unregister(core);

	return 0;
}

static const struct of_device_id bpu_core_of_match[] = {
	{ .compatible = "hobot,hobot-bpu", },
	{ /* sentinel */ }/*PRQA S ALL*/
};
MODULE_DEVICE_TABLE(of, bpu_core_of_match);/*PRQA S ALL*/

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

static int __init bpu_core_init(void)/*PRQA S ALL*/
{
	int32_t ret;

	ret = platform_driver_register(&bpu_core_platform_driver);  /* PRQA S ALL */
	if (ret != 0) {
		pr_err("BPU Core driver register failed\n");/*PRQA S ALL*/
	}

	return ret;
}

static void __exit bpu_core_exit(void)
{
	platform_driver_unregister(&bpu_core_platform_driver);
}

// PRQA S ALL ++
module_init(bpu_core_init);
module_exit(bpu_core_exit);
MODULE_DESCRIPTION("Driver for Horizon BPU Process Core");
MODULE_AUTHOR("Zhang Guoying<guoying.zhang@horizon.ai>");
MODULE_LICENSE("GPL v2");
// PRQA S ALL ++
