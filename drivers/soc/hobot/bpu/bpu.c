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

#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <asm/cacheflush.h>
#include "bpu.h"
#include "bpu_core.h"
#include "bpu_ctrl.h"

struct bpu *g_bpu;

/* create bpu_fc from user fc info*/
int bpu_fc_create_from_user(struct bpu_fc *fc,
		struct user_bpu_fc *user_fc, const void *data)
{
	int ret;

	if (!user_fc)
		return -EINVAL;

	if (!fc) {
		pr_err("bpu user fc need have space to place\n");
		return -EINVAL;
	}

	fc->info = *user_fc;
	fc->fc_data = NULL;
	ret = bpu_fc_bind_group(fc, user_fc->g_id);
	if (ret) {
		pr_err("bpu fc bind group failed\n");
		return ret;
	}
	
	if (data && (user_fc->length > 0)) {
		fc->fc_data = kmalloc(user_fc->length, GFP_KERNEL);
		if (!fc->fc_data) {
			pr_err("create bpu fc mem failed\n");
			return -ENOMEM;
		}

		if (copy_from_user(fc->fc_data, (void __user *)data,
					user_fc->length)) {
			kfree(fc->fc_data);
			fc->fc_data = NULL;
			pr_err("%s: copy fc data failed from userspace\n", __func__);
			return -EFAULT;
		}
	}

	return user_fc->length;
}

/* mainly clear fc data in bpu_fc*/
void bpu_fc_clear(struct bpu_fc *fc)
{
	if (!fc)
		return;

	if (fc->fc_data) {
		kfree(fc->fc_data);
		fc->fc_data = NULL;
		fc->info.length = 0;
	}
}

int bpu_fc_bind_user(struct bpu_fc *fc, struct bpu_user *user)
{
	if (!fc || !user) {
		pr_err("%s[%d]:bpu bind invalid fc or user\n", __func__, __LINE__);
		return -EINVAL;
	}

	/*
	 * use the pointer's point to user for judging
	 * whether valid of user after file released
	 */
	fc->user = user->p_file_private;

	return 0;
}

struct bpu_fc_group *bpu_find_group(uint32_t group_id)
{
	struct bpu_fc_group *tmp_group;
	struct bpu_fc_group *group = NULL;
	struct list_head *pos, *pos_n;

	list_for_each_safe(pos, pos_n, &g_bpu->group_list) {
		tmp_group = (struct bpu_fc_group *)pos;
		if (tmp_group) {
			if (tmp_group->id == group_id) {
				group = tmp_group;
				break;
			}
		}
	}

	return group;
}

struct bpu_fc_group *bpu_create_group(uint32_t group_id)
{
	struct bpu_fc_group *tmp_group = NULL;

	tmp_group = bpu_find_group(group_id);
	if (!tmp_group) {
		tmp_group = kzalloc(sizeof(struct bpu_fc_group), GFP_KERNEL);
		if (!tmp_group) {
			pr_err("BPU create group[%d] failed\n", group_id);
			return tmp_group;
		}
		tmp_group->id = group_id;

		if(kfifo_alloc(&tmp_group->buffer_fc_fifo, 50, GFP_KERNEL)) {
			kfree(tmp_group);
			pr_err("BPU create group[%d] fc buffer failed\n", group_id);
			return tmp_group;
		}

		spin_lock_init(&tmp_group->spin_lock);
		list_add((struct list_head *)tmp_group, &g_bpu->group_list);
	} else
		pr_info("BPU already have group[%d]\n", group_id);

	return tmp_group;
}

void bpu_delete_group(uint32_t group_id)
{
	struct bpu_fc_group *tmp_group;

	tmp_group = bpu_find_group(group_id);
	if (tmp_group) {
		kfifo_free(&tmp_group->buffer_fc_fifo);
		list_del((struct list_head *)tmp_group);
		kfree(tmp_group);
	}
}

int bpu_fc_bind_group(struct bpu_fc *fc, uint32_t group_id)
{
	struct bpu_fc_group *group = NULL;

	if (!g_bpu) {
		pr_err("%s[%d]:bpu not inited\n", __func__, __LINE__);
		return -ENODEV;
	}

	if (!fc) {
		pr_err("%s[%d]:bpu bind invalid fc\n", __func__, __LINE__);
		return -EINVAL;
	}

	group = bpu_find_group(group_id);
	if (group)
		fc->g_id = &group->id;
	else {
		pr_err("bpu fc find no correspond group\n");
		return -ENODEV;
	}

	return 0;
}

int bpu_write_prepare(struct bpu_user *user,
			const char __user *buf, size_t len,
			struct bpu_fc *bpu_fc)
{
	int tmp_raw_fc_len;
	int ret = 0;

	if (!user || !buf || !len) {
		pr_err("Write bpu buffer error\n");
		return -EINVAL;
	}

	if (len <= sizeof(struct user_bpu_fc)) {
		pr_err("Write invalied data\n");
		return -EINVAL;
	}

	tmp_raw_fc_len = bpu_fc_create_from_user(bpu_fc,
			(struct user_bpu_fc *)buf,
			buf + sizeof(struct user_bpu_fc));
	if (tmp_raw_fc_len <= 0) {
		pr_err("bpu fc from user error\n");
		return -EINVAL;
	}

	do_gettimeofday(&bpu_fc->start_point);

	ret = bpu_fc_bind_user(bpu_fc, user);
	if (ret < 0) {
		bpu_fc_clear(bpu_fc);
		pr_err("bpu buffer bind user error\n");
		return ret;
	}

	return tmp_raw_fc_len + sizeof(struct user_bpu_fc);
}

/*
 * write the user fc buffer to real core and bind user
 */
int bpu_write_fc_to_core(struct bpu_core *core,
		struct bpu_fc *bpu_fc, unsigned int offpos)
{
	struct bpu_user *tmp_user;
	unsigned long flags;
	int ret = 0;
	int write_fc_num;
	int prio = 0;

	if (!core || !bpu_fc) {
		dev_err(core->dev, "Write bpu buffer error\n");
		return -EINVAL;
	}

	tmp_user = bpu_get_user(bpu_fc);
	if (!tmp_user) {
		/* no user, so report fake complete */
		ret = bpu_fc->info.slice_num - offpos;
		bpu_fc_clear(bpu_fc);
		return ret;
	}

	if (!tmp_user->is_alive) {
		ret = bpu_fc->info.slice_num - offpos;
		bpu_fc_clear(bpu_fc);
		dev_err(core->dev, "bpu user now is not alive\n");
		return ret;
	}

	if (bpu_core_is_pending(core)) {
		dev_dbg(core->dev, "bpu core now pending\n");
		return -EBUSY;
	}

	/* write data to bpu fc range */
	if (core->hw_ops->write_fc) {
		prio = FC_PRIO(bpu_fc->info.priority);
		if (bpu_fc->info.id != 0) {
			bpu_fc->hw_id = atomic_read(&core->hw_id_counter[prio]);
			if (unlikely(bpu_fc->hw_id >= FC_ID(HW_ID_MAX))) {
				atomic_set(&core->hw_id_counter[prio], 1);
			} else
				atomic_inc(&core->hw_id_counter[prio]);
		}
		/* use the hw_id to tell soc id and priority */
		bpu_fc->hw_id = FC_PRIO_ID(prio, bpu_fc->hw_id);
		spin_lock_irqsave(&core->spin_lock, flags);
		write_fc_num = core->hw_ops->write_fc(core, bpu_fc, offpos);
		if (write_fc_num != (bpu_fc->info.slice_num - offpos)) {
			/* write raw fc to hw fifo not complete or error */
			spin_unlock_irqrestore(&core->spin_lock, flags);
			return write_fc_num;
		}
		if (bpu_fc->info.id != 0) {
			tmp_user->running_task_num++;
			core->running_task_num++;
			ret = kfifo_in(&core->run_fc_fifo[prio],
					bpu_fc, 1);
			if (ret < 1) {
				core->running_task_num--;
				tmp_user->running_task_num--;
				spin_unlock_irqrestore(&core->spin_lock, flags);
				dev_err(core->dev, "bpu request to fifo failed\n");
				return -EBUSY;
			}
		}
		spin_unlock_irqrestore(&core->spin_lock, flags);
	} else {
		bpu_fc_clear(bpu_fc);
		dev_err(core->dev, "no real bpu to process\n");
		return -ENODEV;
	}

	/* data has been set to hw, so clear data */
	bpu_fc_clear(bpu_fc);

	return write_fc_num;
}

static struct bpu_core *bpu_opt_core(struct bpu *bpu, uint64_t core_mask)
{
	struct bpu_core *tmp_core, *tmp_opt_core = NULL;
	struct list_head *pos, *pos_n;
	int tmp_val, tmp_last_val = -1;

	if (!bpu)
		return NULL;

	list_for_each_safe(pos, pos_n, &bpu->core_list) {
		tmp_core = (struct bpu_core*)pos;
		if (tmp_core) {
			if (core_mask & (0x1 << tmp_core->index)) {
				mutex_lock(&tmp_core->mutex_lock);
				if ((atomic_read(&tmp_core->open_counter) != 0) &&
						tmp_core->hw_enabled) {
					tmp_val = kfifo_avail(&tmp_core->run_fc_fifo[0]);
					if (tmp_val > tmp_last_val) {
						tmp_opt_core = tmp_core;
						tmp_last_val = tmp_val;
					}
				}
				mutex_unlock(&tmp_core->mutex_lock);
			}
		}
	}

	return tmp_opt_core;
}

int bpu_write_with_user(struct bpu_core *core,
			struct bpu_user *user,
			const char __user *buf, size_t len)
{
	uint64_t tmp_core_mask = ((struct user_bpu_fc *)buf)->core_mask;
	uint64_t tmp_run_c_mask = ((struct user_bpu_fc *)buf)->run_c_mask;
	struct bpu_core *tmp_core;
	struct bpu_fc tmp_bpu_fc;
	int use_len = 0;
	int prepare_len = 0;
	int ret;

	if (!user || !buf || !len) {
		dev_err(core->dev, "Write bpu buffer error\n");
		return -EINVAL;
	}

	if (len <= sizeof(struct user_bpu_fc)) {
		pr_err("Write invalied data\n");
		return -EINVAL;
	}

	while ((len - use_len) > sizeof(struct user_bpu_fc)) {
		prepare_len = bpu_write_prepare(user, buf + use_len,
				len - use_len, &tmp_bpu_fc);
		if (prepare_len <= 0) {
			pr_err("BPU prepare user write data!");
			return -EINVAL;
		}

		if (!core) {
			/*
			* choose optimal core according to core stauts
			* FIXME: need find core according core_mask flag
			*/
			mutex_lock(&g_bpu->mutex_lock);
			if (tmp_run_c_mask)
				tmp_core = bpu_opt_core(g_bpu, tmp_core_mask & tmp_run_c_mask);
			else
				tmp_core = bpu_opt_core(g_bpu, tmp_core_mask);
			if (!tmp_core) {
				mutex_unlock(&g_bpu->mutex_lock);
				pr_err("BPU has no suitable core, 0x%llx!", tmp_run_c_mask);
				return -ENODEV;
			}
			mutex_unlock(&g_bpu->mutex_lock);
		} else {
			tmp_core = core;
			tmp_bpu_fc.info.core_mask = (0x1 << core->index);
		}

		ret = bpu_prio_in(tmp_core->prio_sched, &tmp_bpu_fc);
		if (ret < 0) {
			dev_err(core->dev, "write bpu fc to core failed\n");
			return ret;
		}

		use_len += prepare_len;
	}

	return use_len;
}
EXPORT_SYMBOL(bpu_write_with_user);

/*
 * user read fc done fifo to userspace 
 */
int bpu_read_with_user(struct bpu_core *core,
			struct bpu_user *user,
			char __user *buf, size_t len)
{
	int ret, copied;

	if (!user || !buf || !len) {
		return 0;
	}

	if (!kfifo_initialized(&user->done_fcs)) {
		return -EINVAL;
	}

	mutex_lock(&user->mutex_lock);

	ret = kfifo_to_user(&user->done_fcs, buf, len, &copied);
	if (ret < 0) {
		mutex_unlock(&user->mutex_lock);
		return ret;
	}
	mutex_unlock(&user->mutex_lock);

	return copied;
}
EXPORT_SYMBOL(bpu_read_with_user);

static long bpu_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;
	struct list_head *pos, *pos_n;
	struct bpu_fc_group *group;
	union bpu_ioctl_arg data;
	struct bpu_core *tmp_core;

	switch (cmd) {
	case BPU_SET_GROUP:
		if (copy_from_user(&data, (void __user *)arg, _IOC_SIZE(cmd))) {
			pr_err("%s: copy data failed from userspace\n", __func__);
			return -EFAULT;
		}

		group = bpu_find_group(data.group.group_id);
		if (group) {
			if (data.group.prop <= 0) {
				bpu_delete_group(data.group.group_id);
				break;
			}
			group->proportion = data.group.prop;
		} else {
			if (data.group.prop <= 0)
				break;
			group = bpu_create_group(data.group.group_id);
			if (group)
				group->proportion = data.group.prop;
			else {
				pr_err("BPU create group(%d)(%d) prop(%d) error\n",
						GROUP_ID(data.group.group_id),
						GROUP_USER(data.group.group_id),
						data.group.prop);
				return -EINVAL;
			}
		}
		pr_debug("BPU set group(%d)(%d) prop [%d]\n",
				GROUP_ID(data.group.group_id),
				GROUP_USER(data.group.group_id),
				data.group.prop);
		break;
	case BPU_GET_GROUP:
		if (copy_from_user(&data, (void __user *)arg, _IOC_SIZE(cmd))) {
			pr_err("%s: copy data failed from userspace\n", __func__);
			return -EFAULT;
		}

		group = bpu_find_group(data.group.group_id);
		if (group) {
			data.group.prop = group->proportion;
			data.group.ratio = bpu_fc_group_ratio(group);
		}

		if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd))) {
			pr_err("copy data to userspace failed\n");
			return -EFAULT;
		}
		break;
	case BPU_GET_RATIO:
		data.ratio = bpu_ratio(g_bpu);
		if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd))) {
			pr_err("copy data to userspace failed\n");
			return -EFAULT;
		}
		break;
	case BPU_GET_CAP:
		data.cap = 0;
		mutex_lock(&g_bpu->mutex_lock);
		list_for_each_safe(pos, pos_n, &g_bpu->core_list) {
			tmp_core = (struct bpu_core*)pos;
			if (tmp_core)
				data.cap = max((int)kfifo_avail(&tmp_core->run_fc_fifo[0]),
						(int)data.cap);
		}
		mutex_unlock(&g_bpu->mutex_lock);
		if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd))) {
			pr_err("copy data to userspace failed\n");
			return -EFAULT;
		}
		break;
	case BPU_OPT_CORE:
		if (copy_from_user(&data, (void __user *)arg, _IOC_SIZE(cmd))) {
			pr_err("%s: copy data failed from userspace\n", __func__);
			return -EFAULT;
		}
		mutex_lock(&g_bpu->mutex_lock);
		tmp_core = bpu_opt_core(g_bpu, data.core);
		if (!tmp_core) {
			mutex_unlock(&g_bpu->mutex_lock);
			pr_err("Can't find an optimal BPU Core\n");
			return -EFAULT;
		}
		data.core = tmp_core->index;
		mutex_unlock(&g_bpu->mutex_lock);

		if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd))) {
			pr_err("copy data to userspace failed\n");
			return -EFAULT;
		}
		break;
	case BPU_RESET:
		list_for_each_safe(pos, pos_n, &g_bpu->core_list) {
			tmp_core = (struct bpu_core*)pos;
			if (tmp_core)
				bpu_core_reset(tmp_core);
		}
		break;
	case BPU_SET_LIMIT:
		if (copy_from_user(&data, (void __user *)arg, _IOC_SIZE(cmd))) {
			pr_err("%s: copy data failed from userspace\n", __func__);
			return -EFAULT;
		}
		list_for_each_safe(pos, pos_n, &g_bpu->core_list) {
			tmp_core = (struct bpu_core*)pos;
			if (tmp_core)
				bpu_core_set_limit(tmp_core, data.limit);
		}
		break;
	default:
		pr_err("%s: BPU invalid ioctl argument\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static unsigned int bpu_poll(struct file *filp, poll_table *wait)
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &user->poll_wait, wait);
	mutex_lock(&user->mutex_lock);

	if (kfifo_len(&user->done_fcs))
		mask |= POLLIN | POLLRDNORM;

	mutex_unlock(&user->mutex_lock);

	return mask;
}

static ssize_t bpu_read(struct file *filp,
		char __user *buf, size_t len, loff_t *f_pos)
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;

	return bpu_read_with_user(NULL, user, buf, len);
}

static ssize_t bpu_write(struct file *filp,
		const char __user *buf, size_t len, loff_t *f_pos)
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;

	return bpu_write_with_user(NULL, user, buf, len);
}

static int bpu_open(struct inode *inode, struct file *filp)
{
	struct bpu *bpu = container_of(filp->private_data,
					      struct bpu, miscdev);
	struct bpu_user *user;
	int ret;

	if (atomic_read(&bpu->open_counter) == 0) {
		/* first open init something files */
		bpu_sched_start(bpu);
	}

	user = kzalloc(sizeof(struct bpu_user), GFP_KERNEL);
	if (!user) {
		pr_err("Can't alloc user mem");
		return -ENOMEM;
	}

	user->id = task_pid_nr(current->group_leader);

	/* init fifo which report to userspace */
	ret = kfifo_alloc(&user->done_fcs, BPU_CORE_RECORE_NUM, GFP_KERNEL);
	if (ret) {
		kfree(user);
		pr_err("Can't alloc user fifo dev mem");
		return -ret;
	}

	init_waitqueue_head(&user->poll_wait);
	user->host = bpu;
	user->p_file_private = &filp->private_data;
	list_add((struct list_head *)user, &g_bpu->user_list);
	spin_lock_init(&user->spin_lock);
	mutex_init(&user->mutex_lock);
	init_completion(&user->no_task_comp);
	user->is_alive = 1;
	user->running_task_num = 0;
	/* replace user the private to store user */
	filp->private_data = user;

	atomic_inc(&bpu->open_counter);

	return 0;
}

static int bpu_release(struct inode *inode, struct file *filp)
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;
	struct bpu *bpu = (struct bpu *)user->host;
	struct list_head *pos, *pos_n;
	struct bpu_fc_group *tmp_group;
	unsigned long flags;

	user->is_alive = 0;
	/* wait user running fc done */
	while(user->running_task_num > 0) {
		/* timeout to prevent bpu hung make system hung*/
		if(!wait_for_completion_timeout(&user->no_task_comp, HZ))
			user->running_task_num--;
	}

	atomic_dec(&bpu->open_counter);

	if (atomic_read(&bpu->open_counter) == 0) {
		/* release the real bpu*/
		bpu_sched_stop(bpu);

		spin_lock_irqsave(&bpu->spin_lock, flags);
		list_for_each_safe(pos, pos_n, &g_bpu->group_list) {
			tmp_group = (struct bpu_fc_group *)pos;
			if (tmp_group)
				bpu_delete_group(tmp_group->id);
		}
		spin_unlock_irqrestore(&bpu->spin_lock, flags);
	}

	spin_lock_irqsave(&bpu->spin_lock, flags);
	list_del((struct list_head *)user);
	kfifo_free(&user->done_fcs);
	kfree(user);
	filp->private_data = NULL;
	spin_unlock_irqrestore(&bpu->spin_lock, flags);

	return 0;
}

static const struct file_operations bpu_fops = {
	.owner		= THIS_MODULE,
	.open		= bpu_open,
	.release	= bpu_release,
	.read		= bpu_read,
	.write		= bpu_write,
	.poll		= bpu_poll,
	.unlocked_ioctl = bpu_ioctl,
	.compat_ioctl = bpu_ioctl,
};

int bpu_core_register(struct bpu_core *core)
{
	if (!g_bpu) {
		pr_err("bpu not inited!\n");
		return -ENODEV;
	}

	if (!core) {
		pr_err("Register Invalid core!\n");
		return -EINVAL;
	}

	/* add to bpu core list */
	mutex_lock(&g_bpu->mutex_lock);
	list_add_tail((struct list_head *)core, &g_bpu->core_list);
	core->host = g_bpu;
	mutex_unlock(&g_bpu->mutex_lock);

	return 0;
}
EXPORT_SYMBOL(bpu_core_register);

void bpu_core_unregister(struct bpu_core *core)
{
	if (!g_bpu) {
		pr_err("bpu not inited!\n");
		return;
	}

	if (!core) {
		pr_err("Unregister Invalid core!\n");
		return;
	}

	/* del to bpu core list */
	mutex_lock(&g_bpu->mutex_lock);
	core->host = NULL;
	list_del((struct list_head *)core);
	mutex_unlock(&g_bpu->mutex_lock);
}
EXPORT_SYMBOL(bpu_core_unregister);

static int __init bpu_init(void)
{
	struct bpu *bpu;
	int ret;

	bpu = kzalloc(sizeof(struct bpu_user), GFP_KERNEL);
	if (!bpu) {
		pr_err("Can't alloc bpu mem\n");
		return -ENOMEM;
	}

	mutex_init(&bpu->mutex_lock);
	spin_lock_init(&bpu->spin_lock);
	INIT_LIST_HEAD(&bpu->core_list);
	INIT_LIST_HEAD(&bpu->user_list);
	INIT_LIST_HEAD(&bpu->group_list);
	bpu->busy_thres = 100;

	bpu->miscdev.minor = MISC_DYNAMIC_MINOR;
	bpu->miscdev.name = "bpu";
	bpu->miscdev.fops = &bpu_fops;

	ret = misc_register(&bpu->miscdev);
	if (ret) {
		pr_err("Register bpu device failed\n");
		goto err1;
	}

	atomic_set(&bpu->open_counter, 0);

	ret = bpu_sys_system_init(bpu);
	if (ret) {
		pr_err("Register bpu sub system failed\n");
		goto err2;
	}

	g_bpu = bpu;

	return 0;

err2:
	misc_deregister(&bpu->miscdev);
err1:
	kfree(bpu);

	return ret;
}

static void __exit bpu_exit(void)
{
	struct bpu *bpu = g_bpu;

	if (!bpu)
		return;

	misc_deregister(&bpu->miscdev);

	kfree(bpu);

	g_bpu = NULL;
}

module_init(bpu_init);
module_exit(bpu_exit);

MODULE_DESCRIPTION("Driver for Horizon BPU");
MODULE_AUTHOR("Zhang Guoying <guoying.zhang@horizon.ai>");
MODULE_LICENSE("GPL v2");
