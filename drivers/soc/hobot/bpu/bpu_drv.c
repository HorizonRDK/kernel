/*
 * Copyright (C) 2019 Horizon Robotics
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

/* mainly clear fc data in bpu_fc*/
void bpu_fc_clear(struct bpu_fc *fc)
{
	if (fc != NULL) {
		if (fc->fc_data != NULL) {
			vfree((void *)fc->fc_data);/*PRQA S ALL*/
			fc->fc_data = NULL;
			fc->info.length = 0;
		}
	}
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_fc_clear);
// PRQA S ALL --

static int32_t bpu_fc_bind_user(struct bpu_fc *fc, struct bpu_user *user)
{
	int32_t ret;

	if ((fc == NULL) || (user == NULL)) {
		pr_err("%s[%d]:bpu bind invalid fc or user\n", __func__, __LINE__);/*PRQA S ALL*/
		ret = -EINVAL;
	} else {
		/*
		 * use the pointer's point to user for judging
		 * whether valid of user after file released
		 */
		fc->user = user->p_file_private;
		ret = 0;
	}

	return ret;
}

static struct bpu_fc_group *bpu_find_group(uint32_t group_id)
{
	struct bpu_fc_group *tmp_group;
	struct bpu_fc_group *group = NULL;
	struct list_head *pos, *pos_n;

	list_for_each_safe(pos, pos_n, &g_bpu->group_list) {/*PRQA S ALL*/
		tmp_group = (struct bpu_fc_group *)pos;/*PRQA S ALL*/
		if (tmp_group != NULL) {
			if (tmp_group->id == group_id) {
				group = tmp_group;
				break;
			}
		}
	}

	return group;
}

static struct bpu_fc_group *bpu_create_group(uint32_t group_id)
{
	struct bpu_fc_group *tmp_group;

	tmp_group = bpu_find_group(group_id);
	if (tmp_group == NULL) {
		tmp_group = (struct bpu_fc_group *)kzalloc(sizeof(struct bpu_fc_group), GFP_KERNEL);/*PRQA S ALL*/
		if (tmp_group == NULL) {
			pr_err("BPU create group[%d] failed\n", group_id);/*PRQA S ALL*/
			return tmp_group;
		}
		tmp_group->id = group_id;

		if(kfifo_alloc(&tmp_group->buffer_fc_fifo, 50, GFP_KERNEL) != 0) {/*PRQA S ALL*/
			kfree((void *)tmp_group);/*PRQA S ALL*/
			pr_err("BPU create group[%d] fc buffer failed\n", group_id);/*PRQA S ALL*/
			return NULL;
		}

		spin_lock_init(&tmp_group->spin_lock);/*PRQA S ALL*/
		list_add((struct list_head *)tmp_group, &g_bpu->group_list);/*PRQA S ALL*/
	} else {
		pr_info("BPU already have group[%d]\n", group_id);/*PRQA S ALL*/
	}

	return tmp_group;
}

static void bpu_delete_group(uint32_t group_id)
{
	struct bpu_fc_group *tmp_group;

	tmp_group = bpu_find_group(group_id);
	if (tmp_group != NULL) {
		kfifo_free(&tmp_group->buffer_fc_fifo);/*PRQA S ALL*/
		list_del((struct list_head *)tmp_group);/*PRQA S ALL*/
		kfree((void *)tmp_group);/*PRQA S ALL*/
	}
}

static int32_t bpu_fc_bind_group(struct bpu_fc *fc, uint32_t group_id)
{
	struct bpu_fc_group *group;

	if (g_bpu == NULL) {
		pr_err("%s[%d]:bpu not inited\n", __func__, __LINE__);/*PRQA S ALL*/
		return -ENODEV;
	}

	if (fc == NULL) {
		pr_err("%s[%d]:bpu bind invalid fc\n", __func__, __LINE__);/*PRQA S ALL*/
		return -EINVAL;
	}

	group = bpu_find_group(group_id);
	if (group != NULL) {
		fc->g_id = &group->id;
	} else {
		pr_err("bpu fc find no correspond group\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	return 0;
}

/* create bpu_fc from user fc info*/
static int32_t bpu_fc_create_from_user(struct bpu_fc *fc,
		const struct user_bpu_fc *user_fc, const void *data)
{
	int32_t ret;

	if (user_fc == NULL) {
		return -EINVAL;
	}

	if (fc == NULL) {
		pr_err("bpu user fc need have space to place\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	fc->info = *user_fc;
	fc->fc_data = NULL;
	ret = bpu_fc_bind_group(fc, user_fc->g_id);
	if (ret != 0) {
		pr_err("bpu fc bind group failed\n");/*PRQA S ALL*/
		return ret;
	}

	if ((data != NULL) && (user_fc->length > 0u)) {
		fc->fc_data = vmalloc(user_fc->length);/*PRQA S ALL*/
		if (fc->fc_data == NULL) {
			pr_err("create bpu fc mem failed\n");/*PRQA S ALL*/
			return -ENOMEM;
		}

		if (copy_from_user(fc->fc_data, (void __user *)data,/*PRQA S ALL*/
					user_fc->length) != 0u) {
			vfree((void *)fc->fc_data);/*PRQA S ALL*/
			fc->fc_data = NULL;
			pr_err("%s: copy fc data failed from userspace\n", __func__);/*PRQA S ALL*/
			return -EFAULT;
		}
	}

	return (int32_t)user_fc->length;
}

static int32_t bpu_write_prepare(struct bpu_user *user,
		const struct user_bpu_fc *header,
		const char __user *buf, size_t len,
		struct bpu_fc *bpu_fc)
{
	int32_t tmp_raw_fc_len;
	int32_t ret;

	if ((user == NULL) || (header == NULL) ||(buf == NULL) || (len == 0u)) {
		pr_err("Write bpu buffer error\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	tmp_raw_fc_len = bpu_fc_create_from_user(bpu_fc,
		header, buf);/*PRQA S ALL*/
	if (tmp_raw_fc_len <= 0) {
		pr_err("bpu fc from user error\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	do_gettimeofday(&bpu_fc->start_point);

	ret = bpu_fc_bind_user(bpu_fc, user);
	if (ret < 0) {
		bpu_fc_clear(bpu_fc);
		pr_err("bpu buffer bind user error\n");/*PRQA S ALL*/
		return ret;
	}

	return tmp_raw_fc_len + (int32_t)sizeof(struct user_bpu_fc);
}

/*
 * write the user fc buffer to real core and bind user
 */
int32_t bpu_write_fc_to_core(struct bpu_core *core,
		struct bpu_fc *bpu_fc, uint32_t offpos)
{
	struct bpu_user *tmp_user;
	unsigned long flags;/*PRQA S ALL*/
	int32_t ret;
	int32_t write_fc_num;
	uint32_t prio;

	if ((core == NULL) || (bpu_fc == NULL)) {
		pr_err("Write bpu buffer error\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	tmp_user = bpu_get_user(bpu_fc);
	if (tmp_user == NULL) {
		/* no user, so report fake complete */
		ret = (int32_t)bpu_fc->info.slice_num - (int32_t)offpos;
		bpu_fc_clear(bpu_fc);
		return ret;
	}

	if (tmp_user->is_alive == 0u) {
		ret = (int32_t)bpu_fc->info.slice_num - (int32_t)offpos;
		bpu_fc_clear(bpu_fc);
		dev_err(core->dev, "bpu user now is not alive\n");
		return ret;
	}

	if (bpu_core_is_pending(core) != 0) {
		dev_dbg(core->dev, "bpu core now pending\n");/*PRQA S ALL*/
		return -EBUSY;
	}

	/* write data to bpu fc range */
	if (core->hw_ops->write_fc != NULL) {
		prio = FC_PRIO(bpu_fc->info.priority);
		if (bpu_fc->info.id != 0u) {
			bpu_fc->hw_id = atomic_read(&core->hw_id_counter[prio]);/*PRQA S ALL*/
			/* HW_ID_MAX for sched trig */
			if (bpu_fc->hw_id >= FC_ID(HW_ID_MAX - 1)) {
				atomic_set(&core->hw_id_counter[prio], 1);/*PRQA S ALL*/
			} else {
				atomic_inc(&core->hw_id_counter[prio]);/*PRQA S ALL*/
			}
		} else {
			bpu_fc->hw_id = 0;
		}
		/* use the hw_id to tell soc id and priority */
		bpu_fc->hw_id = FC_PRIO_ID(prio, bpu_fc->hw_id);
		spin_lock_irqsave(&core->spin_lock, flags);/*PRQA S ALL*/
		write_fc_num = core->hw_ops->write_fc(core, bpu_fc, offpos);
		if (write_fc_num != ((int32_t)bpu_fc->info.slice_num - (int32_t)offpos)) {
			/* write raw fc to hw fifo not complete or error */
			spin_unlock_irqrestore(&core->spin_lock, flags);
			return write_fc_num;
		}
		if (bpu_fc->info.id != 0u) {
			tmp_user->running_task_num++;
			core->running_task_num++;
			ret = kfifo_in(&core->run_fc_fifo[prio],/*PRQA S ALL*/
					bpu_fc, 1);
			if (ret < 1) {
				core->running_task_num--;
				tmp_user->running_task_num--;
				spin_unlock_irqrestore(&core->spin_lock, flags);
				dev_err(core->dev, "bpu request to fifo failed\n");
				return -EBUSY;
			}
		} else {
			bpu_fc_clear(bpu_fc);
		}
		spin_unlock_irqrestore(&core->spin_lock, flags);
	} else {
		bpu_fc_clear(bpu_fc);
		dev_err(core->dev, "no real bpu to process\n");
		return -ENODEV;
	}

	return write_fc_num;
}

bool bpu_core_is_online(struct bpu_core *core)
{
	if (core == NULL) {
		return false;
	}

	if (core->hw_enabled == 0) {
		return false;
	}

	return bpu_prio_is_plug_in(core->prio_sched);
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_is_online);
// PRQA S ALL --

static struct bpu_core *bpu_opt_core(const struct bpu *bpu, uint64_t core_mask)
{
	struct bpu_core *tmp_core, *tmp_opt_core = NULL;
	struct list_head *pos, *pos_n;
	int32_t tmp_val, tmp_last_val = -1;

	if (bpu == NULL) {
		return NULL;
	}

	list_for_each_safe(pos, pos_n, &bpu->core_list) {/*PRQA S ALL*/
		tmp_core = (struct bpu_core*)pos;/*PRQA S ALL*/
		if (bpu_core_is_online(tmp_core)) {
			if ((core_mask & ((uint64_t)0x1u << (uint32_t)tmp_core->index)) != 0u) {
				mutex_lock(&tmp_core->mutex_lock);
				if ((atomic_read(&tmp_core->open_counter) != 0) &&/*PRQA S ALL*/
						(tmp_core->hw_enabled != 0u)) {
					tmp_val = kfifo_avail(&tmp_core->run_fc_fifo[0]);/*PRQA S ALL*/
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

int32_t  bpu_write_with_user(const struct bpu_core *core,
			struct bpu_user *user,
			const char __user buf[], size_t len)
{
	uint64_t tmp_core_mask;
	uint64_t tmp_run_c_mask;
	struct bpu_fc tmp_bpu_fc;
	struct bpu_core *tmp_core;
	struct user_bpu_fc header;
	uint32_t use_len = 0;
	int32_t prepare_len;
	int32_t ret;

	if ((user == NULL) || (buf == NULL) || (len == 0u)) {
		pr_err("Write bpu buffer error\n");/*PRQA S ALL*/
		return -EINVAL;
	}


	if (len <= sizeof(struct user_bpu_fc)) {
		pr_err("Write invalied data\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	while ((len - use_len) > (uint32_t)sizeof(struct user_bpu_fc)) {
		if (copy_from_user(&header, &buf[use_len], /*PRQA S ALL*/
					sizeof(struct user_bpu_fc)) != 0) { /*PRQA S ALL*/
			pr_err("%s: copy data failed from userspace when write\n", __func__);/*PRQA S ALL*/
			return -EFAULT;
		}
		tmp_core_mask = header.core_mask;/*PRQA S ALL*/
		tmp_run_c_mask = header.run_c_mask;/*PRQA S ALL*/

		prepare_len = bpu_write_prepare(user, &header,
				&buf[use_len + sizeof(struct user_bpu_fc)],
				len - use_len, &tmp_bpu_fc);
		if (prepare_len <= 0) {
			pr_err("BPU prepare user write data!");/*PRQA S ALL*/
			return -EINVAL;
		}

		if (!bpu_core_is_online((struct bpu_core *)core)) {
			/*
			* choose optimal core according to core stauts
			* FIXME: need find core according core_mask flag
			*/
			mutex_lock(&g_bpu->mutex_lock);
			if (tmp_run_c_mask != 0u) {
				tmp_core_mask &= tmp_run_c_mask;
			}

			if (core != NULL) {
				/* if core support hotplug choose other cores */
				if ((tmp_core_mask == ((uint64_t)0x1 << core->index))
						&& (core->hotplug != 0)) {
					tmp_core_mask = ALL_CORE_MASK;
				}
			}
			tmp_core = bpu_opt_core(g_bpu, tmp_core_mask);
			if (tmp_core == NULL) {
				mutex_unlock(&g_bpu->mutex_lock);
				pr_err("BPU has no suitable core, 0x%llx!", tmp_run_c_mask);/*PRQA S ALL*/
				return -ENODEV;
			}
			mutex_unlock(&g_bpu->mutex_lock);
			ret = bpu_prio_in(tmp_core->prio_sched, &tmp_bpu_fc);
		} else {
			tmp_bpu_fc.info.core_mask = ((uint64_t)0x1u << (uint32_t)core->index);
			ret = bpu_prio_in(core->prio_sched, &tmp_bpu_fc);
		}

		if (ret < 0) {
			pr_err("write bpu fc to core failed\n");/*PRQA S ALL*/
			return ret;
		}

		use_len += (uint32_t)prepare_len;
	}

	return (int32_t)use_len;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_write_with_user);
// PRQA S ALL --

/*
 * user read fc done fifo to userspace 
 */
int32_t bpu_read_with_user(struct bpu_core *core, /*PRQA S ALL*/
			struct bpu_user *user,
			const char __user *buf, size_t len)
{
	int32_t ret, copied;

	if ((user == NULL) || (buf == NULL) || (len == 0u)) {
		return 0;
	}

	if (kfifo_initialized(&user->done_fcs) == 0) {/*PRQA S ALL*/
		return -EINVAL;
	}

	mutex_lock(&user->mutex_lock);

	ret = kfifo_to_user(&user->done_fcs, (void __user *)buf, len, &copied);/*PRQA S ALL*/
	if (ret < 0) {
		mutex_unlock(&user->mutex_lock);
		return ret;
	}
	mutex_unlock(&user->mutex_lock);

	return copied;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_read_with_user);
// PRQA S ALL --

static long bpu_ioctl(struct file *filp,/*PRQA S ALL*/
		unsigned int cmd, unsigned long arg)/*PRQA S ALL*/
{
	struct list_head *pos, *pos_n;
	struct bpu_fc_group *group;
	struct bpu_core *tmp_core;

	struct bpu_group tmp_group;
	uint32_t ratio;
	uint64_t core;
	uint16_t cap;
	uint32_t limit;
	int32_t ret = 0;

	switch (cmd) {
	case BPU_SET_GROUP:/*PRQA S ALL*/
		if (copy_from_user(&tmp_group, (void __user *)arg, _IOC_SIZE(cmd))) {/*PRQA S ALL*/
			pr_err("%s: copy data failed from userspace\n", __func__);/*PRQA S ALL*/
			return -EFAULT;
		}

		group = bpu_find_group(tmp_group.group_id);
		if (group != NULL) {
			if (tmp_group.prop <= 0u) {
				bpu_delete_group(tmp_group.group_id);
				break;
			}
			group->proportion = (int32_t)tmp_group.prop;
		} else {
			if (tmp_group.prop <= 0u) {
				break;
			}
			group = bpu_create_group(tmp_group.group_id);
			if (group != NULL) {
				group->proportion = (int32_t)tmp_group.prop;
			} else {
				pr_err("BPU create group(%d)(%d) prop(%d) error\n",/*PRQA S ALL*/
						bpu_group_id(tmp_group.group_id),
						bpu_group_user(tmp_group.group_id),
						tmp_group.prop);
				return -EINVAL;
			}
		}
		pr_debug("BPU set group(%d)(%d) prop [%d]\n",/*PRQA S ALL*/
				bpu_group_id(tmp_group.group_id),
				bpu_group_user(tmp_group.group_id),
				tmp_group.prop);
		break;
	case BPU_GET_GROUP:/*PRQA S ALL*/
		if (copy_from_user(&tmp_group, (void __user *)arg, _IOC_SIZE(cmd))) {/*PRQA S ALL*/
			pr_err("%s: copy data failed from userspace\n", __func__);/*PRQA S ALL*/
			return -EFAULT;
		}

		group = bpu_find_group(tmp_group.group_id);
		if (group != NULL) {
			tmp_group.prop = (uint32_t)group->proportion;
			tmp_group.ratio = bpu_fc_group_ratio(group);
		}

		if (copy_to_user((void __user *)arg, &tmp_group, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			pr_err("copy data to userspace failed\n");/*PRQA S ALL*/
			return -EFAULT;
		}
		break;
	case BPU_GET_RATIO:/*PRQA S ALL*/
		ratio = bpu_ratio(g_bpu);
		if (copy_to_user((void __user *)arg, &ratio, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			pr_err("copy data to userspace failed\n");/*PRQA S ALL*/
			return -EFAULT;
		}
		break;
	case BPU_GET_CAP:/*PRQA S ALL*/
		cap = 0;
		mutex_lock(&g_bpu->mutex_lock);
		list_for_each_safe(pos, pos_n, &g_bpu->core_list) {/*PRQA S ALL*/
			tmp_core = (struct bpu_core*)pos;/*PRQA S ALL*/
			if (tmp_core != NULL) {
				cap = max((uint16_t)kfifo_avail(&tmp_core->run_fc_fifo[0]), cap);/*PRQA S ALL*/
			}
		}
		mutex_unlock(&g_bpu->mutex_lock);
		if (copy_to_user((void __user *)arg, &cap, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			pr_err("copy data to userspace failed\n");/*PRQA S ALL*/
			return -EFAULT;
		}
		break;
	case BPU_OPT_CORE:/*PRQA S ALL*/
		if (copy_from_user(&core, (void __user *)arg, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			pr_err("%s: copy data failed from userspace\n", __func__);/*PRQA S ALL*/
			return -EFAULT;
		}
		mutex_lock(&g_bpu->mutex_lock);
		tmp_core = bpu_opt_core(g_bpu, core);
		if (tmp_core == NULL) {
			mutex_unlock(&g_bpu->mutex_lock);
			pr_err("Can't find an optimal BPU Core\n");/*PRQA S ALL*/
			return -EFAULT;
		}
		core = (uint32_t)tmp_core->index;
		mutex_unlock(&g_bpu->mutex_lock);

		if (copy_to_user((void __user *)arg, &core, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			pr_err("copy data to userspace failed\n");/*PRQA S ALL*/
			return -EFAULT;
		}
		break;
	case BPU_RESET:/*PRQA S ALL*/
		list_for_each_safe(pos, pos_n, &g_bpu->core_list) {/*PRQA S ALL*/
			tmp_core = (struct bpu_core*)pos;/*PRQA S ALL*/
			if (tmp_core != NULL) {
				mutex_lock(&tmp_core->mutex_lock);
				ret = bpu_core_reset(tmp_core);
				mutex_unlock(&tmp_core->mutex_lock);
				if (ret != 0) {
					pr_err("BPU Core reset failed\n");/*PRQA S ALL*/
				}
			}
		}
		break;
	case BPU_SET_LIMIT:/*PRQA S ALL*/
		if (copy_from_user(&limit, (void __user *)arg, _IOC_SIZE(cmd)) != 0) {/*PRQA S ALL*/
			pr_err("%s: copy data failed from userspace\n", __func__);/*PRQA S ALL*/
			return -EFAULT;
		}
		list_for_each_safe(pos, pos_n, &g_bpu->core_list) {/*PRQA S ALL*/
			tmp_core = (struct bpu_core*)pos;/*PRQA S ALL*/
			if (tmp_core != NULL) {
				ret = bpu_core_set_limit(tmp_core, (int32_t)limit);
				if (ret != 0) {
					pr_err("BPU Core set prio limit failed\n");/*PRQA S ALL*/
				}
			}
		}
		break;
	default:
		pr_err("%s: BPU invalid ioctl argument\n", __func__);/*PRQA S ALL*/
		ret = -EINVAL;
		break;
	}

	return ret;
}

static unsigned int bpu_poll(struct file *filp, poll_table *wait)/*PRQA S ALL*/
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;/*PRQA S ALL*/
	uint32_t mask = 0;

	poll_wait(filp, &user->poll_wait, wait);
	mutex_lock(&user->mutex_lock);

	if (kfifo_len(&user->done_fcs) != 0) {/*PRQA S ALL*/
		mask |= POLLIN | POLLRDNORM;/*PRQA S ALL*/
	}

	mutex_unlock(&user->mutex_lock);

	return mask;
}

static ssize_t bpu_read(struct file *filp,/*PRQA S ALL*/
		char __user *buf, size_t len, loff_t *f_pos)/*PRQA S ALL*/
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;/*PRQA S ALL*/

	return bpu_read_with_user(NULL, user, buf, len);
}

static ssize_t bpu_write(struct file *filp,/*PRQA S ALL*/
		const char __user *buf, size_t len, loff_t *f_pos)/*PRQA S ALL*/
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;/*PRQA S ALL*/

	return bpu_write_with_user(NULL, user, buf, len);
}

static int bpu_open(struct inode *inode, struct file *filp)/*PRQA S ALL*/
{
	struct bpu *bpu = (struct bpu *)container_of(filp->private_data, struct bpu, miscdev);/*PRQA S ALL*/
	struct bpu_user *user;
	int32_t ret;

	if (atomic_read(&bpu->open_counter) == 0) {/*PRQA S ALL*/
		/* first open init something files */
		ret = bpu_sched_start(bpu);
		if (ret != 0) {
			pr_err("BPU sched start failed\n");/*PRQA S ALL*/
			return -EFAULT;
		}
	}

	user = (struct bpu_user *)kzalloc(sizeof(struct bpu_user), GFP_KERNEL);/*PRQA S ALL*/
	if (user == NULL) {
		pr_err("Can't alloc user mem");/*PRQA S ALL*/
		return -ENOMEM;
	}

	user->id = (uint32_t)task_pid_nr(current->group_leader);

	/* init fifo which report to userspace */
	ret = kfifo_alloc(&user->done_fcs, BPU_CORE_RECORE_NUM, GFP_KERNEL);/*PRQA S ALL*/
	if (ret != 0) {
		kfree((void *)user);/*PRQA S ALL*/
		pr_err("Can't alloc user fifo dev mem");/*PRQA S ALL*/
		return ret;
	}

	init_waitqueue_head(&user->poll_wait);/*PRQA S ALL*/
	user->host = (void *)bpu;/*PRQA S ALL*/
	user->p_file_private = &filp->private_data;
	list_add((struct list_head *)user, &g_bpu->user_list);/*PRQA S ALL*/
	spin_lock_init(&user->spin_lock);/*PRQA S ALL*/
	mutex_init(&user->mutex_lock);/*PRQA S ALL*/
	init_completion(&user->no_task_comp);/*PRQA S ALL*/
	user->is_alive = 1;
	user->running_task_num = 0;
	/* replace user the private to store user */
	filp->private_data = user;/*PRQA S ALL*/

	atomic_inc(&bpu->open_counter);/*PRQA S ALL*/

	return ret;
}

static int bpu_release(struct inode *inode, struct file *filp)/*PRQA S ALL*/
{
	struct bpu_user *user = (struct bpu_user *)filp->private_data;/*PRQA S ALL*/
	struct bpu *bpu = (struct bpu *)user->host;/*PRQA S ALL*/
	struct list_head *pos, *pos_n;
	struct bpu_fc_group *tmp_group;
	unsigned long flags;/*PRQA S ALL*/
	int32_t ret;

	user->is_alive = 0;

	atomic_dec(&bpu->open_counter);/*PRQA S ALL*/

	if (atomic_read(&bpu->open_counter) == 0) {/*PRQA S ALL*/
		/* release the real bpu*/
		ret = bpu_sched_stop(bpu);
		if (ret != 0) {
			pr_err("BPU sched stop failed\n");/*PRQA S ALL*/
			return -EFAULT;
		}

		spin_lock_irqsave(&bpu->spin_lock, flags);/*PRQA S ALL*/
		list_for_each_safe(pos, pos_n, &g_bpu->group_list) {/*PRQA S ALL*/
			tmp_group = (struct bpu_fc_group *)pos;/*PRQA S ALL*/
			if (tmp_group != NULL) {
				bpu_delete_group(tmp_group->id);
			}
		}
		spin_unlock_irqrestore(&bpu->spin_lock, flags);
	}

	spin_lock_irqsave(&bpu->spin_lock, flags);/*PRQA S ALL*/
	list_del((struct list_head *)user);/*PRQA S ALL*/
	kfifo_free(&user->done_fcs);/*PRQA S ALL*/
	kfree((void *)user);/*PRQA S ALL*/
	filp->private_data = NULL;
	spin_unlock_irqrestore(&bpu->spin_lock, flags);

	return 0;
}

static const struct file_operations bpu_fops = {/*PRQA S ALL*/
	.owner		= THIS_MODULE,
	.open		= bpu_open,
	.release	= bpu_release,
	.read		= bpu_read,
	.write		= bpu_write,
	.poll		= bpu_poll,
	.unlocked_ioctl = bpu_ioctl,
	.compat_ioctl = bpu_ioctl,
};

int32_t bpu_core_register(struct bpu_core *core)
{
	if (g_bpu == NULL) {
		pr_err("bpu not inited!\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	if (core == NULL) {
		pr_err("Register Invalid core!\n");/*PRQA S ALL*/
		return -EINVAL;
	}

	/* add to bpu core list */
	mutex_lock(&g_bpu->mutex_lock);
	list_add_tail((struct list_head *)core, &g_bpu->core_list);/*PRQA S ALL*/
	core->host = g_bpu;
	mutex_unlock(&g_bpu->mutex_lock);

	return 0;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_register);
// PRQA S ALL --

void bpu_core_unregister(struct bpu_core *core)
{
	if (g_bpu == NULL) {
		pr_err("bpu not inited!\n");/*PRQA S ALL*/
		return;
	}

	if (core == NULL) {
		pr_err("Unregister Invalid core!\n");/*PRQA S ALL*/
		return;
	}

	/* del to bpu core list */
	mutex_lock(&g_bpu->mutex_lock);
	core->host = NULL;
	list_del((struct list_head *)core);/*PRQA S ALL*/
	mutex_unlock(&g_bpu->mutex_lock);
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_unregister);
// PRQA S ALL --

static int __init bpu_init(void)/*PRQA S ALL*/
{
	struct bpu *bpu;
	int32_t ret;

	bpu = (struct bpu *)kzalloc(sizeof(struct bpu), GFP_KERNEL);/*PRQA S ALL*/
	if (bpu == NULL) {
		pr_err("Can't alloc bpu mem\n");/*PRQA S ALL*/
		return -ENOMEM;
	}

	mutex_init(&bpu->mutex_lock);/*PRQA S ALL*/
	spin_lock_init(&bpu->spin_lock);/*PRQA S ALL*/
	INIT_LIST_HEAD(&bpu->core_list);
	INIT_LIST_HEAD(&bpu->user_list);
	INIT_LIST_HEAD(&bpu->group_list);

	bpu->miscdev.minor = MISC_DYNAMIC_MINOR;
	bpu->miscdev.name = "bpu";
	bpu->miscdev.fops = &bpu_fops;

	ret = misc_register(&bpu->miscdev);
	if (ret != 0) {
		kfree((void *)bpu);/*PRQA S ALL*/
		pr_err("Register bpu device failed\n");/*PRQA S ALL*/
		return ret;
	}

	atomic_set(&bpu->open_counter, 0);/*PRQA S ALL*/

	ret = bpu_sys_system_init(bpu);
	if (ret != 0) {
		misc_deregister(&bpu->miscdev);
		kfree((void *)bpu);/*PRQA S ALL*/
		pr_err("Register bpu sub system failed\n");/*PRQA S ALL*/
		return ret;
	}

	g_bpu = bpu;

	return 0;
}

static void __exit bpu_exit(void)
{
	struct bpu *bpu = g_bpu;

	if (bpu != NULL) {
		bpu_sys_system_exit(bpu);
		misc_deregister(&bpu->miscdev);

		kfree((void *)bpu);/*PRQA S ALL*/

		g_bpu = NULL;
	}
}

// PRQA S ALL ++
module_init(bpu_init);
module_exit(bpu_exit);

MODULE_DESCRIPTION("Driver for Horizon BPU");
MODULE_AUTHOR("Zhang Guoying <guoying.zhang@horizon.ai>");
MODULE_LICENSE("GPL v2");
// PRQA S ALL --
