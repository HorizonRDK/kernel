/*
 * Copyright (C) 2019 Horizon Robotics
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */
#include <linux/device.h>
#include <linux/slab.h>
#include "bpu.h"
#include "bpu_core.h"
#include "bpu_ctrl.h"

static ssize_t bpu_core_ratio_show(struct device *dev, struct device_attribute *attr, char *buf)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/

	return sprintf(buf, "%d\n", bpu_core_ratio(core));
}

static ssize_t bpu_core_queue_show(struct device *dev, struct device_attribute *attr, char *buf)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/

	return sprintf(buf, "%d\n", kfifo_avail(&core->run_fc_fifo[0]));/*PRQA S ALL*/
}

static ssize_t bpu_core_fc_time_show(struct device *dev, struct device_attribute *attr, char *buf)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/
	struct bpu_fc *tmp_fcs;
	int32_t i, len;
	int32_t ret;

	ret = sprintf(buf, "%s\t%s\t\t%s\t%s\t%s\t%s\t%s\n",
					"index",
					"id:hwid",
					"group",
					"prio",
					"s_time",
					"e_time",
					"r_time");

	len = kfifo_len(&core->done_fc_fifo);/*PRQA S ALL*/
	if (len <= 0) {
		return ret;
	}

	tmp_fcs = (struct bpu_fc *)kzalloc((sizeof(struct bpu_fc) /*PRQA S ALL*/
			* BPU_CORE_RECORE_NUM), GFP_KERNEL);/*PRQA S ALL*/

	len = kfifo_out_peek(&core->done_fc_fifo, tmp_fcs, len);/*PRQA S ALL*/
	for(i = 0; i < len; i++) {
		ret += sprintf(buf + ret,/*PRQA S ALL*/
				"%d\t%d:%d\t%d:%d\t%d\t%ld\t%ld\t%ldus\n",
				tmp_fcs[i].index, tmp_fcs[i].info.id, tmp_fcs[i].hw_id,
				bpu_group_id(tmp_fcs[i].info.g_id), bpu_group_user(tmp_fcs[i].info.g_id),
				tmp_fcs[i].info.priority,
				(tmp_fcs[i].start_point.tv_sec * SECTOMS)
				+ (tmp_fcs[i].start_point.tv_usec / SECTOMS),
				(tmp_fcs[i].end_point.tv_sec * SECTOMS)
				+ (tmp_fcs[i].end_point.tv_usec / SECTOMS),
				((tmp_fcs[i].end_point.tv_sec * SECTOUS)
				+ tmp_fcs[i].end_point.tv_usec)
				- ((tmp_fcs[i].start_point.tv_sec * SECTOUS)
				+ tmp_fcs[i].start_point.tv_usec));
	}

	kfree(tmp_fcs);

	return ret;
}

static ssize_t bpu_core_users_show(struct device *dev, struct device_attribute *attr, char *buf)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/
	struct bpu_user *tmp_user;
	struct list_head *pos, *pos_n;
	int32_t ret = 0;

	if (core == NULL) {
		return sprintf(buf, "core not inited!\n");
	}

	ret += sprintf(buf, "*User via BPU Core(%d)*\n", core->index);/*PRQA S ALL*/
	ret += sprintf(buf + ret, "%s\t\t%s\n", "user", "ratio");/*PRQA S ALL*/
	list_for_each_safe(pos, pos_n, &core->user_list) {/*PRQA S ALL*/
		tmp_user = (struct bpu_user *)list_entry(pos, struct bpu_user, node);/*PRQA S ALL*/
		if (tmp_user != NULL) {
			ret += sprintf(buf + ret, "%d\t\t%d\n",/*PRQA S ALL*/
				tmp_user->id, bpu_user_ratio(tmp_user));
		}
	}

	return ret;
}

static ssize_t bpu_core_hotplug_show(struct device *dev, struct device_attribute *attr, char *buf)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/

	return sprintf(buf, "%d\n", core->hotplug);
}

static ssize_t bpu_core_hotplug_store(struct device *dev, struct device_attribute *attr,/*PRQA S ALL*/
				  const char *buf, size_t len)
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/
	int32_t hotplug_en;
	int32_t ret;

	ret = sscanf(buf, "%du", &hotplug_en);
	if (ret < 0) {
		return 0;
	}

	if (hotplug_en <= 0) {
		core->hotplug = 0;
	} else {
		core->hotplug = 1;
	}

	return (ssize_t)len;
}

static ssize_t bpu_core_power_en_show(struct device *dev, struct device_attribute *attr, char *buf)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/

	return sprintf(buf, "%d\n", core->hw_enabled);
}

static ssize_t bpu_core_power_en_store(struct device *dev, struct device_attribute *attr,/*PRQA S ALL*/
				  const char *buf, size_t len)
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/
	int32_t power_en;
	int32_t ret;

	ret = sscanf(buf, "%du", &power_en);
	if (ret < 0) {
		return 0;
	}

	mutex_lock(&core->mutex_lock);
	if (power_en <= 0) {
		ret = bpu_core_disable(core);
	} else {
		ret = bpu_core_enable(core);
	}
	mutex_unlock(&core->mutex_lock);
	if (ret < 0) {
		return 0;
	}

	return (ssize_t)len;
}

static ssize_t bpu_core_power_show(struct device *dev, struct device_attribute *attr, char *buf)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/

	return sprintf(buf, "%d\n", core->power_level);
}

static ssize_t bpu_core_power_store(struct device *dev, struct device_attribute *attr,/*PRQA S ALL*/
				  const char *buf, size_t len)
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/
	int32_t power_level;
	int32_t ret;

	ret = sscanf(buf, "%du", &power_level);
	if (ret < 0) {
		return 0;
	}

	mutex_lock(&core->mutex_lock);
	ret = bpu_core_set_freq_level(core, power_level);
	mutex_unlock(&core->mutex_lock);
	if (ret < 0) {
		return 0;
	}

	return (ssize_t)len;
}

static ssize_t bpu_core_limit_show(struct device *dev, struct device_attribute *attr, char *buf)/*PRQA S ALL*/
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/

	return sprintf(buf, "%d\n", core->fc_buf_limit);
}

static ssize_t bpu_core_limit_store(struct device *dev, struct device_attribute *attr,/*PRQA S ALL*/
				  const char *buf, size_t len)
{
	struct bpu_core *core = (struct bpu_core *)dev_get_drvdata(dev);/*PRQA S ALL*/
	int32_t fc_buf_limit;
	int32_t ret;

	ret = sscanf(buf, "%du", &fc_buf_limit);
	if (ret < 0) {
		return 0;
	}

	ret = bpu_core_set_limit(core, fc_buf_limit);
	if (ret != 0) {
		pr_err("Bpu core Set prio limit failed\n");/*PRQA S ALL*/
	}

	return (ssize_t)len;
}
// PRQA S ALL ++
static DEVICE_ATTR(ratio, S_IRUGO, bpu_core_ratio_show, NULL);
static DEVICE_ATTR(queue, S_IRUGO, bpu_core_queue_show, NULL);
static DEVICE_ATTR(fc_time, S_IRUGO, bpu_core_fc_time_show, NULL);
static DEVICE_ATTR(users, S_IRUGO, bpu_core_users_show, NULL);

/*
 * core power level >0: kernel dvfs;
 * 0:highest; -1,-2...: different work power level */
static DEVICE_ATTR(power_level, S_IRUGO | S_IWUSR,
		bpu_core_power_show, bpu_core_power_store);

/* power on/off, > 0 power on; <= 0 power off*/
static DEVICE_ATTR(power_enable, S_IRUGO | S_IWUSR,
		bpu_core_power_en_show, bpu_core_power_en_store);

/* power on/off, > 0 power on; <= 0 power off*/
static DEVICE_ATTR(hotplug, S_IRUGO | S_IWUSR,
		bpu_core_hotplug_show, bpu_core_hotplug_store);

static DEVICE_ATTR(limit, S_IRUGO | S_IWUSR,
		bpu_core_limit_show, bpu_core_limit_store);
// PRQA S ALL --

static struct attribute *bpu_core_attrs[] = {
	&dev_attr_ratio.attr,
	&dev_attr_queue.attr,
	&dev_attr_fc_time.attr,
	&dev_attr_power_level.attr,
	&dev_attr_power_enable.attr,
	&dev_attr_hotplug.attr,
	&dev_attr_users.attr,
	&dev_attr_limit.attr,
	NULL,
};

static struct attribute_group bpu_core_attr_group = {
	.attrs = bpu_core_attrs,
};

static ssize_t bpu_ratio_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)/*PRQA S ALL*/
{
	if (g_bpu == NULL) {
		return sprintf(buf, "bpu not inited!\n");
	}

	return sprintf(buf, "%d\n", bpu_ratio(g_bpu));
}

static ssize_t bpu_core_num_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)/*PRQA S ALL*/
{
	struct list_head *pos, *pos_n;
	int32_t core_num = 0;

	if (g_bpu == NULL) {
		return sprintf(buf, "bpu not inited!\n");
	}

	list_for_each_safe(pos, pos_n, &g_bpu->core_list) {/*PRQA S ALL*/
		core_num++;
	}

	return sprintf(buf, "%d\n", core_num);
}

static ssize_t bpu_group_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)/*PRQA S ALL*/
{
	struct bpu_fc_group *tmp_group;
	struct list_head *pos, *pos_n;
	int32_t ret = 0;

	if (g_bpu == NULL) {
		return sprintf(buf, "bpu not inited!\n");
	}
	ret += sprintf(buf, "%s\t\t%s\t%s\n", "group", "prop", "ratio");/*PRQA S ALL*/
	list_for_each_safe(pos, pos_n, &g_bpu->group_list) {/*PRQA S ALL*/
		tmp_group = (struct bpu_fc_group *)list_entry(pos, struct bpu_fc_group, node);/*PRQA S ALL*/
		if (tmp_group != NULL) {
			ret += sprintf(buf + ret, "%d(%d)\t\t%d\t%d\n",/*PRQA S ALL*/
				bpu_group_id(tmp_group->id),
				bpu_group_user(tmp_group->id),
				tmp_group->proportion,
				bpu_fc_group_ratio(tmp_group));
		}
	}

	return ret;
}

static ssize_t bpu_users_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)/*PRQA S ALL*/
{
	struct bpu_user *tmp_user;
	struct bpu_core *tmp_core;
	struct list_head *pos, *pos_n;
	struct list_head *bpos, *bpos_n;
	int32_t ret = 0;

	if (g_bpu == NULL) {
		return sprintf(buf, "bpu not inited!\n");
	}

	ret += sprintf(buf, "*User via BPU Bus*\n");/*PRQA S ALL*/
	ret += sprintf(buf + ret, "%s\t\t%s\n", "user", "ratio");/*PRQA S ALL*/
	list_for_each_safe(pos, pos_n, &g_bpu->user_list) {/*PRQA S ALL*/
		tmp_user = (struct bpu_user *)list_entry(pos, struct bpu_user, node);/*PRQA S ALL*/
		if (tmp_user != NULL) {
			ret += sprintf(buf + ret, "%d\t\t%d\n",/*PRQA S ALL*/
				tmp_user->id, bpu_user_ratio(tmp_user));
		}
	}

	list_for_each_safe(pos, pos_n, &g_bpu->core_list) {/*PRQA S ALL*/
		tmp_core = (struct bpu_core *)list_entry(pos, struct bpu_core, node);/*PRQA S ALL*/
		if (tmp_core != NULL) {
			ret += sprintf(buf + ret,/*PRQA S ALL*/
					"*User via BPU Core(%d)*\n",
					tmp_core->index);
			list_for_each_safe(bpos, bpos_n, &tmp_core->user_list) {/*PRQA S ALL*/
				tmp_user = (struct bpu_user *)list_entry(bpos, struct bpu_user, node);/*PRQA S ALL*/
				if (tmp_user != NULL) {
					ret += sprintf(buf + ret, "%d\t\t%d\n",/*PRQA S ALL*/
							tmp_user->id,
							bpu_user_ratio(tmp_user));
				}
			}
		}
	}

	return ret;
}
// PRQA S ALL ++
static struct kobj_attribute ratio_info = __ATTR(ratio, S_IRUGO, bpu_ratio_show, NULL);
static struct kobj_attribute core_num_info = __ATTR(core_num, S_IRUGO, bpu_core_num_show, NULL);
static struct kobj_attribute group_info = __ATTR(group, S_IRUGO, bpu_group_show, NULL);
static struct kobj_attribute users_info = __ATTR(users, S_IRUGO, bpu_users_show, NULL);
// PRQA S ALL ++--

static struct attribute *bpu_attrs[] = {
	&ratio_info.attr,
	&core_num_info.attr,
	&group_info.attr,
	&users_info.attr,
	NULL,
};

static struct attribute_group bpu_attr_group = {
	.attrs = bpu_attrs,
};

static const struct attribute_group *bpu_attr_groups[] = {
	&bpu_attr_group,
	NULL,
};

struct bus_type bpu_subsys = {
	.name = "bpu",
};

int32_t bpu_core_create_sys(struct bpu_core *core)
{
	char core_name[10];
	int32_t ret;

	if (core == NULL) {
		return -ENODEV;
	}

	if (g_bpu->bus == NULL) {
		dev_err(core->dev, "BPU not register bus for sys\n");
		return -ENODEV;
	}

	ret = sprintf(core_name, "bpu%d", core->index);
	if (ret < 0) {
		dev_err(core->dev, "Create debug name failed\n");
		return ret;
	}

	ret = device_add_group(core->dev, &bpu_core_attr_group);
	if (ret < 0) {
		dev_err(core->dev, "Create bpu core debug group failed\n");
		return ret;
	}

	if (core->hw_ops->debug != NULL) {
		ret = core->hw_ops->debug(core, 1);
		if (ret < 0) {
			device_remove_group(core->dev, &bpu_core_attr_group);
			dev_err(core->dev, "Create bpu core hw debug failed\n");
			return ret;
		}
	}

	ret = sysfs_create_link(&bpu_subsys.dev_root->kobj,
			&core->dev->kobj, core_name);
	if (ret != 0) {
		if (core->hw_ops->debug != NULL) {
			ret = core->hw_ops->debug(core, 0);
		}

		device_remove_group(core->dev, &bpu_core_attr_group);
		dev_err(core->dev, "Create link to bpu bus failed\n");
		return ret;
	}

	return 0;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_create_sys);
// PRQA S ALL --

void bpu_core_discard_sys(const struct bpu_core *core)
{
	char core_name[10];
	int32_t ret;

	if (core == NULL) {
		return;
	}

	ret = sprintf(core_name, "bpu%d", core->index);
	if (ret > 0) {
		sysfs_remove_link(&bpu_subsys.dev_root->kobj, core_name);
		device_remove_group(core->dev, &bpu_core_attr_group);
		if (core->hw_ops->debug != NULL) {
			core->hw_ops->debug(core, 0);
		}
	}

	return;
}
// PRQA S ALL ++
EXPORT_SYMBOL(bpu_core_discard_sys);
// PRQA S ALL --

/* create bus sub system in /sys/devices/system/ */
int32_t bpu_sys_system_init(struct bpu *bpu)
{
	int32_t ret;

	if (bpu == NULL) {
		pr_err("no device for bpu subsystem\n");/*PRQA S ALL*/
		return -ENODEV;
	}

	ret = subsys_system_register(&bpu_subsys, bpu_attr_groups);
	if (ret != 0) {
		pr_err("failed to register bpu subsystem\n");/*PRQA S ALL*/
	}

	bpu->bus = &bpu_subsys;

	return ret;
}

void bpu_sys_system_exit(struct bpu *bpu)
{
	if (bpu == NULL) {
		return;
	}

	if (bpu->bus != NULL) {
		bus_unregister(bpu->bus);
	}
}
