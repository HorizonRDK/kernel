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
#include <linux/device.h>
#include "bpu.h"
#include "bpu_core.h"
#include "bpu_ctrl.h"

extern struct bpu *g_bpu;

static ssize_t bpu_core_ratio_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct bpu_core *core = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", bpu_core_ratio(core));
}

static ssize_t bpu_core_queue_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct bpu_core *core = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", kfifo_avail(&core->run_fc_fifo[0]));
}

static ssize_t bpu_core_fc_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct bpu_core *core = dev_get_drvdata(dev);
	struct bpu_fc tmp_fcs[BPU_CORE_RECORE_NUM];
	int32_t i, len;
	int32_t ret;

	ret = sprintf(buf, "%-6s%-10s\t%-6s\t%-6s\t%-10s\t%-10s\t%s\n",
					"index",
					"id:hwid",
					"group",
					"prio",
					"start_time",
					"end_time",
					"exe_time");

	len = kfifo_len(&core->done_fc_fifo);
	if (len <= 0)
		return ret;

	len = kfifo_out_peek(&core->done_fc_fifo, tmp_fcs, len);
	for(i = 0; i < len; i++) {
		ret += sprintf(buf + ret,
				"%-6d%d:%-10d\t%d:%-6d%-6d\t%-10ld\t%-10ld\t%ldus\n",
				tmp_fcs[i].index, tmp_fcs[i].info.id, tmp_fcs[i].hw_id,
				tmp_fcs[i].info.g_id & 0xFFFF, tmp_fcs[i].info.g_id >> 16,
				tmp_fcs[i].info.priority,
				tmp_fcs[i].start_point.tv_sec * 1000
				+ tmp_fcs[i].start_point.tv_usec / 1000,
				tmp_fcs[i].end_point.tv_sec * 1000
				+ tmp_fcs[i].end_point.tv_usec / 1000,
				(tmp_fcs[i].end_point.tv_sec * 1000000
				+ tmp_fcs[i].end_point.tv_usec)
				- (tmp_fcs[i].start_point.tv_sec * 1000000
				+ tmp_fcs[i].start_point.tv_usec));
	}

	return ret;
}

static ssize_t bpu_core_users_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct bpu_core *core = dev_get_drvdata(dev);
	struct bpu_user *tmp_user;
	struct list_head *pos, *pos_n;
	int32_t ret = 0;

	if (core == NULL)
		return sprintf(buf, "core not inited!\n");

	ret += sprintf(buf, "*User via BPU Core(%d)*\n", core->index);
	ret += sprintf(buf + ret, "%s\t\t%s\n", "user", "ratio");
	list_for_each_safe(pos, pos_n, &core->user_list) {
		tmp_user = list_entry(pos, struct bpu_user, node);
		if (tmp_user) {
		ret += sprintf(buf + ret, "%d\t\t%d\n",
				tmp_user->id, bpu_user_ratio(tmp_user));
		}
	}

	return ret;
}

static ssize_t bpu_core_power_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct bpu_core *core = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", core->power_level);
}

static ssize_t bpu_core_power_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct bpu_core *core = dev_get_drvdata(dev);
	int32_t power_level;
	int32_t ret;

	ret = sscanf(buf, "%du", &power_level);
	if (ret < 0)
		return 0;

	ret = bpu_core_set_freq_level(core, power_level);
	if (ret < 0)
		return 0;

	return len;
}

static ssize_t bpu_core_limit_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct bpu_core *core = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", core->fc_buf_limit);
}

static ssize_t bpu_core_limit_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct bpu_core *core = dev_get_drvdata(dev);
	int32_t fc_buf_limit;
	int32_t ret;

	ret = sscanf(buf, "%du", &fc_buf_limit);
	if (ret < 0)
		return 0;

	bpu_core_set_limit(core, fc_buf_limit);

	return len;
}

static DEVICE_ATTR(ratio, S_IRUGO, bpu_core_ratio_show, NULL);
static DEVICE_ATTR(queue, S_IRUGO, bpu_core_queue_show, NULL);
static DEVICE_ATTR(fc_time, S_IRUGO, bpu_core_fc_time_show, NULL);
static DEVICE_ATTR(users, S_IRUGO, bpu_core_users_show, NULL);

/*
 * core power level >0: kernel dvfs;
 * 0:highest; -1,-2...: different work power level */
static DEVICE_ATTR(power_level, S_IRUGO | S_IWUSR,
		bpu_core_power_show, bpu_core_power_store);

static DEVICE_ATTR(limit, S_IRUGO | S_IWUSR,
		bpu_core_limit_show, bpu_core_limit_store);

static struct attribute *bpu_core_attrs[] = {
	&dev_attr_ratio.attr,
	&dev_attr_queue.attr,
	&dev_attr_fc_time.attr,
	&dev_attr_power_level.attr,
	&dev_attr_users.attr,
	&dev_attr_limit.attr,
	NULL,
};

static struct attribute_group bpu_core_attr_group = {
	.attrs = bpu_core_attrs,
};

static ssize_t bpu_ratio_show(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	if (g_bpu == NULL)
		return sprintf(buf, "bpu not inited!\n");

	return sprintf(buf, "%d\n", bpu_ratio(g_bpu));
}

static ssize_t bpu_core_num_show(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	struct list_head *pos, *pos_n;
	int32_t core_num = 0;

	if (g_bpu == NULL)
		return sprintf(buf, "bpu not inited!\n");

	list_for_each_safe(pos, pos_n, &g_bpu->core_list)
		core_num++;

	return sprintf(buf, "%d\n", core_num);
}

static ssize_t bpu_group_show(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	struct bpu_fc_group *tmp_group;
	struct list_head *pos, *pos_n;
	int32_t ret = 0;

	if (g_bpu == NULL)
		return sprintf(buf, "bpu not inited!\n");

	ret += sprintf(buf, "%s\t\t%s\t%s\n", "group", "prop", "ratio");
	list_for_each_safe(pos, pos_n, &g_bpu->group_list) {
		tmp_group = list_entry(pos, struct bpu_fc_group, node);
		if (tmp_group) {
		ret += sprintf(buf + ret, "%d(%d)\t\t%d\t%d\n",
				GROUP_ID(tmp_group->id),
				GROUP_USER(tmp_group->id),
				tmp_group->proportion,
				bpu_fc_group_ratio(tmp_group));
		}
	}

	return ret;
}

static ssize_t bpu_users_show(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	struct bpu_user *tmp_user;
	struct bpu_core *tmp_core;
	struct list_head *pos, *pos_n;
	struct list_head *bpos, *bpos_n;
	int32_t ret = 0;

	if (g_bpu == NULL)
		return sprintf(buf, "bpu not inited!\n");

	ret += sprintf(buf, "*User via BPU Bus*\n");
	ret += sprintf(buf + ret, "%s\t\t%s\n", "user", "ratio");
	list_for_each_safe(pos, pos_n, &g_bpu->user_list) {
		tmp_user = list_entry(pos, struct bpu_user, node);
		if (tmp_user) {
		ret += sprintf(buf + ret, "%d\t\t%d\n",
				tmp_user->id, bpu_user_ratio(tmp_user));
		}
	}

	list_for_each_safe(pos, pos_n, &g_bpu->core_list) {
		tmp_core = list_entry(pos, struct bpu_core, node);
		if (tmp_core) {
			ret += sprintf(buf + ret,
					"*User via BPU Core(%d)*\n",
					tmp_core->index);
			list_for_each_safe(bpos, bpos_n, &tmp_core->user_list) {
				tmp_user = list_entry(bpos, struct bpu_user, node);
				if (tmp_user) {
					ret += sprintf(buf + ret, "%d\t\t%d\n",
							tmp_user->id,
							bpu_user_ratio(tmp_user));
				}
			}
		}
	}

	return ret;
}

static struct kobj_attribute ratio_info = __ATTR(ratio,
		S_IRUGO, bpu_ratio_show, NULL);
static struct kobj_attribute core_num_info = __ATTR(core_num,
		S_IRUGO, bpu_core_num_show, NULL);
static struct kobj_attribute group_info = __ATTR(group,
		S_IRUGO, bpu_group_show, NULL);
static struct kobj_attribute users_info = __ATTR(users,
		S_IRUGO, bpu_users_show, NULL);

static struct attribute *bpu_attrs[] = {
	&ratio_info.attr,
	&core_num_info.attr,
	&group_info.attr,
	&users_info.attr,
	NULL,
};

static struct attribute *bpu_cores_attrs[] = {
	NULL,
};

static struct attribute_group bpu_cores_attr_group = {
	.name = "cores",
	.attrs = bpu_cores_attrs,
};

static struct attribute_group bpu_attr_group = {
	.attrs = bpu_attrs,
};

static const struct attribute_group *bpu_attr_groups[] = {
	&bpu_attr_group,
	&bpu_cores_attr_group,
	NULL,
};

struct bus_type bpu_subsys = {
	.name = "bpu",
};

int32_t bpu_core_create_sys(struct bpu_core *core)
{
	char core_name[10];
	int32_t ret;

	if (core == NULL)
		return -ENODEV;

	if (g_bpu->bus == NULL) {
		dev_err(core->dev, "BPU not register bus for sys\n");
		return -ENODEV;
	}

	ret = sprintf(core_name, "%d", core->index);
	if (ret < 0) {
		dev_err(core->dev, "Create debug name failed\n");
		goto err;
	}

	ret = device_add_group(core->dev, &bpu_core_attr_group);
	if (ret < 0) {
		dev_err(core->dev, "Create bpu core debug group failed\n");
		goto err;
	}

	if (core->hw_ops->debug) {
		ret = core->hw_ops->debug(core, 1);
		if (ret < 0) {
			device_remove_group(core->dev, &bpu_core_attr_group);
			dev_err(core->dev, "Create bpu core hw debug failed\n");
			goto err;
		}
	}

	ret = sysfs_add_link_to_group(&bpu_subsys.dev_root->kobj, "cores",
				      &core->dev->kobj, core_name);
	if (ret) {
		if (core->hw_ops->debug)
			ret = core->hw_ops->debug(core, 0);
		device_remove_group(core->dev, &bpu_core_attr_group);
		dev_err(core->dev, "Create link to bpu bus failed\n");
		goto err;
	}

	return 0;

err:
	return ret;
}
EXPORT_SYMBOL(bpu_core_create_sys);

int32_t bpu_core_discard_sys(struct bpu_core *core)
{
	char core_name[10];

	if (core == NULL)
		return 0;

	sprintf(core_name, "%d", core->index);

	sysfs_remove_link_from_group(&bpu_subsys.dev_root->kobj, "cores",
						core_name);
	device_remove_group(core->dev, &bpu_core_attr_group);
	if (core->hw_ops->debug)
		core->hw_ops->debug(core, 0);

	return 0;
}
EXPORT_SYMBOL(bpu_core_discard_sys);

/* create bus sub system in /sys/devices/system/ */
int32_t bpu_sys_system_init(struct bpu *bpu)
{
	int32_t ret = 0;

	if (bpu == NULL) {
		pr_err("no device for bpu subsystem\n");
		return -ENODEV;
	}

	ret = subsys_system_register(&bpu_subsys, bpu_attr_groups);
	if (ret)
		pr_err("failed to register bpu subsystem\n");

	bpu->bus = &bpu_subsys;

	return ret;
}
