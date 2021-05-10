/*
 * Copyright (C) 2019 Horizon Robotics
 * X2 CNN driver (found in Hobot Platform)
 *
 * 2017 - 2018 (C) Hobot Inc.
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any
 * later version.
 *
 */
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <asm/cacheflush.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/of_reserved_mem.h>
#include <linux/reset.h>
#include <linux/printk.h>
#include <linux/poll.h>
#include <linux/debugfs.h>
#include <linux/time.h>
#include <linux/kfifo.h>
#include <linux/kthread.h>
#include <linux/clk-provider.h>
#include <linux/regulator/consumer.h>
#include <soc/hobot/diag.h>
#ifdef CONFIG_HOBOT_CNN_DEVFREQ
#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>
#include <linux/pm_opp.h>
#endif
#include "hobot_cnn_host.h"

#define CNN_FREQ_CHANGE(id) (50 + id)
#define BPU_CLOCK_DIS   (1 << 0)
#define BPU_REGU_DIS   (1 << 1)
#define NETLINK_BPU 24
#define MOD_FRQ_DONE 0X0
#define MOD_FRQ      0X01
#define CHECK_IRQ_LOST

#define HW_ID_MAX				0xFFF

#ifdef CHECK_IRQ_LOST
static void hobot_check_cnn(unsigned long arg);
#endif

#define FC_TIME_CNT 53
static DEFINE_MUTEX(hobot_bpu_mutex);
static char *g_chrdev_name = "cnn";
static struct hobot_bpu_dev *cnn0_dev;
static struct hobot_bpu_dev *cnn1_dev;
static int profiler_frequency;
static int profiler_n_seconds = 1;
static int profiler_enable;
static int burst_length = 0x80;
static int fc_time_enable;
static int ratio0;
static int ratio1;
static int queue0;
static int queue1;
static int bpu0_clk;
static int bpu1_clk;
static int bpu0_power;
static int bpu1_power;
static int bpu0_hotplug;
static int bpu1_hotplug;
static int bpu_err_flag;

static struct timer_list check_timer;
static struct mutex enable_lock;
static int hotplug_ext_flag;

#define MAX_PID_NUM 0x8
static pid_t pid_fc_id_mask[MAX_PID_NUM];

#ifdef CHECK_IRQ_LOST
static int checkirq0_enable = 1;
static int checkirq1_enable = 1;
#endif
static int has_busy_status;

#define MAINT_PREP_START_TIMES	500
static bool recovery = 0;
module_param(recovery, bool, S_IRUSR);
static int maintain_pre_time;
static uint64_t max_check_interval;
static int hobot_bpu_maintain_thread(void *data);

static inline u32 hobot_bpu_reg_read(struct hobot_bpu_dev *dev, u32 off)
{
	return readl(dev->cnn_base + off);
}

static inline void hobot_bpu_reg_write(struct hobot_bpu_dev *dev,
	u32 off, u32 val)
{
	writel(val, dev->cnn_base + off);
}

static int soc_is_x3e(void)
{
	int32_t chipid;

	void __iomem *chipid_reg = ioremap_nocache(0xa6008070, 4);
	chipid = readl(chipid_reg);
	iounmap(chipid_reg);

	if (((chipid>>12)&0x1) == 0x1) {
		return 1;
	}

	return 0;
}

int fc_fifo_stat_info(struct seq_file *m, void *data)
{
	struct cnn_info_node *node = (struct cnn_info_node *)m->private;
	struct hobot_bpu_dev *dev = node->cnn_dev;
	u32 fc_head_idx, fc_tail_idx,
	    fc_head_flag, fc_tail_flag,
	    fc_depth, inst_num;

	fc_head_idx = hobot_bpu_reg_read(dev, CNN_FC_HEAD);
	fc_head_flag = fc_head_idx & CNN_FC_IDX_FLAG;
	fc_head_idx &= CNN_MAX_FC_LEN_MASK;

	fc_tail_idx = hobot_bpu_reg_read(dev, CNN_FC_TAIL);
	fc_tail_flag = fc_tail_idx & CNN_FC_IDX_FLAG;
	fc_tail_idx &= CNN_MAX_FC_LEN_MASK;

	fc_depth = hobot_bpu_reg_read(dev, CNN_FC_LEN);
	inst_num = hobot_bpu_reg_read(dev, CNNINT_INST_NUM);
	seq_printf(m, "%s\t%s\t%s\t%s\t%s\t%s\t%s\t\n",
			"fc_len",
			"fc_head",
			"fc_head_flag",
			"fc_tail",
			"fc_tail_flag",
			"inst_num",
			"wait_cnt");
	seq_printf(m, "%d\t%d\t%-9d\t%d\t%-9d\t%-9d\t%-9d\t\n",
		   fc_depth,
		   fc_head_idx,
		   fc_head_flag,
		   fc_tail_idx,
		   fc_tail_flag,
		   inst_num,
		   atomic_read(&dev->wait_fc_cnt));

	return 0;
}

int fc_dump_info(struct seq_file *m, void *data)
{
	int i;
	struct cnn_info_node *node = (struct cnn_info_node *)m->private;
	struct hobot_bpu_dev *dev = node->cnn_dev;
	unsigned char *fc_buf = kmalloc(CNN_FC_SPACE_LEN, GFP_KERNEL);

	memset(fc_buf, 0, CNN_FC_SPACE_LEN);

	memcpy(fc_buf, dev->fc_base, CNN_FC_SPACE_LEN);
	seq_printf(m, "0x%x ", fc_buf[0]);
	//for (i = 1; i <= CNN_FC_SPACE_LEN; i++) {
	for (i = 1; i < CNN_FC_SPACE_LEN; i++) {
		seq_printf(m, "[%d] 0x%x ", i, fc_buf[i]);
		if (i % 15 == 0)
			seq_puts(m, "\n");
	}
	return 0;
}

static struct cnn_debugfs_info cnn_debugfs_list[] = {
	{"fc_fifo_stat", fc_fifo_stat_info},
	{"fc_dump", fc_dump_info},
};
#define CNN_DEBUGFS_ENTRIES ARRAY_SIZE(cnn_debugfs_list)

static int cnn_debugfs_open(struct inode *inode, struct file *file)
{
	struct cnn_info_node *node = inode->i_private;

	return single_open(file, node->info_ent->show, node);
}


static const struct file_operations cnn_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = cnn_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/**
 * hobot_bpu_reset_assert - asserts reset using reset framework
 * @rstc: pointer to reset_control
 *
 * Return: 0 on success or error on failure
 */
static int hobot_bpu_reset_assert(struct reset_control *rstc)
{
	int rc = 0;

	rc = reset_control_assert(rstc);
	if (rc < 0) {
		pr_err("%s: failed\n", __func__);
		return rc;
	}

	return rc;
}

/**
 * hobot_bpu_reset_release - de-asserts reset using reset framework
 * @rstc: pointer to reset_control
 *
 * Return: 0 on success or error on failure
 */
static int hobot_bpu_reset_release(struct reset_control *rstc)
{
	int rc = 0;

	rc = reset_control_deassert(rstc);
	if (rc < 0) {
		pr_err("%s: failed\n", __func__);
		return rc;
	}

	return rc;
}

static void hobot_bpubus_wm_set(struct hobot_bpu_dev *dev, u32 reg_off,
	u32 wd_maxlen, u32 wd_endian, u32 wd_priority)
{
	u32 reg_val;

	reg_val = hobot_bpu_reg_read(dev, reg_off);
	reg_val &= ~(CNN_WD_MAXLEN_M_MASK |
		CNN_WD_ENDIAN_M_MASK |
		CNN_WD_PRIORITY_M_MASK);

	reg_val |= CNN_WD_MAXLEN_M(wd_maxlen) |
		CNN_WD_ENDIAN_M(wd_endian) |
		CNN_WD_PRIORITY_M(wd_priority);

	hobot_bpu_reg_write(dev, reg_off, reg_val);
}

static void hobot_bpubus_rm_set(struct hobot_bpu_dev *dev, u32 reg_off,
	u32 rd_maxlen, u32 rd_endian, u32 rd_priority)
{
	u32 reg_val;

	reg_val = hobot_bpu_reg_read(dev, reg_off);
	reg_val &= ~(CNN_RD_MAXLEN_M_MASK |
		CNN_RD_ENDIAN_M_MASK |
		CNN_RD_PRIORITY_M_MASK);

	reg_val |= CNN_RD_MAXLEN_M(rd_maxlen) |
		CNN_RD_ENDIAN_M(rd_endian) |
		CNN_RD_PRIORITY_M(rd_priority);

	hobot_bpu_reg_write(dev, reg_off, reg_val);
}

static void hobot_bpu_set_default_fc_depth(struct hobot_bpu_dev *dev, int fc_depth)
{
	u32 reg_val;

	if (fc_depth >= 1023)
		fc_depth = 1023;

	reg_val = hobot_bpu_reg_read(dev, CNN_FC_LEN);
	reg_val &=  ~(CNN_PE0_FC_LENGTH_MASK);

	reg_val |= CNN_PE0_FC_LENGTH(fc_depth);
	hobot_bpu_reg_write(dev, CNN_FC_LEN, reg_val);
}

static void hobot_bpu_set_fc_base(struct hobot_bpu_dev *dev)
{
	u32 reg_val;

	reg_val = hobot_bpu_reg_read(dev, CNN_FC_BASE);
	reg_val &=  ~(CNN_PE0_FC_BASE_MASK);

	reg_val |= CNN_PE0_FC_BASE(dev->fc_phys_base);
	hobot_bpu_reg_write(dev, CNN_FC_BASE, reg_val);
}

static int hobot_bpu_hw_init(struct hobot_bpu_dev *dev)
{

	/* Config axi write master */
	hobot_bpubus_wm_set(dev, CNNBUS_CTRL_WM_0, burst_length, 0xf, 0x1);
	hobot_bpubus_wm_set(dev, CNNBUS_CTRL_WM_1, 0x80, 0xf, 0x2);
	hobot_bpubus_wm_set(dev, CNNBUS_CTRL_WM_2, 0x8, 0x0, 0x3);
	hobot_bpubus_wm_set(dev, CNNBUS_CTRL_WM_3, 0x80, 0x0, 0x4);

	/* Config axi read master */
	hobot_bpubus_rm_set(dev, CNNBUS_CTRL_RM_0, 0x80, 0xf, 0x4);
	hobot_bpubus_rm_set(dev, CNNBUS_CTRL_RM_1, 0x80, 0xf, 0x4);
	hobot_bpubus_rm_set(dev, CNNBUS_CTRL_RM_2, 0x8, 0xf, 0x4);
	hobot_bpubus_rm_set(dev, CNNBUS_CTRL_RM_3, 0x80, 0x8, 0x5);
	hobot_bpubus_rm_set(dev, CNNBUS_CTRL_RM_4, 0x80, 0xf, 0x4);
	hobot_bpubus_rm_set(dev, CNNBUS_CTRL_RM_5, 0x80, 0x0, 0x6);

	/* Set axibus id */
	hobot_bpu_reg_write(dev, CNNBUS_AXIID, 0x0);

	return 0;
}


/**
 * hobot_bpu_get_resets - sw reset cnn controller
 * @cnn_dev: pointer cnn dev struct
 * @cnn_id:  cnn id (0/1)
 *
 * Return: 0 on success or error on failure
 */

static int hobot_bpu_get_resets(struct hobot_bpu_dev *cnn_dev)
{
	char *name;
	struct reset_control *rst_temp;

	cnn_dev->cnn_rst =
		devm_reset_control_get(cnn_dev->dev, "cnn_rst");
	if (IS_ERR(cnn_dev->cnn_rst)) {
		name = "cnn_rst";
		rst_temp = cnn_dev->cnn_rst;
		goto error;
	}

	return 0;
error:
	dev_err(cnn_dev->dev, "failed to get %s reset signal\n", name);
	return PTR_ERR(rst_temp);
}
static void report_bpu_diagnose_msg(u32 err, int core_index)
{
	u32 ret;
	u8 bpu_event;
	u8 bpu_diag_envdata[5]; // bpu diag env data: core_id(1ybte) +  error_code(4 bytes)

	ret = err & 0xf000;
	bpu_event = EventIdBpu0Err + core_index;

	if (ret == 0x1000 || ret == 0x2000 || ret == 0x3000 ||
		ret == 0x4000 || ret == 0x5000 || ret == 0x6000 ||
		ret == 0x7000 || ret == 0x8000 || ret == 0x9000 ) {

		bpu_diag_envdata[0] = (u8)core_index;
		memcpy(bpu_diag_envdata + 1, (uint8_t *)&ret, sizeof(u32));
		diag_send_event_stat_and_env_data(
									DiagMsgPrioHigh, ModuleDiag_bpu,
									bpu_event, DiagEventStaFail,
									DiagGenEnvdataWhenErr, bpu_diag_envdata, 5);
	} else
		diag_send_event_stat(DiagMsgPrioMid, ModuleDiag_bpu,
							bpu_event, DiagEventStaSuccess);
}
/**
 * hobot_bpu_hw_reset_reinit - sw reset cnn controller
 * @cnn_dev: pointer cnn dev struct
 * @cnn_id:  cnn id (0/1)
 *
 * Return: 0 on success or error on failure
 */
static int hobot_bpu_hw_reset_reinit(struct hobot_bpu_dev *cnn_dev, int cnn_id)
{
	int ret = 0;
	hobot_bpu_reset_assert(cnn_dev->cnn_rst);
	udelay(1);
	hobot_bpu_reset_release(cnn_dev->cnn_rst);

	hobot_bpu_hw_init(cnn_dev);
	hobot_bpu_set_fc_base(cnn_dev);
	hobot_bpu_set_default_fc_depth(cnn_dev, 1023);
	return ret;
}
static void lock_bpu(struct hobot_bpu_dev *dev)
{
	mutex_lock(&dev->cnn_lock);
	while (dev->zero_int_cnt > 0) {
		dev->wait_nega_flag = 1;
		mutex_unlock(&dev->cnn_lock);
		/*wait for positive int num*/
		wait_for_completion(&dev->nega_completion);
		mutex_lock(&dev->cnn_lock);
	}

	if ((atomic_read(&dev->wait_fc_cnt) > 0)) {
		atomic_set(&dev->hw_flg, MOD_FRQ);
		wait_for_completion_timeout(&dev->bpu_completion,
					    HZ / 50);
		while ((atomic_read(&dev->wait_fc_cnt) > 0)) {
			wait_for_completion_timeout(&dev->bpu_completion,
						    HZ / 50);
		}
	}
}

static void unlock_bpu(struct hobot_bpu_dev *dev)
{

	atomic_set(&dev->hw_flg, MOD_FRQ_DONE);
	mutex_unlock(&dev->cnn_lock);
}

static irqreturn_t hobot_bpu_interrupt_handler(int irq, void *dev_id)
{
	int ret;
	struct hobot_bpu_dev *dev = (struct hobot_bpu_dev *)dev_id;
	unsigned long flags;
	u32 irq_status;
	u32 tmp_irq;
	u32 irq_err;
	struct bpu_int_info tmp;
	struct cnn_user_info **p_user_info;
	int lost_report = 0;

	spin_lock_irqsave(&dev->cnn_spin_lock, flags);

	irq_status = hobot_bpu_reg_read(dev, CNNINT_STATUS);
	hobot_bpu_reg_write(dev, CNNINT_MASK, 0x1);
	tmp_irq = hobot_bpu_reg_read(dev, CNNINT_NUM);

	hobot_bpu_reg_write(dev, CNNINT_MASK, 0x0);
	irq_err = tmp_irq & 0xf000;
	if (bpu_err_flag == 1) {
		irq_err = 0x1000;
		bpu_err_flag = 0;
	}
	report_bpu_diagnose_msg(irq_err, dev->core_index);
	spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
	dev->head_value = 0;
	dev->inst_num = 0;

	do {
		lost_report = 0;
		if (tmp_irq & 0xf000) {
			ret = kfifo_out_peek(&dev->int_info_fifo, &tmp, sizeof(struct bpu_int_info));
			if (ret) {
				spin_lock_irqsave(&dev->cnn_spin_lock, flags);
				p_user_info = tmp.p_user_info;
				if (!(*p_user_info) || !p_user_info) {
					spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
					break;
				}
				(*p_user_info)->cnn_int_num.cnn_int_num[(*p_user_info)->cnn_int_num.cnn_int_count] = tmp_irq;
				(*p_user_info)->cnn_int_num.cnn_int_interval[(*p_user_info)->cnn_int_num.cnn_int_count] = 0;
				if ((*p_user_info)->cnn_int_num.cnn_int_count < CNN_INT_NUM - 1)
					(*p_user_info)->cnn_int_num.cnn_int_count++;
				(*p_user_info)->irq_triggered = 1;
				wake_up_interruptible(&(*p_user_info)->cnn_int_wait);
				spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
			}
			break;
		}

		ret = kfifo_out_spinlocked(&dev->int_info_fifo, &tmp,
						sizeof(struct bpu_int_info),
						&dev->kfifo_lock);

		if (!ret) {
			pr_debug("BPU get interrupt but no fifo in!!!\n");
			break;
		}

		spin_lock_irqsave(&dev->cnn_spin_lock, flags);
		p_user_info = tmp.p_user_info;
		if (!(*p_user_info) || !p_user_info) {
			spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
			atomic_sub(tmp.fc_total, &dev->wait_fc_cnt);
			break;
		}
		(*p_user_info)->cnn_int_num.cnn_int_num[(*p_user_info)->cnn_int_num.cnn_int_count] = tmp.int_num;
		do_gettimeofday(&tmp.end_time);
		if (tmp.start_time.tv_sec * 1000000 + tmp.start_time.tv_usec
				< dev->int_point.tv_sec * 1000000 + dev->int_point.tv_usec)
			tmp.start_time = dev->int_point;

		dev->int_point = tmp.end_time;
		(*p_user_info)->cnn_int_num.cnn_int_interval[(*p_user_info)->cnn_int_num.cnn_int_count] =
			(tmp.end_time.tv_sec * 1000000 + tmp.end_time.tv_usec)
			- (tmp.start_time.tv_sec * 1000000 + tmp.start_time.tv_usec);

		if (recovery) {
			if (maintain_pre_time < MAINT_PREP_START_TIMES) {
				max_check_interval = max(max_check_interval,
						(*p_user_info)->cnn_int_num.cnn_int_interval[
						(*p_user_info)->cnn_int_num.cnn_int_count]);
				maintain_pre_time++;
			}
		}

		dev->run_time += (*p_user_info)->cnn_int_num.cnn_int_interval[(*p_user_info)->cnn_int_num.cnn_int_count];
		if ((*p_user_info)->cnn_int_num.cnn_int_count < CNN_INT_NUM - 1) {
			(*p_user_info)->cnn_int_num.cnn_int_count++;
			dev->real_int_cnt++;
		} else {
			spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
			atomic_sub(tmp.fc_total, &dev->wait_fc_cnt);
			break;
		}

		(*p_user_info)->irq_triggered = 1;
		wake_up_interruptible(&(*p_user_info)->cnn_int_wait);
		spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
		atomic_sub(tmp.fc_total, &dev->wait_fc_cnt);

		/* maybe lost a hw irq */
		if (tmp.hw_id < tmp_irq)
			lost_report = 1;
		else if (tmp.hw_id > tmp_irq) {
			if (tmp.hw_id - tmp_irq > HW_ID_MAX - 10)
				lost_report = 1;
		}

	} while (lost_report);

	pr_debug("hobot bpu interrupt\n");
	tasklet_schedule(&dev->tasklet);
	return IRQ_HANDLED;
}

static ssize_t hobot_bpu_read(struct file *filp, char __user *userbuf,
	size_t count, loff_t *f_pos)
{
	return 0;
}

static ssize_t hobot_bpu_write(struct file *filp,
	const char __user *userbuf, size_t count, loff_t *f_pos)
{
	return 0;
}

static void hobot_bpu_set_fc_tail_idx(struct hobot_bpu_dev *dev, u32 fc_tail_idx)
{
	u32 reg_val;

	reg_val = hobot_bpu_reg_read(dev, CNN_FC_TAIL);
	reg_val &=  ~(CNN_PE0_FC_TAIL_MASK);

	reg_val |= CNN_PE0_FC_TAIL(fc_tail_idx);
	hobot_bpu_reg_write(dev, CNN_FC_TAIL, reg_val);
}

static void hobot_bpu_set_fc_start_time(struct hobot_bpu_dev *dev,
					int int_num, int hw_id,
				    int count, struct cnn_user_info **p_user_info)
{
	int ret;
	struct bpu_int_info bpu_int;

	count &= CNN_MAX_FC_LEN_MASK;
	if (fc_time_enable) {
		dev->fc_time[dev->time_tail].fc_count = count;
		dev->fc_time[dev->time_tail].int_num = int_num;
	}
	if (atomic_read(&dev->wait_fc_cnt) > 0) {
		if (fc_time_enable) {
			spin_lock_bh(&dev->set_time_lock);
			dev->fc_time[dev->time_tail].time_flag = 1;
			spin_unlock_bh(&dev->set_time_lock);
		}
		/*bpu busy,check int_num*/
		if (int_num) {
			if (fc_time_enable) {
				dev->time_tail++;
				dev->time_tail %= FC_TIME_CNT;
			}
			if ((dev->zero_int_start_time.tv_sec == 0)
					&& (dev->zero_int_start_time.tv_usec == 0)) {
				do_gettimeofday(&bpu_int.start_time);
			} else {
				bpu_int.start_time = dev->zero_int_start_time;
				dev->zero_int_start_time.tv_sec = 0;
				dev->zero_int_start_time.tv_usec = 0;
			}
			bpu_int.fc_total = dev->zero_int_cnt + 1;
			bpu_int.int_num = int_num;
			bpu_int.hw_id = hw_id;
			bpu_int.p_user_info = p_user_info;
			ret = kfifo_in(&dev->int_info_fifo, &bpu_int,
				       sizeof(struct bpu_int_info));
			if (ret < sizeof(struct bpu_int_info))
				pr_err("%s[%d]:hobot bpu interrupt info fifo no space\n",
					__func__, __LINE__);
			dev->zero_int_cnt = 0;
			if (dev->wait_nega_flag) {
				complete(&dev->nega_completion);
				dev->wait_nega_flag = 0;
			}

		} else {
			if (!dev->zero_int_cnt) {
				do_gettimeofday(&dev->zero_int_start_time);
			}
			dev->zero_int_cnt++;
		}
	} else {
		if (fc_time_enable) {
			spin_lock_bh(&dev->set_time_lock);
			do_gettimeofday(&dev->fc_time[dev->time_tail].start_time);
			spin_unlock_bh(&dev->set_time_lock);
		}

		if (int_num) {
			if (fc_time_enable) {
				dev->time_tail++;
				dev->time_tail %= FC_TIME_CNT;
			}
			if ((dev->zero_int_start_time.tv_sec == 0)
					&& (dev->zero_int_start_time.tv_usec == 0)) {
				do_gettimeofday(&bpu_int.start_time);
			} else {
				bpu_int.start_time = dev->zero_int_start_time;
				dev->zero_int_start_time.tv_sec = 0;
				dev->zero_int_start_time.tv_usec = 0;
			}
			bpu_int.fc_total = dev->zero_int_cnt + 1;
			bpu_int.int_num = int_num;
			bpu_int.hw_id = hw_id;
			dev->zero_int_cnt = 0;
			if (dev->wait_nega_flag) {
				complete(&dev->nega_completion);
				dev->wait_nega_flag = 0;
			}
			bpu_int.p_user_info = p_user_info;
			ret = kfifo_in(&dev->int_info_fifo, &bpu_int,
					sizeof(struct bpu_int_info));
			if (ret < sizeof(struct bpu_int_info))
				pr_err("%s[%d]:hobot bpu interrupt info fifo no space\n",
					__func__, __LINE__);

		} else {
			if (!dev->zero_int_cnt) {
				do_gettimeofday(&dev->zero_int_start_time);
			}
			dev->zero_int_cnt++;
		}
	}
	atomic_inc(&dev->wait_fc_cnt);
}

#ifdef CNN_DEBUG
static void dump_fc_fifo_status(struct hobot_bpu_dev *dev)
{
	u32 fc_head_idx, fc_tail_idx,
	    fc_head_flag, fc_tail_flag,
	    fc_depth;

	fc_head_idx = hobot_bpu_reg_read(dev, CNN_FC_HEAD);
	fc_head_flag = fc_head_idx & CNN_FC_IDX_FLAG;
	fc_head_idx &= CNN_MAX_FC_LEN_MASK;

	fc_tail_idx = hobot_bpu_reg_read(dev, CNN_FC_TAIL);
	fc_tail_flag = fc_tail_idx & CNN_FC_IDX_FLAG;
	fc_tail_idx &= CNN_MAX_FC_LEN_MASK;

	fc_depth = hobot_bpu_reg_read(dev, CNN_FC_LEN);

	pr_info("X2-CNN function call fifo status:\n");
	pr_info("current function call length:%d\n", fc_depth);
	pr_info("head_idx:%d head_flag:%d\n", fc_head_idx, fc_head_flag);
	pr_info("tail_idx:%d tail_flag:%d\n", fc_tail_idx, fc_tail_flag);

}
#endif

/**
 * hobot_bpu_get_fc_fifo_spaces - get available spaces from cnn fc fifo queue
 * @dev: pointer to struct hobot_bpu_dev
 *
 * Return: free_fc_fifo = 0 indicate no spaces
 *	   free_fc_fifo > 0 indicate have free_fc_fifo spaces can used
 *	   others failed
 */

static u32 hobot_bpu_get_fc_fifo_spaces(struct hobot_bpu_dev *dev)
{
	u32 free_fc_fifo = 0;
	u32 fc_head_idx, fc_tail_idx,
	    fc_head_flag, fc_tail_flag;
	u32 fc_depth;

	fc_depth = hobot_bpu_reg_read(dev, CNN_FC_LEN);

	fc_head_idx = hobot_bpu_reg_read(dev, CNN_FC_HEAD);
	fc_head_flag = fc_head_idx & CNN_FC_IDX_FLAG;

	fc_tail_idx = hobot_bpu_reg_read(dev, CNN_FC_TAIL);
	fc_tail_flag = fc_tail_idx & CNN_FC_IDX_FLAG;

	fc_head_idx &= CNN_MAX_FC_LEN_MASK;
	fc_tail_idx &= CNN_MAX_FC_LEN_MASK;

	if (fc_head_flag != fc_tail_flag)
		free_fc_fifo = fc_head_idx - fc_tail_idx;
	else
		free_fc_fifo = fc_depth - fc_tail_idx + fc_head_idx + 1;

	pr_debug("fc_depth:0x%x, get fifo spaces return val:%d, head:[%d], tail:[%d]\n",
			fc_depth, free_fc_fifo, fc_head_idx, fc_tail_idx);
	return free_fc_fifo;
}
/**
 * hobot_bpu_clock_up - start cnn
 * @dev: pointer to struct hobot_bpu_dev
 */
static void hobot_bpu_clock_up(struct hobot_bpu_dev *dev)
{
	unsigned int tmp;

	lock_bpu(dev);
	if (dev->disable_bpu & BPU_REGU_DIS) {
		unlock_bpu(dev);
		return;
	}
	if (!(dev->disable_bpu & BPU_CLOCK_DIS)) {
		unlock_bpu(dev);
		return;
	}
	pr_info("%s\n", __func__);
	tmp = readl(dev->cnn_pmu);

	tmp &= ~(1 << dev->iso_bit);
	writel(tmp, dev->cnn_pmu);
	udelay(5);

	hobot_bpu_reset_release(dev->cnn_rst);
	if (!__clk_is_enabled(dev->cnn_aclk))
		clk_enable(dev->cnn_aclk);
	if (!__clk_is_enabled(dev->cnn_mclk))
		clk_enable(dev->cnn_mclk);
	hobot_bpu_hw_init(dev);
	hobot_bpu_set_fc_base(dev);
	hobot_bpu_set_default_fc_depth(dev, 1023);
	dev->disable_bpu &= ~BPU_CLOCK_DIS;
	unlock_bpu(dev);
}
/**
 * hobot_bpu_clock_down - stop cnn
 * @dev: pointer to struct hobot_bpu_dev
 */
static void hobot_bpu_clock_down(struct hobot_bpu_dev *dev)
{
	unsigned int tmp;

	lock_bpu(dev);
	if (dev->disable_bpu & BPU_REGU_DIS) {
		unlock_bpu(dev);
		return;
	}
	if (!__clk_is_enabled(dev->cnn_aclk) &&
		!__clk_is_enabled(dev->cnn_mclk)) {
		unlock_bpu(dev);
		return;
	}
	pr_info("%s\n", __func__);
	if (__clk_is_enabled(dev->cnn_aclk))
		clk_disable(dev->cnn_aclk);
	if (__clk_is_enabled(dev->cnn_mclk))
		clk_disable(dev->cnn_mclk);
	tmp = readl(dev->cnn_pmu);

	tmp |= (1 << dev->iso_bit);
	writel(tmp, dev->cnn_pmu);
	udelay(5);

	hobot_bpu_reset_assert(dev->cnn_rst);
	dev->disable_bpu |= BPU_CLOCK_DIS;
	unlock_bpu(dev);
}

/**
 * hobot_bpu_power_up - start cnn
 * @dev: pointer to struct hobot_bpu_dev
 */
static void hobot_bpu_power_up(struct hobot_bpu_dev *dev)
{
	int ret;
	unsigned int tmp;

	lock_bpu(dev);
	if (!dev->disable_bpu) {
		unlock_bpu(dev);
		return;
	}
	pr_info("%s\n", __func__);
	if (!regulator_is_enabled(dev->cnn_regulator)) {
		ret = regulator_enable(dev->cnn_regulator);
		if (ret != 0)
			dev_err(dev->dev, "regulator enable error\n");
	}
	dev->disable_bpu &= ~BPU_REGU_DIS;
	tmp = readl(dev->cnn_pmu);

	tmp &= ~(1 << dev->iso_bit);
	writel(tmp, dev->cnn_pmu);
	udelay(5);

	hobot_bpu_reset_release(dev->cnn_rst);
	if (!__clk_is_enabled(dev->cnn_aclk))
		clk_enable(dev->cnn_aclk);
	if (!__clk_is_enabled(dev->cnn_mclk))
		clk_enable(dev->cnn_mclk);
	hobot_bpu_hw_init(dev);
	hobot_bpu_set_fc_base(dev);
	hobot_bpu_set_default_fc_depth(dev, 1023);
	dev->disable_bpu &= ~BPU_CLOCK_DIS;
	unlock_bpu(dev);
}
/**
 * hobot_bpu_power_down - stop cnn
 * @dev: pointer to struct hobot_bpu_dev
 */
static void hobot_bpu_power_down(struct hobot_bpu_dev *dev)
{
	unsigned int tmp;

	lock_bpu(dev);
	if (dev->disable_bpu == (BPU_CLOCK_DIS | BPU_REGU_DIS)) {
		unlock_bpu(dev);
		return;
	}
	pr_info("%s\n", __func__);
	if (__clk_is_enabled(dev->cnn_aclk))
		clk_disable(dev->cnn_aclk);
	if (__clk_is_enabled(dev->cnn_mclk))
		clk_disable(dev->cnn_mclk);
	dev->disable_bpu |= BPU_REGU_DIS;
	tmp = readl(dev->cnn_pmu);

	tmp |= (1 << dev->iso_bit);
	writel(tmp, dev->cnn_pmu);
	udelay(5);

	hobot_bpu_reset_assert(dev->cnn_rst);
	if (regulator_is_enabled(dev->cnn_regulator))
		regulator_disable(dev->cnn_regulator);
	dev->disable_bpu |= BPU_CLOCK_DIS;
	unlock_bpu(dev);
}

static int hobot_bpu_replace_hw_id(struct hobot_bpu_dev *dev, void *data)
{
	struct hbrt_x2_funccall_s *fc = data;
	int tmp_id;

	if (!fc | !dev)
		return 0;

	if (!fc->interrupt_num)
		return 0;

	tmp_id = atomic_read(&dev->hw_id_counter);
	if (unlikely(tmp_id >= HW_ID_MAX))
		atomic_set(&dev->hw_id_counter, 1);
	else
		atomic_inc(&dev->hw_id_counter);

	fc->interrupt_num = tmp_id;

	return tmp_id;
}

static struct hobot_bpu_dev *hobot_check_power(struct hobot_bpu_dev *bpu_dev)
{
	struct hobot_bpu_dev * dev = NULL;

	if ((bpu_dev->disable_bpu || bpu_dev->bpu_detached) && hotplug_ext_flag) {
		mutex_unlock(&bpu_dev->cnn_lock);
		if (bpu_dev->core_index)
			dev = cnn0_dev;
		else
			dev = cnn1_dev;
		/*power up dev if closed*/
		mutex_lock(&dev->cnn_lock);

		if (!dev->bpu_detached && dev->disable_bpu) {
			mutex_unlock(&dev->cnn_lock);
			if (bpu_dev->has_regulator)
				hobot_bpu_power_up(dev);
			else
				hobot_bpu_clock_up(dev);
			mutex_lock(&dev->cnn_lock);
		}

	} else {
		dev = bpu_dev;
	}
	return dev;
}

/**
 * hobot_bpu_fc_fifo_enqueue - fill the function call into the reserved memory
 * @dev: pointer to struct hobot_bpu_dev
 * fc_buf: function call buf struct
 *
 * Return: 0 success, others failed
 */
static int hobot_bpu_fc_fifo_enqueue(struct hobot_bpu_dev *dev,
		struct hobot_bpu_fc_info *fc_buf, struct cnn_user_info **p_user_info)
{
	u32 rc = 0;
	u32 i;
	u32 free_fc_fifo = 0;
	u32 fc_head_idx, fc_tail_idx,
	    fc_head_flag, fc_tail_flag;
	u32 fc_depth, insert_fc_cnt, residue_fc_cnt;
	u32 count;
	struct hbrt_x2_funccall_s *tmp_ptr = NULL;
	int tmp_orgin_id;

#ifndef CONFIG_HOBOT_FPGA_X3
	if (dev->disable_bpu) {
		pr_err("%s:line:%d bpu no power\n", __func__, __LINE__);
		return -1;
	}

#endif
	fc_depth = hobot_bpu_reg_read(dev, CNN_FC_LEN);
	fc_head_idx = hobot_bpu_reg_read(dev, CNN_FC_HEAD);
	fc_head_flag = fc_head_idx & CNN_FC_IDX_FLAG;

	fc_tail_idx = hobot_bpu_reg_read(dev, CNN_FC_TAIL);
	fc_tail_flag = fc_tail_idx & CNN_FC_IDX_FLAG;
	tmp_ptr = (struct hbrt_x2_funccall_s *)fc_buf->fc_info;

	fc_head_idx &= CNN_MAX_FC_LEN_MASK;
	fc_tail_idx &= CNN_MAX_FC_LEN_MASK;
	if (fc_head_flag != fc_tail_flag)
		free_fc_fifo = fc_head_idx - fc_tail_idx;
	else
		free_fc_fifo = fc_depth - fc_tail_idx + fc_head_idx + 1;

	if (fc_buf->fc_cnt > free_fc_fifo) {
		rc = -1;
		pr_err("no available fc fifo spaces\n");
		return rc;
	}

	if (!(*p_user_info) || !p_user_info) {
		rc = -1;
		return rc;
	}
	count = fc_tail_idx;
	if ((fc_tail_idx + fc_buf->fc_cnt)  > fc_depth) {
		insert_fc_cnt = fc_depth - fc_tail_idx + 1;

		for (i = 0; i < insert_fc_cnt; i++) {
			tmp_orgin_id = tmp_ptr->interrupt_num;
			hobot_bpu_set_fc_start_time(dev,
						 tmp_orgin_id,
						 hobot_bpu_replace_hw_id(dev, tmp_ptr),
						 count, p_user_info);
			tmp_ptr++;
			count++;
		}

		memcpy(dev->fc_base + fc_tail_idx * CNN_FC_SIZE,
			fc_buf->fc_info, insert_fc_cnt * CNN_FC_SIZE);

		if (fc_tail_flag)
			fc_tail_flag = 0;
		else
			fc_tail_flag = CNN_FC_IDX_FLAG;

		residue_fc_cnt = fc_buf->fc_cnt - insert_fc_cnt;
		if (residue_fc_cnt > 0) {
			for (i = 0; i < residue_fc_cnt; i++) {
				tmp_orgin_id = tmp_ptr->interrupt_num;
				hobot_bpu_set_fc_start_time(dev,
							tmp_orgin_id,
							hobot_bpu_replace_hw_id(dev, tmp_ptr),
							count, p_user_info);
				tmp_ptr++;
				count++;
			}
			memcpy(dev->fc_base,
			fc_buf->fc_info + (insert_fc_cnt * CNN_FC_SIZE),
			residue_fc_cnt * CNN_FC_SIZE);
		}

		hobot_bpu_set_fc_tail_idx(dev, residue_fc_cnt | fc_tail_flag);
	} else {
		for (i = 0; i < fc_buf->fc_cnt; i++) {
			tmp_orgin_id = tmp_ptr->interrupt_num;
			hobot_bpu_set_fc_start_time(dev,
						 tmp_orgin_id,
						 hobot_bpu_replace_hw_id(dev, tmp_ptr),
						 count, p_user_info);
			tmp_ptr++;
			count++;
		}
		memcpy(dev->fc_base + (fc_tail_idx * CNN_FC_SIZE),
			fc_buf->fc_info, fc_buf->fc_cnt * CNN_FC_SIZE);

		hobot_bpu_set_fc_tail_idx(dev,
				fc_tail_flag | (fc_tail_idx + fc_buf->fc_cnt));
	}

#ifdef CNN_DEBUG
	dump_fc_fifo_status(dev);
#endif
	return rc;
}

static int hobot_bpu_open(struct inode *inode, struct file *filp)
{
	int rc = 0;
	struct hobot_bpu_dev *devdata;
	struct cnn_user_info *user_info;
	int i;

	filp->private_data = kzalloc(sizeof(struct cnn_user_info), GFP_KERNEL);
	if (!filp->private_data) {
		pr_err("BPU open failed, can't create user\n");
		return -EINVAL;
	}

	user_info = filp->private_data;

	mutex_lock(&hobot_bpu_mutex);

	devdata = container_of(inode->i_cdev, struct hobot_bpu_dev, i_cdev);

	user_info->cnn_dev = devdata;

	init_waitqueue_head(&user_info->cnn_int_wait);

	for (i = 0; i < MAX_PID_NUM; i++) {
		if (pid_fc_id_mask[i] == 0
				|| pid_fc_id_mask[i] == task_pid_nr(current->group_leader))
			break;
	}

	if (i == MAX_PID_NUM) {
		mutex_unlock(&hobot_bpu_mutex);
		kfree(filp->private_data);
		filp->private_data = NULL;
		pr_err("Too many processes, BPU now support max %d processes\n", MAX_PID_NUM);
		return -EINVAL;
	}

	if (pid_fc_id_mask[i] != task_pid_nr(current->group_leader))
		pid_fc_id_mask[i] = task_pid_nr(current->group_leader);

	if (!devdata->bpu_detached) {
		if (!(devdata->ref_cnt++)) {
			/*power up bpu first*/
			if (devdata->has_regulator)
				hobot_bpu_power_up(devdata);
			else
				hobot_bpu_clock_up(devdata);

			hobot_bpu_reg_write(devdata,
					CNN_FC_LEN, (devdata->fc_mem_size / 64) - 1);
			hobot_bpu_reg_write(devdata, CNN_FC_BASE,
					devdata->fc_phys_base);
			hobot_bpu_reg_write(devdata, CNNINT_MASK, 0x0);

			devdata->zero_int_cnt = 0;
			devdata->time_head = 0;
			devdata->time_tail = 0;
			memset(devdata->fc_time, 0 ,
			       FC_TIME_CNT * sizeof(struct hobot_fc_time));
#ifdef CHECK_IRQ_LOST
			if ((devdata->core_index && checkirq1_enable) ||
				(devdata->core_index == 0 && checkirq0_enable)) {
				devdata->cnn_timer.expires = jiffies + HZ * 5;
				add_timer(&devdata->cnn_timer);
			}
#endif
			if (recovery) {
				maintain_pre_time = 0;
				devdata->maintain_task = kthread_run(
						hobot_bpu_maintain_thread,
						devdata, devdata->chrdev_name);
			}
		}
	}
	mutex_unlock(&hobot_bpu_mutex);
	hobot_bpu_reg_read(devdata, CNN_FC_LEN);

	return rc;
}

static int hobot_bpu_release(struct inode *inode, struct file *filp)
{
	struct hobot_bpu_dev *devdata;
	struct cnn_user_info *user_info;
	unsigned long flags;
	int i;

	mutex_lock(&hobot_bpu_mutex);
	user_info = filp->private_data;
	devdata = user_info->cnn_dev;
	if (!devdata->bpu_detached) {
		if (!(--devdata->ref_cnt)) {
			if (devdata->has_regulator)
				hobot_bpu_power_down(devdata);
			else
				hobot_bpu_clock_down(devdata);
			user_info->cnn_int_num.cnn_int_count = 0;
			atomic_set(&devdata->wait_fc_cnt, 0);
			kfifo_reset(&devdata->int_info_fifo);
#ifdef CHECK_IRQ_LOST
			if ((devdata->core_index && checkirq1_enable) ||
				(devdata->core_index == 0 && checkirq0_enable))
				del_timer(&devdata->cnn_timer);
#endif
			if (recovery) {
				if (devdata->maintain_task) {
					kthread_stop(devdata->maintain_task);
					devdata->maintain_head = 0;
					devdata->maintain_task = NULL;
				}
			}

			devdata->inst_num = 0;
			devdata->head_value = 0;
		}
	} else {
		if (devdata->has_regulator)
			hobot_bpu_power_down(devdata);
		else
			hobot_bpu_clock_down(devdata);
	}

	mutex_unlock(&hobot_bpu_mutex);

	for (i = 0; i < MAX_PID_NUM; i++) {
		if (pid_fc_id_mask[i] == task_pid_nr(current->group_leader)) {
			pid_fc_id_mask[i] = 0;
			break;
		}
	}

	spin_lock_irqsave(&devdata->cnn_spin_lock, flags);
	kfree(filp->private_data);
	filp->private_data = NULL;
	spin_unlock_irqrestore(&devdata->cnn_spin_lock, flags);

	return 0;
}

static unsigned int cnn_ioctl_dir(unsigned int cmd)
{
	switch (cmd) {
	case CNN_IOC_GET_FC_STA:
	case CNN_IOC_GET_INT_NUM:
	case CNN_IOC_GET_ID_MASK:
		return _IOC_READ;
	case CNN_IOC_RST:
	case CNN_IOC_FC_ENQUEUE:
		return _IOC_WRITE;
	default:
		return _IOC_DIR(cmd);
	}
}

static long hobot_bpu_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int  rc = 0;
	struct hobot_bpu_dev *dev;
	u32 dir;
	union cnn_ioctl_arg data;
	void *kernel_fc_data;
	struct hbrt_x2_funccall_s *tmp_ptr = NULL;
	struct cnn_user_info *user_info = file->private_data;
	unsigned long flags;
	int i;

	dir = cnn_ioctl_dir(cmd);

	if (_IOC_TYPE(cmd) != CNN_IOCTL_MAGIC) {
		pr_err("ioctl command magic number does not match.\n");
		return -EINVAL;
	}

	if (_IOC_SIZE(cmd) > sizeof(data))
		return -EINVAL;

	/*
	 * The copy_from_user is unconditional here for both read and write
	 * to do the validate. If there is no write for the ioctl, the
	 * buffer is cleared
	 */
	if (copy_from_user(&data, (void __user *)arg, _IOC_SIZE(cmd))) {
		pr_err("%s: copy data failed from userspace\n", __func__);
		return -EFAULT;
	}

	if (!(dir & _IOC_WRITE))
		memset(&data, 0, sizeof(data));

	dev = user_info->cnn_dev;
	switch (cmd) {
	case CNN_IOC_GET_FC_STA:

		mutex_lock(&dev->cnn_lock);
		dev = hobot_check_power(dev);
		data.fc_status.free_fc_fifo_cnt =
			hobot_bpu_get_fc_fifo_spaces(dev);
		mutex_unlock(&dev->cnn_lock);
		break;
	case CNN_IOC_GET_ID_MASK:

		mutex_lock(&dev->cnn_lock);
		for (i = 0; i < MAX_PID_NUM; i++) {
			if (pid_fc_id_mask[i] == task_pid_nr(current->group_leader))
				break;
		}
		if (i == MAX_PID_NUM)
			data.pid_fc_mask = -1;
		else
			data.pid_fc_mask = i;
		mutex_unlock(&dev->cnn_lock);
		break;
	case CNN_IOC_GET_CORE_RUNTIME:

		mutex_lock(&dev->cnn_lock);
		data.core_run_time = dev->run_time;
		mutex_unlock(&dev->cnn_lock);
		break;
	case CNN_IOC_FC_ENQUEUE:

		kernel_fc_data = kmalloc(data.fc_data.fc_cnt * CNN_FC_SIZE,
					GFP_KERNEL);
		if (!kernel_fc_data) {
			pr_err("kmalloc kernel_fc_data failed\n");
			return -ENOMEM;
		}
		if (copy_from_user(kernel_fc_data,
				(void __user *)data.fc_data.fc_info,
				data.fc_data.fc_cnt * CNN_FC_SIZE)) {
			pr_err("%s: copy fc data failed from userspace\n",
				__func__);
			kfree(kernel_fc_data);
			return -EFAULT;
		}
		data.fc_data.fc_info = kernel_fc_data;

		tmp_ptr = (struct hbrt_x2_funccall_s *)data.fc_data.fc_info;

		pr_debug("!!!!func: %s, line: %d\n"
				"tmp_ptr->dyn_base_addr5 is 0x%x,\n"
				"tmp_ptr->dyn_base_addr4 is 0x%x,\n"
				"tmp_ptr->interrupt_num is %d,\n"
				"tmp_ptr->instruction_length is %d,\n"
				"tmp_ptr->instruction_address is 0x%x,\n"
				"tmp_ptr->dyn_base_addr0 is 0x%x,\n"
				"tmp_ptr->dyn_base_addr1 is 0x%x,\n"
				"tmp_ptr->dyn_base_addr3 is 0x%x.\n",
				__func__, __LINE__,
				tmp_ptr->dyn_base_addr5,
				tmp_ptr->dyn_base_addr4,
				tmp_ptr->interrupt_num,
				tmp_ptr->instruction_length,
				tmp_ptr->instruction_address,
				tmp_ptr->dyn_base_addr0,
				tmp_ptr->dyn_base_addr1,
				tmp_ptr->dyn_base_addr3);
		mutex_lock(&dev->cnn_lock);
		dev = hobot_check_power(dev);
		rc = hobot_bpu_fc_fifo_enqueue(dev, &data.fc_data,
				(struct cnn_user_info **)&file->private_data);
		if (rc < 0) {
			mutex_unlock(&dev->cnn_lock);
			pr_err("%s: failed to fill fc fifo\n", __func__);
			kfree(kernel_fc_data);
			return rc;
		}
		mutex_unlock(&dev->cnn_lock);
		kfree(kernel_fc_data);
		break;
	case CNN_IOC_RST:
		mutex_lock(&dev->cnn_lock);
		if (dev->ref_cnt > 1 || dev->disable_bpu) {
			mutex_unlock(&dev->cnn_lock);
			rc = 0;
			break;
		}

		rc = hobot_bpu_hw_reset_reinit(dev, data.rst_data.cnn_rst_id);
		if (rc < 0) {
			mutex_unlock(&dev->cnn_lock);
			pr_err("%s: failed to reset cnn%d\n",
				__func__, data.rst_data.cnn_rst_id);
			return rc;
		}
		mutex_unlock(&dev->cnn_lock);
		break;
	case CNN_IOC_GET_INT_NUM:

		mutex_lock(&dev->cnn_lock);
		spin_lock_irqsave(&dev->cnn_spin_lock, flags);
		data.int_num_data = user_info->cnn_int_num;
		user_info->cnn_int_num.cnn_int_count = 0;
		spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
		mutex_unlock(&dev->cnn_lock);
		break;
	default:
		pr_err("%s: Invalid ioctl Argument\n", __func__);
		return -EINVAL;

	}

	if (dir & _IOC_READ) {
		if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd))) {
			pr_err("%s: copy data to userspace failed\n", __func__);
			return -EFAULT;
		}
	}
	return rc;
}

static u32 hobot_bpu_poll(struct file *filp, poll_table *wait)
{
	struct cnn_user_info *user_info = filp->private_data;
	unsigned int mask = 0;
	unsigned long flags;

	struct hobot_bpu_dev *dev = user_info->cnn_dev;

	poll_wait(filp, &user_info->cnn_int_wait, wait);
	spin_lock_irqsave(&dev->cnn_spin_lock, flags);
	if (user_info->irq_triggered) {
		mask |= POLLIN | POLLRDNORM;
		user_info->irq_triggered = 0;
	}
	spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);

	return mask;
}

static const struct file_operations cnn_fops = {
	.owner		= THIS_MODULE,
	.read		= hobot_bpu_read,
	.poll		= hobot_bpu_poll,
	.write		= hobot_bpu_write,
	.open		= hobot_bpu_open,
	.release	= hobot_bpu_release,
	.unlocked_ioctl = hobot_bpu_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= hobot_bpu_ioctl,
#endif
};

static int hobot_bpu_init_chrdev(struct hobot_bpu_dev *dev)
{
	int rc;

	// Allocate a major and minor number region for the character device
	rc = alloc_chrdev_region(&dev->dev_num, dev->minor_num,
			dev->num_devices, dev->chrdev_name);
	if (rc < 0) {
		pr_err("Unable to allocate character device region.\n");
		goto ret;
	}

	// Create a device class for our device
	dev->dev_class = class_create(THIS_MODULE, dev->chrdev_name);
	if (IS_ERR(dev->dev_class)) {
		pr_err("Unable to create a device class.\n");
		rc = PTR_ERR(dev->dev_class);
		goto free_chrdev_region;
	}

	/* Create a device for our module. This will create a file on the
	 * filesystem, under "/dev/dev->chrdev_name".
	 */
	device_create(dev->dev_class, dev->dev, dev->dev_num, NULL,
		dev->chrdev_name);

	// Register our character device with the kernel
	cdev_init(&dev->i_cdev, &cnn_fops);
	rc = cdev_add(&dev->i_cdev, dev->dev_num, dev->num_devices);
	if (rc < 0) {
		pr_err("Unable to add a character device.\n");
		goto device_cleanup;
	}

	return 0;

device_cleanup:
	device_destroy(dev->dev_class, dev->dev_num);
free_chrdev_region:
	unregister_chrdev_region(dev->dev_num, dev->num_devices);
ret:
	return rc;
}

/**
 * hobot_bpu_do_tasklet - Schedule wake up tasklet
 * @data: Pointer to cnn_dev structure
 */
static void hobot_bpu_do_tasklet(unsigned long data)
{
	struct hobot_bpu_dev *dev = (struct hobot_bpu_dev *)data;
	unsigned long flags;
	int int_cnt;

	spin_lock_irqsave(&dev->cnn_spin_lock, flags);
	int_cnt = dev->real_int_cnt;
	dev->real_int_cnt = 0;
	spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
	if (fc_time_enable) {
		int head_tmp;

		while (int_cnt--) {
			spin_lock(&dev->set_time_lock);
			if (dev->time_head == dev->time_tail) {
				spin_unlock(&dev->set_time_lock);
				break;
			}
			do_gettimeofday(&dev->fc_time[dev->time_head].end_time);
			head_tmp = dev->time_head + 1;
			head_tmp %= FC_TIME_CNT;

				if (dev->fc_time[head_tmp].time_flag == 1) {

					dev->fc_time[head_tmp].start_time.tv_sec =
						dev->fc_time[dev->time_head].end_time.tv_sec;
					dev->fc_time[head_tmp].start_time.tv_usec =
						dev->fc_time[dev->time_head].end_time.tv_usec;

					dev->fc_time[head_tmp].time_flag = 0;
				}
			dev->time_head++;
			dev->time_head %= FC_TIME_CNT;
			spin_unlock(&dev->set_time_lock);
		}
	}
	if (atomic_read(&dev->wait_fc_cnt) == 0 &&
	    atomic_read(&dev->hw_flg))
		complete(&dev->bpu_completion);
}

int cnn_debugfs_remove_files(const struct cnn_debugfs_info *files, int count,
			     struct hobot_bpu_dev *dev)
{
	struct list_head *pos, *q;
	struct cnn_info_node *tmp;
	int i;

	mutex_lock(&dev->debugfs_lock);
	for (i = 0; i < count; i++) {
		list_for_each_safe(pos, q, &dev->debugfs_list) {
			tmp = list_entry(pos, struct cnn_info_node, list);
			if (tmp->info_ent == &files[i]) {
				debugfs_remove(tmp->dent);
				list_del(pos);
				kfree(tmp);
			}
		}
	}
	mutex_unlock(&dev->debugfs_lock);
	return 0;
}

int cnn_debugfs_cleanup(struct hobot_bpu_dev *dev)
{

	if (!dev->debugfs_root)
		return 0;
	cnn_debugfs_remove_files(cnn_debugfs_list, CNN_DEBUGFS_ENTRIES, dev);

	debugfs_remove_recursive(dev->debugfs_root);
	dev->debugfs_root = NULL;

	return 0;
}


int cnn_debugfs_create_files(const struct cnn_debugfs_info *files, int count,
			     struct dentry *root, struct hobot_bpu_dev *dev)
{
	struct dentry *ent;
	struct cnn_info_node *tmp;
	int i, ret;

	for (i = 0; i < count; i++) {
		tmp = kmalloc(sizeof(struct cnn_info_node), GFP_KERNEL);
		if (tmp == NULL) {
			ret = -1;
			goto fail;
		}
		ent = debugfs_create_file(files[i].name, S_IFREG | S_IRUGO,
			root, tmp, &cnn_debugfs_fops);
		if (!ent) {
			pr_err("Cannot create /sys/kernel/debug/dri/%pd/%s\n",
				root, files[i].name);
			kfree(tmp);
			ret = -1;
			goto fail;
		}

		tmp->cnn_dev = dev;
		tmp->dent = ent;
		tmp->info_ent = &files[i];

		mutex_lock(&dev->debugfs_lock);
		list_add(&tmp->list, &dev->debugfs_list);
		mutex_unlock(&dev->debugfs_lock);
	}
	return 0;

fail:
	cnn_debugfs_remove_files(files, count, dev);
	return ret;

}

int cnn_debugfs_init(struct hobot_bpu_dev *dev, int cnn_id, struct dentry *root)
{
	char name[8];
	int ret;

	INIT_LIST_HEAD(&dev->debugfs_list);
	mutex_init(&dev->debugfs_lock);
	snprintf(name, 5, "%s%d", g_chrdev_name, cnn_id);
	dev->debugfs_root = debugfs_create_dir(name, root);
	if (!dev->debugfs_root) {
		pr_err("Cannot create /sys/kernel/debug/%s\n", name);
		return -1;
	}

	ret = cnn_debugfs_create_files(cnn_debugfs_list, CNN_DEBUGFS_ENTRIES,
				       dev->debugfs_root, dev);
	if (ret) {
		debugfs_remove(dev->debugfs_root);
		dev->debugfs_root = NULL;
		pr_err("Failed to create core drm debugfs files\n");
		return ret;
	}

	return 0;
}
struct bus_type bpu_subsys = {
	.name = "bpu",
};

#ifdef CONFIG_HOBOT_CNN_DEVFREQ
static int cnnfreq_target(struct device *dev, unsigned long *freq,
				   u32 flags)
{
	struct hobot_bpu_dev *cnn_dev = dev_get_drvdata(dev);
	struct hobot_bpufreq *cnnfreq = cnn_dev->cnnfreq;
	struct dev_pm_opp *opp;
	unsigned long old_clk_rate = cnnfreq->rate;
	unsigned long rate, target_volt, target_rate;
	int err;

	if (!cnn_dev->has_regulator) {
		pr_info("no regulator support\n");
		return 1;
	}
	lock_bpu(cnn_dev);

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp)) {
		err = PTR_ERR(opp);
		goto opp_err;
	}
	rate = dev_pm_opp_get_freq(opp);
	target_volt = dev_pm_opp_get_voltage(opp);

	target_rate = clk_round_rate(cnn_dev->cnn_mclk, rate);
	if ((long)target_rate <= 0)
		target_rate = rate;

	if (soc_is_x3e() == 1) {
		if (rate > 600000000) {
			rate = 600000000;
		}
	}

	if (cnnfreq->rate == target_rate) {
		if (cnnfreq->volt == target_volt) {
			err = 0;
			goto out;
		}
		err = regulator_set_voltage(cnn_dev->cnn_regulator,
					target_volt,INT_MAX);
		if (err) {
			dev_err(dev, "Cannot set voltage %lu uV\n",
				target_volt);
			goto out;
		}
	}

	if (old_clk_rate < target_rate) {
		err = regulator_set_voltage(cnn_dev->cnn_regulator,
					target_volt, INT_MAX);
		if (err) {
			dev_err(dev, "Cannot set voltage %lu uV\n",
				target_volt);
			goto out;
		}
	}

	err = clk_set_rate(cnn_dev->cnn_mclk, target_rate);
	if (err) {
		dev_err(dev, "Cannot set frequency %lu (%d)\n",
			target_rate, err);
		regulator_set_voltage(cnn_dev->cnn_regulator,
				cnnfreq->volt, INT_MAX);
		goto out;
	}

	cnnfreq->rate = clk_get_rate(cnn_dev->cnn_mclk);

	if (cnnfreq->rate != target_rate) {
		dev_err(dev, "Get wrong frequency, Request %lu, Current %llu\n",
			target_rate, cnnfreq->rate);
		regulator_set_voltage(cnn_dev->cnn_regulator,
				cnnfreq->volt, INT_MAX);
		goto out;
	} else if (old_clk_rate > target_rate) {
		err = regulator_set_voltage(cnn_dev->cnn_regulator,
				target_volt,INT_MAX);
		if (err) {
			dev_err(dev, "Cannot set vol %lu uV\n", target_volt);
			goto out;
		}
	}
	/* Send the bpu freq change notify to listener */
	diag_send_event_stat_and_env_data(DiagMsgPrioLow,
			ModuleDiag_bpu, CNN_FREQ_CHANGE(cnn_dev->core_index),
			DiagEventStaUnknown,
			DiagGenEnvdataWhenSuccess,
			(uint8_t *)&target_rate, sizeof(target_rate));

	cnnfreq->volt = target_volt;
out:
	dev_pm_opp_put(opp);
opp_err:
	unlock_bpu(cnn_dev);
	return err;
}

static int cnnfreq_get_cur_freq(struct device *dev,
					 unsigned long *freq)
{
	struct hobot_bpu_dev *cnn_dev = dev_get_drvdata(dev);

	*freq = cnn_dev->cnnfreq->rate;

	return 0;
}

static int hobot_bpufreq_register(struct hobot_bpu_dev *cnn_dev)
{
	struct device *dev = cnn_dev->dev;
	struct hobot_bpufreq *data;
	struct devfreq_dev_profile *devp;
	const char *gov_name;
	char bpu_name[10];
	int ret;

	dev_err(dev, "%s probe\n", __func__);

	data = devm_kzalloc(dev, sizeof(struct hobot_bpufreq), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	if (dev_pm_opp_of_add_table(dev)) {
		dev_err(dev, "Invalid operating-points in device tree.\n");
		return -EINVAL;
	}

	cnn_dev->cnnfreq = data;
	devp = &data->devp;
	devp->polling_ms	= 0;
	devp->target		= cnnfreq_target;
	devp->get_cur_freq	= cnnfreq_get_cur_freq;
	devp->initial_freq = clk_get_rate(cnn_dev->cnn_mclk);

	data->rate = devp->initial_freq;
	data->volt = regulator_get_voltage(cnn_dev->cnn_regulator);

	if (of_property_read_string(dev->of_node, "governor", &gov_name))
		gov_name = "userspace";

	data->devfreq = devm_devfreq_add_device(dev, devp, gov_name, NULL);
	if (IS_ERR(data->devfreq))
		return PTR_ERR(data->devfreq);

	data->devfreq->min_freq = devp->freq_table[0];
	data->devfreq->max_freq = devp->freq_table[devp->max_state ?
						devp->max_state - 1 : 0];

	devm_devfreq_register_opp_notifier(dev, data->devfreq);

	data->cooling = of_devfreq_cooling_register(dev->of_node,
							data->devfreq);

	sprintf(bpu_name, "bpu%d", cnn_dev->core_index);
	ret = sysfs_add_link_to_group(&bpu_subsys.dev_root->kobj,
				      bpu_name,
				      &data->devfreq->dev.kobj,
				      "bpufreq");
	if (ret)
		pr_err("add bpu freq link error\n");
	dev_err(dev, "%s end\n", __func__);
	return 0;
}
#endif
#ifdef CHECK_IRQ_LOST
static void hobot_bpu_simu_irq(struct hobot_bpu_dev *dev)
{
	int ret = 0;
	struct bpu_int_info tmp;
	struct cnn_user_info **p_user_info;
	unsigned long flags;

	ret = kfifo_out_spinlocked(&dev->int_info_fifo, &tmp,
					sizeof(struct bpu_int_info),
					&dev->kfifo_lock);
	if (ret == sizeof(struct bpu_int_info)) {
		spin_lock_irqsave(&dev->cnn_spin_lock, flags);
		p_user_info = tmp.p_user_info;
		if (!(*p_user_info) || !p_user_info) {
			spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
			return;
		}
		(*p_user_info)->cnn_int_num.cnn_int_num[(*p_user_info)->cnn_int_num.cnn_int_count] = tmp.int_num;
		do_gettimeofday(&tmp.end_time);
		(*p_user_info)->cnn_int_num.cnn_int_interval[(*p_user_info)->cnn_int_num.cnn_int_count] =
			(tmp.end_time.tv_sec * 1000000 + tmp.end_time.tv_usec)
			- (tmp.start_time.tv_sec * 1000000 + tmp.start_time.tv_usec);
		if ((*p_user_info)->cnn_int_num.cnn_int_count < CNN_INT_NUM - 1) {
			(*p_user_info)->cnn_int_num.cnn_int_count++;
			dev->real_int_cnt++;
		}

		(*p_user_info)->irq_triggered = 1;
		wake_up_interruptible(&(*p_user_info)->cnn_int_wait);
		spin_unlock_irqrestore(&dev->cnn_spin_lock, flags);
		atomic_sub(tmp.fc_total, &dev->wait_fc_cnt);
		tasklet_schedule(&dev->tasklet);
	} else
		pr_err("%s:interrupt info fifo is empty\n", __func__);
}

static int hobot_bpu_recover(struct hobot_bpu_dev *dev, int head, int tail)
{
	int head_tmp = 0;
	int tail_tmp = 0;
	int fc_depth = 0;
	void *tmp_trans_buf;
	int tmp_trans_fc_num = 0;

	head_tmp = head & CNN_MAX_FC_LEN_MASK;
	tail_tmp = tail & CNN_MAX_FC_LEN_MASK;

	fc_depth = hobot_bpu_reg_read(dev, CNN_FC_LEN);

	if (tail_tmp > head_tmp) {
		tmp_trans_fc_num = tail_tmp - head_tmp;
	} else if (tail_tmp < head_tmp)
		tmp_trans_fc_num = fc_depth + 1 + tail_tmp - head_tmp;
	else {
		if (kfifo_len(&dev->int_info_fifo))
			tmp_trans_fc_num = 1;
	}

	if (!tmp_trans_fc_num)
		return 0;

	tmp_trans_buf = kzalloc(tmp_trans_fc_num * CNN_FC_SIZE, GFP_KERNEL);
	if (!tmp_trans_buf) {
		pr_err("cnn core(%d), recover error\n", dev->core_index);
		return -ENOMEM;
	}

	if (tail_tmp >= head_tmp)
		memcpy(tmp_trans_buf, dev->fc_base + head_tmp* CNN_FC_SIZE,
			tmp_trans_fc_num * CNN_FC_SIZE);
	else {
		memcpy(tmp_trans_buf, dev->fc_base + head_tmp* CNN_FC_SIZE,
			(fc_depth + 1 - head_tmp) * CNN_FC_SIZE);
		memcpy(tmp_trans_buf + (fc_depth + 1 - head_tmp) * CNN_FC_SIZE,
				dev->fc_base,
			(tail_tmp + 1) * CNN_FC_SIZE);
	}

	hobot_bpu_hw_reset_reinit(dev, dev->core_index);

	memcpy(dev->fc_base, tmp_trans_buf,
			tmp_trans_fc_num * CNN_FC_SIZE);

	hobot_bpu_set_fc_tail_idx(dev, tmp_trans_fc_num);

	kfree(tmp_trans_buf);

	return 0;
}

static int hobot_bpu_maintain_thread(void *data)
{
	struct hobot_bpu_dev *dev = (struct hobot_bpu_dev *)data;
	struct bpu_int_info tmp;
	int head_tmp, tail_tmp;
	int last_peek_hw_id = -1;
	int fifo_len;
	int recove_len;
	int fc_depth;
	int check_time = 1000;
	int ret;

	fc_depth = hobot_bpu_reg_read(dev, CNN_FC_LEN);
	while (!kthread_should_stop()) {
		if (maintain_pre_time >= MAINT_PREP_START_TIMES)
			/* use the 5 * max fc process time to check if bpu hung */
			check_time = max_check_interval / 1000 * 5;
#ifdef CONFIG_HOBOT_CNN_DEVFREQ
		/* if the bpu clock is not highest, accroding the highest */
		check_time *= dev->cnnfreq->devfreq->max_freq
			/ clk_get_rate(dev->cnn_mclk);
#endif
		msleep(check_time);

		mutex_lock(&dev->cnn_lock);
		ret = kfifo_out_peek(&dev->int_info_fifo, &tmp, sizeof(struct bpu_int_info));
		if (ret) {
			if (last_peek_hw_id < 0) {
				last_peek_hw_id = tmp.hw_id;
				mutex_unlock(&dev->cnn_lock);
				continue;
			}
		}
		/* check wether bpu hung */
		head_tmp = hobot_bpu_reg_read(dev, CNN_FC_HEAD);
		tail_tmp = hobot_bpu_reg_read(dev, CNN_FC_TAIL);
		fifo_len = kfifo_len(&dev->int_info_fifo) / sizeof(struct bpu_int_info);
		if ((dev->maintain_head == head_tmp) && (last_peek_hw_id == tmp.hw_id)) {
			/* check debug reg */
			if ((head_tmp == tail_tmp) && (!fifo_len)) {
				/* no data to process */
				mutex_unlock(&dev->cnn_lock);
				continue;
			}

			if (head_tmp >= tail_tmp)
				recove_len = max(head_tmp - tail_tmp, fifo_len);
			else
				recove_len = max(head_tmp + fc_depth + 1 - tail_tmp, fifo_len);

			if (tail_tmp - recove_len >= 0)
				head_tmp = tail_tmp - recove_len;
			else
				head_tmp = tail_tmp + fc_depth + 1 - recove_len;
		} else {
			if (last_peek_hw_id != tmp.hw_id)
				last_peek_hw_id = tmp.hw_id;
			if (dev->maintain_head != head_tmp)
				dev->maintain_head = head_tmp;
			mutex_unlock(&dev->cnn_lock);
			continue;
		}

		pr_err("bpu core[%d] may hung, try to recover[%d, %d]\n",
				dev->core_index, head_tmp, tail_tmp);
		if (maintain_pre_time < MAINT_PREP_START_TIMES)
			maintain_pre_time = MAINT_PREP_START_TIMES;
		ret = hobot_bpu_recover(dev, head_tmp, tail_tmp);
		if (ret < 0) {
			pr_err("cnn core %d recover failed\n", dev->core_index);
		}
		mutex_unlock(&dev->cnn_lock);
	}

	return 0;
}

static void hobot_check_cnn(unsigned long arg)
{
	struct hobot_bpu_dev *dev = (struct hobot_bpu_dev *)arg;
	int head_tmp = 0;
	int inst_num_tmp = 0;
	int ret = 0;

	pr_debug("%s:[%d]\n", __func__, __LINE__);
	head_tmp = hobot_bpu_reg_read(dev, CNN_FC_HEAD);
	inst_num_tmp = hobot_bpu_reg_read(dev, CNNINT_INST_NUM);
	ret = kfifo_len(&dev->int_info_fifo);
	if (ret) {
		if (dev->head_value == head_tmp &&
			dev->inst_num == inst_num_tmp &&
			dev->inst_num != 0) {
			pr_err("bpu lost irq!!!\n");
			pr_err("head:%d,inst_num:%d,head_tmp:%d,inst_tmp:%d,id:%d,tail:%d,ret:%d\n",
				dev->head_value,
				dev->inst_num,
				head_tmp,
				inst_num_tmp,
				dev->core_index,
				hobot_bpu_reg_read(dev, CNN_FC_TAIL),
				ret);
			hobot_bpu_simu_irq(dev);
		}
	}
	dev->head_value = head_tmp;
	dev->inst_num = inst_num_tmp;
	dev->cnn_timer.expires = jiffies + HZ * 5;
	add_timer(&dev->cnn_timer);

}
#endif

static void bpu_diag_test(void *p, size_t len)
{
	bpu_err_flag = 1;
}
int hobot_bpu_probe(struct platform_device *pdev)
{

	int rc;
	struct hobot_bpu_dev *cnn_dev;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	struct resource *pmu;
	int cnn_id;
	char dev_name[8];

	cnn_dev = devm_kzalloc(&pdev->dev, sizeof(*cnn_dev), GFP_KERNEL);
	if (!cnn_dev)
		return -ENOMEM;
	cnn_dev->dev = &pdev->dev;
	cnn_dev->fc_time = devm_kzalloc(&pdev->dev,
					sizeof(struct hobot_fc_time) * FC_TIME_CNT,
					GFP_KERNEL);
	if (!cnn_dev->fc_time)
		return -ENOMEM;
	cnn_dev->time_head = 0;
	cnn_dev->time_tail = 0;
	memset(cnn_dev->fc_time, 0, sizeof(struct hobot_fc_time) * FC_TIME_CNT);
	rc = of_property_read_u32(np, "cnn-id", &cnn_id);
	if (rc < 0) {
		dev_err(cnn_dev->dev, "missing cnn-id property\n");
		goto err_out;
	}

	if ((cnn_id != 0) && (cnn_id != 1)) {
		pr_err("Invalid cnn id\n");
		rc = -1;
		goto err_out;
	}


	if (cnn_id == 0)
		cnn0_dev = cnn_dev;
	else
		cnn1_dev = cnn_dev;
	rc = cnn_debugfs_init(cnn_dev, cnn_id, cnn_dev->cnn_debugfs_root);
	if (rc)
		pr_err("init cnn%d debugfs failed\n", cnn_id);
#ifndef CONFIG_HOBOT_FPGA_X3
	/*get regulator*/
	cnn_dev->cnn_regulator = devm_regulator_get(cnn_dev->dev, "cnn");
	if (IS_ERR(cnn_dev->cnn_regulator)) {
		pr_info("get regulator error, no regulator support\n");
		cnn_dev->has_regulator = 0;
	} else {
		cnn_dev->has_regulator = 1;
		cnn_dev->disable_bpu = BPU_REGU_DIS;
	}
	cnn_dev->disable_bpu |= BPU_CLOCK_DIS;
	/*get cnn clock and prepare*/
	cnn_dev->cnn_aclk = devm_clk_get(cnn_dev->dev, "cnn_aclk");
	if (IS_ERR(cnn_dev->cnn_aclk)) {
		pr_info("get cnn0 aclock err\n");
		goto err_out;
	}
	cnn_dev->cnn_mclk = devm_clk_get(cnn_dev->dev, "cnn_mclk");
	if (IS_ERR(cnn_dev->cnn_mclk)) {
		pr_info("get cnn0 mclock err\n");
		goto err_out;
	}
	rc = clk_prepare(cnn_dev->cnn_aclk);
	if(rc) {
		pr_err("%s:%d prepare bpu aclk error\n", __func__, __LINE__);
		goto err_out;
	}

	rc = clk_prepare(cnn_dev->cnn_mclk);
	if(rc) {
		pr_err("%s:%d prepare bpu mclk error\n", __func__, __LINE__);
		goto err_out;
	}
#endif
	pmu = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!pmu) {
		rc = -ENODEV;
		goto err_out;
	}
	cnn_dev->cnn_pmu = devm_ioremap(&pdev->dev, pmu->start, 4);
	if (IS_ERR(cnn_dev->cnn_pmu)) {
		rc = PTR_ERR(cnn_dev->cnn_pmu);
		pr_err("%s:%d err_out get cnn_pmu failed\n",
				__func__, __LINE__);
		goto err_out;
	}
	rc = of_property_read_u32(np, "iso-bit", &cnn_dev->iso_bit);
	if (rc < 0) {
		dev_err(cnn_dev->dev, "missing iso-bit property\n");
		goto err_out;
	}
	rc = hobot_bpu_get_resets(cnn_dev);
	if (rc < 0) {
		pr_err("failed get cnn%d resets\n", cnn_id);
		goto err_out;
	}

	of_property_read_u32(np, "busy_check", &has_busy_status);
	cnn_dev->irq = irq_of_parse_and_map(np, 0);
	if (cnn_dev->irq < 0) {
		dev_err(&pdev->dev, "no cnn irq found\n");
		rc = -1;
		goto err_out;
	}
	/* get cnn controller base addr */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res) {
		rc = -ENODEV;
		goto err_out;
	}
	cnn_dev->cnn_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(cnn_dev->cnn_base)) {
		rc = PTR_ERR(cnn_dev->cnn_base);
		pr_err("%s:%d err_out get cnn_base failed\n",
				__func__, __LINE__);
		goto err_out;
	}

	rc = request_irq(cnn_dev->irq, hobot_bpu_interrupt_handler, 0,
			CNN_DRV_NAME, cnn_dev);
	if (rc) {
		dev_err(cnn_dev->dev, "request_irq '%d' failed with %d\n",
			cnn_dev->irq, rc);
		goto err_out;
	}

	cnn_dev->core_index = cnn_id;
	cnn_dev->fc_base = dma_alloc_coherent(&pdev->dev,
			CNN_FC_SPACE_LEN, &cnn_dev->fc_phys_base, GFP_KERNEL);
	if (cnn_dev->fc_base == NULL) {
		dev_err(&pdev->dev, "bpu core alloc fc mem failed\n");
		rc = -1;
		goto err_out;
	}
	cnn_dev->fc_mem_size  = CNN_FC_SPACE_LEN;

	atomic_set(&cnn_dev->hw_id_counter, 1);
	atomic_set(&cnn_dev->wait_fc_cnt, 0);
	atomic_set(&cnn_dev->hw_flg, 0);
	cnn_dev->zero_int_cnt = 0;
	cnn_dev->real_int_cnt = 0;
	cnn_dev->wait_nega_flag = 0;

	pr_info("bpu%d fc phy base = 0x%llx, len = 0x%x, default fc len = 0x%x\n",
			cnn_dev->core_index,
			cnn_dev->fc_phys_base,
			cnn_dev->fc_mem_size,
			(cnn_dev->fc_mem_size / 64) - 1);

	mutex_init(&cnn_dev->cnn_lock);
	spin_lock_init(&cnn_dev->set_time_lock);
	spin_lock_init(&cnn_dev->cnn_spin_lock);
	spin_lock_init(&cnn_dev->kfifo_lock);

	rc = kfifo_alloc(&cnn_dev->int_info_fifo,
		    sizeof(struct bpu_int_info) * ((cnn_dev->fc_mem_size / 64) - 1), GFP_KERNEL);
	if (rc < 0) {
		pr_err("kfifo alloc error\n");
		goto err_out;
	}

	/* Create the chardev for cnn0 and cnn1 */
	cnn_dev->chrdev_name = dev_name;
	snprintf(cnn_dev->chrdev_name, sizeof(dev_name),
			"%s%d", g_chrdev_name, cnn_id);
	cnn_dev->minor_num = MINOR_NUMBER;
	cnn_dev->num_devices = NUM_DEVICES;
	rc = hobot_bpu_init_chrdev(cnn_dev);

	if (rc < 0) {
		dev_err(&pdev->dev,
			"Failed create char dev for cnn%d\n", cnn_id);
		goto err_out;
	}

	/* Initialize the tasklet */
	tasklet_init(&cnn_dev->tasklet, hobot_bpu_do_tasklet,
			(unsigned long)cnn_dev);

	platform_set_drvdata(pdev, cnn_dev);

	/*
	 * init bpu completion and negative completion
	 * for lock_bpu
	 */
	init_completion(&cnn_dev->bpu_completion);
	init_completion(&cnn_dev->nega_completion);

	/* diag ref init */
	if ((EventIdBpu0Err + cnn_id) <= EventIdBpu1Err) {
		if (diag_register(ModuleDiag_bpu, EventIdBpu0Err + cnn_id, 5, 300,
					7000, bpu_diag_test) < 0)
			dev_err(&pdev->dev, "bpu%d diag register fail\n", cnn_id);
	} else
			dev_err(&pdev->dev, "bpu event id overun: max = 2,but now is:%d\n",
					EventIdBpu0Err + cnn_id);

	if (diag_register(ModuleDiag_bpu,
			  CNN_FREQ_CHANGE(cnn_id), 16, 300, 5000, NULL) < 0)
		dev_err(&pdev->dev, "bpu%d freq notify register fail\n", cnn_id);
	pr_info("hobot bpu%d probe OK!!\n", cnn_id);
#ifdef CONFIG_HOBOT_CNN_DEVFREQ
	if (cnn_dev->has_regulator)
		hobot_bpufreq_register(cnn_dev);
#endif

#ifdef CHECK_IRQ_LOST
	init_timer(&cnn_dev->cnn_timer);
	cnn_dev->cnn_timer.function = &hobot_check_cnn;
	cnn_dev->cnn_timer.data = (unsigned long)cnn_dev;
#endif
	return rc;

err_out:
	devm_kfree(&pdev->dev, cnn_dev);
	return rc;
return 0;
}
void cnn_fc_time_kfifo_clean(struct hobot_bpu_dev *dev)
{
	kfifo_free(&dev->int_info_fifo);
}
static void cnn_regulator_remove(struct hobot_bpu_dev *dev)
{
	if (dev->has_regulator && !dev->disable_bpu)
		hobot_bpu_power_down(dev);
	clk_unprepare(dev->cnn_mclk);
	clk_unprepare(dev->cnn_aclk);

}
#ifdef CONFIG_HOBOT_CNN_DEVFREQ
static void cnn_devfreq_remove(struct hobot_bpu_dev *dev)
{
	devfreq_cooling_unregister(dev->cnnfreq->cooling);
	//devm_devfreq_dev_release(dev->dev, dev->cnnfreq->devfreq);
	devm_devfreq_unregister_opp_notifier(dev->dev, dev->cnnfreq->devfreq);
	dev_pm_opp_of_remove_table(dev->dev);
}
#endif
static void cnn_dev_remove(struct hobot_bpu_dev *dev)
{
	cdev_del(&dev->i_cdev);
	device_destroy(dev->dev_class, dev->dev_num);
	class_destroy(dev->dev_class);
	unregister_chrdev_region(dev->dev_num, dev->num_devices);
}

static int hobot_bpu_remove(struct platform_device *pdev)
{

	u32 cnn_int_mask;
	struct hobot_bpu_dev *dev = platform_get_drvdata(pdev);
	cnn_int_mask = hobot_bpu_reg_read(dev, CNNINT_MASK);
	cnn_int_mask |= CNN_PE0_INT_MASK(1);
	hobot_bpu_reg_write(dev, CNNINT_MASK, cnn_int_mask);
	if (profiler_enable)
		del_timer(&check_timer);
	tasklet_kill(&dev->tasklet);

	dma_free_coherent(&pdev->dev, CNN_FC_SPACE_LEN,
			dev->fc_base, dev->fc_phys_base);

	dev->cnn_base = 0;
	cnn_debugfs_cleanup(dev);
	cnn_fc_time_kfifo_clean(dev);

#ifdef CONFIG_HOBOT_CNN_DEVFREQ
	cnn_devfreq_remove(dev);
#endif
	cnn_regulator_remove(dev);
	free_irq(dev->irq, dev);
	devm_kfree(dev->dev, dev->fc_time);
	cnn_dev_remove(dev);
	devm_kfree(dev->dev, dev);

	return 0;
}

static const struct of_device_id hobot_bpu_of_match[] = {
	{ .compatible = "hobot,hobot-bpu",},
	{}
};
static int hobot_bpu_drvsuspend(struct device *dev)
{
	struct hobot_bpu_dev *cnn_dev = dev_get_drvdata(dev);
	unsigned int tmp;
	pr_info("%s!!\n", __func__);
	lock_bpu(cnn_dev);
	if (__clk_is_enabled(cnn_dev->cnn_aclk))
		clk_disable(cnn_dev->cnn_aclk);
	if (__clk_is_enabled(cnn_dev->cnn_mclk))
		clk_disable(cnn_dev->cnn_mclk);
	tmp = readl(cnn_dev->cnn_pmu);

	tmp |= (1 << cnn_dev->iso_bit);
	writel(tmp, cnn_dev->cnn_pmu);
	udelay(5);

	hobot_bpu_reset_assert(cnn_dev->cnn_rst);
	if (regulator_is_enabled(cnn_dev->cnn_regulator))
		regulator_disable(cnn_dev->cnn_regulator);
	unlock_bpu(cnn_dev);
	return 0;
}

static int hobot_bpu_drvresume(struct device *dev)
{
	struct hobot_bpu_dev *cnn_dev = dev_get_drvdata(dev);
	int ret;
	unsigned int tmp;

	pr_info("%s!!\n", __func__);

	tmp = readl(cnn_dev->cnn_pmu);

	tmp |= (1 << cnn_dev->iso_bit);
	writel(tmp, cnn_dev->cnn_pmu);
	udelay(5);

	hobot_bpu_reset_assert(cnn_dev->cnn_rst);
	if (!(cnn_dev->disable_bpu & BPU_REGU_DIS)) {
		ret = regulator_enable(cnn_dev->cnn_regulator);
		if (ret != 0)
			dev_err(cnn_dev->dev, "regulator enable error\n");
		tmp = readl(cnn_dev->cnn_pmu);

		tmp &= ~(1 << cnn_dev->iso_bit);
		writel(tmp, cnn_dev->cnn_pmu);
		udelay(5);

		hobot_bpu_reset_release(cnn_dev->cnn_rst);
		if (!__clk_is_enabled(cnn_dev->cnn_aclk))
			clk_enable(cnn_dev->cnn_aclk);
		if (!__clk_is_enabled(cnn_dev->cnn_mclk))
			clk_enable(cnn_dev->cnn_mclk);
		hobot_bpu_hw_init(cnn_dev);
		hobot_bpu_set_fc_base(cnn_dev);
		hobot_bpu_set_default_fc_depth(cnn_dev, 1023);

		if (cnn_dev->disable_bpu & BPU_CLOCK_DIS)
			hobot_bpu_clock_down(cnn_dev);
	} else {
		if (!(cnn_dev->disable_bpu & BPU_CLOCK_DIS))
			hobot_bpu_clock_up(cnn_dev);
	}
	return 0;
}

static const struct dev_pm_ops hobot_bpu_pmops = {
	.suspend        = hobot_bpu_drvsuspend,
	.resume         = hobot_bpu_drvresume,
};
#define CNN_PMOPS (&hobot_bpu_pmops)

static struct platform_driver hobot_bpu_platform_driver = {
	.probe	 = hobot_bpu_probe,
	.remove  = hobot_bpu_remove,
	.driver  = {
		.name = CNN_DRV_NAME,
		.of_match_table = hobot_bpu_of_match,
		.pm     = CNN_PMOPS,
	},
};
static void hobot_bpu_check_func(unsigned long arg)
{
	static int time;
	static int cnn0_busy;
	static int cnn1_busy;
	static int fre;
	static int fre_tmp;
	static int nseconds_tmp = 1;
	int bpu0_status = 0;
	int bpu1_status = 0;
	int cnn0_head = 0;
	int cnn0_tail = 0;
	int cnn1_head = 0;
	int cnn1_tail = 0;

	fre_tmp = profiler_frequency;
	if (fre != fre_tmp || nseconds_tmp != profiler_n_seconds) {
		/* frequency changed*/
		nseconds_tmp = profiler_n_seconds;
		fre = fre_tmp;
		cnn0_busy = 0;
		cnn1_busy = 0;
		time = 0;

	}
	if (has_busy_status) {
		bpu0_status = hobot_bpu_reg_read(cnn0_dev, CNN_BUSY_STATUS);
		bpu1_status = hobot_bpu_reg_read(cnn1_dev, CNN_BUSY_STATUS);
		if (bpu0_status)
			cnn0_busy++;
		if (bpu1_status)
			cnn1_busy++;
	} else {
		cnn0_head = hobot_bpu_reg_read(cnn0_dev, CNN_FC_HEAD);
		cnn0_tail = hobot_bpu_reg_read(cnn0_dev, CNN_FC_TAIL);
		cnn1_head = hobot_bpu_reg_read(cnn1_dev, CNN_FC_HEAD);
		cnn1_tail = hobot_bpu_reg_read(cnn1_dev, CNN_FC_TAIL);
		if (cnn0_head != cnn0_tail ||
		    (atomic_read(&cnn0_dev->wait_fc_cnt) > 0))
			cnn0_busy++;
		if (cnn1_head != cnn1_tail ||
		    (atomic_read(&cnn1_dev->wait_fc_cnt) > 0))
			cnn1_busy++;
	}
	if (++time == fre * nseconds_tmp) {
		time = 0;
		ratio0 = cnn0_busy * 100 / fre / nseconds_tmp;
		ratio1 = cnn1_busy * 100 / fre / nseconds_tmp;
		cnn0_busy = 0;
		cnn1_busy = 0;
	}
	check_timer.expires = jiffies + HZ / fre;
	add_timer(&check_timer);
}

static ssize_t fre_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", profiler_frequency);
}


static ssize_t fre_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	int ret;

	ret = sscanf(buf, "%du", &profiler_frequency);
	if (ret < 0) {
		pr_info("%s sscanf error\n", __func__);
		return 0;
	}
	pr_info("func:%s, FRE:%d\n", __func__, profiler_frequency);
	if (profiler_frequency <= 0 || profiler_frequency > HZ) {
		pr_warn("err!, profiler frequency range is 0~%d HZ\n", HZ);
		return -1;
	}
	if (!profiler_enable)
		check_timer.expires = jiffies + HZ / profiler_frequency;
	return count;
}

static ssize_t burst_len_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", burst_length * 16);
}

static ssize_t nseconds_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", profiler_n_seconds);
}

static ssize_t nseconds_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	int ret;

	ret = sscanf(buf, "%du", &profiler_n_seconds);
	if (ret < 0) {
		pr_info("%s sscanf error\n", __func__);
		return 0;
	}
	return count;
}

static ssize_t hotplug_ext_flag_show(struct kobject *kobj, struct kobj_attribute *attr,
 char *buf)
{
	return sprintf(buf, "%d\n", hotplug_ext_flag);
}

static ssize_t hotplug_ext_flag_store(struct kobject *kobj, struct kobj_attribute *attr,
		  const char *buf, size_t count)
{
	int ret;

	ret = sscanf(buf, "%du", &hotplug_ext_flag);
	if (ret < 0) {
		pr_info("%s sscanf error\n", __func__);
		return 0;
	}
	return count;
}

static ssize_t burst_len_store(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	int tmp_val;

	ret = sscanf(buf, "%du", &tmp_val);
	if (ret < 0) {
		pr_info("%s sscanf error\n", __func__);
		return 0;
	}

	if (tmp_val % 16 > 0) {
		pr_err("burst len must align 16");
	}

	burst_length = tmp_val / 16;

	if (burst_length <= 0)
		burst_length = 1;
	else if (burst_length > 0x80)
		burst_length = 0x80;

	pr_info("func:%s, set burst len:%d\n", __func__, burst_length);

	return count;
}

static ssize_t enable_show(struct kobject *kobj, struct kobj_attribute *attr,
						 char *buf)
{
	return sprintf(buf, "%d\n", profiler_enable);
}

static ssize_t fc_time_enable_show(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   char *buf)
{
	return sprintf(buf, "%d\n", fc_time_enable);
}

static ssize_t fc_time_enable_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	int tmp;

	sscanf(buf, "%du", &tmp);
	if (tmp == fc_time_enable)
		return count;
	lock_bpu(cnn0_dev);
	lock_bpu(cnn1_dev);

	memset(cnn0_dev->fc_time, 0,
		   sizeof(struct hobot_fc_time) * FC_TIME_CNT);
	memset(cnn1_dev->fc_time, 0,
		   sizeof(struct hobot_fc_time) * FC_TIME_CNT);
	cnn0_dev->time_head = 0;
	cnn0_dev->time_tail = 0;
	cnn1_dev->time_head = 0;
	cnn1_dev->time_tail = 0;
	fc_time_enable = tmp;
	unlock_bpu(cnn0_dev);
	unlock_bpu(cnn1_dev);
	return count;
}

static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
						  const char *buf, size_t count)
{
	int ret;
	static int enable_tmp;

	mutex_lock(&enable_lock);
	ret = sscanf(buf, "%du", &profiler_enable);
	mutex_unlock(&enable_lock);
	if (ret < 0) {
		pr_info("%s sscanf error\n", __func__);
		return 0;
	}
	pr_info("set bpu profiler enable:%d\n", profiler_enable);
	if (profiler_enable) {
		if (!profiler_frequency) {
			pr_warn("cnn profiler frequency is 0, please set frequency!\n");
			profiler_enable = 0;
			return -1;
		}
		mutex_lock(&enable_lock);
		if (!enable_tmp) {
			add_timer(&check_timer);
			enable_tmp = profiler_enable;
		}
		mutex_unlock(&enable_lock);
	} else {
		mutex_lock(&enable_lock);
		if (enable_tmp) {
			del_timer(&check_timer);
			enable_tmp = profiler_enable;
		}
		mutex_unlock(&enable_lock);
	}
	return count;
}


static ssize_t ratio0_show(struct kobject *kobj, struct kobj_attribute *attr,
						 char *buf)
{
	return sprintf(buf, "%d\n", ratio0);
}

static ssize_t ratio1_show(struct kobject *kobj, struct kobj_attribute *attr,
						 char *buf)
{
	return sprintf(buf, "%d\n", ratio1);
}
static ssize_t queue0_show(struct kobject *kobj, struct kobj_attribute *attr,
						 char *buf)
{
	mutex_lock(&cnn0_dev->cnn_lock);
	if (cnn0_dev->disable_bpu)
		queue0 = 0;
	else
		queue0 = hobot_bpu_get_fc_fifo_spaces(cnn0_dev);
	mutex_unlock(&cnn0_dev->cnn_lock);
	return sprintf(buf, "%d\n", queue0);
}
static ssize_t queue1_show(struct kobject *kobj, struct kobj_attribute *attr,
						 char *buf)
{
	mutex_lock(&cnn1_dev->cnn_lock);
	if (cnn1_dev->disable_bpu)
		queue1 = 0;
	else
		queue1 = hobot_bpu_get_fc_fifo_spaces(cnn1_dev);
	mutex_unlock(&cnn1_dev->cnn_lock);
	return sprintf(buf, "%d\n", queue1);
}

static ssize_t fc_time_show(struct hobot_bpu_dev *dev, char *buf)
{
	int ret = 0;
	int sum = 0;
	int head, tail;
	unsigned long flags;
	int elapse_time = 0;
	char buf_start[24] = {0};
	char buf_end[24] = {0};
	int cnt = 0;
	struct hobot_fc_time *tmp = vmalloc(sizeof(struct hobot_fc_time) * FC_TIME_CNT);

	if (!fc_time_enable)
		return sprintf(buf, "Please enable get fc time feature\n");

	spin_lock_irqsave(&dev->set_time_lock, flags);
	memcpy(tmp, dev->fc_time, sizeof(struct hobot_fc_time) * FC_TIME_CNT);
	tail = dev->time_head;
	spin_unlock_irqrestore(&dev->set_time_lock, flags);

	if (tmp[FC_TIME_CNT - 1].start_time.tv_sec == 0)
		head = 0;
	else {
		head = tail + 3;
		head %= FC_TIME_CNT;
	}

	ret = sprintf(buf, "%-6s%-17s\t%-17s\t%-17s\t%s\n",
					"num",
					"intterupt",
					"start_time",
					"end_time",
					"exe_time");
	sum += ret;

	do {
		memset(buf_start, 0, 24);
		memset(buf_end, 0, 24);
		sprintf(buf_start, "%lds %ldus",
			tmp[head].start_time.tv_sec,
			tmp[head].start_time.tv_usec);
		sprintf(buf_end, "%lds %ldus",
			tmp[head].end_time.tv_sec,
			tmp[head].end_time.tv_usec);

		elapse_time = tmp[head].end_time.tv_sec * 1000 +
			      tmp[head].end_time.tv_usec / 1000 -
			      tmp[head].start_time.tv_sec * 1000 -
			      tmp[head].start_time.tv_usec / 1000;
		ret = sprintf(buf + sum, "%-6d%-17d\t%-17s\t%-17s\t%4dms\n",
				tmp[head].fc_count, tmp[head].int_num,
				buf_start,
				buf_end,
				elapse_time);
		sum += ret;
		head++;
		head %= FC_TIME_CNT;
	} while (head != tail && (++cnt < 50));
	vfree(tmp);
	return sum;
}

static ssize_t bpu0_fc_time_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	int ret;

	ret = fc_time_show(cnn0_dev, buf);
	return ret;
}
static ssize_t bpu1_fc_time_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	int ret;

	ret = fc_time_show(cnn1_dev, buf);
	return ret;

}

static ssize_t bpu0_clock_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	int ret_aclk = __clk_is_enabled(cnn0_dev->cnn_aclk);
	int ret_mclk = __clk_is_enabled(cnn0_dev->cnn_mclk);

	return sprintf(buf, "%d\n", (ret_aclk && ret_mclk));
}
static ssize_t bpu0_clock_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int ret;

	ret = sscanf(buf, "%du", &bpu0_clk);
	if (bpu0_clk) {
		hobot_bpu_clock_up(cnn0_dev);
	} else {
		hobot_bpu_clock_down(cnn0_dev);
	}
	return count;

}

static ssize_t bpu1_clock_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	int ret_aclk = __clk_is_enabled(cnn1_dev->cnn_aclk);
	int ret_mclk = __clk_is_enabled(cnn1_dev->cnn_mclk);

	return sprintf(buf, "%d\n", (ret_aclk && ret_mclk));
}
static ssize_t bpu1_clock_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int ret;

	ret = sscanf(buf, "%du", &bpu1_clk);
	if (bpu1_clk) {
		hobot_bpu_clock_up(cnn1_dev);
	} else {
		hobot_bpu_clock_down(cnn1_dev);
	}
	return count;
}
static ssize_t bpu0_power_show(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       char *buf)
{
	int ret;

	if (!cnn0_dev->has_regulator) {
		pr_info("no regulator support\n");
		ret = -1;
		return sprintf(buf, "%d\n", ret);
	}
	ret = regulator_is_enabled(cnn0_dev->cnn_regulator);

	return sprintf(buf, "%d\n", ret);

}
static ssize_t bpu0_power_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int ret;

	if (!cnn0_dev->has_regulator) {
		pr_info("no regulator support\n");
		return count;
	}
	ret = sscanf(buf, "%du", &bpu0_power);
	if (bpu0_power)
		hobot_bpu_power_up(cnn0_dev);
	else
		hobot_bpu_power_down(cnn0_dev);
	return count;
}

static ssize_t bpu1_power_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	int ret;

	if (!cnn1_dev->has_regulator) {
		pr_info("no regulator support\n");
		ret = -1;
		return sprintf(buf, "%d\n", ret);
	}
	ret = regulator_is_enabled(cnn1_dev->cnn_regulator);

	return sprintf(buf, "%d\n", ret);

}
static ssize_t bpu1_power_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int ret;

	if (!cnn1_dev->has_regulator) {
		pr_info("no regulator support\n");
		return count;
	}
	ret = sscanf(buf, "%du", &bpu1_power);
	if (bpu1_power)
		hobot_bpu_power_up(cnn1_dev);
	else
		hobot_bpu_power_down(cnn1_dev);
	return count;
}

static ssize_t bpu0_hotplug_en_show(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   char *buf)
{
	return sprintf(buf, "%d\n", bpu0_hotplug);
}
static ssize_t bpu0_hotplug_en_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int ret;

	if (!cnn0_dev->has_regulator) {
		pr_info("no regulator support\n");
		return count;
	}
	ret = sscanf(buf, "%du", &bpu0_hotplug);
	if (bpu0_hotplug) {
		mutex_lock(&cnn0_dev->cnn_lock);
		cnn0_dev->bpu_detached = 0;
		mutex_unlock(&cnn0_dev->cnn_lock);
		hobot_bpu_power_up(cnn0_dev);
	} else {
		mutex_lock(&cnn0_dev->cnn_lock);
		cnn0_dev->bpu_detached = 1;
		mutex_unlock(&cnn0_dev->cnn_lock);
		hobot_bpu_power_down(cnn0_dev);
	}
	return count;
}

static ssize_t bpu1_hotplug_en_show(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   char *buf)
{
	return sprintf(buf, "%d\n", bpu1_hotplug);
}
static ssize_t bpu1_hotplug_en_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int ret;

	if (!cnn1_dev->has_regulator) {
		pr_info("no regulator support\n");
		return count;
	}
	ret = sscanf(buf, "%du", &bpu1_hotplug);
	if (bpu1_hotplug) {
		mutex_lock(&cnn1_dev->cnn_lock);
		cnn1_dev->bpu_detached = 0;
		mutex_unlock(&cnn1_dev->cnn_lock);
		hobot_bpu_power_up(cnn1_dev);
	} else {
		mutex_lock(&cnn1_dev->cnn_lock);
		cnn1_dev->bpu_detached = 1;
		mutex_unlock(&cnn1_dev->cnn_lock);
		hobot_bpu_power_down(cnn1_dev);
	}
	return count;
}


#ifdef CHECK_IRQ_LOST
static ssize_t bpu0_checkirq_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", checkirq0_enable);
}
static ssize_t bpu0_checkirq_store(struct kobject *kobj,
			   struct kobj_attribute *attr,
			   const char *buf, size_t count)
{
	int ret;
	static int irq0_enable_tmp = 1;

	ret = sscanf(buf, "%du", &checkirq0_enable);
	if (checkirq0_enable != 0 && checkirq0_enable != 1) {
		pr_info("set 0 or 1!\n");
		checkirq0_enable = irq0_enable_tmp;
	}
	if (irq0_enable_tmp != checkirq0_enable) {
		irq0_enable_tmp = checkirq0_enable;
		if (cnn0_dev->ref_cnt) {
			if (checkirq0_enable) {
				cnn0_dev->cnn_timer.expires = jiffies + HZ * 5;
				add_timer(&cnn0_dev->cnn_timer);
			} else
				del_timer(&cnn0_dev->cnn_timer);
		}
	}
	return count;
}

static ssize_t bpu1_checkirq_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", checkirq1_enable);
}
static ssize_t bpu1_checkirq_store(struct kobject *kobj,
			   struct kobj_attribute *attr,
			   const char *buf, size_t count)
{
	int ret;
	static int irq1_enable_tmp = 1;

	ret = sscanf(buf, "%du", &checkirq1_enable);
	if (checkirq1_enable != 0 && checkirq1_enable != 1) {
		pr_info("set 0 or 1!\n");
		checkirq1_enable = irq1_enable_tmp;
	}
	if (irq1_enable_tmp != checkirq1_enable) {
		irq1_enable_tmp = checkirq1_enable;
		if (cnn1_dev->ref_cnt) {
			if (checkirq1_enable) {
				cnn1_dev->cnn_timer.expires = jiffies + HZ * 5;
				add_timer(&cnn1_dev->cnn_timer);
			} else
				del_timer(&cnn1_dev->cnn_timer);
		}
	}
	return count;

}
#endif
static struct kobj_attribute pro_frequency = __ATTR(profiler_frequency, 0664,
						    fre_show, fre_store);
static struct kobj_attribute pro_enable    = __ATTR(profiler_enable, 0664,
						    enable_show, enable_store);
static struct kobj_attribute burst_len = __ATTR(burst_len, 0664,
						    burst_len_show, burst_len_store);
static struct kobj_attribute fc_enable    = __ATTR(fc_time_enable, 0664,
						    fc_time_enable_show,
						    fc_time_enable_store);
static struct kobj_attribute pro_nseconds   = __ATTR(profiler_n_seconds, 0664,
						    nseconds_show, nseconds_store);
static struct kobj_attribute hotplug_ext   = __ATTR(hotplug_ext_flag, 0664,
						    hotplug_ext_flag_show, hotplug_ext_flag_store);
static struct kobj_attribute pro_ratio0    = __ATTR(ratio, 0444,
						    ratio0_show, NULL);
static struct kobj_attribute pro_ratio1    = __ATTR(ratio, 0444,
						    ratio1_show, NULL);
static struct kobj_attribute pro_queue0    = __ATTR(queue, 0444,
						    queue0_show, NULL);
static struct kobj_attribute pro_queue1    = __ATTR(queue, 0444,
						    queue1_show, NULL);
static struct kobj_attribute bpu0_fc_time  = __ATTR(fc_time, 0444,
						    bpu0_fc_time_show, NULL);
static struct kobj_attribute bpu1_fc_time  = __ATTR(fc_time, 0444,
						    bpu1_fc_time_show, NULL);
static struct kobj_attribute bpu0_clk_en   = __ATTR(clock_enable, 0644,
						    bpu0_clock_show,
						    bpu0_clock_store);
static struct kobj_attribute bpu1_clk_en   = __ATTR(clock_enable, 0644,
						    bpu1_clock_show,
						    bpu1_clock_store);
static struct kobj_attribute bpu0_power_en = __ATTR(power_enable, 0644,
						    bpu0_power_show,
						    bpu0_power_store);
static struct kobj_attribute bpu1_power_en  = __ATTR(power_enable, 0644,
						     bpu1_power_show,
						     bpu1_power_store);
static struct kobj_attribute bpu0_hotplug_en = __ATTR(hotplug_enable, 0644,
						    bpu0_hotplug_en_show,
						    bpu0_hotplug_en_store);
static struct kobj_attribute bpu1_hotplug_en  = __ATTR(hotplug_enable, 0644,
						     bpu1_hotplug_en_show,
						     bpu1_hotplug_en_store);

#ifdef CHECK_IRQ_LOST
static struct kobj_attribute bpu0_check_irq = __ATTR(check_irq, 0644,
						    bpu0_checkirq_show,
						    bpu0_checkirq_store);
static struct kobj_attribute bpu1_check_irq  = __ATTR(check_irq, 0644,
						     bpu1_checkirq_show,
						     bpu1_checkirq_store);
#endif

static struct attribute *bpu_attrs[] = {
	&pro_frequency.attr,
	&pro_enable.attr,
	&pro_nseconds.attr,
	&fc_enable.attr,
	&burst_len.attr,
	&hotplug_ext.attr,
	NULL,
};
static struct attribute *bpu0_attrs[] = {

	&pro_ratio0.attr,
	&pro_queue0.attr,
	&bpu0_fc_time.attr,
	&bpu0_clk_en.attr,
	&bpu0_power_en.attr,
	&bpu0_hotplug_en.attr,
#ifdef CHECK_IRQ_LOST
	&bpu0_check_irq.attr,
#endif
	NULL,
};
static struct attribute *bpu1_attrs[] = {
	&pro_ratio1.attr,
	&pro_queue1.attr,
	&bpu1_fc_time.attr,
	&bpu1_clk_en.attr,
	&bpu1_power_en.attr,
	&bpu1_hotplug_en.attr,
#ifdef CHECK_IRQ_LOST
	&bpu1_check_irq.attr,
#endif
	NULL,
};


static struct attribute_group bpu_attr_group = {
	.attrs = bpu_attrs,
};
static struct attribute_group bpu0_attr_group = {
	.name = "bpu0",
	.attrs = bpu0_attrs,
};
static struct attribute_group bpu1_attr_group = {
	.name = "bpu1",
	.attrs = bpu1_attrs,
};

static const struct attribute_group *bpu_attr_groups[] = {
	&bpu_attr_group,
	&bpu0_attr_group,
	&bpu1_attr_group,
	NULL,
};
static int __init hobot_bpu_init(void)
{
	int retval = 0;
	pr_info("%s\n", __func__);

	/*register bpu sys node*/
	if (subsys_system_register(&bpu_subsys, bpu_attr_groups))
		pr_err("fialed to register bpu subsystem");
	/* Register the platform driver */
	retval = platform_driver_register(&hobot_bpu_platform_driver);
	if (retval)
		pr_err("hobot bpu driver register failed\n");

	/*set bpu profiler timer*/
	init_timer(&check_timer);
	check_timer.function = &hobot_bpu_check_func;
	mutex_init(&enable_lock);
	fc_time_enable = 0;
	return retval;

}

static void __exit hobot_bpu_exit(void)
{
	bus_unregister(&bpu_subsys);
	/* Unregister the platform driver */
	platform_driver_unregister(&hobot_bpu_platform_driver);
}

module_init(hobot_bpu_init);
module_exit(hobot_bpu_exit);

MODULE_DESCRIPTION("Driver for HOBOT CNN");
MODULE_AUTHOR("Hobot Inc.");
MODULE_LICENSE("GPL");