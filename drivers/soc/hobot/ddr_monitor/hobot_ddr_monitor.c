/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ":%s " fmt, __func__

#include <linux/init.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/devfreq-event.h>

#include "./hobot_ddr_monitor.h"
#include "./hobot_ddr_mpu.h"

static DEFINE_MUTEX(ddr_mo_mutex);

struct ddr_monitor_dev_s {
	struct platform_device *pdev;
	void __iomem *regaddr;
	phys_addr_t res_paddr;
	uint32_t res_memsize;
	void *res_vaddr;
	int irq;
	int major;
	int minor;
	struct cdev cdev;
	dev_t dev_num;
	struct class *ddr_monitor_classes;
	wait_queue_head_t wq_head;
	spinlock_t lock;
	struct clk *sif_mclk;
	struct clk *ddr_mclk;
	int sif_mclk_is_open;
	struct devfreq_event_desc *desc;
	struct devfreq_event_dev *edev;
};

#ifdef CONFIG_HOBOT_XJ3

#define PORT_NUM 8
#define SYS_PCLK_HZ 300000000

/* Define DDRC registers for DDR detect */
#define uMCTL2_BASE 0xA2D00000
#define uMCTL2_MRCTRL0 0x10
#define uMCTL2_MRCTRL1 0x14
#define uMCTL2_MRSTAT  0x18


/* Define DDRC registers for ECC interrupt */
#define uMCTL2_ECCCFG0         0x0070
#define uMCTL2_ECCSTAT         0x0078
#define ECC_STAT_CORR_ERR      BIT8
#define ECC_STAT_UNCORR_ERR    BIT16

#define uMCTL2_ECCERRCNT       0x0080

#define uMCTL2_ECCCTL          0x007c
#define ECC_CORR_ERR_INTR_EN   BIT8
#define ECC_UNCORR_ERR_INTR_EN BIT9
#define ECC_UNCORR_ERR_CNT_CLR BIT3
#define ECC_CORR_ERR_CNT_CLR   BIT2
#define ECC_UNCORR_ERR_CLR     BIT1
#define ECC_CORR_ERR_CLR       BIT0

struct ddr_ecc_s {
	int irq;
	int enabled;
	void __iomem *ddrc_base;
	u64 corr_cnt;        // 1bit corrected error hit count
	u64 uncorr_cnt;      // 2bit uncorrected error hit count
	u64 uncorr_corr_cnt; // 3bit uncertain error hit count
	struct notifier_block panic_blk;
} ddr_ecc;


#else

#define PORT_NUM 6
#define SYS_PCLK_HZ 333333333

#endif

struct ddr_monitor_dev_s* g_ddr_monitor_dev = NULL;
int rd_cmd_bytes = 64;

struct ddr_portdata_s {
	unsigned int waddr_num;
	unsigned int wdata_num;
	unsigned int waddr_cyc;
	unsigned int waddr_latency;
	unsigned int raddr_num;
	unsigned int rdata_num;
	unsigned int raddr_cyc;
	unsigned int raddr_latency;
#ifdef CONFIG_HOBOT_XJ3
	unsigned int max_rtrans_cnt;
	unsigned int max_rvalid_cnt;
	unsigned int acc_rtrans_cnt;
	unsigned int min_rtrans_cnt;

	unsigned int max_wtrans_cnt;
	unsigned int max_wready_cnt;
	unsigned int acc_wtrans_cnt;
	unsigned int min_wtrans_cnt;
#endif
};

struct ddr_monitor_result_s {
	unsigned long long curtime;
	struct ddr_portdata_s portdata[PORT_NUM];
	unsigned int rd_cmd_num;
	unsigned int wr_cmd_num;
	unsigned int mwr_cmd_num;
	unsigned int rdwr_swi_num;
	unsigned int war_haz_num;
	unsigned int raw_haz_num;
	unsigned int waw_haz_num;
	unsigned int act_cmd_num;
	unsigned int act_cmd_rd_num;
	unsigned int per_cmd_num;
	unsigned int per_cmd_rdwr_num;

};

typedef struct ddr_monitor_sample_s {
       unsigned int sample_period;
       unsigned int sample_number;
} ddr_monitor_sample;

#define TOTAL_RECORD_NUM 400
#define TOTAL_RESULT_SIZE (400*1024)
ktime_t g_ktime_start;

struct ddr_monitor_result_s* ddr_info = NULL;
char * result_buf = NULL;
unsigned int g_current_index = 0;
volatile unsigned int g_record_num = 0;
unsigned int g_monitor_period = 10000;//unit is us, use 10ms by default
unsigned int g_sample_number = 0;
unsigned int g_extra_rd_info = 0;

module_param(g_current_index, uint, 0644);
//module_param(g_record_num, uint, 0644);
module_param(g_monitor_period, uint, 0644);
module_param(g_extra_rd_info, uint, 0644);

static int dbg_ddr_monitor_show(struct seq_file *s, void *unused)
{
#if 0
	int i,j;
	int start = 0;
	int cur = 0;
	volatile int num = 0;
	printk("g_record_num:%d\n", g_record_num);
	seq_printf(s, "The ddr_monitor module status: g_record_num:%d\n", g_record_num);
	if (!ddr_info) {
		seq_printf(s, "ddr_monitor not started \n");
		return 0;
	}
	if (g_record_num > 0) {
		num = g_record_num;
		printk("g_record_num@@@@@@:%d, num:%d\n", g_record_num, num);
		if (num >= TOTAL_RECORD_NUM)
			num = TOTAL_RECORD_NUM;
		start = (g_current_index + TOTAL_RECORD_NUM - num) % TOTAL_RECORD_NUM;
		g_record_num = 0;

		seq_printf(s, "The ddr_monitor module status: num:%d\n", num);
		for (j = 0; j < num; j++) {
			seq_printf(s, "\nTime %llu ", ddr_info[cur].curtime);
			seq_printf(s, "Read: ");
			cur = (start + j) % TOTAL_RECORD_NUM;
			for (i = 0; i < 6; i++)
			{
				if (ddr_info[cur].portdata[i].raddr_num) {
					seq_printf(s, "p[%d](bw:%u stall:%u delay:%u) ", i, \
						(ddr_info[cur].portdata[i].rdata_num * 16 * (1000/g_monitor_period)) >> 20, \
						ddr_info[cur].portdata[i].raddr_cyc / ddr_info[cur].portdata[i].rdata_num, \
						ddr_info[cur].portdata[i].raddr_latency / ddr_info[cur].portdata[i].rdata_num);
				} else {
					seq_printf(s, "p[%d](bw:%u stall:%u delay:%u) ", i, 0, 0, 0);
				}
			}
			seq_printf(s, "ddrc:%u MB/s; ", (ddr_info[cur].rd_cmd_num * 64 * (1000/g_monitor_period)) >> 20);
			seq_printf(s, "Write: ");
			for (i = 0; i < 6; i++)
			{
				if (ddr_info[cur].portdata[i].waddr_num) {
					seq_printf(s, "p[%d](bw:%u stall:%u delay:%u) ", i, \
						(ddr_info[cur].portdata[i].wdata_num * 16 * (1000/g_monitor_period)) >> 20, \
						ddr_info[cur].portdata[i].waddr_cyc / ddr_info[cur].portdata[i].wdata_num, \
						ddr_info[cur].portdata[i].waddr_latency / ddr_info[cur].portdata[i].wdata_num);
				} else {
					seq_printf(s, "p[%d](bw:%u stall:%u delay:%u) ", i, 0, 0, 0);
				}
			}
			seq_printf(s, "ddrc %u MB/s, mask %u MB/s\n", (ddr_info[cur].wr_cmd_num * 64 * (1000/g_monitor_period)) >> 20, (ddr_info[cur].mwr_cmd_num * 64 * (1000/g_monitor_poriod)) >> 20);
		}
		seq_printf(s, "The ddr_monitor module status end: g_record_num:%d\n", g_record_num);
	}
#endif
	return 0;
}

ssize_t ddr_monitor_debug_write(struct file *file, const char __user *buf, size_t size, loff_t *p)
{
	char info[255];

	memset(info, 0, 255);
	if (copy_from_user(info, buf, size))
		return size;

	pr_info(" %s\n", info);
	if (!memcmp(info, "ddrstart", 7)) {
		ddr_monitor_start();
		return size;
	} else if (!memcmp(info, "ddrstop", 7)) {
		ddr_monitor_stop();
		return size;
	}

	return size;
}


static int hobot_dfi_disable(struct devfreq_event_dev *edev)
{
//	struct hobot_dfi *info = devfreq_event_get_drvdata(edev);

	pr_debug("%s %d\n", __func__, __LINE__);
	ddr_monitor_stop();
#if 0
	clk_disable_unprepare(info->clk);
#endif

	return 0;
}

static int hobot_dfi_enable(struct devfreq_event_dev *edev)
{
#if 0
	struct hobot_dfi *info = devfreq_event_get_drvdata(edev);
	int ret;

	ret = clk_prepare_enable(info->clk);
	if (ret) {
		dev_err(&edev->dev, "failed to enable dfi clk: %d\n", ret);
		return ret;
	}
#endif

	pr_debug("%s %d\n", __func__, __LINE__);
	ddr_monitor_start();

	return 0;
}

static int hobot_dfi_set_event(struct devfreq_event_dev *edev)
{
	pr_debug("%s %d\n", __func__, __LINE__);
	/* we don't support. */
	return 0;
}

static int hobot_dfi_get_event(struct devfreq_event_dev *edev,
				struct devfreq_event_data *edata)
{
	unsigned long flags;
	struct ddr_monitor_result_s* cur_info;
	unsigned long read_cnt = 0;
	unsigned long write_cnt = 0;
	unsigned long cur_ddr_clk;

	pr_debug("%s %d\n", __func__, __LINE__);

	if (g_record_num <= 0) {
		pr_err("%s:%d ddr monitor record is empty\n", __func__, __LINE__);
		return -1;
	}

	cur_ddr_clk = clk_get_rate(g_ddr_monitor_dev->ddr_mclk) * 4;;
	pr_debug("cur_ddr_freq: %ld\n", cur_ddr_clk);

	spin_lock_irqsave(&g_ddr_monitor_dev->lock, flags);
	cur_info = &ddr_info[g_current_index];

	read_cnt = cur_info->rd_cmd_num * rd_cmd_bytes *
			(1000000 / g_monitor_period) >> 20;
	write_cnt = cur_info->wr_cmd_num * 64 *
			(1000000 / g_monitor_period) >> 20;

	edata->load_count = read_cnt + write_cnt;
	edata->total_count = (cur_ddr_clk * 4) >> 20;

	spin_unlock_irqrestore(&g_ddr_monitor_dev->lock, flags);

	pr_debug("rd_cnt:%ld,  wr_cnt:%ld, load:%ld, total:%ld\n",
		read_cnt, write_cnt, edata->load_count , edata->total_count);

	return 0;
}


static const struct devfreq_event_ops hobot_dfi_ops = {
	.disable = hobot_dfi_disable,
	.enable = hobot_dfi_enable,
	.get_event = hobot_dfi_get_event,
	.set_event = hobot_dfi_set_event,
};


static int get_monitor_data(char* buf)
{
	int i,j;
	int start = 0;
	int cur = 0;
	int length = 0;
	volatile int num = 0;
	unsigned long read_bw = 0;
	unsigned long write_bw = 0;
	unsigned long mask_bw = 0;

	if (!ddr_info) {
		length += sprintf(buf + length, "ddr_monitor not started \n");
		return 0;
	}

	if (g_record_num > 0) {

		num = g_record_num;
		if (num >= TOTAL_RECORD_NUM)
			num = TOTAL_RECORD_NUM;
		start = (g_current_index + TOTAL_RECORD_NUM - num) % TOTAL_RECORD_NUM;
		g_record_num = 0;
		for (j = 0; j < num; j++) {
			cur = (start + j) % TOTAL_RECORD_NUM;
			length += sprintf(buf + length, "Time %llu ", ddr_info[cur].curtime);
			length += sprintf(buf + length, "\nRead : ");
			for (i = 0; i < PORT_NUM; i++)
			{
				if (ddr_info[cur].portdata[i].raddr_num) {
					read_bw = ((unsigned long) ddr_info[cur].portdata[i].rdata_num) *
						  16 * (1000000 / g_monitor_period) >> 20;
					length += sprintf(buf + length, "p[%d](bw:%lu stall:%u delay:%u) ", i, \
						read_bw,\
						ddr_info[cur].portdata[i].raddr_cyc / ddr_info[cur].portdata[i].raddr_num, \
						ddr_info[cur].portdata[i].raddr_latency / ddr_info[cur].portdata[i].raddr_num);
#ifdef CONFIG_HOBOT_XJ3
					if (g_extra_rd_info) {
						length += sprintf(buf + length, "(maxRTrans:%u maxRvalid:%u accRtrans:%u minRtrans:%u) ", \
							ddr_info[cur].portdata[i].max_rtrans_cnt, \
							ddr_info[cur].portdata[i].max_rvalid_cnt, \
							ddr_info[cur].portdata[i].acc_rtrans_cnt / ddr_info[cur].portdata[i].waddr_num, \
							ddr_info[cur].portdata[i].min_rtrans_cnt);
					}
#endif
				} else {
					length += sprintf(buf + length, "p[%d](bw:%u stall:%u delay:%u) ", i, 0, 0, 0);
				}
			}
			read_bw = ((unsigned long) ddr_info[cur].rd_cmd_num) *
				  rd_cmd_bytes * (1000000/g_monitor_period) >> 20;
			length += sprintf(buf + length, "ddrc:%lu MB/s;\n", read_bw);
			length += sprintf(buf + length, "Write: ");
			for (i = 0; i < PORT_NUM; i++) {
				if (ddr_info[cur].portdata[i].waddr_num) {
					write_bw = ((unsigned long) ddr_info[cur].portdata[i].wdata_num) *
						    16 * (1000000 / g_monitor_period) >> 20;
					length += sprintf(buf + length, "p[%d](bw:%lu stall:%u delay:%u) ", i, \
							write_bw, \
						ddr_info[cur].portdata[i].waddr_cyc / ddr_info[cur].portdata[i].waddr_num, \
						ddr_info[cur].portdata[i].waddr_latency / ddr_info[cur].portdata[i].waddr_num);
#ifdef CONFIG_HOBOT_XJ3
					if (g_extra_rd_info) {
						length += sprintf(buf + length, "(maxWTrans:%u maxWready:%u accWtrans:%u minWtrans:%u) ", \
							ddr_info[cur].portdata[i].max_wtrans_cnt, \
							ddr_info[cur].portdata[i].max_wready_cnt, \
							ddr_info[cur].portdata[i].acc_wtrans_cnt / ddr_info[cur].portdata[i].waddr_num, \
							ddr_info[cur].portdata[i].min_wtrans_cnt);
					}
#endif
				} else {
					length += sprintf(buf + length, "p[%d](bw:%u stall:%u delay:%u) ", i, 0, 0, 0);
				}
			}
			write_bw = ((unsigned long) ddr_info[cur].wr_cmd_num) *
				    64 * (1000000 / g_monitor_period) >> 20;
			mask_bw = ((unsigned int) ddr_info[cur].mwr_cmd_num) *
				   64 * (1000000 / g_monitor_period) >> 20;
			length += sprintf(buf + length, "ddrc %lu MB/s, mask %lu MB/s\n", write_bw, mask_bw);
			length += sprintf(buf + length, "\n");

		}
	}
	return length;
}

static int dbg_ddr_monitor_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_ddr_monitor_show, &inode->i_private);
}

static const struct file_operations debug_fops = {
	.open = dbg_ddr_monitor_open,
	.read = seq_read,
	.write = ddr_monitor_debug_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init ddr_monitor_debuginit(void)
{
#ifdef CONFIG_HOBOT_XJ2
	(void)debugfs_create_file("x2-ddrmonitor", S_IRUGO, NULL, NULL, &debug_fops);
#else
	(void)debugfs_create_file("ddrmonitor", S_IRUGO, NULL, NULL, &debug_fops);
#endif

	return 0;
}


typedef struct _reg_s {
	uint32_t offset;
	uint32_t value;
} reg_t;

#define DDR_MONITOR_READ	_IOWR('m', 0, reg_t)
#define DDR_MONITOR_WRITE	_IOW('m', 1, reg_t)
#define DDR_MONITOR_CUR	_IOWR('m', 2, struct ddr_monitor_result_s)
#define DDR_MONITOR_SAMPLE_CONFIG_SET  _IOW('m', 3, ddr_monitor_sample)

static int ddr_monitor_mod_open(struct inode *pinode, struct file *pfile)
{
	pr_debug("\n");

	if (g_ddr_monitor_dev && !ddr_info) {
		ddr_info = vmalloc(sizeof(struct ddr_monitor_result_s) * TOTAL_RECORD_NUM);
		result_buf = g_ddr_monitor_dev->res_vaddr;//vmalloc(1024*80);
		g_current_index = 0;
		g_record_num = 0;
	}

	return 0;
}

static int ddr_monitor_mod_release(struct inode *pinode, struct file *pfile)
{
	pr_debug("\n");

	ddr_monitor_stop();

	return 0;
}

static ssize_t ddr_monitor_mod_read(struct file *pfile, char *puser_buf, size_t len, loff_t *poff)
{
	int result_len = 0;

	wait_event_interruptible(g_ddr_monitor_dev->wq_head,
					g_record_num >= g_sample_number);

	spin_lock_irq(&g_ddr_monitor_dev->lock);
	result_len = get_monitor_data(result_buf);
	spin_unlock_irq(&g_ddr_monitor_dev->lock);

	return result_len;
}

static ssize_t ddr_monitor_mod_write(struct file *pfile, const char *puser_buf, size_t len, loff_t *poff)
{
	pr_debug("\n");

	return 0;
}

static long ddr_monitor_mod_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
	void __iomem *iomem = g_ddr_monitor_dev->regaddr;
	reg_t reg;
	unsigned int temp = 0;
	ddr_monitor_sample sample_config;
	sample_config.sample_period = 1;
	sample_config.sample_number = 50;

	if ( NULL == iomem ) {
		pr_err("ddr_monitor no iomem\n");
		return -EINVAL;
	}

	switch (cmd) {
	case DDR_MONITOR_READ:
		if (!arg) {
			pr_err("reg read error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
			pr_err("reg read error, copy data from user failed\n");
			return -EINVAL;
		}
		reg.value = readl(iomem + reg.offset);
		if ( copy_to_user((void __user *)arg, (void *)&reg, sizeof(reg)) ) {
			pr_err("reg read error, copy data to user failed\n");
			return -EINVAL;
		}
		break;
	case DDR_MONITOR_WRITE:
		if ( !arg ) {
			pr_err("reg write error, reg should not be NULL");
			return -EINVAL;
		}
		if (copy_from_user((void *)&reg, (void __user *)arg, sizeof(reg))) {
			pr_err("reg write error, copy data from user failed\n");
			return -EINVAL;
		}
		writel(reg.value, iomem + reg.offset );
		break;
	case DDR_MONITOR_CUR:
		{
			int cur = 0;
			if (!arg) {
				pr_err("get cur error\n");
				return -EINVAL;
			}
			cur  = (g_current_index - 1 + TOTAL_RECORD_NUM) % TOTAL_RECORD_NUM;
			if ( copy_to_user((void __user *)arg, (void *)(ddr_info + cur), sizeof(struct ddr_monitor_result_s)) ) {
				pr_err("get cur error, copy data to user failed\n");
				return -EINVAL;
			}
		}
		break;
	case DDR_MONITOR_SAMPLE_CONFIG_SET:
		if (!arg) {
			dev_err(&g_ddr_monitor_dev->pdev->dev,
					"sampletime set arg should not be NULL\n");
			return -EINVAL;
		}
		if (copy_from_user((void *)&sample_config,
					(void __user *)arg, sizeof(ddr_monitor_sample))) {
			dev_err(&g_ddr_monitor_dev->pdev->dev,
					"sampletime set copy_from_user failed\n");
			return -EINVAL;
		}
		/* sample_period's unit is ms */
		temp = sample_config.sample_period;
		g_sample_number = sample_config.sample_number;
		writel(temp * (SYS_PCLK_HZ / 1000),
			g_ddr_monitor_dev->regaddr + PERF_MONITOR_PERIOD);
		g_monitor_period = temp * 1000;

		writel(0x1, g_ddr_monitor_dev->regaddr + PERF_MONITOR_ENABLE_UNMASK);
		writel(0xfff, g_ddr_monitor_dev->regaddr + PERF_MONITOR_ENABLE);
		g_ktime_start = ktime_get();
		break;
	default:

		break;
	}
	return 0;
}

int ddr_monitor_mmap(struct file *filp, struct vm_area_struct *pvma)
{

	if (remap_pfn_range(pvma, pvma->vm_start,
						virt_to_pfn(g_ddr_monitor_dev->res_vaddr),
						pvma->vm_end - pvma->vm_start,
						pvma->vm_page_prot)) {
		pr_err("ddr_monitor_mmap fail\n");
		return -EAGAIN;
	}

	return 0;
}

struct file_operations ddr_monitor_mod_fops = {
	.owner			= THIS_MODULE,
	.mmap 			= ddr_monitor_mmap,
	.open			= ddr_monitor_mod_open,
	.read			= ddr_monitor_mod_read,
	.write			= ddr_monitor_mod_write,
	.release		= ddr_monitor_mod_release,
	.unlocked_ioctl = ddr_monitor_mod_ioctl,
};

int ddr_monitor_cdev_create(void)
{
	int ret = 0;
	int error;

	pr_debug("\n");

	g_ddr_monitor_dev->ddr_monitor_classes = class_create(THIS_MODULE, "ddr_monitor");
	if (IS_ERR(g_ddr_monitor_dev->ddr_monitor_classes))
		return PTR_ERR(g_ddr_monitor_dev->ddr_monitor_classes);

	error = alloc_chrdev_region(&g_ddr_monitor_dev->dev_num, 0, 1, "ddr_monitor");

	if (!error) {
		g_ddr_monitor_dev->major = MAJOR(g_ddr_monitor_dev->dev_num);
		g_ddr_monitor_dev->minor = MINOR(g_ddr_monitor_dev->dev_num);
	}

	if (ret < 0)
		return ret;

	cdev_init(&g_ddr_monitor_dev->cdev, &ddr_monitor_mod_fops);
	g_ddr_monitor_dev->cdev.owner	= THIS_MODULE;

	cdev_add(&g_ddr_monitor_dev->cdev, g_ddr_monitor_dev->dev_num, 1);

	device_create(g_ddr_monitor_dev->ddr_monitor_classes, NULL, g_ddr_monitor_dev->dev_num, NULL, "ddrmonitor");
	if (ret)
		return ret;

	return ret;
}

void ddr_monitor_dev_remove(void)
{
	pr_debug("\n");

	cdev_del(&g_ddr_monitor_dev->cdev);

	unregister_chrdev_region(g_ddr_monitor_dev->dev_num, 1);
}

int ddr_monitor_start(void)
{
	pr_debug("\n");

	if (g_ddr_monitor_dev && !ddr_info) {

		writel(0xFFF, g_ddr_monitor_dev->regaddr + PERF_MONITOR_ENABLE);
		writel(0x1, g_ddr_monitor_dev->regaddr + PERF_MONITOR_ENABLE_UNMASK);
		ddr_info = vmalloc(sizeof(struct ddr_monitor_result_s) * TOTAL_RECORD_NUM);
		result_buf = g_ddr_monitor_dev->res_vaddr;
		g_current_index = 0;
		g_record_num = 0;
		pr_info("PERF_MONITOR_PERIOD is %dms\n", g_monitor_period/1000);
		writel((SYS_PCLK_HZ /1000000) * g_monitor_period, g_ddr_monitor_dev->regaddr + PERF_MONITOR_PERIOD);

	}

	return 0;
}

int ddr_monitor_stop(void)
{
	pr_debug("\n");
	if (g_ddr_monitor_dev && ddr_info) {
		writel(0, g_ddr_monitor_dev->regaddr + PERF_MONITOR_ENABLE);
		writel(0x1, g_ddr_monitor_dev->regaddr + PERF_MONITOR_ENABLE_SETMASK);
		vfree(ddr_info);
		result_buf = NULL;
		ddr_info = NULL;
	}
	return 0;
}

int ddr_get_port_status(void)
{
	int i = 0;
	ktime_t ktime;
	int step;

	ktime = ktime_sub(ktime_get(), g_ktime_start);
	ddr_info[g_current_index].curtime = ktime_to_us(ktime);

	for (i = 0; i < PORT_NUM; i++) {
		step = i * MP_REG_OFFSET;
#ifdef CONFIG_HOBOT_XJ3
		if (i >= 6) {
			pr_debug("i:%d step: %08x\n", i, step);
			step += MP_REG_OFFSET;
		}
#endif

		ddr_info[g_current_index].portdata[i].raddr_num = readl(g_ddr_monitor_dev->regaddr + MP_BASE_RADDR_TX_NUM + step);
		ddr_info[g_current_index].portdata[i].rdata_num = readl(g_ddr_monitor_dev->regaddr + MP_BASE_RDATA_TX_NUM + step);
		ddr_info[g_current_index].portdata[i].raddr_cyc = readl(g_ddr_monitor_dev->regaddr + MP_BASE_RADDR_ST_CYC + step);
		ddr_info[g_current_index].portdata[i].raddr_latency = readl(g_ddr_monitor_dev->regaddr + MP_BASE_RA2LSTRD_LATENCY + step);

#ifdef CONFIG_HOBOT_XJ3
		if (g_extra_rd_info) {
			ddr_info[g_current_index].portdata[i].max_rtrans_cnt = readl(g_ddr_monitor_dev->regaddr + MP_BASE_MAX_RTRANS_CNT + step);
			ddr_info[g_current_index].portdata[i].max_rvalid_cnt = readl(g_ddr_monitor_dev->regaddr + MP_BASE_MAX_RVALID_CNT + step);
			ddr_info[g_current_index].portdata[i].acc_rtrans_cnt = readl(g_ddr_monitor_dev->regaddr + MP_BASE_ACC_RTRANS_CNT + step);
			ddr_info[g_current_index].portdata[i].min_rtrans_cnt = readl(g_ddr_monitor_dev->regaddr + MP_BASE_MIN_RTRANS_CNT + step);

			ddr_info[g_current_index].portdata[i].max_wtrans_cnt = readl(g_ddr_monitor_dev->regaddr + MP_BASE_MAX_WTRANS_CNT + step);
			ddr_info[g_current_index].portdata[i].max_wready_cnt = readl(g_ddr_monitor_dev->regaddr + MP_BASE_MAX_WREADY_CNT + step);
			ddr_info[g_current_index].portdata[i].acc_wtrans_cnt = readl(g_ddr_monitor_dev->regaddr + MP_BASE_ACC_WTRANS_CNT + step);
			ddr_info[g_current_index].portdata[i].min_wtrans_cnt = readl(g_ddr_monitor_dev->regaddr + MP_BASE_MIN_WTRANS_CNT + step);
		}
#endif
		ddr_info[g_current_index].portdata[i].waddr_num = readl(g_ddr_monitor_dev->regaddr + MP_BASE_WADDR_TX_NUM + step);
		ddr_info[g_current_index].portdata[i].wdata_num = readl(g_ddr_monitor_dev->regaddr + MP_BASE_WDATA_TX_NUM + step);
		ddr_info[g_current_index].portdata[i].waddr_cyc = readl(g_ddr_monitor_dev->regaddr + MP_BASE_WADDR_ST_CYC + step);
		ddr_info[g_current_index].portdata[i].waddr_latency = readl(g_ddr_monitor_dev->regaddr + MP_BASE_WA2BRESP_LATENCY + step);
	}

	ddr_info[g_current_index].rd_cmd_num = readl(g_ddr_monitor_dev->regaddr + RD_CMD_TX_NUM);
	ddr_info[g_current_index].wr_cmd_num= readl(g_ddr_monitor_dev->regaddr + WR_CMD_TX_NUM);
	ddr_info[g_current_index].mwr_cmd_num = readl(g_ddr_monitor_dev->regaddr + MWR_CMD_TX_NUM);
	ddr_info[g_current_index].rdwr_swi_num = readl(g_ddr_monitor_dev->regaddr + RDWR_SWITCH_NUM);
	ddr_info[g_current_index].act_cmd_num = readl(g_ddr_monitor_dev->regaddr + ACT_CMD_TX_NUM);
	ddr_info[g_current_index].act_cmd_rd_num = readl(g_ddr_monitor_dev->regaddr + ACT_CMD_TX_FOR_RD_TX_NUM);
	ddr_info[g_current_index].war_haz_num = readl(g_ddr_monitor_dev->regaddr + WAR_HAZARD_NUM);
	ddr_info[g_current_index].raw_haz_num = readl(g_ddr_monitor_dev->regaddr + RAW_HAZARD_NUM);
	ddr_info[g_current_index].waw_haz_num = readl(g_ddr_monitor_dev->regaddr + WAW_HAZARD_NUM);
	ddr_info[g_current_index].per_cmd_num = readl(g_ddr_monitor_dev->regaddr + PERCHARGE_CMD_TX_NUM);
	ddr_info[g_current_index].per_cmd_rdwr_num = readl(g_ddr_monitor_dev->regaddr + PERCHARGE_CMD_FOR_RDWR_TX_NUM);


	//ddr_info[g_current_index].curtime = jiffies;
	g_current_index = (g_current_index + 1) % TOTAL_RECORD_NUM;
	g_record_num ++;

	if (g_record_num >= g_sample_number)
		wake_up_interruptible(&g_ddr_monitor_dev->wq_head);
	return 0;
}

static irqreturn_t ddr_monitor_isr(int this_irq, void *data)
{
	writel(0x1, g_ddr_monitor_dev->regaddr + PERF_MONITOR_SRCPND);

	spin_lock(&g_ddr_monitor_dev->lock);
	ddr_get_port_status();
	spin_unlock(&g_ddr_monitor_dev->lock);

	return IRQ_HANDLED;
}

static irqreturn_t ddr_ecc_isr(int this_irq, void *data)
{
	void __iomem *ddrc_base = ddr_ecc.ddrc_base;
	void __iomem *ecc_ctrl = ddrc_base + uMCTL2_ECCCTL;
	u32 ecc_stat;
	u32 ecc_cnt;


	ecc_stat = readl(ddrc_base + uMCTL2_ECCSTAT);
	ecc_cnt  = readl(ddrc_base + uMCTL2_ECCERRCNT);

	if ((ecc_stat & ECC_STAT_CORR_ERR) &&
	     (ecc_stat & ECC_STAT_UNCORR_ERR)) {
		ddr_ecc.uncorr_corr_cnt += (ecc_cnt & 0xFFFF) + (ecc_cnt >> 16);
		pr_debug("3+ bits ecc error hits %llu\n", ddr_ecc.uncorr_corr_cnt);
	} else if (ecc_stat & ECC_STAT_CORR_ERR) {
		ddr_ecc.corr_cnt += ecc_cnt & 0xFFFF;
		printk_once("1 bit corrected ecc error detected\n");
		pr_debug("1 bit ecc error hits %llu\n", ddr_ecc.corr_cnt);
	} else if (ecc_stat & ECC_STAT_UNCORR_ERR) {
		ddr_ecc.uncorr_cnt += ecc_cnt >> 16;
		pr_debug("2 bits ecc error hits %llu\n", ddr_ecc.uncorr_cnt);
	}

	writel(readl(ecc_ctrl) | ECC_UNCORR_ERR_CNT_CLR, ecc_ctrl);
	writel(readl(ecc_ctrl) | ECC_CORR_ERR_CNT_CLR, ecc_ctrl);

	writel(readl(ecc_ctrl) | ECC_UNCORR_ERR_CLR, ecc_ctrl);
	writel(readl(ecc_ctrl) | ECC_CORR_ERR_CLR, ecc_ctrl);


	return IRQ_HANDLED;
}


static unsigned int read_ctl_value;
static unsigned int write_ctl_value;
static ssize_t cpu_read_ctl_store(struct device_driver *drv,
				  const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~0x0f;
	tmp |= read_ctl_value;
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t cpu_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= 0x0f;

	return sprintf(buf, "%x\n", tmp);
}
static ssize_t bifdma_read_ctl_store(struct device_driver *drv,
				     const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 4);
	tmp |= (read_ctl_value << 4);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t bifdma_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 4;
	tmp &= 0x0f;

	return sprintf(buf, "%x\n", tmp);
}
static ssize_t bpu0_read_ctl_store(struct device_driver *drv,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 8);
	tmp |= (read_ctl_value << 8);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t bpu0_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 8;
	tmp &= 0x0f;

	return sprintf(buf, "%x\n", tmp);
}

static ssize_t bpu1_read_ctl_store(struct device_driver *drv,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 12);
	tmp |= (read_ctl_value << 12);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t bpu1_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 12;
	tmp &= 0x0f;

	return sprintf(buf, "%x\n", tmp);
}
static ssize_t vio0_read_ctl_store(struct device_driver *drv,
				  const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 16);
	tmp |= (read_ctl_value << 16);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t vio0_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 16;
	tmp &= 0x0f;

	return sprintf(buf, "%x\n", tmp);
}

#ifdef CONFIG_HOBOT_XJ3
static ssize_t vpu_read_ctl_store(struct device_driver *drv,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 20);
	tmp |= (read_ctl_value << 20);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t vpu_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 20;
	tmp &= 0x0f;

	return sprintf(buf, "%x\n", tmp);
}

static ssize_t vio1_read_ctl_store(struct device_driver *drv,
				  const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << 24);
	tmp |= (read_ctl_value << 24);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t vio1_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= 24;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
#endif

static ssize_t periph_read_ctl_store(struct device_driver *drv,
				     const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;
	int shift = 20;

#ifdef CONFIG_HOBOT_XJ3
	shift = 28;
#endif

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &read_ctl_value);
	if (read_ctl_value > 15 || read_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", read_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp &= ~(0x0f << shift);
	tmp |= (read_ctl_value << shift);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t periph_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int shift = 20;

#ifdef CONFIG_HOBOT_XJ3
	shift = 28;
#endif

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	tmp >>= shift;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t all_read_ctl_store(struct device_driver *drv,
				  const char *buf, size_t count)
{
	mutex_lock(&ddr_mo_mutex);

	sscanf(buf, "%x", &read_ctl_value);
	writel(read_ctl_value, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);

	mutex_unlock(&ddr_mo_mutex);

	return count;
}

static ssize_t all_read_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	len = sprintf(buf, "%08x:\n", tmp);
	len += sprintf(buf + len, "P0_CPU:    %d\n", tmp & 0xF);
	len += sprintf(buf + len, "P1_BIFDMA: %d\n", (tmp >> 4) & 0xF);
	len += sprintf(buf + len, "P2_CNN0:   %d\n", (tmp >> 8) & 0xF);
	len += sprintf(buf + len, "P3_CNN1:   %d\n", (tmp >> 12) & 0xF);
	len += sprintf(buf + len, "P4_VIO0:   %d\n", (tmp >> 16) & 0xF);
#ifdef CONFIG_HOBOT_XJ2
	len += sprintf(buf + len, "P5_PERI:   %d\n", (tmp >> 20) & 0xF);
#else
	len += sprintf(buf + len, "P5_VPU:    %d\n", (tmp >> 20) & 0xF);
	len += sprintf(buf + len, "P6_VIO1:   %d\n", (tmp >> 24) & 0xF);
	len += sprintf(buf + len, "P7_PERI:   %d\n", (tmp >> 28) & 0xF);
#endif

	return len;
}


static struct driver_attribute cpu_read_ctl = __ATTR(cpu, 0664,
						   cpu_read_ctl_show,
						   cpu_read_ctl_store);
static struct driver_attribute bifdma_read_ctl = __ATTR(bifdma, 0664,
						      bifdma_read_ctl_show,
						      bifdma_read_ctl_store);
static struct driver_attribute bpu0_read_ctl = __ATTR(bpu0, 0664,
						    bpu0_read_ctl_show,
						    bpu0_read_ctl_store);
static struct driver_attribute bpu1_read_ctl = __ATTR(bpu1, 0664,
						    bpu1_read_ctl_show,
						    bpu1_read_ctl_store);
static struct driver_attribute vio0_read_ctl    = __ATTR(vio0, 0664,
						      vio0_read_ctl_show,
						      vio0_read_ctl_store);
#ifdef CONFIG_HOBOT_XJ3
static struct driver_attribute vpu_read_ctl    = __ATTR(vpu, 0664,
						      vpu_read_ctl_show,
						      vpu_read_ctl_store);
static struct driver_attribute vio1_read_ctl    = __ATTR(vio1, 0664,
						      vio1_read_ctl_show,
						      vio1_read_ctl_store);
#endif
static struct driver_attribute periph_read_ctl = __ATTR(peri, 0664,
						      periph_read_ctl_show,
						      periph_read_ctl_store);
static struct driver_attribute all_read_ctl    = __ATTR(all, 0664,
						      all_read_ctl_show,
						      all_read_ctl_store);

static struct attribute *read_qctrl_attrs[] = {
	&cpu_read_ctl.attr,
	&bifdma_read_ctl.attr,
	&bpu0_read_ctl.attr,
	&bpu1_read_ctl.attr,
	&vio0_read_ctl.attr,
#ifdef CONFIG_HOBOT_XJ3
	&vpu_read_ctl.attr,
	&vio1_read_ctl.attr,
#endif
	&periph_read_ctl.attr,
	&all_read_ctl.attr,
	NULL,
};


static ssize_t cpu_write_ctl_store(struct device_driver *drv,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~0x0f;
	tmp |= write_ctl_value;
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t cpu_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t bifdma_write_ctl_store(struct device_driver *drv,
				      const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 4);
	tmp |= (write_ctl_value << 4);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t bifdma_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 4;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t bpu0_write_ctl_store(struct device_driver *drv,
				    const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 8);
	tmp |= (write_ctl_value << 8);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t bpu0_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 8;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}

static ssize_t bpu1_write_ctl_store(struct device_driver *drv,
				    const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 12);
	tmp |= (write_ctl_value << 12);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t bpu1_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 12;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t vio0_write_ctl_store(struct device_driver *drv,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 16);
	tmp |= (write_ctl_value << 16);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;

}

static ssize_t vio0_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 16;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}

#ifdef CONFIG_HOBOT_XJ3
static ssize_t vpu_write_ctl_store(struct device_driver *drv,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 20);
	tmp |= (write_ctl_value << 20);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;

}

static ssize_t vpu_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 20;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}

static ssize_t vio1_write_ctl_store(struct device_driver *drv,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << 24);
	tmp |= (write_ctl_value << 24);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;

}

static ssize_t vio1_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= 24;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
#endif

static ssize_t periph_write_ctl_store(struct device_driver *drv,
				      const char *buf, size_t count)
{
	int ret;
	unsigned int tmp;
	int shift = 20;

#ifdef CONFIG_HOBOT_XJ3
	shift = 28;
#endif

	mutex_lock(&ddr_mo_mutex);
	ret = sscanf(buf, "%du", &write_ctl_value);
	if (write_ctl_value > 15 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~15\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp &= ~(0x0f << shift);
	tmp |= (write_ctl_value << shift);
	writel(tmp, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t periph_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int shift = 20;

#ifdef CONFIG_HOBOT_XJ3
	shift = 28;
#endif

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	tmp >>= shift;
	tmp &= 0x0f;
	return sprintf(buf, "%x\n", tmp);
}
static ssize_t all_write_ctl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	mutex_lock(&ddr_mo_mutex);

	sscanf(buf, "%x", &write_ctl_value);
	writel(write_ctl_value, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);

	mutex_unlock(&ddr_mo_mutex);
	return count;
}

static ssize_t all_write_ctl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;

	tmp = readl(g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
	len = sprintf(buf, "%08x:\n", tmp);
	len += sprintf(buf + len, "P0_CPU:    %d\n", tmp & 0xF);
	len += sprintf(buf + len, "P1_BIFDMA: %d\n", (tmp >> 4) & 0xF);
	len += sprintf(buf + len, "P2_CNN0:   %d\n", (tmp >> 8) & 0xF);
	len += sprintf(buf + len, "P3_CNN1:   %d\n", (tmp >> 12) & 0xF);
	len += sprintf(buf + len, "P4_VIO0:   %d\n", (tmp >> 16) & 0xF);
#ifdef CONFIG_HOBOT_XJ2
	len += sprintf(buf + len, "P5_PERI:   %d\n", (tmp >> 20) & 0xF);
#else
	len += sprintf(buf + len, "P5_VPU:    %d\n", (tmp >> 20) & 0xF);
	len += sprintf(buf + len, "P6_VIO1:   %d\n", (tmp >> 24) & 0xF);
	len += sprintf(buf + len, "P7_PERI:   %d\n", (tmp >> 28) & 0xF);
#endif

	return len;
}


static struct driver_attribute cpu_write_ctl = __ATTR(cpu, 0664,
						    cpu_write_ctl_show,
						    cpu_write_ctl_store);
static struct driver_attribute bifdma_write_ctl = __ATTR(bifdma, 0664,
						       bifdma_write_ctl_show,
						       bifdma_write_ctl_store);
static struct driver_attribute bpu0_write_ctl = __ATTR(bpu0, 0664,
						     bpu0_write_ctl_show,
						     bpu0_write_ctl_store);
static struct driver_attribute bpu1_write_ctl = __ATTR(bpu1, 0664,
						     bpu1_write_ctl_show,
						     bpu1_write_ctl_store);
static struct driver_attribute vio0_write_ctl    = __ATTR(vio0, 0664,
						       vio0_write_ctl_show,
						       vio0_write_ctl_store);
#ifdef CONFIG_HOBOT_XJ3
static struct driver_attribute vpu_write_ctl    = __ATTR(vpu, 0664,
						       vpu_write_ctl_show,
						       vpu_write_ctl_store);
static struct driver_attribute vio1_write_ctl    = __ATTR(vio1, 0664,
						       vio1_write_ctl_show,
						       vio1_write_ctl_store);
#endif

static struct driver_attribute periph_write_ctl = __ATTR(peri, 0664,
						       periph_write_ctl_show,
						       periph_write_ctl_store);
static struct driver_attribute all_write_ctl    = __ATTR(all, 0664,
						       all_write_ctl_show,
						       all_write_ctl_store);

static struct attribute *write_qctl_attrs[] = {
	&cpu_write_ctl.attr,
	&bifdma_write_ctl.attr,
	&bpu0_write_ctl.attr,
	&bpu1_write_ctl.attr,
	&vio0_write_ctl.attr,
#ifdef CONFIG_HOBOT_XJ3
	&vpu_write_ctl.attr,
	&vio1_write_ctl.attr,
#endif
	&periph_write_ctl.attr,
	&all_write_ctl.attr,
	NULL,
};

#ifdef CONFIG_HOBOT_XJ3
ssize_t open_sif_mclk(void)
{
	if ((g_ddr_monitor_dev == NULL) || (g_ddr_monitor_dev->sif_mclk == NULL))
		return -1;
	g_ddr_monitor_dev->sif_mclk_is_open =
		__clk_is_enabled(g_ddr_monitor_dev->sif_mclk);
	if (g_ddr_monitor_dev->sif_mclk_is_open == 0) {
		clk_prepare_enable(g_ddr_monitor_dev->sif_mclk);
	}

	return 0;
}

ssize_t close_sif_mclk(void)
{
	if ((g_ddr_monitor_dev == NULL) || (g_ddr_monitor_dev->sif_mclk == NULL))
		return -1;
	if (g_ddr_monitor_dev->sif_mclk_is_open == 0) {
		clk_disable_unprepare(g_ddr_monitor_dev->sif_mclk);
	}

	return 0;
}

u32 read_axibus_reg(void)
{
	unsigned int val = 0;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);

	mutex_lock(&ddr_mo_mutex);
	open_sif_mclk();

	val = readl(axibus_reg);

	close_sif_mclk();
	mutex_unlock(&ddr_mo_mutex);

	iounmap(axibus_reg);

	return val;
}

static ssize_t sifw_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 17;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);
	mutex_lock(&ddr_mo_mutex);
	open_sif_mclk();

	sscanf(buf, "%x", &write_ctl_value);
	if (write_ctl_value > 1 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~1\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (write_ctl_value << shift);

	writel(tmp, axibus_reg);
	close_sif_mclk();
	mutex_unlock(&ddr_mo_mutex);
	iounmap(axibus_reg);
	return count;
}

static ssize_t sifw_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 17;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);

	open_sif_mclk();
	tmp = readl(axibus_reg);
	len = sprintf(buf, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += sprintf(buf + len, "sif: vio1\n");
	} else {
		len += sprintf(buf + len, "sif: vio0\n");
	}
	iounmap(axibus_reg);
	close_sif_mclk();
	return len;
}

static ssize_t isp0m0_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 18;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);
	mutex_lock(&ddr_mo_mutex);

	open_sif_mclk();
	sscanf(buf, "%x", &write_ctl_value);
	if (write_ctl_value > 1 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~1\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (write_ctl_value << shift);

	writel(tmp, axibus_reg);
	mutex_unlock(&ddr_mo_mutex);
	iounmap(axibus_reg);
	close_sif_mclk();
	return count;
}

static ssize_t isp0m0_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 18;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);

	open_sif_mclk();
	tmp = readl(axibus_reg);
	len = sprintf(buf, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += sprintf(buf + len, "isp_0_m0: vio1\n");
	} else {
		len += sprintf(buf + len, "isp_0_m0: vio0\n");
	}
	iounmap(axibus_reg);
	close_sif_mclk();
	return len;
}

// isp_0_m1
static ssize_t isp0m1_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 19;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);
	mutex_lock(&ddr_mo_mutex);

	open_sif_mclk();
	sscanf(buf, "%x", &write_ctl_value);
	if (write_ctl_value > 1 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~1\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (write_ctl_value << shift);

	writel(tmp, axibus_reg);
	mutex_unlock(&ddr_mo_mutex);
	iounmap(axibus_reg);
	close_sif_mclk();
	return count;
}

static ssize_t isp0m1_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 19;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);

	open_sif_mclk();
	tmp = readl(axibus_reg);
	len = sprintf(buf, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += sprintf(buf + len, "isp_0_m1: vio1\n");
	} else {
		len += sprintf(buf + len, "isp_0_m1: vio0\n");
	}
	iounmap(axibus_reg);
	close_sif_mclk();
	return len;
}

// isp_0_m2
static ssize_t isp0m2_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 20;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);

	mutex_lock(&ddr_mo_mutex);
	open_sif_mclk();
	sscanf(buf, "%x", &write_ctl_value);
	if (write_ctl_value > 1 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~1\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (write_ctl_value << shift);

	writel(tmp, axibus_reg);
	mutex_unlock(&ddr_mo_mutex);
	iounmap(axibus_reg);
	close_sif_mclk();
	return count;
}

static ssize_t isp0m2_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 20;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);

	open_sif_mclk();
	tmp = readl(axibus_reg);
	len = sprintf(buf, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += sprintf(buf + len, "isp_0_m2: vio1\n");
	} else {
		len += sprintf(buf + len, "isp_0_m2: vio0\n");
	}
	iounmap(axibus_reg);
	close_sif_mclk();
	return len;
}

// gdc_0
static ssize_t gdc0_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 24;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);
	mutex_lock(&ddr_mo_mutex);

	open_sif_mclk();
	sscanf(buf, "%x", &write_ctl_value);
	if (write_ctl_value > 1 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~1\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (write_ctl_value << shift);

	writel(tmp, axibus_reg);
	mutex_unlock(&ddr_mo_mutex);
	iounmap(axibus_reg);
	close_sif_mclk();
	return count;
}

static ssize_t gdc0_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 24;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);

	open_sif_mclk();
	tmp = readl(axibus_reg);
	len = sprintf(buf, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += sprintf(buf + len, "gdc_0: vio1\n");
	} else {
		len += sprintf(buf + len, "gdc_0: vio0\n");
	}
	iounmap(axibus_reg);
	close_sif_mclk();
	return len;
}

// t21
static ssize_t t21_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 25;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);
	mutex_lock(&ddr_mo_mutex);

	open_sif_mclk();
	sscanf(buf, "%x", &write_ctl_value);
	if (write_ctl_value > 1 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~1\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (write_ctl_value << shift);

	writel(tmp, axibus_reg);
	mutex_unlock(&ddr_mo_mutex);
	iounmap(axibus_reg);
	close_sif_mclk();
	return count;
}

static ssize_t t21_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 25;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);

	open_sif_mclk();
	tmp = readl(axibus_reg);
	len = sprintf(buf, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += sprintf(buf + len, "t21: vio1\n");
	} else {
		len += sprintf(buf + len, "t21: vio0\n");
	}
	iounmap(axibus_reg);
	close_sif_mclk();
	return len;
}

// gdc_1
static ssize_t gdc1_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 26;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);
	mutex_lock(&ddr_mo_mutex);

	open_sif_mclk();
	sscanf(buf, "%x", &write_ctl_value);
	if (write_ctl_value > 1 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~1\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (write_ctl_value << shift);

	writel(tmp, axibus_reg);
	mutex_unlock(&ddr_mo_mutex);
	iounmap(axibus_reg);
	close_sif_mclk();
	return count;
}

static ssize_t gdc1_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 26;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);

	open_sif_mclk();
	tmp = readl(axibus_reg);
	len = sprintf(buf, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += sprintf(buf + len, "gdc_1: vio1\n");
	} else {
		len += sprintf(buf + len, "gdc_1: vio0\n");
	}
	iounmap(axibus_reg);
	close_sif_mclk();
	return len;
}

// sifr
static ssize_t sifr_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 27;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);
	mutex_lock(&ddr_mo_mutex);

	open_sif_mclk();
	sscanf(buf, "%x", &write_ctl_value);
	if (write_ctl_value > 1 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~1\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (write_ctl_value << shift);

	writel(tmp, axibus_reg);
	mutex_unlock(&ddr_mo_mutex);
	iounmap(axibus_reg);
	close_sif_mclk();
	return count;
}

static ssize_t sifr_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 27;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);

	open_sif_mclk();
	tmp = readl(axibus_reg);
	len = sprintf(buf, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += sprintf(buf + len, "sifr: vio1\n");
	} else {
		len += sprintf(buf + len, "sifr: vio0\n");
	}
	iounmap(axibus_reg);
	close_sif_mclk();
	return len;
}

// ipu
static ssize_t ipu0_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 28;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);
	mutex_lock(&ddr_mo_mutex);

	open_sif_mclk();
	sscanf(buf, "%x", &write_ctl_value);
	if (write_ctl_value > 1 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~1\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (write_ctl_value << shift);

	writel(tmp, axibus_reg);
	mutex_unlock(&ddr_mo_mutex);
	iounmap(axibus_reg);
	close_sif_mclk();
	return count;
}

static ssize_t ipu0_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 28;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);

	open_sif_mclk();
	tmp = readl(axibus_reg);
	len = sprintf(buf, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += sprintf(buf + len, "ipu0: vio1\n");
	} else {
		len += sprintf(buf + len, "ipu0: vio0\n");
	}
	iounmap(axibus_reg);
	close_sif_mclk();
	return len;
}

// pym
static ssize_t pym_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 30;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);
	mutex_lock(&ddr_mo_mutex);

	open_sif_mclk();
	sscanf(buf, "%x", &write_ctl_value);
	if (write_ctl_value > 1 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~1\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (write_ctl_value << shift);

	writel(tmp, axibus_reg);
	mutex_unlock(&ddr_mo_mutex);
	iounmap(axibus_reg);
	close_sif_mclk();
	return count;
}

static ssize_t pym_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 30;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);

	open_sif_mclk();
	tmp = readl(axibus_reg);
	len = sprintf(buf, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += sprintf(buf + len, "pym: vio1\n");
	} else {
		len += sprintf(buf + len, "pym: vio0\n");
	}
	iounmap(axibus_reg);
	close_sif_mclk();
	return len;
}

// iar
static ssize_t iar_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int shift = 31;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);
	mutex_lock(&ddr_mo_mutex);

	open_sif_mclk();
	sscanf(buf, "%x", &write_ctl_value);
	if (write_ctl_value > 1 || write_ctl_value < 0) {
		pr_err("set value %d error,you should set 0~1\n", write_ctl_value);
		mutex_unlock(&ddr_mo_mutex);
		return count;
	}
	tmp = readl(axibus_reg);
	tmp &= ~(1 << shift);
	tmp |= (write_ctl_value << shift);

	writel(tmp, axibus_reg);
	mutex_unlock(&ddr_mo_mutex);
	iounmap(axibus_reg);
	close_sif_mclk();
	return count;
}

static ssize_t iar_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	int shift = 31;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);

	open_sif_mclk();
	tmp = readl(axibus_reg);
	len = sprintf(buf, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1 << shift)) {
		len += sprintf(buf + len, "iar: vio1\n");
	} else {
		len += sprintf(buf + len, "iar: vio0\n");
	}
	iounmap(axibus_reg);
	close_sif_mclk();
	return len;
}

static ssize_t all_axibus_ctrl_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);
	mutex_lock(&ddr_mo_mutex);
	open_sif_mclk();
	sscanf(buf, "%x", &write_ctl_value);

	writel(write_ctl_value, axibus_reg);
	mutex_unlock(&ddr_mo_mutex);
	iounmap(axibus_reg);
	close_sif_mclk();
	return count;
}

static ssize_t all_axibus_ctrl_show(struct device_driver *drv, char *buf)
{
	unsigned int tmp;
	int len = 0;
	void __iomem *axibus_reg = ioremap_nocache(0xa4000038, 4);

	open_sif_mclk();
	tmp = readl(axibus_reg);
	len = sprintf(buf, "axibus: 0x%08x:\n", tmp);
	if (tmp & (1<<17)) {    // sif bif17
		len += sprintf(buf + len, "sifw: vio1\n");
	}
	if (tmp & (1<<18)) {    // bit18
		len += sprintf(buf + len, "isp_0_m0: vio1\n");
	}
	if (tmp & (1<<19)) {    //  bit19
		len += sprintf(buf + len, "isp_0_m1: vio1\n");
	}
	if (tmp & (1<<20)) {   // bit20
		len += sprintf(buf + len, "isp_0_m2: vio1\n");
	}
	// if (tmp & (1<<21)) {
	// 	len += sprintf(buf + len, "isp_1_m0: vio1\n");
	// }
	// if (tmp & (1<<22)) {
	// 	len += sprintf(buf + len, "isp_1_m1: vio1\n");
	// }
	// if (tmp & (1<<23)) {
	// 	len += sprintf(buf + len, "isp_1_m2: vio1\n");
	// }
	if (tmp & (1<<24)) {
		len += sprintf(buf + len, "gdc0: vio1\n");
	}
	if (tmp & (1<<25)) {
		len += sprintf(buf + len, "t21_0: vio1\n");
	}
	if (tmp & (1<<26)) {
		len += sprintf(buf + len, "gdc_1: vio1\n");
	}
	if (tmp & (1<<27)) {
		len += sprintf(buf + len, "sifr: vio1\n");
	}
	if (tmp & (1<<28)) {
		len += sprintf(buf + len, "ipu_0: vio1\n");
	}
	// if (tmp & (1<<29)) {
	// 	len += sprintf(buf + len, "ipu_1: vio1\n");
	// }
	if (tmp & (1<<30)) {
		len += sprintf(buf + len, "pym: vio1\n");
	}
	if (tmp & (1<<31)) {
		len += sprintf(buf + len, "iar: vio1\n");
	}
	iounmap(axibus_reg);
	close_sif_mclk();

	return len;
}

static struct driver_attribute sifw_axibus_ctrl    = __ATTR(sifw, 0664,
						       sifw_axibus_ctrl_show,
						       sifw_axibus_ctrl_store);
static struct driver_attribute isp0m0_axibus_ctrl    = __ATTR(isp0m0, 0664,
						       isp0m0_axibus_ctrl_show,
						       isp0m0_axibus_ctrl_store);
static struct driver_attribute isp0m1_axibus_ctrl    = __ATTR(isp0m1, 0664,
						       isp0m1_axibus_ctrl_show,
						       isp0m1_axibus_ctrl_store);
static struct driver_attribute isp0m2_axibus_ctrl    = __ATTR(isp0m2, 0664,
						       isp0m2_axibus_ctrl_show,
						       isp0m2_axibus_ctrl_store);

static struct driver_attribute gdc0_axibus_ctrl    = __ATTR(gdc0, 0664,
						       gdc0_axibus_ctrl_show,
						       gdc0_axibus_ctrl_store);
static struct driver_attribute t21_axibus_ctrl    = __ATTR(t21, 0664,
						       t21_axibus_ctrl_show,
						       t21_axibus_ctrl_store);
static struct driver_attribute gdc1_axibus_ctrl    = __ATTR(gdc1, 0664,
						       gdc1_axibus_ctrl_show,
						       gdc1_axibus_ctrl_store);
static struct driver_attribute ipu0_axibus_ctrl    = __ATTR(ipu0, 0664,
						       ipu0_axibus_ctrl_show,
						       ipu0_axibus_ctrl_store);
static struct driver_attribute sifr_axibus_ctrl    = __ATTR(sifr, 0664,
						       sifr_axibus_ctrl_show,
						       sifr_axibus_ctrl_store);
static struct driver_attribute pym_axibus_ctrl    = __ATTR(pym, 0664,
						       pym_axibus_ctrl_show,
						       pym_axibus_ctrl_store);
static struct driver_attribute iar_axibus_ctrl    = __ATTR(iar, 0664,
						       iar_axibus_ctrl_show,
						       iar_axibus_ctrl_store);

static struct driver_attribute all_axibus_ctrl    = __ATTR(all, 0664,
						       all_axibus_ctrl_show,
						       all_axibus_ctrl_store);

static struct attribute *axibus_ctrl_attrs[] = {
	&iar_axibus_ctrl.attr,
	&pym_axibus_ctrl.attr,
	&sifr_axibus_ctrl.attr,
	&ipu0_axibus_ctrl.attr,
	&gdc1_axibus_ctrl.attr,
	&t21_axibus_ctrl.attr,
	&gdc0_axibus_ctrl.attr,
	&isp0m2_axibus_ctrl.attr,
	&isp0m1_axibus_ctrl.attr,
	&isp0m0_axibus_ctrl.attr,
	&sifw_axibus_ctrl.attr,
	&all_axibus_ctrl.attr,
	NULL,
};
static struct attribute_group axibus_group = {
	.name = "axibus_ctrl",
	.attrs = axibus_ctrl_attrs,
};

#endif

static struct attribute_group read_attr_group = {
	.name = "read_qos_ctrl",
	.attrs = read_qctrl_attrs,
};
static struct attribute_group write_attr_group = {
	.name = "write_qos_ctrl",
	.attrs = write_qctl_attrs,
};



static const struct attribute_group *ddr_attr_groups[] = {
	&read_attr_group,
	&write_attr_group,
#ifdef CONFIG_HOBOT_XJ3
	&axibus_group,
#endif
	NULL,
};
struct bus_type ddr_monitor_subsys = {
	.name = "ddr_monitor",
};

static int ddr_ecc_enable_irq(void *ddrc_base, int enable)
{
	u32 *ecc_ctrl;

	if (IS_ERR_OR_NULL(ddrc_base)) {
		pr_err("invalid ddrc base address\n");
		return -EINVAL;
	}

	ecc_ctrl = ddrc_base + uMCTL2_ECCCTL;
	if (enable) {
		writel(readl(ecc_ctrl) | ECC_UNCORR_ERR_CLR, ecc_ctrl);
		writel(readl(ecc_ctrl) | ECC_CORR_ERR_CLR, ecc_ctrl);

		writel(readl(ecc_ctrl) | ECC_CORR_ERR_INTR_EN, ecc_ctrl);
		writel(readl(ecc_ctrl) | ECC_UNCORR_ERR_INTR_EN, ecc_ctrl);
	} else {
		writel(readl(ecc_ctrl) & ~ECC_CORR_ERR_INTR_EN, ecc_ctrl);
		writel(readl(ecc_ctrl) & ~ECC_UNCORR_ERR_INTR_EN, ecc_ctrl);

		writel(readl(ecc_ctrl) | ECC_UNCORR_ERR_CLR, ecc_ctrl);
		writel(readl(ecc_ctrl) | ECC_CORR_ERR_CLR, ecc_ctrl);
	}

	return 0;
}

static ssize_t ddr_ecc_stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t len = 0;

	if (ddr_ecc.enabled == 0) {
		len += snprintf(buf+len, 64, "ddr ecc is not enabled\n");
	} else {
		len += snprintf(buf+len, 64, "ddr ecc is enabled\n");

		len += snprintf(buf+len, 64, "1 bit corrected error hits %llu times\n",
				ddr_ecc.corr_cnt);

		len += snprintf(buf+len, 64, "2 bits uncorrected error hits %llu times\n",
				ddr_ecc.uncorr_cnt);

		len += snprintf(buf+len, 64, "3+ bits uncertain error hits %llu times\n",
				ddr_ecc.uncorr_corr_cnt);
	}

	return len;
}

static DEVICE_ATTR_RO(ddr_ecc_stat);

static int ddr_ecc_panic_handler(struct notifier_block *this,
        unsigned long event, void *ptr)
{
	if (ddr_ecc.enabled != 0) {
		pr_err("ddr ecc is enabled.\n");

		pr_err("1 bit corrected error hits %llu times\n",
		               ddr_ecc.corr_cnt);
		pr_err("2 bits uncorrected error hits %llu times\n",
		               ddr_ecc.uncorr_cnt);
		pr_err("3+ bits uncertain error hits %llu times\n",
		               ddr_ecc.uncorr_corr_cnt);
	}

	return NOTIFY_DONE;
}


static int ddr_monitor_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *pres;
	void __iomem *ddrc_base = NULL;
	struct devfreq_event_desc *desc;
	u32 reg_val;

	pr_debug("\n");

	g_ddr_monitor_dev = devm_kmalloc(&pdev->dev, sizeof(struct ddr_monitor_dev_s), GFP_KERNEL);
	if (!g_ddr_monitor_dev) {
		return -ENOMEM;
	}
	memset(g_ddr_monitor_dev, 0, sizeof(struct ddr_monitor_dev_s));

	pres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pres) {
		devm_kfree(&pdev->dev, g_ddr_monitor_dev);
		return -ENOMEM;
	}

	g_ddr_monitor_dev->regaddr = devm_ioremap_resource(&pdev->dev, pres);
	g_ddr_monitor_dev->irq = platform_get_irq(pdev, 0);

	ret = request_irq(g_ddr_monitor_dev->irq, ddr_monitor_isr, IRQF_TRIGGER_HIGH,
			  dev_name(&pdev->dev), g_ddr_monitor_dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		return -ENODEV;
	}

#ifdef CONFIG_HOBOT_XJ3
	/* support ECC start */
	pres = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!pres) {
		pr_err("DDR controller base is not detected.\n");
		return -ENODEV;
	}

	ddrc_base = devm_ioremap_resource(&pdev->dev, pres);
	if (IS_ERR(ddrc_base)) {
		pr_err("DDRC base address map failed\n");
		return PTR_ERR(ddrc_base);
	}

	if (readl(ddrc_base + uMCTL2_ECCCFG0) & 0x7) {
		ddr_ecc.enabled = 1;
		ddr_ecc.ddrc_base = ddrc_base;
		ddr_ecc.irq = platform_get_irq(pdev, 1);
		if (ddr_ecc.irq < 0) {
			pr_info("ddr ecc irq is not in dts\n");
			return -ENODEV;
		}

		ret = request_irq(ddr_ecc.irq, ddr_ecc_isr, IRQF_TRIGGER_HIGH,
				  "ddr_ecc", &ddr_ecc);
		if (ret) {
			dev_err(&pdev->dev, "Could not request IRQ\n");
			return -ENODEV;
		}

		ddr_ecc.panic_blk.notifier_call = ddr_ecc_panic_handler;
		atomic_notifier_chain_register(&panic_notifier_list,
		                   &ddr_ecc.panic_blk);

		ddr_ecc_enable_irq(ddrc_base, 1);
	}

	if (sysfs_create_file(&pdev->dev.kobj, &dev_attr_ddr_ecc_stat.attr)) {
		pr_err("ddr_ecc_stat attr create failed\n");
		return -ENOMEM;
	}

	/* support ECC end */

	ret = ddr_mpu_init(pdev);
	if (ret < 0)
		pr_info("mpu is not supported\n");

	reg_val = readl(ddrc_base);
	if (reg_val & 0x10) {
		pr_info("DDR4 detected\n");
		rd_cmd_bytes = 32;
	} else if (reg_val & 0x20) {
		pr_info("LPDDR4 detected\n");
		rd_cmd_bytes = 64;
	} else {
		pr_err("Can't detected DDR type\n");
	}
#endif

	desc = devm_kzalloc(&pdev->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	desc->ops = &hobot_dfi_ops;
	desc->driver_data = g_ddr_monitor_dev;
	desc->name = pdev->dev.of_node->name;
	g_ddr_monitor_dev->desc = desc;

	g_ddr_monitor_dev->edev = devm_devfreq_event_add_edev(&pdev->dev, desc);
	if (IS_ERR(g_ddr_monitor_dev->edev)) {
		dev_err(&pdev->dev,
			"failed to add devfreq-event device %d\n",
			PTR_ERR(g_ddr_monitor_dev->edev));

		return PTR_ERR(g_ddr_monitor_dev->edev);
	}

	g_ddr_monitor_dev->res_vaddr = kmalloc(TOTAL_RESULT_SIZE, GFP_KERNEL);
	if (g_ddr_monitor_dev->res_vaddr == NULL) {
		dev_err(&pdev->dev, "memory alloc fail\n");
		return -1;
	}
	platform_set_drvdata(pdev, g_ddr_monitor_dev);
	ddr_monitor_debuginit();
	ddr_monitor_cdev_create();
	init_waitqueue_head(&g_ddr_monitor_dev->wq_head);
	spin_lock_init(&g_ddr_monitor_dev->lock);

#ifdef CONFIG_HOBOT_XJ3
	writel(0x2021100, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	writel(0x2021100, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
#else
	writel(0x21100, g_ddr_monitor_dev->regaddr + DDR_PORT_READ_QOS_CTRL);
	writel(0x21100, g_ddr_monitor_dev->regaddr + DDR_PORT_WRITE_QOS_CTRL);
#endif
	g_ddr_monitor_dev->sif_mclk = devm_clk_get(&pdev->dev, "sif_mclk");
	g_ddr_monitor_dev->ddr_mclk = devm_clk_get(&pdev->dev, "ddr_mclk");
	if (g_ddr_monitor_dev->ddr_mclk == NULL) {
		pr_err("failed to get ddr_mclk");
		return -ENODEV;
	}
	pr_info("ddr monitor init finished.");

	return ret;
}

static int ddr_monitor_remove(struct platform_device *pdev)
{
	pr_info("\n");

	ddr_monitor_dev_remove();
	devm_kfree(&pdev->dev, g_ddr_monitor_dev);
	g_ddr_monitor_dev = NULL;

	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_ddr_ecc_stat.attr);
	ddr_mpu_deinit(pdev);

	return 0;
}

static const struct of_device_id ddr_monitor_match[] = {
	{.compatible = "hobot,x2-ddrmonitor"},
	{.compatible = "hobot,ddrmonitor"},
	{}
};

MODULE_DEVICE_TABLE(of, ddr_monitor_match);

static struct platform_driver ddr_monitor_driver = {
	.probe	= ddr_monitor_probe,
	.remove = ddr_monitor_remove,
	.driver = {
		.name	= "ddr_monitor",
		.of_match_table = ddr_monitor_match,
		.groups = ddr_attr_groups,
	},
};

module_platform_driver(ddr_monitor_driver);

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("ddr performance monitor module");
MODULE_LICENSE("GPL");
