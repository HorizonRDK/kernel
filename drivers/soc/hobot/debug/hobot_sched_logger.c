/*
 * Horizon Robotics
 *
 *  Copyright (C) 2021 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/sched/clock.h>
#include <linux/sched/debug.h>
#include <linux/debugfs.h>

#define BUF_SIZE_KB_DEFAULT (2048)
#define BUF_SIZE_KB_LIMIT (204800)
#define BUF_SIZE_KB_MIN (2)
#define BUF_REMAINING_MIN (1024)
#define BUF_WRITING_POS_OFFSET (512)
#define KB_TO_B_SHIFT (10)
#define NS_TO_SEC (1000000000)
#define NS_TO_US (1000)

enum SCHED_MODE {
	SCHED_MODE_VADDR = 1,
	SCHED_MODE_PSTORE
};

struct persistent_ram_buffer_temp {
	uint32_t    sig;
	atomic_t    start;
	atomic_t    size;
	uint8_t     data[0];
};

extern void * pstore_get_sched_aera_vaddr(void);
extern size_t pstore_get_sched_aera_size(void);

static int buf_size_kbytes = BUF_SIZE_KB_DEFAULT;
module_param(buf_size_kbytes, int, 0644);
static int mode = SCHED_MODE_PSTORE;
module_param(mode, int, 0644);

static DEFINE_PER_CPU(char *, buf);
static DEFINE_PER_CPU(atomic_t, pos);
static atomic_t en;
static int bufsz = 0;

static int hobot_sched_store_info(struct task_struct *prev,
				struct task_struct *next)
{
	u64 ts;
	int ret, cpuid, p;
	unsigned long rem_nsec;

	if (!atomic_read(&en))
		return 0;
	ts = local_clock();
	rem_nsec = do_div(ts, NS_TO_SEC);
	cpuid = smp_processor_id();
	p = atomic_read(this_cpu_ptr(&pos));
	ret = snprintf(__this_cpu_read(buf) + p, bufsz - p,
		"[%5lu.%06lu] [%d] : %s(%d) -> %s(%d)\n",
		(unsigned long)ts, rem_nsec / NS_TO_US, cpuid,
		prev->comm, task_pid_nr(prev), next->comm, task_pid_nr(next));
	if (bufsz - p > BUF_REMAINING_MIN)
		atomic_add(ret, this_cpu_ptr(&pos));
	else
		atomic_set(this_cpu_ptr(&pos), 0);

	return 0;
}

int start_sched_logger(void)
{
	atomic_set(&en, 1);
	pr_notice("start sched logger\n");

	return 0;
}
EXPORT_SYMBOL(start_sched_logger);

int stop_sched_logger(void)
{
	atomic_set(&en, 0);
	pr_notice("stop sched logger\n");

	return 0;
}
EXPORT_SYMBOL(stop_sched_logger);

#ifdef CONFIG_DEBUG_FS
static int hobot_sched_logger_show(struct seq_file *s, void *unused)
{
	int cpu;

	for (cpu = 0; cpu < NR_CPUS; cpu++) {
		seq_printf(s, "%s", per_cpu(buf, cpu) +
				   atomic_read(per_cpu_ptr(&pos, cpu)) +
				   BUF_WRITING_POS_OFFSET);
		seq_printf(s, "%s\n", per_cpu(buf, cpu));
	}

	return 0;
}

static int hobot_sched_logger_open(struct inode *inode, struct file *file)
{
	return single_open(file, hobot_sched_logger_show, NULL);
}

static const struct file_operations sched_switch_debug_fops = {
	.open           = hobot_sched_logger_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int attr_enable_get(void *data, u64 *val)
{
	*val = atomic_read(&en);

	return 0;
}

static int attr_enable_set(void *data, u64 val)
{
	atomic_set(&en, (int)val);
	if (val == 0)
		pr_notice("stop sched logger via sysfs\n");
	else
		pr_notice("start sched logger via sysfs\n");

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(logger_enable_fops, attr_enable_get,
						attr_enable_set, "%llu\n");

static struct dentry *sched_switch_debugfs_dir;

static int hobot_sched_init_debugfs(void)
{
	struct dentry *d;

	d = debugfs_create_dir("sched_logger", NULL);
	if (d == NULL)
		return -ENOMEM;
	sched_switch_debugfs_dir = d;
	d = debugfs_create_file("buf", S_IRUGO, sched_switch_debugfs_dir,
			NULL, &sched_switch_debug_fops);
	if (d == NULL)
		goto fail;
	d = debugfs_create_file("enable", S_IRUSR | S_IWUSR,
			sched_switch_debugfs_dir, NULL, &logger_enable_fops);
	if (d == NULL)
		goto fail;

	return 0;
fail:
	pr_err("Failed to create debugfs node\n");
	debugfs_remove_recursive(sched_switch_debugfs_dir);
	sched_switch_debugfs_dir = NULL;

	return -ENOMEM;
}

static void hobot_sched_uninit_debugfs(void)
{
	if (sched_switch_debugfs_dir)
		debugfs_remove_recursive(sched_switch_debugfs_dir);
}
#else
static inline int hobot_sched_init_debugfs(void)
{
	return 0;
}
static inline void hobot_sched_uninit_debugfs(void)
{
}
#endif

static int __init hobot_sched_logger_init(void)
{
	int i;
	char *p = NULL;
	void *sched_vaddr;
	size_t sched_total_size;
	size_t sched_percpu_size;

	if (mode != SCHED_MODE_VADDR
		&& mode != SCHED_MODE_PSTORE) {
		mode = SCHED_MODE_VADDR;
	}
	if (mode == SCHED_MODE_VADDR) {
		if (buf_size_kbytes > BUF_SIZE_KB_LIMIT)
			buf_size_kbytes = BUF_SIZE_KB_LIMIT;
		if (buf_size_kbytes < BUF_SIZE_KB_MIN)
			buf_size_kbytes = BUF_SIZE_KB_MIN;
		pr_notice("%d ring buffer,per size %d KBytes\n",
				  NR_CPUS, buf_size_kbytes);
		bufsz = buf_size_kbytes << KB_TO_B_SHIFT;
		for (i = 0; i < NR_CPUS; i++) {
			p = (char *)vmalloc(buf_size_kbytes << KB_TO_B_SHIFT);
			if (p == NULL) {
				pr_err("Failed to vmalloc %d KB\n", buf_size_kbytes);
				return -ENOMEM;
			}
			memset(p, 0, bufsz);
			per_cpu(buf, i) = p;
		}
		pr_info("vmalloc 4 * %d KBytes buffer\n", buf_size_kbytes);
	} else {
		struct persistent_ram_buffer_temp *prb;

		sched_vaddr = pstore_get_sched_aera_vaddr();
		sched_total_size = pstore_get_sched_aera_size();
		pr_info("sched_vaddr=%p, sched_total_size=%ld\n",
					sched_vaddr, sched_total_size);

		if (!sched_vaddr || !sched_total_size) {
			return -EFAULT;
		}

		prb = (struct persistent_ram_buffer_temp *)sched_vaddr;
		mode = SCHED_MODE_PSTORE;
		/*  */
		sched_percpu_size = (sched_total_size - 64) / NR_CPUS;
		/* skip 64 Bytes */
		sched_vaddr += 64;
		bufsz = sched_percpu_size;
		for (i = 0; i < NR_CPUS; i++) {
			p = (char *)(sched_vaddr + sched_percpu_size * i);
			memset(p, 0, bufsz);
			pr_info("buf[%d]=%p, size=%ld\n", i, p, sched_percpu_size);
			per_cpu(buf, i) = p;
		}
		atomic_set(&(prb->start), 0);
		atomic_set(&(prb->size),
				sched_total_size - sizeof(struct persistent_ram_buffer_temp));
	}
	atomic_set(&en, 1);
	register_sched_logger(hobot_sched_store_info);

	hobot_sched_init_debugfs();

	pr_info("success mode=%d\n", mode);

	return 0;
}

static void __exit hobot_sched_logger_exit(void)
{
	int i;

	hobot_sched_uninit_debugfs();

	unregister_sched_logger();

	if (mode == SCHED_MODE_VADDR) {
		for (i = 0; i < NR_CPUS; i++) {
			if (per_cpu(buf, i))
				vfree(per_cpu(buf, i));
		}
	}
}

module_init(hobot_sched_logger_init)
module_exit(hobot_sched_logger_exit)

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("sched switch logger module");
MODULE_LICENSE("GPL");
