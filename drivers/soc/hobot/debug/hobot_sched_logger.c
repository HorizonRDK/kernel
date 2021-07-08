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

#define BUF_SIZE_KB_DEFAULT (1024)
#define BUF_SIZE_KB_LIMIT (204800)
#define BUF_REMAINING_MIN (512)
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

static char *buf[NR_CPUS] = {0};
static int pos[NR_CPUS] = {0};
static int bufsz = 0;

static int hobot_sched_store_info(struct task_struct *prev,
				struct task_struct *next)
{
	u64 ts;
	int ret, cpuid;
	unsigned long rem_nsec;

	ts = local_clock();
	rem_nsec = do_div(ts, NS_TO_SEC);
	cpuid = smp_processor_id();
	//bufsz = buf_size_kbytes << KB_TO_B_SHIFT;
	//if(!buf[cpuid] || !bufsz)
	//	return ;
	ret = snprintf(buf[cpuid] + pos[cpuid], bufsz - pos[cpuid],
		"[%5lu.%06lu] [%d] : %s(%d) -> %s(%d)\n",
		(unsigned long)ts, rem_nsec / NS_TO_US, cpuid,
		prev->comm, task_pid_nr(prev), next->comm, task_pid_nr(next));
	if (bufsz - pos[cpuid] > BUF_REMAINING_MIN)
		pos[cpuid] += ret;
	else
		pos[cpuid] = 0;

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int hobot_sched_logger_show(struct seq_file *s, void *unused)
{
	int i;

	for (i = 0; i < NR_CPUS; i++)
		seq_printf(s, "%s\n", buf[i]);

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
	if (d == NULL) {
		pr_err("Failed to create debugfs node\n");
		debugfs_remove_recursive(sched_switch_debugfs_dir);
		sched_switch_debugfs_dir = NULL;
		return -ENOMEM;
	}

	return 0;
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
	void *sched_vaddr;
	size_t sched_total_size;
	size_t sched_percpu_size;

	if (buf_size_kbytes > BUF_SIZE_KB_LIMIT)
		buf_size_kbytes = BUF_SIZE_KB_LIMIT;

	if (mode != SCHED_MODE_VADDR
		&& mode != SCHED_MODE_PSTORE) {
		mode = SCHED_MODE_VADDR;
	}

	if (mode == SCHED_MODE_VADDR) {
		bufsz = buf_size_kbytes << KB_TO_B_SHIFT;
		for (i = 0; i < NR_CPUS; i++) {
			buf[i] = (char *)vmalloc(buf_size_kbytes << KB_TO_B_SHIFT);
			if (buf[i] == NULL) {
				pr_err("Failed to vmalloc %d KB\n", buf_size_kbytes);
				return -ENOMEM;
			}
			memset(buf[i], 0, bufsz);
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
			buf[i] = (char *)(sched_vaddr + sched_percpu_size * i);
			memset(buf[i], 0, bufsz);
			pr_info("buf[%d]=%p, size=%ld\n", i, buf[i], sched_percpu_size);
		}
		atomic_set(&(prb->start), 0);
		atomic_set(&(prb->size),
				sched_total_size - sizeof(struct persistent_ram_buffer_temp));
	}

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
			if (buf[i])
				vfree(buf[i]);
		}
	}
}

module_init(hobot_sched_logger_init)
module_exit(hobot_sched_logger_exit)

MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("sched switch logger module");
MODULE_LICENSE("GPL");
