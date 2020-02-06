/*
 * watchdog test
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DEBUG
#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/sched/debug.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/bottom_half.h>
#include <uapi/linux/sched/types.h>

enum {
	MODE_IRQ_ENABLED,
	MODE_IRQ_DISABLED,
	MODE_BH_DISABLED,
	MODE_MAX
};
static u8 mode_map[NR_CPUS];

struct dentry *dent;

static int wdt_test_deadloop_kthread(void *arg)
{
	u64 cnt = 0;
	u64 lasttime, curtime;
	u8 mode = *(u8*)arg;

	switch (mode) {
	case MODE_IRQ_ENABLED:
		break;
	case MODE_IRQ_DISABLED:
		pr_info("disable irq\n");
		local_irq_disable();
		break;
	case MODE_BH_DISABLED:
		pr_info("disable bottom half\n");
		local_bh_disable();
		break;
	default:
		pr_err("invalid mode %d!", mode);
		return -1;
	}

	pr_info("trigger deadloop on cpu:%d, mode: %d, irq_disabled_state:%d, preempt_count:%08x\n",
			get_cpu(), mode, irqs_disabled(), preempt_count());

	lasttime = curtime = jiffies_to_msecs(jiffies);

	while(1) {
		curtime = jiffies_to_msecs(jiffies);
		if ((curtime - lasttime) > 1000) {
			pr_info("deadloop on cpu%d, cnt:%llu\n", get_cpu(), cnt);
			lasttime = curtime;
		}

		cnt++;
	}

	pr_err("error: shouldn't reach here!!!\n");

	return 0;
}

static void wdt_test_trigger_deadloop(u32 cpu_mask)
{
	int i;
	char name[32];
	struct task_struct *pkthread[NR_CPUS];

	pr_info("cpu_mask: %x\n", cpu_mask);

	for (i = 0; i < NR_CPUS; i++) {
		if (cpu_mask & BIT(i)) {

			snprintf(name, sizeof(name), "deadloop_%d\n", i);
			pkthread[i] = kthread_create_on_cpu(wdt_test_deadloop_kthread,
					&mode_map[i], i, name);
			if (pkthread[i] == NULL) {
				pr_err("failed to create deadloop thread\n");
			}
			wake_up_process(pkthread[i]);
		}
	}
}

static int wdt_test_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "Usage:   deadloop cpu_mask irq_disable_mask\n");
	seq_printf(s, "exmaple: deadloop 1000 1000"
						"   deadloop on cpu0 with irq disabled\n\n");
	seq_printf(s, "         deadloop 1000 2000"
						"   deadloop on cpu0 with below half disabled\n");
	seq_printf(s, "         deadloop 1100 0000"
						"   deadloop on cpu0 and cpu1 with irq enabled\n");

	return 0;
}

static int wdt_test_open(struct inode *inode, struct file *file)
{
    return single_open(file, wdt_test_show, &inode->i_private);
}

static ssize_t wdt_test_write(struct file *file, const char __user *buf, size_t size, loff_t *p)
{
#define CMD_BUF_LEN 32
	char info[CMD_BUF_LEN];
	u32  cpu_mask = 0;
	char *ptr = &info[0];
	char *token;
	int  i;

	memset(info, 0, CMD_BUF_LEN);
	size = min((u32)size, (u32)(CMD_BUF_LEN - 1));
	if (copy_from_user(info, buf, size))
		return size;

	pr_info("%s\n", info);

	/* Get arg: 'deadloop' */
	token = strsep(&ptr, " ");
	if (!token)
		return -EINVAL;

	if (strcmp(token, "deadloop"))
		return -EINVAL;

	/* Get arg: 'cpumask' */
	token = strsep(&ptr, " ");
	if (!token)
		return -EINVAL;

	for (i = 0; i < NR_CPUS; i++) {
		if (token[i] != '0' && token[i] != '1' )
			return -EINVAL;
		if (token[i] == '1')
			cpu_mask |= BIT(i);
	}

	/* Get arg: 'irqmask' */
	token = strsep(&ptr, " ");
	if (!token)
		return -EINVAL;

	for (i = 0; i < NR_CPUS; i++) {
		if(token[i] < '0')
			return -EINVAL;

		if (cpu_mask & BIT(i))
			mode_map[i] = token[i] - '0';
	}

	wdt_test_trigger_deadloop(cpu_mask);


	return size;
}

static const struct file_operations debug_fops = {
    .open = wdt_test_open,
    .read = seq_read,
    .write = wdt_test_write,
    .llseek = seq_lseek,
    .release = single_release,
};


static int __init wdt_test_init(void)
{
    dent = debugfs_create_file("wdt_test", 0600, NULL, NULL, &debug_fops);
	if (dent == NULL)
		return -EPERM;

	pr_info("wdt test init\n");

	return 0;
}

late_initcall(wdt_test_init);

static void __exit wdt_test_cleanup(void)
{
	debugfs_remove(dent);
}

module_exit(wdt_test_cleanup);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("Watchdog test driver");
MODULE_LICENSE("GPL v2");
