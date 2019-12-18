/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
/**
 * @author   Jesse.Huang (Jesse.Huang@hobot.cc)
 * @date     2019/11/27
 * @version  V1.0
 * @par      Horizon Robotics
 */

#ifndef DRIVERS_IPS_USRDRV_INC_PROCFS_MACRO_H_
#define DRIVERS_IPS_USRDRV_INC_PROCFS_MACRO_H_

#define PROCFS_WRITE(name, func) \
	static ssize_t usrdrv_ ## name ## _write(struct file *file, \
	const char __user *ubuf, size_t count, loff_t *ppos)  \
	 { \
		if(copy_from_user(g_msg, ubuf, count) != 0) return -1; \
		/*echo will append extra '\n'*/\
		if(g_msg[count-1] == '\n')g_msg[count-1]='\0'; \
		printk("%s: %s", __FUNCTION__, g_msg); \
	/*need return count, if not, remain char will enter this function again*/\
		return func(g_regs, g_msg) ? count : -1; \
	 }
#define PROCFS_READ(name, func) \
	static ssize_t usrdrv_ ## name ## _read(struct file *file, \
	char __user *ubuf, size_t count, loff_t *ppos) \
	 { \
		printk("%s\n", __FUNCTION__); \
		func(); \
		return 0; \
	 }
#define PROCFS_DECLARE(name, rf, wf) \
	PROCFS_READ(name, rf) \
	PROCFS_WRITE(name, wf) \
	static struct proc_dir_entry *usrdrv_ ## name ## _ent; \
	static struct file_operations usrdrv_ ## name = { \
		.owner = THIS_MODULE, \
		.read = usrdrv_ ## name ## _read, \
		.write = usrdrv_ ## name ## _write, \
	};
#define PROCFS_CREATE(name) \
	usrdrv_ ## name ## _ent = proc_create("usrdrv_"#name, 0777, \
	NULL, &usrdrv_ ## name);
#define PROCFS_REMOVE(name) \
	proc_remove(usrdrv_ ## name ## _ent);

#endif // DRIVERS_IPS_USRDRV_INC_PROCFS_MACRO_H_
