/*
*						COPYRIGHT NOTICE
*				Copyright 2019 Horizon Robotics, Inc.
*						All rights reserved.
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/unistd.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/kthread.h>

#include <soc/hobot/diag.h>
#include "./scheduler/inc/a53_stl_arrays.h"
#include "./scheduler/inc/a53_stl_global_defs.h"
#include "./scheduler/inc/a53_stl.h"
#include "./shared/ARMv8/inc/v8_aarch64.h"


#define CPU_CAL_TEST_KTHREAD_NAME  	"cpu_cal_test_kthread"
#define ERR_EVENT_BASE (0x400)
#define CFG_MODE_UNUSED      (0u)
// Trigger the input ID test failure
// default -1.should be >=1 && <=22
static int err_test_id = -1;
module_param(err_test_id, int, 0644);
MODULE_PARM_DESC(err_test_id,
	"fault injection,default -1.should be >=1 && <=22");

struct cpu_cal_test_data {
        struct task_struct *cpu_cal_test_tsk;
        int cpu_cal_test_kthread_end;
};

struct cpu_cal_test_data cpu_cal_testdata;
static int cpu_cal_test_major;
static struct cdev cpu_cal_test_cdev;
static struct class  *cpu_cal_test_class;
static struct device *cpu_cal_test_dev;
static a53_stl_state_t stlRegs;
static char test_id_no_fp[] = {1, 2, 3, 4, 5,
								6, 8, 9, 10,
								11, 12, 13,
								14, 15, 34,
								35, 36, 37,
								38, 39, 40};
static char total_test_num = 0;
uint32_t cpu_cal_test_init(void)
{
	uint32_t result;
	uint32_t *base;
	result = A53_STL_RET_OK;
	base = (uint32_t *) &stlRegs;
	stlRegs.fctlr = 0;
	stlRegs.fpir  = 0;
	stlRegs.ffmri = 0;
	// STL configuration
	result = A53_STL_init(CFG_MODE_UNUSED, base);
	return result;
}
static int align_is_en(void)
{
	int ret = 0;
	__asm volatile (
		"mrs x25, SCTLR_EL1\n\t"
        "mov %0, x25\n\t"
		:"=g"(ret)
	);
//	printk("%s %x\n",__func__,ret);
	return ret;
}
static void cpu_cal_diag_report(uint16_t err_id)
{
	if(err_id >= 1 && err_id <= total_test_num + 1) {
		diag_send_event_stat_and_env_data(
							   DiagMsgPrioHigh,
							   ModuleDiag_cpu_cal,
							   err_id,
							   DiagEventStaFail,
							   DiagGenEnvdataWhenErr,
							   NULL,
							   0);
	}
}
static int cpu_cal_test_kthread(void *data)
{
	int i = 0;
	uint32_t result;
	uint16_t err_event_id = 0;
	struct cpu_cal_test_data *cpu_cal_data = (struct cpu_cal_test_data *)data;
	result = cpu_cal_test_init();
	if(result != A53_STL_RET_OK || err_test_id == 1) {
		err_test_id = -1;
		cpu_cal_diag_report(EventIdCpuCalTestInitErr);
		return 0;
	}
	while (1) {
		if (cpu_cal_data->cpu_cal_test_kthread_end)
		       break;
		for(i = 0; i >= 0 && i < total_test_num; i++) {
			result = A53_STL_setParam(
									A53_STL_RT_MODE,
									test_id_no_fp[i] - 1,
									test_id_no_fp[i] - 1,
									A53_STL_EL0_EL1);
			if(result == A53_STL_RET_OK) {
				local_irq_disable();
				result = A53_STL();
				local_irq_enable();
			}
			if(result != A53_STL_RET_OK || err_test_id == i + 2) {
				err_test_id = -1;
				err_event_id = i + 2;
				cpu_cal_diag_report(err_event_id);
			}
		}
		msleep(100);
	}
}

static int cpu_cal_test_open(struct inode *inode, struct file *filp)
{
	cpu_cal_testdata.cpu_cal_test_kthread_end = 0;
	cpu_cal_testdata.cpu_cal_test_tsk = kthread_run(cpu_cal_test_kthread,
								&cpu_cal_testdata, CPU_CAL_TEST_KTHREAD_NAME);
	return 0;
}


static int cpu_cal_test_release(struct inode *inode, struct file *filp)
{
	cpu_cal_testdata.cpu_cal_test_kthread_end = 1;
	kthread_stop(cpu_cal_testdata.cpu_cal_test_tsk);
	return 0;
}


static long cpu_cal_test_ioctrl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct timeval tv_temp;
	int	retval = 0;
	int copied = 0;

	switch (cmd) {
	default:
	break;
	}
	return retval;
}

static const struct file_operations cpu_cal_test_fops = {
	.owner =	THIS_MODULE,
	.unlocked_ioctl = cpu_cal_test_ioctrl,
	.open =		cpu_cal_test_open,
	.release =	cpu_cal_test_release,
};
static void cpu_cal_test_dev_err(int err_flag)
{
	switch (err_flag) {
	case 4:
		device_destroy(cpu_cal_test_class, MKDEV(cpu_cal_test_major, 0));
	case 3:
		class_destroy(cpu_cal_test_class);
	case 2:
		cdev_del(&cpu_cal_test_cdev);
	}
	pr_err("cpu_cal dev init fail %d\n", err_flag);
}

static int cpu_cal_test_dev_init
	(struct platform_device *pdev, dev_t devno)
{
	int ret = 0, err_flag = 0;
	struct cdev  *p_cdev = &cpu_cal_test_cdev;
	cpu_cal_test_major = MAJOR(devno);
	cdev_init(p_cdev, &cpu_cal_test_fops);
	p_cdev->owner = THIS_MODULE;
	ret = cdev_add(p_cdev, devno, 1);
	if (ret) {
		err_flag = 1;
		goto err;
	}
	cpu_cal_test_class = class_create(THIS_MODULE, "cpu_cal_test");
	if (IS_ERR(cpu_cal_test_class)) {
		err_flag = 2;
//		ret = PTR_ERR(a53_test_class);
		goto err;
	}
	cpu_cal_test_dev = device_create(cpu_cal_test_class, NULL,
		MKDEV(cpu_cal_test_major, 0), NULL, "cpu_cal_test");
	if (IS_ERR(cpu_cal_test_dev)) {
		err_flag = 3;
//		ret = PTR_ERR(a53_test_dev);
		goto err;
	}
	return err_flag;
err:
	cpu_cal_test_dev_err(err_flag);
	return err_flag;
}
static int cpu_cal_diag_init(void)
{
	struct diag_register_info cpu_cal_info;
	int i = 0, ret = 0;
	cpu_cal_info.module_id = (uint8_t)ModuleDiag_cpu_cal;
	cpu_cal_info.event_cnt = total_test_num + 1;
	for(i = 0; i < total_test_num + 1; ++i) {
		cpu_cal_info.event_handle[i].event_id = (uint8_t)(i + 1);
		cpu_cal_info.event_handle[i].min_snd_ms = 100;
		cpu_cal_info.event_handle[i].max_snd_ms = 148;
		cpu_cal_info.event_handle[i].cb = NULL;
	}
	ret = diagnose_register(&cpu_cal_info);
	if (ret < 0) {
		pr_err("cpu_cal diagnose register fail\n");
	}
	return ret;
}
static int cpu_cal_test_probe(struct platform_device *pdev)
{
	int	ret = 0, err_flag = 0;
	dev_t	devno;
	int	status = -ENXIO;
	cpu_cal_test_major = 0;
	ret = alloc_chrdev_region(&devno, 0, 1, "cpu_cal_test");
	if (ret < 0) {
		err_flag = 1;
		goto alloc_chrdev_error;
	}
	ret = cpu_cal_test_dev_init(pdev, devno);
	if (ret < 0) {
		err_flag = 2;
		goto cpu_cal_test_dev_init_error;
	}
	total_test_num = sizeof(test_id_no_fp) / sizeof(test_id_no_fp[0]);
	ret = cpu_cal_diag_init();
	if (ret < 0) {
		err_flag = 3;
		goto diag_init_err;
	}
	return 0;
diag_init_err:
	cpu_cal_test_dev_err(4);
cpu_cal_test_dev_init_error:
	unregister_chrdev_region(MKDEV(cpu_cal_test_major, 0), 1);
alloc_chrdev_error:
	pr_err("cpu_cal init failed %d_%d\n", err_flag, ret);
	return -1;
}
static int cpu_cal_test_remove(struct platform_device *pdev)
{
	device_destroy(cpu_cal_test_class, MKDEV(cpu_cal_test_major, 0));
	class_destroy(cpu_cal_test_class);
	cdev_del(&cpu_cal_test_cdev);
	unregister_chrdev_region(MKDEV(cpu_cal_test_major, 0), 1);
	return 0;
}

static const struct of_device_id cpu_cal_test_of_match[] = {
	{.compatible = "cpu_cal_test"},
	{},
};
static struct platform_driver cpu_cal_test_driver = {
	.probe	  = cpu_cal_test_probe,
	.remove   = cpu_cal_test_remove,
	.driver   = {
			.owner	= THIS_MODULE,
			.name	= "cpu_cal_test",
			.of_match_table = cpu_cal_test_of_match,
	},
};
static int cpu_cal_test_module_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&cpu_cal_test_driver);
	if (ret)
		pr_err("register cpu_cal_test_driver error\n");
	return ret;
}
static void cpu_cal_test_module_exit(void)
{
	platform_driver_unregister(&cpu_cal_test_driver);
}
MODULE_LICENSE("Dual BSD/GPL");
module_init(cpu_cal_test_module_init);
module_exit(cpu_cal_test_module_exit);
