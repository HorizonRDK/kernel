/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/reset.h>
#include <linux/pinctrl/consumer.h>
#include <linux/suspend.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/poll.h>
#include <linux/eventpoll.h>
#include <linux/printk.h>
#include <linux/ion.h>

// #include "../notify/hb_notify.h"
#include "hb_vcam.h"

#define VCAM_DEV_NAME		"vcam"
#define VCAM_IOC_MAGIC		'v'

#define HB_VCAM_INIT		_IO(VCAM_IOC_MAGIC, 0)
#define HB_VCAM_DEINIT		_IO(VCAM_IOC_MAGIC, 1)
#define HB_VCAM_START		_IO(VCAM_IOC_MAGIC, 2)
#define HB_VCAM_STOP		_IO(VCAM_IOC_MAGIC, 3)
#define HB_VCAM_NEXT_GROUP	_IO(VCAM_IOC_MAGIC, 4)
#define HB_VCAM_FRAME_DONE	_IO(VCAM_IOC_MAGIC, 5)
#define HB_VCAM_FREE_IMG	_IO(VCAM_IOC_MAGIC, 6)
#define HB_VCAM_GET_IMG		_IO(VCAM_IOC_MAGIC, 7)
#define HB_VCAM_MEMINFO		_IO(VCAM_IOC_MAGIC, 8)
#define HB_VCAM_CLEAN		_IO(VCAM_IOC_MAGIC, 9)

#define VCAM_GROUP_FREE	0
#define VCAM_GROUP_BUSY	1

#define VCAM_GROUP_MAX	4
#define VCAM_SLOT_MAX	8
#define VCAM_SLOT_SIZE	0x200000
#define VCAM_MEM_64K	0x10000

#define VCAM_FRAME_DONE_NOTIFY	1
#define VCAM_FRAME_FREE_NOTIFY	2

#define GROUP_BASE(base, id) (base + id * VCAM_SLOT_SIZE * VCAM_SLOT_MAX)

static struct vcam_ctx_t g_vcam_ctx;			// main struct
static struct hb_vcam_msg_t g_vcam_msg[VCAM_GROUP_MAX];
static int  g_slot_free_cnt[VCAM_GROUP_MAX];	// for count vio free times
static uint64_t g_addr[VCAM_GROUP_MAX][VCAM_SLOT_MAX];
static int g_slot_flag[VCAM_GROUP_MAX];
static struct hb_vcam_msg_t g_get_info;
static int g_enable_ipu;
static int g_ion_flag;				// to produce ion alloc once
static int g_vcam_next_flag;		// for weak up next_frame
static int g_vcam_get_flag;			// for weak up frame done

static struct vcam_to_ipu_t vcam_to_ipu(struct hb_vcam_msg_t *vcam)
{
	struct vcam_to_ipu_t to_ipu = {0};
	int size;

	if (vcam == NULL) {
		pr_err("vcam to ipu fail %d\n", __LINE__);
		// return -EFAULT;
	}
	size = vcam->slot_info.img_info.width * vcam->slot_info.img_info.heigh;
	to_ipu.g_id = vcam->group_info.g_id;
	to_ipu.cam_id = vcam->slot_info.cam_id;
	to_ipu.frame_id = vcam->slot_info.frame_id;
	to_ipu.slot_id = vcam->slot_info.slot_id;
	to_ipu.timestamp = vcam->slot_info.timestamp;
	to_ipu.src_img.width = vcam->slot_info.img_info.width;
	to_ipu.src_img.height = vcam->slot_info.img_info.heigh;
	to_ipu.src_img.step = vcam->slot_info.img_info.stride;
	to_ipu.src_img.y_paddr = vcam->group_info.base + (vcam->slot_info.slot_id * g_vcam_msg[0].group_info.slot_size);
	to_ipu.src_img.c_paddr = to_ipu.src_img.y_paddr + size;

	return to_ipu;
}

/* to get group id from ddr info */
static int addr_to_gid(struct addr_info_t *addr_info)
{
	int g_id;

	if (addr_info == NULL) {
		pr_err(" get free slot is null!!\n");
		return -EFAULT;
	}
	g_id = (addr_info->y_paddr - g_vcam_ctx.p_base) / (VCAM_SLOT_SIZE * VCAM_SLOT_MAX);
	return g_id;
}

/* init vcam each slot paddr */
static int hb_vcam_buff_init(struct hb_vcam_msg_t *init_buff)
{
	int i, j;
	uint64_t base;

	base = g_vcam_ctx.p_base;
	for (i = 0; i < VCAM_GROUP_MAX; i++) {
		for (j = 0; j < VCAM_SLOT_MAX; j++)
			g_addr[i][j] = base + (i * VCAM_SLOT_MAX + j) * VCAM_SLOT_SIZE;
	}
	return 0;
}

/* init vcam group */
static int hb_vcam_group_init(struct hb_vcam_msg_t *init_group)
{
	int i, slot_size, slot_num;
	uint64_t base;

	base = g_vcam_ctx.p_base;
	slot_num = VCAM_SLOT_MAX;
	if (init_group->slot_info.img_info.format == VCAM_YUV_420_8) {

		slot_size = ALIGN(init_group->slot_info.img_info.width * init_group->slot_info.img_info.heigh * 3 / 2, VCAM_MEM_64K);
	} else if (init_group->slot_info.img_info.format == VCAM_YUV_422_8) {

		slot_size = ALIGN(init_group->slot_info.img_info.width * init_group->slot_info.img_info.heigh * 2, VCAM_MEM_64K);
	} else {

		slot_size = VCAM_SLOT_SIZE;
		pr_err("no match imgage format, user default slot size: 0x%x", slot_size);
	}
	for (i = 0; i < VCAM_GROUP_MAX; i++) {
			g_vcam_msg[i].group_info.g_id = i;
			g_vcam_msg[i].group_info.base = GROUP_BASE(base, i);
			g_vcam_msg[i].group_info.flag = VCAM_GROUP_FREE;
			g_vcam_msg[i].group_info.slot_size = slot_size;
			g_vcam_msg[i].group_info.slot_num = slot_num;
			g_vcam_msg[i].slot_info.slot_id = 0;
			g_slot_free_cnt[i] = VCAM_SLOT_MAX;
			g_slot_flag[i] = 0;
			pr_err("%s %d base[%d]: 0x%llx size[%d]: %d\n", __func__, __LINE__, i, g_vcam_msg[i].group_info.base, i, g_vcam_msg[i].group_info.slot_size);
	}
	return 0;
}

/* for notify each group free times */
static int hb_vcam_free_notify(struct notifier_block *this, unsigned long event, void *ptr)
{
	struct addr_info_t ipu;
	int g_id;

	if (ptr == NULL) {
		pr_err("free frame fail %d\n", __LINE__);
		return -EFAULT;
	}
	memcpy(&ipu, ptr, sizeof(struct addr_info_t));
	g_id = addr_to_gid(&ipu);
	g_slot_free_cnt[g_id]++;
	if (g_slot_free_cnt[g_id] == VCAM_SLOT_MAX)
		g_vcam_msg[g_id].group_info.flag = VCAM_GROUP_FREE;
	return 0;
}

static struct notifier_block notify_from_vio = {
	.notifier_call = hb_vcam_free_notify,
};

/* get a free group for next group */
static int hb_vcam_get_next_group(struct hb_vcam_msg_t *next_group)
{
	int i;

	if (next_group == NULL) {
		pr_err("get next group fail %d\n", __LINE__);
		return -EFAULT;
	}
	for (i = 0; i < VCAM_GROUP_MAX; i++) {
		if (g_vcam_msg[i].group_info.flag == VCAM_GROUP_FREE) {
			g_slot_free_cnt[i] = 0;
			memcpy(next_group, &g_vcam_msg[i], sizeof(struct hb_vcam_msg_t));
			return 0;
		}
	}
	pr_err("get next group fail %d\n", __LINE__);
	return -ENOMEM;
}

static int group_to_busy(int g_id, int slot_id)
{
	if ((g_id < 0) | (g_id >= VCAM_GROUP_MAX)) {
		pr_err("g_id %d is bad\n", g_id);
		return -EFAULT;
	}
	g_vcam_msg[g_id].group_info.flag = VCAM_GROUP_BUSY;
	g_slot_flag[g_id] |= (1 << slot_id);
	return 0;
}

static int group_to_free(int g_id, int slot_id)
{
	if ((g_id < 0) | (g_id >= VCAM_GROUP_MAX)) {
		pr_err("g_id %d is bad\n", g_id);
		return -EFAULT;
	}
	g_slot_flag[g_id] &= (~(1 << slot_id));
	if (g_slot_flag[g_id] == 0)
		g_vcam_msg[g_id].group_info.flag = VCAM_GROUP_FREE;
	return 0;
}

/*
 * vcam ion alloc
 * align 16 bate
 */
static int hb_vcam_ion_alloc(void)
{
	int ret, ion_size;
	struct vcam_ctx_t *vcam_ctx = &g_vcam_ctx;

	pr_err("vcam ion alloc start\n");

	if (!vcam_ctx->vcam_iclient) {
		pr_err("vcam ion client create failed!!\n");
		return -EFAULT;
	}
	ion_size = VCAM_GROUP_MAX * VCAM_SLOT_MAX * VCAM_SLOT_SIZE; // TBD
	vcam_ctx->vcam_ihandle = ion_alloc(vcam_ctx->vcam_iclient, ion_size, 0x10, ION_HEAP_CARVEOUT_MASK, 0);
	if (IS_ERR(vcam_ctx->vcam_ihandle)) {
		pr_err("vcam alloc ion buffer failed!!\n");
		goto ion_err;
	}
	ret = ion_phys(vcam_ctx->vcam_iclient, vcam_ctx->vcam_ihandle->id, &vcam_ctx->p_base, (size_t *)&ion_size);
	if (ret) {
		pr_err("vcam ion phys failed!!\n");
		ion_free(vcam_ctx->vcam_iclient, vcam_ctx->vcam_ihandle);
		vcam_ctx->p_base = 0;
		goto ion_err;
	}

	vcam_ctx->v_base = ion_map_kernel(vcam_ctx->vcam_iclient, vcam_ctx->vcam_ihandle);
	if (IS_ERR(vcam_ctx->v_base)) {
		pr_err("vcam ion mmap failed!!\n");
		ion_free(vcam_ctx->vcam_iclient, vcam_ctx->vcam_ihandle);
		vcam_ctx->p_base = 0;
		vcam_ctx->v_base = NULL;
		goto ion_err;
	}
	pr_err("vcam ion alloc end successful\n");
	return 0;
ion_err:
	// ion_client_destroy(vcam_ctx->vcam_iclient);
	return -ENOMEM;
}

/*
 * vcam ion free
 */
static int hb_vcam_ion_free(void)
{
	struct vcam_ctx_t *vcam_ctx = &g_vcam_ctx;

	pr_err("vcam ion free start\n");
	if (vcam_ctx->vcam_iclient) {
		if (!IS_ERR(vcam_ctx->vcam_ihandle)) {
			ion_unmap_kernel(vcam_ctx->vcam_iclient, vcam_ctx->vcam_ihandle);
			ion_free(vcam_ctx->vcam_iclient, vcam_ctx->vcam_ihandle);
		}
	}
	pr_err("vcam ion free end\n");
	return 0;
}

/*
 * not use now
 * vcam thread
 * when get slot cnt == (VCAM_SLOT_MAX - 2),need a new group;
 *
 */
#if 0
static int hb_vcam_thread(void *data)
{

	while (!kthread_should_stop()) {
		// wait_event_interruptible();
		if (g_vcam_ctx.free_group)
			wake_up_interruptible(&g_vcam_ctx.event_head);
	}
	return 0;
}
#endif
/*
 * hb_vcam_init
 * init g_vcam_ctx.vcam_msg
 * init vcam group info
 * init vcam buff info
 *
 */
static int hb_vcam_init(struct hb_vcam_msg_t *init_info)
{

	pr_err("[vcam] init %d\n", __LINE__);
	if (init_info == NULL) {
		pr_err("[vcam] %d init fail\n", __LINE__);
		return -EFAULT;
	}
	memcpy(&g_vcam_ctx.vcam_msg, init_info, sizeof(struct hb_vcam_msg_t));
	hb_vcam_group_init(init_info);
	hb_vcam_buff_init(init_info);
	pr_err("[vcam] init end %d, pbase: 0x%llx\n", __LINE__, g_vcam_ctx.p_base);
	return 0;
}

static int hb_vcam_start(struct hb_vcam_msg_t *first_group)
{
	pr_err("[vcam] start %d\n", __LINE__);
	if (first_group == NULL) {
		pr_err("vcam start fail\n");
		return -EAGAIN;
	}
	hb_vcam_get_next_group(first_group);
	pr_err("[vcam] start %d first group base 0x%llx\n", __LINE__, first_group->group_info.base);
	return 0;
}

static int hb_vcam_stop(void)
{
	/* TBD */
	g_vcam_next_flag = 0;
	g_vcam_get_flag = 0;
	pr_err("vcam stop %d\n", __LINE__);
	return 0;
}

static int hb_vcam_deinit(void)
{
	/* release vcam buff */
	g_vcam_next_flag = 0;
	g_vcam_get_flag = 0;
	pr_err("vcam deinit %d\n", __LINE__);
	return 0;
}

/*
 * vcam open
 * kzallc private_data
 * vcam ion alloc
 *
 */
static int hb_vcam_open(struct inode *inode, struct file *file)
{
	struct vcam_ctx_t *vcam_ctx;

	pr_err("vcam open start %d\n", __LINE__);
	vcam_ctx = kzalloc(sizeof(*vcam_ctx), GFP_KERNEL);
	file->private_data = vcam_ctx;
	/* ion alloc vcam buff */
	if (g_ion_flag == 0) {
		if (!hb_ion_dev) {
			pr_err("ion dev is null\n");
			return -EFAULT;
		}
		/* create vcam ion client */
		g_vcam_ctx.vcam_iclient = ion_client_create(hb_ion_dev, "hb_vcam");
		if (IS_ERR(g_vcam_ctx.vcam_iclient)) {
			pr_err("vcam ion client create failed!!\n");
			return -ENOMEM;
		}
		if (hb_vcam_ion_alloc() < 0) {
			pr_err("vcam open fail ion alloc fail\n");
			return -ENOMEM;
		}
	}
	g_ion_flag++;
	pr_err("%s %d g_ion_flag: %d\n", __func__, __LINE__, g_ion_flag);
	pr_err("vcam open end successful\n");
	return 0;
}

/*
 * vcam close
 * free private_data
 * free vcam ion
 *
 */
static int hb_vcam_close(struct inode *inode, struct file *file)
{
	/* TBD */
	struct vcam_ctx_t *vcam_ctx = file->private_data;

	kfree(vcam_ctx);
	file->private_data = NULL;
	g_vcam_next_flag = 0;
	g_vcam_get_flag = 0;
	g_ion_flag--;
	/* free vcam ion */
	if (g_ion_flag == 0) {
		hb_vcam_ion_free();
		ion_client_destroy(g_vcam_ctx.vcam_iclient);
	}
	return 0;
}

static long hb_vcam_ioctl(struct file *file, unsigned int cmd, unsigned long data)
{
	int ret = 0;
	struct hb_vcam_msg_t next_data;
	struct hb_vcam_msg_t done_data;
	struct hb_vcam_msg_t first_data;
	struct hb_vcam_msg_t init_info;
	struct vcam_to_ipu_t to_ipu;
	struct vcam_to_ipu_t free_info;
	struct vcam_to_ipu_t clean_info;
	struct vcam_to_ipu_t get_info;
	struct mem_info_t mem_info;

	switch (cmd) {
	case HB_VCAM_GET_IMG:
		ret = wait_event_interruptible_timeout(g_vcam_ctx.frame_done, g_vcam_get_flag,
												msecs_to_jiffies(5000));
		if (ret == 0) {
			pr_err("%s %d get img timeout\n", __func__, __LINE__);
			return -EFAULT;
		}
		if (ret < 0) {
			pr_err("%s %d get img fail\n", __func__, __LINE__);
			return -EFAULT;
		}
		ret = group_to_busy(g_get_info.group_info.g_id, g_get_info.slot_info.slot_id);
		if (ret < 0) {
			pr_err("%s %d get img group to busy fail\n", __func__, __LINE__);
			return -EFAULT;
		}
		g_vcam_get_flag = 0;
		get_info = vcam_to_ipu(&g_get_info);
		ret = copy_to_user((void __user *)(data), (void *)(&get_info), sizeof(struct vcam_to_ipu_t));
		if (ret < 0) {
			pr_err("%s %d get img fail\n", __func__, __LINE__);
			return -EFAULT;
		}
		break;
	case HB_VCAM_FREE_IMG:
		ret = copy_from_user((void *)(&free_info), (const void __user *)data,
							sizeof(struct vcam_to_ipu_t));
		if (ret < 0) {
			pr_err("%s %d free img fail\n", __func__, __LINE__);
			return -EFAULT;
		}
		ret = group_to_free(free_info.g_id, free_info.slot_id);
		if (ret < 0) {
			pr_err("%s %d free img fail\n", __func__, __LINE__);
			return -EFAULT;
		}
		break;
	case HB_VCAM_CLEAN:
		ret = copy_from_user((void *)(&clean_info), (const void __user *)data,
							sizeof(struct vcam_to_ipu_t));
		if (clean_info.g_id < 0 || clean_info.g_id > (VCAM_GROUP_MAX - 1)) {
			pr_err("group id not available %d\n", clean_info.g_id);
			return -EFAULT;
		} else {
			/* TBD for more special case */
			g_vcam_msg[clean_info.g_id].group_info.flag = VCAM_GROUP_FREE;
		}
		break;
	case HB_VCAM_MEMINFO:
		mem_info.base = g_vcam_ctx.p_base;
		mem_info.size = VCAM_SLOT_SIZE * VCAM_SLOT_MAX * VCAM_GROUP_MAX;
		ret = copy_to_user((void __user *)(data), (void *)(&mem_info), sizeof(struct mem_info_t));
		if (ret) {
			pr_err("[vcam] ioctl meminfo copy to user fail\n");
			return -EFAULT;
		}
		break;
	case HB_VCAM_NEXT_GROUP:
		/*  when get slot cnt == (VCAM_SLOT_MAX - 1), need a new group */
		ret = wait_event_interruptible(g_vcam_ctx.next_frame, g_vcam_next_flag);
		if (ret) {
			pr_err("[vcam] %d ioctl wake up fail\n", __LINE__);
			return -EFAULT;
		}
		g_vcam_next_flag = 0;
		ret = hb_vcam_get_next_group(&next_data);
		if (ret) {
			pr_err("[vcam] ioctl next group fail\n");
			return -EFAULT;
		}
		ret = copy_to_user((void __user *)(data), (void *)(&next_data), sizeof(struct hb_vcam_msg_t));
		if (ret) {
			pr_err("[vcam] ioctl next frame copy to user fail\n");
			return -EFAULT;
		}
		break;
	case HB_VCAM_FRAME_DONE:

		ret = copy_from_user((void *)(&done_data), (const void __user *)data, sizeof(struct hb_vcam_msg_t));
		if (ret) {
			pr_err("[vcam] ioctl frame done fail\n");
			return -EFAULT;
		}

		/* wake up frame done */
		memcpy(&g_get_info, &done_data, sizeof(struct hb_vcam_msg_t));
		g_vcam_get_flag = 1;
		wake_up_interruptible(&g_vcam_ctx.frame_done);

		/* notify others module */
		if (g_enable_ipu) {
			to_ipu = vcam_to_ipu(&done_data);
			ret = vcam_notify_frame_done(VCAM_FRAME_DONE_NOTIFY, &to_ipu);
		}

		/* wake up next group */
		if (done_data.slot_info.slot_id == (VCAM_SLOT_MAX - 2)) {
			g_vcam_next_flag = 1;
			wake_up_interruptible(&g_vcam_ctx.next_frame);
			pr_err("[vcam] %d need send next group\n", __LINE__);
		}
		break;
	case HB_VCAM_START:
		/* get first_data */
		ret = hb_vcam_start(&first_data);
		if (ret < 0) {
			pr_err("[vcam] ioctl vcam start fail %d\n", __LINE__);
			return -ENOMEM;
		}
		ret = copy_to_user((void __user *)(data), (void *)(&first_data), sizeof(struct hb_vcam_msg_t));
		if (ret) {
			pr_err("[vcam] ioctl vcam start copy to user fail %d\n", __LINE__);
			return -EFAULT;
		}
		break;
	case HB_VCAM_STOP:
		ret = hb_vcam_stop();
		if (ret) {
			pr_err("[vcam] ioctl vcam stop fail\n");
			return -EFAULT;
		}
		break;
	case HB_VCAM_INIT:
		g_enable_ipu = 0;
		ret = copy_from_user((void *)(&init_info), (const void __user *)data, sizeof(struct hb_vcam_msg_t));
		if (ret) {
			pr_err("[vcam] ioctl init copy from uesr fail %d\n", __LINE__);
			return -EFAULT;
		}
		ret = hb_vcam_init(&init_info);
		if (ret) {
			pr_err("[vcam] ioctl vcam init fail %d\n", __LINE__);
			return -EFAULT;
		}
		break;
	case HB_VCAM_DEINIT:
		ret = hb_vcam_deinit();
		if (ret) {
			pr_err("[vcam] ioctl vcam deinit fail\n");
			return -EFAULT;
		}
		break;
	default:
		pr_err("[vcam] ioctl fail cmd: %d\n", _IOC_NR(cmd));
		ret = -EFAULT;
		break;
	}
	return ret;
}

/* TBD: poll is not support now */
unsigned int hb_vcam_poll(struct file *file, struct poll_table_struct *wait)
{
	unsigned int mask = 0;

	poll_wait(file, &g_vcam_ctx.event_head, wait);
	mask = EPOLLIN | EPOLLET;
	return mask;
}

static const struct file_operations hb_vcam_fops = {
	.owner		= THIS_MODULE,
	.open		= hb_vcam_open,
	.release	= hb_vcam_close,
	.poll		= hb_vcam_poll,
	.unlocked_ioctl = hb_vcam_ioctl,
	.compat_ioctl = hb_vcam_ioctl,
};

/*
 * hb_vcam_probe()
 *
 * register vcam dev
 * init wait queue
 * create vcam ion client
 * create vcam kthread
 * register others module notify
 *
 */
static int hb_vcam_probe(struct platform_device *pdev)
{
	struct vcam_ctx_t *vcam_ctx = &g_vcam_ctx;

	pr_err("vcam probe start\n");

	/* init some global value */
	memset(vcam_ctx, 0, sizeof(struct vcam_ctx_t));
	g_ion_flag = 0;
	g_vcam_get_flag = 0;
	g_vcam_next_flag = 0;

	/* Register char driver */
	if (alloc_chrdev_region(&vcam_ctx->vcam_cdev.dev_num, 0, 1, VCAM_DEV_NAME)) {
		pr_err("[vcam] Allocate device no failed\n");
		return -EAGAIN;
	}

	/* Attatch file operation */
	cdev_init(&vcam_ctx->vcam_cdev.cdev, &hb_vcam_fops);
	vcam_ctx->vcam_cdev.cdev.owner = THIS_MODULE;

	/* Add to system */
	if (cdev_add(&vcam_ctx->vcam_cdev.cdev, vcam_ctx->vcam_cdev.dev_num, 1)) {
		pr_err("[vcam] Attatch file operation failed\n");
		unregister_chrdev_region(vcam_ctx->vcam_cdev.dev_num, 1);
		return -EAGAIN;
	}

	/* class create */
	vcam_ctx->vcam_cdev.class = class_create(THIS_MODULE, "vcam_drv");
	if (IS_ERR(vcam_ctx->vcam_cdev.class)) {
		int ret = PTR_ERR(vcam_ctx->vcam_cdev.class);

		pr_err("[vcam] Unable to create class, err = %d\n", ret);
		return ret;
	}

	/* devices create */
	vcam_ctx->vcam_cdev.device = device_create(vcam_ctx->vcam_cdev.class, NULL, vcam_ctx->vcam_cdev.dev_num, NULL, VCAM_DEV_NAME);
	if (IS_ERR(vcam_ctx->vcam_cdev.device)) {
		int ret = PTR_ERR(vcam_ctx->vcam_cdev.device);

		pr_err("[vcam] Unable to create class, err = %d\n", ret);
		return ret;
	}

	/* init wait queue */
	init_waitqueue_head(&vcam_ctx->event_head);
	init_waitqueue_head(&vcam_ctx->next_frame);
	init_waitqueue_head(&vcam_ctx->frame_done);

	#if 0
	/* vcam kthread create */
	if (vcam_ctx->vcam_task == NULL) {
		vcam_ctx->vcam_task = kthread_run(hb_vcam_thread, (void *)vcam_ctx, "hb_vcam");
		if (IS_ERR(vcam_ctx->vcam_task)) {
			int ret = PTR_ERR(vcam_ctx->vcam_task);

			vcam_ctx->vcam_task = NULL;
			pr_err("vcam kthread failed\n");
			return ret;
		}
	}
	#endif

	/* register others module notify */
	vio_register_notify(&notify_from_vio);

	pr_err("vcam probe end successful\n");
	return 0;
}

/*
 * hb_vcam_remove()
 *
 * stop vcam kthread
 * destroy vcam ion client
 * remove vcam device
 * unregister others module notify
 *
 */
static int hb_vcam_remove(struct platform_device *pdev)
{
	struct vcam_ctx_t *vcam_ctx = &g_vcam_ctx;

	pr_err("vcam remove start\n");
	/* stop vcam kthread */
	#if 0
	if (!IS_ERR(vcam_ctx->vcam_task))
		kthread_stop(vcam_ctx->vcam_task);
	vcam_ctx->vcam_task = NULL;
	#endif

	/* Release char driver */
	cdev_del(&vcam_ctx->vcam_cdev.cdev);
	unregister_chrdev_region(vcam_ctx->vcam_cdev.dev_num, 1);
	device_destroy(vcam_ctx->vcam_cdev.class, vcam_ctx->vcam_cdev.dev_num);
	class_destroy(vcam_ctx->vcam_cdev.class);

	/* unregister others module notify */
	vio_unregister_notify(&notify_from_vio);
	pr_err("vcam remove end\n");
	return 0;
}

static int __init vcam_init(void)
{
	pr_err("[vcam] init start\n");
	hb_vcam_probe(NULL);
	pr_err("[vcam] init end\n");
	return 0;
}

static void __exit vcam_exit(void)
{
	pr_err("[vcam] exit start\n");
	hb_vcam_remove(NULL);
	pr_err("[vcam] exit end\n");
}

module_init(vcam_init);
module_exit(vcam_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Horizon Robotics");
MODULE_DESCRIPTION("vcam driver");
