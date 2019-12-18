#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/device.h>
#include <linux/compiler.h>
#include <linux/slab.h>
#include <soc/hobot/x2_ips.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/eventpoll.h>
#include "ipu_slot_dual.h"
#include "ipu_dev.h"
#include "ipu_dual.h"
#include "ipu_drv.h"
#include "ipu_common.h"
#include <asm-generic/io.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>

#include "ipu_feed_back.h"

#define IPU_IOC_MAGIC	'm'

#define IPUC_FB_INIT			_IOW(IPU_IOC_MAGIC, 0, ipu_init_t)
#define IPUC_FB_GET_DONE_INFO	_IOR(IPU_IOC_MAGIC, 1, struct img_info_t)
#define IPUC_FB_GET_ERR_STATUS	_IOR(IPU_IOC_MAGIC, 2, uint32_t)
#define IPUC_FB_START			_IO(IPU_IOC_MAGIC, 3)
#define IPUC_FB_STOP			_IO(IPU_IOC_MAGIC, 4)
#define IPUC_FB_MANUAL_PROCESS	_IO(IPU_IOC_MAGIC, 5)

#define X2_IPU_FEED_BACK_NAME	"x2-ipu-feedback"
#define IPU_FEED_BACK_SIZE		0x2000000
#define IPU_FEED_BACK_BASE		0x10000000
#define PYM_ERR_STATUS			0x87654321
#define PYM_DONE_STATUS			0x12345678
#define IPU_MEM_4k				4096

struct ipu_f_cdev {
	const char *name;
	struct x2_ipu_data *ipu;
	struct class *class;
	struct cdev cdev;
	dev_t dev_num;
	spinlock_t slock;
	wait_queue_head_t event_head;
	bool pymid_done;
	bool stop;
	uint32_t err_status;
};
static struct ipu_f_cdev *g_ipu_f_cdev;
static uint32_t g_ipu_status;
static struct img_info_t g_img_info;

void ipu_feed_back_mode_process(uint32_t status)
{
	ipu_dbg("fb mode process status = %x\n", status);
	spin_lock(&g_ipu_f_cdev->slock);
	if (status & PYM_DS_FRAME_DROP || status & PYM_US_FRAME_DROP) {

		g_ipu_status = PYM_ERR_STATUS;
		g_ipu_f_cdev->err_status = status;
		wake_up_interruptible(&g_ipu_f_cdev->event_head);
		ipu_err("fb pym error irq 0x%x\n", g_ipu_f_cdev->err_status);
	}
	if (status & PYM_FRAME_DONE) {
		g_ipu_status = PYM_DONE_STATUS;
		wake_up_interruptible(&g_ipu_f_cdev->event_head);
		ipu_dbg("fb pym done irq\n");
	}
	spin_unlock(&g_ipu_f_cdev->slock);
}

static int8_t ipu_set_ddr(uint32_t ddrbase, ipu_cfg_t *ipu)
{
	uint32_t w = 0, h = 0;
	uint32_t size = 0;
	uint32_t i = 0;
	uint32_t limit = ddrbase + IPU_FEED_BACK_SIZE;

	/* calculate pymid src space */
	ddrbase = ALIGN(ddrbase, IPU_MEM_4k);
	w = ipu->pymid.ds_roi[0].w;
	h = ipu->pymid.ds_roi[0].h;
	size = w * h;
	ipu->crop_ddr.y_addr = ddrbase;
	ddrbase = ddrbase + size;
	ipu->crop_ddr.c_addr = ddrbase;
	ddrbase += size >> 1;
	ddrbase = ALIGN(ddrbase, IPU_MEM_4k);
	if (ddrbase >= limit)
		goto err_out;
	set_ds_src_addr(ipu->crop_ddr.y_addr, ipu->crop_ddr.c_addr);

	/* calculate pymid ds space */
	for (i = 0; i <= ipu->pymid.ds_layer_en; i++) {
		if (i % 4 != 0 && ipu->pymid.ds_factor[i] == 0) {
			/* if factor == 0, bypass layer */
			ipu->ds_ddr[i].y_addr = 0;
			ipu->ds_ddr[i].c_addr = 0;
			continue;
		}
		w = ALIGN_16(ipu->pymid.ds_roi[i].w);
		h = ipu->pymid.ds_roi[i].h;
		size = w * h;
		ipu->ds_ddr[i].y_addr = ddrbase;
		ddrbase = ddrbase + size;
		if (ipu->pymid.ds_uv_bypass & (1 << i)) {
			/* uv bypass layer won't write to ddr */
			ipu->ds_ddr[i].c_addr = 0;
		} else {
			ipu->ds_ddr[i].c_addr = ddrbase;
			ddrbase += size >> 1;
		}
		ddrbase = ALIGN(ddrbase, IPU_MEM_4k);
		ipu_dbg("pym_ds %d %d %d %d %d 0x%llx 0x%llx",
		i, w, h, ipu->pymid.ds_roi[i].w, ipu->pymid.ds_roi[i].h,
		ipu->ds_ddr[i].y_addr, ipu->ds_ddr[i].c_addr);
		if (ddrbase >= limit)
			goto err_out;
		set_ds_layer_addr(i, ipu->ds_ddr[i].y_addr,
		ipu->ds_ddr[i].c_addr);
	}
	/* calculate pymid us space */
	for (i = 0; i < 6; i++) {
		if (!(ipu->pymid.us_layer_en & 1 << i)) {
			/* layer disable */
			ipu->us_ddr[i].y_addr = 0;
			ipu->us_ddr[i].c_addr = 0;
			continue;
		}
		w = ALIGN_16(ipu->pymid.us_roi[i].w);
		h = ipu->pymid.us_roi[i].h;
		size = w * h;
		ipu->us_ddr[i].y_addr = ddrbase;
		ddrbase = ddrbase + size;
		if (ipu->pymid.us_uv_bypass & 1 << i) {
			/* uv bypass layer won't write to ddr */
			ipu->us_ddr[i].c_addr = 0;
		} else {
			ipu->us_ddr[i].c_addr = ddrbase;
			ddrbase += size >> 1;
		}
		ddrbase = ALIGN(ddrbase, IPU_MEM_4k);
		if (ddrbase >= limit)
			goto err_out;
		set_us_layer_addr(i, ipu->us_ddr[i].y_addr,
						ipu->us_ddr[i].c_addr);
	}
	return 0;
err_out:
	return -1;
}

static int8_t ipu_feed_back_init(ipu_cfg_t *ipu_cfg)
{
	ipu_dbg("%s\n", __func__);
	ips_module_reset(RST_IPU);
	ipu_set(IPUC_SET_BASE, ipu_cfg, 0);
	ipu_set(IPUC_SET_PYMID, ipu_cfg, 0);
	ipu_set(IPUC_SET_FRAME_ID, ipu_cfg, 0);
	ipu_set_ddr(IPU_FEED_BACK_BASE, ipu_cfg);
	memset(&g_img_info, 0, sizeof(struct img_info_t));
	{
	int i;

	g_img_info.slot_id = 0;
	g_img_info.frame_id = 0;
	g_img_info.timestamp = 0;
	g_img_info.img_format = 0;
	g_img_info.ds_pym_layer = ipu_cfg->pymid.ds_layer_en;
	g_img_info.us_pym_layer = ipu_cfg->pymid.us_layer_en;
	g_img_info.src_img.width = ipu_cfg->pymid.ds_roi[0].w;
	g_img_info.src_img.height = ipu_cfg->pymid.ds_roi[0].h;
	g_img_info.src_img.step = ALIGN_16(ipu_cfg->pymid.ds_roi[0].w);
	g_img_info.src_img.y_paddr = (uint8_t *)ipu_cfg->crop_ddr.y_addr;
	g_img_info.src_img.c_paddr = (uint8_t *)ipu_cfg->crop_ddr.c_addr;
	ipu_info("s_w = %d\n", g_img_info.src_img.width);
	ipu_info("s_h = %d\n", g_img_info.src_img.height);
	ipu_info("s_s = %d\n", g_img_info.src_img.step);
	ipu_info("s_py = %p\n", g_img_info.src_img.y_paddr);
	ipu_info("s_pc = %p\n", g_img_info.src_img.c_paddr);
	/* TBD */
	g_img_info.src_img.y_vaddr = (uint8_t *)ipu_cfg->crop_ddr.y_addr;
	g_img_info.src_img.c_vaddr = (uint8_t *)ipu_cfg->crop_ddr.c_addr;
	for (i = 0; i <= ipu_cfg->pymid.ds_layer_en; i++) {
		if (i == 0 || ipu_cfg->pymid.ds_factor[i]) {
			g_img_info.down_scale[i].width =
					ipu_cfg->pymid.ds_roi[i].w;
			g_img_info.down_scale[i].height =
					ipu_cfg->pymid.ds_roi[i].h;
			g_img_info.down_scale[i].step =
					ALIGN_16(ipu_cfg->pymid.ds_roi[i].w);
			g_img_info.down_scale[i].y_paddr =
					(uint8_t *)ipu_cfg->ds_ddr[i].y_addr;
			g_img_info.down_scale[i].c_paddr =
					(uint8_t *)ipu_cfg->ds_ddr[i].c_addr;
			/* TBD */
			g_img_info.down_scale[i].y_vaddr =
					(uint8_t *)ipu_cfg->ds_ddr[i].y_addr;
			g_img_info.down_scale[i].c_vaddr =
					(uint8_t *)ipu_cfg->ds_ddr[i].c_addr;
		}
	}
	for (i = 0; i < 6; i++) {
		g_img_info.up_scale[i].width =
				ipu_cfg->pymid.us_roi[i].w;
		g_img_info.up_scale[i].height =
				ipu_cfg->pymid.us_roi[i].h;
		g_img_info.up_scale[i].step =
				ALIGN_16(ipu_cfg->pymid.us_roi[i].w);
		g_img_info.up_scale[i].y_paddr =
				(uint8_t *)ipu_cfg->ds_ddr[i].y_addr;
		g_img_info.up_scale[i].c_paddr =
				(uint8_t *)ipu_cfg->ds_ddr[i].c_addr;
		/* TBD */
		g_img_info.up_scale[i].y_vaddr =
				(uint8_t *)ipu_cfg->ds_ddr[i].y_addr;
		g_img_info.up_scale[i].c_vaddr =
				(uint8_t *)ipu_cfg->ds_ddr[i].c_addr;
	}
	}
	return 0;
}

static int8_t ipu_feed_back_start(ipu_cfg_t *ipu_cfg)
{
	ipu_dbg("%s\n", __func__);
	ips_irq_enable(IPU_INT);
	g_ipu->stop = false;
	return 0;
}

static int8_t ipu_feed_back_stop(ipu_cfg_t *ipu_cfg)
{
	ipu_dbg("%s\n", __func__);
	ips_irq_disable(IPU_INT);
	return 0;
}

int ipu_feed_back_open(struct inode *node, struct file *filp)
{
	struct ipu_f_cdev *ipu_cdev = NULL;
	int ret = 0;

	if (!g_ipu->ion_cnt) {
		ret = ipu_ion_alloc();
		if (ret < 0) {
			pr_err("[%s] %d:ipu ion alloc fail\n", __func__, __LINE__);
			return -EFAULT;
		}
		g_ipu->ion_cnt = 1;
	} else {
		pr_err("[%s] %d: ion have been alloc\n", __func__, __LINE__);
	}
	ipu_dbg("%s\n", __func__);
	ipu_cdev = container_of(node->i_cdev, struct ipu_f_cdev, cdev);
	filp->private_data = ipu_cdev;
	ipu_cdev->ipu->ipu_mode = IPU_FEED_BACK;
	return 0;
}

long ipu_feed_back_ioctl(struct file *filp, unsigned int cmd,
						unsigned long data)
{
	struct ipu_f_cdev *ipu_cdev = filp->private_data;
	struct x2_ipu_data *ipu = ipu_cdev->ipu;
	int ret = 0;
	struct img_info_t *img_info = NULL;
	ipu_cfg_t *ipu_cfg = (ipu_cfg_t *)ipu->cfg;

	ipu_dbg("ipu cmd: %d\n", _IOC_NR(cmd));
	switch (cmd) {
	case IPUC_FB_INIT:
		{
			ret = copy_from_user((void *)ipu_cfg,
			(const void __user *)data, sizeof(ipu_init_t));
			if (ret) {
				ipu_err("ioctl init fail\n");
				return -EFAULT;
			}
			ret = ipu_feed_back_init(ipu_cfg);
			if (ret < 0) {
				ipu_err("ioctl init fail\n");
				return -EFAULT;
			}
			ret = 0;
		}
		break;
	case IPUC_FB_GET_ERR_STATUS:
		{
		ret = copy_to_user((void __user *)data,
		(const void *)&g_ipu_f_cdev->err_status, sizeof(uint32_t));
			if (ret) {
				ipu_err("ioctl get err fail\n");
				return -EFAULT;
			}
			g_ipu_f_cdev->err_status = 0;
		}
		break;
	case IPUC_FB_GET_DONE_INFO:
		{
			img_info = &g_img_info;
			ret = copy_to_user((void __user *)data,
			(const void *)img_info, sizeof(struct img_info_t));
			if (ret)
				ipu_err("copy to user fail\n");
		}
		break;
	case IPUC_FB_STOP:
		{
			// ret = copy_from_user((void *)ipu_cfg,
			//(const void __user *)data, sizeof(ipu_cfg_t));
			ipu_feed_back_stop(NULL);
		}
		break;
	case IPUC_FB_START:
		{
			// ret = copy_from_user((void *)ipu_cfg,
			//(const void __user *)data, sizeof(ipu_cfg_t));
			// ipu_set_ddr(ddrbase, ipu_cfg_t *ipu);
			ipu_feed_back_start(NULL);
		}
		break;
	case IPUC_FB_MANUAL_PROCESS:
		{
			// ret = copy_from_user((void *)ipu_cfg,
			//(const void __user *)data, sizeof(ipu_cfg_t));
			// ipu_set_ddr(ddrbase, ipu_cfg_t *ipu);
			pym_manual_start();
		}
		break;
	default:
		ipu_err("ipu cmd: %d not supported\n", _IOC_NR(cmd));
		ret = -EINVAL;
		break;
	}
	ipu_dbg("ipu cmd: %d end\n", _IOC_NR(cmd));
	return ret;
}

int ipu_feed_back_mmap(struct file *filp, struct vm_area_struct *vma)
{
	uint64_t offset = vma->vm_pgoff << PAGE_SHIFT;

	if (!offset)
		offset = g_ipu->paddr;
	ipu_info("ipu mmap offset: 0x%llx\n", offset);
	if ((vma->vm_end - vma->vm_start) > g_ipu->memsize)
		return -ENOMEM;
	vma->vm_flags |= VM_IO;
	vma->vm_flags |= VM_LOCKED;
	if (remap_pfn_range(vma, vma->vm_start, offset >> PAGE_SHIFT,
	vma->vm_end - vma->vm_start, pgprot_noncached(vma->vm_page_prot))) {
		ipu_err("ipu mmap fail\n");
		return -EAGAIN;
	}

	ipu_info("ipu mmap ok\n");
	return 0;
}

unsigned int ipu_feed_back_poll(struct file *file,
					struct poll_table_struct *wait)
{
	unsigned int mask = 0;

	ipu_dbg("fb poll g_ipu_status = %x\n", g_ipu_status);
	poll_wait(file, &g_ipu_f_cdev->event_head, wait);
	if (g_ipu_f_cdev->stop) {
		mask = EPOLLHUP;
		ipu_dbg("ipu exit request\n");
	} else if (g_ipu_status == PYM_ERR_STATUS) {
		ipu_info("POLLERR:0x%x\n", g_ipu_f_cdev->err_status);
		mask = EPOLLERR;
		g_ipu_status = 0;
	} else if (g_ipu_status == PYM_DONE_STATUS) {
		mask = EPOLLIN | EPOLLET;
		g_ipu_status = 0;
		ipu_dbg("fb EPOLLIN\n");
		return mask;
	}
	return mask;
}

int ipu_feed_back_close(struct inode *inode, struct file *filp)
{
	struct ipu_f_cdev *ipu_cdev = filp->private_data;

	ipu_cdev->ipu->ipu_mode = IPU_INVALID;
	 ipu_drv_stop();
	return 0;
}

static const struct file_operations ipu_feed_back_fops = {
	.owner = THIS_MODULE,
	.mmap = ipu_feed_back_mmap,
	.open = ipu_feed_back_open,
	.poll = ipu_feed_back_poll,
	.release = ipu_feed_back_close,
	.unlocked_ioctl = ipu_feed_back_ioctl,
	.compat_ioctl = ipu_feed_back_ioctl,
};

static int __init x2_ipu_feed_back_init(void)
{
	int ret = 0;
	struct device *dev = NULL;

	g_ipu_f_cdev = kmalloc(sizeof(struct ipu_f_cdev), GFP_KERNEL);
	g_ipu_status = 0;
	g_ipu_f_cdev->name = X2_IPU_FEED_BACK_NAME;
	g_ipu_f_cdev->ipu = g_ipu;
	g_ipu_f_cdev->err_status = 0;
	g_ipu_f_cdev->stop = 0;
	g_ipu_f_cdev->class = class_create(THIS_MODULE, X2_IPU_FEED_BACK_NAME);
	alloc_chrdev_region(&g_ipu_f_cdev->dev_num, 0, 1, "ipu-feedback");
	cdev_init(&g_ipu_f_cdev->cdev, &ipu_feed_back_fops);
	cdev_add(&g_ipu_f_cdev->cdev, g_ipu_f_cdev->dev_num, 1);
	dev = device_create(g_ipu_f_cdev->class,
			NULL, g_ipu_f_cdev->dev_num, NULL, "ipu-feedback");
	if (IS_ERR(dev)) {
		ret  = -EINVAL;
		ipu_err("ipu device create fail\n");
	}
	init_waitqueue_head(&g_ipu_f_cdev->event_head);
	spin_lock_init(&g_ipu_f_cdev->slock);
	g_ipu->ipu_handle[IPU_FEED_BACK] = ipu_feed_back_mode_process;
	return ret;
}

static void __exit x2_ipu_feed_back_exit(void)
{
	device_destroy(g_ipu_f_cdev->class, g_ipu_f_cdev->dev_num);
	unregister_chrdev_region(g_ipu_f_cdev->dev_num, 1);
	class_destroy(g_ipu_f_cdev->class);
}

module_init(x2_ipu_feed_back_init);
module_exit(x2_ipu_feed_back_exit);

MODULE_DESCRIPTION("Driver for X2 IPU Feed Back");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL");
