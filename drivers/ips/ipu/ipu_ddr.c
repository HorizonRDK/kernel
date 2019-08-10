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
#include <x2/x2_ips.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/eventpoll.h>
#include "ipu_slot.h"
#include "ipu_dev.h"
#include "ipu_drv.h"
#include "ipu_ddr.h"
#include "ipu_common.h"
#include <asm-generic/io.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/time.h>
#include <asm/cacheflush.h>
#include <linux/semaphore.h>
#include "../../iar/x2_iar.h"

#define IPU_IOC_MAGIC       'm'

#define IPUC_INIT		_IOW(IPU_IOC_MAGIC, 0, ipu_init_t)
#define IPUC_GET_IMG	_IOR(IPU_IOC_MAGIC, 1, int)
#define IPUC_CNN_DONE	_IO(IPU_IOC_MAGIC, 2)
#define IPUC_GET_DONE_INFO		_IOR(IPU_IOC_MAGIC, 3, info_h_t)
#define IPUC_GET_ERR_STATUS		_IOR(IPU_IOC_MAGIC, 4, uint32_t)
#define IPUC_DUMP_REG			_IO(IPU_IOC_MAGIC, 5)
#define IPUC_START				_IO(IPU_IOC_MAGIC, 6)
#define IPUC_STOP				_IO(IPU_IOC_MAGIC, 7)
#define IPUC_UPDATE_CFG			_IOW(IPU_IOC_MAGIC, 8, ipu_init_t)
#define IPUC_GET_MEM_INFO		_IOR(IPU_IOC_MAGIC, 9, ipu_meminfo_t)
#define IPUC_PYM_MANUAL_PROCESS	_IO(IPU_IOC_MAGIC, 10)
#define IPUC_SRC_DONE			_IO(IPU_IOC_MAGIC, 11)
#define IPUC_PYM_DONE			_IO(IPU_IOC_MAGIC, 12)
#define IPUC_FB_SRC_DONE			_IO(IPU_IOC_MAGIC, 13)
#define IPUC_FB_FLUSH			_IO(IPU_IOC_MAGIC, 14)

#define X2_IPU_DDR_NAME		"x2-ipu-ddr"

#define ENABLE 1
#define DISABLE 0

static wait_queue_head_t wq_frame_done;
static wait_queue_head_t wq_pym_done;
static unsigned long  runflags;
static unsigned long  pym_runflags;
static int started;
static int timeout;

struct ipu_ddr_cdev {
	const char *name;
	struct x2_ipu_data *ipu;
	struct class *class;
	struct cdev cdev;
	dev_t dev_num;
	wait_queue_head_t event_head;
	spinlock_t slock;
	bool pymid_done;
	bool stop;
	int8_t done_idx;
	uint32_t err_status;
	unsigned long ipuflags;
	slot_ddr_info_t s_info;
};

struct ipu_ddr_cdev *g_ipu_ddr_cdev;
static int64_t g_ipu_time;

/* new process */
static struct semaphore sem_src;
static struct semaphore sem_pym;
static struct src_img_info_t g_process_info;
/* new process */

/* for mipi/dvp frame id */
static uint32_t decode_frame_id(uint16_t *addr)
{
	uint32_t d = 0;
	uint8_t i = 0;

	for (i = 0; i < 8; i++) {
		if (addr[i] == 0xffff) {
			d <<= 1;
			d |= 1;
		} else {
			d <<= 1;
		}
	}
	return d;
}

/* for mipi/dvp timestamp */
static int64_t ipu_current_time(void)
{
	struct timeval tv;
	int64_t ipu_time;

	do_gettimeofday(&tv);
	ipu_time = tv.tv_sec * 1000000 + tv.tv_usec;
	return ipu_time;
}

/* for HISI bt timestamp */
static int decode_timestamp(void *addr, int64_t *timestamp)
{
	char *addrp = (char *)addr;
	char *datap = (char *)timestamp;
	int i = 0;

	memset(timestamp, 0, sizeof(int64_t));
	for (i = 15; i >= 0; i--) {
		if (i % 2)
			datap[(15 - i) / 2] |= (addrp[i] & 0x0f);
		else
			datap[(15 - i) / 2] |= ((addrp[i] & 0x0f) << 4);
	}
	return 0;
}

static int8_t ipu_get_frameid(struct x2_ipu_data *ipu, ipu_slot_h_t *slot)
{
	uint8_t *tmp = NULL;
	uint64_t vaddr = (uint64_t)IPU_GET_SLOT(slot->info_h.slot_id, ipu->vaddr);
	ipu_cfg_t *cfg = (ipu_cfg_t *)ipu->cfg;

	if (!cfg->frame_id.id_en)
		return 0;
	if (cfg->frame_id.crop_en && cfg->ctrl.crop_ddr_en) {
		/* get id from crop ddr address */
		tmp = (uint8_t *)(slot->info_h.ddr_info.crop.y_offset + vaddr);
		if (cfg->frame_id.bus_mode == 0) {
			slot->info_h.cf_timestamp = g_ipu_time;
			slot->info_h.cf_id = tmp[0] << 8 | tmp[1];
			ipu_dbg("cframe_id=%d, %d, %d\n",
			tmp[0], tmp[1], slot->info_h.cf_id);
		} else {
			decode_timestamp(tmp,
			&(slot->info_h.cf_timestamp));
			slot->info_h.cf_id = decode_frame_id((uint16_t *)tmp);
			ipu_dbg("cframe_id=%d\n", slot->info_h.cf_id);
		}
	}
	if (cfg->frame_id.scale_en) {
		if (cfg->ctrl.scale_ddr_en) {
			/* get id from scale ddr address */
			tmp = (uint8_t *)(slot->info_h.ddr_info.scale.y_offset + vaddr);
			if (cfg->frame_id.bus_mode == 0) {
				slot->info_h.sf_timestamp = g_ipu_time;
				slot->info_h.sf_id = tmp[0] << 8 | tmp[1];
				ipu_dbg("sframe_id=%d, %d, %d\n",
				tmp[0], tmp[1], slot->info_h.sf_id);
			} else {
				decode_timestamp(tmp,
				&(slot->info_h.sf_timestamp));
				slot->info_h.sf_id = decode_frame_id((uint16_t *)tmp);
				ipu_dbg("sframe_id=%d\n", slot->info_h.sf_id);
			}
		} else {
			/* get id from pymid ddr address */
			tmp = (uint8_t *)(slot->info_h.ddr_info.ds[0].y_offset + vaddr);
			if (cfg->frame_id.bus_mode == 0) {
				slot->info_h.sf_timestamp = g_ipu_time;
				slot->info_h.sf_id = tmp[0] << 8 | tmp[1];
				ipu_dbg("pframe_id=%d, %d, %d\n",
				tmp[0], tmp[1], slot->info_h.sf_id);
			} else {
				decode_timestamp(tmp,
				&(slot->info_h.sf_timestamp));
				slot->info_h.sf_id = decode_frame_id((uint16_t *)tmp);
				ipu_dbg("pframe_id=%d\n", slot->info_h.sf_id);
			}
		}
	}
	return 0;
}


void ipu_handle_frame_done(void)
{
	ipu_slot_h_t *g_slot_h;
	unsigned long flags;
	struct x2_ipu_data *ipu = g_ipu_ddr_cdev->ipu;
	ipu_slot_h_t *slot_h = NULL;
	int count;

	spin_lock_irqsave(&g_ipu_ddr_cdev->slock, flags);
	count = slot_left_num(BUSY_SLOT_LIST);
	if (count > 1)	{
		g_slot_h = slot_busy_to_done();
		if (g_slot_h) {
			runflags = 1;
			wake_up_interruptible(&wq_frame_done);
		}
	}
	spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);
}

void ipu_handle_pym_frame_done(void)
{

	ipu_slot_h_t *done_slot;

	pr_debug("ipu: ipu_handle_pym_frame_done\n");

	done_slot = ipu_read_done_slot();//TODO

	pr_debug("free slot num is %d\n", slot_left_num(FREE_SLOT_LIST));
	pr_debug("busy slot num is %d\n", slot_left_num(BUSY_SLOT_LIST));
	pr_debug("done slot num is %d\n", slot_left_num(DONE_SLOT_LIST));

	if (!test_and_set_bit(1, &pym_runflags))
		wake_up_interruptible(&wq_pym_done);

/*
 *	done_slot = ipu_read_done_slot();
 *	if (done_slot != NULL) {
 *		pr_info("ipu: done slot id is %d.\n",
 *		done_slot->info_h.slot_id);
 *		iar_set_video_buffer(g_process_info.slot_id);
 *	}
 */
	pr_debug("ipu: done slot id is %d.\n", g_process_info.slot_id);
	iar_set_video_buffer(g_process_info.slot_id);

}

void ipu_handle_frame_start(void)
{
	ipu_slot_h_t *slot_h = NULL;
	struct x2_ipu_data *ipu = g_ipu_ddr_cdev->ipu;
	unsigned long flags;

	g_ipu_time = ipu_current_time();

	if (started == 0)
		return;

	spin_lock_irqsave(&g_ipu_ddr_cdev->slock, flags);
	slot_h = slot_free_to_busy();
	if (slot_h) {
		ipu_set(IPUC_SET_DDR, ipu->cfg,
		IPU_GET_SLOT(slot_h->info_h.slot_id, ipu->paddr));
	}
	spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);

}
void ipu_ddr_mode_process(uint32_t status)
{
	struct x2_ipu_data *ipu = g_ipu_ddr_cdev->ipu;
	unsigned long flags;

	spin_lock_irqsave(&g_ipu_ddr_cdev->slock, flags);
	if (status & IPU_BUS01_TRANSMIT_ERRORS ||
		status & IPU_BUS23_TRANSMIT_ERRORS ||
		status & PYM_DS_FRAME_DROP ||
		status & PYM_US_FRAME_DROP) {
		ipu_slot_h_t *slot_h = NULL;

		g_ipu_ddr_cdev->err_status = status;
		ipu_err("ipu error 0x%x\n", g_ipu_ddr_cdev->err_status);
		slot_h = slot_busy_to_free(&g_ipu_ddr_cdev->s_info);
		if (slot_h) {
			ipu_err("meet error, slot-%d, 0x%x\n",
					slot_h->info_h.slot_id, status);
			wake_up_interruptible(&g_ipu_ddr_cdev->event_head);
		}
	}


	spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);

}

static int8_t ipu_sinfo_init(ipu_cfg_t *ipu_cfg)
{
	uint8_t i = 0;
	memset(&g_ipu_ddr_cdev->s_info, 0, sizeof(slot_ddr_info_t));
	if (ipu_cfg->ctrl.crop_ddr_en == 1) {
		g_ipu_ddr_cdev->s_info.crop.y_offset = ipu_cfg->crop_ddr.y_addr;
		g_ipu_ddr_cdev->s_info.crop.c_offset = ipu_cfg->crop_ddr.c_addr;
		g_ipu_ddr_cdev->s_info.crop.y_width = ipu_cfg->crop.crop_ed.w - ipu_cfg->crop.crop_st.w;
		g_ipu_ddr_cdev->s_info.crop.y_height = ipu_cfg->crop.crop_ed.h - ipu_cfg->crop.crop_st.h;
		g_ipu_ddr_cdev->s_info.crop.y_stride = ALIGN_16(g_ipu_ddr_cdev->s_info.crop.y_width);
		g_ipu_ddr_cdev->s_info.crop.c_width = (ipu_cfg->crop.crop_ed.w - ipu_cfg->crop.crop_st.w) >> 1;
		g_ipu_ddr_cdev->s_info.crop.c_height = ipu_cfg->crop.crop_ed.h - ipu_cfg->crop.crop_st.h;
		g_ipu_ddr_cdev->s_info.crop.c_stride = ALIGN_16(g_ipu_ddr_cdev->s_info.crop.c_width);
	}
	if (ipu_cfg->ctrl.scale_ddr_en == 1) {
		g_ipu_ddr_cdev->s_info.scale.y_offset = ipu_cfg->scale_ddr.y_addr;
		g_ipu_ddr_cdev->s_info.scale.c_offset = ipu_cfg->scale_ddr.c_addr;
		g_ipu_ddr_cdev->s_info.scale.y_width = ipu_cfg->scale.scale_tgt.w;
		g_ipu_ddr_cdev->s_info.scale.y_height = ipu_cfg->scale.scale_tgt.h;
		g_ipu_ddr_cdev->s_info.scale.y_stride = ALIGN_16(g_ipu_ddr_cdev->s_info.scale.y_width);
		g_ipu_ddr_cdev->s_info.scale.c_width = ALIGN_16(ipu_cfg->scale.scale_tgt.w >> 1);
		g_ipu_ddr_cdev->s_info.scale.c_height = ipu_cfg->scale.scale_tgt.h;
		g_ipu_ddr_cdev->s_info.scale.c_stride = ALIGN_16(g_ipu_ddr_cdev->s_info.scale.c_width);
	}
	if (ipu_cfg->pymid.pymid_en == 1) {
		for (i = 0; i <= ipu_cfg->pymid.ds_layer_en; i++) {
			if (i == 0 || ipu_cfg->pymid.ds_factor[i]) {
				g_ipu_ddr_cdev->s_info.ds[i].y_offset = ipu_cfg->ds_ddr[i].y_addr;
				g_ipu_ddr_cdev->s_info.ds[i].c_offset = ipu_cfg->ds_ddr[i].c_addr;
				g_ipu_ddr_cdev->s_info.ds[i].y_width = ipu_cfg->pymid.ds_roi[i].w;
				g_ipu_ddr_cdev->s_info.ds[i].y_height = ipu_cfg->pymid.ds_roi[i].h;
				g_ipu_ddr_cdev->s_info.ds[i].y_stride = ALIGN_16(g_ipu_ddr_cdev->s_info.ds[i].y_width);
				g_ipu_ddr_cdev->s_info.ds[i].c_width = ipu_cfg->pymid.ds_roi[i].w >> 1;
				g_ipu_ddr_cdev->s_info.ds[i].c_height = ipu_cfg->pymid.ds_roi[i].h;
				g_ipu_ddr_cdev->s_info.ds[i].c_stride = ALIGN_16(g_ipu_ddr_cdev->s_info.ds[i].c_width);
			}
		}
		for (i = 0; i < 6; i++) {
			if (ipu_cfg->pymid.us_layer_en & 1 << i) {
				if (ipu_cfg->pymid.us_factor[i]) {
					g_ipu_ddr_cdev->s_info.us[i].y_offset = ipu_cfg->us_ddr[i].y_addr;
					g_ipu_ddr_cdev->s_info.us[i].c_offset = ipu_cfg->us_ddr[i].c_addr;
					g_ipu_ddr_cdev->s_info.us[i].y_width = ipu_cfg->pymid.us_roi[i].w;
					g_ipu_ddr_cdev->s_info.us[i].y_height = ipu_cfg->pymid.us_roi[i].h;
					g_ipu_ddr_cdev->s_info.us[i].y_stride = ALIGN_16(g_ipu_ddr_cdev->s_info.us[i].y_width);
					g_ipu_ddr_cdev->s_info.us[i].c_width = ipu_cfg->pymid.us_roi[i].w >> 1;
					g_ipu_ddr_cdev->s_info.us[i].c_height = ipu_cfg->pymid.us_roi[i].h;
					g_ipu_ddr_cdev->s_info.us[i].c_stride = ALIGN_16(g_ipu_ddr_cdev->s_info.us[i].c_width);
				}
			}
		}
	}

	g_ipu_ddr_cdev->s_info.ds[0].y_offset = ipu_cfg->crop_ddr.y_addr;
	g_ipu_ddr_cdev->s_info.ds[0].c_offset = ipu_cfg->crop_ddr.c_addr;

	return 0;
}

int8_t iar_get_ipu_display_addr_ddrmode(uint32_t display_addr[][2])
{
	int i = 0;

	if (g_ipu_ddr_cdev == NULL)
		return -1;
	display_addr[0][0] = g_ipu_ddr_cdev->ipu->paddr;
	display_addr[0][1] = IPU_SLOT_SIZE;
	display_addr[1][0] = g_ipu_ddr_cdev->s_info.crop.y_offset;
	display_addr[1][1] = g_ipu_ddr_cdev->s_info.crop.c_offset;
	display_addr[2][0] = g_ipu_ddr_cdev->s_info.scale.y_offset;
	display_addr[2][1] = g_ipu_ddr_cdev->s_info.scale.c_offset;

	for (i = 0; i < 24; i++) {
		display_addr[i+3][0] = g_ipu_ddr_cdev->s_info.ds[i].y_offset;
		display_addr[i+3][1] = g_ipu_ddr_cdev->s_info.ds[i].c_offset;
	}

	for (i = 0; i < 6; i++) {
		display_addr[i+27][0] = g_ipu_ddr_cdev->s_info.us[i].y_offset;
		display_addr[i+27][1] = g_ipu_ddr_cdev->s_info.us[i].c_offset;
	}
	pr_debug("g_ipu_s_cdev->ipu.paddr = 0x%x\n", display_addr[0][0]);

	return 0;
}

static int8_t ipu_core_update(ipu_cfg_t *ipu_cfg)
{
	//ips_module_reset(RST_IPU);
	ipu_cfg_ddrinfo_init(ipu_cfg);
	ipu_set(IPUC_SET_BASE, ipu_cfg, 0);
	ipu_set(IPUC_SET_CROP, ipu_cfg, 0);
	ipu_set(IPUC_SET_SCALE, ipu_cfg, 0);
	ipu_set(IPUC_SET_PYMID, ipu_cfg, 0);
	ipu_set(IPUC_SET_FRAME_ID, ipu_cfg, 0);
	ipu_sinfo_init(ipu_cfg);
	ipu_slot_recfg(&g_ipu_ddr_cdev->s_info);
	ipu_clean_slot(&g_ipu_ddr_cdev->s_info);

	return 0;
}

static int8_t ipu_core_init(ipu_cfg_t *ipu_cfg)
{
	ips_module_reset(RST_IPU);
	ipu_cfg_ddrinfo_init(ipu_cfg);
	ipu_set(IPUC_SET_BASE, ipu_cfg, 0);
	ipu_set(IPUC_SET_CROP, ipu_cfg, 0);
	ipu_set(IPUC_SET_SCALE, ipu_cfg, 0);
	ipu_set(IPUC_SET_PYMID, ipu_cfg, 0);
	ipu_set(IPUC_SET_FRAME_ID, ipu_cfg, 0);
	ipu_sinfo_init(ipu_cfg);
	init_ipu_slot((uint64_t)g_ipu->vaddr, &g_ipu_ddr_cdev->s_info);
	return 0;
}

int ipu_ddr_open(struct inode *node, struct file *filp)
{
	struct ipu_ddr_cdev *ipu_cdev = NULL;
	started = 0;

	/* The memory alloc trigger by sys node */
	if (!g_ipu->paddr || !g_ipu->vaddr || !g_ipu->memsize) {
		ipu_err("No Memory Can Use, Makesure init the slot!!\n");
		return -ENOMEM;
	}

	ipu_dbg("ipu ddr open\n");
	ipu_cdev = container_of(node->i_cdev, struct ipu_ddr_cdev, cdev);
	filp->private_data = ipu_cdev;
	ipu_cdev->ipu->ipu_mode = IPU_DDR_SIGNLE;
	return 0;
}

long ipu_ddr_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	struct ipu_ddr_cdev *ipu_cdev = filp->private_data;
	struct x2_ipu_data *ipu = ipu_cdev->ipu;
	ipu_cfg_t *ipu_cfg = (ipu_cfg_t *)ipu->cfg;
	ipu_slot_h_t *slot_h = NULL;
	info_h_t *info = NULL;
	ipu_slot_h_t *g_get_slot_h_1st = NULL;
	info_h_t *pym_img_info = NULL;
	struct src_img_info_t *src_img_info = NULL;
	info_h_t pym_info;
	struct src_img_info_t src_info;
	int empty;
	int retry = 3;
	int ret = 0;

	ipu_dbg("ipu ddr cmd: %d\n", _IOC_NR(cmd));
	switch (cmd) {
	case IPUC_INIT:
		{
			ret = copy_from_user((void *)ipu_cfg,
			(const void __user *)data, sizeof(ipu_init_t));
			if (ret) {
				ipu_err("ioctl init fail\n");
				return -EFAULT;
			}
			timeout = ipu_cfg->timeout;
			if (timeout <= 0)
				timeout = 500;
			ret = ipu_core_init(ipu_cfg);
			if (ret < 0) {
				ipu_err("ioctl init fail\n");
				return -EFAULT;
			}
			started = 0;
			/* new process */
			sema_init(&sem_src, 0);
			sema_init(&sem_pym, 0);
			/* new process */
			ret = 0;
		}
		break;
	case IPUC_PYM_MANUAL_PROCESS:
		{
			unsigned long flags;

			src_img_info = &src_info;
			ret = copy_from_user((void *)src_img_info,
			(const void __user *)data,
			sizeof(struct src_img_info_t));
			if (ret) {
				ipu_err("ioctl process fail\n");
				return -EFAULT;
			}
			spin_lock_irqsave(&g_ipu_ddr_cdev->slock, flags);
			set_ds_src_addr(src_img_info->src_img.y_paddr, src_img_info->src_img.c_paddr);
			ipu_set(IPUC_SET_PYM_DDR, g_ipu->cfg, IPU_GET_SLOT(src_img_info->slot_id,
			g_ipu->paddr));
			memcpy(&g_process_info, src_img_info, sizeof(struct src_img_info_t));
			pym_manual_start();
			spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);
		}

		break;
	case IPUC_SRC_DONE:
	{
		int data_end;
		started = 1;
wait:
		runflags = 0;
		ret = wait_event_interruptible_timeout(wq_frame_done, runflags, msecs_to_jiffies(timeout));
		if (ret < 0) {

			pr_err("wait src err %d %d\n", __LINE__, ret);
			return -EFAULT;
		} else if (ret == 0) {

			pr_debug("wait src timeout %d %d\n", __LINE__, ret);
			return -EFAULT;
		}
		unsigned long flags;
		ipu_slot_h_t *g_get_slot_h;

		spin_lock_irqsave(&g_ipu_ddr_cdev->slock, flags);
		do {
			g_get_slot_h_1st = ipu_get_done_slot();
			if (g_get_slot_h_1st) {
				empty = is_slot_done_empty();
				if (!empty) {
					insert_slot_to_free(g_get_slot_h_1st->info_h.slot_id);
					continue;
				} else
					break;

			}

		} while (g_get_slot_h_1st);
		g_get_slot_h = g_get_slot_h_1st;
		if (!g_get_slot_h) {
			spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);

			if (retry--)
				goto wait;

			return -EFAULT;
		}

		//printk("get src done !! %d\n", g_get_slot_h->info_h.slot_id);
		info = &g_get_slot_h->info_h;
		info->base = (uint64_t)IPU_GET_SLOT(g_get_slot_h->info_h.slot_id,
		g_ipu->paddr);
		spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);
		data_end = info->ddr_info.scale.c_offset+
		    info->ddr_info.scale.c_stride * info->ddr_info.scale.c_height;
		dma_sync_single_for_cpu(NULL,
		    info->base, data_end, DMA_FROM_DEVICE);
		ipu_get_frameid(ipu, g_get_slot_h);
		ret = copy_to_user((void __user *)data, (const void *)info,
		sizeof(info_h_t));
		if (ret)
			ipu_err("copy to user fail\n");

	}
		break;

	case IPUC_FB_SRC_DONE:
	{
			unsigned long flags;
			ipu_slot_h_t *g_get_fb_slot_h;

			spin_lock_irqsave(&g_ipu_ddr_cdev->slock, flags);

			g_get_fb_slot_h = ipu_get_free_slot();
			if (!g_get_fb_slot_h) {

				g_get_fb_slot_h = ipu_get_done_slot();
				if (!g_get_fb_slot_h) {
					spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);
					return -EFAULT;
				}
				//printk("fb none free\n");
			}
			//printk("@@ get src done sema !! %d\n",g_get_slot_h->info_h.slot_id);
			info = &g_get_fb_slot_h->info_h;
			info->base = (uint64_t)IPU_GET_SLOT(g_get_fb_slot_h->info_h.slot_id,
			g_ipu->paddr);
			//printk(" ipu->paddr %x\n", g_ipu->paddr);
			spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);
			ret = copy_to_user((void __user *)data, (const void *)info,
			sizeof(info_h_t));
			if (ret)
				ipu_err("copy to user fail\n");

	}
		break;
	case IPUC_PYM_DONE:
	{
			unsigned long flags;

			ret = wait_event_interruptible_timeout(wq_pym_done, test_and_clear_bit(1, &pym_runflags), msecs_to_jiffies(timeout));
			if (ret < 0) {

				pr_err("wait pym err %d ret %d\n", __LINE__, ret);
				return -EFAULT;
			} else if (ret == 0) {

				pr_debug("wait pym timeout %d ret %d\n", __LINE__, ret);
				return -EFAULT;
			}
			spin_lock_irqsave(&g_ipu_ddr_cdev->slock, flags);
			pym_img_info = &pym_info;
			pym_img_info->slot_id = g_process_info.slot_id;
			pym_img_info->slot_flag = 0;
			pym_img_info->ipu_flag = 0;
			pym_img_info->cnn_flag = 0;
			pym_img_info->cf_id = g_process_info.frame_id;
			pym_img_info->sf_id = g_process_info.frame_id;
			pym_img_info->cf_timestamp = g_process_info.timestamp;
			pym_img_info->sf_timestamp = g_process_info.timestamp;
			pym_img_info->base = (uint64_t)IPU_GET_SLOT(g_process_info.slot_id,
			g_ipu->paddr);
			pym_img_info->ddr_info = g_ipu_ddr_cdev->s_info;
			spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);
			ret = copy_to_user((void __user *)data, (const void *)pym_img_info,
			sizeof(info_h_t));
			if (ret)
				ipu_err("copy to user fail\n");
	}
		break;
	case IPUC_GET_IMG:
		/* TODO get description */
		/* could be used read api, caused user can define length */
		//dma_transfer((void *)data, IPU_SLOT_SIZE);
		ret = IPU_SLOT_SIZE;
		break;
	case IPUC_GET_ERR_STATUS:
		ret = copy_to_user((void __user *)data, (const void *)&g_ipu_ddr_cdev->err_status,
		sizeof(uint32_t));
		g_ipu_ddr_cdev->err_status = 0;
		break;
	case IPUC_CNN_DONE:
		/* step 1 mv slot from done to free */
		{
			unsigned long flags;
			int slot_id = 0;

			ret = copy_from_user((void *)&slot_id,
							(const void __user *)data, sizeof(int));
			if (ret) {
				ipu_err("ioctl cnn done msg fail\n");
				return -EFAULT;
			}
			spin_lock_irqsave(&g_ipu_ddr_cdev->slock, flags);
			insert_slot_to_free(slot_id);
			spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);
		}
		break;
	case IPUC_GET_DONE_INFO:
		{
			unsigned long flags;

			spin_lock_irqsave(&g_ipu_ddr_cdev->slock, flags);
			slot_h = ipu_get_done_slot();
			if (!slot_h) {
				spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);
				ipu_info("done list empty\n");
				return -EFAULT;
			}
			info = &slot_h->info_h;
			info->base = (uint64_t)IPU_GET_SLOT(slot_h->info_h.slot_id, ipu->paddr);
			spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);
			ret = copy_to_user((void __user *)data, (const void *)info, sizeof(info_h_t));
			if (ret)
				ipu_err("copy to user fail\n");
		}
		break;
	case IPUC_DUMP_REG:
		ipu_dump_regs();
		break;
	case IPUC_STOP:
		{
			unsigned long flags;

			ipu_drv_stop();
			spin_lock_irqsave(&g_ipu_ddr_cdev->slock, flags);
			ipu_clean_slot(&g_ipu_ddr_cdev->s_info);
			spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);
			ipu_cdev->err_status = 0;
			wake_up_interruptible(&ipu_cdev->event_head);
			ddr_mode = 0;
		}
		break;
	case IPUC_START:
		{
			unsigned long flags;

			spin_lock_irqsave(&g_ipu_ddr_cdev->slock, flags);
			slot_h = slot_free_to_busy();
			if (slot_h)
				ipu_set(IPUC_SET_DDR, ipu_cfg, IPU_GET_SLOT(slot_h->info_h.slot_id, ipu->paddr));
			else {
				ret = EFAULT;
				spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);
				break;
			}
			spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);
			ipu_drv_start();
			ddr_mode = 1;
		}
		break;
	case IPUC_UPDATE_CFG:
		{
			unsigned long flags;

			ret = copy_from_user((void *)ipu_cfg,
								 (const void __user *)data, sizeof(ipu_init_t));
			if (ret) {
				ipu_err("ioctl update fail\n");
				return -EFAULT;
			}
			//stop
			ipu_drv_stop();
			spin_lock_irqsave(&g_ipu_ddr_cdev->slock, flags);
			wake_up_interruptible(&ipu_cdev->event_head);
			//update
			ipu_core_update(ipu_cfg);
			//restart
			slot_h = slot_free_to_busy();
			if (slot_h) {
				ipu_set(IPUC_SET_DDR, ipu_cfg, IPU_GET_SLOT(slot_h->info_h.slot_id, ipu->paddr));
			} else {
				ret = EFAULT;
				spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);
				break;
			}
			spin_unlock_irqrestore(&g_ipu_ddr_cdev->slock, flags);
			ipu_drv_start();
		}
		break;
	case IPUC_GET_MEM_INFO:
		{
			ipu_meminfo_t meminfo;

			meminfo.paddr = g_ipu->paddr;
			meminfo.memsize = g_ipu->memsize;
			ret = copy_to_user((void __user *)data,
				(const void *)&meminfo, sizeof(ipu_meminfo_t));
			if (ret)
				ipu_err("copy to user fail\n");
		}
		break;
	case IPUC_FB_FLUSH:
	{
		int data_end;
		src_img_info = &src_info;
		ret = copy_from_user((void *)src_img_info,
		(const void __user *)data,
		sizeof(struct src_img_info_t));
		dma_sync_single_for_device(NULL,
		src_img_info->src_img.y_paddr,
		src_img_info->src_img.width*src_img_info->src_img.height+
		src_img_info->src_img.width*src_img_info->src_img.height/2,
			DMA_TO_DEVICE);
	}
	break;
	default:
		ipu_err("ipu cmd: %d not supported\n", _IOC_NR(cmd));
		ret = -EINVAL;
		break;
	}
	ipu_dbg("ipu cmd: %d end ret:%d\n", _IOC_NR(cmd), ret);
	return ret;
}

int ipu_ddr_mmap(struct file *filp, struct vm_area_struct *vma)
{
	uint64_t offset = vma->vm_pgoff << PAGE_SHIFT;

	if (!offset)
		offset = g_ipu->paddr;
	ipu_info("ipu mmap offset: 0x%llx, size: %ld\n",
			offset, vma->vm_end - vma->vm_start);
	if ((vma->vm_end - vma->vm_start) > g_ipu->memsize)
		return -ENOMEM;

	vma->vm_flags |= VM_LOCKED;
	if (remap_pfn_range(vma, vma->vm_start, offset >> PAGE_SHIFT,
		vma->vm_end - vma->vm_start,
		vma->vm_page_prot)) {
		ipu_err("ipu mmap fail\n");
		return -EAGAIN;
	}
	ipu_info("ipu mmap ok\n");
	return 0;
}

unsigned int ipu_ddr_poll(struct file *file,
						struct poll_table_struct *wait)
{
	unsigned int mask = 0;

	ipu_dbg("ipu ddr poll\n");
	poll_wait(file, &g_ipu_ddr_cdev->event_head, wait);
	if (g_ipu->stop) {
		mask = EPOLLHUP;
		ipu_dbg("EPOLLHUP: ipu exit request\n");
	} else if (g_ipu_ddr_cdev->err_status) {
		mask = EPOLLERR;
		ipu_err("POLLERR: err_status 0x%x\n",
				g_ipu_ddr_cdev->err_status);
		g_ipu_ddr_cdev->err_status = 0;
	} else if (!is_slot_done_empty()) {
		mask = EPOLLIN;
		ipu_dbg("EPOLLIN\n");
	}
	return mask;
}

int ipu_ddr_close(struct inode *inode, struct file *filp)
{
	struct ipu_ddr_cdev *ipu_cdev = filp->private_data;
	started = 0;

	ipu_drv_stop();
	ipu_cdev->ipu->ipu_mode = IPU_INVALID;
	return 0;
}

static const struct file_operations ipu_ddr_fops = {
	.owner = THIS_MODULE,
	.mmap = ipu_ddr_mmap,
	.open = ipu_ddr_open,
	.poll = ipu_ddr_poll,
	.release = ipu_ddr_close,
	.unlocked_ioctl = ipu_ddr_ioctl,
};

static int __init x2_ipu_ddr_init(void)
{
	int ret = 0;
	struct device *dev = NULL;

	g_ipu_ddr_cdev = kzalloc(sizeof(struct ipu_ddr_cdev), GFP_KERNEL);
	if (!g_ipu_ddr_cdev) {
		ipu_err("Unable to alloc IPU DEV\n");
		return -ENOMEM;
	}
	if (!g_ipu) {
		pr_err("g_ipu is not allocated yet\n");
		kfree(g_ipu_ddr_cdev);
		return -ENODEV;
	}
	started = 0;

	init_waitqueue_head(&wq_frame_done);
	init_waitqueue_head(&wq_pym_done);
	/* new process */
	init_waitqueue_head(&g_ipu_ddr_cdev->event_head);
	g_ipu_time = 0;
	spin_lock_init(&g_ipu_ddr_cdev->slock);
	g_ipu_ddr_cdev->name = X2_IPU_DDR_NAME;
	g_ipu_ddr_cdev->err_status = 0;
	g_ipu_ddr_cdev->ipu = g_ipu;
	g_ipu_ddr_cdev->class = class_create(THIS_MODULE, X2_IPU_DDR_NAME);
	alloc_chrdev_region(&g_ipu_ddr_cdev->dev_num, 0, 1, "ipu-ddr");
	cdev_init(&g_ipu_ddr_cdev->cdev, &ipu_ddr_fops);
	cdev_add(&g_ipu_ddr_cdev->cdev, g_ipu_ddr_cdev->dev_num, 1);
	dev = device_create(g_ipu_ddr_cdev->class,
				NULL, g_ipu_ddr_cdev->dev_num, NULL, "ipu-ddr");
	if (IS_ERR(dev)) {
		ret = -EINVAL;
		ipu_err("ipu device create fail\n");
	}
	g_ipu->ipu_handle[IPU_DDR_SIGNLE] = ipu_ddr_mode_process;
	return ret;
}

static void __exit x2_ipu_ddr_exit(void)
{
	device_destroy(g_ipu_ddr_cdev->class, g_ipu_ddr_cdev->dev_num);
	unregister_chrdev_region(g_ipu_ddr_cdev->dev_num, 1);
	class_destroy(g_ipu_ddr_cdev->class);
	kfree(g_ipu_ddr_cdev);
}

module_init(x2_ipu_ddr_init);
module_exit(x2_ipu_ddr_exit);

MODULE_DESCRIPTION("Driver for X2 IPU Dev");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL");
