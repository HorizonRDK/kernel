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
#include "ipu_common.h"
#include <asm-generic/io.h>
#include <asm/string.h>
#include <linux/uaccess.h>

#define X2_IPU_NAME         "x2-ipu"

struct x2_ipu_data {
	unsigned char __iomem *regbase;	/* read/write[bwl] */
	unsigned char __iomem *paddr;
	uint64_t vaddr;
	uint32_t memsize;
	struct task_struct *ipu_task;
	struct class *class;
	struct completion comp;
	struct dma_chan *dma_chan;
	dma_addr_t io_phys;
	wait_queue_head_t wq_head;
	spinlock_t slock;
	wait_queue_head_t event_head;
	spinlock_t elock;
	bool trigger_isr;
	bool pymid_done;
	bool stop;
	int8_t done_idx;
	uint32_t isr_data;
	uint32_t err_status;
	int32_t major;
	ipu_cfg_t *cfg;
	void *private;
	struct resource *io_r;
};

static struct x2_ipu_data *g_ipu = NULL;

/********************************************************************
 * @brief ipu_set_ddr
 *
 * @param ipu
 * @param ddrbase physical address
 *
 * @return
 ********************************************************************/
static int8_t ipu_set_ddr(ipu_cfg_t * ipu, uint64_t ddrbase)
{
	uint32_t w = 0, h = 0;
	uint32_t size = 0;
	uint32_t i = 0;
	uint32_t limit = ddrbase + IPU_SLOT_SIZE;

	/* step0. calculate slot head info size */
	ddrbase = ALIGN_16(ddrbase);

	/* step1. calculate crop space */
	if (ipu->ctrl.crop_ddr_en == 1) {
		w = ALIGN_16(ipu->crop.crop_ed.w - ipu->crop.crop_st.w);
		h = ALIGN_16(ipu->crop.crop_ed.h - ipu->crop.crop_st.h);
		size = w * h;
		ipu->crop_ddr.y_addr = ddrbase;
		ipu->crop_ddr.c_addr = ddrbase + size;
		ipu->crop_ddr.y_size = size;
		ipu->crop_ddr.c_size = size >> 1;
		ddrbase += size * 3 >> 1;
		ddrbase = ALIGN_16(ddrbase);
		if (ddrbase >= limit)
			goto err_out;
		set_ipu_addr(0, ipu->crop_ddr.y_addr, ipu->crop_ddr.c_addr);
	}

	/* step2. calculate scale space */
	if (ipu->ctrl.scale_ddr_en == 1) {
		w = ALIGN_16(ipu->scale.scale_tgt.w);
		h = ALIGN_16(ipu->scale.scale_tgt.h);
		size = w * h;
		ipu->scale_ddr.y_addr = ddrbase;
		ipu->scale_ddr.c_addr = ddrbase + size;
		ipu->scale_ddr.y_size = size;
		ipu->scale_ddr.c_size = size >> 1;
		ddrbase += size * 3 >> 1;
		ddrbase = ALIGN_16(ddrbase);
		if (ddrbase >= limit)
			goto err_out;
		set_ipu_addr(1, ipu->scale_ddr.y_addr, ipu->scale_ddr.c_addr);
	}

	/* step3. calculate pymid ds space */
	if (ipu->pymid.pymid_en == 1) {
		for (i = 0; i < ipu->pymid.ds_layer_en; i++) {
			if (i != 0 && ipu->pymid.ds_factor[i] == 0) {
				/* if factor == 0, bypass layer */
				ipu->ds_ddr[i].y_addr = 0;
				ipu->ds_ddr[i].y_size = 0;
				ipu->ds_ddr[i].c_addr = 0;
				ipu->ds_ddr[i].c_size = 0;
				continue;
			}

			w = ALIGN_16(ipu->pymid.ds_roi[i].w);
			h = ALIGN_16(ipu->pymid.ds_roi[i].h);
			size = w * h;
			ipu->ds_ddr[i].y_addr = ddrbase;
			ipu->ds_ddr[i].y_size = size;
			if (ipu->pymid.ds_uv_bypass & (1 << i)) {
				/* uv bypass layer won't write to ddr */
				ipu->ds_ddr[i].c_addr = 0;
				ipu->ds_ddr[i].c_size = 0;
				ddrbase += size;
			} else {
				ipu->ds_ddr[i].c_addr = ddrbase + size;
				ipu->ds_ddr[i].c_size = size >> 1;
				ddrbase += size * 3 >> 1;
			}
			ddrbase = ALIGN_16(ddrbase);
			if (ddrbase >= limit)
				goto err_out;
			set_ds_layer_addr(i, ipu->ds_ddr[i].y_addr,
					  ipu->ds_ddr[i].c_addr);
		}

		/* step3.1. calculate pymid us space */
		for (i = 0; i < 6; i++) {
			if (!(ipu->pymid.us_layer_en & 1 << i)) {
				/* layer disable */
				ipu->us_ddr[i].y_addr = 0;
				ipu->us_ddr[i].y_size = 0;
				ipu->us_ddr[i].c_addr = 0;
				ipu->us_ddr[i].c_size = 0;
				continue;
			}
			w = ALIGN_16(ipu->pymid.us_roi[i].w);
			h = ALIGN_16(ipu->pymid.us_roi[i].h);
			size = w * h;
			ipu->us_ddr[i].y_addr = ddrbase;
			ipu->us_ddr[i].y_size = size;
			if (ipu->pymid.us_uv_bypass & 1 << i) {
				ipu->us_ddr[i].c_addr = ddrbase + size;
				ipu->us_ddr[i].c_size = size >> 1;
				ddrbase += size * 3 >> 1;
			} else {
				/* uv bypass layer won't write to ddr */
				ipu->us_ddr[i].c_addr = 0;
				ipu->us_ddr[i].c_size = 0;
				ddrbase += size;
			}
			ddrbase = ALIGN_16(ddrbase);
			if (ddrbase >= limit)
				goto err_out;
			set_us_layer_addr(i, ipu->us_ddr[i].y_addr,
					  ipu->us_ddr[i].c_addr);
		}
	}

	return 0;
err_out:
	return -1;
}

int8_t ipu_set(ipu_cmd_e cmd, ipu_cfg_t * ipu_cfg, uint64_t data)
{
	switch (cmd) {
	case IPUC_SET_DDR:
		ipu_set_ddr(ipu_cfg, data);
		break;
	case IPUC_SET_BASE:
		set_ipu_ctrl(&ipu_cfg->ctrl);
		set_ipu_video_size(&ipu_cfg->video_in);
		break;
	case IPUC_SET_CROP:
		set_ipu_crop(&ipu_cfg->crop);
		break;
	case IPUC_SET_SCALE:
		set_ipu_scale(&ipu_cfg->scale);
		break;
	case IPUC_SET_FRAME_ID:
		set_ipu_frame_id(&ipu_cfg->frame_id);
		break;
	case IPUC_SET_PYMID:
		set_ipu_pymid(&ipu_cfg->pymid);
		break;
	default:
		break;
	}

	return 0;
}

static uint32_t decode_frame_id(uint16_t * addr)
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

static int8_t ipu_get_frameid(struct x2_ipu_data *ipu, ipu_slot_h_t * slot)
{
	uint8_t *tmp = NULL;
	uint64_t vaddr =
	    (uint64_t) IPU_GET_SLOT(slot->info_h.slot_id, ipu->vaddr);
	ipu_cfg_t *cfg = (ipu_cfg_t *) ipu->cfg;

	if (!cfg->frame_id.id_en)
		return 0;

	if (cfg->frame_id.crop_en && cfg->ctrl.crop_ddr_en) {
		/* get id from crop ddr address */
		tmp = (uint8_t *) (slot->info_h.ddr_info.crop.y_offset + vaddr);
		if (cfg->frame_id.bus_mode == 0) {
			slot->info_h.cf_id = tmp[0] << 8 | tmp[1];
			ipu_dbg("cframe_id=%d, %d, %d\n", tmp[0], tmp[1],
				slot->info_h.cf_id);
		} else {
			slot->info_h.cf_id = decode_frame_id((uint16_t *) tmp);
			ipu_dbg("cframe_id=%d\n", slot->info_h.cf_id);
		}
	}
	if (cfg->frame_id.scale_en) {
		if (cfg->ctrl.scale_ddr_en) {
			/* get id from scale ddr address */
			tmp =
			    (uint8_t *) (slot->info_h.ddr_info.scale.y_offset +
					 vaddr);
			if (cfg->frame_id.bus_mode == 0) {
				slot->info_h.sf_id = tmp[0] << 8 | tmp[1];
				ipu_dbg("sframe_id=%d, %d, %d\n", tmp[0],
					tmp[1], slot->info_h.sf_id);
			} else {
				slot->info_h.sf_id =
				    decode_frame_id((uint16_t *) tmp);
				ipu_dbg("sframe_id=%d\n", slot->info_h.sf_id);
			}
		} else {
			/* get id from pymid ddr address */
			tmp =
			    (uint8_t *) (slot->info_h.ddr_info.ds[0].y_offset +
					 vaddr);
			if (cfg->frame_id.bus_mode == 0) {
				slot->info_h.sf_id = tmp[0] << 8 | tmp[1];
				ipu_info("pframe_id=%d, %d, %d\n", tmp[0],
					 tmp[1], slot->info_h.sf_id);
			} else {
				slot->info_h.sf_id =
				    decode_frame_id((uint16_t *) tmp);
				ipu_dbg("pframe_id=%d\n", slot->info_h.sf_id);
			}
		}
	}
	return 0;
}

static int ipu_thread(void *data)
{
	unsigned long flags;
	uint32_t status = 0;
	struct x2_ipu_data *ipu = (struct x2_ipu_data *)data;
	ipu_cfg_t *ipu_cfg = (ipu_cfg_t *) ipu->cfg;
	ipu_slot_h_t *slot_h = NULL;
	ipu_info("ipu thread run\n");
	do {
		wait_event_interruptible(ipu->wq_head,
					 ipu->trigger_isr == true);
		ipu->err_status = 0;
		ipu->pymid_done = false;
		ipu->done_idx = -1;
		if (kthread_should_stop())
			break;
		status = ipu->isr_data;
		spin_lock_irqsave(&ipu->slock, flags);

		if (status & IPU_BUS01_TRANSMIT_ERRORS ||
		    status & IPU_BUS23_TRANSMIT_ERRORS ||
		    status & PYM_DS_FRAME_DROP || status & PYM_US_FRAME_DROP) {
			ipu->err_status = status;
			ipu_err("ipu error 0x%x\n", ipu->err_status);
			slot_h = slot_busy_to_free();
			if (slot_h) {
				ipu_err("meet error, slot-%d, 0x%x\n",
					slot_h->info_h.slot_id, status);
				wake_up_interruptible(&ipu->event_head);
			}
		}

		if (status & IPU_FRAME_START) {
			slot_h = slot_free_to_busy();
			if (slot_h) {
				ipu_dbg("slot-%d, 0x%llx\n",
					slot_h->info_h.slot_id,
					(uint64_t) IPU_GET_SLOT(slot_h->info_h.
								slot_id,
								(uint64_t) ipu->
								paddr));
				ipu_set(IPUC_SET_DDR, ipu_cfg,
					IPU_GET_SLOT(slot_h->info_h.slot_id,
						     (uint64_t) ipu->paddr));
			} else {
				ipu_dbg("free slot empty\n");
			}
		}

		if (status & PYM_FRAME_DONE) {
			/* TODO need flash cache? */
			slot_h = slot_busy_to_done();
			if (slot_h) {
				ipu_info("pyramid done, slot-%d, cnt %d\n",
					 slot_h->info_h.slot_id,
					 slot_h->slot_cnt);
				ipu->pymid_done = true;
				ipu->done_idx = slot_h->info_h.slot_id;
				ipu_get_frameid(ipu, slot_h);
				wake_up_interruptible(&ipu->event_head);
			} else {
				ipu_dbg("busy slot empty\n");
			}
		}
		ipu->trigger_isr = false;
		ipu->isr_data = 0;
		spin_unlock_irqrestore(&ipu->slock, flags);
	} while (!kthread_should_stop());
	ipu_info("ipu thread exit\n");
	return 0;
}

void x2_ipu_isr(unsigned int status, void *data)
{
	struct x2_ipu_data *ipu = (struct x2_ipu_data *)data;

	ipu->isr_data = status;
	spin_lock(&ipu->slock);
	ipu->trigger_isr = true;
	spin_unlock(&ipu->slock);
	wake_up_interruptible(&ipu->wq_head);
}

static int8_t ipu_stop_thread(struct x2_ipu_data *ipu)
{
	if (!IS_ERR(ipu->ipu_task)) {
		kthread_stop(g_ipu->ipu_task);
		wake_up_interruptible(&ipu->wq_head);
	}
	ipu->ipu_task = NULL;
	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id x2_ipu_of_match[] = {
	{.compatible = "hobot,x2-ipu",},
	{}
};

MODULE_DEVICE_TABLE(of, x2_ipu_of_match);

static int8_t ipu_core_init(ipu_cfg_t * ipu_cfg)
{
	slot_ddr_info_t s_info;
	uint64_t pbase = (uint64_t) g_ipu->paddr;
	uint8_t i = 0;

	ips_module_reset(RST_IPU);
	memset(&s_info, 0, sizeof(slot_ddr_info_t));

	ipu_set(IPUC_SET_BASE, ipu_cfg, 0);
	ipu_set(IPUC_SET_CROP, ipu_cfg, 0);
	ipu_set(IPUC_SET_SCALE, ipu_cfg, 0);
	ipu_set(IPUC_SET_PYMID, ipu_cfg, 0);
	ipu_set(IPUC_SET_FRAME_ID, ipu_cfg, 0);
	ipu_set(IPUC_SET_DDR, ipu_cfg, pbase);

	if (ipu_cfg->ctrl.crop_ddr_en == 1) {
		s_info.crop.y_offset = ipu_cfg->crop_ddr.y_addr - pbase;
		s_info.crop.c_offset = ipu_cfg->crop_ddr.c_addr - pbase;
		s_info.crop.y_width =
		    ALIGN_16(ipu_cfg->crop.crop_ed.w - ipu_cfg->crop.crop_st.w);
		s_info.crop.c_width =
		    ALIGN_16((ipu_cfg->crop.crop_ed.w -
			      ipu_cfg->crop.crop_st.w) >> 1);
	}
	if (ipu_cfg->ctrl.scale_ddr_en == 1) {
		s_info.scale.y_offset = ipu_cfg->scale_ddr.y_addr - pbase;
		s_info.scale.c_offset = ipu_cfg->scale_ddr.c_addr - pbase;
		s_info.scale.y_width = ALIGN_16(ipu_cfg->scale.scale_tgt.w);
		s_info.scale.c_width =
		    ALIGN_16(ipu_cfg->scale.scale_tgt.w >> 1);
	}
	if (ipu_cfg->pymid.pymid_en == 1) {
		for (i = 0; i < ipu_cfg->pymid.ds_layer_en; i++) {
			if (ipu_cfg->ds_ddr[i].y_size) {
				s_info.ds[i].y_offset =
				    ipu_cfg->ds_ddr[i].y_addr - pbase;
				s_info.ds[i].c_offset =
				    ipu_cfg->ds_ddr[i].c_addr - pbase;
				s_info.ds[i].y_width =
				    ALIGN_16(ipu_cfg->pymid.ds_roi[i].w);
				s_info.ds[i].c_width =
				    ALIGN_16(ipu_cfg->pymid.ds_roi[i].w >> 1);
			}
		}
		for (i = 0; i < 6; i++) {
			if (ipu_cfg->pymid.us_layer_en & 1 << i) {
				if (ipu_cfg->us_ddr[i].y_size) {
					s_info.us[i].y_offset =
					    ipu_cfg->us_ddr[i].y_addr - pbase;
					s_info.us[i].c_offset =
					    ipu_cfg->us_ddr[i].c_addr - pbase;
					s_info.us[i].y_width =
					    ALIGN_16(ipu_cfg->pymid.us_roi[i].
						     w);
					s_info.us[i].c_width =
					    ALIGN_16(ipu_cfg->pymid.us_roi[i].
						     w >> 1);
				}
			}
		}
	}
	init_ipu_slot((uint64_t) g_ipu->vaddr, &s_info);

	ips_irq_disable(IPU_INT);
	ips_register_irqhandle(IPU_INT, x2_ipu_isr, (void *)g_ipu);

	return 0;
}

int ipu_open(struct inode *node, struct file *filp)
{
	return 0;
}

#if 0
static void dma_comp_func(void *completion)
{
	complete(completion);
}

int dma_transfer(void *buf, int len)
{
	struct dma_device *dma_dev;
	struct dma_async_tx_descriptor *tx = NULL;
	dma_addr_t dma_src_addr, dma_dst_addr, phys_addr;
	dma_cap_mask_t mask;
	ipu_slot_h_t *slot_h;
	dma_cookie_t cookie;
	enum dma_status status;
	int rc = 0;

	/* request dma */
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	g_ipu->dma_chan = dma_request_channel(mask, NULL, NULL);
	if (!g_ipu->dma_chan) {
		ipu_err("fail to request dma\n");
		return -EFAULT;
	} else {
		ipu_err("Using %s for DMA transfer\n",
			dma_chan_name(ipu->dma_chan));
	}

	dma_dev = g_ipu->dma_chan->device;
	phys_addr = dma_map_single(dma_dev->dev, buf, len, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dma_dev->dev, phys_addr)) {
		ipu_err("fail to dma_map_single\n");
		rc = -EFAULT;
		goto err_out;
	}

	dma_dst_addr = phys_addr;
	slot_h = ipu_get_done_slot();
	if (!slot_h) {
		rc = -ENODATA;
		goto err_out1;
	}
	dma_src_addr = virt_to_phys((void *)slot_h->membase);

	tx = dma_dev->device_prep_dma_memcpy(g_ipu->dma_chan, dma_dst_addr,
					     dma_src_addr, len,
					     DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!tx) {
		ipu_err("fail to prepare dma memory\n");
	}
	init_completion(&g_ipu->comp);
	tx->callback = dma_comp_func;
	tx->callback_param = &g_ipu->comp;

	cookie = tx->tx_submit(tx);
	if (dma_submit_error(cookie)) {
		ipu_err("fail to do dma tx\n");
		rc = -EFAULT;
		goto err_out1;
	}
	dma_async_issue_pending(g_ipu->dma_chan);
	wait_for_completion(&g_ipu->comp);

	status = dma_async_is_tx_complete(g_ipu->dma_chan, cookie, NULL, NULL);
	if (status != DMA_COMPLETE) {
		ipu_err("dma tx complete check fail\n");
		rc = -EFAULT;
		goto err_out1;
	}

err_out1:
	dma_unmap_single(dma_dev->dev, phys_addr, len, DMA_BIDIRECTIONAL);
err_out:
	dma_release_channel(g_ipu->dma_chan);
	return rc;
}
#endif

long ipu_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	ipu_cfg_t *ipu_cfg = (ipu_cfg_t *) g_ipu->cfg;
	ipu_slot_h_t *slot_h = NULL;
	info_h_t *info = NULL;
	int ret = 0;
	unsigned long flag;
	ipu_info("ipu cmd: %d\n", _IOC_NR(cmd));

	switch (cmd) {
	case IPUC_INIT:
		ret = copy_from_user((void *)ipu_cfg,
				     (const void __user *)data,
				     sizeof(ipu_init_t));
		if (ret) {
			ipu_err("ioctl init fail\n");
			return -EFAULT;
		}
		ret = ipu_core_init(ipu_cfg);
		if (ret < 0) {
			ipu_err("ioctl init fail\n");
			return -EFAULT;
		}
		ret = 0;
		break;
	case IPUC_GET_IMG:
		/* TODO get description */
		/* could be used read api, caused user can define length */
		//dma_transfer((void *)data, IPU_SLOT_SIZE);
		ret = IPU_SLOT_SIZE;
		break;
	case IPUC_GET_ERR_STATUS:
		ret =
		    copy_to_user((void __user *)data,
				 (const void *)&g_ipu->err_status,
				 sizeof(uint32_t));
		g_ipu->err_status = 0;
		break;
	case IPUC_CNN_DONE:
		/* step 1 mv slot from done to free */
		ret = 0;
		spin_lock_irqsave(&g_ipu->slock, flag);
		slot_h = slot_done_to_free();
		if (!slot_h) {
			ipu_err("no done slot exist!!!\n");
			ret = -EFAULT;
		}
		spin_unlock_irqrestore(&g_ipu->slock, flag);
		break;
	case IPUC_GET_DONE_INFO:
		spin_lock_irqsave(&g_ipu->slock, flag);
		slot_h = ipu_get_done_slot();
		if (!slot_h) {
			spin_unlock_irqrestore(&g_ipu->slock, flag);
			return -EFAULT;
		}

		if (g_ipu->done_idx != -1 &&
		    g_ipu->done_idx != slot_h->info_h.slot_id) {
			ipu_err("cnn slot delay\n");
		}
		info = &slot_h->info_h;
		info->base =
		    (uint64_t) IPU_GET_SLOT(slot_h->info_h.slot_id,
					    (uint64_t) g_ipu->paddr);
		spin_unlock_irqrestore(&g_ipu->slock, flag);
		ret =
		    copy_to_user((void __user *)data, (const void *)info,
				 sizeof(info_h_t));
		break;
	case IPUC_DUMP_REG:
		ipu_dump_regs();
		break;
	case IPUC_STOP:
		ips_irq_disable(IPU_INT);
		spin_lock_irqsave(&g_ipu->elock, flag);
		g_ipu->stop = true;
		spin_unlock_irqrestore(&g_ipu->elock, flag);
		wake_up_interruptible(&g_ipu->event_head);
		spin_lock_irqsave(&g_ipu->slock, flag);
		ipu_clean_slot();
		spin_unlock_irqrestore(&g_ipu->slock, flag);
		break;
	case IPUC_START:
		ips_irq_enable(IPU_INT);
		break;
	default:
		ipu_err("ipu cmd: %d not supported\n", _IOC_NR(cmd));
		break;
	}
	ipu_info("ipu cmd: %d end\n", _IOC_NR(cmd));
	return ret;
}

int ipu_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int8_t ret = 0;
	uint64_t offset = vma->vm_pgoff << PAGE_SHIFT;
	uint64_t vaddr = g_ipu->vaddr + (offset - (uint64_t) g_ipu->paddr);

	ipu_info("ipu mmap offset: 0x%llx, vaddr: 0x%llx\n", offset, vaddr);
	ret = remap_pfn_range(vma, vma->vm_start, virt_to_pfn(vaddr),
			      vma->vm_end - vma->vm_start, vma->vm_page_prot);
	if (ret)
		return -EAGAIN;
	ipu_info("ipu mmap ok\n");
	return 0;
}

unsigned int ipu_poll(struct file *file, struct poll_table_struct *wait)
{
	unsigned long flags;
	unsigned int mask = 0;

	poll_wait(file, &g_ipu->event_head, wait);
	spin_lock_irqsave(&g_ipu->elock, flags);
	if (g_ipu->stop) {
		mask = EPOLLHUP;
		g_ipu->stop = false;
		ipu_info("ipu exit request\n");
	} else if (g_ipu->err_status) {
		ipu_info("POLLERR: err_status 0x%x\n", g_ipu->err_status);
		mask = EPOLLERR;
	} else if (g_ipu->pymid_done) {
		mask = EPOLLIN | EPOLLET;
	}
	spin_unlock_irqrestore(&g_ipu->elock, flags);
	return mask;
}

int ipu_close(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations ipu_fops = {
	.owner = THIS_MODULE,
	.mmap = ipu_mmap,
	.open = ipu_open,
	.poll = ipu_poll,
	.release = ipu_close,
	.unlocked_ioctl = ipu_ioctl,
};

void init_test_data(ipu_cfg_t * info)
{
	info->video_in.w = 1280;
	info->video_in.h = 720;
	info->ctrl.crop_ddr_en = 1;
	info->ctrl.crop_en = 1;
	info->ctrl.scale_ddr_en = 1;
	info->ctrl.src_fmt = 0;	// from sif
	info->ctrl.to_pymid = 1;
	info->ctrl.uv_fmt = 1;

	info->crop.crop_st.w = 0;
	info->crop.crop_st.h = 0;
	info->crop.crop_ed.w = 1280;
	info->crop.crop_ed.h = 720;

	info->scale.scale_src.w = 1280;
	info->scale.scale_src.h = 720;
	info->scale.scale_tgt.w = 1280;
	info->scale.scale_tgt.h = 720;
	info->scale.step_x = 4095;
	info->scale.step_y = 4095;
	info->scale.bypass_x = 1;
	info->scale.bypass_y = 1;
	info->scale.pre_scale_x = 0;
	info->scale.pre_scale_y = 0;

	info->frame_id.bus_mode = 0;
	info->frame_id.crop_en = 0;
	info->frame_id.scale_en = 0;

	info->pymid.pymid_en = 0;
	info->pymid.src_from = 0;	//isp mode
	info->pymid.ds_layer_en = 3;
	info->pymid.ds_factor[1] = 18;
	info->pymid.ds_roi[1].l = 600;
	info->pymid.ds_roi[1].t = 250;
	info->pymid.ds_roi[1].w = 326;
	info->pymid.ds_roi[1].h = 30;
	info->pymid.ds_src_width[1] = 418;

	info->pymid.ds_factor[2] = 50;
	info->pymid.ds_roi[2].l = 300;
	info->pymid.ds_roi[2].t = 194;
	info->pymid.ds_roi[2].w = 330;
	info->pymid.ds_roi[2].h = 54;
	info->pymid.ds_src_width[2] = 394;

	info->pymid.ds_factor[3] = 12;
	info->pymid.ds_roi[3].l = 116;
	info->pymid.ds_roi[3].t = 60;
	info->pymid.ds_roi[3].w = 132;
	info->pymid.ds_roi[3].h = 92;
	info->pymid.ds_src_width[3] = 250;

	info->pymid.ds_factor[5] = 57;
	info->pymid.ds_roi[5].l = 100;
	info->pymid.ds_roi[5].t = 102;
	info->pymid.ds_roi[5].w = 80;
	info->pymid.ds_roi[5].h = 100;
	info->pymid.ds_src_width[5] = 104;

	info->pymid.us_layer_en = 5;
	info->pymid.us_roi[0].l = 782;
	info->pymid.us_roi[0].t = 0;
	info->pymid.us_roi[0].w = 48;
	info->pymid.us_roi[0].h = 356;
	info->pymid.us_src_width[0] = 38;

	info->pymid.us_roi[2].l = 200;
	info->pymid.us_roi[2].t = 186;
	info->pymid.us_roi[2].w = 516;
	info->pymid.us_roi[2].h = 160;
	info->pymid.us_src_width[2] = 260;
}

static void *ipu_vmap(phys_addr_t start, size_t size)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);
	prot = pgprot_noncached(PAGE_KERNEL);
	pages = kmalloc_array(page_count, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		pr_err("%s: Failed to allocate array for %u pages\n", __func__,
		       page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;
		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vm_map_ram(pages, page_count, -1, prot);
	kfree(pages);

	return vaddr;
}

static int x2_ipu_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct resource *res;
	struct resource r;
	struct x2_ipu_data *ipu = NULL;
	struct device_node *np = NULL;
	ipu_cfg_t *ipu_cfg = NULL;
	struct device *dev = NULL;
	//const struct of_device_id *match;

#ifdef IPU_SPT_PROC
	struct proc_dir_entry *entry = NULL;
	entry =
	    proc_create("ipu", S_IRUGO | S_IWUGO | S_IXUGO, NULL, &ipu_fops);
#endif

	ipu = devm_kzalloc(&pdev->dev, sizeof(*ipu), GFP_KERNEL);
	if (!ipu) {
		return -ENOMEM;
	}

	ipu_cfg = (ipu_cfg_t *) kzalloc(sizeof(ipu_cfg_t), GFP_KERNEL);
	if (!ipu_cfg) {
		rc = -ENOMEM;
		goto err_out;
	}

	ipu->cfg = ipu_cfg;

	/* get driver info from dts */
	//match = of_match_node(x2_ipu_of_match, pdev->dev.of_node);
	//if (match && match->data) {
	//    /* Nothing to do, maybe can be used in future */
	//}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		rc = -ENODEV;
		goto err_out;
	}
	if (!request_mem_region(res->start, resource_size(res), X2_IPU_NAME)) {
		rc = -ENOMEM;
		goto err_out;
	}
#ifndef X2_ZU3
	ipu->regbase = ioremap(res->start, resource_size(res));
	if (!ipu->regbase) {
		dev_err(&pdev->dev, "unable to map registers\n");
		rc = -ENOMEM;
		goto err_out1;
	}
#else
	ipu->regbase = (unsigned char __iomem *)res->start;
#endif
	dev_info(&pdev->dev, "ipu res regbase=0x%x, mapbase=0x%llx\n",
		 (uint32_t) res->start, (uint64_t) ipu->regbase);
	set_ipu_regbase(ipu->regbase);
	ipu->io_r = res;

	/* request memory address */
	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!np) {
		dev_err(&pdev->dev, "No %s specified\n", "memory-region");
		rc = -ENODEV;
		goto err_out2;
	}

	rc = of_address_to_resource(np, 0, &r);
	if (rc) {
		dev_err(&pdev->dev,
			"No memory address assigned to the region\n");
		goto err_out2;
	}
	ipu->paddr = (unsigned char __iomem *)r.start;
	ipu->memsize = resource_size(&r);
	//ipu->vaddr = (unsigned char __iomem *)memremap(r.start, resource_size(&r), MEMREMAP_WC);
	//ipu->vaddr = (unsigned char __iomem *)ioremap_nocache(r.start, resource_size(&r));
	ipu->vaddr = (uint64_t) ipu_vmap(r.start, ipu->memsize);
	dev_info(&pdev->dev,
		 "Allocate reserved memory, paddr: 0x%0llx, vaddr: 0x%0llx, len=0x%x\n",
		 (uint64_t) ipu->paddr, (uint64_t) ipu->vaddr, ipu->memsize);

	platform_set_drvdata(pdev, ipu);

	/* create device node */
	ipu->class = class_create(THIS_MODULE, X2_IPU_NAME);
	ipu->major = register_chrdev(0, X2_IPU_NAME, &ipu_fops);
	if (ipu->major < 0) {
		dev_err(&pdev->dev, "No major\n");
		rc = -ENODEV;
		goto err_out3;
	}
	dev =
	    device_create(ipu->class, NULL, MKDEV(ipu->major, 0), NULL, "ipu");
	if (IS_ERR(dev)) {
		rc = -EINVAL;
		dev_err(&pdev->dev, "ipu device create fail\n");
		goto err_out4;
	}

	g_ipu = ipu;
	spin_lock_init(&ipu->slock);
	spin_lock_init(&ipu->elock);
	init_waitqueue_head(&ipu->wq_head);
	init_waitqueue_head(&ipu->event_head);
	if (ipu->ipu_task == NULL) {
		ipu->ipu_task =
		    kthread_run(ipu_thread, (void *)g_ipu, "ipu_thread");
		if (IS_ERR(ipu->ipu_task)) {
			ipu->ipu_task = NULL;
			ipu_err("thread create fail\n");
			return -1;
		}
	}
	dev_info(&pdev->dev, "x2 ipu probe success\n");
	return 0;

err_out4:
	device_destroy(ipu->class, MKDEV(ipu->major, 0));
err_out3:
	class_destroy(ipu->class);
err_out2:
	clr_ipu_regbase();
	iounmap(ipu->regbase);
	ipu->io_r = NULL;
#ifndef X2_ZU3
err_out1:
	release_mem_region(res->start, resource_size(res));
#endif
err_out:
	kfree(ipu);
	return rc;
}

static int x2_ipu_remove(struct platform_device *pdev)
{
	struct x2_ipu_data *ipu = platform_get_drvdata(pdev);

	ipu_stop_thread(ipu);
	vm_unmap_ram(ipu->paddr, ipu->memsize / PAGE_SIZE);
	release_mem_region(ipu->io_r->start, resource_size(ipu->io_r));
	clr_ipu_regbase();
	iounmap(ipu->regbase);
	ipu->regbase = NULL;
	ipu->io_r = NULL;

	if (ipu->cfg)
		kfree(ipu->cfg);

	device_destroy(ipu->class, MKDEV(ipu->major, 0));
	unregister_chrdev(ipu->major, X2_IPU_NAME);
	class_destroy(ipu->class);

	if (ipu)
		devm_kfree(&pdev->dev, ipu);

	g_ipu = NULL;

	return 0;
}

static struct platform_driver x2_ipu_platform_driver = {
	.probe = x2_ipu_probe,
	.remove = x2_ipu_remove,
	.driver = {
		   .name = X2_IPU_NAME,
		   .of_match_table = x2_ipu_of_match,
		   },
};

static int __init x2_ipu_init(void)
{
	int ret = 0;

	/* Register the platform driver */
	ret = platform_driver_register(&x2_ipu_platform_driver);
	return ret;
}

static void __exit x2_ipu_exit(void)
{
	/* Unregister the platform driver */
	platform_driver_unregister(&x2_ipu_platform_driver);
}

module_init(x2_ipu_init);
module_exit(x2_ipu_exit);

MODULE_DESCRIPTION("Driver for X2 IPU Dev");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL");
