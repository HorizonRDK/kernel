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
#include "ipu_slot_dual.h"
#include "ipu_dev.h"
#include "ipu_dual.h"
#include "ipu_drv.h"
#include "ipu_common.h"
#include <asm-generic/io.h>
#include <asm/string.h>
#include <linux/uaccess.h>

#define ENABLE 1
#define DISABLE 0

#define IPU_IOC_MAGIC	'm'

#define IPUC_INIT			_IOW(IPU_IOC_MAGIC, 0, ipu_init_t)
#define IPUC_GET_IMG		_IOR(IPU_IOC_MAGIC, 1, int)
#define IPUC_CNN_DONE		_IO(IPU_IOC_MAGIC, 2)
#define IPUC_GET_DONE_INFO	_IOR(IPU_IOC_MAGIC, 3, info_dual_h_t)
#define IPUC_GET_ERR_STATUS	_IOR(IPU_IOC_MAGIC, 4, uint32_t)
#define IPUC_DUMP_REG		_IO(IPU_IOC_MAGIC, 5)
#define IPUC_START			_IO(IPU_IOC_MAGIC, 6)
#define IPUC_STOP			_IO(IPU_IOC_MAGIC, 7)
#define IPUC_UPDATE_CFG		_IOW(IPU_IOC_MAGIC, 8, ipu_init_t)
#define IPUC_GET_MEM_INFO	_IOR(IPU_IOC_MAGIC, 9, ipu_meminfo_t)

#define X2_IPU_DUAL_NAME	"x2-ipu-dual"

struct ipu_dual_cdev {
	const char *name;
	struct x2_ipu_data *ipu;
	void *vaddr;
	struct class *class;
	struct cdev cdev;
	dev_t dev_num;
	spinlock_t slock;
	wait_queue_head_t event_head;
	bool pymid_done;
	bool stop;
	int8_t done_idx;
	uint32_t err_status;
	unsigned long ipuflags;
	slot_ddr_info_dual_t s_info;
};
extern struct x2_ipu_data *g_ipu;

static struct ipu_dual_cdev *g_ipu_d_cdev = NULL;

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

static int8_t ipu_get_frameid(struct x2_ipu_data *ipu, ipu_slot_dual_h_t *slot)
{
	uint8_t *tmp = NULL;
	uint64_t vaddr = (uint64_t)IPU_GET_DUAL_SLOT(slot->info_h.slot_id, ipu->vaddr);
	ipu_cfg_t *cfg = (ipu_cfg_t *)ipu->cfg;

	if (!cfg->frame_id.id_en)
		return 0;
	/* get first id from pymid ddr address */
	tmp = (uint8_t *)(slot->info_h.dual_ddr_info.ds_1st[0].y_offset + vaddr);
	if (cfg->frame_id.bus_mode == 0) {
		slot->info_h.cf_id = tmp[0] << 8 | tmp[1];
		ipu_dbg("pframe_id_fst=%d, %d, %d\n", tmp[0], tmp[1], slot->info_h.cf_id);
	} else {
		slot->info_h.cf_id = decode_frame_id((uint16_t *)tmp);
		ipu_dbg("pframe_id_fst=%d\n", slot->info_h.cf_id);
	}
	/* get second id from pymid ddr address */
	tmp = (uint8_t *)(slot->info_h.dual_ddr_info.ds_2nd[0].y_offset + vaddr);
	if (cfg->frame_id.bus_mode == 0) {
		slot->info_h.sf_id = tmp[0] << 8 | tmp[1];
		ipu_dbg("pframe_id_sec=%d, %d, %d\n", tmp[0], tmp[1], slot->info_h.sf_id);
	} else {
		slot->info_h.sf_id = decode_frame_id((uint16_t *)tmp);
		ipu_dbg("pframe_id_sec=%d\n", slot->info_h.sf_id);
	}

	return 0;
}

int set_next_recv_frame(ipu_slot_dual_h_t *slot_h)
{
	if (!slot_h)
		return -1;
	slot_h->info_h.slot_flag = SLOT_RECVING;
	ipu_set(IPUC_SET_CROP_DDR, g_ipu->cfg, IPU_GET_DUAL_SLOT(slot_h->info_h.slot_id, g_ipu->paddr));
	ipu_set(IPUC_SET_SCALE_DDR, g_ipu->cfg, IPU_GET_DUAL_SLOT(slot_h->info_h.slot_id, g_ipu->paddr));
	return 0;
}

int set_next_pym_frame(ipu_slot_dual_h_t *slot_h, bool first)
{
	if (!slot_h)
		return -1;
	if (first) {
		slot_h->info_h.slot_flag = SLOT_PYM_1ST;
		ipu_set(IPUC_SET_PYM_1ST_SRC_DDR, g_ipu->cfg, IPU_GET_DUAL_SLOT(slot_h->info_h.slot_id, g_ipu->paddr));
		ipu_set(IPUC_SET_PYM_DDR, g_ipu->cfg, IPU_GET_FST_PYM_OF_SLOT(slot_h->info_h.slot_id, g_ipu->paddr));
	} else {
		slot_h->info_h.slot_flag = SLOT_PYM_2ND;
		ipu_set(IPUC_SET_PYM_2ND_SRC_DDR, g_ipu->cfg, IPU_GET_DUAL_SLOT(slot_h->info_h.slot_id, g_ipu->paddr));
		ipu_set(IPUC_SET_PYM_DDR, g_ipu->cfg, IPU_GET_SEC_PYM_OF_SLOT(slot_h->info_h.slot_id, g_ipu->paddr));
	}
	return 0;
}

void ipu_dual_mode_process(uint32_t status)
{
	struct x2_ipu_data *ipu = g_ipu_d_cdev->ipu;

	spin_lock(&g_ipu_d_cdev->slock);
	if (status & IPU_BUS01_TRANSMIT_ERRORS ||
		status & IPU_BUS23_TRANSMIT_ERRORS ||
		status & PYM_DS_FRAME_DROP ||
		status & PYM_US_FRAME_DROP) {
		g_ipu_d_cdev->err_status = status;
		ipu_err("ipu error 0x%x\n", g_ipu_d_cdev->err_status);
		pym_slot_busy_to_free();
	}

	if (status & IPU_FRAME_DONE) {
		ipu_slot_dual_h_t *slot_h = NULL;
		ipu_info("ipu done\n");
		slot_h = recv_slot_busy_to_done();
		//pym first time startup or resume pym from stop when pic ava
		if (slot_h && test_and_clear_bit(IPU_PYM_STARTUP, &g_ipu_d_cdev->ipuflags)) { //||
																					  // test_and_clear_bit(IPU_PYM_NOT_AVALIABLE, &g_ipu_d_cdev->ipuflags) ) {
			slot_h = pym_slot_free_to_busy();
			if (slot_h) {
				set_next_pym_frame(slot_h, true);
				pym_manual_start();
			}
		}
	}

	if (status & IPU_FRAME_START) {
		ipu_slot_dual_h_t *slot_h = NULL;
		ipu_info("ipu start\n");
		slot_h = recv_slot_free_to_busy();
		if (slot_h) {
			set_next_recv_frame(slot_h);
			ipu_dbg("fram start slot-%d, 0x%llx\n", slot_h->info_h.slot_id,
					(uint64_t)IPU_GET_DUAL_SLOT(slot_h->info_h.slot_id, ipu->paddr));
			// resume to recv pic from stop
			//if ( test_and_clear_bit(IPU_RCV_NOT_AVALIABLE, &g_ipu_d_cdev->ipuflags) ) {
			//	ctrl_ipu_to_ddr(CROP_TO_DDR | SCALAR_TO_DDR, ENABLE);
			//}
		}
		//no avaliable slot to recv next pic
		//else if ( !test_and_set_bit(IPU_RCV_NOT_AVALIABLE, &g_ipu_d_cdev->ipuflags) ) {
		//	ctrl_ipu_to_ddr(CROP_TO_DDR | SCALAR_TO_DDR, DISABLE);
		//}
	}
	if (status & PYM_FRAME_START) {
		ipu_slot_dual_h_t *slot_h = NULL;
		ipu_info("pym start\n");
		slot_h = get_last_pym_slot();
		if (slot_h->info_h.slot_flag == SLOT_PYM_1ST)
			set_next_pym_frame(slot_h, false);
		else {
			if (slot_h->info_h.slot_flag == SLOT_PYM_2ND)
				slot_h->info_h.slot_flag = SLOT_DONE;
			slot_h = pym_slot_free_to_busy();
			if (slot_h)
				set_next_pym_frame(slot_h, true);
		}
	}

	if (status & PYM_FRAME_DONE) {
		ipu_slot_dual_h_t *slot_h = NULL;
		ipu_info("pym done\n");
		slot_h = get_cur_pym_slot();
		if (slot_h->info_h.slot_flag == SLOT_DONE) {
			slot_h = pym_slot_busy_to_done();
			if (slot_h) {
				ipu_info("pyramid done, slot-%d, cnt %d\n", slot_h->info_h.slot_id, slot_h->slot_cnt);
				ipu->pymid_done = true;
				ipu->done_idx = slot_h->info_h.slot_id;
				ipu_get_frameid(ipu, slot_h);
				wake_up_interruptible(&g_ipu_d_cdev->event_head);

			} else {
				ipu_err("not finished slot in queue\n");
			}
		}
		if (!ipu_is_pym_busy_empty())
			pym_manual_start();
		else {
			set_bit(IPU_PYM_STARTUP, &g_ipu_d_cdev->ipuflags);
			ipu_info("pym pause\n");
		}
	}

	spin_unlock(&g_ipu_d_cdev->slock);

}

static int8_t ipu_sinfo_init(ipu_cfg_t *ipu_cfg)
{
	uint8_t i = 0;
	memset(&g_ipu_d_cdev->s_info, 0, sizeof(slot_ddr_info_dual_t));
	if (ipu_cfg->ctrl.crop_ddr_en == 1) {
		g_ipu_d_cdev->s_info.crop.y_offset = ipu_cfg->crop_ddr.y_addr;
		g_ipu_d_cdev->s_info.crop.c_offset = ipu_cfg->crop_ddr.c_addr;
		g_ipu_d_cdev->s_info.crop.y_width = ipu_cfg->crop.crop_ed.w - ipu_cfg->crop.crop_st.w;
		g_ipu_d_cdev->s_info.crop.y_height = ipu_cfg->crop.crop_ed.h - ipu_cfg->crop.crop_st.h;
		g_ipu_d_cdev->s_info.crop.y_stride = ALIGN_16(g_ipu_d_cdev->s_info.crop.y_width);
		g_ipu_d_cdev->s_info.crop.c_width = (ipu_cfg->crop.crop_ed.w - ipu_cfg->crop.crop_st.w) >> 1;
		g_ipu_d_cdev->s_info.crop.c_height = ipu_cfg->crop.crop_ed.h - ipu_cfg->crop.crop_st.h;
		g_ipu_d_cdev->s_info.crop.c_stride = ALIGN_16(g_ipu_d_cdev->s_info.crop.c_width);
	}
	if (ipu_cfg->ctrl.scale_ddr_en == 1) {
		g_ipu_d_cdev->s_info.scale.y_offset = ipu_cfg->scale_ddr.y_addr;
		g_ipu_d_cdev->s_info.scale.c_offset = ipu_cfg->scale_ddr.c_addr;
		g_ipu_d_cdev->s_info.scale.y_width = ipu_cfg->scale.scale_tgt.w;
		g_ipu_d_cdev->s_info.scale.y_height = ipu_cfg->scale.scale_tgt.h;
		g_ipu_d_cdev->s_info.scale.y_stride = ALIGN_16(g_ipu_d_cdev->s_info.scale.y_width);
		g_ipu_d_cdev->s_info.scale.c_width = ALIGN_16(ipu_cfg->scale.scale_tgt.w >> 1);
		g_ipu_d_cdev->s_info.scale.c_height = ipu_cfg->scale.scale_tgt.h;
		g_ipu_d_cdev->s_info.scale.c_stride = ALIGN_16(g_ipu_d_cdev->s_info.scale.c_width);
	}
	if (ipu_cfg->pymid.pymid_en == 1) {
		for (i = 0; i < ipu_cfg->pymid.ds_layer_en; i++) {
			if (i % 4 == 0 || ipu_cfg->pymid.ds_factor[i]) {
				g_ipu_d_cdev->s_info.ds_1st[i].y_offset = ipu_cfg->ds_ddr[i].y_addr;
				g_ipu_d_cdev->s_info.ds_1st[i].c_offset = ipu_cfg->ds_ddr[i].c_addr;
				g_ipu_d_cdev->s_info.ds_1st[i].y_width = ipu_cfg->pymid.ds_roi[i].w;
				g_ipu_d_cdev->s_info.ds_1st[i].y_height = ipu_cfg->pymid.ds_roi[i].h;
				g_ipu_d_cdev->s_info.ds_1st[i].y_stride = ALIGN_16(g_ipu_d_cdev->s_info.ds_1st[i].y_width);
				g_ipu_d_cdev->s_info.ds_1st[i].c_width = ipu_cfg->pymid.ds_roi[i].w >> 1;
				g_ipu_d_cdev->s_info.ds_1st[i].c_height = ipu_cfg->pymid.ds_roi[i].h;
				g_ipu_d_cdev->s_info.ds_1st[i].c_stride = ALIGN_16(g_ipu_d_cdev->s_info.ds_1st[i].c_width);

				g_ipu_d_cdev->s_info.ds_2nd[i].y_offset = ipu_cfg->ds_ddr[i].y_addr + IPU_SLOT_DAUL_SIZE / 2;
				g_ipu_d_cdev->s_info.ds_2nd[i].c_offset = ipu_cfg->ds_ddr[i].c_addr + IPU_SLOT_DAUL_SIZE / 2;
				g_ipu_d_cdev->s_info.ds_2nd[i].y_width = ipu_cfg->pymid.ds_roi[i].w;
				g_ipu_d_cdev->s_info.ds_2nd[i].y_height = ipu_cfg->pymid.ds_roi[i].h;
				g_ipu_d_cdev->s_info.ds_2nd[i].y_stride = ALIGN_16(g_ipu_d_cdev->s_info.ds_2nd[i].y_width);
				g_ipu_d_cdev->s_info.ds_2nd[i].c_width = ipu_cfg->pymid.ds_roi[i].w >> 1;
				g_ipu_d_cdev->s_info.ds_2nd[i].c_height = ipu_cfg->pymid.ds_roi[i].h;
				g_ipu_d_cdev->s_info.ds_2nd[i].c_stride = ALIGN_16(g_ipu_d_cdev->s_info.ds_2nd[i].c_width);
			}
		}
		for (i = 0; i < 6; i++) {
			if (ipu_cfg->pymid.us_layer_en & 1 << i) {
				if (ipu_cfg->pymid.us_factor[i]) {
					g_ipu_d_cdev->s_info.us_1st[i].y_offset = ipu_cfg->us_ddr[i].y_addr;
					g_ipu_d_cdev->s_info.us_1st[i].c_offset = ipu_cfg->us_ddr[i].c_addr;
					g_ipu_d_cdev->s_info.us_1st[i].y_width = ipu_cfg->pymid.us_roi[i].w;
					g_ipu_d_cdev->s_info.us_1st[i].y_height = ipu_cfg->pymid.us_roi[i].h;
					g_ipu_d_cdev->s_info.us_1st[i].y_stride = ALIGN_16(g_ipu_d_cdev->s_info.us_1st[i].y_width);
					g_ipu_d_cdev->s_info.us_1st[i].c_width = ipu_cfg->pymid.us_roi[i].w >> 1;
					g_ipu_d_cdev->s_info.us_1st[i].c_height = ipu_cfg->pymid.us_roi[i].h;
					g_ipu_d_cdev->s_info.us_1st[i].c_stride = ALIGN_16(g_ipu_d_cdev->s_info.us_1st[i].c_width);

					g_ipu_d_cdev->s_info.us_2nd[i].y_offset = ipu_cfg->us_ddr[i].y_addr + IPU_SLOT_DAUL_SIZE / 2;
					g_ipu_d_cdev->s_info.us_2nd[i].c_offset = ipu_cfg->us_ddr[i].c_addr + IPU_SLOT_DAUL_SIZE / 2;
					g_ipu_d_cdev->s_info.us_2nd[i].y_width = ipu_cfg->pymid.us_roi[i].w;
					g_ipu_d_cdev->s_info.us_2nd[i].y_height = ipu_cfg->pymid.us_roi[i].h;
					g_ipu_d_cdev->s_info.us_2nd[i].y_stride = ALIGN_16(g_ipu_d_cdev->s_info.us_2nd[i].y_width);
					g_ipu_d_cdev->s_info.us_2nd[i].c_width = ipu_cfg->pymid.us_roi[i].w >> 1;
					g_ipu_d_cdev->s_info.us_2nd[i].c_height = ipu_cfg->pymid.us_roi[i].h;
					g_ipu_d_cdev->s_info.us_2nd[i].c_stride = ALIGN_16(g_ipu_d_cdev->s_info.us_2nd[i].c_width);
				}
			}
		}
	}

	return 0;
}

static int8_t ipu_core_update(ipu_cfg_t *ipu_cfg)
{
	ips_module_reset(RST_IPU);
	ipu_cfg_ddrinfo_init(ipu_cfg);
	ipu_set(IPUC_SET_BASE, ipu_cfg, 0);
	ipu_set(IPUC_SET_CROP, ipu_cfg, 0);
	ipu_set(IPUC_SET_SCALE, ipu_cfg, 0);
	ipu_set(IPUC_SET_PYMID, ipu_cfg, 0);
	ipu_set(IPUC_SET_FRAME_ID, ipu_cfg, 0);
	ipu_sinfo_init(ipu_cfg);
	ipu_slot_dual_recfg(&g_ipu_d_cdev->s_info);
	ipu_clean_slot_queue(&g_ipu_d_cdev->s_info);

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
	init_ipu_slot_dual((uint64_t)g_ipu->vaddr, &g_ipu_d_cdev->s_info);

	return 0;
}

int ipu_dual_open(struct inode *node, struct file *filp)
{
	struct ipu_dual_cdev *ipu_cdev = NULL;
	ipu_dbg("ipu_dual_open\n");
	ipu_cdev = container_of(node->i_cdev, struct ipu_dual_cdev, cdev);
	filp->private_data = ipu_cdev;
	ipu_cdev->ipu->ipu_mode = IPU_DDR_DUAL;
	return 0;
}


long ipu_dual_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	struct ipu_dual_cdev *ipu_cdev = filp->private_data;
	struct x2_ipu_data *ipu = ipu_cdev->ipu;
	ipu_cfg_t *ipu_cfg = (ipu_cfg_t *)ipu->cfg;
	ipu_slot_dual_h_t *slot_h = NULL;
	info_dual_h_t *info = NULL;
	int ret = 0;
	ipu_dbg("ipu cmd: %d\n", _IOC_NR(cmd));

	switch (cmd) {
	case IPUC_INIT:
		{
			ret = copy_from_user((void *)ipu_cfg,
								 (const void __user *)data, sizeof(ipu_init_t));
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
		}
		break;
	case IPUC_GET_IMG:
		/* TODO get description */
		/* could be used read api, caused user can define length */
		//dma_transfer((void *)data, IPU_SLOT_DAUL_SIZE);
		//ret = IPU_SLOT_DAUL_SIZE;
		break;
	case IPUC_GET_ERR_STATUS:
		{
			ret = copy_to_user((void __user *)data, (const void *)&g_ipu_d_cdev->err_status, sizeof(uint32_t));
			if (ret) {
				ipu_err("ioctl get err fail\n");
				return -EFAULT;
			}
			g_ipu_d_cdev->err_status = 0;
		}
		break;
	case IPUC_CNN_DONE:
		{
			int slot_id = 0;
			ret = copy_from_user((void *)&slot_id,
								 (const void __user *)data, sizeof(int));
			if (ret) {
				ipu_err("ioctl cnn done msg fail\n");
				return -EFAULT;
			}
			spin_lock(&g_ipu_d_cdev->slock);
			insert_dual_slot_to_free(slot_id, &g_ipu_d_cdev->s_info);
			spin_unlock(&g_ipu_d_cdev->slock);
		}
		break;
	case IPUC_GET_DONE_INFO:
		{
			spin_lock(&g_ipu_d_cdev->slock);
			slot_h = ipu_get_pym_done_slot();
			if (!slot_h) {
				spin_unlock(&g_ipu_d_cdev->slock);
				ipu_dbg("get_done failed\n");
				return -EFAULT;
			}

			if (ipu->done_idx != -1 &&
				ipu->done_idx != slot_h->info_h.slot_id) {
				ipu_err("cnn slot delay\n");
			}
			info = &slot_h->info_h;
			info->base = (uint64_t)IPU_GET_DUAL_SLOT(slot_h->info_h.slot_id, ipu->paddr);
			spin_unlock(&g_ipu_d_cdev->slock);
			ret = copy_to_user((void __user *)data, (const void *)info, sizeof(ipu_slot_dual_h_t));
			if (ret) {
				ipu_err("copy to user fail\n");
			}
		}
		break;
	case IPUC_DUMP_REG:
		ipu_dump_regs();
		break;
	case IPUC_STOP:
		{
			ipu_drv_stop();
			clear_bit(IPU_PYM_STARTUP, &g_ipu_d_cdev->ipuflags);
			spin_lock(&g_ipu_d_cdev->slock);
			ipu_clean_slot_queue(&g_ipu_d_cdev->s_info);
			spin_unlock(&g_ipu_d_cdev->slock);
			wake_up_interruptible(&ipu_cdev->event_head);
		}
		break;
	case IPUC_START:
		{
#if 1
			spin_lock(&g_ipu_d_cdev->slock);
			slot_h = recv_slot_free_to_busy();
			if (slot_h) {
				set_next_recv_frame(slot_h);
				set_bit(IPU_PYM_STARTUP, &g_ipu_d_cdev->ipuflags);
			} else {
				spin_unlock(&g_ipu_d_cdev->slock);
				ret = EFAULT;
				break;
			}
			spin_unlock(&g_ipu_d_cdev->slock);
			ipu_drv_start();
#else
			printk("g_ipu->stop %d\n",g_ipu->stop);
			recv_slot_free_to_busy();
			recv_slot_busy_to_done();
			slot_h = pym_slot_free_to_busy();
			if ( slot_h ) {
				set_next_pym_frame(slot_h, true);
				//ctrl_ipu_to_ddr(PYM_TO_DDR, ENABLE);
				pym_manual_start();
			}
#endif
		}
		break;
	case IPUC_UPDATE_CFG:
		{
			ret = copy_from_user((void *)ipu_cfg,
								 (const void __user *)data, sizeof(ipu_init_t));
			if (ret) {
				ipu_err("ioctl update fail\n");
				return -EFAULT;
			}
			spin_lock(&g_ipu_d_cdev->slock);
			//stop
			ipu_drv_stop();
			wake_up_interruptible(&ipu_cdev->event_head);
			clear_bit(IPU_PYM_STARTUP, &g_ipu_d_cdev->ipuflags);
			//update
			ipu_core_update(ipu_cfg);
			//restart
			slot_h = recv_slot_free_to_busy();
			if (slot_h) {
				set_next_recv_frame(slot_h);
				set_bit(IPU_PYM_STARTUP, &g_ipu_d_cdev->ipuflags);
			} else {
				spin_unlock(&g_ipu_d_cdev->slock);
				ret = EFAULT;
				break;
			}
			spin_unlock(&g_ipu_d_cdev->slock);
			ipu_drv_start();
		}
		break;
	case IPUC_GET_MEM_INFO:
		{
			ipu_meminfo_t meminfo;
			meminfo.paddr = g_ipu->paddr;
			meminfo.memsize = g_ipu->memsize;
			ret = copy_to_user((void __user *)data, (const void *)&meminfo, sizeof(ipu_meminfo_t));
			if (ret) {
				ipu_err("copy to user fail\n");
			}
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

int ipu_dual_mmap(struct file *filp, struct vm_area_struct *vma)
{
	uint64_t offset = vma->vm_pgoff << PAGE_SHIFT;
	if (!offset)
		offset = g_ipu->paddr;
	ipu_info("ipu mmap offset: 0x%llx, size: %ld\n", offset, vma->vm_end - vma->vm_start);
	if ((vma->vm_end - vma->vm_start) > g_ipu->memsize) {
		return -ENOMEM;
	}
	vma->vm_flags |= VM_IO;
	vma->vm_flags |= VM_LOCKED;
	if (remap_pfn_range(vma, vma->vm_start, offset >> PAGE_SHIFT, vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		ipu_err("ipu mmap fail\n");
		return -EAGAIN;
	}

	ipu_info("ipu mmap ok\n");
	return 0;
}

unsigned int ipu_dual_poll(struct file *file, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	if (!ipu_is_pym_done_empty()) {
		mask = EPOLLIN | EPOLLET;
		return mask;
	} else
		poll_wait(file, &g_ipu_d_cdev->event_head, wait);
	if (g_ipu->stop) {
		mask = EPOLLHUP;
		ipu_info("ipu exit request\n");
	} else if (g_ipu_d_cdev->err_status) {
		ipu_info("POLLERR: err_status 0x%x\n", g_ipu_d_cdev->err_status);
		mask = EPOLLERR;
	} else if (!ipu_is_pym_done_empty()) {
		mask = EPOLLIN | EPOLLET;
		return mask;
	}
	return mask;
}

int ipu_dual_close(struct inode *inode, struct file *filp)
{
	struct ipu_dual_cdev *ipu_cdev = filp->private_data;
	ipu_cdev->ipu->ipu_mode = IPU_INVALID;
	return 0;
}

static const struct file_operations ipu_dual_fops = {
	.owner = THIS_MODULE,
	.mmap = ipu_dual_mmap,
	.open = ipu_dual_open,
	.poll = ipu_dual_poll,
	.release = ipu_dual_close,
	.unlocked_ioctl = ipu_dual_ioctl,
};

static int __init x2_ipu_dual_init(void)
{
	int ret = 0;
	struct device *dev = NULL;
	g_ipu_d_cdev = kmalloc(sizeof(struct ipu_dual_cdev), GFP_KERNEL);
	if (!g_ipu_d_cdev) {
		ipu_err("Unable to alloc IPU DUAL DEV\n");
		return -ENOMEM;
	}
	g_ipu_d_cdev->name = X2_IPU_DUAL_NAME;
	g_ipu_d_cdev->ipu = g_ipu;
	g_ipu_d_cdev->class = class_create(THIS_MODULE, X2_IPU_DUAL_NAME);
	alloc_chrdev_region(&g_ipu_d_cdev->dev_num, 0, 1, "ipu-dual");
	cdev_init(&g_ipu_d_cdev->cdev, &ipu_dual_fops);
	cdev_add(&g_ipu_d_cdev->cdev, g_ipu_d_cdev->dev_num, 1);
	dev = device_create(g_ipu_d_cdev->class, NULL, g_ipu_d_cdev->dev_num, NULL, "ipu-dual");
	if (IS_ERR(dev)) {
		ret  = -EINVAL;
		ipu_err("ipu device create fail\n");
	}
	init_waitqueue_head(&g_ipu_d_cdev->event_head);
	spin_lock_init(&g_ipu_d_cdev->slock);
	g_ipu->ipu_handle[IPU_DDR_DUAL] = ipu_dual_mode_process;
	return ret;
}

static void __exit x2_ipu_dual_exit(void)
{
	device_destroy(g_ipu_d_cdev->class, g_ipu_d_cdev->dev_num);
	unregister_chrdev_region(g_ipu_d_cdev->dev_num, 1);
	class_destroy(g_ipu_d_cdev->class);
}

module_init(x2_ipu_dual_init);
module_exit(x2_ipu_dual_exit);

MODULE_DESCRIPTION("Driver for X2 IPU Dual Dev");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL");
