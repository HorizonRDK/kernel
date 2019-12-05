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
#include <x2/diag.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/eventpoll.h>
#include "ipu_slot_dual.h"
#include "ipu_dev.h"
#include "ipu_dual.h"
#include "ipu_ddr.h"
#include "ipu_drv.h"
#include "ipu_common.h"
#include <asm-generic/io.h>
#include <asm/string.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>

#include "ipu_pym.h"

#include "../../iar/x2_iar.h"

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

#define IPUC_PYM_MANUAL_PROCESS	_IO(IPU_IOC_MAGIC, 10)
#define IPUC_SRC_DONE			_IO(IPU_IOC_MAGIC, 11)
#define IPUC_PYM_DONE			_IO(IPU_IOC_MAGIC, 12)
#define IPUC_FB_SRC_DONE			_IO(IPU_IOC_MAGIC, 13)
#define IPUC_FB_FLUSH			_IO(IPU_IOC_MAGIC, 14)

#define X2_IPU_DUAL_NAME	"x2-ipu-dual"

extern struct x2_ipu_data *g_ipu;

static struct ipu_dual_cdev *g_ipu_d_cdev = NULL;
static int64_t g_ipu_time;

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
static int8_t ipu_get_frameid(struct x2_ipu_data *ipu, ipu_slot_dual_h_t *slot)
{
	uint8_t *tmp = NULL;
	uint64_t vaddr = (uint64_t)IPU_GET_DUAL_SLOT(slot->info_h.slot_id, ipu->vaddr);
	ipu_cfg_t *cfg = (ipu_cfg_t *)ipu->cfg;
	int64_t ts = ipu_tsin_get(g_ipu_time);

	slot->info_h.cf_timestamp = ts;
	slot->info_h.sf_timestamp = ts;
	if (!cfg->frame_id.id_en)
		return 0;
	/* get first id from pymid ddr address */
	tmp = (uint8_t *)(slot->info_h.dual_ddr_info.crop.y_offset + vaddr);
	__inval_dcache_area((void *)tmp, L1_CACHE_BYTES);
	if (cfg->frame_id.bus_mode == 0) {
		slot->info_h.cf_id = tmp[0] << 8 | tmp[1];
		ipu_dbg("pframe_id_fst=%d, %d, %d\n", tmp[0], tmp[1], slot->info_h.cf_id);
	} else {
		slot->info_h.cf_id = decode_frame_id((uint16_t *)tmp);
		slot->info_h.cf_timestamp = ts;
		ipu_dbg("pframe_id_fst=%d\n", slot->info_h.cf_id);
	}
	/* get second id from pymid ddr address */
	tmp = (uint8_t *)(slot->info_h.dual_ddr_info.scale.y_offset + vaddr);
	__inval_dcache_area((void *)tmp, L1_CACHE_BYTES);
	if (cfg->frame_id.bus_mode == 0) {
		slot->info_h.sf_id = tmp[0] << 8 | tmp[1];
		ipu_dbg("pframe_id_sec=%d, %d, %d\n", tmp[0], tmp[1], slot->info_h.sf_id);
	} else {
		slot->info_h.sf_id = decode_frame_id((uint16_t *)tmp);
		slot->info_h.sf_timestamp = ts;
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

static void ipu_dual_diag_report(uint8_t errsta, unsigned int status)
{
	unsigned int sta;

	sta = status;
	if (errsta) {
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_VIO,
				EventIdVioIpuDualErr,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				(uint8_t *)&sta,
				sizeof(unsigned int));
	} else {
		diag_send_event_stat(
				DiagMsgPrioMid,
				ModuleDiag_VIO,
				EventIdVioIpuDualErr,
				DiagEventStaSuccess);
	}
}

void ipu_dual_mode_process(uint32_t status)
{
	struct x2_ipu_data *ipu = g_ipu_d_cdev->ipu;
	uint32_t iar_display_yaddr;
	uint32_t iar_display_caddr;
	uint8_t errsta = 0;
	uint32_t len;
	uint64_t base;

	spin_lock(&g_ipu_d_cdev->slock);
	if (status & IPU_BUS01_TRANSMIT_ERRORS ||
		status & IPU_BUS23_TRANSMIT_ERRORS ||
		status & PYM_DS_FRAME_DROP ||
		status & PYM_US_FRAME_DROP) {
		g_ipu_d_cdev->err_status = status;
		//should we set it back to 0 in ioctl?
		ipu_err("ipu error 0x%x\n", g_ipu_d_cdev->err_status);
		pym_slot_busy_to_free();
		ipu_pym_process_done(status);
		errsta = 1;
	}

	if (status & IPU_FRAME_DONE) {
		struct mult_img_info_t tmp_mult_img_info;
		ipu_slot_dual_h_t *slot_h = NULL;
		ipu_info("ipu done\n");

		if (slot_alive(RECVING_SLOT_QUEUE) < 2) {
			ipu_dbg("[%d] ipu recving slot less than 2\n", __LINE__);
		} else {
			slot_h = recv_slot_busy_to_done();
			//if (slot_h && test_and_clear_bit(IPU_PYM_STARTUP, &g_ipu_d_cdev->ipuflags)) {
			if (slot_h) {
				//test_and_clear_bit(IPU_PYM_STARTUP, &g_ipu_d_cdev->ipuflags);
				slot_h = ipu_get_recv_done_slot();
				if (slot_h) {
					ipu_get_frameid(ipu, slot_h);
				/*	set_next_pym_frame(slot_h, true);
					pym_manual_start();
				}*/
					tmp_mult_img_info.src_num = 2;
					tmp_mult_img_info.src_img_info[0].slot_id = slot_h->info_h.slot_id;
					tmp_mult_img_info.src_img_info[1].slot_id = slot_h->info_h.slot_id;
					tmp_mult_img_info.src_img_info[0].frame_id = slot_h->info_h.cf_id;
					tmp_mult_img_info.src_img_info[1].frame_id = slot_h->info_h.sf_id;
					tmp_mult_img_info.src_img_info[0].timestamp = slot_h->info_h.cf_timestamp;
					tmp_mult_img_info.src_img_info[1].timestamp = slot_h->info_h.sf_timestamp;
					tmp_mult_img_info.src_img_info[0].src_img.y_paddr =
						IPU_GET_DUAL_SLOT(slot_h->info_h.slot_id, g_ipu->paddr)
						+ slot_h->info_h.dual_ddr_info.crop.y_offset;
					tmp_mult_img_info.src_img_info[1].src_img.y_paddr =
						IPU_GET_DUAL_SLOT(slot_h->info_h.slot_id, g_ipu->paddr)
						+ slot_h->info_h.dual_ddr_info.scale.y_offset;
					tmp_mult_img_info.src_img_info[0].src_img.c_paddr =
						IPU_GET_DUAL_SLOT(slot_h->info_h.slot_id, g_ipu->paddr)
						+ slot_h->info_h.dual_ddr_info.crop.c_offset;
					tmp_mult_img_info.src_img_info[1].src_img.c_paddr =
						IPU_GET_DUAL_SLOT(slot_h->info_h.slot_id, g_ipu->paddr)
						+ slot_h->info_h.dual_ddr_info.scale.c_offset;

					/* in dual acaler are the same with src  */
					tmp_mult_img_info.src_img_info[0].scaler_img
						= tmp_mult_img_info.src_img_info[0].src_img;
					tmp_mult_img_info.src_img_info[1].scaler_img
						= tmp_mult_img_info.src_img_info[1].src_img;

					/* add to pym process fifo */
					if (ipu_pym_to_process(NULL,
								&tmp_mult_img_info, g_ipu->cfg,
							PYM_SLOT_MULT, PYM_INLINE) < 0) {
						insert_dual_slot_to_free(slot_h->info_h.slot_id,
								&g_ipu_d_cdev->s_info);
					}
				}
			}
		}
	}

	if (status & IPU_FRAME_START) {
		ipu_slot_dual_h_t *slot_h = NULL;
		ipu_info("ipu start\n");
		g_ipu_time = ipu_current_time();
		ipu_tsin_reset();
		slot_h = recv_slot_free_to_busy();
		if (slot_h) {
			set_next_recv_frame(slot_h);
			ipu_dbg("fram start slot-%d, 0x%llx\n", slot_h->info_h.slot_id,
					(uint64_t)IPU_GET_DUAL_SLOT(slot_h->info_h.slot_id, ipu->paddr));
		}
	}
	if (status & PYM_FRAME_START) {
#if 0
		ipu_slot_dual_h_t *slot_h = NULL;
		ipu_info("pym start\n");
		slot_h = get_last_pym_slot();
		if (slot_h) {
			if (slot_h->info_h.slot_flag == SLOT_PYM_1ST)
				set_next_pym_frame(slot_h, false);
			else {
				if (slot_h->info_h.slot_flag == SLOT_PYM_2ND)
					slot_h->info_h.slot_flag = SLOT_DONE;
				if (slot_alive(RECVING_SLOT_QUEUE) < 2) {
					set_bit(IPU_PYM_STARTUP, &g_ipu_d_cdev->ipuflags);
					ipu_dbg("[%d] ipu recving slot less than 2\n", __LINE__);
				} else {
					slot_h = pym_slot_free_to_busy();
					if (slot_h)
						set_next_pym_frame(slot_h, true);
					else {
						set_bit(IPU_PYM_STARTUP, &g_ipu_d_cdev->ipuflags);
						ipu_dbg("[%d] need set start bit\n", __LINE__);
					}
				}
			}
		}
#endif
	}

	if (status & PYM_FRAME_DONE) {
		int pyming_slot_id = -1;
		ipu_info("pym done\n");

		if (g_ipu_pym) {
			if(g_ipu_pym->pyming_slot_info);
				pyming_slot_id =
					g_ipu_pym->pyming_slot_info->img_info.mult_img_info.src_img_info[0].slot_id;
		}

		if(ipu_pym_process_done(0) > 0) {
			if (pyming_slot_id >= 0) {
				pr_debug("ipu: done slot id is %d.\n", pyming_slot_id);
				iar_set_video_buffer(pyming_slot_id);
				base = (uint64_t)IPU_GET_DUAL_SLOT(pyming_slot_id, ipu->paddr);
				len = IPU_SLOT_DAUL_SIZE;
				dma_sync_single_for_cpu(NULL, base, len, DMA_FROM_DEVICE);
			}
			wake_up_interruptible(&g_ipu_d_cdev->event_head);
		}

#if 0
		ipu_slot_dual_h_t *slot_h = NULL;
		ipu_info("pym done\n");
		slot_h = get_cur_pym_slot();
		if (slot_h) {
			if (slot_h->info_h.slot_flag == SLOT_DONE) {
				slot_h = pym_slot_busy_to_done();
				if (slot_h) {
					ipu_info("pyramid done, slot-%d, cnt %d\n", slot_h->info_h.slot_id, slot_h->slot_cnt);
					ipu->pymid_done = true;
					ipu->done_idx = slot_h->info_h.slot_id;
					ipu_get_frameid(ipu, slot_h);
					wake_up_interruptible(&g_ipu_d_cdev->event_head);

					/*
					iar_display_yaddr =
					IPU_GET_DUAL_SLOT(slot_h->info_h.slot_id, ipu->paddr) +
					slot_h->info_h.dual_ddr_info.ds_2nd[5].y_offset;
					iar_display_caddr =
					IPU_GET_DUAL_SLOT(slot_h->info_h.slot_id, ipu->paddr) +
					slot_h->info_h.dual_ddr_info.ds_2nd[5].c_offset;
					*/
					pr_debug("@@slot id is %d\n", slot_h->info_h.slot_id);
					/*
					pr_debug("ipu: slot info ds_2nd[5] yaddr is 0x%x\n",
						iar_display_yaddr);
					pr_debug("ipu: slot info ds_2nd[5] caddr is 0x%x\n",
						iar_display_caddr);
					*/
					//if (iar_is_enabled())
					iar_set_video_buffer(slot_h->info_h.slot_id);

				} else {
					ipu_err("no finished slot in queue\n");
				}
			}
		}
#endif
		/*if (!ipu_is_pym_busy_empty())
			pym_manual_start();
			*/
		else {
			set_bit(IPU_PYM_STARTUP, &g_ipu_d_cdev->ipuflags);
			ipu_info("pym pause\n");
		}

	}

	spin_unlock(&g_ipu_d_cdev->slock);
	ipu_dual_diag_report(errsta, status);
}

static int8_t ipu_sinfo_init(slot_ddr_info_dual_t *s_info, ipu_cfg_t *ipu_cfg)
{
	uint8_t i = 0;

	memset(s_info, 0, sizeof(slot_ddr_info_dual_t));
	if (ipu_cfg->ctrl.crop_ddr_en == 1) {
		s_info->crop.y_offset = ipu_cfg->crop_ddr.y_addr;
		s_info->crop.c_offset = ipu_cfg->crop_ddr.c_addr;
		s_info->crop.y_width = ipu_cfg->crop.crop_ed.w - ipu_cfg->crop.crop_st.w;
		s_info->crop.y_height = ipu_cfg->crop.crop_ed.h - ipu_cfg->crop.crop_st.h;
		s_info->crop.y_stride = ALIGN_16(s_info->crop.y_width);
		s_info->crop.c_width = (ipu_cfg->crop.crop_ed.w - ipu_cfg->crop.crop_st.w) >> 1;
		s_info->crop.c_height = ipu_cfg->crop.crop_ed.h - ipu_cfg->crop.crop_st.h;
		s_info->crop.c_stride = ALIGN_16(s_info->crop.c_width);
	}
	if (ipu_cfg->ctrl.scale_ddr_en == 1) {
		s_info->scale.y_offset = ipu_cfg->scale_ddr.y_addr;
		s_info->scale.c_offset = ipu_cfg->scale_ddr.c_addr;
		s_info->scale.y_width = ipu_cfg->scale.scale_tgt.w;
		s_info->scale.y_height = ipu_cfg->scale.scale_tgt.h;
		s_info->scale.y_stride = ALIGN_16(s_info->scale.y_width);
		s_info->scale.c_width = ALIGN_16(ipu_cfg->scale.scale_tgt.w >> 1);
		s_info->scale.c_height = ipu_cfg->scale.scale_tgt.h;
		s_info->scale.c_stride = ALIGN_16(s_info->scale.c_width);
	}
	if (ipu_cfg->pymid.pymid_en == 1) {
		for (i = 0; i <= ipu_cfg->pymid.ds_layer_en; i++) {
			if (i % 4 == 0 || ipu_cfg->pymid.ds_factor[i]) {
				s_info->ds_1st[i].y_offset = ipu_cfg->ds_ddr[i].y_addr;
				s_info->ds_1st[i].c_offset = ipu_cfg->ds_ddr[i].c_addr;
				s_info->ds_1st[i].y_width = ipu_cfg->pymid.ds_roi[i].w;
				s_info->ds_1st[i].y_height = ipu_cfg->pymid.ds_roi[i].h;
				s_info->ds_1st[i].y_stride = ALIGN_16(s_info->ds_1st[i].y_width);
				s_info->ds_1st[i].c_width = ipu_cfg->pymid.ds_roi[i].w >> 1;
				s_info->ds_1st[i].c_height = ipu_cfg->pymid.ds_roi[i].h;
				s_info->ds_1st[i].c_stride = ALIGN_16(s_info->ds_1st[i].c_width);

				s_info->ds_2nd[i].y_offset = ipu_cfg->ds_ddr[i].y_addr + IPU_SLOT_DAUL_SIZE / 2;
				s_info->ds_2nd[i].c_offset = ipu_cfg->ds_ddr[i].c_addr + IPU_SLOT_DAUL_SIZE / 2;
				s_info->ds_2nd[i].y_width = ipu_cfg->pymid.ds_roi[i].w;
				s_info->ds_2nd[i].y_height = ipu_cfg->pymid.ds_roi[i].h;
				s_info->ds_2nd[i].y_stride = ALIGN_16(s_info->ds_2nd[i].y_width);
				s_info->ds_2nd[i].c_width = ipu_cfg->pymid.ds_roi[i].w >> 1;
				s_info->ds_2nd[i].c_height = ipu_cfg->pymid.ds_roi[i].h;
				s_info->ds_2nd[i].c_stride = ALIGN_16(s_info->ds_2nd[i].c_width);
			}
		}
		for (i = 0; i < 6; i++) {
			if (ipu_cfg->pymid.us_layer_en & 1 << i) {
				if (ipu_cfg->pymid.us_factor[i]) {
					s_info->us_1st[i].y_offset = ipu_cfg->us_ddr[i].y_addr;
					s_info->us_1st[i].c_offset = ipu_cfg->us_ddr[i].c_addr;
					s_info->us_1st[i].y_width = ipu_cfg->pymid.us_roi[i].w;
					s_info->us_1st[i].y_height = ipu_cfg->pymid.us_roi[i].h;
					s_info->us_1st[i].y_stride = ALIGN_16(s_info->us_1st[i].y_width);
					s_info->us_1st[i].c_width = ipu_cfg->pymid.us_roi[i].w >> 1;
					s_info->us_1st[i].c_height = ipu_cfg->pymid.us_roi[i].h;
					s_info->us_1st[i].c_stride = ALIGN_16(s_info->us_1st[i].c_width);

					s_info->us_2nd[i].y_offset = ipu_cfg->us_ddr[i].y_addr + IPU_SLOT_DAUL_SIZE / 2;
					s_info->us_2nd[i].c_offset = ipu_cfg->us_ddr[i].c_addr + IPU_SLOT_DAUL_SIZE / 2;
					s_info->us_2nd[i].y_width = ipu_cfg->pymid.us_roi[i].w;
					s_info->us_2nd[i].y_height = ipu_cfg->pymid.us_roi[i].h;
					s_info->us_2nd[i].y_stride = ALIGN_16(s_info->us_2nd[i].y_width);
					s_info->us_2nd[i].c_width = ipu_cfg->pymid.us_roi[i].w >> 1;
					s_info->us_2nd[i].c_height = ipu_cfg->pymid.us_roi[i].h;
					s_info->us_2nd[i].c_stride = ALIGN_16(s_info->us_2nd[i].c_width);
				}
			}
		}
		/* for layer-0 in dual mode */
		g_ipu_d_cdev->s_info.ds_1st[0].y_offset = ipu_cfg->crop_ddr.y_addr;
		g_ipu_d_cdev->s_info.ds_1st[0].c_offset = ipu_cfg->crop_ddr.c_addr;
		g_ipu_d_cdev->s_info.ds_2nd[0].y_offset = ipu_cfg->scale_ddr.y_addr;
		g_ipu_d_cdev->s_info.ds_2nd[0].c_offset = ipu_cfg->scale_ddr.c_addr;
	}

	return 0;
}

int8_t iar_get_ipu_display_addr_dual(uint32_t display_addr[][2])
{
	int i = 0;

	if (g_ipu_d_cdev == NULL)
		return -1;
	display_addr[0][0] = g_ipu_d_cdev->ipu->paddr;
	display_addr[0][1] = IPU_SLOT_DAUL_SIZE;
	display_addr[1][0] = g_ipu_d_cdev->s_info.crop.y_offset;
	display_addr[1][1] = g_ipu_d_cdev->s_info.crop.c_offset;
	display_addr[2][0] = g_ipu_d_cdev->s_info.scale.y_offset;
	display_addr[2][1] = g_ipu_d_cdev->s_info.scale.c_offset;

	for (i = 0; i < 24; i++) {
		display_addr[i+3][0] = g_ipu_d_cdev->s_info.ds_1st[i].y_offset;
		display_addr[i+3][1] = g_ipu_d_cdev->s_info.ds_1st[i].c_offset;
	}

	for (i = 0; i < 6; i++) {
		display_addr[i+27][0] = g_ipu_d_cdev->s_info.us_1st[i].y_offset;
		display_addr[i+27][1] = g_ipu_d_cdev->s_info.us_1st[i].c_offset;
	}

	for (i = 0; i < 24; i++) {
		display_addr[i+33][0] = g_ipu_d_cdev->s_info.ds_2nd[i].y_offset;
		display_addr[i+33][1] = g_ipu_d_cdev->s_info.ds_2nd[i].c_offset;
	}

	for (i = 0; i < 6; i++) {
		display_addr[i+57][0] = g_ipu_d_cdev->s_info.us_2nd[i].y_offset;
		display_addr[i+57][1] = g_ipu_d_cdev->s_info.us_2nd[i].c_offset;
	}
	pr_debug("g_ipu_s_cdev->ipu.paddr = 0x%x\n", display_addr[0][0]);

	return 0;
}


static int8_t ipu_core_update(ipu_cfg_t *ipu_cfg)
{
	int8_t ret = 0;
	//ips_module_reset(RST_IPU);
	ret = ipu_cfg_ddrinfo_init(ipu_cfg);
	if (ret < 0)
		return ret;
	ipu_set(IPUC_SET_BASE, ipu_cfg, 0);
	ipu_set(IPUC_SET_CROP, ipu_cfg, 0);
	ipu_set(IPUC_SET_SCALE, ipu_cfg, 0);
	ipu_set(IPUC_SET_PYMID, ipu_cfg, 0);
	ipu_set(IPUC_SET_FRAME_ID, ipu_cfg, 0);
	ipu_sinfo_init(&g_ipu_d_cdev->s_info, ipu_cfg);
	ipu_slot_dual_recfg(&g_ipu_d_cdev->s_info);
	ipu_clean_slot_queue(&g_ipu_d_cdev->s_info);

	return 0;
}

static int8_t ipu_core_init(ipu_cfg_t *ipu_cfg)
{
	int8_t ret = 0;
	ips_module_reset(RST_IPU);
	ret = ipu_cfg_ddrinfo_init(ipu_cfg);
	if (ret < 0)
		return ret;

	ipu_set(IPUC_SET_BASE, ipu_cfg, 0);
	ipu_set(IPUC_SET_CROP, ipu_cfg, 0);
	ipu_set(IPUC_SET_SCALE, ipu_cfg, 0);
	ipu_set(IPUC_SET_PYMID, ipu_cfg, 0);
	ipu_set(IPUC_SET_FRAME_ID, ipu_cfg, 0);

	ipu_sinfo_init(&g_ipu_d_cdev->s_info, ipu_cfg);
	init_ipu_slot_dual((uint64_t)g_ipu->vaddr, &g_ipu_d_cdev->s_info);

	return 0;
}

int ipu_dual_open(struct inode *node, struct file *filp)
{
	struct ipu_dual_cdev *ipu_cdev = NULL;
	struct ipu_user *tmp_ipu_user;
	int slot_id;
	int ret = 0;

	if ((g_ipu->vio_mode != IPU_INVALID) && (g_ipu->vio_mode != IPU_DDR_DUAL)) {
		pr_err("ipu have been opened in %d, ipu dual open fail\n", g_ipu->vio_mode);
		return -EFAULT;
	}
	g_ipu->vio_mode = IPU_DDR_DUAL;
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
	ipu_dbg("ipu_dual_open\n");
	ipu_cdev = container_of(node->i_cdev, struct ipu_dual_cdev, cdev);
	ipu_cdev->ipu->ipu_mode = IPU_DDR_DUAL;
	/* init the file user */
	tmp_ipu_user = (struct ipu_user *)kzalloc(sizeof(*tmp_ipu_user), GFP_KERNEL);
	if (!tmp_ipu_user) {
		pr_err("[%s] %d:ipu init user fail\n", __func__, __LINE__);
		return -ENOMEM;
	}

	for (slot_id = 0; slot_id < IPU_MAX_SLOT; slot_id++) {
		tmp_ipu_user->used_slot[slot_id] = 0;
	}

	ret = ipu_pym_user_init(&tmp_ipu_user->pym_user);
	if (ret < 0) {
		kfree(tmp_ipu_user);
		pr_err("[%s] %d:ipu init pym user fail\n", __func__, __LINE__);
		return ret;
	}
	tmp_ipu_user->host = ipu_cdev;
	tmp_ipu_user->pym_user.host = tmp_ipu_user;
	ipu_cdev->open_counter++;


	filp->private_data = tmp_ipu_user;

	return 0;
}


long ipu_dual_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	struct ipu_user *tmp_ipu_user = filp->private_data;
	struct ipu_dual_cdev *ipu_cdev = tmp_ipu_user->host;
	struct x2_ipu_data *ipu = ipu_cdev->ipu;
	ipu_cfg_t *ipu_cfg = &tmp_ipu_user->cfg;
	ipu_slot_dual_h_t *slot_h = NULL;
	info_dual_h_t *info = NULL;
	info_dual_h_t tmp_info;
	struct mult_img_info_t src_info;
	struct mult_img_info_t *mult_img_info = NULL;
	info_dual_h_t pym_info;
	unsigned long flags;
	int ret = 0;
	ipu_dbg("ipu cmd: %d\n", _IOC_NR(cmd));

	switch (cmd) {
	case IPUC_INIT:
		{
			mutex_lock(&ipu_cdev->mutex_lock);
			ret = copy_from_user((void *)ipu_cfg,
								 (const void __user *)data, sizeof(ipu_init_t));
			if (ret) {
				mutex_unlock(&ipu_cdev->mutex_lock);
				ipu_err("ioctl init fail\n");
				return -EFAULT;
			}

			if (ipu_cfg->timeout <= 0)
				ipu_cfg->timeout = 500;

			if (!g_ipu->cfg->video_in.w || !g_ipu->cfg->video_in.h) {
				ret = ipu_core_init(ipu_cfg);
				if (ret < 0) {
					mutex_unlock(&ipu_cdev->mutex_lock);
					ipu_err("ioctl init fail\n");
					return -EFAULT;
				}
				memcpy(g_ipu->cfg, ipu_cfg, sizeof(ipu_cfg_t));
				memcpy(&tmp_ipu_user->dual_ddr_info,
						&g_ipu_d_cdev->s_info, sizeof(slot_ddr_info_dual_t));
				ipu_pym_clear();
			} else {
				ret = ipu_cfg_ddrinfo_init(ipu_cfg);
				if (ret < 0) {
					mutex_unlock(&ipu_cdev->mutex_lock);
					ipu_err("ioctl init fail\n");
					return -EFAULT;
				}

				ipu_sinfo_init(&tmp_ipu_user->dual_ddr_info, ipu_cfg);
			}

			g_ipu_time = 0;
			ret = 0;
			mutex_unlock(&ipu_cdev->mutex_lock);
		}
		break;

	case IPUC_PYM_MANUAL_PROCESS:
		{
			mult_img_info = &src_info;
			mutex_lock(&ipu_cdev->mutex_lock);
			ret = copy_from_user((void *)mult_img_info,
			(const void __user *)data,
			sizeof(struct mult_img_info_t));
			if (ret) {
				mutex_unlock(&ipu_cdev->mutex_lock);
				ipu_err("ioctl process fail\n");
				return -EFAULT;
			}
			__flush_dcache_area(page_address(pfn_to_page(PHYS_PFN((phys_addr_t)mult_img_info->src_img_info[0].src_img.y_paddr))),
								mult_img_info->src_img_info[0].src_img.width * mult_img_info->src_img_info[0].src_img.height +
								mult_img_info->src_img_info[0].src_img.width * mult_img_info->src_img_info[0].src_img.height/2);
			if (mult_img_info->src_num != 1) {
				__flush_dcache_area(page_address(pfn_to_page(PHYS_PFN((phys_addr_t)mult_img_info->src_img_info[1].src_img.y_paddr))),
									mult_img_info->src_img_info[1].src_img.width * mult_img_info->src_img_info[1].src_img.height +
									mult_img_info->src_img_info[1].src_img.width * mult_img_info->src_img_info[1].src_img.height/2);
			}

			ret = ipu_pym_to_process(&tmp_ipu_user->pym_user,
					mult_img_info, g_ipu->cfg,
					PYM_SLOT_MULT, PYM_OFFLINE);
			if (ret) {
				mutex_unlock(&ipu_cdev->mutex_lock);
				ipu_err("pym to process fail\n");
				return ret;
			}
			mutex_unlock(&ipu_cdev->mutex_lock);
		}

		break;

	case IPUC_PYM_DONE:
		{
			struct mult_img_info_t tmp_mult_img_info;
			info_dual_h_t *pym_img_info = NULL;
			mutex_lock(&ipu_cdev->mutex_lock);
			ret = ipu_pym_wait_process_done(&tmp_ipu_user->pym_user,
					&tmp_mult_img_info,
					sizeof(struct mult_img_info_t), PYM_OFFLINE, ipu_cfg->timeout);
			if (ret < 0) {
				mutex_unlock(&ipu_cdev->mutex_lock);
				ipu_err("data from pym failed\n");
				return -EFAULT;
			}

			spin_lock_irqsave(&g_ipu_d_cdev->slock, flags);
			pym_img_info = &pym_info;
			pym_img_info->slot_id = tmp_mult_img_info.src_img_info[0].slot_id;
			pym_img_info->slot_flag = 0;
			pym_img_info->ipu_flag = 0;
			pym_img_info->cnn_flag = 0;
			pym_img_info->cf_id = tmp_mult_img_info.src_img_info[0].frame_id;
			pym_img_info->sf_id = tmp_mult_img_info.src_img_info[1].frame_id;
			pym_img_info->cf_timestamp = tmp_mult_img_info.src_img_info[0].timestamp;
			pym_img_info->sf_timestamp = tmp_mult_img_info.src_img_info[1].timestamp;
			pym_img_info->base =
				(uint64_t)IPU_GET_DUAL_SLOT(pym_img_info->slot_id, ipu->paddr);
			if (pym_img_info->slot_id == g_ipu_pym->new_slot_id)
				g_ipu_pym->new_slot_id = -1;

			pym_img_info->dual_ddr_info = g_ipu_d_cdev->s_info;
			spin_unlock_irqrestore(&g_ipu_d_cdev->slock, flags);
			ret = copy_to_user((void __user *)data, (const void *)pym_img_info,
			sizeof(info_dual_h_t));
			if (ret)
				ipu_err("copy to user fail\n");

			mutex_unlock(&ipu_cdev->mutex_lock);
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
			mutex_lock(&ipu_cdev->mutex_lock);
			ret = copy_from_user((void *)&slot_id,
								 (const void __user *)data, sizeof(int));
			if (ret) {
				mutex_unlock(&ipu_cdev->mutex_lock);
				ipu_err("ioctl cnn done msg fail\n");
				return -EFAULT;
			}
			spin_lock_irqsave(&g_ipu_d_cdev->slock, flags);
			if (insert_dual_slot_to_free(slot_id, &g_ipu_d_cdev->s_info) < 0) {
				pr_err("mult free bad slot id: %d\n", slot_id);
				spin_unlock_irqrestore(&g_ipu_d_cdev->slock, flags);
				mutex_unlock(&ipu_cdev->mutex_lock);
				return -EFAULT;
			}
			tmp_ipu_user->used_slot[slot_id] = 0;
			spin_unlock_irqrestore(&g_ipu_d_cdev->slock, flags);
			mutex_unlock(&ipu_cdev->mutex_lock);
		}
		break;
	case IPUC_GET_DONE_INFO:
		{
			struct mult_img_info_t tmp_mult_img_info;
			info_dual_h_t *pym_img_info = NULL;
			mutex_lock(&ipu_cdev->mutex_lock);
again:
			ret = ipu_pym_wait_process_done(NULL,
					&tmp_mult_img_info,
					sizeof(struct mult_img_info_t), PYM_INLINE, 0);
			if (ret < 0) {
				if (ret == -EAGAIN) {
					/* processed error */
					insert_dual_slot_to_free(
							tmp_mult_img_info.src_img_info[0].slot_id,
							&g_ipu_d_cdev->s_info);
					tmp_ipu_user->used_slot[tmp_mult_img_info.src_img_info[0].slot_id] = 0;
					goto again;
				}
				/* offline slot free by user */
				ret = ipu_pym_wait_process_done(&tmp_ipu_user->pym_user,
						&tmp_mult_img_info,
						sizeof(struct mult_img_info_t), PYM_OFFLINE, 0);
				if (ret < 0) {
					mutex_unlock(&ipu_cdev->mutex_lock);
					ipu_err("copy to user fail\n");
					return -EFAULT;
				}
			}

			spin_lock_irqsave(&g_ipu_d_cdev->slock, flags);
			pym_img_info = &pym_info;
			pym_img_info->slot_id = tmp_mult_img_info.src_img_info[0].slot_id;
			pym_img_info->slot_flag = 0;
			pym_img_info->ipu_flag = 0;
			pym_img_info->cnn_flag = 0;
			pym_img_info->cf_id = tmp_mult_img_info.src_img_info[0].frame_id;
			pym_img_info->sf_id = tmp_mult_img_info.src_img_info[1].frame_id;
			pym_img_info->cf_timestamp = tmp_mult_img_info.src_img_info[0].timestamp;
			pym_img_info->sf_timestamp = tmp_mult_img_info.src_img_info[1].timestamp;
			pym_img_info->base =
				(uint64_t)IPU_GET_DUAL_SLOT(pym_img_info->slot_id, ipu->paddr);

			pym_img_info->dual_ddr_info = g_ipu_d_cdev->s_info;
			if (pym_img_info->slot_id == g_ipu_pym->new_slot_id)
				g_ipu_pym->new_slot_id = -1;

			spin_unlock_irqrestore(&g_ipu_d_cdev->slock, flags);
			tmp_ipu_user->used_slot[pym_img_info->slot_id] = 1;
			ret = copy_to_user((void __user *)data, (const void *)pym_img_info,
			sizeof(info_dual_h_t));
			if (ret)
				ipu_err("copy to user fail\n");

			mutex_unlock(&ipu_cdev->mutex_lock);
		}
		break;
	case IPUC_DUMP_REG:
		ipu_dump_regs();
		break;
	case IPUC_STOP:
		{
			ipu_pym_user_exit(&tmp_ipu_user->pym_user);

			if (ipu_cdev->open_counter > 1)
				break;

			mutex_lock(&ipu_cdev->mutex_lock);
			ipu_drv_stop();
			clear_bit(IPU_PYM_STARTUP, &g_ipu_d_cdev->ipuflags);
			spin_lock_irqsave(&g_ipu_d_cdev->slock, flags);
			ipu_clean_slot_queue(&g_ipu_d_cdev->s_info);
			spin_unlock_irqrestore(&g_ipu_d_cdev->slock, flags);
			wake_up_interruptible(&ipu_cdev->event_head);
			mutex_unlock(&ipu_cdev->mutex_lock);
		}
		break;
	case IPUC_START:
		{
#if 1
			mutex_lock(&ipu_cdev->mutex_lock);
			if (slot_alive(RECVING_SLOT_QUEUE) < 1) {
				spin_lock_irqsave(&g_ipu_d_cdev->slock, flags);
				slot_h = recv_slot_free_to_busy();
				if (slot_h) {
					set_next_recv_frame(slot_h);
					set_bit(IPU_PYM_STARTUP, &g_ipu_d_cdev->ipuflags);
				} else {
					spin_unlock_irqrestore(&g_ipu_d_cdev->slock, flags);
					mutex_unlock(&ipu_cdev->mutex_lock);
					ret = EFAULT;
					break;
				}
				spin_unlock_irqrestore(&g_ipu_d_cdev->slock, flags);
			}
			mutex_unlock(&ipu_cdev->mutex_lock);
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
			spin_lock_irqsave(&g_ipu_d_cdev->slock, flags);
			//stop
			ipu_drv_stop();
			wake_up_interruptible(&ipu_cdev->event_head);
			clear_bit(IPU_PYM_STARTUP, &g_ipu_d_cdev->ipuflags);
			//update
			ret = ipu_core_update(ipu_cfg);
			if (ret < 0) {
				spin_unlock_irqrestore(&g_ipu_d_cdev->slock,
							flags);
				ret = EFAULT;
				break;
			}
			//restart
			slot_h = recv_slot_free_to_busy();
			if (slot_h) {
				set_next_recv_frame(slot_h);
				set_bit(IPU_PYM_STARTUP, &g_ipu_d_cdev->ipuflags);
			} else {
				spin_unlock_irqrestore(&g_ipu_d_cdev->slock, flags);
				ret = EFAULT;
				break;
			}
			spin_unlock_irqrestore(&g_ipu_d_cdev->slock, flags);
			ipu_drv_start();
		}
		break;
	case IPUC_GET_MEM_INFO:
		{
			ipu_meminfo_t meminfo;
			meminfo.paddr = g_ipu->paddr;
			meminfo.memsize = g_ipu->memsize;
			meminfo.slot_num = g_ipu->slot_num;
			meminfo.slot_size = g_ipu->slot_size;
			ret = copy_to_user((void __user *)data, (const void *)&meminfo, sizeof(ipu_meminfo_t));
			if (ret) {
				ipu_err("copy to user fail\n");
			}
		}
		break;

	case IPUC_FB_SRC_DONE:
		{
			ipu_slot_dual_h_t *g_get_fb_slot_h;
			info_dual_h_t *pym_img_info = NULL;

			mutex_lock(&ipu_cdev->mutex_lock);
			// spin_lock_irqsave(&g_ipu_d_cdev->slock, flags);

			g_get_fb_slot_h = ipu_get_pym_free_slot();
			if (!g_get_fb_slot_h) {
				struct mult_img_info_t tmp_mult_img_info;

				ret = ipu_pym_wait_process_done(NULL,
						&tmp_mult_img_info,
						sizeof(struct mult_img_info_t), PYM_INLINE,
						ipu_cfg->timeout);
				if (ret < 0) {
					// spin_unlock_irqrestore(&g_ipu_d_cdev->slock, flags);
					mutex_unlock(&ipu_cdev->mutex_lock);
					return -EFAULT;
				}
				spin_lock_irqsave(&g_ipu_d_cdev->slock, flags);
				pym_img_info = &pym_info;
				pym_img_info->slot_id = tmp_mult_img_info.src_img_info[0].slot_id;
				pym_img_info->slot_flag = 0;
				pym_img_info->ipu_flag = 0;
				pym_img_info->cnn_flag = 0;
				pym_img_info->cf_id = tmp_mult_img_info.src_img_info[0].frame_id;
				pym_img_info->sf_id = tmp_mult_img_info.src_img_info[1].frame_id;
				pym_img_info->cf_timestamp = tmp_mult_img_info.src_img_info[0].timestamp;
				pym_img_info->sf_timestamp = tmp_mult_img_info.src_img_info[1].timestamp;
				pym_img_info->base =
					(uint64_t)IPU_GET_DUAL_SLOT(pym_img_info->slot_id, ipu->paddr);
				if (pym_img_info->slot_id == g_ipu_pym->new_slot_id)
					g_ipu_pym->new_slot_id = -1;

				pym_img_info->dual_ddr_info = tmp_ipu_user->dual_ddr_info;
				info = pym_img_info;
				spin_unlock_irqrestore(&g_ipu_d_cdev->slock, flags);
			} else {
				spin_lock_irqsave(&g_ipu_d_cdev->slock, flags);
				tmp_info = g_get_fb_slot_h->info_h; 
				tmp_info.dual_ddr_info = tmp_ipu_user->dual_ddr_info;
				tmp_info.base = (uint64_t)IPU_GET_DUAL_SLOT(tmp_info.slot_id, g_ipu->paddr);
				info = &tmp_info;
				spin_unlock_irqrestore(&g_ipu_d_cdev->slock, flags);
			}
			ret = copy_to_user((void __user *)data, (const void *)info,
			sizeof(info_dual_h_t));
			if (ret)
				ipu_err("copy to user fail\n");

			mutex_unlock(&ipu_cdev->mutex_lock);
		}
		break;

	case IPUC_FB_FLUSH:
	{
		int data_end;
		mult_img_info = &src_info;
		struct src_img_info_t *src_img_info;

		mutex_lock(&ipu_cdev->mutex_lock);
		ret = copy_from_user((void *)mult_img_info,
			(const void __user *)data,
		sizeof(struct mult_img_info_t));

		src_img_info = &mult_img_info->src_img_info[0];
		dma_sync_single_for_device(NULL,
		src_img_info->src_img.y_paddr,
		src_img_info->src_img.width*src_img_info->src_img.height+
		src_img_info->src_img.width*src_img_info->src_img.height/2,
			DMA_TO_DEVICE);
		__flush_dcache_area(page_address(pfn_to_page(PHYS_PFN((phys_addr_t)src_img_info->src_img.y_paddr))),
							src_img_info->src_img.width*src_img_info->src_img.height+
							src_img_info->src_img.width*src_img_info->src_img.height/2);
		src_img_info = &mult_img_info->src_img_info[1];
		dma_sync_single_for_device(NULL,
		src_img_info->src_img.y_paddr,
		src_img_info->src_img.width*src_img_info->src_img.height+
		src_img_info->src_img.width*src_img_info->src_img.height/2,
			DMA_TO_DEVICE);
		__flush_dcache_area(page_address(pfn_to_page(PHYS_PFN((phys_addr_t)src_img_info->src_img.y_paddr))),
							src_img_info->src_img.width*src_img_info->src_img.height+
							src_img_info->src_img.width*src_img_info->src_img.height/2);
		mutex_unlock(&ipu_cdev->mutex_lock);
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
	struct ipu_user *tmp_ipu_user = file->private_data;
	struct ipu_dual_cdev *ipu_cdev = tmp_ipu_user->host;
	unsigned int mask = 0;

	if (kfifo_len(&g_ipu_pym->done_inline_pym_slots)
			|| kfifo_len(&tmp_ipu_user->pym_user.done_offline_pym_slots)) {
		mask = EPOLLIN | EPOLLET;
		return mask;
	} else
		poll_wait(file, &g_ipu_d_cdev->event_head, wait);
	if (g_ipu->stop) {
		mask = EPOLLHUP;
		ipu_info("ipu exit request\n");
	} else if (g_ipu_d_cdev->err_status) {
		ipu_err("POLLERR: err_status 0x%x\n", g_ipu_d_cdev->err_status);
		mask = EPOLLERR;
		g_ipu_d_cdev->err_status = 0;
	} else if (kfifo_len(&g_ipu_pym->done_inline_pym_slots)
			|| kfifo_len(&tmp_ipu_user->pym_user.done_offline_pym_slots)) {
		mask = EPOLLIN | EPOLLET;
		return mask;
	}
	return mask;
}

int ipu_dual_close(struct inode *inode, struct file *filp)
{
	struct ipu_user *tmp_ipu_user = filp->private_data;
	struct ipu_dual_cdev *ipu_cdev = tmp_ipu_user->host;
	int slot_id;

	ipu_pym_user_exit(&tmp_ipu_user->pym_user);

	mutex_lock(&ipu_cdev->mutex_lock);
	for (slot_id = 0; slot_id < IPU_MAX_SLOT; slot_id++) {
		if (tmp_ipu_user->used_slot[slot_id]) {
			insert_dual_slot_to_free(slot_id, &ipu_cdev->s_info);
			tmp_ipu_user->used_slot[slot_id] = 0;
		}
	}
	mutex_unlock(&ipu_cdev->mutex_lock);

	kfree(tmp_ipu_user);
	filp->private_data = NULL;
	ipu_cdev->open_counter--;
	if (ipu_cdev->open_counter)
		return 0;

	ipu_cdev->ipu->ipu_mode = IPU_INVALID;
	ipu_pym_clear();
	g_ipu->vio_mode = IPU_INVALID;
	g_ipu->cfg->video_in.w = 0;
	g_ipu->cfg->video_in.h = 0;
	return 0;
}

static const struct file_operations ipu_dual_fops = {
	.owner = THIS_MODULE,
	.mmap = ipu_dual_mmap,
	.open = ipu_dual_open,
	.poll = ipu_dual_poll,
	.release = ipu_dual_close,
	.unlocked_ioctl = ipu_dual_ioctl,
	.compat_ioctl = ipu_dual_ioctl,
};

static int __init x2_ipu_dual_init(void)
{
	int ret = 0;
	struct device *dev = NULL;
	g_ipu_d_cdev = kzalloc(sizeof(struct ipu_dual_cdev), GFP_KERNEL);
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
	mutex_init(&g_ipu_d_cdev->mutex_lock);
	g_ipu->ipu_handle[IPU_DDR_DUAL] = ipu_dual_mode_process;
	if (diag_register(ModuleDiag_VIO, EventIdVioIpuDualErr,
					8, 380, 7200, NULL) < 0)
		pr_err("ipu dual diag register fail\n");

	ipu_pym_init();
	return ret;
}

static void __exit x2_ipu_dual_exit(void)
{
	ipu_pym_exit();
	device_destroy(g_ipu_d_cdev->class, g_ipu_d_cdev->dev_num);
	unregister_chrdev_region(g_ipu_d_cdev->dev_num, 1);
	class_destroy(g_ipu_d_cdev->class);
}

module_init(x2_ipu_dual_init);
module_exit(x2_ipu_dual_exit);

MODULE_DESCRIPTION("Driver for X2 IPU Dual Dev");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL");
