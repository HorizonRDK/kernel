#ifndef __HOBOT_SIF_H__
#define __HOBOT_SIF_H__

#include <uapi/linux/types.h>
#include <linux/cdev.h>
#include <linux/wait.h>

#include "sif_config.h"
#include "vio_config.h"
#include "vio_framemgr.h"
#include "vio_group_api.h"

#define SIF_MUX_MAX	8 
#define X2A_SIF_NAME  "x2a-sif"

#define MAX_DEVICE  2

#define SIF_IOC_MAGIC 'x'

#define SIF_IOC_INIT             _IOW(SIF_IOC_MAGIC, 0, sif_cfg_t)
#define SIF_IOC_STREAM           _IOW(SIF_IOC_MAGIC, 1, int)
#define SIF_IOC_QBUF        	 _IOW(SIF_IOC_MAGIC, 2, int)
#define SIF_IOC_DQBUF       	 _IOR(SIF_IOC_MAGIC, 3, int)
#define SIF_IOC_REQBUFS       	 _IOW(SIF_IOC_MAGIC, 4, int)
#define SIF_IOC_BIND_GROUP       _IOW(SIF_IOC_MAGIC, 5, int)
#define SIF_IOC_END_OF_STREAM    _IOW(SIF_IOC_MAGIC, 6, int)


#define SIF_YUV_INPUT	8

struct sif_irq_src {
	u32 sif_frm_int;
	u32 sif_out_int;
	u32 sif_err_status;
	u32 sif_in_buf_overflow;
};

struct sif_multi_frame {
	u8 period;
	u8 mux_index;
	u8 trigger_mode;
	u8 enable;
};

enum sif_out_interrupt_map{
	SIF_ISP_OUT_FS,
	SIF_ISP_OUT_FE,
	SIF_IPU_OUT_FS = 4,
	SIF_IPU_OUT_FE = 5,
};

enum sif_frame_interrupt_map{
	INTR_SIF_MUX0_OUT_FS,
	INTR_SIF_MUX1_OUT_FS,
	INTR_SIF_MUX2_OUT_FS,
	INTR_SIF_MUX3_OUT_FS,
	INTR_SIF_MUX4_OUT_FS,
	INTR_SIF_MUX5_OUT_FS,
	INTR_SIF_MUX6_OUT_FS,
	INTR_SIF_MUX7_OUT_FS,
	INTR_SIF_MUX0_FRAME_DONE,
	INTR_SIF_MUX1_FRAME_DONE,
	INTR_SIF_MUX2_FRAME_DONE,
	INTR_SIF_MUX3_FRAME_DONE,
	INTR_SIF_MUX4_FRAME_DONE,
	INTR_SIF_MUX5_FRAME_DONE,
	INTR_SIF_MUX6_FRAME_DONE,
	INTR_SIF_MUX7_FRAME_DONE,
	INTR_SIF_MIPI_TX_IPI0_FS,
	INTR_SIF_MIPI_TX_IPI1_FS,
	INTR_SIF_MIPI_TX_IPI2_FS,
	INTR_SIF_MIPI_TX_IPI3_FS,
	INTR_SIF_OUT_BUF_ERROR = 27,
	INTR_SIF_IN_SIZE_MISMATCH = 28,
	INTR_SIF_IN_OVERFLOW = 29,
	INTR_SIF_MULTI_FRAME_ID = 31,
};

enum sif_group_state {
	SIF_GROUP_OPEN,
	SIF_GROUP_INIT,
	SIF_GROUP_START,
	SIF_GROUP_SHOT,
	SIF_GROUP_REQUEST_FSTOP,
	SIF_GROUP_FORCE_STOP,
	SIF_GROUP_OTF_INPUT,
	SIF_GROUP_OTF_OUTPUT,
	SIF_GROUP_LEADER,
};

struct sif_video_ctx{
	wait_queue_head_t		done_wq;
	struct vio_framemgr 	framemgr;
	struct vio_group		*group;
	unsigned long			state;

	u32 mux_index;
	u32 dol_num;
	u32 id;
	u32 rx_num;
	bool initial_frameid;
	u64 bufcount;
	struct frame_id			info;
	struct x2a_sif_dev 		*sif_dev;
	struct semaphore 		smp_resource;
};

enum sif_state {
	/* one the fly output */
	SIF_OTF_OUTPUT = 10,
	/* WDMA flag */
	SIF_DMA_IN_ENABLE,
	SIF_DOL2_MODE,
};

struct x2a_sif_dev {
	u32 __iomem			*base_reg;
	resource_size_t			regs_start;
	resource_size_t			regs_end;
	int				irq;

	struct cdev cdev;
	struct class *class;

	unsigned long		state;	
	atomic_t			instance;
	atomic_t			rsccount;
	spinlock_t			shared_slock;

	struct vio_group		*sif_input[VIO_MAX_STREAM];
	struct vio_group		*sif_mux[SIF_MUX_MAX];

	struct vio_group_task	sifin_task;
	struct vio_group_task	sifout_task[SIF_MUX_MAX];
};

#endif
