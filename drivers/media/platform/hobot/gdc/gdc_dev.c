/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/bug.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>
#include <linux/io.h>
#ifdef CONFIG_HOBOT_DIAG
#include <soc/hobot/diag.h>
#endif

#include "gdc_dev.h"
#include "gdc_hw_api.h"

#define MODULE_NAME "X3 GDC"
#define GDC_MAX_NUM 2
static int g_gdc_fps[GDC_MAX_NUM][VIO_MAX_STREAM] = {0, };
static int g_gdc_idx[GDC_MAX_NUM][VIO_MAX_STREAM] = {0, };
static int g_gdc_fps_lasttime[GDC_MAX_NUM][VIO_MAX_STREAM] = {0, };

static uint32_t gdc_default_color = 0x008080;
module_param(gdc_default_color, uint, 0644);

extern void write_gdc_mask(uint32_t model, uint32_t *enable);
extern void write_gdc_status(uint32_t model, uint32_t *enable);

typedef void(*rst_func)(void);
extern void gdc_call_install(rst_func p);

struct x3_gdc_dev *g_gdc_dev = NULL;

// PRQA S 0497,3238 ++
struct gdc_group *gdc_get_group(struct x3_gdc_dev *gdc)
{
	u32 stream;
	struct gdc_group *group;
	for (stream = 0; stream < VIO_MAX_STREAM; ++stream) {
		group = &gdc->group[stream];
		if (!test_bit(GDC_GROUP_OPEN, &group->state)) {
			group->instance = stream;
			return group;
		}
	}
	return NULL;
}

static int x3_gdc_open(struct inode *inode, struct file *file)
{
	struct gdc_video_ctx *gdc_ctx;
	struct x3_gdc_dev *gdc;
	struct gdc_group *group;
	int ret = 0;
	uint32_t enbale = 1u;

	gdc = container_of(inode->i_cdev, struct x3_gdc_dev, cdev);/*PRQA S ALL*/
	gdc_ctx = kzalloc(sizeof(struct gdc_video_ctx), GFP_KERNEL);
	if (gdc_ctx == NULL) {
		vio_err("kzalloc is fail");
		ret = -ENOMEM;
		goto p_err;
	}

	group = gdc_get_group(gdc);
	if (group == NULL) {
		ret = -EBUSY;
		vio_err("can get free group\n");
		goto p_err_free;
	}

	group->sub_ctx[0] = gdc_ctx;
	gdc_ctx->group = group;
	set_bit(GDC_GROUP_OPEN, &group->state);

	gdc_ctx->gdc_dev = gdc;
	file->private_data = gdc_ctx;

	if (atomic_inc_return(&gdc->open_cnt) == 1) {
		mutex_lock(&gdc->gdc_mutex);
		vio_dwe_clk_enable();
		vio_gdc_clk_enable(gdc->hw_id);
		write_gdc_mask(gdc->hw_id, &enbale);
		mutex_unlock(&gdc->gdc_mutex);
	}

	vio_info("GDC%d open node\n", gdc->hw_id);

	return ret;
p_err_free:
	kfree(gdc_ctx);
p_err:
	return ret;
}

static ssize_t x3_gdc_write(struct file *file, const char __user * buf,
			     size_t count, loff_t * ppos)
{
	return 0;
}

static ssize_t x3_gdc_read(struct file *file, char __user * buf, size_t size,
			    loff_t * ppos)
{
	return 0;

}

static int x3_gdc_close(struct inode *inode, struct file *file)
{
	struct gdc_video_ctx *gdc_ctx;
	struct gdc_group *group;
	struct x3_gdc_dev *gdc;
	int enbale = 0u;
	int ret = 0;

	gdc_ctx = file->private_data;
	if (gdc_ctx == NULL) {
		vio_err("gdc ctx was null\n");
		goto p_err;
	}
	group = gdc_ctx->group;

	clear_bit(GDC_GROUP_OPEN, &group->state);

	gdc = gdc_ctx->gdc_dev;

	if (atomic_dec_return(&gdc->open_cnt) == 0) {
		mutex_lock(&gdc->gdc_mutex);
		write_gdc_mask(gdc->hw_id, &enbale);
		vio_gdc_clk_disable(gdc->hw_id);
		vio_dwe_clk_disable();
		mutex_unlock(&gdc->gdc_mutex);
	}

p_err:
	if (gdc_ctx) {
		kfree(gdc_ctx);
		gdc_ctx = NULL;
	}

	return ret;
}

void dwe0_reset_control(void)
{
	mutex_lock(&g_gdc_dev->gdc_mutex);
	ips_set_module_reset(DWE0_RST);
	mutex_unlock(&g_gdc_dev->gdc_mutex);
}

/**
 *   This function starts the gdc block
 *
 *   Writing 0->1 transition is necessary for trigger
 *
 *   @param  gdc_settings - overall gdc settings and state
 *
 */
void gdc_start(struct x3_gdc_dev *gdc_dev)
{
	gdc_process_enable(gdc_dev->base_reg, 0);
	gdc_process_enable(gdc_dev->base_reg, 1);
	// vio_set_stat_info(0, GDC_FS, 0);
	vio_dbg("%s\n", __func__);/*PRQA S ALL*/
}

/**
 *   Configure the output gdc configuration address/size and buffer address/size; and resolution.
 *
 *   More than one gdc settings can be accessed by index to a gdc_config_t.
 *
 *   @return 0 - success
 *           -1
 */

void gdc_init(struct x3_gdc_dev *gdc_dev, gdc_settings_t *gdc_settings)
{
	void __iomem *base_addr;

	base_addr = gdc_dev->base_reg;

	gdc_process_enable(base_addr, 0);
	gdc_process_reset(base_addr, 1);
	gdc_process_reset(base_addr, 0);
	gdc_set_config_addr(base_addr, gdc_settings->gdc_config.config_addr);
	gdc_set_config_size(base_addr,
			    gdc_settings->gdc_config.config_size / 4);
	gdc_set_rdma_img_width(base_addr, gdc_settings->gdc_config.input_width);
	gdc_set_rdma_img_height(base_addr,
				gdc_settings->gdc_config.input_height);
	gdc_set_wdma_img_width(base_addr,
			       gdc_settings->gdc_config.output_width);
	gdc_set_wdma_img_height(base_addr,
				gdc_settings->gdc_config.output_height);
	gdc_set_default_ch1(base_addr, (gdc_default_color >> 16) & 0xff);
	gdc_set_default_ch2(base_addr, (gdc_default_color >> 8) & 0xff);
	gdc_set_default_ch3(base_addr, gdc_default_color & 0xff);
	vio_dbg("GDC config_addr:%x,config_size:%x\n",/*PRQA S ALL*/
		gdc_settings->gdc_config.config_addr,
		gdc_settings->gdc_config.config_size);
	vio_dbg("gdc in w:%d, h:%d\n",/*PRQA S ALL*/
		gdc_settings->gdc_config.input_width,
		gdc_settings->gdc_config.input_height);
	vio_dbg("gdc out w:%d, h:%d\n",/*PRQA S ALL*/
		gdc_settings->gdc_config.output_width,
		gdc_settings->gdc_config.output_height);
}

/**
 *   This function points gdc to its input resolution and yuv address and offsets
 *
 *   Shown inputs to GDC are Y and UV plane address and offsets
 *
 *
 *   @return 0 - success
 *           -1 - no interrupt from GDC.
 */

int gdc_check_phyaddr(u32 addr)
{
	int ret = 0;

	ret = ion_check_in_heap_carveout(addr, 0);
	if (ret < 0) {
		vio_err("gdc phyaddr 0x%x is beyond ion address region\n",
				addr);
	}

	return ret;
}

int gdc_check(gdc_settings_t *gdc_settings)
{
	int ret = 0;

	// u32 lineoffset, height;	//, size;
	u32 num_input = 0;
	u32 *input_addr;
	u32 *output_addr;

	num_input = gdc_settings->total_planes;
	input_addr = gdc_settings->In_buffer_addr;
	output_addr = gdc_settings->Out_buffer_addr;

	//process input addresses
	if (num_input >= 1) {
		ret += gdc_check_phyaddr(input_addr[0]);
	}
	if (num_input >= 2 && gdc_settings->gdc_config.sequential_mode == 0) {
		ret += gdc_check_phyaddr(input_addr[1]);
	}
	if (num_input >= 3 && gdc_settings->gdc_config.sequential_mode == 0) {
		ret += gdc_check_phyaddr(input_addr[2]);
	}
	//outputs
	if (num_input >= 1) {
		ret += gdc_check_phyaddr(output_addr[0]);
	}

	if (num_input >= 2 && gdc_settings->gdc_config.sequential_mode == 0) {
		ret += gdc_check_phyaddr(output_addr[1]);
	}

	if (num_input >= 3 && gdc_settings->gdc_config.sequential_mode == 0) {
		ret += gdc_check_phyaddr(output_addr[2]);
	}

	return ret;
}

int gdc_process(struct x3_gdc_dev *gdc_dev, gdc_settings_t *gdc_settings)
{
	void __iomem *base_addr;

	u32 lineoffset, height;	//, size;
	u32 num_input = 0;
	u32 *input_addr;

	num_input = gdc_settings->total_planes;
	input_addr = gdc_settings->In_buffer_addr;
	base_addr = gdc_dev->base_reg;

	//process input addresses
	if (num_input >= 1) {
		lineoffset = gdc_settings->gdc_config.input_stride;
		height = gdc_settings->gdc_config.input_height;
		if (gdc_settings->gdc_config.sequential_mode == 1){
			gdc_set_rdma0_img_addr(base_addr,
				input_addr[gdc_settings->seq_planes_pos]);
			if (gdc_settings->seq_planes_pos > 0){	//UV planes
				lineoffset = gdc_settings->gdc_config.output_width
				    >> gdc_settings->gdc_config.div_width;
				height = gdc_settings->gdc_config.output_height
				    >> gdc_settings->gdc_config.div_height;
			}
		} else{
			gdc_set_rdma0_img_addr(base_addr, input_addr[0]);
			vio_dbg("GDC input1 addr:%x\n", input_addr[0]);/*PRQA S ALL*/
		}
		gdc_set_rdma0_line_offset(base_addr, lineoffset);
		vio_dbg("GDC data1in lineoffset:%d, height:%d\n",/*PRQA S ALL*/
			lineoffset, height);
	}
	if (num_input >= 2 && gdc_settings->gdc_config.sequential_mode == 0) {			//only processed if not in toggle mode
		vio_dbg("GDC plain_num:%d case\n", num_input);/*PRQA S ALL*/
		lineoffset = gdc_settings->gdc_config.input_stride
			>> gdc_settings->gdc_config.div_width;
		height = gdc_settings->gdc_config.input_height
			>> gdc_settings->gdc_config.div_height;

		gdc_set_rdma1_img_addr(base_addr, input_addr[1]);
		gdc_set_rdma1_line_offset(base_addr, lineoffset);
		vio_dbg("GDC input2 addr:%x\n", input_addr[1]);/*PRQA S ALL*/
		vio_dbg("GDC data2in lineoffset:%d, height:%d\n",/*PRQA S ALL*/
			lineoffset, height);
	}
	if (num_input >= 3 && gdc_settings->gdc_config.sequential_mode == 0) {			//only processed if not in toggle mode
		lineoffset = gdc_settings->gdc_config.input_stride
			>> gdc_settings->gdc_config.div_width;
		height = gdc_settings->gdc_config.input_height
			>> gdc_settings->gdc_config.div_height;
		gdc_set_rdma2_img_addr(base_addr, input_addr[2]);
		gdc_set_rdma2_line_offset(base_addr, lineoffset);
		vio_dbg("GDC data3in lineoffset:%d, height:%d\n",/*PRQA S ALL*/
			lineoffset, height);
	}
	//outputs
	if (num_input >= 1) {
		lineoffset = gdc_settings->gdc_config.output_stride;
		height = gdc_settings->gdc_config.output_height;
		vio_dbg("GDC out1: plain_num:%d case, lineoffset:%d,"/*PRQA S ALL*/
			" height:%d\n",
			 num_input, lineoffset, height);
		if (gdc_settings->gdc_config.sequential_mode == 1
		    && gdc_settings->seq_planes_pos > 0){		//UV planes
			lineoffset = gdc_settings->gdc_config.output_stride
				>> gdc_settings->gdc_config.div_width;
			height = gdc_settings->gdc_config.output_height
				>> gdc_settings->gdc_config.div_height;
		}
		gdc_set_wdma0_img_addr(base_addr,
				       gdc_settings->Out_buffer_addr[0]);
		gdc_set_wdma0_line_offset(base_addr, lineoffset);
		vio_dbg("GDC out1: data1out_addr_write:%x\n",/*PRQA S ALL*/
			 gdc_settings->Out_buffer_addr[0]);
		vio_dbg("GDC out1 lineoffset:%d, height:%d\n",/*PRQA S ALL*/
			lineoffset, height);
	}

	if (num_input >= 2 && gdc_settings->gdc_config.sequential_mode == 0) {
		lineoffset = gdc_settings->gdc_config.output_stride
			>> gdc_settings->gdc_config.div_width;
		height = gdc_settings->gdc_config.output_height
			>> gdc_settings->gdc_config.div_height;
		vio_dbg("GDC out2: plain_num:%d case, lineoffset:%d, "/*PRQA S ALL*/
			"height:%d\n",
			 num_input, lineoffset, height);
		gdc_set_wdma1_img_addr(base_addr,
				       gdc_settings->Out_buffer_addr[1]);
		gdc_set_wdma1_line_offset(base_addr, lineoffset);
		vio_dbg("GDC out2: data2out_addr_write:%x\n",/*PRQA S ALL*/
			 gdc_settings->Out_buffer_addr[1]);
		vio_dbg("GDC out2 lineoffset:%d, height:%d\n",/*PRQA S ALL*/
			lineoffset, height);
	}

	if (num_input >= 3 && gdc_settings->gdc_config.sequential_mode == 0) {
		lineoffset = gdc_settings->gdc_config.output_width
			>> gdc_settings->gdc_config.div_width;
		height = gdc_settings->gdc_config.output_height
			>> gdc_settings->gdc_config.div_height;
		gdc_set_wdma2_img_addr(base_addr,
				       gdc_settings->Out_buffer_addr[2]);
		gdc_set_wdma2_line_offset(base_addr, lineoffset);
		vio_dbg("out2: data2out_addr_write:%x\n",/*PRQA S ALL*/
			 gdc_settings->Out_buffer_addr[1]);
		vio_dbg("GDC out3 lineoffset:%d, height:%d\n",/*PRQA S ALL*/
			lineoffset, height);
	}

	if (gdc_settings->gdc_config.sequential_mode == 1) {			//update the planes position
		if (++(gdc_settings->seq_planes_pos) >=
		    gdc_settings->gdc_config.total_planes)
			gdc_settings->seq_planes_pos = 0;
	}

	return 0;

}

void gdc_set_iar_output(struct x3_gdc_dev *gdc_dev,
		gdc_settings_t *gdc_settings)
{
#ifdef X3_IAR_INTERFACE
	u8 dis_instance[2] = {0, 0};
	u8 display_layer[2] ={0, 0};
	int ret = 0;

	if (iar_get_type == NULL || iar_set_addr == NULL)
		return;
	ret = iar_get_type(dis_instance, display_layer);
	if (!ret) {
		if (gdc_dev->hw_id == (display_layer[0] - 37) ||
				gdc_dev->hw_id == (display_layer[1] - 37))
			iar_set_addr(0, gdc_settings->Out_buffer_addr[0],
				gdc_settings->Out_buffer_addr[1]);
	}
	vio_dbg("GDC display_layer = %d, dis_instance = %d, ret(%d)",/*PRQA S ALL*/
		display_layer[0], dis_instance[0], ret);
#endif
}

int gdc_video_process(struct gdc_video_ctx *gdc_ctx, unsigned long arg)
{
	int ret = 0;
	int timeout = 0;
	gdc_settings_t gdc_settings;
	struct x3_gdc_dev *gdc_dev;
	uint32_t enbale = 1u;
	struct timeval tmp_tv;

	ret = copy_from_user(&gdc_settings, (gdc_settings_t *) arg,
			   sizeof(gdc_settings_t));
	if (ret) {
		vio_err("GDC_IOCTL_PROCESS copy from user failed (ret=%d)\n",
			 ret);
		return -EFAULT;
	}

	gdc_dev = gdc_ctx->gdc_dev;

	ret = gdc_check(&gdc_settings);
	if (ret) {
		vio_err("%s gdc check fail(%d)", __func__, ret);
		goto p_err_ignore;
	}

	ret = down_interruptible(&gdc_dev->smp_gdc_enable);
	if (ret) {
		vio_err("%s down fail(%d)", __func__, ret);
		goto p_err_ignore;
	}

	mutex_lock(&gdc_dev->gdc_mutex);
	gdc_dev->state = GDC_DEV_PROCESS;
	gdc_dev->isr_err = 0;
	gdc_ctx->event = 0;
	atomic_set(&gdc_dev->instance, gdc_ctx->group->instance);

	write_gdc_mask(gdc_dev->hw_id, &enbale);
	gdc_init(gdc_dev, &gdc_settings);
	ret = gdc_process(gdc_dev, &gdc_settings);
	gdc_start(gdc_dev);
	vio_set_stat_info(gdc_ctx->group->instance,
		GDC_MOD, event_gdc_fs,  0, 0, NULL);

	timeout = down_timeout(&gdc_dev->gdc_done_resource,/*PRQA S ALL*/
				msecs_to_jiffies(GDC_PROCESS_TIMEOUT));
	if (timeout < 0) {
		mutex_unlock(&gdc_dev->gdc_mutex);
		up(&gdc_dev->smp_gdc_enable);
		vio_err("[S%d]%s timeout or Abnormal interrupt, ;\n",
			gdc_ctx->group->instance, __func__);
		return -ETIMEDOUT;
	}
	gdc_dev->state = GDC_DEV_FREE;

	if (gdc_ctx->event == VIO_FRAME_DONE) {/*PRQA S ALL*/
		gdc_set_iar_output(gdc_dev, &gdc_settings);/*PRQA S ALL*/
	} else {
		vio_err("GDC process failed\n");
		ret = -gdc_dev->isr_err;
	}
	vio_set_stat_info(gdc_ctx->group->instance,
		GDC_MOD, event_gdc_fe, 0, 0, NULL);

	do_gettimeofday(&tmp_tv);
	g_gdc_idx[gdc_dev->hw_id][gdc_ctx->group->instance]++;
	if (tmp_tv.tv_sec >
		g_gdc_fps_lasttime[gdc_dev->hw_id][gdc_ctx->group->instance]) {
		g_gdc_fps[gdc_dev->hw_id][gdc_ctx->group->instance] =
				g_gdc_idx[gdc_dev->hw_id][gdc_ctx->group->instance];
		g_gdc_fps_lasttime[gdc_dev->hw_id][gdc_ctx->group->instance] =
				tmp_tv.tv_sec;
		g_gdc_idx[gdc_dev->hw_id][gdc_ctx->group->instance] = 0;
	}

	mutex_unlock(&gdc_dev->gdc_mutex);
	up(&gdc_dev->smp_gdc_enable);
	vio_dbg("%s done: ret(%d), timeout %d\n",/*PRQA S ALL*/
		__func__, ret, timeout);
p_err_ignore:
	return ret;
}

static long x3_gdc_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct gdc_video_ctx *gdc_ctx;
	struct gdc_group *group;
	int ret = 0;

	gdc_ctx = file->private_data;
	group = gdc_ctx->group;

	if (_IOC_TYPE(cmd) != GDC_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case GDC_IOC_PROCESS:/*PRQA S ALL*/
		ret = gdc_video_process(gdc_ctx, arg);
		break;
	default:
		vio_err("wrong ioctl cmd for GDC\n");
		break;
	}

	return ret;
}

#ifdef CONFIG_HOBOT_DIAG
static void gdc_diag_report(uint8_t errsta, unsigned int status,
	uint32_t instance, uint32_t hw_id, uint32_t err_type)
{
	uint8_t env_data[8];
	env_data[0] = instance;
	env_data[2] = 0;
	env_data[3] = sizeof(uint32_t);
	env_data[4] = status & 0xff;
	env_data[5] = (status >> 8) & 0xff;
	env_data[6] = (status >> 16) & 0xff;
	env_data[7] = (status >> 24) & 0xff;

	if (errsta) {
		env_data[1] = (uint8_t)err_type;
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_VIO,
				EventIdVioGdc0Err + hw_id,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				env_data,
				sizeof(env_data));
	} else {
		env_data[1] = 0xff;
		diag_send_event_stat_and_env_data(
				DiagMsgPrioMid,
				ModuleDiag_VIO,
				EventIdVioGdc0Err + hw_id,
				DiagEventStaSuccess,
				DiagGenEnvdataWhenErr,
				env_data,
				sizeof(env_data));
	}

	return;
}
#endif

static irqreturn_t gdc_isr(int irq, void *data)
{
	u32 status;
	u32 dwe_status = 1;
	u32 instance;
	struct x3_gdc_dev *gdc;
	struct gdc_group *gdc_group;
	struct gdc_video_ctx *gdc_ctx;

	gdc = data;
	instance = atomic_read(&gdc->instance);
	gdc_group = &gdc->group[instance];
	gdc_ctx = gdc_group->sub_ctx[0];
	gdc_ctx->event = VIO_FRAME_DONE;

	status = gdc_get_intr_status(gdc->base_reg);
	write_gdc_status(gdc->hw_id, &dwe_status);

	vio_dbg("%s:status = 0x%x, dwe_status = 0x%x\n",/*PRQA S ALL*/
		__func__, status, dwe_status);

	if (status & 1 << INTR_GDC_BUSY) {
		vio_info("GDC current frame is processing\n");
	} else if (status & 1 << INTR_GDC_ERROR) {
		if (status & 1 << INTR_GDC_CONF_ERROR) {
			vio_err("GDC configuration error\n");
			gdc->isr_err = INTR_GDC_CONF_ERROR;
		}

		if (status & 1 << INTR_GDC_USER_ABORT) {
			vio_err("GDC user abort(stop/reset command)\n");
			gdc->isr_err = INTR_GDC_USER_ABORT;
		}

		if (status & 1 << INTR_GDC_AXI_READER_ERROR) {
			vio_err("GDC AXI reader error\n");
			gdc->isr_err = INTR_GDC_AXI_READER_ERROR;
		}

		if (status & 1 << INTR_GDC_AXI_WRITER_ERROR) {
			vio_err("GDC AXI writer error\n");
			gdc->isr_err = INTR_GDC_AXI_WRITER_ERROR;
		}

		if (status & 1 << INTR_GDC_UNALIGNED_ACCESS) {
			vio_err("GDC address pionter is not aligned\n");
			gdc->isr_err = INTR_GDC_UNALIGNED_ACCESS;
		}

		if (status & 1 << INTR_GDC_INCOMPATIBLE_CONF) {
			vio_err("GDC incopatible configuration\n");
			gdc->isr_err = INTR_GDC_INCOMPATIBLE_CONF;
		}

		gdc_ctx->event = VIO_FRAME_NDONE;
#ifdef CONFIG_HOBOT_DIAG
		atomic_set(&gdc->diag_state, 1);
		gdc_diag_report(1, status, atomic_read(&gdc->instance), gdc->hw_id,
			gdc->isr_err);
#endif
	} else {
#ifdef CONFIG_HOBOT_DIAG
		if (atomic_read(&gdc->diag_state) == 1) {
			atomic_set(&gdc->diag_state, 0);
			gdc_diag_report(0, status, atomic_read(&gdc->instance), gdc->hw_id,
				gdc->isr_err);
		}
#endif
	}

	if (gdc_ctx->event) {
		up(&gdc->gdc_done_resource);
	}

	return IRQ_HANDLED;
}

static struct file_operations x3_gdc_fops = {
	.owner = THIS_MODULE,
	.open = x3_gdc_open,
	.write = x3_gdc_write,
	.read = x3_gdc_read,
	.release = x3_gdc_close,
	.unlocked_ioctl = x3_gdc_ioctl,
	.compat_ioctl = x3_gdc_ioctl,
};

static int x3_gdc_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);
	if (g_gdc_dev)
		vio_irq_affinity_set(g_gdc_dev->irq, MOD_GDC, 1, 0);

	return ret;
}

static int x3_gdc_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);
	if (g_gdc_dev)
		vio_irq_affinity_set(g_gdc_dev->irq, MOD_GDC, 0, 0);

	return ret;
}

static int x3_gdc_runtime_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_gdc_runtime_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static const struct dev_pm_ops x3_gdc_pm_ops = {
	.suspend = x3_gdc_suspend,
	.resume = x3_gdc_resume,
	.runtime_suspend = x3_gdc_runtime_suspend,
	.runtime_resume = x3_gdc_runtime_resume,
};

int x3_gdc_device_node_init(struct x3_gdc_dev *gdc)
{
	int ret = 0;
	struct device *dev = NULL;
	char name[32];

	snprintf(name, 32, "gdc%d", gdc->hw_id);

	ret = alloc_chrdev_region(&gdc->devno, 0, GDC_MAX_DEVICE, name);
	if (ret < 0){
		vio_err("Error %d while alloc chrdev gdc", ret);
		goto err_req_cdev;
	}

	cdev_init(&gdc->cdev, &x3_gdc_fops);
	gdc->cdev.owner = THIS_MODULE;
	ret = cdev_add(&gdc->cdev, gdc->devno, GDC_MAX_DEVICE);
	if (ret){
		vio_err("Error %d while adding x2 gdc cdev", ret);
		goto err;
	}

	if (vps_class)
		gdc->class = vps_class;
	else
		gdc->class = class_create(THIS_MODULE, name);/*PRQA S ALL*/

	dev = device_create(gdc->class, NULL, MKDEV(MAJOR(gdc->devno), 0),
		NULL, name);
	if (IS_ERR(dev)){
		ret = -EINVAL;
		vio_err("gdc device create fail\n");
		goto err;
	}

	return ret;
err:
	class_destroy(gdc->class);
err_req_cdev:
	unregister_chrdev_region(gdc->devno, GDC_MAX_DEVICE);
	return ret;
}

static ssize_t gdc_reg_dump(struct device *dev,struct device_attribute *attr, char* buf)
{
	struct x3_gdc_dev *gdc;

	gdc = dev_get_drvdata(dev);

	gdc_hw_dump(gdc->base_reg);

	return 0;
}

static DEVICE_ATTR(regdump, 0444, gdc_reg_dump, NULL);/*PRQA S ALL*/

static ssize_t gdc_fps_show(struct device *dev,
				struct device_attribute *attr, char* buf)
{
	u32 offset = 0;
	int i, j, len;
	for (j = 0; j < GDC_MAX_NUM; j++) {
		for (i = 0; i < VIO_MAX_STREAM; i++) {
			if (g_gdc_fps[j][i] > 0) {
				len = snprintf(&buf[offset], PAGE_SIZE - offset,
					"gdc%d pipe %d: output fps %d\n", j, i, g_gdc_fps[j][i]);
				offset += len;
				g_gdc_fps[j][i] = 0;
			}
		}
	}
	return offset;
}

static DEVICE_ATTR(fps, S_IRUGO, gdc_fps_show, NULL);/*PRQA S ALL*/

static int x3_gdc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct x3_gdc_dev *gdc;
	struct resource *mem_res;
	struct device_node *dnode;
	struct device *dev;

	BUG_ON(!pdev);

	dev = &pdev->dev;
	dnode = dev->of_node;

	gdc = kzalloc(sizeof(struct x3_gdc_dev), GFP_KERNEL);
	if (!gdc) {
		vio_err("gdc is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		vio_err("Failed to get io memory region(%p)", mem_res);
		ret = -EBUSY;
		goto err_get_resource;
	}

	gdc->regs_start = mem_res->start;
	gdc->regs_end = mem_res->end;
	gdc->base_reg =devm_ioremap_nocache(&pdev->dev,
		mem_res->start, resource_size(mem_res));
	if (!gdc->base_reg) {
		vio_err("Failed to remap io region(%p)", gdc->base_reg);
		ret = -ENOMEM;
		goto err_get_resource;
	}

	/* Get IRQ SPI number */
	gdc->irq = platform_get_irq(pdev, 0);
	if (gdc->irq < 0) {
		vio_err("Failed to get gdc_irq(%d)", gdc->irq);
		ret = -EBUSY;
		goto err_get_irq;
	}

	ret = request_irq(gdc->irq, gdc_isr, IRQF_TRIGGER_HIGH, "gdc", gdc);
	if (ret) {
		vio_err("request_irq(IRQ_GDC %d) is fail(%d)", gdc->irq, ret);
		goto err_get_irq;
	}

	vio_irq_affinity_set(gdc->irq, MOD_GDC, 0, 0);

#ifdef CONFIG_OF
	ret = of_property_read_u32(dnode, "id", &gdc->hw_id);
	if (ret){
		vio_err("id read is fail(%d)", ret);
	}
#endif

	x3_gdc_device_node_init(gdc);

	ret = device_create_file(dev, &dev_attr_regdump);
	if(ret < 0) {
		vio_err("create regdump failed (%d)\n",ret);
		goto p_err;
	}

	ret = device_create_file(dev, &dev_attr_fps);
	if(ret < 0) {
		vio_err("create fps failed (%d)\n", ret);
		goto p_err;
	}

	platform_set_drvdata(pdev, gdc);

	sema_init(&gdc->smp_gdc_enable, 1);
	sema_init(&gdc->gdc_done_resource, 0);
	mutex_init(&gdc->gdc_mutex);
	atomic_set(&gdc->open_cnt, 0);

	if (gdc->hw_id == 0)
		g_gdc_dev = gdc;

	gdc_call_install(dwe0_reset_control);
	/* gdc clock default is open, close it*/
	ips_set_clk_ctrl(GDC0_CLOCK_GATE - gdc->hw_id, true);
	ips_set_clk_ctrl(GDC0_CLOCK_GATE - gdc->hw_id, false);
	vio_info("[FRT:D] %s(%d)\n", __func__, ret);

#ifdef CONFIG_HOBOT_DIAG
	if (diag_register(ModuleDiag_VIO, EventIdVioGdc0Err + gdc->hw_id,
					4, 74, DIAG_MSG_INTERVAL_MAX, NULL) < 0) {
		vio_err("GDC diag register fail\n");
	}
	atomic_set(&gdc->diag_state, 0);
#endif

	return 0;

err_get_irq:
	iounmap(gdc->base_reg);

err_get_resource:
	kfree(gdc);
p_err:
	vio_err("[FRT:D] %s(%d)\n", __func__, ret);
	return ret;

}

static int x3_gdc_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct x3_gdc_dev *gdc;

	BUG_ON(!pdev);

	gdc = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_regdump);
	device_remove_file(&pdev->dev, &dev_attr_fps);

	free_irq(gdc->irq, gdc);

	mutex_destroy(&gdc->gdc_mutex);
	device_destroy(gdc->class, gdc->devno);
	if (!vps_class)
		class_destroy(gdc->class);
	cdev_del(&gdc->cdev);
	unregister_chrdev_region(gdc->devno, GDC_MAX_DEVICE);
	kfree(gdc);

	vio_info("%s\n", __func__);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id x3_gdc_match[] = {
	{
	 .compatible = "hobot,gdc",
	 },
	{},
};

MODULE_DEVICE_TABLE(of, x3_gdc_match);

static struct platform_driver x3_gdc_driver = {
	.probe = x3_gdc_probe,
	.remove = x3_gdc_remove,
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &x3_gdc_pm_ops,
		   .of_match_table = x3_gdc_match,
		   }
};

#else
static struct platform_device_id x3_gdc_driver_ids[] = {
	{
	 .name = MODULE_NAME,
	 .driver_data = 0,
	 },
	{},
};

MODULE_DEVICE_TABLE(platform, x3_gdc_driver_ids);

static struct platform_driver x3_gdc_driver = {
	.probe = x3_gdc_probe,
	.remove = __devexit_p(x3_gdc_remove),
	.id_table = x3_gdc_driver_ids,
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &x3_gdc_pm_ops,
		   }
};
#endif

static int __init x3_gdc_init(void)
{
	int ret = platform_driver_register(&x3_gdc_driver);
	if (ret)
		vio_err("platform_driver_register failed: %d\n", ret);

	return ret;
}

late_initcall(x3_gdc_init);/*PRQA S ALL*/

static void __exit x3_gdc_exit(void)
{
	platform_driver_unregister(&x3_gdc_driver);
}

module_exit(x3_gdc_exit);/*PRQA S ALL*/

MODULE_AUTHOR("Sun Kaikai<kaikai.sun@horizon.com>");
MODULE_DESCRIPTION("X3 GDC driver");
MODULE_LICENSE("GPL v2");
// PRQA S --
