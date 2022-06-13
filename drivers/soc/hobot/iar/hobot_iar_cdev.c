/***************************************************************************
 *						COPYRIGHT NOTICE
 *			   Copyright 2018 Horizon Robotics, Inc.
 *					   All rights reserved.
 ***************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <linux/mm.h>
#include <linux/types.h>
#include <linux/major.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/fb.h>
#include <asm/io.h>
#include <linux/mutex.h>
#include <asm/cacheflush.h>
#include <soc/hobot/hobot_iar.h>

#define IAR_DMA_MODE

#define IAR_CDEV_MAGIC 'R'
#define IAR_GETVALUE		_IOR(IAR_CDEV_MAGIC,0x11, unsigned int)
#define IAR_START			_IO(IAR_CDEV_MAGIC,0x12)
#define IAR_STOP			_IO(IAR_CDEV_MAGIC,0x13)
#define IAR_CHANNEL_CFG 	_IOW(IAR_CDEV_MAGIC,0x14, channel_base_cfg_t)
#define IAR_DISPLAY_UPDATE	_IOW(IAR_CDEV_MAGIC,0x15, update_cmd_t)
#define IAR_GAMMA_CFG		_IOW(IAR_CDEV_MAGIC,0x17, gamma_cfg_t)
#define IAR_SCALE_CFG		_IOW(IAR_CDEV_MAGIC,0x18, upscaling_cfg_t)
#define IAR_OUTPUT_CFG		_IOW(IAR_CDEV_MAGIC,0x19, output_cfg_t)
#define IAR_BACKLIGHT_CFG	_IOW(IAR_CDEV_MAGIC, 0x20, unsigned int)
#define IAR_SET_VIDEO_CHANNEL	_IOW(IAR_CDEV_MAGIC, 0x21, unsigned int)
#define IAR_SET_VIDEO_DDR_LAYER _IOW(IAR_CDEV_MAGIC, 0x22, unsigned int)
#define DISP_SET_VIDEO_ADDR	\
	_IOW(IAR_CDEV_MAGIC, 0x23, struct display_video_vaddr)
#define GET_DISP_DONE     _IOR(IAR_CDEV_MAGIC, 0x24, unsigned int)
#define DISP_SET_TIMING	_IOW(IAR_CDEV_MAGIC, 0x25, struct disp_timing)

#define IAR_WB_QBUF  	_IOW(IAR_CDEV_MAGIC, 0x26, unsigned int)
#define IAR_WB_DQBUF 	_IOW(IAR_CDEV_MAGIC, 0x27, unsigned int)
#define IAR_WB_REQBUFS 	_IOW(IAR_CDEV_MAGIC, 0x28, unsigned int)
#define IAR_WB_STREAM 	_IOW(IAR_CDEV_MAGIC, 0x29, unsigned int)
#define IAR_WB_GET_CFG 	_IOW(IAR_CDEV_MAGIC, 0x2a, unsigned int)
#define IAR_WB_SET_CFG 	_IOW(IAR_CDEV_MAGIC, 0x2b, unsigned int)

#define IAR_OUTPUT_LAYER0_QBUF _IOW(IAR_CDEV_MAGIC, 0x30, unsigned int)
#define IAR_OUTPUT_LAYER0_DQBUF _IOW(IAR_CDEV_MAGIC, 0x31, unsigned int)
#define IAR_OUTPUT_LAYER0_REQBUFS _IOW(IAR_CDEV_MAGIC, 0x32, unsigned int)
#define IAR_OUTPUT_LAYER1_QBUF _IOW(IAR_CDEV_MAGIC, 0x33, unsigned int)
#define IAR_OUTPUT_LAYER1_DQBUF _IOW(IAR_CDEV_MAGIC, 0x34, unsigned int)
#define IAR_OUTPUT_LAYER1_REQBUFS _IOW(IAR_CDEV_MAGIC, 0x35, unsigned int)
#define IAR_OUTPUT_LAYER0_STREAM _IOW(IAR_CDEV_MAGIC, 0x36, unsigned int)
#define DISP_SET_VIO_CHN_PIPE \
        _IOW(IAR_CDEV_MAGIC, 0x37, struct display_vio_channel_pipe)
#define IAR_OUTPUT_LAYER1_STREAM _IOW(IAR_CDEV_MAGIC, 0x38, unsigned int)
#define SCREEN_BACKLIGHT_SET       _IOW(IAR_CDEV_MAGIC, 0x39, unsigned int)
#define HDMI_CONFIG       _IO(IAR_CDEV_MAGIC, 0x40)
#define IAR_GET_START_CNT       _IOR(IAR_CDEV_MAGIC, 0x42, unsigned int)
#define IAR_GET_STOP_CNT       _IOR(IAR_CDEV_MAGIC, 0x43, unsigned int)
#define HDMI_SHUT_DOWN       _IO(IAR_CDEV_MAGIC, 0x44)
#define DISP_SET_VIDEO_PAUSE   _IOW(IAR_CDEV_MAGIC, 0x80, int)
#define DISP_SET_INTERLACE_MODE   _IO(IAR_CDEV_MAGIC, 0x45)
#define DISP_SET_PIXEL_CLK   _IOW(IAR_CDEV_MAGIC, 0x46, unsigned int)
#define DISP_SET_POSITION _IOW(IAR_CDEV_MAGIC, 0x47, struct position_cfg_t)
#define DISP_SET_CROP _IOW(IAR_CDEV_MAGIC, 0x48, struct position_cfg_t)
#define DISP_UPDATE_PAR       _IO(IAR_CDEV_MAGIC, 0x49)
#define DISP_SET_MIPI_DSI_LCP _IOW(IAR_CDEV_MAGIC, 0x50, struct mipi_dsi_lcp)
#define DISP_SET_MIPI_DSI_S1P _IOW(IAR_CDEV_MAGIC, 0x51, struct mipi_dsi_s1p)
#define DISP_SET_MIPI_DSI_SNP _IOW(IAR_CDEV_MAGIC, 0x52, uint8_t)
#define DISP_MIPI_DSI_CORE_INIT \
	_IOW(IAR_CDEV_MAGIC, 0x53, struct mipi_dsi_core_init_data)
#define DISP_MIPI_PANEL_INIT_BEGIN _IO(IAR_CDEV_MAGIC, 0x54)
#define DISP_MIPI_PANEL_INIT_END _IO(IAR_CDEV_MAGIC, 0x55)

unsigned int iar_open_cnt = 0;
unsigned int iar_start_cnt = 0;
extern bool iar_video_not_pause;
typedef struct _update_cmd_t {
	unsigned int enable_flag[IAR_CHANNEL_MAX];
	unsigned int frame_size[IAR_CHANNEL_MAX];
	frame_buf_t srcframe[IAR_CHANNEL_MAX];
} update_cmd_t;

struct iar_cdev_s {
	const char	*name;
	int major;
	int minor;
	struct cdev cdev;
	struct device *dev;
	dev_t dev_num;
	struct class *iar_classes;
	struct completion completion;
	frame_buf_t *framebuf_user[IAR_CHANNEL_MAX];
	struct mutex iar_mutex;
};
struct iar_cdev_s *g_iar_cdev;
hdmi_set_config_callback config_hdmi;
void hdmi_register_config_callback(hdmi_set_config_callback func)
{
       config_hdmi = func;
}
EXPORT_SYMBOL(hdmi_register_config_callback);

hdmi_set_stop_output_callback shut_down_hdmi;
void hdmi_register_stop_output_callback(hdmi_set_stop_output_callback func)
{
	shut_down_hdmi = func;
}
EXPORT_SYMBOL(hdmi_register_stop_output_callback);

int32_t iar_write_framebuf_poll(uint32_t channel, void __user *srcaddr, uint32_t size)
{
	frame_buf_t *bufaddr;
	bufaddr = iar_get_framebuf_addr(channel);
	if (copy_from_user(bufaddr->vaddr, srcaddr, size))
		return -EFAULT;

	IAR_DEBUG_PRINT("iar_write_framebuf_poll :%d vaddr:0x%p paddr:0x%llx,size:%d\n", channel, bufaddr->vaddr, bufaddr->paddr, size);

	return iar_switch_buf(channel);

}

static void iar_edma_callback(void *data)
{
	complete(&g_iar_cdev->completion);
}

/**
 * Display one frame through DMA method
 * @channel: display layer of IAR module
 * @srcaddr: frame address to be displayed
 * @size: frame size to be displayed
 */
int32_t iar_write_framebuf_dma(uint32_t channel, phys_addr_t srcaddr, uint32_t size)
{
	struct dma_chan *ch;
	int ret = 0;
	struct dma_async_tx_descriptor *tx;
	dma_cap_mask_t mask;
	dma_cookie_t cookie;
	frame_buf_t *bufaddr;

	reinit_completion(&g_iar_cdev->completion);
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	ch = dma_request_channel(mask, NULL, NULL);
	if (!ch) {
		printk(KERN_ERR"%s: dma_request_channel failed\n", __func__);
		return -1;
	}
	bufaddr = iar_get_framebuf_addr(channel);

	tx = ch->device->device_prep_dma_memcpy(ch, bufaddr->paddr, srcaddr, size, 0);
	if (!tx) {
		printk(KERN_ERR"%s: device_prep_dma_memcpy failed\n", __func__);
		return -EIO;
	}

	tx->callback = iar_edma_callback;
	tx->callback_result = NULL;
	tx->callback_param = g_iar_cdev;
	cookie = dmaengine_submit(tx);
	ret = dma_submit_error(cookie);
	if (ret) {
		printk(KERN_ERR"dma_submit_error %d\n", cookie);
		return -EIO;
	}
	dma_async_issue_pending(ch);
	ret = (int)wait_for_completion_timeout(&g_iar_cdev->completion,
			msecs_to_jiffies(1000));
	dma_release_channel(ch);
	if (!ret) {
		printk(KERN_ERR"%s: timeout !!\n", __func__);
		return -EIO;
	}
//	iar_switch_buf(channel);
	IAR_DEBUG_PRINT("DMA trans done\n");

	return 0;
}
EXPORT_SYMBOL_GPL(iar_write_framebuf_dma);

int32_t iar_display_update(update_cmd_t *update_cmd)
{
	int index = 0;
	int ret = -1;

	if (g_iar_cdev == NULL) {
		pr_err("%s: iar cdev not init!\n", __func__);
		return -1;
	}
	if (g_iar_cdev->framebuf_user[IAR_CHANNEL_1] == NULL ||
			g_iar_cdev->framebuf_user[IAR_CHANNEL_3] == NULL)
		return -1;
	for (index = IAR_CHANNEL_1; index < IAR_CHANNEL_MAX; index++) {
		if (index == IAR_CHANNEL_2 || index == IAR_CHANNEL_4)
			continue; //TODO, now channnel 2 and 4 is disable
		IAR_DEBUG_PRINT("update_cmd->enable_flag[index]:%d addr:0x%p\n", update_cmd->enable_flag[index], update_cmd->srcframe[index].vaddr);
		if (update_cmd->enable_flag[index] && update_cmd->frame_size[index] < MAX_FRAME_BUF_SIZE) {
#ifdef IAR_DMA_MODE
			if (g_iar_cdev->framebuf_user[index]) {
				//__clean_dcache_area_poc(g_iar_cdev->framebuf_user[index]->vaddr,
				//update_cmd->frame_size[index]);
				ret = iar_write_framebuf_dma(index, g_iar_cdev->framebuf_user[index]->paddr, update_cmd->frame_size[index]);
			}
#else
			if (update_cmd->srcframe[index].vaddr)
				ret = iar_write_framebuf_poll(index, update_cmd->srcframe[index].vaddr, update_cmd->frame_size[index]);
#endif
		}
	}
	return ret;
}

static int iar_cdev_open(struct inode *inode, struct file *filp)
{
	struct iar_cdev_s *iarcdev_p;
	int ret = 0;

	if (g_iar_cdev == NULL) {
		pr_err("%s: iar cdev not init!\n", __func__);
		return -1;
	}
	mutex_lock(&g_iar_cdev->iar_mutex);
	if (iar_open_cnt == 0) {
		iarcdev_p = container_of(inode->i_cdev, struct iar_cdev_s, cdev);
		filp->private_data = iarcdev_p;
		ret = iar_open();
	}
	iar_open_cnt++;
	mutex_unlock(&g_iar_cdev->iar_mutex);

	return ret;
}

static long iar_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long p)
{
	int ret = 0;
	void	__user *arg = (void __user *)p;

	if (g_iar_cdev == NULL) {
		pr_err("%s: iar cdev not init!\n", __func__);
		return -1;
	}
	mutex_lock(&g_iar_cdev->iar_mutex);
	switch (cmd) {
	case IAR_START:
		{
			IAR_DEBUG_PRINT("IAR_START \n");
			ret = iar_start(1);
		}
		break;
	case IAR_GET_START_CNT:
		 {
			unsigned int cnt;

			cnt = iar_start_cnt;
			if (copy_to_user(arg, &cnt,
					sizeof(unsigned int))) {
				ret = -EFAULT;
				break;
			}
			iar_start_cnt++;
		}
		break;
	case IAR_GET_STOP_CNT:
		 {
			unsigned int cnt;

			iar_start_cnt--;
			cnt = iar_start_cnt;
			if (copy_to_user(arg, &cnt,
					sizeof(unsigned int)))
				ret = -EFAULT;
		}
		break;
	case HDMI_CONFIG:
		 {
			uint32_t vmode;

			IAR_DEBUG_PRINT("HEMI_CONFIG \n");
			if (copy_from_user(&vmode, arg, sizeof(uint32_t))) {
				ret = -EFAULT;
				break;
			}
			if (config_hdmi != NULL) {
				if (vmode == 0) {
					ret = config_hdmi(9, 4, 2);
				} else {
					pr_debug("iar_cdev: vmode = %d\n", vmode);
					ret = config_hdmi((short unsigned int)vmode, 0, 2);//CEA-VIC
				}
			} else {
				pr_err("no sii902x HDMI device!!\n");
				ret = -EFAULT;
			}
		}
		break;
	case HDMI_SHUT_DOWN:
		 {
			IAR_DEBUG_PRINT("HDMI SHUT DOWN\n");
			if (shut_down_hdmi != NULL) {
				shut_down_hdmi();
			} else {
				pr_err("no sii902x HDMI device!!\n");
				ret = -EFAULT;
			}
		}
		break;
	case IAR_STOP:
		{
			IAR_DEBUG_PRINT("IAR_STOP \n");
			ret = iar_stop();
		}
		break;
	case IAR_DISPLAY_UPDATE:

		{
			update_cmd_t update_cmd;
			IAR_DEBUG_PRINT("IAR_DISPLAY_UPDATE \n");
			if (copy_from_user(&update_cmd, arg, sizeof(update_cmd_t))) {
				ret = -EFAULT;
				break;
			}
			ret = iar_display_update(&update_cmd);
			if (!ret)
				iar_update();
		}
		break;
	case IAR_CHANNEL_CFG:
		{
			channel_base_cfg_t channel_cfg;
			IAR_DEBUG_PRINT("IAR_CHANNEL_CFG \n");
			if (copy_from_user(&channel_cfg, arg, sizeof(channel_base_cfg_t))) {
				ret = -EFAULT;
				break;
			}
			ret = iar_channel_base_cfg(&channel_cfg);
			if (!ret)
				iar_update();
		}
		break;
	case IAR_GAMMA_CFG:
		{
			gamma_cfg_t gamma_cfg;
			IAR_DEBUG_PRINT("IAR_GAMMA_CFG \n");
			if (copy_from_user(&gamma_cfg, arg, sizeof(gamma_cfg_t))) {
				ret = -EFAULT;
				break;
			}
			ret = iar_gamma_cfg(&gamma_cfg);
			if (!ret)
				iar_update();
		}
		break;
	case IAR_SCALE_CFG:
		{
			upscaling_cfg_t upscaling_cfg;
			IAR_DEBUG_PRINT("IAR_SCALE_CFG \n");
			if (copy_from_user(&upscaling_cfg, arg, sizeof(upscaling_cfg_t))) {
				ret = -EFAULT;
				break;
			}
			ret = iar_upscaling_cfg(&upscaling_cfg);
			if (!ret)
				iar_update();
		}
		break;
	case IAR_OUTPUT_CFG:
		{
			output_cfg_t output_cfg;
			IAR_DEBUG_PRINT("IAR_OUTPUT_CFG \n");
			if (copy_from_user(&output_cfg, arg, sizeof(output_cfg_t))) {
				ret = -EFAULT;
				break;
			}
			ret = iar_output_cfg(&output_cfg);
			disp_user_config_done = 1;
			if (!ret)
				iar_update();
		}
		break;
	case IAR_SET_VIDEO_CHANNEL:
		{
			unsigned int channel_number;

			IAR_DEBUG_PRINT("IAR_SET_VIDEO_CHANNEL");
			if (copy_from_user(&channel_number, arg,
						sizeof(unsigned int))) {
				ret = -EFAULT;
				break;
			}
#ifdef CONFIG_HOBOT_XJ2
			ret = set_video_display_channel(channel_number);
#else
			iar_display_cam_no = (uint32_t)channel_number;
#endif
		}
		break;
	case IAR_SET_VIDEO_DDR_LAYER:
		{
			unsigned int ddr_layer_number;

			IAR_DEBUG_PRINT("IAR_SET_VIDEO_DDR_LAYER_NUMBER");
			if (copy_from_user(&ddr_layer_number, arg,
						sizeof(unsigned int))) {
				ret = -EFAULT;
				break;
			}
#ifdef CONFIG_HOBOT_XJ2
				ret = set_video_display_ddr_layer(ddr_layer_number);
#else
				iar_display_addr_type = (uint32_t)ddr_layer_number;
#endif
		}
		break;
	case DISP_SET_VIDEO_ADDR:
		{
			struct display_video_vaddr disp_vaddr;

			pr_debug("iar_cdev: %s: disp set video0&1 addr\n",
					__func__);
			disp_copy_done = 0;
			if (copy_from_user(&disp_vaddr, arg,
						sizeof(disp_vaddr))) {
				ret = -EFAULT;
				break;
			}
			if (disp_vaddr.channel0_y_addr == NULL &&
					disp_vaddr.channel1_y_addr == NULL) {
				pr_err("iar_cdev : %s: layer0&1 display yaddr is NULL!\n", __func__);
				ret = -1;
				break;
			}
			ret = disp_set_ppbuf_addr(0,
			disp_vaddr.channel0_y_addr, disp_vaddr.channel0_c_addr);
			ret += disp_set_ppbuf_addr(1,
			disp_vaddr.channel1_y_addr, disp_vaddr.channel1_c_addr);
			iar_update();
		}
		break;
	case DISP_SET_VIO_CHN_PIPE:
		 {
			struct display_vio_channel_pipe disp_vio_cfg;
			if (copy_from_user(&disp_vio_cfg, arg,
						sizeof(disp_vio_cfg))) {
				ret = -EFAULT;
				break;
			}
			if (disp_vio_cfg.disp_layer_no == 0) {
				iar_display_cam_no = disp_vio_cfg.vio_pipeline;
				iar_display_addr_type = disp_vio_cfg.vio_channel;
			} else if (disp_vio_cfg.disp_layer_no == 1) {
				iar_display_cam_no_video1 = disp_vio_cfg.vio_pipeline;
				iar_display_addr_type_video1 = disp_vio_cfg.vio_channel;
			} else {
				pr_err("%s: error set display layer!!\n", __func__);
				ret = -1;
			}
		}
		break;
	case GET_DISP_DONE:
		 {
			uint8_t display_done;

			display_done = disp_copy_done;
			if (copy_to_user(arg, &display_done,
						sizeof(uint8_t)))
				ret = -EFAULT;
		}
		break;
	case DISP_SET_TIMING:
		 {
			struct disp_timing user_disp_timing;

			pr_debug("iar_cdev: %s: user set video timing\n",
					__func__);
			if (copy_from_user(&user_disp_timing, arg,
						sizeof(user_disp_timing))) {
				ret = -EFAULT;
				break;
			}
			ret = disp_set_panel_timing(&user_disp_timing);
			if (ret)
				pr_err("error user set video timing!\n");
			else
				iar_update();
		}
		break;
	case DISP_UPDATE_PAR:
		 {
			iar_update();
			ret = 0;
		}
		break;
	case DISP_SET_POSITION:
		 {
			struct position_cfg_t position;

			if (copy_from_user(&position, arg,
						sizeof(position))) {
				ret = -EFAULT;
				break;
			}
			ret = disp_set_display_position(&position);
			if (ret)
				pr_err("error user set display position!\n");
		}
		break;
	case DISP_SET_CROP:
		 {
			struct position_cfg_t crop;

			if (copy_from_user(&crop, arg,
						sizeof(crop))) {
				ret = -EFAULT;
				break;
			}
			ret = disp_set_display_crop(&crop);
			if (ret)
				pr_err("error user set crop!\n");
		}
		break;
	case DISP_SET_MIPI_DSI_LCP:
		 {
			struct mipi_dsi_lcp dsi_lcp;

			if (copy_from_user(&dsi_lcp, arg, sizeof(dsi_lcp))) {
				ret = -EFAULT;
				break;
			}
			ret = user_set_dsi_panel_long_cmd(dsi_lcp.cmd_len, dsi_lcp.cmd_data);
			if (ret)
				pr_err("iar_cdev: error set mipi dsi cmd!\n");
		}
		break;
	case DISP_SET_MIPI_DSI_S1P:
		 {
			struct mipi_dsi_s1p dsi_s1p;

			if (copy_from_user(&dsi_s1p, arg, sizeof(dsi_s1p))) {
				ret = -EFAULT;
				break;
			}
			ret = dsi_panel_write_cmd_poll(dsi_s1p.cmd, dsi_s1p.data, 0x15);
			if (ret)
				pr_err("iar_cdev: error set mipi dsi cmd!\n");
		}
		break;
	case DISP_SET_MIPI_DSI_SNP:
		 {
			uint8_t dsi_snp;

			if (copy_from_user(&dsi_snp, arg, sizeof(dsi_snp))) {
				ret = -EFAULT;
				break;
			}
			ret = dsi_panel_write_cmd_poll(dsi_snp, 0x00, 0x05);
			if (ret)
				pr_err("iar_cdev: error set mipi dsi cmd!\n");
		}
		break;
	case DISP_MIPI_DSI_CORE_INIT:
		 {
			struct mipi_dsi_core_init_data init_data;
			if (copy_from_user(&init_data, arg, sizeof(init_data))) {
				ret = -EFAULT;
				break;
			}
			ret = user_init_mipi_dsi_core(&init_data);
			if (ret)
				pr_err("iar_cdev: error init mipi dsi core!\n");
		}
		break;
	case DISP_MIPI_PANEL_INIT_BEGIN:
		 {
			mipi_dsi_panel_config_begin();
		}
		break;
	case DISP_MIPI_PANEL_INIT_END:
		 {
			mipi_dsi_set_mode(0);//video mode
		}
		break;
	case DISP_SET_PIXEL_CLK:
		 {
			unsigned int pixel_clock;

			pr_debug("iar_cdev: %s: user set video timing\n",
					__func__);
			if (copy_from_user(&pixel_clock, arg,
						sizeof(unsigned int))) {
				ret = -EFAULT;
				break;
			}
			ret = disp_set_pixel_clk(pixel_clock);
			if (ret)
				pr_err("error user set pixel clk!\n");
		}
		break;
	case DISP_SET_INTERLACE_MODE:
		 {
			IAR_DEBUG_PRINT("disp set interlace mode!\n");
			ret = disp_set_interlace_mode();
		}
		break;
	case SCREEN_BACKLIGHT_SET:
		 {
			unsigned int duty_level;

			pr_debug("%s: begin set screen backlight\n", __func__);
			if (copy_from_user(&duty_level, arg,
					sizeof(unsigned int))) {
				ret = -EFAULT;
				break;
			}
			pr_info("%s: level is %d!!\n", __func__, duty_level);
			ret = set_screen_backlight(duty_level);
		}
		break;
	case IAR_WB_SET_CFG:
		 {
			int value = 0;
			ret = get_user(value, (u32 __user *) arg);
			if (ret) {
				ret = -EFAULT;
				break;
			}
			iar_wb_setcfg(value);
		}
		break;
	case IAR_WB_GET_CFG:
		 {
			int value = iar_wb_getcfg();
			ret = (int)copy_to_user((void __user *) arg, (char *) &value,
				 sizeof(int));
			if (ret)
				ret = -EFAULT;
		}
		break;
	case IAR_WB_STREAM:
		 {
			int on = 0;

			pr_err("iar_wb_stream.\n");
			ret = get_user(on, (u32 __user *) arg);
			if (ret) {
				ret = -EFAULT;
				break;
			}
			if (on) {
				iar_wb_stream_on();
			} else {
				iar_wb_stream_off();
			}
		}
		break;
	case IAR_WB_REQBUFS:
		 {
			int buffers = 0;

			pr_err("iar_wb_reqbufs.\n");
			ret = get_user(buffers, (u32 __user *) arg);
			if (ret) {
				ret = -EFAULT;
				break;
			}
			iar_wb_reqbufs(buffers);
		}
		break;
	case IAR_WB_QBUF:
		 {
			struct frame_info frameinfo;

			ret = (int)copy_from_user((char *) &frameinfo, (u32 __user *) arg,
				   sizeof(struct frame_info));
			if (ret) {
				pr_err("IAR_WB_QBUF, copy failed\n");
				ret = -EFAULT;
				break;
			}
			iar_wb_qbuf(&frameinfo);
		}
		break;
	case IAR_WB_DQBUF:
		 {
			struct frame_info frameinfo;

			iar_wb_dqbuf(&frameinfo);
			ret = (int)copy_to_user((void __user *) arg, (char *) &frameinfo,
				 sizeof(struct frame_info));
			if (ret)
				ret = -EFAULT;
		}
		break;
    case IAR_OUTPUT_LAYER0_DQBUF:
		 {
      		struct frame_info frameinfo;

      		iar_output_dqbuf(0, &frameinfo);
      		ret = (int)copy_to_user((void __user *)arg, (char *)&frameinfo,
                         sizeof(struct frame_info));
      		if (ret)
			ret = -EFAULT;
    		}
		break;
    case IAR_OUTPUT_LAYER0_REQBUFS:
		 {
      		int buffers = 0;

      		pr_debug("iar_output_reqbufs.\n");
      		ret = get_user(buffers, (u32 __user *)arg);
      		if (ret) {
			ret = -EFAULT;
			break;
		}
      		iar_output_reqbufs(0, buffers);
    		}
		break;
    case IAR_OUTPUT_LAYER0_QBUF:
		 {
			struct frame_info frameinfo;

			ret = (int)copy_from_user((char *)&frameinfo, (u32 __user *)arg,
							sizeof(struct frame_info));
			if (ret) {
				pr_err("IAR_OUTPUT_QBUF, copy failed\n");
				ret = -EFAULT;
				break;
			}
			iar_output_qbuf(0, &frameinfo);
		}
		break;
    case IAR_OUTPUT_LAYER1_DQBUF:
		 {
			struct frame_info frameinfo;

			iar_output_dqbuf(1, &frameinfo);
			ret = (int)copy_to_user((void __user *)arg, (char *)&frameinfo,
								sizeof(struct frame_info));
			if (ret)
				ret = -EFAULT;
		}
		break;
    case IAR_OUTPUT_LAYER1_REQBUFS:
		 {
			int buffers = 0;

			pr_err("iar_output1_reqbufs.\n");
			ret = get_user(buffers, (u32 __user *)arg);
			if (ret) {
				ret = -EFAULT;
				break;
			}
			iar_output_reqbufs(1, buffers);
		}
		break;
    case IAR_OUTPUT_LAYER1_QBUF:
		 {
			struct frame_info frameinfo;

			ret = (int)copy_from_user((char *)&frameinfo, (u32 __user *)arg,
								sizeof(struct frame_info));
			if (ret) {
				pr_err("IAR_WB_QBUF, copy failed\n");
				ret = -EFAULT;
				break;
			}
			iar_output_qbuf(1, &frameinfo);
		}
		break;
    case IAR_OUTPUT_LAYER0_STREAM:
		 {
			int on = 0;

			pr_err("iar_output_stream.\n");
			ret = get_user(on, (u32 __user *)arg);
			if (ret) {
				ret = -EFAULT;
				break;
			}
			if (on) {
				iar_output_stream_on(0);
			} else {
				iar_output_stream_off(0);
			}
		}
		break;
	case IAR_OUTPUT_LAYER1_STREAM:
		 {
			int on = 0;

			pr_err("iar_output_stream.\n");
			ret = get_user(on, (u32 __user *)arg);
			if (ret) {
				ret = -EFAULT;
				break;
			}
			if (on) {
				iar_output_stream_on(1);
			} else {
				iar_output_stream_off(1);
			}
		}
		break;
	case DISP_SET_VIDEO_PAUSE:
		 {
			int pause = 0;

			ret = get_user(pause, (u32 __user *)arg);
			pr_debug("DISP_SET_VIDEO_PAUSE in, pause:%d\n", pause);
			if (ret) {
				ret = -EFAULT;
				break;
			}
			if (pause) {
				iar_save_cur_buf();
				iar_update();
			}
			iar_video_not_pause = !pause;
		}
		break;
	default:
		ret = -EPERM;
		break;
	}
	mutex_unlock(&g_iar_cdev->iar_mutex);
	return ret;
}

static ssize_t iar_cdev_write(struct file *filp, const char __user *ubuf,
							  size_t len, loff_t *ppos)
{
	return len;
}

static ssize_t iar_cdev_read(struct file *filp, char __user *ubuf,
							 size_t len, loff_t *offp)
{
	unsigned int size;
	size = 0;
	return size;
}

int iar_cdev_release(struct inode *inode, struct file *filp)
{
	if (g_iar_cdev == NULL) {
		pr_err("%s: iar cdev not init!\n", __func__);
		return -1;
	}
	mutex_lock(&g_iar_cdev->iar_mutex);
	iar_open_cnt--;
	if (iar_open_cnt == 0) {
		filp->private_data = NULL;
		iar_stop();
		iar_start_cnt = 0;
		iar_close();
	}
	mutex_unlock(&g_iar_cdev->iar_mutex);
	return 0;
}

static const struct file_operations iar_cdev_ops = {
	.owner		= THIS_MODULE,
	.open		= iar_cdev_open,
	.release	= iar_cdev_release,
	.write		= iar_cdev_write,
	.read		= iar_cdev_read,
	.unlocked_ioctl = iar_cdev_ioctl,
	.compat_ioctl = iar_cdev_ioctl
};

struct kobject *hobot_iar_kobj;
static ssize_t hobot_iar_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	if (enable_sif_mclk() != 0)
		goto err;
	if (iar_pixel_clk_enable() != 0)
		goto err;

	hobot_iar_dump();
	if (iar_pixel_clk_disable() != 0)
		goto err;
	if (disable_sif_mclk() != 0)
		goto err;
err:
	return (s - buf);
}
static ssize_t hobot_iar_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n)
{
	const char *tmp;
	int error = -EINVAL;
	int ret = 0;
	unsigned long tmp_value = 0, pipeline = 0,
		      disp_layer = 0, disp_vio_addr_type = 0;
	char value[3];
	long unsigned int board_id = 0;
	struct disp_timing timing = {0};

	tmp = (char *)buf;
	if (enable_sif_mclk() != 0) {
		ret = error;
		goto err1;
	}
	if (iar_pixel_clk_enable() != 0) {
		ret = error;
		goto err1;
	}
	board_id = simple_strtoul(base_board_name, NULL, 16);
	if (strncmp(tmp, "start", 5) == 0) {
		pr_info("iar start......\n");
		if (board_id == 0x1)
			screen_backlight_init();
		iar_start(1);
		if (display_type == HDMI_TYPE) {
			if (config_hdmi == NULL)
				goto err;
			else
				config_hdmi(9, 4, 2);
		}
	} else if (strncmp(tmp, "stop", 4) == 0) {
		pr_info("iar stop......\n");
		iar_stop();
		if (board_id == 0x1)
			screen_backlight_deinit();
	} else if (strncmp(tmp, "cam", 3) == 0) {
		tmp = tmp + 3;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret == 0) {
			pr_info("checkout camera %ld display!!\n", tmp_value);
			if (tmp_value > 1) {
				pr_info("wrong camera channel, exit!!\n");
				ret = error;
				goto err;
			}
			set_video_display_channel((unsigned char)tmp_value);
		} else {
			pr_info("error input, exit!!\n");
			ret = error;
			goto err;
		}
	} else if (strncmp(tmp, "hbp", 3) == 0) {
		tmp = tmp + 3;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret != 0) {
			pr_info("error input value, exit!!\n");
			ret = error;
			goto err;
		}
		iar_get_timing(&timing);
		timing.hbp = tmp_value;
		disp_set_panel_timing(&timing);
		pr_info("hbp=%d, hfp=%d, hs=%d, vbp=%d, vfp=%d, vs=%d\n",
				timing.hbp, timing.hfp, timing.hs,
				timing.vbp, timing.vfp, timing.vs);
	} else if (strncmp(tmp, "hfp", 3) == 0) {
		tmp = tmp + 3;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret != 0) {
			pr_info("error input value, exit!!\n");
			ret = error;
			goto err;
		}
		iar_get_timing(&timing);
		timing.hfp = tmp_value;
		disp_set_panel_timing(&timing);
		pr_info("hbp=%d, hfp=%d, hs=%d, vbp=%d, vfp=%d, vs=%d\n",
				timing.hbp, timing.hfp, timing.hs,
				timing.vbp, timing.vfp, timing.vs);
	} else if (strncmp(tmp, "hs", 2) == 0) {
		tmp = tmp + 2;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret != 0) {
			pr_info("error input value, exit!!\n");
			ret = error;
			goto err;
		}
		iar_get_timing(&timing);
		timing.hs = tmp_value;
		disp_set_panel_timing(&timing);
		pr_info("hbp=%d, hfp=%d, hs=%d, vbp=%d, vfp=%d, vs=%d\n",
				timing.hbp, timing.hfp, timing.hs,
				timing.vbp, timing.vfp, timing.vs);
	} else if (strncmp(tmp, "vbp", 3) == 0) {
		tmp = tmp + 3;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret != 0) {
			pr_info("error input value, exit!!\n");
			ret = error;
			goto err;
		}
		iar_get_timing(&timing);
		timing.vbp = tmp_value;
		disp_set_panel_timing(&timing);
		pr_info("hbp=%d, hfp=%d, hs=%d, vbp=%d, vfp=%d, vs=%d\n",
				timing.hbp, timing.hfp, timing.hs,
				timing.vbp, timing.vfp, timing.vs);
	} else if (strncmp(tmp, "vfp", 3) == 0) {
		tmp = tmp + 3;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret != 0) {
			pr_info("error input value, exit!!\n");
			ret = error;
			goto err;
		}
		iar_get_timing(&timing);
		timing.vfp = tmp_value;
		disp_set_panel_timing(&timing);
		pr_info("hbp=%d, hfp=%d, hs=%d, vbp=%d, vfp=%d, vs=%d\n",
				timing.hbp, timing.hfp, timing.hs,
				timing.vbp, timing.vfp, timing.vs);
	} else if (strncmp(tmp, "vs", 2) == 0) {
		tmp = tmp + 2;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret != 0) {
			pr_info("error input value, exit!!\n");
			ret = error;
			goto err;
		}
		iar_get_timing(&timing);
		timing.vs = tmp_value;
		disp_set_panel_timing(&timing);
		pr_info("hbp=%d, hfp=%d, hs=%d, vbp=%d, vfp=%d, vs=%d\n",
				timing.hbp, timing.hfp, timing.hs,
				timing.vbp, timing.vfp, timing.vs);
	} else if (strncmp(tmp, "clock", 5) == 0) {
		tmp = tmp + 5;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret != 0) {
			pr_info("error input value, exit!!\n");
			ret = error;
			goto err;
		}
		tmp_value = tmp_value * 100000;
		pr_info("%s: begin set pixel clock is %ld\n", __func__, tmp_value);
		disp_set_pixel_clk(tmp_value);
	} else if (strncmp(tmp, "mhsa", 4) == 0) {
		tmp = tmp + 4;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret != 0) {
			pr_info("error input value, exit!!\n");
			ret = error;
			goto err;
		}
		mipi_timing.vid_hsa = tmp_value;
		pr_info("mipi: vid_hsa=%d, vid_hbp=%d, vid_hline_time=%d, vid_vsa=%d, vid_vbp=%d, vid_vfp=%d\n",
			mipi_timing.vid_hsa,
			mipi_timing.vid_hbp,
			mipi_timing.vid_hline_time,
			mipi_timing.vid_vsa,
			mipi_timing.vid_vbp,
			mipi_timing.vid_vfp);
		mipi_dsi_video_config(&mipi_timing);
	} else if (strncmp(tmp, "mhbp", 4) == 0) {
		tmp = tmp + 4;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret != 0) {
			pr_info("error input value, exit!!\n");
			ret = error;
			goto err;
		}
		mipi_timing.vid_hbp = tmp_value;
		pr_info("mipi: vid_hsa=%d, vid_hbp=%d, vid_hline_time=%d, vid_vsa=%d, vid_vbp=%d, vid_vfp=%d\n",
			mipi_timing.vid_hsa,
			mipi_timing.vid_hbp,
			mipi_timing.vid_hline_time,
			mipi_timing.vid_vsa,
			mipi_timing.vid_vbp,
			mipi_timing.vid_vfp);
		mipi_dsi_video_config(&mipi_timing);
	} else if (strncmp(tmp, "mhline", 6) == 0) {
		tmp = tmp + 6;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret != 0) {
			pr_info("error input value, exit!!\n");
			ret = error;
			goto err;
		}
		mipi_timing.vid_hline_time = tmp_value;
		pr_info("mipi: vid_hsa=%d, vid_hbp=%d, vid_hline_time=%d, vid_vsa=%d, vid_vbp=%d, vid_vfp=%d\n",
			mipi_timing.vid_hsa,
			mipi_timing.vid_hbp,
			mipi_timing.vid_hline_time,
			mipi_timing.vid_vsa,
			mipi_timing.vid_vbp,
			mipi_timing.vid_vfp);
		mipi_dsi_video_config(&mipi_timing);
	} else if (strncmp(tmp, "mvsa", 4) == 0) {
		tmp = tmp + 4;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret != 0) {
			pr_info("error input value, exit!!\n");
			ret = error;
			goto err;
		}
		mipi_timing.vid_vsa = tmp_value;
                mipi_timing.vid_vactive_line = mipi_timing.vid_pkt_size +
			mipi_timing.vid_vsa + mipi_timing.vid_vbp + mipi_timing.vid_vfp;
		pr_info("mipi: vid_hsa=%d, vid_hbp=%d, vid_hline_time=%d, vid_vsa=%d, vid_vbp=%d, vid_vfp=%d\n",
			mipi_timing.vid_hsa,
			mipi_timing.vid_hbp,
			mipi_timing.vid_hline_time,
			mipi_timing.vid_vsa,
			mipi_timing.vid_vbp,
			mipi_timing.vid_vfp);
		mipi_dsi_video_config(&mipi_timing);
	} else if (strncmp(tmp, "mvbp", 4) == 0) {
		tmp = tmp + 4;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret != 0) {
			pr_info("error input value, exit!!\n");
			ret = error;
			goto err;
		}
		mipi_timing.vid_vbp = tmp_value;
		mipi_timing.vid_vactive_line = mipi_timing.vid_pkt_size +
			mipi_timing.vid_vsa + mipi_timing.vid_vbp + mipi_timing.vid_vfp;
		pr_info("mipi: vid_hsa=%d, vid_hbp=%d, vid_hline_time=%d, vid_vsa=%d, vid_vbp=%d, vid_vfp=%d\n",
			mipi_timing.vid_hsa,
			mipi_timing.vid_hbp,
			mipi_timing.vid_hline_time,
			mipi_timing.vid_vsa,
			mipi_timing.vid_vbp,
			mipi_timing.vid_vfp);
		mipi_dsi_video_config(&mipi_timing);
	} else if (strncmp(tmp, "mvfp", 4) == 0) {
		tmp = tmp + 4;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret != 0) {
			pr_info("error input value, exit!!\n");
			ret = error;
			goto err;
		}
		mipi_timing.vid_vfp = tmp_value;
		mipi_timing.vid_vactive_line = mipi_timing.vid_pkt_size +
			mipi_timing.vid_vsa + mipi_timing.vid_vbp + mipi_timing.vid_vfp;
		pr_info("mipi: vid_hsa=%d, vid_hbp=%d, vid_hline_time=%d, vid_vsa=%d, vid_vbp=%d, vid_vfp=%d\n",
			mipi_timing.vid_hsa,
			mipi_timing.vid_hbp,
			mipi_timing.vid_hline_time,
			mipi_timing.vid_vsa,
			mipi_timing.vid_vbp,
			mipi_timing.vid_vfp);
		mipi_dsi_video_config(&mipi_timing);
	} else if (strncmp(tmp, "pipe", 4) == 0) {
		tmp = tmp + 4;
		memcpy((void *)(value), (void *)(tmp), 1);
		value[1] = '\0';
		ret = kstrtoul(value, 0, &disp_layer);
		if (ret == 0) {
			if (disp_layer > 1) {
				pr_err("wrong video layer number, exit!!\n");
				ret = error;
				goto err;
			} else {
				pr_err("display layer is %ld!!\n", disp_layer);
			}
		} else {
			pr_err("error input type, exit!!\n");
			ret = error;
			goto err;
		}
		tmp = tmp + 1;
		memcpy((void *)(value), (void *)(tmp), 1);
                value[1] = '\0';
		ret = kstrtoul(value, 0, &pipeline);
		if (ret == 0) {
			if (pipeline > 7) {
				pr_err("wrong pipeline number, exit!!\n");
				ret = error;
				goto err;
			} else {
				pr_err("checkout pipeline %ld display!!\n", pipeline);
			}
		} else {
			pr_err("error input type, exit!!\n");
			ret = error;
			goto err;
		}
		tmp = tmp + 1;
		ret = kstrtoul(tmp, 0, &disp_vio_addr_type);
		if (ret == 0) {
			if (disp_vio_addr_type > 38) {
				pr_err("wrong vio address type, exit!!\n");
				ret = error;
				goto err;
			} else {
				pr_err("display vio address type is %ld!!\n", disp_vio_addr_type);
			}
		} else {
			pr_err("error input type, exit!!\n");
			ret = error;
			goto err;
		}
		if (disp_layer == 0) {
				iar_display_cam_no = (uint32_t)pipeline;
				iar_display_addr_type = (uint32_t)disp_vio_addr_type;
		} else if (disp_layer == 1) {
				iar_display_cam_no_video1 = (uint32_t)pipeline;
				iar_display_addr_type_video1 = (uint32_t)disp_vio_addr_type;
		}
	} else if (strncmp(tmp, "lcd", 3) == 0) {
		pr_info("iar output lcd rgb panel config......\n");
		display_type = LCD_7_TYPE;
		if (board_id == 0x1)
			screen_backlight_init();
		iar_start(1);
		user_config_display(display_type);
	} else if (strncmp(tmp, "mipi", 4) == 0) {
		pr_info("iar output lcd mipi 720p panel config......\n");
		display_type = MIPI_720P;
		disp_set_pixel_clk(69000000);
		user_config_display(display_type);
	} else if (strncmp(tmp, "dsi1080", 7) == 0) {
		pr_info("iar output lcd mipi 1080p panel config......\n");
		display_type = MIPI_1080P;
		if (board_id == 0x1)
			screen_backlight_init();
		iar_start(1);
		user_config_display(display_type);
		set_mipi_display(0);
	} else if (strncmp(tmp, "dsi720p", 7) == 0) {
		pr_info("iar output lcd mipi 720p touch panel config......\n");
		display_type = MIPI_720P_TOUCH;
		if (board_id == 0x1)
			screen_backlight_init();
		iar_start(1);
		user_config_display(display_type);
		set_mipi_display(1);
	} else if (strncmp(tmp, "dsi720x1280", 11) == 0) {
		pr_info("iar output lcd mipi 720p sdb touch panel config......\n");
		display_type = MIPI_720P_TOUCH;
		screen_backlight_init();
		iar_start(1);
		user_config_display(display_type);
		set_mipi_display(2);
	} else if (strncmp(tmp, "dsi1280x720", 11) == 0) {
		pr_info("iar output lcd mipi 1280x720 panel config......\n");
		display_type = MIPI_720P_H;
		screen_backlight_init();
		iar_start(1);
		user_config_display(display_type);
		set_mipi_display(3);
	} else if (strncmp(tmp, "hdmi", 4) == 0) {
		pr_info("iar output hdmi panel config......\n");
		display_type = HDMI_TYPE;
		user_config_display(display_type);
		iar_start(1);
		if (config_hdmi == NULL)
			goto err;
		else
			config_hdmi(9, 4, 2);
	} else if (strncmp(tmp, "shutdown", 8) == 0) {
		pr_info("user shut down hdmi output......\n");
		if (shut_down_hdmi == NULL)
			goto err;
		else
			shut_down_hdmi();
	} else if (strncmp(tmp, "reopen", 6) == 0) {
		pr_info("user start hdmi output......\n");
		if (config_hdmi == NULL)
			goto err;
		else
			config_hdmi(9, 4, 2);
	} else if (strncmp(tmp, "bt656", 4) == 0) {
		pr_info("iar output bt656 panel config......\n");
		display_type = BT656_TYPE;
		user_config_display(display_type);
		iar_start(1);
	} else if (strncmp(tmp, "ipi", 3) == 0) {
		pr_info("iar output ipi panel config......\n");
		display_type = SIF_IPI;
		user_config_display(display_type);
		iar_start(1);
	} else if (strncmp(buf, "enable", 6) == 0) {
		tmp = buf + 6;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret == 0) {
			pr_info("enable channel %ld\n", tmp_value);
			iar_layer_enable((int)tmp_value);
			iar_update();
		} else {
			pr_err("error input, exit!!\n");
			ret = error;
			goto err;
		}

	} else if (strncmp(buf, "disable", 7) == 0) {
		tmp = buf + 7;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret == 0) {
			pr_info("disable channel %ld\n", tmp_value);
			iar_layer_disable((int)tmp_value);
			iar_update();
		} else {
			pr_err("error input, exit!!\n");
			ret = error;
			goto err;
		}
	} else if (strncmp(buf, "backlight", 9) == 0) {
		tmp = buf + 9;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret == 0) {
			if (tmp_value < 11) {
				pr_info("set screen backlight level %ld\n", tmp_value);
				set_screen_backlight((unsigned int)tmp_value);
			} else {
				pr_info("error backlight level!!\n");
				goto err;
			}
		} else {
			pr_err("error input, exit!!\n");
			ret = error;
			goto err;
		}
	}
err:
	if (iar_pixel_clk_disable() != 0)
		ret = error;
	if (disable_sif_mclk() != 0)
		ret = error;
err1:
	return ret ? ret : n;
}

static struct kobj_attribute iar_test_attr = {
	.attr	= {
		.name = __stringify(iar_test_attr),
		.mode = 0644,
	},
	.show	= hobot_iar_show,
	.store	= hobot_iar_store,
};

static struct attribute *attributes[] = {
	&iar_test_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attributes,
};

int __init iar_cdev_init(void)
{
	int ret = 0;

	g_iar_cdev = kmalloc(sizeof(struct iar_cdev_s), GFP_KERNEL);
	if (!g_iar_cdev) {
		printk(KERN_ERR"Unable to alloc IAR DEV\n");
		return -ENOMEM;
	}
	g_iar_cdev->name = "iar_cdev";
	mutex_init(&g_iar_cdev->iar_mutex);
	init_completion(&g_iar_cdev->completion);

	g_iar_cdev->iar_classes = fb_class;

	ret = alloc_chrdev_region(&g_iar_cdev->dev_num, 0, 1, g_iar_cdev->name);
	if (!ret) {
		g_iar_cdev->major = MAJOR(g_iar_cdev->dev_num);
		g_iar_cdev->minor = MINOR(g_iar_cdev->dev_num);
	} else {
		pr_err("%s: error alloc chrdev region with return value %d!\n",
				__func__, ret);
		goto err;
	}

	cdev_init(&g_iar_cdev->cdev, &iar_cdev_ops);

	ret = cdev_add(&g_iar_cdev->cdev, g_iar_cdev->dev_num, 1);
	if (ret) {
		unregister_chrdev_region(g_iar_cdev->dev_num, 1);
		pr_err("%s: error add cdev with return value %d\n", __func__, ret);
		goto err1;
	}

	g_iar_cdev->dev = device_create(g_iar_cdev->iar_classes, NULL, g_iar_cdev->dev_num, NULL, g_iar_cdev->name);
	if (IS_ERR(g_iar_cdev->dev)) {
		pr_err("%s: error create device!\n", __func__);
		ret = -1;
		goto err2;
	}
	g_iar_cdev->framebuf_user[IAR_CHANNEL_1] = hobot_iar_get_framebuf_addr(IAR_CHANNEL_1);
	g_iar_cdev->framebuf_user[IAR_CHANNEL_3] = hobot_iar_get_framebuf_addr(IAR_CHANNEL_3);

	ret = sysfs_create_group(&g_iar_cdev->dev->kobj, &attr_group);
	if (ret) {
		pr_err("%s: error create sysfs group with return value %d!\n", __func__, ret);
		goto err3;
	}
	return 0;
err3:
	device_destroy(g_iar_cdev->iar_classes, g_iar_cdev->dev_num);
err2:
	cdev_del(&g_iar_cdev->cdev);
err1:
	unregister_chrdev_region(g_iar_cdev->dev_num, 1);
err:
	kfree(g_iar_cdev);
	g_iar_cdev = NULL;
	return ret;
}

void __exit iar_cdev_exit(void)
{
	sysfs_remove_group(&g_iar_cdev->dev->kobj, &attr_group);
	device_destroy(g_iar_cdev->iar_classes, g_iar_cdev->dev_num);
	cdev_del(&g_iar_cdev->cdev);
	unregister_chrdev_region(g_iar_cdev->dev_num, 1);
	if (g_iar_cdev != NULL) {
		kfree(g_iar_cdev);
		g_iar_cdev = NULL;
	}
}

late_initcall(iar_cdev_init);
module_exit(iar_cdev_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("Platform: Hobot SoC");

