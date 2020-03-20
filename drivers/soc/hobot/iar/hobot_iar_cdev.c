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
	ret = wait_for_completion_timeout(&g_iar_cdev->completion, msecs_to_jiffies(1000));
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
	for (index = IAR_CHANNEL_1; index < IAR_CHANNEL_MAX; index++) {
		if (index == IAR_CHANNEL_2 || index == IAR_CHANNEL_4)
			continue; //TODO, now channnel 2 and 4 is disable
		IAR_DEBUG_PRINT("update_cmd->enable_flag[index]:%d addr:0x%p\n", update_cmd->enable_flag[index], update_cmd->srcframe[index].vaddr);
		if (update_cmd->enable_flag[index] && update_cmd->frame_size[index] < MAX_FRAME_BUF_SIZE) {
#ifdef IAR_DMA_MODE
			if (g_iar_cdev->framebuf_user[index]) {
				__clean_dcache_area_poc(g_iar_cdev->framebuf_user[index]->vaddr, update_cmd->frame_size[index]);
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

	iarcdev_p = container_of(inode->i_cdev, struct iar_cdev_s, cdev);
	filp->private_data = iarcdev_p;
	return iar_open();
}

static long iar_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long p)
{
	int ret = 0;
	void	__user *arg = (void __user *)p;
	mutex_lock(&g_iar_cdev->iar_mutex);
	switch (cmd) {
	case IAR_START:
		{
			IAR_DEBUG_PRINT("IAR_START \n");
			ret = iar_start(1);
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
			if (copy_from_user(&update_cmd, arg, sizeof(update_cmd_t)))
				return -EFAULT;
			ret = iar_display_update(&update_cmd);
			if (!ret)
				iar_update();
		}
		break;
	case IAR_CHANNEL_CFG:
		{
			channel_base_cfg_t channel_cfg;
			IAR_DEBUG_PRINT("IAR_CHANNEL_CFG \n");
			if (copy_from_user(&channel_cfg, arg, sizeof(channel_base_cfg_t)))
				return -EFAULT;
			ret = iar_channel_base_cfg(&channel_cfg);
			if (!ret)
				iar_update();
		}
		break;
	case IAR_GAMMA_CFG:
		{
			gamma_cfg_t gamma_cfg;
			IAR_DEBUG_PRINT("IAR_GAMMA_CFG \n");
			if (copy_from_user(&gamma_cfg, arg, sizeof(gamma_cfg_t)))
				return -EFAULT;
			ret = iar_gamma_cfg(&gamma_cfg);
			if (!ret)
				iar_update();
		}
		break;
	case IAR_SCALE_CFG:
		{
			upscaling_cfg_t upscaling_cfg;
			IAR_DEBUG_PRINT("IAR_SCALE_CFG \n");
			if (copy_from_user(&upscaling_cfg, arg, sizeof(upscaling_cfg_t)))
				return -EFAULT;
			ret = iar_upscaling_cfg(&upscaling_cfg);
			if (!ret)
				iar_update();
		}
		break;
	case IAR_OUTPUT_CFG:
		{
			output_cfg_t output_cfg;
			IAR_DEBUG_PRINT("IAR_OUTPUT_CFG \n");
			if (copy_from_user(&output_cfg, arg, sizeof(output_cfg_t)))
				return -EFAULT;
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
						sizeof(unsigned int)))
				return -EFAULT;
			//pr_info("\niar_cdev_driver: camera channel
			//number is %d!!\n", channel_number);
#ifdef CONFIG_HOBOT_XJ2
			ret = set_video_display_channel(channel_number);
#else
			iar_display_cam_no = channel_number;
#endif
		}
		break;
	case IAR_SET_VIDEO_DDR_LAYER:
		{
			unsigned int ddr_layer_number;

			IAR_DEBUG_PRINT("IAR_SET_VIDEO_DDR_LAYER_NUMBER");
			if (copy_from_user(&ddr_layer_number, arg,
						sizeof(unsigned int)))
				return -EFAULT;
			//pr_info("\niar_cdev_driver: display ddr layer
			//number is %d!!\n", ddr_layer_number);
#ifdef CONFIG_HOBOT_XJ2
			ret = set_video_display_ddr_layer(ddr_layer_number);
#else
			iar_display_addr_type = ddr_layer_number;
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
						sizeof(disp_vaddr)))
				return -EFAULT;
			ret = disp_set_ppbuf_addr(0,
			disp_vaddr.channel0_y_addr, disp_vaddr.channel0_c_addr);
			if (ret)
				pr_err("%s: channel 0 not display\n", __func__);
			ret = disp_set_ppbuf_addr(1,
			disp_vaddr.channel1_y_addr, disp_vaddr.channel1_c_addr);
			if (ret)
				pr_err("%s: channel 1 not display\n", __func__);
			iar_update();
		}
		break;
	case DISP_SET_VIO_CHN_PIPE:
		 {
			struct display_vio_channel_pipe disp_vio_cfg;
			if (copy_from_user(&disp_vio_cfg, arg,
						sizeof(disp_vio_cfg)))
				return -EFAULT;
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

			//display_done = disp_get_display_done();
			//display_done = (uint8_t)((readl(g_iar_dev->regaddr +
			//	REG_IAR_DE_SRCPNDREG) & 0x00400000) >> 22);
			display_done = disp_copy_done;
			if (copy_to_user(arg, &display_done,
						sizeof(uint8_t)))
				return -EFAULT;
		}
		break;
	case DISP_SET_TIMING:
		 {
			struct disp_timing user_disp_timing;

			pr_debug("iar_cdev: %s: user set video timing\n",
					__func__);
			if (copy_from_user(&user_disp_timing, arg,
						sizeof(user_disp_timing)))
				return -EFAULT;
			ret = disp_set_panel_timing(&user_disp_timing);
			if (ret)
				pr_err("error user set video timing!\n");
			iar_update();
		}
		break;
	case IAR_WB_SET_CFG:
		 {
			int value = 0;
			ret = get_user(value, (u32 __user *) arg);
			if (ret)
				return -EFAULT;

			iar_wb_setcfg(value);
		}
		break;
	case IAR_WB_GET_CFG:
		 {
			int value = iar_wb_getcfg();
			ret = copy_to_user((void __user *) arg, (char *) &value,
				 sizeof(int));
			// pr_info("=====IAR_WB_DQBUF==ret %d===========\n", ret);
			if (ret)
				return -EFAULT;
		}
		break;
	case IAR_WB_STREAM:
		 {
			int on = 0;

			pr_err("iar_wb_stream.\n");
			ret = get_user(on, (u32 __user *) arg);
			if (ret)
				return -EFAULT;

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
			if (ret)
				return -EFAULT;
			iar_wb_reqbufs(buffers);
		}
		break;
	case IAR_WB_QBUF:
		 {
			struct frame_info frameinfo;

			// pr_err("iar_wb_qbuf.\n");
			ret = copy_from_user((char *) &frameinfo, (u32 __user *) arg,
				   sizeof(struct frame_info));
			// pr_info("=====IAR_WB_QBUF==ret %d===========\n", ret);
			if (ret) {
				pr_err("IAR_WB_QBUF, copy failed\n");
				return -EFAULT;
			}

			iar_wb_qbuf(&frameinfo);
		}
		break;
	case IAR_WB_DQBUF:
		 {
			struct frame_info frameinfo;

			// pr_err("iar_wb_qbuf.\n");
			iar_wb_dqbuf(&frameinfo);
			ret = copy_to_user((void __user *) arg, (char *) &frameinfo,
				 sizeof(struct frame_info));
			// pr_info("=====IAR_WB_DQBUF==ret %d===========\n", ret);
			if (ret)
				return -EFAULT;
		}
		break;
    case IAR_OUTPUT_LAYER0_DQBUF:
		 {
      		struct frame_info frameinfo;

      		// pr_err("iar_wb_qbuf.\n");
      		iar_output_dqbuf(0, &frameinfo);
      		ret = copy_to_user((void __user *)arg, (char *)&frameinfo,
                         sizeof(struct frame_info));
      		// pr_info("=====IAR_WB_DQBUF==ret %d===========\n", ret);
      		if (ret) return -EFAULT;
    	}
		break;
    case IAR_OUTPUT_LAYER0_REQBUFS:
		 {
      		int buffers = 0;

      		pr_err("iar_output_reqbufs.\n");
      		ret = get_user(buffers, (u32 __user *)arg);
      		if (ret)
			  	return -EFAULT;
      		iar_output_reqbufs(0, buffers);
    	}
		break;
    case IAR_OUTPUT_LAYER0_QBUF:
		 {
			struct frame_info frameinfo;

			// pr_err("iar_wb_qbuf.\n");
			ret = copy_from_user((char *)&frameinfo, (u32 __user *)arg,
							sizeof(struct frame_info));
			// pr_info("=====IAR_WB_QBUF==ret %d===========\n", ret);
			if (ret) {
				pr_err("IAR_OUTPUT_QBUF, copy failed\n");
				return -EFAULT;
			}

			iar_output_qbuf(0, &frameinfo);
		}
		break;
    case IAR_OUTPUT_LAYER1_DQBUF:
		 {
			struct frame_info frameinfo;

			// pr_err("iar_wb_qbuf.\n");
			iar_output_dqbuf(1, &frameinfo);
			ret = copy_to_user((void __user *)arg, (char *)&frameinfo,
								sizeof(struct frame_info));
			// pr_info("=====IAR_WB_DQBUF==ret %d===========\n", ret);
			if (ret)
				return -EFAULT;
		}
		break;
    case IAR_OUTPUT_LAYER1_REQBUFS:
		 {
			int buffers = 0;

			pr_err("iar_output1_reqbufs.\n");
			ret = get_user(buffers, (u32 __user *)arg);
			if (ret)
				return -EFAULT;
			iar_output_reqbufs(1, buffers);
		}
		break;
    case IAR_OUTPUT_LAYER1_QBUF:
		 {
			struct frame_info frameinfo;

			// pr_err("iar_wb_qbuf.\n");
			ret = copy_from_user((char *)&frameinfo, (u32 __user *)arg,
								sizeof(struct frame_info));
			// pr_info("=====IAR_WB_QBUF==ret %d===========\n", ret);
			if (ret) {
				pr_err("IAR_WB_QBUF, copy failed\n");
				return -EFAULT;
			}

			iar_output_qbuf(1, &frameinfo);
		}
		break;
    case IAR_OUTPUT_LAYER0_STREAM:
		 {
			int on = 0;

			pr_err("iar_output_stream.\n");
			ret = get_user(on, (u32 __user *)arg);
			if (ret) return -EFAULT;

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
			if (ret) return -EFAULT;

			if (on) {
				iar_output_stream_on(1);
			} else {
				iar_output_stream_off(1);
			}
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
	filp->private_data = NULL;
	return iar_close();
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

struct kobject *x2_iar_kobj;
static ssize_t x2_iar_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	x2_iar_dump();

	return (s - buf);
}
static ssize_t x2_iar_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n)
{
	char *tmp;
	int error = -EINVAL;
	int ret = 0;
	unsigned long tmp_value = 0, pipeline = 0, disp_layer = 0;
	char value[2];

	tmp = (char *)buf;
	if (strncmp(tmp, "start", 5) == 0) {
		pr_info("iar start......\n");
		iar_start(1);
	} else if (strncmp(tmp, "stop", 4) == 0) {
		pr_info("iar stop......\n");
		iar_stop();
	} else if (strncmp(tmp, "cam", 3) == 0) {
		tmp = tmp + 3;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret == 0) {
			pr_info("checkout camera %d display!!\n", tmp_value);
			if (tmp_value > 1) {
				pr_info("wrong camera channel, exit!!\n");
				return error;
			}
			set_video_display_channel(tmp_value);
		} else {
			pr_info("error input, exit!!\n");
			return error;
		}
	} else if (strncmp(tmp, "pipe", 4) == 0) {
		tmp = tmp + 4;
		memcpy((void *)(value), (void *)(tmp), 1);
		value[1] = '\0';
		ret = kstrtoul(value, 0, &pipeline);
		if (ret == 0) {
			if (pipeline > 3) {
				pr_info("wrong pipeline number, exit!!\n");
				return error;
			} else {
				pr_info("checkout pipeline %d display!!\n", pipeline);
			}
		} else {
			pr_info("error input type, exit!!\n");
			return error;
		}
		tmp = tmp + 1;
		memcpy((void *)(value), (void *)(tmp), 1);
                value[1] = '\0';
		ret = kstrtoul(value, 0, &disp_layer);
		if (ret == 0) {
			if (disp_layer > 1) {
				pr_info("wrong video layer number, exit!!\n");
				return error;
			} else {
				pr_info("display layer is %d!!\n", disp_layer);
			}
		} else {
			pr_info("error input type, exit!!\n");
			return error;
		}
		if (disp_layer == 0) {
				iar_display_cam_no = pipeline;
		} else if (disp_layer == 1) {
				iar_display_cam_no_video1 = pipeline;
		}
	} else if (strncmp(tmp, "lcd", 3) == 0) {
		pr_info("iar output lcd rgb panel config......\n");
		display_type = LCD_7_TYPE;
		iar_start(1);
		user_set_fb();
	} else if (strncmp(tmp, "mipi", 4) == 0) {
		pr_info("iar output lcd mipi 720p panel config......\n");
		display_type = MIPI_720P;
		disp_set_pixel_clk(69000000);
		user_set_fb();
	} else if (strncmp(tmp, "dsi1080", 7) == 0) {
		pr_info("iar output lcd mipi 1080p panel config......\n");
		display_type = MIPI_1080P;
		iar_start(1);
		user_set_fb();
		set_mipi_display(0);
		//msleep(2000);
		//mipi_dsi_panel_init(0);
	} else if (strncmp(tmp, "dsi720p", 7) == 0) {
		pr_info("iar output lcd mipi 720p touch panel config......\n");
		display_type = MIPI_720P_TOUCH;
		iar_start(1);
		user_set_fb();
		set_mipi_display(1);
		//msleep(2000);
		//mipi_dsi_panel_init(0);
	} else if (strncmp(buf, "enable", 6) == 0) {
		tmp = buf + 6;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret == 0) {
			pr_info("enable channel %d\n", tmp_value);
			iar_layer_enable(tmp_value);
			iar_start(1);
		} else {
			pr_info("error input, exit!!\n");
			return error;
		}

	} else if (strncmp(buf, "disable", 7) == 0) {
		tmp = buf + 7;
		ret = kstrtoul(tmp, 0, &tmp_value);
		if (ret == 0) {
			pr_info("disable channel %d\n", tmp_value);
			iar_layer_disable(tmp_value);
			iar_start(1);
		} else {
			pr_info("error input, exit!!\n");
			return error;
		}
	}

        //return error ? error : n;
        return n;
}

static struct kobj_attribute iar_test_attr = {
	.attr	= {
		.name = __stringify(iar_test_attr),
		.mode = 0644,
	},
	.show	= x2_iar_show,
	.store	= x2_iar_store,
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
	int error;
	struct device *dev;

	g_iar_cdev = kmalloc(sizeof(struct iar_cdev_s), GFP_KERNEL);
	if (!g_iar_cdev) {
		printk(KERN_ERR"Unable to alloc IAR DEV\n");
		return -ENOMEM;
	}
	g_iar_cdev->name = "iar_cdev";
	mutex_init(&g_iar_cdev->iar_mutex);
	init_completion(&g_iar_cdev->completion);

	g_iar_cdev->iar_classes = fb_class;

	error = alloc_chrdev_region(&g_iar_cdev->dev_num, 0, 1, g_iar_cdev->name);
	if (!error) {
		g_iar_cdev->major = MAJOR(g_iar_cdev->dev_num);
		g_iar_cdev->minor = MINOR(g_iar_cdev->dev_num);
	}

	cdev_init(&g_iar_cdev->cdev, &iar_cdev_ops);

	error = cdev_add(&g_iar_cdev->cdev, g_iar_cdev->dev_num, 1);
	if (error) {
		unregister_chrdev_region(g_iar_cdev->dev_num, 1);
		return error;
	}

	g_iar_cdev->dev = device_create(g_iar_cdev->iar_classes, NULL, g_iar_cdev->dev_num, NULL, g_iar_cdev->name);

	g_iar_cdev->framebuf_user[IAR_CHANNEL_1] = x2_iar_get_framebuf_addr(IAR_CHANNEL_1);
	g_iar_cdev->framebuf_user[IAR_CHANNEL_3] = x2_iar_get_framebuf_addr(IAR_CHANNEL_3);

	return sysfs_create_group(&g_iar_cdev->dev->kobj, &attr_group);
}

void __exit iar_cdev_exit(void)
{
	device_destroy(g_iar_cdev->iar_classes, g_iar_cdev->dev_num);
	cdev_del(&g_iar_cdev->cdev);
	unregister_chrdev_region(g_iar_cdev->dev_num, 1);

	sysfs_remove_group(&g_iar_cdev->dev->kobj, &attr_group);
}

module_init(iar_cdev_init);
module_exit(iar_cdev_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: x2");

