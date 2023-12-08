/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/memblock.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <video/of_display_timing.h>
#include <asm/cacheflush.h>
#include <linux/linux_logo.h>
//#include <soc/hobot/hobot_iar.h>
#include "../../misc/bt2hdmi-lt/hobot_lt8618sxb_config.h"
#include "../../misc/bt2hdmi-lt/hobot_lt8618sxb.h"
#include "hobot_fb.h"
#include "edid.h"

#define IAR_MAX_HEIGHT 1080
#define IAR_MAX_WIDTH 1920
#define DRIVER_NAME "hobot-fb"
#define IAR_DMA_MODE
#define CONFIG_LOGO_FROM_KERNEL

#define FORMAT_ORGANIZATION_VAL 0x9c36
#define REFRESH_CFG_VAL 0x808
#define PSEUDO_PALETTE_SIZE 16

#define HBFB_DEBUG_PRINT(format, args...)    \
	pr_err("IAR debug: " format, ## args)

static const u32 cfb_tab8_be[] = {
	0x00000000, 0x000000ff, 0x0000ff00, 0x0000ffff,
	0x00ff0000, 0x00ff00ff, 0x00ffff00, 0x00ffffff,
	0xff000000, 0xff0000ff, 0xff00ff00, 0xff00ffff,
	0xffff0000, 0xffff00ff, 0xffffff00, 0xffffffff
};

static const u32 cfb_tab8_le[] = {
	0x00000000, 0xff000000, 0x00ff0000, 0xffff0000,
	0x0000ff00, 0xff00ff00, 0x00ffff00, 0xffffff00,
	0x000000ff, 0xff0000ff, 0x00ff00ff, 0xffff00ff,
	0x0000ffff, 0xff00ffff, 0x00ffffff, 0xffffffff
};

static const u32 cfb_tab16_be[] = {
	0x00000000, 0x0000ffff, 0xffff0000, 0xffffffff
};

static const u32 cfb_tab16_le[] = {
	0x00000000, 0xffff0000, 0x0000ffff, 0xffffffff
};

static const u32 cfb_tab32[] = {
	0x00000000, 0xffffffff
};

#define FB_WRITEL fb_writel
#define FB_READL  fb_readl

struct update_cmd_t {
	unsigned int enable_flag[IAR_CHANNEL_MAX];
	unsigned int frame_size[IAR_CHANNEL_MAX];
	frame_buf_t srcframe[IAR_CHANNEL_MAX];
};

struct hbfb_info {
	struct fb_info fb;
	struct fb_info fb1;
	struct platform_device *pdev;
	int channel3_en;
	void __iomem *regaddr;
	void __iomem	*sysctrl;
	struct update_cmd_t update_cmd;
	int memory_mode;
	frame_buf_t framebuf[IAR_CHANNEL_MAX];
	channel_base_cfg_t channel_base_cfg[IAR_CHANNEL_MAX];
	gamma_cfg_t gamma_cfg;
	output_cfg_t output_cfg;
	upscaling_cfg_t scale_cfg;

};
struct hbfb_info *hobot_fbi;

enum {
	RGB888_500 = 0,
	RGB888_700 = 1,
	DSI_PANEL = 2,
};

enum {
	USER_CFG,
	CHANNEL_CFG,
	OVERLAY_CFG,
	SCALE_CFG,
	GAMMA_CFG,
	OUTPUT_CFG,
};
unsigned int frame_size = 1920*1080*4;
static int lcd_type = RGB888_700;
//static int outmode = OUTPUT_RGB888;
static int outmode = OUTPUT_BT1120;
static u32 hbfb_pseudo_palette[16];

//extern int set_lt9211_config(struct fb_info *fb, unsigned int convert_type);
//extern int32_t iar_write_framebuf_dma(uint32_t channel,
//		phys_addr_t srcaddr, uint32_t size);

static int hbfb_set_par(struct fb_info *fb);
//static void hbfb_activate_par(void);
//static int iar_get_framesize(void);
static int hbfb_setcolreg(unsigned int regno, unsigned int red,
		unsigned int green, unsigned int blue, unsigned int transp,
		struct fb_info *info);
static int hbfb_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *info);
static int hbfb_blank(int blank, struct fb_info *info);
static u_long get_line_length(int xres_virtual, int bpp);
static int hbfb_mmap(struct fb_info *info, struct vm_area_struct *pvma);
static int hbfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info);
static void hbfb_imageblit(struct fb_info *p, const struct fb_image *image);
static inline void hobot_color_imageblit(const struct fb_image *image,
		struct fb_info *p, u8 __iomem *dst1,
		u32 start_index, u32 pitch_index);
static inline void hobot_slow_imageblit(const struct fb_image *image,
		struct fb_info *p, u8 __iomem *dst1, u32 fgcolor,
		u32 bgcolor, u32 start_index, u32 pitch_index);
static inline void hobot_fast_imageblit(const struct fb_image *image,
		struct fb_info *p, u8 __iomem *dst1, u32 fgcolor,
		u32 bgcolor);

static int flag;
static int start_flag = 0;
//static uint32_t logo_addr;
u8 dummy_edid[] = {
0x00,0xff,0xff,0xf,0xff,0xff,0xff,0x00,0x4a,0x8b,0x33,0xbc,0x01,0x01,0x01,0x01,
0x09,0x1a,0x01,0x3,0x80,0x3c,0x22,0x78,0x3e,0xee,0x91,0xa3,0x54,0x4c,0x99,0x26,
0x0f,0x50,0x54,0xf,0xef,0x80,0xd1,0xc0,0xd1,0xe8,0xd1,0xfc,0x95,0x00,0x90,0x40,
0x81,0x80,0x81,0x0,0x81,0xc0,0x02,0x3a,0x80,0x18,0x71,0x38,0x2d,0x40,0x58,0x2c,
0x25,0x00,0x55,0x0,0x21,0x00,0x00,0x1e,0x00,0x00,0x00,0xfc,0x00,0x52,0x54,0x4b,
0x20,0x46,0x48,0x4,0x20,0x48,0x44,0x52,0x20,0x0a,0x00,0x00,0x00,0xff,0x00,0x64,
0x65,0x6d,0x6f,0x3,0x65,0x74,0x2d,0x31,0x0a,0x20,0x30,0x20,0x00,0x00,0x00,0xfd,
0x00,0x32,0x4b,0x4,0x54,0x13,0x01,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x01,0x5b,
0x02,0x03,0x2b,0x2,0x4f,0x04,0x04,0x10,0x13,0x13,0x1f,0x20,0x21,0x22,0x22,0x48,
0x49,0x4a,0x4b,0xc,0xe2,0x00,0xd5,0xe3,0x05,0x00,0x00,0x23,0x09,0x07,0x07,0x83,
0x01,0x00,0x00,0x7,0x03,0x0c,0x00,0x10,0x00,0x00,0x3c,0x02,0x3a,0x80,0x18,0x71,
0x38,0x2d,0x40,0x8,0x2c,0x25,0x00,0x55,0x50,0x21,0x00,0x00,0x1e,0x01,0x1d,0x00,
0x72,0x51,0xd0,0xe,0x20,0x6e,0x28,0x55,0x00,0x55,0x50,0x21,0x00,0x00,0x1e,0x00,
0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x17
};

static unsigned long get_line_length(int xres_virtual, int bpp)
{
	pr_err("%s: xr:%d,bpp:%d\n",__func__,xres_virtual,bpp);
	//return (unsigned long)((((xres_virtual*bpp)+31)&~31) >> 3);
	unsigned long length;
	length = xres_virtual * bpp;
	length = (length + 31) & ~31;
	length >>= 3;
	length = ((length + 7) / 8) * 8;
	return length;
}

static int hbfb_blank(int blank, struct fb_info *info)
{
	switch (blank) {
	case FB_BLANK_UNBLANK:
		// by lmg. set_lt9211_config(&hobot_fbi->fb, 0);
		break;
	default:
		//set timing_v_sync 0writel
		//TODO
		break;
	}

	return 0;
}

static int hbfb_mmap(struct fb_info *info, struct vm_area_struct *pvma)
{
	int ret = 0;

	pr_debug("hobot mmap begin!\n");
	flag = 0;

	if (info->fix.smem_start == 0 ||
			(pvma->vm_end - pvma->vm_start) > frame_size) {
		pr_err("%s: mmap size = %d info->fix.smem_start = %d exceed layer size!\n", __func__,pvma->vm_end - pvma->vm_start,info->fix.smem_start);
		return -ENOMEM;
	}

	pvma->vm_flags |= VM_IO;
	pvma->vm_flags |= VM_LOCKED;
	pvma->vm_page_prot = pgprot_writecombine(pvma->vm_page_prot);
	if (remap_pfn_range(pvma, pvma->vm_start,
			info->fix.smem_start >> PAGE_SHIFT,
			pvma->vm_end - pvma->vm_start, pvma->vm_page_prot)) {
		pr_err("hbfb mmap fail\n");
		return -EAGAIN;
	}
	pr_err("hobot mmap end!:%lx\n", info->fix.smem_start);
	return ret;
}

static int hbfb_setcolreg(unsigned int regno, unsigned int red,
		unsigned int green, unsigned int blue, unsigned int transp,
		struct fb_info *info)
{
	int ret = 0;
	u32 *pal = info->pseudo_palette;
	u32 cr = red >> (16 - info->var.red.length);
	u32 cg = green >> (16 - info->var.green.length);
	u32 cb = blue >> (16 - info->var.blue.length);
	u32 value;

	if (regno >= PSEUDO_PALETTE_SIZE)
		return -EINVAL;

	value = (cr << info->var.red.offset) |
		(cg << info->var.green.offset) |
		(cb << info->var.blue.offset);
	if (info->var.transp.length > 0) {
		u32 mask = (1 << info->var.transp.length) - 1;
		mask <<= info->var.transp.offset;
		value |= mask;
	}
	pal[regno] = value;
	return ret;
}

static int hbfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	int ret = 0;

	return ret;
}
//static long hbfb_ioctl(struct file *filp, unsigned int cmd, unsigned long p)
//{
//	int ret = 0;
//	unsigned int frame_size = 0;
//	int ret = 0;
//	frame_buf_t *framebuf = hobot_iar_get_framebuf_addr(2);
//	hobot_fbi->channel3_en = 1;
//	iar_get_framesize();
//	frame_size = hobot_fbi->update_cmd.frame_size[2];
//
//	pr_info("hbfb display begin!\n");
//
////	if (!framebuf || (pvma->vm_end - pvma->vm_start) > MAX_FRAME_BUF_SIZE)
////		return -ENOMEM;
////
////	pvma->vm_flags |= VM_IO;
////	pvma->vm_flags |= VM_LOCKED;
////	if (remap_pfn_range(pvma, pvma->vm_start, framebuf->paddr >> PAGE_SHIFT,
////			pvma->vm_end - pvma->vm_start, pvma->vm_page_prot)) {
////		pr_err("hbfb mmap fail\n");
////		return -EAGAIN;
////	}
////	pr_info("hbfb mmap end!:%llx\n", framebuf->paddr);
////	pr_info("channel3_en is 0x%x, fram_size is 0x%x.", hobot_fbi->channel3_en, frame_size);
//	if (hobot_fbi->channel3_en && frame_size <= MAX_FRAME_BUF_SIZE) {
////#ifdef IAR_DMA_MODE
//		printk(".......................................\n");
//		if (framebuf) {
//			pr_info("begin write framebuffer to pingpong buffer by dma.\n");
//			//clean_cache(framebuf->vaddr, frame_size);
//			__clean_dcache_area_poc(framebuf->vaddr, frame_size);
//			ret = iar_write_framebuf_dma(2,
//					framebuf->paddr, frame_size);
//			pr_info("write framebuffer to pinpong buffer by dma success!\n");
//		}
////#else
////		if (pvma->vm_start)
////			ret = iar_write_framebuf_poll(2,
////					pvma->vm_start, frame_size);
////#endif
//	}
////	return ret;
//	return ret;
//}
static int hbfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	u_long line_length;

	pr_info("%s begin.\n", __func__);

	return 0;

	if (!var->xres)
		var->xres = 1;
	if (!var->yres)
		var->yres = 1;
	if (var->xres > var->xres_virtual)
		var->xres_virtual = var->xres;
	if (var->yres > var->yres_virtual)
		var->yres_virtual = var->yres;
	if (var->bits_per_pixel <= 1)
		var->bits_per_pixel = 1;
	else if (var->bits_per_pixel <= 8)
		var->bits_per_pixel = 8;
	else if (var->bits_per_pixel <= 16)
		var->bits_per_pixel = 16;
	else if (var->bits_per_pixel <= 24)
		var->bits_per_pixel = 24;
	else if (var->bits_per_pixel <= 32)
		var->bits_per_pixel = 32;
	else
		return -EINVAL;

	if (var->xres_virtual < var->xoffset + var->xres)
		var->xres_virtual = var->xoffset + var->xres;
	if (var->yres_virtual < var->yoffset + var->yres)
		var->yres_virtual = var->yoffset + var->yres;
	if (var->xres > 1920)
		var->xres = 1920;
	if (var->yres > 1080)
		var->yres = 1080;

	/*
	 *  Memory limit
	 */
	line_length = get_line_length(var->xres_virtual, var->bits_per_pixel);
	if (line_length * var->yres_virtual > MAX_FRAME_BUF_SIZE)
		return -ENOMEM;

	/*
	 * Now that we checked it we alter var. The reason being is
	 * that the video
	 * mode passed in might not work but slight changes to it might make it
	 * work. This way we let the user know what is acceptable.
	 */
	switch (var->bits_per_pixel) {
	case 1:
		var->red.offset = 0;
		var->red.length = 1;
		var->green.offset = 0;
		var->green.length = 1;
		var->blue.offset = 0;
		var->blue.length = 1;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 8:
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 0;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 16:		/* RGBA 5551 */
		if (var->transp.length) {
			var->red.offset = 0;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 5;
			var->blue.offset = 10;
			var->blue.length = 5;
			var->transp.offset = 15;
			var->transp.length = 1;
		} else {	/* RGB 565 */
			var->red.offset = 0;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 6;
			var->blue.offset = 11;
			var->blue.length = 5;
			var->transp.offset = 0;
			var->transp.length = 0;
		}
		break;
	case 24:		/* RGB 888 */
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 16;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 32:		/* RGBA 8888 */
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 16;
		var->blue.length = 8;
		var->transp.offset = 24;
		var->transp.length = 8;
		break;
	}
	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->transp.msb_right = 0;

	//check LCD panel timing parameter
	if (outmode == OUTPUT_RGB && lcd_type == RGB888_500) {
		if (var->pixclock < 20000)
			var->pixclock = 20000;
		if (var->pixclock > 33000)
			var->pixclock = 33000;
		if (var->left_margin < 16)
			var->left_margin = 16;
		if (var->left_margin > 354)
			var->left_margin = 354;
		var->right_margin = 46;
		if (var->upper_margin < 7)
			var->upper_margin = 7;
		if (var->upper_margin > 147)
			var->upper_margin = 147;
		var->lower_margin = 23;
		if (var->hsync_len < 1)
			var->hsync_len = 1;
		if (var->hsync_len > 40)
			var->hsync_len = 40;
		if (var->vsync_len < 3)
			var->vsync_len = 3;
		if (var->vsync_len > 20)
			var->vsync_len = 20;

	} else if (outmode == OUTPUT_RGB && lcd_type == RGB888_700) {
		if (var->pixclock < 15873)
			var->pixclock = 15873;
		if (var->pixclock > 22272)
			var->pixclock = 22272;
		if (var->left_margin < 16)
			var->left_margin = 16;
		if (var->left_margin > 216)
			var->left_margin = 216;
		var->right_margin = 160;
		if (var->upper_margin < 1)
			var->upper_margin = 1;
		if (var->upper_margin > 127)
			var->upper_margin = 127;
		var->lower_margin = 23;
		if (var->hsync_len < 1)
			var->hsync_len = 1;
		if (var->hsync_len > 140)
			var->hsync_len = 140;
		if (var->vsync_len < 1)
			var->vsync_len = 1;
		if (var->vsync_len > 20)
			var->vsync_len = 20;
	} else if (outmode == OUTPUT_BT1120) {

	}
	pr_debug("%s: end.\n", __func__);
	return 0;
}

/*
static int iar_get_framesize(void)
{
	int i = 0;

	for (i = 0; i < IAR_CHANNEL_MAX; i++) {
		if (i == IAR_CHANNEL_1 || i == IAR_CHANNEL_2) {
			switch (hobot_fbi->channel_base_cfg[i].format) {
			case FORMAT_YUV420P_VU:
			case FORMAT_YUV420P_UV:
			case FORMAT_YUV420SP_UV:
			case FORMAT_YUV420SP_VU:
				//hobot_fbi->update_cmd.frame_size[i] =
				//(hobot_fbi->channel_base_cfg[i].buf_height*
				//hobot_fbi->channel_base_cfg[i].buf_width)*2;
				hobot_fbi->update_cmd.frame_size[i] =
				(hobot_fbi->channel_base_cfg[i].buf_height *
				 hobot_fbi->channel_base_cfg[i].buf_width) * 3 / 2;
				break;
			case FORMAT_YUV422_UYVY:
			case FORMAT_YUV422_VYUY:
			case FORMAT_YUV422_YVYU:
			case FORMAT_YUV422_YUYV:
			case FORMAT_YUV422SP_UV:
			case FORMAT_YUV422SP_VU:
			case FORMAT_YUV422P_UV:
			case FORMAT_YUV422P_VU:
				//hobot_fbi->update_cmd.frame_size[i] =
				//(hobot_fbi->channel_base_cfg[i].buf_height*
				//hobot_fbi->channel_base_cfg[i].buf_width)*3/2;
				hobot_fbi->update_cmd.frame_size[i] =
				(hobot_fbi->channel_base_cfg[i].buf_height *
				 hobot_fbi->channel_base_cfg[i].buf_width) * 2;
				break;
			default:
				HBFB_DEBUG_PRINT("not supported	%d format%d\n",
					i, hobot_fbi->channel_base_cfg[i].format);
				break;
			}
		} else {
			switch (hobot_fbi->channel_base_cfg[i].format) {
			case FORMAT_ARGB8888:
			case FORMAT_RGBA8888:
				hobot_fbi->update_cmd.frame_size[i] =
				(hobot_fbi->channel_base_cfg[i].buf_height *
				 hobot_fbi->channel_base_cfg[i].buf_width) * 4;
				break;
			case FORMAT_8BPP:
				hobot_fbi->update_cmd.frame_size[i] =
				(hobot_fbi->channel_base_cfg[i].buf_height *
				 hobot_fbi->channel_base_cfg[i].buf_width);
				break;
			case FORMAT_RGB565:
				hobot_fbi->update_cmd.frame_size[i] =
				(hobot_fbi->channel_base_cfg[i].buf_height *
				 hobot_fbi->channel_base_cfg[i].buf_width) * 2;
				break;
			case FORMAT_RGB888:
				pr_debug("IAR channel 2 config format as RGB888!\n");
				hobot_fbi->update_cmd.frame_size[i] =
				(hobot_fbi->channel_base_cfg[i].buf_height *
				 hobot_fbi->channel_base_cfg[i].buf_width) * 3;
				break;
			case FORMAT_RGB888P:
			default:
				HBFB_DEBUG_PRINT("not supported %d format %d\n",
					i, hobot_fbi->channel_base_cfg[i].format);
				break;
			}
		}
	}
	return 0;
}
*/
static channel_base_cfg_t channel_base_cfg[2] = {{0}, {0}};

int user_config(uint32_t src_w, uint32_t src_h,uint32_t dst_w, uint32_t dst_h)
{
	uint32_t value = 0;

	if (g_iar_dev == NULL) {
		pr_err("%s: iar not init, exit!!\n", __func__);
		return -1;
	}
	enable_sif_mclk();
	iar_pixel_clk_enable();

	channel_base_cfg[0].enable = 1;
	channel_base_cfg[1].enable = 0;
	channel_base_cfg[0].channel = IAR_CHANNEL_3;
	channel_base_cfg[0].pri = 0;
	channel_base_cfg[0].width = src_w;
	channel_base_cfg[0].height = src_h;
	channel_base_cfg[0].buf_width = src_w;
	channel_base_cfg[0].buf_height = src_h;
	channel_base_cfg[0].format = 3;
	channel_base_cfg[0].alpha_sel = 0;
	channel_base_cfg[0].ov_mode = 0;
	channel_base_cfg[0].alpha_en = 1;
	channel_base_cfg[0].alpha = 255;
	channel_base_cfg[0].crop_width = dst_w;
	channel_base_cfg[0].crop_height = dst_h;
	channel_base_cfg[1].channel = IAR_CHANNEL_4;
	channel_base_cfg[1].pri = 1;
	channel_base_cfg[1].width = src_w;
	channel_base_cfg[1].height = src_h;
	channel_base_cfg[1].buf_width = src_w;
	channel_base_cfg[1].buf_height = src_h;
	channel_base_cfg[1].format = 4;//ARGB8888
	channel_base_cfg[1].alpha_sel = 0;
	channel_base_cfg[1].ov_mode = 0;
	channel_base_cfg[1].alpha_en = 1;
	channel_base_cfg[1].alpha = 255;
	channel_base_cfg[1].crop_width = dst_w;
	channel_base_cfg[1].crop_height = dst_h;

	iar_channel_base_cfg(&channel_base_cfg[0]);
	iar_channel_base_cfg(&channel_base_cfg[1]);

	value = IAR_REG_SET_FILED(IAR_PANEL_WIDTH, dst_w, 0);
	value = IAR_REG_SET_FILED(IAR_PANEL_HEIGHT, dst_h, value);
	writel(value, g_iar_dev->regaddr + REG_IAR_PANEL_SIZE);

	iar_update();

	iar_pixel_clk_disable();
	disable_sif_mclk();
	return 0;
}

static int hbfb_set_par(struct fb_info *info)
{
	// dump_stack();
	int ret = 0;
	unsigned int layer_width = 0;
	unsigned int layer_height = 0;
	struct disp_timing iar_timing;
	hobot_hdmi_sync_t hdmi_timing;
	uint64_t iar_pixel_clk = 0;
	extern channel_base_cfg_t store_chn_cfg;

	user_config(1920,info->var.yres_virtual,info->var.xres,info->var.yres);
	if(ubuntu_desktop == 0){
		info->fix.line_length = get_line_length(info->var.xres_virtual,24);

	}else{
		info->fix.line_length = get_line_length(1920,24);
		memcpy(&store_chn_cfg,&channel_base_cfg[0],sizeof(channel_base_cfg_t));
	}
	pr_err("%s: xv:%d,yv:%d,xr:%d,yr:%d\n",__func__,info->var.xres_virtual,info->var.yres_virtual,info->var.xres,info->var.yres);
	iar_timing.hfp = info->var.right_margin;
	iar_timing.hbp = info->var.left_margin;
	iar_timing.vfp = info->var.lower_margin;
	iar_timing.vbp = info->var.upper_margin;
	iar_timing.hs = info->var.hsync_len;
	iar_timing.vs = info->var.vsync_len;
	iar_timing.vfp_cnt = 0X0;

	disp_set_panel_timing(&iar_timing);
	

	disp_set_pixel_clk(1000000000000 / info->var.pixclock);
	
	if (enable_sif_mclk() != 0)
		return -1;
	if (iar_pixel_clk_enable() != 0)
		return -1;

	iar_update();
	iar_stop();
	msleep(1);
	iar_start(1);
	msleep(5);
	disable_sif_mclk();
	iar_pixel_clk_disable();
	if(iar_get_output_mode() == OUTPUT_BT1120){
			hdmi_timing.hfp = iar_timing.hfp;
			hdmi_timing.hbp = iar_timing.hbp;

			hdmi_timing.vfp = iar_timing.vfp;
			hdmi_timing.vbp = iar_timing.vbp;

			hdmi_timing.hs = iar_timing.hs;
			hdmi_timing.vs = iar_timing.vs;

			hdmi_timing.hact = info->var.xres;
			hdmi_timing.vact = info->var.yres;
			
			hdmi_timing.htotal = hdmi_timing.hfp + hdmi_timing.hbp + hdmi_timing.hs + hdmi_timing.hact;
			hdmi_timing.vtotal = hdmi_timing.vfp + hdmi_timing.vbp + hdmi_timing.vs + hdmi_timing.vact;

			hdmi_timing.clk = 1000000000000 / info->var.pixclock;
			hdmi_timing.vic = 0;

			hdmi_timing.auto_detect = 0;
			hdmi_set_resolution(&hdmi_timing);
	}
	

	return 0;

}

/*
static void hbfb_activate_par(void)
{
	iar_update();
}
*/

static void hbfb_imageblit(struct fb_info *p, const struct fb_image *image)
{
#ifdef CONFIG_LOGO_FROM_KERNEL
	u32 fgcolor, bgcolor, start_index, bitstart, pitch_index = 0;
	u32 bpl = sizeof(u32), bpp = p->var.bits_per_pixel;
	u32 width = image->width;
	u32 dx = image->dx, dy = image->dy;
	u8 __iomem *dst1;

	if (p->state != FBINFO_STATE_RUNNING)
		return;

	bitstart = (dy * p->fix.line_length * 8) + (dx * bpp);
	start_index = bitstart & (32 - 1);
	pitch_index = (p->fix.line_length & (bpl - 1)) * 8;
	bitstart /= 8;
	bitstart &= ~(bpl - 1);
	dst1 = p->screen_base + bitstart;

	if (p->fbops->fb_sync)
		p->fbops->fb_sync(p);

	if (image->depth == 1) {
		if (p->fix.visual == FB_VISUAL_TRUECOLOR ||
			p->fix.visual == FB_VISUAL_DIRECTCOLOR) {
			fgcolor = ((u32 *)(p->pseudo_palette))[image->fg_color];
			bgcolor = ((u32 *)(p->pseudo_palette))[image->bg_color];
		} else {
			fgcolor = image->fg_color;
			bgcolor = image->bg_color;
		}

		if (32 % bpp == 0 && !start_index && !pitch_index &&
			((width & (32/bpp-1)) == 0) && bpp >= 8 && bpp <= 32)
			hobot_fast_imageblit(image, p, dst1, fgcolor, bgcolor);
		else
			hobot_slow_imageblit(image, p, dst1, fgcolor, bgcolor,
			start_index, pitch_index);
	} else {
		hobot_color_imageblit(image, p, dst1, start_index, pitch_index);
	}
#endif
}

static inline void hobot_color_imageblit(const struct fb_image *image,
				struct fb_info *p, u8 __iomem *dst1,
				u32 start_index, u32 pitch_index)
{
	/* Draw the penguin */
	u32 __iomem *dst, *dst2;
	u32 color = 0, val, shift;
	u32 color_r, color_g, color_b;
	int i, n;
	//int bpp = p->var.bits_per_pixel;
	//u32 null_bits = 32 - bpp;
	u32 *palette = (u32 *) p->pseudo_palette;
	const u8 *src = image->data;
	//u32 bswapmask = fb_compute_bswapmask(p);
	const unsigned char *logo_lut = logo_linux_clut224.clut;

	dst2 = (u32 __iomem *) dst1;
	for (i = image->height; i--; ) {
		n = image->width;
		dst = (u32 __iomem *) dst1;
		shift = 0;
		val = 0;

/*		if (start_index) {
 *			u32 start_mask = ~fb_shifted_pixels_mask_u32(p,
 *				start_index, bswapmask);
 *			val = FB_READL(dst) & start_mask;
 *			shift = start_index;
 *		}
 */
		while (n--) {
			if (p->fix.visual == FB_VISUAL_TRUECOLOR ||
					p->fix.visual == FB_VISUAL_DIRECTCOLOR)
				color = palette[*src];
			else
				color = *src;
			color = color - 0x20;
			//color_r = *((u32 *)(logo_lut + color*3));
			color_r = *(logo_lut + color*3);
			color_g = *(logo_lut + color*3 + 1);
			color_b = *(logo_lut + color*3 + 2);
			val = 0xff000000 |
				color_r << 16 | color_g << 8 | color_b;

			FB_WRITEL(val, dst++);
			src++;
		}
		dst1 += p->fix.line_length;
	}
}

static inline void hobot_slow_imageblit(const struct fb_image *image,
		struct fb_info *p, u8 __iomem *dst1, u32 fgcolor,
		u32 bgcolor, u32 start_index, u32 pitch_index)
{
	u32 shift, color = 0, bpp = p->var.bits_per_pixel;
	u32 __iomem *dst, *dst2;
	u32 val, pitch = p->fix.line_length;
	u32 null_bits = 32 - bpp;
	u32 spitch = (image->width+7)/8;
	const u8 *src = image->data, *s;
	u32 i, j, l;
	u32 bswapmask = fb_compute_bswapmask(p);

	dst2 = (u32 __iomem *) dst1;
	fgcolor <<= FB_LEFT_POS(p, bpp);
	bgcolor <<= FB_LEFT_POS(p, bpp);



	for (i = image->height; i--; ) {
		shift = val = 0;
		l = 8;
		j = image->width;
		dst = (u32 __iomem *) dst1;
		s = src;

		/* write leading bits */
		if (start_index) {
			u32 start_mask = ~fb_shifted_pixels_mask_u32(p,
					start_index, bswapmask);
			val = FB_READL(dst) & start_mask;
			shift = start_index;
		}

		while (j--) {
			l--;
			color = (*s & (1 << l)) ? fgcolor : bgcolor;
			val |= FB_SHIFT_HIGH(p, color, shift ^ bswapmask);

			/* Did the bitshift spill bits to the next long? */
			if (shift >= null_bits) {
				FB_WRITEL(val, dst++);
				val = (shift == null_bits) ? 0 :
					FB_SHIFT_LOW(p, color, 32 - shift);
			}
			shift += bpp;
			shift &= (32 - 1);
			if (!l) {
				l = 8;
				s++;
			}
		}

		/* write trailing bits */
		if (shift) {
			u32 end_mask = fb_shifted_pixels_mask_u32(p,
					shift, bswapmask);

			FB_WRITEL((FB_READL(dst) & end_mask) | val, dst);
		}
		dst1 += pitch;
		src += spitch;
		if (pitch_index) {
			dst2 += pitch;
			dst1 = (u8 __iomem *)
				((u64 __force)dst2 & ~(sizeof(u32) - 1));
			start_index += pitch_index;
			start_index &= 32 - 1;
		}
	}
}

static inline void hobot_fast_imageblit(const struct fb_image *image,
		struct fb_info *p, u8 __iomem *dst1, u32 fgcolor,
		u32 bgcolor)
{
	fgcolor = 0xFFFFFFFF;
	bgcolor = 0;
	u32 fgx = fgcolor, bgx = bgcolor, bpp = p->var.bits_per_pixel;
	u32 ppw = 32/bpp, spitch = (image->width + 7)/8;
	u32 bit_mask, end_mask, eorx, shift;
	const char *s = image->data, *src;
	u32 __iomem *dst;
	const u32 *tab = NULL;
	int i, j, k;

	switch (bpp) {
	case 8:
		tab = fb_be_math(p) ? cfb_tab8_be : cfb_tab8_le;
		break;
	case 16:
		tab = fb_be_math(p) ? cfb_tab16_be : cfb_tab16_le;
		break;
	case 32:
	default:
		tab = cfb_tab32;
		break;
	}

	for (i = ppw-1; i--; ) {
		fgx <<= bpp;
		bgx <<= bpp;
		fgx |= fgcolor;
		bgx |= bgcolor;
	}

	bit_mask = (1 << ppw) - 1;
	eorx = fgx ^ bgx;
	k = image->width/ppw;

	for (i = image->height; i--; ) {
		dst = (u32 __iomem *) dst1, shift = 8; src = s;

		for (j = k; j--; ) {
			shift -= ppw;
			end_mask = tab[(*src >> shift) & bit_mask];
			FB_WRITEL((end_mask & eorx)^bgx, dst++);
			if (!shift) {
				shift = 8;
				src++;
			}
		}
		dst1 += p->fix.line_length;
		s += spitch;
	}
}

void hbfb_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
#ifndef CONFIG_HOBOT_XJ3
	unsigned long pat, pat2, fg;
	unsigned long width = rect->width, height = rect->height;
	int bits = BITS_PER_LONG, bytes = bits >> 3;
	u32 bpp = p->var.bits_per_pixel;
	unsigned long __iomem *dst;
	int dst_idx, left;

	if (p->state != FBINFO_STATE_RUNNING)
		return;

	if (p->fix.visual == FB_VISUAL_TRUECOLOR ||
		p->fix.visual == FB_VISUAL_DIRECTCOLOR )
		fg = ((u32 *) (p->pseudo_palette))[rect->color];
	else
		fg = rect->color;

	pat = pixel_to_pat(bpp, fg);

	dst = (unsigned long __iomem *)((unsigned long)p->screen_base & ~(bytes-1));
	dst_idx = ((unsigned long)p->screen_base & (bytes - 1))*8;
	dst_idx += rect->dy*p->fix.line_length*8+rect->dx*bpp;
	/* FIXME For now we support 1-32 bpp only */
	left = bits % bpp;
	if (p->fbops->fb_sync)
		p->fbops->fb_sync(p);
	if (!left) {
		u32 bswapmask = fb_compute_bswapmask(p);
		void (*fill_op32)(struct fb_info *p,
				unsigned long __iomem *dst, int dst_idx,
				unsigned long pat, unsigned n, int bits,
				u32 bswapmask) = NULL;

		switch (rect->rop) {
		case ROP_XOR:
			fill_op32 = bitfill_aligned_rev;
			break;
		case ROP_COPY:
			fill_op32 = bitfill_aligned;
			break;
		default:
			pr_err("cfb_fillrect(): unknown rop, defaulting to ROP_COPY\n");
			fill_op32 = bitfill_aligned;
			break;
		}
		while (height--) {
			dst += dst_idx >> (ffs(bits) - 1);
			dst_idx &= (bits - 1);
			fill_op32(p, dst, dst_idx, pat, width*bpp, bits,
				bswapmask);
			dst_idx += p->fix.line_length*8;
		}
	} else {
		int right, r;
		void (*fill_op)(struct fb_info *p, unsigned long __iomem *dst,
				int dst_idx, unsigned long pat, int left,
				int right, unsigned n, int bits) = NULL;
#ifdef __LITTLE_ENDIAN
		right = left;
		left = bpp - right;
#else
		right = bpp - left;
#endif
		switch (rect->rop) {
		case ROP_XOR:
			fill_op = bitfill_unaligned_rev;
			break;
		case ROP_COPY:
			fill_op = bitfill_unaligned;
			break;
		default:
			printk(KERN_ERR "cfb_fillrect(): unknown rop, defaulting to ROP_COPY\n");
			fill_op = bitfill_unaligned;
			break;
		}
		while (height--) {
			dst += dst_idx / bits;
			dst_idx &= (bits - 1);
			r = dst_idx % bpp;
			/* rotate pattern to the correct start position */
			pat2 = le_long_to_cpu(rolx(cpu_to_le_long(pat), r, bpp));
			fill_op(p, dst, dst_idx, pat2, left, right,
				width*bpp, bits);
			dst_idx += p->fix.line_length*8;
		}
	}
#endif
}

static struct fb_ops hbfb_ops = {
	.fb_check_var	= hbfb_check_var,
	.fb_set_par	= hbfb_set_par,
	.fb_setcolreg	= hbfb_setcolreg,
	.fb_pan_display	= hbfb_pan_display,
	.fb_blank	= hbfb_blank,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
 	//.fb_imageblit	= cfb_imageblit,
	.fb_imageblit   = hbfb_imageblit,
	.fb_mmap	= hbfb_mmap,
};

struct fb_var_screeninfo fb_var_default = {
	.xres = 1920,
	.yres = 1080,
	.xres_virtual = 1920,
	.yres_virtual = 1080,
	.xoffset = 0,
	.yoffset = 0,
	.bits_per_pixel = 24,
	.grayscale = 0,
	.red = {
		.offset = 16,
		.length = 8,
		.msb_right = 0,//MSB left; !=0,MSB right
	},
	.green = {
		.offset = 8,
		.length = 8,
		.msb_right = 0,
	},
	.blue = {
		.offset = 0,
		.length = 8,
		.msb_right = 0,
	},
	.transp = {
#ifdef CONFIG_HOBOT_X3_UBUNTU
		.offset = 0,
		.length = 0,
#else
		.offset = 24,
		.length = 8,
#endif

		.msb_right = 0,
	},
	.nonstd = 0,
	.activate = FB_ACTIVATE_NOW,
	.height = 110,
	.width = 62,
	.accel_flags = FB_ACCEL_NONE,
//Referenced from uboot/drivers/video/videomodes.c:106
	.pixclock = 6734,//33.3M,
	.left_margin = 148,//100+20
	.right_margin = 88,//52+28
	.upper_margin = 36,//42+1
	.lower_margin = 4,//31+1
	.hsync_len = 44,//1~87,no type value
	.vsync_len = 5,//1~3,no type value

	.sync = 1 | 2,/* horizontal sync high active	*/
	.vmode = FB_VMODE_NONINTERLACED,
	.rotate = 1,
	.colorspace = 0,
	.reserved = {0x0},
};

struct fb_fix_screeninfo fb_fix_default = {
	.id = "x2-fb",
	.smem_start = 0x0,
	.smem_len = MAX_FRAME_BUF_SIZE,
	.type = FB_TYPE_PACKED_PIXELS,
	.type_aux = 0,
	.visual = FB_VISUAL_TRUECOLOR,  //FB_VISUAL_PSEUDOCOLOR,
	.xpanstep = 0,
	.ypanstep = 0,
	.ywrapstep = 0,
	.line_length = 5760,
	.mmio_start = 0,
	.mmio_len = 0,
	.accel = FB_ACCEL_NONE,
	.capabilities = 0,
	.reserved = {0x0},
};



static int hbfb_probe(struct platform_device *pdev)
{
	int ret;
	frame_buf_t framebuf_user;
	frame_buf_t framebuf_user1;
	uint32_t rgba[4] = {8, 16, 24, 0};
	int64_t iar_pixel_clk = -1;
	unsigned int layer_width = 0;
	unsigned int layer_height = 0;
	struct disp_timing iar_timing;
	u8 *edid = NULL;
	pr_info("Hobot fb probe!!!\n");

	hobot_fbi = devm_kzalloc(&pdev->dev, sizeof(struct hbfb_info), GFP_KERNEL);
	if (!hobot_fbi) {
		dev_err(&pdev->dev, "Unable to alloc hobot framebuffer DEV\n");
		return -ENOMEM;
	}

	strcpy(hobot_fbi->fb.fix.id, DRIVER_NAME);
	if (hobot_iar_get_framebuf_addr(2) == NULL ||
			hobot_iar_get_framebuf_addr(3) == NULL) {
		pr_err("%s: error get fb paddr!!\n", __func__);
		ret = -1;
		goto err0;
	}
	framebuf_user = *hobot_iar_get_framebuf_addr(2);
	framebuf_user1 = *hobot_iar_get_framebuf_addr(3);
	pr_debug("framebuf_user.paddr = 0x%llx\n", framebuf_user.paddr);
	pr_debug("framebuf_uset.vaddr = 0x%p\n", framebuf_user.vaddr);

	frame_size = IAR_MAX_HEIGHT * IAR_MAX_WIDTH * 4; //RGBA 
	
	pr_debug("frame_size is %d\n", frame_size);



	fb_fix_default.smem_start = framebuf_user.paddr;

	hobot_fbi->fb.var = fb_var_default;

	//We assume that the output mode at this time is bt1120 and try to get edid.
	if (1)
	{
		extern edid_raw_t edid_raw_data;
		struct fb_info* info = &hobot_fbi->fb;
		if(edid_raw_data.edid_data && edid_raw_data.block_num != -1){
			edid = edid_raw_data.edid_data;
		}else{
			edid = dummy_edid;
		}
		if (edid) {
			fb_edid_to_monspecs(edid, &info->monspecs);
			if (!info->monspecs.modedb)
				dev_err(info->device, "error getting mode database\n");
			else {
				const struct fb_videomode *m;

				fb_videomode_to_modelist(info->monspecs.modedb,
							info->monspecs.modedb_len,
							&info->modelist);
				m = fb_find_best_display(&info->monspecs, &info->modelist);
				if (m) {
					fb_videomode_to_var(&info->var, m);
					/* fill all other info->var's fields */
				}
			}
		}
	}
	
	if(ubuntu_desktop == 0){
		fb_fix_default.line_length = get_line_length(hobot_fbi->fb.var.xres_virtual,24);

	}else{
		fb_fix_default.line_length = get_line_length(1920,24);
	}

	hobot_fbi->fb.fix = fb_fix_default;

	hobot_fbi->fb.fbops = &hbfb_ops;
	hobot_fbi->fb.screen_base = framebuf_user.vaddr;
	//hobot_fbi->fb.screen_size = frame_size;
	hobot_fbi->fb.screen_size = MAX_FRAME_BUF_SIZE;
	hobot_fbi->fb.pseudo_palette = &hbfb_pseudo_palette;
	if (fb_alloc_cmap(&hobot_fbi->fb.cmap, 256, 0)) {
		pr_err("%s: fb0 error alloc cmap!!\n", __func__);
		ret = -ENOMEM;
		goto err0;
	}
	hobot_fbi->fb1 = hobot_fbi->fb;
	hobot_fbi->fb1.fix.smem_start = framebuf_user1.paddr;
	snprintf(hobot_fbi->fb1.fix.id, sizeof(hobot_fbi->fb1.fix.id),
			"hobot-fb1");
	hobot_fbi->fb1.screen_base = framebuf_user1.vaddr;
	if (fb_alloc_cmap(&hobot_fbi->fb1.cmap, 256, 0)) {
		pr_err("%s: fb1 error alloc cmap!!\n", __func__);
		ret = -ENOMEM;
		goto err1;
	}

	platform_set_drvdata(pdev, hobot_fbi);

	ret = register_framebuffer(&hobot_fbi->fb);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to register framebuffer device: %d\n", ret);
		if (hobot_fbi->fb.cmap.len)
			goto err1;
	}

	fb_info(&hobot_fbi->fb, "%s frame buffer at 0x%lx-0x%lx\n",
		hobot_fbi->fb.fix.id, hobot_fbi->fb.fix.smem_start,
		hobot_fbi->fb.fix.smem_start + hobot_fbi->fb.fix.smem_len - 1);

#ifdef CONFIG_HOBOT_XJ3
	if (fb_num == 2) {
		ret = register_framebuffer(&hobot_fbi->fb1);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Failed to register framebuffer device: %d\n", ret);
			if (hobot_fbi->fb1.cmap.len)
				goto err2;
		}
		fb_info(&hobot_fbi->fb1, "%s frame buffer at 0x%lx-0x%lx\n",
			hobot_fbi->fb1.fix.id, hobot_fbi->fb1.fix.smem_start,
			hobot_fbi->fb1.fix.smem_start + hobot_fbi->fb1.fix.smem_len - 1);
	}
#endif
	pr_err("Hobot fb probe ok!!!\n");
	return 0;
err2:
	fb_dealloc_cmap(&hobot_fbi->fb1.cmap);
	unregister_framebuffer(&hobot_fbi->fb);
err1:
	fb_dealloc_cmap(&hobot_fbi->fb.cmap);
err0:
	devm_kfree(&pdev->dev, hobot_fbi);
	hobot_fbi = NULL;
	return ret;
}

static int hbfb_remove(struct platform_device *pdev)
{
	struct hbfb_info *fbi = platform_get_drvdata(pdev);

	iar_stop();
	unregister_framebuffer(&fbi->fb);

	if (fbi->fb.cmap.len)
		fb_dealloc_cmap(&fbi->fb.cmap);
#ifdef CONFIG_HOBOT_XJ3
	if (fb_num == 2) {
		unregister_framebuffer(&fbi->fb1);
		if (fbi->fb1.cmap.len)
			fb_dealloc_cmap(&fbi->fb1.cmap);
	}
#endif
	return 0;
}

static const struct of_device_id hobot_dt_ids[] = {
	{ .compatible = "hobot,hobot-fb", },
	{}
};

static struct platform_driver hbfb_driver = {
	.probe		= hbfb_probe,
	.remove		= hbfb_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = hobot_dt_ids,
	},
};

int __init hbfb_init(void)
{
	int ret = platform_driver_register(&hbfb_driver);

	return ret;
}

static void __exit hbfb_cleanup(void)
{
	enable_sif_mclk();
	iar_pixel_clk_enable();
	platform_driver_unregister(&hbfb_driver);
	disable_sif_mclk();
	iar_pixel_clk_disable();
}

late_initcall_sync(hbfb_init);
module_exit(hbfb_cleanup);

//module_platform_driver(hbfb_driver);

MODULE_AUTHOR("rui.guo <rui.guo@horizon.ai>");
MODULE_DESCRIPTION("Framebuffer driver for Hobot SoC");
MODULE_LICENSE("GPL");
