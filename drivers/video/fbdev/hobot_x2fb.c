/*
 * Copyright 2019-2022
 * Beijing Horizon Robotics Technology Research and Development Co.,Ltd.
 *
 * This code is for horizon robotics x2 Frame Buffer device driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License v2.0 as published by
 * the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
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

#include "hobot_x2fb.h"
#include "../../iar/x2_iar.h"

#define DRIVER_NAME "x2-fb"
#define IAR_DMA_MODE

#define X2FB_DEBUG_PRINT(format, args...)    \
	pr_info("IAR debug: " format, ## args)

struct update_cmd_t {
	unsigned int enable_flag[IAR_CHANNEL_MAX];
	unsigned int frame_size[IAR_CHANNEL_MAX];
	frame_buf_t srcframe[IAR_CHANNEL_MAX];
};

struct x2fb_info {
	struct fb_info fb;
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
struct x2fb_info *x2_fbi;

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
static int iar_config = CONFIG_CHANNEL3;
static int lcd_type = RGB888_700;
//static int outmode = OUTPUT_RGB888;
static int outmode = OUTPUT_BT1120;
static u32 x2fb_pseudo_palette[16];

//extern int set_lt9211_config(struct fb_info *fb, unsigned int convert_type);
//extern int32_t iar_write_framebuf_dma(uint32_t channel,
//		phys_addr_t srcaddr, uint32_t size);

static int x2fb_set_par(struct fb_info *fb);
static void x2fb_activate_par(void);
static int iar_get_framesize(void);
static int x2fb_setcolreg(unsigned int regno, unsigned int red,
		unsigned int green, unsigned int blue, unsigned int transp,
		struct fb_info *info);
static int x2fb_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *info);
static int x2fb_blank(int blank, struct fb_info *info);
static u_long get_line_length(int xres_virtual, int bpp);
static int x2fb_mmap(struct fb_info *info, struct vm_area_struct *pvma);
static int iar_paser_config(int type, int config_arrray[]);
static int x2fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info);
static int init_config(void);

static int flag;
//int display_type = LCD_7_TYPE;


int channelconfig[] = {
	0, //channel 0
	1, //enable
	3, //pri
	1920, //display width
	1080, //displayheight
	1920, //buf width
	1080, //buf height
	0, //xposition
	0, //yposition
	4, //format
	255, //alpha
	0, //keycolor
	0, //alpha_sel
	0, //ov_mode
	1, //alpha_en
	   //-----------------------
	1, //channel 1
	0, //enable
	2, //pri
	1920, //display width
	1080, //displayheight
	1920, //buf width
	1080, //buf height
	0, //xposition
	0, //yposition
	4, //format
	255, //alpha
	0, //keycolor
	0, //alpha_sel
	0, //ov_mode
	1, //alpha_en
	   //-----------------------
	2, //channel 2
	1, //enable
	1, //pri
	1920, //display width
	1080, //displayheight
	1920, //buf width
	1080, //buf height
	0, //xposition
	0, //yposition
	4, //format
	255, //alpha
	0, //keycolor
	0, //alpha_sel
	0, //ov_mode
	1, //alpha_en
	   //-----------------------
	3, //channel 3
	0, //enable
	0, //pri
	1920, //display width
	1080, //displayheight
	1920, //buf width
	1080, //buf height
	0, //xposition
	0, //yposition
	4, //format
	255, //alpha
	0, //keycolor
	0, //alpha_sel
	0, //ov_mode
	1, //alpha_en
};
int output_cfg[] = {
	0xff7f88, //bgcolor
	0, //output
	1920, //width
	1080, //height
	0, //dithering_flag
	0, //dithering_en
	0, //gamma_en
	0, //hue_en
	0, //sat_en
	0, //con_en
	0, //bright_en
	0, //theta_sign
	0, //contrast
	0xdf, //theta_abs
	0x6a, //saturation
	0, //off_contrast
	0x9a, //off_bright
	0, //dbi_refresh_mode
	2, //panel_corlor_type
	0, //interlace_sel
	0, //odd_polarity
	0, //pixel_rate
	0, //ycbcr_out
	0, //uv_sequence
	0, //itu_r656_en
	0, //auto_dbi_refresh_cnt
	0, //auto_dbi_refresh_en
};
int scale_config[] = {
	0, //scale_en
	1920, //src_width
	1080, //src_height
	1920, //tgt_width
	1080, //tgt_height
	0, //step_x
	0, //step_y
	0, //pos_x
	0, //pos_y
};
int gammma_config[] = {
	0, //gamma_x1_r
	0, //gamma_x2_r
	0, //gamma_x3_r
	0, //gamma_x4_r
	0, //gamma_x5_r
	0, //gamma_x6_r
	0, //gamma_x7_r
	0, //gamma_x8_r
	0, //gamma_x9_r
	0, //gamma_x10_r
	0, //gamma_x11_r
	0, //gamma_x12_r
	0, //gamma_x13_r
	0, //gamma_x14_r
	0, //gamma_x15_r
	0, //reserve
	0, //gamma_x1_g
	0, //gamma_x2_g
	0, //gamma_x3_g
	0, //gamma_x4_g
	0, //gamma_x5_g
	0, //gamma_x6_g
	0, //gamma_x7_g
	0, //gamma_x8_g
	0, //gamma_x9_g
	0, //gamma_x10_g
	0, //gamma_x11_g
	0, //gamma_x12_g
	0, //gamma_x13_g
	0, //gamma_x14_g
	0, //gamma_x15_g
	0, //reserve
	0, //gamma_x1_b
	0, //gamma_x2_b
	0, //gamma_x3_b
	0, //gamma_x4_b
	0, //gamma_x5_b
	0, //gamma_x6_b
	0, //gamma_x7_b
	0, //gamma_x8_b
	0, //gamma_x9_b
	0, //gamma_x10_b
	0, //gamma_x11_b
	0, //gamma_x12_b
	0, //gamma_x13_b
	0, //gamma_x14_b
	0, //gamma_x15_b
	0, //reserve
	0, //gamma_y1_start_r
	0, //gamma_y1_r
	0, //gamma_y2_r
	0, //gamma_y3_r
	0, //gamma_y4_r
	0, //gamma_y5_r
	0, //gamma_y6_r
	0, //gamma_y7_r
	0, //gamma_y8_r
	0, //gamma_y9_r
	0, //gamma_y10_r
	0, //gamma_y11_r
	0, //gamma_y12_r
	0, //gamma_y13_r
	0, //gamma_y14_r
	0, //gamma_y15_r
	0, //gamma_y1_start_g
	0, //gamma_y1_g
	0, //gamma_y2_g
	0, //gamma_y3_g
	0, //gamma_y4_g
	0, //gamma_y5_g
	0, //gamma_y6_g
	0, //gamma_y7_g
	0, //gamma_y8_g
	0, //gamma_y9_g
	0, //gamma_y10_g
	0, //gamma_y11_g
	0, //gamma_y12_g
	0, //gamma_y13_g
	0, //gamma_y14_g
	0, //gamma_y15_g
	0, //gamma_y1_start_b
	0, //gamma_y1_b
	0, //gamma_y2_b
	0, //gamma_y3_b
	0, //gamma_y4_b
	0, //gamma_y5_b
	0, //gamma_y6_b
	0, //gamma_y7_b
	0, //gamma_y8_b
	0, //gamma_y9_b
	0, //gamma_y10_b
	0, //gamma_y11_b
	0, //gamma_y12_b
	0, //gamma_y13_b
	0, //gamma_y14_b
	0, //gamma_y15_b
	0, //gamma_y16_r
	0, //gamma_y16_b
	0, //gamma_y16_g
};


struct fb_var_screeninfo RGB500_var_default = {
	.xres = 800,
	.yres = 480,
	.xres_virtual = 800,
	.yres_virtual = 480,
	.xoffset = 0,
	.yoffset = 0,
	.bits_per_pixel = 24,
	.grayscale = 0,
	.red = {
		.offset = 0,
		.length = 8,
		.msb_right = 0,//MSB left; !=0,MSB right
	},
	.green = {
		.offset = 8,
		.length = 8,
		.msb_right = 0,
	},
	.blue = {
		.offset = 16,
		.length = 8,
		.msb_right = 0,
	},
	.transp = {
		.offset = 0,
		.length = 0,
		.msb_right = 0,
	},
	.nonstd = 0,
	.activate = FB_ACTIVATE_NOW,
	.height = 64,
	.width = 108,
	.accel_flags = FB_ACCEL_NONE,
	.pixclock = 33000,//30MHz~50MHz
	.left_margin = 210,//16~354,horizon front porch(pixclocks)
	.right_margin = 46,//46~46,horizon back porch
	.upper_margin = 22,//7~147,vertical front porch
	.lower_margin = 23,//23~23,vertical back porch
	.hsync_len = 20,//1~40,
	.vsync_len = 11,//3~20,
	.sync = 0,//????????
	.vmode = FB_VMODE_NONINTERLACED,
	.rotate = 0,
	.colorspace = 0,
	.reserved = {0x0},

};

struct fb_fix_screeninfo RGB500_fix_default = {
	.id = "x2-fb",
	.smem_start = 0x0,
	.smem_len = MAX_FRAME_BUF_SIZE,
	.type = FB_TYPE_PACKED_PIXELS,
	.type_aux = 0,//#define FB_AUX_TEXT_MDA	0/* Monochrome text */
	.visual = FB_VISUAL_PSEUDOCOLOR,//??????????
	.xpanstep = 0,
	.ypanstep = 0,
	.ywrapstep = 0,
	.line_length = 2400,//?
	.mmio_start = 0,
	.mmio_len = 0,
	.accel = FB_ACCEL_NONE,
	.capabilities = 0,
	.reserved = {0x0},
};

/*
struct fb_var_screeninfo RGB700_var_default = {
	.xres = 800,
	.yres = 480,
	.xres_virtual = 800,
	.yres_virtual = 480,
	.xoffset = 0,
	.yoffset = 0,
	.bits_per_pixel = 24,
	.grayscale = 0,
	.red = {
		.offset = 0,
		.length = 8,
		.msb_right = 0,//MSB left; !=0,MSB right
	},
	.green = {
		.offset = 8,
		.length = 8,
		.msb_right = 0,
	},
	.blue = {
		.offset = 16,
		.length = 8,
		.msb_right = 0,
	},
	.transp = {
		.offset = 0,
		.length = 0,
		.msb_right = 0,
	},
	.nonstd = 0,
	.activate = FB_ACTIVATE_NOW,
	.height = 85,
	.width = 153,
	.accel_flags = FB_ACCEL_NONE,
	.pixclock = 19608,//29.2MHz,spec:20,33.3,50
	.left_margin = 160,//20~200
	.right_margin = 160,//87~1
	.upper_margin = 12,//5~200
	.lower_margin = 23,//31~29
	.hsync_len = 70,//1~87,no type value
	.vsync_len = 10,//1~3,no type value

//	.pixclock = 34247,//29.2MHz,spec:20,33.3,50
	.pixclock = 30030,//33.3M,
//	.left_margin = 40,//20~200
//	.right_margin = 40,//87~1
	.left_margin = 120,//100+20
	.right_margin = 80,//52+28
//	.upper_margin = 12,//5~200
//	.lower_margin = 30,//31~29
	.upper_margin = 43,//42+1
	.lower_margin = 32,//31+1
	.hsync_len = 48,//1~87,no type value
	.vsync_len = 2,//1~3,no type value

	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.rotate = 0,
	.colorspace = 0,
	.reserved = {0x0},

};

struct fb_fix_screeninfo RGB700_fix_default = {
	.id = "x2-fb",
	.smem_start = 0x0,
	.smem_len = MAX_FRAME_BUF_SIZE,
	.type = FB_TYPE_PACKED_PIXELS,
	.type_aux = 0,
	.visual = FB_VISUAL_PSEUDOCOLOR,
	.xpanstep = 0,
	.ypanstep = 0,
	.ywrapstep = 0,
	.line_length = 2400,
	.mmio_start = 0,
	.mmio_len = 0,
	.accel = FB_ACCEL_NONE,
	.capabilities = 0,
	.reserved = {0x0},
};
*/
struct fb_var_screeninfo RGB700_var_default = {
	.xres = 800,
	.yres = 480,
	.xres_virtual = 800,
	.yres_virtual = 480,
	.xoffset = 0,
	.yoffset = 0,
	.bits_per_pixel = 32,
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
		.offset = 24,
		.length = 8,
		.msb_right = 0,
	},
	.nonstd = 0,
	.activate = FB_ACTIVATE_NOW,
	.height = 85,
	.width = 153,
	.accel_flags = FB_ACCEL_NONE,

//	.pixclock = 34247,//29.2MHz,spec:20,33.3,50
	.pixclock = 30030,//33.3M,
//	.left_margin = 40,//20~200
//	.right_margin = 40,//87~1
	.left_margin = 120,//100+20
	.right_margin = 80,//52+28
//	.upper_margin = 12,//5~200
//	.lower_margin = 30,//31~29
	.upper_margin = 43,//42+1
	.lower_margin = 32,//31+1
	.hsync_len = 48,//1~87,no type value
	.vsync_len = 2,//1~3,no type value

	.sync = 0,//????????
	.vmode = FB_VMODE_NONINTERLACED,
	.rotate = 0,
	.colorspace = 0,
	.reserved = {0x0},

};

struct fb_fix_screeninfo RGB700_fix_default = {
	.id = "x2-fb",
	.smem_start = 0x0,
	.smem_len = MAX_FRAME_BUF_SIZE,
	.type = FB_TYPE_PACKED_PIXELS,
	.type_aux = 0,
	.visual = FB_VISUAL_PSEUDOCOLOR,
	.xpanstep = 0,
	.ypanstep = 0,
	.ywrapstep = 0,
	.line_length = 3200,
	.mmio_start = 0,
	.mmio_len = 0,
	.accel = FB_ACCEL_NONE,
	.capabilities = 0,
	.reserved = {0x0},
};


static u_long get_line_length(int xres_virtual, int bpp)
{
	u_long length;

	length = xres_virtual * bpp;
	length = (length + 31) & ~31;
	length >>= 3;
	return length;
}

static int x2fb_blank(int blank, struct fb_info *info)
{
	switch (blank) {
	case FB_BLANK_UNBLANK:
		// by lmg. set_lt9211_config(&x2_fbi->fb, 0);
		break;
	default:
		//set timing_v_sync 0writel
		//TODO
		break;
	}

	return 0;
}
//------------------------------------------------------------------------------
static int x2fb_mmap(struct fb_info *info, struct vm_area_struct *pvma)
{
	unsigned int frame_size = 0;
	int ret = 0;
	frame_buf_t *framebuf = x2_iar_get_framebuf_addr(2);
	x2_fbi->channel3_en = 1;
	iar_get_framesize();
	frame_size = x2_fbi->update_cmd.frame_size[2];

	pr_info("x2fb mmap begin!\n");
	flag = 0;
	if (!framebuf || (pvma->vm_end - pvma->vm_start) > MAX_FRAME_BUF_SIZE)
		return -ENOMEM;

	pvma->vm_flags |= VM_IO;
	pvma->vm_flags |= VM_LOCKED;
	//pvma->vm_page_prot = pgprot_noncached(pvma->vm_page_prot);
	if (remap_pfn_range(pvma, pvma->vm_start,
			(framebuf->paddr +  MAX_FRAME_BUF_SIZE*3) >> PAGE_SHIFT,
			pvma->vm_end - pvma->vm_start, pvma->vm_page_prot)) {
		pr_err("x2fb mmap fail\n");
		return -EAGAIN;
	}
	pr_info("x2fb mmap end!:%llx\n", framebuf->paddr);
	return ret;
}
/*
static int x2fb_mmap(struct fb_info *info, struct vm_area_struct *pvma)
{
	unsigned int frame_size = 0;
	int ret = 0;
	frame_buf_t *framebuf = x2_iar_get_framebuf_addr(2);
	x2_fbi->channel3_en = 1;
	iar_get_framesize();
	frame_size = x2_fbi->update_cmd.frame_size[2];
	if(flag == 0){
		pr_info("x2fb mmap begin!\n");
		flag = 1;
		if (!framebuf || (pvma->vm_end - pvma->vm_start) >
						MAX_FRAME_BUF_SIZE)
			return -ENOMEM;

		pvma->vm_flags |= VM_IO;
		pvma->vm_flags |= VM_LOCKED;
		if (remap_pfn_range(pvma, pvma->vm_start,
			framebuf->paddr >> PAGE_SHIFT,
			pvma->vm_end - pvma->vm_start, pvma->vm_page_prot)) {
			pr_err("x2fb mmap fail\n");
			return -EAGAIN;
		}
		pr_info("x2fb mmap end!:%llx\n", framebuf->paddr);
	} else if (flag == 1) {
		flag = 0;
		printk("channel3_en is 0x%x, fram_size is 0x%x.",
			x2_fbi->channel3_en, frame_size);
		if (x2_fbi->channel3_en && frame_size <= MAX_FRAME_BUF_SIZE) {
			printk(".......................................\n");
			if (framebuf) {
				printk("begin write framebuffer to pingpong buffer by dma.\n");
				//clean_cache(framebuf->vaddr, frame_size);
				__clean_dcache_area_poc(framebuf->vaddr, frame_size);
				ret = iar_write_framebuf_dma(2,
						framebuf->paddr, frame_size);
				pr_info("write framebuffer to pinpong buffer by dma success!\n");
			}
		}
	}
	return ret;
}
*/

static int x2fb_setcolreg(unsigned int regno, unsigned int red,
		unsigned int green, unsigned int blue, unsigned int transp,
		struct fb_info *info)
{
	int ret = 0;

	return ret;
}

static int x2fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	int ret = 0;

	return ret;
}
//static long x2fb_ioctl(struct file *filp, unsigned int cmd, unsigned long p)
//{
//	int ret = 0;
//	unsigned int frame_size = 0;
//	int ret = 0;
//	frame_buf_t *framebuf = x2_iar_get_framebuf_addr(2);
//	x2_fbi->channel3_en = 1;
//	iar_get_framesize();
//	frame_size = x2_fbi->update_cmd.frame_size[2];
//
//	pr_info("x2fb display begin!\n");
//
////	if (!framebuf || (pvma->vm_end - pvma->vm_start) > MAX_FRAME_BUF_SIZE)
////		return -ENOMEM;
////
////	pvma->vm_flags |= VM_IO;
////	pvma->vm_flags |= VM_LOCKED;
////	if (remap_pfn_range(pvma, pvma->vm_start, framebuf->paddr >> PAGE_SHIFT,
////			pvma->vm_end - pvma->vm_start, pvma->vm_page_prot)) {
////		pr_err("x2fb mmap fail\n");
////		return -EAGAIN;
////	}
////	pr_info("x2fb mmap end!:%llx\n", framebuf->paddr);
////	pr_info("channel3_en is 0x%x, fram_size is 0x%x.", x2_fbi->channel3_en, frame_size);
//	if (x2_fbi->channel3_en && frame_size <= MAX_FRAME_BUF_SIZE) {
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
static int x2fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	u_long line_length;

	pr_info("%s begin.\n", __func__);
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
	if (outmode == OUTPUT_RGB888 && lcd_type == RGB888_500) {
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

	} else if (outmode == OUTPUT_RGB888 && lcd_type == RGB888_700) {
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
	pr_info("%s: end.\n", __func__);
	return 0;
}


static int iar_paser_config(int type, int config_arrray[])
{
	int i;

	switch (type) {
	case USER_CFG:
//		{
//			iar_mode_t iarmode;
//			layer_config_t layercfg[IAR_CHANNEL_MAX];
//			for (i = 0; i < sizeof(iar_mode_t) /
//			sizeof(unsigned int); i++) {
//				unsigned int *tmp = (unsigned int *)&iarmode;
//				*(tmp + i) = config_arrray[i];
//			}
//			for (j = 0; j < (IAR_PRI_MAX)*sizeof(layer_config_t) /
//			sizeof(unsigned int); j++) {
//				unsigned int *tmp = (unsigned int *)&layercfg;
//				*(tmp + j) = config_arrray[i + j];
//			}
//			iar_fill_userconfig(&iarmode, layercfg);
//			iar_get_framesize();
//		}
		break;
	case CHANNEL_CFG:
	{
		unsigned int *cfg = (unsigned int *)(&x2_fbi->channel_base_cfg);

		for (i = 0; i < (IAR_CHANNEL_MAX)*sizeof(channel_base_cfg_t) /
						sizeof(unsigned int); i++)
			*(cfg + i) = config_arrray[i];
	}
	break;
	case SCALE_CFG:
	{
		unsigned int *cfg = (unsigned int *)(&x2_fbi->scale_cfg);

		for (i = 0; i < sizeof(upscaling_cfg_t) / sizeof(unsigned int);
				i++)
			*(cfg + i) = config_arrray[i];
	}
	break;
	case GAMMA_CFG:
	{
		gamma_cfg_t *cfg = &x2_fbi->gamma_cfg;

		for (i = 0; i < sizeof(gamma_cfg_t); i++) {
			gama_para_t *gama_para = (gama_para_t *)cfg + (i / 4);

			gama_para->bit.part_a = config_arrray[i++];
			gama_para->bit.part_b = config_arrray[i++];
			gama_para->bit.part_c = config_arrray[i++];
			gama_para->bit.part_d = config_arrray[i];
		}
	}
	break;
	case OUTPUT_CFG:
	{
		unsigned int *cfg = (unsigned int *)(&x2_fbi->output_cfg);

		for (i = 0; i < sizeof(output_cfg_t) /
				sizeof(unsigned int); i++)
			*(cfg + i) = config_arrray[i];
	}
	break;
	}
	return 0;
}
static int init_config(void)
{
	iar_paser_config(CHANNEL_CFG, channelconfig);
	iar_paser_config(SCALE_CFG, scale_config);
	iar_paser_config(GAMMA_CFG, gammma_config);
	iar_paser_config(OUTPUT_CFG, output_cfg);

	return 0;
}

static int iar_get_framesize(void)
{
	int i = 0;

	for (i = 0; i < IAR_CHANNEL_MAX; i++) {
		if (i == IAR_CHANNEL_1 || i == IAR_CHANNEL_2) {
			switch (x2_fbi->channel_base_cfg[i].format) {
			case FORMAT_YUV420P_VU:
			case FORMAT_YUV420P_UV:
			case FORMAT_YUV420SP_UV:
			case FORMAT_YUV420SP_VU:
				//x2_fbi->update_cmd.frame_size[i] =
				//(x2_fbi->channel_base_cfg[i].buf_height*
				//x2_fbi->channel_base_cfg[i].buf_width)*2;
				x2_fbi->update_cmd.frame_size[i] =
				(x2_fbi->channel_base_cfg[i].buf_height *
				 x2_fbi->channel_base_cfg[i].buf_width) * 3 / 2;
				break;
			case FORMAT_YUV422_UYVY:
			case FORMAT_YUV422_VYUY:
			case FORMAT_YUV422_YVYU:
			case FORMAT_YUV422_YUYV:
			case FORMAT_YUV422SP_UV:
			case FORMAT_YUV422SP_VU:
			case FORMAT_YUV422P_UV:
			case FORMAT_YUV422P_VU:
				//x2_fbi->update_cmd.frame_size[i] =
				//(x2_fbi->channel_base_cfg[i].buf_height*
				//x2_fbi->channel_base_cfg[i].buf_width)*3/2;
				x2_fbi->update_cmd.frame_size[i] =
				(x2_fbi->channel_base_cfg[i].buf_height *
				 x2_fbi->channel_base_cfg[i].buf_width) * 2;
				break;
			default:
				X2FB_DEBUG_PRINT("not supported	%d format%d\n",
					i, x2_fbi->channel_base_cfg[i].format);
				break;
			}
		} else {
			switch (x2_fbi->channel_base_cfg[i].format) {
			case FORMAT_ARGB8888:
			case FORMAT_RGBA8888:
				x2_fbi->update_cmd.frame_size[i] =
				(x2_fbi->channel_base_cfg[i].buf_height *
				 x2_fbi->channel_base_cfg[i].buf_width) * 4;
				break;
			case FORMAT_8BPP:
				x2_fbi->update_cmd.frame_size[i] =
				(x2_fbi->channel_base_cfg[i].buf_height *
				 x2_fbi->channel_base_cfg[i].buf_width);
				break;
			case FORMAT_RGB565:
				x2_fbi->update_cmd.frame_size[i] =
				(x2_fbi->channel_base_cfg[i].buf_height *
				 x2_fbi->channel_base_cfg[i].buf_width) * 2;
				break;
			case FORMAT_RGB888:
				pr_info("IAR channel 2 config format as RGB888!\n");
				x2_fbi->update_cmd.frame_size[i] =
				(x2_fbi->channel_base_cfg[i].buf_height *
				 x2_fbi->channel_base_cfg[i].buf_width) * 3;
				break;
			case FORMAT_RGB888P:
			default:
				X2FB_DEBUG_PRINT("not supported %d format %d\n",
					i, x2_fbi->channel_base_cfg[i].format);
				break;
			}
		}
	}
	return 0;
}

static int x2fb_set_par(struct fb_info *fb)
{

	int ret = 0;
	void __iomem *hitm1_reg_addr;
	//int hitm1_reg_value = 0;

	pr_info("%s: begin.\n", __func__);
	iar_stop();
//	init_config();

	X2FB_DEBUG_PRINT("## iar_parser_config.\n");

	if (display_type == HDMI_TYPE) {
		pr_info("fb set HDMI display!!!!\n");
		x2_fbi->memory_mode = 0;

		x2_fbi->channel_base_cfg[0].enable = 0;
		x2_fbi->channel_base_cfg[1].enable = 0;
		x2_fbi->channel_base_cfg[3].enable = 0;
		x2_fbi->channel_base_cfg[2].channel = IAR_CHANNEL_3;
		x2_fbi->channel_base_cfg[2].enable = x2_fbi->channel3_en;
		x2_fbi->update_cmd.enable_flag[2] = x2_fbi->channel3_en;
		x2_fbi->channel_base_cfg[2].enable = 1;
		x2_fbi->update_cmd.enable_flag[2] = 1;

		x2_fbi->channel_base_cfg[2].pri = 1;

		x2_fbi->channel_base_cfg[2].width = 1920;
		x2_fbi->channel_base_cfg[2].height = 1080;
		x2_fbi->channel_base_cfg[2].buf_width = 1920;
		x2_fbi->channel_base_cfg[2].buf_height = 1080;
		// x2_fbi->channel_base_cfg[2].format = 4;//ARGB8888
		// x2_fbi->channel_base_cfg[2].xposition = fb->var.xoffset;
		// x2_fbi->channel_base_cfg[2].yposition = fb->var.xoffset;
		x2_fbi->channel_base_cfg[2].alpha_sel = 0;
		x2_fbi->channel_base_cfg[2].ov_mode = 0;
		x2_fbi->channel_base_cfg[2].alpha_en = 1;
		x2_fbi->channel_base_cfg[2].alpha = 255;

		x2_fbi->output_cfg.out_sel = 1;  // 1-BT.
		x2_fbi->output_cfg.width = 1920;
		x2_fbi->output_cfg.height = 1080;
		x2_fbi->output_cfg.bgcolor = 16744328; // white.

		// iar_get_framesize();
		iar_channel_base_cfg(&x2_fbi->channel_base_cfg[2]);
		iar_output_cfg(&x2_fbi->output_cfg);
		iar_set_panel_timing(fb, display_type);

		iar_switch_buf(2);
		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x204, 4);
		writel(0x0, hitm1_reg_addr);
		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x00, 4);
		writel(0x041bf00f, hitm1_reg_addr);
//		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x68, 4);
//		writel(0x00000000, hitm1_reg_addr);//bg_color
		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x204, 4);
		writel(0x00000008, hitm1_reg_addr);//refresh
/*
 *		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x208, 4);
 *		hitm1_reg_value = 0x0280a030;
 *		writel(hitm1_reg_value, hitm1_reg_addr);
 *		//printk();
 *		vitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x20c, 4);
 *		vitm1_reg_value = 0x01e03003;
 *		writel(vitm1_reg_value, vitm1_reg_addr);
 *
 *		hitm2_reg_addr = ioremap_nocache(0xA4001000 + 0x210, 4);
 *		hitm2_reg_value = 0x0280a030;
 *		writel(hitm2_reg_value, hitm2_reg_addr);
 *		//printk();
 *		vitm2_reg_addr = ioremap_nocache(0xA4001000 + 0x214, 4);
 *		vitm2_reg_value = 0x01e03003;
 *		writel(vitm2_reg_value, vitm2_reg_addr);
 */
		iar_start(1);
		iar_switch_buf(2);

		msleep(500);
		msleep(500);
	} else if ((display_type == LCD_7_TYPE) &&
			(iar_config == CONFIG_CHANNEL3)) {
		//display rgb picture
		printk("fb set 7inch lcd display!!!\n");
		x2_fbi->memory_mode = 0;//????

		x2_fbi->channel_base_cfg[0].enable = 0;
		x2_fbi->channel_base_cfg[1].enable = 0;
		x2_fbi->channel_base_cfg[3].enable = 0;
		x2_fbi->channel_base_cfg[2].channel = IAR_CHANNEL_3;
		x2_fbi->channel_base_cfg[2].enable = x2_fbi->channel3_en;
		x2_fbi->update_cmd.enable_flag[2] = x2_fbi->channel3_en;
		x2_fbi->channel_base_cfg[2].enable = 1;
		x2_fbi->update_cmd.enable_flag[2] = 1;

		x2_fbi->channel_base_cfg[2].pri = 1;
		x2_fbi->channel_base_cfg[2].width = fb->var.xres;
		x2_fbi->channel_base_cfg[2].height = fb->var.yres;
		x2_fbi->channel_base_cfg[2].buf_width = fb->var.xres;
		x2_fbi->channel_base_cfg[2].buf_height = fb->var.yres;
		x2_fbi->channel_base_cfg[2].format = FORMAT_ARGB8888;//ARGB8888
		x2_fbi->channel_base_cfg[2].xposition = fb->var.xoffset;
		x2_fbi->channel_base_cfg[2].yposition = fb->var.xoffset;
		x2_fbi->channel_base_cfg[2].alpha_sel = 0;
		x2_fbi->channel_base_cfg[2].ov_mode = 0;
		x2_fbi->channel_base_cfg[2].alpha_en = 0;
		x2_fbi->channel_base_cfg[2].alpha = 255;

		x2_fbi->output_cfg.out_sel = 1;
		x2_fbi->output_cfg.width = 800;
		x2_fbi->output_cfg.height = 480;
		x2_fbi->output_cfg.bgcolor = 16744328;

		iar_get_framesize();
		iar_channel_base_cfg(&x2_fbi->channel_base_cfg[2]);
		iar_output_cfg(&x2_fbi->output_cfg);
		iar_set_panel_timing(fb, display_type);

		iar_switch_buf(2);
		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x00, 4);
		writel(0x041bf00f, hitm1_reg_addr);

//		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x68, 4);
//		//bg_color
//		writel(0x000000, hitm1_reg_addr);
		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x204, 4);
		writel(0x00000008, hitm1_reg_addr);//refresh
		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x318, 4);
		writel(0x00000000, hitm1_reg_addr);
		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x98, 4);
		writel(0x00000001, hitm1_reg_addr);//updat
		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x218, 4);
		writel(0x0000000a, hitm1_reg_addr);

		//hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x208, 4);
		//hitm1_reg_value = 0x0280a030;
		//writel(hitm1_reg_value, hitm1_reg_addr);
		//vitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x20c, 4);
		//vitm1_reg_value = 0x01e03003;
		//writel(vitm1_reg_value, vitm1_reg_addr);

		//hitm2_reg_addr = ioremap_nocache(0xA4001000 + 0x210, 4);
		//hitm2_reg_value = 0x0280a030;
		//writel(hitm2_reg_value, hitm2_reg_addr);
		//vitm2_reg_addr = ioremap_nocache(0xA4001000 + 0x214, 4);
		//vitm2_reg_value = 0x01e03003;
		//writel(vitm2_reg_value, vitm2_reg_addr);

		iar_start(1);
		iar_switch_buf(2);

		msleep(500);
		msleep(500);

		set_lt9211_config(&x2_fbi->fb, 0);
	} else if ((display_type == LCD_7_TYPE) &&
			(iar_config == CONFIG_CHANNEL1)) {
		//display video YUV420
		x2_fbi->memory_mode = 0;

		x2_fbi->channel_base_cfg[0].enable = 1;
		x2_fbi->channel_base_cfg[1].enable = 0;
		x2_fbi->channel_base_cfg[2].enable = 0;
		x2_fbi->channel_base_cfg[3].enable = 0;
		x2_fbi->channel_base_cfg[0].channel = IAR_CHANNEL_1;
		x2_fbi->channel_base_cfg[0].enable = 1;
		x2_fbi->update_cmd.enable_flag[0] = 1;
		//x2_fbi->channel_base_cfg[0].pri = 3;
		x2_fbi->channel_base_cfg[0].pri = 1;
		x2_fbi->channel_base_cfg[0].width = 800;
		x2_fbi->channel_base_cfg[0].height = 480;
		x2_fbi->channel_base_cfg[0].buf_width = 800;
		x2_fbi->channel_base_cfg[0].buf_height = 480;
		x2_fbi->channel_base_cfg[0].format = FORMAT_YUV420SP_UV;
		//x2_fbi->channel_base_cfg[2].xposition = fb->var.xoffset;
		//x2_fbi->channel_base_cfg[2].yposition = fb->var.xoffset;
		x2_fbi->channel_base_cfg[0].alpha_sel = 0;
		x2_fbi->channel_base_cfg[0].ov_mode = 0;
		x2_fbi->channel_base_cfg[0].alpha_en = 1;
		x2_fbi->channel_base_cfg[0].alpha = 255;

		x2_fbi->output_cfg.out_sel = 1;//1-BT.
		x2_fbi->output_cfg.width = 800;
		x2_fbi->output_cfg.height = 480;
		//x2_fbi->output_cfg.bgcolor = 16744328;// white.
		x2_fbi->output_cfg.bgcolor = 88888888;//green.

		iar_channel_base_cfg(&x2_fbi->channel_base_cfg[0]);
		iar_output_cfg(&x2_fbi->output_cfg);
		iar_set_panel_timing(fb, display_type);

		iar_switch_buf(0);

		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x00, 4);
		writel(0x011bf00f, hitm1_reg_addr);

		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x204, 4);
		writel(0x00000008, hitm1_reg_addr);

		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x48, 4);//format
		writel(0x00000006, hitm1_reg_addr);

		iar_start(1);
		iar_switch_buf(0);

		msleep(500);
		set_lt9211_config(&x2_fbi->fb, 0);

	} else if ((display_type == LCD_7_TYPE) &&
			(iar_config == CONFIG_CHANNEL13)) {
		//display video and picture
		x2_fbi->memory_mode = 0;

		x2_fbi->channel_base_cfg[0].enable = 1;
		x2_fbi->channel_base_cfg[1].enable = 0;
		x2_fbi->channel_base_cfg[2].enable = 1;
		x2_fbi->channel_base_cfg[3].enable = 0;
		x2_fbi->channel_base_cfg[0].channel = IAR_CHANNEL_1;
		x2_fbi->channel_base_cfg[0].enable = 1;
		x2_fbi->update_cmd.enable_flag[0] = 1;
		x2_fbi->update_cmd.enable_flag[2] = 1;
		x2_fbi->channel_base_cfg[0].pri = 3;
		x2_fbi->channel_base_cfg[0].width = 800;
		x2_fbi->channel_base_cfg[0].height = 480;
		x2_fbi->channel_base_cfg[0].buf_width = 800;
		x2_fbi->channel_base_cfg[0].buf_height = 480;
		x2_fbi->channel_base_cfg[0].format = FORMAT_YUV420SP_UV;
		x2_fbi->channel_base_cfg[0].alpha_sel = 0;
		x2_fbi->channel_base_cfg[0].ov_mode = 0;
		x2_fbi->channel_base_cfg[0].alpha_en = 1;
		x2_fbi->channel_base_cfg[0].alpha = 255;

		x2_fbi->channel_base_cfg[2].channel = IAR_CHANNEL_3;
		x2_fbi->channel_base_cfg[2].enable = 1;
		x2_fbi->update_cmd.enable_flag[2] = 1;
		x2_fbi->channel_base_cfg[2].pri = 1;
		x2_fbi->channel_base_cfg[2].width = 800;
		x2_fbi->channel_base_cfg[2].height = 480;
		x2_fbi->channel_base_cfg[2].buf_width = 800;
		x2_fbi->channel_base_cfg[2].buf_height = 480;
		x2_fbi->channel_base_cfg[2].format = 4;//ARGB8888
		x2_fbi->channel_base_cfg[2].alpha_sel = 0;
		x2_fbi->channel_base_cfg[2].ov_mode = 0;
		x2_fbi->channel_base_cfg[2].alpha_en = 1;
		x2_fbi->channel_base_cfg[2].alpha = 128;

		x2_fbi->output_cfg.out_sel = 1;
		x2_fbi->output_cfg.width = 800;
		x2_fbi->output_cfg.height = 480;
		//x2_fbi->output_cfg.bgcolor = 16744328;//white.
		x2_fbi->output_cfg.bgcolor = 88888888;//green

		iar_channel_base_cfg(&x2_fbi->channel_base_cfg[0]);
		iar_channel_base_cfg(&x2_fbi->channel_base_cfg[2]);
		iar_output_cfg(&x2_fbi->output_cfg);
		iar_set_panel_timing(fb, display_type);

		iar_switch_buf(0);
		iar_switch_buf(2);

		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x00, 4);
		writel(0x051bf00f, hitm1_reg_addr);

		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x204, 4);
		writel(0x00000008, hitm1_reg_addr);

		hitm1_reg_addr = ioremap_nocache(0xA4001000 + 0x48, 4);
		writel(0x00001c36, hitm1_reg_addr);

		iar_start(1);
		iar_switch_buf(0);
		iar_switch_buf(2);

		msleep(500);

		set_lt9211_config(&x2_fbi->fb, 0);
	}
	pr_info("%s: end.\n", __func__);

	return ret;

}


static void x2fb_activate_par(void)
{
	iar_update();
}

static struct fb_ops x2fb_ops = {
	.fb_check_var	= x2fb_check_var,
	.fb_set_par	= x2fb_set_par,
	.fb_setcolreg	= x2fb_setcolreg,
	.fb_pan_display	= x2fb_pan_display,
	.fb_blank	= x2fb_blank,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_mmap	= x2fb_mmap,
};

static int x2fb_probe(struct platform_device *pdev)
{
	int ret;
	frame_buf_t framebuf_user;

	pr_info("x2fb probe!!!\n");
	x2_fbi = devm_kzalloc(&pdev->dev, sizeof(struct x2fb_info), GFP_KERNEL);
	if (!x2_fbi) {
		dev_err(&pdev->dev, "Unable to alloc x2 framebuffer DEV\n");
		return -ENOMEM;
	}

	strcpy(x2_fbi->fb.fix.id, DRIVER_NAME);

	//res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	//x2_fbi->regaddr = devm_ioremap_resource(&pdev->dev, res);
	//if (IS_ERR(x2_fbi->regaddr))
	//return PTR_ERR(x2_fbi->regaddr);

//	framebuf_user = x2_iar_get_framebuf_addr(2);
	pr_info("x2 iar get framebuf addr begin here!\n");
	framebuf_user.paddr = 0x3D08A000 + MAX_FRAME_BUF_SIZE;
	pr_info("x2 iar get framebuf addr vaddr begin here!\n");
	framebuf_user.vaddr = memremap(0x3D08A000, MAX_FRAME_BUF_SIZE * 2,
			MEMREMAP_WB) + MAX_FRAME_BUF_SIZE;
	RGB500_fix_default.smem_start = framebuf_user.paddr;
	RGB700_fix_default.smem_start = framebuf_user.paddr;
	RGB500_fix_default.line_length =
		get_line_length(RGB500_var_default.xres_virtual,
				RGB500_var_default.bits_per_pixel);

	RGB700_fix_default.line_length =
		get_line_length(RGB700_var_default.xres_virtual,
				RGB700_var_default.bits_per_pixel);

	if (outmode == OUTPUT_RGB888 && lcd_type == RGB888_500) {
		x2_fbi->fb.fix = RGB500_fix_default;
		x2_fbi->fb.var = RGB500_var_default;
		x2_fbi->fb.flags = FBINFO_DEFAULT | FBINFO_HWACCEL_YPAN;
		x2_fbi->fb.fbops = &x2fb_ops;
		x2_fbi->fb.screen_base = framebuf_user.vaddr;
		x2_fbi->fb.screen_size = MAX_FRAME_BUF_SIZE;
		x2_fbi->fb.pseudo_palette = &x2fb_pseudo_palette;
		if (fb_alloc_cmap(&x2_fbi->fb.cmap, 256, 0))
			return -ENOMEM;
	} else if (outmode == OUTPUT_RGB888 && lcd_type == RGB888_700) {
		x2_fbi->fb.fix = RGB700_fix_default;
		x2_fbi->fb.var = RGB700_var_default;
		x2_fbi->fb.flags = FBINFO_DEFAULT | FBINFO_HWACCEL_YPAN;
		x2_fbi->fb.fbops = &x2fb_ops;
		x2_fbi->fb.screen_base = framebuf_user.vaddr;
		x2_fbi->fb.screen_size = MAX_FRAME_BUF_SIZE;
		x2_fbi->fb.pseudo_palette = &x2fb_pseudo_palette;
		if (fb_alloc_cmap(&x2_fbi->fb.cmap, 256, 0))
			return -ENOMEM;
	} else if (outmode == OUTPUT_BT1120 && lcd_type == RGB888_500) {
		x2_fbi->fb.fix = RGB500_fix_default;
		x2_fbi->fb.var = RGB500_var_default;
		x2_fbi->fb.fbops = &x2fb_ops;
		x2_fbi->fb.screen_base = framebuf_user.vaddr;
		x2_fbi->fb.screen_size = MAX_FRAME_BUF_SIZE;
		x2_fbi->fb.pseudo_palette = &x2fb_pseudo_palette;
		if (fb_alloc_cmap(&x2_fbi->fb.cmap, 256, 0))
			return -ENOMEM;
	} else if (outmode == OUTPUT_BT1120 && lcd_type == RGB888_700) {
		x2_fbi->fb.fix = RGB700_fix_default;
		x2_fbi->fb.var = RGB700_var_default;
		//x2_fbi->fb.flags = FBINFO_DEFAULT | FBINFO_HWACCEL_YPAN;
		x2_fbi->fb.fbops = &x2fb_ops;
		x2_fbi->fb.screen_base = framebuf_user.vaddr;
		x2_fbi->fb.screen_size = MAX_FRAME_BUF_SIZE;
		x2_fbi->fb.pseudo_palette = &x2fb_pseudo_palette;
		if (fb_alloc_cmap(&x2_fbi->fb.cmap, 256, 0))
			return -ENOMEM;

	} else if (outmode == OUTPUT_BT1120 && lcd_type == DSI_PANEL) {

	}
	init_config();

	platform_set_drvdata(pdev, x2_fbi);
	pr_info("*********begin register framebuffer********\n");
	ret = register_framebuffer(&x2_fbi->fb);
	pr_info("*********end register framebuffer**********\n");
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to register framebuffer device: %d\n", ret);
		if (x2_fbi->fb.cmap.len)
			fb_dealloc_cmap(&x2_fbi->fb.cmap);
		return ret;
	}

	/* create device files */
//	ret = device_create_file(&pdev->dev, &dev_attr_debug);
//	if (ret)
//		dev_err(&pdev->dev, "failed to add debug attribute\n");

	fb_info(&x2_fbi->fb, "%s frame buffer at 0x%lx-0x%lx\n",
		x2_fbi->fb.fix.id, x2_fbi->fb.fix.smem_start,
		x2_fbi->fb.fix.smem_start + x2_fbi->fb.fix.smem_len - 1);

	pr_info("x2 fb probe ok!!!\n");
	return 0;
}

static int x2fb_remove(struct platform_device *pdev)
{
	struct x2fb_info *fbi = platform_get_drvdata(pdev);

	iar_stop();
	//device_remove_file(&pdev->dev, &dev_attr_debug);

	unregister_framebuffer(&fbi->fb);

	if (fbi->fb.cmap.len)
		fb_dealloc_cmap(&fbi->fb.cmap);

	return 0;
}

static const struct of_device_id x2_dt_ids[] = {
	{ .compatible = "hobot,x2-fb", },
	{}
};

static struct platform_driver x2fb_driver = {
	.probe		= x2fb_probe,
	.remove		= x2fb_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = x2_dt_ids,
	},
};

int __init x2fb_init(void)
{
	int ret = platform_driver_register(&x2fb_driver);

	return ret;
}

static void __exit x2fb_cleanup(void)
{
	platform_driver_unregister(&x2fb_driver);
}

late_initcall(x2fb_init);
module_exit(x2fb_cleanup);

//module_platform_driver(x2fb_driver);

MODULE_AUTHOR("rui.guo <rui.guo@horizon.ai>");
MODULE_DESCRIPTION("Framebuffer driver for x2");
MODULE_LICENSE("GPL");
