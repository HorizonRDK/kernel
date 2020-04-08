/*
 * Copyright rui.guo@horizon.ai
 */
#ifndef __HOBOT_FB_H__
#define __HOBOT_FB_H__

#include<linux/fb.h>
#include<soc/hobot/hobot_iar.h>
#include"core/fb_draw.h"

//enum DISPLAY_TYPE {
//	LCD_7_TYPE,
//	HDMI_TYPE,
//};
enum IAR_CONFIG {
	CONFIG_CHANNEL3,
	CONFIG_CHANNEL1,
	CONFIG_CHANNEL13,

};
extern const struct linux_logo logo_linux_clut224;
//extern int display_type;
extern void *logo_vaddr;
extern int set_lt9211_config(struct fb_info *fb);
extern int32_t iar_write_framebuf_dma(uint32_t channel,
		phys_addr_t srcaddr, uint32_t size);
//extern struct disp_timing video_1920x1080;
//extern struct disp_timing video_800x480;
//extern struct disp_timing video_720x1280;
//extern struct disp_timing video_1080x1920;
//extern struct disp_timing video_720x1280_touch;

#endif
