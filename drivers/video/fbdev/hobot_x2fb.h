/*
 * Copyright rui.guo@horizon.ai
 */
#ifndef DRIVERS_VIDEO_FBDEV_HOBOT_X2FB_H_
#define DRIVERS_VIDEO_FBDEV_HOBOT_X2FB_H_

#include<linux/fb.h>
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
extern int display_type;
extern void *logo_vaddr;
extern int set_lt9211_config(struct fb_info *fb, unsigned int convert_type);
extern int32_t iar_write_framebuf_dma(uint32_t channel,
		phys_addr_t srcaddr, uint32_t size);

#endif
