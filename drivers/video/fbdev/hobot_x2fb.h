#if !defined (_X2FB_H)
#define _X2FB_H

#include<linux/fb.h>

extern int set_lt9211_config(struct fb_info *fb, unsigned int convert_type);
extern int32_t iar_write_framebuf_dma(uint32_t channel,
		phys_addr_t srcaddr, uint32_t size);

#endif
