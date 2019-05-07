#ifndef __IPU_FEED_BACK_H__
#define __IPU_FEED_BACK_H__

#include "ipu_dev.h"

#define DOWN_SCALE_MAX	24
#define UP_SCALE_MAX	6

extern struct x2_ipu_data *g_ipu;

struct addr_info_t {
	uint16_t width;
	uint16_t height;
	uint16_t step;
	uint8_t *y_paddr;
	uint8_t *c_paddr;
	uint8_t *y_vaddr;
	uint8_t *c_vaddr;
};
struct img_info_t {
	int slot_id;		// 缓存的slot
	int frame_id;		// for x2 may be 0 - 0xFFFF or 0x7FFF
	uint64_t timestamp;
	int img_format;		// now only support yuv420sp
	int ds_pym_layer;	// down scale pym 当前处理层数
	int us_pym_layer;	// up scale pym 当前处理层数
	struct addr_info_t src_img;	// for x2 src img = crop img
	struct addr_info_t down_scale[DOWN_SCALE_MAX];
	struct addr_info_t up_scale[UP_SCALE_MAX];
};

#endif
