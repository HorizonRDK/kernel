#ifndef __IPU_DDR_H__
#define __IPU_DDR_H__

extern struct x2_ipu_data *g_ipu;
extern int ddr_mode;

struct addr_info_t {
	uint16_t width;
	uint16_t height;
	uint16_t step;
	uint8_t *y_paddr;
	uint8_t *c_paddr;
	uint8_t *y_vaddr;
	uint8_t *c_vaddr;
};

struct src_img_info_t {
	int cam_id;
	int slot_id;
	int img_format;
	int frame_id;
	uint64_t timestamp;
	struct addr_info_t src_img;
};
#endif
