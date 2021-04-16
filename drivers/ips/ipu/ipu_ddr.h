#ifndef __IPU_DDR_H__
#define __IPU_DDR_H__

#define PYM_SRC_FROM_ERR     -1
#define PYM_SRC_FROM_CROP    0
#define PYM_SRC_FROM_SCALE   1

extern struct x2_ipu_data *g_ipu;
extern int ddr_mode;

struct addr_info_t {
	uint16_t width;
	uint16_t height;
	uint16_t step;
	uint64_t y_paddr;
	uint64_t c_paddr;
	uint64_t y_vaddr;
	uint64_t c_vaddr;
};

struct src_img_info_t {
	int cam_id;
	int slot_id;
	int img_format;
	int frame_id;
	int64_t timestamp;
	struct addr_info_t src_img;
	struct addr_info_t scaler_img;
};

#define SRC_MAX 4
struct mult_img_info_t {
	int src_num;
	struct src_img_info_t src_img_info[SRC_MAX];
};
#endif
