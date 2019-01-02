#ifndef __IPU_DEV_H__
#define __IPU_DEV_H__

#include <linux/types.h>
#include <linux/compiler.h>

typedef struct {
	uint16_t w;
	uint16_t h;
} wh_t;

typedef struct {
	uint16_t l;
	uint16_t t;
	uint16_t w;
	uint16_t h;
} box_t;

typedef struct {
	uint8_t mipi_2_lines;
	uint8_t src_fmt;
	uint8_t uv_fmt;
	uint8_t scale_ddr_en;
	uint8_t crop_ddr_en;
	uint8_t crop_en;
	uint8_t to_pymid;
	uint8_t testpattern_en;
} ipu_ctrl_t;

/********************************************************************
 * @brief ltrb
 ********************************************************************/
typedef struct {
	wh_t crop_st;		/* stx, sty must even */
	wh_t crop_ed;		/* edx, edy must even */
} crop_t;

typedef struct {
	wh_t scale_src;		/* all be even */
	wh_t scale_tgt;		/* all be even */
	uint16_t step_x;
	uint16_t step_y;
	uint8_t bypass_x;
	uint8_t bypass_y;
	uint8_t pre_scale_x;
	uint8_t pre_scale_y;
} scale_t;

typedef struct {
	uint64_t y_addr;	/* align to 16 bytes */
	uint64_t c_addr;	/* align to 16 bytes */
} ddr_t;

typedef struct {
	uint8_t id_en;
	uint8_t bus_mode;	/* 0 for dvp/mipi, 1 for bt */
	uint8_t scale_en;
	uint8_t crop_en;
} frame_id_t;

typedef struct {
	box_t ds_roi[24];	/* index 0 is src w, h, others roi w, h */
	box_t us_roi[6];	/* us roi size, after up scale value */
	uint8_t sw_start;	/* only used in software mode */
	uint8_t src_from;	/* isp or ddr */
	uint8_t pymid_en;	/* 1 enable, 0 disable */
	uint8_t ds_layer_en;	/* direct max layer num, not reg style */
	uint8_t ds_factor[24];	/* all layer info, include 4, 8, 12, 16, 20 */
	uint16_t ds_src_width[24];
	uint32_t ds_uv_bypass;	/* layer mask */
	uint8_t us_layer_en;	/* layer mask */
	uint8_t us_uv_bypass;	/* layer mask */
	uint8_t us_factor[6];	/* normally use default value, 50,40,32,25,20,16 */
	uint16_t us_src_width[6];
	uint16_t reserved;
} pymid_t;

typedef struct {
	wh_t video_in;
	ipu_ctrl_t ctrl;
	crop_t crop;
	scale_t scale;
	pymid_t pymid;
	frame_id_t frame_id;
	ddr_t crop_ddr;
	ddr_t scale_ddr;
	ddr_t ds_ddr[24];
	ddr_t us_ddr[6];
} ipu_cfg_t;

typedef struct {
	wh_t video_in;
	ipu_ctrl_t ctrl;
	crop_t crop;
	scale_t scale;
	pymid_t pymid;
	frame_id_t frame_id;
} ipu_init_t;

enum {
	CROP_TO_DDR = 0x01,
	SCALAR_TO_DDR = 0x02,
	PYM_TO_DDR = 0x04,
};
#define ENABLE 1
#define DISABLE 0

int8_t ipu_dump_regs(void);
int8_t set_ipu_regbase(unsigned char __iomem * base);
int8_t clr_ipu_regbase(void);
int8_t set_ipu_ctrl(ipu_ctrl_t * info);
int8_t set_ipu_video_size(wh_t * info);
int8_t set_ipu_crop(crop_t * info);
int8_t set_ipu_scale(scale_t * info);
int8_t set_ipu_frame_id(frame_id_t * info);
int8_t set_ipu_pymid(pymid_t * info);
int8_t set_ipu_addr(uint8_t id, uint32_t y_addr, uint32_t c_addr);
int8_t set_ds_layer_addr(uint8_t id, uint32_t y_addr, uint32_t c_addr);
int8_t set_us_layer_addr(uint8_t id, uint32_t y_addr, uint32_t c_addr);
void ctrl_ipu_to_ddr(uint32_t module, bool status);

#endif
