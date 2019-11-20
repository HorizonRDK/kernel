#ifndef __X2A_PYM_CONFIG_H__
#define __X2A_PYM_CONFIG_H__

#define MAX_PYM_DS_COUNT        24
#define MAX_PYM_US_COUNT        6

typedef struct pym_scale_box_s {
 u8  factor;
 u16 roi_x;
 u16 roi_y;
 u16 roi_width;
 u16 roi_height;
 u16 tgt_width;
 u16 tgt_height;
} pym_scale_box_t;


typedef struct pym_cfg_s {
 u8  img_scr;	//1: ipu mode 0: ddr mode
 u16 img_width;
 u16 img_height;
 u32 frame_id;
 u32 ds_uv_bypass;
 u16 ds_layer_en;
 u8  us_layer_en;
 u8  us_uv_bypass;
 pym_scale_box_t stds_box[MAX_PYM_DS_COUNT];
 pym_scale_box_t stus_box[MAX_PYM_US_COUNT];
} pym_cfg_t;

#endif
