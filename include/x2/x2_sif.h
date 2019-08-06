#ifndef __X2_SIF_H__
#define __X2_SIF_H__

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/io.h>

#define SIF_IOC_MAGIC 'v'
#define FRAME_ID_NUM_MAX  (4)

typedef enum _sif_event_e {
	SIF_START  = 1,
	SIF_STOP   = 2,
	SIF_ERROR  = 4,
	SIF_MOTDET = 8,
} sif_event_t;

typedef enum _sif_fmt_in_e {
	FMT_RAW         = 0,
	FMT_YUV422_YUYV = 8,
	FMT_YUV422_YVYU,
	FMT_YUV422_UYVY,
	FMT_YUV422_VYUY,
	FMT_MAX = 0x7fffffff,
} sif_fmt_in_t;

typedef enum _sif_pix_len_e {
	PIX_LEN_8,
	PIX_LEN_10,
	PIX_LEN_12,
	PIX_LEN_14,
	PIX_LEN_16,
	PIX_LEN_20,
	PIX_LEN_MAX = 0x7fffffff,
} sif_pix_len_t;

typedef enum _sif_bus_type_e {
	BUS_TYPE_DVP,
	BUS_TYPE_BT1120,
	BUS_TYPE_MIPI,
	BUS_TYPE_DUALRX,
	BUS_TYPE_MAX = 0x7fffffff,
} sif_bus_type_t;

typedef struct _frame_id_t {
	uint32_t    enable;
	uint32_t    init_value;
	uint32_t    fix_mode;
} frame_id_t;

typedef struct _mot_det_t {
	uint32_t    enable;
	uint32_t    refresh;
	uint32_t    top;
	uint32_t    left;
	uint32_t    width;
	uint32_t    height;
	uint32_t    step;
	uint32_t    thresh;
	uint32_t    diff_thresh;
	uint32_t    wgt_decay;
	uint32_t    dec_prec;
} mot_det_t;

typedef struct _sif_init_t {
	uint32_t    format;
	uint32_t    pix_len;
	uint32_t    bus_type;
	uint32_t    vsync_inv;
	uint32_t    hsync_inv;
	uint32_t    pclk_in_inv;
	uint32_t    pclk_out_inv;
	uint32_t    drop_frame;
	uint32_t    raw_16bit_mode;
	uint32_t    raw_20bit_mode;
	uint32_t    yuv_10bit_mode;
	uint32_t    dualrx_mode;
	uint32_t    mipi2ap_sel;
	uint32_t    bt_detect_mode;
	uint32_t    bt_in_exchange;
	uint32_t    bt_out_exchange;
	uint32_t    width;
	uint32_t    height;
	uint32_t    bypass_en;
} sif_init_t;

typedef struct _sif_cfg_t {
	sif_init_t  sif_init;
	mot_det_t   mot_det;
	frame_id_t  frame_id;
} sif_cfg_t;

typedef struct _sif_status_t {
	uint16_t    dvp_width;
	uint16_t    dvp_height;
	uint16_t    bt_width;
	uint16_t    bt_height;
	uint16_t    mipi_width;
	uint16_t    mipi_height;
	uint16_t    sif_width;
	uint16_t    sif_height;
} sif_status_t;

typedef struct _sif_info_t {
	uint16_t    format;
	uint16_t    pix_len;
	uint16_t    bus_type;
	uint16_t    vsync_inv;
	uint16_t    hsync_inv;
	uint16_t    pclk_in_inv;
	uint16_t    pclk_out_inv;
	uint16_t    sif_enable;
	uint16_t    drop_frame;
	uint16_t    dvp2ap_enable;
	uint16_t    bt2ap_enable;
	uint16_t    mipi2ap_enable;
	uint16_t    raw_16bit_mode;
	uint16_t    raw_20bit_mode;
	uint16_t    yuv_10bit_mode;
	uint16_t    dualrx_mode;
	uint16_t    mipi2ap_sel;
	uint16_t    bt_detect_mode;
	uint16_t    bt_in_exchange;
	uint16_t    bt_out_exchange;
	uint16_t    width;
	uint16_t    height;
} sif_info_t;

typedef struct _frame_id_info_t {
	uint32_t   frame_id[FRAME_ID_NUM_MAX];
}frame_id_info_t;

typedef struct _bypass_ctrl_info_t {
	uint32_t    port;
	uint32_t    enable;
} bypass_ctrl_info_t;

#define SIFIOC_INIT             _IOW(SIF_IOC_MAGIC, 0, sif_cfg_t)
#define SIFIOC_DEINIT           _IO(SIF_IOC_MAGIC,  1)
#define SIFIOC_START            _IO(SIF_IOC_MAGIC,  2)
#define SIFIOC_STOP             _IO(SIF_IOC_MAGIC,  3)
#define SIFIOC_GET_STATUS       _IOR(SIF_IOC_MAGIC, 4, sif_status_t)
#define SIFIOC_GET_INFO         _IOR(SIF_IOC_MAGIC, 5, sif_info_t)
#define SIFIOC_GET_FRAME_ID     _IOR(SIF_IOC_MAGIC, 6, frame_id_info_t)
#define SIFIOC_UPDATE           _IOW(SIF_IOC_MAGIC, 7, sif_cfg_t)
#define SIFIOC_BYPASS_CTRL      _IOW(SIF_IOC_MAGIC, 8, bypass_ctrl_info_t)

#endif //__X2_SIF_H__
