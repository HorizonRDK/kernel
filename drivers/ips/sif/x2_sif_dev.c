/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
/**
 * @file     sif_dev.c
 * @brief    SIF Device function file
 * @author   tarryzhang (tianyu.zhang@hobot.cc)
 * @date     2017/7/6
 * @version  V1.0
 * @par      Horizon Robotics
 */
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/seq_file.h>

#include "x2_sif_dev.h"
#include "x2_sif_regs.h"
#include "x2_sif_utils.h"

/*SIF BASE REGISTER OFFSET*/
#define OFFSET_PIC_FORMAT      (0)
#define OFFSET_PIXEL_LEN       (4)
#define OFFSET_BUS_TYPE        (8)
#define OFFSET_VSYNC_INV       (12)
#define OFFSET_HSYNC_INV       (13)
#define OFFSET_PCLK_INV        (14)
#define OFFSET_PCLK_OUTINV     (15)
#define OFFSET_ERROR_CLEAR     (16)
#define OFFSET_DROP_FRAME      (19)
#define OFFSET_SIF_EN          (20)
#define OFFSET_DVP2AP_EN       (21)
#define OFFSET_BT2AP_EN        (22)
#define OFFSET_MIPI2AP_EN      (23)
#define OFFSET_RAW16_MODE      (24)
#define OFFSET_RAW20_MODE      (25)
#define OFFSET_YUV10_MODE      (26)
#define OFFSET_DUALRX_MODE     (27)
#define OFFSET_MIPI2AP_SEL     (28)
#define OFFSET_MOT_DET_EN      (29)
#define OFFSET_MOT_DET_REFRESH (30)

/*SIF VIDEO SIZE REGISTER OFFSET*/
#define OFFSET_INPUT_WIDTH     (16)
#define OFFSET_INPUT_HEIGHT    (0)
/*SIF BT MODE REGISTER OFFSET*/
#define OFFSET_BT_DETECT       (0)
#define OFFSET_BT_INEXCHANGE   (2)
#define OFFSET_BT_OUTEXCHANGE  (3)
/*SIF VIDEO IN SIZE ERR REGISTER OFFSET*/
#define OFFSET_WIDTH_ERROR     (0)
#define OFFSET_HEIGHT_ERROR    (16)

/*SIF DET LT REGISTER OFFSET*/
#define OFFSET_DET_L           (0)
#define OFFSET_DET_T           (16)

/*SIF DET WH REGISTER OFFSET*/
#define OFFSET_DET_W           (0)
#define OFFSET_DET_H           (16)

/*SIF DET PARA1 REGISTER OFFSET*/
#define OFFSET_DET_STEP        (0)
#define OFFSET_DET_THRESH      (8)
#define OFFSET_DET_DIFF_THRESH (16)

/*SIF DET PARA2 REGISTER OFFSET*/
#define OFFSET_WGT_DECAY       (0)
#define OFFSET_DEC_PREC        (8)

/*SIF FRAME ID CFG REGISTER OFFSET*/
#define OFFSET_ID_EN           (0)
#define OFFSET_ID_INIT         (8)
#define OFFSET_ID_SET_EN       (16)

#define SIF_ENABLE             (1)
#define SIF_DISABLE            (0)
#define DVP_RAW16_4_8          (0)
#define DVP_RAW16_8_4          (1)
#define DVP_RAW20_8_4          (0)
#define DVP_RAW20_10_2         (1)
#define DUALRX_SIDE_BY_SIDE    (0)
#define DUALRX_VIRTUAL_CHANNEL (1)
#define MIPI2AP_SEL_IPI0       (0)
#define MIPI2AP_SEL_IPI1       (1)
#define YUV42210_UV            (0)
#define YUV42210_VU            (1)
#define BT_SYNC_HEAD_16BITS    (0)
#define BT_SYNC_HEAD_H8BITS    (1)
#define BT_SYNC_HEAD_L8BITS    (2)
#define BT_IN_D_NOEXCHG        (0)
#define BT_OUT_D_NOEXCHG       (0)
#define BT_IN_D_EXCHG          (1)
#define BT_OUT_D_EXCHG         (1)

#define SIF_BITS1              (0x00000001)
#define SIF_BITS2              (0x00000003)
#define SIF_BITS3              (0x00000007)
#define SIF_BITS4              (0x0000000f)
#define SIF_BITS5              (0x0000001f)
#define SIF_BITS6              (0x0000003f)
#define SIF_BITS7              (0x0000007f)
#define SIF_BITS8              (0x000000ff)
#define SIF_BITS9              (0x000001ff)
#define SIF_BITS10             (0x000003ff)
#define SIF_BITS11             (0x000007ff)
#define SIF_BITS12             (0x00000fff)
#define SIF_BITS13             (0x00001fff)
#define SIF_BITS14             (0x00003fff)
#define SIF_BITS15             (0x00007fff)
#define SIF_BITS16             (0x0000ffff)
#define SIF_BITS17             (0x0001ffff)
#define SIF_BITS18             (0x0003ffff)
#define SIF_BITS19             (0x0007ffff)
#define SIF_BITS20             (0x000fffff)
#define SIF_BITS32             (0xffffffff)

#define VALUE_CLEAR(b,o)        (~((b) << (o)))
#define VALUE_SET(v,b,o)        (((v)&(b)) << (o))
#define VALUE_GET(v,b,o)        (((v)>>(o)) & (b))

typedef enum _sif_status_key_e {
	STATUS_BUS_WIDTH = 0,
	STATUS_BUS_HEIGHT,
	STATUS_SIF_WIDTH,
	STATUS_SIF_HEIGHT,
} sif_status_key_t;

typedef enum _sif_cfg_key_e {
	BASE_FORMAT = 0,
	BASE_PIX_LEN,
	BASE_BUS_TYPE,
	BASE_VSYNC_INV,
	BASE_HSYNC_INV,
	BASE_PCLK_IN_INV,
	BASE_PCLK_OUT_INV,
	BASE_ERROR_CLEAR,
	BASE_DROP_FRAME,
	BASE_SIF_ENABLE,
	BASE_DVP2AP_ENABLE,
	BASE_BT2AP_ENABLE,
	BASE_MIPI2AP_ENABLE,
	BASE_RAW_16BIT_MODE,
	BASE_RAW_20BIT_MODE,
	BASE_YUV_10BIT_MODE,
	BASE_DUALRX_MODE,
	BASE_MIPI2AP_SEL,
	SIZE_WIDTH,
	SIZE_HEIGHT,
	BT_DETECT_MODE,
	BT_IN_EXCHANGE,
	BT_OUT_EXCHANGE,
	MOT_DET_EN,
	MOT_DET_REFRESH,
	MOT_DET_ROI_L,
	MOT_DET_ROI_T,
	MOT_DET_ROI_W,
	MOT_DET_ROI_H,
	MOT_DET_STEP,
	MOT_DET_THRESH,
	MOT_DET_DIFF_THRESH,
	MOT_DET_DIFF_WGT_DECAY,
	MOT_DET_DIFF_DEC_PREC,
	FRAME_ID_EN,
	FRAME_ID_INIT,
	FRAME_ID_SET_EN,
} sif_cfg_key_t;

typedef enum _sif_table_e {
	TABLE_BITS = 0,
	TABLE_OFFSET,
	TABLE_MAX,
} sif_table_e;

const uint32_t g_sif_status_table[][TABLE_MAX] = {
	{SIF_BITS16, OFFSET_WIDTH_ERROR},	/*STATUS_ERROR_CLEAR */
	{SIF_BITS16, OFFSET_HEIGHT_ERROR},	/*STATUS_SIF_ENABLE */
	{SIF_BITS16, OFFSET_WIDTH_ERROR},	/*STATUS_DVP2AP_ENABLE */
	{SIF_BITS16, OFFSET_HEIGHT_ERROR},	/*STATUS_BT2AP_ENABLE */
};

const uint32_t g_sif_cfg_table[][TABLE_MAX] = {
	{SIF_BITS4, OFFSET_PIC_FORMAT},	/*BASE_FORMAT */
	{SIF_BITS3, OFFSET_PIXEL_LEN},	/*BASE_PIX_LEN */
	{SIF_BITS2, OFFSET_BUS_TYPE},	/*BASE_BUS_TYPE */
	{SIF_BITS1, OFFSET_VSYNC_INV},	/*BASE_VSYNC_INV */
	{SIF_BITS1, OFFSET_HSYNC_INV},	/*BASE_HSYNC_INV */
	{SIF_BITS1, OFFSET_PCLK_INV},	/*BASE_PCLK_IN_INV */
	{SIF_BITS1, OFFSET_PCLK_OUTINV},	/*BASE_PCLK_OUT_INV */
	{SIF_BITS1, OFFSET_ERROR_CLEAR},	/*BASE_ERROR_CLEAR */
	{SIF_BITS1, OFFSET_DROP_FRAME},	/*BASE_DROP_FRAME */
	{SIF_BITS1, OFFSET_SIF_EN},	/*BASE_SIF_ENABLE */
	{SIF_BITS1, OFFSET_DVP2AP_EN},	/*BASE_DVP2AP_ENABLE */
	{SIF_BITS1, OFFSET_BT2AP_EN},	/*BASE_BT2AP_ENABLE */
	{SIF_BITS1, OFFSET_MIPI2AP_EN},	/*BASE_MIPI2AP_ENABLE */
	{SIF_BITS1, OFFSET_RAW16_MODE},	/*BASE_RAW_16BIT_MODE */
	{SIF_BITS1, OFFSET_RAW20_MODE},	/*BASE_RAW_20BIT_MODE */
	{SIF_BITS1, OFFSET_YUV10_MODE},	/*BASE_YUV_10BIT_MODE */
	{SIF_BITS1, OFFSET_DUALRX_MODE},	/*BASE_DUALRX_MODE */
	{SIF_BITS1, OFFSET_MIPI2AP_SEL},	/*BASE_MIPI2AP_SEL */
	{SIF_BITS13, OFFSET_INPUT_WIDTH},	/*SIZE_WIDTH */
	{SIF_BITS13, OFFSET_INPUT_HEIGHT},	/*SIZE_HEIGHT */
	{SIF_BITS2, OFFSET_BT_DETECT},	/*BT_DETECT_MODE */
	{SIF_BITS1, OFFSET_BT_INEXCHANGE},	/*BT_DATA_IN_EXCHANGE */
	{SIF_BITS1, OFFSET_BT_OUTEXCHANGE},	/*BT_DATA_OUT_EXCHANGE */
	{SIF_BITS1, OFFSET_MOT_DET_EN},	/*MOT_DET_EN */
	{SIF_BITS1, OFFSET_MOT_DET_REFRESH},	/*MOT_DET_REFRESH */
	{SIF_BITS12, OFFSET_DET_L},	/*MOT_DET_ROI_L */
	{SIF_BITS12, OFFSET_DET_T},	/*MOT_DET_ROI_T */
	{SIF_BITS12, OFFSET_DET_W},	/*MOT_DET_ROI_W */
	{SIF_BITS12, OFFSET_DET_H},	/*MOT_DET_ROI_H */
	{SIF_BITS8, OFFSET_DET_STEP},	/*MOT_DET_STEP */
	{SIF_BITS8, OFFSET_DET_THRESH},	/*MOT_DET_THRESH */
	{SIF_BITS8, OFFSET_DET_DIFF_THRESH},	/*MOT_DET_DIFF_THRESH */
	{SIF_BITS8, OFFSET_WGT_DECAY},	/*MOT_DET_DIFF_WGT_DECAY */
	{SIF_BITS2, OFFSET_DEC_PREC},	/*MOT_DET_DIFF_DEC_PREC */
	{SIF_BITS1, OFFSET_ID_EN},	/*FRAME_ID_EN */
	{SIF_BITS16, OFFSET_ID_INIT},	/*FRAME_ID_INIT */
	{SIF_BITS1, OFFSET_ID_SET_EN},	/*FRAME_ID_SET_EN */
};

#define CONFIG_CLEAR(key)      VALUE_CLEAR(g_sif_cfg_table[key][TABLE_BITS], g_sif_cfg_table[key][TABLE_OFFSET])
#define CONFIG_SET(key, value) VALUE_SET(value, g_sif_cfg_table[key][TABLE_BITS], g_sif_cfg_table[key][TABLE_OFFSET])
#define CONFIG_GET(key, value) VALUE_GET(value, g_sif_cfg_table[key][TABLE_BITS], g_sif_cfg_table[key][TABLE_OFFSET])

typedef struct _sif_dev_s {
	void __iomem *iomem;
	int bustype;
} sif_dev_t;

sif_dev_t *g_sif_dev = NULL;

#ifndef CONFIG_X2_FPGA
#undef sif_getreg
#undef sif_putreg
extern int bifdev_get_cpchip_reg(unsigned int addr, int *value);
extern int bifdev_set_cpchip_reg(unsigned int addr, int value);
#define sif_getreg(a)          ({uint32_t value = 0;\
                                 bifdev_get_cpchip_reg((uint32_t)a, &value);\
                                 sifinfo("read 0x%x: 0x%x", (uint32_t)a, value);\
                                 value;})
#define sif_putreg(a,v)        ({bifdev_set_cpchip_reg((uint32_t)a, v);\
                                 sifinfo("write 0x%x: 0x%x", (uint32_t)a, v);})
#endif
static void sif_dev_base_config(sif_init_t * cfg)
{
	uint32_t base = 0;
	void __iomem *iomem = NULL;
	if (NULL == g_sif_dev) {
		siferr("sif dev not inited!");
		return;
	}
	iomem = g_sif_dev->iomem;
	g_sif_dev->bustype = cfg->bus_type;
	base |= CONFIG_SET(BASE_FORMAT, cfg->format);
	base |= CONFIG_SET(BASE_PIX_LEN, cfg->pix_len);
	base |= CONFIG_SET(BASE_BUS_TYPE, cfg->bus_type);
	base |= CONFIG_SET(BASE_VSYNC_INV, cfg->vsync_inv);
	base |= CONFIG_SET(BASE_HSYNC_INV, cfg->hsync_inv);
	base |= CONFIG_SET(BASE_PCLK_IN_INV, cfg->pclk_in_inv);
	base |= CONFIG_SET(BASE_PCLK_OUT_INV, cfg->pclk_out_inv);
	base |= CONFIG_SET(BASE_DROP_FRAME, cfg->drop_frame);
	base |= CONFIG_SET(BASE_RAW_16BIT_MODE, cfg->raw_16bit_mode);
	base |= CONFIG_SET(BASE_RAW_20BIT_MODE, cfg->raw_20bit_mode);
	base |= CONFIG_SET(BASE_YUV_10BIT_MODE, cfg->yuv_10bit_mode);
	base |= CONFIG_SET(BASE_DUALRX_MODE, cfg->dualrx_mode);
	base |= CONFIG_SET(BASE_MIPI2AP_SEL, cfg->mipi2ap_sel);
	if (BUS_TYPE_BT1120 == cfg->bus_type) {
		base |= CONFIG_SET(BASE_BT2AP_ENABLE, SIF_ENABLE);
	} else if (BUS_TYPE_DVP == cfg->bus_type) {
		base |= CONFIG_SET(BASE_DVP2AP_ENABLE, SIF_ENABLE);
	} else if (BUS_TYPE_MIPI == cfg->bus_type
		   || BUS_TYPE_DUALRX == cfg->bus_type) {
		base |= CONFIG_SET(BASE_MIPI2AP_ENABLE, SIF_ENABLE);
	}
	sif_putreg(iomem + REG_SIF_BASE_CTRL, base);
	return;
}

static void sif_dev_input_size(sif_init_t * cfg)
{
	uint32_t size = 0;
	void __iomem *iomem = NULL;
	if (NULL == g_sif_dev) {
		siferr("sif dev not inited!");
		return;
	}
	iomem = g_sif_dev->iomem;
	size |= CONFIG_SET(SIZE_WIDTH, cfg->width);
	size |= CONFIG_SET(SIZE_HEIGHT, cfg->height);
	sif_putreg(iomem + REG_SIF_VIDEO_IN_SIZE, size);
	return;
}

static void sif_dev_bt_mode(sif_init_t * cfg)
{
	uint32_t bt_mode = 0;
	void __iomem *iomem = NULL;
	if (NULL == g_sif_dev) {
		siferr("sif dev not inited!");
		return;
	}
	iomem = g_sif_dev->iomem;
	bt_mode |= CONFIG_SET(BT_DETECT_MODE, cfg->bt_detect_mode);
	bt_mode |= CONFIG_SET(BT_IN_EXCHANGE, cfg->bt_in_exchange);
	bt_mode |= CONFIG_SET(BT_OUT_EXCHANGE, cfg->bt_out_exchange);
	sif_putreg(iomem + REG_SIF_BT_CFG, bt_mode);
	return;
}

int32_t sif_dev_frame_id_cfg(frame_id_t * cfg)
{
	uint32_t frameid_cfg = 0;
	void __iomem *iomem = NULL;
	if (NULL == g_sif_dev) {
		siferr("sif dev not inited!");
		return -1;
	}
	iomem = g_sif_dev->iomem;
	frameid_cfg |= CONFIG_SET(FRAME_ID_EN, cfg->enable);
	frameid_cfg |= CONFIG_SET(FRAME_ID_SET_EN, cfg->fix_mode);
	frameid_cfg |= CONFIG_SET(FRAME_ID_INIT, cfg->init_value);
	sif_putreg(iomem + REG_FRAME_ID_CFG, frameid_cfg);
	return 0;
}

int32_t sif_dev_frame_id_get(frame_id_info_t * frameid)
{
	void __iomem *iomem = NULL;
	if (NULL == g_sif_dev) {
		siferr("sif dev not inited!");
		return -1;
	}
	iomem = g_sif_dev->iomem;
	if (g_sif_dev->bustype == BUS_TYPE_BT1120) {
		frameid->frame_id[0] = sif_getreg(iomem + REG_FRAME_ID_BT0);
		frameid->frame_id[1] = sif_getreg(iomem + REG_FRAME_ID_BT1);
		frameid->frame_id[2] = sif_getreg(iomem + REG_FRAME_ID_BT2);
		frameid->frame_id[3] = sif_getreg(iomem + REG_FRAME_ID_BT3);
	} else {
		frameid->frame_id[0] = sif_getreg(iomem + REG_FRAME_ID_VAL);
	}
	return 0;
}

int32_t sif_dev_mot_det_cfg(mot_det_t * cfg)
{
	uint32_t det_en = 0;
	uint32_t det_lt = 0;
	uint32_t det_wh = 0;
	uint32_t det_para1 = 0;
	uint32_t det_para2 = 0;
	void __iomem *iomem = NULL;
	if (NULL == g_sif_dev) {
		siferr("sif dev not inited!");
		return -1;
	}
	iomem = g_sif_dev->iomem;
	det_en = sif_getreg(iomem + REG_SIF_BASE_CTRL);
	det_en &= CONFIG_CLEAR(MOT_DET_EN);
	det_en &= CONFIG_CLEAR(MOT_DET_REFRESH);
	det_en |= CONFIG_SET(MOT_DET_EN, cfg->enable);
	det_en |= CONFIG_SET(MOT_DET_REFRESH, cfg->refresh);
	sif_putreg(iomem + REG_FRAME_ID_CFG, det_en);

	det_lt |= CONFIG_SET(MOT_DET_ROI_L, cfg->left);
	det_lt |= CONFIG_SET(MOT_DET_ROI_T, cfg->top);
	sif_putreg(iomem + REG_MOT_DET_ROI_LT, det_lt);

	det_wh |= CONFIG_SET(MOT_DET_ROI_W, cfg->width);
	det_wh |= CONFIG_SET(MOT_DET_ROI_H, cfg->height);
	sif_putreg(iomem + REG_MOT_DET_ROI_WH, det_wh);

	det_para1 |= CONFIG_SET(MOT_DET_STEP, cfg->step);
	det_para1 |= CONFIG_SET(MOT_DET_THRESH, cfg->thresh);
	det_para1 |= CONFIG_SET(MOT_DET_DIFF_THRESH, cfg->diff_thresh);
	sif_putreg(iomem + REG_MOT_PARA1, det_para1);

	det_para2 |= CONFIG_SET(MOT_DET_DIFF_WGT_DECAY, cfg->wgt_decay);
	det_para2 |= CONFIG_SET(MOT_DET_DIFF_DEC_PREC, cfg->dec_prec);
	sif_putreg(iomem + REG_MOT_PARA2, det_para2);

	return 0;
}

static void sif_dev_clear_err(void)
{
	uint32_t base = 0;
	void __iomem *iomem = NULL;
	if (NULL == g_sif_dev) {
		siferr("sif dev not inited!");
		return;
	}
	iomem = g_sif_dev->iomem;
	base = sif_getreg(iomem + REG_SIF_BASE_CTRL);
	base |= CONFIG_SET(BASE_ERROR_CLEAR, SIF_ENABLE);
	sif_putreg(iomem + REG_SIF_BASE_CTRL, base);
	return;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * @brief sif_dev_get_info : Get detail info of SIF Device
 *
 * @param [in] df_config : Use to save detail info read from SIF Device
 *
 * @return void
 */
void sif_dev_get_info(sif_info_t * info)
{
	uint32_t base = 0;
	uint32_t size = 0;
	uint32_t bt_mode = 0;
	void __iomem *iomem = NULL;
	if (NULL == g_sif_dev) {
		siferr("sif dev not inited!");
		return;
	}
	iomem = g_sif_dev->iomem;
	base = sif_getreg(iomem + REG_SIF_BASE_CTRL);
	info->format = CONFIG_GET(BASE_FORMAT, base);
	info->pix_len = CONFIG_GET(BASE_PIX_LEN, base);
	info->bus_type = CONFIG_GET(BASE_BUS_TYPE, base);
	info->vsync_inv = CONFIG_GET(BASE_VSYNC_INV, base);
	info->hsync_inv = CONFIG_GET(BASE_HSYNC_INV, base);
	info->pclk_in_inv = CONFIG_GET(BASE_PCLK_IN_INV, base);
	info->pclk_out_inv = CONFIG_GET(BASE_PCLK_OUT_INV, base);
	info->sif_enable = CONFIG_GET(BASE_SIF_ENABLE, base);
	info->drop_frame = CONFIG_GET(BASE_DROP_FRAME, base);
	info->dvp2ap_enable = CONFIG_GET(BASE_DVP2AP_ENABLE, base);
	info->bt2ap_enable = CONFIG_GET(BASE_BT2AP_ENABLE, base);
	info->mipi2ap_enable = CONFIG_GET(BASE_MIPI2AP_ENABLE, base);
	info->raw_16bit_mode = CONFIG_GET(BASE_RAW_16BIT_MODE, base);
	info->raw_20bit_mode = CONFIG_GET(BASE_RAW_20BIT_MODE, base);
	info->yuv_10bit_mode = CONFIG_GET(BASE_YUV_10BIT_MODE, base);
	info->dualrx_mode = CONFIG_GET(BASE_DUALRX_MODE, base);
	info->mipi2ap_sel = CONFIG_GET(BASE_MIPI2AP_SEL, base);

	size = sif_getreg(iomem + REG_SIF_VIDEO_IN_SIZE);
	info->width = CONFIG_GET(SIZE_WIDTH, size);
	info->height = CONFIG_GET(SIZE_HEIGHT, size);

	bt_mode = sif_getreg(iomem + REG_SIF_BT_CFG);
	info->bt_detect_mode = CONFIG_GET(BT_DETECT_MODE, bt_mode);
	info->bt_in_exchange = CONFIG_GET(BT_IN_EXCHANGE, bt_mode);
	info->bt_out_exchange = CONFIG_GET(BT_OUT_EXCHANGE, bt_mode);
}

/**
 * @brief sif_dev_get_status : Get status info of SIF Device
 *
 * @param [in] status : Use to save status info read from SIF Device
 *
 * @return void
 */
void sif_dev_get_status(sif_status_t * status)
{
	uint32_t size = 0;
	void __iomem *iomem = NULL;
	if (NULL == g_sif_dev) {
		siferr("sif dev not inited!");
		return;
	}
	iomem = g_sif_dev->iomem;
	size = sif_getreg(iomem + REG_SIF_DVP_ERR);
	status->dvp_width = CONFIG_GET(SIZE_WIDTH, size);
	status->dvp_height = CONFIG_GET(SIZE_HEIGHT, size);
	size = sif_getreg(iomem + REG_SIF_BT_ERR);
	status->bt_width = CONFIG_GET(SIZE_WIDTH, size);
	status->bt_height = CONFIG_GET(SIZE_HEIGHT, size);
	size = sif_getreg(iomem + REG_SIF_MIPI_ERR);
	status->mipi_width = CONFIG_GET(SIZE_WIDTH, size);
	status->mipi_height = CONFIG_GET(SIZE_HEIGHT, size);
	size = sif_getreg(iomem + REG_SIF_SIF_ERR);
	status->sif_width = CONFIG_GET(SIZE_WIDTH, size);
	status->sif_height = CONFIG_GET(SIZE_HEIGHT, size);
	sif_dev_clear_err();
}

/**
 * @brief sif_dev_start : SIF Device start working
 *
 * @param [] void :
 *
 * @return int32_t : OK/ERROR
 */
int32_t sif_dev_start(void)
{
	uint32_t base = 0;
	void __iomem *iomem = NULL;
	if (NULL == g_sif_dev) {
		siferr("sif dev not inited!");
		return -1;
	}
	iomem = g_sif_dev->iomem;
	base = sif_getreg(iomem + REG_SIF_BASE_CTRL);
	base |= CONFIG_SET(BASE_SIF_ENABLE, SIF_ENABLE);
	sif_putreg(iomem + REG_SIF_BASE_CTRL, base);
	return 0;
}

/**
 * @brief sif_dev_stop : SIF Device stop working
 *
 * @param [] void :
 *
 * @return int32_t : OK/ERROR
 */
int32_t sif_dev_stop(void)
{
	uint32_t base = 0;
	void __iomem *iomem = NULL;
	if (NULL == g_sif_dev) {
		siferr("sif dev not inited!");
		return -1;
	}
	iomem = g_sif_dev->iomem;
	base = sif_getreg(iomem + REG_SIF_BASE_CTRL);
	base &= CONFIG_CLEAR(BASE_SIF_ENABLE);
	sif_putreg(iomem + REG_SIF_BASE_CTRL, base);
	return 0;
}

/**
 * @brief sif_dev_init : SIF Device initialize
 *
 * @param [in] sif_cfg : SIF Device configuration
 *
 * @return int32_t : OK/ERROR
 */
int32_t sif_dev_init(sif_init_t * sif_cfg)
{
	sif_dev_base_config(sif_cfg);
	sif_dev_input_size(sif_cfg);
	if (BUS_TYPE_BT1120 == sif_cfg->bus_type) {
		sif_dev_bt_mode(sif_cfg);
	}
	return 0;
}

static int x2_sif_dev_probe(struct platform_device *pdev)
{
	int ret = 0;
	sif_dev_t *pack_dev = NULL;
#ifdef CONFIG_X2_FPGA
	struct resource *res;
#endif
	pack_dev = devm_kmalloc(&pdev->dev, sizeof(sif_dev_t), GFP_KERNEL);
	if (!pack_dev) {
		dev_err(&pdev->dev, "Unable to allloc sif pack dev.\n");
		return -ENOMEM;
	}
#ifdef CONFIG_X2_FPGA
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pack_dev->iomem = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pack_dev->iomem)) {
		devm_kfree(&pdev->dev, pack_dev);
		return PTR_ERR(pack_dev->iomem);
	}
#else
	pack_dev->iomem = (void __iomem *)0xA4000100;
#endif
	platform_set_drvdata(pdev, pack_dev);
	g_sif_dev = pack_dev;
	dev_info(&pdev->dev, "X2 sif dev prop OK\n");
	return ret;
}

static int x2_sif_dev_remove(struct platform_device *pdev)
{
	sif_dev_t *pack_dev = platform_get_drvdata(pdev);
#ifdef CONFIG_X2_FPGA
	devm_iounmap(&pdev->dev, pack_dev->iomem);
#endif
	devm_kfree(&pdev->dev, pack_dev);
	g_sif_dev = NULL;
	return 0;
}

static const struct of_device_id x2_sif_dev_match[] = {
	{.compatible = "hobot,x2-sif"},
	{}
};

MODULE_DEVICE_TABLE(of, x2_sif_dev_match);

static struct platform_driver x2_sif_dev_driver = {
	.probe = x2_sif_dev_probe,
	.remove = x2_sif_dev_remove,
	.driver = {
		   .name = "x2_sif_dev",
		   .of_match_table = x2_sif_dev_match,
		   },
};

module_platform_driver(x2_sif_dev_driver);
MODULE_AUTHOR("Zhang Tianyu <tianyu.zhang@hobot.cc>");
MODULE_DESCRIPTION("X2 SIF Dev Driver");
