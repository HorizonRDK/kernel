#include <asm-generic/io.h>
#include "ipu_dev_regs.h"
#include "ipu_dev.h"
#include "ipu_common.h"

/* reg ipu ctrl */
#define SET_XC9080_EN				(1 << 9)
#define SET_FROM_ISP				(1 << 7)
#define SET_FMT_UV					(1 << 5)
#define SET_SCALE_DDR_EN			(1 << 4)
#define SET_CROP_DDR_EN 			(1 << 3)
#define SET_CROP_EN 				(1 << 2)
#define SET_TESTPATTERN_EN			(1 << 1)
#define SET_DATA_TO_PYMID			(1 << 0)
#define SET_IPU_NORMAL				(SET_FMT_UV | SET_DATA_TO_PYMID)	/* from sif, nv12, output to pymid */

/* reg ipu video in size, scale_src, scale_tgt */
#define SET_SCALE_X_BYPASS			(1 << 0)
#define SET_SCALE_Y_BYPASS			(1 << 1)
#define MAKEUP_WH(w, h) 			(((w) & 0x1fff) << 16 | ((h) & 0x1fff))
#define SET_SCALE_STEP(x, y)		(((y) & 0x3fff) << 16 | ((x) & 0x3fff))
#define SET_PRE_SCALE(x, y) 		(((y) & 0x7) << 5 | (((x) & 0x7) << 2))

/* reg crop_st, crop_ed */
#define MAKEUP_HW(w, h) 			(((h) & 0x1fff) << 16 | ((w) & 0x1fff))

/* reg ipu frame id */
#define SET_FRAME_ID_BT_MODE		(1 << 2)						/* 0 is dvp/mipi mode */
#define SET_FRAME_ID_SCALE_EN		(1 << 1)						/* enable frame id when scale to ddr */
#define SET_FRAME_ID_CROP_EN		(1 << 0)						/* enable frame id when crop to ddr */

/* reg pymid ds ctrl */
#define SET_START_PYMID_SW_MODE 	(1 << 7)						/* only used in software pymid mode */
#define SET_PYMID_HW_MODE			(1 << 6)						/* 0 for sw mode */
#define SET_PYMID_EN				(1 << 5)
#define SET_DS_UV_BYPASS(d) 		(((d) & 0x3f) << 8)
#define SET_DS_LAYER_EN(d)			(((d) & 0x1f) << 0)
#define SET_PYMID_DS_NORMAL 		(SET_PYMID_HW_MODE | SET_PYMID_EN | SET_DS_LAYER_EN(1))

/* reg pymid us ctrl */
#define SET_US_UV_BYPASS(d) 		(((d) & 0x3f) << 8)
#define SET_US_LAYER_EN(d)			(((d) & 0x3f) << 0)

/* ds factor */
/* factor 1 */
#define SET_DS_FACTOR_P1(d) 		(((d) & 0x3f) << 24)
#define SET_DS_FACTOR_P2(d) 		(((d) & 0x3f) << 18)
#define SET_DS_FACTOR_P3(d) 		(((d) & 0x3f) << 12)
#define SET_DS_FACTOR_P5(d) 		(((d) & 0x3f) << 6)
#define SET_DS_FACTOR_P6(d) 		(((d) & 0x3f) << 0)
/* factor 2 */
#define SET_DS_FACTOR_P7(d) 		(((d) & 0x3f) << 24)
#define SET_DS_FACTOR_P9(d) 		(((d) & 0x3f) << 18)
#define SET_DS_FACTOR_P10(d)		(((d) & 0x3f) << 12)
#define SET_DS_FACTOR_P11(d)		(((d) & 0x3f) << 6)
#define SET_DS_FACTOR_P13(d)		(((d) & 0x3f) << 0)
/* factor 3 */
#define SET_DS_FACTOR_P14(d)		(((d) & 0x3f) << 24)
#define SET_DS_FACTOR_P15(d)		(((d) & 0x3f) << 18)
#define SET_DS_FACTOR_P17(d)		(((d) & 0x3f) << 12)
#define SET_DS_FACTOR_P18(d)		(((d) & 0x3f) << 6)
#define SET_DS_FACTOR_P19(d)		(((d) & 0x3f) << 0)
/* factor 4 */
#define SET_DS_FACTOR_P21(d)		(((d) & 0x3f) << 12)
#define SET_DS_FACTOR_P22(d)		(((d) & 0x3f) << 6)
#define SET_DS_FACTOR_P23(d)		(((d) & 0x3f) << 0)

/* us factor */
/* factor 1 */
#define SET_US_FACTOR_P0(d) 		(((d) & 0x3f) << 24)
#define SET_US_FACTOR_P1(d) 		(((d) & 0x3f) << 16)
#define SET_US_FACTOR_P2(d) 		(((d) & 0x3f) << 8)
#define SET_US_FACTOR_P3(d) 		(((d) & 0x3f) << 0)

#define SET_US_FACTOR_P4(d) 		(((d) & 0x3f) << 8)
#define SET_US_FACTOR_P5(d) 		(((d) & 0x3f) << 0)

/* src width */
#define SET_SRC_WIDTH(d1, d2)		(((d1) & 0xfff) << 16 | ((d2) & 0xfff))

/* ds roi */
/* index 1,2,3 */
#define SET_ROI0_P0(l, t)			(((l) & 0x7ff) << 16 | ((t) & 0x7ff))
#define SET_ROI1_P0(w, h)			(((w) & 0xfff) << 16 | ((h) & 0xfff))
/* index 5,6,7 */
#define SET_ROI0_P1(l, t)			(((l) & 0x3ff) << 16 | ((t) & 0x3ff))
#define SET_ROI1_P1(w, h)			(((w) & 0x7ff) << 16 | ((h) & 0x7ff))
/* index 9,10,11 */
#define SET_ROI0_P2(l, t)			(((l) & 0x1ff) << 16 | ((t) & 0x1ff))
#define SET_ROI1_P2(w, h)			(((w) & 0x3ff) << 16 | ((h) & 0x3ff))
/* index 13,14,15 */
#define SET_ROI0_P3(l, t)			(((l) & 0xff) << 16 | ((t) & 0xff))
#define SET_ROI1_P3(w, h)			(((w) & 0x1ff) << 16 | ((h) & 0x1ff))
/* index 17,18,19 */
#define SET_ROI0_P4(l, t)			(((l) & 0x7f) << 16 | ((t) & 0x7f))
#define SET_ROI1_P4(w, h)			(((w) & 0xff) << 16 | ((h) & 0xff))
/* index 21,22,23 */
#define SET_ROI0_P5(l, t)			(((l) & 0x3f) << 16 | ((t) & 0x3f))
#define SET_ROI1_P5(w, h)			(((w) & 0x7f) << 16 | ((h) & 0x7f))

/* us roi */
#define SET_US_ROI0(l, t)			(((l) & 0x7ff) << 16 | ((t) & 0x7ff))
#define SET_US_ROI1(w, h)			(((w) & 0xfff) << 16 | ((h) & 0xfff))

unsigned char __iomem *g_regbase = NULL;

int8_t ipu_dump_regs(void)
{
	uint8_t 	   i = 0;
	uint32_t	   d = 0;
	void __iomem  *addr = NULL;

	for (i = 0; i < 17; i++) {
		addr = g_regbase + i * 4;
		d = readl(addr);
		ipu_dbg("0x%llx=0x%x\n", (uint64_t)addr, d);
	}

	for (i = 0; i < 16; i++) {
		addr = g_regbase + i * 4 + 0x100;
		d = readl(addr);
		ipu_dbg("0x%llx=0x%x\n", (uint64_t)addr, d);
	}

	for (i = 0; i < 16; i++) {
		addr = g_regbase + i * 4 + 0x200;
		d = readl(addr);
		ipu_dbg("0x%llx=0x%x\n", (uint64_t)addr, d);
	}

	return 0;
}

int8_t set_ipu_regbase(unsigned char __iomem *base)
{
	g_regbase = base;
	return 0;
}

int8_t clr_ipu_regbase(void)
{
	g_regbase = NULL;
	return 0;
}

int8_t set_ipu_ctrl(ipu_ctrl_t *info)
{
	uint32_t d = 0;

	if (g_regbase == NULL)
		return -1;

	if (info->mipi_2_lines == 1)
		d |= SET_XC9080_EN;

	if (info->src_fmt == 1)
		d |= SET_FROM_ISP;

	if (info->uv_fmt == 1)
		d |= SET_FMT_UV;

	if (info->crop_ddr_en == 1)
		d |= SET_CROP_DDR_EN;

	if (info->scale_ddr_en == 1)
		d |= SET_SCALE_DDR_EN;

	if (info->crop_en == 1)
		d |= SET_CROP_EN;

	if (info->to_pymid == 1)
		d |= SET_DATA_TO_PYMID;

	if (info->testpattern_en == 1)
		d |= SET_TESTPATTERN_EN;

	writel(d, g_regbase + IPU_CTRL);

	return 0;
}

int8_t set_ipu_video_size(wh_t *info)
{
	uint32_t d = 0;

	if (g_regbase == NULL)
		return -1;

	d = MAKEUP_WH(info->w, info->h);
	writel(d, g_regbase + IPU_VIDEO_IN);

	return 0;
}

int8_t set_ipu_crop(crop_t *info)
{
	uint32_t d = 0;

	if (g_regbase == NULL)
		return -1;

	d = MAKEUP_HW(info->crop_st.w, info->crop_st.h);
	writel(d, g_regbase + IPU_CROP_ST);

	d = MAKEUP_HW(info->crop_ed.w, info->crop_ed.h);
	writel(d, g_regbase + IPU_CROP_ED);

	return 0;
}

int8_t set_ipu_scale(scale_t *info)
{
	uint32_t d = 0;

	if (g_regbase == NULL)
		return -1;

	d = MAKEUP_WH(info->scale_src.w, info->scale_src.h);
	writel(d, g_regbase + IPU_SCALAR_SRC);

	d = MAKEUP_WH(info->scale_tgt.w, info->scale_tgt.h);
	writel(d, g_regbase + IPU_SCALAR_TGT);

	d = SET_SCALE_STEP(info->step_x, info->step_y);
	writel(d, g_regbase + IPU_SCALAR_STEP);

	d = SET_PRE_SCALE(info->pre_scale_x, info->pre_scale_y);
	if (info->bypass_x == 1)
		d |= SET_SCALE_X_BYPASS;
	if (info->bypass_y == 1)
		d |= SET_SCALE_Y_BYPASS;
	writel(d, g_regbase + IPU_SCALAR_BYPASS);

	return 0;
}

int8_t set_ipu_frame_id(frame_id_t *info)
{
	uint32_t d = 0;

	if (g_regbase == NULL)
		return -1;

	if (info->bus_mode == 1)
		d |= SET_FRAME_ID_BT_MODE;

	if (info->crop_en == 1)
		d |= SET_FRAME_ID_CROP_EN;

	if (info->scale_en == 1)
		d |= SET_FRAME_ID_SCALE_EN;

	writel(d, g_regbase + IPU_FRAME_ID);

	return 0;
}

int8_t set_ipu_pymid(pymid_t *info)
{
	uint32_t d = 0, s = 0, m = 0;
	int8_t i = 0;

	if (g_regbase == NULL)
		return -1;

	/* step 1. write ds ctrl reg */
	if (info->ds_uv_bypass != 0) {
		s = info->ds_uv_bypass;
		for (i = 23; i >= 1; i--) {
			if (i == 4 || i == 8 || i == 12\
						|| i == 16 || i == 20)
				continue;
			m <<= 1;
			if (s >> i & 0x1) {
				m |= 1;
			}
		}
	}
	d = SET_DS_UV_BYPASS(m);

	if (info->src_from == 0) {
		/* ddr mode */
		d &= ~SET_PYMID_HW_MODE;
	} else {
		d |= SET_PYMID_HW_MODE;
	}

	if (info->pymid_en == 1)
		d |= SET_PYMID_EN;

	m = 0;
	for (i = 0; i < info->ds_layer_en >> 2; i++) { // hard to understand
		m <<= 1;
		m |= 1;
	}
	d |= SET_DS_LAYER_EN(m);
	d |= (0xf << 28);
	writel(d, g_regbase + PYMID_DS_CTRL);

	/* step 1.2. write ds factor reg */
	d = SET_DS_FACTOR_P1(info->ds_factor[1]);
	d |= SET_DS_FACTOR_P2(info->ds_factor[2]);
	d |= SET_DS_FACTOR_P3(info->ds_factor[3]);
	d |= SET_DS_FACTOR_P5(info->ds_factor[5]);
	d |= SET_DS_FACTOR_P6(info->ds_factor[6]);
	writel(d, g_regbase + PYMID_FACTOR1);

	d = SET_DS_FACTOR_P7(info->ds_factor[7]);
	d |= SET_DS_FACTOR_P9(info->ds_factor[9]);
	d |= SET_DS_FACTOR_P10(info->ds_factor[10]);
	d |= SET_DS_FACTOR_P11(info->ds_factor[11]);
	d |= SET_DS_FACTOR_P13(info->ds_factor[13]);
	writel(d, g_regbase + PYMID_FACTOR2);

	d = SET_DS_FACTOR_P14(info->ds_factor[14]);
	d |= SET_DS_FACTOR_P15(info->ds_factor[15]);
	d |= SET_DS_FACTOR_P17(info->ds_factor[17]);
	d |= SET_DS_FACTOR_P18(info->ds_factor[18]);
	d |= SET_DS_FACTOR_P19(info->ds_factor[19]);
	writel(d, g_regbase + PYMID_FACTOR3);

	d = SET_DS_FACTOR_P21(info->ds_factor[21]);
	d |= SET_DS_FACTOR_P22(info->ds_factor[22]);
	d |= SET_DS_FACTOR_P23(info->ds_factor[23]);
	writel(d, g_regbase + PYMID_FACTOR4);

	/* step 1.3. write ds src width reg */
	d = SET_SRC_WIDTH(info->ds_src_width[2], info->ds_src_width[1]);
	writel(d, g_regbase + PYMID_SRC_WIDTH_P1);
	d |= SET_SRC_WIDTH(info->ds_src_width[5], info->ds_src_width[3]);
	writel(d, g_regbase + PYMID_SRC_WIDTH_P2);
	d |= SET_SRC_WIDTH(info->ds_src_width[7], info->ds_src_width[6]);
	writel(d, g_regbase + PYMID_SRC_WIDTH_P3);
	d |= SET_SRC_WIDTH(info->ds_src_width[10], info->ds_src_width[9]);
	writel(d, g_regbase + PYMID_SRC_WIDTH_P4);
	d |= SET_SRC_WIDTH(info->ds_src_width[13], info->ds_src_width[11]);
	writel(d, g_regbase + PYMID_SRC_WIDTH_P5);
	d |= SET_SRC_WIDTH(info->ds_src_width[15], info->ds_src_width[14]);
	writel(d, g_regbase + PYMID_SRC_WIDTH_P6);
	d |= SET_SRC_WIDTH(info->ds_src_width[18], info->ds_src_width[17]);
	writel(d, g_regbase + PYMID_SRC_WIDTH_P7);
	d |= SET_SRC_WIDTH(info->ds_src_width[21], info->ds_src_width[19]);
	writel(d, g_regbase + PYMID_SRC_WIDTH_P8);
	d |= SET_SRC_WIDTH(info->ds_src_width[23], info->ds_src_width[22]);
	writel(d, g_regbase + PYMID_SRC_WIDTH_P9);

	/* step 1.4. write ds roi reg */
	d = SET_ROI0_P0(info->ds_roi[1].l, info->ds_roi[1].t);
	writel(d, g_regbase + PYMID_ROI0_P1);
	d = SET_ROI1_P0(info->ds_roi[1].w, info->ds_roi[1].h);
	writel(d, g_regbase + PYMID_ROI1_P1);
	d = SET_ROI0_P0(info->ds_roi[2].l, info->ds_roi[2].t);
	writel(d, g_regbase + PYMID_ROI0_P2);
	d = SET_ROI1_P0(info->ds_roi[2].w, info->ds_roi[2].h);
	writel(d, g_regbase + PYMID_ROI1_P2);
	d = SET_ROI0_P0(info->ds_roi[3].l, info->ds_roi[3].t);
	writel(d, g_regbase + PYMID_ROI0_P3);
	d = SET_ROI1_P0(info->ds_roi[3].w, info->ds_roi[3].h);
	writel(d, g_regbase + PYMID_ROI1_P3);

	d = SET_ROI0_P1(info->ds_roi[5].l, info->ds_roi[5].t);
	writel(d, g_regbase + PYMID_ROI0_P5);
	d = SET_ROI1_P1(info->ds_roi[5].w, info->ds_roi[5].h);
	writel(d, g_regbase + PYMID_ROI1_P5);
	d = SET_ROI0_P1(info->ds_roi[6].l, info->ds_roi[6].t);
	writel(d, g_regbase + PYMID_ROI0_P6);
	d = SET_ROI1_P1(info->ds_roi[6].w, info->ds_roi[6].h);
	writel(d, g_regbase + PYMID_ROI1_P6);
	d = SET_ROI0_P1(info->ds_roi[7].l, info->ds_roi[7].t);
	writel(d, g_regbase + PYMID_ROI0_P7);
	d = SET_ROI1_P1(info->ds_roi[7].w, info->ds_roi[7].h);
	writel(d, g_regbase + PYMID_ROI1_P7);

	d = SET_ROI0_P2(info->ds_roi[9].l, info->ds_roi[9].t);
	writel(d, g_regbase + PYMID_ROI0_P9);
	d = SET_ROI1_P2(info->ds_roi[9].w, info->ds_roi[9].h);
	writel(d, g_regbase + PYMID_ROI1_P9);
	d = SET_ROI0_P2(info->ds_roi[10].l, info->ds_roi[10].t);
	writel(d, g_regbase + PYMID_ROI0_P10);
	d = SET_ROI1_P2(info->ds_roi[10].w, info->ds_roi[10].h);
	writel(d, g_regbase + PYMID_ROI1_P10);
	d = SET_ROI0_P2(info->ds_roi[11].l, info->ds_roi[11].t);
	writel(d, g_regbase + PYMID_ROI0_P11);
	d = SET_ROI1_P2(info->ds_roi[11].w, info->ds_roi[11].h);
	writel(d, g_regbase + PYMID_ROI1_P11);

	d = SET_ROI0_P3(info->ds_roi[13].l, info->ds_roi[13].t);
	writel(d, g_regbase + PYMID_ROI0_P13);
	d = SET_ROI1_P3(info->ds_roi[13].w, info->ds_roi[13].h);
	writel(d, g_regbase + PYMID_ROI1_P13);
	d = SET_ROI0_P3(info->ds_roi[14].l, info->ds_roi[14].t);
	writel(d, g_regbase + PYMID_ROI0_P14);
	d = SET_ROI1_P3(info->ds_roi[14].w, info->ds_roi[14].h);
	writel(d, g_regbase + PYMID_ROI1_P14);
	d = SET_ROI0_P3(info->ds_roi[15].l, info->ds_roi[15].t);
	writel(d, g_regbase + PYMID_ROI0_P15);
	d = SET_ROI1_P3(info->ds_roi[15].w, info->ds_roi[15].h);
	writel(d, g_regbase + PYMID_ROI1_P15);

	d = SET_ROI0_P4(info->ds_roi[17].l, info->ds_roi[17].t);
	writel(d, g_regbase + PYMID_ROI0_P17);
	d = SET_ROI1_P4(info->ds_roi[17].w, info->ds_roi[17].h);
	writel(d, g_regbase + PYMID_ROI1_P17);
	d = SET_ROI0_P4(info->ds_roi[18].l, info->ds_roi[18].t);
	writel(d, g_regbase + PYMID_ROI0_P18);
	d = SET_ROI1_P4(info->ds_roi[18].w, info->ds_roi[18].h);
	writel(d, g_regbase + PYMID_ROI1_P18);
	d = SET_ROI0_P4(info->ds_roi[19].l, info->ds_roi[19].t);
	writel(d, g_regbase + PYMID_ROI0_P19);
	d = SET_ROI1_P4(info->ds_roi[19].w, info->ds_roi[19].h);
	writel(d, g_regbase + PYMID_ROI1_P19);

	d = SET_ROI0_P5(info->ds_roi[21].l, info->ds_roi[21].t);
	writel(d, g_regbase + PYMID_ROI0_P21);
	d = SET_ROI1_P5(info->ds_roi[21].w, info->ds_roi[21].h);
	writel(d, g_regbase + PYMID_ROI1_P21);
	d = SET_ROI0_P5(info->ds_roi[22].l, info->ds_roi[22].t);
	writel(d, g_regbase + PYMID_ROI0_P22);
	d = SET_ROI1_P5(info->ds_roi[22].w, info->ds_roi[22].h);
	writel(d, g_regbase + PYMID_ROI1_P22);
	d = SET_ROI0_P5(info->ds_roi[23].l, info->ds_roi[23].t);
	writel(d, g_regbase + PYMID_ROI0_P23);
	d = SET_ROI1_P5(info->ds_roi[23].w, info->ds_roi[23].h);
	writel(d, g_regbase + PYMID_ROI1_P23);

	/* step 2. write us ctrl reg */
	d = SET_US_UV_BYPASS(info->us_uv_bypass);
	d |= SET_US_LAYER_EN(info->us_layer_en);
	writel(d, g_regbase + PYMID_US_CTRL);

	/* step 2.2. write us factor reg */
	if (info->us_factor[0] != 0 && info->us_factor[1] != 0 &&\
				info->us_factor[2] != 0 && info->us_factor[3] != 0) {
		d = SET_US_FACTOR_P0(info->us_factor[0]);
		d |= SET_US_FACTOR_P1(info->us_factor[1]);
		d |= SET_US_FACTOR_P2(info->us_factor[2]);
		d |= SET_US_FACTOR_P3(info->us_factor[3]);
		writel(d, g_regbase + PYMID_US_FACTOR1);
	}
	if (info->us_factor[4] != 0 && info->us_factor[5] != 0) {
		d = SET_US_FACTOR_P4(info->us_factor[4]);
		d |= SET_US_FACTOR_P5(info->us_factor[5]);
		writel(d, g_regbase + PYMID_US_FACTOR2);
	}
	/* step 2.3. write us src width reg */
	d = SET_SRC_WIDTH(info->ds_src_width[1], info->ds_src_width[0]);
	writel(d, g_regbase + PYMID_SRC_WIDTH_U1);
	d |= SET_SRC_WIDTH(info->ds_src_width[3], info->ds_src_width[2]);
	writel(d, g_regbase + PYMID_SRC_WIDTH_U2);
	d |= SET_SRC_WIDTH(info->ds_src_width[5], info->ds_src_width[4]);
	writel(d, g_regbase + PYMID_SRC_WIDTH_U3);

	/* step 2.4. write us roi reg */
	d = SET_US_ROI0(info->us_roi[0].l, info->us_roi[0].t);
	writel(d, g_regbase + PYMID_ROI0_U0);
	d = SET_US_ROI1(info->us_roi[0].w, info->us_roi[0].h);
	writel(d, g_regbase + PYMID_ROI1_U0);

	d = SET_US_ROI0(info->us_roi[1].l, info->us_roi[1].t);
	writel(d, g_regbase + PYMID_ROI0_U1);
	d = SET_US_ROI1(info->us_roi[1].w, info->us_roi[1].h);
	writel(d, g_regbase + PYMID_ROI1_U1);

	d = SET_US_ROI0(info->us_roi[2].l, info->us_roi[2].t);
	writel(d, g_regbase + PYMID_ROI0_U2);
	d = SET_US_ROI1(info->us_roi[2].w, info->us_roi[2].h);
	writel(d, g_regbase + PYMID_ROI1_U2);

	d = SET_US_ROI0(info->us_roi[3].l, info->us_roi[3].t);
	writel(d, g_regbase + PYMID_ROI0_U3);
	d = SET_US_ROI1(info->us_roi[3].w, info->us_roi[3].h);
	writel(d, g_regbase + PYMID_ROI1_U3);

	d = SET_US_ROI0(info->us_roi[4].l, info->us_roi[4].t);
	writel(d, g_regbase + PYMID_ROI0_U4);
	d = SET_US_ROI1(info->us_roi[4].w, info->us_roi[4].h);
	writel(d, g_regbase + PYMID_ROI1_U4);

	d = SET_US_ROI0(info->us_roi[5].l, info->us_roi[5].t);
	writel(d, g_regbase + PYMID_ROI0_U5);
	d = SET_US_ROI1(info->us_roi[5].w, info->us_roi[5].h);
	writel(d, g_regbase + PYMID_ROI1_U5);

	/* step 3. write pymid reg */
	d = MAKEUP_WH(info->ds_roi[0].w, info->ds_roi[0].h);
	writel(d, g_regbase + PYMID_SRC_IMG);

	return 0;
}

int8_t set_ipu_addr(uint8_t id, uint32_t y_addr, uint32_t c_addr)
{
	if (g_regbase == NULL)
		return -1;
	ipu_info("set ipuaddr %d 0x%x 0x%x \n", id, y_addr, c_addr);
	switch (id) {
	case 0:
		writel(y_addr, g_regbase + IPU_Y_BASE_ADDR_0);
		writel(c_addr, g_regbase + IPU_C_BASE_ADDR_0);
		break;
	case 1:
		writel(y_addr, g_regbase + IPU_Y_BASE_ADDR_1);
		writel(c_addr, g_regbase + IPU_C_BASE_ADDR_1);
		break;
	default:
		return -1;
	}
	return 0;
}

int8_t set_ds_src_addr(uint32_t y_addr, uint32_t c_addr)
{
	if (g_regbase == NULL)
		return -1;
	ipu_info("set_ds_src 0x%x 0x%x \n", y_addr, c_addr);
	writel(y_addr, g_regbase + PYMID_SRC_Y_ADDR);
	writel(c_addr, g_regbase + PYMID_SRC_C_ADDR);

	return 0;
}

int8_t set_ds_layer_addr(uint8_t id, uint32_t y_addr, uint32_t c_addr)
{
	if (y_addr == 0)
		return -1;

	if (g_regbase == NULL)
		return -1;
	ipu_info("set_ds %d 0x%x 0x%x \n", id, y_addr, c_addr);
	switch (id) {
	case 0:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P0);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P0);
		break;
	case 1:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P1);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P1);
		break;
	case 2:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P2);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P2);
		break;
	case 3:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P3);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P3);
		break;
	case 4:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P4);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P4);
		break;
	case 5:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P5);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P5);
		break;
	case 6:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P6);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P6);
		break;
	case 7:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P7);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P7);
		break;
	case 8:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P8);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P8);
		break;
	case 9:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P9);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P9);
		break;
	case 10:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P10);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P10);
		break;
	case 11:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P11);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P11);
		break;
	case 12:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P12);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P12);
		break;
	case 13:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P13);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P13);
		break;
	case 14:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P14);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P14);
		break;
	case 15:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P15);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P15);
		break;
	case 16:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P16);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P16);
		break;
	case 17:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P17);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P17);
		break;
	case 18:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P18);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P18);
		break;
	case 19:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P19);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P19);
		break;
	case 20:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P20);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P20);
		break;
	case 21:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P21);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P21);
		break;
	case 22:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P22);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P22);
		break;
	case 23:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_P23);
		writel(c_addr, g_regbase + PYMID_C_ADDR_P23);
		break;
	default:
		return -1;
	}
	return 0;
}

int8_t pym_manual_start(void)
{
	uint32_t regval = 0;
	ipu_info("pym_manual_start\n");
	regval = readl(g_regbase + PYMID_DS_CTRL);
	regval |= SET_START_PYMID_SW_MODE;
	writel(regval, g_regbase + PYMID_DS_CTRL);
	return 0;
}

int8_t set_us_layer_addr(uint8_t id, uint32_t y_addr, uint32_t c_addr)
{
	if (y_addr == 0)
		return -1;

	if (g_regbase == NULL)
		return -1;

	switch (id) {
	case 0:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_U0);
		writel(c_addr, g_regbase + PYMID_C_ADDR_U0);
		break;
	case 1:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_U1);
		writel(c_addr, g_regbase + PYMID_C_ADDR_U1);
		break;
	case 2:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_U2);
		writel(c_addr, g_regbase + PYMID_C_ADDR_U2);
		break;
	case 3:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_U3);
		writel(c_addr, g_regbase + PYMID_C_ADDR_U3);
		break;
	case 4:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_U4);
		writel(c_addr, g_regbase + PYMID_C_ADDR_U4);
		break;
	case 5:
		writel(y_addr, g_regbase + PYMID_Y_ADDR_U5);
		writel(c_addr, g_regbase + PYMID_C_ADDR_U5);
		break;
	default:
		return -1;
	}
	return 0;
}
void ctrl_ipu_to_ddr(uint32_t module, bool status)
{
	uint32_t regval = 0;

	if (g_regbase == NULL)
		return;

	if (module & CROP_TO_DDR) {
		regval = readl(g_regbase + IPU_CTRL);
		if (status)
			regval |= BIT(3);
		else
			regval &= ~BIT(3);
		writel(regval, g_regbase + IPU_CTRL);
	}
	if (module & SCALAR_TO_DDR) {
		regval = readl(g_regbase + IPU_CTRL);
		if (status)
			regval |= BIT(4);
		else
			regval &= ~BIT(4);
		writel(regval, g_regbase + IPU_CTRL);
	}
	if (module & PYM_TO_DDR) {
		regval = readl(g_regbase + IPU_CTRL);
		if (status) {
			ipu_info("start pym\n");
			regval |= SET_DATA_TO_PYMID;
		} else {
			regval &= ~SET_DATA_TO_PYMID;
			ipu_info("stop pym\n");
		}
		writel(regval, g_regbase + IPU_CTRL);
	}
	return;
}
