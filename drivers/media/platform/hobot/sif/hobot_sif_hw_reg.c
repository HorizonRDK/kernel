/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/delay.h>
#include "hobot_sif_hw_reg.h"
#include "hobot_dev_sif.h"
#include "sif_hw_api.h"

static uint32_t s_enable_pattern_gen = 0;

void sif_enable_frame_intr(void __iomem *base_reg, u32 mux_index,
				bool enable)
{
	vio_hw_set_field(base_reg, &sif_regs[SIF_FRM_EN_INT],
			&sif_fields[SW_SIF_FRM0_DONE_INT_EN - mux_index], enable);
	vio_hw_set_field(base_reg, &sif_regs[SIF_FRM_EN_INT],
			&sif_fields[SW_SIF_MUX0_OUT_FS_INT_EN - mux_index], enable);
	vio_hw_set_field(base_reg, &sif_regs[SIF_FRM_EN_INT],
			&sif_fields[SW_SIF_MIPI_TX_IPI0_FS_INT_EN - mux_index], enable);
}

u32 sif_get_frame_intr(void __iomem *base_reg)
{
	u32 intr = 0;
	intr = vio_hw_get_reg(base_reg, &sif_regs[SIF_FRM_EN_INT]);

	return intr;
}

/*
 * @brief Enable multi-frame interrupt
 *
 */
#ifdef SIF_TEST_MULTI_FRAME_ID
static void sif_enable_multi_frame_id(u32 __iomem *base_reg)
{
	vio_hw_set_field(base_reg, &sif_regs[SIF_FRM_EN_INT],
			&sif_fields[SW_SIF_MULTI_FRAME_ID_INT_EN], 1);

	vio_hw_set_field(base_reg, &sif_regs[SIF_MULTI_FRAME_INT],
			&sif_fields[SW_SIF_MULTI_FRAME_ID_INT_PERIOD], 1);

	vio_hw_set_field(base_reg, &sif_regs[SIF_MULTI_FRAME_INT],
			&sif_fields[SW_SIF_MULTI_FRAME_ID_INT_MUX_OUT_SELECT], 4);

	vio_hw_set_field(base_reg, &sif_regs[SIF_MULTI_FRAME_INT],
			&sif_fields[SW_SIF_MULTI_FRAME_ID_INT_TRIG_SELECT], 1);

	vio_hw_set_field(base_reg, &sif_regs[SIF_MULTI_FRAME_INT],
			&sif_fields[SW_SIF_MULTI_FRAME_ID_INT_ENABLE], 1);
}
#endif

/*
 * @brief Enable drop frame
 *
 */
#ifdef SIF_TEST_DROP_FRAME
static void sif_enable_drop_frame(u32 __iomem *base_reg, bool enable)
{
	vio_hw_set_field(base_reg, &sif_regs[SIF_SETTING],
			&sif_fields[SW_DROP_FRAME], enable);
}
#endif

/*
 * @brief Enable a frame interrupt
 *
 * @param mux_index the index of 8 mux output
 * @param ipi_index the index of 13 ipi/dvp input
 * @param lines the number of line buffers to use
 */
static void sif_enable_mux_out(u32 __iomem *base_reg, u32 mux_index,
				u32 ipi_index, u32 lines)
{
	if (mux_index < 8) {
		vio_hw_set_field(base_reg, &sif_regs[SIF_MUX_OUT_MODE],
				&sif_fields[SW_SIF_MUX0_OUT_ENABLE - mux_index], 1);
		vio_hw_set_field(base_reg, &sif_regs[SIF_IN_BUF_OVERFLOW_MASK],
				&sif_fields[SW_SIF_BUF0_CTRL_OVERFLOW_ERROR_MASK - mux_index],
				0);
		vio_hw_set_field(base_reg, &sif_regs[SIF_MUX_OUT_SEL],
				&sif_fields[SW_SIF_MUX0_OUT_SELECT - mux_index], ipi_index);
	}
	vio_info("mux_index = %d, ipi_index = %d\n", mux_index, ipi_index);
	if (lines == 2) {
		switch (mux_index) {
			case 0:
				vio_hw_set_field(base_reg, &sif_regs[SIF_MUX_OUT_MODE],
						&sif_fields[SW_SIF_BUF_CTRL_F2TO0_2LENGTH], 1);
				break;
			case 1:
				vio_hw_set_field(base_reg, &sif_regs[SIF_MUX_OUT_MODE],
						&sif_fields[SW_SIF_BUF_CTRL_F3TO1_2LENGTH], 1);
				break;
			case 4:
				vio_hw_set_field(base_reg, &sif_regs[SIF_MUX_OUT_MODE],
						&sif_fields[SW_SIF_BUF_CTRL_F6TO4_2LENGTH], 1);
				break;
			case 5:
				vio_hw_set_field(base_reg, &sif_regs[SIF_MUX_OUT_MODE],
						&sif_fields[SW_SIF_BUF_CTRL_F7TO5_2LENGTH], 1);
				break;
			default:
				vio_err("Mux Out can't support lines == 2: %d", mux_index);
		}
	} else if (lines == 4) {
		vio_hw_set_field(base_reg, &sif_regs[SIF_MUX_OUT_MODE],
				&sif_fields[SW_SIF_BUF_CTRL_F0TO3_DVP20BIT], 1);
	} else if (lines == 3 || lines > 4) {
		vio_err("unsupport lines(%d)\n", lines);
	}

}

void sif_hw_enable_bypass(u32 __iomem *base_reg, u32 ch_index, bool enable)
{
	vio_hw_set_field(base_reg, &sif_regs[SIF_VIO_BYPASS_CFG],
			&sif_fields[SW_MIPI_VIO_BYPASS_MUX0_ENABLE - ch_index], enable);
}
/*
 * @brief Enable a MIPI Rx to MIPI Tx
 *
 * @param rx_index the MIPI Rx Index
 * @param bypass_channels the number of bypass channels from the MIPI Rx
 * @return check whether valid to config or not
 */
static void sif_config_bypass(u32 __iomem *base_reg, u32 bypass_channels)
{
	int i = 0;
	u32 bypass_sel = bypass_channels % 10;
	u32 bypass_mask = (bypass_channels % 1000) / 10;
	u32 bypass_nsel = bypass_channels / 1000;

	if (bypass_sel < 0 || bypass_sel > 4)
		return;

	if (!bypass_nsel) {
		vio_hw_set_field(base_reg, &sif_regs[SIF_VIO_BYPASS_CFG],
			&sif_fields[SW_MIPI_VIO_BYPASS_MUX_SELECT], bypass_sel);
		if (bypass_sel == 2)
			bypass_mask = 0x1;
	}
	if (bypass_mask == 0)
		bypass_mask = 0x1;

	for (i = 0; i < 4; i++) {
		if (bypass_mask & (0x1 << i))
			sif_hw_enable_bypass(base_reg, i, true);
		else
			sif_hw_enable_bypass(base_reg, i, false);
	}
}

void sif_set_bypass_cfg(u32 __iomem *base_reg, sif_input_bypass_t *cfg)
{
	u32 bypass_channels = 0;
	int i = 0;

	bypass_channels = cfg->set_bypass_channels;

	if (cfg->enable_bypass) {
		if ((bypass_channels / 1000) &&
			vio_hw_get_field(base_reg, &sif_regs[SIF_VIO_BYPASS_CFG],
			&sif_fields[SW_MIPI_VIO_BYPASS_MUX_SELECT]) == 5) {
			vio_hw_set_field(base_reg, &sif_regs[SIF_VIO_BYPASS_CFG],
				&sif_fields[SW_MIPI_VIO_BYPASS_MUX_SELECT], 2);
		}
		sif_config_bypass(base_reg, bypass_channels);
		if ((bypass_channels % 10) == 2 && (bypass_channels / 1000) == 0) {
			vio_hw_set_field(base_reg, &sif_regs[SIF_FRM_EN_INT],
					&sif_fields[SW_SIF_MIPI_TX_IPI0_FS_INT_EN], 1);

			vio_hw_set_field(base_reg, &sif_regs[SIF_FRAME_ID_IAR_CFG],
						&sif_fields[SW_SIF_IAR_MIPI_FRAME_ID_ENABLE],
						cfg->enable_frame_id);
			if (cfg->init_frame_id != 0) {
				vio_hw_set_field(base_reg, &sif_regs[SIF_FRAME_ID_IAR_CFG],
						&sif_fields[SW_SIF_IAR_MIPI_FRAME_ID_SET_EN], 1);
				vio_hw_set_field(base_reg, &sif_regs[SIF_FRAME_ID_IAR_CFG],
						&sif_fields[SW_SIF_IAR_MIPI_FRAME_ID_INIT],
						cfg->init_frame_id);
			}
		}
	} else {
		if (vio_hw_get_field(base_reg, &sif_regs[SIF_VIO_BYPASS_CFG],
			&sif_fields[SW_MIPI_VIO_BYPASS_MUX_SELECT]) == 2) {
			vio_hw_set_field(base_reg, &sif_regs[SIF_VIO_BYPASS_CFG],
				&sif_fields[SW_MIPI_VIO_BYPASS_MUX_SELECT], 5);
		}
		for (i = 0; i < 4; i++) {
			sif_hw_enable_bypass(base_reg, i, false);
		}
	}
}

extern int testpattern_fps;
/*
 * Config Pattern Gen Engine
 *
 * @param pat_index 0:DVP, 1:IPI0, 2:IPI1, 3:IPI2, 4:IPI3
 */
void sif_set_pattern_gen(u32 __iomem *base_reg, u32 pat_index,
				sif_data_desc_t* p_data, u32 framerate)
{
	const u32 padding = 1;
	u32 is_raw = (p_data->format == 0);
	u32 y, cb, cr;
	u32 h_time;
	u32 v_line;
	const u32 clk_hz = vio_get_clk_rate("sif_mclk");

	if (framerate)
		testpattern_fps = framerate;
	else if (testpattern_fps)
		framerate = testpattern_fps;
	else
		framerate = testpattern_fps = 30;

	v_line = p_data->height + 400 + 1;
	h_time = clk_hz / framerate / v_line;
	while (h_time >= 0x10000) {
		v_line = v_line << 1;
		h_time = h_time >> 1;
	}
	if (h_time <= p_data->width) {
		vio_err("wrong fps%d, h_time(%d) <= width(%d), force set h_time\n",
				framerate, h_time, p_data->width);
		h_time = 4096;
	}

	y = 128;
	cr = 160;
	cb = 192;

	if (v_line < p_data->height)
		vio_err("Wrong argument: v_line:%d height%d", v_line, p_data->height);

	vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN0_SIZE + pat_index * 6],
			&sif_fields[SW_PAT_GEN0_HLINE_TIME + pat_index * 10], h_time);

	vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN0_SIZE + pat_index * 6],
			&sif_fields[SW_PAT_GEN0_VTOTAL_LINE + pat_index * 10], v_line);

	vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN0_IMG + pat_index * 6],
			&sif_fields[SW_PAT_GEN0_HACTIVE_PIX + pat_index * 10],
			p_data->width);

	vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN0_IMG + pat_index * 6],
			&sif_fields[SW_PAT_GEN0_VACTIVE_LINE + pat_index * 10],
			p_data->height);

	vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN0_CFG + pat_index * 6],
			&sif_fields[SW_PAT_GEN0_VBP + pat_index * 10], padding);

	vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN0_CFG + pat_index * 6],
			&sif_fields[SW_PAT_GEN0_YUV_OUT + pat_index * 10], !is_raw);

	vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN0_CFG + pat_index * 6],
			&sif_fields[SW_PAT_GEN0_MODE + pat_index * 10],
			is_raw ? ((pat_index % 2) + 4) : 0);

	vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN0_COL0 + pat_index * 6],
			&sif_fields[SW_PAT_GEN0_R_VAL + pat_index * 10],
			is_raw ? 2 + pat_index * 2 : y * 256);

	vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN0_COL1 + pat_index * 6],
			&sif_fields[SW_PAT_GEN0_G_VAL + pat_index * 10],
			is_raw ? 0 : cr * 256);
	vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN0_COL2 + pat_index * 6],
			&sif_fields[SW_PAT_GEN0_B_VAL + pat_index * 10],
			is_raw ? 0 : cb * 256);

	s_enable_pattern_gen |= (0x1 << pat_index);

}

void sif_start_pattern_gen(u32 __iomem *base_reg, u32 pat_index)
{
	int i = 0;

    for (i = 0; i < 5; i++) {
        if (!(s_enable_pattern_gen & (1 << i)))
            continue;

		switch(i){
			case 0:
				vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN_ENABLE],
						&sif_fields[SW_PAT_GEN0_ENABLE], 1);
				break;
			case 1:
				vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN_ENABLE],
						&sif_fields[SW_PAT_GEN1_ENABLE], 1);
				break;
			case 2:
				vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN_ENABLE],
						&sif_fields[SW_PAT_GEN2_ENABLE], 1);
				break;
			case 3:
				vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN_ENABLE],
						&sif_fields[SW_PAT_GEN3_ENABLE], 1);
				break;
			case 4:
				vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN_ENABLE],
						&sif_fields[SW_PAT_GEN4_ENABLE], 1);
				break;
			default:
				vio_err("wrong pat index %d\n", s_enable_pattern_gen);

		}
    }
}

static void sif_stop_pattern_gen(u32 __iomem *base_reg)
{
	s_enable_pattern_gen = 0;
	vio_hw_set_reg(base_reg, &sif_regs[SIF_PAT_GEN_IPI_EN], 0);
	vio_hw_set_reg(base_reg, &sif_regs[SIF_PAT_GEN_ENABLE], 0);
}

/*
 * @brief Set a DVP input
 *
 * Set all IPI belonging to the DVP Rx, and config all related functions
 * Configuration:
 *  - DVP (Width/Height/Format/Pixel Length)
 *  - Enable Frame Interrupt
 *  - Enable Frame ID
 *  - Enable Error Report
 * Finally, trigger the Shadow Update
 *
 * @param p_dvp the pointer of sif_input_dvp_t
 */
static void sif_set_dvp_input(u32 __iomem *base_reg, sif_input_dvp_t* p_dvp)
{
	u32 yuv_format	  = !!p_dvp->data.format;
	u32 mux_out_index = p_dvp->func.set_mux_out_index;
	u32 lines = (p_dvp->data.width / LINE_BUFFER_SIZE) + 1;
	const u32 input_index_start = 15;
	u32 enable_frame_id   = p_dvp->func.enable_frame_id;
	u32 enable_pattern	  = p_dvp->func.enable_pattern;

	if(!p_dvp->enable)
		return;

	vio_hw_set_field(base_reg, &sif_regs[SIF_DVP_IN_SET],
			&sif_fields[SW_DVP_IN_PIC_FORMAT], p_dvp->data.format);
	vio_hw_set_field(base_reg, &sif_regs[SIF_DVP_IN_SET],
			&sif_fields[SW_DVP_IN_PIX_LENGTH], p_dvp->data.pix_length);
	vio_hw_set_field(base_reg, &sif_regs[SIF_SETTING],
			&sif_fields[SW_HSYNC_INV], p_dvp->data.pix_length);
	vio_hw_set_field(base_reg, &sif_regs[SIF_SETTING],
			&sif_fields[SW_VSYNC_INV], p_dvp->data.pix_length);
	vio_hw_set_field(base_reg, &sif_regs[SIF_DVP_IN_CFG0],
			&sif_fields[SW_DVP_IN_HEIGHT], p_dvp->data.height);
	vio_hw_set_field(base_reg, &sif_regs[SIF_DVP_IN_CFG0],
			&sif_fields[SW_DVP_IN_WIDTH], p_dvp->data.width);

	/* Enable Error Report */
	vio_hw_set_field(base_reg, &sif_regs[SIF_ERR_STATUS_MASK],
			&sif_fields[SW_SIF_DVP_IN_V_ERROR_MASK], 0);

	// Output DVP if enable mux output
	if (p_dvp->func.enable_mux_out) {

		// 20Bit have to set 4 lines.
		if (p_dvp->data.pix_length == 5)
			lines = 4;

		sif_enable_mux_out(base_reg,
				mux_out_index,
				input_index_start,
				lines);
		// Frame Done Interrupt
		sif_enable_frame_intr(base_reg, mux_out_index, true);

		if (yuv_format) {
			if (lines > 1)
				vio_err("Not supported: YUV over 2K");
			else {
				sif_enable_mux_out(base_reg,
						mux_out_index + 1,
						input_index_start + 1,
						lines);
				// Frame Done Interrupt
				sif_enable_frame_intr(base_reg, mux_out_index + 1, true);
			}
		}


		vio_hw_set_field(base_reg, &sif_regs[SIF_FRM_ID_DVP_IN_CFG],
				&sif_fields[SW_DVP_FRAME_ID_ENABLE], enable_frame_id);
		vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN_IPI_EN],
				&sif_fields[SW_DVP_IN_PAT_GEN_OUT], enable_pattern);

		if (enable_pattern)
			sif_set_pattern_gen(base_reg, 0, &p_dvp->data, 0);

		if (yuv_format) {
			if (mux_out_index % 1) {
				vio_err("The mux_out_index should be even if using YUV format");
				return;
			} else {
				switch (mux_out_index / 2) {
					case 0:
						vio_hw_set_field(base_reg, &sif_regs[SIF_YUV422_TRANS],
								&sif_fields[SW_YUV422TO420SP_MUX01_ENABLE], 1);
						break;
					case 1:
						vio_hw_set_field(base_reg, &sif_regs[SIF_YUV422_TRANS],
								&sif_fields[SW_YUV422TO420SP_MUX23_ENABLE], 1);
						break;
					case 2:
						vio_hw_set_field(base_reg, &sif_regs[SIF_YUV422_TRANS],
								&sif_fields[SW_YUV422TO420SP_MUX45_ENABLE], 1);
						break;
					case 3:
						vio_hw_set_field(base_reg, &sif_regs[SIF_YUV422_TRANS],
								&sif_fields[SW_YUV422TO420SP_MUX67_ENABLE], 1);
						break;
				}
			}
		}
	}
	// FIXME: Workaround
	{
		vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_BUF_FIFO_SIZE],
				&sif_fields[SW_SIF_ISP0_PIC_FORMAT], p_dvp->data.format);
		vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_BUF_ISP0_CFG],
				&sif_fields[SW_SIF_ISP0_PIX_LENGTH], p_dvp->data.pix_length);
		vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_BUF_ISP0_CFG],
				&sif_fields[SW_SIF_ISP0_WIDTH], p_dvp->data.width);
		vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_BUF_ISP0_CFG],
				&sif_fields[SW_SIF_ISP0_HEIGHT], p_dvp->data.height);
	}
}

/*
 * @brief Set IAR input for bypass of MIPI CSI Tx
 *
 * @param p_iar the pointer of sif_input_iar_t
 */
static void sif_set_iar_input(u32 __iomem *base_reg, sif_input_iar_t *p_iar)
{
	u32 init_frame_id = p_iar->func.set_init_frame_id;

	if (!p_iar->enable)
		return;

	if (p_iar->func.enable_bypass) {
		sif_config_bypass(base_reg, 2);
		vio_hw_set_field(base_reg, &sif_regs[SIF_FRM_EN_INT],
				&sif_fields[SW_SIF_MIPI_TX_IPI0_FS_INT_EN], 1);
	}

	vio_hw_set_field(base_reg, &sif_regs[SIF_FRAME_ID_IAR_CFG],
				&sif_fields[SW_SIF_IAR_MIPI_FRAME_ID_ENABLE],
				p_iar->func.enable_frame_id);
	if(init_frame_id != 0){
		vio_hw_set_field(base_reg, &sif_regs[SIF_FRAME_ID_IAR_CFG],
				&sif_fields[SW_SIF_IAR_MIPI_FRAME_ID_SET_EN], 1);
		vio_hw_set_field(base_reg, &sif_regs[SIF_FRAME_ID_IAR_CFG],
				&sif_fields[SW_SIF_IAR_MIPI_FRAME_ID_INIT], init_frame_id);
	}
}

void sif_config_mipi_rx(u32 __iomem *base_reg, u32 index,
	sif_data_desc_t *data)
{
	switch(index){
		case 0:
			vio_hw_set_field(base_reg, &sif_regs[SIF_MIPI_RX_SET],
					&sif_fields[SW_MIPI_RX0_PIC_FORMAT], data->format);
			vio_hw_set_field(base_reg, &sif_regs[SIF_MIPI_RX_SET],
					&sif_fields[SW_MIPI_RX0_PIX_LENGTH], data->pix_length);
			break;
		case 1:
			vio_hw_set_field(base_reg, &sif_regs[SIF_MIPI_RX_SET],
					&sif_fields[SW_MIPI_RX1_PIC_FORMAT], data->format);
			vio_hw_set_field(base_reg, &sif_regs[SIF_MIPI_RX_SET],
					&sif_fields[SW_MIPI_RX1_PIX_LENGTH], data->pix_length);
			break;
		case 2:
			vio_hw_set_field(base_reg, &sif_regs[SIF_MIPI_RX_SET],
					&sif_fields[SW_MIPI_RX2_PIC_FORMAT], data->format);
			vio_hw_set_field(base_reg, &sif_regs[SIF_MIPI_RX_SET],
					&sif_fields[SW_MIPI_RX2_PIX_LENGTH], data->pix_length);
			break;
		case 3:
			vio_hw_set_field(base_reg, &sif_regs[SIF_MIPI_RX_SET],
					&sif_fields[SW_MIPI_RX3_PIC_FORMAT], data->format);
			vio_hw_set_field(base_reg, &sif_regs[SIF_MIPI_RX_SET],
					&sif_fields[SW_MIPI_RX3_PIX_LENGTH], data->pix_length);
			break;
		default:
			vio_err("wrong mipi rx index(%d)\n", index);
			break;
	}

}

void sif_enable_ipi_init_frameid(u32 __iomem *base_reg, u32 index,
				u32 vc_channel, bool enable)
{
	switch(index){
		case 0:
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX0_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX0_IPI0_FRAME_ID_SET_EN + 3 * vc_channel],
					enable);
			break;
		case 1:
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX1_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX1_IPI0_FRAME_ID_SET_EN + 3 * vc_channel],
					enable);
			break;
		case 2:
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX2_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX2_IPI0_FRAME_ID_SET_EN + 3 * vc_channel],
					enable);
			break;
		case 3:
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX3_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX3_IPI0_FRAME_ID_SET_EN + 3 * vc_channel],
					enable);
			break;
		default:
			vio_err("wrong mipi rx index(%d)\n", index);
			break;
	}
}

void sif_enable_init_frameid(u32 __iomem *base_reg, u32 index, bool enable)
{
	int ipi_channels = 0;
	int  i = 0;

	if(index == 0 || index == 1)
		ipi_channels = 4;
	else if(index == 2 || index == 3)
		ipi_channels = 2;

	for(i =0; i < ipi_channels; i++)
		sif_enable_ipi_init_frameid(base_reg, index, i, enable);

	// Shadow Update: IPI + DVP
	vio_hw_set_reg(base_reg, &sif_regs[SIF_SHD_UP_RDY], 0xFFFFFFFF);
}

void sif_config_rx_ipi(u32 __iomem *base_reg, u32 index, u32 vc_channel,
	sif_input_mipi_t* p_mipi)
{
	u32 enable_frame_id   = p_mipi->func.enable_frame_id;
	u32 enable_pattern    = p_mipi->func.enable_pattern;
	u32 init_frame_id     = p_mipi->func.set_init_frame_id;

	switch(index){
		case 0:
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_MIPI_RX0_CFG0 + vc_channel],
					&sif_fields[SW_MIPI_RX0_IPI0_ENABLE + 3 * vc_channel],
					p_mipi->enable);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_MIPI_RX0_CFG0 + vc_channel],
					&sif_fields[SW_MIPI_RX0_IPI0_HEIGHT + 3 * vc_channel],
					p_mipi->data.height);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_MIPI_RX0_CFG0 + vc_channel],
					&sif_fields[SW_MIPI_RX0_IPI0_WIDTH + 3 * vc_channel],
					p_mipi->data.width);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX0_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX0_IPI0_FRAME_ID_ENABLE + 3 * vc_channel],
					enable_frame_id);
			if(init_frame_id != 0){
				vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX0_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX0_IPI0_FRAME_ID_SET_EN + 3 * vc_channel],
					1);
				vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX0_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX0_IPI0_FRAME_ID_INIT + 3 * vc_channel],
					init_frame_id);
			}

			vio_hw_set_field(base_reg,
					&sif_regs[SIF_PAT_GEN_IPI_EN],
					&sif_fields[SW_RX0_IPI0_PAT_GEN_OUT - vc_channel],
					enable_pattern);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_ERR_STATUS_MASK],
					&sif_fields[SW_SIF_RX0_IPI0_H_ERROR_MASK - vc_channel], 0);
			vio_hw_set_field(base_reg, &sif_regs[SIF_ERR_STATUS_MASK],
					&sif_fields[SW_SIF_RX0_IPI0_V_ERROR_MASK - vc_channel], 0);
			break;
		case 1:
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_MIPI_RX1_CFG0 + vc_channel],
					&sif_fields[SW_MIPI_RX1_IPI0_ENABLE + 3 * vc_channel],
					p_mipi->enable);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_MIPI_RX1_CFG0 + vc_channel],
					&sif_fields[SW_MIPI_RX1_IPI0_HEIGHT + 3 * vc_channel],
					p_mipi->data.height);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_MIPI_RX1_CFG0 + vc_channel],
					&sif_fields[SW_MIPI_RX1_IPI0_WIDTH + 3 * vc_channel],
					p_mipi->data.width);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX1_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX1_IPI0_FRAME_ID_ENABLE + 3 * vc_channel],
					enable_frame_id);
			if(init_frame_id != 0){
				vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX1_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX1_IPI0_FRAME_ID_SET_EN + 3 * vc_channel],
					1);
				vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX1_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX1_IPI0_FRAME_ID_INIT + 3 * vc_channel],
					init_frame_id);
			}

			vio_hw_set_field(base_reg,
					&sif_regs[SIF_PAT_GEN_IPI_EN],
					&sif_fields[SW_RX1_IPI0_PAT_GEN_OUT - vc_channel],
					enable_pattern);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_ERR_STATUS_MASK],
					&sif_fields[SW_SIF_RX1_IPI0_H_ERROR_MASK - vc_channel],
					0);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_ERR_STATUS_MASK],
					&sif_fields[SW_SIF_RX1_IPI0_V_ERROR_MASK - vc_channel], 0);

			break;
		case 2:
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_MIPI_RX2_CFG0 + vc_channel],
					&sif_fields[SW_MIPI_RX2_IPI0_ENABLE + 3 * vc_channel],
					p_mipi->enable);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_MIPI_RX2_CFG0 + vc_channel],
					&sif_fields[SW_MIPI_RX2_IPI0_HEIGHT + 3 * vc_channel],
					p_mipi->data.height);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_MIPI_RX2_CFG0 + vc_channel],
					&sif_fields[SW_MIPI_RX2_IPI0_WIDTH + 3 * vc_channel],
					p_mipi->data.width);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX2_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX2_IPI0_FRAME_ID_ENABLE + 3 * vc_channel],
					enable_frame_id);
			if(init_frame_id != 0){
				vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX2_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX2_IPI0_FRAME_ID_SET_EN + 3 * vc_channel],
					1);
				vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX2_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX2_IPI0_FRAME_ID_INIT + 3 * vc_channel],
					init_frame_id);
			}

			vio_hw_set_field(base_reg, &sif_regs[SIF_PAT_GEN_IPI_EN],
					&sif_fields[SW_RX2_IPI0_PAT_GEN_OUT - vc_channel],
					enable_pattern);
			vio_hw_set_field(base_reg, &sif_regs[SIF_ERR_STATUS_MASK],
					&sif_fields[SW_SIF_RX2_IPI0_H_ERROR_MASK - vc_channel], 0);
			vio_hw_set_field(base_reg, &sif_regs[SIF_ERR_STATUS_MASK],
					&sif_fields[SW_SIF_RX2_IPI0_V_ERROR_MASK - vc_channel], 0);

			break;
		case 3:
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_MIPI_RX3_CFG0 + vc_channel],
					&sif_fields[SW_MIPI_RX3_IPI0_ENABLE + 3 * vc_channel],
					p_mipi->enable);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_MIPI_RX3_CFG0 + vc_channel],
					&sif_fields[SW_MIPI_RX3_IPI0_HEIGHT + 3 * vc_channel],
					p_mipi->data.height);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_MIPI_RX3_CFG0 + vc_channel],
					&sif_fields[SW_MIPI_RX3_IPI0_WIDTH + 3 * vc_channel],
					p_mipi->data.width);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX3_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX3_IPI0_FRAME_ID_ENABLE + 3 * vc_channel],
					enable_frame_id);
			if(init_frame_id != 0){
				vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX3_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX3_IPI0_FRAME_ID_SET_EN + 3 * vc_channel],
					1);
				vio_hw_set_field(base_reg,
					&sif_regs[SIF_FRM_ID_RX3_IPI0_CFG + vc_channel],
					&sif_fields[SW_RX3_IPI0_FRAME_ID_INIT + 3 * vc_channel],
					init_frame_id);
			}

			vio_hw_set_field(base_reg,
					&sif_regs[SIF_PAT_GEN_IPI_EN],
					&sif_fields[SW_RX3_IPI0_PAT_GEN_OUT - vc_channel],
					enable_pattern);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_ERR_STATUS_MASK],
					&sif_fields[SW_SIF_RX3_IPI0_H_ERROR_MASK - vc_channel],
					0);
			vio_hw_set_field(base_reg,
					&sif_regs[SIF_ERR_STATUS_MASK],
					&sif_fields[SW_SIF_RX3_IPI0_V_ERROR_MASK - vc_channel],
					0);

			break;
		default:
			vio_err("wrong mipi rx index(%d)\n", index);
			break;
	}

	if(enable_pattern)
		sif_set_pattern_gen(base_reg, vc_channel + 1, &p_mipi->data, 0);
}

void sif_set_dol_channels(sif_input_mipi_t* p_mipi, sif_output_isp_t *p_isp,
			u32 *ch_index)
{
	u32 vc_short_seq = 0;
	u32 vc_long_seq = 0;
	u32 vc_medium_seq = 0;
	u32 channels = 0;

	vc_short_seq = p_isp->func.vc_short_seq;
	vc_medium_seq = p_isp->func.vc_medium_seq;
	vc_long_seq = p_isp->func.vc_long_seq;
	channels = p_mipi->channels;

	vio_info("vc index for dol%d(%d, %d, %d)\n", channels,
			vc_long_seq, vc_medium_seq, vc_short_seq);

	if( vc_medium_seq >= channels ||
		vc_short_seq >= channels ||
		vc_long_seq >= channels) {
		vio_err("wrong vc index for dol(%d, %d, %d)\n",
				vc_long_seq, vc_medium_seq, vc_short_seq);
	}

	if (channels == 2) {
		if (vc_short_seq != vc_long_seq) {
			ch_index[0] = vc_long_seq;
			ch_index[1] = vc_short_seq;
		}
	} else if (channels == 3) {
		if (vc_short_seq != vc_long_seq &&
			vc_medium_seq != vc_long_seq ) {
			ch_index[0] = vc_long_seq;
			ch_index[1] = vc_short_seq;
			ch_index[2] = vc_medium_seq;
		}
	}
}
/*
 * @brief Set a MIPI Rx
 *
 * Set all IPI belonging to the MIPI Rx, and config all related functions
 * Configuration:
 *  - IPI (Width/Height/Format/Pixel Length)
 *  - DOL Mode (VC/Line Shift/ID Code)
 *  - MUX (Cross Bar)
 *  - Enable Frame Interrupt
 *  - Enable Frame ID & Initial Value
 *  - Enable Pattern Gen
 *  - Mipi Tx Bypass
 *  - Enable Error Report
 * Finally, trigger the Shadow Update
 *
 * @param p_mipi the pointer of sif_input_mipi_t
 */
static void sif_set_mipi_rx(u32 __iomem *base_reg, sif_input_mipi_t* p_mipi,
			sif_output_t *p_out, bool *online_ddr_enable)
{
	int i = 0;
	u32 enable_mux_out    = p_mipi->func.enable_mux_out;
	u32 set_mux_out_index = p_mipi->func.set_mux_out_index;
	u32 yuv_format        = !!p_mipi->data.format;
	const static u32 map_mux_input[] = {0, 4, 8, 10};
	u32 input_index_start = map_mux_input[p_mipi->mipi_rx_index];
	u32 i_step, mux_out_index, lines, ddr_mux_out_index;
	u8 *vc_index;
	u8 ipi_index = 0;
	u32 ch_index[3] = {0, 1, 2};

	if (!p_mipi->enable)
		return;

	/*yuv format setting*/
	if (yuv_format && enable_mux_out && p_out->ddr.enable) {
		if (set_mux_out_index % 2) {
			vio_err("The index of mux-out should be even if using YUV format");
			return;
		} else {
			switch (set_mux_out_index / 2) {
				case 0:
					vio_hw_set_field(base_reg, &sif_regs[SIF_YUV422_TRANS],
							&sif_fields[SW_YUV422TO420SP_MUX01_ENABLE], 1);
					break;
				case 1:
					vio_hw_set_field(base_reg, &sif_regs[SIF_YUV422_TRANS],
							&sif_fields[SW_YUV422TO420SP_MUX23_ENABLE], 1);
					break;
				case 2:
					vio_hw_set_field(base_reg, &sif_regs[SIF_YUV422_TRANS],
							&sif_fields[SW_YUV422TO420SP_MUX45_ENABLE], 1);
					break;
				case 3:
					vio_hw_set_field(base_reg, &sif_regs[SIF_YUV422_TRANS],
							&sif_fields[SW_YUV422TO420SP_MUX67_ENABLE], 1);
					break;
			}
		}
	}
	sif_config_mipi_rx(base_reg, p_mipi->mipi_rx_index, &p_mipi->data);
	vc_index = p_mipi->vc_index;
	for (i = 0; i < p_mipi->channels; i++) {
		ipi_index = vc_index[i];
		sif_config_rx_ipi(base_reg, p_mipi->mipi_rx_index, ipi_index, p_mipi);
		vio_info("%s: vc_index[%d] = %d\n", __func__, i, vc_index[i]);
	}
	if (p_mipi->func.enable_line_shift) {
		vio_hw_set_field(base_reg, &sif_regs[SIF_ISP_EXP_CFG],
				&sif_fields[SW_RX0_DOL_HDR_MODE - p_mipi->mipi_rx_index], 1);
		//TODO:SHITF
	} else if (p_mipi->func.enable_id_decoder) {
		vio_hw_set_field(base_reg, &sif_regs[SIF_ISP_EXP_CFG],
				&sif_fields[SW_RX0_DOL_HDR_MODE - p_mipi->mipi_rx_index], 3);
		vio_hw_set_field(base_reg, &sif_regs[SIF_ISP_EXP_CFG],
				&sif_fields[SW_MIPI_RX0_IDCODE_EXP_NUM - p_mipi->mipi_rx_index],
				p_mipi->channels);
	} else {
		vio_hw_set_field(base_reg, &sif_regs[SIF_ISP_EXP_CFG],
				&sif_fields[SW_RX0_DOL_HDR_MODE - p_mipi->mipi_rx_index], 2);
	}
	// Output all virtual channels if enable mux output
	if (p_mipi->func.enable_mux_out) {
		// YUV Format: it'll occupy two paths of mux out
		// One for Y plane, and the other for UV plane
		i_step = yuv_format ? 2 : 1;
		if (p_mipi->channels > 1)
			sif_set_dol_channels(p_mipi, &p_out->isp, ch_index);

		for (i = 0; i < p_mipi->channels; i += i_step) {
			mux_out_index = p_mipi->func.set_mux_out_index + ch_index[i];
			ddr_mux_out_index = p_out->ddr.mux_index + ch_index[i];

			lines = (p_mipi->data.width / LINE_BUFFER_SIZE) + 1;
			ipi_index = vc_index[i];
			sif_enable_mux_out(base_reg,
					mux_out_index,
					input_index_start + ipi_index,
					lines);

			if(ddr_mux_out_index != mux_out_index) {
				vio_hw_set_field(base_reg, &sif_regs[SIF_MUX_OUT_SEL],
						&sif_fields[SW_SIF_MUX0_OUT_SELECT - ddr_mux_out_index],
						input_index_start + ipi_index);
				sif_enable_frame_intr(base_reg, ddr_mux_out_index, true);
				*online_ddr_enable = true;
			}
			// Frame Done Interrupt
			sif_enable_frame_intr(base_reg, mux_out_index, true);

			if (yuv_format) {
				if (lines > 1)
					vio_err("Not supported: YUV over 2K");
				else {
					sif_enable_mux_out(base_reg,
							mux_out_index + 1,
							input_index_start + ipi_index + 1,
							lines);
					//sif_enable_frame_intr(base_reg, mux_out_index + 1, true);
				}
			}
		}
	}

	/*bypass enable*/
	if (p_mipi->func.enable_bypass) {
		sif_config_bypass(base_reg, p_mipi->func.set_bypass_channels);
	}

	// FIXME: Workaround
	vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_BUF_FIFO_SIZE],
			&sif_fields[SW_SIF_ISP0_PIC_FORMAT], p_mipi->data.format);
	vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_BUF_ISP0_CFG],
			&sif_fields[SW_SIF_ISP0_PIX_LENGTH], p_mipi->data.pix_length);
	vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_BUF_ISP0_CFG],
			&sif_fields[SW_SIF_ISP0_WIDTH], p_mipi->data.width);
	vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_BUF_ISP0_CFG],
			&sif_fields[SW_SIF_ISP0_HEIGHT], p_mipi->data.height);

	ips_set_md_resolution(p_mipi->data.width, p_mipi->data.height);
}

/*
 * @brief config an input buffer
 *
 * Set a buffer with a specific mux output index.
 *
 * @param p_ddr the pointer of sif_input_ddr_t
 *
 */
void sif_set_ddr_input(u32 __iomem *base_reg, sif_input_ddr_t* p_ddr)
{
	u32 height = 0;
	u32 width = 0;
	u32 format = 0;
	u32 pixel_length = 0;
	u32 stride = 0;
	int i = 0;
	if(!p_ddr->enable)
		return;

	height = p_ddr->data.height;
	width = p_ddr->data.width;
	format  = p_ddr->data.format;
	pixel_length = p_ddr->data.pix_length;

	sif_config_rdma_fmt(base_reg, pixel_length, width, height);
	stride = sif_get_stride(pixel_length, width);
	for(i = 0; i < 4; i++)
		sif_set_rdma_buf_stride(base_reg, i, stride);
}


/*
 * @brief transfer buffer's ownership to HW
 *
 * @param mux_out_index index of mux out
 * @param buf_index index of buffer on the specific mux ou
 *
 */
void sif_transfer_ddr_owner(u32 __iomem *base_reg, u32 mux_out_index,
	u32 buf_index)
{
	u32 shift = 0;

	if (mux_out_index < 8 && buf_index < 4) {
		shift = mux_out_index*4 + buf_index;
		vio_hw_set_field(base_reg, &sif_regs[SIF_AXI_BUS_OWNER],
				&sif_fields[SW_SIF_OUT_FRM0_W_DDR0_OWNER - shift], 1);
	} else {
		vio_err("sif_transfer_ddr_owner wrong index[%d,%d]\n",
				mux_out_index, buf_index);
	}
}

/*
 * @brief Get current buffer's address from SIF register
 *
 * @param mux_out_index index of mux out
 * @param buf_index index of buffer on the specific mux ou
 *
 */
u32 sif_get_ddr_addr(u32 __iomem *base_reg, u32 mux_out_index, u32 buf_index)
{
	u32 phyaddr = 0;

	phyaddr = vio_hw_get_reg(base_reg,
			&sif_regs[SIF_AXI_FRM0_W_ADDR0 + mux_out_index * 4 + buf_index]);

	return phyaddr;
}


/*
 * @brief config an output buffer
 *
 * Every output buffer belongs to a mux output, and a mux output has maximum 4 buffers.
 * Set a buffer with a specific mux output and buffer index.
 *
 * @param p_ddr the pointer of sif_output_ddr_t
 *
 */
void sif_set_ddr_output(u32 __iomem *base_reg, sif_output_ddr_t* p_ddr,
	u32 *enbale)
{
	if(!p_ddr->enable)
		return;

	if(p_ddr->stride)
		vio_hw_set_reg(base_reg,
			&sif_regs[SIF_AXI_FRM0_W_STRIDE + p_ddr->mux_index], p_ddr->stride);

	sif_set_wdma_enable(base_reg, p_ddr->mux_index, true);

	*enbale = 1;
}

/*
 * @brief config an ISP output
 *
 * Set the number of exposure, flyby mode, and dgain.
 *
 * @param p_isp the pointer of sif_output_isp_t
 *
 */
static void sif_set_isp_output(u32 __iomem *base_reg,
				sif_output_t *p_out)
{

	sif_output_isp_t *p_isp;

	u32 iram_addr_range[2][2] =
	{
		{0, 0x80000},     // 512KB
		{0x80000, 0x100000},    // 512KB
	};
	u32 iram_stride = 8192;
	u32 gain = 0;
	int i = 0;
	int iram_size = 0;

	p_isp = &p_out->isp;

	if (p_isp->dol_exp_num == 2) {
		if (p_isp->func.short_maxexp_lines) {
			iram_stride = p_out->ddr.stride;
			iram_size = iram_stride *
					(p_isp->func.short_maxexp_lines + 1);
			iram_addr_range[0][1] = iram_size;
		} else {
			iram_size = iram_addr_range[0][1];
		}
	} else if (p_isp->dol_exp_num == 3) {
		if (p_isp->func.short_maxexp_lines && p_isp->func.medium_maxexp_lines) {
			iram_stride = p_out->ddr.stride;
			iram_size = iram_stride *
					(p_isp->func.short_maxexp_lines + 1);
			iram_addr_range[1][0] = iram_addr_range[0][1];
			iram_size += iram_stride *
					(p_isp->func.medium_maxexp_lines + 1);
			iram_addr_range[1][1] = iram_size;
		} else {
			iram_size = iram_addr_range[1][1];
		}
	}

	if (iram_size > IRAM_MAX_RANG) {
		vio_err("beyond iram rang (0x%x)\n", iram_size);
		return;
	}
	ips_set_iram_size(iram_size);

	vio_dbg("%s: iram_stride = %d\n", __func__, iram_stride);
	if (p_isp->enable) {
		vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_EN_INT],
				&sif_fields[SIF_ISP0_OUT_FE_INT_EN], 1);
		vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_EN_INT],
				&sif_fields[SIF_ISP0_OUT_FS_INT_EN], 1);

		vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_BUF_CTRL],
				&sif_fields[SW_SIF_ISP0_FLYBY_ENABLE],
				p_isp->func.enable_flyby);
		/*vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_BUF_CTRL],
				&sif_fields[SW_SIF_ISP0_YUV_ENABLE],
				1);
		*/
		vio_hw_set_field(base_reg, &sif_regs[SIF_ISP_EXP_CFG],
				&sif_fields[SW_SIF_ISP0_DOL_EXP_NUM], p_isp->dol_exp_num);
		// Expected: At least one frame to ISP if flyby mode
		if (p_isp->func.enable_flyby) {
			// For DOL 2/3
			if (p_isp->dol_exp_num > 1) {
				vio_hw_set_reg(base_reg,
					&sif_regs[SIF_AXI_FRM1_W_ADDR0], iram_addr_range[0][0]);
				vio_hw_set_reg(base_reg,
					&sif_regs[SIF_AXI_FRM1_W_ADDR3], iram_addr_range[0][1]);
				vio_hw_set_reg(base_reg,
					&sif_regs[SIF_AXI_FRM1_W_STRIDE], iram_stride);
			}

			// For DOL 3
			if (p_isp->dol_exp_num > 2) {
				vio_hw_set_reg(base_reg,
					&sif_regs[SIF_AXI_FRM2_W_ADDR0], iram_addr_range[1][0]);
				vio_hw_set_reg(base_reg,
					&sif_regs[SIF_AXI_FRM2_W_ADDR3], iram_addr_range[1][1]);
				vio_hw_set_reg(base_reg,
					&sif_regs[SIF_AXI_FRM2_W_STRIDE], iram_stride);
			}
		}
	} else {
		vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_EN_INT],
				&sif_fields[SIF_ISP0_OUT_FE_INT_EN], 0);
		vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_EN_INT],
				&sif_fields[SIF_ISP0_OUT_FS_INT_EN], 0);

		vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_BUF_CTRL],
				&sif_fields[SW_SIF_ISP0_FLYBY_ENABLE], 0);
		vio_hw_set_field(base_reg, &sif_regs[SIF_ISP_EXP_CFG],
				&sif_fields[SW_SIF_ISP0_DOL_EXP_NUM], 0);
	}

	if (p_isp->func.enable_dgain) {
		gain = p_isp->func.set_dgain_short << 24
				| p_isp->func.set_dgain_medium << 16
				| p_isp->func.set_dgain_long << 8;
		vio_hw_set_reg(base_reg, &sif_regs[SIF_VC_GAIN_ISP0], gain);
	} else {
		vio_hw_set_reg(base_reg, &sif_regs[SIF_VC_GAIN_ISP0], 0);
	}

	for (i = 0; i < 4; i++) {
		if(p_isp->enable && i < p_isp->dol_exp_num){
			sif_set_wdma_enable(base_reg, i, true);
			sif_set_rdma_enable(base_reg, i, true);
		}
	}
}

/*
 * @brief config an IPU output
 *
 * @param p_ipu the pointer of sif_output_ipu_t
 *
 */
static void sif_set_ipu_output(u32 __iomem *base_reg,
	sif_output_ipu_t* p_ipu)
{
	vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_BUF_CTRL],
			&sif_fields[SW_SIF_IPU0_OUT_ENABLE], p_ipu->enable_flyby);
	vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_EN_INT],
			&sif_fields[SIF_IPU0_OUT_FE_INT_EN], p_ipu->enable_flyby);
	vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_EN_INT],
			&sif_fields[SIF_IPU0_OUT_FS_INT_EN], p_ipu->enable_flyby);
	if (p_ipu->enable_flyby) {
		// The format to IPU must be YUV422
		vio_hw_set_reg(base_reg, &sif_regs[SIF_YUV422_TRANS], 0);
		// otf to ipu doesn't pass axi/iram
		sif_enable_dma(base_reg, 0);
	}
}

/*
 * @brief config motion detection, and must config ISP output
 *
 * @param p_md the pointer of sif_output_md_t
 *
 */
void sif_set_md_output(u32 __iomem *base_reg, sif_output_md_t *p_md)
{
	if (!p_md->enable)
        return;

	ips_set_md_cfg(p_md);
	ips_set_md_refresh(1);

	if (p_md->path_sel == 1) {
		ips_set_md_fmt(0x8);
		vio_hw_set_field(base_reg, &sif_regs[SIF_MOT_DET_MODE],
				&sif_fields[SW_SIF_IPU_MD_ENABLE], 1);
	} else {
		ips_set_md_fmt(0x0);
		vio_hw_set_field(base_reg, &sif_regs[SIF_MOT_DET_MODE],
				&sif_fields[SW_SIF_ISP_MD_ENABLE], 1);
	}
}

void sif_hw_config(u32 __iomem *base_reg, sif_cfg_t* c)
{
	bool online_ddr_enable = 0;
	u32 enable_output_ddr = 0;
	u32 yuv_format = 0;
	u32 dol_exp_num = 0;
	sif_output_ddr_t ddr;

	// Input: IAR
	sif_set_iar_input(base_reg, &c->input.iar);

	// Input: DVP
	sif_set_dvp_input(base_reg, &c->input.dvp);

	// Input: SIF
	sif_set_mipi_rx(base_reg, &c->input.mipi, &c->output, &online_ddr_enable);

	// output: ddr
	sif_set_ddr_output(base_reg, &c->output.ddr, &enable_output_ddr);

	if(enable_output_ddr){
		yuv_format = !!c->input.mipi.data.format;
		dol_exp_num = c->output.isp.dol_exp_num;
		if(yuv_format || dol_exp_num > 1){
			memcpy(&ddr, &c->output.ddr, sizeof(sif_output_ddr_t));
			ddr.mux_index += 1;
			sif_set_ddr_output(base_reg, &ddr, &enable_output_ddr);
		}

		if(dol_exp_num > 2){
			memcpy(&ddr, &c->output.ddr, sizeof(sif_output_ddr_t));
			ddr.mux_index += 2;
			sif_set_ddr_output(base_reg, &ddr, &enable_output_ddr);
		}
	}

	if (enable_output_ddr) {
		if (!online_ddr_enable && !c->output.isp.func.enable_flyby)
			vio_hw_set_field(base_reg, &sif_regs[SIF_SETTING],
					&sif_fields[SW_SIF_OWNBIT_UNDERRUN_SKIP_FRM_ENABLE], 1);
		vio_hw_set_field(base_reg, &sif_regs[SIF_FRM_EN_INT],
				&sif_fields[SW_SIF_IN_BUF_OVERFLOW_INT_EN], 1);
		vio_hw_set_field(base_reg, &sif_regs[SIF_FRM_EN_INT],
				&sif_fields[SW_SIF_IN_SIZE_MISMATCH_INT_EN], 1);

	} else {
		vio_hw_set_field(base_reg, &sif_regs[SIF_SETTING],
				&sif_fields[SW_SIF_OWNBIT_UNDERRUN_SKIP_FRM_ENABLE], 0);
	}

	// Debug: It switches register value from shadow to HW state
	// sif_write_reg(SIF_SHD_UP_SEL, 0xFFFFFFFF);

#ifdef SIF_TEST_MULTI_FRAME_ID
	// Multi-Frame Interrupt
	sif_enable_multi_frame_id(base_reg);
#endif

	// Output: ISP (from DDR / online)
	sif_set_isp_output(base_reg, &c->output);

	// Output: IPU (only from DDR)
	sif_set_ipu_output(base_reg, &c->output.ipu);

	// Output: MD (only from ISP for NOW)
	sif_set_md_output(base_reg, &c->output.md);

	sif_set_ddr_input(base_reg, &c->input.ddr);

}

void sif_disable_ipi(u32 __iomem *base_reg, u8 ipi_channel)
{
	vio_hw_set_reg(base_reg, &sif_regs[SIF_FRM_ID_RX0_IPI0_CFG + ipi_channel],
			0);
	vio_hw_set_reg(base_reg, &sif_regs[SIF_MIPI_RX0_CFG0 + ipi_channel], 0);
}

static void sif_disable_input_and_output(u32 __iomem *base_reg)
{
	uint32_t t = 0, value = 0;
	int i = 0;
	// Disable Bypass && all TX IPIs
	// NOTICE: SW_MIPI_RX_OUT_TX_LINE_INS_ENABLE's default value = 1
	// To disable that will not attach the last blanking hsync at frame end.
	// The MIPI TX's IPI_LINE should set height + 1, while SW_MIPI_RX_OUT_TX_LINE_INS_ENABLE == 1

	// [FPGA] Disable Clone IPI
	vio_hw_set_field(base_reg, &sif_regs[SIF_VIO_BYPASS_CFG],
			&sif_fields[SW_MIPI_VIO_BYPASS_MUX0_ENABLE], 0);
	vio_hw_set_field(base_reg, &sif_regs[SIF_VIO_BYPASS_CFG],
			&sif_fields[SW_MIPI_VIO_BYPASS_MUX1_ENABLE], 0);
	vio_hw_set_field(base_reg, &sif_regs[SIF_VIO_BYPASS_CFG],
			&sif_fields[SW_MIPI_VIO_BYPASS_MUX2_ENABLE], 0);
	vio_hw_set_field(base_reg, &sif_regs[SIF_VIO_BYPASS_CFG],
			&sif_fields[SW_MIPI_VIO_BYPASS_MUX3_ENABLE], 0);

	// DVP
	vio_hw_set_field(base_reg, &sif_regs[SIF_DVP_IN_CFG0],
			&sif_fields[SW_DVP_IN_ENABLE], 0);

	sif_stop_pattern_gen(base_reg);
	// Clear: YUV Transform
	vio_hw_set_reg(base_reg, &sif_regs[SIF_YUV422_TRANS], 0);

	vio_hw_set_reg(base_reg, &sif_regs[SIF_MOT_DET_MODE], 0);

	for (i = 0; i < 12; i++)
		sif_disable_ipi(base_reg, i);
	// Shadow Update: IPI + DVP
	vio_hw_set_reg(base_reg, &sif_regs[SIF_SHD_UP_RDY], 0xFFFFFFFF);

	vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_BUF_CTRL],
			&sif_fields[SW_SIF_ISP0_FLYBY_ENABLE], 0);
	vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_BUF_CTRL],
			&sif_fields[SW_SIF_IPU0_OUT_ENABLE], 0);

	do
	{
		value = vio_hw_get_field(base_reg, &sif_regs[SIF_AXI_FRM_W_BUSY_RPT],
				&sif_fields[SW_SIF_OUT_FRM_IDLE_REPORT]);
		if (value == 1)
			break;

		if (t++ < 1000) {
			mdelay(1);
			vio_info("wait for SIF idle");
			continue;
		}
		vio_err("Timeout to wait idle: 1s");
	} while(0);
}


void sif_hw_disable(u32 __iomem *base_reg)
{
	/* Disable all inputs and wait until drained */
	sif_disable_input_and_output(base_reg);

	/* Disable & Clear all interrupts */

	vio_hw_set_reg(base_reg, &sif_regs[SIF_OUT_EN_INT], 0);
	vio_hw_set_reg(base_reg, &sif_regs[SIF_FRM_EN_INT], 0);
	vio_hw_set_reg(base_reg, &sif_regs[SIF_OUT_INT], 0xFFFFFFFF);
	vio_hw_set_reg(base_reg, &sif_regs[SIF_FRM_INT], 0xFFFFFFFF);

	/* SIF Enable */
	//remove it for test only
	vio_hw_set_field(base_reg, &sif_regs[SIF_SETTING],
			&sif_fields[SW_SIF_ENABLE], 0);


	/* SIF SW Reset */
	vio_hw_set_reg(base_reg, &sif_regs[SIF_SW_RESET], 0x00000001); //remove it for test only
	vio_hw_set_reg(base_reg, &sif_regs[SIF_SW_RESET], 0x00000000); //remove it for test only

	sif_set_isp_performance(base_reg, 0);
	ips_disable_md();

#ifdef SIF_MEMORY_DEBUG
	// For Debug: Memory Violation
	vio_hw_set_reg(base_reg, &sif_regs[SIF_AXI_FRM_W_LIMIT_SET], 1);
	vio_hw_set_reg(base_reg, &sif_regs[SIF_AXI_FRM_W_LIMIT_UP], 0x70000000);
	vio_hw_set_field(base_reg, &sif_regs[SIF_AXI_FRM_W_LIMIT_BOT], 0x40000000);
#endif
}

void sif_hw_enable(u32 __iomem *base_reg)
{
	// Clear statistic
	vio_hw_set_field(base_reg, &sif_regs[SIF_SETTING],
			&sif_fields[SW_STATICS_ERR_CLR], 1);
	vio_hw_set_field(base_reg, &sif_regs[SIF_SETTING],
			&sif_fields[SW_STATICS_ERR_CLR], 0);

	// TODO: DVP 20bit mode
#if 0
	value = vio_hw_get_reg(base_reg, &sif_regs[SIF_SETTING]);
	vio_hw_set_reg(base_reg, &sif_regs[SIF_SETTING], value | (0x01 << 25));
#endif

#ifdef SIF_TEST_DROP_FRAME
	// To enable drop frame will be applied immediately, should not set this on streaming
	vio_hw_set_field(base_reg, &sif_regs[SIF_SETTING],
			&sif_fields[SW_DROP_FRAME_ENABLE], 1);
#endif

	// NOTICE: The shadow update of IPI must be after this
	// The shadow update would not be stopped while SIF is disabled.
	// It'll trigger update whenever VSYNC comes.
	vio_hw_set_field(base_reg, &sif_regs[SIF_SETTING],
			&sif_fields[SW_SIF_ENABLE], 1);

	// FIXME: IAR Frame ID
	//usleep(30*1000);
	//sif_write_field(SW_SIF_IAR_MIPI_FRAME_ID_SET_EN, 0);

	vio_hw_set_reg(base_reg, &sif_regs[SIF_SHD_UP_RDY], 0xFFFFFFFF);

	// NOTICE: SIF Pattern Gen should be enabled after SIF_ENABLE
	// PG is behind the data gate.
	// To enable PG first will cause somedata inside data path, while SIF is stopped.

	//sif_start_pattern_gen(base_reg, 0);

#ifdef SIF_TEST_DROP_FRAME
	vio_info("Delay 0.5s - Start");
	usleep(500 * 1000);
	vio_info("Delay 0.5s - End");
	sif_enable_drop_frame();
	vio_info("Drop Frame - Start");
	sleep(3);
	sif_disable_drop_frame();
	vio_info("Drop Frame - End");
#endif

}

void sif_get_frameid_timestamps(u32 __iomem *base_reg, u32 mux,
			u32 ipi_index, struct frame_id *info, u32 dol_num)
{
	u32 value;
	u64 timestamp_l, timestamp_m;

	if (dol_num >= 2)
		value = vio_hw_get_reg(base_reg,
					&sif_regs[SIF_FRAME_ID_IPI_0_1 + ipi_index / 2]);
	else
		value = vio_hw_get_reg(base_reg,
					&sif_regs[SIF_FRAME_ID_IN_BUF_0_1 + mux / 2]);
	if(mux % 2 == 0)
		info->frame_id = value & 0xffff;
	else
		info->frame_id = value >> 16;

	timestamp_l = vio_hw_get_reg(base_reg,
						&sif_regs[SIF_TIMESTAMP0_LSB + mux * 2]);
	timestamp_m = vio_hw_get_reg(base_reg,
						&sif_regs[SIF_TIMESTAMP0_MSB + mux * 2]);
	info->timestamps = timestamp_l | timestamp_m << 32;

	vio_dbg("[mux%d]sif frame ID = %d\n", mux, info->frame_id);
}

u32 sif_get_current_bufindex(u32 __iomem *base_reg, u32 mux)
{
	u32 value = 0xffff;
	value = vio_hw_get_field(base_reg, &sif_regs[SIF_AXI_BUF_STATUS],
			&sif_fields[SIF_OUT_FRM0_BUF_IDX_REPORT - mux]);

	return value;
}

void sif_print_rx_status(u32 __iomem *base_reg, u32 err_status)
{
	int i = 0;
	int value = 0;

	for (i = 0; i < 13; i++) {
		if ((err_status & 1 << i) || (err_status & 1 << (i + 16))) {
			value = vio_hw_get_reg(base_reg, &sif_regs[SIF_MIPI_RX_STATUS0 + i]);
			vio_err("ipi%d rx status = 0x%x\n", i, value);
		}
	}
}
void sif_print_buffer_status(u32 __iomem *base_reg)
{
	int cur_index = 0;
	int value = 0;

	cur_index = vio_hw_get_reg(base_reg, &sif_regs[SIF_AXI_BUF_STATUS]);
	value = vio_hw_get_reg(base_reg, &sif_regs[SIF_AXI_BUS_OWNER]);

	vio_err("current buffer index = 0x%x, buffer owner = 0x%x\n\n",
			cur_index, value);
}

bool sif_get_wdma_enable(u32 __iomem *base_reg, u32 mux)
{
	/* if DMA_DISABLE field value is 1, this means dma output is disabled */
	if (vio_hw_get_field(base_reg, &sif_regs[SIF_OUT_FRM_CTRL],
				&sif_fields[SW_SIF_OUT_FRM0_W_ENABLE - mux]))
		return true;
	else
		return false;
}

void sif_set_wdma_buf_addr(u32 __iomem *base_reg, u32 mux_index,
				u32 number, u32 addr)
{
	vio_hw_set_reg(base_reg,
		&sif_regs[SIF_AXI_FRM0_W_ADDR0 + (mux_index * 4) + number], addr);
}

void sif_set_wdma_enable(u32 __iomem *base_reg, u32 mux_index, bool enable)
{
	vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_FRM_CTRL],
			&sif_fields[SW_SIF_OUT_FRM0_W_ENABLE - mux_index], enable);

}

void sif_set_rdma_enable(u32 __iomem *base_reg, u32 mux_index, bool enable)
{
	vio_hw_set_field(base_reg, &sif_regs[SIF_OUT_FRM_CTRL],
			&sif_fields[SW_SIF_OUT_FRM0_R_ENABLE - mux_index], enable);

}

void sif_enable_dma(u32 __iomem *base_reg, u32 cfg)
{
	vio_hw_set_reg(base_reg, &sif_regs[SIF_OUT_FRM_CTRL], cfg);
}

void sif_config_rdma_fmt(u32 __iomem *base_reg, u32 pix_length, u32 width,
				u32 height)
{
	u32 value = 0;

	value = pix_length << 29 | height << 16 | width;
	vio_hw_set_reg(base_reg, &sif_regs[SIF_OUT_BUF_ISP0_CFG], value);
}

void sif_set_rdma_buf_addr(u32 __iomem *base_reg, u32 index, u32 addr)
{
	vio_hw_set_reg(base_reg, &sif_regs[SIF_AXI_FRM0_R_ADDR + (index)], addr);
}

void sif_set_rdma_buf_stride(u32 __iomem *base_reg, u32 index, u32 stride)
{
	vio_hw_set_reg(base_reg, &sif_regs[SIF_AXI_FRM0_R_STRIDE + (index)],
				stride);
}

void sif_set_rdma_trigger(u32 __iomem *base_reg, u32 enable)
{
	vio_hw_set_reg(base_reg, &sif_regs[SIF_AXI_FRM_R1_START], enable);
}

int sif_get_irq_src(u32 __iomem *base_reg, struct sif_irq_src *src,
				bool clear)
{
	src->sif_frm_int = vio_hw_get_reg(base_reg, &sif_regs[SIF_FRM_INT]);
	src->sif_out_int = vio_hw_get_reg(base_reg, &sif_regs[SIF_OUT_INT]);

	if (clear) {
		vio_hw_set_reg(base_reg, &sif_regs[SIF_FRM_INT], src->sif_frm_int);
		vio_hw_set_reg(base_reg, &sif_regs[SIF_OUT_INT], src->sif_out_int);
	}

	src->sif_err_status = vio_hw_get_reg(base_reg, &sif_regs[SIF_ERR_STATUS]);
	src->sif_in_buf_overflow =
			vio_hw_get_reg(base_reg, &sif_regs[SIF_IN_BUF_OVERFLOW]);

	return 0;
}

void sif_set_isp_performance(u32 __iomem *base_reg, u8 value)
{
	vio_hw_set_reg(base_reg, &sif_regs[SIF_ISP_PERFORMANCE], value);
	vio_dbg("%s:%d\n", __func__, value);
}

void sif_hw_dump(u32 __iomem *base_reg)
{
	vio_hw_dump_regs(base_reg, sif_regs, NUM_OF_SIF_REG);
}
