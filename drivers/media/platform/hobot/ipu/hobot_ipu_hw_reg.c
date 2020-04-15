/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include "hobot_ipu_hw_reg.h"
#include "ipu_hw_api.h"

void ipu_set_input_img_size(void __iomem *base_addr, u8 shadow_index,
			    u32 width, u32 height)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_0_SRC_WIDTH], width);
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_0_SRC_HEIGHT], height);
		break;
	case SDW_ID_1:
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_1_SRC_WIDTH], width);
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_1_SRC_HEIGHT], height);
		break;
	case SDW_ID_2:
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_2_SRC_WIDTH], width);
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_2_SRC_HEIGHT], height);
		break;
	case SDW_ID_3:
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_3_SRC_WIDTH], width);
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_3_SRC_HEIGHT], height);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}
}

void ipu_set_rdma_stride(void __iomem *base_addr, u8 shadow_index,
			 u32 y_stride, u32 uv_stride)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_0_RD_DDR_STRIDE_LEN_Y],
			       y_stride);
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_0_RD_DDR_STRIDE_LEN_UV],
			       uv_stride);
		break;
	case SDW_ID_1:
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_1_RD_DDR_STRIDE_LEN_Y],
			       y_stride);
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_1_RD_DDR_STRIDE_LEN_UV],
			       uv_stride);
		break;
	case SDW_ID_2:
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_2_RD_DDR_STRIDE_LEN_Y],
			       y_stride);
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_2_RD_DDR_STRIDE_LEN_UV],
			       uv_stride);
		break;
	case SDW_ID_3:
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_3_RD_DDR_STRIDE_LEN_Y],
			       y_stride);
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_3_RD_DDR_STRIDE_LEN_UV],
			       uv_stride);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}
}

void ipu_set_us_wdma_stride(void __iomem *base_addr, u8 shadow_index,
			    u32 y_stride, u32 uv_stride)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_0_US_DDR_STRIDE_LEN_Y],
			       y_stride);
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_0_US_DDR_STRIDE_LEN_UV],
			       uv_stride);
		break;
	case SDW_ID_1:
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_1_US_DDR_STRIDE_LEN_Y],
			       y_stride);
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_1_US_DDR_STRIDE_LEN_UV],
			       uv_stride);
		break;
	case SDW_ID_2:
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_2_US_DDR_STRIDE_LEN_Y],
			       y_stride);
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_2_US_DDR_STRIDE_LEN_UV],
			       uv_stride);
		break;
	case SDW_ID_3:
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_3_US_DDR_STRIDE_LEN_Y],
			       y_stride);
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_3_US_DDR_STRIDE_LEN_UV],
			       uv_stride);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}

}

void ipu_set_ds_wdma_stride(void __iomem *base_addr, u8 shadow_index, u8 ds_ch,
			    u32 y_stride, u32 uv_stride)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_0_DS_0_DDR_STRIDE_LEN_Y +
					 2 * ds_ch], y_stride);
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_0_DS_0_DDR_STRIDE_LEN_UV +
					 2 * ds_ch], uv_stride);
		break;
	case SDW_ID_1:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_1_DS_0_DDR_STRIDE_LEN_Y +
					 2 * ds_ch], y_stride);
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_1_DS_0_DDR_STRIDE_LEN_UV +
					 2 * ds_ch], uv_stride);
		break;
	case SDW_ID_2:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_2_DS_0_DDR_STRIDE_LEN_Y +
					 2 * ds_ch], y_stride);
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_2_DS_0_DDR_STRIDE_LEN_UV +
					 2 * ds_ch], uv_stride);
		break;
	case SDW_ID_3:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_3_DS_0_DDR_STRIDE_LEN_Y +
					 2 * ds_ch], y_stride);
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_3_DS_0_DDR_STRIDE_LEN_UV +
					 2 * ds_ch], uv_stride);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}

}

void ipu_set_ds_roi_enable(void __iomem *base_addr, u8 shadow_index, u8 ds_ch,
			   u8 enable)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_0_DS_0_ROI_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_EN], enable);
		break;
	case SDW_ID_1:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_1_DS_0_ROI_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_EN], enable);
		break;
	case SDW_ID_2:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_2_DS_0_ROI_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_EN], enable);
		break;
	case SDW_ID_3:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_3_DS_0_ROI_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_EN], enable);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}

}

void ipu_set_ds_roi_rect(void __iomem *base_addr, u8 shadow_index, u8 ds_ch,
			 u16 start_x, u16 start_y, u16 width, u16 height)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_0_DS_0_ROI_SIZE + 2 * ds_ch],
				 &ipu_fields[IPU_F_US_ROI_HEIGHT], height);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_0_DS_0_ROI_SIZE + 2 * ds_ch],
				 &ipu_fields[IPU_F_US_ROI_WIDTH], width);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_0_DS_0_ROI_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_US_ROI_START_Y], start_y);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_0_DS_0_ROI_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_US_ROI_START_X], start_x);
		break;
	case SDW_ID_1:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_1_DS_0_ROI_SIZE + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_HEIGHT], height);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_1_DS_0_ROI_SIZE + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_WIDTH], width);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_1_DS_0_ROI_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_START_Y], start_y);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_1_DS_0_ROI_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_START_X], start_x);
		break;
	case SDW_ID_2:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_2_DS_0_ROI_SIZE + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_HEIGHT], height);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_2_DS_0_ROI_SIZE + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_WIDTH], width);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_2_DS_0_ROI_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_START_Y], start_y);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_2_DS_0_ROI_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_START_X], start_x);
		break;
	case SDW_ID_3:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_3_DS_0_ROI_SIZE + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_HEIGHT], height);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_3_DS_0_ROI_SIZE + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_WIDTH], width);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_3_DS_0_ROI_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_START_Y], start_y);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_3_DS_0_ROI_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_ROI_START_X], start_x);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}
}

void ipu_set_ds_enable(void __iomem *base_addr, u8 shadow_index, u8 ds_ch,
		       bool enable)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_0_DS_0_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_EN], enable);
		break;
	case SDW_ID_1:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_1_DS_0_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_EN], enable);
		break;
	case SDW_ID_2:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_2_DS_0_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_EN], enable);
		break;
	case SDW_ID_3:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_3_DS_0_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_EN], enable);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}
}

void ipu_set_ds2_wdma_enable(void __iomem *base_addr, u8 shadow_index,
			     u8 enable)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_0_DS_2_EN],
				 &ipu_fields[IPU_F_DS_2_DDR_ENABLE], enable);
		break;
	case SDW_ID_1:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_1_DS_2_EN],
				 &ipu_fields[IPU_F_DS_2_DDR_ENABLE], enable);
		break;
	case SDW_ID_2:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_2_DS_2_EN],
				 &ipu_fields[IPU_F_DS_2_DDR_ENABLE], enable);
		break;
	case SDW_ID_3:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_3_DS_2_EN],
				 &ipu_fields[IPU_F_DS_2_DDR_ENABLE], enable);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}

}

void ipu_set_ds_target_size(void __iomem *base_addr, u8 shadow_index, u8 ds_ch,
			    u16 tgt_width, u16 tgt_height)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_0_DS_0_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_TGT_HEIGHT],
				 tgt_height);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_0_DS_0_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_TGT_WIDTH], tgt_width);
		break;
	case SDW_ID_1:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_1_DS_0_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_TGT_HEIGHT],
				 tgt_height);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_1_DS_0_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_TGT_WIDTH], tgt_width);
		break;
	case SDW_ID_2:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_2_DS_0_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_TGT_HEIGHT],
				 tgt_height);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_2_DS_0_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_TGT_WIDTH], tgt_width);
		break;
	case SDW_ID_3:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_3_DS_0_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_TGT_HEIGHT],
				 tgt_height);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_3_DS_0_EN + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_TGT_WIDTH], tgt_width);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}

}

void ipu_set_ds_step(void __iomem *base_addr, u8 shadow_index, u8 ds_ch,
		     u16 step_x, u16 step_y, u8 pre_x, u8 pre_y)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_0_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_PRE_Y], pre_y);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_0_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_PRE_X], pre_x);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_0_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_STEP_Y], step_y);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_0_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_STEP_X], step_x);
		break;
	case SDW_ID_1:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_1_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_PRE_Y], pre_y);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_1_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_PRE_X], pre_x);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_1_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_STEP_Y], step_y);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_1_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_STEP_X], step_x);
		break;
	case SDW_ID_2:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_2_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_PRE_Y], pre_y);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_2_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_PRE_X], pre_x);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_2_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_STEP_Y], step_y);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_2_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_STEP_X], step_x);
		break;
	case SDW_ID_3:
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_3_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_PRE_Y], pre_y);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_3_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_PRE_X], pre_x);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_3_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_STEP_Y], step_y);
		vio_hw_set_field(base_addr,
				 &ipu_regs[IPU_3_DS_0_STEP + 2 * ds_ch],
				 &ipu_fields[IPU_F_DS_0_STEP_X], step_x);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}

}

void ipu_set_us_roi_enable(void __iomem *base_addr, u8 shadow_index, u8 enable)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_0_US_ROI_EN],
				 &ipu_fields[IPU_F_US_ROI_EN], enable);
		break;
	case SDW_ID_1:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_1_US_ROI_EN],
				 &ipu_fields[IPU_F_US_ROI_EN], enable);
		break;
	case SDW_ID_2:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_2_US_ROI_EN],
				 &ipu_fields[IPU_F_US_ROI_EN], enable);
		break;
	case SDW_ID_3:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_3_US_ROI_EN],
				 &ipu_fields[IPU_F_US_ROI_EN], enable);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}
}

void ipu_set_us_roi_rect(void __iomem *base_addr, u8 shadow_index, u16 start_x,
			 u16 start_y, u16 width, u16 height)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_0_US_ROI_EN],
				 &ipu_fields[IPU_F_US_ROI_START_Y], start_y);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_0_US_ROI_EN],
				 &ipu_fields[IPU_F_US_ROI_START_X], start_x);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_0_US_ROI_SIZE],
				 &ipu_fields[IPU_F_US_ROI_HEIGHT], height);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_0_US_ROI_SIZE],
				 &ipu_fields[IPU_F_US_ROI_WIDTH], width);
		break;
	case SDW_ID_1:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_1_US_ROI_EN],
				 &ipu_fields[IPU_F_US_ROI_START_Y], start_y);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_1_US_ROI_EN],
				 &ipu_fields[IPU_F_US_ROI_START_X], start_x);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_1_US_ROI_SIZE],
				 &ipu_fields[IPU_F_US_ROI_HEIGHT], height);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_1_US_ROI_SIZE],
				 &ipu_fields[IPU_F_US_ROI_WIDTH], width);
		break;
	case SDW_ID_2:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_2_US_ROI_EN],
				 &ipu_fields[IPU_F_US_ROI_START_Y], start_y);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_2_US_ROI_EN],
				 &ipu_fields[IPU_F_US_ROI_START_X], start_x);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_2_US_ROI_SIZE],
				 &ipu_fields[IPU_F_US_ROI_HEIGHT], height);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_2_US_ROI_SIZE],
				 &ipu_fields[IPU_F_US_ROI_WIDTH], width);
		break;
	case SDW_ID_3:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_3_US_ROI_EN],
				 &ipu_fields[IPU_F_US_ROI_START_Y], start_y);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_3_US_ROI_EN],
				 &ipu_fields[IPU_F_US_ROI_START_X], start_x);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_3_US_ROI_SIZE],
				 &ipu_fields[IPU_F_US_ROI_HEIGHT], height);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_3_US_ROI_SIZE],
				 &ipu_fields[IPU_F_US_ROI_WIDTH], width);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}

}

void ipu_set_us_enable(void __iomem *base_addr, u8 shadow_index, bool enable)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_0_US_EN],
				 &ipu_fields[IPU_F_US_EN], enable);
		break;
	case SDW_ID_1:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_1_US_EN],
				 &ipu_fields[IPU_F_US_EN], enable);
		break;
	case SDW_ID_2:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_2_US_EN],
				 &ipu_fields[IPU_F_US_EN], enable);
		break;
	case SDW_ID_3:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_3_US_EN],
				 &ipu_fields[IPU_F_US_EN], enable);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}
}

void ipu_set_us_target(void __iomem *base_addr, u8 shadow_index, u16 step_x,
		       u16 step_y, u16 tgt_width, u16 tgt_height)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_0_US_EN],
				 &ipu_fields[IPU_F_US_TGT_HEIGHT], tgt_height);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_0_US_EN],
				 &ipu_fields[IPU_F_US_TGT_WIDTH], tgt_width);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_0_US_STEP],
				 &ipu_fields[IPU_F_US_STEP_Y], step_y);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_0_US_STEP],
				 &ipu_fields[IPU_F_US_STEP_X], step_x);
		break;
	case SDW_ID_1:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_1_US_EN],
				 &ipu_fields[IPU_F_US_TGT_HEIGHT], tgt_height);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_1_US_EN],
				 &ipu_fields[IPU_F_US_TGT_WIDTH], tgt_width);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_1_US_STEP],
				 &ipu_fields[IPU_F_US_STEP_Y], step_y);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_1_US_STEP],
				 &ipu_fields[IPU_F_US_STEP_X], step_x);
		break;
	case SDW_ID_2:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_2_US_EN],
				 &ipu_fields[IPU_F_US_TGT_HEIGHT], tgt_height);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_2_US_EN],
				 &ipu_fields[IPU_F_US_TGT_WIDTH], tgt_width);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_2_US_STEP],
				 &ipu_fields[IPU_F_US_STEP_Y], step_y);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_2_US_STEP],
				 &ipu_fields[IPU_F_US_STEP_X], step_x);
		break;
	case SDW_ID_3:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_3_US_EN],
				 &ipu_fields[IPU_F_US_TGT_HEIGHT], tgt_height);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_3_US_EN],
				 &ipu_fields[IPU_F_US_TGT_WIDTH], tgt_width);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_3_US_STEP],
				 &ipu_fields[IPU_F_US_STEP_Y], step_y);
		vio_hw_set_field(base_addr, &ipu_regs[IPU_3_US_STEP],
				 &ipu_fields[IPU_F_US_STEP_X], step_x);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}

}

void ipu_set_osd_enable(void __iomem *base_addr, u8 shadow_index, u32 osd_num,
			u32 cfg)
{

	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_0_OSD_EN],
				 &ipu_fields[IPU_F_OSD_0_EN - osd_num], cfg);
		break;
	case SDW_ID_1:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_1_OSD_EN],
				 &ipu_fields[IPU_F_OSD_0_EN - osd_num], cfg);
		break;
	case SDW_ID_2:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_2_OSD_EN],
				 &ipu_fields[IPU_F_OSD_0_EN - osd_num], cfg);
		break;
	case SDW_ID_3:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_3_OSD_EN],
				 &ipu_fields[IPU_F_OSD_0_EN - osd_num], cfg);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}

}

void ipu_set_osd_overlay_mode(void __iomem *base_addr, u8 shadow_index,
			      u32 osd_num, u8 cfg)
{

	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_0_OSD_EN],
				 &ipu_fields[IPU_F_OSD_0_OVERLAY_MODE - osd_num],
				 cfg);
		break;
	case SDW_ID_1:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_1_OSD_EN],
				 &ipu_fields[IPU_F_OSD_0_OVERLAY_MODE - osd_num],
				 cfg);
		break;
	case SDW_ID_2:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_2_OSD_EN],
				 &ipu_fields[IPU_F_OSD_0_OVERLAY_MODE - osd_num],
				 cfg);
		break;
	case SDW_ID_3:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_3_OSD_EN],
				 &ipu_fields[IPU_F_OSD_0_OVERLAY_MODE - osd_num],
				 cfg);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}

}

void ipu_set_osd_roi(void __iomem *base_addr, u8 shadow_index, u32 osd_num,
		     u32 osd_layer, u16 start_x, u16 start_y, u16 width,
		     u16 height)
{
	int shift, start_xy, rect;

	shift = osd_num * 3 + osd_layer;
	start_xy = start_y << 12 | start_x;
	rect = height << 12 | width;
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_0_OSD_0_ROI_0_START + shift * 2],
			       start_xy);
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_0_OSD_0_ROI_0_SIZE + shift * 2],
			       rect);
		break;
	case SDW_ID_1:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_1_OSD_0_ROI_0_START + shift * 2],
			       start_xy);
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_1_OSD_0_ROI_0_SIZE + shift * 2],
			       rect);
		break;
	case SDW_ID_2:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_2_OSD_0_ROI_0_START + shift * 2],
			       start_xy);
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_2_OSD_0_ROI_0_SIZE + shift * 2],
			       rect);
		break;
	case SDW_ID_3:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_3_OSD_0_ROI_0_START + shift * 2],
			       start_xy);
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_3_OSD_0_ROI_0_SIZE + shift * 2],
			       rect);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}

}

void ipu_set_osd_sta_enable(void __iomem *base_addr, u8 shadow_index,
			    u32 osd_num, u32 cfg)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_0_OSD_STA_ENABLE],
				 &ipu_fields[IPU_F_OSD_0_STA_ENABLE - osd_num],
				 cfg);
		break;
	case SDW_ID_1:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_1_OSD_STA_ENABLE],
				 &ipu_fields[IPU_F_OSD_0_STA_ENABLE - osd_num],
				 cfg);
		break;
	case SDW_ID_2:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_2_OSD_STA_ENABLE],
				 &ipu_fields[IPU_F_OSD_0_STA_ENABLE - osd_num],
				 cfg);
		break;
	case SDW_ID_3:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_3_OSD_STA_ENABLE],
				 &ipu_fields[IPU_F_OSD_0_STA_ENABLE - osd_num],
				 cfg);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}
}

void ipu_set_osd_sta_roi(void __iomem *base_addr, u8 shadow_index, u32 osd_num,
			 u32 osd_layer, u16 start_x, u16 start_y, u16 width,
			 u16 height)
{
	int shift, start_xy, rect;

	shift = (osd_num * MAX_STA_NUM + osd_layer) * 2;
	start_xy = start_y << 12 | start_x;
	rect = height << 8 | width;
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_0_OSD_0_STA_ROI_0_START + shift], start_xy);
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_0_OSD_0_STA_ROI_0_SIZE + shift], rect);
		break;
	case SDW_ID_1:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_1_OSD_0_STA_ROI_0_START + shift], start_xy);
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_1_OSD_0_STA_ROI_0_SIZE + shift], rect);
		break;
	case SDW_ID_2:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_2_OSD_0_STA_ROI_0_START + shift], start_xy);
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_2_OSD_0_STA_ROI_0_SIZE + shift], rect);
		break;
	case SDW_ID_3:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_3_OSD_0_STA_ROI_0_START + shift], start_xy);
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_3_OSD_0_STA_ROI_0_SIZE + shift], rect);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}

}

void ipu_set_osd_sta_level(void __iomem *base_addr, u8 shadow_index,
			   u32 osd_num, u8 level)
{
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_0_OSD_STA_LEVEL],
				 &ipu_fields[IPU_F_OSD_STA_LEVEL_0 - osd_num],
				 level);
		break;
	case SDW_ID_1:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_1_OSD_STA_LEVEL],
				 &ipu_fields[IPU_F_OSD_STA_LEVEL_0 - osd_num],
				 level);
		break;
	case SDW_ID_2:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_2_OSD_STA_LEVEL],
				 &ipu_fields[IPU_F_OSD_STA_LEVEL_0 - osd_num],
				 level);
		break;
	case SDW_ID_3:
		vio_hw_set_field(base_addr, &ipu_regs[IPU_3_OSD_STA_LEVEL],
				 &ipu_fields[IPU_F_OSD_STA_LEVEL_0 - osd_num],
				 level);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}
}

void ipu_set_osd_addr(void __iomem *base_addr, u8 shadow_index, u32 osd_num,
		      u32 osd_layer, u32 addr)
{
	int shift;

	shift = osd_num * 3 + osd_layer;
	switch (shadow_index) {
	case SDW_ID_0:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_0_OSD_0_ROI_0_ADDR + shift], addr);
		break;
	case SDW_ID_1:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_1_OSD_0_ROI_0_ADDR + shift], addr);
		break;
	case SDW_ID_2:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_2_OSD_0_ROI_0_ADDR + shift], addr);
		break;
	case SDW_ID_3:
		vio_hw_set_reg(base_addr,
			       &ipu_regs[IPU_3_OSD_0_ROI_0_ADDR + shift], addr);
		break;
	default:
		vio_err("[%s]invalid shadow index(%d) for ipu\n", __func__, shadow_index);
		break;
	}

}

void ipu_set_shd_rdy(void __iomem *base_addr, u8 cfg)
{
	vio_hw_set_reg(base_addr, &ipu_regs[IPU_CFG_RDY], cfg);
}

u32 ipu_get_shd_rdy(void __iomem *base_addr)
{
	return vio_hw_get_reg(base_addr, &ipu_regs[IPU_CFG_RDY]);
}

u32 ipu_get_size_err(void __iomem *base_addr)
{
	return vio_hw_get_reg(base_addr, &ipu_regs[IPU_SIZE_ERR]);
}

void ipu_clear_size_err(void __iomem *base_addr, u8 value)
{
	vio_hw_set_field(base_addr, &ipu_regs[IPU_ERR_CLR],
		&ipu_fields[IPU_F_ERR_CLR], value);
}

void ipu_set_line_delay(void __iomem *base_addr, u8 value)
{
	vio_hw_set_field(base_addr, &ipu_regs[IPU_ERR_CLR],
		&ipu_fields[IPU_F_LINE_DELAY_SET], value);
}

void ipu_set_shd_select(void __iomem *base_addr, u8 cfg)
{
	vio_hw_set_reg(base_addr, &ipu_regs[IPU_CFG_SEL], cfg);
}

void ipu_set_osd_color(void __iomem *base_addr, u32 color_index, u32 color)
{
	vio_hw_set_reg(base_addr, &ipu_regs[IPU_OSD_COLOR_0 + color_index],
		       color);
}

/*
 00: sif,yuv422
 01: isp0,yuv420
 10: isp1,yuv420
 11: ddr,yuv420
*/
void ipu_src_select(void __iomem *base_addr, u8 cfg)
{
	vio_hw_set_field(base_addr, &ipu_regs[IPU_CFG],
			 &ipu_fields[IPU_F_SRC_SEL], cfg);
}

void ipu_set_frameid_enable(void __iomem *base_addr, u8 enable)
{
	vio_hw_set_field(base_addr, &ipu_regs[IPU_CFG],
			 &ipu_fields[IPU_F_FRAME_ID_EN], enable);
}

void ipu_set_frameid_value(void __iomem *base_addr, u16 value)
{
	vio_hw_set_field(base_addr, &ipu_regs[IPU_CFG],
			 &ipu_fields[IPU_F_FRAME_ID_SET], value);
}

void ipu_get_frame_id(void __iomem *base_addr, u32 * frameid)
{
	*frameid = vio_hw_get_reg(base_addr, &ipu_regs[IPU_FRAME_ID]);
}

void ipu_set_rdma_addr(void __iomem *base_addr, u32 y_addr, u32 cb_addr)
{
	vio_hw_set_reg(base_addr, &ipu_regs[IPU_RD_DDR_ADDR_Y], y_addr);
	vio_hw_set_reg(base_addr, &ipu_regs[IPU_RD_DDR_ADDR_UV], cb_addr);
}

void ipu_set_us_wdma_addr(void __iomem *base_addr, u32 y_addr, u32 cb_addr)
{
	vio_hw_set_reg(base_addr, &ipu_regs[IPU_US_DDR_Y], y_addr);
	vio_hw_set_reg(base_addr, &ipu_regs[IPU_US_DDR_UV], cb_addr);
	vio_dbg("ipu_set_us_wdma_addr : 0x%x, 0x%x\n", y_addr, cb_addr);
}

void ipu_set_ds_wdma_addr(void __iomem *base_addr, u8 ds_ch, u32 y_addr,
			  u32 cb_addr)
{
	vio_hw_set_reg(base_addr, &ipu_regs[IPU_DS_0_DDR_Y + 2 * ds_ch],
		       y_addr);
	vio_hw_set_reg(base_addr, &ipu_regs[IPU_DS_0_DDR_UV + 2 * ds_ch],
		       cb_addr);
	vio_dbg("ipu_set_ds%d_wdma_addr : 0x%x, 0x%x\n", ds_ch, y_addr, cb_addr);
}

void ipu_get_osd_sta_bin(void __iomem *base_addr, u8 osd_num, u8 osd_layer,
			 u16 * bin)
{
	int shift;

	if(osd_num < 3 && osd_layer < 8){
		shift = osd_num * 16 + osd_layer * 2;

		bin[0] = vio_hw_get_field(base_addr,
				     &ipu_regs[IPU_OSD_0_STA_0_BIN01 + shift],
				     &ipu_fields[IPU_F_OSD_STA_BIN_0_NUM]);
		bin[1] = vio_hw_get_field(base_addr,
				     &ipu_regs[IPU_OSD_0_STA_0_BIN01 + shift],
				     &ipu_fields[IPU_F_OSD_STA_BIN_1_NUM]);
		bin[2] = vio_hw_get_field(base_addr,
				     &ipu_regs[IPU_OSD_0_STA_0_BIN23 + shift],
				     &ipu_fields[IPU_F_OSD_STA_BIN_2_NUM]);
		bin[3] = vio_hw_get_field(base_addr,
				     &ipu_regs[IPU_OSD_0_STA_0_BIN23 + shift],
				     &ipu_fields[IPU_F_OSD_STA_BIN_3_NUM]);
	}else
		vio_err("invalid osd_num (%d) osd_layer (%d) for ipu\n", osd_num, osd_layer);
}

void ipu_set_rdma_start(void __iomem *base_addr)
{
	vio_hw_set_reg(base_addr, &ipu_regs[IPU_DDR_START], 1);
}

void ipu_get_intr_status(void __iomem *base_addr, u32 * status, bool clear)
{
	*status = vio_hw_get_reg(base_addr, &ipu_regs[IPU_INT_STATUS]);
	if (clear)
		vio_hw_set_reg(base_addr, &ipu_regs[IPU_INT_STATUS], *status);
}

void ipu_set_intr_mask(void __iomem *base_addr, u32 intr_mask)
{
	vio_hw_set_reg(base_addr, &ipu_regs[IPU_INT_MASK], intr_mask);
}

void ipu_set_ddr_fifo5(void __iomem *base_addr, u8 value)
{
	vio_hw_set_field(base_addr, &ipu_regs[IPU_WR_DDR_FIFO_THRED_1],
			&ipu_fields[IPU_F_WD_DDR_FIFO_THRED_5], value);
}

int ipu_get_err_status(void __iomem *base_addr)
{
	return vio_hw_get_reg(base_addr, &ipu_regs[IPU_ERR_STATUS]);
}

void ipu_hw_dump(u32 __iomem *base_reg)
{
	vio_hw_dump_regs(base_reg, ipu_regs, NUM_OF_IPU_REG);
}
