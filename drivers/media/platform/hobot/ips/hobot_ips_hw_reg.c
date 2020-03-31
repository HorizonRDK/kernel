/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/delay.h>
#include "hobot_ips_hw_reg.h"
#include "ips_hw_api.h"

void ips_module_reset(void __iomem *base_addr, u32 module)
{
	int field_index = 0;

	switch(module){
		case SIF_RST:
			field_index = IPS_F_RSTN_SIF;
			break;
		case DWE1_RST:
			field_index = IPS_F_RSTN_DWE1;
			break;
		case DWE0_RST:
			field_index = IPS_F_RSTN_DWE0;
			break;
		case IPU0_RST:
			field_index = IPS_F_RSTN_IPU0;
			break;
		case ISP0_RST:
			field_index = IPS_F_RSTN_ISP0;
			break;
		case IRAM_RST:
			field_index = IPS_F_RSTN_IRAM;
			break;
		case IPU_PYM_RST:
			field_index = IPS_F_RSTN_IPU_PYM;
			break;
		case PYM_RST:
			field_index = IPS_F_RSTN_PYM;
			break;
		default:
			vio_err("wrong reset module(%d)\n", module);
			break;
	}

	vio_hw_set_field(base_addr, &ips_regs[IPS_GENERAL_REG],
		&ips_fields[field_index], 0);
	mdelay(1);
	vio_hw_set_field(base_addr, &ips_regs[IPS_GENERAL_REG],
		&ips_fields[field_index], 1);
}

void ips_set_intr_mask(void __iomem *base_addr, u32 cfg)
{
	vio_hw_set_reg(base_addr, &ips_regs[IPS_INT_ENABLE], cfg);
}

void ips_enable_intr(void __iomem *base_addr, u32 module, bool enable)
{
	int field_index = 0;

	switch(module){
		case MOD_INTR:
			field_index = IPS_F_MD_INT_EN;
			break;
		case IRAM_INTR:
			field_index = IPS_F_IRAM_INT_EN;
			break;
		case AXI1_INTR:
			field_index = IPS_F_AXI1_INT_EN;
			break;
		case AXI0_INTR:
			field_index = IPS_F_AXI0_INT_EN;
			break;
		default:
			vio_err("wrong reset module(%d)\n", module);
			break;
	}

	vio_hw_set_field(base_addr, &ips_regs[IPS_INT_ENABLE],
		&ips_fields[field_index], enable);	
}

void ips_get_intr_status(void __iomem *base_addr, u32 module, u32 *status,
			bool clear)
{
	int field_index = 0;

	switch(module){
		case MOD_INTR:
			field_index = IPS_F_MD_INT_EN;
			break;
		case IRAM_INTR:
			field_index = IPS_F_IRAM_INT_EN;
			break;
		case AXI1_INTR:
			field_index = IPS_F_AXI1_INT_EN;
			break;
		case AXI0_INTR:
			field_index = IPS_F_AXI0_INT_EN;
			break;
			break;
		default:
			vio_err("wrong reset module(%d)\n", module);
			break;
	}

	*status = vio_hw_get_field(base_addr, &ips_regs[IPS_INT_STATUS],
					&ips_fields[field_index]);
	if (clear)
		vio_hw_set_field(base_addr, &ips_regs[IPS_INT_STATUS],
				&ips_fields[field_index], *status);
}

int ips_clk_ctrl(void __iomem *base_addr, u32 module, bool enable)
{
	int field_index = 0;
	switch(module){
		case IRAM_CLOCK_GATE:
			field_index = IPS_F_IRAM_CG_EN;
			break;
		case MD_CLOCK_GATE:
			field_index = IPS_F_MD_CG_EN;
			break;
		case IPU0_CLOCK_GATE:
			field_index = IPS_F_IPU0_CG_EN;
			break;
		case LDC1_CLOCK_GATE:
			field_index = IPS_F_LDC1_CG_EN;
			break;
		case LDC0_CLOCK_GATE:
			field_index = IPS_F_LDC0_CG_EN;
			break;
		case T2L1_CLOCK_GATE:
			field_index = IPS_F_T2L1_CG_EN;
			break;
		case T2L0_CLOCK_GATE:
			field_index = IPS_F_T2L0_CG_EN;
			break;
		case GDC1_CLOCK_GATE:
			field_index = IPS_F_GDC1_CG_EN;
			break;
		case GDC0_CLOCK_GATE:
			field_index = IPS_F_GDC0_CG_EN;
			break;
		case DWE1_CLOCK_GATE:
			field_index = IPS_F_DWE1_CG_EN;
			break;
		case DWE0_CLOCK_GATE:
			field_index = IPS_F_DWE0_CG_EN;
			break;
		case ISP0_CLOCK_GATE:
			field_index = IPS_F_ISP0_CG_EN;
			break;
		case SIF_CLOCK_GATE:
			field_index = IPS_F_SIF_CG_EN;
			break;
		default:
			vio_err("wrong reset module(%d)\n", module);
			return -1;
			break;
	}

	vio_hw_set_field(base_addr, &ips_regs[IPS_CLK_CTRL],
		&ips_fields[field_index], enable);

	return 0;
}

void ips_mot_set_roi(void __iomem *base_addr, struct roi_rect *rect)
{
	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_ROT_LT],
		&ips_fields[IPS_F_MOT_DET_ROI_TOP], rect->roi_y);
	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_ROT_LT],
		&ips_fields[IPS_F_MOT_DET_ROI_LEFT], rect->roi_x);	

	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_ROT_WH],
		&ips_fields[IPS_F_MOT_DET_ROI_HEIGHT], rect->roi_height);
	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_ROT_WH],
		&ips_fields[IPS_F_MOT_DET_ROI_WIDTH], rect->roi_width);	

}

void ips_mot_set_diff_thd(void __iomem *base_addr, u32 diff_thresh)
{
	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_PARA1],
		&ips_fields[IPS_F_MOT_DET_DIFF_THRESH], diff_thresh);
}

void ips_mot_set_thresh(void __iomem *base_addr, u32 thresh)
{
	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_PARA1],
		&ips_fields[IPS_F_MOT_DET_THRESH], thresh);
}

void ips_mot_set_step(void __iomem *base_addr, u32 step)
{
	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_PARA1],
		&ips_fields[IPS_F_MOT_DET_STEP], step);
}

void ips_mot_set_fmt(void __iomem *base_addr, u32 fmt, u32 pixel_lenght)
{
	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_PARA2],
		&ips_fields[IPS_F_MOT_DET_IMG_FMT], fmt);
	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_PARA2],
		&ips_fields[IPS_F_MOT_DET_PIX_WIDTH], pixel_lenght);
}

void ips_mot_data_sel(void __iomem *base_addr, u8 data_sel)
{
	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_PARA2],
		&ips_fields[IPS_F_MOT_DET_DATA_SEL], data_sel);
}

void ips_mot_set_refresh(void __iomem *base_addr, bool enable)
{
	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_PARA2],
		&ips_fields[IPS_F_MOT_DET_REFRESH], enable);
}

void ips_mot_set_prec(void __iomem *base_addr, u32 prec, u32 decay)
{
	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_PARA2],
		&ips_fields[IPS_F_MOT_DET_DEC_PREC], prec);
	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_PARA2],
		&ips_fields[IPS_F_MOT_DET_WGT_DECAY], decay);
}

void ips_mot_enable(void __iomem *base_addr, bool enable)
{
	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_PARA2],
		&ips_fields[IPS_F_MOT_DET_EN], enable);
}

void ips_mot_set_resolution(void __iomem *base_addr, u32 width, u32 height)
{
	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_IMG],
		&ips_fields[IPS_F_MOT_DET_IMG_WIDTH], width);
	vio_hw_set_field(base_addr, &ips_regs[MOT_DET_IMG],
		&ips_fields[IPS_F_MOT_DET_IMG_HEIGHT], height);
}

u32 ipu_get_axi_statue(void __iomem *base_addr)
{
	u32 status = 0;

	status = vio_hw_get_reg(base_addr, &ips_regs[AXI_BUS_STATUS]);
	return status;
}

void ips_set_axi_bus_ctrl(void __iomem *base_addr, u32 cfg)
{
	vio_hw_set_reg(base_addr, &ips_regs[AXI_BUS_CTRL], cfg);
}

u32 ips_get_axi_bus_ctrl(void __iomem *base_addr)
{
	u32 status = 0;

	status = vio_hw_get_reg(base_addr, &ips_regs[AXI_BUS_CTRL]);
	return status;
}

void ips_enable_isp0_intr(void __iomem *base_addr, bool enable)
{
	vio_hw_set_reg(base_addr, &ips_regs[ISP0_INT_EN], enable);
}

u16 ips_get_isp_frame_id(void __iomem *base_addr)
{	
	u16 frame_id = 0;

	frame_id = vio_hw_get_reg(base_addr, &ips_regs[ISP0_FRAME_ID]);

	return frame_id;
}

void isp_vcke_ctrl(void __iomem *base_addr, bool en)
{
	vio_hw_set_reg(base_addr, &ips_regs[ISP0_VCKE_CTRL], en);
}

void isp_vcke_th0(void __iomem *base_addr, u32 cfg)
{
	vio_hw_set_reg(base_addr, &ips_regs[ISP0_VCKE_TH0], cfg);
}

void isp_vcke_th1(void __iomem *base_addr, u32 cfg)
{
	vio_hw_set_reg(base_addr, &ips_regs[ISP0_VCKE_TH1], cfg);
}

u32 ips_get_bw_cnt(void __iomem *base_addr, u32 ch)
{	
	u32 count = 0;

	count = vio_hw_get_reg(base_addr, &ips_regs[IPS_BW_CNT_CH0 + ch]);
	return count;
}

void ips_hw_dump(u32 __iomem *base_reg)
{
	vio_hw_dump_regs(base_reg, ips_regs, NUM_OF_IPS_REG);
}
