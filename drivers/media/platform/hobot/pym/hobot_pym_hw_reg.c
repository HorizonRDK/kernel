#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include "hobot_pym_hw_reg.h"
#include "pym_hw_api.h"

/*
1:ipu mode;
0:ddr mode;
*/
void pym_select_input_path(void __iomem *base_addr, u8 mode)
{
	vio_hw_set_field(base_addr, &pym_regs[PYM_PYRAMID_CTRL],
		&pym_fields[PYM_F_IMG_SRC], mode);	
}

void pym_enable_module(void __iomem *base_addr, bool enable)
{
	vio_hw_set_field(base_addr, &pym_regs[PYM_PYRAMID_CTRL],
		&pym_fields[PYM_F_ENABLE], enable);	
}

void pym_rdma_set_addr(void __iomem *base_addr, u32 y_addr, u32 uv_addr)
{
	vio_hw_set_reg(base_addr, &pym_regs[PYM_IMG_Y_ADDR_DDR], y_addr);
	vio_hw_set_reg(base_addr, &pym_regs[PYM_IMG_C_ADDR_DDR], uv_addr);
}

void pym_wdma_ds_set_addr(void __iomem *base_addr, u32 port_num, u32 y_addr, u32 uv_addr)
{
	vio_hw_set_reg(base_addr, &pym_regs[PYM_IMG_ADDR_Y + port_num], y_addr);
	vio_hw_set_reg(base_addr, &pym_regs[PYM_IMG_ADDR_C + port_num], uv_addr);
}


void pym_wdma_us_set_addr(void __iomem *base_addr, u32 port_num, u32 y_addr, u32 uv_addr)
{
	vio_hw_set_reg(base_addr, &pym_regs[PYM_IMG_Y_ADDR_U0 + port_num], y_addr);
	vio_hw_set_reg(base_addr, &pym_regs[PYM_IMG_C_ADDR_U0 + port_num], uv_addr);
}


void pym_get_size_err(void __iomem *base_addr)
{
	u32 v_err, h_err;
	v_err = vio_hw_get_field(base_addr, &pym_regs[PYM_ERR_CNT],
		&pym_fields[PYM_F_V_ERR]);
	h_err = vio_hw_get_field(base_addr, &pym_regs[PYM_ERR_CNT],
		&pym_fields[PYM_F_H_ERR]);	
}

void pym_set_axi_id(void __iomem *base_addr, u32 axi_id)
{
	vio_hw_set_reg(base_addr, &pym_regs[PYM_CFG], axi_id);
}

void pym_set_wline(void __iomem *base_addr, u16 y_line, u16 uv_line)
{
	vio_hw_set_field(base_addr, &pym_regs[PYM_WR_LINE],
		&pym_fields[PYM_F_WR_LINE_Y], y_line);
	vio_hw_set_field(base_addr, &pym_regs[PYM_WR_LINE],
		&pym_fields[PYM_F_WR_LINE_UV], uv_line);
}

void pym_set_frame_id(void __iomem *base_addr, u32 frame_id)
{
	vio_hw_set_reg(base_addr, &pym_regs[PYM_FRAME_ID], frame_id);
}

int pym_get_frame_id(void __iomem *base_addr)
{
	int frame_id = 0;

	frame_id = vio_hw_get_reg(base_addr, &pym_regs[PYM_FRAME_ID_VALUE]);

	return frame_id;
}

void pym_get_intr_status(void __iomem *base_addr, u32 *status, bool clear)
{
	*status = vio_hw_get_reg(base_addr, &pym_regs[PYM_INT_STATUS]);
	if(clear)
		vio_hw_set_reg(base_addr, &pym_regs[PYM_INT_STATUS], *status);
}

void pym_set_intr_mask(void __iomem *base_addr, u32 intr_mask)
{
	vio_hw_set_reg(base_addr, &pym_regs[PYM_INT_MASK], intr_mask);
}

void pym_set_rdma_start(void __iomem *base_addr)
{
	vio_hw_set_reg(base_addr, &pym_regs[PYM_DDR_START], 1);
}

void pym_set_common_rdy(void __iomem *base_addr, bool enable)
{
	vio_hw_set_reg(base_addr, &pym_regs[PYM_CONFIG_RDY], enable);
}

void pym_set_shd_select(void __iomem *base_addr, u8 cfg)
{
	vio_hw_set_reg(base_addr, &pym_regs[PYM_CONFIG_ID], cfg);
}

void pym_ds_uv_bypass(void __iomem *base_addr, u8 shadow_index, u32 port_num)
{	
	u32 shift = 0;

	if(port_num < 4)
		shift = port_num - 1;
	else if(port_num < 8)
		shift = port_num - 2;
	else if(port_num < 12)
		shift = port_num - 3;
	else if(port_num < 16)
		shift = port_num - 4;
	else if(port_num < 20)
		shift = port_num - 5;
	else if(port_num < 24)
		shift = port_num - 6;
	else
		vio_err("wrong port number(%d)", port_num);

	switch(shadow_index){
		case SDW_ID_0:
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_PYRAMIDE_DS_CTRL], 
				&pym_fields[PYM_F_DS_UV_BYPASS], 1 << shift);
			break;
		case SDW_ID_1:
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_PYRAMIDE_DS_CTRL], 
				&pym_fields[PYM_F_DS_UV_BYPASS], 1 << shift);
			break;
		case SDW_ID_2:
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_PYRAMIDE_DS_CTRL], 
				&pym_fields[PYM_F_DS_UV_BYPASS], 1 << shift);
			break;
		case SDW_ID_3:
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_PYRAMIDE_DS_CTRL], 
				&pym_fields[PYM_F_DS_UV_BYPASS], 1 << shift);
			break;
		default:
			vio_err("invalid shadow index(%d) for ipu\n",shadow_index);
			break;
		}

}

void pym_ds_enabe_base_layer(void __iomem *base_addr, u8 shadow_index, u32 layer_nums)
{
	u32 cfg = 0;

	cfg = (1 << (layer_nums - 1))  - 1;
	switch(shadow_index){
		case SDW_ID_0:
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_PYRAMIDE_DS_CTRL], 
				&pym_fields[PYM_F_DS_LAYER_EN], cfg);
			break;
		case SDW_ID_1:
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_PYRAMIDE_DS_CTRL], 
				&pym_fields[PYM_F_DS_LAYER_EN], cfg);
			break;
		case SDW_ID_2:
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_PYRAMIDE_DS_CTRL], 
				&pym_fields[PYM_F_DS_LAYER_EN], cfg);
			break;
		case SDW_ID_3:
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_PYRAMIDE_DS_CTRL], 
				&pym_fields[PYM_F_DS_LAYER_EN], cfg);
			break;
		default:
			vio_err("invalid shadow index(%d) for pym\n", shadow_index);
			break;
		}
}

void pym_us_clk_gate(void __iomem *base_addr, u8 shadow_index, bool uv_gate, bool clk_gate)
{
	switch(shadow_index){
		case SDW_ID_0:
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_UV_CLK_GATE_EN], uv_gate);
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_US_CLK_GATE_EN], clk_gate);
			break;
		case SDW_ID_1:
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_UV_CLK_GATE_EN], uv_gate);
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_US_CLK_GATE_EN], clk_gate);
			break;
		case SDW_ID_2:
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_UV_CLK_GATE_EN], uv_gate);
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_US_CLK_GATE_EN], clk_gate);
			break;
		case SDW_ID_3:
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_UV_CLK_GATE_EN], uv_gate);
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_US_CLK_GATE_EN], clk_gate);
			break;
		default:
			vio_err("invalid shadow index(%d) for ipu\n", shadow_index);
			break;
		}
}


void pym_us_uv_bypass(void __iomem *base_addr, u8 shadow_index, u32 layer_index)
{
	switch(shadow_index){
		case SDW_ID_0:
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_US_UV_BYPASS], 1 << layer_index);
			break;
		case SDW_ID_1:
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_US_UV_BYPASS], 1 << layer_index);
			break;
		case SDW_ID_2:
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_US_UV_BYPASS], 1 << layer_index);
			break;
		case SDW_ID_3:
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_US_UV_BYPASS], 1 << layer_index);
			break;
		default:
			vio_err("invalid shadow index(%d) for ipu\n", shadow_index);
			break;
		}
}

void pym_us_enabe_layer(void __iomem *base_addr, u8 shadow_index, u32 layer_index)
{
	switch(shadow_index){
		case SDW_ID_0:
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_US_LAYER_EN], 1 << layer_index);
			break;
		case SDW_ID_1:
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_US_LAYER_EN], 1 << layer_index);
			break;
		case SDW_ID_2:
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_US_LAYER_EN], 1 << layer_index);
			break;
		case SDW_ID_3:
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_PYRAMIDE_US_CTRL], 
				&pym_fields[PYM_F_US_LAYER_EN], 1 << layer_index);
			break;
		default:
			vio_err("invalid shadow index(%d) for ipu\n", shadow_index);
			break;
		}
}

void pym_config_src_size(void __iomem *base_addr, u8 shadow_index, u32 src_w, u32 src_h)
{
	switch(shadow_index){
		case SDW_ID_0:
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_IMG_RES_SRC], 
				&pym_fields[PYM_F_IMG_SRC_W], src_w);
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_IMG_RES_SRC], 
				&pym_fields[PYM_F_IMG_SRC_H], src_h);
			break;
		case SDW_ID_1:
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_IMG_RES_SRC], 
				&pym_fields[PYM_F_IMG_SRC_W], src_w);
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_IMG_RES_SRC], 
				&pym_fields[PYM_F_IMG_SRC_H], src_h);
			break;
		case SDW_ID_2:
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_IMG_RES_SRC], 
				&pym_fields[PYM_F_IMG_SRC_W], src_w);
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_IMG_RES_SRC], 
				&pym_fields[PYM_F_IMG_SRC_H], src_h);
			break;
		case SDW_ID_3:
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_IMG_RES_SRC], 
				&pym_fields[PYM_F_IMG_SRC_W], src_w);
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_IMG_RES_SRC], 
				&pym_fields[PYM_F_IMG_SRC_H], src_h);
			break;
		default:
			vio_err("invalid shadow index(%d) for ipu\n", shadow_index);
			break;
		}

}

void pym_ds_config_factor(void __iomem *base_addr, u8 shadow_index, u32 port_num, u32 factor)
{
	u32 reg_index = 0, shift = 0;

	if ( port_num % 4 == 0)
		return;

	if(port_num < 4)
		shift = port_num - 1;
	else if(port_num < 8)
		shift = port_num - 2;
	else if(port_num < 12)
		shift = port_num - 3;
	else if(port_num < 16)
		shift = port_num - 4;
	else if(port_num < 20)
		shift = port_num - 5;
	else if(port_num < 24)
		shift = port_num - 6;
	else
		vio_err("wrong port number(%d)", port_num);

	reg_index = shift/5;

	switch(shadow_index){
		case SDW_ID_0:
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_DS_FACTOR_1 + reg_index], 
				&pym_fields[PYM_F_DS_FACTOR_P1 + shift], factor);
			break;
		case SDW_ID_1:
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_DS_FACTOR_1 + reg_index], 
				&pym_fields[PYM_F_DS_FACTOR_P1 + shift], factor);
			break;
		case SDW_ID_2:
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_DS_FACTOR_1 + reg_index], 
				&pym_fields[PYM_F_DS_FACTOR_P1 + shift], factor);
			break;
		case SDW_ID_3:
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_DS_FACTOR_1 + reg_index], 
				&pym_fields[PYM_F_DS_FACTOR_P1 + shift], factor);
			break;
		default:
			vio_err("invalid shadow index(%d) for ipu\n", shadow_index);
			break;
		}

}

void pym_ds_config_roi(void __iomem *base_addr, u8 shadow_index, u32 port_num, struct roi_rect *rect)
{
	u32 shift = 0;

	if ( port_num % 4 == 0)
		return;

	if(port_num < 4)
		shift = port_num - 1;
	else if(port_num < 8)
		shift = port_num - 2;
	else if(port_num < 12)
		shift = port_num - 3;
	else if(port_num < 16)
		shift = port_num - 4;
	else if(port_num < 20)
		shift = port_num - 5;
	else if(port_num < 24)
		shift = port_num - 6;
	else
		vio_err("wrong port number(%d)", port_num);

	switch(shadow_index){
		case SDW_ID_0:
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_ROI0_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_LEFT_P1 + shift * 4], rect->roi_x);
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_ROI0_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_TOP_P1 + shift * 4], rect->roi_y);
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_ROI1_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_WIDTH_P1 + shift * 4], rect->roi_width);
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_ROI1_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_HEIGHT_P1 + shift * 4], rect->roi_height);
			break;
		case SDW_ID_1:
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_ROI0_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_LEFT_P1 + shift * 4], rect->roi_x);
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_ROI0_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_TOP_P1 + shift * 4], rect->roi_y);
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_ROI1_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_WIDTH_P1 + shift * 4], rect->roi_width);
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_ROI1_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_HEIGHT_P1 + shift * 4], rect->roi_height);
			break;
		case SDW_ID_2:
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_ROI0_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_LEFT_P1 + shift * 4], rect->roi_x);
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_ROI0_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_TOP_P1 + shift * 4], rect->roi_y);
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_ROI1_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_WIDTH_P1 + shift * 4], rect->roi_width);
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_ROI1_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_HEIGHT_P1 + shift * 4], rect->roi_height);
			break;
		case SDW_ID_3:
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_ROI0_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_LEFT_P1 + shift * 4], rect->roi_x);
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_ROI0_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_TOP_P1 + shift * 4], rect->roi_y);
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_ROI1_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_WIDTH_P1 + shift * 4], rect->roi_width);
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_ROI1_P1 + shift*2], 
				&pym_fields[PYM_F_ROI_HEIGHT_P1 + shift * 4], rect->roi_height);
			break;
		default:
			vio_err("invalid shadow index(%d) for ipu\n", shadow_index);
			break;
		}

}

void pym_us_config_roi(void __iomem *base_addr, u8 shadow_index, u32 layer_index, struct roi_rect *rect)
{

	switch(shadow_index){
		case SDW_ID_0:
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_ROI0_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_LEFT_U0 + layer_index * 4], rect->roi_x);
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_ROI0_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_TOP_U0 + layer_index * 4], rect->roi_y);
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_ROI1_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_WIDTH_U0 + layer_index * 4], rect->roi_width);
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_ROI1_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_HEIGHT_U0 + layer_index * 4], rect->roi_height);
			break;
		case SDW_ID_1:
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_ROI0_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_LEFT_U0 + layer_index * 4], rect->roi_x);
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_ROI0_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_TOP_U0 + layer_index * 4], rect->roi_y);
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_ROI1_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_WIDTH_U0 + layer_index * 4], rect->roi_width);
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_ROI1_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_HEIGHT_U0 + layer_index * 4], rect->roi_height);
			break;
		case SDW_ID_2:
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_ROI0_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_LEFT_U0 + layer_index * 4], rect->roi_x);
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_ROI0_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_TOP_U0 + layer_index * 4], rect->roi_y);
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_ROI1_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_WIDTH_U0 + layer_index * 4], rect->roi_width);
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_ROI1_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_HEIGHT_U0 + layer_index * 4], rect->roi_height);
			break;
		case SDW_ID_3:
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_ROI0_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_LEFT_U0 + layer_index * 4], rect->roi_x);
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_ROI0_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_TOP_U0 + layer_index * 4], rect->roi_y);
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_ROI1_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_WIDTH_U0 + layer_index * 4], rect->roi_width);
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_ROI1_U0 + layer_index*2], 
				&pym_fields[PYM_F_ROI_HEIGHT_U0 + layer_index * 4], rect->roi_height);
			break;
		default:
			vio_err("invalid shadow index(%d) for ipu\n", shadow_index);
			break;
		}
}

void pym_us_config_factor(void __iomem *base_addr, u8 shadow_index, u32 layer_index, u32 factor)
{
	u32 reg_index;

	reg_index = layer_index / 4;

	switch(shadow_index){
		case SDW_ID_0:
			vio_hw_set_field(base_addr, &pym_regs[PYM_0_US_FACTOR_1 + reg_index], 
				&pym_fields[PYM_F_US_FACTOR_U0 + layer_index], factor);
			break;
		case SDW_ID_1:
			vio_hw_set_field(base_addr, &pym_regs[PYM_1_US_FACTOR_1 + reg_index], 
				&pym_fields[PYM_F_US_FACTOR_U0 + layer_index], factor);
			break;
		case SDW_ID_2:
			vio_hw_set_field(base_addr, &pym_regs[PYM_2_US_FACTOR_1 + reg_index], 
				&pym_fields[PYM_F_US_FACTOR_U0 + layer_index], factor);
			break;
		case SDW_ID_3:
			vio_hw_set_field(base_addr, &pym_regs[PYM_3_US_FACTOR_1 + reg_index], 
				&pym_fields[PYM_F_US_FACTOR_U0 + layer_index], factor);
			break;
		default:
			vio_err("invalid shadow index(%d) for ipu\n", shadow_index);
			break;
		}
}

void pym_ds_set_src_width(void __iomem *base_addr, u8 shadow_index, u32 port_num, u16 width)
{
	u32 shift = 0, reg_index = 0;

	if ( port_num % 4 == 0)
		return;

	if(port_num < 4)
		shift = port_num - 1;
	else if(port_num < 8)
		shift = port_num - 2;
	else if(port_num < 12)
		shift = port_num - 3;
	else if(port_num < 16)
		shift = port_num - 4;
	else if(port_num < 20)
		shift = port_num - 5;
	else if(port_num < 24)
		shift = port_num - 6;
	else
		vio_err("wrong port number(%d)", port_num);

	reg_index = shift / 2;

	switch(shadow_index){
		case SDW_ID_0:
			if(shift % 2 == 0)
				vio_hw_set_field(base_addr, &pym_regs[PYM_0_SRC_WIDTH_DS1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_P1 + 2 * reg_index], width);
			else
				vio_hw_set_field(base_addr, &pym_regs[PYM_0_SRC_WIDTH_DS1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_P2 + 2 * reg_index], width);
			break;
		case SDW_ID_1:
			if(shift % 2 == 0)
				vio_hw_set_field(base_addr, &pym_regs[PYM_1_SRC_WIDTH_DS1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_P1 + 2 * reg_index], width);
			else
				vio_hw_set_field(base_addr, &pym_regs[PYM_1_SRC_WIDTH_DS1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_P2 + 2 * reg_index], width);
			break;
		case SDW_ID_2:
			if(shift % 2 == 0)
				vio_hw_set_field(base_addr, &pym_regs[PYM_2_SRC_WIDTH_DS1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_P1 + 2 * reg_index], width);
			else
				vio_hw_set_field(base_addr, &pym_regs[PYM_2_SRC_WIDTH_DS1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_P2 + 2 * reg_index], width);
			break;
		case SDW_ID_3:
			if(shift % 2 == 0)
				vio_hw_set_field(base_addr, &pym_regs[PYM_3_SRC_WIDTH_DS1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_P1 + 2 * reg_index], width);
			else
				vio_hw_set_field(base_addr, &pym_regs[PYM_3_SRC_WIDTH_DS1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_P2 + 2 * reg_index], width);
			break;
		default:
			vio_err("invalid shadow index(%d) for ipu\n", shadow_index);
			break;
		}
}


void pym_us_set_src_width(void __iomem *base_addr, u8 shadow_index, u32 layer_index, u16 width)
{
	u32 reg_index = 0;

	reg_index = layer_index / 2;

	switch(shadow_index){
		case SDW_ID_0:
			if(layer_index % 2 == 0)
				vio_hw_set_field(base_addr, &pym_regs[PYM_0_SRC_WIDTH_US1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_U0 + reg_index * 2], width);
			else
				vio_hw_set_field(base_addr, &pym_regs[PYM_0_SRC_WIDTH_US1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_U1 + reg_index * 2], width);
			break;
		case SDW_ID_1:
			if(layer_index % 2 == 0)
				vio_hw_set_field(base_addr, &pym_regs[PYM_1_SRC_WIDTH_US1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_U0 + reg_index * 2], width);
			else
				vio_hw_set_field(base_addr, &pym_regs[PYM_1_SRC_WIDTH_US1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_U1 + reg_index * 2], width);
			break;
		case SDW_ID_2:
			if(layer_index % 2 == 0)
				vio_hw_set_field(base_addr, &pym_regs[PYM_2_SRC_WIDTH_US1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_U0 + reg_index * 2], width);
			else
				vio_hw_set_field(base_addr, &pym_regs[PYM_2_SRC_WIDTH_US1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_U1 + reg_index * 2], width);
			break;
		case SDW_ID_3:
			if(layer_index % 2 == 0)
				vio_hw_set_field(base_addr, &pym_regs[PYM_3_SRC_WIDTH_US1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_U0 + reg_index * 2], width);
			else
				vio_hw_set_field(base_addr, &pym_regs[PYM_3_SRC_WIDTH_US1 + reg_index], 
					&pym_fields[PYM_F_SRC_WIDTH_U1 + reg_index * 2], width);
			break;
		default:
			vio_err("invalid shadow index(%d) for ipu\n", shadow_index);
			break;
		}

}

void pym_set_shd_rdy(void __iomem *base_addr, u8 shadow_index, bool enable)
{
	switch(shadow_index){
		case SDW_ID_0:
			vio_hw_set_reg(base_addr, &pym_regs[PYM_0_CONFIG_RDY], enable);
			break;
		case SDW_ID_1:
			vio_hw_set_reg(base_addr, &pym_regs[PYM_1_CONFIG_RDY], enable);
			break;
		case SDW_ID_2:
			vio_hw_set_reg(base_addr, &pym_regs[PYM_2_CONFIG_RDY], enable);
			break;
		case SDW_ID_3:
			vio_hw_set_reg(base_addr, &pym_regs[PYM_3_CONFIG_RDY], enable);
			break;
		default:
			vio_err("invalid shadow index(%d) for ipu\n", shadow_index);
			break;
		}
}

void pym_hw_dump(u32 __iomem *base_reg)
{
	vio_hw_dump_regs(base_reg, pym_regs, NUM_OF_PYM_REG);
}

