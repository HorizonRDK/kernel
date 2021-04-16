/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_PYM_HW_API__
#define __HOBOT_PYM_HW_API__

void pym_select_input_path(void __iomem *base_addr, u8 mode);
void pym_enable_module(void __iomem *base_addr, bool enable);
void pym_rdma_set_addr(void __iomem *base_addr, u32 y_addr, u32 uv_addr);
void pym_wdma_ds_set_addr(void __iomem *base_addr, u32 port_num, u32 y_addr, u32 uv_addr);
void pym_wdma_us_set_addr(void __iomem *base_addr, u32 port_num, u32 y_addr, u32 uv_addr);
void pym_get_size_err(void __iomem *base_addr);
void pym_set_axi_id(void __iomem *base_addr, u32 axi_id);
void pym_set_wline(void __iomem *base_addr, u16 y_line, u16 uv_line);
void pym_set_frame_id(void __iomem *base_addr, u32 frame_id);
int pym_get_frame_id(void __iomem *base_addr);
void pym_get_intr_status(void __iomem *base_addr, u32 *status, bool clear);
void pym_set_intr_mask(void __iomem *base_addr, u32 intr_mask);
void pym_enable_otf_clk(void __iomem *base_addr, bool enable);
void pym_enable_ddr_clk(void __iomem *base_addr, bool enable);
void pym_set_rdma_start(void __iomem *base_addr);
void pym_set_common_rdy(void __iomem *base_addr, bool enable);
void pym_set_shd_select(void __iomem *base_addr, u8 cfg);
void pym_ds_uv_bypass(void __iomem *base_addr, u8 shadow_index, u32 port_num);
void pym_ds_enabe_base_layer(void __iomem *base_addr, u8 shadow_index, u32 layer_index);
void pym_us_clk_gate(void __iomem *base_addr, u8 shadow_index, bool uv_gate, bool clk_gate);
void pym_us_uv_bypass(void __iomem *base_addr, u8 shadow_index, u32 layer_index);
void pym_us_enabe_layer(void __iomem *base_addr, u8 shadow_index, u32 layer_index);
void pym_config_src_size(void __iomem *base_addr, u8 shadow_index, u32 src_w, u32 src_h);
void pym_ds_config_factor(void __iomem *base_addr, u8 shadow_index, u32 port_num, u32 factor);
void pym_ds_config_roi(void __iomem *base_addr, u8 shadow_index, u32 port_num, struct roi_rect *rect);
void pym_us_config_roi(void __iomem *base_addr, u8 shadow_index, u32 layer_index, struct roi_rect *rect);
void pym_us_config_factor(void __iomem *base_addr, u8 shadow_index, u32 layer_index, u32 factor);
void pym_ds_set_src_width(void __iomem *base_addr, u8 shadow_index, u32 port_num, u16 width);
void pym_us_set_src_width(void __iomem *base_addr, u8 shadow_index, u32 layer_index, u16 width);
void pym_set_shd_rdy(void __iomem *base_addr, u8 shadow_index, bool enable);
void pym_hw_dump(u32 __iomem *base_reg);

#endif
