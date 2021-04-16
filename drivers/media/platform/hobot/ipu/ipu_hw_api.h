/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_IPU_HW_API__
#define __HOBOT_IPU_HW_API__

void ipu_set_input_img_size(void __iomem *base_addr, u8 shadow_index,
		u32 width, u32 height);
void ipu_set_rdma_stride(void __iomem *base_addr, u8 shadow_index,
		u32 y_stride, u32 uv_stride);
void ipu_set_us_wdma_stride(void __iomem *base_addr, u8 shadow_index,
		u32 y_stride, u32 uv_stride);
void ipu_set_ds_wdma_stride(void __iomem *base_addr, u8 shadow_index,
		u8 ds_ch, u32 y_stride, u32 uv_stride);
void ipu_set_ds_roi_enable(void __iomem *base_addr, u8 shadow_index,
		u8 ds_ch, u8 enable);
void ipu_set_ds_roi_rect(void __iomem *base_addr, u8 shadow_index,
		u8 ds_ch, u16 start_x, u16 start_y, u16 width, u16 height);
void ipu_set_ds_enable(void __iomem *base_addr, u8 shadow_index,
		u8 ds_ch, bool enable);
void ipu_set_ds2_wdma_enable(void __iomem *base_addr,
		u8 shadow_index, u8 enable);
void ipu_set_ds_target_size(void __iomem *base_addr, u8 shadow_index,
		u8 ds_ch, u16 tgt_width, u16 tgt_height);
void ipu_set_ds_step(void __iomem *base_addr, u8 shadow_index, u8 ds_ch,
		u16 step_x, u16 step_y, u8 pre_x, u8 pre_y);
void ipu_set_us_roi_enable(void __iomem *base_addr, u8 shadow_index,
		u8 enable);
void ipu_set_us_roi_rect(void __iomem *base_addr, u8 shadow_index,
		u16 start_x, u16 start_y, u16 width, u16 height);
void ipu_set_us_enable(void __iomem *base_addr, u8 shadow_index,
		bool enable);
void ipu_set_us_target(void __iomem *base_addr, u8 shadow_index,
		u16 step_x, u16 step_y, u16 tgt_width, u16 tgt_height);
void ipu_set_osd_enable(void __iomem *base_addr, u8 shadow_index,
		u32 osd_num, u32 cfg);
void ipu_set_osd_overlay_mode(void __iomem *base_addr, u8 shadow_index,
		u32 osd_num, u8 cfg);
void ipu_set_osd_roi(void __iomem *base_addr, u8 shadow_index, u32 osd_num,
		u32 osd_layer, u16 start_x, u16 start_y, u16 width, u16 height);
void ipu_set_osd_sta_enable(void __iomem *base_addr, u8 shadow_index,
		u32 osd_num, u32 cfg);
void ipu_set_osd_sta_roi(void __iomem *base_addr, u8 shadow_index,
		u32 osd_num, u32 osd_layer,
		u16 start_x, u16 start_y, u16 width, u16 height);
void ipu_set_osd_sta_level(void __iomem *base_addr, u8 shadow_index,
		u32 osd_num, u8 level);
void ipu_set_osd_addr(void __iomem *base_addr, u8 shadow_index,
		u32 osd_num, u32 osd_layer, u32 addr);
void ipu_set_shd_rdy(void __iomem *base_addr, u8 cfg);
u32 ipu_get_shd_rdy(void __iomem *base_addr);
u32 ipu_get_size_err(void __iomem *base_addr);
void ipu_clear_size_err(void __iomem *base_addr, u8 value);
void ipu_set_shd_select(void __iomem *base_addr, u8 cfg);
void ipu_set_osd_color(void __iomem *base_addr, u32 color_index, u32 color);

void ipu_set_line_delay(void __iomem *base_addr, u8 value);
void ipu_src_select(void __iomem *base_addr, u8 cfg);
void ipu_set_frameid_enable(void __iomem *base_addr, u8 enable);
void ipu_set_frameid_value(void __iomem *base_addr, u16 value);
void ipu_get_frame_id(void __iomem *base_addr, u32 *frameid);
void ipu_set_rdma_addr(void __iomem *base_addr, u32 y_addr, u32 cb_addr);
void ipu_set_us_wdma_addr(void __iomem *base_addr, u32 y_addr,
		u32 cb_addr);
void ipu_set_ds_wdma_addr(void __iomem *base_addr, u8 ds_ch,
		u32 y_addr, u32 cb_addr);
void ipu_get_osd_sta_bin(void __iomem *base_addr, u8 osd_num,
		u8 osd_layer, u16 *bin);
void ipu_set_rdma_start(void __iomem *base_addr);
void ipu_get_intr_status(void __iomem *base_addr, u32 *status, bool clear);

void ipu_set_intr_mask(void __iomem *base_addr, u32 intr_mask);
void ipu_set_ddr_fifo5(void __iomem *base_addr, u8 value);
void ipu_set_ddr_fifo_thred(void __iomem *base_addr, u8 index, u32 value);
int ipu_get_err_status(void __iomem *base_addr);
void ipu_hw_dump(u32 __iomem *base_reg);
void ipu_get_ddr_addr_dump(u32 __iomem *base_reg);
void ipu_set_chn0_chn4_prio(void __iomem *base_addr);
#endif
