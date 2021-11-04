/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_GDC_HW_API__
#define __HOBOT_GDC_HW_API__

void gdc_set_config_addr(void __iomem *base_addr, u32 config_addr);
void gdc_set_config_size(void __iomem *base_addr, u32 config_addr);
void gdc_set_rdma_img_width(void __iomem *base_addr, u32 width);
void gdc_set_rdma_img_height(void __iomem *base_addr, u32 height);
void gdc_set_wdma_img_width(void __iomem *base_addr, u32 width);
void gdc_set_wdma_img_height(void __iomem *base_addr, u32 height);
void gdc_process_enable(void __iomem *base_addr, bool enable);
void gdc_process_reset(void __iomem *base_addr, bool enable);
void gdc_set_rdma0_img_addr(void __iomem *base_addr, u32 addr);
void gdc_set_rdma1_img_addr(void __iomem *base_addr, u32 addr);
void gdc_set_rdma2_img_addr(void __iomem *base_addr, u32 addr);
void gdc_set_rdma0_line_offset(void __iomem *base_addr, u32 lineoffset);
void gdc_set_rdma1_line_offset(void __iomem *base_addr, u32 lineoffset);
void gdc_set_rdma2_line_offset(void __iomem *base_addr, u32 lineoffset);
void gdc_set_wdma0_img_addr(void __iomem *base_addr, u32 addr);
void gdc_set_wdma1_img_addr(void __iomem *base_addr, u32 addr);
void gdc_set_wdma2_img_addr(void __iomem *base_addr, u32 addr);
void gdc_set_wdma0_line_offset(void __iomem *base_addr, u32 lineoffset);
void gdc_set_wdma1_line_offset(void __iomem *base_addr, u32 lineoffset);
void gdc_set_wdma2_line_offset(void __iomem *base_addr, u32 lineoffset);
void gdc_set_default_ch1(void __iomem *base_addr, u32 default_ch);
void gdc_set_default_ch2(void __iomem *base_addr, u32 default_ch);
void gdc_set_default_ch3(void __iomem *base_addr, u32 default_ch);
u32 gdc_get_intr_status(void __iomem *base_addr);
void gdc_hw_dump(u32 __iomem *base_reg);

#endif
