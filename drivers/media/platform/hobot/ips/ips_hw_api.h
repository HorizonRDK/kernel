/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_IPS_HW_API__
#define __HOBOT_IPS_HW_API__

#include "vio_config.h"

void ips_module_reset(void __iomem *base_addr, u32 module);
void ips_set_intr_mask(void __iomem *base_addr, u32 cfg);
void ips_enable_intr(void __iomem *base_addr, u32 module, bool enable);
void ips_get_intr_status(void __iomem *base_addr, u32 module, u32 *status,
		bool clear);
int ips_clk_ctrl(void __iomem *base_addr, u32 module, bool enable);
void ips_mot_set_roi(void __iomem *base_addr, struct roi_rect *rect);
void ips_mot_set_diff_thd(void __iomem *base_addr, u32 diff_thresh);
void ips_mot_set_thresh(void __iomem *base_addr, u32 thresh);
void ips_mot_set_step(void __iomem *base_addr, u32 step);
void ips_mot_set_fmt(void __iomem *base_addr, u32 fmt, u32 pixel_lenght);
void ips_mot_data_sel(void __iomem *base_addr, u8 data_sel);
void ips_mot_set_refresh(void __iomem *base_addr, bool enable);
void ips_mot_set_prec(void __iomem *base_addr, u32 prec, u32 decay);
void ips_mot_enable(void __iomem *base_addr, bool enable);
void ips_mot_set_resolution(void __iomem *base_addr, u32 width, u32 height);
u32 ipu_get_axi_statue(void __iomem *base_addr);
void ips_set_axi_bus_ctrl(void __iomem *base_addr, u32 cfg);
u32 ips_get_axi_bus_ctrl(void __iomem *base_addr);
void ips_enable_isp0_intr(void __iomem *base_addr, bool enable);
u16 ips_get_isp_frame_id(void __iomem *base_addr);
void isp_vcke_ctrl(void __iomem *base_addr, bool en);
void isp_vcke_th0(void __iomem *base_addr, u32 cfg);
void isp_vcke_th1(void __iomem *base_addr, u32 cfg);
void ips_hw_dump(u32 __iomem *base_reg);
void ips_set_sram_mux(void __iomem *base_addr, u32 md_cfg);
#endif
