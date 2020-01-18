/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_IPS_HW_API__
#define __HOBOT_IPS_HW_API__

void ips_module_reset(void __iomem *base_addr, unsigned int module);
void ips_enable_intr(void __iomem *base_addr, unsigned int module, bool enable);
void ips_get_intr_status(void __iomem *base_addr, unsigned int module, u32 *status);
int ips_clk_ctrl(void __iomem *base_addr, unsigned int module, bool enable);
void ips_mot_set_roi(void __iomem *base_addr, struct roi_rect *rect);
u32 ipu_get_axi_statue(void __iomem *base_addr);
void ips_set_axi_bus_ctrl(void __iomem *base_addr, u32 cfg);
int ips_get_axi_bus_ctrl(void __iomem *base_addr);
void ips_enable_isp0_intr(void __iomem *base_addr, bool enable);
u16 ips_get_isp_frame_id(void __iomem *base_addr);

#endif
