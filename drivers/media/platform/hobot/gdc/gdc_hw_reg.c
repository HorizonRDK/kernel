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

#include "gdc_hw_reg.h"
#include "gdc_hw_api.h"

void gdc_set_config_addr(void __iomem *base_addr, u32 config_addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_CONFIG_ADDR], config_addr);
}

void gdc_set_config_size(void __iomem *base_addr, u32 config_addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_CONFIG_SIZE], config_addr);
}

void gdc_set_rdma_img_width(void __iomem *base_addr, u32 width)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA_IMG_WIDTH], width);
}

void gdc_set_rdma_img_height(void __iomem *base_addr, u32 height)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA_IMG_HEIGHT], height);
}

void gdc_set_wdma_img_width(void __iomem *base_addr, u32 width)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA_IMG_WIDTH], width);
}

void gdc_set_wdma_img_height(void __iomem *base_addr, u32 height)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA_IMG_HEIGHT], height);
}

void gdc_process_enable(void __iomem *base_addr, bool enable)
{
	vio_hw_set_field(base_addr, &gdc_regs[GDC_PROCESS_CONFIG],
		&gdc_fields[GDC_F_START_FLAG], enable);
}

void gdc_process_reset(void __iomem *base_addr, bool enable)
{
	vio_hw_set_field(base_addr, &gdc_regs[GDC_PROCESS_CONFIG],
		&gdc_fields[GDC_F_STOP_FLAG], enable);
}

void gdc_set_rdma0_img_addr(void __iomem *base_addr, u32 addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA0_IMG_ADDR], addr);
}

void gdc_set_rdma1_img_addr(void __iomem *base_addr, u32 addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA1_IMG_ADDR], addr);
}

void gdc_set_rdma2_img_addr(void __iomem *base_addr, u32 addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA2_IMG_ADDR], addr);
}

void gdc_set_rdma0_line_offset(void __iomem *base_addr, u32 lineoffset)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA0_LINE_OFFSET], lineoffset);
}

void gdc_set_rdma1_line_offset(void __iomem *base_addr, u32 lineoffset)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA1_LINE_OFFSET], lineoffset);
}

void gdc_set_rdma2_line_offset(void __iomem *base_addr, u32 lineoffset)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_RDMA2_LINE_OFFSET], lineoffset);
}

void gdc_set_wdma0_img_addr(void __iomem *base_addr, u32 addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA0_IMG_ADDR], addr);
}

void gdc_set_wdma1_img_addr(void __iomem *base_addr, u32 addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA1_IMG_ADDR], addr);
}

void gdc_set_wdma2_img_addr(void __iomem *base_addr, u32 addr)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA2_IMG_ADDR], addr);
}

void gdc_set_wdma0_line_offset(void __iomem *base_addr, u32 lineoffset)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA0_LINE_OFFSET], lineoffset);
}

void gdc_set_wdma1_line_offset(void __iomem *base_addr, u32 lineoffset)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA1_LINE_OFFSET], lineoffset);
}

void gdc_set_wdma2_line_offset(void __iomem *base_addr, u32 lineoffset)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_WDMA2_LINE_OFFSET], lineoffset);
}

void gdc_set_default_ch1(void __iomem *base_addr, u32 default_ch)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_DEFAULT_CH1], default_ch);
}

void gdc_set_default_ch2(void __iomem *base_addr, u32 default_ch)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_DEFAULT_CH2], default_ch);
}

void gdc_set_default_ch3(void __iomem *base_addr, u32 default_ch)
{
	vio_hw_set_reg(base_addr, &gdc_regs[GDC_DEFAULT_CH3], default_ch);
}

u32 gdc_get_intr_status(void __iomem *base_addr)
{
	u32 status = 0;

	status = vio_hw_get_reg(base_addr, &gdc_regs[GDC_STATUS]);
	return status;
}

void gdc_hw_dump(u32 __iomem *base_reg)
{
	vio_hw_dump_regs(base_reg, gdc_regs, NUM_OF_GDC_REG);
}
