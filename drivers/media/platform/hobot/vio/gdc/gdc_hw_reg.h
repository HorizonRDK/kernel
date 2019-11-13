#ifndef __X2A_GDC_REG_H__
#define __X2A_GDC_REG_H__

#include "../vio_hw_common_api.h"
#include "gdc_dev.h"

enum gdc_reg {
	GDC_ID,
	GDC_CONFIG_ADDR,
	GDC_CONFIG_SIZE,
	GDC_RDMA_IMG_WIDTH,
	GDC_RDMA_IMG_HEIGHT,
	GDC_RDMA0_IMG_ADDR,
	GDC_RDMA0_LINE_OFFSET,
	GDC_RDMA1_IMG_ADDR,
	GDC_RDMA1_LINE_OFFSET,
	GDC_RDMA2_IMG_ADDR,
	GDC_RDMA2_LINE_OFFSET,
	GDC_WDMA_IMG_WIDTH,
	GDC_WDMA_IMG_HEIGHT,
	GDC_WDMA0_IMG_ADDR,
	GDC_WDMA0_LINE_OFFSET,
	GDC_WDMA1_IMG_ADDR,
	GDC_WDMA1_LINE_OFFSET,
	GDC_WDMA2_IMG_ADDR,
	GDC_WDMA2_LINE_OFFSET,
	GDC_STATUS,
	GDC_PROCESS_CONFIG,
	GDC_CAPABILITY_STATUS,
	GDC_DEFAULT_CH1,
	GDC_DEFAULT_CH2,
	GDC_DEFAULT_CH3,
	GDC_DIAG_CFG_STALL_CNT0,
	GDC_DIAG_CFG_STALL_CNT1,
	GDC_DIAG_CFG_STALL_CNT2,
	GDC_DIAG_CFG_STALL_CNT3,
	GDC_DIAG_CFG_STALL_CNT4,
	GDC_DIAG_INT_READ_STALL_CNT,
	GDC_DIAG_INT_COORD_STALL_CNT,
	GDC_DIAG_INT_WRITE_WAIT_CNT,
	GDC_DIAG_WRT_WRITE_WAIT_CNT,
	GDC_DIAG_INT_DUAL_CNT,
	GDC_AXI_SETTING_CONFIG_READER,
	GDC_AXI_SETTING_TILE_READER,
	GDC_AXI_SETTING_TILE_WRITER,
	NUM_OF_GDC_REG,
};

static struct vio_reg_def gdc_regs[NUM_OF_GDC_REG]={
	{"GDC_ID",                          0x0000, RO},
	{"GDC_CONFIG_ADDR",                 0x0010, RW},
	{"GDC_CONFIG_SIZE",                 0x0014, RW},
	{"GDC_RDMA_IMG_WIDTH",              0x0020, RW},
	{"GDC_RDMA_IMG_HEIGHT",             0x0024, RW},
	{"GDC_RDMA0_IMG_ADDR",              0x0028, RW},
	{"GDC_RDMA0_LINE_OFFSET",           0x002c, RW},
	{"GDC_RDMA1_IMG_ADDR",              0x0030, RW},
	{"GDC_RDMA1_LINE_OFFSET",           0x0034, RW},
	{"GDC_RDMA2_IMG_ADDR",              0x0038, RW},
	{"GDC_RDMA2_LINE_OFFSET",           0x003c, RW},
	{"GDC_WDMA_IMG_WIDTH",              0x0040, RW},
	{"GDC_WDMA_IMG_HEIGHT",             0x0044, RW},
	{"GDC_WDMA0_IMG_ADDR",              0x0048, RW},
	{"GDC_WDMA0_LINE_OFFSET",           0x004c, RW},
	{"GDC_WDMA1_IMG_ADDR",              0x0050, RW},
	{"GDC_WDMA1_LINE_OFFSET",           0x0054, RW},
	{"GDC_WDMA2_IMG_ADDR",              0x0058, RW},
	{"GDC_WDMA2_LINE_OFFSET",           0x005c, RW},
	{"GDC_STATUS",                      0x0060, RO},
	{"GDC_PROCESS_CONFIG",              0x0064, RW},
	{"GDC_CAPABILITY_STATUS",           0x0068, RO},
	{"GDC_DEFAULT_CH1",                 0x0070, RW},
	{"GDC_DEFAULT_CH2",                 0x0074, RW},
	{"GDC_DEFAULT_CH3",                 0x0078, RW},
	{"GDC_DIAG_CFG_STALL_CNT0",         0x0080, RO},
	{"GDC_DIAG_CFG_STALL_CNT1",         0x0084, RO},
	{"GDC_DIAG_CFG_STALL_CNT2",         0x0088, RO},
	{"GDC_DIAG_CFG_STALL_CNT3",         0x008c, RO},
	{"GDC_DIAG_CFG_STALL_CNT4",         0x0090, RO},
	{"GDC_DIAG_INT_READ_STALL_CNT",     0x0094, RO},
	{"GDC_DIAG_INT_COORD_STALL_CNT",    0x0098, RO},
	{"GDC_DIAG_INT_WRITE_WAIT_CNT",     0x009c, RO},
	{"GDC_DIAG_WRT_WRITE_WAIT_CNT",     0x00a0, RO},
	{"GDC_DIAG_INT_DUAL_CNT",           0x00a4, RO},
	{"GDC_AXI_SETTING_CONFIG_READER",   0x00a8, RW},
	{"GDC_AXI_SETTING_TILE_READER",     0x00ac, RW},
	{"GDC_AXI_SETTING_TILE_WRITER",     0x00b0, RW},
};

enum gdc_reg_field{
	GDC_F_STOP_FLAG,
	GDC_F_START_FLAG,
	GDC_F_CONF_READER_RXACT_MAXOSTAND,
	GDC_F_CONF_READER_FIFO_WATERMARK,
	GDC_F_CONF_READER_MAX_ARLEN,
	GDC_F_TILE_READER_RXACT_MAXOSTAND,
	GDC_F_TILE_READER_FIFO_WATERMARK,
	GDC_F_TILE_READER_MAX_ARLEN,
	GDC_F_TILE_WRITER_RXACT_MAXOSTAND,
	GDC_F_TILE_WRITER_FIFO_WATERMARK,
	GDC_F_TILE_WRITER_MAX_ARLEN,
	NUM_OF_GDC_FIELD,
};

static struct vio_field_def gdc_fields[NUM_OF_GDC_FIELD] = {
	{GDC_PROCESS_CONFIG,	GDC_F_STOP_FLAG,  1 , 1 , 0},
	{GDC_PROCESS_CONFIG,	GDC_F_START_FLAG, 0 , 1 , 0},
	{GDC_AXI_SETTING_CONFIG_READER,	GDC_F_CONF_READER_RXACT_MAXOSTAND,  16, 8 , 0},
	{GDC_AXI_SETTING_CONFIG_READER,	GDC_F_CONF_READER_FIFO_WATERMARK,   8 , 8 , 16},
	{GDC_AXI_SETTING_CONFIG_READER,	GDC_F_CONF_READER_MAX_ARLEN,   		0 , 4 , 15},
	{GDC_AXI_SETTING_TILE_READER,	GDC_F_TILE_READER_RXACT_MAXOSTAND,  16, 8 , 0},
	{GDC_AXI_SETTING_TILE_READER,	GDC_F_TILE_READER_FIFO_WATERMARK,   8 , 8 , 16},
	{GDC_AXI_SETTING_TILE_READER,	GDC_F_TILE_READER_MAX_ARLEN,   		0 , 4 , 15},
	{GDC_AXI_SETTING_TILE_WRITER,	GDC_F_TILE_WRITER_RXACT_MAXOSTAND,  16, 8 , 0},
	{GDC_AXI_SETTING_TILE_WRITER,	GDC_F_TILE_WRITER_FIFO_WATERMARK,   8 , 8 , 16},
	{GDC_AXI_SETTING_TILE_WRITER,	GDC_F_TILE_WRITER_MAX_ARLEN,   		0 , 4 , 15},
};

#endif
