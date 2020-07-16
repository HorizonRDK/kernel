/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_VIO_CONFIG_H__
#define __HOBOT_VIO_CONFIG_H__

#define vio_err(fmt, ...)	printk( fmt, ##__VA_ARGS__)
#define vio_warn(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define vio_dbg(fmt, ...)	pr_debug(fmt, ##__VA_ARGS__)
#define vio_info(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define vio_cont(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)

#define IRAM_MAX_RANG	0x140000

#define VIO_MAX_STREAM	8
#define CONFIG_QEMU_TEST 0
enum vio_video_state { 
	VIO_VIDEO_CLOSE,
	VIO_VIDEO_OPEN,
	VIO_VIDEO_S_INPUT,
	VIO_VIDEO_INIT,
	VIO_VIDEO_REBUFS,
	VIO_VIDEO_STOP,
	VIO_VIDEO_START, 
	VIO_VIDEO_ION_ALLOC,
};

enum FrameErrorType{
	VIO_FRAME_DONE = 1,
	VIO_FRAME_NDONE = 2,
};

enum RST_id{
	SIF_RST,
	DWE1_RST,
	DWE0_RST,
	IPU0_RST,
	ISP0_RST,
	IRAM_RST,
	IPU_PYM_RST,
	PYM_RST,
};

enum clock_gate{
	IRAM_CLOCK_GATE,
	MD_CLOCK_GATE,
	IPU0_CLOCK_GATE,
	LDC1_CLOCK_GATE,
	LDC0_CLOCK_GATE,
	T2L1_CLOCK_GATE,
	T2L0_CLOCK_GATE,
	GDC1_CLOCK_GATE,
	GDC0_CLOCK_GATE,
	DWE1_CLOCK_GATE,
	DWE0_CLOCK_GATE,
	ISP0_CLOCK_GATE,
	SIF_CLOCK_GATE,
};

enum intr_ip{
	MOD_INTR,
	IRAM_INTR,
	AXI1_INTR,
	AXI0_INTR,
};

struct roi_rect{
    u16 roi_x;
    u16 roi_y;
    u16 roi_width;
    u16 roi_height;
};

typedef struct sif_output_md {
	uint32_t	enable;
	uint32_t	path_sel; //ipu:0, isp:1
	uint32_t	roi_top;
	uint32_t	roi_left;
	uint32_t	roi_width;
	uint32_t	roi_height;
	uint32_t	grid_step;
	uint32_t	grid_tolerance;
	uint32_t	threshold;
	uint32_t	weight_decay;
	uint32_t	precision;
} sif_output_md_t;

#endif /* __HOBOT_VIO_CONFIG_H__ */
