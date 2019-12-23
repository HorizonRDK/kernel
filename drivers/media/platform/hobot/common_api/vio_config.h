#ifndef __HOBOT_VIO_CONFIG_H__
#define __HOBOT_VIO_CONFIG_H__

#define vio_err(fmt, ...)	printk( fmt, ##__VA_ARGS__)
#define vio_warn(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define vio_dbg(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define vio_info(fmt, ...)	printk( fmt, ##__VA_ARGS__)
#define vio_cont(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
    
#define VIO_MAX_STREAM	6 
#define CONFIG_QEMU_TEST 0
enum vio_video_state { 
	VIO_VIDEO_CLOSE,
	VIO_VIDEO_OPEN,
	VIO_VIDEO_S_INPUT,
	VIO_VIDEO_INIT,
	VIO_VIDEO_REBUFS,
	VIO_VIDEO_STOP,
	VIO_VIDEO_START, 
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

#endif	/*  */
