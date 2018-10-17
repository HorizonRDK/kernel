#ifndef __IPU_DRV_H__
#define __IPU_DRV_H__

#ifdef DEBUG
#define ipu_dbg(fmt, args...)       printk(KERN_DEBUG "[ipu][debug]: "fmt"\n", ##args)
#define ipu_err(fmt, args...)       printk(KERN_DEBUG "[ipu][error]: "fmt"\n", ##args)
#else
#define ipu_dbg(fmt, args...)
#define ipu_err(fmt, args...)
#endif

#define ALIGN_16(d)         (((d) + 15) & ~0xf)

typedef enum {
	IPUC_INIT = 0,
	IPUC_GET_IMG = 1,
	IPUC_CNN_DONE = 2,
	IPUC_SET_DDR = 20,
	IPUC_SET_CROP,
	IPUC_SET_SCALE,
	IPUC_SET_PYMID,
	IPUC_SET_BASE,
	IPUC_SET_FRAME_ID,
	IPUC_END,
} ipu_cmd_e;

#endif
