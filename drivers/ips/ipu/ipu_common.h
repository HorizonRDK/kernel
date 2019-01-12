#ifndef __IPU_COMMON_H__
#define __IPU_COMMON_H__

#define IPU_LOG_DEBUG	(1)
#define IPU_LOG_INFO	(2)
extern unsigned int ipu_debug_level;
#define ipu_info(fmt, args...)	\
	do {									\
		if((ipu_debug_level >= IPU_LOG_INFO))	\
			printk(KERN_INFO "[ipu][info]: "fmt, ##args);		\
	} while(0)

#define ipu_dbg(fmt, args...)	\
	do {									\
		if((ipu_debug_level >= IPU_LOG_DEBUG)) \
			printk(KERN_INFO "[ipu][debug]: "fmt, ##args);		\
	} while(0)

#define ipu_err(fmt, args...)       printk(KERN_ERR "[ipu][error]: "fmt, ##args)

#define IPU_MAX_SLOT				8
#define IPU_SLOT_SIZE				0x4000000

#define IPU_MAX_SLOT_DUAL			4
#define IPU_SLOT_DAUL_SIZE			0x8000000

#define ALIGN_16(d)         (((d) + 15) & ~0xf)

#define IPU_SLOT_NOT_AVALIABLE	0
#define IPU_TRIGGER_ISR	1
#define IPU_RCV_NOT_AVALIABLE	2
#define IPU_PYM_NOT_AVALIABLE	3
#define IPU_PYM_STARTUP		4
#define IPU_RCV_CFG_UPDATE	5
#define IPU_PYM_CFG_UPDATE	6

#endif
