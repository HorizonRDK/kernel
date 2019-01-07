#ifndef __IPU_COMMON_H__
#define __IPU_COMMON_H__

#define IPU_DEBUG_LOG (0)
#define IPU_INFO_LOG (0)

#define IPU_MAX_SLOT				8
#define IPU_SLOT_SIZE				0x4000000

#define IPU_MAX_SLOT_DUAL			4
#define IPU_SLOT_DAUL_SIZE			0x8000000

#define ipu_dbg(fmt, args...)       {if(IPU_INFO_LOG && printk_ratelimit()) printk(KERN_INFO "[ipu][debug]: "fmt, ##args);}
#define ipu_info(fmt, args...)      {if(IPU_INFO_LOG && printk_ratelimit()) printk(KERN_INFO "[ipu][info]: "fmt, ##args);}
#define ipu_err(fmt, args...)       printk(KERN_ERR "[ipu][error]: "fmt, ##args)
#define ALIGN_16(d)         (((d) + 15) & ~0xf)

#define IPU_SLOT_NOT_AVALIABLE	0
#define IPU_TRIGGER_ISR	1
#define IPU_RCV_NOT_AVALIABLE	2
#define IPU_PYM_NOT_AVALIABLE	3
#define IPU_PYM_STARTUP		4

#endif
