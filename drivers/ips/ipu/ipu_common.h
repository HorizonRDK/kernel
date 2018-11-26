#ifndef __IPU_COMMON_H__
#define __IPU_COMMON_H__

//#define DEBUG
//#define X2_ZU3
//#define IPU_DEBUG

#ifdef X2_ZU3
extern int bifdev_get_cpchip_reg(uint32_t addr, int *value );
extern int bifdev_set_cpchip_reg(uint32_t addr, int value );
#define ipu_reg_w(d, addr)      bifdev_set_cpchip_reg((uint64_t)(addr), (d));
#define ipu_reg_r(addr)         ({uint32_t v=0; bifdev_get_cpchip_reg((uint64_t)(addr), &v); v;})
#else
#define ipu_reg_w(d, addr)      writel((d), (addr))
#define ipu_reg_r(addr)         readl((addr))
#endif

#ifdef DEBUG
#define ipu_dbg(fmt, args...)       printk(KERN_INFO "[ipu][debug]: "fmt, ##args)
#else
#define ipu_dbg(fmt, args...)
#endif
#define ipu_info(fmt, args...)      printk(KERN_INFO "[ipu][info]: "fmt, ##args)
#define ipu_err(fmt, args...)       printk(KERN_ERR "[ipu][error]: "fmt, ##args)
#define ALIGN_16(d)         (((d) + 15) & ~0xf)

#endif
