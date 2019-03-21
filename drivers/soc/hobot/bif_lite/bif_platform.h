#ifndef _BIF_PLAT_H_
#define _BIF_PLAT_H_
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/device.h>
#include <linux/compiler.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/eventpoll.h>
#include <asm-generic/io.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/time.h>

#define addr_t unsigned long long

//#define DEBUG
#ifdef DEBUG
#define bif_debug  pr_info
#else
#define bif_debug(format, ...) do {} while (0)
#endif

#define bif_err  pr_info

#ifndef CONFIG_HOBOT_BIF_AP
#define RX_LOCAL_INFO_OFFSET   AP_TO_CP_RING_CP_INFO_OFFSET
#define RX_REMOTE_INFO_OFFSET  AP_TO_CP_RING_AP_INFO_OFFSET
#define TX_LOCAL_INFO_OFFSET  CP_TO_AP_RING_CP_INFO_OFFSET
#define TX_REMOTE_INFO_OFFSET CP_TO_AP_RING_AP_INFO_OFFSET
#define RX_BUFFER_OFFSET AP_TO_CP_BUFFER_OFFSET
#define TX_BUFFER_OFFSET CP_TO_AP_BUFFER_OFFSET
#else
#define RX_LOCAL_INFO_OFFSET   CP_TO_AP_RING_AP_INFO_OFFSET
#define RX_REMOTE_INFO_OFFSET  CP_TO_AP_RING_CP_INFO_OFFSET
#define TX_LOCAL_INFO_OFFSET  AP_TO_CP_RING_AP_INFO_OFFSET
#define TX_REMOTE_INFO_OFFSET AP_TO_CP_RING_CP_INFO_OFFSET
#define RX_BUFFER_OFFSET CP_TO_AP_BUFFER_OFFSET
#define TX_BUFFER_OFFSET AP_TO_CP_BUFFER_OFFSET
#endif

void *bif_memset(void *s, int c, size_t n);
void *bif_memcpy(void *dest, const void *src, size_t n);
void  bif_rx_sleep(unsigned int msec);
void  bif_lock(void);
void  bif_unlock(void);
void *bif_malloc(size_t size);
void  bif_free(void *p);
int  bif_read_cp_ddr(void *dst, addr_t addr, int len);
int  bif_write_cp_ddr(void *src, addr_t addr, int len);
addr_t bif_phys_to_vaddr(addr_t start, size_t size);
void bif_set_base_addr(addr_t bif_base_addr);

#endif
