/*
 *			 COPYRIGHT NOTICE
 *		 Copyright 2019 Horizon Robotics, Inc.
 *			 All rights reserved.
 */

#ifndef _BIF_PLAT_H_
#define _BIF_PLAT_H_
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
//#include <linux/wait.h>
#include <linux/sched.h>
//#include <linux/sched/signal.h>
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
//#include <linux/wait.h>
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
//#include <asm-generic/io.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/vmalloc.h>

#if __SIZEOF_POINTER__ == 4
#define addr_t unsigned int
#else
#define addr_t unsigned long
#endif

//#define DEBUG
#ifdef DEBUG
#define bif_debug pr_info
#else
#define bif_debug(format, ...) do {} while (0)
#endif

#define bif_err pr_info

void *bif_memset(void *s, int c, size_t n);
void *bif_memcpy(void *dest, const void *src, size_t n);
unsigned long bif_sleep(unsigned int msec);
void bif_lock(void);
void bif_unlock(void);
void *bif_malloc(size_t size);
void bif_free(void *p);

#endif
