/*
 * isp.h - interfaces internal to the isp framework
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __X2_ISP_DEV_H__
#define __X2_ISP_DEV_H__

#include <linux/types.h>
#include <asm/compiler.h>

#define ISP_NAME    "x2-isp"

struct isp_dev_s {
	phys_addr_t mapbase;
	phys_addr_t mapbaseio;
	uint32_t memsize;
	void __iomem *regbase;
	void *vaddr;
	int irq;
};

/**
 * isp_get_dev
 * @void: NULL
 *
 * Return: isp_dev_t type, ref isp_dev_s struction.
 */
struct isp_dev_s *isp_get_dev(void);

#endif /* __X2_ISP_DEV_H__ */
