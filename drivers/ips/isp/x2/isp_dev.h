/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __X2_ISP_DEV_H__
#define __X2_ISP_DEV_H__

#include <linux/types.h>
#include <asm/compiler.h>

#define ISP_NAME    "x2-isp"

struct isp_dev_s {
	/* if isp mem from ion */
	struct ion_client *isp_iclient;
	struct ion_handle *isp_ihandle;
	/* if isp mem from reserve mem */
	phys_addr_t mapbase;
	phys_addr_t mapbaseio;
	size_t memsize;
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
