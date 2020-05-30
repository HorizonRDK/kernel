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

#ifndef __HOBOT_DWE_H__
#define __HOBOT_DWE_H__

#include <linux/types.h>

#define X2A_LDC_NAME  "hobot,x3-ldc"
#define X2A_DIS_NAME  "hobot,x3-dis0"
#define X2A_DIS1_NAME  "hobot,x3-dis1"
#define X2A_DWE_NAME  "X3-DWE"

typedef struct _dwe_subdev_s {
        phys_addr_t io_paddr;
        char __iomem *io_vaddr;
        size_t io_memsize;
        int irq_num;
} dwe_subdev_s;

struct dwe_dev_s {
	dwe_subdev_s *ldc_dev;
	dwe_subdev_s *dis_dev;	
	dwe_subdev_s *dis1_dev;	
};

#endif /* __X2_ISP_H__ */
