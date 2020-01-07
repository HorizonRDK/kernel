/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef VIO_HW_API_COMMON_H
#define VIO_HW_API_COMMON_H

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;
typedef signed int s32;
typedef signed short s16;
typedef signed char s8;
typedef float f32;
typedef signed long long s64;
typedef unsigned long long u64;

enum regdata_type { 
	/* read write */ 
	RW = 0, 
	/* read only */ 
	RO = 1, 
	/* write only */ 
	WO = 2, 
	/* write input */ 
	WI = 2, 
	/* clear after read */ 
	RAC = 3, 
	/* write 1 -> clear */ 
	W1C = 4, 
	/* write 1 set */ 
	W1S = 5, 
	/* write 1 trigger */ 
	W1T = 6, 
};

struct vio_reg_def {
	char *reg_name;
	u32 sfr_offset;
	enum regdata_type attr;
};

struct vio_field_def {
	u32 reg;
	u32 index;
	u8 bit_start;
	u8 bit_width;
	u32 default_val;
};

u32 vio_hw_get_reg(void __iomem * base_addr, const struct vio_reg_def *reg);
void vio_hw_set_reg(void __iomem * base_addr, const struct vio_reg_def *reg,
		     u32 val);
u32 vio_hw_get_field(void __iomem * base_addr, const struct vio_reg_def *reg,
		      const struct vio_field_def *field);
void vio_hw_set_field(void __iomem * base_addr, const struct vio_reg_def *reg,
		       const struct vio_field_def *field, u32 val);
u32 vio_hw_get_field_value(u32 reg_value, const struct vio_field_def *field);
u32 vio_hw_set_field_value(u32 reg_value, const struct vio_field_def *field,
			    u32 val);
 
void vio_hw_dump_regs(void __iomem *base_addr, const struct vio_reg_def *regs,
				u32 total_cnt);
#endif	/*  */
