/*
 *    driver, vb2_buffer interface
 *
 *    Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#ifndef __SYSTEM_HW_IO_H__
#define __SYSTEM_HW_IO_H__

#include <linux/types.h>
#include <asm/compiler.h>
#include <linux/string.h>
//#include <asm-generic/io.h>
#include <asm/io.h>

//-------------------------------------------------------------------------------------------- //
//register operation
//-------------------------------------------------------------------------------------------- //
static __inline void sys_write_32reg(const char __iomem *regbase,
	uint32_t addr, uint32_t *buffer)
{
	writel(*buffer, regbase + addr);
}

static __inline void sys_read_32reg(const char __iomem *regbase,
	uint32_t addr, uint32_t *buffer)
{
        *buffer = readl(regbase + addr);
}

static __inline void sys_write_buffer(const char __iomem *regbase,
	uint32_t addr, uint32_t *buffer, uint32_t len)
{
	uint32_t temp_i = 0;

	for (temp_i = 0; temp_i < len; temp_i++)
        	writel(*(buffer + temp_i), (regbase + addr + 4 * temp_i));
	//memcpy_toio((regbase + addr), buffer, len);
}

static __inline void sys_read_buffer(const char __iomem *regbase, uint32_t addr,
	uint32_t *buffer, uint32_t len)
{
	uint32_t temp_i = 0;
	
	for (temp_i = 0; temp_i < len; temp_i++)
        	*(buffer + temp_i) = readl(regbase + addr + 4 * temp_i);
	//memcpy_fromio(buffer, (regbase + addr), len);
}


#endif /* __LDC_BASE_IO_H__ */
