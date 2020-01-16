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

#ifndef __ACAMERA_DWE_CONFIG_H__
#define __ACAMERA_DWE_CONFIG_H__


#define FIRMWARE_CONTEXT_NUMBER 4
#define HADRWARE_CONTEXT_MAX    4
#define BUFFER_DMA 1 // 1 use dma  0 use vmalloc
#define DIS_STAT_SIZE 12000 // (1920 + 1080)*1080 byte


#endif
