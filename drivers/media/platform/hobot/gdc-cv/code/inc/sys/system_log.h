/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/
#ifndef __SYSTEM_LOG_H__
#define __SYSTEM_LOG_H__

#include "system_stdlib.h"

#ifdef BUILD_GDC0
#define GDC_DEV_NAME  "gdc0"
#else
#define GDC_DEV_NAME  "gdc1"
#endif

int printk( const char *, ... );

enum {
    LOG_NOTHING,
    LOG_EMERG,
    LOG_ALERT,
    LOG_CRIT,
    LOG_ERR,
    LOG_WARNING,
    LOG_NOTICE,
    LOG_INFO,
    LOG_DEBUG,
    LOG_IRQ,
    LOG_MAX
};

extern const char *const log_level[LOG_MAX];

#define FILE ( strrchr( __FILE__, '/' ) ? strrchr( __FILE__, '/' ) + 1 : __FILE__ )

#define LOG( level, fmt, ... ) \
    if ( ( level ) <= FW_LOG_LEVEL ) printk( "%s: %s: %s(%d) %s: " fmt "\n", GDC_DEV_NAME, FILE, __func__, __LINE__, log_level[level], ##__VA_ARGS__ )

#endif // __SYSTEM_LOG_H__
