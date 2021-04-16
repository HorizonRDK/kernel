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

#ifndef __SYSTEM_CALIBDEV_H__
#define __SYSTEM_CALIBEV_H__

//int system_calib_init( void );
//int system_calib_destroy( void );

int register_calib( ACameraCalibrations *c, uint8_t port );
int unregister_calib( ACameraCalibrations *c, uint8_t port );


#endif /* __SYSTEM_CHARDEV_H__ */
