/*
 * Horizon Robotics
 *
 *  Author:	Liwei Zhang <liwei.zhang@hobot.cc>
 *  Copyright 	(C) 2021 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef __HOBOT_FIQ_DEBUGGER_H
#define __HOBOT_FIQ_DEBUGGER_H

/**
 * hobot_hardlockup_fiq_support - displays whether fiq debugger should be used for hardlockup.
 * Returns 1 means support, while 0 means not supported.
 */
int hobot_hardlockup_fiq_support(void);

/**
 * hobot_fiqdebug_affinity_set - set fiq handler to specified cpu.
 * @id: cpu id.
 */
void hobot_fiqdebug_affinity_set(int id);
#endif //__HOBOT_FIQ_DEBUGGER_H

