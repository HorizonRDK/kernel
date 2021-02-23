/***************************************************************************
 *				COPYRIGHT NOTICE
 *			   Copyright 2020 Horizon Robotics, Inc.
 *				All rights reserved.
 ***************************************************************************/
#ifndef INCLUDE_J3_HOBOT_FUSA_FATAL_H_
#define INCLUDE_J3_HOBOT_FUSA_FATAL_H_

#define FUSA_FATAL_MAGIC 'f'
#define FUSA_SET_ERROR_PIN             _IOW(FUSA_FATAL_MAGIC, 0, unsigned int)

#define ERR_PIN_NORMAL 0
#define ERR_PIN_FATAL   1
void panic_diagnose_notify(void);

#endif//INCLUDE_J3_HOBOT_FUSA_FATAL_H_
