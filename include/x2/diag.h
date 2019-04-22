/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2019 Horizon Robotics, Inc.
 * All rights reserved.
 **************************************************************************/
/**
 * @file     diag.h
 * @brief    diag api(send diag msg to usrspace)
 * @author   chenbo(bo01.chen@horizon.ai)
 * @date     2019/4/18
 * @version  V1.0
 * @par      Horizon Robotics
 */
#ifndef DIAG_H
#define DIAG_H

extern struct net init_net;

/*
 * send diag msg to userspace
 * @pbuf: data buffer.
 * @len:  data length(how many bytes)
 * @return: -1:error, >=0:success
 */
#ifdef CONFIG_X2_DIAG
extern int send_diag_msg(char *pbuf, uint16_t len);
#else
static inline int send_diag_msg(char *buf, uint16_t len) { return 0; }
#endif

#endif
