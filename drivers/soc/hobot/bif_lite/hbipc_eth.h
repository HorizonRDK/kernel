/*
 *			 COPYRIGHT NOTICE
 *		 Copyright 2019 Horizon Robotics, Inc.
 *			 All rights reserved.
 */

#ifndef _HBIPC_ETH_H_
#define _HBIPC_ETH_H_

int hbeth_bind_sock(void);
int hbeth_sendframe(void *buf, int len);
int hbeth_recvframe(void *buf, int len);
int hbeth_set_sendtimeout(int timeout_ms);
int hbeth_check_ready(void);
void hbeth_ap_start_connect(void);

#endif
