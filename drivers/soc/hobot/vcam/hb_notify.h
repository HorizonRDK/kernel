/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#ifndef DRIVERS_SOC_HOBOT_VCAM_HB_NOTIFY_H_
#define DRIVERS_SOC_HOBOT_VCAM_HB_NOTIFY_H_

#define VCAM_FRAME_DONE_NOTIFY	1
#define VCAM_FRAME_FREE_NOTIFY	2

int vio_register_notify(struct notifier_block *nb);
int vio_unregister_notify(struct notifier_block *nb);
int vio_notify_free_frame(unsigned long val, void *v);
int vcam_register_notify(struct notifier_block *nb);
int vcam_unregister_notify(struct notifier_block *nb);
int vcam_notify_frame_done(unsigned long val, void *v);

#endif // DRIVERS_SOC_HOBOT_VCAM_HB_NOTIFY_H_
