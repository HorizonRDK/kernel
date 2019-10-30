/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/notifier.h>

#include "hb_notify.h"

/* define vio notify head */
static RAW_NOTIFIER_HEAD(vio_head);

/* define vcam notify head */
static RAW_NOTIFIER_HEAD(vcam_head);

/* register vio notifier */
int vio_register_notify(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&vio_head, nb);
}
EXPORT_SYMBOL(vio_register_notify);

/* unregister vio notifier */
int vio_unregister_notify(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&vio_head, nb);
}
EXPORT_SYMBOL(vio_unregister_notify);

/* notify others modules free frame */
int vio_notify_free_frame(unsigned long val, void *v)
{
	return raw_notifier_call_chain(&vio_head, val, v);
}
EXPORT_SYMBOL(vio_notify_free_frame);

/* register vcam notifier */
int vcam_register_notify(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&vcam_head, nb);
}
EXPORT_SYMBOL(vcam_register_notify);

/* unregister vcam notifier */
int vcam_unregister_notify(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&vcam_head, nb);
}
EXPORT_SYMBOL(vcam_unregister_notify);

/* notify others modules frame done */
int vcam_notify_frame_done(unsigned long val, void *v)
{
	return raw_notifier_call_chain(&vcam_head, val, v);
}
EXPORT_SYMBOL(vcam_notify_frame_done);
