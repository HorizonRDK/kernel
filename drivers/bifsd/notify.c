/*
 * All the BIFSD notify logic
 *
 * (C) Copyright 2018 shaochuanzhang <shaochuan.zhang@horizon.ai>
 *
 * notifier functions originally based on those in kernel/sys.c
 * but fixed up to not be so broken.
 *
 */

#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include "x2_bifsd.h"

BLOCKING_NOTIFIER_HEAD(bifsd_notifier_list);

/**
 * bifsd_register_notify - register a notifier callback whenever a cnn data received
 * @nb: pointer to the notifier block for the callback events.
 *
 */
void bifsd_register_notify(struct notifier_block *nb)
{
	blocking_notifier_chain_register(&bifsd_notifier_list, nb);
}

EXPORT_SYMBOL_GPL(bifsd_register_notify);

/**
 * bifsd_unregister_notify - unregister a notifier callback
 * @nb: pointer to the notifier block for the callback events.
 *
 * bifsd_register_notify() must have been previously called for this function
 * to work properly.
 */
void bifsd_unregister_notify(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&bifsd_notifier_list, nb);
}

EXPORT_SYMBOL_GPL(bifsd_unregister_notify);

void bifsd_notify_rx_data(void *data)
{
	blocking_notifier_call_chain(&bifsd_notifier_list, BIFSD_DATA_RX_COMP,
				     data);
}
