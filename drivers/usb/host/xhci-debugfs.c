/**
 * xhci-debugfs.c
 *
 * Copyright (C) 2020 Horizon Robotics
 *
 * Authors: jianghe xu<jianghe.xu@horizon.ai>,
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
// #include <linux/pm.h>

#include "xhci.h"
#include "xhci-debugfs.h"

static struct dentry *xhci_debugfs_root;
enum xhci_hcd_state xhci_state = XHCI_HCD_UNKNOWN;

static int xhci_state_show(struct seq_file *s, void *unused)
{
	switch (xhci_state) {
	case XHCI_HCD_SUSPEND:
		seq_printf(s, "suspend\n");
		break;
	case XHCI_HCD_ACTIVE:
	default:
		seq_printf(s, "active\n");
		break;
	}

	return 0;
}

static int xhci_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, xhci_state_show, inode->i_private);
}

static ssize_t xhci_state_write(struct file *file, const char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct seq_file		*s = file->private_data;
	struct usb_hcd		*hcd = s->private;
	char			buf[32];
	u32			size = 0;

	size = min_t(size_t, sizeof(buf) - 1, count);
	if (copy_from_user(&buf, ubuf, size))
		return -EFAULT;

	if (!strncmp(buf, "suspend", 7)) {
		xhci_state = XHCI_HCD_SUSPEND;
		// xhci_suspend(xhci, 1);
		// usb_suspend(dev, PMSG_SUSPEND);
		xhci_bus_suspend(hcd);
	}

	if (!strncmp(buf, "resume", 6)) {
		xhci_state = XHCI_HCD_ACTIVE;
		// xhci_resume(xhci, 1);
		// usb_resume(dev, PMSG_RESUME);
		xhci_bus_resume(hcd);
	}

	return count;
}

static const struct file_operations xhci_state_fops = {
	.open		= xhci_state_open,
	.write		= xhci_state_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void xhci_debugfs_init(struct xhci_hcd *xhci)
{
	struct device	*dev = xhci_to_hcd(xhci)->self.controller;
	struct usb_hcd *usb2_hcd;
	struct usb_hcd *usb3_hcd;

	xhci->debugfs_root = debugfs_create_dir(dev_name(dev),
			xhci_debugfs_root);

	usb2_hcd = xhci->main_hcd;
	usb3_hcd = xhci->shared_hcd;

	debugfs_create_file("usb2_state", S_IRUGO | S_IWUSR, xhci->debugfs_root,
			usb2_hcd, &xhci_state_fops);

	debugfs_create_file("usb3_state", S_IRUGO | S_IWUSR, xhci->debugfs_root,
			usb3_hcd, &xhci_state_fops);
}

void xhci_debugfs_exit(struct xhci_hcd *xhci)
{
	debugfs_remove_recursive(xhci->debugfs_root);
	xhci->debugfs_root = NULL;
}

void __init xhci_debugfs_create_root(void)
{
	xhci_debugfs_root = debugfs_create_dir("xhci", usb_debug_root);
}

void __exit xhci_debugfs_remove_root(void)
{
	debugfs_remove_recursive(xhci_debugfs_root);
	xhci_debugfs_root = NULL;
}
