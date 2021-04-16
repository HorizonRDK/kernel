/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_XHCI_DEBUGFS_H
#define __LINUX_XHCI_DEBUGFS_H

enum xhci_hcd_state {
	XHCI_HCD_UNKNOWN,
	XHCI_HCD_ACTIVE,
	XHCI_HCD_SUSPEND,
};

#ifdef CONFIG_DEBUG_FS
void xhci_debugfs_init(struct xhci_hcd *xhci);
void xhci_debugfs_exit(struct xhci_hcd *xhci);
void __init xhci_debugfs_create_root(void);
void __exit xhci_debugfs_remove_root(void);
#else
static inline void xhci_debugfs_init(struct xhci_hcd *xhci) { }
static inline void xhci_debugfs_exit(struct xhci_hcd *xhci) { }
static inline void __init xhci_debugfs_create_root(void) { }
static inline void __exit xhci_debugfs_remove_root(void) { }
#endif /* CONFIG_DEBUG_FS */

#endif /* __LINUX_XHCI_DEBUGFS_H */
