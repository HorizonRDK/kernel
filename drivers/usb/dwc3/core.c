/**
 * core.c - DesignWare USB3 DRD Controller Core file
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *         Sebastian Andrzej Siewior <bigeasy@linutronix.de>
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

#include <linux/clk.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/acpi.h>
#include <linux/pinctrl/consumer.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/of.h>
#include <linux/usb/otg.h>

#include <soc/hobot/hobot_bus.h>

#include "core.h"
#include "gadget.h"
#include "io.h"

#include "debug.h"

#define DWC3_DEFAULT_AUTOSUSPEND_DELAY	5000 /* ms */

// #define HOBOT_FPGA_TEST
#ifdef HOBOT_FPGA_TEST
#define HOBOT_TEST_REGISTER_RW
#define HOBOT_FPGA_TEST_TIMING_FINETUNE
#define HOBOT_FPGA_USING_M31_PHY
#endif

#define HOBOT_SOC_USB_RESET
// #define HOBOT_SOC_DWC3_SETTING
// #define HOBOT_SOC_DWC3_DEBUG

#define HOBOT_DDR_DFS_ENABLE
// #define HOBOT_ENABLE_USB_REGULATOR	/* some issue, debugging... */

static int dwc3_suspend(struct device *dev);
static int dwc3_resume(struct device *dev);
static int dwc3_probe(struct platform_device *pdev);
static int dwc3_remove(struct platform_device *pdev);

#ifdef HOBOT_DDR_DFS_ENABLE
/*
 * For ddr dynamic frequency selection & handling.
 * Register a ddr frequency change notifier.
 * When ddr frequency change, hb_bus module will notify HB_BUS_SIGNAL_START
 * and HB_BUS_SIGNAL_END signals. Between this 2 signals, we need to guarantee
 * usb controller don't access the ddr. Therefore, use the spinlock dwc->lock
 * to do synchronize.
 */
static int ddr_dfs_call(struct notifier_block *self,
		unsigned long action, void *data)
{
	struct dwc3 *dwc = container_of(self, struct dwc3, ddrfreq_nb);

	switch (action) {
	case HB_BUS_SIGNAL_START:
		/*
		 * Between START & END, there is sleep function, so couldn't use
		 * below spin_lock.
		 */
		// spin_lock_irqsave(&dwc->lock, flags);
		// dev_dbg(dwc->dev, "%s: Grab semaphore success\n", __func__);
		dev_info(dwc->dev, "%s: bus signal start.\n", __func__);
		break;

	case HB_BUS_SIGNAL_END:
		// spin_unlock_irqrestore(&dwc->lock, flags);
		// dev_dbg(dwc->dev, "%s: Release semaphore success\n", __func__);

		/* enter powersave state, suspend dwc3 & shutdown the power */
		dev_info(dwc->dev, "%s: bus signal end.\n", __func__);
		break;

	default:
		dev_err(dwc->dev, "%s: Not defined action (%lu)\n",
				__func__, action);
		break;
	}

	return NOTIFY_OK;
}

static int power_save_call(struct notifier_block *self,
		unsigned long action, void *data)
{
	struct dwc3 *dwc = container_of(self, struct dwc3, powersave_nb);
	// struct platform_device *pdev = container_of(dwc->dev, struct platform_device, dev);
	int ret;

	switch (action) {
	case POWERSAVE_STATE:
		/* before enter powersave state, suspend & shutdown the power */
		dev_info(dwc->dev, "%s: powersave state.\n", __func__);
#ifdef CONFIG_PM_SLEEP
		dev_info(dwc->dev, "%s: dwc3_suspend.\n", __func__);
		dwc3_suspend(dwc->dev);
#else
		dev_info(dwc->dev, "%s: CONFIG_PM_SLEEP not enable\n", __func__);
#endif
		// dev_info(dwc->dev, "%s: dwc3_remove.\n", __func__);
		// dwc3_remove(pdev);
#ifdef HOBOT_ENABLE_USB_REGULATOR
		ret = regulator_disable(dwc->regulator);
		if (ret)
			dev_err(dwc->dev, "usb regulator disable error\n");
		else
			dev_info(dwc->dev, "%s: regulator disabled.\n", __func__);
#endif
		break;
	case OTHER_STATE:
		/* before leave powersave state, supply power & resume */
		dev_info(dwc->dev, "%s: otherstate state.\n", __func__);
#ifdef HOBOT_ENABLE_USB_REGULATOR
		ret = regulator_enable(dwc->regulator);
		if (ret)
			dev_err(dwc->dev, "usb regulator enable error\n");
		else
			dev_info(dwc->dev, "%s: regulator enabled.\n", __func__);
#endif
		// dev_info(dwc->dev, "%s: dwc3_probe.\n", __func__);
		// dwc3_probe(pdev);
#ifdef CONFIG_PM_SLEEP
		dev_info(dwc->dev, "%s: dwc3_resume.\n", __func__);
		dwc3_resume(dwc->dev);
#else
		dev_info(dwc->dev, "%s: CONFIG_PM_SLEEP not enable\n", __func__);
#endif
		break;
	default:
		dev_err(dwc->dev, "%s: Not defined action (%lu)\n",
				__func__, action);
		break;
	}

	return NOTIFY_OK;
}
#endif

/*
 * The code in this file is utility code, used to build a gadget driver
 * from one or more "function" drivers, one or more "configuration"
 * objects, and a "usb_composite_driver" by gluing them together along
 * with the relevant device-wide data.
 */

/**
 * dwc3_get_dr_mode - Validates and sets dr_mode
 * @dwc: pointer to our context structure
 */
static int dwc3_get_dr_mode(struct dwc3 *dwc)
{
	enum usb_dr_mode mode;
	struct device *dev = dwc->dev;
	unsigned int hw_mode;

	if (dwc->dr_mode == USB_DR_MODE_UNKNOWN)
		dwc->dr_mode = USB_DR_MODE_OTG;

	mode = dwc->dr_mode;
	hw_mode = DWC3_GHWPARAMS0_MODE(dwc->hwparams.hwparams0);

	switch (hw_mode) {
	case DWC3_GHWPARAMS0_MODE_GADGET:
		if (IS_ENABLED(CONFIG_USB_DWC3_HOST)) {
			dev_err(dev,
				"Controller does not support host mode.\n");
			return -EINVAL;
		}
		mode = USB_DR_MODE_PERIPHERAL;
		break;
	case DWC3_GHWPARAMS0_MODE_HOST:
		if (IS_ENABLED(CONFIG_USB_DWC3_GADGET)) {
			dev_err(dev,
				"Controller does not support device mode.\n");
			return -EINVAL;
		}
		mode = USB_DR_MODE_HOST;
		break;
	default:
		if (IS_ENABLED(CONFIG_USB_DWC3_HOST))
			mode = USB_DR_MODE_HOST;
		else if (IS_ENABLED(CONFIG_USB_DWC3_GADGET))
			mode = USB_DR_MODE_PERIPHERAL;
	}

	if (mode != dwc->dr_mode) {
		dev_warn(dev,
			 "Configuration mismatch. dr_mode forced to %s\n",
			 mode == USB_DR_MODE_HOST ? "host" : "gadget");

		if (IS_ENABLED(CONFIG_USB_DWC3_GADGET)) {
			dwc->dr_mode = USB_DR_MODE_PERIPHERAL;
			printk(">>> dwc3_get_dr_mode: force to PERIPHERAL \n");
		} else if (IS_ENABLED(CONFIG_USB_DWC3_GADGET)) {
			dwc->dr_mode = USB_DR_MODE_HOST;
			printk(">>> dwc3_get_dr_mode: force to HOST Mode\n");
		}
	}

	return 0;
}

static void dwc3_event_buffers_cleanup(struct dwc3 *dwc);
static int dwc3_event_buffers_setup(struct dwc3 *dwc);

static void dwc3_set_prtcap(struct dwc3 *dwc, u32 mode)
{
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~(DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG));
	reg |= DWC3_GCTL_PRTCAPDIR(mode);
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);
}

static void __dwc3_set_mode(struct work_struct *work)
{
	struct dwc3 *dwc = work_to_dwc(work);
	unsigned long flags;
	int ret;

	if (!dwc->desired_dr_role)
		return;

	if (dwc->desired_dr_role == dwc->current_dr_role)
		return;

	if (dwc->dr_mode != USB_DR_MODE_OTG)
		return;

	if (dwc->desired_dr_role == DWC3_GCTL_PRTCAP_OTG)
		return;

	switch (dwc->current_dr_role) {
	case DWC3_GCTL_PRTCAP_HOST:
		dwc3_host_exit(dwc);
		break;
	case DWC3_GCTL_PRTCAP_DEVICE:
		dwc3_gadget_exit(dwc);
		dwc3_event_buffers_cleanup(dwc);
		break;
	default:
		break;
	}

	spin_lock_irqsave(&dwc->lock, flags);

	dwc3_set_prtcap(dwc, dwc->desired_dr_role);

	dwc->current_dr_role = dwc->desired_dr_role;

	spin_unlock_irqrestore(&dwc->lock, flags);

	switch (dwc->desired_dr_role) {
	case DWC3_GCTL_PRTCAP_HOST:
		ret = dwc3_host_init(dwc);
		if (ret) {
			dev_err(dwc->dev, "failed to initialize host\n");
		} else {
			if (dwc->usb2_phy)
				otg_set_vbus(dwc->usb2_phy->otg, true);
			if (dwc->usb2_generic_phy)
				phy_set_mode(dwc->usb2_generic_phy, PHY_MODE_USB_HOST);

		}
		break;
	case DWC3_GCTL_PRTCAP_DEVICE:
		dwc3_event_buffers_setup(dwc);

		if (dwc->usb2_phy)
			otg_set_vbus(dwc->usb2_phy->otg, false);
		if (dwc->usb2_generic_phy)
			phy_set_mode(dwc->usb2_generic_phy, PHY_MODE_USB_DEVICE);

		ret = dwc3_gadget_init(dwc);
		if (ret)
			dev_err(dwc->dev, "failed to initialize peripheral\n");
		break;
	default:
		break;
	}
}

void dwc3_set_mode(struct dwc3 *dwc, u32 mode)
{
	unsigned long flags;

	spin_lock_irqsave(&dwc->lock, flags);
	dwc->desired_dr_role = mode;
	spin_unlock_irqrestore(&dwc->lock, flags);

	queue_work(system_freezable_wq, &dwc->drd_work);
}

u32 dwc3_core_fifo_space(struct dwc3_ep *dep, u8 type)
{
	struct dwc3		*dwc = dep->dwc;
	u32			reg;

	dwc3_writel(dwc->regs, DWC3_GDBGFIFOSPACE,
			DWC3_GDBGFIFOSPACE_NUM(dep->number) |
			DWC3_GDBGFIFOSPACE_TYPE(type));

	reg = dwc3_readl(dwc->regs, DWC3_GDBGFIFOSPACE);

	return DWC3_GDBGFIFOSPACE_SPACE_AVAILABLE(reg);
}

/**
 * dwc3_core_soft_reset - Issues core soft reset and PHY reset
 * @dwc: pointer to our context structure
 */
static int dwc3_core_soft_reset(struct dwc3 *dwc)
{
	u32		reg;
	int		retries = 1000;
	int		ret;

	usb_phy_init(dwc->usb2_phy);
	usb_phy_init(dwc->usb3_phy);
	ret = phy_init(dwc->usb2_generic_phy);
	if (ret < 0)
		return ret;

	ret = phy_init(dwc->usb3_generic_phy);
	if (ret < 0) {
		phy_exit(dwc->usb2_generic_phy);
		return ret;
	}

	/*
	 * We're resetting only the device side because, if we're in host mode,
	 * XHCI driver will reset the host block. If dwc3 was configured for
	 * host-only mode, then we can return early.
	 */
	if (dwc->dr_mode == USB_DR_MODE_HOST)
		return 0;

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg |= DWC3_DCTL_CSFTRST;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	do {
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		if (!(reg & DWC3_DCTL_CSFTRST))
			goto done;

		udelay(1);
	} while (--retries);

	phy_exit(dwc->usb3_generic_phy);
	phy_exit(dwc->usb2_generic_phy);

	return -ETIMEDOUT;

done:
	/*
	 * For DWC_usb31 controller, once DWC3_DCTL_CSFTRST bit is cleared,
	 * we must wait at least 50ms before accessing the PHY domain
	 * (synchronization delay). DWC_usb31 programming guide section 1.3.2.
	 */
	if (dwc3_is_usb31(dwc))
		msleep(50);

	return 0;
}

static const struct clk_bulk_data dwc3_core_clks[] = {
	{ .id = "usb_aclk" },
};

/*
 * dwc3_frame_length_adjustment - Adjusts frame length if required
 * @dwc3: Pointer to our controller context structure
 */
static void dwc3_frame_length_adjustment(struct dwc3 *dwc)
{
	u32 reg;
	u32 dft;

	if (dwc->revision < DWC3_REVISION_250A)
		return;

	if (dwc->fladj == 0)
		return;

	reg = dwc3_readl(dwc->regs, DWC3_GFLADJ);
	dft = reg & DWC3_GFLADJ_30MHZ_MASK;
	if (!dev_WARN_ONCE(dwc->dev, dft == dwc->fladj,
	    "request value same as default, ignoring\n")) {
		reg &= ~DWC3_GFLADJ_30MHZ_MASK;
		reg |= DWC3_GFLADJ_30MHZ_SDBND_SEL | dwc->fladj;
		dwc3_writel(dwc->regs, DWC3_GFLADJ, reg);
	}
}

/**
 * dwc3_free_one_event_buffer - Frees one event buffer
 * @dwc: Pointer to our controller context structure
 * @evt: Pointer to event buffer to be freed
 */
static void dwc3_free_one_event_buffer(struct dwc3 *dwc,
		struct dwc3_event_buffer *evt)
{
	dma_free_coherent(dwc->sysdev, evt->length, evt->buf, evt->dma);
}

/**
 * dwc3_alloc_one_event_buffer - Allocates one event buffer structure
 * @dwc: Pointer to our controller context structure
 * @length: size of the event buffer
 *
 * Returns a pointer to the allocated event buffer structure on success
 * otherwise ERR_PTR(errno).
 */
static struct dwc3_event_buffer *dwc3_alloc_one_event_buffer(struct dwc3 *dwc,
		unsigned length)
{
	struct dwc3_event_buffer	*evt;

	evt = devm_kzalloc(dwc->dev, sizeof(*evt), GFP_KERNEL);
	if (!evt)
		return ERR_PTR(-ENOMEM);

	evt->dwc	= dwc;
	evt->length	= length;
	evt->cache	= devm_kzalloc(dwc->dev, length, GFP_KERNEL);
	if (!evt->cache)
		return ERR_PTR(-ENOMEM);

	evt->buf	= dma_alloc_coherent(dwc->sysdev, length,
			&evt->dma, GFP_KERNEL);
	if (!evt->buf)
		return ERR_PTR(-ENOMEM);

	return evt;
}

/**
 * dwc3_free_event_buffers - frees all allocated event buffers
 * @dwc: Pointer to our controller context structure
 */
static void dwc3_free_event_buffers(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;

	evt = dwc->ev_buf;
	if (evt)
		dwc3_free_one_event_buffer(dwc, evt);
}

/**
 * dwc3_alloc_event_buffers - Allocates @num event buffers of size @length
 * @dwc: pointer to our controller context structure
 * @length: size of event buffer
 *
 * Returns 0 on success otherwise negative errno. In the error case, dwc
 * may contain some buffers allocated but not all which were requested.
 */
static int dwc3_alloc_event_buffers(struct dwc3 *dwc, unsigned length)
{
	struct dwc3_event_buffer *evt;

	evt = dwc3_alloc_one_event_buffer(dwc, length);
	if (IS_ERR(evt)) {
		dev_err(dwc->dev, "can't allocate event buffer\n");
		return PTR_ERR(evt);
	}
	dwc->ev_buf = evt;

	return 0;
}

/**
 * dwc3_event_buffers_setup - setup our allocated event buffers
 * @dwc: pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int dwc3_event_buffers_setup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;

	evt = dwc->ev_buf;
	evt->lpos = 0;
	dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(0),
			lower_32_bits(evt->dma));
	dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(0),
			upper_32_bits(evt->dma));
	dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(0),
			DWC3_GEVNTSIZ_SIZE(evt->length));
	dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(0), 0);

	return 0;
}

static void dwc3_event_buffers_cleanup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;

	evt = dwc->ev_buf;

	evt->lpos = 0;

	dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(0), 0);
	dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(0), 0);
	dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(0), DWC3_GEVNTSIZ_INTMASK
			| DWC3_GEVNTSIZ_SIZE(0));
	dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(0), 0);
}

static int dwc3_alloc_scratch_buffers(struct dwc3 *dwc)
{
	if (!dwc->has_hibernation)
		return 0;

	if (!dwc->nr_scratch)
		return 0;

	dwc->scratchbuf = kmalloc_array(dwc->nr_scratch,
			DWC3_SCRATCHBUF_SIZE, GFP_KERNEL);
	if (!dwc->scratchbuf)
		return -ENOMEM;

	return 0;
}

static int dwc3_setup_scratch_buffers(struct dwc3 *dwc)
{
	dma_addr_t scratch_addr;
	u32 param;
	int ret;

	if (!dwc->has_hibernation)
		return 0;

	if (!dwc->nr_scratch)
		return 0;

	 /* should never fall here */
	if (!WARN_ON(dwc->scratchbuf))
		return 0;

	scratch_addr = dma_map_single(dwc->sysdev, dwc->scratchbuf,
			dwc->nr_scratch * DWC3_SCRATCHBUF_SIZE,
			DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dwc->sysdev, scratch_addr)) {
		dev_err(dwc->sysdev, "failed to map scratch buffer\n");
		ret = -EFAULT;
		goto err0;
	}

	dwc->scratch_addr = scratch_addr;

	param = lower_32_bits(scratch_addr);

	ret = dwc3_send_gadget_generic_command(dwc,
			DWC3_DGCMD_SET_SCRATCHPAD_ADDR_LO, param);
	if (ret < 0)
		goto err1;

	param = upper_32_bits(scratch_addr);

	ret = dwc3_send_gadget_generic_command(dwc,
			DWC3_DGCMD_SET_SCRATCHPAD_ADDR_HI, param);
	if (ret < 0)
		goto err1;

	return 0;

err1:
	dma_unmap_single(dwc->sysdev, dwc->scratch_addr, dwc->nr_scratch *
			DWC3_SCRATCHBUF_SIZE, DMA_BIDIRECTIONAL);

err0:
	return ret;
}

static void dwc3_free_scratch_buffers(struct dwc3 *dwc)
{
	if (!dwc->has_hibernation)
		return;

	if (!dwc->nr_scratch)
		return;

	 /* should never fall here */
	if (!WARN_ON(dwc->scratchbuf))
		return;

	dma_unmap_single(dwc->sysdev, dwc->scratch_addr, dwc->nr_scratch *
			DWC3_SCRATCHBUF_SIZE, DMA_BIDIRECTIONAL);
	kfree(dwc->scratchbuf);
}

static void dwc3_core_num_eps(struct dwc3 *dwc)
{
	struct dwc3_hwparams	*parms = &dwc->hwparams;

	dwc->num_eps = DWC3_NUM_EPS(parms);
	dwc->num_in_eps = DWC3_NUM_IN_EPS(parms);
	dwc->num_out_eps = dwc->num_eps - dwc->num_in_eps;

	dev_info(dwc->dev, "found %d IN and %d OUT endpoints",
			dwc->num_in_eps, dwc->num_out_eps);
}

static void dwc3_cache_hwparams(struct dwc3 *dwc)
{
	struct dwc3_hwparams	*parms = &dwc->hwparams;

	parms->hwparams0 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS0);
	parms->hwparams1 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS1);
	parms->hwparams2 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS2);
	parms->hwparams3 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS3);
	parms->hwparams4 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS4);
	parms->hwparams5 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS5);
	parms->hwparams6 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS6);
	parms->hwparams7 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS7);
	parms->hwparams8 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS8);
}

static int dwc3_core_ulpi_init(struct dwc3 *dwc)
{
	int intf;
	int ret = 0;

	intf = DWC3_GHWPARAMS3_HSPHY_IFC(dwc->hwparams.hwparams3);

	if (intf == DWC3_GHWPARAMS3_HSPHY_IFC_ULPI ||
	    (intf == DWC3_GHWPARAMS3_HSPHY_IFC_UTMI_ULPI &&
	     dwc->hsphy_interface &&
	     !strncmp(dwc->hsphy_interface, "ulpi", 4)))
		ret = dwc3_ulpi_init(dwc);

	return ret;
}

/**
 * dwc3_phy_setup - Configure USB PHY Interface of DWC3 Core
 * @dwc: Pointer to our controller context structure
 *
 * Returns 0 on success. The USB PHY interfaces are configured but not
 * initialized. The PHY interfaces and the PHYs get initialized together with
 * the core in dwc3_core_init.
 */
static int dwc3_phy_setup(struct dwc3 *dwc)
{
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));

	/*
	 * Make sure UX_EXIT_PX is cleared as that causes issues with some
	 * PHYs. Also, this bit is not supposed to be used in normal operation.
	 */
	reg &= ~DWC3_GUSB3PIPECTL_UX_EXIT_PX;

	/*
	 * Above 1.94a, it is recommended to set DWC3_GUSB3PIPECTL_SUSPHY
	 * to '0' during coreConsultant configuration. So default value
	 * will be '0' when the core is reset. Application needs to set it
	 * to '1' after the core initialization is completed.
	 */
	if (dwc->revision > DWC3_REVISION_194A)
		reg |= DWC3_GUSB3PIPECTL_SUSPHY;

	if (dwc->u2ss_inp3_quirk)
		reg |= DWC3_GUSB3PIPECTL_U2SSINP3OK;

	if (dwc->dis_rxdet_inp3_quirk)
		reg |= DWC3_GUSB3PIPECTL_DISRXDETINP3;

	if (dwc->req_p1p2p3_quirk)
		reg |= DWC3_GUSB3PIPECTL_REQP1P2P3;

	if (dwc->del_p1p2p3_quirk)
		reg |= DWC3_GUSB3PIPECTL_DEP1P2P3_EN;

	if (dwc->del_phy_power_chg_quirk)
		reg |= DWC3_GUSB3PIPECTL_DEPOCHANGE;

	if (dwc->lfps_filter_quirk)
		reg |= DWC3_GUSB3PIPECTL_LFPSFILT;

	if (dwc->rx_detect_poll_quirk)
		reg |= DWC3_GUSB3PIPECTL_RX_DETOPOLL;

	if (dwc->tx_de_emphasis_quirk)
		reg |= DWC3_GUSB3PIPECTL_TX_DEEPH(dwc->tx_de_emphasis);

	if (dwc->dis_u3_susphy_quirk)
		reg &= ~DWC3_GUSB3PIPECTL_SUSPHY;

	if (dwc->dis_del_phy_power_chg_quirk)
		reg &= ~DWC3_GUSB3PIPECTL_DEPOCHANGE;

	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));

	/* Select the HS PHY interface */
	switch (DWC3_GHWPARAMS3_HSPHY_IFC(dwc->hwparams.hwparams3)) {
	case DWC3_GHWPARAMS3_HSPHY_IFC_UTMI_ULPI:
		if (dwc->hsphy_interface &&
				!strncmp(dwc->hsphy_interface, "utmi", 4)) {
			reg &= ~DWC3_GUSB2PHYCFG_ULPI_UTMI;
			break;
		} else if (dwc->hsphy_interface &&
				!strncmp(dwc->hsphy_interface, "ulpi", 4)) {
			reg |= DWC3_GUSB2PHYCFG_ULPI_UTMI;
			dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
		} else {
			/* Relying on default value. */
			if (!(reg & DWC3_GUSB2PHYCFG_ULPI_UTMI))
				break;
		}
		/* FALLTHROUGH */
	case DWC3_GHWPARAMS3_HSPHY_IFC_ULPI:
		/* FALLTHROUGH */
	default:
		break;
	}

	switch (dwc->hsphy_mode) {
	case USBPHY_INTERFACE_MODE_UTMI:
		reg &= ~(DWC3_GUSB2PHYCFG_PHYIF_MASK |
		       DWC3_GUSB2PHYCFG_USBTRDTIM_MASK);
		reg |= DWC3_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_8_BIT) |
		       DWC3_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_8_BIT);
		break;
	case USBPHY_INTERFACE_MODE_UTMIW:
		reg &= ~(DWC3_GUSB2PHYCFG_PHYIF_MASK |
		       DWC3_GUSB2PHYCFG_USBTRDTIM_MASK);
		reg |= DWC3_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_16_BIT) |
		       DWC3_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_16_BIT);
		break;
	default:
		break;
	}

	/*
	 * Above 1.94a, it is recommended to set DWC3_GUSB2PHYCFG_SUSPHY to
	 * '0' during coreConsultant configuration. So default value will
	 * be '0' when the core is reset. Application needs to set it to
	 * '1' after the core initialization is completed.
	 */
	if (dwc->revision > DWC3_REVISION_194A)
		reg |= DWC3_GUSB2PHYCFG_SUSPHY;

	if (dwc->dis_u2_susphy_quirk)
		reg &= ~DWC3_GUSB2PHYCFG_SUSPHY;

	if (dwc->dis_enblslpm_quirk)
		reg &= ~DWC3_GUSB2PHYCFG_ENBLSLPM;

	if (dwc->dis_u2_freeclk_exists_quirk)
		reg &= ~DWC3_GUSB2PHYCFG_U2_FREECLK_EXISTS;

	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);

	return 0;
}

static void dwc3_core_exit(struct dwc3 *dwc)
{
	dwc3_event_buffers_cleanup(dwc);

	usb_phy_shutdown(dwc->usb2_phy);
	usb_phy_shutdown(dwc->usb3_phy);
	phy_exit(dwc->usb2_generic_phy);
	phy_exit(dwc->usb3_generic_phy);

	usb_phy_set_suspend(dwc->usb2_phy, 1);
	usb_phy_set_suspend(dwc->usb3_phy, 1);
	phy_power_off(dwc->usb2_generic_phy);
	phy_power_off(dwc->usb3_generic_phy);
	clk_bulk_disable(dwc->num_clks, dwc->clks);
	clk_bulk_unprepare(dwc->num_clks, dwc->clks);
}

static bool dwc3_core_is_valid(struct dwc3 *dwc)
{
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_GSNPSID);

	/* This should read as U3 followed by revision number */
	if ((reg & DWC3_GSNPSID_MASK) == 0x55330000) {
		/* Detected DWC_usb3 IP */
		dwc->revision = reg;
	} else if ((reg & DWC3_GSNPSID_MASK) == 0x33310000) {
		/* Detected DWC_usb31 IP */
		dwc->revision = dwc3_readl(dwc->regs, DWC3_VER_NUMBER);
		dwc->revision |= DWC3_REVISION_IS_DWC31;
	} else {
		return false;
	}

	return true;
}

static void dwc3_core_setup_global_control(struct dwc3 *dwc)
{
	u32 hwparams4 = dwc->hwparams.hwparams4;
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~DWC3_GCTL_SCALEDOWN_MASK;

	switch (DWC3_GHWPARAMS1_EN_PWROPT(dwc->hwparams.hwparams1)) {
	case DWC3_GHWPARAMS1_EN_PWROPT_CLK:
		/**
		 * WORKAROUND: DWC3 revisions between 2.10a and 2.50a have an
		 * issue which would cause xHCI compliance tests to fail.
		 *
		 * Because of that we cannot enable clock gating on such
		 * configurations.
		 *
		 * Refers to:
		 *
		 * STAR#9000588375: Clock Gating, SOF Issues when ref_clk-Based
		 * SOF/ITP Mode Used
		 */
		if ((dwc->dr_mode == USB_DR_MODE_HOST ||
				dwc->dr_mode == USB_DR_MODE_OTG) &&
				(dwc->revision >= DWC3_REVISION_210A &&
				dwc->revision <= DWC3_REVISION_250A))
			reg |= DWC3_GCTL_DSBLCLKGTNG | DWC3_GCTL_SOFITPSYNC;
		else
			reg &= ~DWC3_GCTL_DSBLCLKGTNG;
		break;
	case DWC3_GHWPARAMS1_EN_PWROPT_HIB:
		/* enable hibernation here */
		dwc->nr_scratch = DWC3_GHWPARAMS4_HIBER_SCRATCHBUFS(hwparams4);

		/*
		 * REVISIT Enabling this bit so that host-mode hibernation
		 * will work. Device-mode hibernation is not yet implemented.
		 */
		reg |= DWC3_GCTL_GBLHIBERNATIONEN;
		break;
	default:
		/* nothing */
		break;
	}

	/* check if current dwc3 is on simulation board */
	if (dwc->hwparams.hwparams6 & DWC3_GHWPARAMS6_EN_FPGA) {
		dev_info(dwc->dev, "Running with FPGA optmizations\n");
		dwc->is_fpga = true;
	}

	WARN_ONCE(dwc->disable_scramble_quirk && !dwc->is_fpga,
			"disable_scramble cannot be used on non-FPGA builds\n");

	if (dwc->disable_scramble_quirk && dwc->is_fpga)
		reg |= DWC3_GCTL_DISSCRAMBLE;
	else
		reg &= ~DWC3_GCTL_DISSCRAMBLE;

	if (dwc->u2exit_lfps_quirk)
		reg |= DWC3_GCTL_U2EXIT_LFPS;

	/*
	 * WORKAROUND: DWC3 revisions <1.90a have a bug
	 * where the device can fail to connect at SuperSpeed
	 * and falls back to high-speed mode which causes
	 * the device to enter a Connect/Disconnect loop
	 */
	if (dwc->revision < DWC3_REVISION_190A)
		reg |= DWC3_GCTL_U2RSTECN;

	dwc3_writel(dwc->regs, DWC3_GCTL, reg);
}

static int dwc3_core_get_phy(struct dwc3 *dwc);
static int dwc3_core_ulpi_init(struct dwc3 *dwc);

/**
 * dwc3_core_init - Low-level initialization of DWC3 Core
 * @dwc: Pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int dwc3_core_init(struct dwc3 *dwc)
{
	u32			reg;
	int			ret;

	if (!dwc3_core_is_valid(dwc)) {
		dev_err(dwc->dev, "this is not a DesignWare USB3 DRD Core\n");
		ret = -ENODEV;
		goto err0;
	}

	/*
	 * Write Linux Version Code to our GUID register so it's easy to figure
	 * out which kernel version a bug was found.
	 */
	dwc3_writel(dwc->regs, DWC3_GUID, LINUX_VERSION_CODE);

	/* Handle USB2.0-only core configuration */
	if (DWC3_GHWPARAMS3_SSPHY_IFC(dwc->hwparams.hwparams3) ==
			DWC3_GHWPARAMS3_SSPHY_IFC_DIS) {
		if (dwc->maximum_speed == USB_SPEED_SUPER)
			dwc->maximum_speed = USB_SPEED_HIGH;
	}

	ret = dwc3_phy_setup(dwc);
	if (ret)
		goto err0;

	if (!dwc->ulpi_ready) {
		ret = dwc3_core_ulpi_init(dwc);
		if (ret)
			goto err0;
		dwc->ulpi_ready = true;
	}

	if (!dwc->phys_ready) {
		ret = dwc3_core_get_phy(dwc);
		if (ret)
			goto err0a;
		dwc->phys_ready = true;
	}

	ret = dwc3_core_soft_reset(dwc);
	if (ret)
		goto err0a;

	dwc3_core_setup_global_control(dwc);
	dwc3_core_num_eps(dwc);

	ret = dwc3_setup_scratch_buffers(dwc);
	if (ret)
		goto err1;

	/* Adjust Frame Length */
	dwc3_frame_length_adjustment(dwc);

	usb_phy_set_suspend(dwc->usb2_phy, 0);
	usb_phy_set_suspend(dwc->usb3_phy, 0);
	ret = phy_power_on(dwc->usb2_generic_phy);
	if (ret < 0)
		goto err2;

	ret = phy_power_on(dwc->usb3_generic_phy);
	if (ret < 0)
		goto err3;

	ret = dwc3_event_buffers_setup(dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to setup event buffers\n");
		goto err4;
	}

	/*
	 * ENDXFER polling is available on version 3.10a and later of
	 * the DWC_usb3 controller. It is NOT available in the
	 * DWC_usb31 controller.
	 */
	if (!dwc3_is_usb31(dwc) && dwc->revision >= DWC3_REVISION_310A) {
		reg = dwc3_readl(dwc->regs, DWC3_GUCTL2);
		reg |= DWC3_GUCTL2_RST_ACTBITLATER;
		dwc3_writel(dwc->regs, DWC3_GUCTL2, reg);
	}

	if (dwc->revision >= DWC3_REVISION_250A) {
		reg = dwc3_readl(dwc->regs, DWC3_GUCTL1);

		/*
		 * Enable hardware control of sending remote wakeup
		 * in HS when the device is in the L1 state.
		 */
		if (dwc->revision >= DWC3_REVISION_290A)
			reg |= DWC3_GUCTL1_DEV_L1_EXIT_BY_HW;

		if (dwc->dis_tx_ipgap_linecheck_quirk)
			reg |= DWC3_GUCTL1_TX_IPGAP_LINECHECK_DIS;

		dwc3_writel(dwc->regs, DWC3_GUCTL1, reg);
	}

	return 0;

err4:
	phy_power_off(dwc->usb3_generic_phy);

err3:
	phy_power_off(dwc->usb2_generic_phy);

err2:
	usb_phy_set_suspend(dwc->usb2_phy, 1);
	usb_phy_set_suspend(dwc->usb3_phy, 1);

err1:
	usb_phy_shutdown(dwc->usb2_phy);
	usb_phy_shutdown(dwc->usb3_phy);
	phy_exit(dwc->usb2_generic_phy);
	phy_exit(dwc->usb3_generic_phy);

err0a:
	dwc3_ulpi_exit(dwc);

err0:
	return ret;
}

static int dwc3_core_get_phy(struct dwc3 *dwc)
{
	struct device		*dev = dwc->dev;
	struct device_node	*node = dev->of_node;
	int ret;

	if (node) {
		dwc->usb2_phy = devm_usb_get_phy_by_phandle(dev, "usb-phy", 0);
		dwc->usb3_phy = devm_usb_get_phy_by_phandle(dev, "usb-phy", 1);
	} else {
		dwc->usb2_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
		dwc->usb3_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB3);
	}

	if (IS_ERR(dwc->usb2_phy)) {
		ret = PTR_ERR(dwc->usb2_phy);
		if (ret == -ENXIO || ret == -ENODEV) {
			dwc->usb2_phy = NULL;
		} else if (ret == -EPROBE_DEFER) {
			return ret;
		} else {
			dev_err(dev, "no usb2 phy configured\n");
			return ret;
		}
	}

	if (IS_ERR(dwc->usb3_phy)) {
		ret = PTR_ERR(dwc->usb3_phy);
		if (ret == -ENXIO || ret == -ENODEV) {
			dwc->usb3_phy = NULL;
		} else if (ret == -EPROBE_DEFER) {
			return ret;
		} else {
			dev_err(dev, "no usb3 phy configured\n");
			return ret;
		}
	}

	dwc->usb2_generic_phy = devm_phy_get(dev, "usb2-phy");
	if (IS_ERR(dwc->usb2_generic_phy)) {
		ret = PTR_ERR(dwc->usb2_generic_phy);
		if (ret == -ENOSYS || ret == -ENODEV) {
			dwc->usb2_generic_phy = NULL;
		} else if (ret == -EPROBE_DEFER) {
			return ret;
		} else {
			dev_err(dev, "no usb2 phy configured\n");
			return ret;
		}
	}

	dwc->usb3_generic_phy = devm_phy_get(dev, "usb3-phy");
	if (IS_ERR(dwc->usb3_generic_phy)) {
		ret = PTR_ERR(dwc->usb3_generic_phy);
		if (ret == -ENOSYS || ret == -ENODEV) {
			dwc->usb3_generic_phy = NULL;
		} else if (ret == -EPROBE_DEFER) {
			return ret;
		} else {
			dev_err(dev, "no usb3 phy configured\n");
			return ret;
		}
	}

	return 0;
}

static int dwc3_core_init_mode(struct dwc3 *dwc)
{
	struct device *dev = dwc->dev;
	int ret;

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		dwc3_set_prtcap(dwc, DWC3_GCTL_PRTCAP_DEVICE);

		if (dwc->usb2_phy)
			otg_set_vbus(dwc->usb2_phy->otg, false);
		if (dwc->usb2_generic_phy)
			phy_set_mode(dwc->usb2_generic_phy, PHY_MODE_USB_DEVICE);

		ret = dwc3_gadget_init(dwc);
		if (ret) {
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "failed to initialize gadget\n");
			return ret;
		}
		break;
	case USB_DR_MODE_HOST:
		dwc3_set_prtcap(dwc, DWC3_GCTL_PRTCAP_HOST);

		if (dwc->usb2_phy)
			otg_set_vbus(dwc->usb2_phy->otg, true);
		if (dwc->usb2_generic_phy)
			phy_set_mode(dwc->usb2_generic_phy, PHY_MODE_USB_HOST);

		ret = dwc3_host_init(dwc);
		if (ret) {
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "failed to initialize host\n");
			return ret;
		}
		break;
	case USB_DR_MODE_OTG:
		INIT_WORK(&dwc->drd_work, __dwc3_set_mode);
		ret = dwc3_drd_init(dwc);
		if (ret) {
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "failed to initialize dual-role\n");
			return ret;
		}
		break;
	default:
		dev_err(dev, "Unsupported mode of operation %d\n", dwc->dr_mode);
		return -EINVAL;
	}

	return 0;
}

static void dwc3_core_exit_mode(struct dwc3 *dwc)
{
	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		dwc3_gadget_exit(dwc);
		break;
	case USB_DR_MODE_HOST:
		dwc3_host_exit(dwc);
		break;
	case USB_DR_MODE_OTG:
		dwc3_drd_exit(dwc);
		break;
	default:
		/* do nothing */
		break;
	}
}

static void dwc3_get_properties(struct dwc3 *dwc)
{
	struct device		*dev = dwc->dev;
	u8			lpm_nyet_threshold;
	u8			tx_de_emphasis;
	u8			hird_threshold;

	/* default to highest possible threshold */
	lpm_nyet_threshold = 0xff;

	/* default to -3.5dB de-emphasis */
	tx_de_emphasis = 1;

	/*
	 * default to assert utmi_sleep_n and use maximum allowed HIRD
	 * threshold value of 0b1100
	 */
	hird_threshold = 12;

	dwc->maximum_speed = usb_get_maximum_speed(dev);
	dwc->dr_mode = usb_get_dr_mode(dev);
	dwc->hsphy_mode = of_usb_get_phy_mode(dev->of_node);

	dwc->sysdev_is_parent = device_property_read_bool(dev,
				"linux,sysdev_is_parent");
	if (dwc->sysdev_is_parent)
		dwc->sysdev = dwc->dev->parent;
	else
		dwc->sysdev = dwc->dev;

	dwc->has_lpm_erratum = device_property_read_bool(dev,
				"snps,has-lpm-erratum");
	device_property_read_u8(dev, "snps,lpm-nyet-threshold",
				&lpm_nyet_threshold);
	dwc->is_utmi_l1_suspend = device_property_read_bool(dev,
				"snps,is-utmi-l1-suspend");
	device_property_read_u8(dev, "snps,hird-threshold",
				&hird_threshold);
	dwc->usb3_lpm_capable = device_property_read_bool(dev,
				"snps,usb3_lpm_capable");

	dwc->disable_scramble_quirk = device_property_read_bool(dev,
				"snps,disable_scramble_quirk");
	dwc->u2exit_lfps_quirk = device_property_read_bool(dev,
				"snps,u2exit_lfps_quirk");
	dwc->u2ss_inp3_quirk = device_property_read_bool(dev,
				"snps,u2ss_inp3_quirk");
	dwc->req_p1p2p3_quirk = device_property_read_bool(dev,
				"snps,req_p1p2p3_quirk");
	dwc->del_p1p2p3_quirk = device_property_read_bool(dev,
				"snps,del_p1p2p3_quirk");
	dwc->del_phy_power_chg_quirk = device_property_read_bool(dev,
				"snps,del_phy_power_chg_quirk");
	dwc->lfps_filter_quirk = device_property_read_bool(dev,
				"snps,lfps_filter_quirk");
	dwc->rx_detect_poll_quirk = device_property_read_bool(dev,
				"snps,rx_detect_poll_quirk");
	dwc->dis_u3_susphy_quirk = device_property_read_bool(dev,
				"snps,dis_u3_susphy_quirk");
	dwc->dis_u2_susphy_quirk = device_property_read_bool(dev,
				"snps,dis_u2_susphy_quirk");
	dwc->dis_enblslpm_quirk = device_property_read_bool(dev,
				"snps,dis_enblslpm_quirk");
	dwc->dis_rxdet_inp3_quirk = device_property_read_bool(dev,
				"snps,dis_rxdet_inp3_quirk");
	dwc->dis_u2_freeclk_exists_quirk = device_property_read_bool(dev,
				"snps,dis-u2-freeclk-exists-quirk");
	dwc->dis_del_phy_power_chg_quirk = device_property_read_bool(dev,
				"snps,dis-del-phy-power-chg-quirk");
	dwc->dis_tx_ipgap_linecheck_quirk = device_property_read_bool(dev,
				"snps,dis-tx-ipgap-linecheck-quirk");

	dwc->tx_de_emphasis_quirk = device_property_read_bool(dev,
				"snps,tx_de_emphasis_quirk");
	device_property_read_u8(dev, "snps,tx_de_emphasis",
				&tx_de_emphasis);
	device_property_read_string(dev, "snps,hsphy_interface",
				    &dwc->hsphy_interface);
	device_property_read_u32(dev, "snps,quirk-frame-length-adjustment",
				 &dwc->fladj);

	dwc->lpm_nyet_threshold = lpm_nyet_threshold;
	dwc->tx_de_emphasis = tx_de_emphasis;

	dwc->hird_threshold = hird_threshold
		| (dwc->is_utmi_l1_suspend << 4);

	dwc->imod_interval = 0;
}

/* check whether the core supports IMOD */
bool dwc3_has_imod(struct dwc3 *dwc)
{
	return ((dwc3_is_usb3(dwc) &&
		 dwc->revision >= DWC3_REVISION_300A) ||
		(dwc3_is_usb31(dwc) &&
		 dwc->revision >= DWC3_USB31_REVISION_120A));
}

static void dwc3_check_params(struct dwc3 *dwc)
{
	struct device *dev = dwc->dev;

	/* Check for proper value of imod_interval */
	if (dwc->imod_interval && !dwc3_has_imod(dwc)) {
		dev_warn(dwc->dev, "Interrupt moderation not supported\n");
		dwc->imod_interval = 0;
	}

	/*
	 * Workaround for STAR 9000961433 which affects only version
	 * 3.00a of the DWC_usb3 core. This prevents the controller
	 * interrupt from being masked while handling events. IMOD
	 * allows us to work around this issue. Enable it for the
	 * affected version.
	 */
	if (!dwc->imod_interval &&
	    (dwc->revision == DWC3_REVISION_300A))
		dwc->imod_interval = 1;

	/* Check the maximum_speed parameter */
	switch (dwc->maximum_speed) {
	case USB_SPEED_LOW:
	case USB_SPEED_FULL:
	case USB_SPEED_HIGH:
	case USB_SPEED_SUPER:
	case USB_SPEED_SUPER_PLUS:
		break;
	default:
		dev_err(dev, "invalid maximum_speed parameter %d\n",
			dwc->maximum_speed);
		/* fall through */
	case USB_SPEED_UNKNOWN:
		/* default to superspeed */
		dwc->maximum_speed = USB_SPEED_SUPER;

		/*
		 * default to superspeed plus if we are capable.
		 */
		if (dwc3_is_usb31(dwc) &&
		    (DWC3_GHWPARAMS3_SSPHY_IFC(dwc->hwparams.hwparams3) ==
		     DWC3_GHWPARAMS3_SSPHY_IFC_GEN2))
			dwc->maximum_speed = USB_SPEED_SUPER_PLUS;

		break;
	}
}

static ssize_t dwc3_role_show(struct device *dev,
		       struct device_attribute *attr, char *buf)
{
	struct dwc3	*dwc = dev_get_drvdata(dev);
	unsigned long	flags;
	u32		reg;
	int r = 0;

	spin_lock_irqsave(&dwc->lock, flags);
	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	spin_unlock_irqrestore(&dwc->lock, flags);

	switch (DWC3_GCTL_PRTCAP(reg)) {
	case DWC3_GCTL_PRTCAP_HOST:
		r = sprintf(buf, "%s\n", "host");
		break;
	case DWC3_GCTL_PRTCAP_DEVICE:
		r = sprintf(buf, "%s\n", "device");
		break;
	case DWC3_GCTL_PRTCAP_OTG:
		r = sprintf(buf, "%s\n", "otg");
		break;
	default:
		r = sprintf(buf, "UNKNOWN %08x\n", DWC3_GCTL_PRTCAP(reg));
	}

	return r;
}

static ssize_t dwc3_role_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct dwc3	*dwc = dev_get_drvdata(dev);
	u32		role = 0;

	if (!strncmp(buf, "host", 4))
		role = DWC3_GCTL_PRTCAP_HOST;

	if (!strncmp(buf, "device", 6))
		role = DWC3_GCTL_PRTCAP_DEVICE;

	if (!strncmp(buf, "otg", 3))
		role = DWC3_GCTL_PRTCAP_OTG;

	if (role)
		dwc3_set_mode(dwc, role);

	return count;
}

static DEVICE_ATTR(role, 0644, dwc3_role_show, dwc3_role_store);

static struct attribute *dwc3_attrs[] = {
	&dev_attr_role.attr,
	NULL,
};

static struct attribute_group dwc3_attr_group = {
	.attrs = dwc3_attrs,
};

static void dwc3_sysfs_init(struct dwc3 *dwc)
{
	int ret;

	ret = sysfs_create_group(&dwc->dev->kobj, &dwc3_attr_group);
}

static void dwc3_sysfs_exit(struct dwc3 *dwc)
{
	sysfs_remove_group(&dwc->dev->kobj, &dwc3_attr_group);
}

#ifdef HOBOT_TEST_REGISTER_RW
static void hobot_usb_register_test(struct dwc3 *dwc)
{
	u32			reg;
	unsigned int		i;
	volatile unsigned int	*reg_ptr;

	// Register R/W test
	reg = dwc3_readl(dwc->regs, DWC3_GUID);
	printk(">>> usb_test: read register[DWC3_GUID] = 0x%x, and write to 0x5555AAAA\n", reg);
	reg = 0x5555AAAA;
	dwc3_writel(dwc->regs, DWC3_GUID, reg);
	reg = dwc3_readl(dwc->regs, DWC3_GUID);
	printk(">>> usb_test: read register[DWC3_GUID] = 0x%x\n", reg);

	printk(">>> usb_test: regs = 0x%lx\n", (long) dwc->regs);
	reg_ptr = dwc->regs;
	for (i = 0; i < (4 * 20); i += 4) {
		printk(">>> register (0x%04x): 0x%08x 0x%08x 0x%08x 0x%08x\n",
				0xC100 + i * 4,
				*(reg_ptr + i),
				*(reg_ptr + i + 1),
				*(reg_ptr + i + 2),
				*(reg_ptr + i + 3));
	}

	printk(">>> usb_test: regs_sys = 0x%lx\n", (long) dwc->regs_sys);
	reg_ptr = dwc->regs_sys + USB3_CTRL_REG0;
	for (i = 0; i < (4 * 3); i += 4) {
		printk(">>> register: 0x%08x 0x%08x 0x%08x 0x%08x\n",
				*(reg_ptr + i),
				*(reg_ptr + i + 1),
				*(reg_ptr + i + 2),
				*(reg_ptr + i + 3));
	}
}
#endif

#ifdef HOBOT_FPGA_TEST_TIMING_FINETUNE
#define FPGA_TIMING_FINETUNE_SHIFT			  29
#define FPGA_TIMING_FINETUNE_MASK			  ((0x7) << FPGA_TIMING_FINETUNE_SHIFT)
#define FPGA_TIMING_FINETUNE_SHIFT_DEGREE(n)  (n / 45)

static void hobot_phy_finetune_timing(struct dwc3 *dwc)
{
	unsigned int		field_value;
	volatile unsigned int	*reg_ptr;

	reg_ptr = dwc->regs_sys + USB3_CTRL_REG0;

	// For Host/Device Clock Shift: Non/45/90/135/180/225/270/315
	if (IS_ENABLED(CONFIG_USB_DWC3_HOST))
		field_value = FPGA_TIMING_FINETUNE_SHIFT_DEGREE(270);
	else if (IS_ENABLED(CONFIG_USB_DWC3_GADGET))
		field_value = FPGA_TIMING_FINETUNE_SHIFT_DEGREE(0);

	printk(">>> [FPGA] Clock Shift: %d", field_value * 45);
	*reg_ptr = (*reg_ptr & ~FPGA_TIMING_FINETUNE_MASK) | (field_value << FPGA_TIMING_FINETUNE_SHIFT);
}
#endif

#ifdef HOBOT_FPGA_USING_M31_PHY
static void hobot_usb_set_mode(struct dwc3 *dwc)
{
	volatile unsigned int	*reg_ptr;

	//Setting for USB Host/Device
	reg_ptr = dwc->regs_sys + USB3_CTRL_REG0;
	if (IS_ENABLED(CONFIG_USB_DWC3_HOST))
		*reg_ptr = (*reg_ptr & ~BUS_FILTER_BYPASS_MASK) | BUS_FILTER_BYPASS_HOST;
	else if (IS_ENABLED(CONFIG_USB_DWC3_GADGET))
		*reg_ptr = (*reg_ptr & ~BUS_FILTER_BYPASS_MASK) | BUS_FILTER_BYPASS_DEVICE;
}

static void hobot_m31_phy_reset(struct dwc3 *dwc)
{
	// For Phy Reset
	volatile unsigned int	*reg_ptr;

	reg_ptr = dwc->regs_sys + USB3_PHY_REG2;
	// TODO: [ASIC] need to modify the RESET code
	*reg_ptr = 0x80000000;
	*reg_ptr = (*reg_ptr & ~PHY_RESET_MASK) | PHY_RESET;
	printk(">>> Register PHY_REG2: 0x%x\n", *reg_ptr);
	// Wait for 1 us? Need to confirm.
	printk(">>> Register PHY_REG2: Write to 0\n");
	*reg_ptr = (*reg_ptr & ~PHY_RESET_MASK) | ~PHY_RESET;
	printk(">>> Register PHY_REG2: 0x%x\n", *reg_ptr);
}
#endif

#ifdef HOBOT_SOC_DWC3_DEBUG
static void hobot_dwc3_info_register(struct dwc3 *dwc)
{
	u32 reg;

	/* dwc3 register */
	reg = dwc3_readl(dwc->regs, DWC3_GSBUSCFG0);
	dev_info(dwc->dev, "DWC3_GSBUSCFG0: 0x%x", reg);

	reg = dwc3_readl(dwc->regs, DWC3_GSBUSCFG1);
	dev_info(dwc->dev, "DWC3_GSBUSCFG1: 0x%x", reg);

	reg = dwc3_readl(dwc->regs, DWC3_GTXTHRCFG);
	dev_info(dwc->dev, "DWC3_GTXTHRCFG: 0x%x", reg);

	reg = dwc3_readl(dwc->regs, DWC3_GRXTHRCFG);
	dev_info(dwc->dev, "DWC3_GRXTHRCFG: 0x%x", reg);

	reg = dwc3_readl(dwc->regs, DWC3_GSNPSID);
	dev_info(dwc->dev, "DWC3_GSNPSID: 0x%x", reg);

	reg = dwc3_readl(dwc->regs, DWC3_GUID);
	dev_info(dwc->dev, "DWC3_GUID: 0x%x", reg);

	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
	dev_info(dwc->dev, "DWC3_GUSB2PHYCFG(0): 0x%x", reg);

	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
	dev_info(dwc->dev, "DWC3_GUSB3PIPECTL(0): 0x%x", reg);

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	dev_info(dwc->dev, "DWC3_GCTL: 0x%x", reg);

	reg = dwc3_readl(dwc->regs, DWC3_GUCTL);
	dev_info(dwc->dev, "DWC3_GUCTL: 0x%x", reg);

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	dev_info(dwc->dev, "DWC3_DCTL: 0x%x", reg);

	/* sysctl register */
	reg = readl(dwc->regs_sys + USB3_PHY_REG0);
	dev_info(dwc->dev, "USB3_PHY_REG0: 0x%x", reg);

	reg = readl(dwc->regs_sys + USB3_CTRL_REG0);
	dev_info(dwc->dev, "USB3_CTRL_REG0: 0x%x", reg);

	reg = readl(dwc->regs_sys + CPUSYS_SW_RSTEN);
	dev_info(dwc->dev, "CPUSYS_SW_RSTEN: 0x%x", reg);
}
#endif

#ifdef HOBOT_SOC_DWC3_SETTING
static void hobot_usb_reset_register(struct dwc3 *dwc) {
	u32 reg;

	/* sysctl register */
	reg = readl(dwc->regs_sys + USB3_CTRL_REG0);
	reg = 0x200f;
	writel(reg, dwc->regs_sys + USB3_CTRL_REG0);

	reg = readl(dwc->regs_sys + USB3_PHY_REG0);
	reg &= ~RX_LOS_LFPSFILT;
	writel(reg, dwc->regs_sys + USB3_PHY_REG0);

	/* dwc3 register */
	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
	// reg = 0x40105408;
	reg = 0x40101400;
	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0),reg);

	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
	// reg = 0x01000002;
	reg = 0x00000002;
	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0),reg);

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg = 0x30c11004;
	dwc3_writel(dwc->regs, DWC3_GCTL,reg);
}
#endif

#ifdef HOBOT_SOC_USB_RESET
static void hobot_usb_sw_reset(struct dwc3* dwc) {
	u32 reg;

	reg = readl(dwc->regs_sys + CPUSYS_SW_RSTEN);
	reg |= SYS_USB_RSTEN;
	writel(reg, dwc->regs_sys + CPUSYS_SW_RSTEN);
	reg &= ~SYS_USB_RSTEN;
	writel(reg, dwc->regs_sys + CPUSYS_SW_RSTEN);
}

static void hobot_usb_reset(struct dwc3 *dwc)
{
	hobot_usb_sw_reset(dwc);

#ifdef HOBOT_SOC_DWC3_SETTING
	/* reset some dwc3/sysctl registers */
	hobot_usb_reset_register(dwc);
#endif
}
#endif

static int dwc3_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct resource		*res;
	struct resource		*res_sys;
	struct dwc3		*dwc;

	int			ret;

	void __iomem		*regs;
	void __iomem		*regs_sys;
	int			i;
	unsigned long		clkrate;

	dwc = devm_kzalloc(dev, sizeof(*dwc), GFP_KERNEL);
	if (!dwc)
		return -ENOMEM;

	dwc->clks = devm_kmemdup(dev, dwc3_core_clks, sizeof(dwc3_core_clks),
				 GFP_KERNEL);
	if (!dwc->clks)
		return -ENOMEM;

	dwc->num_clks = ARRAY_SIZE(dwc3_core_clks);
	dwc->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "missing memory resource\n");
		return -ENODEV;
	}
	res_sys = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res_sys) {
		dev_err(dev, "missing memory resource\n");
		return -ENODEV;
	}

	dwc->xhci_resources[0].start = res->start;
	dwc->xhci_resources[0].end = dwc->xhci_resources[0].start +
					DWC3_XHCI_REGS_END;
	dwc->xhci_resources[0].flags = res->flags;
	dwc->xhci_resources[0].name = res->name;

	res->start += DWC3_GLOBALS_REGS_START;

	/*
	 * Request memory region but exclude xHCI regs,
	 * since it will be requested by the xhci-plat driver.
	 */
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs)) {
		ret = PTR_ERR(regs);
		goto err0;
	}

	regs_sys = devm_ioremap(dev, res_sys->start, resource_size(res_sys));
	if (IS_ERR(regs_sys)) {
		ret = PTR_ERR(regs_sys);
		goto err0;
	}

	dwc->regs	= regs;
	dwc->regs_size	= resource_size(res);

	dwc->regs_sys	= regs_sys;
	dwc->regs_sys_size	= resource_size(res_sys);

#ifdef HOBOT_TEST_REGISTER_RW
	hobot_usb_register_test(dwc);
#endif

#ifdef HOBOT_FPGA_TEST_TIMING_FINETUNE
	hobot_phy_finetune_timing(dwc);
#endif

#ifdef HOBOT_FPGA_USING_M31_PHY
	hobot_usb_set_mode(dwc);
	hobot_m31_phy_reset(dwc);
#endif

#ifdef HOBOT_SOC_USB_RESET
	// hobot_dwc3_info_register(dwc);
	hobot_usb_reset(dwc);
	// hobot_dwc3_info_register(dwc);
#endif

#ifdef HOBOT_DDR_DFS_ENABLE
	/* get & enable regulator */
	dwc->regulator = devm_regulator_get(dev, "usb_0v8");

	if (IS_ERR(dwc->regulator)) { /* PRQA S ALL */
		/* some platform not has regulator, so just report error info */
		dwc->regulator = NULL;
		dev_err(dev, "can't get usb regulator\n");
	}

	if (dwc->regulator) {
		ret = regulator_enable(dwc->regulator);
		if (ret) {
			dev_err(dev, "usb regulator enable error\n");
			goto regulator_fail;
		}
		dev_info(dev, "usb regulator enable succeed\n");
	}

	/* register ddr dfs notifier */
	dwc->ddrfreq_nb.notifier_call = ddr_dfs_call;
	ret = hb_bus_register_client(&dwc->ddrfreq_nb);
	if (ret)
		goto notifier_err1;

	/* register powersave notifier */
	dwc->powersave_nb.notifier_call = power_save_call;
	ret = hb_usb_register_client(&dwc->powersave_nb);
	if (ret)
		goto notifier_err2;
#endif

	dwc3_get_properties(dwc);

	ret = clk_bulk_get(dev, dwc->num_clks, dwc->clks);
	if (ret == -EPROBE_DEFER)
		goto get_clks;

	/*
	 * Clocks are optional, but new DT platforms should support all clocks
	 * as required by the DT-binding.
	 */
	if (ret)
		dwc->num_clks = 0;

	ret = clk_bulk_prepare(dwc->num_clks, dwc->clks);
	if (ret)
		goto put_clks;

	ret = clk_bulk_enable(dwc->num_clks, dwc->clks);
	if (ret)
		goto unprepare_clks;

	dev_dbg(dev, "%d clock enabled, info clock rate:\n", dwc->num_clks);
	for (i = 0; i < dwc->num_clks; i++) {
		clkrate = clk_get_rate(dwc->clks[i].clk);
		dev_dbg(dev, "<%d>clock[%s] rate: %lu\n",
				i, dwc->clks[i].id, clkrate);
	}

	platform_set_drvdata(pdev, dwc);
	dwc3_cache_hwparams(dwc);

	spin_lock_init(&dwc->lock);

	pm_runtime_set_active(dev);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, DWC3_DEFAULT_AUTOSUSPEND_DELAY);
	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		goto err1;

	pm_runtime_forbid(dev);

	ret = dwc3_alloc_event_buffers(dwc, DWC3_EVENT_BUFFERS_SIZE);
	if (ret) {
		dev_err(dwc->dev, "failed to allocate event buffers\n");
		ret = -ENOMEM;
		goto err2;
	}

	ret = dwc3_get_dr_mode(dwc);
	if (ret)
		goto err3;

	ret = dwc3_alloc_scratch_buffers(dwc);
	if (ret)
		goto err3;

	ret = dwc3_core_init(dwc);
	if (ret) {
		dev_err(dev, "failed to initialize core\n");
		goto err4;
	}

	dwc3_check_params(dwc);

	ret = dwc3_core_init_mode(dwc);
	if (ret)
		goto err5;

	dwc3_debugfs_init(dwc);
	pm_runtime_put(dev);

	dwc3_sysfs_init(dwc);

	return 0;

err5:
	dwc3_event_buffers_cleanup(dwc);

err4:
	dwc3_free_scratch_buffers(dwc);

err3:
	dwc3_free_event_buffers(dwc);

err2:
	pm_runtime_allow(&pdev->dev);

err1:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	clk_bulk_disable(dwc->num_clks, dwc->clks);
unprepare_clks:
	clk_bulk_unprepare(dwc->num_clks, dwc->clks);
put_clks:
	clk_bulk_put(dwc->num_clks, dwc->clks);
get_clks:
	hb_usb_unregister_client(&dwc->powersave_nb);
notifier_err2:
	hb_bus_unregister_client(&dwc->ddrfreq_nb);
notifier_err1:
	if (dwc->regulator) {
		ret = regulator_disable(dwc->regulator);
		if (ret)
			dev_err(dev, "usb regulator disable error\n");
	}
regulator_fail:
err0:
	/*
	 * restore res->start back to its original value so that, in case the
	 * probe is deferred, we don't end up getting error in request the
	 * memory region the next time probe is called.
	 */
	res->start -= DWC3_GLOBALS_REGS_START;

	return ret;
}

static int dwc3_remove(struct platform_device *pdev)
{
	struct dwc3	*dwc = platform_get_drvdata(pdev);
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	pm_runtime_get_sync(&pdev->dev);
	/*
	 * restore res->start back to its original value so that, in case the
	 * probe is deferred, we don't end up getting error in request the
	 * memory region the next time probe is called.
	 */
	res->start -= DWC3_GLOBALS_REGS_START;

	dwc3_debugfs_exit(dwc);
	dwc3_sysfs_exit(dwc);
	dwc3_core_exit_mode(dwc);

	dwc3_core_exit(dwc);
	dwc3_ulpi_exit(dwc);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_allow(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	dwc3_free_event_buffers(dwc);
	dwc3_free_scratch_buffers(dwc);
	clk_bulk_put(dwc->num_clks, dwc->clks);

#ifdef HOBOT_DDR_DFS_ENABLE
	/* unregister power save notifier */
	hb_usb_unregister_client(&dwc->powersave_nb);

	/* unregister ddr dfs notifier */
	hb_bus_unregister_client(&dwc->ddrfreq_nb);

	if (dwc->regulator) {
		if (regulator_disable(dwc->regulator))
			dev_err(dwc->dev, "usb regulator disable error\n");
	}
#endif

	return 0;
}

#ifdef CONFIG_PM
static int dwc3_suspend_common(struct dwc3 *dwc, pm_message_t msg)
{
	unsigned long	flags;
	u32 reg;

	switch (dwc->current_dr_role) {
	case DWC3_GCTL_PRTCAP_DEVICE:
		/* pm runtime suspend has problem, to be done in next version */
#if 0
		if (pm_runtime_suspend(dwc->dev))
			break;
#endif
		spin_lock_irqsave(&dwc->lock, flags);
		dwc3_gadget_suspend(dwc);
		spin_unlock_irqrestore(&dwc->lock, flags);
		synchronize_irq(dwc->irq_gadget);
		dwc3_core_exit(dwc);
		break;
	case DWC3_GCTL_PRTCAP_HOST:
		if (!PMSG_IS_AUTO(msg)) {
			dwc3_core_exit(dwc);
			break;
		}

		/* Let controller to suspend HSPHY before PHY driver suspends */
		if (dwc->dis_u2_susphy_quirk ||
				dwc->dis_enblslpm_quirk) {
			reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
			reg |= DWC3_GUSB2PHYCFG_ENBLSLPM |
				DWC3_GUSB2PHYCFG_SUSPHY;
			dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);

			/* Give some time for USB2 PHY to suspend */
			usleep_range(5000, 6000);
		}

		phy_pm_runtime_put_sync(dwc->usb2_generic_phy);
		phy_pm_runtime_put_sync(dwc->usb3_generic_phy);
		break;
	case DWC3_GCTL_PRTCAP_OTG:
		// TODO: currently just support static drd
		dev_err(dwc->dev, "otg to be done, currently just support static drd\n");
		break;
	default:
		/* do nothing */
		break;
	}

	return 0;
}

static int dwc3_resume_common(struct dwc3 *dwc, pm_message_t msg)
{
	unsigned long	flags;
	int		ret;
	u32		reg;

	ret = clk_bulk_prepare(dwc->num_clks, dwc->clks);
	if (ret)
		return ret;

	ret = clk_bulk_enable(dwc->num_clks, dwc->clks);
	if (ret)
		return ret;

	switch (dwc->current_dr_role) {
	case DWC3_GCTL_PRTCAP_DEVICE:
		ret = dwc3_core_init(dwc);
		if (ret)
			return ret;

		dwc3_set_prtcap(dwc, DWC3_GCTL_PRTCAP_DEVICE);
		spin_lock_irqsave(&dwc->lock, flags);
		dwc3_gadget_resume(dwc);
		spin_unlock_irqrestore(&dwc->lock, flags);
		break;
	case DWC3_GCTL_PRTCAP_HOST:
		if (!PMSG_IS_AUTO(msg)) {
			ret = dwc3_core_init(dwc);
			if (ret)
				return ret;
			dwc3_set_prtcap(dwc, DWC3_GCTL_PRTCAP_HOST);
			break;
		}

		/* Restore GUSB2PHYCFG bits that were modified in suspend */
		reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
		if (dwc->dis_u2_susphy_quirk)
			reg &= ~DWC3_GUSB2PHYCFG_SUSPHY;

		if (dwc->dis_enblslpm_quirk)
			reg &= ~DWC3_GUSB2PHYCFG_ENBLSLPM;

		dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);

		phy_pm_runtime_get_sync(dwc->usb2_generic_phy);
		phy_pm_runtime_get_sync(dwc->usb3_generic_phy);
		break;
	case DWC3_GCTL_PRTCAP_OTG:
		// TODO: currently just support static drd
		dev_err(dwc->dev, "otg to be done, currently just support static drd\n");
		break;
	default:
		/* do nothing */
		break;
	}

	return 0;
}

static int dwc3_runtime_checks(struct dwc3 *dwc)
{
	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
	case USB_DR_MODE_OTG:
		if (dwc->connected)
			return -EBUSY;
		break;
	case USB_DR_MODE_HOST:
	default:
		/* do nothing */
		break;
	}

	return 0;
}

static int dwc3_runtime_suspend(struct device *dev)
{
	struct dwc3     *dwc = dev_get_drvdata(dev);
	int		ret;

	if (dwc3_runtime_checks(dwc))
		return -EBUSY;

	ret = dwc3_suspend_common(dwc, PMSG_AUTO_SUSPEND);
	if (ret)
		return ret;

	device_init_wakeup(dev, true);

	return 0;
}

static int dwc3_runtime_resume(struct device *dev)
{
	struct dwc3     *dwc = dev_get_drvdata(dev);
	int		ret;

	device_init_wakeup(dev, false);

	ret = dwc3_resume_common(dwc, PMSG_AUTO_RESUME);
	if (ret)
		return ret;

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
	case USB_DR_MODE_OTG:
		dwc3_gadget_process_pending_events(dwc);
		break;
	case USB_DR_MODE_HOST:
	default:
		/* do nothing */
		break;
	}

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put(dev);

	return 0;
}

static int dwc3_runtime_idle(struct device *dev)
{
	struct dwc3     *dwc = dev_get_drvdata(dev);

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
	case USB_DR_MODE_OTG:
		if (dwc3_runtime_checks(dwc))
			return -EBUSY;
		break;
	case USB_DR_MODE_HOST:
	default:
		/* do nothing */
		break;
	}

	pm_runtime_mark_last_busy(dev);
	pm_runtime_autosuspend(dev);

	return 0;
}
#endif /* CONFIG_PM */

#ifdef CONFIG_PM_SLEEP
static int dwc3_suspend(struct device *dev)
{
	struct dwc3	*dwc = dev_get_drvdata(dev);
	int		ret;

	ret = dwc3_suspend_common(dwc, PMSG_SUSPEND);
	if (ret)
		return ret;

	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int dwc3_resume(struct device *dev)
{
	struct dwc3	*dwc = dev_get_drvdata(dev);
	int		ret;

	pinctrl_pm_select_default_state(dev);

	ret = dwc3_resume_common(dwc, PMSG_RESUME);
	if (ret)
		return ret;

	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops dwc3_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_suspend, dwc3_resume)
	SET_RUNTIME_PM_OPS(dwc3_runtime_suspend, dwc3_runtime_resume,
			dwc3_runtime_idle)
};

#ifdef CONFIG_OF
static const struct of_device_id of_dwc3_match[] = {
	{
		.compatible = "hobot,usb"
	},
	{ },
};
MODULE_DEVICE_TABLE(of, of_dwc3_match);
#endif

#ifdef CONFIG_ACPI

#define ACPI_ID_INTEL_BSW	"808622B7"

static const struct acpi_device_id dwc3_acpi_match[] = {
	{ ACPI_ID_INTEL_BSW, 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, dwc3_acpi_match);
#endif

static struct platform_driver dwc3_driver = {
	.probe		= dwc3_probe,
	.remove		= dwc3_remove,
	.driver		= {
		.name	= "usb",
		.of_match_table	= of_match_ptr(of_dwc3_match),
		.acpi_match_table = ACPI_PTR(dwc3_acpi_match),
		.pm	= &dwc3_dev_pm_ops,
	},
};

module_platform_driver(dwc3_driver);

MODULE_ALIAS("platform:dwc3");
MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 DRD Controller Driver");
