/* xhci-exynos.c - Driver for USB HOST on Samsung EXYNOS platform device
 *
 * Bus Glue for SAMSUNG EXYNOS USB HOST xHCI Controller
 * xHCI host controller driver
 *
 * Copyright (C) 2008 Intel Corp.
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 *
 * Author: Yulgon Kim <yulgon.kim@samsung.com>
 *
 * Based on "xhci-pci.c" by Sarah Sharp
 * Modified for SAMSUNG EXYNOS XHCI by Yulgon Kim <yulgon.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <linux/usb/otg.h>
#include <linux/usb/exynos_usb3_drd.h>
#include <linux/platform_data/dwc3-exynos.h>

#include "xhci.h"

struct exynos_xhci_hcd {
	struct device		*dev;
	struct dwc3_exynos_data	*pdata;
	struct usb_hcd		*hcd;
	struct exynos_drd_core	*core;
	struct clk		*clk;
	int			irq;
};

struct xhci_hcd *exynos_xhci_dbg;

static const char hcd_name[] = "xhci_hcd";

#ifdef CONFIG_PM
static int exynos_xhci_suspend(struct device *dev)
{
	struct exynos_xhci_hcd	*exynos_xhci;
	struct usb_hcd		*hcd;
	struct xhci_hcd		*xhci;
	int			retval = 0;

	exynos_xhci = dev_get_drvdata(dev);
	if (!exynos_xhci)
		return -EINVAL;

	hcd = exynos_xhci->hcd;
	if (!hcd)
		return -EINVAL;

	xhci = hcd_to_xhci(hcd);

	if (hcd->state != HC_STATE_SUSPENDED ||
			xhci->shared_hcd->state != HC_STATE_SUSPENDED)
		return -EINVAL;

	if (pm_runtime_suspended(dev))
		return 0;

	retval = xhci_suspend(xhci);

	if (exynos_xhci->core->ops->change_mode)
		exynos_xhci->core->ops->change_mode(exynos_xhci->core, false);

	return retval;
}

static int exynos_xhci_resume(struct device *dev)
{
	struct exynos_xhci_hcd	*exynos_xhci;
	struct usb_hcd		*hcd;
	struct xhci_hcd		*xhci;
	int			retval = 0;

	exynos_xhci = dev_get_drvdata(dev);
	if (!exynos_xhci)
		return -EINVAL;

	hcd = exynos_xhci->hcd;
	if (!hcd)
		return -EINVAL;

	pm_runtime_resume(dev);

	if (exynos_xhci->core->ops->change_mode)
		exynos_xhci->core->ops->change_mode(exynos_xhci->core, true);

	xhci = hcd_to_xhci(hcd);
	retval = xhci_resume(xhci, 0);

	/* Update runtime PM status and clear runtime_error */
	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return retval;
}
#else
#define exynos_xhci_suspend	NULL
#define exynos_xhci_resume	NULL
#endif

#ifdef CONFIG_USB_SUSPEND
static int exynos_xhci_runtime_suspend(struct device *dev)
{
	struct exynos_xhci_hcd	*exynos_xhci;
	struct usb_hcd		*hcd;
	struct xhci_hcd		*xhci;
	int			retval = 0;

	dev_dbg(dev, "%s\n", __func__);

	exynos_xhci = dev_get_drvdata(dev);
	if (!exynos_xhci)
		return -EINVAL;

	hcd = exynos_xhci->hcd;
	if (!hcd)
		return -EINVAL;

	xhci = hcd_to_xhci(hcd);

	if (hcd->state != HC_STATE_SUSPENDED ||
			xhci->shared_hcd->state != HC_STATE_SUSPENDED)
		return -EINVAL;

	retval = xhci_suspend(xhci);

	pm_runtime_put_sync(exynos_xhci->dev->parent);

	return retval;
}

static int exynos_xhci_runtime_resume(struct device *dev)
{
	struct exynos_xhci_hcd	*exynos_xhci;
	struct usb_hcd		*hcd;
	struct xhci_hcd		*xhci;
	int			retval = 0;

	dev_dbg(dev, "%s\n", __func__);

	exynos_xhci = dev_get_drvdata(dev);
	if (!exynos_xhci)
		return -EINVAL;

	hcd = exynos_xhci->hcd;
	if (!hcd)
		return -EINVAL;

	if (dev->power.is_suspended)
		return 0;

	pm_runtime_get_sync(exynos_xhci->dev->parent);

	if (exynos_xhci->core->ops->change_mode)
		exynos_xhci->core->ops->change_mode(exynos_xhci->core, true);

	if (exynos_xhci->core->ops->core_init)
		exynos_xhci->core->ops->core_init(exynos_xhci->core);

	xhci = hcd_to_xhci(hcd);
	retval = xhci_resume(xhci, 0);

	return retval;
}
#else
#define exynos_xhci_runtime_suspend	NULL
#define exynos_xhci_runtime_resume	NULL
#endif

#ifdef CONFIG_PM
int exynos_xhci_bus_resume(struct usb_hcd *hcd)
{
	/* When suspend is failed, re-enable clocks & PHY */
	pm_runtime_resume(hcd->self.controller);

	return xhci_bus_resume(hcd);
}
#else
#define exynos_xhci_bus_resume NULL
#endif

static void exynos_xhci_quirks(struct device *dev, struct xhci_hcd *xhci)
{
	/* Don't use MSI interrupt */
	xhci->quirks |= XHCI_BROKEN_MSI;
}

/* called during probe() after chip reset completes */
static int exynos_xhci_setup(struct usb_hcd *hcd)
{
	struct device		*dev = hcd->self.controller;
	struct exynos_xhci_hcd	*exynos_xhci = dev_get_drvdata(dev);
	struct xhci_hcd		*xhci;
	int			retval;

	retval = xhci_gen_setup(hcd, exynos_xhci_quirks);
	if (retval)
		return retval;

	/*
	 * During xhci_gen_setup() GSBUSCFG0 DRD register resets (detected by
	 * experiment). We need to configure it again here.
	 */
	if (exynos_xhci->core->ops->config)
		exynos_xhci->core->ops->config(exynos_xhci->core);

	xhci = hcd_to_xhci(hcd);
	if (!usb_hcd_is_primary_hcd(hcd))
		return 0;

	xhci->sbrn = HCD_USB3;
	xhci_dbg(xhci, "Got SBRN %u\n", (unsigned int) xhci->sbrn);

	return retval;
}


static const struct hc_driver exynos_xhci_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "EXYNOS xHCI Host Controller",
	.hcd_priv_size		= sizeof(struct xhci_hcd *),

	/*
	 * generic hardware linkage
	 */
	.irq			= xhci_irq,
	.flags			= HCD_MEMORY | HCD_USB3 | HCD_SHARED,

	/*
	 * basic lifecycle operations
	 */
	.reset			= exynos_xhci_setup,
	.start			= xhci_run,
	.stop			= xhci_stop,
	.shutdown		= xhci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue		= xhci_urb_enqueue,
	.urb_dequeue		= xhci_urb_dequeue,
	.alloc_dev		= xhci_alloc_dev,
	.free_dev		= xhci_free_dev,
	.alloc_streams		= xhci_alloc_streams,
	.free_streams		= xhci_free_streams,
	.add_endpoint		= xhci_add_endpoint,
	.drop_endpoint		= xhci_drop_endpoint,
	.endpoint_reset		= xhci_endpoint_reset,
	.check_bandwidth	= xhci_check_bandwidth,
	.reset_bandwidth	= xhci_reset_bandwidth,
	.address_device		= xhci_address_device,
	.update_hub_device	= xhci_update_hub_device,
	.reset_device		= xhci_discover_or_reset_device,

	/*
	 * scheduling support
	 */
	.get_frame_number	= xhci_get_frame,

	/* Root hub support */
	.hub_control		= xhci_hub_control,
	.hub_status_data	= xhci_hub_status_data,
	.bus_suspend		= xhci_bus_suspend,
	.bus_resume		= exynos_xhci_bus_resume,
};

static int usb_hcd_exynos_probe(struct platform_device *pdev,
				const struct hc_driver *driver)
{
	struct exynos_xhci_hcd	*exynos_xhci;
	struct dwc3_exynos_data	*pdata = pdev->dev.platform_data;
	struct device		*dev = &pdev->dev;
	struct usb_hcd		*hcd;
	struct resource		*res;
	int			err;

	if (usb_disabled())
		return -ENODEV;

	if (!driver)
		return -EINVAL;

	if (!pdata) {
		dev_err(dev, "No platform data defined\n");
		return -EINVAL;
	}

	exynos_xhci = devm_kzalloc(dev, sizeof(struct exynos_xhci_hcd),
				   GFP_KERNEL);
	if (!exynos_xhci)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Failed to get I/O memory\n");
		return -ENXIO;
	}

	if (!devm_request_mem_region(dev, res->start, resource_size(res),
				dev_name(dev))) {
		dev_err(dev, "Failed to reserve registers\n");
		return -ENOENT;
	}

	hcd = usb_create_hcd(driver, dev, dev_name(dev));
	if (!hcd)
		return -ENOMEM;

	/* EHCI, OHCI */
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	hcd->regs = devm_ioremap_nocache(dev, res->start, resource_size(res));
	if (!hcd->regs) {
		dev_err(dev, "Failed to remap I/O memory\n");
		err = -ENOMEM;
		goto fail_io;
	}
	hcd->regs -= EXYNOS_USB3_XHCI_REG_START;

	exynos_xhci->irq = platform_get_irq(pdev, 0);
	if (!exynos_xhci->irq) {
		dev_err(dev, "Failed to get IRQ\n");
		err = -ENODEV;
		goto fail_io;
	}

	exynos_xhci->dev = dev;
	exynos_xhci->pdata = pdata;
	exynos_xhci->hcd = hcd;
	exynos_xhci->core = exynos_drd_bind(pdev);
	platform_set_drvdata(pdev, exynos_xhci);

	pm_runtime_get_sync(exynos_xhci->dev->parent);

	if (exynos_xhci->core->ops->change_mode)
		exynos_xhci->core->ops->change_mode(exynos_xhci->core, true);
	if (exynos_xhci->core->ops->core_init)
		exynos_xhci->core->ops->core_init(exynos_xhci->core);

	err = usb_add_hcd(hcd, exynos_xhci->irq, IRQF_DISABLED | IRQF_SHARED);
	if (err) {
		dev_err(dev, "Failed to add USB HCD\n");
		goto fail_io;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return err;

fail_io:
	usb_put_hcd(hcd);
	return err;
}

void usb_hcd_exynos_remove(struct platform_device *pdev)
{
	struct exynos_xhci_hcd	*exynos_xhci;
	struct usb_hcd		*hcd;

	exynos_xhci = dev_get_drvdata(&pdev->dev);
	hcd = exynos_xhci->hcd;
	if (!hcd)
		return;

	pm_runtime_disable(&pdev->dev);

	/* Fake an interrupt request in order to give the driver a chance
	 * to test whether the controller hardware has been removed (e.g.,
	 * cardbus physical eject).
	 */
	local_irq_disable();
	usb_hcd_irq(0, hcd);
	local_irq_enable();

	usb_remove_hcd(hcd);

	if (exynos_xhci->core->ops->change_mode)
		exynos_xhci->core->ops->change_mode(exynos_xhci->core, false);

	usb_put_hcd(hcd);
}

static int __devinit exynos_xhci_probe(struct platform_device *pdev)
{
	struct exynos_xhci_hcd	*exynos_xhci;
	struct usb_hcd		*hcd;
	struct xhci_hcd		*xhci;
	int			err;

	/* Register the USB 2.0 roothub.
	 * FIXME: USB core must know to register the USB 2.0 roothub first.
	 * This is sort of silly, because we could just set the HCD driver flags
	 * to say USB 2.0, but I'm not sure what the implications would be in
	 * the other parts of the HCD code.
	 */
	err = usb_hcd_exynos_probe(pdev, &exynos_xhci_hc_driver);
	if (err)
		return err;

	exynos_xhci = dev_get_drvdata(&pdev->dev);

	hcd = exynos_xhci->hcd;
	xhci = hcd_to_xhci(hcd);
	xhci->shared_hcd = usb_create_shared_hcd(&exynos_xhci_hc_driver,
				&pdev->dev, dev_name(&pdev->dev), hcd);
	if (!xhci->shared_hcd) {
		dev_err(&pdev->dev, "Unable to create HCD\n");
		err = -ENOMEM;
		goto dealloc_usb2_hcd;
	}

	xhci->shared_hcd->regs = hcd->regs;
	exynos_xhci_dbg = xhci;

	/*
	 * Set the xHCI pointer before exynos_xhci_setup()
	 * (aka hcd_driver.reset) is called by usb_add_hcd().
	 */
	*((struct xhci_hcd **) xhci->shared_hcd->hcd_priv) = xhci;

	err = usb_add_hcd(xhci->shared_hcd, exynos_xhci->irq,
			IRQF_DISABLED | IRQF_SHARED);
	if (err)
		goto put_usb3_hcd;

	if (exynos_xhci->core->otg) {
		err = otg_set_host(exynos_xhci->core->otg, &hcd->self);
		if (err) {
			dev_err(&pdev->dev, "Unable to bind hcd to DRD switch\n");
			goto put_usb3_hcd;
		}
	}

	/* Roothub already marked as USB 3.0 speed */

	return 0;

put_usb3_hcd:
	usb_put_hcd(xhci->shared_hcd);
dealloc_usb2_hcd:
	usb_hcd_exynos_remove(pdev);

	return err;
}

static int __devexit exynos_xhci_remove(struct platform_device *pdev)
{
	struct exynos_xhci_hcd	*exynos_xhci = platform_get_drvdata(pdev);
	struct usb_hcd		*hcd = exynos_xhci->hcd;
	struct xhci_hcd		*xhci;

	if (exynos_xhci->core->otg)
		otg_set_host(exynos_xhci->core->otg, NULL);

	xhci = hcd_to_xhci(hcd);
	if (xhci->shared_hcd) {
		usb_remove_hcd(xhci->shared_hcd);
		usb_put_hcd(xhci->shared_hcd);
	}

	usb_hcd_exynos_remove(pdev);
	kfree(xhci);

	return 0;
}

static void exynos_xhci_shutdown(struct platform_device *pdev)
{
	struct exynos_xhci_hcd	*s5p_xhci = platform_get_drvdata(pdev);
	struct usb_hcd		*hcd = s5p_xhci->hcd;

	if (hcd->driver->shutdown)
		hcd->driver->shutdown(hcd);
}

static const struct dev_pm_ops exynos_xhci_pm_ops = {
	.suspend		= exynos_xhci_suspend,
	.resume			= exynos_xhci_resume,
	.runtime_suspend	= exynos_xhci_runtime_suspend,
	.runtime_resume		= exynos_xhci_runtime_resume,
};

static struct platform_driver exynos_xhci_driver = {
	.probe		= exynos_xhci_probe,
	.remove		= __devexit_p(exynos_xhci_remove),
	.shutdown	= exynos_xhci_shutdown,
	.driver = {
		.name	= "exynos-xhci",
		.owner	= THIS_MODULE,
		.pm = &exynos_xhci_pm_ops,
	}
};

int xhci_register_exynos(void)
{
	return platform_driver_register(&exynos_xhci_driver);
}

void xhci_unregister_exynos(void)
{
	platform_driver_unregister(&exynos_xhci_driver);
}

MODULE_ALIAS("platform:exynos-xhci");
