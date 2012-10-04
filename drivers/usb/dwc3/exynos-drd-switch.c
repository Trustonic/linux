/**
 * exynos-drd-switch.c - Exynos SuperSpeed USB 3.0 DRD role switch
 *
 * This driver implements the otg final state machine and controls the
 * activation of the device controller or host controller. The ID pin
 * GPIO interrupt is used for this purpose. The VBus GPIO is used to
 * detect valid B-Session in B-Device mode.
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Anton Tikhomirov <av.tikhomirov@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/usb/hcd.h>
#include <linux/usb/gadget.h>
#include <linux/usb/exynos_usb3_drd.h>
#include <linux/platform_data/dwc3-exynos.h>

#include "exynos-drd.h"
#include "exynos-drd-switch.h"
#include "xhci.h"

static inline enum id_pin_state exynos_drd_switch_sanitize_id(int state)
{
	enum id_pin_state id_state;

	if (state < 0)
		id_state = NA;
	else
		id_state = state ? B_DEV : A_DEV;

	return id_state;
}

/**
 * exynos_drd_switch_get_id_state - get connector ID state.
 *
 * @drd_switch: Pointer to the DRD switch structure.
 *
 * Returns ID pin state.
 */
static enum id_pin_state
exynos_drd_switch_get_id_state(struct exynos_drd_switch *drd_switch)
{
	struct exynos_drd *drd = container_of(drd_switch->core,
						struct exynos_drd, core);
	struct platform_device *pdev = to_platform_device(drd->dev);
	struct dwc3_exynos_data *pdata = drd->pdata;
	int state;
	enum id_pin_state id_state;

	if (pdata->get_id_state)
		state = pdata->get_id_state(pdev);
	else
		state = -1;

	id_state = exynos_drd_switch_sanitize_id(state);

	return id_state;
}

/**
 * exynos_drd_switch_get_bses_vld - get B-Session status.
 *
 * @drd_switch: Pointer to the DRD switch structure.
 *
 * Returns true if B-Session is valid, false otherwise.
 */
static bool exynos_drd_switch_get_bses_vld(struct exynos_drd_switch *drd_switch)
{
	struct exynos_drd *drd = container_of(drd_switch->core,
						struct exynos_drd, core);
	struct platform_device *pdev = to_platform_device(drd->dev);
	struct dwc3_exynos_data *pdata = drd->pdata;
	bool valid;

	if (pdata->get_bses_vld)
		valid = pdata->get_bses_vld(pdev);
	else
		/* If VBus sensing is not available we always return true */
		valid = true;

	return valid;
}

/**
 * exynos_drd_switch_ases_vbus_ctrl - turn on VBus switch in A-Device mode.
 *
 * @drd_switch: Pointer to the DRD switch structure.
 * @on: turn on / turn off VBus switch.
 */
static void
exynos_drd_switch_ases_vbus_ctrl(struct exynos_drd_switch *drd_switch, int on)
{
	struct exynos_drd *drd = container_of(drd_switch->core,
						struct exynos_drd, core);
	struct platform_device *pdev = to_platform_device(drd->dev);
	struct dwc3_exynos_data *pdata = drd->pdata;

	if (on) {
		if (pdata->vbus_ctrl)
			pdata->vbus_ctrl(pdev, 1);
	} else {
		if (pdata->vbus_ctrl)
			pdata->vbus_ctrl(pdev, 0);
	}
}

/**
 * exynos_drd_switch_schedule_work - schedule OTG state machine work.
 *
 * @work: Work to schedule.
 *
 * Prevents state machine running if ID state is N/A. Use this function
 * to schedule work.
 */
static void exynos_drd_switch_schedule_work(struct work_struct *work)
{
	struct exynos_drd_switch *drd_switch;

	if (work) {
		drd_switch = container_of(work, struct exynos_drd_switch, work);

		if (drd_switch->id_state != NA)
			schedule_work(work);
	}
}

/**
 * exynos_drd_switch_is_host_off - check if host's runtime pm status.
 *
 * @otg: Pointer to the usb_otg structure.
 */
static bool exynos_drd_switch_is_host_off(struct usb_otg *otg)
{
	struct usb_hcd *hcd;
	struct device *dev;

	if (!otg->host)
		/* REVISIT: what should we return here? */
		return true;

	hcd = bus_to_hcd(otg->host);
	dev = hcd->self.controller;

	return pm_runtime_suspended(dev);
}

/**
 * exynos_drd_switch_start_host -  helper function for starting/stoping the host
 * controller driver.
 *
 * @otg: Pointer to the usb_otg structure.
 * @on: start / stop the host controller driver.
 *
 * Returns 0 on success otherwise negative errno.
 */
static int exynos_drd_switch_start_host(struct usb_otg *otg, int on)
{
	struct exynos_drd_switch *drd_switch = container_of(otg,
					struct exynos_drd_switch, otg);
	struct usb_hcd *hcd;
	struct xhci_hcd *xhci;
	struct device *xhci_dev;
	int ret = 0;

	dev_dbg(otg->phy->dev, "%s: turn %s host %s\n",
			__func__, on ? "on" : "off", otg->host->bus_name);

	if (!otg->host)
		return -EINVAL;

	hcd = bus_to_hcd(otg->host);
	xhci = hcd_to_xhci(hcd);
	xhci_dev = hcd->self.controller;

	if (on) {
		ret = pm_runtime_get_sync(xhci_dev);
		if (ret < 0) {
			dev_dbg(otg->phy->dev, "%s: host resume failed (%d)\n",
						__func__, ret);
			goto err;
		}

		exynos_drd_switch_ases_vbus_ctrl(drd_switch, 1);
	} else {
		exynos_drd_switch_ases_vbus_ctrl(drd_switch, 0);

		ret = pm_runtime_put_sync(xhci_dev);
		if (ret < 0) {
			dev_dbg(otg->phy->dev, "%s: host suspend failed (%d)\n",
						__func__, ret);
			goto err;
		}
	}

err:
	/* ret can be 1 after pm_runtime_get_sync */
	return (ret < 0) ? ret : 0;
}

/**
 * exynos_drd_switch_set_host -  bind/unbind the host controller driver.
 *
 * @otg: Pointer to the usb_otg structure.
 * @host: Pointer to the usb_bus structure.
 *
 * Returns 0 on success otherwise negative errno.
 */
static int exynos_drd_switch_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct exynos_drd_switch *drd_switch = container_of(otg,
					struct exynos_drd_switch, otg);

	if (host) {
		dev_dbg(otg->phy->dev, "%s: binding host %s\n",
					__func__, host->bus_name);
		otg->host = host;

		/*
		 * Prevents unnecessary activation of the work function.
		 * If both peripheral and host are set or if ID pin is low
		 * then we ensure that work function will enter to valid state.
		 */
		if (otg->gadget || drd_switch->id_state == A_DEV)
			exynos_drd_switch_schedule_work(&drd_switch->work);
	} else {
		dev_dbg(otg->phy->dev, "%s: set unbinding host\n", __func__);

		if (otg->phy->state == OTG_STATE_A_HOST) {
			exynos_drd_switch_start_host(otg, 0);
			otg->host = NULL;
			otg->phy->state = OTG_STATE_UNDEFINED;
			exynos_drd_switch_schedule_work(&drd_switch->work);
		} else {
			otg->host = NULL;
		}
	}

	return 0;
}

/**
 * exynos_drd_switch_start_peripheral -  bind/unbind the peripheral controller.
 *
 * @otg: Pointer to the usb_otg structure.
 * @on: start / stop the gadget controller driver.
 *
 * Returns 0 on success otherwise negative errno.
 */
static int exynos_drd_switch_start_peripheral(struct usb_otg *otg, int on)
{
	dev_dbg(otg->phy->dev, "%s: turn %s gadget %s\n",
			__func__, on ? "on" : "off", otg->host->bus_name);

	if (!otg->gadget)
		return -EINVAL;

	if (on) {
		/* Start device only if host is off */
		if (!exynos_drd_switch_is_host_off(otg))
			return -EAGAIN;

		usb_gadget_vbus_connect(otg->gadget);
		/* TODO: handle return value here */
	} else {
		usb_gadget_vbus_disconnect(otg->gadget);
		/* TODO: handle return value here */
	}

	return 0;
}

/**
 * exynos_drd_switch_set_peripheral -  bind/unbind the peripheral controller driver.
 *
 * @otg: Pointer to the usb_otg structure.
 * @gadget: pointer to the usb_gadget structure.
 *
 * Returns 0 on success otherwise negative errno.
 */
static int exynos_drd_switch_set_peripheral(struct usb_otg *otg,
				struct usb_gadget *gadget)
{
	struct exynos_drd_switch *drd_switch = container_of(otg,
					struct exynos_drd_switch, otg);

	if (gadget) {
		dev_dbg(otg->phy->dev, "%s: binding gadget %s\n",
					__func__, gadget->name);
		otg->gadget = gadget;

		/*
		 * Prevents unnecessary activation of the work function.
		 * If both peripheral and host are set or if ID pin is high
		 * then we ensure that work function will enter to valid state.
		 */
		if (otg->host || drd_switch->id_state == B_DEV)
			exynos_drd_switch_schedule_work(&drd_switch->work);
	} else {
		dev_dbg(otg->phy->dev, "%s: unbinding gadget\n", __func__);

		if (otg->phy->state == OTG_STATE_B_PERIPHERAL) {
			exynos_drd_switch_start_peripheral(otg, 0);
			otg->gadget = NULL;
			otg->phy->state = OTG_STATE_UNDEFINED;
			exynos_drd_switch_schedule_work(&drd_switch->work);
		} else {
			otg->gadget = NULL;
		}
	}

	return 0;
}

/**
 * exynos_drd_switch_debounce - GPIO debounce timer handler.
 *
 * @data: Pointer to DRD switch structure represented as unsigned long.
 */
static void exynos_drd_switch_debounce(unsigned long data)
{
	struct exynos_drd_switch *drd_switch =
				(struct exynos_drd_switch *) data;
	struct usb_phy *phy = drd_switch->otg.phy;

	exynos_drd_switch_schedule_work(&drd_switch->work);
	dev_dbg(phy->dev, "id: %d, vbus: %d\n",
		drd_switch->id_state, drd_switch->vbus_active ? 1 : 0);
}

/**
 * exynos_drd_switch_handle_vbus - handle VBus state.
 *
 * @drd_switch: Pointer to the DRD switch structure.
 * @vbus_active: VBus state, true if active, false otherwise.
 */
static void exynos_drd_switch_handle_vbus(struct exynos_drd_switch *drd_switch,
					  bool vbus_active)
{
	/* REVISIT: handle VBus Change Event only in B-device mode */
	if (drd_switch->id_state != B_DEV)
		return;

	if (vbus_active != drd_switch->vbus_active) {
		drd_switch->vbus_active = vbus_active;
		/*
		 * Debouncing: timer will not expire untill
		 * gpio is stable.
		 */
		mod_timer(&drd_switch->vbus_db_timer,
				jiffies + VBUS_DEBOUNCE_DELAY);
	}
}

/**
 * exynos_drd_switch_handle_id - handle ID pin state.
 *
 * @drd_switch: Pointer to the DRD switch structure.
 * @id_state: ID pin state.
 */
static void exynos_drd_switch_handle_id(struct exynos_drd_switch *drd_switch,
					enum id_pin_state id_state)
{
	if (id_state != drd_switch->id_state) {
		drd_switch->id_state = id_state;
		if ((drd_switch->otg.phy->state == OTG_STATE_B_IDLE) ||
		    (drd_switch->otg.phy->state == OTG_STATE_A_IDLE)) {

			/*
			 * OTG state is ABOUT to change to A or B device, but
			 * since ID sts was changed, then we return the state
			 * machine to the start point.
			 */
			 drd_switch->otg.phy->state = OTG_STATE_UNDEFINED;
		}
		/*
		 * Debouncing: timer will not expire untill
		 * ID state is stable.
		 */
		mod_timer(&drd_switch->id_db_timer,
				jiffies + ID_DEBOUNCE_DELAY);
	}
}

/**
 * exynos_drd_switch_vbus_interrupt - interrupt handler for VBUS GPIO.
 *
 * @irq: irq number.
 * @_drdsw: Pointer to DRD switch structure.
 */
static irqreturn_t exynos_drd_switch_vbus_interrupt(int irq, void *_drdsw)
{
	struct exynos_drd_switch *drd_switch =
				(struct exynos_drd_switch *)_drdsw;
	bool vbus_active;

	vbus_active = exynos_drd_switch_get_bses_vld(drd_switch);

	exynos_drd_switch_handle_vbus(drd_switch, vbus_active);

	return IRQ_HANDLED;
}

/**
 * exynos_drd_switch_id_interrupt - interrupt handler for ID GPIO.
 *
 * @irq: irq number.
 * @_drdsw: Pointer to DRD switch structure.
 */
static irqreturn_t exynos_drd_switch_id_interrupt(int irq, void *_drdsw)
{
	struct exynos_drd_switch *drd_switch =
				(struct exynos_drd_switch *)_drdsw;
	enum id_pin_state id_state;

	/*
	 * ID sts has changed, read it and later, in the workqueue
	 * function, switch from A to B or from B to A.
	 */
	id_state = exynos_drd_switch_get_id_state(drd_switch);

	exynos_drd_switch_handle_id(drd_switch, id_state);

	return IRQ_HANDLED;
}

/**
 * exynos_drd_switch_vbus_event - receive VBus change event.
 *
 * @pdev: DRD that receives the event.
 * @vbus_active: New VBus state, true if active, false otherwise.
 *
 * Other drivers may use this function to tell the role switch driver
 * about the VBus change.
 */
int exynos_drd_switch_vbus_event(struct platform_device *pdev, bool vbus_active)
{
	struct exynos_drd *drd;
	struct usb_otg *otg;
	struct exynos_drd_switch *drd_switch;

	drd = platform_get_drvdata(pdev);
	if (!drd)
		return -ENOENT;

	otg = drd->core.otg;
	if (!otg)
		return -ENOENT;

	drd_switch = container_of(otg, struct exynos_drd_switch, otg);

	exynos_drd_switch_handle_vbus(drd_switch, vbus_active);

	return 0;
}

/**
 * exynos_drd_switch_id_event - receive ID pin state change event.
 *
 * @pdev: DRD that receives the event.
 * @state: New ID pin state.
 *
 * Other drivers may use this function to tell the role switch driver
 * about the ID pin state change.
 */
int exynos_drd_switch_id_event(struct platform_device *pdev, int state)
{
	struct exynos_drd *drd;
	struct usb_otg *otg;
	struct exynos_drd_switch *drd_switch;
	enum id_pin_state id_state;

	drd = platform_get_drvdata(pdev);
	if (!drd)
		return -ENOENT;

	otg = drd->core.otg;
	if (!otg)
		return -ENOENT;

	drd_switch = container_of(otg, struct exynos_drd_switch, otg);

	id_state = exynos_drd_switch_sanitize_id(state);

	exynos_drd_switch_handle_id(drd_switch, id_state);

	return 0;
}

/**
 * exynos_drd_switch_work - work function.
 *
 * @w: Pointer to the exynos otg work structure.
 *
 * NOTE: After any change in phy->state,
 * we must reschdule the state machine.
 */
static void exynos_drd_switch_work(struct work_struct *w)
{
	struct exynos_drd_switch *drd_switch = container_of(w,
					struct exynos_drd_switch, work);
	struct usb_phy *phy = drd_switch->otg.phy;
	struct work_struct *work = &drd_switch->work;
	int ret = 0;

	/* provide serialization */

	mutex_lock(&drd_switch->mutex);

	/* Check OTG state */
	switch (phy->state) {
	case OTG_STATE_UNDEFINED:
		/* Switch to A or B-Device according to ID state */
		if (drd_switch->id_state == B_DEV)
			phy->state = OTG_STATE_B_IDLE;
		else if (drd_switch->id_state == A_DEV)
			phy->state = OTG_STATE_A_IDLE;

		exynos_drd_switch_schedule_work(work);
		break;
	case OTG_STATE_B_IDLE:
		if (drd_switch->vbus_active) {
			/* Start peripheral only if B-Session is valid */
			ret = exynos_drd_switch_start_peripheral(
							&drd_switch->otg, 1);
			if (!ret) {
				phy->state = OTG_STATE_B_PERIPHERAL;
				exynos_drd_switch_schedule_work(work);
			} else if (ret == -EAGAIN) {
				phy->state = OTG_STATE_UNDEFINED;
				exynos_drd_switch_schedule_work(work);
			} else {
				/* Fatal error */
				dev_err(phy->dev,
					"unable to start B-device\n");
			}
		} else {
			dev_dbg(phy->dev, "VBus is not active\n");
		}
		break;
	case OTG_STATE_B_PERIPHERAL:
		dev_dbg(phy->dev, "OTG_STATE_B_PERIPHERAL\n");
		if (drd_switch->id_state == A_DEV) {
			exynos_drd_switch_start_peripheral(&drd_switch->otg, 0);
			phy->state = OTG_STATE_A_IDLE;
			exynos_drd_switch_schedule_work(work);
		}
		if (!drd_switch->vbus_active) {
			/* B-Session is not valid anymore, go to idle state */
			exynos_drd_switch_start_peripheral(&drd_switch->otg, 0);
			phy->state = OTG_STATE_B_IDLE;
			exynos_drd_switch_schedule_work(work);
		}
		break;
	case OTG_STATE_A_IDLE:
		/* Switch to A-Device */
		ret = exynos_drd_switch_start_host(&drd_switch->otg, 1);
		if (!ret || ret == -EINPROGRESS) {
			phy->state = OTG_STATE_A_HOST;
			exynos_drd_switch_schedule_work(work);
		} else if (ret == -EAGAIN || ret == -EBUSY) {
			phy->state = OTG_STATE_UNDEFINED;
			exynos_drd_switch_schedule_work(work);
		} else {
			/* Fatal error */
			dev_err(phy->dev,
				"unable to start A-device\n");
		}
		break;
	case OTG_STATE_A_HOST:
		dev_dbg(phy->dev, "OTG_STATE_A_HOST\n");
		if (drd_switch->id_state == B_DEV) {
			ret = exynos_drd_switch_start_host(&drd_switch->otg, 0);
			/* Currently we ignore most of the errors */
			if (ret == -EAGAIN) {
				exynos_drd_switch_schedule_work(work);
			} else if (ret == -EINVAL) {
				/* Fatal error */
				dev_err(phy->dev,
					"unable to stop A-device\n");
			} else {
				phy->state = OTG_STATE_B_IDLE;
				exynos_drd_switch_schedule_work(work);
			}
		}
		break;
	default:
		dev_err(phy->dev, "%s: invalid otg-state\n", __func__);

	}

	mutex_unlock(&drd_switch->mutex);
}

/**
 * exynos_drd_switch_reset - reset DRD role switch.
 *
 * @drd: Pointer to DRD controller structure.
 * @run: Start sm if 1.
 */
void exynos_drd_switch_reset(struct exynos_drd *drd, int run)
{
	struct usb_otg *otg = drd->core.otg;
	struct exynos_drd_switch *drd_switch;

	if (otg) {
		drd_switch = container_of(otg,
					struct exynos_drd_switch, otg);

		if (drd->pdata->quirks & FORCE_RUN_PERIPHERAL)
			drd_switch->id_state = B_DEV;
		else
			drd_switch->id_state =
				exynos_drd_switch_get_id_state(drd_switch);

		drd_switch->vbus_active =
			exynos_drd_switch_get_bses_vld(drd_switch);

		otg->phy->state = OTG_STATE_UNDEFINED;

		if (run)
			exynos_drd_switch_schedule_work(&drd_switch->work);
	}
}

/*
 * id and vbus attributes allow to change DRD mode and VBus state.
 * Can be used for debug purpose.
 */

static ssize_t
exynos_drd_switch_show_vbus(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct exynos_drd *drd = dev_get_drvdata(dev);
	struct usb_otg *otg = drd->core.otg;
	struct exynos_drd_switch *drd_switch = container_of(otg,
						struct exynos_drd_switch, otg);

	return sprintf(buf, "%d\n", drd_switch->vbus_active);
}

static ssize_t
exynos_drd_switch_store_vbus(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	struct exynos_drd *drd = dev_get_drvdata(dev);
	struct usb_otg *otg = drd->core.otg;
	struct exynos_drd_switch *drd_switch = container_of(otg,
						struct exynos_drd_switch, otg);
	int vbus_active;

	sscanf(buf, "%d", &vbus_active);
	exynos_drd_switch_handle_vbus(drd_switch, !!vbus_active);

	return n;
}

static DEVICE_ATTR(vbus, S_IWUSR | S_IRUGO,
	exynos_drd_switch_show_vbus, exynos_drd_switch_store_vbus);

static ssize_t
exynos_drd_switch_show_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct exynos_drd *drd = dev_get_drvdata(dev);
	struct usb_otg *otg = drd->core.otg;
	struct exynos_drd_switch *drd_switch = container_of(otg,
						struct exynos_drd_switch, otg);

	return sprintf(buf, "%d\n", drd_switch->id_state);
}

static ssize_t
exynos_drd_switch_store_id(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	struct exynos_drd *drd = dev_get_drvdata(dev);
	struct usb_otg *otg = drd->core.otg;
	struct exynos_drd_switch *drd_switch = container_of(otg,
						struct exynos_drd_switch, otg);
	int state;
	enum id_pin_state id_state;

	sscanf(buf, "%d", &state);
	id_state = exynos_drd_switch_sanitize_id(state);
	exynos_drd_switch_handle_id(drd_switch, id_state);

	return n;
}

static DEVICE_ATTR(id, S_IWUSR | S_IRUGO,
	exynos_drd_switch_show_id, exynos_drd_switch_store_id);

static struct attribute *exynos_drd_switch_attributes[] = {
	&dev_attr_id.attr,
	&dev_attr_vbus.attr,
	NULL
};

static const struct attribute_group exynos_drd_switch_attr_group = {
	.attrs = exynos_drd_switch_attributes,
};

/**
 * exynos_drd_switch_init - Initializes DRD role switch.
 *
 * @drd: Pointer to DRD controller structure.
 *
 * Returns 0 on success otherwise negative errno.
 */
int exynos_drd_switch_init(struct exynos_drd *drd)
{
	struct dwc3_exynos_data *pdata = drd->pdata;
	struct exynos_drd_switch *drd_switch;
	int ret = 0;
	unsigned long irq_flags = 0;

	dev_dbg(drd->dev, "%s\n", __func__);

	drd_switch = devm_kzalloc(drd->dev, sizeof(struct exynos_drd_switch),
				  GFP_KERNEL);
	if (!drd_switch) {
		dev_err(drd->dev, "not enough memory for DRD switch\n");
		return -ENOMEM;
	}

	drd_switch->core = &drd->core;

	/* ID pin gpio IRQ */
	drd_switch->id_irq = pdata->id_irq;
	if (drd_switch->id_irq < 0)
		dev_info(drd->dev, "cannot find ID irq\n");

	init_timer(&drd_switch->id_db_timer);
	drd_switch->id_db_timer.data = (unsigned long) drd_switch;
	drd_switch->id_db_timer.function = exynos_drd_switch_debounce;

	/* VBus pin gpio IRQ */
	drd_switch->vbus_irq = pdata->vbus_irq;
	if (drd_switch->vbus_irq < 0)
		dev_info(drd->dev, "cannot find VBUS irq\n");

	init_timer(&drd_switch->vbus_db_timer);
	drd_switch->vbus_db_timer.data = (unsigned long) drd_switch;
	drd_switch->vbus_db_timer.function = exynos_drd_switch_debounce;

	irq_flags = pdata->irq_flags;

	drd_switch->otg.set_peripheral = exynos_drd_switch_set_peripheral;
	drd_switch->otg.set_host = exynos_drd_switch_set_host;

	/* Save for using by host and peripheral */
	drd->core.otg = &drd_switch->otg;

	drd_switch->otg.phy = devm_kzalloc(drd->dev, sizeof(struct usb_phy),
					   GFP_KERNEL);
	if (!drd_switch->otg.phy) {
		dev_err(drd->dev, "cannot allocate OTG phy\n");
		return -ENOMEM;
	}

	drd_switch->otg.phy->otg = &drd_switch->otg;
	drd_switch->otg.phy->dev = drd->dev;
#if 0
	/*
	 * TODO: we need to have support for multiple transceivers here.
	 * Kernel > 3.5 should already have it. Now it works only for one
	 * drd channel.
	 */
	ret = usb_set_transceiver(drd_switch->otg.phy);
	if (ret) {
		dev_err(drd->dev,
			"failed to set transceiver, already exists\n",
			__func__);
		goto err2;
	}
#endif

	exynos_drd_switch_reset(drd, 0);

	INIT_WORK(&drd_switch->work, exynos_drd_switch_work);
	mutex_init(&drd_switch->mutex);

	if (drd_switch->id_irq >= 0) {
		ret = devm_request_irq(drd->dev, drd_switch->id_irq,
				  exynos_drd_switch_id_interrupt, irq_flags,
				  "drd_switch_id", drd_switch);
		if (ret) {
			dev_err(drd->dev, "cannot claim ID irq\n");
			goto err_irq;
		}
	}

	if (drd_switch->vbus_irq >= 0) {
		ret = devm_request_irq(drd->dev, drd_switch->vbus_irq,
				  exynos_drd_switch_vbus_interrupt, irq_flags,
				  "drd_switch_vbus", drd_switch);
		if (ret) {
			dev_err(drd->dev, "cannot claim VBUS irq\n");
			goto err_irq;
		}
	}

	ret = sysfs_create_group(&drd->dev->kobj, &exynos_drd_switch_attr_group);
	if (ret) {
		dev_err(drd->dev, "cannot create switch attributes\n");
		goto err_irq;
	}

	dev_info(drd->dev, "DRD switch initialization finished normally\n");

	return 0;

err_irq:
	cancel_work_sync(&drd_switch->work);

	return ret;
}

/**
 * exynos_drd_switch_exit
 *
 * @drd: Pointer to DRD controller structure
 *
 * Returns 0 on success otherwise negative errno.
 */
void exynos_drd_switch_exit(struct exynos_drd *drd)
{
	struct usb_otg *otg = drd->core.otg;
	struct exynos_drd_switch *drd_switch;

	if (otg) {
		drd_switch = container_of(otg,
					struct exynos_drd_switch, otg);

		sysfs_remove_group(&drd->dev->kobj,
			&exynos_drd_switch_attr_group);
		cancel_work_sync(&drd_switch->work);
	}
}
