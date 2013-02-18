/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/platform_data/dwc3-exynos.h>

#include <plat/usb-phy.h>
#include <plat/ehci.h>
#include <plat/udc-hs.h>
#include <plat/devs.h>
#include <plat/gpio-cfg.h>

#include <mach/ohci.h>
#include <mach/usb3-drd.h>
#include <mach/usb-switch.h>

/* USB EHCI */
static struct s5p_ehci_platdata smdk5250_ehci_pdata;

static void __init smdk5250_ehci_init(void)
{
	struct s5p_ehci_platdata *pdata = &smdk5250_ehci_pdata;

	s5p_ehci_set_platdata(pdata);
}

/* USB OHCI */
static struct exynos4_ohci_platdata smdk5250_ohci_pdata;

static void __init smdk5250_ohci_init(void)
{
	struct exynos4_ohci_platdata *pdata = &smdk5250_ohci_pdata;

	exynos4_ohci_set_platdata(pdata);
}

#ifdef CONFIG_USB_S3C_OTGD
static struct s3c_hsotg_plat smdk5250_hsotg_pdata;

static void __init smdk5250_udc_init(void)
{
	struct s3c_hsotg_plat *pdata = &smdk5250_hsotg_pdata;

	s3c_hsotg_set_platdata(pdata);
}
#endif

#ifdef CONFIG_USB_EXYNOS_SWITCH
static struct s5p_usbswitch_platdata smdk5250_usbswitch_pdata __initdata;

static void __init smdk5250_usbswitch_init(void)
{
	struct s5p_usbswitch_platdata *pdata = &smdk5250_usbswitch_pdata;
	int err;

	/* USB 2.0 detect GPIO */
#if defined(CONFIG_USB_EHCI_S5P) || defined(CONFIG_USB_OHCI_EXYNOS)
	pdata->gpio_host_detect = EXYNOS5_GPX1(6);
	err = gpio_request_one(pdata->gpio_host_detect, GPIOF_IN,
			"HOST_DETECT");
	if (err) {
		printk(KERN_ERR "failed to request host gpio\n");
		return;
	}

	s3c_gpio_cfgpin(pdata->gpio_host_detect, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(pdata->gpio_host_detect, S3C_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_host_detect);

	pdata->gpio_host_vbus = EXYNOS5_GPX2(6);
	err = gpio_request_one(pdata->gpio_host_vbus,
			GPIOF_OUT_INIT_LOW,
			"HOST_VBUS_CONTROL");
	if (err) {
		printk(KERN_ERR "failed to request host_vbus gpio\n");
		return;
	}

	s3c_gpio_setpull(pdata->gpio_host_vbus, S3C_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_host_vbus);
#endif

#ifdef CONFIG_USB_S3C_OTGD
	pdata->gpio_device_detect = EXYNOS5_GPX3(4);
	err = gpio_request_one(pdata->gpio_device_detect, GPIOF_IN,
		"DEVICE_DETECT");
	if (err) {
		printk(KERN_ERR "failed to request device gpio\n");
		return;
	}

	s3c_gpio_cfgpin(pdata->gpio_device_detect, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(pdata->gpio_device_detect, S3C_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_device_detect);
#endif

	s5p_usbswitch_set_platdata(pdata);
}
#endif

#ifdef CONFIG_USB_EXYNOS5_USB3_DRD_CH0

static int smdk5250_vbus_ctrl(struct platform_device *pdev, int on)
{
	return 0;
}

#define SMDK5250_ID0_GPIO	EXYNOS5_GPX1(7)
#define SMDK5250_VBUS0_GPIO	EXYNOS5_GPX0(6)

static int smdk5250_get_id_state(struct platform_device *pdev)
{
	int phy_num = pdev->id;
	unsigned gpio;

	if (phy_num == 0)
		gpio = SMDK5250_ID0_GPIO;
	else
		return -EINVAL;

	return gpio_get_value(gpio);
}

static bool smdk5250_get_bsession_valid(struct platform_device *pdev)
{
	int phy_num = pdev->id;
	unsigned gpio;

	if (phy_num == 0)
		gpio = SMDK5250_VBUS0_GPIO;
	else
		/*
		 * If something goes wrong, we return true,
		 * because we don't want switch stop working.
		 */
		return true;

	return !!gpio_get_value(gpio);
}

static struct dwc3_exynos_data smdk5250_drd_pdata __initdata = {
	.udc_name		= "exynos-ss-udc",
	.xhci_name		= "exynos-xhci",
	.phy_type		= S5P_USB_PHY_DRD,
	.phy_init		= s5p_usb_phy_init,
	.phy_exit		= s5p_usb_phy_exit,
	.phy_crport_ctrl	= exynos5_usb_phy_crport_ctrl,
	.vbus_ctrl		= smdk5250_vbus_ctrl,
	.get_id_state		= smdk5250_get_id_state,
	.get_bses_vld		= smdk5250_get_bsession_valid,
	.irq_flags		= IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
};

static void __init smdk5250_drd0_init(void)
{
	int err;

#ifdef CONFIG_USB_XHCI_EXYNOS
	//err = gpio_request_one(EXYNOS5_GPX1(7), GPIOF_IN,
	err = gpio_request_one(EXYNOS5_GPX0(6), GPIOF_IN,
		"DRD_HOST_DETECT");
	if (err) {
		printk(KERN_ERR "failed to request drd_host gpio\n");
		return;
	}

	/*s3c_gpio_cfgpin(EXYNOS5_GPX1(7), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS5_GPX1(7),
		S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS5_GPX1(7));*/
	s3c_gpio_cfgpin(EXYNOS5_GPX0(6), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS5_GPX0(6),
		S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS5_GPX0(6));
#endif

#ifdef CONFIG_USB_EXYNOS5_USB3_DRD_CH0
	err = gpio_request_one(EXYNOS5_GPX0(6), GPIOF_IN,
		"DRD_DEVICE_DETECT");
	if (err) {
		printk(KERN_ERR "failed to request drd_device\n");
		return;
	}

	s3c_gpio_cfgpin(EXYNOS5_GPX0(6), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS5_GPX0(6), S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS5_GPX0(6));

	smdk5250_drd_pdata.id_irq = gpio_to_irq(SMDK5250_ID0_GPIO);
	smdk5250_drd_pdata.vbus_irq = gpio_to_irq(SMDK5250_VBUS0_GPIO);

#if !defined(CONFIG_USB_XHCI_EXYNOS)
	smdk5250_drd_pdata.quirks |= FORCE_RUN_PERIPHERAL;
#endif

#endif
	exynos5_usb3_drd0_set_platdata(&smdk5250_drd_pdata);
}
#endif

static struct platform_device *smdk5250_usb_devices[] __initdata = {
	&s5p_device_ehci,
	&exynos4_device_ohci,
#ifdef CONFIG_USB_S3C_OTGD
	&s3c_device_usb_hsotg,
#endif
#ifdef CONFIG_USB_EXYNOS5_USB3_DRD_CH0
	&exynos5_device_usb3_drd0,
#endif
};

void __init exynos5_smdk5250_usb_init(void)
{
	smdk5250_ehci_init();
	smdk5250_ohci_init();
#ifdef CONFIG_USB_S3C_OTGD
	smdk5250_udc_init();
#endif
#ifdef CONFIG_USB_EXYNOS5_USB3_DRD_CH0
	smdk5250_drd0_init();
#endif
#ifdef CONFIG_USB_EXYNOS_SWITCH
	smdk5250_usbswitch_init();
#endif

	platform_add_devices(smdk5250_usb_devices,
			ARRAY_SIZE(smdk5250_usb_devices));
}
