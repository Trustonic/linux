/*
 * Copyright (C) 2012 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <linux/if.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/mmc/host.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <mach/dwmci.h>
#include <mach/map.h>

#include <linux/random.h>
#include <linux/jiffies.h>
#include <linux/module.h>


#include "board-smdk5250.h"

#ifdef CONFIG_ATH6KL
#include <linux/ath6kl.h>
#define ARNDALE_WLAN_WOW    EXYNOS5_GPX1(0)
#define ARNDALE_WLAN_RESET  EXYNOS5_GPX3(0)

static void (*wifi_status_cb)(struct platform_device *, int state);

static int arndale_wifi_ext_cd_init(void (*notify_func)
		(struct platform_device *, int state))
{
	/* Assign sdhci_s3c_notify_change to wifi_status_cb */
	if (!notify_func)
		return -EAGAIN;
	else
		wifi_status_cb = notify_func;

	return 0;
}

static int arndale_wifi_ext_cd_cleanup(void (*notify_func)
        (struct platform_device *, int))
{
	wifi_status_cb = NULL;
	return 0;
}


static void arndale_wifi_set_power(int val)
{
    int err;

    err = gpio_request_one(ARNDALE_WLAN_RESET, GPIOF_OUT_INIT_LOW, "GPX3_0"); 

    if (err) {
        pr_warning("ARNDALE: Not obtain WIFI gpios\n");
        return;
    }

    if (val) {
        s3c_gpio_cfgpin(ARNDALE_WLAN_RESET, S3C_GPIO_OUTPUT);
        s3c_gpio_setpull(ARNDALE_WLAN_RESET,
                                        S3C_GPIO_PULL_NONE);
        /* VDD33,I/O Supply must be done */
        gpio_set_value(ARNDALE_WLAN_WOW, 0);
        gpio_set_value(ARNDALE_WLAN_RESET, 0);
        udelay(300);     /*Tb */
		printk("ARNDALE: ARNDALE_WLAN_RESET on setting!!!!\n");
        gpio_direction_output(ARNDALE_WLAN_RESET, 1);
		gpio_free(ARNDALE_WLAN_RESET);
    } else {
		printk("ARNDALE: ARNDALE_WLAN_RESET off setting!!!!\n");
        gpio_direction_output(ARNDALE_WLAN_RESET, 0);
		gpio_free(ARNDALE_WLAN_RESET);
    }

    mdelay(100);

    return;
}

static int arndale_wifi_set_detect(int val)
{
	if (val) {
		arndale_wifi_set_power(val);
		wifi_status_cb(&exynos5_device_dwmci1, val);
	} else {
		arndale_wifi_set_power(val);
		wifi_status_cb(&exynos5_device_dwmci1, val);
	}

	return 0;
}

struct ath6kl_platform_data arndale_wifi_data  __initdata = {
	.setup_power = arndale_wifi_set_detect,
};
#endif

#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_WIRELESS_SOLUTION)

#define GPIO_COMBO_PMU_EN_PIN ( EXYNOS5_GPX2(7) )
#define GPIO_COMBO_EEDI_PIN ( EXYNOS5_GPX3(0) )
#define GPIO_COMBO_RST_PIN ( EXYNOS5_GPA0(3) )
#define GPIO_COMBO_GPS_SYNC_PIN ( EXYNOS5_GPA0(2) )
#define GPIO_COMBO_BGF_EINT_PIN ( EXYNOS5_GPX1(0) )
#define GPIO_COMBO_WIFI_EINT_PIN ( EXYNOS5_GPX0(7) )
static int mt6620_setup_gpio(void)
{
	/* EEDI_PIN */
	if (gpio_request(GPIO_COMBO_EEDI_PIN, "GPIO_COMBO_EEDI_PIN")) {
		pr_err("%s : gpio request failed.\n", __func__);
		return 1;
	}
	s3c_gpio_cfgpin(GPIO_COMBO_EEDI_PIN, S3C_GPIO_SFN(0x1));
	s3c_gpio_setpull(GPIO_COMBO_EEDI_PIN, S3C_GPIO_PULL_DOWN);
	s5p_gpio_set_drvstr(GPIO_COMBO_EEDI_PIN, S5P_GPIO_DRVSTR_LV4);
	gpio_direction_output(GPIO_COMBO_EEDI_PIN, 1);
	gpio_free(GPIO_COMBO_EEDI_PIN);

	/* PMU_EN_PIN */
	if (gpio_request(GPIO_COMBO_PMU_EN_PIN, "GPIO_COMBO_PMU_EN_PIN")) {
		pr_err("%s : gpio request failed.\n", __func__);
		return 1;
	}
	s3c_gpio_cfgpin(GPIO_COMBO_PMU_EN_PIN, S3C_GPIO_SFN(0x1));
	s3c_gpio_setpull(GPIO_COMBO_PMU_EN_PIN, S3C_GPIO_PULL_DOWN);
	s5p_gpio_set_drvstr(GPIO_COMBO_PMU_EN_PIN, S5P_GPIO_DRVSTR_LV4);
	gpio_direction_output(GPIO_COMBO_PMU_EN_PIN, 0);
	gpio_free(GPIO_COMBO_PMU_EN_PIN);

	/* RST_PIN */
	if (gpio_request(GPIO_COMBO_RST_PIN, "GPIO_COMBO_RST_PIN")) {
		pr_err("%s : gpio request failed.\n", __func__);
		return 1;
	}
	s3c_gpio_cfgpin(GPIO_COMBO_RST_PIN, S3C_GPIO_SFN(0x1));
	s3c_gpio_setpull(GPIO_COMBO_RST_PIN, S3C_GPIO_PULL_DOWN);
	s5p_gpio_set_drvstr(GPIO_COMBO_RST_PIN, S5P_GPIO_DRVSTR_LV4);
	gpio_direction_output(GPIO_COMBO_RST_PIN, 0);
	gpio_free(GPIO_COMBO_RST_PIN);

	/* BGF_EINT_PIN */
	if (gpio_request(GPIO_COMBO_BGF_EINT_PIN, "GPIO_COMBO_BGF_EINT_PIN")) {
		pr_err("%s : gpio request failed.\n", __func__);
		return 1;
	}
	s3c_gpio_cfgpin(GPIO_COMBO_BGF_EINT_PIN, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(GPIO_COMBO_BGF_EINT_PIN, S3C_GPIO_PULL_NONE);
	gpio_free(GPIO_COMBO_BGF_EINT_PIN);

	/* WIFI_EINT PIN */
	if (gpio_request(GPIO_COMBO_WIFI_EINT_PIN, "GPIO_COMBO_WIFI_EINT_PIN")) {
		pr_err("%s : gpio request failed.\n", __func__);
		return 1;
	}
	s3c_gpio_cfgpin(GPIO_COMBO_WIFI_EINT_PIN, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(GPIO_COMBO_WIFI_EINT_PIN, S3C_GPIO_PULL_NONE);
	gpio_free(GPIO_COMBO_WIFI_EINT_PIN);

	return 0;
}
#if 0
static void (*wifi_status_cb)(struct platform_device *, int state);

static int arndale_wifi_ext_cd_init(void (*notify_func)
		(struct platform_device *, int state))
{
	/* Assign sdhci_s3c_notify_change to wifi_status_cb */
	if (!notify_func)
		return -EAGAIN;
	else
		wifi_status_cb = notify_func;

	return 0;
}

static int arndale_wifi_ext_cd_cleanup(void (*notify_func)
		(struct platform_device *, int))
{
	wifi_status_cb = NULL;
	return 0;
}
#else
int wifi_on= 0;
EXPORT_SYMBOL(wifi_on);

static struct platform_device *wifi_status_cb_devid;
static void (*wifi_status_cb) (struct platform_device *dev, int state);

void MTK6220_wifi_set_carddetect(int val)
{
	printk("%s: %d\n", __func__, val);
	wifi_on=val;
	if (wifi_status_cb) {
		wifi_status_cb(wifi_status_cb_devid, val);
	} else
		printk("%s: Nobody to notify\n", __func__);
}
EXPORT_SYMBOL(MTK6220_wifi_set_carddetect);

int MTK6220_wifi_status_register(
                void (*cb)(struct platform_device *dev, int state))
{
	if (wifi_status_cb)
		return -EBUSY;
	wifi_status_cb = cb;
	wifi_status_cb_devid = &exynos5_device_dwmci1;
	return 0;
}
#endif
#endif

static void exynos5_setup_wlan_cfg_gpio(int width)
{
	unsigned int gpio;

	/* Set all the necessary GPC2[0:1] pins to special-function 2 */
	for (gpio = EXYNOS5_GPC2(0); gpio < EXYNOS5_GPC2(2); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
	}

	for (gpio = EXYNOS5_GPC2(3); gpio <= EXYNOS5_GPC2(6); gpio++) {
		/* Data pin GPC2[3:6] to special-function 2 */
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
	}

	gpio = EXYNOS5_GPC2(2);
	s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
}

static struct dw_mci_board exynos_wlan_pdata __initdata = {
	.num_slots		= 1,
	.cd_type                = DW_MCI_CD_EXTERNAL,
	.quirks			= DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 200 * 1000 * 1000,
	.caps			= MMC_CAP_CMD23, 
	.fifo_depth		= 0x80,
	.detect_delay_ms	= 200,
	.hclk_name		= "dwmci",
	.cclk_name		= "sclk_dwmci",
	.cfg_gpio		= exynos5_setup_wlan_cfg_gpio,
#ifdef CONFIG_ATH6KL
	.ext_cd_init		= arndale_wifi_ext_cd_init,
	.ext_cd_cleanup		= arndale_wifi_ext_cd_cleanup,
#endif
#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_WIRELESS_SOLUTION)
	.ext_cd_init		= MTK6220_wifi_status_register,
#endif
	.sdr_timing		= 0x03020001,
	.ddr_timing		= 0x03030002,
	.clk_drv 		= 4,
};

static struct platform_device *arndale_wifi_devs[] __initdata = {
	&exynos5_device_dwmci1,
};

void __init exynos5_arndale_wifi_init(void)
{
#ifdef CONFIG_ATH6KL
	ath6kl_set_platform_data(&arndale_wifi_data);
#endif
#if defined(CONFIG_MTK_COMBO) || defined(CONFIG_MTK_WIRELESS_SOLUTION)
	mt6620_setup_gpio();
#endif

	exynos_dwmci_set_platdata(&exynos_wlan_pdata, 1);
	dev_set_name(&exynos5_device_dwmci1.dev, "exynos4-sdhci.1");
	clk_add_alias("dwmci", "dw_mmc.1", "hsmmc", &exynos5_device_dwmci1.dev);
	clk_add_alias("sclk_dwmci", "dw_mmc.1", "sclk_mmc",
		      &exynos5_device_dwmci1.dev);
	platform_add_devices(arndale_wifi_devs, ARRAY_SIZE(arndale_wifi_devs));

}

