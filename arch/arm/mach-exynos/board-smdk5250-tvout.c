/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <linux/i2c.h>

#include <plat/clock.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/iic.h>
#include <plat/tv-core.h>

#include <mach/sysmmu.h>

#include "board-smdk5250.h"

static struct i2c_board_info i2c_devs2[] __initdata = {
	{
		I2C_BOARD_INFO("exynos_hdcp", (0x74 >> 1)),
	},
	{
		I2C_BOARD_INFO("exynos_edid", (0xA0 >> 1)),
	},
};

#if defined(CONFIG_VIDEO_EXYNOS_TV) && defined(CONFIG_VIDEO_EXYNOS_HDMI)
static struct s5p_hdmi_platdata hdmi_platdata __initdata = {
};
#endif

#if defined(CONFIG_VIDEO_EXYNOS_TV) && defined(CONFIG_VIDEO_EXYNOS_HDMI_CEC)
static struct s5p_platform_cec hdmi_cec_data __initdata = {

};
#endif

static struct platform_device *smdk5250_tvout_devices[] __initdata = {
	&s3c_device_i2c2,
#ifdef CONFIG_VIDEO_EXYNOS_TV
#ifdef CONFIG_VIDEO_EXYNOS_HDMI
	&s5p_device_hdmi,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_HDMIPHY
	&s5p_device_i2c_hdmiphy,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_MIXER
	&s5p_device_mixer,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_HDMI_CEC
	&s5p_device_cec,
#endif
#endif
};

static void __init smdk5250_tvout_sysmmu_init(void)
{
#if defined(CONFIG_VIDEO_EXYNOS_TV) && defined(CONFIG_VIDEO_EXYNOS_MIXER)
	platform_set_sysmmu(&SYSMMU_PLATDEV(tv).dev, &s5p_device_mixer.dev);
#endif
}

void __init exynos5_smdk5250_tvout_init(void)
{
	smdk5250_tvout_sysmmu_init();

	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(2, i2c_devs2, ARRAY_SIZE(i2c_devs2));

#if defined(CONFIG_VIDEO_EXYNOS_TV) && defined(CONFIG_VIDEO_EXYNOS_HDMI)
	dev_set_name(&s5p_device_hdmi.dev, "exynos5-hdmi");
	clk_add_alias("hdmi", "s5p-hdmi", "hdmi", &s5p_device_hdmi.dev);

	/* direct HPD to HDMI chip */
	gpio_request(EXYNOS5_GPX3(7), "hpd-plug");
	gpio_direction_input(EXYNOS5_GPX3(7));
	s3c_gpio_cfgpin(EXYNOS5_GPX3(7), S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(EXYNOS5_GPX3(7), S3C_GPIO_PULL_NONE);

	/* HDMI CEC */
	gpio_request(EXYNOS5_GPX3(6), "hdmi-cec");
	gpio_direction_input(EXYNOS5_GPX3(6));
	s3c_gpio_cfgpin(EXYNOS5_GPX3(6), S3C_GPIO_SFN(0x3));
	s3c_gpio_setpull(EXYNOS5_GPX3(6), S3C_GPIO_PULL_NONE);

#if defined(CONFIG_VIDEO_EXYNOS_HDMIPHY)
	s5p_hdmi_set_platdata(&hdmi_platdata);
	s5p_i2c_hdmiphy_set_platdata(NULL);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_HDMI_CEC
	s5p_hdmi_cec_set_platdata(&hdmi_cec_data);
#endif
#endif
	platform_add_devices(smdk5250_tvout_devices,
			ARRAY_SIZE(smdk5250_tvout_devices));
}
