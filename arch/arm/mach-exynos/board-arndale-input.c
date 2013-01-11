/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/i2c.h>

#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/iic.h>

#include "board-smdk5250.h"

static struct gpio_keys_button smdk5250_button[] = {
	{
		.code		= KEY_POWER,
		.gpio		= EXYNOS5_GPX0(0),
		.active_low	= 1,
		.wakeup		= 1,
	}
};

static struct gpio_keys_platform_data smdk5250_gpiokeys_platform_data = {
	smdk5250_button,
	ARRAY_SIZE(smdk5250_button),
};

static struct platform_device smdk5250_gpio_keys = {
	.name	= "gpio-keys",
	.dev	= {
		.platform_data = &smdk5250_gpiokeys_platform_data,
	},
};

static void exynos5_smdk5250_touch_init(void)
{
	// drive up gpios of i2c7
	s5p_gpio_set_drvstr(EXYNOS5_GPB2(2), S5P_GPIO_DRVSTR_LV4);
	s5p_gpio_set_drvstr(EXYNOS5_GPB2(3), S5P_GPIO_DRVSTR_LV4);
}

static struct i2c_board_info i2c_devs7[] __initdata = {
#ifdef CONFIG_TOUCHSCREEN_UNIDISPLAY_TS
	{
		I2C_BOARD_INFO("unidisplay_ts", 0x41),
		.irq	= IRQ_EINT(9),
	},
#endif
};

static struct platform_device *smdk5250_input_devices[] __initdata = {
	&s3c_device_i2c7,
	&smdk5250_gpio_keys,
};

void __init exynos5_smdk5250_input_init(void)
{
	s3c_i2c7_set_platdata(NULL);
	i2c_register_board_info(7, i2c_devs7, ARRAY_SIZE(i2c_devs7));

	exynos5_smdk5250_touch_init();

	platform_add_devices(smdk5250_input_devices,
			ARRAY_SIZE(smdk5250_input_devices));
}
