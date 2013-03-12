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
#ifdef CONFIG_TOUCHSCREEN_COASIA
	int gpio;

	if (get_smdk5250_regulator() == SMDK5250_REGULATOR_MAX77686)
		gpio = EXYNOS5_GPX2(4);
	else
		gpio = EXYNOS5_GPX2(1);

	if (gpio_request(gpio, "GPX2")) {
		pr_err("%s : TS_RST request port error\n", __func__);
	} else {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_OUTPUT);
		gpio_direction_output(gpio, 0);
		usleep_range(20000, 21000);
		gpio_direction_output(gpio, 1);
		gpio_free(gpio);
	}
#endif
}

static struct i2c_board_info i2c_devs3[] __initdata = {
	{
		I2C_BOARD_INFO("pixcir_ts", 0x5C),
	},
};

static struct i2c_board_info i2c_devs7[] __initdata = {
	{
		I2C_BOARD_INFO("egalax_i2c", 0x04),
		.irq	= IRQ_EINT(25),
	},
};

struct s3c2410_platform_i2c i2c_data3 __initdata = {
	.bus_num	= 3,
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 200*1000,
	.sda_delay	= 100,
};

static struct platform_device *smdk5250_input_devices[] __initdata = {
	&s3c_device_i2c3,
	&s3c_device_i2c7,
	&smdk5250_gpio_keys,
};

void __init exynos5_smdk5250_input_init(void)
{
	s3c_i2c3_set_platdata(&i2c_data3);
	if (get_smdk5250_regulator() == SMDK5250_REGULATOR_MAX77686)
		i2c_devs3[0].irq = IRQ_EINT(21);
	else
		i2c_devs3[0].irq = IRQ_EINT(18);
	i2c_register_board_info(3, i2c_devs3, ARRAY_SIZE(i2c_devs3));

	s3c_i2c7_set_platdata(NULL);
	i2c_register_board_info(7, i2c_devs7, ARRAY_SIZE(i2c_devs7));

	exynos5_smdk5250_touch_init();

	platform_add_devices(smdk5250_input_devices,
			ARRAY_SIZE(smdk5250_input_devices));
}
