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
#include <linux/gpio_event.h>
#include <linux/i2c.h>

#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/iic.h>

#include "board-smdk5250.h"

static struct gpio_event_direct_entry smdk5250_keypad_key_map[] = {
	{
		.gpio	= EXYNOS5_GPX0(0),
		.code	= KEY_POWER,
	}
};

static struct gpio_event_input_info smdk5250_keypad_key_info = {
	.info.func		= gpio_event_input_func,
	.info.no_suspend	= true,
	.debounce_time.tv64	= 5 * NSEC_PER_MSEC,
	.type			= EV_KEY,
	.keymap			= smdk5250_keypad_key_map,
	.keymap_size		= ARRAY_SIZE(smdk5250_keypad_key_map)
};

static struct gpio_event_info *smdk5250_input_info[] = {
	&smdk5250_keypad_key_info.info,
};

static struct gpio_event_platform_data smdk5250_input_data = {
	.names	= {
		"smdk5250-keypad",
		NULL,
	},
	.info		= smdk5250_input_info,
	.info_count	= ARRAY_SIZE(smdk5250_input_info),
};

static struct platform_device smdk5250_input_device = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= 0,
	.dev	= {
		.platform_data = &smdk5250_input_data,
	},
};

static void __init exynos5_smdk5250_gpio_power_init(void)
{
	int err = 0;

	err = gpio_request_one(EXYNOS5_GPX0(0), 0, "GPX0");
	if (err) {
		pr_err("failed to request GPX0 for "
		       "suspend/resume control\n");
		return;
	}
	s3c_gpio_setpull(EXYNOS5_GPX0(0), S3C_GPIO_PULL_NONE);

	gpio_free(EXYNOS5_GPX0(0));
}

static void exynos5_smdk5250_touch_init(void)
{
#ifdef CONFIG_TOUCHSCREEN_COASIA
	int gpio;

	if (get_smdk5250_rev() == SMDK5250_REV_0_0)
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
	&smdk5250_input_device,
};

void __init exynos5_smdk5250_input_init(void)
{
	s3c_i2c3_set_platdata(&i2c_data3);
	if (get_smdk5250_rev() == SMDK5250_REV_0_0)
		i2c_devs3[0].irq = IRQ_EINT(21);
	else
		i2c_devs3[0].irq = IRQ_EINT(18);
	i2c_register_board_info(3, i2c_devs3, ARRAY_SIZE(i2c_devs3));

	s3c_i2c7_set_platdata(NULL);
	i2c_register_board_info(7, i2c_devs7, ARRAY_SIZE(i2c_devs7));

	exynos5_smdk5250_gpio_power_init();
	exynos5_smdk5250_touch_init();

	platform_add_devices(smdk5250_input_devices,
			ARRAY_SIZE(smdk5250_input_devices));
}
