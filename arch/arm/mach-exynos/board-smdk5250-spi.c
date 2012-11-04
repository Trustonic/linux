/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/spi/spi.h>
#include <linux/gpio.h>

#include <plat/devs.h>
#include <plat/s3c64xx-spi.h>

#include <mach/spi-clocks.h>

#include "board-smdk5250.h"

static struct s3c64xx_spi_csinfo spi0_csi[] = {
	[0] = {
		.line		= EXYNOS5_GPA2(1),
		.set_level	= gpio_set_value,
		.fb_delay	= 0x2,
	},
};

static struct spi_board_info spi0_board_info[] __initdata = {
	{
		.modalias		= "spidev",
		.platform_data		= NULL,
		.max_speed_hz		= 10 * 1000 * 1000,
		.bus_num		= 0,
		.chip_select		= 0,
		.mode			= SPI_MODE_0,
		.controller_data	= &spi0_csi[0],
	}
};

static struct s3c64xx_spi_csinfo spi1_csi[] = {
	[0] = {
		.line		= EXYNOS5_GPA2(5),
		.set_level	= gpio_set_value,
		.fb_delay	= 0x2,
	},
};

static struct spi_board_info spi1_board_info[] __initdata = {
	{
		.modalias		= "fimc_is_spi",
		.platform_data		= NULL,
		.max_speed_hz		= 10 * 1000 * 1000,
		.bus_num		= 1,
		.chip_select		= 0,
		.mode			= SPI_MODE_0,
		.controller_data	= &spi1_csi[0],
	}
};

static struct s3c64xx_spi_csinfo spi2_csi[] = {
	[0] = {
		.line		= EXYNOS5_GPB1(2),
		.set_level	= gpio_set_value,
		.fb_delay	= 0x2,
	},
};

static struct spi_board_info spi2_board_info[] __initdata = {
	{
		.modalias		= "spidev",
		.platform_data		= NULL,
		.max_speed_hz		= 10 * 1000 * 1000,
		.bus_num		= 2,
		.chip_select		= 0,
		.mode			= SPI_MODE_0,
		.controller_data	= &spi2_csi[0],
	}
};

static struct platform_device *smdk5250_spi_devices[] __initdata = {
	&s3c64xx_device_spi0,
	&s3c64xx_device_spi1,
	&s3c64xx_device_spi2,
};

void __init exynos5_smdk5250_spi_init(void)
{
	exynos_spi_clock_setup(&s3c64xx_device_spi0.dev, 0);
	exynos_spi_clock_setup(&s3c64xx_device_spi1.dev, 1);
	exynos_spi_clock_setup(&s3c64xx_device_spi2.dev, 2);

	if (!exynos_spi_cfg_cs(spi0_csi[0].line, 0)) {
		s3c64xx_spi0_set_platdata(&s3c64xx_spi0_pdata,
				EXYNOS_SPI_SRCCLK_SCLK, ARRAY_SIZE(spi0_csi));

		spi_register_board_info(spi0_board_info,
				ARRAY_SIZE(spi0_board_info));
	} else {
		pr_err("%s: Error requesting gpio for SPI-CH0 CS\n", __func__);
	}

	if (!exynos_spi_cfg_cs(spi1_csi[0].line, 1)) {
		s3c64xx_spi1_set_platdata(&s3c64xx_spi1_pdata,
				EXYNOS_SPI_SRCCLK_SCLK, ARRAY_SIZE(spi1_csi));

		spi_register_board_info(spi1_board_info,
				ARRAY_SIZE(spi1_board_info));
	} else {
		pr_err("%s: Error requesting gpio for SPI-CH1 CS\n", __func__);
	}

	if (!exynos_spi_cfg_cs(spi2_csi[0].line, 2)) {
		s3c64xx_spi2_set_platdata(&s3c64xx_spi2_pdata,
				EXYNOS_SPI_SRCCLK_SCLK, ARRAY_SIZE(spi2_csi));

		spi_register_board_info(spi2_board_info,
				ARRAY_SIZE(spi2_board_info));
	} else {
		pr_err("%s: Error requesting gpio for SPI-CH2 CS\n", __func__);
	}

	platform_add_devices(smdk5250_spi_devices,
			ARRAY_SIZE(smdk5250_spi_devices));
}
