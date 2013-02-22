/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MACH_EXYNOS_BOARD_SMDK5250_H
#define __MACH_EXYNOS_BOARD_SMDK5250_H

#include <asm/system_info.h>

#define SMDK5250_REV_0_0		0x0
#define SMDK5250_REV_0_1		0x1
#define SMDK5250_REV_0_2		0x2
#define SMDK5250_REV_MASK		0xf

#define SMDK5250_REGULATOR_MAX77686	0x0
#define SMDK5250_REGULATOR_MAX8997	0x1
#define SMDK5250_REGULATOR_S5M8767	0x2
#define SMDK5250_REGULATOR_SHIFT	16
#define SMDK5250_REGULATOR_MASK		0xf

static inline int get_smdk5250_rev(void)
{
	return system_rev & SMDK5250_REV_MASK;
}

static inline int get_smdk5250_regulator(void)
{
	return (system_rev >> SMDK5250_REGULATOR_SHIFT) \
		& SMDK5250_REGULATOR_MASK;
}

void exynos5_smdk5250_mmc_init(void);
void exynos5_smdk5250_audio_init(void);
void exynos5_smdk5250_input_init(void);
void exynos5_smdk5250_usb_init(void);
void exynos5_smdk5250_power_init(void);
void exynos5_smdk5250_spi_init(void);
void exynos5_smdk5250_display_init(void);
void exynos5_smdk5250_media_init(void);
void exynos5_smdk5250_tvout_init(void);
void exynos5_smdk5250_camera_init(void);
void exynos5_arndale_wifi_init(void);
void exynos5_arndale_sensors_init(void);
#ifdef CONFIG_GPS_POWER
void exynos5_arndale_gps_init(void);
#endif

#endif
