/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/clk.h>

#include <media/exynos_gscaler.h>

#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/jpeg.h>
#include <plat/fimg2d.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-pmu.h>
#include <mach/sysmmu.h>
#include <mach/exynos-ion.h>
#include <mach/exynos-mfc.h>

#include "board-smdk5250.h"

#ifdef CONFIG_EXYNOS_MEDIA_DEVICE
struct platform_device exynos_device_md0 = {
	.name = "exynos-mdev",
	.id = 0,
};

struct platform_device exynos_device_md1 = {
	.name = "exynos-mdev",
	.id = 1,
};

struct platform_device exynos_device_md2 = {
	.name = "exynos-mdev",
	.id = 2,
};
#endif

#ifdef CONFIG_VIDEO_EXYNOS_FIMG2D
static struct fimg2d_platdata fimg2d_data __initdata = {
	.ip_ver		= IP_VER_G2D_5G,
	.hw_ver		= 0x42,
	.gate_clkname	= "fimg2d",
};
#endif

static struct platform_device *smdk5250_media_devices[] __initdata = {
#ifdef CONFIG_VIDEO_EXYNOS_MFC
	&s5p_device_mfc,
#endif
#ifdef CONFIG_EXYNOS_MEDIA_DEVICE
	&exynos_device_md0,
	&exynos_device_md1,
	&exynos_device_md2,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_GSCALER
	&exynos5_device_gsc0,
	&exynos5_device_gsc1,
	&exynos5_device_gsc2,
	&exynos5_device_gsc3,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMG2D
	&s5p_device_fimg2d,
#endif
	&exynos5_device_rotator,
#ifdef CONFIG_VIDEO_EXYNOS_JPEG
	&s5p_device_jpeg,
#endif
};

#if defined(CONFIG_VIDEO_EXYNOS_MFC)
static struct s5p_mfc_platdata smdk5250_mfc_pd = {
	.clock_rate = 333 * MHZ,
};
#endif

static void __init smdk5250_media_sysmmu_init(void)
{
#ifdef CONFIG_VIDEO_EXYNOS_JPEG
	platform_set_sysmmu(&SYSMMU_PLATDEV(jpeg).dev, &s5p_device_jpeg.dev);
#endif
#if defined(CONFIG_VIDEO_EXYNOS_MFC)
	platform_set_sysmmu(&SYSMMU_PLATDEV(mfc_lr).dev, &s5p_device_mfc.dev);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_GSCALER
	platform_set_sysmmu(&SYSMMU_PLATDEV(gsc0).dev,
						&exynos5_device_gsc0.dev);
	platform_set_sysmmu(&SYSMMU_PLATDEV(gsc1).dev,
						&exynos5_device_gsc1.dev);
	platform_set_sysmmu(&SYSMMU_PLATDEV(gsc2).dev,
						&exynos5_device_gsc2.dev);
	platform_set_sysmmu(&SYSMMU_PLATDEV(gsc3).dev,
						&exynos5_device_gsc3.dev);
#endif
	platform_set_sysmmu(&SYSMMU_PLATDEV(rot).dev,
						&exynos5_device_rotator.dev);
#ifdef CONFIG_VIDEO_EXYNOS_FIMG2D
	platform_set_sysmmu(&SYSMMU_PLATDEV(2d).dev,
						&s5p_device_fimg2d.dev);
#endif
}

void __init exynos5_smdk5250_media_init(void)
{
	smdk5250_media_sysmmu_init();

#ifdef CONFIG_VIDEO_EXYNOS_MFC
	s5p_mfc_set_platdata(&smdk5250_mfc_pd);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMG2D
	s5p_fimg2d_set_platdata(&fimg2d_data);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_GSCALER
	s3c_set_platdata(&exynos_gsc0_default_data,
			sizeof(exynos_gsc0_default_data), &exynos5_device_gsc0);
	s3c_set_platdata(&exynos_gsc1_default_data,
			sizeof(exynos_gsc1_default_data), &exynos5_device_gsc1);
	s3c_set_platdata(&exynos_gsc2_default_data,
			sizeof(exynos_gsc2_default_data), &exynos5_device_gsc2);
	s3c_set_platdata(&exynos_gsc3_default_data,
			sizeof(exynos_gsc3_default_data), &exynos5_device_gsc3);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_JPEG
	exynos5_jpeg_setup_clock(&s5p_device_jpeg.dev, 150000000);
#endif
	platform_add_devices(smdk5250_media_devices,
			ARRAY_SIZE(smdk5250_media_devices));
}
