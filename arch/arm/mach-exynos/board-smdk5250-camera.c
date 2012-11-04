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
#include <linux/regulator/fixed.h>

#include <media/m5mols.h>
#include <media/exynos_flite.h>
#include <media/exynos_fimc_is.h>

#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/mipi_csis.h>

#include <mach/sysmmu.h>

#include "board-smdk5250.h"

#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
#if defined(CONFIG_ITU_A)
static int smdk5250_cam0_reset(int dummy)
{
	int err;
	/* Camera A */
	err = gpio_request(EXYNOS5_GPX1(2), "GPX1");
	if (err)
		pr_err("#### failed to request GPX1_2 ####\n");

	s3c_gpio_setpull(EXYNOS5_GPX1(2), S3C_GPIO_PULL_NONE);
	gpio_direction_output(EXYNOS5_GPX1(2), 0);
	gpio_direction_output(EXYNOS5_GPX1(2), 1);
	gpio_free(EXYNOS5_GPX1(2));

	return 0;
}
#endif
#if defined(CONFIG_ITU_B)
static int smdk5250_cam1_reset(int dummy)
{
	int err;
	/* Camera A */
	err = gpio_request(EXYNOS5_GPX1(0), "GPX1");
	if (err)
		printk(KERN_ERR "#### failed to request GPX1_2 ####\n");

	s3c_gpio_setpull(EXYNOS5_GPX1(0), S3C_GPIO_PULL_NONE);
	gpio_direction_output(EXYNOS5_GPX1(0), 0);
	gpio_direction_output(EXYNOS5_GPX1(0), 1);
	gpio_free(EXYNOS5_GPX1(0));

	return 0;
}
#endif

/* 1 MIPI Cameras */
#ifdef CONFIG_VIDEO_M5MOLS
static struct m5mols_platform_data m5mols_platdata = {
#ifdef CONFIG_CSI_C
	.gpio_rst = EXYNOS5_GPX1(2), /* ISP_RESET */
#endif
#ifdef CONFIG_CSI_D
	.gpio_rst = EXYNOS5_GPX1(0), /* ISP_RESET */
#endif
	.enable_rst = true, /* positive reset */
	.irq = IRQ_EINT(22),
};

static struct i2c_board_info m5mols_board_info = {
	I2C_BOARD_INFO("M5MOLS", 0x1F),
	.platform_data = &m5mols_platdata,
};
#endif
#endif /* CONFIG_VIDEO_EXYNOS_FIMC_LITE */

#ifdef CONFIG_VIDEO_EXYNOS_MIPI_CSIS
static struct regulator_consumer_supply mipi_csi_fixed_voltage_supplies[] = {
	REGULATOR_SUPPLY("mipi_csi", "s5p-mipi-csis.0"),
	REGULATOR_SUPPLY("mipi_csi", "s5p-mipi-csis.1"),
};

static struct regulator_init_data mipi_csi_fixed_voltage_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(mipi_csi_fixed_voltage_supplies),
	.consumer_supplies	= mipi_csi_fixed_voltage_supplies,
};

static struct fixed_voltage_config mipi_csi_fixed_voltage_config = {
	.supply_name	= "DC_5V",
	.microvolts	= 5000000,
	.gpio		= -EINVAL,
	.init_data	= &mipi_csi_fixed_voltage_init_data,
};

static struct platform_device mipi_csi_fixed_voltage = {
	.name		= "reg-fixed-voltage",
	.id		= 3,
	.dev		= {
		.platform_data	= &mipi_csi_fixed_voltage_config,
	},
};
#endif

#ifdef CONFIG_VIDEO_M5MOLS
static struct regulator_consumer_supply m5mols_fixed_voltage_supplies[] = {
	REGULATOR_SUPPLY("core", NULL),
	REGULATOR_SUPPLY("dig_18", NULL),
	REGULATOR_SUPPLY("d_sensor", NULL),
	REGULATOR_SUPPLY("dig_28", NULL),
	REGULATOR_SUPPLY("a_sensor", NULL),
	REGULATOR_SUPPLY("dig_12", NULL),
};

static struct regulator_init_data m5mols_fixed_voltage_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(m5mols_fixed_voltage_supplies),
	.consumer_supplies	= m5mols_fixed_voltage_supplies,
};

static struct fixed_voltage_config m5mols_fixed_voltage_config = {
	.supply_name	= "CAM_SENSOR",
	.microvolts	= 1800000,
	.gpio		= -EINVAL,
	.init_data	= &m5mols_fixed_voltage_init_data,
};

static struct platform_device m5mols_fixed_voltage = {
	.name		= "reg-fixed-voltage",
	.id		= 4,
	.dev		= {
		.platform_data	= &m5mols_fixed_voltage_config,
	},
};
#endif

#if defined CONFIG_VIDEO_EXYNOS5_FIMC_IS2
static struct exynos5_platform_fimc_is exynos5_fimc_is_data;

#if defined CONFIG_VIDEO_S5K4E5
static struct exynos5_fimc_is_sensor_info s5k4e5 = {
	.sensor_name = "S5K4E5",
	.sensor_id = SENSOR_NAME_S5K4E5,
#if defined CONFIG_S5K4E5_POSITION_FRONT
	.sensor_position = SENSOR_POSITION_FRONT,
#elif defined CONFIG_S5K4E5_POSITION_REAR
	.sensor_position = SENSOR_POSITION_REAR,
#endif
#if defined CONFIG_S5K4E5_CSI_C
	.csi_id = CSI_ID_A,
	.flite_id = FLITE_ID_A,
	.i2c_channel = SENSOR_CONTROL_I2C0,
#elif defined CONFIG_S5K4E5_CSI_D
	.csi_id = CSI_ID_B,
	.flite_id = FLITE_ID_B,
	.i2c_channel = SENSOR_CONTROL_I2C1,
#endif
	.max_width = 2560,
	.max_height = 1920,
	.max_frame_rate = 30,

	.mipi_lanes = 2,
	.mipi_settle = 12,
	.mipi_align = 24,
	.sensor_power = {
		.cam_core = "5m_core_1.5v",
		.cam_io_myself = "cam_io_1.8v",
		.cam_io_peer = "vt_cam_1.8v",
		.cam_af = "cam_af_2.8v",
	},
	.sensor_gpio = {
		.cfg[0] = {
			.pin = EXYNOS5_GPE0(0),
			.name = "GPE0",
			.value = (2<<0),
			.act = GPIO_PULL_NONE,
		},
		.cfg[1] = {
			.pin = EXYNOS5_GPE0(1),
			.name = "GPE0",
			.value = (2<<4),
			.act = GPIO_PULL_NONE,
		},
		.cfg[2] = {
			.pin = EXYNOS5_GPE0(2),
			.name = "GPE0",
			.value = (3<<8),
			.act = GPIO_PULL_NONE,
		},
		.cfg[3] = {
			.pin = EXYNOS5_GPE0(3),
			.name = "GPE0",
			.value = (3<<12),
			.act = GPIO_PULL_NONE,
		},
		.cfg[4] = {
			.pin = EXYNOS5_GPE0(4),
			.name = "GPE0",
			.value = (3<<16),
			.act = GPIO_PULL_NONE,
		},
		.cfg[5] = {
			.pin = EXYNOS5_GPE0(5),
			.name = "GPE0",
			.value = (3<<20),
			.act = GPIO_PULL_NONE,
		},
		.cfg[6] = {
			.pin = EXYNOS5_GPE0(6),
			.name = "GPE0",
			.value = (3<<24),
			.act = GPIO_PULL_NONE,
		},
		.cfg[7] = {
			.pin = EXYNOS5_GPE0(7),
			.name = "GPE0",
			.value = (3<<28),
			.act = GPIO_PULL_NONE,
		},
		.cfg[8] = {
			.pin = EXYNOS5_GPE1(0),
			.name = "GPE1",
			.value = (3<<0),
			.act = GPIO_PULL_NONE,
		},
		.cfg[9] = {
			.pin = EXYNOS5_GPE1(1),
			.name = "GPE1",
			.value = (3<<4),
			.act = GPIO_PULL_NONE,
		},
		.cfg[10] = {
			.pin = EXYNOS5_GPF0(0),
			.name = "GPF0",
			.value = (2<<0),
			.act = GPIO_PULL_NONE,
		},
		.cfg[11] = {
			.pin = EXYNOS5_GPF0(1),
			.name = "GPF0",
			.value = (2<<4),
			.act = GPIO_PULL_NONE,
		},
		.cfg[12] = {
			.pin = EXYNOS5_GPF0(2),
			.name = "GPF0",
			.value = (2<<8),
			.act = GPIO_PULL_NONE,
		},
		.cfg[13] = {
			.pin = EXYNOS5_GPF0(3),
			.name = "GPF0",
			.value = (2<<12),
			.act = GPIO_PULL_NONE,
		},
		.cfg[14] = {
			.pin = EXYNOS5_GPF1(0),
			.name = "GPF1",
			.value = (3<<0),
			.act = GPIO_PULL_NONE,
		},
		.cfg[15] = {
			.pin = EXYNOS5_GPF1(1),
			.name = "GPF1",
			.value = (3<<4),
			.act = GPIO_PULL_NONE,
		},
		.cfg[16] = {
			.pin = EXYNOS5_GPG2(1),
			.name = "GPG2",
			.value = (2<<4),
			.act = GPIO_PULL_NONE,
		},
		.cfg[17] = {
			.pin = EXYNOS5_GPH0(3),
			.name = "GPH0",
			.value = (2<<12),
			.act = GPIO_PULL_NONE,
		},
		.reset_myself = {
			.pin = EXYNOS5_GPX1(2),
			.name = "GPX1",
			.value = 0,
			.act = GPIO_RESET,

		},
	},
};
#endif

#if defined CONFIG_VIDEO_S5K6A3
static struct exynos5_fimc_is_sensor_info s5k6a3 = {
	.sensor_name = "S5K6A3",
	.sensor_id = SENSOR_NAME_S5K6A3,
#if defined CONFIG_S5K6A3_POSITION_FRONT
	.sensor_position = SENSOR_POSITION_FRONT,
#elif defined CONFIG_S5K6A3_POSITION_REAR
	.sensor_position = SENSOR_POSITION_REAR,
#endif
#if defined CONFIG_S5K6A3_CSI_C
	.csi_id = CSI_ID_A,
	.flite_id = FLITE_ID_A,
	.i2c_channel = SENSOR_CONTROL_I2C0,
#elif defined CONFIG_S5K6A3_CSI_D
	.csi_id = CSI_ID_B,
	.flite_id = FLITE_ID_B,
	.i2c_channel = SENSOR_CONTROL_I2C1,
#endif
	.max_width = 1280,
	.max_height = 720,
	.max_frame_rate = 30,

	.mipi_lanes = 1,
	.mipi_settle = 12,
	.mipi_align = 24,
	.sensor_power = {
		.cam_core = "5m_core_1.5v",
		.cam_io_myself = "vt_cam_1.8v",
		.cam_io_peer = "cam_io_1.8v",
	},
	.sensor_gpio = {
		.cfg[0] = {
			.pin = EXYNOS5_GPE0(0),
			.name = "GPE0",
			.value = (2<<0),
			.act = GPIO_PULL_NONE,
		},
		.cfg[1] = {
			.pin = EXYNOS5_GPE0(1),
			.name = "GPE0",
			.value = (2<<4),
			.act = GPIO_PULL_NONE,
		},
		.cfg[2] = {
			.pin = EXYNOS5_GPE0(2),
			.name = "GPE0",
			.value = (3<<8),
			.act = GPIO_PULL_NONE,
		},
		.cfg[3] = {
			.pin = EXYNOS5_GPE0(3),
			.name = "GPE0",
			.value = (3<<12),
			.act = GPIO_PULL_NONE,
		},
		.cfg[4] = {
			.pin = EXYNOS5_GPE0(4),
			.name = "GPE0",
			.value = (3<<16),
			.act = GPIO_PULL_NONE,
		},
		.cfg[5] = {
			.pin = EXYNOS5_GPE0(5),
			.name = "GPE0",
			.value = (3<<20),
			.act = GPIO_PULL_NONE,
		},
		.cfg[6] = {
			.pin = EXYNOS5_GPE0(6),
			.name = "GPE0",
			.value = (3<<24),
			.act = GPIO_PULL_NONE,
		},
		.cfg[7] = {
			.pin = EXYNOS5_GPE0(7),
			.name = "GPE0",
			.value = (3<<28),
			.act = GPIO_PULL_NONE,
		},
		.cfg[8] = {
			.pin = EXYNOS5_GPE1(0),
			.name = "GPE1",
			.value = (3<<0),
			.act = GPIO_PULL_NONE,
		},
		.cfg[9] = {
			.pin = EXYNOS5_GPE1(1),
			.name = "GPE1",
			.value = (3<<4),
			.act = GPIO_PULL_NONE,
		},
		.cfg[10] = {
			.pin = EXYNOS5_GPF0(0),
			.name = "GPF0",
			.value = (2<<0),
			.act = GPIO_PULL_NONE,
		},
		.cfg[11] = {
			.pin = EXYNOS5_GPF0(1),
			.name = "GPF0",
			.value = (2<<4),
			.act = GPIO_PULL_NONE,
		},
		.cfg[12] = {
			.pin = EXYNOS5_GPF0(2),
			.name = "GPF0",
			.value = (2<<8),
			.act = GPIO_PULL_NONE,
		},
		.cfg[13] = {
			.pin = EXYNOS5_GPF0(3),
			.name = "GPF0",
			.value = (2<<12),
			.act = GPIO_PULL_NONE,
		},
		.cfg[14] = {
			.pin = EXYNOS5_GPF1(0),
			.name = "GPF1",
			.value = (3<<0),
			.act = GPIO_PULL_NONE,
		},
		.cfg[15] = {
			.pin = EXYNOS5_GPF1(1),
			.name = "GPF1",
			.value = (3<<4),
			.act = GPIO_PULL_NONE,
		},
		.cfg[16] = {
			.pin = EXYNOS5_GPG2(1),
			.name = "GPG2",
			.value = (2<<4),
			.act = GPIO_PULL_NONE,
		},
		.cfg[17] = {
			.pin = EXYNOS5_GPH0(3),
			.name = "GPH0",
			.value = (2<<12),
			.act = GPIO_PULL_NONE,
		},
		.reset_myself = {
			.pin = EXYNOS5_GPX1(0),
			.name = "GPX1",
			.value = 0,
			.act = GPIO_RESET,

		},
	},
};
#endif
#endif

#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
static void __init smdk5250_camera_gpio_cfg(void)
{
	/* CAM A port(b0010) : PCLK, VSYNC, HREF, CLK_OUT */
	s3c_gpio_cfgrange_nopull(EXYNOS5_GPH0(0), 4, S3C_GPIO_SFN(2));
	/* CAM A port(b0010) : DATA[0-7] */
	s3c_gpio_cfgrange_nopull(EXYNOS5_GPH1(0), 8, S3C_GPIO_SFN(2));
	/* CAM B port(b0010) : PCLK, BAY_RGB[0-6] */
	s3c_gpio_cfgrange_nopull(EXYNOS5_GPG0(0), 8, S3C_GPIO_SFN(2));
	/* CAM B port(b0010) : BAY_Vsync, BAY_RGB[7-13] */
	s3c_gpio_cfgrange_nopull(EXYNOS5_GPG1(0), 8, S3C_GPIO_SFN(2));
	/* CAM B port(b0010) : BAY_Hsync, BAY_MCLK */
	s3c_gpio_cfgrange_nopull(EXYNOS5_GPG2(0), 2, S3C_GPIO_SFN(2));
	/* This is externel interrupt for m5mo */
#ifdef CONFIG_VIDEO_M5MOLS
	s3c_gpio_cfgpin(EXYNOS5_GPX2(6), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS5_GPX2(6), S3C_GPIO_PULL_NONE);
#endif
}
#endif

#if defined(CONFIG_VIDEO_EXYNOS_GSCALER) && \
	defined(CONFIG_VIDEO_EXYNOS_FIMC_LITE)
#if defined(CONFIG_VIDEO_M5MOLS)
static struct exynos_isp_info m5mols = {
	.board_info	= &m5mols_board_info,
	.cam_srclk_name	= "xxti",
	.clk_frequency  = 24000000UL,
	.bus_type	= CAM_TYPE_MIPI,
#ifdef CONFIG_CSI_C
	.cam_clk_name	= "sclk_cam0",
	.i2c_bus_num	= 4,
	.cam_port	= CAM_PORT_A, /* A-Port : 0, B-Port : 1 */
#endif
#ifdef CONFIG_CSI_D
	.cam_clk_name	= "sclk_cam1",
	.i2c_bus_num	= 5,
	.cam_port	= CAM_PORT_B, /* A-Port : 0, B-Port : 1 */
#endif
	.flags		= CAM_CLK_INV_PCLK | CAM_CLK_INV_VSYNC,
	.csi_data_align = 32,
};
/* This is for platdata of fimc-lite */
static struct s3c_platform_camera flite_m5mo = {
	.type		= CAM_TYPE_MIPI,
	.use_isp	= true,
	.inv_pclk	= 1,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
};
#endif

static void __set_gsc_camera_config(struct exynos_platform_gscaler *data,
					u32 active_index, u32 preview,
					u32 camcording, u32 max_cam)
{
	data->active_cam_index = active_index;
	data->cam_preview = preview;
	data->cam_camcording = camcording;
	data->num_clients = max_cam;
}

static void __set_flite_camera_config(struct exynos_platform_flite *data,
					u32 active_index, u32 max_cam)
{
	data->active_cam_index = active_index;
	data->num_clients = max_cam;
}

static void __init smdk5250_set_camera_platdata(void)
{
	int gsc_cam_index = 0;
	int flite0_cam_index = 0;
	int flite1_cam_index = 0;
#if defined(CONFIG_VIDEO_M5MOLS)
	exynos_gsc0_default_data.isp_info[gsc_cam_index++] = &m5mols;
#if defined(CONFIG_CSI_C)
	exynos_flite0_default_data.cam[flite0_cam_index] = &flite_m5mo;
	exynos_flite0_default_data.isp_info[flite0_cam_index] = &m5mols;
	flite0_cam_index++;
#endif
#if defined(CONFIG_CSI_D)
	exynos_flite1_default_data.cam[flite1_cam_index] = &flite_m5mo;
	exynos_flite1_default_data.isp_info[flite1_cam_index] = &m5mols;
	flite1_cam_index++;
#endif
#endif
	/* flite platdata register */
	__set_flite_camera_config(&exynos_flite0_default_data,
			0, flite0_cam_index);
	__set_flite_camera_config(&exynos_flite1_default_data,
			0, flite1_cam_index);

	/* gscaler platdata register */
	/* GSC-0 */
	__set_gsc_camera_config(&exynos_gsc0_default_data,
			0, 1, 0, gsc_cam_index);

	/* GSC-1 */
	/* GSC-2 */
	/* GSC-3 */
}
#endif /* CONFIG_VIDEO_EXYNOS_GSCALER */

static struct platform_device *smdk5250_camera_devices[] __initdata = {
#ifdef CONFIG_VIDEO_EXYNOS5_FIMC_IS2
	&exynos5_device_fimc_is,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
	&exynos_device_flite0,
	&exynos_device_flite1,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_MIPI_CSIS
	&s5p_device_mipi_csis0,
	&s5p_device_mipi_csis1,
	&mipi_csi_fixed_voltage,
#endif
#ifdef CONFIG_VIDEO_M5MOLS
	&m5mols_fixed_voltage,
#endif
};

static void __init smdk5250_camera_sysmmu_init(void)
{
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
	platform_set_sysmmu(&SYSMMU_PLATDEV(camif0).dev,
						&exynos_device_flite0.dev);
	platform_set_sysmmu(&SYSMMU_PLATDEV(camif1).dev,
						&exynos_device_flite1.dev);
#endif
#ifdef CONFIG_VIDEO_EXYNOS5_FIMC_IS2
	platform_set_sysmmu(&SYSMMU_PLATDEV(isp).dev,
						&exynos5_device_fimc_is.dev);
#endif
}

void __init exynos5_smdk5250_camera_init(void)
{
	smdk5250_camera_sysmmu_init();

#ifdef CONFIG_VIDEO_EXYNOS_MIPI_CSIS
	s3c_set_platdata(&s5p_mipi_csis0_default_data,
			sizeof(s5p_mipi_csis0_default_data),
			&s5p_device_mipi_csis0);
	s3c_set_platdata(&s5p_mipi_csis1_default_data,
			sizeof(s5p_mipi_csis1_default_data),
			&s5p_device_mipi_csis1);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
	smdk5250_camera_gpio_cfg();
	smdk5250_set_camera_platdata();
	s3c_set_platdata(&exynos_flite0_default_data,
			sizeof(exynos_flite0_default_data),
			&exynos_device_flite0);
	s3c_set_platdata(&exynos_flite1_default_data,
			sizeof(exynos_flite1_default_data),
			&exynos_device_flite1);
/* In EVT0, for using camclk, gscaler clock should be enabled */
	dev_set_name(&exynos_device_flite0.dev, "exynos-gsc.0");
	clk_add_alias("gscl", "exynos-fimc-lite.0", "gscl",
			&exynos_device_flite0.dev);
	dev_set_name(&exynos_device_flite0.dev, "exynos-fimc-lite.0");

	dev_set_name(&exynos_device_flite1.dev, "exynos-gsc.0");
	clk_add_alias("gscl", "exynos-fimc-lite.1", "gscl",
			&exynos_device_flite1.dev);
	dev_set_name(&exynos_device_flite1.dev, "exynos-fimc-lite.1");
#endif
#ifdef CONFIG_VIDEO_EXYNOS5_FIMC_IS2
	dev_set_name(&exynos5_device_fimc_is.dev, "s5p-mipi-csis.0");
	clk_add_alias("gscl_wrap0", FIMC_IS_MODULE_NAME,
			"gscl_wrap0", &exynos5_device_fimc_is.dev);
	clk_add_alias("sclk_gscl_wrap0", FIMC_IS_MODULE_NAME,
			"sclk_gscl_wrap0", &exynos5_device_fimc_is.dev);

	dev_set_name(&exynos5_device_fimc_is.dev, "s5p-mipi-csis.1");
	clk_add_alias("gscl_wrap1", FIMC_IS_MODULE_NAME,
			"gscl_wrap1", &exynos5_device_fimc_is.dev);
	clk_add_alias("sclk_gscl_wrap1", FIMC_IS_MODULE_NAME,
			"sclk_gscl_wrap1", &exynos5_device_fimc_is.dev);

	dev_set_name(&exynos5_device_fimc_is.dev, "exynos-gsc.0");
	clk_add_alias("gscl", FIMC_IS_MODULE_NAME,
			"gscl", &exynos5_device_fimc_is.dev);
	dev_set_name(&exynos5_device_fimc_is.dev, FIMC_IS_MODULE_NAME);

#if defined CONFIG_VIDEO_S5K6A3
	exynos5_fimc_is_data.sensor_info[s5k6a3.sensor_position] = &s5k6a3;
#endif
#if defined CONFIG_VIDEO_S5K4E5
	exynos5_fimc_is_data.sensor_info[s5k4e5.sensor_position] = &s5k4e5;
#endif

	exynos5_fimc_is_set_platdata(&exynos5_fimc_is_data);
#endif
	platform_add_devices(smdk5250_camera_devices,
			ARRAY_SIZE(smdk5250_camera_devices));
}
