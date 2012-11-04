/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/cma.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/persistent_ram.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>

#include <media/m5mols.h>
#include <media/exynos_gscaler.h>
#include <media/exynos_flite.h>
#include <media/exynos_fimc_is.h>

#include <plat/adc.h>
#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/regs-serial.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/iic.h>
#include <plat/mipi_csis.h>
#include <plat/jpeg.h>

#include <mach/exynos_fiq_debugger.h>
#include <mach/map.h>
#include <mach/sysmmu.h>
#include <mach/exynos-ion.h>
#include <mach/exynos-mfc.h>
#include <mach/tmu.h>

#include <plat/fimg2d.h>

#include "common.h"
#include "board-smdk5250.h"

static char smdk5250_board_info_string[255];

static void smdk5250_init_hw_rev(void)
{
	snprintf(smdk5250_board_info_string, sizeof(smdk5250_board_info_string) - 1,
		 "SMDK HW revision: %d, CPU EXYNOS5250 Rev%d.%d",
		 get_smdk5250_rev(),
		 samsung_rev() >> 4,
		 samsung_rev() & 0xf);
	pr_info("%s\n", smdk5250_board_info_string);
	mach_panic_string = smdk5250_board_info_string;
}

static struct platform_device ramconsole_device = {
	.name           = "ram_console",
	.id             = -1,
};

static struct platform_device persistent_trace_device = {
	.name           = "persistent_trace",
	.id             = -1,
};

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDK5250_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDK5250_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDK5250_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg smdk5250_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDK5250_UCON_DEFAULT,
		.ulcon		= SMDK5250_ULCON_DEFAULT,
		.ufcon		= SMDK5250_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDK5250_UCON_DEFAULT,
		.ulcon		= SMDK5250_ULCON_DEFAULT,
		.ufcon		= SMDK5250_UFCON_DEFAULT,
	},
	[2] = {
#ifndef CONFIG_EXYNOS_FIQ_DEBUGGER
	/*
	 * Don't need to initialize hwport 2, when FIQ debugger is
	 * enabled. Because it will be handled by fiq_debugger.
	 */
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDK5250_UCON_DEFAULT,
		.ulcon		= SMDK5250_ULCON_DEFAULT,
		.ufcon		= SMDK5250_UFCON_DEFAULT,
	},
	[3] = {
#endif
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDK5250_UCON_DEFAULT,
		.ulcon		= SMDK5250_ULCON_DEFAULT,
		.ufcon		= SMDK5250_UFCON_DEFAULT,
	},
};

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

#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
#if defined(CONFIG_ITU_A)
static int smdk5250_cam0_reset(int dummy)
{
	int err;
	/* Camera A */
	err = gpio_request(EXYNOS5_GPX1(2), "GPX1");
	if (err)
		printk(KERN_ERR "#### failed to request GPX1_2 ####\n");

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

/* ADC */
static struct s3c_adc_platdata smdk5250_adc_data __initdata = {
	.phy_init       = s3c_adc_phy_init,
	.phy_exit       = s3c_adc_phy_exit,
};

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

#if defined(CONFIG_VIDEO_EXYNOS_GSCALER) && defined(CONFIG_VIDEO_EXYNOS_FIMC_LITE)
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
	__set_flite_camera_config(&exynos_flite0_default_data, 0, flite0_cam_index);
	__set_flite_camera_config(&exynos_flite1_default_data, 0, flite1_cam_index);

	/* gscaler platdata register */
	/* GSC-0 */
	__set_gsc_camera_config(&exynos_gsc0_default_data, 0, 1, 0, gsc_cam_index);

	/* GSC-1 */
	/* GSC-2 */
	/* GSC-3 */
}
#endif /* CONFIG_VIDEO_EXYNOS_GSCALER */

#ifdef CONFIG_VIDEO_EXYNOS_FIMG2D
static struct fimg2d_platdata fimg2d_data __initdata = {
	.hw_ver		= 0x42,
	.gate_clkname	= "fimg2d",
};
#endif

#ifdef CONFIG_BATTERY_SAMSUNG
static struct platform_device samsung_device_battery = {
	.name	= "samsung-fake-battery",
	.id	= -1,
};
#endif

/* DEVFREQ controlling mif */
static struct platform_device exynos_bus_mif_devfreq = {
	.name                   = "exynos5-bus-mif",
};

/* DEVFREQ controlling int */
static struct platform_device exynos_bus_int_devfreq = {
	.name                   = "exynos5-bus-int",
};


static struct platform_device *smdk5250_devices[] __initdata = {
	&ramconsole_device,
	&persistent_trace_device,
	&s3c_device_rtc,
	&s3c_device_i2c0,
	&s3c_device_i2c4,
	&s3c_device_i2c5,
	&s3c_device_adc,
	&s3c_device_wdt,
#ifdef CONFIG_VIDEO_EXYNOS_MFC
	&s5p_device_mfc,
#endif
#ifdef CONFIG_ION_EXYNOS
	&exynos_device_ion,
#endif
#ifdef CONFIG_BATTERY_SAMSUNG
	&samsung_device_battery,
#endif
#ifdef CONFIG_EXYNOS_DEV_TMU
	&exynos_device_tmu,
#endif
#ifdef CONFIG_EXYNOS_MEDIA_DEVICE
	&exynos_device_md0,
	&exynos_device_md1,
	&exynos_device_md2,
#endif
#ifdef CONFIG_VIDEO_EXYNOS5_FIMC_IS2
	&exynos5_device_fimc_is,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_GSCALER
	&exynos5_device_gsc0,
	&exynos5_device_gsc1,
	&exynos5_device_gsc2,
	&exynos5_device_gsc3,
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
#ifdef CONFIG_VIDEO_EXYNOS_FIMG2D
	&s5p_device_fimg2d,
#endif
	&exynos5_device_rotator,
#ifdef CONFIG_VIDEO_M5MOLS
	&m5mols_fixed_voltage,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_JPEG
	&s5p_device_jpeg,
#endif
#ifdef CONFIG_S5P_DEV_ACE
	&s5p_device_ace,
#endif
	&exynos_bus_mif_devfreq,
	&exynos_bus_int_devfreq,
#ifdef CONFIG_MALI_T6XX
	&exynos5_device_g3d,
#endif
};

/* TMU */
static struct tmu_data smdk5250_tmu_pdata __initdata = {
	.ts = {
		.stop_throttle		= 78,
		.start_throttle		= 80,
		.start_tripping		= 110,
		.start_emergency	= 120,
		.stop_mem_throttle	= 80,
		.start_mem_throttle	= 85,
	},

	.efuse_value	= 80,
	.slope		= 0x10608802,
};

#if defined(CONFIG_CMA)
/* defined in arch/arm/mach-exynos/reserve-mem.c */
extern void exynos_cma_region_reserve(struct cma_region *,
				struct cma_region *, size_t, const char *);
static void __init exynos_reserve_mem(void)
{
	static struct cma_region regions[] = {
		{
			.name = "ion",
#ifdef CONFIG_ION_EXYNOS_CONTIGHEAP_SIZE
			.size = CONFIG_ION_EXYNOS_CONTIGHEAP_SIZE * SZ_1K,
#endif
			{
				.alignment = SZ_1M
			}
		},
#ifdef CONFIG_AUDIO_SAMSUNG_MEMSIZE_SRP
		{
			.name = "srp",
			.size = CONFIG_AUDIO_SAMSUNG_MEMSIZE_SRP * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_EXYNOS_CONTENT_PATH_PROTECTION
#ifdef CONFIG_ION_EXYNOS_DRM_MFC_SH
		{
			.name = "drm_mfc_sh",
			.size = SZ_1M,
		},
#endif
#ifdef CONFIG_ION_EXYNOS_DRM_MSGBOX_SH
		{
			.name = "drm_msgbox_sh",
			.size = SZ_1M,
		},
#endif
#endif
		{
			.size = 0
		},
	};
#ifdef CONFIG_EXYNOS_CONTENT_PATH_PROTECTION
	static struct cma_region regions_secure[] = {
#ifdef CONFIG_ION_EXYNOS_DRM_MEMSIZE_FIMD_VIDEO
	       {
		       .name = "drm_fimd_video",
		       .size = CONFIG_ION_EXYNOS_DRM_MEMSIZE_FIMD_VIDEO *
			       SZ_1K,
	       },
#endif
#ifdef CONFIG_ION_EXYNOS_DRM_MEMSIZE_MFC_OUTPUT
	       {
		       .name = "drm_mfc_output",
		       .size = CONFIG_ION_EXYNOS_DRM_MEMSIZE_MFC_OUTPUT *
			       SZ_1K,
	       },
#endif
#ifdef CONFIG_ION_EXYNOS_DRM_MEMSIZE_MFC_INPUT
	       {
		       .name = "drm_mfc_input",
		       .size = CONFIG_ION_EXYNOS_DRM_MEMSIZE_MFC_INPUT *
			       SZ_1K,
	       },
#endif
#ifdef CONFIG_ION_EXYNOS_DRM_MFC_FW
		{
			.name = "drm_mfc_fw",
			.size = SZ_1M,
		},
#endif
#ifdef CONFIG_ION_EXYNOS_DRM_SECTBL
		{
			.name = "drm_sectbl",
			.size = SZ_1M,
		},
#endif
		{
			.size = 0
		},
	};
#else /* !CONFIG_EXYNOS_CONTENT_PATH_PROTECTION */
	struct cma_region *regions_secure = NULL;
#endif /* CONFIG_EXYNOS_CONTENT_PATH_PROTECTION */
	static const char map[] __initconst =
#ifdef CONFIG_EXYNOS_C2C
		"samsung-c2c=c2c_shdmem;"
#endif
		"samsung-rp=srp;"
#ifdef CONFIG_EXYNOS_CONTENT_PATH_PROTECTION
		"ion-exynos/mfc_sh=drm_mfc_sh;"
		"ion-exynos/msgbox_sh=drm_msgbox_sh;"
		"ion-exynos/fimd_video=drm_fimd_video;"
		"ion-exynos/mfc_output=drm_mfc_output;"
		"ion-exynos/mfc_input=drm_mfc_input;"
		"ion-exynos/mfc_fw=drm_mfc_fw;"
		"ion-exynos/sectbl=drm_sectbl;"
		"s5p-smem/mfc_sh=drm_mfc_sh;"
		"s5p-smem/msgbox_sh=drm_msgbox_sh;"
		"s5p-smem/fimd_video=drm_fimd_video;"
		"s5p-smem/mfc_output=drm_mfc_output;"
		"s5p-smem/mfc_input=drm_mfc_input;"
		"s5p-smem/mfc_fw=drm_mfc_fw;"
		"s5p-smem/sectbl=drm_sectbl;"
#endif
		"ion-exynos=ion;"
		"exynos-rot=rot;"
		"s5p-mfc-v6/f=fw;"
		"s5p-mfc-v6/a=b1;";

	exynos_cma_region_reserve(regions, regions_secure, 0, map);
}
#else /* !CONFIG_CMA*/
static inline void exynos_reserve_mem(void)
{
}
#endif

#if defined(CONFIG_VIDEO_EXYNOS_MFC)
static struct s5p_mfc_platdata smdk5250_mfc_pd = {
	.clock_rate = 333 * MHZ,
};
#endif

static void __init smdk5250_map_io(void)
{
	clk_xusbxti.rate = 24000000;
	clk_xxti.rate = 24000000;
	exynos_init_io(NULL, 0);
	s3c24xx_init_clocks(clk_xusbxti.rate);
	s3c24xx_init_uarts(smdk5250_uartcfgs, ARRAY_SIZE(smdk5250_uartcfgs));
}

static void __init exynos_sysmmu_init(void)
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
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
	platform_set_sysmmu(&SYSMMU_PLATDEV(camif0).dev,
						&exynos_device_flite0.dev);
	platform_set_sysmmu(&SYSMMU_PLATDEV(camif1).dev,
						&exynos_device_flite1.dev);
#endif
	platform_set_sysmmu(&SYSMMU_PLATDEV(rot).dev,
						&exynos5_device_rotator.dev);
#ifdef CONFIG_VIDEO_EXYNOS_FIMG2D
	platform_set_sysmmu(&SYSMMU_PLATDEV(2d).dev,
						&s5p_device_fimg2d.dev);
#endif
#ifdef CONFIG_VIDEO_EXYNOS5_FIMC_IS2
	platform_set_sysmmu(&SYSMMU_PLATDEV(isp).dev,
						&exynos5_device_fimc_is.dev);
#endif
}

static struct persistent_ram_descriptor smdk5250_prd[] __initdata = {
	{
		.name = "ram_console",
		.size = SZ_2M,
	},
#ifdef CONFIG_PERSISTENT_TRACER
	{
		.name = "persistent_trace",
		.size = SZ_1M,
	},
#endif
};

static struct persistent_ram smdk5250_pr __initdata = {
	.descs = smdk5250_prd,
	.num_descs = ARRAY_SIZE(smdk5250_prd),
	.start = PLAT_PHYS_OFFSET + SZ_1G + SZ_512M,
#ifdef CONFIG_PERSISTENT_TRACER
	.size = 3 * SZ_1M,
#else
	.size = SZ_2M,
#endif
};

static void __init smdk5250_init_early(void)
{
	persistent_ram_early_init(&smdk5250_pr);
}

static void __init smdk5250_machine_init(void)
{
	smdk5250_init_hw_rev();

#ifdef CONFIG_EXYNOS_FIQ_DEBUGGER
	exynos_serial_debug_init(2, 0);
#endif

	s3c_i2c0_set_platdata(NULL);
	s3c_i2c4_set_platdata(NULL);
	s3c_i2c5_set_platdata(NULL);

	s3c_adc_set_platdata(&smdk5250_adc_data);

	exynos_sysmmu_init();
	exynos_ion_set_platdata();

#ifdef CONFIG_VIDEO_EXYNOS_MFC
	s5p_mfc_set_platdata(&smdk5250_mfc_pd);
#endif
#ifdef CONFIG_EXYNOS_DEV_TMU
	exynos_tmu_set_platdata(&smdk5250_tmu_pdata);
#endif
	exynos5_smdk5250_mmc_init();

	platform_add_devices(smdk5250_devices, ARRAY_SIZE(smdk5250_devices));

	exynos5_smdk5250_audio_init();
	exynos5_smdk5250_input_init();
	exynos5_smdk5250_usb_init();
	exynos5_smdk5250_power_init();
	exynos5_smdk5250_spi_init();
	exynos5_smdk5250_display_init();
	exynos5_smdk5250_tvout_init();

#ifdef CONFIG_VIDEO_EXYNOS_MIPI_CSIS
	s3c_set_platdata(&s5p_mipi_csis0_default_data,
			sizeof(s5p_mipi_csis0_default_data), &s5p_device_mipi_csis0);
	s3c_set_platdata(&s5p_mipi_csis1_default_data,
			sizeof(s5p_mipi_csis1_default_data), &s5p_device_mipi_csis1);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMG2D
	s5p_fimg2d_set_platdata(&fimg2d_data);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
	smdk5250_camera_gpio_cfg();
	smdk5250_set_camera_platdata();
	s3c_set_platdata(&exynos_flite0_default_data,
			sizeof(exynos_flite0_default_data), &exynos_device_flite0);
	s3c_set_platdata(&exynos_flite1_default_data,
			sizeof(exynos_flite1_default_data), &exynos_device_flite1);
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
#ifdef CONFIG_VIDEO_EXYNOS_GSCALER
	s3c_set_platdata(&exynos_gsc0_default_data, sizeof(exynos_gsc0_default_data),
			&exynos5_device_gsc0);
	s3c_set_platdata(&exynos_gsc1_default_data, sizeof(exynos_gsc1_default_data),
			&exynos5_device_gsc1);
	s3c_set_platdata(&exynos_gsc2_default_data, sizeof(exynos_gsc2_default_data),
			&exynos5_device_gsc2);
	s3c_set_platdata(&exynos_gsc3_default_data, sizeof(exynos_gsc3_default_data),
			&exynos5_device_gsc3);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_JPEG
	exynos5_jpeg_setup_clock(&s5p_device_jpeg.dev, 150000000);
#endif
}

MACHINE_START(SMDK5250, "SMDK5250")
	.atag_offset	= 0x100,
	.init_early	= smdk5250_init_early,
	.init_irq	= exynos5_init_irq,
	.map_io		= smdk5250_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= smdk5250_machine_init,
	.timer		= &exynos4_timer,
	.restart	= exynos5_restart,
	.reserve	= exynos_reserve_mem,
MACHINE_END
