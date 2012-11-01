/* linux/arch/arm/mach-exynos/board-smdk5250-power.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/mfd/max77686.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/s5m8767.h>
#include <linux/regulator/machine.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>

#include <asm/system_misc.h>

#include <plat/iic.h>
#include <plat/gpio-cfg.h>

#include <mach/regs-pmu.h>

#include "board-smdk5250.h"
#include "common.h"

#define SMDK5250_PMIC_EINT	IRQ_EINT(26)

#define REBOOT_MODE_PREFIX	0x12345670
#define REBOOT_MODE_NONE	0
#define REBOOT_MODE_RECOVERY	4
#define REBOOT_MODE_FAST_BOOT	7

#define REBOOT_MODE_NO_LPM	0x12345678
#define REBOOT_MODE_LPM		0

static struct regulator_consumer_supply ldo3_supply[] = {
	REGULATOR_SUPPLY("vcc_1.8v", NULL),
	REGULATOR_SUPPLY("AVDD2", NULL),
	REGULATOR_SUPPLY("CPVDD", NULL),
	REGULATOR_SUPPLY("DBVDD1", NULL),
	REGULATOR_SUPPLY("DBVDD2", NULL),
	REGULATOR_SUPPLY("DBVDD3", NULL)
};

static struct regulator_consumer_supply ldo8_supply[] = {
	REGULATOR_SUPPLY("vmipi_1.0v", NULL),
};

static struct regulator_consumer_supply ldo9_supply[] = {
	REGULATOR_SUPPLY("vdd", "3-004a"),
};

static struct regulator_consumer_supply ldo10_supply[] = {
	REGULATOR_SUPPLY("vmipi_1.8v", NULL),
};

static struct regulator_consumer_supply ldo12_supply[] = {
	REGULATOR_SUPPLY("votg_3.0v", NULL),
};

static struct regulator_consumer_supply ldo15_supply[] = {
	REGULATOR_SUPPLY("vhsic_1.0v", NULL),
};

static struct regulator_consumer_supply ldo16_supply[] = {
	REGULATOR_SUPPLY("vhsic_1.8v", NULL),
};

static struct regulator_consumer_supply ldo17_supply[] = {
	REGULATOR_SUPPLY("5m_core_1.5v", NULL),
};

static struct regulator_consumer_supply ldo18_supply[] = {
	REGULATOR_SUPPLY("cam_io_1.8v", NULL),
};

static struct regulator_consumer_supply ldo19_supply[] = {
	REGULATOR_SUPPLY("vt_cam_1.8v", NULL),
};

static struct regulator_consumer_supply ldo23_supply[] = {
	REGULATOR_SUPPLY("avdd", "3-004a"),
};

static struct regulator_consumer_supply ldo24_supply[] = {
	REGULATOR_SUPPLY("cam_af_2.8v", NULL),
};

static struct regulator_consumer_supply ldo25_supply[] = {
	REGULATOR_SUPPLY("vadc_3.3v", NULL),
};

static struct regulator_consumer_supply max77686_buck1 =
REGULATOR_SUPPLY("vdd_mif", NULL);

static struct regulator_consumer_supply max77686_buck2 =
REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply max77686_buck3 =
REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply max77686_buck4 =
REGULATOR_SUPPLY("vdd_g3d", NULL);

static struct regulator_consumer_supply max77686_enp32khz[] = {
	REGULATOR_SUPPLY("lpo_in", "bcm47511"),
	REGULATOR_SUPPLY("lpo", "bcm4334_bluetooth"),
};

#define REGULATOR_INIT(_ldo, _name, _min_uV, _max_uV, _always_on, _ops_mask, \
		       _disabled)					\
	static struct regulator_init_data _ldo##_init_data = {		\
		.constraints = {					\
			.name	= _name,				\
			.min_uV = _min_uV,				\
			.max_uV = _max_uV,				\
			.always_on	= _always_on,			\
			.boot_on	= _always_on,			\
			.apply_uV	= 1,				\
			.valid_ops_mask = _ops_mask,			\
			.state_mem	= {				\
				.disabled =				\
					(_disabled == -1 ? 0 : _disabled),\
				.enabled =				\
					(_disabled == -1 ? 0 : !(_disabled)),\
			},						\
		},							\
		.num_consumer_supplies = ARRAY_SIZE(_ldo##_supply),	\
		.consumer_supplies = &_ldo##_supply[0],			\
	};

REGULATOR_INIT(ldo3, "VCC_1.8V_AP", 1800000, 1800000, 1, 0, 0);
REGULATOR_INIT(ldo8, "VMIPI_1.0V", 1000000, 1000000, 1,
	       REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo9, "TOUCH_VDD_1.8V", 1800000, 1800000, 0,
	       REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo10, "VMIPI_1.8V", 1800000, 1800000, 1,
	       REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo12, "VUOTG_3.0V", 3000000, 3000000, 1,
	       REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo15, "VHSIC_1.0V", 1000000, 1000000, 1,
	       REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo16, "VHSIC_1.8V", 1800000, 1800000, 1,
	       REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo17, "5M_CORE_1.5V", 1500000, 1500000, 0,
	       REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo18, "CAM_IO_1.8V", 1800000, 1800000, 0,
	       REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo19, "VT_CAM_1.8V", 1800000, 1800000, 0,
	       REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo23, "TSP_AVDD_2.8V", 2800000, 2800000, 0,
	       REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo24, "CAM_AF_2.8V", 2800000, 2800000, 0,
	       REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo25, "VADC_3.3V", 3300000, 3300000, 1,
	       REGULATOR_CHANGE_STATUS, 1);

static struct regulator_init_data max77686_buck1_data = {
	.constraints = {
			.name = "vdd_mif range",
			.min_uV = 850000,
			.max_uV = 1200000,
			.always_on = 1,
			.boot_on = 1,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &max77686_buck1,
};

static struct regulator_init_data max77686_buck2_data = {
	.constraints = {
			.name = "vdd_arm range",
			.min_uV = 850000,
			.max_uV = 1500000,
			.always_on = 1,
			.boot_on = 1,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &max77686_buck2,
};

static struct regulator_init_data max77686_buck3_data = {
	.constraints = {
			.name = "vdd_int range",
			.min_uV = 850000,
			.max_uV = 1300000,
			.always_on = 1,
			.boot_on = 1,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &max77686_buck3,
};

static struct regulator_init_data max77686_buck4_data = {
	.constraints = {
			.name = "vdd_g3d range",
			.min_uV = 850000,
			.max_uV = 1250000,
			.boot_on = 1,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &max77686_buck4,
};

static struct regulator_init_data max77686_enp32khz_data = {
	.constraints = {
			.name = "32KHZ_PMIC",
			.always_on = 1,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.state_mem = {
				      .enabled = 1,
				      .disabled = 0,
				      },
			},
	.num_consumer_supplies = ARRAY_SIZE(max77686_enp32khz),
	.consumer_supplies = max77686_enp32khz,
};

static struct max77686_regulator_data max77686_regulators[] = {
	{MAX77686_BUCK1, &max77686_buck1_data,},
	{MAX77686_BUCK2, &max77686_buck2_data,},
	{MAX77686_BUCK3, &max77686_buck3_data,},
	{MAX77686_BUCK4, &max77686_buck4_data,},
	{MAX77686_LDO3, &ldo3_init_data,},
	{MAX77686_LDO8, &ldo8_init_data,},
	{MAX77686_LDO9, &ldo9_init_data,},
	{MAX77686_LDO10, &ldo10_init_data,},
	{MAX77686_LDO12, &ldo12_init_data,},
	{MAX77686_LDO15, &ldo15_init_data,},
	{MAX77686_LDO16, &ldo16_init_data,},
	{MAX77686_LDO17, &ldo17_init_data,},
	{MAX77686_LDO18, &ldo18_init_data,},
	{MAX77686_LDO19, &ldo19_init_data,},
	{MAX77686_LDO23, &ldo23_init_data,},
	{MAX77686_LDO24, &ldo24_init_data,},
	{MAX77686_LDO25, &ldo25_init_data,},
	{MAX77686_P32KH, &max77686_enp32khz_data,},
};

struct max77686_opmode_data max77686_opmode_data[MAX77686_REG_MAX] = {
	[MAX77686_LDO3] = {MAX77686_LDO3, MAX77686_OPMODE_NORMAL},
	[MAX77686_LDO8] = {MAX77686_LDO8, MAX77686_OPMODE_STANDBY},
	[MAX77686_LDO10] = {MAX77686_LDO10, MAX77686_OPMODE_STANDBY},
	[MAX77686_LDO12] = {MAX77686_LDO12, MAX77686_OPMODE_STANDBY},
	[MAX77686_LDO15] = {MAX77686_LDO15, MAX77686_OPMODE_STANDBY},
	[MAX77686_LDO16] = {MAX77686_LDO16, MAX77686_OPMODE_STANDBY},
	[MAX77686_BUCK1] = {MAX77686_BUCK1, MAX77686_OPMODE_STANDBY},
	[MAX77686_BUCK2] = {MAX77686_BUCK2, MAX77686_OPMODE_STANDBY},
	[MAX77686_BUCK3] = {MAX77686_BUCK3, MAX77686_OPMODE_STANDBY},
	[MAX77686_BUCK4] = {MAX77686_BUCK4, MAX77686_OPMODE_STANDBY},
};

static struct max77686_wtsr_smpl wtsr_smpl_data = {
	.wtsr_en = true,
	.smpl_en = true,
	.wtsr_timer_val = 3,	/* 1000ms */
	.smpl_timer_val = 0,	/* 0.5s */
};

/* If it's first boot, reset rtc to 1/1/2012 12:00:00(SUN) */
static struct rtc_time init_time_data = {
	.tm_sec = 0,
	.tm_min = 0,
	.tm_hour = 12,
	.tm_wday = 0,
	.tm_mday = 1,
	.tm_mon = 0,
	.tm_year = 112,
	.tm_yday = 0,
	.tm_isdst = 0,
};

static struct max77686_platform_data smdk5250_max77686_info = {
	.num_regulators = ARRAY_SIZE(max77686_regulators),
	.regulators = max77686_regulators,
	.irq_gpio = 0,
	.irq_base = 0,
	.wakeup = 0,

	.opmode_data = max77686_opmode_data,
	.ramp_rate = MAX77686_RAMP_RATE_27MV,
	.has_full_constraints = 1,

	.buck234_gpio_dvs = {
			     EXYNOS5_GPV0(7),	/* GPIO_PMIC_DVS1, */
			     EXYNOS5_GPV0(6),	/* GPIO_PMIC_DVS2, */
			     EXYNOS5_GPV0(5),	/* GPIO_PMIC_DVS3, */
			     },
	.buck234_gpio_selb = {
			      EXYNOS5_GPV0(4),	/* GPIO_BUCK2_SEL, */
			      EXYNOS5_GPV0(1),	/* GPIO_BUCK3_SEL, */
			      EXYNOS5_GPV0(0),	/* GPIO_BUCK4_SEL, */
			      },

	/* for future work after DVS Table */
	.buck2_voltage[0] = 1100000,	/* 1.1V */
	.buck2_voltage[1] = 1100000,	/* 1.1V */
	.buck2_voltage[2] = 1100000,	/* 1.1V */
	.buck2_voltage[3] = 1100000,	/* 1.1V */
	.buck2_voltage[4] = 1100000,	/* 1.1V */
	.buck2_voltage[5] = 1100000,	/* 1.1V */
	.buck2_voltage[6] = 1100000,	/* 1.1V */
	.buck2_voltage[7] = 1100000,	/* 1.1V */

	.buck3_voltage[0] = 1100000,	/* 1.1V */
	.buck3_voltage[1] = 1100000,	/* 1.1V */
	.buck3_voltage[2] = 1100000,	/* 1.1V */
	.buck3_voltage[3] = 1100000,	/* 1.1V */
	.buck3_voltage[4] = 1100000,	/* 1.1V */
	.buck3_voltage[5] = 1100000,	/* 1.1V */
	.buck3_voltage[6] = 1100000,	/* 1.1V */
	.buck3_voltage[7] = 1100000,	/* 1.1V */

	.buck4_voltage[0] = 1100000,	/* 1.1V */
	.buck4_voltage[1] = 1100000,	/* 1.1V */
	.buck4_voltage[2] = 1100000,	/* 1.1V */
	.buck4_voltage[3] = 1100000,	/* 1.1V */
	.buck4_voltage[4] = 1100000,	/* 1.1V */
	.buck4_voltage[5] = 1100000,	/* 1.1V */
	.buck4_voltage[6] = 1100000,	/* 1.1V */
	.buck4_voltage[7] = 1100000,	/* 1.1V */

	/* for RTC */
	.wtsr_smpl = &wtsr_smpl_data,
	.init_time = &init_time_data,
};

/* S5M8767 Regulator */
static struct regulator_consumer_supply s5m8767_buck1_consumer =
	REGULATOR_SUPPLY("vdd_mif", NULL);

static struct regulator_consumer_supply s5m8767_buck2_consumer =
	REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply s5m8767_buck3_consumer =
	REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply s5m8767_buck4_consumer =
	REGULATOR_SUPPLY("vdd_g3d", NULL);

static struct regulator_consumer_supply s5m8767_ldo4_consumer =
	REGULATOR_SUPPLY("vdd_ldo4", NULL);

static struct regulator_consumer_supply s5m8767_ldo11_consumer =
	REGULATOR_SUPPLY("vdd_ldo11", NULL);

static struct regulator_consumer_supply s5m8767_ldo14_consumer =
	REGULATOR_SUPPLY("vdd_ldo14", NULL);

static struct regulator_consumer_supply s5m8767_ldo17_consumer =
	REGULATOR_SUPPLY("5m_core_1.5v", NULL);

static struct regulator_consumer_supply s5m8767_ldo18_consumer =
	REGULATOR_SUPPLY("cam_io_1.8v", NULL);

static struct regulator_consumer_supply s5m8767_ldo19_consumer =
	REGULATOR_SUPPLY("vt_cam_1.8v", NULL);

static struct regulator_consumer_supply s5m8767_ldo24_consumer =
	REGULATOR_SUPPLY("cam_af_2.8v", NULL);

static struct regulator_init_data s5m8767_buck1_data = {
	.constraints	= {
		.name		= "vdd_mif range",
		.min_uV		=  950000,
		.max_uV		= 1300000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck1_consumer,
};

static struct regulator_init_data s5m8767_buck2_data = {
	.constraints	= {
		.name		= "vdd_arm range",
		.min_uV		=  800000,
		.max_uV		= 1350000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck2_consumer,
};

static struct regulator_init_data s5m8767_buck3_data = {
	.constraints	= {
		.name		= "vdd_int range",
		.min_uV		=  900000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck3_consumer,
};

static struct regulator_init_data s5m8767_buck4_data = {
	.constraints	= {
		.name		= "vdd_g3d range",
		.min_uV		=  700000,
		.max_uV		= 1300000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.boot_on = 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck4_consumer,
};

static struct regulator_init_data s5m8767_ldo4_data = {
	.constraints	= {
		.name		= "vdd_ldo4 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_ldo4_consumer,
};

static struct regulator_init_data s5m8767_ldo11_data = {
	.constraints	= {
		.name		= "vdd_ldo11 range",
		.min_uV		= 1900000,
		.max_uV		= 1900000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_ldo11_consumer,
};

static struct regulator_init_data s5m8767_ldo14_data = {
	.constraints	= {
		.name		= "vdd_ldo14 range",
		.min_uV		= 1900000,
		.max_uV		= 1900000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_ldo14_consumer,
};

static struct regulator_init_data s5m8767_ldo17_data = {
	.constraints	= {
		.name		= "5m_core_1.5v",
		.min_uV		= 1500000,
		.max_uV		= 1500000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_ldo17_consumer,
};

static struct regulator_init_data s5m8767_ldo18_data = {
	.constraints	= {
		.name		= "cam_io_1.8v",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_ldo18_consumer,
};

static struct regulator_init_data s5m8767_ldo19_data = {
	.constraints	= {
		.name		= "vt_cam_1.8v",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_ldo19_consumer,
};

static struct regulator_init_data s5m8767_ldo24_data = {
	.constraints	= {
		.name		= "cam_af_2.8v",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_ldo24_consumer,
};

static struct sec_regulator_data exynos_regulators[] = {
	{S5M8767_BUCK1, &s5m8767_buck1_data},
	{S5M8767_BUCK2, &s5m8767_buck2_data},
	{S5M8767_BUCK3, &s5m8767_buck3_data},
	{S5M8767_BUCK4, &s5m8767_buck4_data},
	{S5M8767_LDO4, &s5m8767_ldo4_data},
	{S5M8767_LDO11, &s5m8767_ldo11_data},
	{S5M8767_LDO14, &s5m8767_ldo14_data},
	{S5M8767_LDO17, &s5m8767_ldo17_data},
	{S5M8767_LDO18, &s5m8767_ldo18_data},
	{S5M8767_LDO19, &s5m8767_ldo19_data},
	{S5M8767_LDO24, &s5m8767_ldo24_data},
};

struct sec_opmode_data s5m8767_opmode_data[S5M8767_REG_MAX] = {
	[S5M8767_BUCK1] = {S5M8767_BUCK1, SEC_OPMODE_STANDBY},
	[S5M8767_BUCK2] = {S5M8767_BUCK2, SEC_OPMODE_STANDBY},
	[S5M8767_BUCK3] = {S5M8767_BUCK3, SEC_OPMODE_STANDBY},
	[S5M8767_BUCK4] = {S5M8767_BUCK4, SEC_OPMODE_STANDBY},
	[S5M8767_LDO4] = {S5M8767_LDO4, SEC_OPMODE_STANDBY},
	[S5M8767_LDO11] = {S5M8767_LDO11, SEC_OPMODE_STANDBY},
	[S5M8767_LDO14] = {S5M8767_LDO14, SEC_OPMODE_STANDBY},
	[S5M8767_LDO17] = {S5M8767_LDO17, SEC_OPMODE_STANDBY},
	[S5M8767_LDO18] = {S5M8767_LDO18, SEC_OPMODE_STANDBY},
	[S5M8767_LDO19] = {S5M8767_LDO19, SEC_OPMODE_STANDBY},
	[S5M8767_LDO24] = {S5M8767_LDO24, SEC_OPMODE_STANDBY},
};

static int sec_cfg_irq(void)
{
	unsigned int pin = irq_to_gpio(SMDK5250_PMIC_EINT);

	s3c_gpio_cfgpin(pin, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(pin, S3C_GPIO_PULL_UP);

	return 0;
}

static struct sec_pmic_platform_data smdk5250_s5m8767_pdata = {
	.device_type		= S5M8767X,
	.irq_base		= IRQ_BOARD_START,
	.num_regulators		= ARRAY_SIZE(exynos_regulators),
	.regulators		= exynos_regulators,
	.cfg_pmic_irq		= sec_cfg_irq,
	.wakeup			= 1,
	.wtsr_smpl		= 1,
	.opmode_data		= s5m8767_opmode_data,

	.buck_default_idx	= 1,
	.buck_gpios[0]		= EXYNOS5_GPD1(0),
	.buck_gpios[1]		= EXYNOS5_GPD1(1),
	.buck_gpios[2]		= EXYNOS5_GPD1(2),

	.buck_ramp_delay	= 25,
	.buck2_ramp_enable	= 1,
	.buck3_ramp_enable	= 1,
	.buck4_ramp_enable	= 1,
};

static struct i2c_board_info i2c_devs0[] __initdata = {
	{
		I2C_BOARD_INFO("sec-pmic", 0xCC >> 1),
		.platform_data	= &smdk5250_s5m8767_pdata,
		.irq		= SMDK5250_PMIC_EINT,
	}, {
		I2C_BOARD_INFO("max77686", (0x12 >> 1)),
		.platform_data = &smdk5250_max77686_info,
	},
};

static void smdk5250_power_off(void)
{
	local_irq_disable();

	writel(readl(EXYNOS_PS_HOLD_CONTROL) & ~BIT(8),
		EXYNOS_PS_HOLD_CONTROL);

	exynos5_restart(0, 0);
}

static void smdk5250_reboot(char str, const char *cmd)
{
	local_irq_disable();

	writel(REBOOT_MODE_NO_LPM, EXYNOS_INFORM2); /* Don't enter lpm mode */
	writel(REBOOT_MODE_PREFIX | REBOOT_MODE_NONE, EXYNOS_INFORM3);

	if (cmd) {
		if (!strcmp(cmd, "recovery"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_RECOVERY,
			       EXYNOS_INFORM3);
		else if (!strcmp(cmd, "bootloader"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_FAST_BOOT,
			       EXYNOS_INFORM2);
	}

	exynos5_restart(str, cmd); /* S/W reset: INFORM0~3:  Keep its value */
}

void __init exynos5_smdk5250_power_init(void)
{
	pm_power_off = smdk5250_power_off;
	arm_pm_restart = smdk5250_reboot;

	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));
}
