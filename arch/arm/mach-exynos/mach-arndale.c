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
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/sys_soc.h>
#include <linux/persistent_ram.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/ion.h>

#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/system_info.h>

#include <plat/adc.h>
#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/regs-serial.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/iic.h>

#include <mach/exynos_fiq_debugger.h>
#include <mach/map.h>
#include <mach/exynos-ion.h>
#include <mach/tmu.h>

#include "../../../drivers/staging/android/ram_console.h"
#include "common.h"
#include "board-arndale.h"
#include "resetreason.h"

#define SMDK5250_CPU0_DEBUG_PA		0x10890000
#define SMDK5250_CPU1_DEBUG_PA		0x10892000
#define SMDK5250_CPU_DBGPCSR		0xa0

static void __iomem *smdk5250_cpu0_debug;
static void __iomem *smdk5250_cpu1_debug;

static char smdk5250_board_info_string[255];

static void smdk5250_init_hw_rev(void)
{
	snprintf(smdk5250_board_info_string,
		 sizeof(smdk5250_board_info_string) - 1,
		 "SMDK HW revision: %d, CPU EXYNOS5250 Rev%d.%d",
		 get_smdk5250_rev(),
		 samsung_rev() >> 4,
		 samsung_rev() & 0xf);
	pr_info("%s\n", smdk5250_board_info_string);
	mach_panic_string = smdk5250_board_info_string;
}

static struct ram_console_platform_data ramconsole_pdata;

static struct platform_device ramconsole_device = {
	.name           = "ram_console",
	.id             = -1,
	.dev		= {
		.platform_data = &ramconsole_pdata,
	},
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

/* ADC */
static struct s3c_adc_platdata smdk5250_adc_data __initdata = {
	.phy_init       = s3c_adc_phy_init,
	.phy_exit       = s3c_adc_phy_exit,
};

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
#ifdef CONFIG_ION_EXYNOS
	&exynos_device_ion,
#endif
#ifdef CONFIG_BATTERY_SAMSUNG
	&samsung_device_battery,
#endif
#ifdef CONFIG_EXYNOS_DEV_TMU
	&exynos_device_tmu,
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
		.stop_tc		= 13,
		.start_tc		= 10,
		.stop_mif_vc		= 27,
		.start_mif_vc		= 25,
		.stop_throttle		= 78,
		.start_throttle		= 80,
		.start_tripping		= 110,
		.start_emergency	= 120,
		.stop_mem_throttle	= 80,
		.start_mem_throttle	= 85,
	},

	.temp_compensate = {
		.arm_volt	= 900000, /* uV */
		.bus_mif_volt	= 900000, /* uV */
		.bus_int_volt	= 900000, /* uV */
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
	ion_reserve(&exynos_ion_pdata);
}
#else /* !CONFIG_CMA*/
static inline void exynos_reserve_mem(void)
{
}
#endif

static void __init smdk5250_map_io(void)
{
	clk_xusbxti.rate = 24000000;
	clk_xxti.rate = 24000000;
	exynos_init_io(NULL, 0);
	s3c24xx_init_clocks(clk_xusbxti.rate);
	s3c24xx_init_uarts(smdk5250_uartcfgs, ARRAY_SIZE(smdk5250_uartcfgs));
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

static void __init soc_info_populate(struct soc_device_attribute *soc_dev_attr)
{
	soc_dev_attr->soc_id = kasprintf(GFP_KERNEL, "%08x%08x\n",
					 system_serial_high, system_serial_low);
	soc_dev_attr->machine = kasprintf(GFP_KERNEL, "Exynos 5250\n");
	soc_dev_attr->family = kasprintf(GFP_KERNEL, "Exynos 5\n");
	soc_dev_attr->revision = kasprintf(GFP_KERNEL, "%d.%d\n",
					   samsung_rev() >> 4,
					   samsung_rev() & 0xf);
}

static ssize_t smdk5250_get_board_revision(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	return sprintf(buf, "%d\n", get_smdk5250_rev());
}

struct device_attribute smdk5250_soc_attr =
	__ATTR(board_rev,  S_IRUGO, smdk5250_get_board_revision,  NULL);

static void __init exynos5_smdk5250_sysfs_soc_init(void)
{
	struct device *parent;
	struct soc_device *soc_dev;
	struct soc_device_attribute *soc_dev_attr;

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr) {
		printk(KERN_ERR "Failed to allocate memory for soc_dev_attr\n");
		return;
	}

	soc_info_populate(soc_dev_attr);

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR_OR_NULL(soc_dev)) {
		kfree(soc_dev_attr);
		printk(KERN_ERR "Failed to register a soc device under /sys\n");
		return;
	}

	parent = soc_device_to_device(soc_dev);
	if (!IS_ERR_OR_NULL(parent))
		device_create_file(parent, &smdk5250_soc_attr);

	return;  /* Or return parent should you need to use one later */
}

void smdk5250_panic_dump_cpu_pc(int cpu, unsigned long dbgpcsr)
{
       void *pc = NULL;

       pr_err("CPU%d DBGPCSR: %08lx\n", cpu, dbgpcsr);
       if ((dbgpcsr & 3) == 0)
               pc = (void *)(dbgpcsr - 8);
       else if ((dbgpcsr & 1) == 1)
               pc = (void *)((dbgpcsr & ~1) - 4);

       pr_err("CPU%d PC: <%p> %pF\n", cpu, pc, pc);
}

int smdk5250_panic_notify(struct notifier_block *nb, unsigned long event, void *p)
{
       unsigned long dbgpcsr;

       if (smdk5250_cpu0_debug && cpu_online(0)) {
               dbgpcsr = __raw_readl(smdk5250_cpu0_debug + SMDK5250_CPU_DBGPCSR);
               smdk5250_panic_dump_cpu_pc(0, dbgpcsr);
       }
       if (smdk5250_cpu1_debug && cpu_online(1)) {
               dbgpcsr = __raw_readl(smdk5250_cpu1_debug + SMDK5250_CPU_DBGPCSR);
               smdk5250_panic_dump_cpu_pc(1, dbgpcsr);
       }
       return NOTIFY_OK;
}

struct notifier_block smdk5250_panic_nb = {
       .notifier_call = smdk5250_panic_notify,
};

static void __init smdk5250_panic_init(void)
{
       smdk5250_cpu0_debug = ioremap(SMDK5250_CPU0_DEBUG_PA, SZ_4K);
       smdk5250_cpu1_debug = ioremap(SMDK5250_CPU1_DEBUG_PA, SZ_4K);

       atomic_notifier_chain_register(&panic_notifier_list, &smdk5250_panic_nb);
}

static void __init smdk5250_machine_init(void)
{
	smdk5250_init_hw_rev();

#ifdef CONFIG_EXYNOS_FIQ_DEBUGGER
	exynos_serial_debug_init(2, 0);
#endif
	smdk5250_panic_init();

	s3c_i2c0_set_platdata(NULL);
	s3c_i2c4_set_platdata(NULL);
	s3c_i2c5_set_platdata(NULL);

	s3c_adc_set_platdata(&smdk5250_adc_data);

#ifdef CONFIG_EXYNOS_DEV_TMU
	exynos_tmu_set_platdata(&smdk5250_tmu_pdata);
#endif
	exynos5_smdk5250_mmc_init();

	ramconsole_pdata.bootinfo = exynos_get_resetreason();
	platform_add_devices(smdk5250_devices, ARRAY_SIZE(smdk5250_devices));

	exynos5_smdk5250_audio_init();
	exynos5_smdk5250_input_init();
	exynos5_smdk5250_usb_init();
	exynos5_smdk5250_power_init();
	exynos5_smdk5250_spi_init();
	exynos5_smdk5250_display_init();
	exynos5_smdk5250_media_init();
	exynos5_smdk5250_tvout_init();
	exynos5_smdk5250_camera_init();

	exynos5_smdk5250_sysfs_soc_init();
	exynos5_arndale_wifi_init();
	exynos5_arndale_sensors_init();
#ifdef CONFIG_GPS_POWER
	exynos5_arndale_gps_init();
#endif
}

MACHINE_START(ARNDALE, "ARNDALE")
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
