/*
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clockchips.h>
#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/moduleparam.h>
#include <linux/ratelimit.h>
#include <linux/time.h>
#include <linux/serial_core.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>

#include <asm/cp15.h>
#include <asm/cacheflush.h>
#include <asm/proc-fns.h>
#include <asm/smp_scu.h>
#include <asm/suspend.h>
#include <asm/unified.h>
#include <asm/hardware/gic.h>
#include <asm/system_misc.h>
#include <mach/regs-pmu.h>
#include <mach/regs-clock.h>
#include <mach/pmu.h>
#include <mach/smc.h>
#include <mach/cpuidle.h>

#include <plat/pm.h>
#include <plat/cpu.h>
#include <plat/regs-serial.h>
#include <plat/regs-watchdog.h>
#include <plat/gpio-cfg.h>
#include <plat/gpio-core.h>
#include <plat/devs.h>

#include <trace/events/power.h>

#ifdef CONFIG_ARM_TRUSTZONE
#define REG_DIRECTGO_ADDR	(S5P_VA_SYSRAM_NS + 0x24)
#define REG_DIRECTGO_FLAG	(S5P_VA_SYSRAM_NS + 0x20)
#define BOOT_VECTOR		(S5P_VA_SYSRAM_NS + 0x1C)
#else
#define REG_DIRECTGO_ADDR	(S5P_VA_SYSRAM + 0x24)
#define REG_DIRECTGO_FLAG	(S5P_VA_SYSRAM + 0x20)
#define BOOT_VECTOR		S5P_VA_SYSRAM
#endif

static bool allow_coupled_idle = true;
static bool allow_lpa = true;
module_param(allow_coupled_idle, bool, 0644);
module_param(allow_lpa, bool, 0644);

#define EXYNOS_CHECK_DIRECTGO	0xFCBA0D10

static atomic_t exynos_idle_barrier;
static volatile bool cpu1_abort;
#ifdef CONFIG_ARM_TRUSTZONE
static unsigned int misc_save;
#endif

static int exynos_enter_idle(struct cpuidle_device *dev,
			     struct cpuidle_driver *drv,
			     int index);
static int exynos_enter_lowpower(struct cpuidle_device *dev,
				 struct cpuidle_driver *drv,
				 int index);

static inline void cpu_enter_lowpower_a9(void)
{
	unsigned int v;

	flush_cache_all();
	asm volatile(
	"	mcr	p15, 0, %1, c7, c5, 0\n"
	"	mcr	p15, 0, %1, c7, c10, 4\n"
	/*
	 * Turn off coherency
	 */
	"	mrc	p15, 0, %0, c1, c0, 1\n"
	"	bic	%0, %0, %3\n"
	"	mcr	p15, 0, %0, c1, c0, 1\n"
	"	mrc	p15, 0, %0, c1, c0, 0\n"
	"	bic	%0, %0, %2\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	  : "=&r" (v)
	  : "r" (0), "Ir" (CR_C), "Ir" (0x40)
	  : "cc");
}

static inline void cpu_enter_lowpower_a15(void)
{
	unsigned int v;

	asm volatile(
	"       mrc     p15, 0, %0, c1, c0, 0\n"
	"       bic     %0, %0, %1\n"
	"       mcr     p15, 0, %0, c1, c0, 0\n"
	  : "=&r" (v)
	  : "Ir" (CR_C)
	  : "cc");

	flush_cache_all();

#ifdef CONFIG_ARM_TRUSTZONE
	asm volatile("mrc p15, 0, %0, c1, c0, 1" : "=r" (misc_save));
#endif

	asm volatile(
	/*
	* Turn off coherency
	*/
	"	dmb\n"
	"       mrc     p15, 0, %0, c1, c0, 1\n"
	"       bic     %0, %0, %1\n"
	"       mcr     p15, 0, %0, c1, c0, 1\n"
	: "=&r" (v)
	: "Ir" (0x40)
	: "cc");

	isb();
	dsb();
}

static inline void cpu_leave_lowpower(void)
{
	unsigned int v;

	asm volatile(
	"mrc	p15, 0, %0, c1, c0, 0\n"
	"	orr	%0, %0, %1\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	"	mrc	p15, 0, %0, c1, c0, 1\n"
	"	orr	%0, %0, %2\n"
	"	mcr	p15, 0, %0, c1, c0, 1\n"
	  : "=&r" (v)
	  : "Ir" (CR_C), "Ir" (0x40)
	  : "cc");

#ifdef CONFIG_ARM_TRUSTZONE
	if (v != misc_save)
		exynos_smc(SMC_CMD_REG, SMC_REG_ID_CP15(1, 0, 0, 1), misc_save, 0);
#endif
}

struct check_reg_lpa {
	void __iomem	*check_reg;
	unsigned int	check_bit;
};

struct check_device_op {
	void __iomem		*base;
	struct platform_device	*pdev;
};

/*
 * List of check power domain list for LPA mode
 * These register are have to power off to enter LPA mode
 */
static struct check_reg_lpa exynos5_power_domain[] = {
	{.check_reg = EXYNOS5_DISP1_STATUS,	.check_bit = 0x7},
	{.check_reg = EXYNOS5_G3D_STATUS,	.check_bit = 0x7},
	{.check_reg = EXYNOS5_ISP_STATUS,	.check_bit = 0x7},
	{.check_reg = EXYNOS5_GSCL_STATUS,	.check_bit = 0x7},
};

/*
 * List of check clock gating list for LPA mode
 * If clock of list is not gated, system can not enter LPA mode.
 */
static struct check_reg_lpa exynos5_clock_gating[] = {
	{.check_reg = EXYNOS5_CLKSRC_MASK_DISP1_0,	.check_bit = 0x00000001},
	{.check_reg = EXYNOS5_CLKGATE_IP_DISP1,		.check_bit = 0x00000010},
	{.check_reg = EXYNOS5_CLKGATE_IP_MFC,		.check_bit = 0x00000001},
	{.check_reg = EXYNOS5_CLKGATE_IP_GEN,		.check_bit = 0x00000016},
	{.check_reg = EXYNOS5_CLKGATE_IP_FSYS,		.check_bit = 0x00000006},
	{.check_reg = EXYNOS5_CLKGATE_IP_PERIC,		.check_bit = 0x00377fc0},
	{.check_reg = EXYNOS5_CLKGATE_IP_ACP,		.check_bit = 0x00000002},
};

static struct check_device_op chk_sdhc_op_exynos5250[] = {
#if defined(CONFIG_EXYNOS_DEV_DWMCI)
	{.base = 0, .pdev = &exynos5_device_dwmci0},
	{.base = 0, .pdev = &exynos5_device_dwmci1},
	{.base = 0, .pdev = &exynos5_device_dwmci2},
	{.base = 0, .pdev = &exynos5_device_dwmci3},
#endif
};

/* Global list of devices for check the enter LPA */
static struct list_head dev_list = LIST_HEAD_INIT(dev_list);

int exynos_add_lpa_device(enum lpa_check_device_t cdev)
{
	struct cpuidle_lpa_device *lpa_device;
	struct cpuidle_lpa_device *lpa_cdev, *next;
	int ret = 0;

	if (cdev < 0 || cdev >= LPA_CDEV_END) {
		ret = -EINVAL;
		goto cdev_err;
	}

	list_for_each_entry_safe(lpa_cdev, next, &dev_list, node) {
		if (lpa_cdev->cdev == cdev) {
			pr_err("already registered cpuidle_lpa_device(%d)\n",
				cdev);
			ret = -EINVAL;
			goto cdev_err;
		}
	}

	lpa_device = kzalloc(sizeof(struct cpuidle_lpa_device), GFP_KERNEL);

	if (lpa_device) {
		lpa_device->cdev = cdev;
		lpa_device->usage = false;
		list_add(&lpa_device->node, &dev_list);
	} else {
		pr_err("Unable to create new cpuidle_lpa_device\n");
		ret = -ENOMEM;
		goto cdev_err;
	}

cdev_err:
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_add_lpa_device);

int exynos_remove_lpa_device(enum lpa_check_device_t cdev)
{
	struct cpuidle_lpa_device *lpa_device, *next;
	int ret = 0;

	if (cdev < 0 || cdev >= LPA_CDEV_END) {
		ret = -EINVAL;
		goto del_err;
	}

	list_for_each_entry_safe(lpa_device, next, &dev_list, node) {
		if (lpa_device->cdev == cdev) {
			if (lpa_device->usage) {
				pr_err("lpa_device(%d) is still in use\n",
					cdev);
				ret = -EINVAL;
				goto del_err;
			}
			list_del(&lpa_device->node);
			kfree(lpa_device);
			break;
		}
	}

del_err:
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_remove_lpa_device);

int exynos_set_lpa_device_usage(enum lpa_check_device_t cdev, int usage)
{
	struct cpuidle_lpa_device *lpa_device;
	int ret = 0;

	if (cdev < 0 || cdev >= LPA_CDEV_END) {
		ret = -EINVAL;
		goto set_err;
	}

	list_for_each_entry(lpa_device, &dev_list, node) {
		if (lpa_device->cdev == cdev) {
			lpa_device->usage = usage;
			pr_debug("set lpa_cdev(%d) usage as %s\n",
				lpa_device->cdev, usage ? "use" : "not use");
			break;
		}
	}

set_err:
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_set_lpa_device_usage);

static int exynos_check_lpa_devices(void)
{
	struct cpuidle_lpa_device *lpa_device;

	list_for_each_entry(lpa_device, &dev_list, node) {
		pr_debug("lpa device(%d) usage is %d\n",
			lpa_device->cdev, lpa_device->usage);

		if (lpa_device->usage)
			return -EBUSY;
	}

	return 0;
}

#define MSHCI_CLKENA		(0x10)	/* Clock enable */
#define MSHCI_STATUS		(0x48)	/* Status */
#define MSHCI_DATA_BUSY		(0x1<<9)
#define MSHCI_DATA_STAT_BUSY	(0x1<<10)
#define MSHCI_ENCLK		(0x1)

static int sdmmc_dev_num;

/* If SD/MMC interface is working: return = 1 or not 0 */
static int check_sdmmc_op(unsigned int ch)
{
	unsigned int reg1;
	void __iomem *base_addr;

	if (unlikely(ch >= sdmmc_dev_num)) {
		pr_err("Invalid ch[%d] for SD/MMC\n", ch);
		return 0;
	}

	base_addr = chk_sdhc_op_exynos5250[ch].base;

	/* Check STATUS [9] for data busy */
	reg1 = readl(base_addr + MSHCI_STATUS);

	return (reg1 & (MSHCI_DATA_BUSY)) ||
			(reg1 & (MSHCI_DATA_STAT_BUSY));
}

/* Check all sdmmc controller */
static int loop_sdmmc_check(void)
{
	unsigned int iter;

	for (iter = 0; iter < sdmmc_dev_num; iter++) {
		if (check_sdmmc_op(iter)) {
			pr_debug("SDMMC [%d] working\n", iter);
			return -EBUSY;
		}
	}
	return 0;
}

/*
 * To keep value of gpio on power down mode
 * set Power down register of gpio
 */
static void exynos5_gpio_set_pd_reg(void)
{
	struct samsung_gpio_chip *target_chip;
	unsigned int gpio_nr;
	unsigned int tmp;

	for (gpio_nr = 0; gpio_nr < EXYNOS5_GPIO_END; gpio_nr++) {
		target_chip = samsung_gpiolib_getchip(gpio_nr);

		if (!target_chip)
			continue;

		if (!target_chip->pm)
			continue;

		/* Keep the previous state in LPA mode */
		s5p_gpio_set_pd_cfg(gpio_nr, S5P_GPIO_PD_PREV_STATE);

		/* Pull up-down state in LPA mode is same as normal */
		tmp = s3c_gpio_getpull(gpio_nr);
		s5p_gpio_set_pd_pull(gpio_nr, tmp);
	}
}

static int exynos_check_reg_status(struct check_reg_lpa *reg_list,
				    unsigned int list_cnt)
{
	unsigned int i;
	unsigned int tmp;

	for (i = 0; i < list_cnt; i++) {
		tmp = __raw_readl(reg_list[i].check_reg);
		if (tmp & reg_list[i].check_bit)
			return -EBUSY;
	}

	return 0;
}

static int exynos_uart_fifo_check(void)
{
	int ret;
	unsigned int check_val;

	/* Check UART for console is empty */
	check_val = __raw_readl(S5P_VA_UART(CONFIG_S3C_LOWLEVEL_UART_PORT) +
				S3C2410_UFSTAT);

	ret = ((check_val >> 16) & 0xff);

	return ret;
}

static int __maybe_unused exynos_check_enter_mode(void)
{
	/* Check power domain */
	if (exynos_check_reg_status(exynos5_power_domain,
				    ARRAY_SIZE(exynos5_power_domain)))
		return EXYNOS_CHECK_DIDLE;

	/* Check clock gating */
	if (exynos_check_reg_status(exynos5_clock_gating,
				    ARRAY_SIZE(exynos5_clock_gating)))
		return EXYNOS_CHECK_DIDLE;

	if (loop_sdmmc_check())
		return EXYNOS_CHECK_DIDLE;

	/* check device usage */
	if (exynos_check_lpa_devices())
		return EXYNOS_CHECK_DIDLE;

	return EXYNOS_CHECK_LPA;
}

static struct cpuidle_state exynos_cpuidle_set[] __initdata = {
	[0] = {
		.enter			= exynos_enter_idle,
		.exit_latency		= 10,
		.target_residency	= 10,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "C0",
		.desc			= "ARM clock gating(WFI)",
	},
	[1] = {
		.enter			= exynos_enter_lowpower,
		.exit_latency		= 5000,
		.target_residency	= 10000,
		.flags			= CPUIDLE_FLAG_TIME_VALID |
					  CPUIDLE_FLAG_COUPLED,
		.name			= "C1",
		.desc			= "ARM power down",
	},
};

static DEFINE_PER_CPU(struct cpuidle_device, exynos_cpuidle_device);

static struct cpuidle_driver exynos_idle_driver = {
	.name		= "exynos_idle",
	.owner		= THIS_MODULE,
	.en_core_tk_irqen = true,
};

/* Ext-GIC nIRQ/nFIQ is the only wakeup source in AFTR */
static void exynos_set_wakeupmask(void)
{
	__raw_writel(0x0000ff3e, EXYNOS_WAKEUP_MASK);
}

static void save_cpu_arch_register(void)
{
}

static void restore_cpu_arch_register(void)
{
}

static int cpu1_idle_finisher(unsigned long flags)
{
	cpu_do_idle();
	return 1;
}

static int idle_finisher(unsigned long flags)
{
#if defined(CONFIG_ARM_TRUSTZONE)
	exynos_smc(SMC_CMD_CPU0AFTR, 0, 0, 0);
#else
	cpu_do_idle();
#endif
	return 1;
}

static struct sleep_save exynos5_lpa_save[] = {
	/* CMU side */
	SAVE_ITEM(EXYNOS5_CLKSRC_MASK_TOP),
	SAVE_ITEM(EXYNOS5_CLKSRC_MASK_GSCL),
	SAVE_ITEM(EXYNOS5_CLKSRC_MASK_DISP1_0),
	SAVE_ITEM(EXYNOS5_CLKSRC_MASK_MAUDIO),
	SAVE_ITEM(EXYNOS5_CLKSRC_MASK_FSYS),
	SAVE_ITEM(EXYNOS5_CLKSRC_MASK_PERIC0),
	SAVE_ITEM(EXYNOS5_CLKSRC_MASK_PERIC1),
	SAVE_ITEM(EXYNOS5_CLKSRC_TOP3),
};

static struct sleep_save exynos5_set_clksrc[] = {
	{ .reg = EXYNOS5_CLKSRC_MASK_TOP		, .val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_GSCL		, .val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_DISP1_0		, .val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_MAUDIO		, .val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_FSYS		, .val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_PERIC0		, .val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_PERIC1		, .val = 0xffffffff, },
};

static int exynos_enter_core0(enum sys_powerdown powerdown)
{
	unsigned long tmp;

	__raw_writel(virt_to_phys(s3c_cpu_resume), REG_DIRECTGO_ADDR);
	__raw_writel(EXYNOS_CHECK_DIRECTGO, REG_DIRECTGO_FLAG);

	if (powerdown == SYS_LPA)
		__raw_writel(EXYNOS_CHECK_LPA, EXYNOS_INFORM1);

	exynos_sys_powerdown_conf(powerdown);

	save_cpu_arch_register();

	/* Setting Central Sequence Register for power down mode */
	tmp = __raw_readl(EXYNOS_CENTRAL_SEQ_CONFIGURATION);
	tmp &= ~EXYNOS_CENTRAL_LOWPWR_CFG;
	__raw_writel(tmp, EXYNOS_CENTRAL_SEQ_CONFIGURATION);

	if (cpu_pm_enter())
		goto abort_cpu;

	cpu_suspend(0, idle_finisher);

#ifdef CONFIG_SMP
#if !defined(CONFIG_ARM_TRUSTZONE)
	if (!soc_is_exynos5250())
		scu_enable(S5P_VA_SCU);
#endif
#endif

	cpu_pm_exit();
abort_cpu:

	restore_cpu_arch_register();

	/*
	 * If PMU failed while entering sleep mode, WFI will be
	 * ignored by PMU and then exiting cpu_do_idle().
	 * S5P_CENTRAL_LOWPWR_CFG bit will not be set automatically
	 * in this situation.
	 */
	tmp = __raw_readl(EXYNOS_CENTRAL_SEQ_CONFIGURATION);
	if (!(tmp & EXYNOS_CENTRAL_LOWPWR_CFG)) {
		tmp |= EXYNOS_CENTRAL_LOWPWR_CFG;
		__raw_writel(tmp, EXYNOS_CENTRAL_SEQ_CONFIGURATION);
		return -EBUSY;
	}

	return 0;
}

static int exynos_enter_core0_aftr(struct cpuidle_device *dev,
				   struct cpuidle_driver *drv,
				   int index)
{
	exynos_set_wakeupmask();

	exynos_enter_core0(SYS_AFTR);

	/* Clear wakeup state register */
	__raw_writel(0x0, EXYNOS_WAKEUP_STAT);

	return index;
}

static int exynos_enter_core0_lpa(struct cpuidle_device *dev,
				  struct cpuidle_driver *drv,
				  int index)
{
	/* Configure GPIO Power down control register */
	s3c_pm_do_save(exynos5_lpa_save, ARRAY_SIZE(exynos5_lpa_save));

	/*
	 * Before enter central sequence mode, clock src register have to set
	 */
	s3c_pm_do_restore_core(exynos5_set_clksrc,
				ARRAY_SIZE(exynos5_set_clksrc));

	/*
	 * Unmasking all wakeup source.
	 */
	__raw_writel(0x0, EXYNOS_WAKEUP_MASK);

	do {
		/* Waiting for flushing UART fifo */
	} while (exynos_uart_fifo_check());

	exynos5_gpio_set_pd_reg();

	if (exynos_enter_core0(SYS_LPA))
		goto early_wakeup;

	/* For release retention */
	__raw_writel((1 << 28), EXYNOS_PAD_RET_MAUDIO_OPTION);
	__raw_writel((1 << 28), EXYNOS_PAD_RET_GPIO_OPTION);
	__raw_writel((1 << 28), EXYNOS_PAD_RET_UART_OPTION);
	__raw_writel((1 << 28), EXYNOS_PAD_RET_MMCA_OPTION);
	__raw_writel((1 << 28), EXYNOS_PAD_RET_MMCB_OPTION);
	__raw_writel((1 << 28), EXYNOS_PAD_RET_EBIA_OPTION);
	__raw_writel((1 << 28), EXYNOS_PAD_RET_EBIB_OPTION);
	__raw_writel((1 << 28), EXYNOS5_PAD_RETENTION_SPI_OPTION);
	__raw_writel((1 << 28), EXYNOS5_PAD_RETENTION_GPIO_SYSMEM_OPTION);

early_wakeup:
	s3c_pm_do_restore_core(exynos5_lpa_save, ARRAY_SIZE(exynos5_lpa_save));

	/* Clear wakeup state register */
	__raw_writel(0x0, EXYNOS_WAKEUP_STAT);

	__raw_writel(0x0, EXYNOS_WAKEUP_MASK);

	return index;
}

static int exynos_enter_idle(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int index)
{
	struct timeval before, after;
	int idle_time;

	local_irq_disable();
	do_gettimeofday(&before);

	cpu_do_idle();

	do_gettimeofday(&after);
	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
		    (after.tv_usec - before.tv_usec);

	dev->last_residency = idle_time;
	return index;
}

void exynos_nop(void *info)
{
}

static int exynos_enter_lowpower(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int index)
{
	int ret = index;

	__raw_writel(virt_to_phys(s3c_cpu_resume), BOOT_VECTOR);
	cpuidle_coupled_parallel_barrier(dev, &exynos_idle_barrier);

	if (!allow_coupled_idle) {
		exynos_enter_idle(dev, drv, dev->safe_state_index);
		smp_call_function_single(!dev->cpu, exynos_nop, NULL, 0);
		return index;
	}

	/* Both cpus will reach this point at the same time */
	if (dev->cpu == 0) {
		/* Idle sequence for cpu 0 */
		if (cpu_online(1)) {
			/* Wait for cpu1 to turn itself off */
			while (__raw_readl(EXYNOS_ARM_CORE1_STATUS) & 3) {
				/* cpu1 may skip idle and boot back up again */
				if (cpu1_abort)
					goto abort;

				/*
				 * cpu1 may bounce through idle and boot back up
				 * again, getting stuck in the boot rom code
				 */
				if (__raw_readl(BOOT_VECTOR) == 0)
					goto abort;

				cpu_relax();
			}
		}

		watchdog_save();
		/* Enter the final low power state */
		if (exynos_check_enter_mode() == EXYNOS_CHECK_DIDLE || !allow_lpa)
			ret = exynos_enter_core0_aftr(dev, drv, index);
		else
			ret = exynos_enter_core0_lpa(dev, drv, index);

abort:
		if (cpu_online(1)) {
			/* Set the boot vector to something non-zero */
			__raw_writel(virt_to_phys(s3c_cpu_resume),
				BOOT_VECTOR);
			dsb();

			/* Turn on cpu1 and wait for it to be on */
			__raw_writel(0x3, EXYNOS_ARM_CORE1_CONFIGURATION);
			while ((__raw_readl(EXYNOS_ARM_CORE1_STATUS) & 3) != 3)
				cpu_relax();

#ifdef CONFIG_ARM_TRUSTZONE
			exynos_smc(SMC_CMD_CPU1BOOT, 0, 0, 0);
#endif

			/* Wait for cpu1 to get stuck in the boot rom */
			while ((__raw_readl(BOOT_VECTOR) != 0) && !cpu1_abort) {
#ifdef CONFIG_ARM_TRUSTZONE
				exynos_smc(SMC_CMD_CPU1BOOT, 0, 0, 0);
#endif
				cpu_relax();
			}

			if (!cpu1_abort) {
				/* Poke cpu1 out of the boot rom */
				__raw_writel(virt_to_phys(s3c_cpu_resume),
					BOOT_VECTOR);
				dsb_sev();
			}

			/* Wait for cpu1 to finish booting */
			while (!cpu1_abort)
				cpu_relax();
		}
		watchdog_restore();
	} else {
		/* Idle sequence for cpu 1 */

		/*
		 * Turn off localtimer to prevent ticks from waking up cpu 1
		 * before cpu 0.
		 */
		clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &dev->cpu);

		if (cpu_pm_enter())
			goto cpu1_aborted;

		cpu_enter_lowpower_a15();

		/* Turn off cpu 1 */
		__raw_writel(0, EXYNOS_ARM_CORE1_CONFIGURATION);
		cpu_suspend(0, cpu1_idle_finisher);

		cpu_leave_lowpower();

		cpu_pm_exit();
		clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &dev->cpu);

cpu1_aborted:
		/* Notify cpu 0 that cpu 1 is awake */
		dsb();
		cpu1_abort = true;
	}

	cpuidle_coupled_parallel_barrier(dev, &exynos_idle_barrier);

	cpu1_abort = false;

	return ret;
}

static int exynos_cpuidle_pm_notifier(struct notifier_block *notifier,
					unsigned long pm_event, void *v)
{
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		disable_hlt();
		break;
	case PM_POST_SUSPEND:
		enable_hlt();
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos_cpuidle_nb = {
	.notifier_call = exynos_cpuidle_pm_notifier,
};

static void __setup_broadcast_timer(void *arg)
{
	unsigned long reason = (unsigned long)arg;
	int cpu = smp_processor_id();

	reason = reason ?
		CLOCK_EVT_NOTIFY_BROADCAST_ON : CLOCK_EVT_NOTIFY_BROADCAST_OFF;

	clockevents_notify(reason, &cpu);
}

static int setup_broadcast_cpuhp_notify(struct notifier_block *n,
		unsigned long action, void *hcpu)
{
	int hotcpu = (unsigned long)hcpu;

	switch (action & 0xf) {
	case CPU_ONLINE:
		smp_call_function_single(hotcpu, __setup_broadcast_timer,
			(void *)true, 1);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block setup_broadcast_notifier = {
	.notifier_call = setup_broadcast_cpuhp_notify,
};

static void __init exynos5_core_down_clk(void)
{
	unsigned int tmp;

	tmp = __raw_readl(EXYNOS5_PWR_CTRL1);

	tmp &= ~(PWR_CTRL1_CORE2_DOWN_MASK | PWR_CTRL1_CORE1_DOWN_MASK);

	/* set arm clock divider value on idle state */
	tmp |= ((0x7 << PWR_CTRL1_CORE2_DOWN_RATIO) |
		(0x7 << PWR_CTRL1_CORE1_DOWN_RATIO));

	tmp |= (PWR_CTRL1_DIV2_DOWN_EN |
		PWR_CTRL1_DIV1_DOWN_EN |
		PWR_CTRL1_USE_CORE1_WFE |
		PWR_CTRL1_USE_CORE0_WFE |
		PWR_CTRL1_USE_CORE1_WFI |
		PWR_CTRL1_USE_CORE0_WFI);

	__raw_writel(tmp, EXYNOS5_PWR_CTRL1);

	tmp = __raw_readl(EXYNOS5_PWR_CTRL2);

	tmp &= ~(PWR_CTRL2_DUR_STANDBY2_MASK | PWR_CTRL2_DUR_STANDBY1_MASK |
		PWR_CTRL2_CORE2_UP_MASK | PWR_CTRL2_CORE1_UP_MASK);

	/* set duration value on middle wakeup step */
	tmp |=  ((0x1 << PWR_CTRL2_DUR_STANDBY2) |
		 (0x1 << PWR_CTRL2_DUR_STANDBY1));

	/* set arm clock divier value on middle wakeup step */
	tmp |= ((0x1 << PWR_CTRL2_CORE2_UP_RATIO) |
		(0x1 << PWR_CTRL2_CORE1_UP_RATIO));

	/* Set PWR_CTRL2 register to use step up for arm clock */
	tmp |= (PWR_CTRL2_DIV2_UP_EN | PWR_CTRL2_DIV1_UP_EN);

	__raw_writel(tmp, EXYNOS5_PWR_CTRL2);
	pr_info("Exynos5 : ARM Clock down on idle mode is enabled\n");
}

static struct dentry *lpa_debugfs;

static int lpa_debug_show(struct seq_file *s, void *unused)
{
	struct cpuidle_lpa_device *lpa_device;
	int i;

	seq_printf(s, "[ LPA devices status ]\n");

	seq_printf(s, "power domain status\n");
	for (i = 0; i < ARRAY_SIZE(exynos5_power_domain); i++)
		seq_printf(s, "\tpower_domain : (%d), status : (0x%08x)\n",
			i, __raw_readl(exynos5_power_domain[i].check_reg) &
			exynos5_power_domain[i].check_bit);

	seq_printf(s, "clock gating status\n");
	for (i = 0; i < ARRAY_SIZE(exynos5_clock_gating); i++)
		seq_printf(s, "\tclock_gating : (%d), status : (0x%08x)\n",
			i, __raw_readl(exynos5_clock_gating[i].check_reg) &
			exynos5_clock_gating[i].check_bit);

	seq_printf(s, "SDMMC status\n");
	for (i = 0; i < sdmmc_dev_num; i++)
		seq_printf(s, "\tSDMMC : (%d), usage : (%d)\n",
			i, check_sdmmc_op(i));

	seq_printf(s, "lpa cdev status\n");
	list_for_each_entry(lpa_device, &dev_list, node)
		seq_printf(s, "\tlpa device : (%d), usage : (%d)\n",
			lpa_device->cdev, lpa_device->usage);

	return 0;
}

static int lpa_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, lpa_debug_show, inode->i_private);
}

const static struct file_operations lpa_cdev_status_fops = {
	.open		= lpa_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init exynos_init_cpuidle(void)
{
	int ret;
	int max_cpuidle_state, cpu_id;
	int nr_map, i;
	struct cpuidle_device *device;
	struct cpuidle_driver *drv = &exynos_idle_driver;
	struct platform_device *pdev;
	struct resource *res;

	exynos5_core_down_clk();

	/* Setup cpuidle driver */
	drv->state_count = ARRAY_SIZE(exynos_cpuidle_set);

	max_cpuidle_state = drv->state_count;
	for (i = 0; i < max_cpuidle_state; i++) {
		memcpy(&drv->states[i], &exynos_cpuidle_set[i],
				sizeof(struct cpuidle_state));
	}
	drv->safe_state_index = 0;
	cpuidle_register_driver(&exynos_idle_driver);

	register_pm_notifier(&exynos_cpuidle_nb);

	on_each_cpu(__setup_broadcast_timer, (void *)true, 1);
	ret = register_cpu_notifier(&setup_broadcast_notifier);
	if (ret)
		pr_err("%s: failed to register cpu notifier\n", __func__);

	for_each_cpu(cpu_id, cpu_online_mask) {
		device = &per_cpu(exynos_cpuidle_device, cpu_id);
		device->cpu = cpu_id;

		device->state_count = ARRAY_SIZE(exynos_cpuidle_set);
		device->coupled_cpus = *cpu_possible_mask;

		if (cpuidle_register_device(device)) {
			printk(KERN_ERR "CPUidle register device failed\n,");
			return -EIO;
		}
	}

	sdmmc_dev_num = ARRAY_SIZE(chk_sdhc_op_exynos5250);

	for (nr_map = 0; nr_map < sdmmc_dev_num; nr_map++) {

		pdev = chk_sdhc_op_exynos5250[nr_map].pdev;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			pr_err("failed to get iomem region\n");
			goto res_err;
		}

		chk_sdhc_op_exynos5250[nr_map].base = ioremap(res->start,
							resource_size(res));
		if (!chk_sdhc_op_exynos5250[nr_map].base) {
			pr_err("failed to map io region\n");
			goto res_err;
		}
	}

	lpa_debugfs =
		debugfs_create_file("lpa_cdev_status",
				S_IRUGO, NULL, NULL, &lpa_cdev_status_fops);
	if (IS_ERR_OR_NULL(lpa_debugfs)) {
		lpa_debugfs = NULL;
		pr_err("%s: debugfs_create_file() failed\n", __func__);
		goto res_err;
	}

	return 0;

res_err:
	for (i = nr_map - 1; i >= 0; i--)
		iounmap(chk_sdhc_op_exynos5250[i].base);

	return -ENOMEM;
}
device_initcall(exynos_init_cpuidle);
