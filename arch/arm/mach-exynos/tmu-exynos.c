/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *      http://www.samsung.com
 *
 * EXYNOS - Thermal Management support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/debugfs.h>

#include <plat/cpu.h>

#include <mach/regs-tmu.h>
#include <mach/cpufreq.h>
#include <mach/tmu.h>
#include <mach/map.h>
#include <mach/regs-mem.h>
#include <mach/smc.h>
#include <mach/exynos5_bus.h>
#include <mach/asv-exynos.h>

static DEFINE_MUTEX(tmu_lock);

static struct workqueue_struct *tmu_monitor_wq;

static unsigned int get_refresh_period(unsigned int freq_ref)
{
	unsigned int rclk, tmp, refresh_nsec;

	rclk = freq_ref / 1000000;

#if defined(CONFIG_ARM_TRUSTZONE)
	exynos_smc_read_sfr(SMC_CMD_REG,
		SMC_REG_ID_SFR_R(EXYNOS5_PA_DREXII + EXYNOS_DMC_TIMINGAREF_OFFSET),
		&tmp, 0);
#else
	tmp = __raw_readl(S5P_VA_DREXII + EXYNOS_DMC_TIMINGAREF_OFFSET);
#endif
	refresh_nsec = ((tmp & 0xff) * 1000) / rclk;

	return refresh_nsec;
}

static void set_refresh_period(unsigned int freq_ref,
				unsigned int refresh_nsec)
{
	unsigned int rclk, auto_refresh;

	rclk = freq_ref / 1000000;
	auto_refresh = ((unsigned int)(rclk * refresh_nsec / 1000));

	/* change auto refresh period in TIMING_AREF register of DMC */
#if defined(CONFIG_ARM_TRUSTZONE)
	exynos_smc(SMC_CMD_REG,
		SMC_REG_ID_SFR_W(EXYNOS5_PA_DREXII + EXYNOS_DMC_TIMINGAREF_OFFSET),
		auto_refresh, 0);
#else
	__raw_writel(auto_refresh, S5P_VA_DREXII +
			EXYNOS_DMC_TIMINGAREF_OFFSET);
#endif
}

static int get_cur_temp(struct tmu_info *info)
{
	int curr_temp;
	int temperature;

	curr_temp = __raw_readl(info->tmu_base + CURRENT_TEMP) & 0xff;

	/* compensate and calculate current temperature */
	temperature = curr_temp - info->te1 + TMU_DC_VALUE;

	return temperature;
}

extern int mali_dvfs_freq_under_lock(int level);
extern void mali_dvfs_freq_under_unlock(void);

static int exynos_tc_freq_lock(struct tmu_info *info, int enable)
{
	int ret = 0;

	if (enable == info->tc_state) {
		pr_info("tmu: already is %s.\n",
			enable ? "locked" : "unlocked");
		return ret;
	}

	if (enable) {
		/* locking the cpu frequency */
		ret = exynos_thermal_throttle_min_freq(info->cpulevel_tc);
		if (ret)
			goto err_lock;

#ifdef CONFIG_ARM_EXYNOS5_BUS_DEVFREQ
		/* locking the mif frequency */
		info->mif_handle = exynos5_bus_mif_min(info->miflevel_tc);
		if (!info->mif_handle) {
			ret = -EINVAL;
			goto err_lock;
		}

		/* locking the int frequency */
		info->int_handle = exynos5_bus_int_min(info->intlevel_tc);
		if (!info->int_handle) {
			ret = -EINVAL;
			goto err_lock;
		}
#endif

		/* locking the g3d frequency */
		mali_dvfs_freq_under_lock(1);

		pr_info("Lock for TC is success..\n");
	} else {
		/* unlocking the cpu frequency */
		ret = exynos_thermal_throttle_min_freq(0);
		if (ret)
			goto err_lock;

#ifdef CONFIG_ARM_EXYNOS5_BUS_DEVFREQ
		/* unlocking the mif frequency */
		if (info->mif_handle) {
			ret = exynos5_bus_mif_put(info->mif_handle);
			if (ret)
				goto err_lock;
		}

		/* unlocking the int frequency */
		if (info->int_handle) {
			ret = exynos5_bus_int_put(info->int_handle);
			if (ret)
				goto err_lock;
		}
#endif

		/* unlocking the g3d frequency */
		mali_dvfs_freq_under_unlock();

		pr_info("Unlock for TC is success..\n");
	}

	return ret;

err_lock:
	pr_err("tmu: exynos_tc_freq %s is failed.\n",
			enable ? "locked" : "unlocked");
	return ret;
}

static void tmu_monitor(struct work_struct *work)
{
	struct delayed_work *delayed_work = to_delayed_work(work);
	struct tmu_info *info =
		container_of(delayed_work, struct tmu_info, polling);
	struct tmu_data *data = info->dev->platform_data;
	int cur_temp;

	cur_temp = get_cur_temp(info);

	dev_dbg(info->dev, "Current: %dc, FLAG=%d\n", cur_temp, info->tmu_state);

	mutex_lock(&tmu_lock);
	switch (info->tmu_state) {
	case TMU_STATUS_TC:
		if ((cur_temp <= data->ts.start_tc) &&
			!(info->tc_state)) {
			if (!info->mif_vol_offset_state) {
				exynos5_busfreq_mif_request_voltage_offset(37500);
				info->mif_vol_offset_state = true;
				pr_debug("tmu: mif voltage compensation (+37.5 mV)\n");
			}

			exynos_tc_freq_lock(info, true);

			info->tc_state = true;
			pr_debug("tmu: temperature compensation\n");
		} else if ((cur_temp >= data->ts.stop_tc) &&
				(info->tc_state)) {
			exynos_tc_freq_lock(info, false);

			info->tc_state = false;
			info->tmu_state = TMU_STATUS_MIF_VC;
			pr_debug("tmu: restore temperature compensation\n");
		}
		break;
	case TMU_STATUS_MIF_VC:
		if (cur_temp <= data->ts.start_tc) {
			info->tmu_state = TMU_STATUS_TC;
		} else if ((cur_temp <= data->ts.start_mif_vc) &&
			!(info->mif_vol_offset_state)) {
			exynos5_busfreq_mif_request_voltage_offset(37500);

			info->mif_vol_offset_state = true;
			pr_debug("tmu: mif voltage compensation (+37.5 mV)\n");
		} else if (cur_temp >= data->ts.stop_mif_vc) {
			info->tmu_state = TMU_STATUS_NORMAL;
		}
		break;
	case TMU_STATUS_NORMAL:
		if (info->mif_vol_offset_state) {
			exynos5_busfreq_mif_request_voltage_offset(0);

			info->mif_vol_offset_state = false;
			pr_debug("tmu: restore mif voltage compensation\n");
		} else {
			exynos_thermal_unthrottle();
		}

		/* clear to prevent from interrupt by peindig bit */
		__raw_writel((CLEAR_RISE_INT | CLEAR_FALL_INT),
					info->tmu_base + INTCLEAR);
		enable_irq(info->irq);
		goto out;
	case TMU_STATUS_THROTTLED:
		if (cur_temp >= data->ts.start_tripping)
			info->tmu_state = TMU_STATUS_TRIPPED;
		else if (cur_temp > data->ts.stop_throttle)
			exynos_thermal_throttle();
		else
			info->tmu_state = TMU_STATUS_NORMAL;
		break;
	case TMU_STATUS_TRIPPED:
		if (cur_temp >= data->ts.start_emergency)
			panic("Emergency thermal shutdown: temp=%d\n",
			      cur_temp);
		if (cur_temp >= data->ts.start_tripping) {
			pr_err("thermal tripped: temp=%d\n", cur_temp);
			/* Throttle twice while tripping */
			exynos_thermal_throttle();
		} else {
			info->tmu_state = TMU_STATUS_THROTTLED;
		}
		/* Throttle when tripped */
		exynos_thermal_throttle();
		break;
	default:
		break;
	}

	/* Memory throttling */
	if (cur_temp >= data->ts.start_mem_throttle &&
		!info->mem_throttled) {
		set_refresh_period(FREQ_IN_PLL, info->auto_refresh_mem_throttle);
		info->mem_throttled = true;
		dev_dbg(info->dev, "set auto refresh period %dns\n",
				info->auto_refresh_mem_throttle);
	} else if (cur_temp <= data->ts.stop_mem_throttle &&
		info->mem_throttled) {
		set_refresh_period(FREQ_IN_PLL, info->auto_refresh_normal);
		info->mem_throttled = false;
		dev_dbg(info->dev, "set auto refresh period %dns\n",
				info->auto_refresh_normal);
	}

	queue_delayed_work_on(0, tmu_monitor_wq,
			      &info->polling, info->sampling_rate);
out:
	mutex_unlock(&tmu_lock);
}

static void pm_tmu_save(struct tmu_info *info)
{
	info->reg_save[0] = __raw_readl(info->tmu_base + TMU_CON);
	info->reg_save[1] = __raw_readl(info->tmu_base + SAMPLING_INTERNAL);
	info->reg_save[2] = __raw_readl(info->tmu_base + CNT_VALUE0);
	info->reg_save[3] = __raw_readl(info->tmu_base + CNT_VALUE1);
	info->reg_save[4] = __raw_readl(info->tmu_base + INTEN);
	info->reg_save[5] = __raw_readl(info->tmu_base + THD_TEMP_RISE);
	info->reg_save[6] = __raw_readl(info->tmu_base + THD_TEMP_FALL);
}

static void pm_tmu_restore(struct tmu_info *info)
{
	__raw_writel(info->reg_save[6], info->tmu_base + THD_TEMP_FALL);
	__raw_writel(info->reg_save[5], info->tmu_base + THD_TEMP_RISE);
	__raw_writel(info->reg_save[4], info->tmu_base + INTEN);
	__raw_writel(info->reg_save[3], info->tmu_base + CNT_VALUE1);
	__raw_writel(info->reg_save[2], info->tmu_base + CNT_VALUE0);
	__raw_writel(info->reg_save[1], info->tmu_base + SAMPLING_INTERNAL);
	__raw_writel(info->reg_save[0], info->tmu_base + TMU_CON);
}

static int exynos_tmu_init(struct tmu_info *info)
{
	struct tmu_data *data = info->dev->platform_data;
	unsigned int te_temp, con;
	unsigned int temp_throttle, temp_trip;
	unsigned int temp_mif_vc, temp_tc;
	unsigned int rising_thr, falling_thr;

	/* must reload for using efuse value at EXYNOS4212 */
	__raw_writel(TRIMINFO_RELOAD, info->tmu_base + TRIMINFO_CON);

	/* get the compensation parameter */
	te_temp = __raw_readl(info->tmu_base + TRIMINFO);
	info->te1 = te_temp & TRIM_INFO_MASK;
	info->te2 = ((te_temp >> 8) & TRIM_INFO_MASK);

	if ((EFUSE_MIN_VALUE > info->te1)
	    || (info->te1 > EFUSE_MAX_VALUE)
	    || (info->te2 != 0))
		info->te1 = data->efuse_value;

	/* Map auto refresh period of normal & memory throttle mode */
	info->auto_refresh_normal = get_refresh_period(FREQ_IN_PLL);
	info->auto_refresh_mem_throttle = info->auto_refresh_normal / 2;

	dev_info(info->dev, "Current auto refresh interval(%d nsec),"
			" Normal auto refresh interval(%d nsec),"
			" memory throttle auto refresh internal(%d nsec)\n",
			get_refresh_period(FREQ_IN_PLL),
			info->auto_refresh_normal, info->auto_refresh_mem_throttle);

	info->cpulevel_tc = asv_get_freq(ID_ARM, data->temp_compensate.arm_volt);
	info->miflevel_tc = asv_get_freq(ID_MIF, data->temp_compensate.bus_mif_volt);
	info->intlevel_tc = asv_get_freq(ID_INT, data->temp_compensate.bus_int_volt);

	dev_info(info->dev, "TC ARM frequency = %d, TC MIF frequency = %d,"
			"TC INT frequency = %d\n",
			info->cpulevel_tc, info->miflevel_tc, info->intlevel_tc);

	/*Get rising Threshold and Set interrupt level*/
	temp_throttle = data->ts.start_throttle
			+ info->te1 - TMU_DC_VALUE;
	temp_trip = data->ts.start_tripping
			+ info->te1 - TMU_DC_VALUE;

	rising_thr = (temp_throttle | (temp_trip << 8) |
		     (UNUSED_THRESHOLD << 16) | (UNUSED_THRESHOLD << 24));

	__raw_writel(rising_thr, info->tmu_base + THD_TEMP_RISE);

	/* Get falling Threshold and Set interrupt level */
	temp_mif_vc = data->ts.start_mif_vc
			+ info->te1 - TMU_DC_VALUE;
	temp_tc = data->ts.start_tc
			+ info->te1 - TMU_DC_VALUE;

	falling_thr = (temp_tc | (temp_mif_vc << 8) |
			(UNUSED_THRESHOLD << 16));

	__raw_writel(falling_thr, info->tmu_base + THD_TEMP_FALL);

	/* Set TMU status */
	info->tmu_state = TMU_STATUS_INIT;

	/* To poll current temp, set sampling rate */
	info->sampling_rate = msecs_to_jiffies(1000);

	/* Set init MIF voltage compensation state */
	info->mif_vol_offset_state = false;

	/* Set init temperature compensation state */
	info->tc_state = false;

	/* Need to initail regsiter setting after getting parameter info */
	/* [28:23] vref [11:8] slope - Tunning parameter */
	__raw_writel(data->slope, info->tmu_base + TMU_CON);

	__raw_writel((CLEAR_RISE_INT | CLEAR_FALL_INT), info->tmu_base + INTCLEAR);

	/* TMU core enable */
	con = __raw_readl(info->tmu_base + TMU_CON);
	con |= (MUX_ADDR_VALUE << 20 | CORE_EN);

	__raw_writel(con, info->tmu_base + TMU_CON);

	/* Because temperature sensing time is appro 940us,
	* tmu is enabled and 1st valid sample can get 1ms after.
	*/
	mdelay(1);

	/*LEV0 LEV1 interrupt enable */
	__raw_writel(INTEN_RISE0 | INTEN_RISE1 | INTEN_FALL0 | INTEN_FALL1,
			info->tmu_base + INTEN);

	return 0;
}

static irqreturn_t tmu_irq(int irq, void *id)
{
	struct tmu_info *info = id;
	unsigned int status;

	disable_irq_nosync(irq);

	status = __raw_readl(info->tmu_base + INTSTAT);
	/* To handle multiple interrupt pending,
	* interrupt by high temperature are serviced with priority.
	*/
	if (status & INTSTAT_FALL1) {
		dev_info(info->dev, "MIF voltage compensation interrupt\n");
		info->tmu_state = TMU_STATUS_MIF_VC;
		__raw_writel(CLEAR_FALL_INT, info->tmu_base + INTCLEAR);
	} else if (status & INTSTAT_FALL0) {
		dev_info(info->dev, "Temperature compensation interrupt\n");
		info->tmu_state = TMU_STATUS_TC;
		__raw_writel(CLEAR_FALL_INT, info->tmu_base + INTCLEAR);
	} else if (status & INTSTAT_RISE1) {
		dev_info(info->dev, "Tripping interrupt\n");
		info->tmu_state = TMU_STATUS_TRIPPED;
		__raw_writel(CLEAR_RISE_INT, info->tmu_base + INTCLEAR);
	} else if (status & INTSTAT_RISE0) {
		dev_info(info->dev, "Throttling interrupt\n");
		__raw_writel(CLEAR_RISE_INT, info->tmu_base + INTCLEAR);
		info->tmu_state = TMU_STATUS_THROTTLED;
	} else {
		dev_err(info->dev, "%s: TMU interrupt error. INTSTAT : %x\n", __func__, status);
		__raw_writel(status, info->tmu_base + INTCLEAR);
		return IRQ_HANDLED;
	}

	queue_delayed_work_on(0, tmu_monitor_wq, &info->polling, usecs_to_jiffies(0));

	return IRQ_HANDLED;
}

static struct dentry *tmu_debugfs;

static int tmu_debug_show(struct seq_file *s, void *unused)
{
	struct tmu_info *info = s->private;
	char *cur_tmu_state;

	seq_printf(s, "Current Temperature : %d\n", get_cur_temp(info));
	switch (info->tmu_state) {
	case TMU_STATUS_INIT:
	case TMU_STATUS_NORMAL:
		cur_tmu_state = "TMU_STATUS_NORMAL";
		break;
	case TMU_STATUS_THROTTLED:
		cur_tmu_state = "TMU_STATUS_THROTTLED";
		break;
	case TMU_STATUS_TRIPPED:
		cur_tmu_state = "TMU_STATUS_TRIPPED";
		break;
	default:
		cur_tmu_state = "INVALID STATUS";
		break;
	}
	seq_printf(s, "Current TMU State : %s\n", cur_tmu_state);
	seq_printf(s, "Memory Throttling : %s\n",
			info->mem_throttled ? "throttled" : "unthrottled");
	seq_printf(s, "Memory throttle auto refresh time : %d ns\n",
			info->auto_refresh_mem_throttle);
	seq_printf(s, "Normal auto refresh time : %d ns\n", info->auto_refresh_normal);
	seq_printf(s, "TMU monitoring sample rate : %d ms\n",
			info->sampling_rate);

	return 0;
}

static int tmu_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, tmu_debug_show, inode->i_private);
}

const static struct file_operations tmu_dev_status_fops = {
	.open		= tmu_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __devinit tmu_probe(struct platform_device *pdev)
{
	struct tmu_info *info;
	struct resource *res;
	int ret;

	if (dev_get_platdata(&pdev->dev) == NULL) {
		dev_err(&pdev->dev, "No platform data\n");
		ret = -ENODEV;
		goto err_out;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(struct tmu_info), GFP_KERNEL);
	if (info == NULL) {
		dev_err(&pdev->dev, "failed to alloc memory!\n");
		ret = -ENOMEM;
		goto err_out;
	}

	info->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory region resource\n");
		ret = -ENODEV;
		goto err_out;
	}

	info->tmu_base = devm_request_and_ioremap(&pdev->dev, res);
	if (info->tmu_base == NULL) {
		dev_err(&pdev->dev, "failed ioremap()\n");
		ret = -ENOMEM;
		goto err_out;
	}

	INIT_DELAYED_WORK_DEFERRABLE(&info->polling, tmu_monitor);

	tmu_monitor_wq = create_freezable_workqueue("tmu");
	if (!tmu_monitor_wq) {
		dev_err(&pdev->dev, "Creation of tmu_monitor_wq failed\n");
		ret = -EFAULT;
		goto err_out;
	}

	info->irq = platform_get_irq(pdev, 0);
	if (info->irq < 0) {
		dev_err(&pdev->dev, "no irq for thermal\n");
		ret = info->irq;
		goto err_wq;
	}

	platform_set_drvdata(pdev, info);

	ret = exynos_tmu_init(info);
	if (ret < 0)
		goto err_noinit;

	ret = request_irq(info->irq, tmu_irq,
			IRQF_DISABLED, "tmu", info);
	if (ret) {
		dev_err(&pdev->dev, "IRQ%d error %d\n", info->irq, ret);
		ret = -EBUSY;
		goto err_noinit;
	}

	tmu_debugfs =
		debugfs_create_file("tmu_dev_status",
				S_IRUGO, NULL, info, &tmu_dev_status_fops);
	if (IS_ERR_OR_NULL(tmu_debugfs)) {
		tmu_debugfs = NULL;
		dev_err(&pdev->dev, "%s: debugfs_create_file() failed\n", __func__);
		goto err_nodbgfs;
	}

	dev_info(&pdev->dev, "Tmu Initialization is sucessful...!\n");
	return 0;

err_nodbgfs:
	free_irq(info->irq, NULL);
err_noinit:
	platform_set_drvdata(pdev, NULL);
err_wq:
	destroy_workqueue(tmu_monitor_wq);
err_out:
	dev_err(&pdev->dev, "initialization failed.\n");
	return ret;
}

static int __devexit tmu_remove(struct platform_device *pdev)
{
	struct tmu_info *info = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&info->polling);
	disable_irq(info->irq);
	free_irq(info->irq, info);
	debugfs_remove(tmu_debugfs);
	destroy_workqueue(tmu_monitor_wq);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int tmu_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tmu_info *info = platform_get_drvdata(pdev);

	pm_tmu_save(info);
	return 0;
}

static int tmu_resume(struct platform_device *pdev)
{
	struct tmu_info *info = platform_get_drvdata(pdev);

	pm_tmu_restore(info);
	return 0;
}

#else
#define tmu_suspend	NULL
#define tmu_resume	NULL
#endif

static struct platform_driver tmu_driver = {
	.probe		= tmu_probe,
	.remove		= __devexit_p(tmu_remove),
	.suspend	= tmu_suspend,
	.resume		= tmu_resume,
	.driver		= {
		.name	=	"exynos_tmu",
		.owner	=	THIS_MODULE,
	},
};

module_platform_driver(tmu_driver);

