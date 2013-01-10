/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <linux/pwm_backlight.h>
#include <linux/delay.h>
#include <linux/fb.h>

#include <video/platform_lcd.h>

#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/fb.h>
#include <plat/dp.h>
#include <plat/gpio-cfg.h>
#include <plat/backlight.h>
#include <plat/regs-fb-v4.h>

#include <mach/map.h>

#include <plat/dsim.h>
#include <plat/mipi_dsi.h>

#include "board-smdk5250.h"

static void smdk5250_fimd_gpio_setup_24bpp(void)
{
	u32 reg;

	/* basic fimd init */
	exynos5_fimd1_gpio_setup_24bpp();

	/* Reference clcok selection for DPTX_PHY: pad_osc_clk_24M */
	reg = __raw_readl(S3C_VA_SYS + 0x04d4);
	reg = (reg & ~(0x1 << 0)) | (0x0 << 0);
	__raw_writel(reg, S3C_VA_SYS + 0x04d4);

	/* DPTX_PHY: XXTI */
	reg = __raw_readl(S3C_VA_SYS + 0x04d8);
	reg = (reg & ~(0x1 << 3)) | (0x0 << 3);
	__raw_writel(reg, S3C_VA_SYS + 0x04d8);
}

#if defined(CONFIG_LCD_MIPI_S6E8AB0)
static void mipi_lcd_set_power(struct plat_lcd_data *pd,
				unsigned int power)
{
	/* reset */
	gpio_request_one(EXYNOS5_GPX1(5), GPIOF_OUT_INIT_HIGH, "GPX1");

	mdelay(20);
	if (power) {
		/* fire nRESET on power up */
		gpio_set_value(EXYNOS5_GPX1(5), 0);
		mdelay(20);
		gpio_set_value(EXYNOS5_GPX1(5), 1);
		mdelay(20);
		gpio_free(EXYNOS5_GPX1(5));
	} else {
		/* fire nRESET on power off */
		gpio_set_value(EXYNOS5_GPX1(5), 0);
		mdelay(20);
		gpio_set_value(EXYNOS5_GPX1(5), 1);
		mdelay(20);
		gpio_free(EXYNOS5_GPX1(5));
	}
	mdelay(20);
	/* power */
	gpio_request_one(EXYNOS5_GPX3(0), GPIOF_OUT_INIT_LOW, "GPX3");
	if (power) {
		/* fire nRESET on power up */
		gpio_set_value(EXYNOS5_GPX3(0), 1);
		gpio_free(EXYNOS5_GPX3(0));
	} else {
		/* fire nRESET on power off */
		gpio_set_value(EXYNOS5_GPX3(0), 0);
		gpio_free(EXYNOS5_GPX3(0));
	}
}

static struct plat_lcd_data smdk5250_mipi_lcd_data = {
	.set_power	= mipi_lcd_set_power,
};

static struct platform_device smdk5250_mipi_lcd = {
	.name			= "platform-lcd",
	.dev.platform_data	= &smdk5250_mipi_lcd_data,
};

static struct s3c_fb_pd_win smdk5250_fb_win0 = {
	.win_mode = {
		.left_margin	= 0x4,
		.right_margin	= 0x4,
		.upper_margin	= 4,
		.lower_margin	= 4,
		.hsync_len	= 4,
		.vsync_len	= 4,
		.xres		= 1280,
		.yres		= 800,
	},
	.virtual_x		= 1280,
	.virtual_y		= 800 * 2,
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk5250_fb_win1 = {
	.win_mode = {
		.left_margin	= 0x4,
		.right_margin	= 0x4,
		.upper_margin	= 4,
		.lower_margin	= 4,
		.hsync_len	= 4,
		.vsync_len	= 4,
		.xres		= 1280,
		.yres		= 800,
	},
	.virtual_x		= 1280,
	.virtual_y		= 800 * 2,
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk5250_fb_win2 = {
	.win_mode = {
		.left_margin	= 0x4,
		.right_margin	= 0x4,
		.upper_margin	= 4,
		.lower_margin	= 4,
		.hsync_len	= 4,
		.vsync_len	= 4,
		.xres		= 1280,
		.yres		= 800,
	},
	.virtual_x		= 1280,
	.virtual_y		= 800 * 2,
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};
#elif defined(CONFIG_LCD_MIPI_TC358764)
static void mipi_lcd_set_power(struct plat_lcd_data *pd,
				unsigned int power)
{
	/* reset */
	gpio_request_one(EXYNOS5_GPX1(5), GPIOF_OUT_INIT_HIGH, "GPX1");

	mdelay(20);
	if (power) {
		/* fire nRESET on power up */
		gpio_set_value(EXYNOS5_GPX1(5), 0);
		mdelay(20);
		gpio_set_value(EXYNOS5_GPX1(5), 1);
		mdelay(20);
		gpio_free(EXYNOS5_GPX1(5));
	} else {
		/* fire nRESET on power off */
		gpio_set_value(EXYNOS5_GPX1(5), 0);
		mdelay(20);
		gpio_set_value(EXYNOS5_GPX1(5), 1);
		mdelay(20);
		gpio_free(EXYNOS5_GPX1(5));
	}
	mdelay(20);
	/* power */
	gpio_request_one(EXYNOS5_GPX3(0), GPIOF_OUT_INIT_LOW, "GPX3");
	if (power) {
		/* fire nRESET on power up */
		gpio_set_value(EXYNOS5_GPX3(0), 1);
		gpio_free(EXYNOS5_GPX3(0));
	} else {
		/* fire nRESET on power off */
		gpio_set_value(EXYNOS5_GPX3(0), 0);
		gpio_free(EXYNOS5_GPX3(0));
	}
}

static struct plat_lcd_data smdk5250_mipi_lcd_data = {
	.set_power	= mipi_lcd_set_power,
};

static struct platform_device smdk5250_mipi_lcd = {
	.name			= "platform-lcd",
	.dev.platform_data	= &smdk5250_mipi_lcd_data,
};

static struct s3c_fb_pd_win smdk5250_fb_win0 = {
	.win_mode = {
		.left_margin	= 4,
		.right_margin	= 4,
		.upper_margin	= 4,
		.lower_margin	= 4,
		.hsync_len	= 4,
		.vsync_len	= 4,
		.xres		= 1280,
		.yres		= 800,
	},
	.virtual_x		= 1280,
	.virtual_y		= 800 * 2,
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk5250_fb_win1 = {
	.win_mode = {
		.left_margin	= 4,
		.right_margin	= 4,
		.upper_margin	= 4,
		.lower_margin	= 4,
		.hsync_len	= 4,
		.vsync_len	= 4,
		.xres		= 1280,
		.yres		= 800,
	},
	.virtual_x		= 1280,
	.virtual_y		= 800 * 2,
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};
static struct s3c_fb_pd_win smdk5250_fb_win2 = {
	.win_mode = {
		.left_margin	= 4,
		.right_margin	= 4,
		.upper_margin	= 4,
		.lower_margin	= 4,
		.hsync_len	= 4,
		.vsync_len	= 4,
		.xres		= 1280,
		.yres		= 800,
	},
	.virtual_x		= 1280,
	.virtual_y		= 800 * 2,
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};
#elif defined(CONFIG_S5P_DP)
static void smdk5250_backlight_on(void);
static void smdk5250_backlight_off(void);

#define LCD_POWER_OFF_TIME_US	(500 * USEC_PER_MSEC)

static ktime_t lcd_on_time;

static void smdk5250_lcd_on(void)
{
	s64 us = ktime_us_delta(lcd_on_time, ktime_get_boottime());

	if (us > LCD_POWER_OFF_TIME_US) {
		pr_warn("lcd on sleep time too long\n");
		us = LCD_POWER_OFF_TIME_US;
	}

	if (us > 0)
		usleep_range(us, us);

	if (get_smdk5250_rev() == SMDK5250_REV_0_0) {
		gpio_set_value(EXYNOS5_GPD0(5), 1);
		usleep_range(200000, 200000);
	} else {
		gpio_set_value(EXYNOS5_GPH1(1), 1);
		usleep_range(200000, 200000);
	}
}

static void smdk5250_lcd_off(void)
{
	if (get_smdk5250_rev() == SMDK5250_REV_0_0)
		gpio_set_value(EXYNOS5_GPD0(5), 0);
	else
		gpio_set_value(EXYNOS5_GPH1(1), 0);

	lcd_on_time = ktime_add_us(ktime_get_boottime(), LCD_POWER_OFF_TIME_US);
}

static void smdk5250_lcd_set_power(struct plat_lcd_data *pd,
				unsigned int power)
{
	if (power)
		smdk5250_lcd_on();
	else
		smdk5250_lcd_off();
}

static struct plat_lcd_data smdk5250_dp_lcd_data = {
	.set_power	= smdk5250_lcd_set_power,
};

static struct platform_device smdk5250_dp_lcd = {
	.name	= "platform-lcd",
	.dev	= {
		.parent		= &s5p_device_fimd1.dev,
		.platform_data	= &smdk5250_dp_lcd_data,
	},
};

static struct s3c_fb_pd_win smdk5250_fb_win2 = {
	.win_mode = {
		.left_margin	= 80,
		.right_margin	= 48,
		.upper_margin	= 37,
		.lower_margin	= 3,
		.hsync_len	= 32,
		.vsync_len	= 6,
		.xres		= 2560,
		.yres		= 1600,
	},
	.virtual_x		= 2560,
	.virtual_y		= 1600 * 2,
	.max_bpp		= 32,
	.default_bpp		= 24,
	.width			= 218,
	.height			= 136,
};
#endif

static struct s3c_fb_platdata smdk5250_lcd1_pdata __initdata = {
#if defined(CONFIG_LCD_MIPI_S6E8AB0)
	.win[0]		= &smdk5250_fb_win0,
	.win[1]		= &smdk5250_fb_win1,
	.win[2]		= &smdk5250_fb_win2,
	.win[3]		= &smdk5250_fb_win2,
	.win[4]		= &smdk5250_fb_win2,
#elif defined(CONFIG_LCD_MIPI_TC358764)
	.win[0]		= &smdk5250_fb_win0,
	.win[1]		= &smdk5250_fb_win1,
	.win[2]		= &smdk5250_fb_win2,
	.win[3]		= &smdk5250_fb_win2,
	.win[4]		= &smdk5250_fb_win2,
#elif defined(CONFIG_S5P_DP)
	.win[0]		= &smdk5250_fb_win2,
	.win[1]		= &smdk5250_fb_win2,
	.win[2]		= &smdk5250_fb_win2,
	.win[3]		= &smdk5250_fb_win2,
	.win[4]		= &smdk5250_fb_win2,
#endif
	.default_win	= 0,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
#if defined(CONFIG_LCD_MIPI_S6E8AB0)
	.vidcon1	= VIDCON1_INV_VCLK,
#elif defined(CONFIG_LCD_MIPI_TC358764)
	.vidcon1	= VIDCON1_INV_VCLK,
#elif defined(CONFIG_S5P_DP)
	.vidcon1	= 0,
	.backlight_off	= smdk5250_backlight_off,
	.lcd_off	= smdk5250_lcd_off,
#endif
	.setup_gpio	= smdk5250_fimd_gpio_setup_24bpp,
};

#ifdef CONFIG_FB_MIPI_DSIM
#if defined(CONFIG_LCD_MIPI_S6E8AB0)
static struct mipi_dsim_config dsim_info = {
	.e_interface     = DSIM_VIDEO,
	.e_pixel_format  = DSIM_24BPP_888,
	/* main frame fifo auto flush at VSYNC pulse */
	.auto_flush		= false,
	.eot_disable		= false,
	.auto_vertical_cnt	= true,
	.hse = false,
	.hfp = false,
	.hbp = false,
	.hsa = false,

	.e_no_data_lane	= DSIM_DATA_LANE_4,
	.e_byte_clk	= DSIM_PLL_OUT_DIV8,
	.e_burst_mode	= DSIM_BURST,

	.p = 2,
	.m = 57,
	.s = 1,
	/* D-PHY PLL stable time spec :min = 200usec ~ max 400usec */
	.pll_stable_time = 500,

	.esc_clk = 20 * 1000000,	/* escape clk : 10MHz */

	/* stop state holding counter after bta change count 0 ~ 0xfff */
	.stop_holding_cnt = 0x0fff,
	.bta_timeout = 0xff,		/* bta timeout 0 ~ 0xff */
	.rx_timeout = 0xffff,		/* lp rx timeout 0 ~ 0xffff */

	.dsim_ddi_pd = &s6e8ab0_mipi_lcd_driver,
};
static struct mipi_dsim_lcd_config dsim_lcd_info = {
	.rgb_timing.left_margin  = 0xa,
	.rgb_timing.right_margin = 0xa,
	.rgb_timing.upper_margin = 80,
	.rgb_timing.lower_margin = 48,
	.rgb_timing.hsync_len	 = 5,
	.rgb_timing.vsync_len	 = 32,
	.cpu_timing.cs_setup	 = 0,
	.cpu_timing.wr_setup	 = 1,
	.cpu_timing.wr_act	 = 0,
	.cpu_timing.wr_hold	 = 0,
	.lcd_size.width		 = 1280,
	.lcd_size.height	 = 800,
};
#elif defined(CONFIG_LCD_MIPI_TC358764)
static struct mipi_dsim_config dsim_info = {
	.e_interface	 = DSIM_VIDEO,
	.e_pixel_format  = DSIM_24BPP_888,
	/* main frame fifo auto flush at VSYNC pulse */
	.auto_flush	 = false,
	.eot_disable	 = false,
	.auto_vertical_cnt = false,
	.hse = false,
	.hfp = false,
	.hbp = false,
	.hsa = false,

	.e_no_data_lane  = DSIM_DATA_LANE_4,
	.e_byte_clk	 = DSIM_PLL_OUT_DIV8,
	.e_burst_mode	 = DSIM_BURST,

	.p = 3,
	.m = 115,
	.s = 1,

	/* D-PHY PLL stable time spec :min = 200usec ~ max 400usec */
	.pll_stable_time = 500,

	.esc_clk = 0.4 * 1000000,	/* escape clk : 10MHz */

	/* stop state holding counter after bta change count 0 ~ 0xfff */
	.stop_holding_cnt	= 0x0f,
	.bta_timeout		= 0xff,		/* bta timeout 0 ~ 0xff */
	.rx_timeout		= 0xffff,	/* lp rx timeout 0 ~ 0xffff */

	.dsim_ddi_pd = &tc358764_mipi_lcd_driver,
};

static struct mipi_dsim_lcd_config dsim_lcd_info = {
	.rgb_timing.left_margin  = 0x4,
	.rgb_timing.right_margin = 0x4,
	.rgb_timing.upper_margin = 0x4,
	.rgb_timing.lower_margin = 0x4,
	.rgb_timing.hsync_len	 = 0x4,
	.rgb_timing.vsync_len	 = 0x4,
	.cpu_timing.cs_setup	 = 0,
	.cpu_timing.wr_setup	 = 1,
	.cpu_timing.wr_act	 = 0,
	.cpu_timing.wr_hold	 = 0,
	.lcd_size.width		 = 1280,
	.lcd_size.height	 = 800,
};
#endif

static struct s5p_platform_mipi_dsim dsim_platform_data = {
	.clk_name		= "dsim0",
	.dsim_config		= &dsim_info,
	.dsim_lcd_config	= &dsim_lcd_info,

	.part_reset		= s5p_dsim_part_reset,
	.init_d_phy		= s5p_dsim_init_d_phy,
	.get_fb_frame_done	= NULL,
	.trigger		= NULL,
	/*
	* the stable time of needing to write data on SFR
	* when the mipi mode becomes LP mode.
	*/
	.delay_for_stabilization = 600,
};
#endif

#ifdef CONFIG_S5P_DP
static struct video_info smdk5250_dp_config = {
	.name			= "WQXGA(2560x1600) LCD, for SMDK TEST",

	.h_sync_polarity	= 0,
	.v_sync_polarity	= 0,
	.interlaced		= 0,

	.color_space		= COLOR_RGB,
	.dynamic_range		= VESA,
	.ycbcr_coeff		= COLOR_YCBCR601,
	.color_depth		= COLOR_8,

	.link_rate		= LINK_RATE_2_70GBPS,
	.lane_count		= LANE_COUNT4,
};

static void smdk5250_backlight_on(void)
{
	usleep_range(97000, 97000);

	/* LED_BACKLIGHT_RESET: GPX1_5 */
	gpio_set_value(EXYNOS5_GPX1(5), 1);
}

static void smdk5250_backlight_off(void)
{
	/* LED_BACKLIGHT_RESET: GPX1_5 */
	gpio_set_value(EXYNOS5_GPX1(5), 0);
}

static struct s5p_dp_platdata smdk5250_dp_data __initdata = {
	.video_info	= &smdk5250_dp_config,
	.phy_init	= s5p_dp_phy_init,
	.phy_exit	= s5p_dp_phy_exit,
	.backlight_on	= smdk5250_backlight_on,
	.backlight_off  = smdk5250_backlight_off,
	.lcd_on		= smdk5250_lcd_on,
	.lcd_off	= smdk5250_lcd_off,
};
#endif

/* LCD Backlight data */
static struct samsung_bl_gpio_info smdk5250_bl_gpio_info = {
	.no	= EXYNOS5_GPB2(0),
	.func	= S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data smdk5250_bl_data = {
	.pwm_id		= 0,
	.pwm_period_ns	= 1000000,
	.dft_brightness	= 102,
};

static struct platform_device *smdk5250_display_devices[] __initdata = {
#ifdef CONFIG_FB_MIPI_DSIM
	&smdk5250_mipi_lcd,
	&s5p_device_mipi_dsim,
#endif
	&s5p_device_fimd1,
#ifdef CONFIG_S5P_DP
	&s5p_device_dp,
	&smdk5250_dp_lcd,
#endif
};

void __init exynos5_smdk5250_display_init(void)
{
	if (get_smdk5250_rev() == SMDK5250_REV_0_0) {
		/* LCD_EN: GPD0_5 */
		gpio_request_one(EXYNOS5_GPD0(5), GPIOF_OUT_INIT_LOW, "GPD0");

		/* LCD_APS_EN_2.8V: GPD0_6 */
		gpio_request_one(EXYNOS5_GPD0(6), GPIOF_OUT_INIT_LOW, "GPD0");
		gpio_export(EXYNOS5_GPD0(6), true);
	} else {
		/* LCD_EN: GPH1_1 */
		gpio_request_one(EXYNOS5_GPH1(1), GPIOF_OUT_INIT_LOW, "GPH1");

		/* LCD_APS_EN_2.8V: GPD0_6 */
		gpio_request_one(EXYNOS5_GPH1(2), GPIOF_OUT_INIT_LOW, "GPH1");
		gpio_export(EXYNOS5_GPH1(2), true);
	}

	/* LED_BACKLIGHT_RESET: GPX1_5 */
	gpio_request_one(EXYNOS5_GPX1(5), GPIOF_OUT_INIT_LOW, "GPX1");

	/* LCD_PWM_IN_2.8V: LCD_B_PWM, GPB2_0 */
	gpio_request_one(EXYNOS5_GPB2(0), GPIOF_OUT_INIT_LOW, "GPB2");
	gpio_free(EXYNOS5_GPB2(0));

	samsung_bl_set(&smdk5250_bl_gpio_info, &smdk5250_bl_data);

	s5p_fimd1_set_platdata(&smdk5250_lcd1_pdata);
	dev_set_name(&s5p_device_fimd1.dev, "exynos5-fb.1");
	clk_add_alias("lcd", "exynos5-fb.1", "fimd", &s5p_device_fimd1.dev);
#ifdef CONFIG_S5P_DP
	s5p_dp_set_platdata(&smdk5250_dp_data);
#endif
#ifdef CONFIG_FB_MIPI_DSIM
	s5p_dsim_set_platdata(&dsim_platform_data);
#endif

#if defined(CONFIG_S5P_DP)
	exynos5_fimd1_setup_clock(&s5p_device_fimd1.dev,
			"sclk_fimd", "sclk_vpll", 268 * MHZ);
#else
	exynos5_fimd1_setup_clock(&s5p_device_fimd1.dev,
			"sclk_fimd", "mout_mpll_user", 800 * MHZ);
#endif
	platform_add_devices(smdk5250_display_devices,
			ARRAY_SIZE(smdk5250_display_devices));
}
