#include <linux/init.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>

#include <mach/gcontrol.h>

#define CUSTOM_GPIO_GPS_POWER_EN EXYNOS5_GPX0(7)
#define CUSTOM_GPIO_GPS_RESET EXYNOS5_GPB1(2)

struct csr_platform_data csr_platdata = {
	.power = CUSTOM_GPIO_GPS_POWER_EN,
	.reset = CUSTOM_GPIO_GPS_RESET,
};

static struct platform_device csr_gps = {
	.name           = "csrgps",
	.id		= -1,
	.dev = {
	.platform_data = &csr_platdata,
	},
};

static struct platform_device *arndale_gps_devs[] __initdata = {
	&csr_gps,
};

void __init exynos5_arndale_gps_init(void)
{
	platform_add_devices(arndale_gps_devs, ARRAY_SIZE(arndale_gps_devs));
}

