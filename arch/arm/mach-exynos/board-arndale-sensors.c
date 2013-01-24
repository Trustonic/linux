#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <plat/gpio-cfg.h>

#include <linux/mpu.h>

static struct mpu_platform_data mpu_data = {
	.int_config  = 0x10,
	.level_shifter = 0,
	.orientation = {
		1,  0,  0,
		0,  1,  0,
		0,  0,  1,
	},
	.sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS,
	.sec_slave_id   = COMPASS_ID_AK8975,
	.secondary_i2c_addr = 0x0C,
	.secondary_orientation = {
		 1,  0,  0,
		 0, -1,  0,
		 0,  0,  1,
	},
};

static struct i2c_board_info i2c_devs5[] __initdata = {
        {
                I2C_BOARD_INFO("mpu6050", 0x68),
                .irq = IRQ_EINT(10),
                .platform_data = &mpu_data,
        },
};

void __init exynos5_arndale_sensors_init(void)
{
	i2c_register_board_info(5, i2c_devs5, ARRAY_SIZE(i2c_devs5));
}

