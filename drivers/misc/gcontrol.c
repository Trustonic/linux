#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>

#include <plat/gpio-cfg.h>
#include <mach/gcontrol.h>

unsigned int gps_gpio=0;
unsigned int gps_reset_gpio=0;

int gps_power_control(unsigned int pw, int onoff);
int gps_reset_control(unsigned int rst, int onoff);


static int __devinit csr_gps_drv_probe(struct platform_device *pdev)
{
	int ret;
	struct csr_platform_data *p = (struct csr_platform_data *)pdev->dev.platform_data;

	gps_gpio  = p->power;
	gps_reset_gpio = p->reset;

	ret = gps_power_control( gps_gpio, 0 );

	if(ret)
	{
		printk("============  not gps power probe !!!!! ===============\n");	
		return ret;
	}
	printk("============  gps power probe !!!!! ===============\n");

	ret = gps_reset_control( gps_reset_gpio, 1 );

	if(ret == 0)
	{
		printk("============  not gps reset probe !!!!! ===============\n");	
		return ret;
	}
	printk("============  gps reset probe !!!!! ===============\n");

	return 0;
}

int gps_power_control(unsigned int pw, int onoff)
{
	s3c_gpio_cfgpin(pw, S3C_GPIO_SFN(1));
	s3c_gpio_setpull(pw, S3C_GPIO_PULL_NONE);
	gpio_set_value(pw, !!(onoff));
	return gpio_get_value(pw);
}

int gps_reset_control(unsigned int rst, int onoff)
{
	s3c_gpio_cfgpin(rst, S3C_GPIO_SFN(1));
	s3c_gpio_setpull(rst, S3C_GPIO_PULL_UP);
	gpio_set_value(rst, !!(onoff));
	return gpio_get_value(rst);
}

static int __devexit csr_gps_drv_remove(struct platform_device *pdev)
{
	return 0;
}

static int gps_power = 0;
static int gps_reset = 1;

long 
gps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{	
	int err = 0;
	switch (cmd)
	{
	case 0:
		if (gps_power != 0)
			gps_power = gps_power_control(gps_gpio,0);
		if (gps_power != 0)
			err = -1;
		break;
	default:
	case 1:
		if (gps_power == 0)
			gps_power = gps_power_control(gps_gpio,1);
		if (gps_power == 0)
			err = -1;
		break;
	}
	return err;
}

ssize_t 
gps_read(struct file *filp, char *userbuf, size_t count, loff_t *f_pos)
{
	char buf[2] = { '0' + gps_power, 0 };
	return simple_read_from_buffer(userbuf, count, f_pos, buf, sizeof(buf));
}

ssize_t 
gps_write(struct file *filp, const char *userbuf, size_t count, loff_t *f_pos)
{
	if (count > 0)
	{
		int state = simple_strtoul(userbuf, NULL, 10) ? 1 : 0;
		if (gps_power != state)
			gps_power = gps_power_control(gps_gpio, state);
	}
	return count;
}

int 
gps_open(struct inode *inode, struct file *filp)
{
 	if( !try_module_get(THIS_MODULE) )
    	return -ENODEV;	
	return 0;
}

int 
gps_release(struct inode *inode, struct file *filp)
{
	module_put(THIS_MODULE);
	return 0;
}

struct file_operations gps_fops = 
{
	.owner    = THIS_MODULE,
	.read     = gps_read,
	.write    = gps_write,
	.unlocked_ioctl = gps_ioctl,
	.open     = gps_open,
	.release  = gps_release,
};

struct miscdevice gps_miscdev = 
{
	MISC_DYNAMIC_MINOR,
	"gcontrol",
	&gps_fops
};

long 
gps_reset_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{	
	int err = 0;
	switch (cmd)
	{
	case 0:
		if (gps_reset != 0)
			gps_reset = gps_reset_control(gps_reset_gpio,0);
		if (gps_reset != 0)
			err = -1;
		break;
	default:
	case 1:
		if (gps_reset == 0)
			gps_reset = gps_reset_control(gps_reset_gpio,1);
		if (gps_reset == 0)
			err = -1;
		break;
	}
	return err;
}

ssize_t 
gps_reset_read(struct file *filp, char *userbuf, size_t count, loff_t *f_pos)
{
	char buf[2] = { '0' + gps_reset, 0 };
	return simple_read_from_buffer(userbuf, count, f_pos, buf, sizeof(buf));
}

ssize_t 
gps_reset_write(struct file *filp, const char *userbuf, size_t count, loff_t *f_pos)
{
	if (count > 0)
	{
		int state = simple_strtoul(userbuf, NULL, 10) ? 1 : 0;
		if (gps_reset != state)
			gps_reset = gps_reset_control(gps_reset_gpio, state);
	}
	return count;
}

int 
gps_reset_open(struct inode *inode, struct file *filp)
{
 	if( !try_module_get(THIS_MODULE) )
    	return -ENODEV;	
	return 0;
}

int 
gps_reset_release(struct inode *inode, struct file *filp)
{
	module_put(THIS_MODULE);
	return 0;
}

struct file_operations gps_reset_fops = 
{
	.owner    = THIS_MODULE,
	.read     = gps_reset_read,
	.write    = gps_reset_write,
	.unlocked_ioctl = gps_reset_ioctl,
	.open     = gps_reset_open,
	.release  = gps_reset_release,
};

struct miscdevice gps_reset_miscdev = 
{
	MISC_DYNAMIC_MINOR,
	"gpsreset",
	&gps_reset_fops
};

static struct platform_driver csr_gps_driver = {
	.probe = csr_gps_drv_probe,
	.remove = __devexit_p(csr_gps_drv_remove),
	.driver = {
		.name	= "csrgps",
		.owner	= THIS_MODULE,
	},
	.suspend = NULL,
	.resume = NULL,
};

static int __init 
gps_init(void)
{
	int ret;

	ret =	platform_driver_register(&csr_gps_driver);
	if(ret)
		return ret;

	gps_power_control(gps_gpio,0);

	ret = misc_register(&gps_miscdev);
	if(ret)
		return ret;

	ret = misc_register(&gps_reset_miscdev);
	if(ret)
		return ret;

	return 0;
}

static void __exit 
gps_exit(void)
{
	gps_power_control(gps_gpio,0);
	gps_reset_control(gps_reset_gpio,0);
	platform_driver_unregister(&csr_gps_driver);
	misc_deregister(&gps_miscdev);
	misc_deregister(&gps_reset_miscdev);
}

module_init(gps_init);
module_exit(gps_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GPS Power Driver");

