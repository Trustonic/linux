/*****************touchscreen resolution setting*************/

#define R8C_3GA_2TG
//#define R8C_AUO_I2C

#ifdef R8C_AUO_I2C
  #ifndef R8C_3GA_2TG
  #define R8C_3GA_2TG
  #endif
#endif


#define TOUCHSCREEN_MINX 0
#define TOUCHSCREEN_MAXX 800
#define TOUCHSCREEN_MINY 0
#define TOUCHSCREEN_MAXY 480


#include <plat/gpio-cfg.h>
#include <mach/gpio.h>

#define ATTB		S3C64XX_GPN(11)
#define get_attb_value	gpio_get_value
#define	RESETPIN_CFG	s3c_gpio_cfgpin(S3C64XX_GPE(1),S3C_GPIO_OUTPUT)
#define	RESETPIN_SET0 	gpio_direction_output(S3C64XX_GPE(1),0)
#define	RESETPIN_SET1	gpio_direction_output(S3C64XX_GPE(1),1)	
