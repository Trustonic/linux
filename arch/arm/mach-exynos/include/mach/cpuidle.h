/*
 * Copyright 2013 Samsung Electronics Co., Ltd.
 *	http://www.samsung.com/
 *
 * Header file for cpuidle support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_CPUIDLE_H
#define __ASM_ARCH_CPUIDLE_H

enum lpa_check_device_t {
	LPA_CDEV_USB_D = 0,
	LPA_CDEV_USB_H,
	LPA_CDEV_USB_DRD,
	LPA_CDEV_GPS,
	LPA_CDEV_BT,
	LPA_CDEV_END,
};

struct cpuidle_lpa_device {
	struct list_head node;
	enum lpa_check_device_t cdev;
	bool usage;
};

int exynos_add_lpa_device(enum lpa_check_device_t cdev);
int exynos_remove_lpa_device(enum lpa_check_device_t cdev);
int exynos_set_lpa_device_usage(enum lpa_check_device_t cdev,
					int usage);

#if defined(CONFIG_CPU_IDLE)
#define add_lpa_device(a)		\
				exynos_add_lpa_device(a);
#define remove_lpa_device(a)		\
				exynos_remove_lpa_device(a);
#define set_lpa_device_usage(a, b)	\
				exynos_set_lpa_device_usage(a, b);
#else
#define add_lpa_device(a)		do {} while (0)
#define remove_lpa_device(a)		do {} while (0)
#define set_lpa_device_usage(a, b)	do {} while (0)
#endif

#endif /* __ASM_ARCH_CPUIDLE_H */
