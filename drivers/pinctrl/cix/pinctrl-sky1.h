/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Author: Jerry Zhu <Jerry.Zhu@cixtech.com>
 */

#ifndef __DRIVERS_PINCTRL_SKY1_H
#define __DRIVERS_PINCTRL_SKY1_H

struct sky1_pinctrl_group {
	const char *name;
	unsigned long config;
	unsigned int pin;
};

struct sky1_pin_desc {
	const struct pinctrl_pin_desc pin;
	const char * const *func_group;
	unsigned int nfunc;
};

struct sky1_pinctrl_soc_info {
	unsigned int flags;
	const struct sky1_pin_desc *pins;
	unsigned int npins;
};

#define SKY_PINFUNCTION(_pin, _func)				\
((struct sky1_pin_desc) {					\
		.pin = _pin,					\
		.func_group = _func##_group,			\
		.nfunc = ARRAY_SIZE(_func##_group),		\
	})
/**
 * sky1_pin_reg contains 32 bits
 * bit7:bit8 for function select
 * bit0:bit6 for pad configuration
 */
typedef u32 sky1_pin_reg;

/**
 * @dev: a pointer back to containing device
 * @base: the offset to the controller in virtual memory
 */
struct sky1_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctl;
	void __iomem *base;
	const struct sky1_pinctrl_soc_info *info;
	struct sky1_pinctrl_group *groups;
	const char **grp_names;
	sky1_pin_reg *pin_regs;
	unsigned int group_index;
	struct mutex mutex;
	unsigned long *saved_vals;
};

// Flags based on pinctrl-single.c
#define SKY1_PINCTRL_CONTEXT_LOSS_OFF	(1 << 3)

int sky1_base_pinctrl_probe(struct platform_device *pdev,
			const struct sky1_pinctrl_soc_info *info);

#endif /* __DRIVERS_PINCTRL_SKY1_H */
