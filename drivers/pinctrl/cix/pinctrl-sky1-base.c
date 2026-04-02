// SPDX-License-Identifier: GPL-2.0+
//
// Author: Jerry Zhu <Jerry.Zhu@cixtech.com>
// Author: Gary Yang <gary.yang@cixtech.com>

#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinctrl-acpi.h"
#include "../pinctrl-utils.h"
#include "../pinmux.h"
#include "pinctrl-sky1.h"

#define SKY1_PIN_SIZE		(4)
#define SKY1_MUX_MASK		GENMASK(8, 7)
#define SKY1_MUX_SHIFT		(7)
#define SKY1_PULLCONF_MASK	GENMASK(6, 5)
#define SKY1_PULLUP_BIT		(6)
#define SKY1_PULLDN_BIT		(5)
#define SKY1_DS_MASK		GENMASK(3, 0)

#define CIX_PIN_NO_SHIFT	(8)
#define CIX_PIN_FUN_MASK	GENMASK(1, 0)
#define CIX_GET_PIN_NO(x)	((x) >> CIX_PIN_NO_SHIFT)
#define CIX_GET_PIN_FUNC(x)	((x) & CIX_PIN_FUN_MASK)
#define SKY1_DEFAULT_DS_VAL	(4)

static const char * const sky1_gpio_functions[] = {
	"func0", "func1", "func2", "func3",
};

static unsigned char sky1_ds_table[] = {
	2, 3, 5, 6, 8, 9, 11, 12, 13, 14, 17, 18, 20, 21, 23, 24,
};

static bool sky1_pctrl_is_function_valid(struct sky1_pinctrl *spctl,
		u32 pin_num, u32 fnum)
{
	int i;

	for (i = 0; i < spctl->info->npins; i++) {
		const struct sky1_pin_desc *pin = spctl->info->pins + i;

		if (pin->pin.number == pin_num) {
			if (fnum < pin->nfunc)
				return true;

			break;
		}
	}

	return false;
}

static int sky1_pctrl_dt_node_to_map_func(struct sky1_pinctrl *spctl,
		u32 pin, u32 fnum, struct sky1_pinctrl_group *grp,
		struct pinctrl_map **map, unsigned int *reserved_maps,
		unsigned int *num_maps)
{
	bool ret;

	if (*num_maps == *reserved_maps)
		return -ENOSPC;

	(*map)[*num_maps].type = PIN_MAP_TYPE_MUX_GROUP;
	(*map)[*num_maps].data.mux.group = grp->name;

	ret = sky1_pctrl_is_function_valid(spctl, pin, fnum);
	if (!ret) {
		dev_err(spctl->dev, "invalid function %d on pin %d .\n",
				fnum, pin);
		return -EINVAL;
	}

	(*map)[*num_maps].data.mux.function = sky1_gpio_functions[fnum];
	(*num_maps)++;

	return 0;
}

static struct sky1_pinctrl_group *
sky1_pctrl_find_group_by_pin(struct sky1_pinctrl *spctl, u32 pin)
{
	int i;

	for (i = 0; i < spctl->info->npins; i++) {
		struct sky1_pinctrl_group *grp =
			(struct sky1_pinctrl_group *)spctl->groups + i;

		if (grp->pin == pin)
			return grp;
	}

	return NULL;
}

static int sky1_pctrl_dt_subnode_to_map(struct pinctrl_dev *pctldev,
				      struct device_node *node,
				      struct pinctrl_map **map,
				      unsigned int *reserved_maps,
				      unsigned int *num_maps)
{
	struct property *pins;
	u32 pinfunc, pin, func;
	int num_pins, num_funcs, maps_per_pin;
	unsigned long *configs;
	unsigned int num_configs;
	bool has_config = false;
	int i, err;
	unsigned int reserve = 0;
	struct sky1_pinctrl_group *grp;
	struct sky1_pinctrl *spctl = pinctrl_dev_get_drvdata(pctldev);

	pins = of_find_property(node, "pinmux", NULL);
	if (!pins) {
		dev_err(spctl->dev, "missing pins property in node %pOFn .\n",
				node);
		return -EINVAL;
	}

	err = pinconf_generic_parse_dt_config(node, pctldev, &configs,
		&num_configs);
	if (err)
		return err;

	if (num_configs)
		has_config = true;

	num_pins = pins->length / sizeof(u32);
	num_funcs = num_pins;
	maps_per_pin = 0;
	if (num_funcs)
		maps_per_pin++;
	if (has_config && num_pins >= 1)
		maps_per_pin++;

	if (!num_pins || !maps_per_pin) {
		err = -EINVAL;
		goto exit;
	}

	reserve = num_pins * maps_per_pin;

	err = pinctrl_utils_reserve_map(pctldev, map,
			reserved_maps, num_maps, reserve);
	if (err < 0)
		goto exit;

	for (i = 0; i < num_pins; i++) {
		err = of_property_read_u32_index(node, "pinmux",
				i, &pinfunc);
		if (err)
			goto exit;

		pin = CIX_GET_PIN_NO(pinfunc);
		func = CIX_GET_PIN_FUNC(pinfunc);
		pctldev->num_functions = ARRAY_SIZE(sky1_gpio_functions);

		if (pin >= pctldev->desc->npins ||
			func >= pctldev->num_functions) {
			dev_err(spctl->dev, "invalid pins value.\n");
			err = -EINVAL;
			goto exit;
		}

		grp = sky1_pctrl_find_group_by_pin(spctl, pin);
		if (!grp) {
			dev_err(spctl->dev, "unable to match pin %d to group\n",
					pin);
			err = -EINVAL;
			goto exit;
		}

		err = sky1_pctrl_dt_node_to_map_func(spctl, pin, func, grp,
				map, reserved_maps, num_maps);
		if (err < 0)
			goto exit;

		if (has_config) {
			err = pinctrl_utils_add_map_configs(pctldev, map,
					reserved_maps, num_maps, grp->name,
					configs, num_configs,
					PIN_MAP_TYPE_CONFIGS_GROUP);
			if (err < 0)
				goto exit;
		}
	}

	err = 0;

exit:
	kfree(configs);
	return err;
}

static int sky1_pctrl_dt_node_to_map(struct pinctrl_dev *pctldev,
				 struct device_node *np_config,
				 struct pinctrl_map **map, unsigned int *num_maps)
{
	unsigned int reserved_maps;
	int ret;

	*map = NULL;
	*num_maps = 0;
	reserved_maps = 0;

	for_each_child_of_node_scoped(np_config, np) {
		ret = sky1_pctrl_dt_subnode_to_map(pctldev, np, map,
				&reserved_maps, num_maps);
		if (ret < 0) {
			pinctrl_utils_free_map(pctldev, *map, *num_maps);
			return ret;
		}
	}

	return 0;
}

static void sky1_dt_free_map(struct pinctrl_dev *pctldev,
			     struct pinctrl_map *map,
			     unsigned int num_maps)
{
	kfree(map);
}

static int sky1_acpi_node_to_map(struct pinctrl_dev *pctldev,
				struct pinctrl_acpi_resource *info,
				struct pinctrl_map **map,
				unsigned *num_maps_out)
{
	struct sky1_pinctrl *spctl = pinctrl_dev_get_drvdata(pctldev);
	struct pinctrl_map *new_map;
	struct pinctrl_map_configs *map_config;
	struct pinctrl_acpi_config_node *config_node;
	int fun_selector, grp_selector;
	int ret = 0;

	new_map = kzalloc(sizeof(struct pinctrl_map), GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

	switch (info->type) {
	case PINCTRL_ACPI_PIN_FUNCTION:
		fun_selector = info->function.function_number;
		if (fun_selector >= ARRAY_SIZE(sky1_gpio_functions)) {
			ret = -EINVAL;
			goto out_free;
		}
		new_map->type = PIN_MAP_TYPE_MUX_GROUP;
		new_map->data.mux.function = sky1_gpio_functions[fun_selector];
		/* Find matching group by pin */
		if (info->function.npins > 0) {
			struct sky1_pinctrl_group *grp;

			grp = sky1_pctrl_find_group_by_pin(spctl,
							   info->function.pins[0]);
			new_map->data.mux.group = grp ? grp->name : NULL;
		}
		break;
	case PINCTRL_ACPI_PIN_CONFIG:
		map_config = &new_map->data.configs;
		new_map->type = PIN_MAP_TYPE_CONFIGS_PIN;
		map_config->group_or_pin = pin_get_name(pctldev,
							info->config.pin);
		map_config->configs = devm_kcalloc(pctldev->dev,
						   info->config.nconfigs,
						   sizeof(unsigned long),
						   GFP_KERNEL);
		if (!map_config->configs) {
			ret = -ENOMEM;
			goto out_free;
		}
		map_config->num_configs = 0;
		list_for_each_entry(config_node, info->config.configs, node)
			map_config->configs[map_config->num_configs++] =
				config_node->config;
		break;
	case PINCTRL_ACPI_PIN_GRP_FUNCTION:
		fun_selector = info->function.function_number;
		grp_selector = info->function.group_number;
		if (fun_selector >= ARRAY_SIZE(sky1_gpio_functions) ||
		    grp_selector >= spctl->group_index) {
			ret = -EINVAL;
			goto out_free;
		}
		new_map->type = PIN_MAP_TYPE_MUX_GROUP;
		new_map->data.mux.function = sky1_gpio_functions[fun_selector];
		new_map->data.mux.group = spctl->grp_names[grp_selector];
		break;
	default:
		dev_warn(pctldev->dev, "Unsupported ACPI pin type %d\n",
			 info->type);
		ret = -EINVAL;
		goto out_free;
	}

	*map = new_map;
	*num_maps_out = 1;
	return 0;

out_free:
	kfree(new_map);
	return ret;
}

static void sky1_acpi_free_map(struct pinctrl_dev *pctldev,
			       struct pinctrl_map *map,
			       unsigned num_maps)
{
	kfree(map);
}

static int sky1_pctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct sky1_pinctrl *spctl = pinctrl_dev_get_drvdata(pctldev);

	return spctl->group_index;
}

static const char *sky1_pctrl_get_group_name(struct pinctrl_dev *pctldev,
					      unsigned int group)
{
	struct sky1_pinctrl *spctl = pinctrl_dev_get_drvdata(pctldev);

	return spctl->groups[group].name;
}

static int sky1_pctrl_get_group_pins(struct pinctrl_dev *pctldev,
				      unsigned int group,
				      const unsigned int **pins,
				      unsigned int *num_pins)
{
	struct sky1_pinctrl *spctl = pinctrl_dev_get_drvdata(pctldev);

	*pins = (unsigned int *)&spctl->groups[group].pin;
	*num_pins = 1;

	return 0;
}

static void sky1_pin_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
		   unsigned int offset)
{
	seq_printf(s, "%s", dev_name(pctldev->dev));
}

static const struct pinctrl_ops sky1_pctrl_ops = {
	.dt_node_to_map = sky1_pctrl_dt_node_to_map,
	.dt_free_map = sky1_dt_free_map,
	.acpi_node_to_map = sky1_acpi_node_to_map,
	.acpi_free_map = sky1_acpi_free_map,
	.get_groups_count = sky1_pctrl_get_groups_count,
	.get_group_name = sky1_pctrl_get_group_name,
	.get_group_pins = sky1_pctrl_get_group_pins,
	.pin_dbg_show = sky1_pin_dbg_show,
};

static int sky1_pmx_set_one_pin(struct sky1_pinctrl *spctl,
				    unsigned int pin, unsigned char muxval)
{
	u32 reg_val;
	void __iomem *pin_reg;

	pin_reg = spctl->base + pin * SKY1_PIN_SIZE;
	if (!has_acpi_companion(spctl->dev)) {
		reg_val = readl(pin_reg);
		reg_val &= ~SKY1_MUX_MASK;
		reg_val |= muxval << SKY1_MUX_SHIFT;
	} else {
		/* On ACPI, write the full mux+config value */
		reg_val = muxval << SKY1_MUX_SHIFT;
	}
	writel(reg_val, pin_reg);

	dev_dbg(spctl->dev, "write: offset 0x%x val 0x%x\n",
		pin * SKY1_PIN_SIZE, reg_val);
	return 0;
}

static int sky1_pmx_set_mux(struct pinctrl_dev *pctldev,
			    unsigned int function,
			    unsigned int group)
{
	bool ret;
	struct sky1_pinctrl *spctl = pinctrl_dev_get_drvdata(pctldev);
	struct sky1_pinctrl_group *g =
		(struct sky1_pinctrl_group *)spctl->groups + group;

	ret = sky1_pctrl_is_function_valid(spctl, g->pin, function);
	if (!ret) {
		dev_err(spctl->dev, "invalid function %d on group %d .\n",
				function, group);
		return -EINVAL;
	}

	sky1_pmx_set_one_pin(spctl, g->pin, function);
	return 0;
}

static int sky1_pmx_get_funcs_cnt(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(sky1_gpio_functions);
}

static const char *sky1_pmx_get_func_name(struct pinctrl_dev *pctldev,
					   unsigned int selector)
{
	return sky1_gpio_functions[selector];
}

static int sky1_pmx_get_func_groups(struct pinctrl_dev *pctldev,
				     unsigned int function,
				     const char * const **groups,
				     unsigned int * const num_groups)
{
	struct sky1_pinctrl *spctl = pinctrl_dev_get_drvdata(pctldev);

	*groups = spctl->grp_names;
	*num_groups = spctl->group_index;

	return 0;
}

static const struct pinmux_ops sky1_pmx_ops = {
	.get_functions_count = sky1_pmx_get_funcs_cnt,
	.get_function_groups = sky1_pmx_get_func_groups,
	.get_function_name = sky1_pmx_get_func_name,
	.set_mux = sky1_pmx_set_mux,
};

static int sky1_pconf_set_pull_select(struct sky1_pinctrl *spctl,
		unsigned int pin, bool enable, bool isup)
{
	u32 reg_val, reg_pullsel = 0;
	void __iomem *pin_reg;

	pin_reg = spctl->base + pin * SKY1_PIN_SIZE;
	reg_val = readl(pin_reg);
	reg_val &= ~SKY1_PULLCONF_MASK;

	if (!enable)
		goto update;

	if (isup)
		reg_pullsel = BIT(SKY1_PULLUP_BIT);
	else
		reg_pullsel = BIT(SKY1_PULLDN_BIT);

update:
	reg_val |= reg_pullsel;
	writel(reg_val, pin_reg);

	dev_dbg(spctl->dev, "write: offset 0x%x val 0x%x\n",
		pin * SKY1_PIN_SIZE, reg_val);
	return 0;
}

static int sky1_ds_to_index(unsigned char driving)
{
	int i;

	for (i = 0; i < sizeof(sky1_ds_table); i++)
		if (driving == sky1_ds_table[i])
			return i;
	return SKY1_DEFAULT_DS_VAL;
}

static int sky1_pconf_set_driving(struct sky1_pinctrl *spctl,
		unsigned int pin, unsigned char driving)
{
	unsigned int reg_val, val;
	void __iomem *pin_reg;

	if (pin >= spctl->info->npins)
		return -EINVAL;

	pin_reg = spctl->base + pin * SKY1_PIN_SIZE;
	reg_val = readl(pin_reg);
	reg_val &= ~SKY1_DS_MASK;
	val = sky1_ds_to_index(driving);
	reg_val |= (val & SKY1_DS_MASK);
	writel(reg_val, pin_reg);

	dev_dbg(spctl->dev, "write: offset 0x%x val 0x%x\n",
		pin * SKY1_PIN_SIZE, reg_val);

	return 0;
}

static int sky1_pconf_parse_conf(struct pinctrl_dev *pctldev,
		unsigned int pin, enum pin_config_param param,
		enum pin_config_param arg)
{
	int ret = 0;
	struct sky1_pinctrl *spctl = pinctrl_dev_get_drvdata(pctldev);

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		ret = sky1_pconf_set_pull_select(spctl, pin, false, false);
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		ret = sky1_pconf_set_pull_select(spctl, pin, true, true);
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		ret = sky1_pconf_set_pull_select(spctl, pin, true, false);
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		ret = sky1_pconf_set_driving(spctl, pin, arg);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int sky1_pconf_group_get(struct pinctrl_dev *pctldev,
				 unsigned int group,
				 unsigned long *config)
{
	struct sky1_pinctrl *spctl = pinctrl_dev_get_drvdata(pctldev);
	struct sky1_pinctrl_group *g = &spctl->groups[group];

	*config = g->config;

	return 0;
}

static int sky1_pconf_group_set(struct pinctrl_dev *pctldev, unsigned int group,
				 unsigned long *configs, unsigned int num_configs)
{
	struct sky1_pinctrl *spctl = pinctrl_dev_get_drvdata(pctldev);
	struct sky1_pinctrl_group *g = &spctl->groups[group];
	int i, ret;

	for (i = 0; i < num_configs; i++) {
		ret = sky1_pconf_parse_conf(pctldev, g->pin,
			pinconf_to_config_param(configs[i]),
			pinconf_to_config_argument(configs[i]));
		if (ret < 0)
			return ret;

		g->config = configs[i];
	}

	return 0;
}

static const struct pinconf_ops sky1_pinconf_ops = {
	.pin_config_group_get	= sky1_pconf_group_get,
	.pin_config_group_set	= sky1_pconf_group_set,
};

static int sky1_pctrl_build_state(struct platform_device *pdev)
{
	struct sky1_pinctrl *spctl = platform_get_drvdata(pdev);
	const struct sky1_pinctrl_soc_info *info = spctl->info;
	int i;

	/* Allocate groups */
	spctl->groups = devm_kcalloc(&pdev->dev, info->npins,
				    sizeof(*spctl->groups), GFP_KERNEL);
	if (!spctl->groups)
		return -ENOMEM;

	/* We assume that one pin is one group, use pin name as group name. */
	spctl->grp_names = devm_kcalloc(&pdev->dev, info->npins,
				       sizeof(*spctl->grp_names), GFP_KERNEL);
	if (!spctl->grp_names)
		return -ENOMEM;

	for (i = 0; i < info->npins; i++) {
		const struct sky1_pin_desc *pin = spctl->info->pins + i;
		struct sky1_pinctrl_group *group =
			(struct sky1_pinctrl_group *)spctl->groups + i;

		group->name = pin->pin.name;
		group->pin = pin->pin.number;
		spctl->grp_names[i] = pin->pin.name;
	}

	spctl->group_index = info->npins;
	return 0;
}

static void
sky1_free_acpi_group_desc(struct pinctrl_acpi_group_desc *acpi_grp_desc)
{
	if (!acpi_grp_desc)
		return;

	kfree(acpi_grp_desc->vendor_data);
	kfree(acpi_grp_desc->pins);
	kfree(acpi_grp_desc->name);
	kfree(acpi_grp_desc);
}

static int sky1_pinctrl_probe_acpi(struct platform_device *pdev,
				   struct sky1_pinctrl *spctl)
{
	struct pinctrl_acpi_group_desc *acpi_grp, *temp;
	struct acpi_device *acpi_dev = ACPI_COMPANION(&pdev->dev);
	struct list_head pinctrl_group_list;
	int ret, num_acpi_groups = 0, i = 0;
	struct sky1_pinctrl_group *new_groups;
	const char **new_grp_names;
	int base_groups = spctl->info->npins;

	if (!acpi_dev)
		return 0;

	INIT_LIST_HEAD(&pinctrl_group_list);
	ret = pinctrl_acpi_get_pin_groups(acpi_dev, &pinctrl_group_list);
	if (ret)
		return ret;

	list_for_each_entry(acpi_grp, &pinctrl_group_list, list)
		num_acpi_groups++;

	if (!num_acpi_groups)
		return 0;

	/* Extend groups and grp_names arrays to include ACPI groups */
	new_groups = devm_kcalloc(&pdev->dev, base_groups + num_acpi_groups,
				  sizeof(*new_groups), GFP_KERNEL);
	new_grp_names = devm_kcalloc(&pdev->dev, base_groups + num_acpi_groups,
				     sizeof(*new_grp_names), GFP_KERNEL);
	if (!new_groups || !new_grp_names)
		return -ENOMEM;

	/* Copy existing per-pin groups */
	memcpy(new_groups, spctl->groups,
	       base_groups * sizeof(*new_groups));
	memcpy(new_grp_names, spctl->grp_names,
	       base_groups * sizeof(*new_grp_names));

	/* Parse ACPI groups and append them */
	i = 0;
	list_for_each_entry_safe(acpi_grp, temp, &pinctrl_group_list, list) {
		struct sky1_pinctrl_group *grp = &new_groups[base_groups + i];

		grp->name = devm_kstrdup(&pdev->dev, acpi_grp->name,
					 GFP_KERNEL);
		if (acpi_grp->num_pins > 0) {
			grp->pin = acpi_grp->pins[0];
			/* Store full config from vendor data if available */
			if (acpi_grp->vendor_data) {
				__be16 *vendor = (__be16 *)acpi_grp->vendor_data;
				unsigned int offset = be16_to_cpu(vendor[0]);
				unsigned int config = be16_to_cpu(vendor[1]);

				spctl->pin_regs[grp->pin] = offset;
				grp->config = config;
			}
		}
		new_grp_names[base_groups + i] = grp->name;

		dev_dbg(&pdev->dev, "ACPI group[%d]: %s pin=%d\n",
			base_groups + i, grp->name, grp->pin);

		list_del(&acpi_grp->list);
		sky1_free_acpi_group_desc(acpi_grp);
		i++;
	}

	spctl->groups = new_groups;
	spctl->grp_names = new_grp_names;
	spctl->group_index += num_acpi_groups;

	return 0;
}

int sky1_base_pinctrl_probe(struct platform_device *pdev,
		      const struct sky1_pinctrl_soc_info *info)
{
	struct pinctrl_desc *sky1_pinctrl_desc;
	struct sky1_pinctrl *spctl;
	struct pinctrl_pin_desc *pins;
	int ret, i;

	if (!info || !info->pins || !info->npins) {
		dev_err(&pdev->dev, "wrong pinctrl info\n");
		return -EINVAL;
	}

	/* Create state holders etc for this driver */
	spctl = devm_kzalloc(&pdev->dev, sizeof(*spctl), GFP_KERNEL);
	if (!spctl)
		return -ENOMEM;

	spctl->info = info;
	platform_set_drvdata(pdev, spctl);

	spctl->pin_regs = devm_kmalloc_array(&pdev->dev, info->npins,
					     sizeof(*spctl->pin_regs),
					     GFP_KERNEL);
	if (!spctl->pin_regs)
		return -ENOMEM;
	for (i = 0; i < info->npins; i++)
		spctl->pin_regs[i] = -1;

	spctl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spctl->base))
		return PTR_ERR(spctl->base);

	sky1_pinctrl_desc = devm_kzalloc(&pdev->dev, sizeof(*sky1_pinctrl_desc),
					GFP_KERNEL);
	if (!sky1_pinctrl_desc)
		return -ENOMEM;

	pins = devm_kcalloc(&pdev->dev, info->npins, sizeof(*pins),
			    GFP_KERNEL);
	if (!pins)
		return -ENOMEM;
	for (i = 0; i < info->npins; i++)
		pins[i] = info->pins[i].pin;

	ret = sky1_pctrl_build_state(pdev);
	if (ret)
		return ret;

	sky1_pinctrl_desc->name = dev_name(&pdev->dev);
	sky1_pinctrl_desc->pins = pins;
	sky1_pinctrl_desc->npins = info->npins;
	sky1_pinctrl_desc->pctlops = &sky1_pctrl_ops;
	sky1_pinctrl_desc->pmxops = &sky1_pmx_ops;
	sky1_pinctrl_desc->confops = &sky1_pinconf_ops;
	sky1_pinctrl_desc->owner = THIS_MODULE;
	spctl->dev = &pdev->dev;
	ret = devm_pinctrl_register_and_init(&pdev->dev,
					     sky1_pinctrl_desc, spctl,
					     &spctl->pctl);
	if (ret) {
		dev_err(&pdev->dev, "could not register SKY1 pinctrl driver\n");
		return ret;
	}

	/* Parse ACPI pin groups if running on ACPI */
	if (has_acpi_companion(&pdev->dev)) {
		ret = sky1_pinctrl_probe_acpi(pdev, spctl);
		if (ret)
			dev_warn(&pdev->dev,
				 "ACPI pin group parsing failed: %d\n", ret);
	}

	pinctrl_provide_dummies();

	dev_dbg(&pdev->dev, "initialized SKY1 pinctrl driver\n");

	return pinctrl_enable(spctl->pctl);
}
EXPORT_SYMBOL_GPL(sky1_base_pinctrl_probe);


MODULE_AUTHOR("Jerry Zhu <Jerry.Zhu@cixtech.com>");
MODULE_DESCRIPTION("Cix SKy1 pinctrl base driver");
MODULE_LICENSE("GPL");
