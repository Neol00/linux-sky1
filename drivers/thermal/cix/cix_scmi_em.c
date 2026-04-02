// SPDX-License-Identifier: GPL-2.0-only
/*
 * SCMI based Cix Energy Model driver
 *
 * Copyright 2024 Cix Technology Group Co., Ltd.All Rights Reserved.
 */

#include <linux/energy_model.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/acpi.h>
#include <linux/pm_opp.h>
#include <linux/property.h>
#include <linux/scmi_protocol.h>
#include <linux/units.h>
#include <linux/cix/cix_scmi_em.h>

static struct scmi_protocol_handle *ph;
static const struct scmi_perf_proto_ops *perf_ops;

/* ACPI fallback: runtime-configured perf domain ID (set by GPU driver) */
static atomic_t acpi_perf_domain_id = ATOMIC_INIT(-1);

void cix_scmi_set_acpi_perf_domain_id(int domain_id)
{
	atomic_set(&acpi_perf_domain_id, domain_id);
}
EXPORT_SYMBOL_GPL(cix_scmi_set_acpi_perf_domain_id);

static int cix_scmi_get_domain_id(struct device *dev)
{
	struct device_node *np = dev->of_node;

	/* DT path */
	if (np) {
		struct of_phandle_args domain_id;
		int index;

		if (of_parse_phandle_with_args(np, "clocks", "#clock-cells", 0,
					       &domain_id)) {
			index = of_property_match_string(np, "power-domain-names",
							 "perf");
			if (index < 0)
				goto try_acpi;

			if (of_parse_phandle_with_args(np, "power-domains",
						       "#power-domain-cells", index,
						       &domain_id))
				goto try_acpi;
		}

		of_node_put(domain_id.np);
		return domain_id.args[0];
	}

try_acpi:
	/* ACPI path: check BIOS _DSD property first */
	if (has_acpi_companion(dev)) {
		u32 id;

		if (!device_property_read_u32(dev, "scmi-perf-domain-id", &id))
			return id;
	}

	/* Fallback: runtime-configured domain ID from driver */
	if (atomic_read(&acpi_perf_domain_id) >= 0)
		return atomic_read(&acpi_perf_domain_id);

	return -EINVAL;
}

static int __maybe_unused
cix_scmi_get_em_power(struct device *dev, unsigned long *power,
		   unsigned long *KHz)
{
	enum scmi_power_scale power_scale;
	unsigned long Hz;
	int ret, domain;

	if (!perf_ops || !ph)
		return -EPROBE_DEFER;

	power_scale = perf_ops->power_scale_get(ph);

	domain = cix_scmi_get_domain_id(dev);
	if (domain < 0)
		return domain;

	/* Get the power from SCMI performance domain. */
	Hz = *KHz * 1000;
	ret = perf_ops->est_power_get(ph, domain, &Hz, power);
	if (ret)
		return ret;

	if (power_scale == SCMI_POWER_MILLIWATTS)
		*power *= MICROWATT_PER_MILLIWATT;

	*KHz = Hz / 1000;

	return 0;
}

int cix_scmi_register_em(struct device *dev)
{
	struct em_data_callback em_cb = EM_DATA_CB(cix_scmi_get_em_power);
	enum scmi_power_scale power_scale;
	bool em_power_scale = false;
	int ret, nr_opp;

	if (!perf_ops || !ph) {
		dev_dbg(dev, "SCMI EM not initialized yet\n");
		return -EPROBE_DEFER;
	}

	power_scale = perf_ops->power_scale_get(ph);

	if (power_scale == SCMI_POWER_MILLIWATTS
	    || power_scale == SCMI_POWER_MICROWATTS)
		em_power_scale = true;

	nr_opp = dev_pm_opp_get_opp_count(dev);
	if (nr_opp <= 0) {
		dev_err(dev, "Failed to get OPP counts\n");
		return -EINVAL;
	}

	ret = em_dev_register_perf_domain(dev, nr_opp, &em_cb, NULL, em_power_scale);
	if (ret) {
		dev_dbg(dev, "Couldn't register Energy Model %d\n", ret);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(cix_scmi_register_em);

static int cix_scmi_em_probe(struct scmi_device *sdev)
{
	const struct scmi_handle *handle;

	handle = sdev->handle;

	if (!handle)
		return -ENODEV;

	perf_ops = handle->devm_protocol_get(sdev, SCMI_PROTOCOL_PERF, &ph);
	if (IS_ERR(perf_ops) || !ph)
		return IS_ERR(perf_ops) ? PTR_ERR(perf_ops) : -EPROTO;

	return 0;
}

static void cix_scmi_em_remove(struct scmi_device *sdev)
{
	/* Nothing need to be done now */
}

static const struct scmi_device_id scmi_id_table[] = {
	{ SCMI_PROTOCOL_PERF, "cix_em_perf" },
	{ },
};
MODULE_DEVICE_TABLE(scmi, scmi_id_table);

static struct scmi_driver cix_scmi_em_drv = {
	.name		= "cix-scmi-em",
	.probe		= cix_scmi_em_probe,
	.remove		= cix_scmi_em_remove,
	.id_table	= scmi_id_table,
};
module_scmi_driver(cix_scmi_em_drv);

MODULE_AUTHOR("Cixtech,Inc.");
MODULE_DESCRIPTION("CIX SCMI Energy Model interface driver");
MODULE_LICENSE("GPL v2");
