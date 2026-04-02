// SPDX-License-Identifier: GPL-2.0
/* Copyright 2024 Cix Technology Group Co., Ltd.*/
/**
 * Cix Energy Model driver
 */

#ifndef __CIX_SCMI_EM_H
#define __CIX_SCMI_EM_H

#if IS_ENABLED(CONFIG_CIX_SCMI_ENERGY_MODEL)
/**
 * cix_scmi_register_em() - Register an energy model using
 *              SCMI Message Protocol
 *
 * @dev: Device for which the EM is to register
 *
 * Return: 0 for success; else the error code is returned
 */
int cix_scmi_register_em(struct device *dev);

/**
 * cix_scmi_set_acpi_perf_domain_id() - Set SCMI perf domain ID for ACPI path
 *
 * @domain_id: The SCMI performance domain ID
 *
 * Under ACPI, the DT-based domain ID lookup is unavailable.  Call this
 * before cix_scmi_register_em() to provide the domain ID at runtime.
 * Alternatively, add "scmi-perf-domain-id" to the device's ACPI _DSD.
 */
void cix_scmi_set_acpi_perf_domain_id(int domain_id);

#else /* !CONFIG_CIX_SCMI_ENERGY_MODEL */

static inline int cix_scmi_register_em(struct device *dev)
{
	return -EINVAL;
}

static inline void cix_scmi_set_acpi_perf_domain_id(int domain_id) {}

#endif /* CONFIG_CIX_SCMI_ENERGY_MODEL */

#endif /* __CIX_SCMI_EM_H */