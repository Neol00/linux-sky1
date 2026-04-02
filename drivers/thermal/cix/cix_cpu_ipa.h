/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CIX_CPU_IPA_H
#define _CIX_CPU_IPA_H

#include <linux/cpumask.h>

int cix_get_static_power_cpus(cpumask_var_t cpus);
int cix_get_dynamic_power_cpus(cpumask_var_t cpus);

#endif /* _CIX_CPU_IPA_H */
