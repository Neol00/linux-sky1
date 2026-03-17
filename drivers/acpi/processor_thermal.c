// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * processor_thermal.c - Passive cooling submodule of the ACPI processor driver
 *
 *  Copyright (C) 2001, 2002 Andy Grover <andrew.grover@intel.com>
 *  Copyright (C) 2001, 2002 Paul Diefenbaugh <paul.s.diefenbaugh@intel.com>
 *  Copyright (C) 2004       Dominik Brodowski <linux@brodo.de>
 *  Copyright (C) 2004  Anil S Keshavamurthy <anil.s.keshavamurthy@intel.com>
 *  			- Added processor hotplug support
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/acpi.h>
#include <linux/energy_model.h>
#include <acpi/processor.h>
#include <acpi/cppc_acpi.h>
#include <linux/uaccess.h>

#include "internal.h"

#ifdef CONFIG_CPU_FREQ

/* If a passive cooling situation is detected, primarily CPUfreq is used, as it
 * offers (in most cases) voltage scaling in addition to frequency scaling, and
 * thus a cubic (instead of linear) reduction of energy. Also, we allow for
 * _any_ cpufreq driver and not only the acpi-cpufreq driver.
 */

#define CPUFREQ_THERMAL_MIN_STEP 0

static int cpufreq_thermal_max_step __read_mostly = 3;

/*
 * Minimum throttle percentage for processor_thermal cooling device.
 * The processor_thermal driver uses it to calculate the percentage amount by
 * which cpu frequency must be reduced for each cooling state. This is also used
 * to calculate the maximum number of throttling steps or cooling states.
 */
static int cpufreq_thermal_reduction_pctg __read_mostly = 20;

static DEFINE_PER_CPU(unsigned int, cpufreq_thermal_reduction_step);

#define reduction_step(cpu) \
	per_cpu(cpufreq_thermal_reduction_step, phys_package_first_cpu(cpu))

/*
 * Emulate "per package data" using per cpu data (which should really be
 * provided elsewhere)
 *
 * Note we can lose a CPU on cpu hotunplug, in this case we forget the state
 * temporarily. Fortunately that's not a big issue here (I hope)
 */
static int phys_package_first_cpu(int cpu)
{
	int i;
	int id = topology_physical_package_id(cpu);

	for_each_online_cpu(i)
		if (topology_physical_package_id(i) == id)
			return i;
	return 0;
}

static bool cpu_has_cpufreq(unsigned int cpu)
{
	if (!acpi_processor_cpufreq_init)
		return 0;

	struct cpufreq_policy *policy __free(put_cpufreq_policy) = cpufreq_cpu_get(cpu);

	return policy != NULL;
}

static int cpufreq_get_max_state(unsigned int cpu)
{
	if (!cpu_has_cpufreq(cpu))
		return 0;

	return cpufreq_thermal_max_step;
}

static int cpufreq_get_cur_state(unsigned int cpu)
{
	if (!cpu_has_cpufreq(cpu))
		return 0;

	return reduction_step(cpu);
}

static bool cpufreq_update_thermal_limit(unsigned int cpu, struct acpi_processor *pr)
{
	unsigned long max_freq;
	int ret;

	struct cpufreq_policy *policy __free(put_cpufreq_policy) = cpufreq_cpu_get(cpu);
	if (!policy)
		return false;

	max_freq = (policy->cpuinfo.max_freq *
		(100 - reduction_step(cpu) * cpufreq_thermal_reduction_pctg)) / 100;

	ret = freq_qos_update_request(&pr->thermal_req, max_freq);
	if (ret < 0) {
		pr_warn("Failed to update thermal freq constraint: CPU%d (%d)\n",
			pr->id, ret);
	}

	return true;
}

static int cpufreq_set_cur_state(unsigned int cpu, int state)
{
	struct acpi_processor *pr;
	int i;

	if (!cpu_has_cpufreq(cpu))
		return 0;

	reduction_step(cpu) = state;

	/*
	 * Update all the CPUs in the same package because they all
	 * contribute to the temperature and often share the same
	 * frequency.
	 */
	for_each_online_cpu(i) {
		if (topology_physical_package_id(i) !=
		    topology_physical_package_id(cpu))
			continue;

		pr = per_cpu(processors, i);

		if (unlikely(!freq_qos_request_active(&pr->thermal_req)))
			continue;

		if (!cpufreq_update_thermal_limit(i, pr))
			return -EINVAL;
	}
	return 0;
}

static void acpi_thermal_cpufreq_config(void)
{
	int cpufreq_pctg = acpi_arch_thermal_cpufreq_pctg();

	if (!cpufreq_pctg)
		return;

	cpufreq_thermal_reduction_pctg = cpufreq_pctg;

	/*
	 * Derive the MAX_STEP from minimum throttle percentage so that the reduction
	 * percentage doesn't end up becoming negative. Also, cap the MAX_STEP so that
	 * the CPU performance doesn't become 0.
	 */
	cpufreq_thermal_max_step = (100 / cpufreq_pctg) - 2;
}

void acpi_thermal_cpufreq_init(struct cpufreq_policy *policy)
{
	unsigned int cpu;

	acpi_thermal_cpufreq_config();

	for_each_cpu(cpu, policy->related_cpus) {
		struct acpi_processor *pr = per_cpu(processors, cpu);
		int ret;

		if (!pr)
			continue;

		ret = freq_qos_add_request(&policy->constraints,
					   &pr->thermal_req,
					   FREQ_QOS_MAX, INT_MAX);
		if (ret < 0) {
			pr_err("Failed to add freq constraint for CPU%d (%d)\n",
			       cpu, ret);
			continue;
		}

		thermal_cooling_device_update(pr->cdev);
	}
}

void acpi_thermal_cpufreq_exit(struct cpufreq_policy *policy)
{
	unsigned int cpu;

	for_each_cpu(cpu, policy->related_cpus) {
		struct acpi_processor *pr = per_cpu(processors, cpu);

		if (!pr)
			continue;

		freq_qos_remove_request(&pr->thermal_req);

		thermal_cooling_device_update(pr->cdev);
	}
}
#else				/* ! CONFIG_CPU_FREQ */
static int cpufreq_get_max_state(unsigned int cpu)
{
	return 0;
}

static int cpufreq_get_cur_state(unsigned int cpu)
{
	return 0;
}

static int cpufreq_set_cur_state(unsigned int cpu, int state)
{
	return 0;
}

#endif

/* thermal cooling device callbacks */
static int acpi_processor_max_state(struct acpi_processor *pr)
{
	int max_state = 0;

	/*
	 * There exists four states according to
	 * cpufreq_thermal_reduction_step. 0, 1, 2, 3
	 */
	max_state += cpufreq_get_max_state(pr->id);
	if (pr->flags.throttling)
		max_state += (pr->throttling.state_count -1);

	return max_state;
}
static int
processor_get_max_state(struct thermal_cooling_device *cdev,
			unsigned long *state)
{
	struct acpi_device *device = cdev->devdata;
	struct acpi_processor *pr;

	if (!device)
		return -EINVAL;

	pr = acpi_driver_data(device);
	if (!pr)
		return -EINVAL;

	*state = acpi_processor_max_state(pr);
	return 0;
}

static int
processor_get_cur_state(struct thermal_cooling_device *cdev,
			unsigned long *cur_state)
{
	struct acpi_device *device = cdev->devdata;
	struct acpi_processor *pr;

	if (!device)
		return -EINVAL;

	pr = acpi_driver_data(device);
	if (!pr)
		return -EINVAL;

	*cur_state = cpufreq_get_cur_state(pr->id);
	if (pr->flags.throttling)
		*cur_state += pr->throttling.state;
	return 0;
}

static int
processor_set_cur_state(struct thermal_cooling_device *cdev,
			unsigned long state)
{
	struct acpi_device *device = cdev->devdata;
	struct acpi_processor *pr;
	int result = 0;
	int max_pstate;

	if (!device)
		return -EINVAL;

	pr = acpi_driver_data(device);
	if (!pr)
		return -EINVAL;

	max_pstate = cpufreq_get_max_state(pr->id);

	if (state > acpi_processor_max_state(pr))
		return -EINVAL;

	if (state <= max_pstate) {
		if (pr->flags.throttling && pr->throttling.state)
			result = acpi_processor_set_throttling(pr, 0, false);
		cpufreq_set_cur_state(pr->id, state);
	} else {
		cpufreq_set_cur_state(pr->id, max_pstate);
		result = acpi_processor_set_throttling(pr,
				state - max_pstate, false);
	}
	return result;
}

static int processor_get_requested_power(struct thermal_cooling_device *cdev,
					 u32 *power)
{
	struct acpi_device *device = cdev->devdata;
	struct cpufreq_policy *policy;
	struct acpi_processor *pr;

	if (!device)
		return -EINVAL;

	pr = acpi_driver_data(device);
	if (!pr)
		return -EINVAL;
	if(pr->id >= num_possible_cpus()) {
		pr_err("Invalid CPU device or CPU ID %d\n", pr->id);
		return -EINVAL;
	}

	policy = cpufreq_cpu_get(pr->id);
	if (!policy)
		return -EINVAL;
#ifdef CONFIG_CIX_THERMAL
	*power = processor_get_static_power_cpus(policy->cpus) +
		 processor_get_dynamic_power_cpus(policy->cpus);
#else
	*power = 0;
#endif
	cpufreq_cpu_put(policy);

	return 0;
}

/**
 * processor_state2power() - convert a cpu cdev state to power consumed
 * @cdev:	&thermal_cooling_device pointer
 * @state:	cooling device state to be converted
 * @power:	pointer in which to store the resulting power
 *
 * Convert cooling device state @state into power consumption in
 * milliwatts assuming 100% load.  Store the calculated power in
 * @power.
 *
 * Return: 0 on success, -EINVAL if the cooling device state is bigger
 * than maximum allowed.
 */
static int processor_state2power(struct thermal_cooling_device *cdev,
				 unsigned long state, u32 *power)
{
	unsigned int freq, opp_power, num_cpus, idx;
	struct cpufreq_policy *policy = NULL;
	struct cppc_cpudata *cpu_data;
	struct em_perf_domain *em;
	struct acpi_device *device = cdev->devdata;
	struct acpi_processor *pr;
	int ret = 0;

	if (!device)
		return -EINVAL;

	pr = acpi_driver_data(device);
	if (!pr)
		return -EINVAL;
	if(pr->id >= num_possible_cpus()) {
		pr_err("Invalid CPU device or CPU ID %d\n", pr->id);
		return -EINVAL;
	}

	policy = cpufreq_cpu_get(pr->id);
	if (!policy)
		return -EINVAL;
	cpu_data = policy->driver_data;
	if (!cpu_data) {
		pr_warn("No CPU data for CPU%d\n", policy->cpu);
		ret = -ENODEV;
		goto EXIT;
	}

	/* Request state should be less than max_level */
	if (state > cpu_data->opp_level_num) {
		pr_err("Invalid state %lu for cooling device %s\n",
		       state, cdev->type);
		ret = -EINVAL;
		goto EXIT;
	}

	em = em_cpu_get(policy->cpu);
	if (!em) {
		pr_warn("No energy model for CPU%d\n", policy->cpu);
		ret = -ENODEV;
		goto EXIT;
	}
	rcu_read_lock();
	{
		struct em_perf_state *table = em_perf_state_from_pd(em);
		num_cpus = cpumask_weight(policy->cpus);
		idx = cpu_data->opp_level_num - state - 1;
		freq = table[idx].frequency;
		opp_power = table[idx].power;
	}
	rcu_read_unlock();
	*power = opp_power * num_cpus;
#ifdef CONFIG_CIX_THERMAL
	*power += processor_get_static_power_cpus(policy->cpus);
#endif

EXIT:
	cpufreq_cpu_put(policy);
	return ret;
}

/**
 * processor_power2state() - convert power to a cooling device state
 * @cdev:	&thermal_cooling_device pointer
 * @power:	power in milliwatts to be converted
 * @state:	pointer in which to store the resulting state
 *
 * Calculate a cooling device state for the cpus described by @cdev
 * that would allow them to consume at most @power mW and store it in
 * @state.  Note that this calculation depends on external factors
 * such as the CPUs load.  Calling this function with the same power
 * as input can yield different cooling device states depending on those
 * external factors.
 *
 * Return: 0 on success, this function doesn't fail.
 */
static int processor_power2state(struct thermal_cooling_device *cdev,
				 u32 power, unsigned long *state)
{
	struct cpufreq_policy *policy;
	struct cppc_cpudata *cpu_data;
	struct em_perf_domain *em;
	unsigned int target_freq;
	u32 normalised_power;
	u32 em_power_mw;
	u32 static_power;
	int i;

	struct acpi_device *device = cdev->devdata;
	struct acpi_processor *pr;

	if (!device)
		return -EINVAL;

	pr = acpi_driver_data(device);
	if (!pr)
		return -EINVAL;

	policy = cpufreq_cpu_get(pr->id);
	if (!policy)
		return -EINVAL;
	cpu_data = policy->driver_data;
	if (!cpu_data) {
		pr_warn("No CPU data for CPU%d\n", policy->cpu);
		cpufreq_cpu_put(policy);
		return -ENODEV;
	}

#ifdef CONFIG_CIX_THERMAL
	static_power = processor_get_static_power_cpus(policy->cpus);
#else
	static_power = 0;
#endif
	if (power > static_power)
		normalised_power = power - static_power;
	else
		normalised_power = 0;

	em = em_cpu_get(policy->cpu);
	if (!em) {
		pr_warn("No energy model for CPU%d\n", policy->cpu);
		cpufreq_cpu_put(policy);
		return -ENODEV;
	}
	cpufreq_cpu_put(policy);

	rcu_read_lock();
	{
		struct em_perf_state *table = em_perf_state_from_pd(em);

		for (i = cpu_data->opp_level_num - 1; i >= 0; i--) {
			em_power_mw = table[i].power;
			if (normalised_power >= em_power_mw)
				break;
		}
		if (i < 0) {
			pr_warn("No level found for power %u\n", power);
			*state = cpu_data->opp_level_num - 1;
			rcu_read_unlock();
			return 0;
		}
		target_freq = table[i].frequency;
	}
	rcu_read_unlock();

	*state = cpu_data->opp_level_num - i - 1;
	return 0;
}


const struct thermal_cooling_device_ops processor_cooling_ops = {
	.get_max_state = processor_get_max_state,
	.get_cur_state = processor_get_cur_state,
	.set_cur_state = processor_set_cur_state,
	.get_requested_power = processor_get_requested_power,
	.power2state = processor_power2state,
	.state2power = processor_state2power,
};

int acpi_processor_thermal_init(struct acpi_processor *pr,
				struct acpi_device *device)
{
	int result = 0;

	pr->cdev = thermal_cooling_device_register("Processor", device,
						   &processor_cooling_ops);
	if (IS_ERR(pr->cdev)) {
		result = PTR_ERR(pr->cdev);
		return result;
	}

	dev_dbg(&device->dev, "registered as cooling_device%d\n",
		pr->cdev->id);

	result = sysfs_create_link(&device->dev.kobj,
				   &pr->cdev->device.kobj,
				   "thermal_cooling");
	if (result) {
		dev_err(&device->dev,
			"Failed to create sysfs link 'thermal_cooling'\n");
		goto err_thermal_unregister;
	}

	result = sysfs_create_link(&pr->cdev->device.kobj,
				   &device->dev.kobj,
				   "device");
	if (result) {
		dev_err(&pr->cdev->device,
			"Failed to create sysfs link 'device'\n");
		goto err_remove_sysfs_thermal;
	}

	return 0;

err_remove_sysfs_thermal:
	sysfs_remove_link(&device->dev.kobj, "thermal_cooling");
err_thermal_unregister:
	thermal_cooling_device_unregister(pr->cdev);

	return result;
}

void acpi_processor_thermal_exit(struct acpi_processor *pr,
				 struct acpi_device *device)
{
	if (pr->cdev) {
		sysfs_remove_link(&device->dev.kobj, "thermal_cooling");
		sysfs_remove_link(&pr->cdev->device.kobj, "device");
		thermal_cooling_device_unregister(pr->cdev);
		pr->cdev = NULL;
	}
}
