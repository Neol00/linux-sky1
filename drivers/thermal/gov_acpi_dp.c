// SPDX-License-Identifier: GPL-2.0
// Copyright 2024 Cix Technology Group Co., Ltd.
/*
 * gov_acpi_dp.c - A Thermal processor throttling governor for acpi thermal.
 *
 */
#include <linux/thermal.h>
#include <linux/minmax.h>
#include <linux/units.h>

#include "thermal_core.h"
#include "../acpi/thermal.h"

#define MAX_PERFORMANCE 100
#define MIN_PERFORMANCE 0

static int get_temp(struct thermal_zone_device *thermal, int *pre_temp, int *cur_temp)
{
	struct acpi_thermal *tz = thermal->devdata;
	int ret;

	*pre_temp = deci_kelvin_to_millicelsius_with_offset(tz->temperature,
			tz->kelvin_offset);

	ret = thermal_zone_get_temp(thermal, cur_temp);
	if (ret)
		return ret;

	*pre_temp /= MILLIDEGREE_PER_DEGREE;
	*cur_temp /= MILLIDEGREE_PER_DEGREE;

	return 0;
}

static int get_para(struct thermal_zone_device *thermal, int *tc1, int *tc2)
{
	struct acpi_thermal *tz = thermal->devdata;

	*tc1 = tz->trips.passive.tc1;
	*tc2 = tz->trips.passive.tc2;
	if (!tc1 || !tc2)
		return -EINVAL;

	return 0;
}

static int get_delta_performance(struct thermal_zone_device *tz,
				 struct thermal_instance *instance, int *delta_p)
{
	int ret;
	unsigned int tc1, tc2;
	int cur_temp, pre_temp, trip_temp;
	const struct thermal_trip *trip;

	trip = instance->trip;

	ret = get_para(tz, &tc1, &tc2);
	if (ret)
		return ret;

	ret = get_temp(tz, &pre_temp, &cur_temp);
	if (ret)
		return ret;

	trip_temp = trip->temperature / MILLIDEGREE_PER_DEGREE;

	*delta_p = tc1 * (cur_temp - pre_temp) + tc2 * (cur_temp - trip_temp);

	return 0;
}

static int get_processor_performance(struct thermal_instance *instance, int *cur_p)
{
	int ret;
	unsigned long cur_state;
	struct thermal_cooling_device *cdev;

	cdev = instance->cdev;

	ret = cdev->ops->get_cur_state(cdev, &cur_state);
	if (ret)
		return ret;

	*cur_p = MAX_PERFORMANCE - cur_state;

	return 0;
}

static int get_target_state(struct thermal_zone_device *tz,
			    struct thermal_instance *instance)
{
	int ret;
	unsigned long next_state;
	int cur_p;
	int delta_p;
	int performance;

	ret = get_delta_performance(tz, instance, &delta_p);
	if (ret)
		return ret;

	ret = get_processor_performance(instance, &cur_p);
	if (ret)
		return ret;

	performance = clamp((cur_p - delta_p), MIN_PERFORMANCE, MAX_PERFORMANCE);
	next_state = MAX_PERFORMANCE - performance;

	instance->target = next_state;

	return 0;
}

static void thermal_zone_trip_update(struct thermal_zone_device *tz,
				     const struct thermal_trip_desc *td)
{
	const struct thermal_trip *trip = &td->trip;
	int old_target;
	struct thermal_instance *instance;
	int result;

	list_for_each_entry(instance, &td->thermal_instances, trip_node) {
		if (trip->type == THERMAL_TRIP_ACTIVE) {
			continue;
		} else if (trip->type == THERMAL_TRIP_PASSIVE) {
			old_target = instance->target;
			result = get_target_state(tz, instance);
			if (result)
				continue;

			if (instance->initialized && old_target == instance->target)
				continue;

			instance->initialized = true;
			mutex_lock(&instance->cdev->lock);
			instance->cdev->updated = false; /* cdev needs update */
			mutex_unlock(&instance->cdev->lock);
		} else {
			continue;
		}
	}
}

/*
 * acpi_dp_manage - manages passive cooling devices associated with the
 * given zone using delta performance throttling.
 * @tz: thermal_zone_device
 *
 * Throttling Logic: This uses the delta performance of passive cooling
 * devices to throttle passive cooling devices. Delta performance is calculated
 * according the formula below:
 * delta_p = tc1 * (cur_temp - pre_temp) + tc2 * (cur_temp - trip_temp)
 * @tc1: thermal constant 1
 * @tc2: thermal constant 2
 * @cur_temp: current temperature of devices
 * @pre_temp: previous temperature of devices
 * @trip_temp: target(trip point) temperature of devices
 */
static void acpi_dp_manage(struct thermal_zone_device *tz)
{
	const struct thermal_trip_desc *td;
	struct thermal_instance *instance;

	lockdep_assert_held(&tz->lock);

	for_each_trip_desc(tz, td)
		thermal_zone_trip_update(tz, td);

	for_each_trip_desc(tz, td)
		list_for_each_entry(instance, &td->thermal_instances, trip_node)
			thermal_cdev_update(instance->cdev);
}

static struct thermal_governor thermal_gov_acpi_dp = {
	.name		= "acpi_dp",
	.manage		= acpi_dp_manage,
};
THERMAL_GOVERNOR_DECLARE(thermal_gov_acpi_dp);
