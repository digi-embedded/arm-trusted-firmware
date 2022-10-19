/*
 * Copyright (c) 2020-2021, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>

#include <libfdt.h>

#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <drivers/regulator.h>

#define MAX_PROPERTY_LEN 64

static struct rdev rdev_array[PLAT_NB_RDEVS];

#pragma weak plat_get_lp_mode_name
const char *plat_get_lp_mode_name(int mode)
{
	return NULL;
}

#define for_each_rdev(rdev) \
	for (rdev = rdev_array; rdev < (rdev_array + PLAT_NB_RDEVS); rdev++)

#define for_each_registered_rdev(rdev) \
	for (rdev = rdev_array; \
	     (rdev < (rdev_array + PLAT_NB_RDEVS)) && (rdev->desc != NULL); rdev++)

static void lock_driver(const struct rdev *rdev)
{
	if (rdev->desc->ops->lock != NULL) {
		rdev->desc->ops->lock(rdev->desc);
	}
}

static void unlock_driver(const struct rdev *rdev)
{
	if (rdev->desc->ops->unlock != NULL) {
		rdev->desc->ops->unlock(rdev->desc);
	}
}

static struct rdev *regulator_get_by_phandle(int32_t phandle)
{
	struct rdev *rdev;

	for_each_registered_rdev(rdev) {
		if (rdev->phandle == phandle) {
			return rdev;
		}
	}

	WARN("%s: phandle %d not found\n", __func__, phandle);
	return NULL;
}

/*
 * Get a regulator from its node name
 *
 * @fdt - pointer to device tree memory
 * @node_name - name of the node "ldo1"
 * Return pointer to rdev if succeed, NULL else.
 */
struct rdev *regulator_get_by_name(const char *node_name)
{
	struct rdev *rdev;

	assert(node_name != NULL);
	VERBOSE("get %s\n", node_name);

	for_each_registered_rdev(rdev) {
		if (strcmp(rdev->desc->node_name, node_name) == 0) {
			return rdev;
		}
	}

	WARN("%s: %s not found\n", __func__, node_name);
	return NULL;
}

static int32_t get_supply_phandle(const void *fdt, int node, const char *name)
{
	const fdt32_t *cuint;
	int len __unused;
	int supply_phandle = -FDT_ERR_NOTFOUND;
	char prop_name[MAX_PROPERTY_LEN];

	len = snprintf(prop_name, MAX_PROPERTY_LEN - 1, "%s-supply", name);
	assert((len >= 0) && (len < MAX_PROPERTY_LEN - 1));

	cuint = fdt_getprop(fdt, node, prop_name, NULL);
	if (cuint != NULL) {
		supply_phandle = fdt32_to_cpu(*cuint);
		VERBOSE("%s: supplied by %d\n", name, supply_phandle);
	}

	return supply_phandle;
}

/*
 * Get a regulator from a supply name
 *
 * @fdt - pointer to device tree memory
 * @node - offset of the node that contains the supply description
 * @name - name of the supply "vdd" for "vdd-supply'
 * Return pointer to rdev if succeed, NULL else.
 */
struct rdev *regulator_get_by_supply_name(const void *fdt, int node, const char *name)
{
	const int p = get_supply_phandle(fdt, node, name);

	if (p < 0) {
		return NULL;
	}

	return regulator_get_by_phandle(p);
}

static int __regulator_set_state(struct rdev *rdev, bool state)
{
	if (rdev->desc->ops->set_state == NULL) {
		return -ENODEV;
	}

	return rdev->desc->ops->set_state(rdev->desc, state);
}

#if defined(IMAGE_BL32)
/*
 * Enable regulator supply
 * Enable regulator if use_count == 0
 * Apply ramp delay
 *
 * @rdev - pointer to rdev struct
 * Return 0 if succeed, non 0 else.
 */
static int __regulator_enable(struct rdev *rdev)
{
	VERBOSE("%s: en\n", rdev->desc->node_name);

	if (rdev->desc->ops->set_state == NULL) {
		return -ENODEV;
	}

	if (rdev->supply_dev != NULL)
		regulator_enable(rdev->supply_dev);

	lock_driver(rdev);

	if (rdev->use_count == 0) {
		int ret;

		ret = __regulator_set_state(rdev, STATE_ENABLE);
		if (ret != 0) {
			ERROR("regul %s set state failed: err:%d\n",
			      rdev->desc->node_name, ret);
			unlock_driver(rdev);
			return ret;
		}

		udelay(rdev->enable_ramp_delay);
	}

	rdev->use_count++;

	assert(rdev->use_count != UINT8_MAX);

	VERBOSE("%s: en count:%u\n", rdev->desc->node_name, rdev->use_count);

	unlock_driver(rdev);

	return 0;
}

/*
 * Enable regulator
 *
 * @rdev - pointer to rdev struct
 * Return 0 if succeed, non 0 else.
 */
int regulator_enable(struct rdev *rdev)
{
	assert(rdev != NULL);

	if (rdev->flags & REGUL_ALWAYS_ON) {
		return 0;
	}

	return __regulator_enable(rdev);
}

/*
 * Disable regulator if use_count fall to zero
 * Warn if count is < 0 because too many disable were requested
 * Disable regulator supply
 *
 * @rdev - pointer to rdev struct
 * Return 0 if succeed, non 0 else.
 */
static int __regulator_disable(struct rdev *rdev)
{
	VERBOSE("%s: dis\n", rdev->desc->node_name);

	if (rdev->desc->ops->set_state == NULL) {
		return -ENODEV;
	}

	lock_driver(rdev);

	if (rdev->use_count == 1) {
		int ret;

		ret = __regulator_set_state(rdev, STATE_DISABLE);
		if (ret != 0) {
			ERROR("regul %s set state failed: err:%d\n",
			      rdev->desc->node_name, ret);
			unlock_driver(rdev);
			return ret;
		}
	}

	if (rdev->use_count == 0) {
		WARN("regulator %s unbalanced disable\n", rdev->desc->node_name);
	} else {
		rdev->use_count--;
	}

	VERBOSE("%s: dis count:%u\n", rdev->desc->node_name, rdev->use_count);

	unlock_driver(rdev);

	if (rdev->supply_dev != NULL) {
		regulator_disable(rdev->supply_dev);
	}

	return 0;
}

/*
 * Disable regulator
 *
 * @rdev - pointer to rdev struct
 * Return 0 if succeed, non 0 else.
 */
int regulator_disable(struct rdev *rdev)
{
	assert(rdev != NULL);

	if (rdev->flags & REGUL_ALWAYS_ON) {
		return 0;
	}

	return __regulator_disable(rdev);
}
#else
/*
 * Enable regulator
 *
 * @rdev - pointer to rdev struct
 * Return 0 if succeed, non 0 else.
 */
int regulator_enable(struct rdev *rdev)
{
	int ret;

	assert(rdev != NULL);

	ret = __regulator_set_state(rdev, STATE_ENABLE);

	udelay(rdev->enable_ramp_delay);

	return ret;
}

/*
 * Disable regulator
 *
 * @rdev - pointer to rdev struct
 * Return 0 if succeed, non 0 else.
 */
int regulator_disable(struct rdev *rdev)
{
	int ret;

	assert(rdev != NULL);

	if (rdev->flags & REGUL_ALWAYS_ON) {
		return 0;
	}

	ret = __regulator_set_state(rdev, STATE_DISABLE);

	udelay(rdev->enable_ramp_delay);

	return ret;
}
#endif

/*
 * Regulator enabled query
 *
 * @rdev - pointer to rdev struct
 * Return 0 if disabled, 1 if enabled, <0 else.
 */
int regulator_is_enabled(const struct rdev *rdev)
{
	int ret = 0;

	assert(rdev != NULL);

	VERBOSE("%s: is en\n", rdev->desc->node_name);

	if (rdev->desc->ops->get_state == NULL) {
		return -ENODEV;
	}

	lock_driver(rdev);

	ret = rdev->desc->ops->get_state(rdev->desc);
	if (ret < 0) {
		ERROR("regul %s get state failed: err:%d\n",
		      rdev->desc->node_name, ret);
	}

	unlock_driver(rdev);

	return ret;
}

/*
 * Set regulator voltage
 *
 * @rdev - pointer to rdev struct
 * @mvolt - Target voltage level in millivolt
 * Return 0 if succeed, non 0 else.
 */
int regulator_set_voltage(struct rdev *rdev, uint16_t mvolt)
{
	int ret = 0;

	assert(rdev != NULL);

	VERBOSE("%s: set mvolt\n", rdev->desc->node_name);

	if (rdev->desc->ops->set_voltage == NULL) {
		return -ENODEV;
	}

	if ((mvolt < rdev->min_mv) || (mvolt > rdev->max_mv)) {
		return -EPERM;
	}

	lock_driver(rdev);

	ret = rdev->desc->ops->set_voltage(rdev->desc, mvolt);
	if (ret < 0) {
		ERROR("regul %s set volt failed: err:%d\n",
		      rdev->desc->node_name, ret);
	}

	unlock_driver(rdev);

	return ret;
}

/*
 * Set regulator min voltage
 *
 * @rdev - pointer to rdev struct
 * Return 0 if succeed, non 0 else.
 */
int regulator_set_min_voltage(struct rdev *rdev)
{
	return regulator_set_voltage(rdev, rdev->min_mv);
}

/*
 * Get regulator voltage
 *
 * @rdev - pointer to rdev struct
 * Return milli volts if succeed, <0 else.
 */
int regulator_get_voltage(const struct rdev *rdev)
{
	int ret = 0;

	assert(rdev != NULL);

	VERBOSE("%s: get volt\n", rdev->desc->node_name);

	if (rdev->desc->ops->get_voltage == NULL) {
		return rdev->min_mv;
	}

	lock_driver(rdev);

	ret = rdev->desc->ops->get_voltage(rdev->desc);
	if (ret < 0) {
		ERROR("regul %s get voltage failed: err:%d\n",
		      rdev->desc->node_name, ret);
	}

	unlock_driver(rdev);

	return ret;
}

/*
 * List regulator voltages
 *
 * @rdev - pointer to rdev struct
 * @levels - out: array of supported millitvolt levels from min to max value
 * @count - out: number of possible millivolt values
 * Return 0 if succeed, non 0 else.
 */
int regulator_list_voltages(const struct rdev *rdev, const uint16_t **levels, size_t *count)
{
	int ret;
	size_t n;

	assert(rdev != NULL);
	assert(levels != NULL);
	assert(count != NULL);

	VERBOSE("%s: list volt\n", rdev->desc->node_name);

	if (rdev->desc->ops->list_voltages == NULL) {
		return -ENODEV;
	}

	lock_driver(rdev);

	ret = rdev->desc->ops->list_voltages(rdev->desc, levels, count);

	unlock_driver(rdev);

	if (ret < 0) {
		ERROR("regul %s list_voltages failed: err: %d\n",
		      rdev->desc->node_name, ret);
		return ret;
	}

	/*
	 * Reduce the possible values depending on min and max from device-tree
	 */
	n = *count;
	while (((*levels)[n - 1U] > rdev->max_mv) && (n > 1U)) {
		n--;
	}

	/* Verify that max val is a valid value */
	if (rdev->max_mv != (*levels)[n - 1]) {
		ERROR("regul %s: max value %u is invalid\n",
		      rdev->desc->node_name, rdev->max_mv);
		return -EINVAL;
	}

	while (((*levels[0U]) < rdev->min_mv) && (n > 1U)) {
		(*levels)++;
		n--;
	}

	/* Verify that min is not too high */
	if (n == 0U) {
		ERROR("regul %s set min voltage is too high\n",
		      rdev->desc->node_name);
		return -EINVAL;
	}

	/* Verify that min val is a valid vlue */
	if (rdev->min_mv != (*levels)[0U]) {
		ERROR("regul %s: min value %u is invalid\n",
		      rdev->desc->node_name, rdev->min_mv);
		return -EINVAL;
	}

	*count = n;

	VERBOSE("rdev->min_mv=%u rdev->max_mv=%u\n", rdev->min_mv, rdev->max_mv);

	return 0;
}

/*
 * Get regulator voltages range
 *
 * @rdev - pointer to rdev struct
 * @min_mv - out: min possible millivolt value
 * @max_mv - out: max possible millivolt value
 * Return 0 if succeed, non 0 else.
 */
void regulator_get_range(const struct rdev *rdev, uint16_t *min_mv, uint16_t *max_mv)
{
	assert(rdev != NULL);

	if (min_mv != NULL) {
		*min_mv = rdev->min_mv;
	}
	if (max_mv != NULL) {
		*max_mv = rdev->max_mv;
	}
}

/*
 * Set regulator flag
 *
 * @rdev - pointer to rdev struct
 * @flag - flag value to set (eg: REGUL_OCP)
 * Return 0 if succeed, non 0 else.
 */
int regulator_set_flag(struct rdev *rdev, uint16_t flag)
{
	int ret;

	/* check that only one bit is set on flag */
	if (__builtin_popcount(flag) != 1) {
		return -EINVAL;
	}

	/* REGUL_ALWAYS_ON and REGUL_BOOT_ON are internal properties of the core */
	if ((flag == REGUL_ALWAYS_ON) || (flag == REGUL_BOOT_ON)) {
		rdev->flags |= flag;
		return 0;
	}

	if (rdev->desc->ops->set_flag == NULL) {
		ERROR("%s can not set any flag\n", rdev->desc->node_name);
		return -ENODEV;
	}

	lock_driver(rdev);

	ret = rdev->desc->ops->set_flag(rdev->desc, flag);

	unlock_driver(rdev);

	if (ret != 0) {
		ERROR("%s: could not set flag %d ret=%d\n",
		      rdev->desc->node_name, flag, ret);
		return ret;
	}

	rdev->flags |= flag;

	return 0;
}

#if defined(IMAGE_BL32)

struct regul_property {
	char *name;
	uint16_t flag;
};

static struct regul_property flag_prop[] = {
	{
		.name = "regulator-always-on",
		.flag = REGUL_ALWAYS_ON,
	},
	{
		.name = "regulator-boot-on",
		.flag = REGUL_BOOT_ON,
	},
	{
		.name = "regulator-active-discharge",
		.flag = REGUL_ACTIVE_DISCHARGE,
	},
	{
		.name = "regulator-over-current-protection",
		.flag = REGUL_OCP,
	},
	{
		.name = "regulator-pull-down",
		.flag = REGUL_PULL_DOWN,
	},
	{
		.name = "st,mask-reset",
		.flag = REGUL_MASK_RESET,
	},
	{
		.name = "st,regulator-sink-source",
		.flag = REGUL_SINK_SOURCE,
	},
	{
		.name = "st,regulator-bypass",
		.flag = REGUL_ENABLE_BYPASS,
	},
};

static int parse_properties(const void *fdt, struct rdev *rdev, int node)
{
	const fdt32_t *cuint;
	int ret = 0;
	struct regul_property *prop;

	for (prop = flag_prop; prop < (flag_prop + ARRAY_SIZE(flag_prop)); prop++) {
		if (fdt_getprop(fdt, node, prop->name, NULL) != NULL) {
			VERBOSE("%s: prop 0x%x\n", rdev->desc->node_name, prop->flag);
			ret = regulator_set_flag(rdev, prop->flag);
			if (ret != 0) {
				return ret;
			}
		}
	}

	cuint = fdt_getprop(fdt, node, "regulator-enable-ramp-delay", NULL);
	if (cuint != NULL) {
		rdev->enable_ramp_delay = (uint32_t)(fdt32_to_cpu(*cuint));
		VERBOSE("%s: enable_ramp_delay=%u\n", rdev->desc->node_name,
			rdev->enable_ramp_delay);
	}

	rdev->reg_name = fdt_getprop(fdt, node, "regulator-name", NULL);

	return 0;
}

static void parse_supply(const void *fdt, struct rdev *rdev, int node)
{
	const char *name = rdev->desc->supply_name;

	if (name == NULL) {
		name = rdev->desc->node_name;
	}

	rdev->supply_phandle = get_supply_phandle(fdt, node, name);
	if (rdev->supply_phandle < 0) {
		node = fdt_parent_offset(fdt, node);
		rdev->supply_phandle = get_supply_phandle(fdt, node, name);
	}
}

static void parse_low_power_mode(const void *fdt, struct rdev *rdev, int node, int mode)
{
	const fdt32_t *cuint;

	rdev->lp_state[mode] = 0;

	if (fdt_getprop(fdt, node, "regulator-off-in-suspend", NULL) != NULL) {
		VERBOSE("%s: mode:%d OFF\n", rdev->desc->node_name, mode);
		rdev->lp_state[mode] |= LP_STATE_OFF;
	} else if (fdt_getprop(fdt, node, "regulator-on-in-suspend", NULL) != NULL) {
		VERBOSE("%s: mode:%d ON\n", rdev->desc->node_name, mode);
		rdev->lp_state[mode] |= LP_STATE_ON;
	} else {
		rdev->lp_state[mode] |= LP_STATE_UNCHANGED;
	}

	cuint = fdt_getprop(fdt, node, "regulator-suspend-microvolt", NULL);
	if (cuint != NULL) {
		uint16_t mv;

		mv = (uint16_t)(fdt32_to_cpu(*cuint) / 1000U);
		VERBOSE("%s: mode:%d suspend mv=%u\n", rdev->desc->node_name,
			mode, mv);

		rdev->lp_state[mode] |= LP_STATE_SET_VOLT;
		rdev->lp_mv[mode] = mv;
	}
}

static void parse_low_power_modes(const void *fdt, struct rdev *rdev, int node)
{
	int i;

	for (i = 0; i < PLAT_NB_SUSPEND_MODES; i++) {
		const char *lp_mode_name = plat_get_lp_mode_name(i);
		int n;

		if (lp_mode_name == NULL) {
			continue;
		}

		/* Get the configs from regulator_state_node subnode */
		n = fdt_subnode_offset(fdt, node, lp_mode_name);
		if (n >= 0) {
			parse_low_power_mode(fdt, rdev, n, i);
		}
	}
}
#else
static int parse_properties(const void *fdt, struct rdev *rdev, int node)
{
	const fdt32_t *cuint;
	int ret;

	if (fdt_getprop(fdt, node, "regulator-always-on", NULL) != NULL) {
		VERBOSE("%s: set regulator-always-on\n", rdev->desc->node_name);
		ret = regulator_set_flag(rdev, REGUL_ALWAYS_ON);
		if (ret != 0) {
			return ret;
		}
	}

	cuint = fdt_getprop(fdt, node, "regulator-enable-ramp-delay", NULL);
	if (cuint != NULL) {
		rdev->enable_ramp_delay = fdt32_to_cpu(*cuint);
		VERBOSE("%s: enable_ramp_delay=%u\n", rdev->desc->node_name,
			rdev->enable_ramp_delay);
	}

	return 0;
}
#endif

/*
 * Parse the device-tree for a regulator
 *
 * Read min/max voltage from dt and check its validity
 * Read the properties, and call the driver to set flags
 * Read power supply phandle
 * Read and store low power mode states
 *
 * @rdev - pointer to rdev struct
 * @node - device-tree node offset of the regulator
 * Return 0 if disabled, 1 if enabled, <0 else.
 */
static int parse_dt(struct rdev *rdev, int node)
{
	void *fdt;
	const fdt32_t *cuint;
	const uint16_t *levels;
	size_t size;
	int ret = 0;

	VERBOSE("%s: parse dt\n", rdev->desc->node_name);

	if (fdt_get_address(&fdt) == 0) {
		return -ENOENT;
	}

	rdev->phandle = fdt_get_phandle(fdt, node);

	cuint = fdt_getprop(fdt, node, "regulator-min-microvolt", NULL);
	if (cuint != NULL) {
		uint16_t min_mv;

		min_mv = (uint16_t)(fdt32_to_cpu(*cuint) / 1000U);
		VERBOSE("%s: min_mv=%d\n", rdev->desc->node_name, (int)min_mv);
		if (min_mv <= rdev->max_mv) {
			rdev->min_mv = min_mv;
		} else {
			ERROR("%s: min_mv=%d is too high\n",
			      rdev->desc->node_name, (int)min_mv);
			return -EINVAL;
		}
	}

	cuint = fdt_getprop(fdt, node, "regulator-max-microvolt", NULL);
	if (cuint != NULL) {
		uint16_t max_mv;

		max_mv = (uint16_t)(fdt32_to_cpu(*cuint) / 1000U);
		VERBOSE("%s: max_mv=%d\n", rdev->desc->node_name, (int)max_mv);
		if (max_mv >= rdev->min_mv) {
			rdev->max_mv = max_mv;
		} else {
			ERROR("%s: max_mv=%d is too low\n",
			      rdev->desc->node_name, (int)max_mv);
			return -EINVAL;
		}
	}

	/* validate that min and max values can be used */
	ret = regulator_list_voltages(rdev, &levels, &size);
	if ((ret != 0) && (ret != -ENODEV)) {
		return ret;
	}

	ret = parse_properties(fdt, rdev, node);
	if (ret != 0) {
		return ret;
	}

#if defined(IMAGE_BL32)
	parse_supply(fdt, rdev, node);

	parse_low_power_modes(fdt, rdev, node);
#endif

	return 0;
}

/*
 * Register a regulator driver in regulator framework.
 * Initialize voltage range from driver description
 *
 * @desc - pointer to the regulator description
 * @node - device-tree node offset of the regulator
 * Return 0 if succeed, non 0 else.
 */
int regulator_register(const struct regul_description *desc, int node)
{
	struct rdev *rdev;

	assert(desc != NULL);

	VERBOSE("register %s\n", desc->node_name);

	for_each_rdev(rdev) {
		if (rdev->desc == NULL) {
			break;
		}
	}

	if (rdev == rdev_array + PLAT_NB_RDEVS) {
		WARN("Not enough place for regulators, PLAT_NB_RDEVS should be increased.\n");
		return -ENOMEM;
	}

	rdev->desc = desc;
	rdev->enable_ramp_delay = rdev->desc->enable_ramp_delay;

	if (rdev->desc->ops->list_voltages != NULL) {
		int ret = 0;
		const uint16_t *levels;
		size_t count;

		lock_driver(rdev);

		ret = rdev->desc->ops->list_voltages(rdev->desc, &levels, &count);

		unlock_driver(rdev);

		if (ret < 0) {
			ERROR("regul %s set state failed: err:%d\n",
			      rdev->desc->node_name, ret);
			return ret;
		}

		rdev->min_mv = levels[0];
		rdev->max_mv = levels[count - 1U];
	} else {
		rdev->max_mv = UINT16_MAX;
	}

	return parse_dt(rdev, node);
}

#if defined(IMAGE_BL32)
/*
 * Suspend a single regulator before low power entry
 * Call regulator suspend call back,
 * Enable the regulator if boot_on flag is set as regulator is needed during
 * boot/resume from suspend sequences.
 *
 * @rdev - pointer to rdev struct
 * @mode - low power mode index
 * Return 0 if succeed, non 0 else.
 */
static int suspend_regulator(struct rdev *rdev, int mode)
{
	int ret = 0;

	if (rdev->desc->ops->suspend != NULL) {
		lock_driver(rdev);
		ret = rdev->desc->ops->suspend(rdev->desc,
					       rdev->lp_state[mode],
					       rdev->lp_mv[mode]);
		unlock_driver(rdev);
		if (ret != 0) {
			ERROR("%s failed to suspend: %d\n", rdev->desc->node_name, ret);
			return ret;
		}
	}

	if (rdev->flags & REGUL_BOOT_ON) {
		ret = regulator_enable(rdev);
	}

	return ret;
}

/*
 * Resume a single regulator after low power
 *
 * @rdev - pointer to rdev struct
 * Return 0 if succeed, non 0 else.
 */
static int resume_regulator(struct rdev *rdev)
{
	int ret = 0;

	if (rdev->flags & REGUL_BOOT_ON) {
		/* Revert to the state it was before suspend */
		ret = regulator_disable(rdev);
		if (ret != 0) {
			ERROR("%s failed to resume: %d\n", rdev->desc->node_name, ret);
		}
	}

	return ret;
}

/*
 * Suspend regulators before entering low power
 *
 * Return 0 if succeed, non 0 else.
 */
int regulator_core_suspend(int mode)
{
	struct rdev *rdev;

	VERBOSE("Regulator core suspend\n");

	if (mode >= PLAT_NB_SUSPEND_MODES) {
		return -EINVAL;
	}

	/* Suspend each regulator */
	for_each_registered_rdev(rdev) {
		if (suspend_regulator(rdev, mode) != 0) {
			panic();
		}
	}

#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
	regulator_core_dump();
#endif

	return 0;
}

/*
 * Resume regulators from low power
 *
 * Return 0 if succeed, non 0 else.
 */
int regulator_core_resume(void)
{
	struct rdev *rdev;

	VERBOSE("Regulator core resume\n");

	/* Resume each regulator */
	for_each_registered_rdev(rdev) {
		if (resume_regulator(rdev) != 0) {
			panic();
		}
	}

#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
	regulator_core_dump();
#endif

	return 0;
}

/*
 * save regulators data
 *
 * @backup_area - pointer to save data
 * @backup_size - size of the backup area
 */
void regulator_core_backup_context(void *backup_area, size_t backup_size)
{
	int8_t *data = (int8_t *)backup_area;
	struct rdev *rdev;

	assert(data != NULL);
	assert(backup_size == PLAT_BACKUP_REGULATOR_SIZE);

	for_each_rdev(rdev) {
		*data = rdev->use_count;
		data++;
	}
}

/*
 * restore regulators data
 *
 * @backup_area - pointer to retrieve saved data
 * @backup_size - size of the backup area
 */
void regulator_core_restore_context(void *backup_area, size_t backup_size)
{
	int8_t *data = (int8_t *)backup_area;
	struct rdev *rdev;

	assert(data != NULL);
	assert(backup_size == PLAT_BACKUP_REGULATOR_SIZE);

	for_each_rdev(rdev) {
		rdev->use_count = *data;
		data++;
	}
}

#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
static void sprint_name(char *dest, const char *src, int len)
{
	int l;

	if (src == NULL) {
		src = "";
	}

	l = strlen(src);
	if (l > len) {
		l = len;
	}

	memset(dest, ' ', len);
	memcpy(dest, src, l);
	dest[len] = 0;
}

/*
 * Log regulators state
 */
void regulator_core_dump(void)
{
	struct rdev *rdev;

	VERBOSE("Dump Regulators\n");

	INFO("reg      name     use\ten\tmV\tmin\tmax\tflags\tsupply\n");

	for_each_registered_rdev(rdev) {
		uint16_t min_mv, max_mv;
		char reg[9] = "";
		char name[9];
		const char *supply = "";

		sprint_name(name, rdev->desc->node_name, 8);
		sprint_name(reg, rdev->reg_name, 8);

		regulator_get_range(rdev, &min_mv, &max_mv);
		if (rdev->supply_dev != NULL)
			supply = rdev->supply_dev->desc->node_name;

		INFO("%s %s %d\t%d\t%d\t%d\t%d\t0x%x\t%s\n",
		     reg, name,
		     rdev->use_count,
		     regulator_is_enabled(rdev),
		     regulator_get_voltage(rdev),  min_mv, max_mv,
		     rdev->flags, supply);
	}
}
#endif

/*
 * Connect each regulator to its supply
 * Apply min voltage if the voltage is outside the authorized range
 * Enable always-on regulators
 *
 * Return 0 if succeed, non 0 else.
 */
int regulator_core_config(void)
{
	int ret;
	struct rdev *rdev;

	VERBOSE("Regul Core config\n");

	for_each_registered_rdev(rdev) {
		if (rdev->supply_phandle >= 0) {
			struct rdev *s;

			VERBOSE("%s: connect supply\n", rdev->desc->node_name);

			s = regulator_get_by_phandle(rdev->supply_phandle);
			if (s == NULL) {
				return -EINVAL;
			}

			rdev->supply_dev = s;
		}
	}

	for_each_registered_rdev(rdev) {
		uint16_t mv, min_mv, max_mv;

		regulator_get_range(rdev, &min_mv, &max_mv);

		ret = regulator_get_voltage(rdev);
		if (ret >= 0) {
			mv = ret;
			if ((mv < min_mv) || (mv > max_mv)) {
				ret = regulator_set_voltage(rdev, min_mv);
				if (ret != 0) {
					return ret;
				}
			}
		} else {
			return ret;
		}

		/*
		 * Enable always-on regulator and increment its use_count so that
		 * the regulator is not being disabled during clean-up sequence.
		 */
		if (rdev->flags & REGUL_ALWAYS_ON) {
			ret = __regulator_enable(rdev);
			if (ret != 0) {
				return ret;
			}
		}
	}

	return 0;
}

/*
 * Sync hardware regulator state with use refcount
 *
 * Return 0 if succeed, non 0 else.
 */
int regulator_core_cleanup(void)
{
	struct rdev *rdev;

	VERBOSE("Regul Core cleanup\n");

	for_each_registered_rdev(rdev) {
		if (!(rdev->flags & REGUL_BOOT_ON)) {
			if ((rdev->use_count == 0) && (regulator_is_enabled(rdev) == 1)) {
				VERBOSE("disable %s during cleanup\n", rdev->desc->node_name);
				/* force disable to synchronize the framework */
				__regulator_set_state(rdev, STATE_DISABLE);
			}
		}
	}

#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
	regulator_core_dump();
#endif

	return 0;
}

#endif
