// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022 ROHM Semiconductors
 *
 * ROHM BD71837 regulator driver
 */

#include <common.h>
#include <dm.h>
#include <log.h>
#include <linux/bitops.h>
#include <power/bd71837.h>
#include <power/pmic.h>
#include <power/regulator.h>

/**
 * struct bd71837_vrange - describe linear range of voltages
 *
 * @min_volt:	smallest voltage in range
 * @step:	how much voltage changes at each selector step
 * @min_sel:	smallest selector in the range
 * @max_sel:	maximum selector in the range
 * @rangeval:	register value used to select this range if selectible
 *		ranges are supported
 */
struct bd2659_vrange {
	unsigned int	min_volt;
	unsigned int	step;
	u8		min_sel;
	u8		max_sel;
};

/**
 * struct bd2659_plat - describe regulator control registers
 *
 * @name:	name of the regulator. Used for matching the dt-entry
 * @enable_reg:	register address used to enable/disable regulator
 * @enablemask:	register mask used to enable/disable regulator
 * @volt_reg:	register address used to configure regulator voltage
 * @volt_mask:	register mask used to configure regulator voltage
 * @ranges:	pointer to ranges of regulator voltages and matching register
 *		values
 * @numranges:	number of voltage ranges pointed by ranges
 * @rangemask:	mask for selecting used ranges if multiple ranges are supported
 * @sel_mask:	bit to toggle in order to transfer the register control to SW
 * @dvs:	whether the voltage can be changed when regulator is enabled
 */
struct bd2659_plat {
	const char		*name;
	int			id;
	struct bd2659_vrange	*ranges;
	unsigned int		numranges;
	u8			rangemask;
	u8			sel_mask;
	u8			vsel_cache;
	bool			dvs;
};

#define BD_RANGE(_min, _vstep, _sel_low, _sel_hi) \
{ \
	.min_volt = (_min), .step = (_vstep), .min_sel = (_sel_low), \
	.max_sel = (_sel_hi), \
}

#define BD_DATA(_name, _range) \
{ .name = (_name), .ranges = (_range),  .numranges = ARRAY_SIZE(_range), }

static struct bd2659_vrange buck012_vranges[] = {
	BD_RANGE(500000, 5000, 1, 0xab, 0),
	BD_RANGE(1350000, 0, 0xac, 0xff, 0),
};

static struct bd2659_vrange buck3_vranges[] = {
	BD_RANGE(700000, 10000, 0, 0x3c, 0),
	BD_RANGE(1300000, 0, 0x3d, 0x3f, 0),
};

/*
 * We use enable mask 'HW_STATE_CONTROL' to indicate that this regulator
 * must not be enabled or disabled by SW. The typical use-case for BD71837
 * is powering NXP i.MX8. In this use-case we (for now) only allow control
 * for BUCK3 and BUCK4 which are not boot critical.
 */

static struct bd2659_plat bd2659_reg_data[] = {
	BD_DATA("BUCK0", &buck012_vranges),
	BD_DATA("BUCK1", &buck012_vranges),
	BD_DATA("BUCK2", &buck012_vranges),
	BD_DATA("BUCK3", &buck3_vranges),
};

static int vrange_find_value(struct bd71837_vrange *r, unsigned int sel,
			     unsigned int *val)
{
	if (!val || sel < r->min_sel || sel > r->max_sel)
		return -EINVAL;

	*val = r->min_volt + r->step * (sel - r->min_sel);
	return 0;
}

static int vrange_find_selector(struct bd71837_vrange *r, int val,
				unsigned int *sel)
{
	int ret = -EINVAL;
	int num_vals = r->max_sel - r->min_sel + 1;

	if (val >= r->min_volt &&
	    val <= r->min_volt + r->step * (num_vals - 1)) {
		if (r->step) {
			*sel = r->min_sel + ((val - r->min_volt) / r->step);
			ret = 0;
		} else {
			*sel = r->min_sel;
			ret = 0;
		}
	}
	return ret;
}

static int bd2659_get_enable(struct udevice *dev)
{
	int val;
	struct bd2659_plat *plat = dev_get_plat(dev);
	int reg = TO_BUCKx_REG(BD2659_BUCK0_VID_S0,  plat->id);

	val = pmic_reg_read(dev->parent, reg);
	if (val < 0)
		return val;

	return !!val;
}

static int bd2659_set_enable(struct udevice *dev, bool enable)
{
	int val;
	struct bd2659_plat *plat = dev_get_plat(dev);
	int reg = TO_BUCKx_REG(BD2659_BUCK0_VID_S0,  plat->id);

	if (enable)
		val = plat->vsel_cache;
	else
		val = 0;

	return pmic_reg_write(dev->parent, reg, val);
}

static int bd2659_get_value(struct udevice *dev)
{
	struct bd2659_plat *plat = dev_get_plat(dev);
	unsigned int tmp;
	int i;

	for (i = 0; i < plat->numranges; i++) {
		struct bd2659_vrange *r = &plat->ranges[i];

		if (!vrange_find_value(r, plat->vsel_cache, &tmp))
			return tmp;
	}

	pr_err("Unknown voltage value\n");

	return -EINVAL;
}

static int bd2659_set_value(struct udevice *dev, int uvolt)
{
	struct bd2659_plat *plat = dev_get_plat(dev);
	int reg = TO_BUCKx_REG(BD2659_BUCK0_VID_S0,  plat->id);
	unsigned int sel;
	unsigned int range;
	int i, enable;
	int found = 0;

	for (i = 0; i < plat->numranges; i++) {
		struct bd2659_vrange *r = &plat->ranges[i];

		found = !vrange_find_selector(r, uvolt, &sel);
		if (found) {
			unsigned int tmp;

			/*
			 * We require exactly the requested value to be
			 * supported - this can be changed later if needed
			 */
			found = !vrange_find_value(r, sel, &tmp);
			if (found && tmp == uvolt)
				break;
			found = 0;
		}
	}

	if (!found)
		return -EINVAL;

	plat->vsel_cache = sel;

	/*
	 * If the regulator is not enabled we just cache new voltage value and
	 * return. Also check for errors and return one if read failed.
	 */
	enable = bd2659_get_enable(dev);
	if (!enable || enable < 0)
		return enable;

	 /* If regulator was enabled, then we update new voltage to HW. */

	return pmic_reg_write(dev->parent, reg, plat->vsel_cache);
}

static int bd2659_regulator_probe(struct udevice *dev)
{
	struct bd2659_plat *plat = dev_get_plat(dev);
	struct dm_regulator_uclass_plat *uc_pdata;
	int data_amnt = BD2659_REGULATOR_AMOUNT;
	int i, ret, rev, vendor;
	struct udevice *parent;

	parent = dev_get_parent(dev);
	if (!parent) {
		pr_err("No parent\n");
		return -ENODEV;
	}

	vendor = pmic_reg_read(parent, BD2659_ID);
	if (vendor < 0)
		return vendor;

	rev = pmic_reg_read(parent, BD2659_ID);
	if (rev < 0)
		return rev;

	if (vendor != BD2659_VENDOR_ROHM || rev != BD2659_REV_KNOWN)
		pr_warn("Unknown vendor / revision\n");

	for (i = 0; i < data_amnt; i++) {
		if (!strcmp(dev->name, bd2659_reg_data[i].name)) {
			int reg;

			*plat = bd2659_reg_data[i];
			reg = TO_BUCKx_REG(BD2659_BUCK0_VID_S0,  plat->id);
			plat->vsel_cache = pmic_reg_read(parent, reg);
			if (plat->vsel_cache < 0) {
				pr_err("Reading S3 voltage failed\n");

				return plat->vsel_cache;
			}
			/*
			 * BD2659 does not have separate disable/enable reg/bit
			 * for bucks. Instead, the buck is disabled when voltage
			 * selector is set to 0. This means that we must cache
			 * voltage value in the driver to support the standard
			 * separation of "set voltage" and "enable / disable".
			 * (To allow for example request to enable regulator
			 * without specifying a voltage value).
			 *
			 * So, Initialize the cached voltage with a default one
			 * if the buck was disabled. TODO: Do we need to support
			 * setting the default voltage via DT?
			 */
			if (!plat->vsel_cache)
				plat->vsel_cache = BD2659_VSEL_DEFAULT;

			uc_pdata = dev_get_uclass_plat(dev);
			return bd2659_set_enable(dev, !!(uc_pdata->boot_on ||
						  uc_pdata->always_on));
		}
	}

	pr_err("Unknown regulator '%s'\n", dev->name);

	return -ENOENT;
}

static const struct dm_regulator_ops bd2659_regulator_ops = {
	.get_value  = bd2659_get_value,
	.set_value  = bd2659_set_value,
	.get_enable = bd2659_get_enable,
	.set_enable = bd2659_set_enable,
};

U_BOOT_DRIVER(bd2659_regulator) = {
	.name = BD2659_REGULATOR_DRIVER,
	.id = UCLASS_REGULATOR,
	.ops = &bd2659_regulator_ops,
	.probe = bd2659_regulator_probe,
	.plat_auto = sizeof(struct bd2659_plat),
};

