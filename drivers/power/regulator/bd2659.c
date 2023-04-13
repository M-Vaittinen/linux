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
#include <power/bd2659.h>
#include <power/pmic.h>
#include <power/regulator.h>
#include <power/voltage_range.h>

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
 * @sel_mask:	bit to toggle in order to transfer the register control to SW
 * @dvs:	whether the voltage can be changed when regulator is enabled
 */
struct bd2659_plat {
	const char		*name;
	int			id;
	struct regulator_vrange	*ranges;
	unsigned int		numranges;
	u8			sel_mask;
	u8			vsel_cache;
	bool			dvs;
};

#define BD_DATA(_name, _range, _id) \
{ .name = (_name), .ranges = &(_range)[0],  .numranges = ARRAY_SIZE(_range), .id = (_id) }

#define BD2659_NUM_DYN_VOLTS 0xab
#define BD2659_DYN_VOLT_STEP 5000 /* uV */
static struct regulator_vrange buck123_vranges[] = {
	BD_RANGE(500000, BD2659_DYN_VOLT_STEP, 1, BD2659_NUM_DYN_VOLTS),
	BD_RANGE(1350000, 0, BD2659_NUM_DYN_VOLTS + 1, 0xff),
};

static struct regulator_vrange buck4_vranges[] = {
	BD_RANGE(1100000, BD2659_DYN_VOLT_STEP, 1, BD2659_NUM_DYN_VOLTS),
	BD_RANGE(1850000, 0, BD2659_NUM_DYN_VOLTS + 1, 0xff),
};


static struct bd2659_plat bd2659_reg_data[] = {
	BD_DATA("BUCK1", buck123_vranges, BD2659_BUCK1_ID),
	BD_DATA("BUCK2", buck123_vranges, BD2659_BUCK2_ID),
	BD_DATA("BUCK3", buck123_vranges, BD2659_BUCK3_ID),
	BD_DATA("BUCK4", buck4_vranges, BD2659_BUCK4_ID),
};

#define OTP_PROP "rohm,otp-min-microvolt"

static int bd2659_get_enable(struct udevice *dev)
{
	int val;
	struct bd2659_plat *plat = dev_get_plat(dev);
	int reg = TO_BUCKx_REG(plat->id, BD2659_BUCK0_VID_S0);

	val = pmic_reg_read(dev->parent, reg);
	if (val < 0)
		return val;

	return !!val;
}

static int bd2659_set_enable(struct udevice *dev, bool enable)
{
	int val;
	struct bd2659_plat *plat = dev_get_plat(dev);
	int reg = TO_BUCKx_REG(plat->id, BD2659_BUCK0_VID_S0);

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
		struct regulator_vrange *r = &plat->ranges[i];

		if (!vrange_find_value(r, plat->vsel_cache, &tmp))
			return tmp;
	}

	pr_err("Unknown voltage value\n");

	return -EINVAL;
}

static int bd2659_set_value(struct udevice *dev, int uvolt)
{
	struct bd2659_plat *plat = dev_get_plat(dev);
	int reg = TO_BUCKx_REG(plat->id, BD2659_BUCK0_VID_S0);
	unsigned int sel;
	int i, enable;
	int found = 0;

	for (i = 0; i < plat->numranges; i++) {
		struct regulator_vrange *r = &plat->ranges[i];

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
	int ret, i, rev, vendor;
	struct udevice *parent;
	int buck4_vbase;

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

	/* BD2659 has OTP option to change BUCK4 voltage range */

	for (i = 0; i < data_amnt; i++) {
		if (!strcmp(dev->name, bd2659_reg_data[i].name)) {
			int reg;

			if (i == 3) {
				ret = dev_read_u32(dev, OTP_PROP, &buck4_vbase);
				if (!ret) {
					printf("Found %s\n", OTP_PROP);
					buck4_vranges[0].min_volt = buck4_vbase;
					buck4_vranges[1].min_volt = buck4_vbase +
						(BD2659_NUM_DYN_VOLTS - 1 ) * BD2659_DYN_VOLT_STEP;
				} else {
					printf("Not Found %s\n", OTP_PROP);
				}
			}

			*plat = bd2659_reg_data[i];
			reg = TO_BUCKx_REG(plat->id, BD2659_BUCK0_VID_S0);
			plat->vsel_cache = pmic_reg_read(parent, reg);
			if (plat->vsel_cache < 0) {
				pr_err("Reading S0 voltage failed\n");

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

