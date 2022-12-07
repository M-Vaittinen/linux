// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022 ROHM Semiconductors
 *
 * ROHM BD71885 regulator driver
 */

#include <common.h>
#include <dm.h>
#include <log.h>
#include <linux/bitops.h>
#include <linux/stringify.h>
#include <power/bd71885.h>
#include <power/voltage_range.h>
#include <power/pmic.h>
#include <power/regulator.h>

struct bd71885_plat {
	const char		*name;
	int			id;
	struct regulator_vrange	*ranges;
	unsigned int		numranges;
	u8			sel_mask;
	int			en_reg;
	int			vsel_reg;
	bool			dvs;
};

#define BD_DATA(_id, _range, _en_reg) \
{ \
	.name = __stringify(_id), .ranges = (_range), \
	.numranges = ARRAY_SIZE(_range), .id = (_id), \
	.en_reg = (_en_reg), \
	.vsel_reg = (_id) < LDO1 ? (_en_reg) + 2 : (_en_reg) + 1 \
}

static struct regulator_vrange buck123458_vranges[] = {
	BD_RANGE(500000, 10000, 0, 0x50),
	BD_RANGE(1300000, 0, 0x51, 0xff),
};

static struct regulator_vrange buck5_lo_vranges[] = {
	BD_RANGE(300000, 10000, 0, 0x46),
	BD_RANGE(1000000, 0, 0x47, 0xff),
};

static struct regulator_vrange buck67_vranges[] = {
	BD_RANGE(2140000, 10000, 0, 0xc8),
	BD_RANGE(3500000, 0, 0xc9, 0xff),
};

/*
 * LDO1 and LDO3 default to the low range. There is an OTP option for hi range
 * as well.
 */
static struct regulator_vrange ldo_lo_vranges[] = {
	BD_RANGE(1560000, 10000, 0, 0x78),
	BD_RANGE(1800000, 0, 0x79, 0xff),
};

static struct regulator_vrange ldo_hi_vranges[] = {
	BD_RANGE(1710000, 10000, 0, 0xff),
};

enum {
	BUCK1 = 0,
	BUCK2,
	BUCK3,
	BUCK4,
	BUCK5,
	BUCK6,
	BUCK7,
	BUCK8,
	LDO1,
	LDO2,
	LDO3,
	LDO4,
	BD71885_REGULATOR_AMOUNT
};

static struct bd71885_plat bd71885_reg_data[] = {
	BD_DATA(BUCK1, &buck123458_vranges[0], BD71885_BUCK1_ON),
	BD_DATA(BUCK2, &buck123458_vranges[0], BD71885_BUCK2_ON),
	BD_DATA(BUCK3, &buck123458_vranges[0], BD71885_BUCK3_ON),
	BD_DATA(BUCK4, &buck123458_vranges[0], BD71885_BUCK4_ON),
	BD_DATA(BUCK5, &buck123458_vranges[0], BD71885_BUCK5_ON),
	BD_DATA(BUCK6, &buck67_vranges[0], BD71885_BUCK6_ON),
	BD_DATA(BUCK7, &buck67_vranges[0], BD71885_BUCK7_ON),
	BD_DATA(BUCK8, &buck123458_vranges[0], BD71885_BUCK8_ON),
#ifndef OTP_LDO1_HIRANGE
	BD_DATA(LDO1, &ldo_lo_vranges[0], BD71885_LDO1_ON),
#else
	BD_DATA(LDO1, &ldo_hi_vranges[0], BD71885_LDO1_ON),
#endif
	BD_DATA(LDO2, &ldo_hi_vranges[0], BD71885_LDO2_ON),
#ifndef OTP_LDO3_HIRANGE
	BD_DATA(LDO3, &ldo_lo_vranges[0], BD71885_LDO3_ON),
#else
	BD_DATA(LDO3, &ldo_hi_vranges[0], BD71885_LDO3_ON),
#endif
	BD_DATA(LDO4, &ldo_hi_vranges[0], BD71885_LDO4_ON),
};

static int bd71885_get_enable(struct udevice *dev)
{
	int val;
	struct bd71885_plat *plat = dev_get_plat(dev);

	val = pmic_reg_read(dev->parent, plat->en_reg);
	if (val < 0)
		return val;

	return !!(val & BD71885_MASK_RUN_ON);
}

static int bd71885_set_enable(struct udevice *dev, bool enable)
{
	struct bd71885_plat *plat = dev_get_plat(dev);
	int val;

	if (enable)
		val = BD71885_MASK_RUN_ON;
	else
		val = 0;

	return pmic_clrsetbits(dev->parent, plat->en_reg, BD71885_MASK_RUN_ON,
			       val);
}

static int bd71885_get_value(struct udevice *dev)
{
	struct bd71885_plat *plat = dev_get_plat(dev);
	struct regulator_vrange *r;
	unsigned int tmp, sel;
	int i;

	if (plat->id == BUCK5) {
		int buck5_range;

		buck5_range = pmic_reg_read(dev->parent, BD71885_BUCK5_MODE);
		if (buck5_range < 0)
			return buck5_range;

		/*
		 * The BUCK5 has special range selection. Select correct ranges
		 * depending on the bit 7 in MODE reg. Both HI and LO mode have
		 * same number of ranges though so we can trust in
		 * plat->numranges regardless of the mode here.
		 */
		if (buck5_range & BD71885_BUCK5_LORANGE) {
			r = &buck5_lo_vranges[0];

			goto get;
		}
	}
	r = &plat->ranges[0];

get:
	sel = pmic_reg_read(dev->parent, plat->vsel_reg);
	if (sel < 0)
		return sel;

	for (i = 0; i < plat->numranges; i++) {
		if (!vrange_find_value(&r[i], sel, &tmp))
			return tmp;
	}

	pr_err("Unknown voltage value\n");

	return -EINVAL;
}

#define BUCK5_LOW_MAX 1000000

static int bd71885_set_value(struct udevice *dev, int uvolt)
{
	struct bd71885_plat *plat = dev_get_plat(dev);
	struct regulator_vrange *r;
	unsigned int sel;
	int i, ret;
	int found = 0;
	int buck5_range, retry = 0;

	if (plat->id == BUCK5) {
		retry = 1;
		buck5_range = pmic_reg_read(dev->parent, BD71885_BUCK5_MODE);
		if (buck5_range < 0)
			return buck5_range;

		if (buck5_range & BD71885_BUCK5_LORANGE) {
			r = &buck5_lo_vranges[0];

			goto try_set;
		}
	}
	r = &plat->ranges[0];

try_set:
	for (i = 0; i < plat->numranges; i++) {
		found = !vrange_find_selector(&r[i], uvolt, &sel);
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

	if (!found) {
		/*
		 * If no suitable voltage was found, check whether we are BUCK5.
		 * And if we are, then we will check whether we can find
		 * suitable value from the other range.
		 */
		if (retry != 1)
			return -EINVAL;
		else
			retry++;

		if (!(buck5_range & BD71885_BUCK5_LORANGE))
			r = &buck5_lo_vranges[0];
		else
			r = &plat->ranges[0];

		goto try_set;
	}

	/*
	 * So, we (finally?) found our voltage selector. Let's see if this
	 * requires us to change the BUCK5 range. If it does, then we need to
	 * toggle the existing range - and write the new selector.
	 *
	 * Please note, this change is not atomic. We are going to jump to some
	 * intermediate voltage while doing this change. Let's see what is the
	 * safest way...
	 */
	if (retry == 2) {
		if (bd71885_get_enable(dev)) {
			pr_err("BUCK5 enabled, can't set range\n");
			return -EBUSY;
		}
		buck5_range = ~(BD71885_BUCK5_LORANGE & buck5_range);
		buck5_range &= BD71885_BUCK5_LORANGE;

		ret = pmic_clrsetbits(dev->parent, BD71885_BUCK5_MODE,
				      BD71885_BUCK5_LORANGE, buck5_range);
	}

	return pmic_reg_write(dev->parent, plat->vsel_reg, sel);
}



static int bd71885_regulator_probe(struct udevice *dev)
{
	struct bd71885_plat *plat = dev_get_plat(dev);
	struct dm_regulator_uclass_plat *uc_pdata;
	int data_amnt = BD71885_REGULATOR_AMOUNT;
	int i, vendor, prod_id;
	struct udevice *parent;

	parent = dev_get_parent(dev);
	if (!parent) {
		pr_err("No parent\n");
		return -ENODEV;
	}

	prod_id = pmic_reg_read(parent, BD71885_PROD_ID);
	if (prod_id < 0)
		return prod_id;

	vendor = pmic_reg_read(parent, BD71885_VENDOR);
	if (vendor < 0)
		return vendor;

	if (prod_id != BD71885_PROD_ID_VAL || vendor != BD71885_VENDOR_VAL)
		pr_warn("Unknown product/vendor\n");

	for (i = 0; i < data_amnt; i++) {
		if (!strcmp(dev->name, bd71885_reg_data[i].name)) {
			*plat = bd71885_reg_data[i];

			uc_pdata = dev_get_uclass_plat(dev);
			return bd71885_set_enable(dev, !!(uc_pdata->boot_on ||
						  uc_pdata->always_on));
		}
	}

	pr_err("Unknown regulator '%s'\n", dev->name);

	return -ENOENT;
}

static const struct dm_regulator_ops bd71885_regulator_ops = {
	.get_value  = bd71885_get_value,
	.set_value  = bd71885_set_value,
	.get_enable = bd71885_get_enable,
	.set_enable = bd71885_set_enable,
};

U_BOOT_DRIVER(bd71885_regulator) = {
	.name = BD71885_REGULATOR_DRIVER,
	.id = UCLASS_REGULATOR,
	.ops = &bd71885_regulator_ops,
	.probe = bd71885_regulator_probe,
	.plat_auto = sizeof(struct bd71885_plat),
};

