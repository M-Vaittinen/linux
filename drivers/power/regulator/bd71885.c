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

#define BD71885_BUCK_MODE_MASK		BIT(3)
#define BD71885_LDO_MODE_MASK		BIT(0)

/*
TODO: If this goes to upstream we should allow setting this via DT. Board specific compilation scales well
only when there is one very specific product where the SW is used.
#define OTP_LDO1_LORANGE
#define OTP_LDO3_HIRANGE
*/

struct bd71885_plat {
	const char		*name;
	int			id;
	struct regulator_vrange	*ranges;
	unsigned int		numranges;
	u8			sel_mask;
	int			en_reg;
	int			vsel_reg;
	int			mode_reg;
	int			mode_mask;
	bool			dvs;
};

enum {
	BD71885_REGULATOR_MODE_NORMAL,
	BD71885_REGULATOR_MODE_LPM,
};

static struct dm_regulator_mode bd71885_ldo_modes[] = {
        { .id = BD71885_REGULATOR_MODE_NORMAL,
                .register_value = 0, .name = "NORMAL" },
        { .id = BD71885_REGULATOR_MODE_LPM,
                .register_value = BD71885_LDO_MODE_MASK, .name = "LOWPOWER" },
};

static struct dm_regulator_mode bd71885_buck_modes[] = {
        { .id = BD71885_REGULATOR_MODE_NORMAL,
                .register_value = 0, .name = "NORMAL" },
        { .id = BD71885_REGULATOR_MODE_LPM,
                .register_value = BD71885_BUCK_MODE_MASK, .name = "LOWPOWER" },
};

#define BD_DATA(_id, _range, _en_reg) \
{ \
	.name = __stringify(_id), .ranges = (_range),			\
	.numranges = ARRAY_SIZE(_range), .id = (_id),			\
	.en_reg = (_en_reg),						\
	.vsel_reg = (_id) < LDO1 ? (_en_reg) + 2 : (_en_reg) + 1,	\
	.mode_reg = (_id) < LDO1 ? (_en_reg) + 1 : (_en_reg) + 2,	\
	.mode_mask = (_id) < LDO1 ? BD71885_BUCK_MODE_MASK :		\
				    BD71885_LDO_MODE_MASK,		\
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
	BD_RANGE(1500000, 10000, 0, 0xc8),
	BD_RANGE(3500000, 0, 0xc9, 0xff),
};

/*
 * LDO1 default to hi-range. LDO3 default to the low range.
 * There is an OTP option for changing this.
 */
static struct regulator_vrange ldo_lo_vranges[] = {
	BD_RANGE(600000, 10000, 0, 0x78),
	BD_RANGE(1800000, 0, 0x79, 0xff),
};

static struct regulator_vrange ldo_hi_vranges[] = {
	BD_RANGE(750000, 10000, 0, 0xff),
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
	BD_DATA(BUCK1, buck123458_vranges, BD71885_BUCK1_ON),
	BD_DATA(BUCK2, buck123458_vranges, BD71885_BUCK2_ON),
	BD_DATA(BUCK3, buck123458_vranges, BD71885_BUCK3_ON),
	BD_DATA(BUCK4, buck123458_vranges, BD71885_BUCK4_ON),
	BD_DATA(BUCK5, buck123458_vranges, BD71885_BUCK5_ON),
	BD_DATA(BUCK6, buck67_vranges, BD71885_BUCK6_ON),
	BD_DATA(BUCK7, buck67_vranges, BD71885_BUCK7_ON),
	BD_DATA(BUCK8, buck123458_vranges, BD71885_BUCK8_ON),
#ifndef OTP_LDO1_LORANGE
	BD_DATA(LDO1, ldo_hi_vranges, BD71885_LDO1_ON),
#else
	BD_DATA(LDO1, ldo_lo_vranges, BD71885_LDO1_ON),
#endif
	BD_DATA(LDO2, ldo_hi_vranges, BD71885_LDO2_ON),
#ifndef OTP_LDO3_HIRANGE
	BD_DATA(LDO3, ldo_lo_vranges, BD71885_LDO3_ON),
#else
	BD_DATA(LDO3, ldo_hi_vranges, BD71885_LDO3_ON),
#endif
	BD_DATA(LDO4, ldo_hi_vranges, BD71885_LDO4_ON),
};

static const struct dm_regulator_mode
	*bd71885_find_mode_by_id(int id,
				const struct dm_regulator_mode *modes,
				uint mode_count)
{
	for (; mode_count; mode_count--) {
		if (modes->id == id)
			return modes;
		modes++;
	}
	return NULL;
}

static int bd71885_get_mode(struct udevice *dev)
{
	struct bd71885_plat *plat = dev_get_plat(dev);
	int val;

	printf("Reading mode. plat = %p dev = %p\n", plat, dev);
	val = pmic_reg_read(dev->parent, plat->mode_reg);
	if (val < 0)
		return val;

	printf("Returning mode %d\n", (val & plat->mode_mask) ?
		BD71885_REGULATOR_MODE_LPM : BD71885_REGULATOR_MODE_NORMAL);
	if (val & plat->mode_mask)
		return BD71885_REGULATOR_MODE_LPM;

	return BD71885_REGULATOR_MODE_NORMAL;
}

static int __regulator_set_mode(struct udevice *dev, int mode_id,
				struct dm_regulator_mode *modes, int num_modes)
{
	struct bd71885_plat *plat = dev_get_plat(dev);
	const struct dm_regulator_mode *mode;

	mode = bd71885_find_mode_by_id(mode_id, modes, num_modes);
	if (!mode)
		return -EINVAL;

	return pmic_clrsetbits(dev->parent, plat->mode_reg,
			       plat->mode_mask, mode->register_value);
}

static int ldo_set_mode(struct udevice *dev, int mode_id)
{
	return __regulator_set_mode(dev, mode_id, &bd71885_ldo_modes[0],
				    ARRAY_SIZE(bd71885_ldo_modes));
}

static int buck_set_mode(struct udevice *dev, int mode_id)
{
	return __regulator_set_mode(dev, mode_id, &bd71885_buck_modes[0],
				    ARRAY_SIZE(bd71885_buck_modes));
}

static int __bd71885_get_enable(struct udevice *dev, int mask)
{
	struct bd71885_plat *plat = dev_get_plat(dev);
	int val;

	val = pmic_reg_read(dev->parent, plat->en_reg);
	if (val < 0)
		return val;

	return !!(val & mask);
}

static int bd71885_get_enable(struct udevice *dev)
{
	return __bd71885_get_enable(dev, BD71885_MASK_RUN_ON);
}

static int bd71885_get_suspend_enable(struct udevice *dev)
{
	return __bd71885_get_enable(dev, BD71885_MASK_SUSP_ON);
}

static int bd71885_get_idle_enable(struct udevice *dev) __attribute__((unused));
static int bd71885_get_idle_enable(struct udevice *dev)
{
	return __bd71885_get_enable(dev, BD71885_MASK_IDLE_ON);
}

static int __bd71885_set_enable(struct udevice *dev, bool enable, int mask)
{
	struct bd71885_plat *plat = dev_get_plat(dev);
	int val;

	if (enable)
		val = mask;
	else
		val = 0;

	return pmic_clrsetbits(dev->parent, plat->en_reg, mask, val);
}

static int bd71885_set_enable(struct udevice *dev, bool enable)
{
	return __bd71885_set_enable(dev, enable, BD71885_MASK_RUN_ON);
}

static int bd71885_set_suspend_enable(struct udevice *dev, bool enable)
{
	return __bd71885_set_enable(dev, enable, BD71885_MASK_SUSP_ON);
}

static int bd71885_set_idle_enable(struct udevice *dev,
				   bool enable) __attribute__((unused));
static int bd71885_set_idle_enable(struct udevice *dev, bool enable)
{
	return __bd71885_set_enable(dev, enable, BD71885_MASK_IDLE_ON);
}

#define BD71885_BUCK_SUSPEND_VSEL_OFFSET 2
#define BD71885_BUCK_IDLE_VSEL_OFFSET 1

static int __bd71885_get_value(struct udevice *dev, int vsel)
{
	struct bd71885_plat *plat = dev_get_plat(dev);
	struct regulator_vrange *r;
	unsigned int tmp, sel;
	int i;

	if (plat->id == BUCK5) {
		int buck5_range;

		buck5_range = pmic_reg_read(dev->parent, BD71885_BUCK5_MODE);
		if (buck5_range < 0) {
			printf("Range reading failed\n");

			return buck5_range;
		}

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
	sel = pmic_reg_read(dev->parent, vsel);
	if (sel < 0) {
		printf("sel reading failed\n");

		return sel;
	}

	for (i = 0; i < plat->numranges; i++)
		if (!vrange_find_value(&r[i], sel, &tmp))
			return tmp;

	printf("Unknown voltage value\n");

	return -EINVAL;
}

static int bd71885_get_suspend_value(struct udevice *dev)
{
	struct bd71885_plat *plat = dev_get_plat(dev);

	return __bd71885_get_value(dev, plat->vsel_reg +
                                   BD71885_BUCK_SUSPEND_VSEL_OFFSET);
}

static int bd71885_get_value(struct udevice *dev)
{
	struct bd71885_plat *plat = dev_get_plat(dev);

	return __bd71885_get_value(dev, plat->vsel_reg);
}

static int bd71885_get_idle_value(struct udevice *dev) __attribute__((unused));
static int bd71885_get_idle_value(struct udevice *dev)
{
	struct bd71885_plat *plat = dev_get_plat(dev);

	return __bd71885_get_value(dev, plat->vsel_reg +
				   BD71885_BUCK_IDLE_VSEL_OFFSET);
}

#define BUCK5_LOW_MAX 1000000

static int __bd71885_set_value(struct udevice *dev, int uvolt, int vsel)
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
		 * And if we are and if we are changing RUN voltage, then we
		 * will check whether we can find suitable value from the other
		 * range.
		 */
		if (retry != 1 || plat->id != BUCK5)
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
			printf("BUCK5 enabled, can't set range\n");
			return -EBUSY;
		}
		buck5_range = ~(BD71885_BUCK5_LORANGE & buck5_range);
		buck5_range &= BD71885_BUCK5_LORANGE;

		ret = pmic_clrsetbits(dev->parent, BD71885_BUCK5_MODE,
				      BD71885_BUCK5_LORANGE, buck5_range);
	}

	return pmic_reg_write(dev->parent, vsel, sel);

}

static int bd71885_set_suspend_value(struct udevice *dev, int uvolt)
{
	struct bd71885_plat *plat = dev_get_plat(dev);

	return __bd71885_set_value(dev, uvolt, plat->vsel_reg +
				   BD71885_BUCK_SUSPEND_VSEL_OFFSET);
}

static int bd71885_set_idle_value(struct udevice *dev,
				  int uvolt) __attribute__((unused));
static int bd71885_set_idle_value(struct udevice *dev, int uvolt)
{
	struct bd71885_plat *plat = dev_get_plat(dev);

	if (plat->id > BUCK8) {
		printf("setting SUSPEND voltage for LDOs not supported\n");
		return -EINVAL;
	}

	return __bd71885_set_value(dev, uvolt, plat->vsel_reg +
				   BD71885_BUCK_IDLE_VSEL_OFFSET);
}

static int bd71885_set_value(struct udevice *dev, int uvolt)
{
	struct bd71885_plat *plat = dev_get_plat(dev);

	return __bd71885_set_value(dev, uvolt, plat->vsel_reg);
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
		printf("No parent\n");
		return -ENODEV;
	}

	prod_id = pmic_reg_read(parent, BD71885_PROD_ID);
	if (prod_id < 0)
		return prod_id;

	vendor = pmic_reg_read(parent, BD71885_VENDOR);
	if (vendor < 0)
		return vendor;

	if (prod_id != BD71885_PROD_ID_VAL || vendor != BD71885_VENDOR_VAL)
		printf("Unknown product/vendor\n");

	for (i = 0; i < data_amnt; i++) {
		if (!strcmp(dev->name, bd71885_reg_data[i].name)) {
			printf("Probed '%s'\n", dev->name);
			*plat = bd71885_reg_data[i];

			uc_pdata = dev_get_uclass_plat(dev);
			if (bd71885_reg_data[i].id < LDO1) {
				uc_pdata->mode = &bd71885_buck_modes[0];
				uc_pdata->mode_count = ARRAY_SIZE(bd71885_buck_modes);
				printf("Set mode ptr for buck id %d\n", bd71885_reg_data[i].id);
			} else {
				uc_pdata->mode = &bd71885_ldo_modes[0];
				uc_pdata->mode_count = ARRAY_SIZE(bd71885_ldo_modes);
				printf("Set mode ptr for ldo id %d\n", bd71885_reg_data[i].id);
			}
			return bd71885_set_enable(dev, !!(uc_pdata->boot_on ||
						  uc_pdata->always_on));
		}
	}

	printf("Unknown regulator '%s'\n", dev->name);

	return -ENOENT;
}

static const struct dm_regulator_ops bd71885_buck_ops = {
	.get_value		= bd71885_get_value,
	.set_value		= bd71885_set_value,
	.get_suspend_value	= bd71885_get_suspend_value,
	.set_suspend_value	= bd71885_set_suspend_value,
	.get_enable		= bd71885_get_enable,
	.set_enable		= bd71885_set_enable,
	.get_suspend_enable	= bd71885_get_suspend_enable,
	.set_suspend_enable	= bd71885_set_suspend_enable,
	.get_mode		= bd71885_get_mode,
	.set_mode		= buck_set_mode,
};

static const struct dm_regulator_ops bd71885_ldo_ops = {
	.get_value		= bd71885_get_value,
	.set_value		= bd71885_set_value,
	.get_enable		= bd71885_get_enable,
	.set_enable		= bd71885_set_enable,
	.get_suspend_enable	= bd71885_get_suspend_enable,
	.set_suspend_enable	= bd71885_set_suspend_enable,
	.get_mode		= bd71885_get_mode,
	.set_mode		= ldo_set_mode,
};

U_BOOT_DRIVER(bd71885_buck) = {
	.name = BD71885_BUCK_DRIVER,
	.id = UCLASS_REGULATOR,
	.ops = &bd71885_buck_ops,
	.probe = bd71885_regulator_probe,
	.plat_auto = sizeof(struct bd71885_plat),
};

U_BOOT_DRIVER(bd71885_ldo) = {
	.name = BD71885_LDO_DRIVER,
	.id = UCLASS_REGULATOR,
	.ops = &bd71885_ldo_ops,
	.probe = bd71885_regulator_probe,
	.plat_auto = sizeof(struct bd71885_plat),
};

