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
	bool			dvs;
};

#define BD_DATA(_name, _range) \
{ .name = (_name), .ranges = (_range),  .numranges = ARRAY_SIZE(_range), }

static struct regulator_vrange buck012_vranges[] = {
	BD_RANGE(500000, 5000, 1, 0xab),
	BD_RANGE(1350000, 0, 0xac, 0xff),
};

static struct regulator_vrange buck3_vranges[] = {
	BD_RANGE(700000, 10000, 0, 0x3c),
	BD_RANGE(1300000, 0, 0x3d, 0x3f),
};

static struct bd2659_plat bd2659_reg_data[] = {
	BD_DATA("BUCK0", &buck012_vranges[0]),
	BD_DATA("BUCK1", &buck012_vranges[0]),
	BD_DATA("BUCK2", &buck012_vranges[0]),
	BD_DATA("BUCK3", &buck3_vranges[0]),
};


