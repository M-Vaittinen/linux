// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright 2022 ROHM Semiconductors
 *  Matti Vaittinen <mazziesaccount@gmail.com>
 */

#include <common.h>
#include <errno.h>
#include <dm.h>
#include <i2c.h>
#include <log.h>
#include <asm/global_data.h>
#include <power/pmic.h>
#include <power/regulator.h>
#include <power/bd2659.h>
#include "bdxxxx.h"

DECLARE_GLOBAL_DATA_PTR;

static const struct pmic_child_info pmic_children_info[] = {
	/* buck */
	{ .prefix = "b", .driver = BD2659_REGULATOR_DRIVER},
	/* ldo */
	{ .prefix = "l", .driver = BD2659_REGULATOR_DRIVER},
	{ },
};

static int bd2659_bind(struct udevice *dev)
{
	return bdxxxx_bind(dev, pmic_children_info);
}

static int bd2659_reg_count(struct udevice *dev)
{
	return BD2659_MAX_REGISTER - 1;
}

static int bd2659_probe(struct udevice *dev)
{
	int ret;
	uint8_t mask = BD2659_MASK_REGLOCK;

	/* Ensure the PMIC is not locked */
	ret = pmic_clrsetbits(dev, BD2659_REGLOCK, mask, 0);
	if (ret) {
		debug("%s: %s Failed to unlock PMIC\n", __func__, dev->name);
		return ret;
	}
	debug("%s: '%s' - PMIC registers unlocked\n", __func__, dev->name);

	return 0;
}

static struct dm_pmic_ops bd2659_ops = {
	.reg_count = bd2659_reg_count,
	.read = bdxxxx_read,
	.write = bdxxxx_write,
};

static const struct udevice_id bd2659_ids[] = {
	{ .compatible = "rohm,bd2659", },
	{ }
};

U_BOOT_DRIVER(pmic_bd2659) = {
	.name = "bd2659 pmic",
	.id = UCLASS_PMIC,
	.of_match = bd2659_ids,
	.bind = bd2659_bind,
	.probe = bd2659_probe,
	.ops = &bd2659_ops,
};
