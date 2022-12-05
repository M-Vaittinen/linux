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
#include <power/bd71885.h>
#include "bdxxxx.h"

DECLARE_GLOBAL_DATA_PTR;

static const struct pmic_child_info pmic_children_info[] = {
	/* buck */
	{ .prefix = "b", .driver = BD71885_REGULATOR_DRIVER},
	/* ldo */
	{ .prefix = "l", .driver = BD71885_REGULATOR_DRIVER},
	{ },
};

static int bd71885_bind(struct udevice *dev)
{
	return bdxxxx_bind(dev, pmic_children_info);
}

static int bd71885_reg_count(struct udevice *dev)
{
	return BD71885_MAX_REGISTER - 1;
}

static int bd71885_probe(struct udevice *dev)
{
	debug("%s: '%s' probed\n", __func__, dev->name);

	return 0;
}

static struct dm_pmic_ops bd71885_ops = {
	.reg_count = bd71885_reg_count,
	.read = bdxxxx_read,
	.write = bdxxxx_write,
};

static const struct udevice_id bd71885_ids[] = {
	{ .compatible = "rohm,bd71885", },
	{ }
};

U_BOOT_DRIVER(pmic_bd71885) = {
	.name = "bd71885 pmic",
	.id = UCLASS_PMIC,
	.of_match = bd71885_ids,
	.bind = bd71885_bind,
	.probe = bd71885_probe,
	.ops = &bd71885_ops,
};
