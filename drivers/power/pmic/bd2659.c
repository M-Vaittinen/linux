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

DECLARE_GLOBAL_DATA_PTR;

static const struct pmic_child_info pmic_children_info[] = {
	/* buck */
	{ .prefix = "b", .driver = BD2659_REGULATOR_DRIVER},
	/* ldo */
	{ .prefix = "l", .driver = BD2659_REGULATOR_DRIVER},
	{ },
};

static int bd2659_reg_count(struct udevice *dev)
{
	return BD2659_MAX_REGISTER - 1;
}

static int bd2659_write(struct udevice *dev, uint reg, const uint8_t *buff,
			 int len)
{
	if (dm_i2c_write(dev, reg, buff, len)) {
		pr_err("write error to device: %p register: %#x!", dev, reg);
		return -EIO;
	}

	return 0;
}

static int bd2659_read(struct udevice *dev, uint reg, uint8_t *buff, int len)
{
	if (dm_i2c_read(dev, reg, buff, len)) {
		pr_err("read error from device: %p register: %#x!", dev, reg);
		return -EIO;
	}

	return 0;
}

static int bd2659_bind(struct udevice *dev)
{
	int children;
	ofnode regulators_node;

	regulators_node = dev_read_subnode(dev, "regulators");
	if (!ofnode_valid(regulators_node)) {
		debug("%s: %s regulators subnode not found!\n", __func__,
		      dev->name);
		return -ENXIO;
	}

	debug("%s: '%s' - found regulators subnode\n", __func__, dev->name);

	if (CONFIG_IS_ENABLED(PMIC_CHILDREN)) {
		children = pmic_bind_children(dev, regulators_node, pmic_children_info);
		if (!children)
			debug("%s: %s - no child found\n", __func__, dev->name);
	}
	/* Always return success for this device */
	return 0;
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
	.read = bd2659_read,
	.write = bd2659_write,
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
