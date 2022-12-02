// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright 2022 ROHM Semiconductors
 *  Matti Vaittinen <mazziesaccount@gmail.com>
 */

#ifndef __BDXXXX_H
#define __BDXXXX_H

static inline int bdxxxx_write(struct udevice *dev, uint reg, const uint8_t *buff,
			 int len)
{
	if (dm_i2c_write(dev, reg, buff, len)) {
		pr_err("write error to device: %p register: %#x!", dev, reg);
		return -EIO;
	}

	return 0;
}

static inline int bdxxxx_read(struct udevice *dev, uint reg, uint8_t *buff, int len)
{
	if (dm_i2c_read(dev, reg, buff, len)) {
		pr_err("read error from device: %p register: %#x!", dev, reg);
		return -EIO;
	}

	return 0;
}

static inline int bdxxxx_bind(struct udevice *dev, const struct pmic_child_info *pmic_children_info)
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



#endif //__BDXXXX_H
