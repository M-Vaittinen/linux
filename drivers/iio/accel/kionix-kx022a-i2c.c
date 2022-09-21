// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright (C) 2022 ROHM Semiconductors
//
// ROHM/KIONIX KX022A accelerometer driver

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include "kionix-kx022a.h"

static int kx022a_i2c_probe(struct i2c_client *i2c)
{
	struct regmap *regmap;
	struct device *dev = &i2c->dev;

	if (!i2c->irq) {
		dev_err(dev, "No IRQ configured\n");
		return -EINVAL;
	}

	regmap = devm_regmap_init_i2c(i2c, &kx022a_regmap);
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to initialize Regmap\n");

		return PTR_ERR(regmap);
	}

	return kx022a_probe_internal(dev, i2c->irq);
}

static const struct of_device_id kx022a_of_match[] = {
	{ .compatible = "kionix,kx022a", },
	{ },
};
MODULE_DEVICE_TABLE(of, kx022a_of_match);

static struct i2c_driver kx022a_i2c_driver = {
	.driver = {
			.name  = "kx022a-i2c",
			.of_match_table = kx022a_of_match,
		  },
	.probe_new    = kx022a_i2c_probe,
};

module_i2c_driver(kx022a_i2c_driver);

MODULE_DESCRIPTION("ROHM/Kionix KX022A accelerometer driver");
MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_LICENSE("GPL");
