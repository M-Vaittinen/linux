// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright (C) 2022 ROHM Semiconductors
//
// ROHM/KIONIX KX022 accelerometer driver

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>

#include "kionix-kx022a.h"

static int kx022_i2c_probe(struct i2c_client *i2c)
{
	struct regmap *regmap;
	struct device *dev = &i2c->dev;

	if (!i2c->irq) {
		dev_err(dev, "No IRQ configured\n");
		return -EINVAL;
	}

	regmap = devm_regmap_init_i2c(i2c, &kx022_regmap);
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to initialize Regmap\n");

		return PTR_ERR(regmap);
	}

	return kx022_probe_internal(dev, i2c->irq, BUS_I2C);
}

static const struct of_device_id kx022_of_match[] = {
	{ .compatible = "kionix,kx022a-i2c", },
	{ },
};
MODULE_DEVICE_TABLE(of, kx022_of_match);
/*
static const struct dev_pm_ops kx022_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(kx022_suspend, kx022_resume)
	SET_RUNTIME_PM_OPS(kx022_runtime_suspend, kx022_runtime_resume, NULL)
};
*/
static struct i2c_driver kx022_i2c_driver = {
	.driver = {
			.name  = "kx022a-i2c",
//			.pm    = &kx022_pm_ops,
			.of_match_table = kx022_of_match,
		  },
	.probe_new    = kx022_i2c_probe,
};

module_i2c_driver(kx022_i2c_driver);

MODULE_DESCRIPTION("ROHM/Kionix KX022A accelerometer driver");
MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_LICENSE("GPL");
