// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright (C) 2022 ROHM Semiconductors
//
// ROHM/KIONIX KX022 accelerometer driver

#include <linux/module.h>
#include <linux/input.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>

#include "kionix-kx022a.h"

static int kx022a_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct regmap *regmap;

	if (!spi->irq) {
		dev_err(dev, "No IRQ configured\n");
		return -EINVAL;
	}

	regmap = devm_regmap_init_spi(spi, &kx022a_regmap);
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to initialize Regmap\n");

		return PTR_ERR(regmap);
	}
	return kx022a_probe_internal(dev, spi->irq, BUS_SPI);
}

static const struct of_device_id kx022a_of_match[] = {
	{ .compatible = "rohm,kx022a-spi", },
	{ },
};
MODULE_DEVICE_TABLE(of, kx022a_of_match);
/*
static const struct dev_pm_ops kx022a_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(kx022a_suspend, kx022a_resume)
	SET_RUNTIME_PM_OPS(kx022a_runtime_suspend, kx022a_runtime_resume, NULL)
};
*/
static struct spi_driver kx022a_spi_driver = {
	.driver = {
		.name   = "kx022a-spi",
//		.pm     = &kx022a_spi_pm,
	},
	.probe	= kx022a_spi_probe,
};

module_spi_driver(kx022a_spi_driver);

MODULE_DESCRIPTION("ROHM/Kionix kx022A accelerometer driver");
MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_LICENSE("GPL");
