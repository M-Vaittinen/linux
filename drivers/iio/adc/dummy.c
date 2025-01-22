// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 ROHM Semiconductors
 *
 * ROHM/KIONIX accelerometer driver
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>

static int kx022a_i2c_probe(struct i2c_client *i2c)
{
	pr_info("%s() probed\n", __func__);

	return 0;
}

static const struct i2c_device_id kx022a_i2c_id[] = {
	{ .name = "dummy", },
	{ .name = "dyummy2", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, kx022a_i2c_id);

static const struct of_device_id kx022a_of_match[] = {
	{ .compatible = "rohm,dummy", },
	{ .compatible = "rohm,dummy2", },
	{ }
};
MODULE_DEVICE_TABLE(of, kx022a_of_match);

static struct i2c_driver kx022a_i2c_driver = {
	.driver = {
		.name  = "k-i2c",
		.of_match_table = kx022a_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	  },
	.probe        = kx022a_i2c_probe,
	.id_table     = kx022a_i2c_id,
};
module_i2c_driver(kx022a_i2c_driver);

MODULE_DESCRIPTION("ROHM/Kionix KX022A accelerometer driver");
MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_LICENSE("GPL");

